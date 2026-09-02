/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Xuchen Han
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** @author Xuchen Han (xuchenhan123@gmail.com) */

/* This doesn't test most of the functionality. That is left as a future
 exercise.*/

#include <memory>
#include <set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "fcl/broadphase/broadphase_SaP.h"
#include "fcl/broadphase/broadphase_bruteforce.h"
#include "fcl/geometry/shape/box.h"
#include "fcl/narrowphase/collision_object.h"

namespace fcl {
namespace {

// Overlapping pairs, recorded as object indices so a failure prints something
// a reader can follow.
using PairSet = std::set<std::pair<int, int>>;

struct Recorder {
  const std::vector<CollisionObject<double>*>* objects;
  PairSet pairs;
};

int IndexOf(const std::vector<CollisionObject<double>*>& objects,
            CollisionObject<double>* object) {
  for (std::size_t i = 0; i < objects.size(); ++i) {
    if (objects[i] == object) return static_cast<int>(i);
  }
  return -1;
}

bool RecordPair(CollisionObject<double>* a, CollisionObject<double>* b,
                void* data) {
  auto* recorder = static_cast<Recorder*>(data);
  const int i = IndexOf(*recorder->objects, a);
  const int j = IndexOf(*recorder->objects, b);
  recorder->pairs.emplace(std::min(i, j), std::max(i, j));
  return false;
}

PairSet FindOverlappingPairs(
    BroadPhaseCollisionManager<double>* manager,
    const std::vector<CollisionObject<double>*>& objects) {
  Recorder recorder{&objects, {}};
  manager->collide(&recorder, RecordPair);
  return recorder.pairs;
}

// SaP tracks each object as an interval per axis and splices the interval's
// endpoints into sorted lists as objects move. CollisionObject uses a tight
// local AABB at the identity orientation and a conservative, radius-sized AABB
// at any non-identity orientation. Moving between those two representations
// exercises all four endpoint directions on every axis.
GTEST_TEST(BroadPhaseSaP, UpdateWithChangingExtents) {
  // The unequal dimensions make the identity AABB strictly smaller than the
  // conservative rotated AABB on every axis. Also, the box has the obnoxious
  // AABB computation for non-identity orientations (see below).
  auto shape = std::make_shared<Box<double>>(20.0, 1.0, 1.0);

  std::vector<std::unique_ptr<CollisionObject<double>>> objects;
  std::vector<CollisionObject<double>*> raw;
  // Note: 7 is an arbitrary number of overlapping boxes. With a separation of
  // 3 meters and a length of 20 meters (along the x-axis), we guarantee that
  // the first and last boxes will not be overlapping.
  for (int i = 0; i < 7; ++i) {
    Transform3<double> X_WB = Transform3<double>::Identity();
    X_WB.translation() = Vector3<double>(3.0 * i, 0.5 * i, 0.0);
    objects.push_back(
        std::make_unique<CollisionObject<double>>(shape, X_WB));
    raw.push_back(objects.back().get());
  }

  SaPCollisionManager<double> sap;
  NaiveCollisionManager<double> reference;
  sap.registerObjects(raw);
  reference.registerObjects(raw);
  sap.setup();
  reference.setup();
  const PairSet reference_set = FindOverlappingPairs(&reference, raw);
  EXPECT_GT(reference_set.size(), 0);
  EXPECT_EQ(FindOverlappingPairs(&sap, raw), reference_set);

  // Test the cases where internal endpoints *expand*. Any non-identity
  // orientation selects the conservative AABB (the bounding box of the bounding
  // sphere of the geometry-local bounding box of the box); we'll use a
  // 45-degree rotation to make it as bad as possible (even if the upstream
  // representation of Box AABB eventually changes). From the original
  // identity orientation, all boxes should get bigger.
  for (std::size_t i = 0; i < raw.size(); ++i) {
    const AABB<double> identity_aabb = raw[i]->getAABB();
    raw[i]->setRotation(AngleAxis<double>(M_PI / 4, Vector3<double>::UnitZ())
                            .toRotationMatrix());
    raw[i]->computeAABB();
    for (int axis = 0; axis < 3; ++axis) {
      EXPECT_LT(raw[i]->getAABB().min_[axis], identity_aabb.min_[axis]);
      EXPECT_GT(raw[i]->getAABB().max_[axis], identity_aabb.max_[axis]);
    }
  }
  sap.update();
  reference.update();
  PairSet expanded_set = FindOverlappingPairs(&reference, raw);
  EXPECT_GT(expanded_set.size(), reference_set.size());
  EXPECT_EQ(FindOverlappingPairs(&sap, raw), expanded_set) << "expand";

  // Test the case where the endpoints *contract*. Returning to identity
  // orientation uses the original, tighter AABB, so we'll get contracted
  // endpoints.
  for (std::size_t i = 0; i < raw.size(); ++i) {
    const AABB<double> rotated_aabb = raw[i]->getAABB();
    raw[i]->setRotation(Matrix3<double>::Identity());
    raw[i]->computeAABB();
    for (int axis = 0; axis < 3; ++axis) {
      EXPECT_GT(raw[i]->getAABB().min_[axis], rotated_aabb.min_[axis]);
      EXPECT_LT(raw[i]->getAABB().max_[axis], rotated_aabb.max_[axis]);
    }
  }
  sap.update();
  // Back to the original overlapping pairs.
  EXPECT_EQ(FindOverlappingPairs(&sap, raw), reference_set) << "contract";
}

}  // namespace
}  // namespace fcl

//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

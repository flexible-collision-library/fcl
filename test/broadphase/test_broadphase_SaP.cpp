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

/** Tests the sweep-and-prune manager's incremental update. */

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

PairSet Overlaps(BroadPhaseCollisionManager<double>* manager,
                 const std::vector<CollisionObject<double>*>& objects) {
  Recorder recorder{&objects, {}};
  manager->collide(&recorder, RecordPair);
  return recorder.pairs;
}

// SaP tracks each object as an interval per axis and splices the interval's
// endpoints into sorted lists as objects move. Rotating an elongated box
// changes the width of those intervals, which moves the two endpoints in
// opposite directions. Placing both from a single direction corrupted the
// lists, after which a walk could never terminate.
GTEST_TEST(BroadPhaseSaP, UpdateWithChangingExtents) {
  // Long in x, so the world-frame extents change a great deal with yaw.
  auto shape = std::make_shared<Box<double>>(20.0, 1.0, 1.0);

  std::vector<std::unique_ptr<CollisionObject<double>>> objects;
  std::vector<CollisionObject<double>*> raw;
  for (int i = 0; i < 12; ++i) {
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
  EXPECT_EQ(Overlaps(&sap, raw), Overlaps(&reference, raw));

  // Yaw every object by a different angle, which grows some intervals and
  // shrinks others, then let each manager take the update.
  for (int step = 1; step <= 8; ++step) {
    for (std::size_t i = 0; i < raw.size(); ++i) {
      Transform3<double> X_WB = Transform3<double>::Identity();
      X_WB.linear() =
          AngleAxis<double>(0.37 * step + 0.11 * i, Vector3<double>::UnitZ())
              .toRotationMatrix();
      X_WB.translation() = Vector3<double>(3.0 * i, 0.5 * i, 0.0);
      raw[i]->setTransform(X_WB);
      raw[i]->computeAABB();
    }
    sap.update();
    reference.update();
    EXPECT_EQ(Overlaps(&sap, raw), Overlaps(&reference, raw))
        << "step " << step;
  }
}

}  // namespace
}  // namespace fcl

//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

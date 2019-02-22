/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019. Toyota Research Institute
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
 *   * Neither the name of CNRS-LAAS and AIST nor the names of its
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

/** @author Damrong Guoy (Damrong.Guoy@tri.global) */

/** Tests the dynamic axis-aligned bounding box tree.*/

#include <iostream>
#include <memory>

#include <gtest/gtest.h>

#include "fcl/common/types.h"

#include "fcl/geometry/shape/sphere.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"

using Vector3d = fcl::Vector3d;

// Tests repeatability of a dynamic tree of two spheres when we call update()
// and distance() again and again without changing the poses of the objects.
//
// Currently every call to update() switches the order of the two objects, and I
// use this simple test to check how update() works.
// TODO(DamrongGuoy): Remove the above comment when we solve the
//  repeatability problem as mentioned in:
//  https://github.com/flexible-collision-library/fcl/issues/368
//
GTEST_TEST(DynamicAABBTreeCollisionManager, update) {
  // TODO(DamrongGuoy): Change "fcl::Sphered" to "const fcl::Sphered" when
  //  CollisionObject can take "shared_ptr<const CollisionGeometry>". Right
  //  now it doesn't accept the const version.
  auto sphere1 = std::make_shared<fcl::Sphered>(0.1);
  auto sphere2 = std::make_shared<fcl::Sphered>(0.2);
  fcl::CollisionObjectd object1(sphere1);
  fcl::CollisionObjectd object2(sphere2);
  const Vector3d position1(0.1, 0.2, 0.3);
  const Vector3d position2(0.11, 0.21, 0.31);

  // We will use `objects` to check the order of the two collision objects in
  // our callback function. The distance() only accepts "void*" but not
  // "const void*" as the callback data.  That's why `objects` is not
  // declared as a const vector here.
  //
  // Previously using std::vector<fcl::CollisionObjectd> failed the Eigen
  // alignment assertion on Win32, so we switch from a vector of objects to
  // a vector of pointers to objects.  Previously we also tried the custom
  // allocator:
  //     std::vector<fcl::CollisionObjectd,
  //                 Eigen::aligned_allocator<fcl::CollisionObjectd>>,
  // but some platforms failed to build.
  std::vector<fcl::CollisionObjectd*> objects = {&object1, &object2};
  std::vector<const Vector3d*> positions = {&position1, &position2};

  fcl::DynamicAABBTreeCollisionManager<double> dynamic_tree;
  auto o = objects.begin();
  auto p = positions.begin();
  for (; o != objects.end(); ++o, ++p) {
    (*o)->setTranslation(**p);
    (*o)->computeAABB();
    dynamic_tree.registerObject(*o);
  }

  // This callback function tests the order of the two collision objects from
  // the dynamic tree against the `callback_data`. We do not use the last
  // double& parameter. It specifies the minimum distance beyond which the pair
  // of objects will be skipped.
  auto distance_callback = [](fcl::CollisionObjectd* a,
                              fcl::CollisionObjectd* b, void* callback_data,
                              double&) -> bool {
    const auto& objects =
        *static_cast<std::vector<fcl::CollisionObjectd*>*>(callback_data);
    bool object0_then_object1 = (a == objects[0] && b == objects[1]);
    bool object1_then_object0 = (a == objects[1] && b == objects[0]);
    // TODO(DamrongGuoy): When update() can preserve the tree, change the
    //  expectation to EXPECT_TRUE(object0_then_object1)
    EXPECT_TRUE(object0_then_object1 || object1_then_object0)
        << "Wrong order of the two objects a and b.";
    // Return false will continue the tree traversal; however, we have only
    // one pair of objects, so it will stop anyway.
    return false;
  };
  // We repeat update() and distance() many times.  Each time, in the
  // callback, we check the order of the two objects.
  for (int count = 0; count < 8; ++count) {
    dynamic_tree.update();
    dynamic_tree.distance(&objects, distance_callback);
  }
}

//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

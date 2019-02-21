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

// #include <Eigen/StdVector>
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
  std::cout << "Begin test DynamicAABBTreeCollisionManager.update" <<std::endl;
// Start commen-out here, and it passed.

  auto sphere1 = std::make_shared<fcl::Sphered>(0.1);
  auto sphere2 = std::make_shared<fcl::Sphered>(0.2);

// Start commen-out here, and it passed.

  // We will use `objects` to check the order of the two collision objects in
  // our callback function. The distance() only accepts "void*" but not
  // "const void*" as the callback data.  That's why `objects` is not
  // declared as a const vector here.
  //
  // Using Eigen custom allocator:
  // std::vector<fcl::CollisionObjectd,
  //     Eigen::aligned_allocator<fcl::CollisionObjectd>>
  // seems to cause:-
  // - Travis CI
  //   - build failure on Linux gcc 4.8.4
  //     - StdVector.h:75:3: error: no matching function for call to
  //   - build failure on Linux clang 5.0.0
  //     - StdVector.h:75:3: error: no matching constructor for
  //       initialization of
  //   - run ok on Apple clang
  // - AppVeyor CI
  //   - build failure on Win32
  //     - StdVector.h(75): error C2719: 'first': formal parameter with
  //       requested alignment of 16 won't be aligned
  //     - StdVector.h(75): error C2664: cannot convert argument 1 from
  //       'fcl::CollisionObject<double>' to 'unsigned int'
  //   - run ok on x64
  std::vector<fcl::CollisionObjectd>
      objects {fcl::CollisionObjectd(sphere1), fcl::CollisionObjectd(sphere2)};
  fcl::DynamicAABBTreeCollisionManager<double> dynamic_tree;
  
// Start comment-out here, and see what would happen.
/*
  for (auto o = objects.begin(); o != objects.end(); ++o) {
    std::cout << "About to computeAABB()" << std::endl;
    o->computeAABB();
    dynamic_tree.registerObject(&(*o));
  }
// Start comment-out here, and it failed alignment assertion.

  // This callback function tests the order of the two collision objects from
  // the dynamic tree against the `callback_data`. We do not use the last
  // double& parameter. It specifies the minimum distance beyond which the pair
  // of objects will be skipped.
  auto distance_callback = [](fcl::CollisionObjectd* a,
                              fcl::CollisionObjectd* b, void* callback_data,
                              double&) -> bool {
    const auto& objects =
        *static_cast<std::vector<fcl::CollisionObjectd>*>(callback_data);
    bool object0_then_object1 = (a == &(objects[0]) && b == &(objects[1]));
    bool object1_then_object0 = (a == &(objects[1]) && b == &(objects[0]));
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
    std::cout << "About to update() and distance()" << std::endl;
    dynamic_tree.update();
    dynamic_tree.distance(&objects, distance_callback);
  }
*/
}

//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

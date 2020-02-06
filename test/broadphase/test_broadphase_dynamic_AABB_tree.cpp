/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020. Toyota Research Institute
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
// We only use the distance() method to invoke a hierarchy traversal.
// The distance-callback function in this test does not compute the signed
// distance between the two objects; it only checks their order.
//
// Currently every call to update() switches the order of the two objects.
// TODO(DamrongGuoy): Remove the above comment when we solve the
//  repeatability problem as mentioned in:
//  https://github.com/flexible-collision-library/fcl/issues/368
//
GTEST_TEST(DynamicAABBTreeCollisionManager, update) {
  auto sphere0 = std::make_shared<fcl::Sphered>(0.1);
  auto sphere1 = std::make_shared<fcl::Sphered>(0.2);
  fcl::CollisionObjectd object0(sphere0);
  fcl::CollisionObjectd object1(sphere1);
  const Vector3d position0(0.1, 0.2, 0.3);
  const Vector3d position1(0.11, 0.21, 0.31);

  // We will use `objects` to check the order of the two collision objects in
  // our callback function.
  //
  // We use std::vector that contains *pointers* to CollisionObjectd,
  // instead of std::vector that contains CollisionObjectd's.
  // Previously we used std::vector<fcl::CollisionObjectd>, and it failed the
  // Eigen alignment assertion on Win32. We also tried, without success, the
  // custom allocator:
  //     std::vector<fcl::CollisionObjectd,
  //                 Eigen::aligned_allocator<fcl::CollisionObjectd>>,
  // but some platforms failed to build.
  std::vector<fcl::CollisionObjectd*> objects = {&object0, &object1};
  std::vector<const Vector3d*> positions = {&position0, &position1};

  fcl::DynamicAABBTreeCollisionManager<double> dynamic_tree;
  for (int i = 0; i < static_cast<int>(objects.size()); ++i) {
    objects[i]->setTranslation(*positions[i]);
    objects[i]->computeAABB();
    dynamic_tree.registerObject(objects[i]);
  }

  // Pack the data for callback function.
  struct CallBackData {
    bool expect_object0_then_object1;
    std::vector<fcl::CollisionObjectd*>* objects;
  } data;
  data.expect_object0_then_object1 = false;
  data.objects = &objects;

  // This callback function tests the order of the two collision objects from
  // the dynamic tree against the `data`. We assume that the first two
  // parameters are always objects[0] and objects[1] in two possible orders,
  // so we can safely ignore the second parameter. We do not use the last
  // double& parameter, which specifies the distance beyond which the
  // pair of objects will be skipped.
  auto distance_callback = [](fcl::CollisionObjectd* a, fcl::CollisionObjectd*,
                              void* callback_data, double&) -> bool {
    // Unpack the data.
    auto data = static_cast<CallBackData*>(callback_data);
    const std::vector<fcl::CollisionObjectd*>& objects = *(data->objects);
    const bool object0_first = a == objects[0];
    EXPECT_EQ(data->expect_object0_then_object1, object0_first);
    // TODO(DamrongGuoy): Remove the statement below when we solve the
    //  repeatability problem as mentioned in:
    //  https://github.com/flexible-collision-library/fcl/issues/368
    // Expect to switch the order next time.
    data->expect_object0_then_object1 = !data->expect_object0_then_object1;
    // Return true to stop the tree traversal.
    return true;
  };
  // We repeat update() and distance() many times.  Each time, in the
  // callback function, we check the order of the two objects.
  for (int count = 0; count < 8; ++count) {
    dynamic_tree.update();
    dynamic_tree.distance(&data, distance_callback);
  }
}

//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

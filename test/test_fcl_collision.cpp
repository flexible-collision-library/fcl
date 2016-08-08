/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
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
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
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

/** \author Jia Pan */

#include <gtest/gtest.h>

#include "fcl/collision_node.h"
#include "fcl/collision.h"
#include "fcl/BV/convert_bv.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/narrowphase/gjk_solver_indep.h"
#include "fcl/narrowphase/gjk_solver_libccd.h"
#include "test_fcl_utility.h"
#include "fcl_resources/config.h"

using namespace fcl;

template<typename BV>
bool collide_Test(const Transform3<typename BV::Scalar>& tf,
                  const std::vector<Vector3<typename BV::Scalar>>& vertices1, const std::vector<Triangle>& triangles1,
                  const std::vector<Vector3<typename BV::Scalar>>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose = true);

template<typename BV>
bool collide_Test2(const Transform3<typename BV::Scalar>& tf,
                   const std::vector<Vector3<typename BV::Scalar>>& vertices1, const std::vector<Triangle>& triangles1,
                   const std::vector<Vector3<typename BV::Scalar>>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose = true);

template<typename BV, typename TraversalNode>
bool collide_Test_Oriented(const Transform3<typename BV::Scalar>& tf,
                           const std::vector<Vector3<typename BV::Scalar>>& vertices1, const std::vector<Triangle>& triangles1,
                           const std::vector<Vector3<typename BV::Scalar>>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose = true);


template<typename BV>
bool test_collide_func(const Transform3<typename BV::Scalar>& tf,
                       const std::vector<Vector3<typename BV::Scalar>>& vertices1, const std::vector<Triangle>& triangles1,
                       const std::vector<Vector3<typename BV::Scalar>>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method);

int num_max_contacts = std::numeric_limits<int>::max();
bool enable_contact = true;

template<typename Scalar>
std::vector<Contact<Scalar>>& global_pairs()
{
  static std::vector<Contact<Scalar>> static_global_pairs;
  return static_global_pairs;
}

template<typename Scalar>
std::vector<Contact<Scalar>>& global_pairs_now()
{
  static std::vector<Contact<Scalar>> static_global_pairs_now;
  return static_global_pairs_now;
}

template <typename Scalar>
void test_OBB_Box_test()
{
  Scalar r_extents[] = {-1000, -1000, -1000, 1000, 1000, 1000};
  Eigen::aligned_vector<Transform3<Scalar>> rotate_transform;
  generateRandomTransforms(r_extents, rotate_transform, 1);

  AABB<Scalar> aabb1;
  aabb1.min_ = Vector3<Scalar>(-600, -600, -600);
  aabb1.max_ = Vector3<Scalar>(600, 600, 600);

  OBB<Scalar> obb1;
  convertBV(aabb1, rotate_transform[0], obb1);
  Box<Scalar> box1;
  Transform3<Scalar> box1_tf;
  constructBox(aabb1, rotate_transform[0], box1, box1_tf);

  Scalar extents[] = {-1000, -1000, -1000, 1000, 1000, 1000};
  std::size_t n = 1000;

  Eigen::aligned_vector<Transform3<Scalar>> transforms;
  generateRandomTransforms(extents, transforms, n);

  for(std::size_t i = 0; i < transforms.size(); ++i)
  {
    AABB<Scalar> aabb;
    aabb.min_ = aabb1.min_ * 0.5;
    aabb.max_ = aabb1.max_ * 0.5;

    OBB<Scalar> obb2;
    convertBV(aabb, transforms[i], obb2);

    Box<Scalar> box2;
    Transform3<Scalar> box2_tf;
    constructBox(aabb, transforms[i], box2, box2_tf);

    GJKSolver_libccd<Scalar> solver;

    bool overlap_obb = obb1.overlap(obb2);
    bool overlap_box = solver.shapeIntersect(box1, box1_tf, box2, box2_tf, NULL);

    EXPECT_TRUE(overlap_obb == overlap_box);
  }
}

template <typename Scalar>
void test_OBB_shape_test()
{
  Scalar r_extents[] = {-1000, -1000, -1000, 1000, 1000, 1000};
  Eigen::aligned_vector<Transform3<Scalar>> rotate_transform;
  generateRandomTransforms(r_extents, rotate_transform, 1);

  AABB<Scalar> aabb1;
  aabb1.min_ = Vector3<Scalar>(-600, -600, -600);
  aabb1.max_ = Vector3<Scalar>(600, 600, 600);

  OBB<Scalar> obb1;
  convertBV(aabb1, rotate_transform[0], obb1);
  Box<Scalar> box1;
  Transform3<Scalar> box1_tf;
  constructBox(aabb1, rotate_transform[0], box1, box1_tf);

  Scalar extents[] = {-1000, -1000, -1000, 1000, 1000, 1000};
  std::size_t n = 1000;

  Eigen::aligned_vector<Transform3<Scalar>> transforms;
  generateRandomTransforms(extents, transforms, n);

  for(std::size_t i = 0; i < transforms.size(); ++i)
  {
    Scalar len = (aabb1.max_[0] - aabb1.min_[0]) * 0.5;
    OBB<Scalar> obb2;
    GJKSolver_libccd<Scalar> solver;

    {
      Sphere<Scalar> sphere(len);
      computeBV(sphere, transforms[i], obb2);

      bool overlap_obb = obb1.overlap(obb2);
      bool overlap_sphere = solver.shapeIntersect(box1, box1_tf, sphere, transforms[i], NULL);
      EXPECT_TRUE(overlap_obb >= overlap_sphere);
    }

    {
      Ellipsoid<Scalar> ellipsoid(len, len, len);
      computeBV(ellipsoid, transforms[i], obb2);

      bool overlap_obb = obb1.overlap(obb2);
      bool overlap_ellipsoid = solver.shapeIntersect(box1, box1_tf, ellipsoid, transforms[i], NULL);
      EXPECT_TRUE(overlap_obb >= overlap_ellipsoid);
    }

    {
      Capsule<Scalar> capsule(len, 2 * len);
      computeBV(capsule, transforms[i], obb2);

      bool overlap_obb = obb1.overlap(obb2);
      bool overlap_capsule = solver.shapeIntersect(box1, box1_tf, capsule, transforms[i], NULL);
      EXPECT_TRUE(overlap_obb >= overlap_capsule);
    }

    {
      Cone<Scalar> cone(len, 2 * len);
      computeBV(cone, transforms[i], obb2);

      bool overlap_obb = obb1.overlap(obb2);
      bool overlap_cone = solver.shapeIntersect(box1, box1_tf, cone, transforms[i], NULL);
      EXPECT_TRUE(overlap_obb >= overlap_cone);
    }

    {
      Cylinder<Scalar> cylinder(len, 2 * len);
      computeBV(cylinder, transforms[i], obb2);

      bool overlap_obb = obb1.overlap(obb2);
      bool overlap_cylinder = solver.shapeIntersect(box1, box1_tf, cylinder, transforms[i], NULL);
      EXPECT_TRUE(overlap_obb >= overlap_cylinder);
    }
  }
}

template <typename Scalar>
void test_OBB_AABB_test()
{
  Scalar extents[] = {-1000, -1000, -1000, 1000, 1000, 1000};
  std::size_t n = 1000;

  Eigen::aligned_vector<Transform3<Scalar>> transforms;
  generateRandomTransforms(extents, transforms, n);

  AABB<Scalar> aabb1;
  aabb1.min_ = Vector3<Scalar>(-600, -600, -600);
  aabb1.max_ = Vector3<Scalar>(600, 600, 600);

  OBB<Scalar> obb1;
  convertBV(aabb1, Transform3<Scalar>::Identity(), obb1);

  for(std::size_t i = 0; i < transforms.size(); ++i)
  {
    AABB<Scalar> aabb;
    aabb.min_ = aabb1.min_ * 0.5;
    aabb.max_ = aabb1.max_ * 0.5;

    AABB<Scalar> aabb2 = translate(aabb, transforms[i].translation());

    OBB<Scalar> obb2;
    convertBV(aabb, Transform3<Scalar>(Translation3<Scalar>(transforms[i].translation())), obb2);

    bool overlap_aabb = aabb1.overlap(aabb2);
    bool overlap_obb = obb1.overlap(obb2);
    if(overlap_aabb != overlap_obb)
    {
      std::cout << aabb1.min_.transpose() << " " << aabb1.max_.transpose() << std::endl;
      std::cout << aabb2.min_.transpose() << " " << aabb2.max_.transpose() << std::endl;
      std::cout << obb1.frame.translation().transpose() << " " << obb1.extent.transpose() << " " << obb1.frame.linear().col(0).transpose() << " " << obb1.frame.linear().col(1).transpose() << " " << obb1.frame.linear().col(2).transpose() << std::endl;
      std::cout << obb2.frame.translation().transpose() << " " << obb2.extent.transpose() << " " << obb2.frame.linear().col(0).transpose() << " " << obb2.frame.linear().col(1).transpose() << " " << obb2.frame.linear().col(2).transpose() << std::endl;
    }

    EXPECT_TRUE(overlap_aabb == overlap_obb);
  }
  std::cout << std::endl;
}

template <typename Scalar>
void test_mesh_mesh()
{
  std::vector<Vector3<Scalar>> p1, p2;
  std::vector<Triangle> t1, t2;

  loadOBJFile(TEST_RESOURCES_DIR"/env.obj", p1, t1);
  loadOBJFile(TEST_RESOURCES_DIR"/rob.obj", p2, t2);

  Eigen::aligned_vector<Transform3<Scalar>> transforms;
  Scalar extents[] = {-3000, -3000, 0, 3000, 3000, 3000};
#if FCL_BUILD_TYPE_DEBUG
  std::size_t n = 1;
#else
  std::size_t n = 10;
#endif
  bool verbose = false;

  generateRandomTransforms(extents, transforms, n);

  // collision
  for(std::size_t i = 0; i < transforms.size(); ++i)
  {
    global_pairs<Scalar>().clear();
    global_pairs_now<Scalar>().clear();

    collide_Test<OBB<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);

    collide_Test<OBB<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test<OBB<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test<RSS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test<RSS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test<RSS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test<AABB<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test<AABB<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test<AABB<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test<KDOP<Scalar, 24> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test<KDOP<Scalar, 24> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test<KDOP<Scalar, 24> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test<KDOP<Scalar, 18> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test<KDOP<Scalar, 18> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test<KDOP<Scalar, 18> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test<KDOP<Scalar, 16> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test<KDOP<Scalar, 16> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test<KDOP<Scalar, 16> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test2<OBB<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test2<OBB<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test2<OBB<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test2<RSS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test2<RSS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test2<RSS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test2<AABB<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test2<AABB<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test2<AABB<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test2<KDOP<Scalar, 24> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test2<KDOP<Scalar, 24> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test2<KDOP<Scalar, 24> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test2<KDOP<Scalar, 18> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test2<KDOP<Scalar, 18> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test2<KDOP<Scalar, 18> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test2<KDOP<Scalar, 16> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test2<KDOP<Scalar, 16> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test2<KDOP<Scalar, 16> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test_Oriented<OBB<Scalar>, MeshCollisionTraversalNodeOBB<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);

    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test_Oriented<OBB<Scalar>, MeshCollisionTraversalNodeOBB<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);

    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test_Oriented<OBB<Scalar>, MeshCollisionTraversalNodeOBB<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test_Oriented<RSS<Scalar>, MeshCollisionTraversalNodeRSS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test_Oriented<RSS<Scalar>, MeshCollisionTraversalNodeRSS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test_Oriented<RSS<Scalar>, MeshCollisionTraversalNodeRSS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    test_collide_func<RSS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    test_collide_func<OBB<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    test_collide_func<AABB<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }


    collide_Test<kIOS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test<kIOS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test<kIOS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test2<kIOS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test2<kIOS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test2<kIOS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test_Oriented<kIOS<Scalar>, MeshCollisionTraversalNodekIOS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test_Oriented<kIOS<Scalar>, MeshCollisionTraversalNodekIOS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test_Oriented<kIOS<Scalar>, MeshCollisionTraversalNodekIOS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    test_collide_func<kIOS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    test_collide_func<kIOS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    test_collide_func<kIOS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test<OBBRSS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test<OBBRSS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test<OBBRSS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test2<OBBRSS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test2<OBBRSS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test2<OBBRSS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test_Oriented<OBBRSS<Scalar>, MeshCollisionTraversalNodeOBBRSS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test_Oriented<OBBRSS<Scalar>, MeshCollisionTraversalNodeOBBRSS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    collide_Test_Oriented<OBBRSS<Scalar>, MeshCollisionTraversalNodeOBBRSS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    test_collide_func<OBBRSS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    test_collide_func<OBBRSS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }

    test_collide_func<OBBRSS<Scalar>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER);
    EXPECT_TRUE(global_pairs<Scalar>().size() == global_pairs_now<Scalar>().size());
    for(std::size_t j = 0; j < global_pairs<Scalar>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<Scalar>()[j].b1 == global_pairs_now<Scalar>()[j].b1);
      EXPECT_TRUE(global_pairs<Scalar>()[j].b2 == global_pairs_now<Scalar>()[j].b2);
    }
  }
}

GTEST_TEST(FCL_COLLISION, OBB_Box_test)
{
//  test_OBB_Box_test<float>();
  test_OBB_Box_test<double>();
}

GTEST_TEST(FCL_COLLISION, OBB_shape_test)
{
//  test_OBB_shape_test<float>();
  test_OBB_shape_test<double>();
}

GTEST_TEST(FCL_COLLISION, OBB_AABB_test)
{
//  test_OBB_AABB_test<float>();
  test_OBB_AABB_test<double>();
}

GTEST_TEST(FCL_COLLISION, mesh_mesh)
{
//  test_mesh_mesh<float>();
  test_mesh_mesh<double>();
}

template<typename BV>
bool collide_Test2(const Transform3<typename BV::Scalar>& tf,
                   const std::vector<Vector3<typename BV::Scalar>>& vertices1, const std::vector<Triangle>& triangles1,
                   const std::vector<Vector3<typename BV::Scalar>>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose)
{
  using Scalar = typename BV::Scalar;

  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.reset(new BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new BVSplitter<BV>(split_method));

  std::vector<Vector3<Scalar>> vertices1_new(vertices1.size());
  for(unsigned int i = 0; i < vertices1_new.size(); ++i)
  {
    vertices1_new[i] = tf * vertices1[i];
  }


  m1.beginModel();
  m1.addSubModel(vertices1_new, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  Transform3<Scalar> pose1 = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose2 = Transform3<Scalar>::Identity();

  CollisionResult<Scalar> local_result;
  MeshCollisionTraversalNode<BV> node;

  if(!initialize<BV>(node, m1, pose1, m2, pose2,
                     CollisionRequest<Scalar>(num_max_contacts, enable_contact), local_result))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;

  collide(&node);


  if(local_result.numContacts() > 0)
  {
    if(global_pairs<Scalar>().size() == 0)
    {
      local_result.getContacts(global_pairs<Scalar>());
      std::sort(global_pairs<Scalar>().begin(), global_pairs<Scalar>().end());
    }
    else
    {
      local_result.getContacts(global_pairs_now<Scalar>());
      std::sort(global_pairs_now<Scalar>().begin(), global_pairs_now<Scalar>().end());
    }


    if(verbose)
      std::cout << "in collision " << local_result.numContacts() << ": " << std::endl;
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return true;
  }
  else
  {
    if(verbose) std::cout << "collision free " << std::endl;
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return false;
  }
}

template<typename BV>
bool collide_Test(const Transform3<typename BV::Scalar>& tf,
                  const std::vector<Vector3<typename BV::Scalar>>& vertices1, const std::vector<Triangle>& triangles1,
                  const std::vector<Vector3<typename BV::Scalar>>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose)
{
  using Scalar = typename BV::Scalar;

  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.reset(new BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new BVSplitter<BV>(split_method));

  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  Transform3<Scalar> pose1(tf);
  Transform3<Scalar> pose2 = Transform3<Scalar>::Identity();

  CollisionResult<Scalar> local_result;
  MeshCollisionTraversalNode<BV> node;

  if(!initialize<BV>(node, m1, pose1, m2, pose2,
                     CollisionRequest<Scalar>(num_max_contacts, enable_contact), local_result))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;

  collide(&node);


  if(local_result.numContacts() > 0)
  {
    if(global_pairs<Scalar>().size() == 0)
    {
      local_result.getContacts(global_pairs<Scalar>());
      std::sort(global_pairs<Scalar>().begin(), global_pairs<Scalar>().end());
    }
    else
    {
      local_result.getContacts(global_pairs_now<Scalar>());
      std::sort(global_pairs_now<Scalar>().begin(), global_pairs_now<Scalar>().end());
    }

    if(verbose)
      std::cout << "in collision " << local_result.numContacts() << ": " << std::endl;
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return true;
  }
  else
  {
    if(verbose) std::cout << "collision free " << std::endl;
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return false;
  }
}

template<typename BV, typename TraversalNode>
bool collide_Test_Oriented(const Transform3<typename BV::Scalar>& tf,
                           const std::vector<Vector3<typename BV::Scalar>>& vertices1, const std::vector<Triangle>& triangles1,
                           const std::vector<Vector3<typename BV::Scalar>>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose)
{
  using Scalar = typename BV::Scalar;

  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.reset(new BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new BVSplitter<BV>(split_method));

  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  Transform3<Scalar> pose1(tf);
  Transform3<Scalar> pose2 = Transform3<Scalar>::Identity();

  CollisionResult<Scalar> local_result;
  TraversalNode node;
  if(!initialize(node, (const BVHModel<BV>&)m1, pose1, (const BVHModel<BV>&)m2, pose2, 
                 CollisionRequest<Scalar>(num_max_contacts, enable_contact), local_result))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;

  collide(&node);

  if(local_result.numContacts() > 0)
  {
    if(global_pairs<Scalar>().size() == 0)
    {
      local_result.getContacts(global_pairs<Scalar>());
      std::sort(global_pairs<Scalar>().begin(), global_pairs<Scalar>().end());
    }
    else
    {
      local_result.getContacts(global_pairs_now<Scalar>());
      std::sort(global_pairs_now<Scalar>().begin(), global_pairs_now<Scalar>().end());
    }

    if(verbose)
      std::cout << "in collision " << local_result.numContacts() << ": " << std::endl;
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return true;
  }
  else
  {
    if(verbose) std::cout << "collision free " << std::endl;
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return false;
  }
}


template<typename BV>
bool test_collide_func(const Transform3<typename BV::Scalar>& tf,
                       const std::vector<Vector3<typename BV::Scalar>>& vertices1, const std::vector<Triangle>& triangles1,
                       const std::vector<Vector3<typename BV::Scalar>>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method)
{
  using Scalar = typename BV::Scalar;

  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.reset(new BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new BVSplitter<BV>(split_method));

  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  Transform3<Scalar> pose1(tf);
  Transform3<Scalar> pose2 = Transform3<Scalar>::Identity();

  std::vector<Contact<Scalar>> contacts;

  CollisionRequest<Scalar> request(num_max_contacts, enable_contact);
  CollisionResult<Scalar> result;
  int num_contacts = collide(&m1, pose1, &m2, pose2, request, result);

  result.getContacts(contacts);

  global_pairs_now<Scalar>().resize(num_contacts);

  for(int i = 0; i < num_contacts; ++i)
  {
    global_pairs_now<Scalar>()[i].b1 = contacts[i].b1;
    global_pairs_now<Scalar>()[i].b2 = contacts[i].b2;
  }

  std::sort(global_pairs_now<Scalar>().begin(), global_pairs_now<Scalar>().end());

  if(num_contacts > 0) return true;
  else return false;
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

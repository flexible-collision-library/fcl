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

#include "fcl/narrowphase/gjk_solver_indep.h"
#include "fcl/narrowphase/gjk_solver_libccd.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"
#include "fcl/distance.h"
#include "fcl/collision.h"
#include "test_fcl_utility.h"


using namespace fcl;

template <typename Scalar>
std::array<Scalar, 6>& extents()
{
  static std::array<Scalar, 6> static_extents = {0, 0, 0, 10, 10, 10};
  return static_extents;
}

template <typename Scalar>
void test_consistency_distance_spheresphere_libccd()
{
  Sphere<Scalar> s1(20);
  Sphere<Scalar> s2(20);
  BVHModel<RSS<Scalar>> s1_rss;
  BVHModel<RSS<Scalar>> s2_rss;

  generateBVHModel(s1_rss, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_rss, s2, Transform3<Scalar>::Identity(), 16, 16);

  DistanceRequest<Scalar> request;
  DistanceResult<Scalar> res, res1;

  Transform3<Scalar> pose = Transform3<Scalar>::Identity();

  pose.translation() = Vector3<Scalar>(50, 0, 0);

  res.clear(); res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

  res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  
  res1.clear();
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3<Scalar> t;
    generateRandomTransform(extents<Scalar>(), t);
    
    Transform3<Scalar> pose1(t);
    Transform3<Scalar> pose2 = t * pose;

    res.clear(); res1.clear();
    distance(&s1, pose1, &s2, pose2, request, res);
    distance(&s1_rss, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

    res1.clear();
    distance(&s1, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

    res1.clear();
    distance(&s1_rss, pose1, &s2, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  }

  pose.translation() = Vector3<Scalar>(40.1, 0, 0);

  res.clear(), res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);


  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3<Scalar> t;
    generateRandomTransform(extents<Scalar>(), t);
    
    Transform3<Scalar> pose1(t);
    Transform3<Scalar> pose2 = t * pose;

    res.clear(); res1.clear();
    distance(&s1, pose1, &s2, pose2, request, res);
    distance(&s1_rss, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

    res1.clear();
    distance(&s1, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

    res1.clear();
    distance(&s1_rss, pose1, &s2, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  }
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_distance_spheresphere_libccd)
{
  test_consistency_distance_spheresphere_libccd<float>();
  test_consistency_distance_spheresphere_libccd<double>();
}

template <typename Scalar>
void test_consistency_distance_ellipsoidellipsoid_libccd()
{
  Ellipsoid<Scalar> s1(20, 40, 50);
  Ellipsoid<Scalar> s2(10, 10, 10);
  BVHModel<RSS<Scalar>> s1_rss;
  BVHModel<RSS<Scalar>> s2_rss;

  generateBVHModel(s1_rss, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_rss, s2, Transform3<Scalar>::Identity(), 16, 16);

  DistanceRequest<Scalar> request;
  DistanceResult<Scalar> res, res1;

  Transform3<Scalar> pose = Transform3<Scalar>::Identity();

  pose.translation() = Vector3<Scalar>(40, 0, 0);

  res.clear(); res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

  res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

  res1.clear();
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3<Scalar> t;
    generateRandomTransform(extents<Scalar>(), t);

    Transform3<Scalar> pose1(t);
    Transform3<Scalar> pose2 = t * pose;

    res.clear(); res1.clear();
    distance(&s1, pose1, &s2, pose2, request, res);
    distance(&s1_rss, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

    res1.clear();
    distance(&s1, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

    res1.clear();
    distance(&s1_rss, pose1, &s2, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  }

  pose.translation() = Vector3<Scalar>(30.1, 0, 0);

  res.clear(), res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3<Scalar> t;
    generateRandomTransform(extents<Scalar>(), t);

    Transform3<Scalar> pose1(t);
    Transform3<Scalar> pose2 = t * pose;

    res.clear(); res1.clear();
    distance(&s1, pose1, &s2, pose2, request, res);
    distance(&s1_rss, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

    res1.clear();
    distance(&s1, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

    res1.clear();
    distance(&s1_rss, pose1, &s2, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  }
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_distance_ellipsoidellipsoid_libccd)
{
  test_consistency_distance_ellipsoidellipsoid_libccd<float>();
  test_consistency_distance_ellipsoidellipsoid_libccd<double>();
}

template <typename Scalar>
void test_consistency_distance_boxbox_libccd()
{
  Box<Scalar> s1(20, 40, 50);
  Box<Scalar> s2(10, 10, 10);

  BVHModel<RSS<Scalar>> s1_rss;
  BVHModel<RSS<Scalar>> s2_rss;

  generateBVHModel(s1_rss, s1, Transform3<Scalar>::Identity());
  generateBVHModel(s2_rss, s2, Transform3<Scalar>::Identity());

  DistanceRequest<Scalar> request;
  DistanceResult<Scalar> res, res1;

  Transform3<Scalar> pose = Transform3<Scalar>::Identity();

  pose.translation() = Vector3<Scalar>(50, 0, 0);

  res.clear(); res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);
  
  res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);

  res1.clear();
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);

  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3<Scalar> t;
    generateRandomTransform(extents<Scalar>(), t);
    
    Transform3<Scalar> pose1(t);
    Transform3<Scalar> pose2 = t * pose;

    res.clear(); res1.clear();
    distance(&s1, pose1, &s2, pose2, request, res);
    distance(&s1_rss, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);

    res1.clear();
    distance(&s1, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);

    res1.clear();
    distance(&s1_rss, pose1, &s2, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);
  }

  pose.translation() = Vector3<Scalar>(15.1, 0, 0);
  
  res.clear(); res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  
  res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3<Scalar> t;
    generateRandomTransform(extents<Scalar>(), t);
    
    Transform3<Scalar> pose1(t);
    Transform3<Scalar> pose2 = t * pose;

    res.clear(); res1.clear();
    distance(&s1, pose1, &s2, pose2, request, res);
    distance(&s1_rss, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

    res1.clear();
    distance(&s1, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

    res1.clear();
    distance(&s1_rss, pose1, &s2, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  }
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_distance_boxbox_libccd)
{
  test_consistency_distance_boxbox_libccd<float>();
  test_consistency_distance_boxbox_libccd<double>();
}

template <typename Scalar>
void test_consistency_distance_cylindercylinder_libccd()
{
  Cylinder<Scalar> s1(5, 10);
  Cylinder<Scalar> s2(5, 10);

  BVHModel<RSS<Scalar>> s1_rss;
  BVHModel<RSS<Scalar>> s2_rss;

  generateBVHModel(s1_rss, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_rss, s2, Transform3<Scalar>::Identity(), 16, 16);

  DistanceRequest<Scalar> request;
  DistanceResult<Scalar> res, res1;

  Transform3<Scalar> pose = Transform3<Scalar>::Identity();

  pose.translation() = Vector3<Scalar>(20, 0, 0);

  res.clear(); res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);
  
  res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);
  
  res1.clear();
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);
  
  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3<Scalar> t;
    generateRandomTransform(extents<Scalar>(), t);
    
    Transform3<Scalar> pose1(t);
    Transform3<Scalar> pose2 = t * pose;

    res.clear(); res1.clear();
    distance(&s1, pose1, &s2, pose2, request, res);
    distance(&s1_rss, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);

    res1.clear();
    distance(&s1, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);

    res1.clear();
    distance(&s1_rss, pose1, &s2, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);
  }
  
  pose.translation() = Vector3<Scalar>(15, 0, 0); // libccd cannot use small value here :(
  
  res.clear(); res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  
  res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  
  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3<Scalar> t;
    generateRandomTransform(extents<Scalar>(), t);
    
    Transform3<Scalar> pose1(t);
    Transform3<Scalar> pose2 = t * pose;

    res.clear(); res1.clear();
    distance(&s1, pose1, &s2, pose2, request, res);
    distance(&s1_rss, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

    res1.clear();
    distance(&s1, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

    res1.clear();
    distance(&s1_rss, pose1, &s2, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  }
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_distance_cylindercylinder_libccd)
{
  test_consistency_distance_cylindercylinder_libccd<float>();
  test_consistency_distance_cylindercylinder_libccd<double>();
}

template <typename Scalar>
void test_consistency_distance_conecone_libccd()
{
  Cone<Scalar> s1(5, 10);
  Cone<Scalar> s2(5, 10);

  BVHModel<RSS<Scalar>> s1_rss;
  BVHModel<RSS<Scalar>> s2_rss;

  generateBVHModel(s1_rss, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_rss, s2, Transform3<Scalar>::Identity(), 16, 16);

  DistanceRequest<Scalar> request;
  DistanceResult<Scalar> res, res1;

  Transform3<Scalar> pose = Transform3<Scalar>::Identity();

  pose.translation() = Vector3<Scalar>(20, 0, 0);
  
  res.clear(); res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  
  res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  
  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3<Scalar> t;
    generateRandomTransform(extents<Scalar>(), t);
    
    Transform3<Scalar> pose1(t);
    Transform3<Scalar> pose2 = t * pose;

    res.clear(); res1.clear();
    distance(&s1, pose1, &s2, pose2, request, res);
    distance(&s1_rss, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

    res1.clear();
    distance(&s1, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

    res1.clear();
    distance(&s1_rss, pose1, &s2, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  }
  
  pose.translation() = Vector3<Scalar>(15, 0, 0); // libccd cannot use small value here :(
  
  res.clear(); res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  
  res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  
  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3<Scalar> t;
    generateRandomTransform(extents<Scalar>(), t);
    
    Transform3<Scalar> pose1(t);
    Transform3<Scalar> pose2 = t * pose;

    res.clear(); res1.clear();
    distance(&s1, pose1, &s2, pose2, request, res);
    distance(&s1_rss, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

    res1.clear();
    distance(&s1, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

    res1.clear();
    distance(&s1_rss, pose1, &s2, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  }
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_distance_conecone_libccd)
{
  test_consistency_distance_conecone_libccd<float>();
  test_consistency_distance_conecone_libccd<double>();
}

template <typename Scalar>
void test_consistency_distance_spheresphere_GJK()
{
  Sphere<Scalar> s1(20);
  Sphere<Scalar> s2(20);
  BVHModel<RSS<Scalar>> s1_rss;
  BVHModel<RSS<Scalar>> s2_rss;

  generateBVHModel(s1_rss, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_rss, s2, Transform3<Scalar>::Identity(), 16, 16);

  DistanceRequest<Scalar> request;
  request.gjk_solver_type = GST_INDEP;
  DistanceResult<Scalar> res, res1;

  Transform3<Scalar> pose = Transform3<Scalar>::Identity();

  pose.translation() = Vector3<Scalar>(50, 0, 0);

  res.clear(); res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

  res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  
  res1.clear();
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);


  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3<Scalar> t;
    generateRandomTransform(extents<Scalar>(), t);
    
    Transform3<Scalar> pose1(t);
    Transform3<Scalar> pose2 = t * pose;

    res.clear(); res1.clear();
    distance(&s1, pose1, &s2, pose2, request, res);
    distance(&s1_rss, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

    res1.clear();
    distance(&s1, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

    res1.clear();
    distance(&s1_rss, pose1, &s2, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  }

  pose.translation() = Vector3<Scalar>(40.1, 0, 0);

  res.clear(); res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 4);


  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3<Scalar> t;
    generateRandomTransform(extents<Scalar>(), t);
    
    Transform3<Scalar> pose1(t);
    Transform3<Scalar> pose2 = t * pose;

    res.clear(); res1.clear();
    distance(&s1, pose1, &s2, pose2, request, res);
    distance(&s1_rss, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

    res1.clear();
    distance(&s1, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

    res1.clear();
    distance(&s1_rss, pose1, &s2, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 4);
  }
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_distance_spheresphere_GJK)
{
  test_consistency_distance_spheresphere_GJK<float>();
  test_consistency_distance_spheresphere_GJK<double>();
}

template <typename Scalar>
void test_consistency_distance_ellipsoidellipsoid_GJK()
{
  Ellipsoid<Scalar> s1(20, 40, 50);
  Ellipsoid<Scalar> s2(10, 10, 10);
  BVHModel<RSS<Scalar>> s1_rss;
  BVHModel<RSS<Scalar>> s2_rss;

  generateBVHModel(s1_rss, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_rss, s2, Transform3<Scalar>::Identity(), 16, 16);

  DistanceRequest<Scalar> request;
  request.gjk_solver_type = GST_INDEP;
  DistanceResult<Scalar> res, res1;

  Transform3<Scalar> pose = Transform3<Scalar>::Identity();

  pose.translation() = Vector3<Scalar>(40, 0, 0);

  res.clear(); res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

  res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

  res1.clear();
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);


  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3<Scalar> t;
    generateRandomTransform(extents<Scalar>(), t);

    Transform3<Scalar> pose1(t);
    Transform3<Scalar> pose2 = t * pose;

    res.clear(); res1.clear();
    distance(&s1, pose1, &s2, pose2, request, res);
    distance(&s1_rss, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

    res1.clear();
    distance(&s1, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

    res1.clear();
    distance(&s1_rss, pose1, &s2, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  }

  pose.translation() = Vector3<Scalar>(30.1, 0, 0);

  res.clear(); res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 4);


  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3<Scalar> t;
    generateRandomTransform(extents<Scalar>(), t);

    Transform3<Scalar> pose1(t);
    Transform3<Scalar> pose2 = t * pose;

    res.clear(); res1.clear();
    distance(&s1, pose1, &s2, pose2, request, res);
    distance(&s1_rss, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

    res1.clear();
    distance(&s1, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

    res1.clear();
    distance(&s1_rss, pose1, &s2, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 4);
  }
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_distance_ellipsoidellipsoid_GJK)
{
  test_consistency_distance_ellipsoidellipsoid_GJK<float>();
  test_consistency_distance_ellipsoidellipsoid_GJK<double>();
}

template <typename Scalar>
void test_consistency_distance_boxbox_GJK()
{
  Box<Scalar> s1(20, 40, 50);
  Box<Scalar> s2(10, 10, 10);

  BVHModel<RSS<Scalar>> s1_rss;
  BVHModel<RSS<Scalar>> s2_rss;

  generateBVHModel(s1_rss, s1, Transform3<Scalar>::Identity());
  generateBVHModel(s2_rss, s2, Transform3<Scalar>::Identity());

  DistanceRequest<Scalar> request;
  request.gjk_solver_type = GST_INDEP;
  DistanceResult<Scalar> res, res1;

  Transform3<Scalar> pose = Transform3<Scalar>::Identity();

  pose.translation() = Vector3<Scalar>(50, 0, 0);

  res.clear(); res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);
  
  res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);

  res1.clear();
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);

  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3<Scalar> t;
    generateRandomTransform(extents<Scalar>(), t);
    
    Transform3<Scalar> pose1(t);
    Transform3<Scalar> pose2 = t * pose;

    res.clear(); res1.clear();
    distance(&s1, pose1, &s2, pose2, request, res);
    distance(&s1_rss, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);

    res1.clear();
    distance(&s1, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);

    res1.clear();
    distance(&s1_rss, pose1, &s2, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);
  }

  pose.translation() = Vector3<Scalar>(15.1, 0, 0);
  
  res.clear(); res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  
  res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3<Scalar> t;
    generateRandomTransform(extents<Scalar>(), t);
    
    Transform3<Scalar> pose1(t);
    Transform3<Scalar> pose2 = t * pose;

    res.clear(); res1.clear();
    distance(&s1, pose1, &s2, pose2, request, res);
    distance(&s1_rss, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

    res1.clear();
    distance(&s1, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

    res1.clear();
    distance(&s1_rss, pose1, &s2, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  }
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_distance_boxbox_GJK)
{
  test_consistency_distance_boxbox_GJK<float>();
  test_consistency_distance_boxbox_GJK<double>();
}

template <typename Scalar>
void test_consistency_distance_cylindercylinder_GJK()
{
  Cylinder<Scalar> s1(5, 10);
  Cylinder<Scalar> s2(5, 10);

  BVHModel<RSS<Scalar>> s1_rss;
  BVHModel<RSS<Scalar>> s2_rss;

  generateBVHModel(s1_rss, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_rss, s2, Transform3<Scalar>::Identity(), 16, 16);
  
  DistanceRequest<Scalar> request;
  request.gjk_solver_type = GST_INDEP;
  DistanceResult<Scalar> res, res1;

  Transform3<Scalar> pose = Transform3<Scalar>::Identity();

  pose.translation() = Vector3<Scalar>(20, 0, 0);
  
  res.clear(); res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  
  res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  
  res1.clear();
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  
  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3<Scalar> t;
    generateRandomTransform(extents<Scalar>(), t);
    
    Transform3<Scalar> pose1(t);
    Transform3<Scalar> pose2 = t * pose;

    res.clear(); res1.clear();
    distance(&s1, pose1, &s2, pose2, request, res);
    distance(&s1_rss, pose1, &s2_rss, pose2, request, res1);
    if(fabs(res1.min_distance - res.min_distance) / res.min_distance > 0.05)
      std::cout << "low resolution: " << res1.min_distance << " " << res.min_distance << std::endl;
    else
      EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

    res1.clear();
    distance(&s1, pose1, &s2_rss, pose2, request, res1);
    if(fabs(res1.min_distance - res.min_distance) / res.min_distance > 0.05)
      std::cout << "low resolution: " << res1.min_distance << " " << res.min_distance << std::endl;
    else
      EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

    res1.clear();
    distance(&s1_rss, pose1, &s2, pose2, request, res1);
    if(fabs(res1.min_distance - res.min_distance) / res.min_distance > 0.05)
      std::cout << "low resolution: " << res1.min_distance << " " << res.min_distance << std::endl;
    else
      EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  }
  
  pose.translation() = Vector3<Scalar>(10.1, 0, 0);
  
  res.clear(); res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  
  res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  
  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3<Scalar> t;
    generateRandomTransform(extents<Scalar>(), t);
    
    Transform3<Scalar> pose1(t);
    Transform3<Scalar> pose2 = t * pose;

    res.clear(); res1.clear();
    distance(&s1, pose1, &s2, pose2, request, res);
    distance(&s1_rss, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

    res1.clear();
    distance(&s1, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

    res1.clear();
    distance(&s1_rss, pose1, &s2, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  }
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_distance_cylindercylinder_GJK)
{
  test_consistency_distance_cylindercylinder_GJK<float>();
  test_consistency_distance_cylindercylinder_GJK<double>();
}

template <typename Scalar>
void test_consistency_distance_conecone_GJK()
{
  Cone<Scalar> s1(5, 10);
  Cone<Scalar> s2(5, 10);

  BVHModel<RSS<Scalar>> s1_rss;
  BVHModel<RSS<Scalar>> s2_rss;

  generateBVHModel(s1_rss, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_rss, s2, Transform3<Scalar>::Identity(), 16, 16);

  DistanceRequest<Scalar> request;
  request.gjk_solver_type = GST_INDEP;
  DistanceResult<Scalar> res, res1;

  Transform3<Scalar> pose = Transform3<Scalar>::Identity();

  pose.translation() = Vector3<Scalar>(20, 0, 0);

  res.clear(); res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  
  res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  
  res1.clear();
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  
  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3<Scalar> t;
    generateRandomTransform(extents<Scalar>(), t);
    
    Transform3<Scalar> pose1(t);
    Transform3<Scalar> pose2 = t * pose;

    res.clear(); res1.clear();
    distance(&s1, pose1, &s2, pose2, request, res);
    distance(&s1_rss, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

    res1.clear();
    distance(&s1, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

    res1.clear();
    distance(&s1_rss, pose1, &s2, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  }
  
  pose.translation() = Vector3<Scalar>(10.1, 0, 0);
  
  res.clear(); res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  
  res1.clear();
  distance(&s1, Transform3<Scalar>::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1_rss, Transform3<Scalar>::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  
  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3<Scalar> t;
    generateRandomTransform(extents<Scalar>(), t);
    
    Transform3<Scalar> pose1(t);
    Transform3<Scalar> pose2 = t * pose;

    res.clear(); res1.clear();
    distance(&s1, pose1, &s2, pose2, request, res);
    distance(&s1_rss, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

    res1.clear();
    distance(&s1, pose1, &s2_rss, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

    res1.clear();
    distance(&s1_rss, pose1, &s2, pose2, request, res1);
    EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  }
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_distance_conecone_GJK)
{
  test_consistency_distance_conecone_GJK<float>();
  test_consistency_distance_conecone_GJK<double>();
}

template <typename Scalar>
void test_consistency_collision_spheresphere_libccd()
{
  Sphere<Scalar> s1(20);
  Sphere<Scalar> s2(10);
  BVHModel<AABB<Scalar>> s1_aabb;
  BVHModel<AABB<Scalar>> s2_aabb;
  BVHModel<OBB<Scalar>> s1_obb;
  BVHModel<OBB<Scalar>> s2_obb;

  generateBVHModel(s1_aabb, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_aabb, s2, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s1_obb, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_obb, s2, Transform3<Scalar>::Identity(), 16, 16);

  CollisionRequest<Scalar> request;
  CollisionResult<Scalar> result;

  bool res;

  Transform3<Scalar> pose = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose_aabb = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose_obb = Transform3<Scalar>::Identity();


  // s2 is within s1
  // both are shapes --> collision
  // s1 is shape, s2 is mesh --> in collision
  // s1 is mesh, s2 is shape --> collision free
  // all are reasonable
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);

  pose.translation() = Vector3<Scalar>(40, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(40, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(40, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);

  pose.translation() = Vector3<Scalar>(30, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(30, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(30, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);

  pose.translation() = Vector3<Scalar>(29.9, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(29.8, 0, 0); // 29.9 fails, result depends on mesh precision
  pose_obb.translation() = Vector3<Scalar>(29.8, 0, 0); // 29.9 fails, result depends on mesh precision

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);


  pose.translation() = Vector3<Scalar>(-29.9, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(-29.8, 0, 0); // 29.9 fails, result depends on mesh precision
  pose_obb.translation() = Vector3<Scalar>(-29.8, 0, 0); // 29.9 fails, result depends on mesh precision

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3<Scalar>(-30, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(-30, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(-30, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_collision_spheresphere_libccd)
{
  test_consistency_collision_spheresphere_libccd<float>();
  test_consistency_collision_spheresphere_libccd<double>();
}

template <typename Scalar>
void test_consistency_collision_ellipsoidellipsoid_libccd()
{
  Ellipsoid<Scalar> s1(20, 40, 50);
  Ellipsoid<Scalar> s2(10, 10, 10);
  BVHModel<AABB<Scalar>> s1_aabb;
  BVHModel<AABB<Scalar>> s2_aabb;
  BVHModel<OBB<Scalar>> s1_obb;
  BVHModel<OBB<Scalar>> s2_obb;

  generateBVHModel(s1_aabb, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_aabb, s2, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s1_obb, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_obb, s2, Transform3<Scalar>::Identity(), 16, 16);

  CollisionRequest<Scalar> request;
  CollisionResult<Scalar> result;

  bool res;

  Transform3<Scalar> pose = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose_aabb = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose_obb = Transform3<Scalar>::Identity();


  // s2 is within s1
  // both are shapes --> collision
  // s1 is shape, s2 is mesh --> in collision
  // s1 is mesh, s2 is shape --> collision free
  // all are reasonable
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);

  pose.translation() = Vector3<Scalar>(40, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(40, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(40, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);

  pose.translation() = Vector3<Scalar>(30, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(30, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(30, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);  // libccd cannot detect collision when two ellipsoid is exactly touching each other
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, Transform3<Scalar>(Translation3<Scalar>(Vector3<Scalar>(29.999, 0, 0))), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);

  pose.translation() = Vector3<Scalar>(29.9, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(29.9, 0, 0); // 29.9 fails, result depends on mesh precision
  pose_obb.translation() = Vector3<Scalar>(29.9, 0, 0); // 29.9 fails, result depends on mesh precision

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3<Scalar>(-29.9, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(-29.9, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(-29.9, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3<Scalar>(-30, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(-30, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(-30, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_collision_ellipsoidellipsoid_libccd)
{
  test_consistency_collision_ellipsoidellipsoid_libccd<float>();
  test_consistency_collision_ellipsoidellipsoid_libccd<double>();
}

template <typename Scalar>
void test_consistency_collision_boxbox_libccd()
{
  Box<Scalar> s1(20, 40, 50);
  Box<Scalar> s2(10, 10, 10);

  BVHModel<AABB<Scalar>> s1_aabb;
  BVHModel<AABB<Scalar>> s2_aabb;
  BVHModel<OBB<Scalar>> s1_obb;
  BVHModel<OBB<Scalar>> s2_obb;

  generateBVHModel(s1_aabb, s1, Transform3<Scalar>::Identity());
  generateBVHModel(s2_aabb, s2, Transform3<Scalar>::Identity());
  generateBVHModel(s1_obb, s1, Transform3<Scalar>::Identity());
  generateBVHModel(s2_obb, s2, Transform3<Scalar>::Identity());

  CollisionRequest<Scalar> request;
  CollisionResult<Scalar> result;

  bool res;

  Transform3<Scalar> pose = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose_aabb = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose_obb = Transform3<Scalar>::Identity();

  // s2 is within s1
  // both are shapes --> collision
  // s1 is shape, s2 is mesh --> in collision
  // s1 is mesh, s2 is shape --> collision free
  // all are reasonable
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);

  pose.translation() = Vector3<Scalar>(15.01, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(15.01, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(15.01, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);

  pose.translation() = Vector3<Scalar>(14.99, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(14.99, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(14.99, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_collision_boxbox_libccd)
{
  test_consistency_collision_boxbox_libccd<float>();
  test_consistency_collision_boxbox_libccd<double>();
}

template <typename Scalar>
void test_consistency_collision_spherebox_libccd()
{
  Sphere<Scalar> s1(20);
  Box<Scalar> s2(5, 5, 5);

  BVHModel<AABB<Scalar>> s1_aabb;
  BVHModel<AABB<Scalar>> s2_aabb;
  BVHModel<OBB<Scalar>> s1_obb;
  BVHModel<OBB<Scalar>> s2_obb;

  generateBVHModel(s1_aabb, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_aabb, s2, Transform3<Scalar>::Identity());
  generateBVHModel(s1_obb, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_obb, s2, Transform3<Scalar>::Identity());

  CollisionRequest<Scalar> request;
  CollisionResult<Scalar> result;

  bool res;

  Transform3<Scalar> pose = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose_aabb = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose_obb = Transform3<Scalar>::Identity();

  // s2 is within s1
  // both are shapes --> collision
  // s1 is shape, s2 is mesh --> in collision
  // s1 is mesh, s2 is shape --> collision free
  // all are reasonable
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);

  pose.translation() = Vector3<Scalar>(22.4, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(22.4, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(22.4, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3<Scalar>(22.51, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(22.51, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(22.51, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_collision_spherebox_libccd)
{
  test_consistency_collision_spherebox_libccd<float>();
  test_consistency_collision_spherebox_libccd<double>();
}

template <typename Scalar>
void test_consistency_collision_cylindercylinder_libccd()
{
  Cylinder<Scalar> s1(5, 10);
  Cylinder<Scalar> s2(5, 10);

  BVHModel<AABB<Scalar>> s1_aabb;
  BVHModel<AABB<Scalar>> s2_aabb;
  BVHModel<OBB<Scalar>> s1_obb;
  BVHModel<OBB<Scalar>> s2_obb;

  generateBVHModel(s1_aabb, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_aabb, s2, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s1_obb, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_obb, s2, Transform3<Scalar>::Identity(), 16, 16);

  CollisionRequest<Scalar> request;
  CollisionResult<Scalar> result;

  bool res;

  Transform3<Scalar> pose = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose_aabb = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose_obb = Transform3<Scalar>::Identity();

  pose.translation() = Vector3<Scalar>(9.99, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(9.99, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(9.99, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3<Scalar>(10.01, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(10.01, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(10.01, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_collision_cylindercylinder_libccd)
{
  test_consistency_collision_cylindercylinder_libccd<float>();
  test_consistency_collision_cylindercylinder_libccd<double>();
}

template <typename Scalar>
void test_consistency_collision_conecone_libccd()
{
  Cone<Scalar> s1(5, 10);
  Cone<Scalar> s2(5, 10);

  BVHModel<AABB<Scalar>> s1_aabb;
  BVHModel<AABB<Scalar>> s2_aabb;
  BVHModel<OBB<Scalar>> s1_obb;
  BVHModel<OBB<Scalar>> s2_obb;

  generateBVHModel(s1_aabb, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_aabb, s2, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s1_obb, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_obb, s2, Transform3<Scalar>::Identity(), 16, 16);

  CollisionRequest<Scalar> request;
  CollisionResult<Scalar> result;

  bool res;

  Transform3<Scalar> pose = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose_aabb = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose_obb = Transform3<Scalar>::Identity();

  pose.translation() = Vector3<Scalar>(9.9, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(9.9, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(9.9, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3<Scalar>(10.1, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(10.1, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(10.1, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);

  pose.translation() = Vector3<Scalar>(0, 0, 9.9);
  pose_aabb.translation() = Vector3<Scalar>(0, 0, 9.9);
  pose_obb.translation() = Vector3<Scalar>(0, 0, 9.9);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3<Scalar>(0, 0, 10.1);
  pose_aabb.translation() = Vector3<Scalar>(0, 0, 10.1);
  pose_obb.translation() = Vector3<Scalar>(0, 0, 10.1);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_collision_conecone_libccd)
{
  test_consistency_collision_conecone_libccd<float>();
  test_consistency_collision_conecone_libccd<double>();
}

template <typename Scalar>
void test_consistency_collision_spheresphere_GJK()
{
  Sphere<Scalar> s1(20);
  Sphere<Scalar> s2(10);
  BVHModel<AABB<Scalar>> s1_aabb;
  BVHModel<AABB<Scalar>> s2_aabb;
  BVHModel<OBB<Scalar>> s1_obb;
  BVHModel<OBB<Scalar>> s2_obb;

  generateBVHModel(s1_aabb, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_aabb, s2, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s1_obb, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_obb, s2, Transform3<Scalar>::Identity(), 16, 16);

  CollisionRequest<Scalar> request;
  request.gjk_solver_type = GST_INDEP;

  CollisionResult<Scalar> result;

  bool res;

  Transform3<Scalar> pose = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose_aabb = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose_obb = Transform3<Scalar>::Identity();


  // s2 is within s1
  // both are shapes --> collision
  // s1 is shape, s2 is mesh --> in collision
  // s1 is mesh, s2 is shape --> collision free
  // all are reasonable
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);

  pose.translation() = Vector3<Scalar>(40, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(40, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(40, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);

  pose.translation() = Vector3<Scalar>(30, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(30, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(30, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);

  pose.translation() = Vector3<Scalar>(29.9, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(29.8, 0, 0); // 29.9 fails, result depends on mesh precision
  pose_obb.translation() = Vector3<Scalar>(29.8, 0, 0); // 29.9 fails, result depends on mesh precision

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);


  pose.translation() = Vector3<Scalar>(-29.9, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(-29.8, 0, 0); // 29.9 fails, result depends on mesh precision
  pose_obb.translation() = Vector3<Scalar>(-29.8, 0, 0); // 29.9 fails, result depends on mesh precision

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3<Scalar>(-30, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(-30, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(-30, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_collision_spheresphere_GJK)
{
  test_consistency_collision_spheresphere_GJK<float>();
  test_consistency_collision_spheresphere_GJK<double>();
}

template <typename Scalar>
void test_consistency_collision_ellipsoidellipsoid_GJK()
{
  Ellipsoid<Scalar> s1(20, 40, 50);
  Ellipsoid<Scalar> s2(10, 10, 10);
  BVHModel<AABB<Scalar>> s1_aabb;
  BVHModel<AABB<Scalar>> s2_aabb;
  BVHModel<OBB<Scalar>> s1_obb;
  BVHModel<OBB<Scalar>> s2_obb;

  generateBVHModel(s1_aabb, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_aabb, s2, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s1_obb, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_obb, s2, Transform3<Scalar>::Identity(), 16, 16);

  CollisionRequest<Scalar> request;
  request.gjk_solver_type = GST_INDEP;

  CollisionResult<Scalar> result;

  bool res;

  Transform3<Scalar> pose = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose_aabb = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose_obb = Transform3<Scalar>::Identity();


  // s2 is within s1
  // both are shapes --> collision
  // s1 is shape, s2 is mesh --> in collision
  // s1 is mesh, s2 is shape --> collision free
  // all are reasonable
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);

  pose.translation() = Vector3<Scalar>(40, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(40, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(40, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);

  pose.translation() = Vector3<Scalar>(30, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(30, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(30, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);

  pose.translation() = Vector3<Scalar>(29.9, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(29.9, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(29.9, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);


  pose.translation() = Vector3<Scalar>(-29.9, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(-29.9, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(-29.9, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3<Scalar>(-30, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(-30, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(-30, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_collision_ellipsoidellipsoid_GJK)
{
  test_consistency_collision_ellipsoidellipsoid_GJK<float>();
  test_consistency_collision_ellipsoidellipsoid_GJK<double>();
}

template <typename Scalar>
void test_consistency_collision_boxbox_GJK()
{
  Box<Scalar> s1(20, 40, 50);
  Box<Scalar> s2(10, 10, 10);

  BVHModel<AABB<Scalar>> s1_aabb;
  BVHModel<AABB<Scalar>> s2_aabb;
  BVHModel<OBB<Scalar>> s1_obb;
  BVHModel<OBB<Scalar>> s2_obb;

  generateBVHModel(s1_aabb, s1, Transform3<Scalar>::Identity());
  generateBVHModel(s2_aabb, s2, Transform3<Scalar>::Identity());
  generateBVHModel(s1_obb, s1, Transform3<Scalar>::Identity());
  generateBVHModel(s2_obb, s2, Transform3<Scalar>::Identity());

  CollisionRequest<Scalar> request;
  request.gjk_solver_type = GST_INDEP;

  CollisionResult<Scalar> result;

  bool res;

  Transform3<Scalar> pose = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose_aabb = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose_obb = Transform3<Scalar>::Identity();

  // s2 is within s1
  // both are shapes --> collision
  // s1 is shape, s2 is mesh --> in collision
  // s1 is mesh, s2 is shape --> collision free
  // all are reasonable
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);

  pose.translation() = Vector3<Scalar>(15.01, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(15.01, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(15.01, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);

  pose.translation() = Vector3<Scalar>(14.99, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(14.99, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(14.99, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_collision_boxbox_GJK)
{
  test_consistency_collision_boxbox_GJK<float>();
  test_consistency_collision_boxbox_GJK<double>();
}

template <typename Scalar>
void test_consistency_collision_spherebox_GJK()
{
  Sphere<Scalar> s1(20);
  Box<Scalar> s2(5, 5, 5);

  BVHModel<AABB<Scalar>> s1_aabb;
  BVHModel<AABB<Scalar>> s2_aabb;
  BVHModel<OBB<Scalar>> s1_obb;
  BVHModel<OBB<Scalar>> s2_obb;

  generateBVHModel(s1_aabb, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_aabb, s2, Transform3<Scalar>::Identity());
  generateBVHModel(s1_obb, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_obb, s2, Transform3<Scalar>::Identity());

  CollisionRequest<Scalar> request;
  request.gjk_solver_type = GST_INDEP;

  CollisionResult<Scalar> result;

  bool res;

  Transform3<Scalar> pose = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose_aabb = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose_obb = Transform3<Scalar>::Identity();

  // s2 is within s1
  // both are shapes --> collision
  // s1 is shape, s2 is mesh --> in collision
  // s1 is mesh, s2 is shape --> collision free
  // all are reasonable
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);

  pose.translation() = Vector3<Scalar>(22.4, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(22.4, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(22.4, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3<Scalar>(22.51, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(22.51, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(22.51, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_collision_spherebox_GJK)
{
//  test_consistency_collision_spherebox_GJK<float>();
  test_consistency_collision_spherebox_GJK<double>();
}

template <typename Scalar>
void test_consistency_collision_cylindercylinder_GJK()
{
  Cylinder<Scalar> s1(5, 10);
  Cylinder<Scalar> s2(5, 10);

  BVHModel<AABB<Scalar>> s1_aabb;
  BVHModel<AABB<Scalar>> s2_aabb;
  BVHModel<OBB<Scalar>> s1_obb;
  BVHModel<OBB<Scalar>> s2_obb;

  generateBVHModel(s1_aabb, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_aabb, s2, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s1_obb, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_obb, s2, Transform3<Scalar>::Identity(), 16, 16);

  CollisionRequest<Scalar> request;
  request.gjk_solver_type = GST_INDEP;

  CollisionResult<Scalar> result;

  bool res;

  Transform3<Scalar> pose = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose_aabb = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose_obb = Transform3<Scalar>::Identity();

  pose.translation() = Vector3<Scalar>(9.99, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(9.99, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(9.99, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3<Scalar>(10.01, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(10.01, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(10.01, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_collision_cylindercylinder_GJK)
{
  test_consistency_collision_cylindercylinder_GJK<float>();
  test_consistency_collision_cylindercylinder_GJK<double>();
}

template <typename Scalar>
void test_consistency_collision_conecone_GJK()
{
  Cone<Scalar> s1(5, 10);
  Cone<Scalar> s2(5, 10);

  BVHModel<AABB<Scalar>> s1_aabb;
  BVHModel<AABB<Scalar>> s2_aabb;
  BVHModel<OBB<Scalar>> s1_obb;
  BVHModel<OBB<Scalar>> s2_obb;

  generateBVHModel(s1_aabb, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_aabb, s2, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s1_obb, s1, Transform3<Scalar>::Identity(), 16, 16);
  generateBVHModel(s2_obb, s2, Transform3<Scalar>::Identity(), 16, 16);

  CollisionRequest<Scalar> request;
  request.gjk_solver_type = GST_INDEP;

  CollisionResult<Scalar> result;

  bool res;

  Transform3<Scalar> pose = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose_aabb = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose_obb = Transform3<Scalar>::Identity();

  pose.translation() = Vector3<Scalar>(9.9, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(9.9, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(9.9, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3<Scalar>(10.1, 0, 0);
  pose_aabb.translation() = Vector3<Scalar>(10.1, 0, 0);
  pose_obb.translation() = Vector3<Scalar>(10.1, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);

  pose.translation() = Vector3<Scalar>(0, 0, 9.9);
  pose_aabb.translation() = Vector3<Scalar>(0, 0, 9.9);
  pose_obb.translation() = Vector3<Scalar>(0, 0, 9.9);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3<Scalar>(0, 0, 10.1);
  pose_aabb.translation() = Vector3<Scalar>(0, 0, 10.1);
  pose_obb.translation() = Vector3<Scalar>(0, 0, 10.1);

  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3<Scalar>::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3<Scalar>::Identity(), request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3<Scalar>::Identity(), &s2, pose, request, result) > 0);
  EXPECT_FALSE(res);
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_collision_conecone_GJK)
{
  test_consistency_collision_conecone_GJK<float>();
  test_consistency_collision_conecone_GJK<double>();
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

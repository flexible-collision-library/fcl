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

#define EXPECT_TRUE_FALSE(p) EXPECT_TRUE(!(p))

FCL_REAL extents[6] = {0, 0, 0, 10, 10, 10};

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_distance_spheresphere_libccd)
{
  Sphered s1(20);
  Sphered s2(20);
  BVHModel<RSSd> s1_rss;
  BVHModel<RSSd> s2_rss;

  generateBVHModel(s1_rss, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_rss, s2, Transform3d::Identity(), 16, 16);

  DistanceRequestd request;
  DistanceResultd res, res1;

  Transform3d pose = Transform3d::Identity();

  pose.translation() = Vector3d(50, 0, 0);

  res.clear(); res1.clear();
  distance(&s1, Transform3d::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

  res1.clear();
  distance(&s1, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  
  res1.clear();
  distance(&s1_rss, Transform3d::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3d t;
    generateRandomTransform(extents, t);
    
    Transform3d pose1(t);
    Transform3d pose2 = t * pose;

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

  pose.translation() = Vector3d(40.1, 0, 0);

  res.clear(), res1.clear();
  distance(&s1, Transform3d::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1_rss, Transform3d::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);


  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3d t;
    generateRandomTransform(extents, t);
    
    Transform3d pose1(t);
    Transform3d pose2 = t * pose;

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
  Ellipsoidd s1(20, 40, 50);
  Ellipsoidd s2(10, 10, 10);
  BVHModel<RSSd> s1_rss;
  BVHModel<RSSd> s2_rss;

  generateBVHModel(s1_rss, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_rss, s2, Transform3d::Identity(), 16, 16);

  DistanceRequestd request;
  DistanceResultd res, res1;

  Transform3d pose = Transform3d::Identity();

  pose.translation() = Vector3d(40, 0, 0);

  res.clear(); res1.clear();
  distance(&s1, Transform3d::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

  res1.clear();
  distance(&s1, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

  res1.clear();
  distance(&s1_rss, Transform3d::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3d t;
    generateRandomTransform(extents, t);

    Transform3d pose1(t);
    Transform3d pose2 = t * pose;

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

  pose.translation() = Vector3d(30.1, 0, 0);

  res.clear(), res1.clear();
  distance(&s1, Transform3d::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1_rss, Transform3d::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3d t;
    generateRandomTransform(extents, t);

    Transform3d pose1(t);
    Transform3d pose2 = t * pose;

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
  Boxd s1(20, 40, 50);
  Boxd s2(10, 10, 10);

  BVHModel<RSSd> s1_rss;
  BVHModel<RSSd> s2_rss;

  generateBVHModel(s1_rss, s1, Transform3d::Identity());
  generateBVHModel(s2_rss, s2, Transform3d::Identity());

  DistanceRequestd request;
  DistanceResultd res, res1;

  Transform3d pose = Transform3d::Identity();

  pose.translation() = Vector3d(50, 0, 0);

  res.clear(); res1.clear();
  distance(&s1, Transform3d::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);
  
  res1.clear();
  distance(&s1, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);

  res1.clear();
  distance(&s1_rss, Transform3d::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);

  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3d t;
    generateRandomTransform(extents, t);
    
    Transform3d pose1(t);
    Transform3d pose2 = t * pose;

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

  pose.translation() = Vector3d(15.1, 0, 0);
  
  res.clear(); res1.clear();
  distance(&s1, Transform3d::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  
  res1.clear();
  distance(&s1, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1_rss, Transform3d::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3d t;
    generateRandomTransform(extents, t);
    
    Transform3d pose1(t);
    Transform3d pose2 = t * pose;

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
  Cylinderd s1(5, 10);
  Cylinderd s2(5, 10);

  BVHModel<RSSd> s1_rss;
  BVHModel<RSSd> s2_rss;

  generateBVHModel(s1_rss, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_rss, s2, Transform3d::Identity(), 16, 16);

  DistanceRequestd request;
  DistanceResultd res, res1;

  Transform3d pose = Transform3d::Identity();

  pose.translation() = Vector3d(20, 0, 0);

  res.clear(); res1.clear();
  distance(&s1, Transform3d::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);
  
  res1.clear();
  distance(&s1, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);
  
  res1.clear();
  distance(&s1_rss, Transform3d::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);
  
  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3d t;
    generateRandomTransform(extents, t);
    
    Transform3d pose1(t);
    Transform3d pose2 = t * pose;

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
  
  pose.translation() = Vector3d(15, 0, 0); // libccd cannot use small value here :(
  
  res.clear(); res1.clear();
  distance(&s1, Transform3d::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  
  res1.clear();
  distance(&s1, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1_rss, Transform3d::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  
  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3d t;
    generateRandomTransform(extents, t);
    
    Transform3d pose1(t);
    Transform3d pose2 = t * pose;

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
  Coned s1(5, 10);
  Coned s2(5, 10);

  BVHModel<RSSd> s1_rss;
  BVHModel<RSSd> s2_rss;

  generateBVHModel(s1_rss, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_rss, s2, Transform3d::Identity(), 16, 16);

  DistanceRequestd request;
  DistanceResultd res, res1;

  Transform3d pose = Transform3d::Identity();

  pose.translation() = Vector3d(20, 0, 0);
  
  res.clear(); res1.clear();
  distance(&s1, Transform3d::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  
  res1.clear();
  distance(&s1, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  
  distance(&s1_rss, Transform3d::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  
  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3d t;
    generateRandomTransform(extents, t);
    
    Transform3d pose1(t);
    Transform3d pose2 = t * pose;

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
  
  pose.translation() = Vector3d(15, 0, 0); // libccd cannot use small value here :(
  
  res.clear(); res1.clear();
  distance(&s1, Transform3d::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  
  res1.clear();
  distance(&s1, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1_rss, Transform3d::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  
  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3d t;
    generateRandomTransform(extents, t);
    
    Transform3d pose1(t);
    Transform3d pose2 = t * pose;

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


GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_distance_spheresphere_GJK)
{
  Sphered s1(20);
  Sphered s2(20);
  BVHModel<RSSd> s1_rss;
  BVHModel<RSSd> s2_rss;

  generateBVHModel(s1_rss, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_rss, s2, Transform3d::Identity(), 16, 16);

  DistanceRequestd request;
  request.gjk_solver_type = GST_INDEP;
  DistanceResultd res, res1;

  Transform3d pose = Transform3d::Identity();

  pose.translation() = Vector3d(50, 0, 0);

  res.clear(); res1.clear();
  distance(&s1, Transform3d::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

  res1.clear();
  distance(&s1, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  
  res1.clear();
  distance(&s1_rss, Transform3d::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);


  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3d t;
    generateRandomTransform(extents, t);
    
    Transform3d pose1(t);
    Transform3d pose2 = t * pose;

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

  pose.translation() = Vector3d(40.1, 0, 0);

  res.clear(); res1.clear();
  distance(&s1, Transform3d::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1_rss, Transform3d::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 4);


  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3d t;
    generateRandomTransform(extents, t);
    
    Transform3d pose1(t);
    Transform3d pose2 = t * pose;

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
  Ellipsoidd s1(20, 40, 50);
  Ellipsoidd s2(10, 10, 10);
  BVHModel<RSSd> s1_rss;
  BVHModel<RSSd> s2_rss;

  generateBVHModel(s1_rss, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_rss, s2, Transform3d::Identity(), 16, 16);

  DistanceRequestd request;
  request.gjk_solver_type = GST_INDEP;
  DistanceResultd res, res1;

  Transform3d pose = Transform3d::Identity();

  pose.translation() = Vector3d(40, 0, 0);

  res.clear(); res1.clear();
  distance(&s1, Transform3d::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

  res1.clear();
  distance(&s1, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);

  res1.clear();
  distance(&s1_rss, Transform3d::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);


  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3d t;
    generateRandomTransform(extents, t);

    Transform3d pose1(t);
    Transform3d pose2 = t * pose;

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

  pose.translation() = Vector3d(30.1, 0, 0);

  res.clear(); res1.clear();
  distance(&s1, Transform3d::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1_rss, Transform3d::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 4);


  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3d t;
    generateRandomTransform(extents, t);

    Transform3d pose1(t);
    Transform3d pose2 = t * pose;

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

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_distance_boxbox_GJK)
{
  Boxd s1(20, 40, 50);
  Boxd s2(10, 10, 10);

  BVHModel<RSSd> s1_rss;
  BVHModel<RSSd> s2_rss;

  generateBVHModel(s1_rss, s1, Transform3d::Identity());
  generateBVHModel(s2_rss, s2, Transform3d::Identity());

  DistanceRequestd request;
  request.gjk_solver_type = GST_INDEP;
  DistanceResultd res, res1;

  Transform3d pose = Transform3d::Identity();

  pose.translation() = Vector3d(50, 0, 0);

  res.clear(); res1.clear();
  distance(&s1, Transform3d::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);
  
  res1.clear();
  distance(&s1, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);

  res1.clear();
  distance(&s1_rss, Transform3d::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.01);

  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3d t;
    generateRandomTransform(extents, t);
    
    Transform3d pose1(t);
    Transform3d pose2 = t * pose;

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

  pose.translation() = Vector3d(15.1, 0, 0);
  
  res.clear(); res1.clear();
  distance(&s1, Transform3d::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  
  res1.clear();
  distance(&s1, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1_rss, Transform3d::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3d t;
    generateRandomTransform(extents, t);
    
    Transform3d pose1(t);
    Transform3d pose2 = t * pose;

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
  Cylinderd s1(5, 10);
  Cylinderd s2(5, 10);

  BVHModel<RSSd> s1_rss;
  BVHModel<RSSd> s2_rss;

  generateBVHModel(s1_rss, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_rss, s2, Transform3d::Identity(), 16, 16);
  
  DistanceRequestd request;
  request.gjk_solver_type = GST_INDEP;
  DistanceResultd res, res1;

  Transform3d pose = Transform3d::Identity();

  pose.translation() = Vector3d(20, 0, 0);
  
  res.clear(); res1.clear();
  distance(&s1, Transform3d::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  
  res1.clear();
  distance(&s1, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  
  res1.clear();
  distance(&s1_rss, Transform3d::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  
  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3d t;
    generateRandomTransform(extents, t);
    
    Transform3d pose1(t);
    Transform3d pose2 = t * pose;

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
  
  pose.translation() = Vector3d(10.1, 0, 0);
  
  res.clear(); res1.clear();
  distance(&s1, Transform3d::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  
  res1.clear();
  distance(&s1, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1_rss, Transform3d::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  
  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3d t;
    generateRandomTransform(extents, t);
    
    Transform3d pose1(t);
    Transform3d pose2 = t * pose;

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
  Coned s1(5, 10);
  Coned s2(5, 10);

  BVHModel<RSSd> s1_rss;
  BVHModel<RSSd> s2_rss;

  generateBVHModel(s1_rss, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_rss, s2, Transform3d::Identity(), 16, 16);

  DistanceRequestd request;
  request.gjk_solver_type = GST_INDEP;
  DistanceResultd res, res1;

  Transform3d pose = Transform3d::Identity();

  pose.translation() = Vector3d(20, 0, 0);

  res.clear(); res1.clear();
  distance(&s1, Transform3d::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  
  res1.clear();
  distance(&s1, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  
  res1.clear();
  distance(&s1_rss, Transform3d::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 0.05);
  
  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3d t;
    generateRandomTransform(extents, t);
    
    Transform3d pose1(t);
    Transform3d pose2 = t * pose;

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
  
  pose.translation() = Vector3d(10.1, 0, 0);
  
  res.clear(); res1.clear();
  distance(&s1, Transform3d::Identity(), &s2, pose, request, res);
  distance(&s1_rss, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  
  res1.clear();
  distance(&s1, Transform3d::Identity(), &s2_rss, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);

  res1.clear();
  distance(&s1_rss, Transform3d::Identity(), &s2, pose, request, res1);
  EXPECT_TRUE(fabs(res1.min_distance - res.min_distance) / res.min_distance < 2);
  
  for(std::size_t i = 0; i < 10; ++i)
  {
    Transform3d t;
    generateRandomTransform(extents, t);
    
    Transform3d pose1(t);
    Transform3d pose2 = t * pose;

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



GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_collision_spheresphere_libccd)
{
  Sphered s1(20);
  Sphered s2(10);
  BVHModel<AABBd> s1_aabb;
  BVHModel<AABBd> s2_aabb;
  BVHModel<OBBd> s1_obb;
  BVHModel<OBBd> s2_obb;

  generateBVHModel(s1_aabb, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_aabb, s2, Transform3d::Identity(), 16, 16);
  generateBVHModel(s1_obb, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_obb, s2, Transform3d::Identity(), 16, 16);

  CollisionRequestd request;
  CollisionResultd result;

  bool res;

  Transform3d pose = Transform3d::Identity();
  Transform3d pose_aabb = Transform3d::Identity();
  Transform3d pose_obb = Transform3d::Identity();


  // s2 is within s1
  // both are shapes --> collision
  // s1 is shape, s2 is mesh --> in collision
  // s1 is mesh, s2 is shape --> collision free
  // all are reasonable
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);

  pose.translation() = Vector3d(40, 0, 0);
  pose_aabb.translation() = Vector3d(40, 0, 0);
  pose_obb.translation() = Vector3d(40, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);

  pose.translation() = Vector3d(30, 0, 0);
  pose_aabb.translation() = Vector3d(30, 0, 0);
  pose_obb.translation() = Vector3d(30, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);

  pose.translation() = Vector3d(29.9, 0, 0);
  pose_aabb.translation() = Vector3d(29.8, 0, 0); // 29.9 fails, result depends on mesh precision
  pose_obb.translation() = Vector3d(29.8, 0, 0); // 29.9 fails, result depends on mesh precision

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);


  pose.translation() = Vector3d(-29.9, 0, 0);
  pose_aabb.translation() = Vector3d(-29.8, 0, 0); // 29.9 fails, result depends on mesh precision
  pose_obb.translation() = Vector3d(-29.8, 0, 0); // 29.9 fails, result depends on mesh precision

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3d(-30, 0, 0);
  pose_aabb.translation() = Vector3d(-30, 0, 0);
  pose_obb.translation() = Vector3d(-30, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_collision_ellipsoidellipsoid_libccd)
{
  Ellipsoidd s1(20, 40, 50);
  Ellipsoidd s2(10, 10, 10);
  BVHModel<AABBd> s1_aabb;
  BVHModel<AABBd> s2_aabb;
  BVHModel<OBBd> s1_obb;
  BVHModel<OBBd> s2_obb;

  generateBVHModel(s1_aabb, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_aabb, s2, Transform3d::Identity(), 16, 16);
  generateBVHModel(s1_obb, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_obb, s2, Transform3d::Identity(), 16, 16);

  CollisionRequestd request;
  CollisionResultd result;

  bool res;

  Transform3d pose = Transform3d::Identity();
  Transform3d pose_aabb = Transform3d::Identity();
  Transform3d pose_obb = Transform3d::Identity();


  // s2 is within s1
  // both are shapes --> collision
  // s1 is shape, s2 is mesh --> in collision
  // s1 is mesh, s2 is shape --> collision free
  // all are reasonable
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);

  pose.translation() = Vector3d(40, 0, 0);
  pose_aabb.translation() = Vector3d(40, 0, 0);
  pose_obb.translation() = Vector3d(40, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);

  pose.translation() = Vector3d(30, 0, 0);
  pose_aabb.translation() = Vector3d(30, 0, 0);
  pose_obb.translation() = Vector3d(30, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);  // libccd cannot detect collision when two ellipsoid is exactly touching each other
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, Transform3d(Eigen::Translation3d(Vector3d(29.999, 0, 0))), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);

  pose.translation() = Vector3d(29.9, 0, 0);
  pose_aabb.translation() = Vector3d(29.9, 0, 0); // 29.9 fails, result depends on mesh precision
  pose_obb.translation() = Vector3d(29.9, 0, 0); // 29.9 fails, result depends on mesh precision

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3d(-29.9, 0, 0);
  pose_aabb.translation() = Vector3d(-29.9, 0, 0);
  pose_obb.translation() = Vector3d(-29.9, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3d(-30, 0, 0);
  pose_aabb.translation() = Vector3d(-30, 0, 0);
  pose_obb.translation() = Vector3d(-30, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_collision_boxbox_libccd)
{
  Boxd s1(20, 40, 50);
  Boxd s2(10, 10, 10);

  BVHModel<AABBd> s1_aabb;
  BVHModel<AABBd> s2_aabb;
  BVHModel<OBBd> s1_obb;
  BVHModel<OBBd> s2_obb;

  generateBVHModel(s1_aabb, s1, Transform3d::Identity());
  generateBVHModel(s2_aabb, s2, Transform3d::Identity());
  generateBVHModel(s1_obb, s1, Transform3d::Identity());
  generateBVHModel(s2_obb, s2, Transform3d::Identity());

  CollisionRequestd request;
  CollisionResultd result;

  bool res;

  Transform3d pose = Transform3d::Identity();
  Transform3d pose_aabb = Transform3d::Identity();
  Transform3d pose_obb = Transform3d::Identity();

  // s2 is within s1
  // both are shapes --> collision
  // s1 is shape, s2 is mesh --> in collision
  // s1 is mesh, s2 is shape --> collision free
  // all are reasonable
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);

  pose.translation() = Vector3d(15.01, 0, 0);
  pose_aabb.translation() = Vector3d(15.01, 0, 0);
  pose_obb.translation() = Vector3d(15.01, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);

  pose.translation() = Vector3d(14.99, 0, 0);
  pose_aabb.translation() = Vector3d(14.99, 0, 0);
  pose_obb.translation() = Vector3d(14.99, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_collision_spherebox_libccd)
{
  Sphered s1(20);
  Boxd s2(5, 5, 5);

  BVHModel<AABBd> s1_aabb;
  BVHModel<AABBd> s2_aabb;
  BVHModel<OBBd> s1_obb;
  BVHModel<OBBd> s2_obb;

  generateBVHModel(s1_aabb, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_aabb, s2, Transform3d::Identity());
  generateBVHModel(s1_obb, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_obb, s2, Transform3d::Identity());

  CollisionRequestd request;
  CollisionResultd result;

  bool res;

  Transform3d pose = Transform3d::Identity();
  Transform3d pose_aabb = Transform3d::Identity();
  Transform3d pose_obb = Transform3d::Identity();

  // s2 is within s1
  // both are shapes --> collision
  // s1 is shape, s2 is mesh --> in collision
  // s1 is mesh, s2 is shape --> collision free
  // all are reasonable
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);

  pose.translation() = Vector3d(22.4, 0, 0);
  pose_aabb.translation() = Vector3d(22.4, 0, 0);
  pose_obb.translation() = Vector3d(22.4, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3d(22.51, 0, 0);
  pose_aabb.translation() = Vector3d(22.51, 0, 0);
  pose_obb.translation() = Vector3d(22.51, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_collision_cylindercylinder_libccd)
{
  Cylinderd s1(5, 10);
  Cylinderd s2(5, 10);

  BVHModel<AABBd> s1_aabb;
  BVHModel<AABBd> s2_aabb;
  BVHModel<OBBd> s1_obb;
  BVHModel<OBBd> s2_obb;

  generateBVHModel(s1_aabb, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_aabb, s2, Transform3d::Identity(), 16, 16);
  generateBVHModel(s1_obb, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_obb, s2, Transform3d::Identity(), 16, 16);

  CollisionRequestd request;
  CollisionResultd result;

  bool res;

  Transform3d pose = Transform3d::Identity();
  Transform3d pose_aabb = Transform3d::Identity();
  Transform3d pose_obb = Transform3d::Identity();

  pose.translation() = Vector3d(9.99, 0, 0);
  pose_aabb.translation() = Vector3d(9.99, 0, 0);
  pose_obb.translation() = Vector3d(9.99, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3d(10.01, 0, 0);
  pose_aabb.translation() = Vector3d(10.01, 0, 0);
  pose_obb.translation() = Vector3d(10.01, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_collision_conecone_libccd)
{
  Coned s1(5, 10);
  Coned s2(5, 10);

  BVHModel<AABBd> s1_aabb;
  BVHModel<AABBd> s2_aabb;
  BVHModel<OBBd> s1_obb;
  BVHModel<OBBd> s2_obb;

  generateBVHModel(s1_aabb, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_aabb, s2, Transform3d::Identity(), 16, 16);
  generateBVHModel(s1_obb, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_obb, s2, Transform3d::Identity(), 16, 16);

  CollisionRequestd request;
  CollisionResultd result;

  bool res;

  Transform3d pose = Transform3d::Identity();
  Transform3d pose_aabb = Transform3d::Identity();
  Transform3d pose_obb = Transform3d::Identity();

  pose.translation() = Vector3d(9.9, 0, 0);
  pose_aabb.translation() = Vector3d(9.9, 0, 0);
  pose_obb.translation() = Vector3d(9.9, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3d(10.1, 0, 0);
  pose_aabb.translation() = Vector3d(10.1, 0, 0);
  pose_obb.translation() = Vector3d(10.1, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);

  pose.translation() = Vector3d(0, 0, 9.9);
  pose_aabb.translation() = Vector3d(0, 0, 9.9);
  pose_obb.translation() = Vector3d(0, 0, 9.9);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3d(0, 0, 10.1);
  pose_aabb.translation() = Vector3d(0, 0, 10.1);
  pose_obb.translation() = Vector3d(0, 0, 10.1);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
}




GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_collision_spheresphere_GJK)
{
  Sphered s1(20);
  Sphered s2(10);
  BVHModel<AABBd> s1_aabb;
  BVHModel<AABBd> s2_aabb;
  BVHModel<OBBd> s1_obb;
  BVHModel<OBBd> s2_obb;

  generateBVHModel(s1_aabb, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_aabb, s2, Transform3d::Identity(), 16, 16);
  generateBVHModel(s1_obb, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_obb, s2, Transform3d::Identity(), 16, 16);

  CollisionRequestd request;
  request.gjk_solver_type = GST_INDEP;

  CollisionResultd result;

  bool res;

  Transform3d pose = Transform3d::Identity();
  Transform3d pose_aabb = Transform3d::Identity();
  Transform3d pose_obb = Transform3d::Identity();


  // s2 is within s1
  // both are shapes --> collision
  // s1 is shape, s2 is mesh --> in collision
  // s1 is mesh, s2 is shape --> collision free
  // all are reasonable
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);

  pose.translation() = Vector3d(40, 0, 0);
  pose_aabb.translation() = Vector3d(40, 0, 0);
  pose_obb.translation() = Vector3d(40, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);

  pose.translation() = Vector3d(30, 0, 0);
  pose_aabb.translation() = Vector3d(30, 0, 0);
  pose_obb.translation() = Vector3d(30, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);

  pose.translation() = Vector3d(29.9, 0, 0);
  pose_aabb.translation() = Vector3d(29.8, 0, 0); // 29.9 fails, result depends on mesh precision
  pose_obb.translation() = Vector3d(29.8, 0, 0); // 29.9 fails, result depends on mesh precision

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);


  pose.translation() = Vector3d(-29.9, 0, 0);
  pose_aabb.translation() = Vector3d(-29.8, 0, 0); // 29.9 fails, result depends on mesh precision
  pose_obb.translation() = Vector3d(-29.8, 0, 0); // 29.9 fails, result depends on mesh precision

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3d(-30, 0, 0);
  pose_aabb.translation() = Vector3d(-30, 0, 0);
  pose_obb.translation() = Vector3d(-30, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_collision_ellipsoidellipsoid_GJK)
{
  Ellipsoidd s1(20, 40, 50);
  Ellipsoidd s2(10, 10, 10);
  BVHModel<AABBd> s1_aabb;
  BVHModel<AABBd> s2_aabb;
  BVHModel<OBBd> s1_obb;
  BVHModel<OBBd> s2_obb;

  generateBVHModel(s1_aabb, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_aabb, s2, Transform3d::Identity(), 16, 16);
  generateBVHModel(s1_obb, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_obb, s2, Transform3d::Identity(), 16, 16);

  CollisionRequestd request;
  request.gjk_solver_type = GST_INDEP;

  CollisionResultd result;

  bool res;

  Transform3d pose = Transform3d::Identity();
  Transform3d pose_aabb = Transform3d::Identity();
  Transform3d pose_obb = Transform3d::Identity();


  // s2 is within s1
  // both are shapes --> collision
  // s1 is shape, s2 is mesh --> in collision
  // s1 is mesh, s2 is shape --> collision free
  // all are reasonable
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);

  pose.translation() = Vector3d(40, 0, 0);
  pose_aabb.translation() = Vector3d(40, 0, 0);
  pose_obb.translation() = Vector3d(40, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);

  pose.translation() = Vector3d(30, 0, 0);
  pose_aabb.translation() = Vector3d(30, 0, 0);
  pose_obb.translation() = Vector3d(30, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);

  pose.translation() = Vector3d(29.9, 0, 0);
  pose_aabb.translation() = Vector3d(29.9, 0, 0);
  pose_obb.translation() = Vector3d(29.9, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);


  pose.translation() = Vector3d(-29.9, 0, 0);
  pose_aabb.translation() = Vector3d(-29.9, 0, 0);
  pose_obb.translation() = Vector3d(-29.9, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3d(-30, 0, 0);
  pose_aabb.translation() = Vector3d(-30, 0, 0);
  pose_obb.translation() = Vector3d(-30, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_collision_boxbox_GJK)
{
  Boxd s1(20, 40, 50);
  Boxd s2(10, 10, 10);

  BVHModel<AABBd> s1_aabb;
  BVHModel<AABBd> s2_aabb;
  BVHModel<OBBd> s1_obb;
  BVHModel<OBBd> s2_obb;

  generateBVHModel(s1_aabb, s1, Transform3d::Identity());
  generateBVHModel(s2_aabb, s2, Transform3d::Identity());
  generateBVHModel(s1_obb, s1, Transform3d::Identity());
  generateBVHModel(s2_obb, s2, Transform3d::Identity());

  CollisionRequestd request;
  request.gjk_solver_type = GST_INDEP;

  CollisionResultd result;

  bool res;

  Transform3d pose = Transform3d::Identity();
  Transform3d pose_aabb = Transform3d::Identity();
  Transform3d pose_obb = Transform3d::Identity();

  // s2 is within s1
  // both are shapes --> collision
  // s1 is shape, s2 is mesh --> in collision
  // s1 is mesh, s2 is shape --> collision free
  // all are reasonable
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);

  pose.translation() = Vector3d(15.01, 0, 0);
  pose_aabb.translation() = Vector3d(15.01, 0, 0);
  pose_obb.translation() = Vector3d(15.01, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);

  pose.translation() = Vector3d(14.99, 0, 0);
  pose_aabb.translation() = Vector3d(14.99, 0, 0);
  pose_obb.translation() = Vector3d(14.99, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_collision_spherebox_GJK)
{
  Sphered s1(20);
  Boxd s2(5, 5, 5);

  BVHModel<AABBd> s1_aabb;
  BVHModel<AABBd> s2_aabb;
  BVHModel<OBBd> s1_obb;
  BVHModel<OBBd> s2_obb;

  generateBVHModel(s1_aabb, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_aabb, s2, Transform3d::Identity());
  generateBVHModel(s1_obb, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_obb, s2, Transform3d::Identity());

  CollisionRequestd request;
  request.gjk_solver_type = GST_INDEP;

  CollisionResultd result;

  bool res;

  Transform3d pose = Transform3d::Identity();
  Transform3d pose_aabb = Transform3d::Identity();
  Transform3d pose_obb = Transform3d::Identity();

  // s2 is within s1
  // both are shapes --> collision
  // s1 is shape, s2 is mesh --> in collision
  // s1 is mesh, s2 is shape --> collision free
  // all are reasonable
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);

  pose.translation() = Vector3d(22.4, 0, 0);
  pose_aabb.translation() = Vector3d(22.4, 0, 0);
  pose_obb.translation() = Vector3d(22.4, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3d(22.51, 0, 0);
  pose_aabb.translation() = Vector3d(22.51, 0, 0);
  pose_obb.translation() = Vector3d(22.51, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_collision_cylindercylinder_GJK)
{
  Cylinderd s1(5, 10);
  Cylinderd s2(5, 10);

  BVHModel<AABBd> s1_aabb;
  BVHModel<AABBd> s2_aabb;
  BVHModel<OBBd> s1_obb;
  BVHModel<OBBd> s2_obb;

  generateBVHModel(s1_aabb, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_aabb, s2, Transform3d::Identity(), 16, 16);
  generateBVHModel(s1_obb, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_obb, s2, Transform3d::Identity(), 16, 16);

  CollisionRequestd request;
  request.gjk_solver_type = GST_INDEP;

  CollisionResultd result;

  bool res;

  Transform3d pose = Transform3d::Identity();
  Transform3d pose_aabb = Transform3d::Identity();
  Transform3d pose_obb = Transform3d::Identity();

  pose.translation() = Vector3d(9.99, 0, 0);
  pose_aabb.translation() = Vector3d(9.99, 0, 0);
  pose_obb.translation() = Vector3d(9.99, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3d(10.01, 0, 0);
  pose_aabb.translation() = Vector3d(10.01, 0, 0);
  pose_obb.translation() = Vector3d(10.01, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
}

GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, consistency_collision_conecone_GJK)
{
  Coned s1(5, 10);
  Coned s2(5, 10);

  BVHModel<AABBd> s1_aabb;
  BVHModel<AABBd> s2_aabb;
  BVHModel<OBBd> s1_obb;
  BVHModel<OBBd> s2_obb;

  generateBVHModel(s1_aabb, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_aabb, s2, Transform3d::Identity(), 16, 16);
  generateBVHModel(s1_obb, s1, Transform3d::Identity(), 16, 16);
  generateBVHModel(s2_obb, s2, Transform3d::Identity(), 16, 16);

  CollisionRequestd request;
  request.gjk_solver_type = GST_INDEP;

  CollisionResultd result;

  bool res;

  Transform3d pose = Transform3d::Identity();
  Transform3d pose_aabb = Transform3d::Identity();
  Transform3d pose_obb = Transform3d::Identity();

  pose.translation() = Vector3d(9.9, 0, 0);
  pose_aabb.translation() = Vector3d(9.9, 0, 0);
  pose_obb.translation() = Vector3d(9.9, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3d(10.1, 0, 0);
  pose_aabb.translation() = Vector3d(10.1, 0, 0);
  pose_obb.translation() = Vector3d(10.1, 0, 0);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);

  pose.translation() = Vector3d(0, 0, 9.9);
  pose_aabb.translation() = Vector3d(0, 0, 9.9);
  pose_obb.translation() = Vector3d(0, 0, 9.9);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE(res);

  pose.translation() = Vector3d(0, 0, 10.1);
  pose_aabb.translation() = Vector3d(0, 0, 10.1);
  pose_obb.translation() = Vector3d(0, 0, 10.1);

  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_aabb, pose_aabb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2_obb, pose_obb, &s1, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_aabb, pose_aabb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3d::Identity(), &s2_obb, pose_obb, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_aabb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s2, pose, &s1_obb, Transform3d::Identity(), request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
  result.clear();
  res = (collide(&s1_aabb, Transform3d::Identity(), &s2, pose, request, result) > 0);
  EXPECT_TRUE_FALSE(res);
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

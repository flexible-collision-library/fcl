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

#include "fcl/traversal/traversal_nodes.h"
#include "fcl/collision_node.h"
#include "test_fcl_utility.h"
#include "fcl_resources/config.h"

using namespace fcl;

bool verbose = false;

template <typename S>
S DELTA() { return 0.001; }

template<typename BV>
void distance_Test(const Transform3<typename BV::S>& tf,
                   const std::vector<Vector3<typename BV::S>>& vertices1, const std::vector<Triangle>& triangles1,
                   const std::vector<Vector3<typename BV::S>>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method,
                   int qsize,
                   DistanceRes<typename BV::S>& distance_result,
                   bool verbose = true);

template <typename S>
bool collide_Test_OBB(const Transform3<S>& tf,
                      const std::vector<Vector3<S>>& vertices1, const std::vector<Triangle>& triangles1,
                      const std::vector<Vector3<S>>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose);

template<typename BV, typename TraversalNode>
void distance_Test_Oriented(const Transform3<typename BV::S>& tf,
                            const std::vector<Vector3<typename BV::S>>& vertices1, const std::vector<Triangle>& triangles1,
                            const std::vector<Vector3<typename BV::S>>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method,
                            int qsize,
                            DistanceRes<typename BV::S>& distance_result,
                            bool verbose = true);

template <typename S>
bool nearlyEqual(const Vector3<S>& a, const Vector3<S>& b)
{
  if(fabs(a[0] - b[0]) > DELTA<S>()) return false;
  if(fabs(a[1] - b[1]) > DELTA<S>()) return false;
  if(fabs(a[2] - b[2]) > DELTA<S>()) return false;
  return true;
}

template <typename S>
void test_mesh_distance()
{
  std::vector<Vector3<S>> p1, p2;
  std::vector<Triangle> t1, t2;

  loadOBJFile(TEST_RESOURCES_DIR"/env.obj", p1, t1);
  loadOBJFile(TEST_RESOURCES_DIR"/rob.obj", p2, t2);

  Eigen::aligned_vector<Transform3<S>> transforms; // t0
  S extents[] = {-3000, -3000, 0, 3000, 3000, 3000};
#ifdef NDEBUG
  std::size_t n = 10;
#else
  std::size_t n = 1;
#endif

  generateRandomTransforms(extents, transforms, n);

  double dis_time = 0;
  double col_time = 0;

  DistanceRes<S> res, res_now;
  for(std::size_t i = 0; i < transforms.size(); ++i)
  {
    Timer timer_col;
    timer_col.start();
    collide_Test_OBB(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    timer_col.stop();
    col_time += timer_col.getElapsedTimeInSec();

    Timer timer_dist;
    timer_dist.start();
    distance_Test_Oriented<RSS<S>, MeshDistanceTraversalNodeRSS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 2, res, verbose);
    timer_dist.stop();
    dis_time += timer_dist.getElapsedTimeInSec();

    distance_Test_Oriented<RSS<S>, MeshDistanceTraversalNodeRSS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<RSS<S>, MeshDistanceTraversalNodeRSS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<RSS<S>, MeshDistanceTraversalNodeRSS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<RSS<S>, MeshDistanceTraversalNodeRSS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<RSS<S>, MeshDistanceTraversalNodeRSS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<kIOS<S>, MeshDistanceTraversalNodekIOS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<kIOS<S>, MeshDistanceTraversalNodekIOS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<kIOS<S>, MeshDistanceTraversalNodekIOS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<kIOS<S>, MeshDistanceTraversalNodekIOS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<kIOS<S>, MeshDistanceTraversalNodekIOS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<kIOS<S>, MeshDistanceTraversalNodekIOS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<OBBRSS<S>, MeshDistanceTraversalNodeOBBRSS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<OBBRSS<S>, MeshDistanceTraversalNodeOBBRSS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<OBBRSS<S>, MeshDistanceTraversalNodeOBBRSS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<OBBRSS<S>, MeshDistanceTraversalNodeOBBRSS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<OBBRSS<S>, MeshDistanceTraversalNodeOBBRSS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<OBBRSS<S>, MeshDistanceTraversalNodeOBBRSS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));



    distance_Test<RSS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<RSS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<RSS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<RSS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<RSS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<RSS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<kIOS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<kIOS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<kIOS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<kIOS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<kIOS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<kIOS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));


    distance_Test<OBBRSS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<OBBRSS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<OBBRSS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<OBBRSS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<OBBRSS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<OBBRSS<S>>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

  }

  std::cout << "distance timing: " << dis_time << " sec" << std::endl;
  std::cout << "collision timing: " << col_time << " sec" << std::endl;
}

GTEST_TEST(FCL_DISTANCE, mesh_distance)
{
//  test_mesh_distance<float>();
  test_mesh_distance<double>();
}

template<typename BV, typename TraversalNode>
void distance_Test_Oriented(const Transform3<typename BV::S>& tf,
                            const std::vector<Vector3<typename BV::S>>& vertices1, const std::vector<Triangle>& triangles1,
                            const std::vector<Vector3<typename BV::S>>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method,
                            int qsize,
                            DistanceRes<typename BV::S>& distance_result,
                            bool verbose)
{
  using S = typename BV::S;

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

  DistanceResult<S> local_result;
  TraversalNode node;
  if(!initialize(node, (const BVHModel<BV>&)m1, tf, (const BVHModel<BV>&)m2, Transform3<S>::Identity(), DistanceRequest<S>(true), local_result))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;

  distance(&node, nullptr, qsize);

  // points are in local coordinate, to global coordinate
  Vector3<S> p1 = local_result.nearest_points[0];
  Vector3<S> p2 = local_result.nearest_points[1];


  distance_result.distance = local_result.min_distance;
  distance_result.p1 = p1;
  distance_result.p2 = p2;

  if(verbose)
  {
    std::cout << "distance " << local_result.min_distance << std::endl;

    std::cout << p1[0] << " " << p1[1] << " " << p1[2] << std::endl;
    std::cout << p2[0] << " " << p2[1] << " " << p2[2] << std::endl;
    std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
  }
}

template<typename BV>
void distance_Test(const Transform3<typename BV::S>& tf,
                   const std::vector<Vector3<typename BV::S>>& vertices1, const std::vector<Triangle>& triangles1,
                   const std::vector<Vector3<typename BV::S>>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method,
                   int qsize,
                   DistanceRes<typename BV::S>& distance_result,
                   bool verbose)
{
  using S = typename BV::S;

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

  Transform3<S> pose1(tf);
  Transform3<S> pose2 = Transform3<S>::Identity();

  DistanceResult<S> local_result;
  MeshDistanceTraversalNode<BV> node;

  if(!initialize<BV>(node, m1, pose1, m2, pose2, DistanceRequest<S>(true), local_result))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;

  distance(&node, nullptr, qsize);

  distance_result.distance = local_result.min_distance;
  distance_result.p1 = local_result.nearest_points[0];
  distance_result.p2 = local_result.nearest_points[1];

  if(verbose)
  {
    std::cout << "distance " << local_result.min_distance << std::endl;

    std::cout << local_result.nearest_points[0][0] << " " << local_result.nearest_points[0][1] << " " << local_result.nearest_points[0][2] << std::endl;
    std::cout << local_result.nearest_points[1][0] << " " << local_result.nearest_points[1][1] << " " << local_result.nearest_points[1][2] << std::endl;
    std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
  }
}

template <typename S>
bool collide_Test_OBB(const Transform3<S>& tf,
                      const std::vector<Vector3<S>>& vertices1, const std::vector<Triangle>& triangles1,
                      const std::vector<Vector3<S>>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose)
{
  BVHModel<OBB<S>> m1;
  BVHModel<OBB<S>> m2;
  m1.bv_splitter.reset(new BVSplitter<OBB<S>>(split_method));
  m2.bv_splitter.reset(new BVSplitter<OBB<S>>(split_method));

  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  CollisionResult<S> local_result;
  MeshCollisionTraversalNodeOBB<S> node;
  if(!initialize(node, (const BVHModel<OBB<S>>&)m1, tf, (const BVHModel<OBB<S>>&)m2, Transform3<S>::Identity(),
                 CollisionRequest<S>(), local_result))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;

  collide(&node);

  if(local_result.numContacts() > 0)
    return true;
  else
    return false;
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

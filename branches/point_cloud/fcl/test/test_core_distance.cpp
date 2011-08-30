/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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


#include "fcl/traversal_node_bvhs.h"
#include "fcl/collision_node.h"
#include "fcl/simple_setup.h"
#include "test_core_utility.h"
#include <gtest/gtest.h>


#if USE_PQP
#include <PQP.h>
#endif

using namespace fcl;


void distance_Test(const Transform& tf,
                  const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                  const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method,
                  int qsize,
                  DistanceRes& distance_result,
                  bool verbose = true);

template<typename BV>
void distance_Test2(const Transform& tf,
                    const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                    const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method,
                    int qsize,
                    DistanceRes& distance_result,
                    bool verbose = true);

bool verbose = false;
BVH_REAL DELTA = 0.001;

inline bool nearlyEqual(const Vec3f& a, const Vec3f& b)
{
  if(fabs(a[0] - b[0]) > DELTA) return false;
  if(fabs(a[1] - b[1]) > DELTA) return false;
  if(fabs(a[2] - b[2]) > DELTA) return false;
  return true;
}

TEST(collision_core, mesh_distance)
{
  std::vector<Vec3f> p1, p2;
  std::vector<Triangle> t1, t2;
  loadOBJFile("test/env.obj", p1, t1);
  loadOBJFile("test/rob.obj", p2, t2);

  std::vector<Transform> transforms; // t0
  std::vector<Transform> transforms2; // t1
  std::vector<Transform> transforms_ccd; // t0
  std::vector<Transform> transforms_ccd2; // t1
  BVH_REAL extents[] = {-3000, -3000, 0, 3000, 3000, 3000};
  BVH_REAL delta_trans[] = {1, 1, 1};
  int n = 10;

  generateRandomTransform(extents, transforms, transforms2, delta_trans, 0.005 * 2 * 3.1415, n);

  DistanceRes res, res_now;
  for(unsigned int i = 0; i < transforms.size(); ++i)
  {
#if USE_PQP
    distance_PQP(transforms[i], p1, t1, p2, t2, res, verbose);

    distance_PQP2(transforms[i], p1, t1, p2, t2, res_now, verbose);

    ASSERT_TRUE(fabs(res.distance - res_now.distance) < DELTA);
    ASSERT_TRUE(fabs(res.distance) < DELTA || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));
#endif

#if USE_PQP
    distance_Test(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 2, res_now, verbose);
    ASSERT_TRUE(fabs(res.distance - res_now.distance) < DELTA);
    ASSERT_TRUE(fabs(res.distance) < DELTA || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));
#else
    distance_Test(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 2, res, verbose);
#endif

    distance_Test(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, 2, res_now, verbose);

    ASSERT_TRUE(fabs(res.distance - res_now.distance) < DELTA);
    ASSERT_TRUE(fabs(res.distance) < DELTA || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 2, res_now, verbose);

    ASSERT_TRUE(fabs(res.distance - res_now.distance) < DELTA);
    ASSERT_TRUE(fabs(res.distance) < DELTA || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 20, res_now, verbose);

    ASSERT_TRUE(fabs(res.distance - res_now.distance) < DELTA);
    ASSERT_TRUE(fabs(res.distance) < DELTA || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, 20, res_now, verbose);

    ASSERT_TRUE(fabs(res.distance - res_now.distance) < DELTA);
    ASSERT_TRUE(fabs(res.distance) < DELTA || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 20, res_now, verbose);

    ASSERT_TRUE(fabs(res.distance - res_now.distance) < DELTA);
    ASSERT_TRUE(fabs(res.distance) < DELTA || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test2<RSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 2, res_now, verbose);

    ASSERT_TRUE(fabs(res.distance - res_now.distance) < DELTA);
    ASSERT_TRUE(fabs(res.distance) < DELTA || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test2<RSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, 2, res_now, verbose);

    ASSERT_TRUE(fabs(res.distance - res_now.distance) < DELTA);
    ASSERT_TRUE(fabs(res.distance) < DELTA || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test2<RSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 2, res_now, verbose);

    ASSERT_TRUE(fabs(res.distance - res_now.distance) < DELTA);
    ASSERT_TRUE(fabs(res.distance) < DELTA || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test2<RSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 20, res_now, verbose);

    ASSERT_TRUE(fabs(res.distance - res_now.distance) < DELTA);
    ASSERT_TRUE(fabs(res.distance) < DELTA || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test2<RSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, 20, res_now, verbose);

    ASSERT_TRUE(fabs(res.distance - res_now.distance) < DELTA);
    ASSERT_TRUE(fabs(res.distance) < DELTA || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test2<RSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 20, res_now, verbose);

    ASSERT_TRUE(fabs(res.distance - res_now.distance) < DELTA);
    ASSERT_TRUE(fabs(res.distance) < DELTA || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));
  }
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

void distance_Test(const Transform& tf,
                   const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                   const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method,
                   int qsize,
                   DistanceRes& distance_result,
                   bool verbose)
{
  BVHModel<RSS> m1;
  BVHModel<RSS> m2;
  m1.bv_splitter.reset(new BVSplitter<RSS>(split_method));
  m2.bv_splitter.reset(new BVSplitter<RSS>(split_method));


  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  Vec3f R2[3];
  R2[0] = Vec3f(1, 0, 0);
  R2[1] = Vec3f(0, 1, 0);
  R2[2] = Vec3f(0, 0, 1);
  Vec3f T2;


  MeshDistanceTraversalNodeRSS node;

  if(!initialize(node, (const BVHModel<RSS>&)m1, (const BVHModel<RSS>&)m2, tf.R, tf.T, R2, T2))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;

  distance(&node, NULL, qsize);

  // points are in local coordinate, to global coordinate
  Vec3f p1 = MxV(tf.R, node.p1) + tf.T;
  Vec3f p2 = MxV(R2, node.p2) + T2;


  distance_result.distance = node.min_distance;
  distance_result.p1 = p1;
  distance_result.p2 = p2;

  if(verbose)
  {
    std::cout << "distance " << node.min_distance << std::endl;

    std::cout << p1[0] << " " << p1[1] << " " << p1[2] << std::endl;
    std::cout << p2[0] << " " << p2[1] << " " << p2[2] << std::endl;
    std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
  }
}

template<typename BV>
void distance_Test2(const Transform& tf,
                    const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                    const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method,
                    int qsize,
                    DistanceRes& distance_result,
                    bool verbose)
{
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

  Vec3f R2[3];
  R2[0] = Vec3f(1, 0, 0);
  R2[1] = Vec3f(0, 1, 0);
  R2[2] = Vec3f(0, 0, 1);
  Vec3f T2;

  MeshDistanceTraversalNode<BV> node;

  if(!initialize<BV>(node, m1, m2, tf.R, tf.T, R2, T2))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;

  distance(&node, NULL, qsize);

  distance_result.distance = node.min_distance;
  distance_result.p1 = node.p1;
  distance_result.p2 = node.p2;

  if(verbose)
  {
    std::cout << "distance " << node.min_distance << std::endl;

    std::cout << node.p1[0] << " " << node.p1[1] << " " << node.p1[2] << std::endl;
    std::cout << node.p2[0] << " " << node.p2[1] << " " << node.p2[2] << std::endl;
    std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
  }
}

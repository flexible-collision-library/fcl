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

using namespace fcl;

template<typename BV>
bool collide_front_list_Test(const Transform& tf1, const Transform& tf2,
                             const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                             const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                             SplitMethodType split_method,
                             bool refit_bottomup, bool verbose);

bool collide_front_list_OBB_Test(const Transform& tf1, const Transform& tf2,
                                 const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                                 const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                                 SplitMethodType split_method, bool verbose);

bool collide_front_list_RSS_Test(const Transform& tf1, const Transform& tf2,
                                 const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                                 const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                                 SplitMethodType split_method, bool verbose);

template<typename BV>
bool collide_Test(const Transform& tf,
                  const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                  const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose);

// TODO: randomly still have some runtime error
TEST(collision_test, front_list)
{
  //srand(time(NULL));
  std::vector<Vec3f> p1, p2;
  std::vector<Triangle> t1, t2;
  loadOBJFile("test/env.obj", p1, t1);
  loadOBJFile("test/rob.obj", p2, t2);

  std::vector<Transform> transforms; // t0
  std::vector<Transform> transforms2; // t1
  BVH_REAL extents[] = {-3000, -3000, 0, 3000, 3000, 3000};
  BVH_REAL delta_trans[] = {1, 1, 1};
  int n = 10;
  bool verbose = false;

  generateRandomTransform(extents, transforms, transforms2, delta_trans, 0.005 * 2 * 3.1415, n);

  bool res, res2;

  for(unsigned int i = 0; i < transforms.size(); ++i)
  {
    res = collide_Test<AABB>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    res2 = collide_front_list_Test<AABB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, false, verbose);
    ASSERT_TRUE(res == res2);
    res = collide_Test<AABB>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    res2 = collide_front_list_Test<AABB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, false, verbose);
    ASSERT_TRUE(res == res2);
    res = collide_Test<AABB>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    res2 = collide_front_list_Test<AABB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, false, verbose);
    ASSERT_TRUE(res == res2);
  }

  for(unsigned int i = 0; i < transforms.size(); ++i)
  {
    res = collide_Test<OBB>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    res2 = collide_front_list_Test<OBB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, false, verbose);
    ASSERT_TRUE(res == res2);
    res = collide_Test<OBB>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    res2 = collide_front_list_Test<OBB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, false, verbose);
    ASSERT_TRUE(res == res2);
    res = collide_Test<OBB>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    res2 = collide_front_list_Test<OBB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, false, verbose);
    ASSERT_TRUE(res == res2);
  }

  for(unsigned int i = 0; i < transforms.size(); ++i)
  {
    res = collide_Test<RSS>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    res2 = collide_front_list_Test<RSS>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, false, verbose);
    ASSERT_TRUE(res == res2);
    res = collide_Test<RSS>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    res2 = collide_front_list_Test<RSS>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, false, verbose);
    ASSERT_TRUE(res == res2);
    res = collide_Test<RSS>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    res2 = collide_front_list_Test<RSS>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, false, verbose);
    ASSERT_TRUE(res == res2);
  }

  for(unsigned int i = 0; i < transforms.size(); ++i)
  {
    res = collide_Test<KDOP<16> >(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    res2 = collide_front_list_Test<KDOP<16> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, false, verbose);
    ASSERT_TRUE(res == res2);
    res = collide_Test<KDOP<16> >(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    res2 = collide_front_list_Test<KDOP<16> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, false, verbose);
    ASSERT_TRUE(res == res2);
    res = collide_Test<KDOP<16> >(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    res2 = collide_front_list_Test<KDOP<16> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, false, verbose);
    ASSERT_TRUE(res == res2);
  }

  for(unsigned int i = 0; i < transforms.size(); ++i)
  {
    res = collide_Test<KDOP<18> >(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    res2 = collide_front_list_Test<KDOP<18> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, false, verbose);
    ASSERT_TRUE(res == res2);
    res = collide_Test<KDOP<18> >(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    res2 = collide_front_list_Test<KDOP<18> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, false, verbose);
    ASSERT_TRUE(res == res2);
    res = collide_Test<KDOP<18> >(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    res2 = collide_front_list_Test<KDOP<18> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, false, verbose);
    ASSERT_TRUE(res == res2);
  }

  for(unsigned int i = 0; i < transforms.size(); ++i)
  {
    res = collide_Test<KDOP<24> >(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    res2 = collide_front_list_Test<KDOP<24> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, false, verbose);
    ASSERT_TRUE(res == res2);
    res = collide_Test<KDOP<24> >(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    res2 = collide_front_list_Test<KDOP<24> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, false, verbose);
    ASSERT_TRUE(res == res2);
    res = collide_Test<KDOP<24> >(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    res2 = collide_front_list_Test<KDOP<24> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, false, verbose);
    ASSERT_TRUE(res == res2);
  }

  for(unsigned int i = 0; i < transforms.size(); ++i)
  {
    res = collide_Test<OBB>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    res2 = collide_front_list_OBB_Test(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    ASSERT_TRUE(res == res2);
    res = collide_Test<OBB>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    res2 = collide_front_list_OBB_Test(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    ASSERT_TRUE(res == res2);
    res = collide_Test<OBB>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    res2 = collide_front_list_OBB_Test(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    ASSERT_TRUE(res == res2);
  }

  for(unsigned int i = 0; i < transforms.size(); ++i)
  {
    res = collide_Test<RSS>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    res2 = collide_front_list_RSS_Test(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    ASSERT_TRUE(res == res2);
    res = collide_Test<RSS>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    res2 = collide_front_list_RSS_Test(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    ASSERT_TRUE(res == res2);
    res = collide_Test<RSS>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    res2 = collide_front_list_RSS_Test(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    ASSERT_TRUE(res == res2);
  }
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


template<typename BV>
bool collide_front_list_Test(const Transform& tf1, const Transform& tf2,
                             const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                             const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                             SplitMethodType split_method,
                             bool refit_bottomup, bool verbose)
{
  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.reset(new BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new BVSplitter<BV>(split_method));

  BVHFrontList front_list;


  std::vector<Vec3f> vertices1_new(vertices1.size());
  for(unsigned int i = 0; i < vertices1_new.size(); ++i)
  {
    vertices1_new[i] = tf1.R * vertices1[i] + tf1.T;
  }

  m1.beginModel();
  m1.addSubModel(vertices1_new, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  SimpleTransform pose1, pose2;

  MeshCollisionTraversalNode<BV> node;

  if(!initialize<BV>(node, m1, pose1, m2, pose2))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;
  node.num_max_contacts = std::numeric_limits<int>::max(); // front technique needs all the contacts;
  node.exhaustive = true;
  node.enable_contact = false;

  collide(&node, &front_list);

  if(verbose) std::cout << "front list size " << front_list.size() << std::endl;


  // update the mesh
  for(unsigned int i = 0; i < vertices1.size(); ++i)
  {
    vertices1_new[i] = tf2.R * vertices1[i] + tf2.T;
  }

  m1.beginReplaceModel();
  m1.replaceSubModel(vertices1_new);
  m1.endReplaceModel(true, refit_bottomup);

  m2.beginReplaceModel();
  m2.replaceSubModel(vertices2);
  m2.endReplaceModel(true, refit_bottomup);

  node.pairs.clear();
  collide(&node, &front_list);

  if(node.pairs.size() > 0)
    return true;
  else
    return false;
}


bool collide_front_list_OBB_Test(const Transform& tf1, const Transform& tf2,
                                 const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                                 const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                                 SplitMethodType split_method, bool verbose)
{
  BVHModel<OBB> m1;
  BVHModel<OBB> m2;
  m1.bv_splitter.reset(new BVSplitter<OBB>(split_method));
  m2.bv_splitter.reset(new BVSplitter<OBB>(split_method));

  BVHFrontList front_list;

  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  SimpleTransform pose1(tf1.R, tf1.T), pose2;

  MeshCollisionTraversalNodeOBB node;

  if(!initialize(node, (const BVHModel<OBB>&)m1, pose1, (const BVHModel<OBB>&)m2, pose2))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;
  node.num_max_contacts = std::numeric_limits<int>::max(); // front technique needs all the contacts;
  node.exhaustive = true;
  node.enable_contact = false;

  collide(&node, &front_list);

  if(verbose) std::cout << "front list size " << front_list.size() << std::endl;


  // update the mesh
  pose1.setTransform(tf2.R, tf2.T);
  if(!initialize(node, (const BVHModel<OBB>&)m1, pose1, (const BVHModel<OBB>&)m2, pose2))
    std::cout << "initialize error" << std::endl;

  node.pairs.clear();
  collide(&node, &front_list);

  if(node.pairs.size() > 0)
    return true;
  else
    return false;
}


bool collide_front_list_RSS_Test(const Transform& tf1, const Transform& tf2,
                                 const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                                 const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                                 SplitMethodType split_method, bool verbose)
{
  BVHModel<RSS> m1;
  BVHModel<RSS> m2;
  m1.bv_splitter.reset(new BVSplitter<RSS>(split_method));
  m2.bv_splitter.reset(new BVSplitter<RSS>(split_method));

  BVHFrontList front_list;

  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  SimpleTransform pose1(tf1.R, tf1.T), pose2;

  MeshCollisionTraversalNodeRSS node;

  if(!initialize(node, (const BVHModel<RSS>&)m1, pose1, (const BVHModel<RSS>&)m2, pose2))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;
  node.num_max_contacts = std::numeric_limits<int>::max(); // front technique needs all the contacts;
  node.exhaustive = true;
  node.enable_contact = false;

  collide(&node, &front_list);

  if(verbose) std::cout << "front list size " << front_list.size() << std::endl;


  // update the mesh
  pose1.setTransform(tf2.R, tf2.T);
  if(!initialize(node, (const BVHModel<RSS>&)m1, pose1, (const BVHModel<RSS>&)m2, pose2))
    std::cout << "initialize error" << std::endl;

  node.pairs.clear();
  collide(&node, &front_list);

  if(node.pairs.size() > 0)
    return true;
  else
    return false;
}


template<typename BV>
bool collide_Test(const Transform& tf,
                  const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                  const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose)
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

  SimpleTransform pose1(tf.R, tf.T), pose2;

  MeshCollisionTraversalNode<BV> node;

  if(!initialize<BV>(node, m1, pose1, m2, pose2))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;
  node.num_max_contacts = std::numeric_limits<int>::max();
  node.exhaustive = true;
  node.enable_contact = false;

  collide(&node);


  if(node.pairs.size() > 0)
    return true;
  else
    return false;
}


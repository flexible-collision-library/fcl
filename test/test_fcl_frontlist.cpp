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


#define BOOST_TEST_MODULE "FCL_FRONT_LIST"
#include <boost/test/unit_test.hpp>

#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/collision_node.h"
#include "test_fcl_utility.h"

#include "fcl_resources/config.h"
#include <boost/filesystem.hpp>

using namespace fcl;

template<typename BV>
bool collide_front_list_Test(const Transform3f& tf1, const Transform3f& tf2,
                             const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                             const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                             SplitMethodType split_method,
                             bool refit_bottomup, bool verbose);

template<typename BV, typename TraversalNode>
bool collide_front_list_Test_Oriented(const Transform3f& tf1, const Transform3f& tf2,
                                      const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                                      const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                                      SplitMethodType split_method, bool verbose);


template<typename BV>
bool collide_Test(const Transform3f& tf,
                  const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                  const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose);

// TODO: randomly still have some runtime error
BOOST_AUTO_TEST_CASE(front_list)
{
  std::vector<Vec3f> p1, p2;
  std::vector<Triangle> t1, t2;
  boost::filesystem::path path(TEST_RESOURCES_DIR);
  loadOBJFile((path / "env.obj").string().c_str(), p1, t1);
  loadOBJFile((path / "rob.obj").string().c_str(), p2, t2);

  std::vector<Transform3f> transforms; // t0
  std::vector<Transform3f> transforms2; // t1
  FCL_REAL extents[] = {-3000, -3000, 0, 3000, 3000, 3000};
  FCL_REAL delta_trans[] = {1, 1, 1};
  std::size_t n = 10;
  bool verbose = false;

  generateRandomTransforms(extents, delta_trans, 0.005 * 2 * 3.1415, transforms, transforms2, n);

  bool res, res2;

  for(std::size_t i = 0; i < transforms.size(); ++i)
  {
    res = collide_Test<AABB>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    res2 = collide_front_list_Test<AABB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, false, verbose);
    BOOST_CHECK(res == res2);
    res = collide_Test<AABB>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    res2 = collide_front_list_Test<AABB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, false, verbose);
    BOOST_CHECK(res == res2);
    res = collide_Test<AABB>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    res2 = collide_front_list_Test<AABB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, false, verbose);
    BOOST_CHECK(res == res2);
  }

  for(std::size_t i = 0; i < transforms.size(); ++i)
  {
    res = collide_Test<OBB>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    res2 = collide_front_list_Test<OBB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, false, verbose);
    BOOST_CHECK(res == res2);
    res = collide_Test<OBB>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    res2 = collide_front_list_Test<OBB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, false, verbose);
    BOOST_CHECK(res == res2);
    res = collide_Test<OBB>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    res2 = collide_front_list_Test<OBB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, false, verbose);
    BOOST_CHECK(res == res2);
  }

  for(std::size_t i = 0; i < transforms.size(); ++i)
  {
    // Disabled broken test lines. Please see #25.
    // res = collide_Test<RSS>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    // res2 = collide_front_list_Test<RSS>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, false, verbose);
    // BOOST_CHECK(res == res2);
    res = collide_Test<RSS>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    res2 = collide_front_list_Test<RSS>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, false, verbose);
    BOOST_CHECK(res == res2);
    res = collide_Test<RSS>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    res2 = collide_front_list_Test<RSS>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, false, verbose);
    BOOST_CHECK(res == res2);
  }

  for(std::size_t i = 0; i < transforms.size(); ++i)
  {
    res = collide_Test<KDOP<16> >(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    res2 = collide_front_list_Test<KDOP<16> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, false, verbose);
    BOOST_CHECK(res == res2);
    res = collide_Test<KDOP<16> >(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    res2 = collide_front_list_Test<KDOP<16> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, false, verbose);
    BOOST_CHECK(res == res2);
    res = collide_Test<KDOP<16> >(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    res2 = collide_front_list_Test<KDOP<16> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, false, verbose);
    BOOST_CHECK(res == res2);
  }

  for(std::size_t i = 0; i < transforms.size(); ++i)
  {
    res = collide_Test<KDOP<18> >(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    res2 = collide_front_list_Test<KDOP<18> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, false, verbose);
    BOOST_CHECK(res == res2);
    res = collide_Test<KDOP<18> >(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    res2 = collide_front_list_Test<KDOP<18> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, false, verbose);
    BOOST_CHECK(res == res2);
    res = collide_Test<KDOP<18> >(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    res2 = collide_front_list_Test<KDOP<18> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, false, verbose);
    BOOST_CHECK(res == res2);
  }

  for(std::size_t i = 0; i < transforms.size(); ++i)
  {
    res = collide_Test<KDOP<24> >(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    res2 = collide_front_list_Test<KDOP<24> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, false, verbose);
    BOOST_CHECK(res == res2);
    res = collide_Test<KDOP<24> >(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    res2 = collide_front_list_Test<KDOP<24> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, false, verbose);
    BOOST_CHECK(res == res2);
    res = collide_Test<KDOP<24> >(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    res2 = collide_front_list_Test<KDOP<24> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, false, verbose);
    BOOST_CHECK(res == res2);
  }

  for(std::size_t i = 0; i < transforms.size(); ++i)
  {
    res = collide_Test<RSS>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    res2 = collide_front_list_Test_Oriented<RSS, MeshCollisionTraversalNodeRSS>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    BOOST_CHECK(res == res2);
    res = collide_Test<RSS>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    res2 = collide_front_list_Test_Oriented<RSS, MeshCollisionTraversalNodeRSS>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    BOOST_CHECK(res == res2);
    res = collide_Test<RSS>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    res2 = collide_front_list_Test_Oriented<RSS, MeshCollisionTraversalNodeRSS>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    BOOST_CHECK(res == res2);
  }

  for(std::size_t i = 0; i < transforms.size(); ++i)
  {
    res = collide_Test<OBB>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    res2 = collide_front_list_Test_Oriented<OBB, MeshCollisionTraversalNodeOBB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
    BOOST_CHECK(res == res2);
    res = collide_Test<OBB>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    res2 = collide_front_list_Test_Oriented<OBB, MeshCollisionTraversalNodeOBB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    BOOST_CHECK(res == res2);
    res = collide_Test<OBB>(transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    res2 = collide_front_list_Test_Oriented<OBB, MeshCollisionTraversalNodeOBB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
    BOOST_CHECK(res == res2);
  }

}

template<typename BV>
bool collide_front_list_Test(const Transform3f& tf1, const Transform3f& tf2,
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
  for(std::size_t i = 0; i < vertices1_new.size(); ++i)
  {
    vertices1_new[i] = tf1.transform(vertices1[i]);
  }

  m1.beginModel();
  m1.addSubModel(vertices1_new, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  Transform3f pose1, pose2;

  CollisionResult local_result;	
  MeshCollisionTraversalNode<BV> node;

  if(!initialize<BV>(node, m1, pose1, m2, pose2,
                     CollisionRequest(std::numeric_limits<int>::max(), false), local_result))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;

  collide(&node, &front_list);

  if(verbose) std::cout << "front list size " << front_list.size() << std::endl;


  // update the mesh
  for(std::size_t i = 0; i < vertices1.size(); ++i)
  {
    vertices1_new[i] = tf2.transform(vertices1[i]);
  }

  m1.beginReplaceModel();
  m1.replaceSubModel(vertices1_new);
  m1.endReplaceModel(true, refit_bottomup);

  m2.beginReplaceModel();
  m2.replaceSubModel(vertices2);
  m2.endReplaceModel(true, refit_bottomup);

  local_result.clear();
  collide(&node, &front_list);

  if(local_result.numContacts() > 0)
    return true;
  else
    return false;
}




template<typename BV, typename TraversalNode>
bool collide_front_list_Test_Oriented(const Transform3f& tf1, const Transform3f& tf2,
                                      const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                                      const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                                      SplitMethodType split_method, bool verbose)
{
  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.reset(new BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new BVSplitter<BV>(split_method));

  BVHFrontList front_list;

  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  Transform3f pose1(tf1), pose2;

  CollisionResult local_result;	
  TraversalNode node;

  if(!initialize(node, (const BVHModel<BV>&)m1, pose1, (const BVHModel<BV>&)m2, pose2,
                 CollisionRequest(std::numeric_limits<int>::max(), false), local_result))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;

  collide(&node, &front_list);

  if(verbose) std::cout << "front list size " << front_list.size() << std::endl;


  // update the mesh
  pose1 = tf2;
  if(!initialize(node, (const BVHModel<BV>&)m1, pose1, (const BVHModel<BV>&)m2, pose2, CollisionRequest(), local_result))
    std::cout << "initialize error" << std::endl;

  local_result.clear();
  collide(&node, &front_list);

  if(local_result.numContacts() > 0)
    return true;
  else
    return false;
}


template<typename BV>
bool collide_Test(const Transform3f& tf,
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

  Transform3f pose1(tf), pose2;

  CollisionResult local_result;	
  MeshCollisionTraversalNode<BV> node;

  if(!initialize<BV>(node, m1, pose1, m2, pose2,
                     CollisionRequest(std::numeric_limits<int>::max(), false), local_result))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;

  collide(&node);


  if(local_result.numContacts() > 0)
    return true;
  else
    return false;
}


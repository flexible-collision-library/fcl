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
#include <boost/timer.hpp>

using namespace fcl;

template<typename BV>
bool continuous_collide_Test(const Transform& tf1, const Transform& tf2,
                             const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                             const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                             SplitMethodType split_method,
                             bool refit_bottomup, bool verbose);

template<typename BV>
bool discrete_continuous_collide_Test(const Transform& tf1, const Transform& tf2,
                                      const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                                      const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                                      SplitMethodType split_method,
                                      unsigned int nsamples,
                                      bool verbose);

int num_max_contacts = -1;
bool exhaustive = true;
bool enable_contact = true;
unsigned int n_dcd_samples = 10;



TEST(ccd_test, mesh_mesh_bottomup)
{
  double t_ccd = 0;
  double t_dcd = 0;

  std::vector<Vec3f> p1, p2;
  std::vector<Triangle> t1, t2;
  loadOBJFile("test/env.obj", p1, t1);
  loadOBJFile("test/rob.obj", p2, t2);

  std::vector<Transform> transforms; // t0
  std::vector<Transform> transforms2; // t1
  BVH_REAL extents[] = {-3000, -3000, 0, 3000, 3000, 3000};
  BVH_REAL delta_trans[] = {10, 10, 10};
  int n = 100;
  bool verbose = false;

  generateRandomTransform(extents, transforms, transforms2, delta_trans, 0.005 * 2 * 3.1415, n);

  bool res, res2;

  for(unsigned int i = 0; i < transforms.size(); ++i)
  {
    boost::timer timer1;
    res = discrete_continuous_collide_Test<AABB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, n_dcd_samples, verbose);
    t_dcd += timer1.elapsed();

    boost::timer timer2;
    res2 = continuous_collide_Test<AABB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, true, verbose);
    t_ccd += timer2.elapsed();
    if(res)
      ASSERT_TRUE(res == res2);
    else
    {
      if(res2)
        std::cout << "CCD detects collision missed in DCD" << std::endl;
    }
  }

  std::cout << "dcd timing: " << t_dcd << " sec" << std::endl;
  std::cout << "ccd timing: " << t_ccd << " sec" << std::endl;
}


TEST(ccd_test, mesh_mesh_topdown)
{
  double t_ccd = 0;
  double t_dcd = 0;

  std::vector<Vec3f> p1, p2;
  std::vector<Triangle> t1, t2;
  loadOBJFile("test/env.obj", p1, t1);
  loadOBJFile("test/rob.obj", p2, t2);

  std::vector<Transform> transforms; // t0
  std::vector<Transform> transforms2; // t1
  BVH_REAL extents[] = {-3000, -3000, 0, 3000, 3000, 3000};
  BVH_REAL delta_trans[] = {10, 10, 10};
  int n = 100;
  bool verbose = false;

  generateRandomTransform(extents, transforms, transforms2, delta_trans, 0.005 * 2 * 3.1415, n);

  bool res, res2;

  for(unsigned int i = 0; i < transforms.size(); ++i)
  {
    boost::timer timer1;
    res = discrete_continuous_collide_Test<AABB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, n_dcd_samples, verbose);
    t_dcd += timer1.elapsed();

    boost::timer timer2;
    res2 = continuous_collide_Test<AABB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, false, verbose);
    t_ccd += timer2.elapsed();
    if(res)
      ASSERT_TRUE(res == res2);
    else
    {
      if(res2)
        std::cout << "CCD detects collision missed in DCD" << std::endl;
    }
  }
  std::cout << "dcd timing: " << t_dcd << " sec" << std::endl;
  std::cout << "ccd timing: " << t_ccd << " sec" << std::endl;
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

template<typename BV>
bool continuous_collide_Test(const Transform& tf1, const Transform& tf2,
                             const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                             const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                             SplitMethodType split_method,
                             bool refit_bottomup, bool verbose)
{
  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.reset(new BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new BVSplitter<BV>(split_method));

  std::vector<Vec3f> vertices1_new(vertices1.size());
  for(unsigned int i = 0; i < vertices1_new.size(); ++i)
  {
    vertices1_new[i] = matMulVec(tf1.R, vertices1[i]) + tf1.T;
  }

  m1.beginModel();
  m1.addSubModel(vertices1_new, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  MeshCollisionTraversalNode<BV> node0;

  if(!initialize<BV>(node0, m1, m2))
    std::cout << "initialize error" << std::endl;

  node0.enable_statistics = verbose;
  node0.num_max_contacts = num_max_contacts;
  node0.exhaustive = exhaustive;
  node0.enable_contact = enable_contact;

  collide(&node0);
  if(node0.pairs.size() > 0)
    return true;

  // update
  for(unsigned int i = 0; i < vertices1_new.size(); ++i)
  {
    vertices1_new[i] = matMulVec(tf2.R, vertices1[i]) + tf2.T;
  }

  m1.beginUpdateModel();
  m1.updateSubModel(vertices1_new);
  m1.endUpdateModel(true, refit_bottomup);

  m2.beginUpdateModel();
  m2.updateSubModel(vertices2);
  m2.endUpdateModel(true, refit_bottomup);

  MeshContinuousCollisionTraversalNode<BV> node;

  if(!initialize<BV>(node, m1, m2))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;
  node.num_max_contacts = num_max_contacts;
  node.exhaustive = exhaustive;
  node.enable_contact = enable_contact;

  collide(&node);

  if(node.pairs.size() > 0)
    return true;
  else
    return false;
}

template<typename BV>
bool discrete_continuous_collide_Test(const Transform& tf1, const Transform& tf2,
                                      const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                                      const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                                      SplitMethodType split_method,
                                      unsigned int nsamples,
                                      bool verbose)
{
  std::vector<Vec3f> vertices1_t1(vertices1.size());
  for(unsigned int i = 0; i < vertices1_t1.size(); ++i)
  {
    vertices1_t1[i] = matMulVec(tf1.R, vertices1[i]) + tf1.T;
  }

  std::vector<Vec3f> vertices1_t2(vertices1.size());
  for(unsigned int i = 0; i < vertices1_t2.size(); ++i)
  {
    vertices1_t2[i] = matMulVec(tf2.R, vertices1[i]) + tf2.T;
  }

  std::vector<Vec3f> vertices1_t(vertices1.size());

  for(unsigned int i = 0; i <= nsamples; ++i)
  {
    BVHModel<BV> m1;
    BVHModel<BV> m2;
    m1.bv_splitter.reset(new BVSplitter<BV>(split_method));
    m2.bv_splitter.reset(new BVSplitter<BV>(split_method));

    BVH_REAL delta = i / (BVH_REAL)nsamples;

    for(unsigned int j = 0; j < vertices1_t.size(); ++j)
      vertices1_t[j] = vertices1_t1[j] * (1 - delta) + vertices1_t2[j] * delta;

    m1.beginModel();
    m1.addSubModel(vertices1_t, triangles1);
    m1.endModel();

    m2.beginModel();
    m2.addSubModel(vertices2, triangles2);
    m2.endModel();

    MeshCollisionTraversalNode<BV> node;
    if(!initialize<BV>(node, m1, m2))
      std::cout << "initialize error" << std::endl;

    node.enable_statistics = verbose;
    node.num_max_contacts = num_max_contacts;
    node.exhaustive = exhaustive;
    node.enable_contact = enable_contact;

    collide(&node);

    if(node.pairs.size() > 0) return true;
  }

  return false;
}

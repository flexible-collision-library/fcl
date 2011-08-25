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
#include "fcl/collision.h"
#include "test_core_utility.h"
#include <gtest/gtest.h>

#if USE_PQP
#include <PQP.h>
#endif

using namespace fcl;

template<typename BV>
bool collide_Test(const Transform& tf,
                  const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                  const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose = true);

template<typename BV>
bool collide_Test2(const Transform& tf,
                   const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                   const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose = true);

bool collide_Test_OBB(const Transform& tf,
                      const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                      const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose = true);

bool collide_Test_RSS(const Transform& tf,
                      const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                      const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose = true);

template<typename BV>
bool test_collide_func(const Transform& tf,
                       const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                       const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method);

int num_max_contacts = -1;
bool exhaustive = true;
bool enable_contact = true;

std::vector<BVHCollisionPair> global_pairs;
std::vector<BVHCollisionPair> global_pairs_now;

std::vector<std::pair<int, int> > PQP_pairs;

TEST(collision_test, mesh_mesh)
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
  bool verbose = false;

  generateRandomTransform(extents, transforms, transforms2, delta_trans, 0.005 * 2 * 3.1415, n);

  // collision
  for(unsigned int i = 0; i < transforms.size(); ++i)
  {
    global_pairs.clear();
    global_pairs_now.clear();
#if USE_PQP
    PQP_pairs.clear();
    collide_PQP(transforms[i], p1, t1, p2, t2, PQP_pairs, verbose);
    global_pairs.resize(PQP_pairs.size());
    for(unsigned int j = 0; j < PQP_pairs.size(); ++j)
      global_pairs[j] = BVHCollisionPair(PQP_pairs[j].first, PQP_pairs[j].second);

    PQP_pairs.clear();
    collide_PQP2(transforms[i], p1, t1, p2, t2, PQP_pairs, verbose);
    global_pairs_now.resize(PQP_pairs.size());
    for(unsigned int j = 0; j < PQP_pairs.size(); ++j)
      global_pairs_now[j] = BVHCollisionPair(PQP_pairs[j].first, PQP_pairs[j].second);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }
#endif

    collide_Test<OBB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);

#if USE_PQP
    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }
#endif

    collide_Test<OBB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test<OBB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test<RSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test<RSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test<RSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test<AABB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test<AABB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test<AABB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test<KDOP<24> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test<KDOP<24> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test<KDOP<24> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test<KDOP<18> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test<KDOP<18> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test<KDOP<18> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test<KDOP<16> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test<KDOP<16> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test<KDOP<16> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test2<OBB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test2<OBB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test2<OBB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test2<RSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test2<RSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test2<RSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test2<AABB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test2<AABB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test2<AABB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test2<KDOP<24> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test2<KDOP<24> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test2<KDOP<24> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test2<KDOP<18> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test2<KDOP<18> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test2<KDOP<18> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test2<KDOP<16> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test2<KDOP<16> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test2<KDOP<16> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test_OBB(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test_OBB(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test_OBB(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test_RSS(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test_RSS(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    collide_Test_RSS(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    test_collide_func<RSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    test_collide_func<OBB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }

    test_collide_func<AABB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);

    ASSERT_TRUE(global_pairs.size() == global_pairs_now.size());
    for(unsigned int j = 0; j < global_pairs.size(); ++j)
    {
      ASSERT_TRUE(global_pairs[j].id1 == global_pairs_now[j].id1);
      ASSERT_TRUE(global_pairs[j].id2 == global_pairs_now[j].id2);
    }
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

template<typename BV>
bool collide_Test2(const Transform& tf,
                  const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                  const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose)
{
  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.reset(new BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new BVSplitter<BV>(split_method));

  std::vector<Vec3f> vertices1_new(vertices1.size());
  for(unsigned int i = 0; i < vertices1_new.size(); ++i)
  {
    vertices1_new[i] = MxV(tf.R, vertices1[i]) + tf.T;
  }


  m1.beginModel();
  m1.addSubModel(vertices1_new, triangles1);
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


  if(node.pairs.size() > 0)
  {
    std::vector<BVHCollisionPair> pairs(node.pairs.size());
    for(unsigned i = 0; i < node.pairs.size(); ++i)
      pairs[i] = node.pairs[i];

    std::sort(pairs.begin(), pairs.end(), BVHCollisionPairComp());

    if(global_pairs.size() == 0)
    {
      global_pairs.resize(pairs.size());
      for(unsigned int i = 0 ; i < pairs.size(); ++i)
        global_pairs[i] = pairs[i];
    }
    else
    {
      global_pairs_now.resize(pairs.size());
      for(unsigned int i = 0 ; i < pairs.size(); ++i)
        global_pairs_now[i] = pairs[i];
    }


    if(verbose)
      std::cout << "in collision " << node.pairs.size() << ": " << std::endl;
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

  Vec3f R2[3];
  R2[0] = Vec3f(1, 0, 0);
  R2[1] = Vec3f(0, 1, 0);
  R2[2] = Vec3f(0, 0, 1);
  Vec3f T2;

  MeshCollisionTraversalNode<BV> node;

  if(!initialize<BV>(node, m1, m2, tf.R, tf.T, R2, T2))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;
  node.num_max_contacts = num_max_contacts;
  node.exhaustive = exhaustive;
  node.enable_contact = enable_contact;

  collide(&node);


  if(node.pairs.size() > 0)
  {
    std::vector<BVHCollisionPair> pairs(node.pairs.size());
    for(unsigned i = 0; i < node.pairs.size(); ++i)
      pairs[i] = node.pairs[i];

    std::sort(pairs.begin(), pairs.end(), BVHCollisionPairComp());

    if(global_pairs.size() == 0)
    {
      global_pairs.resize(pairs.size());
      for(unsigned int i = 0 ; i < pairs.size(); ++i)
        global_pairs[i] = pairs[i];
    }
    else
    {
      global_pairs_now.resize(pairs.size());
      for(unsigned int i = 0 ; i < pairs.size(); ++i)
        global_pairs_now[i] = pairs[i];
    }

    if(verbose)
      std::cout << "in collision " << node.pairs.size() << ": " << std::endl;
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

bool collide_Test_OBB(const Transform& tf,
                      const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                      const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose)
{
  BVHModel<OBB> m1;
  BVHModel<OBB> m2;
  m1.bv_splitter.reset(new BVSplitter<OBB>(split_method));
  m2.bv_splitter.reset(new BVSplitter<OBB>(split_method));

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

  MeshCollisionTraversalNodeOBB node;
  if(!initialize(node, (const BVHModel<OBB>&)m1, (const BVHModel<OBB>&)m2, tf.R, tf.T, R2, T2))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;
  node.num_max_contacts = num_max_contacts;
  node.exhaustive = exhaustive;
  node.enable_contact = enable_contact;

  collide(&node);

  if(node.pairs.size() > 0)
  {
    std::vector<BVHCollisionPair> pairs(node.pairs.size());
    for(unsigned i = 0; i < node.pairs.size(); ++i)
      pairs[i] = node.pairs[i];

    std::sort(pairs.begin(), pairs.end(), BVHCollisionPairComp());

    if(global_pairs.size() == 0)
    {
      global_pairs.resize(pairs.size());
      for(unsigned int i = 0 ; i < pairs.size(); ++i)
        global_pairs[i] = pairs[i];
    }
    else
    {
      global_pairs_now.resize(pairs.size());
      for(unsigned int i = 0 ; i < pairs.size(); ++i)
        global_pairs_now[i] = pairs[i];
    }


    if(verbose)
      std::cout << "in collision " << node.pairs.size() << ": " << std::endl;
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

bool collide_Test_RSS(const Transform& tf,
                      const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                      const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose)
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

  MeshCollisionTraversalNodeRSS node;
  if(!initialize(node, (const BVHModel<RSS>&)m1, (const BVHModel<RSS>&)m2, tf.R, tf.T, R2, T2))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;
  node.num_max_contacts = num_max_contacts;
  node.exhaustive = exhaustive;
  node.enable_contact = enable_contact;

  collide(&node);

  if(node.pairs.size() > 0)
  {
    std::vector<BVHCollisionPair> pairs(node.pairs.size());
    for(unsigned i = 0; i < node.pairs.size(); ++i)
      pairs[i] = node.pairs[i];

    std::sort(pairs.begin(), pairs.end(), BVHCollisionPairComp());

    if(global_pairs.size() == 0)
    {
      global_pairs.resize(pairs.size());
      for(unsigned int i = 0 ; i < pairs.size(); ++i)
        global_pairs[i] = pairs[i];
    }
    else
    {
      global_pairs_now.resize(pairs.size());
      for(unsigned int i = 0 ; i < pairs.size(); ++i)
        global_pairs_now[i] = pairs[i];
    }

    if(verbose)
      std::cout << "in collision " << node.pairs.size() << ": " << std::endl;
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
bool test_collide_func(const Transform& tf,
                       const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                       const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method)
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

  m1.setRotation(tf.R);
  m1.setTranslation(tf.T);
  m2.setRotation(R2);
  m2.setTranslation(T2);


  std::vector<Contact> contacts;
  int num_contacts = collide(&m1, &m2, num_max_contacts, enable_contact, exhaustive, contacts);
  global_pairs_now.resize(num_contacts);

  for(int i = 0; i < num_contacts; ++i)
  {
    global_pairs_now[i].id1 = contacts[i].b1;
    global_pairs_now[i].id2 = contacts[i].b2;
  }

  std::sort(global_pairs_now.begin(), global_pairs_now.end(), BVHCollisionPairComp());

  if(num_contacts > 0) return true;
  else return false;
}

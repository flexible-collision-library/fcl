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


#if USE_SVMLIGHT

#include "fcl/traversal_node_bvhs.h"
#include "fcl/collision_node.h"
#include "fcl/simple_setup.h"
#include "test_core_utility.h"

#if USE_PQP
#include <PQP.h>
#endif

using namespace fcl;

template<typename BV>
bool collide_Test_PP(const Transform& tf,
                     const std::vector<Vec3f>& vertices1, const std::vector<Vec3f>& vertices2, SplitMethodType split_method, bool verbose = true);

template<typename BV>
bool collide_Test_MP(const Transform& tf,
                     const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                     const std::vector<Vec3f>& vertices2, SplitMethodType split_method, bool verbose = true);
template<typename BV>
bool collide_Test_PM(const Transform& tf,
                     const std::vector<Vec3f>& vertices1,
                     const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose = true);

bool collide_Test_PP_OBB(const Transform& tf,
                         const std::vector<Vec3f>& vertices1, const std::vector<Vec3f>& vertices2, SplitMethodType split_method, bool verbose = true);

bool collide_Test_MP_OBB(const Transform& tf,
                         const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                         const std::vector<Vec3f>& vertices2, SplitMethodType split_method, bool verbose = true);

bool collide_Test_PM_OBB(const Transform& tf,
                         const std::vector<Vec3f>& vertices1,
                         const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose = true);

int num_max_contacts = 1;

int main()
{
  std::vector<Vec3f> p1, p2;
  std::vector<Triangle> t1, t2;
  loadOBJFile("env.obj", p1, t1);
  loadOBJFile("rob.obj", p2, t2);

  std::vector<Transform> transforms; // t0
  std::vector<Transform> transforms2; // t1
  std::vector<Transform> transforms_ccd; // t0
  std::vector<Transform> transforms_ccd2; // t1
  BVH_REAL extents[] = {-3000, -3000, 0, 3000, 3000, 3000};
  BVH_REAL delta_trans[] = {1, 1, 1};
  int n = 10;

  generateRandomTransform(extents, transforms, transforms2, delta_trans, 0.005 * 2 * 3.1415, n);

  for(unsigned int i = 0; i < transforms.size(); ++i)
  {
    std::cout << "test id " << i << std::endl;

    std::vector<std::pair<int, int> > PQP_pairs;
    DistanceRes PQP_dist;

#if USE_PQP
    collide_PQP(transforms[i], p1, t1, p2, t2, PQP_pairs);

    distance_PQP(transforms[i], p1, t1, p2, t2, PQP_dist);
#endif

    collide_Test_PP<OBB>(transforms[i], p1, p2, SPLIT_METHOD_MEAN);

    collide_Test_PP<OBB>(transforms[i], p1, p2, SPLIT_METHOD_BV_CENTER);

    collide_Test_PP<OBB>(transforms[i], p1, p2, SPLIT_METHOD_MEDIAN);

    collide_Test_PM<OBB>(transforms[i], p1, p2, t2, SPLIT_METHOD_MEAN);

    collide_Test_PM<OBB>(transforms[i], p1, p2, t2, SPLIT_METHOD_BV_CENTER);

    collide_Test_PM<OBB>(transforms[i], p1, p2, t2, SPLIT_METHOD_MEDIAN);

    collide_Test_MP<OBB>(transforms[i], p1, t1, p2, SPLIT_METHOD_MEAN);

    collide_Test_MP<OBB>(transforms[i], p1, t1, p2, SPLIT_METHOD_BV_CENTER);

    collide_Test_MP<OBB>(transforms[i], p1, t1, p2, SPLIT_METHOD_MEDIAN);

    collide_Test_PP_OBB(transforms[i], p1, p2, SPLIT_METHOD_MEAN);

    collide_Test_PP_OBB(transforms[i], p1, p2, SPLIT_METHOD_BV_CENTER);

    collide_Test_PP_OBB(transforms[i], p1, p2, SPLIT_METHOD_MEDIAN);

    collide_Test_PM_OBB(transforms[i], p1, p2, t2, SPLIT_METHOD_MEAN);

    collide_Test_PM_OBB(transforms[i], p1, p2, t2, SPLIT_METHOD_BV_CENTER);

    collide_Test_PM_OBB(transforms[i], p1, p2, t2, SPLIT_METHOD_MEDIAN);

    collide_Test_MP_OBB(transforms[i], p1, t1, p2, SPLIT_METHOD_MEAN);

    collide_Test_MP_OBB(transforms[i], p1, t1, p2, SPLIT_METHOD_BV_CENTER);

    collide_Test_MP_OBB(transforms[i], p1, t1, p2, SPLIT_METHOD_MEDIAN);

    std::cout << std::endl;
  }
}

template<typename BV>
bool collide_Test_PP(const Transform& tf,
                     const std::vector<Vec3f>& vertices1, const std::vector<Vec3f>& vertices2, SplitMethodType split_method, bool verbose)
{
  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.reset(new BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new BVSplitter<BV>(split_method));

  m1.beginModel();
  m1.addSubModel(vertices1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2);
  m2.endModel();

  Vec3f R2[3];
  R2[0] = Vec3f(1, 0, 0);
  R2[1] = Vec3f(0, 1, 0);
  R2[2] = Vec3f(0, 0, 1);
  Vec3f T2;

  PointCloudCollisionTraversalNode<BV> node;

  if(!initialize<BV, false, false>(node, m1, m2, tf.R, tf.T, R2, T2, 0.6, 20))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;
  node.num_max_contacts = num_max_contacts;

  collide(&node);

  if(node.pairs.size() > 0)
  {
    if(verbose)
    {
      std::cout << "in collision " << node.pairs.size() << ": " << std::endl;

      std::vector<BVHPointCollisionPair> pairs(node.pairs.size());
      for(unsigned i = 0; i < node.pairs.size(); ++i)
        pairs[i] = node.pairs[i];

      std::sort(pairs.begin(), pairs.end(), BVHPointCollisionPairComp());

      for(unsigned i = 0; i < pairs.size(); ++i)
      {
        std::cout << "(" << pairs[i].id1_start << "(" << pairs[i].id1_num << ")" << " " << pairs[i].id2_start << "(" << pairs[i].id2_num << ")" << " " << pairs[i].collision_prob << ") ";
      }
      std::cout << std::endl;
    }
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return true;
  }
  else
  {
    if(verbose) std::cout << "collision free " << std::endl;
    if(verbose) std::cout << node.max_collision_prob << std::endl;
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return false;
  }
}

bool collide_Test_PP_OBB(const Transform& tf,
                         const std::vector<Vec3f>& vertices1, const std::vector<Vec3f>& vertices2, SplitMethodType split_method, bool verbose)
{
  BVHModel<OBB> m1;
  BVHModel<OBB> m2;
  m1.bv_splitter.reset(new BVSplitter<OBB>(split_method));
  m2.bv_splitter.reset(new BVSplitter<OBB>(split_method));

  m1.beginModel();
  m1.addSubModel(vertices1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2);
  m2.endModel();

  Vec3f R2[3];
  R2[0] = Vec3f(1, 0, 0);
  R2[1] = Vec3f(0, 1, 0);
  R2[2] = Vec3f(0, 0, 1);
  Vec3f T2;

  PointCloudCollisionTraversalNodeOBB node;

  if(!initialize(node, m1, m2, tf.R, tf.T, R2, T2, 0.6, 20))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;
  node.num_max_contacts = num_max_contacts;

  collide(&node);

  if(node.pairs.size() > 0)
  {
    if(verbose)
    {
      std::cout << "in collision " << node.pairs.size() << ": " << std::endl;

      std::vector<BVHPointCollisionPair> pairs(node.pairs.size());
      for(unsigned i = 0; i < node.pairs.size(); ++i)
        pairs[i] = node.pairs[i];

      std::sort(pairs.begin(), pairs.end(), BVHPointCollisionPairComp());

      for(unsigned i = 0; i < pairs.size(); ++i)
      {
        std::cout << "(" << pairs[i].id1_start << "(" << pairs[i].id1_num << ")" << " " << pairs[i].id2_start << "(" << pairs[i].id2_num << ")" << " " << pairs[i].collision_prob << ") ";
      }
      std::cout << std::endl;
    }
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return true;
  }
  else
  {
    if(verbose) std::cout << "collision free " << std::endl;
    if(verbose) std::cout << node.max_collision_prob << std::endl;
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return false;
  }
}



template<typename BV>
bool collide_Test_MP(const Transform& tf,
                     const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                     const std::vector<Vec3f>& vertices2, SplitMethodType split_method, bool verbose)
{
  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.reset(new BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new BVSplitter<BV>(split_method));

  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2);
  m2.endModel();

  Vec3f R2[3];
  R2[0] = Vec3f(1, 0, 0);
  R2[1] = Vec3f(0, 1, 0);
  R2[2] = Vec3f(0, 0, 1);
  Vec3f T2;

  PointCloudMeshCollisionTraversalNode<BV> node;

  if(!initialize<BV, false, false>(node, m2, m1, R2, T2, tf.R, tf.T, 0.6, 20))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;
  node.num_max_contacts = num_max_contacts;

  collide(&node);

  if(node.pairs.size() > 0)
  {
    if(verbose)
    {
      std::cout << "in collision " << node.pairs.size() << ": " << std::endl;

      std::vector<BVHPointCollisionPair> pairs(node.pairs.size());
      for(unsigned i = 0; i < node.pairs.size(); ++i)
        pairs[i] = node.pairs[i];

      std::sort(pairs.begin(), pairs.end(), BVHPointCollisionPairComp());

      for(unsigned i = 0; i < pairs.size(); ++i)
      {
        std::cout << "(" << pairs[i].id1_start << "(" << pairs[i].id1_num << ")" << " " << pairs[i].id2_start << "(" << pairs[i].id2_num << ")" << " " << pairs[i].collision_prob << ") ";
      }
      std::cout << std::endl;
    }
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return true;
  }
  else
  {
    if(verbose) std::cout << "collision free " << std::endl;
    if(verbose) std::cout << node.max_collision_prob << std::endl;
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return false;
  }
}



bool collide_Test_MP_OBB(const Transform& tf,
                         const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                         const std::vector<Vec3f>& vertices2, SplitMethodType split_method, bool verbose)
{
  BVHModel<OBB> m1;
  BVHModel<OBB> m2;
  m1.bv_splitter.reset(new BVSplitter<OBB>(split_method));
  m2.bv_splitter.reset(new BVSplitter<OBB>(split_method));

  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2);
  m2.endModel();

  Vec3f R2[3];
  R2[0] = Vec3f(1, 0, 0);
  R2[1] = Vec3f(0, 1, 0);
  R2[2] = Vec3f(0, 0, 1);
  Vec3f T2;

  PointCloudMeshCollisionTraversalNodeOBB node;

  if(!initialize(node, m2, m1, R2, T2, tf.R, tf.T, 0.6, 20))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;
  node.num_max_contacts = num_max_contacts;

  collide(&node);

  if(node.pairs.size() > 0)
  {
    if(verbose)
    {
      std::cout << "in collision " << node.pairs.size() << ": " << std::endl;

      std::vector<BVHPointCollisionPair> pairs(node.pairs.size());
      for(unsigned i = 0; i < node.pairs.size(); ++i)
        pairs[i] = node.pairs[i];

      std::sort(pairs.begin(), pairs.end(), BVHPointCollisionPairComp());

      for(unsigned i = 0; i < pairs.size(); ++i)
      {
        std::cout << "(" << pairs[i].id1_start << "(" << pairs[i].id1_num << ")" << " " << pairs[i].id2_start << "(" << pairs[i].id2_num << ")" << " " << pairs[i].collision_prob << ") ";
      }
      std::cout << std::endl;
    }
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return true;
  }
  else
  {
    if(verbose) std::cout << "collision free " << std::endl;
    if(verbose) std::cout << node.max_collision_prob << std::endl;
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return false;
  }
}



template<typename BV>
bool collide_Test_PM(const Transform& tf,
                     const std::vector<Vec3f>& vertices1,
                     const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose)
{
  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.reset(new BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new BVSplitter<BV>(split_method));

  m1.beginModel();
  m1.addSubModel(vertices1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  Vec3f R2[3];
  R2[0] = Vec3f(1, 0, 0);
  R2[1] = Vec3f(0, 1, 0);
  R2[2] = Vec3f(0, 0, 1);
  Vec3f T2;

  PointCloudMeshCollisionTraversalNode<BV> node;

  if(!initialize<BV, false, false>(node, m1, m2, tf.R, tf.T, R2, T2, 0.6, 20))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;
  node.num_max_contacts = num_max_contacts;

  collide(&node);

  if(node.pairs.size() > 0)
  {
    if(verbose)
    {
      std::cout << "in collision " << node.pairs.size() << ": " << std::endl;

      std::vector<BVHPointCollisionPair> pairs(node.pairs.size());
      for(unsigned i = 0; i < node.pairs.size(); ++i)
        pairs[i] = node.pairs[i];

      std::sort(pairs.begin(), pairs.end(), BVHPointCollisionPairComp());

      for(unsigned i = 0; i < pairs.size(); ++i)
      {
        std::cout << "(" << pairs[i].id1_start << "(" << pairs[i].id1_num << ")" << " " << pairs[i].id2_start << "(" << pairs[i].id2_num << ")" << " " << pairs[i].collision_prob << ") ";
      }
      std::cout << std::endl;
    }
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return true;
  }
  else
  {
    if(verbose) std::cout << "collision free " << std::endl;
    if(verbose) std::cout << node.max_collision_prob << std::endl;
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return false;
  }
}


bool collide_Test_PM_OBB(const Transform& tf,
                         const std::vector<Vec3f>& vertices1,
                         const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose)
{
  BVHModel<OBB> m1;
  BVHModel<OBB> m2;
  m1.bv_splitter.reset(new BVSplitter<OBB>(split_method));
  m2.bv_splitter.reset(new BVSplitter<OBB>(split_method));

  m1.beginModel();
  m1.addSubModel(vertices1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  Vec3f R2[3];
  R2[0] = Vec3f(1, 0, 0);
  R2[1] = Vec3f(0, 1, 0);
  R2[2] = Vec3f(0, 0, 1);
  Vec3f T2;

  PointCloudMeshCollisionTraversalNodeOBB node;

  if(!initialize(node, m1, m2, tf.R, tf.T, R2, T2, 0.6, 20))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;
  node.num_max_contacts = num_max_contacts;

  collide(&node);

  if(node.pairs.size() > 0)
  {
    if(verbose)
    {
      std::cout << "in collision " << node.pairs.size() << ": " << std::endl;

      std::vector<BVHPointCollisionPair> pairs(node.pairs.size());
      for(unsigned i = 0; i < node.pairs.size(); ++i)
        pairs[i] = node.pairs[i];

      std::sort(pairs.begin(), pairs.end(), BVHPointCollisionPairComp());

      for(unsigned i = 0; i < pairs.size(); ++i)
      {
        std::cout << "(" << pairs[i].id1_start << "(" << pairs[i].id1_num << ")" << " " << pairs[i].id2_start << "(" << pairs[i].id2_num << ")" << " " << pairs[i].collision_prob << ") ";
      }
      std::cout << std::endl;
    }
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return true;
  }
  else
  {
    if(verbose) std::cout << "collision free " << std::endl;
    if(verbose) std::cout << node.max_collision_prob << std::endl;
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return false;
  }
}

#else
int main()
{
  return 1;
}
#endif



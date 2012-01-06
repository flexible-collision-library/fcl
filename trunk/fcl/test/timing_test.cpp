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
#include "timer.h"

#if USE_PQP
#include <PQP.h>
#endif


int main()
{
  std::vector<Vec3f> vertices1, vertices2;
  std::vector<Triangle> triangles1, triangles2;
  loadOBJFile("test/env.obj", vertices1, triangles1);
  loadOBJFile("test/rob.obj", vertices2, triangles2);

  std::vector<Transform> transforms; // t0
  std::vector<Transform> transforms2; // t1
  BVH_REAL extents[] = {-3000, -3000, 0, 3000, 3000, 3000};
  BVH_REAL delta_trans[] = {1, 1, 1};
  int n = 100000;
  bool verbose = false;

  generateRandomTransform(extents, transforms, transforms2, delta_trans, 0.005 * 2 * 3.1415, n);

  std::cout << "FCL timing 2" << std::endl;
  {
    double t_fcl = 0;

    BVHModel<OBB> m1;
    BVHModel<OBB> m2;
    SplitMethodType split_method = SPLIT_METHOD_MEAN;
    m1.bv_splitter.reset(new BVSplitter<OBB>(split_method));
    m2.bv_splitter.reset(new BVSplitter<OBB>(split_method));

    m1.beginModel();
    m1.addSubModel(vertices1, triangles1);
    m1.endModel();
    m1.makeParentRelative();

    m2.beginModel();
    m2.addSubModel(vertices2, triangles2);
    m2.endModel();
    m2.makeParentRelative();


    Matrix3f R2;
    R2.setIdentity();
    Vec3f T2;

    for(unsigned int i = 0; i < transforms.size(); ++i)
    {
      Transform& tf = transforms[i];

      MeshCollisionTraversalNodeOBB node;
      if(!initialize(node, (const BVHModel<OBB>&)m1, SimpleTransform(tf.R, tf.T), (const BVHModel<OBB>&)m2, SimpleTransform(R2, T2)))
        std::cout << "initialize error" << std::endl;

      node.enable_statistics = false;
      node.num_max_contacts = -1;
      node.exhaustive = false;
      node.enable_contact = false;

      Timer timer;
      timer.start();
      collide2(&node);
      timer.stop();
      t_fcl += timer.getElapsedTime();

      //std::cout << node.pairs.size() << std::endl;
    }

    std::cout << "fcl timing " << t_fcl << " ms" << std::endl;
  }


  std::cout << "PQP timing" << std::endl;
#if USE_PQP
  {
    double t_pqp = 0;

    PQP_Model m1, m2;

    m1.BeginModel();
    for(unsigned int i = 0; i < triangles1.size(); ++i)
    {
      const Triangle& t = triangles1[i];
      const Vec3f& p1 = vertices1[t[0]];
      const Vec3f& p2 = vertices1[t[1]];
      const Vec3f& p3 = vertices1[t[2]];

      PQP_REAL q1[3];
      PQP_REAL q2[3];
      PQP_REAL q3[3];

      q1[0] = p1[0]; q1[1] = p1[1]; q1[2] = p1[2];
      q2[0] = p2[0]; q2[1] = p2[1]; q2[2] = p2[2];
      q3[0] = p3[0]; q3[1] = p3[1]; q3[2] = p3[2];

      m1.AddTri(q1, q2, q3, i);
    }
    m1.EndModel();


    m2.BeginModel();
    for(unsigned int i = 0; i < triangles2.size(); ++i)
    {
      const Triangle& t = triangles2[i];
      const Vec3f& p1 = vertices2[t[0]];
      const Vec3f& p2 = vertices2[t[1]];
      const Vec3f& p3 = vertices2[t[2]];

      PQP_REAL q1[3];
      PQP_REAL q2[3];
      PQP_REAL q3[3];

      q1[0] = p1[0]; q1[1] = p1[1]; q1[2] = p1[2];
      q2[0] = p2[0]; q2[1] = p2[1]; q2[2] = p2[2];
      q3[0] = p3[0]; q3[1] = p3[1]; q3[2] = p3[2];

      m2.AddTri(q1, q2, q3, i);
    }
    m2.EndModel();


    for(unsigned int i = 0; i < transforms.size(); ++i)
    {
      Transform& tf = transforms[i];
      PQP_CollideResult res;
      PQP_REAL R1[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
      PQP_REAL R2[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
      PQP_REAL T1[3] = {0, 0, 0};
      PQP_REAL T2[3] = {0, 0, 0};
      T1[0] = tf.T[0];
      T1[1] = tf.T[1];
      T1[2] = tf.T[2];
      for(int i = 0; i < 3; ++i)
      {
        R1[i][0] = tf.R[i][0];
        R1[i][1] = tf.R[i][1];
        R1[i][2] = tf.R[i][2];
      }

      Timer timer;
      timer.start();
      PQP_Collide(&res, R1, T1, &m1, R2, T2, &m2, PQP_FIRST_CONTACT);
      timer.stop();
      t_pqp += timer.getElapsedTime();

      //std::cout << res.NumPairs() << std::endl;
    }

    std::cout << "pqp timing " << t_pqp << " ms" << std::endl;
  }
#endif

  std::cout << "FCL timing" << std::endl;
  {
    double t_fcl = 0;

    BVHModel<OBB> m1;
    BVHModel<OBB> m2;
    SplitMethodType split_method = SPLIT_METHOD_MEAN;
    m1.bv_splitter.reset(new BVSplitter<OBB>(split_method));
    m2.bv_splitter.reset(new BVSplitter<OBB>(split_method));

    m1.beginModel();
    m1.addSubModel(vertices1, triangles1);
    m1.endModel();

    m2.beginModel();
    m2.addSubModel(vertices2, triangles2);
    m2.endModel();

    Matrix3f R2;
    R2.setIdentity();
    Vec3f T2;

    for(unsigned int i = 0; i < transforms.size(); ++i)
    {
      Transform& tf = transforms[i];

      MeshCollisionTraversalNodeOBB node;
      if(!initialize(node, (const BVHModel<OBB>&)m1, SimpleTransform(tf.R, tf.T), (const BVHModel<OBB>&)m2, SimpleTransform(R2, T2)))
        std::cout << "initialize error" << std::endl;

      node.enable_statistics = false;
      node.num_max_contacts = -1;
      node.exhaustive = false;
      node.enable_contact = false;

      Timer timer;
      timer.start();
      collide(&node);
      timer.stop();
      t_fcl += timer.getElapsedTime();

      //std::cout << node.pairs.size() << std::endl;
    }

    std::cout << "fcl timing " << t_fcl << " ms" << std::endl;
  }



  return 1;

}

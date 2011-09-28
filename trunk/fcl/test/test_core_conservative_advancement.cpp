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

#include "fcl/conservative_advancement.h"
#include "test_core_utility.h"
#include "fcl/collision_node.h"
#include "fcl/simple_setup.h"


using namespace fcl;


bool CA_ccd_Test(const Transform& tf1, const Transform& tf2,
                 const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                 const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                 SplitMethodType split_method);

bool interp_ccd_Test(const Transform& tf1, const Transform& tf2,
                     const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                     const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                     SplitMethodType split_method,
                     unsigned int nsamples);

unsigned int n_dcd_samples = 10;

int main()
{
  std::vector<Vec3f> p1, p2;
  std::vector<Triangle> t1, t2;
  loadOBJFile("test/env.obj", p1, t1);
  loadOBJFile("test/rob.obj", p2, t2);

  std::vector<Transform> transforms; // t0
  std::vector<Transform> transforms2; // t1
  BVH_REAL extents[] = {-3000, -3000, 0, 3000, 3000, 3000};
  BVH_REAL delta_trans[] = {10, 10, 10};
  int n = 100;

  generateRandomTransform(extents, transforms, transforms2, delta_trans, 0.005 * 2 * 3.1415, n);

  for(unsigned int i = 0; i < transforms.size(); ++i)
  {
    std::cout << i << std::endl;
    bool res = CA_ccd_Test(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);
    if(res) std::cout << "yes"; else std::cout << "no";
    std::cout << std::endl;

    std::cout << "-----------" << std::endl;
    bool res2 = interp_ccd_Test(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, n_dcd_samples);

    if(res2) std::cout << "yes"; else std::cout << "no";
    std::cout << std::endl;

    std::cout << std::endl;
  }

  return 1;

}


bool CA_ccd_Test(const Transform& tf1, const Transform& tf2,
                 const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                 const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                 SplitMethodType split_method)
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
  Vec3f T2;
  R2[0] = Vec3f(1, 0, 0);
  R2[1] = Vec3f(0, 1, 0);
  R2[2] = Vec3f(0, 0, 1);

  std::vector<Contact> contacts;
  BVH_REAL toc;

  int b = conservativeAdvancement(&m1, tf1.R, tf1.T, tf2.R, tf2.T, Vec3f(),
                          &m2, R2, T2, R2, T2, Vec3f(),
                          1, false, false, contacts, toc);

  std::cout << "t " << toc << std::endl;

  return (b > 0);
}


bool interp_ccd_Test(const Transform& tf1, const Transform& tf2,
                     const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                     const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                     SplitMethodType split_method,
                     unsigned int nsamples)
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
  Vec3f T2;
  R2[0] = Vec3f(1, 0, 0);
  R2[1] = Vec3f(0, 1, 0);
  R2[2] = Vec3f(0, 0, 1);


  InterpMotion<RSS> motion1(tf1.R, tf1.T, tf2.R, tf2.T, Vec3f());

  for(unsigned int i = 0; i <= nsamples; ++i)
  {
    BVH_REAL curt = i / (BVH_REAL)nsamples;

    Vec3f R[3];
    Vec3f T;
    motion1.integrate(curt);
    motion1.getCurrentTransform(R, T);

    m1.setTransform(R, T);
    m2.setTransform(R2, T2);

    MeshCollisionTraversalNodeRSS node;
    if(!initialize(node, (const BVHModel<RSS>&)m1, (const BVHModel<RSS>&)m2))
      std::cout << "initialize error" << std::endl;

    node.enable_statistics = false;
    node.num_max_contacts = 1;
    node.exhaustive = false;
    node.enable_contact = false;

    collide(&node);

    if(node.pairs.size() > 0)
    {
      std::cout << "t " << curt << std::endl;
      return true;
    }
  }

  return false;
}

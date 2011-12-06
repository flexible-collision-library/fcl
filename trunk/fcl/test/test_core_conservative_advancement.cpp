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


bool CA_linear_Test(const Transform& tf1, const Transform& tf2,
                    const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                    const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                    SplitMethodType split_method,
                    bool use_COM,
                    BVH_REAL& toc);

bool linear_interp_Test(const Transform& tf1, const Transform& tf2,
                        const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                        const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                        SplitMethodType split_method,
                        unsigned int nsamples,
                        bool use_COM,
                        BVH_REAL& toc);

bool CA_screw_Test(const Transform& tf1, const Transform& tf2,
                   const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                   const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                   SplitMethodType split_method,
                   BVH_REAL& toc);


bool screw_interp_Test(const Transform& tf1, const Transform& tf2,
                       const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                       const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                       SplitMethodType split_method,
                       unsigned int nsamples,
                       BVH_REAL& toc);

// for spline test: the deBoor points are generated with screw motion, so will not pass tf1 and tf2 exactly
bool CA_spline_Test(const Transform& tf1, const Transform& tf2,
                    const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                    const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                    SplitMethodType split_method,
                    BVH_REAL& toc);


bool spline_interp_Test(const Transform& tf1, const Transform& tf2,
                        const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                        const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                        SplitMethodType split_method,
                        unsigned int nsamples,
                        BVH_REAL& toc);


unsigned int n_dcd_samples = 1000000;

int main()
{
  std::vector<Vec3f> p1, p2;
  std::vector<Triangle> t1, t2;
  loadOBJFile("test/env.obj", p1, t1);
  loadOBJFile("test/rob.obj", p2, t2);

  std::vector<Transform> transforms; // t0
  std::vector<Transform> transforms2; // t1
  BVH_REAL extents[] = {-3000, -3000, 0, 3000, 3000, 3000};
  BVH_REAL delta_trans[] = {100, 100, 100};
  int n = 100;

  generateRandomTransform(extents, transforms, transforms2, delta_trans, 0.005 * 2 * 3.1415, n);

  for(unsigned int i = 0; i < transforms.size(); ++i)
  {
    //std::cout << i << std::endl;
    BVH_REAL toc;
    bool res = CA_linear_Test(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, false, toc);

    BVH_REAL toc2;
    bool res2 = linear_interp_Test(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, n_dcd_samples, false, toc2);

    BVH_REAL toc3;
    bool res3 = CA_linear_Test(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, true, toc3);

    BVH_REAL toc4;
    bool res4 = linear_interp_Test(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, n_dcd_samples, true, toc4);

    BVH_REAL toc5;
    bool res5 = CA_screw_Test(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, toc5);

    BVH_REAL toc6;
    bool res6 = screw_interp_Test(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, n_dcd_samples, toc6);

    BVH_REAL toc7;
    bool res7 = CA_spline_Test(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, toc7);

    BVH_REAL toc8;
    bool res8 = spline_interp_Test(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, n_dcd_samples, toc8);


    if(!(i == 2 || i == 44 || i == 53))
      continue;


    if(res) std::cout << "yes "; else std::cout << "no ";
    if(res2) std::cout << "yes "; else std::cout << "no ";
    if(res3) std::cout << "yes "; else std::cout << "no ";
    if(res4) std::cout << "yes "; else std::cout << "no ";
    if(res5) std::cout << "yes "; else std::cout << "no ";
    if(res6) std::cout << "yes "; else std::cout << "no ";
    if(res7) std::cout << "yes "; else std::cout << "no ";
    if(res8) std::cout << "yes "; else std::cout << "no ";

    std::cout << std::endl;

    std::cout << toc << " " << toc2 << " " << toc3 << " " << toc4 << " " << toc5 << " " << toc6 << " " << toc7 << " " << toc8 << std::endl;
    std::cout << std::endl;
  }

  return 1;

}


bool CA_linear_Test(const Transform& tf1, const Transform& tf2,
                    const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                    const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                    SplitMethodType split_method,
                    bool use_COM,
                    BVH_REAL& toc)
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

  Matrix3f R2;
  R2.setIdentity();
  Vec3f T2;

  std::vector<Contact> contacts;

  Vec3f m1_ref;
  Vec3f m2_ref;

  if(use_COM)
  {
    for(unsigned int i = 0; i < vertices1.size(); ++i)
      m1_ref += vertices1[i];
    m1_ref *= (1.0 / vertices1.size());

    for(unsigned int i = 0; i < vertices2.size(); ++i)
      m2_ref += vertices2[i];
    m2_ref *= (1.0 / vertices2.size());
  }

  InterpMotion<RSS> motion1(tf1.R, tf1.T, tf2.R, tf2.T, m1_ref);
  InterpMotion<RSS> motion2(R2, T2, R2, T2, m2_ref);

  int b = conservativeAdvancement(&m1, &motion1,
                                  &m2, &motion2,
                                  1, false, false, contacts, toc);

  return (b > 0);
}

bool CA_screw_Test(const Transform& tf1, const Transform& tf2,
                   const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                   const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                   SplitMethodType split_method,
                   BVH_REAL& toc)
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

  Matrix3f R2;
  R2.setIdentity();
  Vec3f T2;

  std::vector<Contact> contacts;

  Vec3f m1_ref;
  Vec3f m2_ref;


  ScrewMotion<RSS> motion1(tf1.R, tf1.T, tf2.R, tf2.T);
  ScrewMotion<RSS> motion2(R2, T2, R2, T2);

  int b = conservativeAdvancement(&m1, &motion1,
                                  &m2, &motion2,
                                  1, false, false, contacts, toc);


  return (b > 0);

}
bool linear_interp_Test(const Transform& tf1, const Transform& tf2,
                     const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                     const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                     SplitMethodType split_method,
                     unsigned int nsamples,
                     bool use_COM,
                     BVH_REAL& toc)
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

  Matrix3f R2;
  R2.setIdentity();
  Vec3f T2;

  Vec3f m1_ref;
  Vec3f m2_ref;

  if(use_COM)
  {
    for(unsigned int i = 0; i < vertices1.size(); ++i)
      m1_ref += vertices1[i];
    m1_ref *= (1.0 / vertices1.size());

    for(unsigned int i = 0; i < vertices2.size(); ++i)
      m2_ref += vertices2[i];
    m2_ref *= (1.0 / vertices2.size());
  }

  InterpMotion<RSS> motion1(tf1.R, tf1.T, tf2.R, tf2.T, m1_ref);

  for(unsigned int i = 0; i <= nsamples; ++i)
  {
    BVH_REAL curt = i / (BVH_REAL)nsamples;

    Matrix3f R;
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
      toc = curt;
      return true;
    }
  }

  toc = 1;
  return false;
}

bool screw_interp_Test(const Transform& tf1, const Transform& tf2,
                       const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                       const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                       SplitMethodType split_method,
                       unsigned int nsamples,
                       BVH_REAL& toc)
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

  Matrix3f R2;
  R2.setIdentity();
  Vec3f T2;


  Vec3f m1_ref;
  Vec3f m2_ref;


  ScrewMotion<RSS> motion1(tf1.R, tf1.T, tf2.R, tf2.T);

  for(unsigned int i = 0; i <= nsamples; ++i)
  {
    BVH_REAL curt = i / (BVH_REAL)nsamples;

    Matrix3f R;
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
      toc = curt;
      return true;
    }
  }

  toc = 1;
  return false;
}


bool CA_spline_Test(const Transform& tf1, const Transform& tf2,
                    const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                    const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                    SplitMethodType split_method,
                    BVH_REAL& toc)
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

  ScrewMotion<RSS> motion(tf1.R, tf1.T, tf2.R, tf2.T);
  Vec3f Td_1[4];
  Vec3f Rd_1[4];
  Vec3f Td_2[4];
  Vec3f Rd_2[4];

  for(int i = 0; i < 4; ++i)
  {
    motion.integrate(i / 4.0);
    Matrix3f R;
    Vec3f T;
    motion.getCurrentTransform(R, T);
    SimpleQuaternion q;
    q.fromRotation(R);
    Vec3f axis;
    BVH_REAL angle;
    q.toAxisAngle(axis, angle);
    Td_1[i] = T;
    Rd_1[i] = axis * angle;
  }

  for(int i = 0; i < 4; ++i)
  {
    Td_2[i].setValue(0);
    Rd_2[i].setValue(0);
  }

  SplineMotion<RSS> motion1(Td_1[0], Td_1[1], Td_1[2], Td_1[3],
                            Rd_1[0], Rd_1[1], Rd_1[2], Rd_1[3]);

  SplineMotion<RSS> motion2(Td_2[0], Td_2[1], Td_2[2], Td_2[3],
                            Rd_2[0], Rd_2[1], Rd_2[2], Rd_2[3]);

  std::vector<Contact> contacts;

  int b = conservativeAdvancement(&m1, &motion1,
                                  &m2, &motion2,
                                  1, false, false, contacts, toc);


  return (b > 0);

}

bool spline_interp_Test(const Transform& tf1, const Transform& tf2,
                        const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                        const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                        SplitMethodType split_method,
                        unsigned int nsamples,
                        BVH_REAL& toc)
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


  ScrewMotion<RSS> motion(tf1.R, tf1.T, tf2.R, tf2.T);
  Vec3f Td_1[4];
  Vec3f Rd_1[4];
  Vec3f Td_2[4];
  Vec3f Rd_2[4];

  for(int i = 0; i < 4; ++i)
  {
    motion.integrate(i / 4.0);
    Matrix3f R;
    Vec3f T;
    motion.getCurrentTransform(R, T);
    SimpleQuaternion q;
    q.fromRotation(R);
    Vec3f axis;
    BVH_REAL angle;
    q.toAxisAngle(axis, angle);
    Td_1[i] = T;
    Rd_1[i] = axis * angle;
  }

  for(int i = 0; i < 4; ++i)
  {
    Td_2[i].setValue(0);
    Rd_2[i].setValue(0);
  }

  SplineMotion<RSS> motion1(Td_1[0], Td_1[1], Td_1[2], Td_1[3],
                            Rd_1[0], Rd_1[1], Rd_1[2], Rd_1[3]);

  SplineMotion<RSS> motion2(Td_2[0], Td_2[1], Td_2[2], Td_2[3],
                            Rd_2[0], Rd_2[1], Rd_2[2], Rd_2[3]);


  for(unsigned int i = 0; i < nsamples; ++i)
  {
    BVH_REAL curt = i / (BVH_REAL)nsamples;

    Matrix3f R;
    Vec3f T;
    motion1.integrate(curt);
    motion2.integrate(curt);

    motion1.getCurrentTransform(R, T);
    m1.setTransform(R, T);

    motion2.getCurrentTransform(R, T);
    m2.setTransform(R, T);

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
      toc = curt;
      return true;
    }
  }

  toc = 1;
  return false;
}

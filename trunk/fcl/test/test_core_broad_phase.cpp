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


#include "fcl/broad_phase_collision.h"
#include "fcl/geometric_shape_to_BVH_model.h"
#include "fcl/transform.h"
#include "test_core_utility.h"
#include <gtest/gtest.h>

using namespace fcl;


void generateEnvironments(std::vector<CollisionObject*>& env, int n);


TEST(test_core, broad_phase_collision)
{
  bool exhaustive = false;

  std::vector<CollisionObject*> env;
  generateEnvironments(env, 100);

  std::vector<CollisionObject*> query;
  generateEnvironments(query, 10);

  BroadPhaseCollisionManager* manager1 = new NaiveCollisionManager();
  BroadPhaseCollisionManager* manager2 = new SSaPCollisionManager();
  BroadPhaseCollisionManager* manager3 = new SaPCollisionManager();
  BroadPhaseCollisionManager* manager4 = new IntervalTreeCollisionManager();


  for(unsigned int i = 0; i < env.size(); ++i)
  {
    manager1->registerObject(env[i]);
    manager2->registerObject(env[i]);
    manager3->registerObject(env[i]);
    manager4->registerObject(env[i]);
  }

  manager1->setup();
  manager2->setup();
  manager3->setup();
  manager4->setup();

  CollisionData self_data1;
  self_data1.exhaustive = exhaustive;
  CollisionData self_data2;
  self_data2.exhaustive = exhaustive;
  CollisionData self_data3;
  self_data3.exhaustive = exhaustive;
  CollisionData self_data4;
  self_data4.exhaustive = exhaustive;

  manager1->collide(&self_data1, defaultCollisionFunction);
  manager2->collide(&self_data2, defaultCollisionFunction);
  manager3->collide(&self_data3, defaultCollisionFunction);
  manager3->collide(&self_data4, defaultCollisionFunction);

  bool self_res1 = (self_data1.contacts.size() > 0);
  bool self_res2 = (self_data2.contacts.size() > 0);
  bool self_res3 = (self_data3.contacts.size() > 0);
  bool self_res4 = (self_data4.contacts.size() > 0);

  ASSERT_TRUE(self_res1 == self_res2);
  ASSERT_TRUE(self_res1 == self_res3);
  ASSERT_TRUE(self_res1 == self_res4);


  for(unsigned int i = 0; i < query.size(); ++i)
  {
    CollisionData query_data1;
    query_data1.exhaustive = exhaustive;
    CollisionData query_data2;
    query_data2.exhaustive = exhaustive;
    CollisionData query_data3;
    query_data3.exhaustive = exhaustive;
    CollisionData query_data4;
    query_data4.exhaustive = exhaustive;

    manager1->collide(query[i], &query_data1, defaultCollisionFunction);
    manager2->collide(query[i], &query_data2, defaultCollisionFunction);
    manager3->collide(query[i], &query_data3, defaultCollisionFunction);
    manager4->collide(query[i], &query_data4, defaultCollisionFunction);

    bool query_res1 = (query_data1.contacts.size() > 0);
    bool query_res2 = (query_data2.contacts.size() > 0);
    bool query_res3 = (query_data3.contacts.size() > 0);
    bool query_res4 = (query_data4.contacts.size() > 0);

    ASSERT_TRUE(query_res1 == query_res2);
    ASSERT_TRUE(query_res1 == query_res3);
    ASSERT_TRUE(query_res1 == query_res4);
  }


  for(unsigned int i = 0; i < env.size(); ++i)
    delete env[i];
  for(unsigned int i = 0; i < query.size(); ++i)
    delete query[i];

  delete manager1;
  delete manager2;
  delete manager3;
  delete manager4;
}



TEST(test_core, broad_phase_self_collision_exhaustive)
{
  bool exhaustive = true;

  std::vector<CollisionObject*> env;
  generateEnvironments(env, 100);

  BroadPhaseCollisionManager* manager1 = new NaiveCollisionManager();
  BroadPhaseCollisionManager* manager2 = new SSaPCollisionManager();
  BroadPhaseCollisionManager* manager3 = new SaPCollisionManager();
  BroadPhaseCollisionManager* manager4 = new IntervalTreeCollisionManager();


  for(unsigned int i = 0; i < env.size(); ++i)
  {
    manager1->registerObject(env[i]);
    manager2->registerObject(env[i]);
    manager3->registerObject(env[i]);
    manager4->registerObject(env[i]);
  }

  manager1->setup();
  manager2->setup();
  manager3->setup();
  manager4->setup();

  CollisionData self_data1;
  self_data1.exhaustive = exhaustive;
  CollisionData self_data2;
  self_data2.exhaustive = exhaustive;
  CollisionData self_data3;
  self_data3.exhaustive = exhaustive;
  CollisionData self_data4;
  self_data4.exhaustive = exhaustive;

  manager1->collide(&self_data1, defaultCollisionFunction);
  manager2->collide(&self_data2, defaultCollisionFunction);
  manager3->collide(&self_data3, defaultCollisionFunction);
  manager3->collide(&self_data4, defaultCollisionFunction);

  unsigned int self_res1 = self_data1.contacts.size();
  unsigned int self_res2 = self_data2.contacts.size();
  unsigned int self_res3 = self_data3.contacts.size();
  unsigned int self_res4 = self_data4.contacts.size();

  ASSERT_TRUE(self_res1 == self_res2);
  ASSERT_TRUE(self_res1 == self_res3);
  ASSERT_TRUE(self_res1 == self_res4);


  for(unsigned int i = 0; i < env.size(); ++i)
    delete env[i];

  delete manager1;
  delete manager2;
  delete manager3;
  delete manager4;
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


void generateEnvironments(std::vector<CollisionObject*>& env, int n)
{
  BVH_REAL extents[] = {-200, 200, -200, 200, -200, 200};
  BVH_REAL delta_trans[] = {1, 1, 1};
  std::vector<Transform> transforms;
  std::vector<Transform> transforms2;


  generateRandomTransform(extents, transforms, transforms2, delta_trans, 0.005 * 2 * 3.1415, n);
  Box box(5, 10, 20);
  for(int i = 0; i < n; ++i)
  {
    BVHModel<OBB>* model = new BVHModel<OBB>();
    box.setLocalTransform(transforms[i].R, transforms[i].T);
    generateBVHModel(*model, box, SimpleTransform());
    env.push_back(new CollisionObject(model));
  }

  generateRandomTransform(extents, transforms, transforms2, delta_trans, 0.005 * 2 * 3.1415, n);
  Sphere sphere(30);
  for(int i = 0; i < n; ++i)
  {
    BVHModel<OBB>* model = new BVHModel<OBB>();
    sphere.setLocalTransform(transforms[i].R, transforms[i].T);
    generateBVHModel(*model, sphere, SimpleTransform());
    env.push_back(new CollisionObject(model));
  }

  generateRandomTransform(extents, transforms, transforms2, delta_trans, 0.005 * 2 * 3.1415, n);
  Cylinder cylinder(10, 40);
  for(int i = 0; i < n; ++i)
  {
    BVHModel<OBB>* model = new BVHModel<OBB>();
    cylinder.setLocalTransform(transforms[i].R, transforms[i].T);
    generateBVHModel(*model, cylinder, SimpleTransform());
    env.push_back(new CollisionObject(model));
  }
}

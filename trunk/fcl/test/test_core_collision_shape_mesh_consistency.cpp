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

#include "fcl/geometric_shape_to_BVH_model.h"
#include "fcl/collision.h"
#include "test_core_utility.h"
#include <gtest/gtest.h>

using namespace fcl;

TEST(consistency_shapemesh, spheresphere)
{
  Sphere s1(20);
  Sphere s2(10);
  BVHModel<AABB> s2_aabb;
  BVHModel<OBB> s2_obb;
  BVHModel<RSS> s2_rss;

  generateBVHModel(s2_aabb, s2);
  generateBVHModel(s2_obb, s2);
  generateBVHModel(s2_rss, s2);

  std::vector<Contact> contacts;
  bool res;

  s2.setTranslation(Vec3f(40, 0, 0));
  s2_aabb.setTranslation(Vec3f(40, 0, 0));
  s2_obb.setTranslation(Vec3f(40, 0, 0));
  s2_rss.setTranslation(Vec3f(40, 0, 0));
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s2_aabb, &s1, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s2_obb, &s1, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  //contacts.clear();
  //res = (collide(&s2_rss, &s1, 1, false, false, contacts) > 0);
  //ASSERT_FALSE(res);

  s2.setTranslation(Vec3f(30, 0, 0));
  s2_aabb.setTranslation(Vec3f(30, 0, 0));
  s2_obb.setTranslation(Vec3f(30, 0, 0));
  s2_rss.setTranslation(Vec3f(30, 0, 0));
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s2_aabb, &s1, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s2_obb, &s1, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  //contacts.clear();
  //res = (collide(&s2_rss, &s1, 1, false, false, contacts) > 0);
  //ASSERT_FALSE(res);

  s2.setTranslation(Vec3f(29.9, 0, 0));
  s2_aabb.setTranslation(Vec3f(29.8, 0, 0)); // 29.9 fails, result depends on mesh precision
  s2_obb.setTranslation(Vec3f(29.8, 0, 0)); // 29.9 fails, result depends on mesh precision
  s2_rss.setTranslation(Vec3f(29.8, 0, 0)); // 29.9 fails, result depends on mesh precision
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s2_aabb, &s1, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s2_obb, &s1, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  //contacts.clear();
  //res = (collide(&s1, &s2_rss, 1, false, false, contacts) > 0);
  //ASSERT_TRUE(res);

}

TEST(consistency, spheresphere)
{
  Sphere s1(20);
  Sphere s2(10);
  BVHModel<AABB> s1_aabb;
  BVHModel<AABB> s2_aabb;
  BVHModel<OBB> s1_obb;
  BVHModel<OBB> s2_obb;
  BVHModel<RSS> s1_rss;
  BVHModel<RSS> s2_rss;

  generateBVHModel(s1_aabb, s1);
  generateBVHModel(s2_aabb, s2);
  generateBVHModel(s1_obb, s1);
  generateBVHModel(s2_obb, s2);
  generateBVHModel(s1_rss, s1);
  generateBVHModel(s2_rss, s2);

  std::vector<Contact> contacts;
  bool res;

  s2.setTranslation(Vec3f(40, 0, 0));
  s2_aabb.setTranslation(Vec3f(40, 0, 0));
  s2_obb.setTranslation(Vec3f(40, 0, 0));
  s2_rss.setTranslation(Vec3f(40, 0, 0));
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_aabb, &s2_aabb, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_obb, &s2_obb, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_rss, &s2_rss, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);

  s2.setTranslation(Vec3f(30, 0, 0));
  s2_aabb.setTranslation(Vec3f(30, 0, 0));
  s2_obb.setTranslation(Vec3f(30, 0, 0));
  s2_rss.setTranslation(Vec3f(30, 0, 0));
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_aabb, &s2_aabb, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_obb, &s2_obb, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_rss, &s2_rss, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);

  s2.setTranslation(Vec3f(29.9, 0, 0));
  s2_aabb.setTranslation(Vec3f(29.8, 0, 0)); // 29.9 fails, result depends on mesh precision
  s2_obb.setTranslation(Vec3f(29.8, 0, 0)); // 29.9 fails, result depends on mesh precision
  s2_rss.setTranslation(Vec3f(29.8, 0, 0)); // 29.9 fails, result depends on mesh precision
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1_aabb, &s2_aabb, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1_obb, &s2_obb, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1_rss, &s2_rss, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);

  s2.setTranslation(Vec3f(0, 0, 0));
  s2_aabb.setTranslation(Vec3f(0, 0, 0)); // mesh can not detect collision when ball contains ball
  s2_obb.setTranslation(Vec3f(0, 0, 0)); // mesh can not detect collision when ball contains ball
  s2_rss.setTranslation(Vec3f(0, 0, 0)); // mesh can not detect collision when ball contains ball
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1_aabb, &s2_aabb, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_obb, &s2_obb, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_rss, &s2_rss, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);

  s2.setTranslation(Vec3f(-29.9, 0, 0));
  s2_aabb.setTranslation(Vec3f(-29.8, 0, 0)); // 29.9 fails, result depends on mesh precision
  s2_obb.setTranslation(Vec3f(-29.8, 0, 0)); // 29.9 fails, result depends on mesh precision
  s2_rss.setTranslation(Vec3f(-29.8, 0, 0)); // 29.9 fails, result depends on mesh precision
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1_aabb, &s2_aabb, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1_obb, &s2_obb, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1_rss, &s2_rss, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);

  s2.setTranslation(Vec3f(-30, 0, 0));
  s2_aabb.setTranslation(Vec3f(-30, 0, 0));
  s2_obb.setTranslation(Vec3f(-30, 0, 0));
  s2_rss.setTranslation(Vec3f(-30, 0, 0));
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_aabb, &s2_aabb, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_obb, &s2_obb, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_rss, &s2_rss, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
}

TEST(consistency, boxbox)
{
  Box s1(20, 40, 50);
  Box s2(10, 10, 10);

  BVHModel<AABB> s1_aabb;
  BVHModel<AABB> s2_aabb;
  BVHModel<OBB> s1_obb;
  BVHModel<OBB> s2_obb;
  BVHModel<RSS> s1_rss;
  BVHModel<RSS> s2_rss;

  generateBVHModel(s1_aabb, s1);
  generateBVHModel(s2_aabb, s2);
  generateBVHModel(s1_obb, s1);
  generateBVHModel(s2_obb, s2);
  generateBVHModel(s1_rss, s1);
  generateBVHModel(s2_rss, s2);

  std::vector<Contact> contacts;
  bool res;

  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1_aabb, &s2_aabb, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res); // mesh can not detect collision when box contains box
  contacts.clear();
  res = (collide(&s1_obb, &s2_obb, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res); // mesh can not detect collision when box contains box
  contacts.clear();
  res = (collide(&s1_rss, &s2_rss, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res); // mesh can not detect collision when box contains box

  s2.setTranslation(Vec3f(15.01, 0, 0));
  s2_aabb.setTranslation(Vec3f(15.01, 0, 0));
  s2_obb.setTranslation(Vec3f(15.01, 0, 0));
  s2_rss.setTranslation(Vec3f(15.01, 0, 0));
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_aabb, &s2_aabb, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_obb, &s2_obb, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_rss, &s2_rss, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);

  s2.setTranslation(Vec3f(14.99, 0, 0));
  s2_aabb.setTranslation(Vec3f(14.99, 0, 0));
  s2_obb.setTranslation(Vec3f(14.99, 0, 0));
  s2_rss.setTranslation(Vec3f(14.99, 0, 0));
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1_aabb, &s2_aabb, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1_obb, &s2_obb, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1_rss, &s2_rss, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
}

TEST(consistency, spherebox)
{
  Sphere s1(20);
  Box s2(5, 5, 5);

  BVHModel<AABB> s1_aabb;
  BVHModel<AABB> s2_aabb;
  BVHModel<OBB> s1_obb;
  BVHModel<OBB> s2_obb;
  BVHModel<RSS> s1_rss;
  BVHModel<RSS> s2_rss;

  generateBVHModel(s1_aabb, s1);
  generateBVHModel(s2_aabb, s2);
  generateBVHModel(s1_obb, s1);
  generateBVHModel(s2_obb, s2);
  generateBVHModel(s1_rss, s1);
  generateBVHModel(s2_rss, s2);

  std::vector<Contact> contacts;
  bool res;

  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1_aabb, &s2_aabb, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res); // mesh can not detect collision when box contains box
  contacts.clear();
  res = (collide(&s1_obb, &s2_obb, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res); // mesh can not detect collision when box contains box
  contacts.clear();
  res = (collide(&s1_rss, &s2_rss, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res); // mesh can not detect collision when box contains box

  s2.setTranslation(Vec3f(22.4, 0, 0));
  s2_aabb.setTranslation(Vec3f(22.4, 0, 0));
  s2_obb.setTranslation(Vec3f(22.4, 0, 0));
  s2_rss.setTranslation(Vec3f(22.4, 0, 0));
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1_aabb, &s2_aabb, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1_obb, &s2_obb, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1_rss, &s2_rss, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);

  s2.setTranslation(Vec3f(22.51, 0, 0));
  s2_aabb.setTranslation(Vec3f(22.51, 0, 0));
  s2_obb.setTranslation(Vec3f(22.51, 0, 0));
  s2_rss.setTranslation(Vec3f(22.51, 0, 0));
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_aabb, &s2_aabb, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_obb, &s2_obb, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_rss, &s2_rss, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
}

TEST(consistency, cylindercylinder)
{
  Cylinder s1(5, 10);
  Cylinder s2(5, 10);

  BVHModel<AABB> s1_aabb;
  BVHModel<AABB> s2_aabb;
  BVHModel<OBB> s1_obb;
  BVHModel<OBB> s2_obb;
  BVHModel<RSS> s1_rss;
  BVHModel<RSS> s2_rss;

  generateBVHModel(s1_aabb, s1);
  generateBVHModel(s2_aabb, s2);
  generateBVHModel(s1_obb, s1);
  generateBVHModel(s2_obb, s2);
  generateBVHModel(s1_rss, s1);
  generateBVHModel(s2_rss, s2);

  std::vector<Contact> contacts;
  bool res;

  s2.setTranslation(Vec3f(9.99, 0, 0));
  s2_aabb.setTranslation(Vec3f(9.99, 0, 0));
  s2_obb.setTranslation(Vec3f(9.99, 0, 0));
  s2_rss.setTranslation(Vec3f(9.99, 0, 0));
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1_aabb, &s2_aabb, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1_obb, &s2_obb, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1_rss, &s2_rss, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);

  s2.setTranslation(Vec3f(10.01, 0, 0));
  s2_aabb.setTranslation(Vec3f(10.01, 0, 0));
  s2_obb.setTranslation(Vec3f(10.01, 0, 0));
  s2_rss.setTranslation(Vec3f(10.01, 0, 0));
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_aabb, &s2_aabb, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_obb, &s2_obb, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_rss, &s2_rss, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
}

TEST(consistency, conecone)
{
  Cone s1(5, 10);
  Cone s2(5, 10);

  BVHModel<AABB> s1_aabb;
  BVHModel<AABB> s2_aabb;
  BVHModel<OBB> s1_obb;
  BVHModel<OBB> s2_obb;
  BVHModel<RSS> s1_rss;
  BVHModel<RSS> s2_rss;

  generateBVHModel(s1_aabb, s1);
  generateBVHModel(s2_aabb, s2);
  generateBVHModel(s1_obb, s1);
  generateBVHModel(s2_obb, s2);
  generateBVHModel(s1_rss, s1);
  generateBVHModel(s2_rss, s2);

  std::vector<Contact> contacts;
  bool res;

  s2.setTranslation(Vec3f(9.9, 0, 0));
  s2_aabb.setTranslation(Vec3f(9.9, 0, 0));
  s2_obb.setTranslation(Vec3f(9.9, 0, 0));
  s2_rss.setTranslation(Vec3f(9.9, 0, 0));
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1_aabb, &s2_aabb, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1_obb, &s2_obb, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1_rss, &s2_rss, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);

  s2.setTranslation(Vec3f(10.1, 0, 0));
  s2_aabb.setTranslation(Vec3f(10.1, 0, 0));
  s2_obb.setTranslation(Vec3f(10.1, 0, 0));
  s2_rss.setTranslation(Vec3f(10.1, 0, 0));
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_aabb, &s2_aabb, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_obb, &s2_obb, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_rss, &s2_rss, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);

  s2.setTranslation(Vec3f(0, 0, 9.9));
  s2_aabb.setTranslation(Vec3f(0, 0, 9.9));
  s2_obb.setTranslation(Vec3f(0, 0, 9.9));
  s2_rss.setTranslation(Vec3f(0, 0, 9.9));
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1_aabb, &s2_aabb, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1_obb, &s2_obb, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1_rss, &s2_rss, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);

  s2.setTranslation(Vec3f(0, 0, 10.1));
  s2_aabb.setTranslation(Vec3f(0, 0, 10.1));
  s2_obb.setTranslation(Vec3f(0, 0, 10.1));
  s2_rss.setTranslation(Vec3f(0, 0, 10.1));
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_aabb, &s2_aabb, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_obb, &s2_obb, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1_rss, &s2_rss, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

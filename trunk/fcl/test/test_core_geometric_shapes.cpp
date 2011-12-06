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


#include "fcl/geometric_shapes_intersect.h"
#include "fcl/collision.h"
#include "test_core_utility.h"
#include <gtest/gtest.h>

using namespace fcl;

BVH_REAL extents [6] = {0, 0, 0, 10, 10, 10};

TEST(shapeIntersection, spheresphere)
{
  Sphere s1(20);
  Sphere s2(10);

  Transform transform;
  generateRandomTransform(extents, transform);
  Transform identity;

  std::vector<Contact> contacts;
  bool res;

  s2.setLocalTranslation(Vec3f(40, 0, 0));
  res = shapeIntersect(s1, s2);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);

  s2.setLocalTranslation(Vec3f(30, 0, 0));
  res = shapeIntersect(s1, s2);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);

  s2.setLocalTranslation(Vec3f(29.9, 0, 0));
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);

  s2.setLocalTranslation(Vec3f(0, 0, 0));
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);

  s2.setLocalTranslation(Vec3f(-29.9, 0, 0));
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);

  s2.setLocalTranslation(Vec3f(-30, 0, 0));
  res = shapeIntersect(s1, s2);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);

}

TEST(shapeIntersection, boxbox)
{
  Box s1(20, 40, 50);
  Box s2(10, 10, 10);

  Transform transform;
  generateRandomTransform(extents, transform);
  Transform identity;

  std::vector<Contact> contacts;

  bool res;

  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);

  s2.setLocalTranslation(Vec3f(15, 0, 0));
  res = shapeIntersect(s1, s2);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);


  SimpleQuaternion q;
  q.fromAxisAngle(Vec3f(0, 0, 1), (BVH_REAL)3.140 / 6);
  Matrix3f R;
  q.toRotation(R);
  s2.setLocalRotation(R);
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);

}

TEST(shapeIntersection, spherebox)
{
  Sphere s1(20);
  Box s2(5, 5, 5);

  Transform transform;
  generateRandomTransform(extents, transform);
  Transform identity;

  std::vector<Contact> contacts;

  bool res;

  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);


  s2.setLocalTranslation(Vec3f(22.5, 0, 0));
  res = shapeIntersect(s1, s2);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);

  s2.setLocalTranslation(Vec3f(22.4, 0, 0));
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);

}

TEST(shapeIntersection, cylindercylinder)
{
  Cylinder s1(5, 10);
  Cylinder s2(5, 10);

  Transform transform;
  generateRandomTransform(extents, transform);
  Transform identity;

  std::vector<Contact> contacts;

  bool res;

  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);

  s2.setLocalTranslation(Vec3f(9.9, 0, 0));
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);

  s2.setLocalTranslation(Vec3f(10, 0, 0));
  res = shapeIntersect(s1, s2);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);
}

TEST(shapeIntersection, conecone)
{
  Cone s1(5, 10);
  Cone s2(5, 10);

  Transform transform;
  generateRandomTransform(extents, transform);
  Transform identity;

  std::vector<Contact> contacts;

  bool res;

  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);

  s2.setLocalTranslation(Vec3f(9.9, 0, 0));
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);

  s2.setLocalTranslation(Vec3f(10.001, 0, 0));
  res = shapeIntersect(s1, s2);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);

  s2.setLocalTranslation(Vec3f(0, 0, 9.9));
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);

  s2.setLocalTranslation(Vec3f(0, 0, 10));
  res = shapeIntersect(s1, s2);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);
}

TEST(shapeIntersection, conecylinder)
{
  Cylinder s1(5, 10);
  Cone s2(5, 10);

  Transform transform;
  generateRandomTransform(extents, transform);
  Transform identity;

  std::vector<Contact> contacts;

  bool res;

  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);

  s2.setLocalTranslation(Vec3f(9.9, 0, 0));
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);

  s2.setLocalTranslation(Vec3f(10, 0, 0));
  res = shapeIntersect(s1, s2);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);

  s2.setLocalTranslation(Vec3f(0, 0, 9.9));
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_TRUE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_TRUE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);

  s2.setLocalTranslation(Vec3f(0, 0, 10));
  res = shapeIntersect(s1, s2);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);

  s1.setTransform(transform.R, transform.T);
  s2.setTransform(transform.R, transform.T);
  res = shapeIntersect(s1, s2);
  ASSERT_FALSE(res);
  contacts.clear();
  res = (collide(&s1, &s2, 1, false, false, contacts) > 0);
  ASSERT_FALSE(res);
  s1.setTransform(identity.R, identity.T);
  s2.setTransform(identity.R, identity.T);
}

TEST(shapeIntersection, spheretriangle)
{
  Sphere s(10);
  Vec3f t[3];
  t[0].setValue(20, 0, 0);
  t[1].setValue(-20, 0, 0);
  t[2].setValue(0, 20, 0);

  Transform transform;
  generateRandomTransform(extents, transform);
  Transform identity;

  bool res;

  res = shapeTriangleIntersect(s, t[0], t[1], t[2]);
  ASSERT_TRUE(res);

  s.setTransform(transform.R, transform.T);
  res =  shapeTriangleIntersect(s, t[0], t[1], t[2], transform.R, transform.T);
  ASSERT_TRUE(res);
  s.setTransform(identity.R, identity.T);

  t[0].setValue(30, 0, 0);
  t[1].setValue(10, -20, 0);
  t[2].setValue(10, 20, 0);
  res = shapeTriangleIntersect(s, t[0], t[1], t[2]);
  ASSERT_FALSE(res);

  s.setTransform(transform.R, transform.T);
  res =  shapeTriangleIntersect(s, t[0], t[1], t[2], transform.R, transform.T);
  ASSERT_FALSE(res);
  s.setTransform(identity.R, identity.T);

  t[0].setValue(30, 0, 0);
  t[1].setValue(9.9, -20, 0);
  t[2].setValue(9.9, 20, 0);
  res = shapeTriangleIntersect(s, t[0], t[1], t[2]);
  ASSERT_TRUE(res);

  s.setTransform(transform.R, transform.T);
  res =  shapeTriangleIntersect(s, t[0], t[1], t[2], transform.R, transform.T);
  ASSERT_TRUE(res);
  s.setTransform(identity.R, identity.T);
}




int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

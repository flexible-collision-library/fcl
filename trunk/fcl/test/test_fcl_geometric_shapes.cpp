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

#include "fcl/narrowphase/narrowphase.h"
#include "fcl/collision.h"
#include "test_fcl_utility.h"
#include <gtest/gtest.h>
#include <iostream>

using namespace fcl;

FCL_REAL extents [6] = {0, 0, 0, 10, 10, 10};

GJKSolver_libccd solver1;
GJKSolver_indep solver2;

TEST(shapeIntersection, spheresphere)
{
  Sphere s1(20);
  Sphere s2(10);

  Transform3f transform;
  generateRandomTransform(extents, transform);
  Transform3f identity;

  CollisionRequest request;
  CollisionResult result;
  bool res;

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(40, 0, 0)), NULL, NULL, NULL);
  ASSERT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(Vec3f(40, 0, 0)), &solver1, request, result) > 0);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(40, 0, 0)), NULL, NULL, NULL);
  ASSERT_FALSE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(Vec3f(40, 0, 0)), &solver1, request, result) > 0);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(30, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(Vec3f(30, 0, 0)), &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(30.01, 0, 0)), NULL, NULL, NULL);
  ASSERT_FALSE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(Vec3f(30.01, 0, 0)), &solver1, request, result) > 0);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(29.9, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(Vec3f(29.9, 0, 0)), &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(29.9, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(Vec3f(29.9, 0, 0)), &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(), &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform, NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform, &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(-29.9, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(Vec3f(-29.9, 0, 0)), &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(-29.9, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(Vec3f(-29.9, 0, 0)), &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(-30, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(Vec3f(-30, 0, 0)), &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(-30.01, 0, 0)), NULL, NULL, NULL);
  ASSERT_FALSE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(Vec3f(-30.01, 0, 0)), &solver1, request, result) > 0);
  ASSERT_FALSE(res);
}

TEST(shapeIntersection, boxbox)
{
  Box s1(20, 40, 50);
  Box s2(10, 10, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);
  Transform3f identity;

  CollisionRequest request;
  CollisionResult result;

  bool res;

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(), &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform, NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform, &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(15, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(Vec3f(15, 0, 0)), &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(15.01, 0, 0)), NULL, NULL, NULL);
  ASSERT_FALSE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(Vec3f(15.01, 0, 0)), &solver1, request, result) > 0);
  ASSERT_FALSE(res);

  Quaternion3f q;
  q.fromAxisAngle(Vec3f(0, 0, 1), (FCL_REAL)3.140 / 6);
  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(q), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(q), &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(q), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(q), &solver1, request, result) > 0);
  ASSERT_TRUE(res);
}

TEST(shapeIntersection, spherebox)
{
  Sphere s1(20);
  Box s2(5, 5, 5);

  Transform3f transform;
  generateRandomTransform(extents, transform);
  Transform3f identity;

  CollisionRequest request;
  CollisionResult result;

  bool res;

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(), &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform, NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform, &solver1, request, result) > 0);
  ASSERT_TRUE(res);


  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(22.5, 0, 0)), NULL, NULL, NULL);
  ASSERT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(Vec3f(22.5, 0, 0)), &solver1, request, result) > 0);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(22.501, 0, 0)), NULL, NULL, NULL);
  ASSERT_FALSE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(Vec3f(22.501, 0, 0)), &solver1, request, result) > 0);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(22.4, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(Vec3f(22.4, 0, 0)), &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(22.4, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(Vec3f(22.4, 0, 0)), &solver1, request, result) > 0);
  ASSERT_TRUE(res);
}

TEST(shapeIntersection, cylindercylinder)
{
  Cylinder s1(5, 10);
  Cylinder s2(5, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);
  Transform3f identity;

  CollisionRequest request;
  CollisionResult result;

  bool res;

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(), &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform, NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform, &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(9.9, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(Vec3f(9.9, 0, 0)), &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(9.9, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(Vec3f(9.9, 0, 0)), &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(10, 0, 0)), NULL, NULL, NULL);
  ASSERT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(Vec3f(10, 0, 0)), &solver1, request, result) > 0);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(10.01, 0, 0)), NULL, NULL, NULL);
  ASSERT_FALSE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(Vec3f(10.01, 0, 0)), &solver1, request, result) > 0);
  ASSERT_FALSE(res);
}

TEST(shapeIntersection, conecone)
{
  Cone s1(5, 10);
  Cone s2(5, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);
  Transform3f identity;

  CollisionRequest request;
  CollisionResult result;

  bool res;

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(), &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform, NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform, &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(9.9, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(Vec3f(9.9, 0, 0)), &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(9.9, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(Vec3f(9.9, 0, 0)), &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(10.001, 0, 0)), NULL, NULL, NULL);
  ASSERT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(Vec3f(10.001, 0, 0)), &solver1, request, result) > 0);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(10.001, 0, 0)), NULL, NULL, NULL);
  ASSERT_FALSE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(Vec3f(10.001, 0, 0)), &solver1, request, result) > 0);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(0, 0, 9.9)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(Vec3f(0, 0, 9.9)), &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(0, 0, 9.9)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(Vec3f(0, 0, 9.9)), &solver1, request, result) > 0);
  ASSERT_TRUE(res);
}

TEST(shapeIntersection, conecylinder)
{
  Cylinder s1(5, 10);
  Cone s2(5, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);
  Transform3f identity;

  CollisionRequest request;
  CollisionResult result;

  bool res;

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(), &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform, NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform, &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(9.9, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(Vec3f(9.9, 0, 0)), &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(9.9, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(Vec3f(9.9, 0, 0)), &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(10, 0, 0)), NULL, NULL, NULL);
  ASSERT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(Vec3f(10, 0, 0)), &solver1, request, result) > 0);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(10, 0, 0)), NULL, NULL, NULL);
  ASSERT_FALSE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(Vec3f(10, 0, 0)), &solver1, request, result) > 0);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(0, 0, 9.9)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(Vec3f(0, 0, 9.9)), &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(0, 0, 9.9)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(Vec3f(0, 0, 9.9)), &solver1, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(0, 0, 10)), NULL, NULL, NULL);
  ASSERT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(Vec3f(0, 0, 10)), &solver1, request, result) > 0);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(0, 0, 10.01)), NULL, NULL, NULL);
  ASSERT_FALSE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(Vec3f(0, 0, 10.01)), &solver1, request, result) > 0);
  ASSERT_FALSE(res);
}

TEST(shapeIntersection, spheretriangle)
{
  Sphere s(10);
  Vec3f t[3];
  t[0].setValue(20, 0, 0);
  t[1].setValue(-20, 0, 0);
  t[2].setValue(0, 20, 0);

  Transform3f transform;
  generateRandomTransform(extents, transform);
  Transform3f identity;

  bool res;

  res = solver1.shapeTriangleIntersect(s, Transform3f(), t[0], t[1], t[2], NULL, NULL, NULL);
  ASSERT_TRUE(res);

  res =  solver1.shapeTriangleIntersect(s, transform, t[0], t[1], t[2], transform, NULL, NULL, NULL);
  ASSERT_TRUE(res);


  t[0].setValue(30, 0, 0);
  t[1].setValue(9.9, -20, 0);
  t[2].setValue(9.9, 20, 0);
  res = solver1.shapeTriangleIntersect(s, Transform3f(), t[0], t[1], t[2], NULL, NULL, NULL);
  ASSERT_TRUE(res);

  res =  solver1.shapeTriangleIntersect(s, transform, t[0], t[1], t[2], transform, NULL, NULL, NULL);
  ASSERT_TRUE(res);
}

TEST(shapeIntersection, halfspacesphere)
{
  Sphere s(10);
  Halfspace hs(Vec3f(1, 0, 0), 0);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;
  bool res;

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 10) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(-5, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 10) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(-5, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 15) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(-2.5, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 15) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(-2.5, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(-5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(-7.5, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(-5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(-7.5, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(-10.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(-10.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(10.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 20.1) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0.05, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(10.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 20.1) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0.05, 0, 0))));
}

TEST(shapeIntersection, planesphere)
{
  Sphere s(10);
  Plane hs(Vec3f(1, 0, 0), 0);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;
  bool res;

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 10) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)) || normal.equal(Vec3f(1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 10) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))) || normal.equal(transform.getQuatRotation().transform(Vec3f(1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(5, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(5, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(-5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(-5, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(-5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(-5, 0, 0))));


  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(-10.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(-10.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(10.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(10.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);
}

TEST(shapeIntersection, halfspacebox)
{
  Box s(5, 10, 20);
  Halfspace hs(Vec3f(1, 0, 0), 0);
  
  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;
  bool res;

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(-1.25, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(-1.25, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(1.25, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 3.75) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(-0.625, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(1.25, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 3.75) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(-0.625, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(-1.25, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 1.25) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(-1.875, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(-1.25, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 1.25) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(-1.875, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(2.51, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5.01) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0.005, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(2.51, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5.01) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0.005, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(-2.51, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(-2.51, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(transform.getQuatRotation()), hs, Transform3f(), &contact, &depth, &normal);
  ASSERT_TRUE(res);
}

TEST(shapeIntersection, planebox)
{
  Box s(5, 10, 20);
  Plane hs(Vec3f(1, 0, 0), 0);
  
  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;
  bool res;

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)) || normal.equal(Vec3f(1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))) || normal.equal(transform.getQuatRotation().transform(Vec3f(1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(1.25, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 1.25) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(1.25, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(1.25, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 1.25) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(1.25, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(-1.25, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 1.25) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(-1.25, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(-1.25, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 1.25) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(-1.25, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(2.51, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(2.51, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(-2.51, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(-2.51, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(transform.getQuatRotation()), hs, Transform3f(), &contact, &depth, &normal);
  ASSERT_TRUE(res);
}

TEST(shapeIntersection, halfspacecapsule)
{
  Capsule s(5, 10);
  Halfspace hs(Vec3f(1, 0, 0), 0);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;
  bool res;

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(-2.5, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(-2.5, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(2.5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 7.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(-1.25, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(2.5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 7.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(-1.25, 0, 0))));
  
  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(-2.5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(-3.75, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(-2.5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(-3.75, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(5.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 10.1) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0.05, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(5.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 10.1) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0.05, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(-5.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(-5.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);




  hs = Halfspace(Vec3f(0, 1, 0), 0);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, -1, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, -2.5, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, -1, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, -2.5, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 2.5, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 7.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, -1, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, -1.25, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 2.5, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 7.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, -1, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, -1.25, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, -2.5, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, -1, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, -3.75, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, -2.5, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, -1, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, -3.75, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 5.1, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 10.1) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, -1, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0.05, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 5.1, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 10.1) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, -1, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0.05, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, -5.1, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, -5.1, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);




  hs = Halfspace(Vec3f(0, 0, 1), 0);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 10) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, 0, -1)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, -5)));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 10) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, 0, -1))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, -5))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 0, 2.5)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 12.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, 0, -1)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, -3.75)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 0, 2.5)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 12.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, 0, -1))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, -3.75))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 0, -2.5)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 7.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, 0, -1)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, -6.25)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 0, -2.5)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 7.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, 0, -1))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, -6.25))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 0, 10.1)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 20.1) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, 0, -1)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, 0.05)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 0, 10.1)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 20.1) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, 0, -1))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, 0.05))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 0, -10.1)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 0, -10.1)), &contact, &depth, &normal);
  ASSERT_FALSE(res);
}

TEST(shapeIntersection, planecapsule)
{
  Capsule s(5, 10);
  Plane hs(Vec3f(1, 0, 0), 0);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;
  bool res;

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)) || normal.equal(Vec3f(1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))) || normal.equal(transform.getQuatRotation().transform(Vec3f(1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(2.5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(2.5, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(2.5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(2.5, 0, 0))));
  
  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(-2.5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(-2.5, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(-2.5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(-2.5, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(5.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(5.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(-5.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(-5.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);




  hs = Plane(Vec3f(0, 1, 0), 0);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, -1, 0)) || normal.equal(Vec3f(0, 1, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, -1, 0))) || normal.equal(transform.getQuatRotation().transform(Vec3f(0, 1, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 2.5, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, 1, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 2.5, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 2.5, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, 1, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 2.5, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, -2.5, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, -1, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, -2.5, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, -2.5, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, -1, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, -2.5, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 5.1, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 5.1, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, -5.1, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, -5.1, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);




  hs = Plane(Vec3f(0, 0, 1), 0);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 10) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, 0, -1)) || normal.equal(Vec3f(0, 0, 1)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 10) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, 0, -1))) || normal.equal(transform.getQuatRotation().transform(Vec3f(0, 0, 1))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 0, 2.5)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 7.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, 0, 1)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, 2.5)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 0, 2.5)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 7.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, 0, 1))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, 2.5))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 0, -2.5)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 7.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, 0, -1)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, -2.5)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 0, -2.5)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 7.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, 0, -1))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, -2.5))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 0, 10.1)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 0, 10.1)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 0, -10.1)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 0, -10.1)), &contact, &depth, &normal);
  ASSERT_FALSE(res);
}

TEST(shapeIntersection, halfspacecylinder)
{
  Cylinder s(5, 10);
  Halfspace hs(Vec3f(1, 0, 0), 0);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;
  bool res;
  
  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(-2.5, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(-2.5, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(2.5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 7.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(-1.25, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(2.5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 7.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(-1.25, 0, 0))));
  
  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(-2.5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(-3.75, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(-2.5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(-3.75, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(5.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 10.1) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0.05, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(5.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 10.1) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0.05, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(-5.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(-5.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);




  hs = Halfspace(Vec3f(0, 1, 0), 0);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, -1, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, -2.5, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, -1, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, -2.5, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 2.5, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 7.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, -1, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, -1.25, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 2.5, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 7.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, -1, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, -1.25, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, -2.5, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, -1, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, -3.75, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, -2.5, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, -1, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, -3.75, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 5.1, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 10.1) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, -1, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0.05, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 5.1, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 10.1) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, -1, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0.05, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, -5.1, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, -5.1, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);




  hs = Halfspace(Vec3f(0, 0, 1), 0);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, 0, -1)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, -2.5)));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, 0, -1))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, -2.5))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 0, 2.5)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 7.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, 0, -1)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, -1.25)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 0, 2.5)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 7.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, 0, -1))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, -1.25))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 0, -2.5)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, 0, -1)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, -3.75)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 0, -2.5)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, 0, -1))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, -3.75))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 0, 5.1)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 10.1) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, 0, -1)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, 0.05)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 0, 5.1)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 10.1) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, 0, -1))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, 0.05))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 0, -5.1)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 0, -5.1)), &contact, &depth, &normal);
  ASSERT_FALSE(res);
}

TEST(shapeIntersection, planecylinder)
{
  Cylinder s(5, 10);
  Plane hs(Vec3f(1, 0, 0), 0);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;
  bool res;

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)) || normal.equal(Vec3f(1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))) || normal.equal(transform.getQuatRotation().transform(Vec3f(1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(2.5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(2.5, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(2.5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(2.5, 0, 0))));
  
  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(-2.5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(-2.5, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(-2.5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(-2.5, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(5.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(5.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(-5.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(-5.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);




  hs = Plane(Vec3f(0, 1, 0), 0);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, -1, 0)) || normal.equal(Vec3f(0, 1, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, -1, 0))) || normal.equal(transform.getQuatRotation().transform(Vec3f(0, 1, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 2.5, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, 1, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 2.5, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 2.5, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, 1, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 2.5, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, -2.5, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, -1, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, -2.5, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, -2.5, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, -1, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, -2.5, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 5.1, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 5.1, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, -5.1, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, -5.1, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);




  hs = Plane(Vec3f(0, 0, 1), 0);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, 0, -1)) || normal.equal(Vec3f(0, 0, 1)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, 0, -1))) || normal.equal(transform.getQuatRotation().transform(Vec3f(0, 0, 1))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 0, 2.5)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, 0, 1)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, 2.5)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 0, 2.5)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, 0, 1))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, 2.5))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 0, -2.5)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, 0, -1)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, -2.5)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 0, -2.5)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, 0, -1))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, -2.5))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 0, 10.1)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 0, 10.1)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 0, -10.1)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 0, -10.1)), &contact, &depth, &normal);
  ASSERT_FALSE(res);
}


TEST(shapeIntersection, halfspacecone)
{
  Cone s(5, 10);
  Halfspace hs(Vec3f(1, 0, 0), 0);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;
  bool res;
  
  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(-2.5, 0, -5)));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(-2.5, 0, -5))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(2.5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 7.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(-1.25, 0, -5)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(2.5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 7.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(-1.25, 0, -5))));
  
  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(-2.5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(-3.75, 0, -5)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(-2.5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(-3.75, 0, -5))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(5.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 10.1) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0.05, 0, -5)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(5.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 10.1) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0.05, 0, -5))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(-5.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(-5.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);




  hs = Halfspace(Vec3f(0, 1, 0), 0);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, -1, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, -2.5, -5)));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, -1, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, -2.5, -5))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 2.5, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 7.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, -1, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, -1.25, -5)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 2.5, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 7.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, -1, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, -1.25, -5))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, -2.5, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, -1, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, -3.75, -5)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, -2.5, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, -1, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, -3.75, -5))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 5.1, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 10.1) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, -1, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0.05, -5)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 5.1, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 10.1) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, -1, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0.05, -5))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, -5.1, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, -5.1, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);




  hs = Halfspace(Vec3f(0, 0, 1), 0);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, 0, -1)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, -2.5)));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, 0, -1))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, -2.5))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 0, 2.5)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 7.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, 0, -1)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, -1.25)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 0, 2.5)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 7.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, 0, -1))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, -1.25))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 0, -2.5)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, 0, -1)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, -3.75)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 0, -2.5)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, 0, -1))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, -3.75))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 0, 5.1)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 10.1) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, 0, -1)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, 0.05)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 0, 5.1)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 10.1) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, 0, -1))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, 0.05))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 0, -5.1)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 0, -5.1)), &contact, &depth, &normal);
  ASSERT_FALSE(res);
}

TEST(shapeIntersection, planecone)
{
  Cone s(5, 10);
  Plane hs(Vec3f(1, 0, 0), 0);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;
  bool res;

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)) || normal.equal(Vec3f(1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))) || normal.equal(transform.getQuatRotation().transform(Vec3f(1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(2.5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(2.5, 0, -2.5)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(2.5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(2.5, 0, -2.5))));
  
  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(-2.5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(-1, 0, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(-2.5, 0, -2.5)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(-2.5, 0, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(-1, 0, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(-2.5, 0, -2.5))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(5.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(5.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(-5.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(-5.1, 0, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);




  hs = Plane(Vec3f(0, 1, 0), 0);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, -1, 0)) || normal.equal(Vec3f(0, 1, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, -1, 0))) || normal.equal(transform.getQuatRotation().transform(Vec3f(0, 1, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 2.5, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, 1, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 2.5, -2.5)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 2.5, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, 1, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 2.5, -2.5))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, -2.5, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, -1, 0)));
  ASSERT_TRUE(contact.equal(Vec3f(0, -2.5, -2.5)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, -2.5, 0)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, -1, 0))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, -2.5, -2.5))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 5.1, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 5.1, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, -5.1, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, -5.1, 0)), &contact, &depth, &normal);
  ASSERT_FALSE(res);




  hs = Plane(Vec3f(0, 0, 1), 0);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, 0, -1)) || normal.equal(Vec3f(0, 0, 1)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, 0)));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, 0, -1))) || normal.equal(transform.getQuatRotation().transform(Vec3f(0, 0, 1))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, 0))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 0, 2.5)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, 0, 1)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, 2.5)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 0, 2.5)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, 0, 1))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, 2.5))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 0, -2.5)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(Vec3f(0, 0, -1)));
  ASSERT_TRUE(contact.equal(Vec3f(0, 0, -2.5)));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 0, -2.5)), &contact, &depth, &normal);
  ASSERT_TRUE(res);
  ASSERT_TRUE(std::abs(depth - 2.5) < 0.001);
  ASSERT_TRUE(normal.equal(transform.getQuatRotation().transform(Vec3f(0, 0, -1))));
  ASSERT_TRUE(contact.equal(transform.transform(Vec3f(0, 0, -2.5))));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 0, 10.1)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 0, 10.1)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(Vec3f(0, 0, -10.1)), &contact, &depth, &normal);
  ASSERT_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(Vec3f(0, 0, -10.1)), &contact, &depth, &normal);
  ASSERT_FALSE(res);
}



TEST(shapeDistance, spheresphere)
{  
  Sphere s1(20);
  Sphere s2(10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist = -1;
  
  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(40, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 10) < 0.001);
  ASSERT_TRUE(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(30.1, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.001);
  ASSERT_TRUE(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(29.9, 0, 0)), &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);

  res = solver1.shapeDistance(s1, Transform3f(Vec3f(40, 0, 0)), s2, Transform3f(), &dist);
  ASSERT_TRUE(fabs(dist - 10) < 0.001);
  ASSERT_TRUE(res);

  res = solver1.shapeDistance(s1, Transform3f(Vec3f(30.1, 0, 0)), s2, Transform3f(), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.001);
  ASSERT_TRUE(res);

  res = solver1.shapeDistance(s1, Transform3f(Vec3f(29.9, 0, 0)), s2, Transform3f(), &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);


  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(40, 0, 0)), &dist);
  // this is one problem: the precise is low sometimes
  ASSERT_TRUE(fabs(dist - 10) < 0.1);
  ASSERT_TRUE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(30.1, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.06);
  ASSERT_TRUE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(29.9, 0, 0)), &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);

  res = solver1.shapeDistance(s1, transform * Transform3f(Vec3f(40, 0, 0)), s2, transform, &dist);
  ASSERT_TRUE(fabs(dist - 10) < 0.1);
  ASSERT_TRUE(res);

  res = solver1.shapeDistance(s1, transform * Transform3f(Vec3f(30.1, 0, 0)), s2, transform, &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.1);
  ASSERT_TRUE(res);

  res = solver1.shapeDistance(s1, transform * Transform3f(Vec3f(29.9, 0, 0)), s2, transform, &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);
}

TEST(shapeDistance, boxbox)
{                      
  Box s1(20, 40, 50);
  Box s2(10, 10, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(), &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform, &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(15.1, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.001);
  ASSERT_TRUE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(15.1, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.001);
  ASSERT_TRUE(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(20, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 5) < 0.001);
  ASSERT_TRUE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(20, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 5) < 0.001);
  ASSERT_TRUE(res);
}

TEST(shapeDistance, boxsphere)
{
  Sphere s1(20);
  Box s2(5, 5, 5);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(), &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform, &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(22.6, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.001);
  ASSERT_TRUE(res);
  
  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(22.6, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.05);
  ASSERT_TRUE(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(40, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 17.5) < 0.001);
  ASSERT_TRUE(res);
  
  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(40, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 17.5) < 0.001);
  ASSERT_TRUE(res);
}

TEST(shapeDistance, cylindercylinder)
{
  Cylinder s1(5, 10);
  Cylinder s2(5, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(), &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform, &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(10.1, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.001);
  ASSERT_TRUE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(10.1, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.001);
  ASSERT_TRUE(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(40, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 30) < 0.001);
  ASSERT_TRUE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(40, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 30) < 0.001);
  ASSERT_TRUE(res);
}



TEST(shapeDistance, conecone)
{
  Cone s1(5, 10);
  Cone s2(5, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(), &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform, &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(10.1, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.001);
  ASSERT_TRUE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(10.1, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.001);
  ASSERT_TRUE(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(0, 0, 40)), &dist);
  ASSERT_TRUE(fabs(dist - 30) < 1);
  ASSERT_TRUE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(0, 0, 40)), &dist);
  ASSERT_TRUE(fabs(dist - 30) < 1);
  ASSERT_TRUE(res);
}

TEST(shapeDistance, conecylinder)
{
  Cylinder s1(5, 10);
  Cone s2(5, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(), &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform, &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(10.1, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.001);
  ASSERT_TRUE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(10.1, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.001);
  ASSERT_TRUE(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(40, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 30) < 0.001);
  ASSERT_TRUE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(40, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 30) < 0.1);
  ASSERT_TRUE(res);
}



TEST(shapeIntersectionGJK, spheresphere)
{
  Sphere s1(20);
  Sphere s2(10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  CollisionRequest request;
  CollisionResult result;

  Vec3f contact;
  FCL_REAL penetration_depth;
  Vec3f normal;  
  bool res;

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(40, 0, 0)), NULL, NULL, NULL);
  ASSERT_FALSE(res); 
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(40, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(Vec3f(40, 0, 0)), &solver2, request, result) > 0);
  ASSERT_FALSE(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(40, 0, 0)), NULL, NULL, NULL);
  ASSERT_FALSE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(40, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_FALSE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(Vec3f(40, 0, 0)), &solver2, request, result) > 0);
  ASSERT_FALSE(res);


  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(30, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(30, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(Vec3f(30, 0, 0)), &solver2, request, result) > 0);
  ASSERT_TRUE(res);


  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(30.01, 0, 0)), NULL, NULL, NULL);
  ASSERT_FALSE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(30.01, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_FALSE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(Vec3f(30.01, 0, 0)), &solver2, request, result) > 0);
  ASSERT_FALSE(res);


  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(29.9, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(29.9, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(Vec3f(29.9, 0, 0)), &solver2, request, result) > 0);
  ASSERT_TRUE(res);


  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(29.9, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(29.9, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(Vec3f(29.9, 0, 0)), &solver2, request, result) > 0);
  ASSERT_TRUE(res);


  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(), &solver2, request, result) > 0);
  ASSERT_TRUE(res);


  res = solver2.shapeIntersect(s1, transform, s2, transform, NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform, &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform, &solver2, request, result) > 0);
  ASSERT_TRUE(res);


  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(-29.9, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(-29.9, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(Vec3f(-29.9, 0, 0)), &solver2, request, result) > 0);
  ASSERT_TRUE(res);



  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(-29.9, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(-29.9, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(Vec3f(-29.9, 0, 0)), &solver2, request, result) > 0);
  ASSERT_TRUE(res);


  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(-30, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(-30, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(Vec3f(-30, 0, 0)), &solver2, request, result) > 0);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(-30.01, 0, 0)), NULL, NULL, NULL);
  ASSERT_FALSE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(-30.01, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_FALSE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(Vec3f(-30.01, 0, 0)), &solver2, request, result) > 0);
  ASSERT_FALSE(res);
}

TEST(shapeIntersectionGJK, boxbox)
{
  Box s1(20, 40, 50);
  Box s2(10, 10, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL penetration_depth;
  Vec3f normal;  
  bool res;

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform, NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform, &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(15, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(15, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(15.01, 0, 0)), NULL, NULL, NULL);
  ASSERT_FALSE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(15.01, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_FALSE(res);

  Quaternion3f q;
  q.fromAxisAngle(Vec3f(0, 0, 1), (FCL_REAL)3.140 / 6);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(q), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(q), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);
  
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(q), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(q), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);
}

TEST(shapeIntersectionGJK, spherebox)
{
  Sphere s1(20);
  Box s2(5, 5, 5);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL penetration_depth;
  Vec3f normal;  
  bool res;

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform, NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform, &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(22.5, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(22.5, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(22.51, 0, 0)), NULL, NULL, NULL);
  ASSERT_FALSE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(22.51, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_FALSE(res);

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(22.4, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(22.4, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(22.4, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(22.4, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);
}

TEST(shapeIntersectionGJK, cylindercylinder)
{
  Cylinder s1(5, 10);
  Cylinder s2(5, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL penetration_depth;
  Vec3f normal;  
  bool res;

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform, NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform, &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(9.9, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(9.9, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(9.9, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(9.9, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(10, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(10, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(10.1, 0, 0)), NULL, NULL, NULL);
  ASSERT_FALSE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(10.1, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_FALSE(res);
}

TEST(shapeIntersectionGJK, conecone)
{
  Cone s1(5, 10);
  Cone s2(5, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL penetration_depth;
  Vec3f normal;  
  bool res;

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform, NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform, &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(9.9, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(9.9, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(9.9, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(9.9, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(10.1, 0, 0)), NULL, NULL, NULL);
  ASSERT_FALSE(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(10.1, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_FALSE(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(10.1, 0, 0)), NULL, NULL, NULL);
  ASSERT_FALSE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(10.1, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_FALSE(res);

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(0, 0, 9.9)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(0, 0, 9.9)), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(0, 0, 9.9)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(0, 0, 9.9)), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);
}

TEST(shapeIntersectionGJK, conecylinder)
{
  Cylinder s1(5, 10);
  Cone s2(5, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL penetration_depth;
  Vec3f normal;  
  bool res;

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform, NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform, &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(9.9, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(9.9, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(9.9, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(9.9, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(10, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(10, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(10, 0, 0)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(10, 0, 0)), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(0, 0, 9.9)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(0, 0, 9.9)), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(0, 0, 9.9)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(0, 0, 9.9)), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(0, 0, 10)), NULL, NULL, NULL);
  ASSERT_TRUE(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(Vec3f(0, 0, 10)), &contact, &penetration_depth, &normal);
  ASSERT_TRUE(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(0, 0, 10.1)), NULL, NULL, NULL);
  ASSERT_FALSE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(Vec3f(0, 0, 10.1)), &contact, &penetration_depth, &normal);
  ASSERT_FALSE(res);
}


TEST(shapeIntersectionGJK, spheretriangle)
{
  Sphere s(10);
  Vec3f t[3];
  t[0].setValue(20, 0, 0);
  t[1].setValue(-20, 0, 0);
  t[2].setValue(0, 20, 0);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;

  res = solver2.shapeTriangleIntersect(s, Transform3f(), t[0], t[1], t[2], NULL, NULL, NULL);
  ASSERT_TRUE(res);

  res =  solver2.shapeTriangleIntersect(s, transform, t[0], t[1], t[2], transform, NULL, NULL, NULL);
  ASSERT_TRUE(res);

  t[0].setValue(30, 0, 0);
  t[1].setValue(9.9, -20, 0);
  t[2].setValue(9.9, 20, 0);
  res = solver2.shapeTriangleIntersect(s, Transform3f(), t[0], t[1], t[2], NULL, NULL, NULL);
  ASSERT_TRUE(res);

  res =  solver2.shapeTriangleIntersect(s, transform, t[0], t[1], t[2], transform, NULL, NULL, NULL);
  ASSERT_TRUE(res);
}




TEST(shapeDistanceGJK, spheresphere)
{  
  Sphere s1(20);
  Sphere s2(10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist = -1;
  
  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(40, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 10) < 0.001);
  ASSERT_TRUE(res);

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(30.1, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.001);
  ASSERT_TRUE(res);

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(29.9, 0, 0)), &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);

  res = solver2.shapeDistance(s1, Transform3f(Vec3f(40, 0, 0)), s2, Transform3f(), &dist);
  ASSERT_TRUE(fabs(dist - 10) < 0.001);
  ASSERT_TRUE(res);

  res = solver2.shapeDistance(s1, Transform3f(Vec3f(30.1, 0, 0)), s2, Transform3f(), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.001);
  ASSERT_TRUE(res);

  res = solver2.shapeDistance(s1, Transform3f(Vec3f(29.9, 0, 0)), s2, Transform3f(), &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);


  res = solver2.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(40, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 10) < 0.001);
  ASSERT_TRUE(res);

  res = solver2.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(30.1, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.001);
  ASSERT_TRUE(res);

  res = solver2.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(29.9, 0, 0)), &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);

  res = solver2.shapeDistance(s1, transform * Transform3f(Vec3f(40, 0, 0)), s2, transform, &dist);
  ASSERT_TRUE(fabs(dist - 10) < 0.001);
  ASSERT_TRUE(res);

  res = solver2.shapeDistance(s1, transform * Transform3f(Vec3f(30.1, 0, 0)), s2, transform, &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.001);
  ASSERT_TRUE(res);

  res = solver2.shapeDistance(s1, transform * Transform3f(Vec3f(29.9, 0, 0)), s2, transform, &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);
}

TEST(shapeDistanceGJK, boxbox)
{                      
  Box s1(20, 40, 50);
  Box s2(10, 10, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(), &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);

  res = solver2.shapeDistance(s1, transform, s2, transform, &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(15.1, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.001);
  ASSERT_TRUE(res);

  res = solver2.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(15.1, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.001);
  ASSERT_TRUE(res);

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(20, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 5) < 0.001);
  ASSERT_TRUE(res);

  res = solver2.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(20, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 5) < 0.001);
  ASSERT_TRUE(res);
}

TEST(shapeDistanceGJK, boxsphere)
{
  Sphere s1(20);
  Box s2(5, 5, 5);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(), &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);

  res = solver2.shapeDistance(s1, transform, s2, transform, &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(22.6, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.01);
  ASSERT_TRUE(res);
  
  res = solver2.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(22.6, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.01);
  ASSERT_TRUE(res);

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(40, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 17.5) < 0.001);
  ASSERT_TRUE(res);
  
  res = solver2.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(40, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 17.5) < 0.001);
  ASSERT_TRUE(res);
}

TEST(shapeDistanceGJK, cylindercylinder)
{
  Cylinder s1(5, 10);
  Cylinder s2(5, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(), &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);

  res = solver2.shapeDistance(s1, transform, s2, transform, &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(10.1, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.001);
  ASSERT_TRUE(res);

  res = solver2.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(10.1, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.001);
  ASSERT_TRUE(res);

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(40, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 30) < 0.001);
  ASSERT_TRUE(res);

  res = solver2.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(40, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 30) < 0.001);
  ASSERT_TRUE(res);
}



TEST(shapeDistanceGJK, conecone)
{
  Cone s1(5, 10);
  Cone s2(5, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(), &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);

  res = solver2.shapeDistance(s1, transform, s2, transform, &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(10.1, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.001);
  ASSERT_TRUE(res);

  res = solver2.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(10.1, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.001);
  ASSERT_TRUE(res);

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(0, 0, 40)), &dist);
  ASSERT_TRUE(fabs(dist - 30) < 0.001);
  ASSERT_TRUE(res);

  res = solver2.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(0, 0, 40)), &dist);
  ASSERT_TRUE(fabs(dist - 30) < 0.001);
  ASSERT_TRUE(res);
}

TEST(shapeDistanceGJK, conecylinder)
{
  Cylinder s1(5, 10);
  Cone s2(5, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(), &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);

  res = solver2.shapeDistance(s1, transform, s2, transform, &dist);
  ASSERT_TRUE(dist < 0);
  ASSERT_FALSE(res);

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(10.1, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.001);
  ASSERT_TRUE(res);

  res = solver2.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(10.1, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 0.1) < 0.001);
  ASSERT_TRUE(res);

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(Vec3f(40, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 30) < 0.001);
  ASSERT_TRUE(res);

  res = solver2.shapeDistance(s1, transform, s2, transform * Transform3f(Vec3f(40, 0, 0)), &dist);
  ASSERT_TRUE(fabs(dist - 30) < 0.001);
  ASSERT_TRUE(res);
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


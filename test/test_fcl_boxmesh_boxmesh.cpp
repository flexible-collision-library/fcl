/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Carnegie Mellon University
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
 *   * Neither the name of Carnegie Mellon University nor the names of its
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

/** \author Jeongseok Lee <jslee02@gmail.com> */

#include <gtest/gtest.h>

#include "fcl/fcl.h"
#include "test_fcl_utility.h"

using namespace fcl;

//==============================================================================
template <typename Scalar>
void test_collision_triangle_triangle()
{
  Vector3<Scalar> P1(0, 0, 0);
  Vector3<Scalar> P2(2, 0, 0);
  Vector3<Scalar> P3(0, 2, 0);

  Vector3<Scalar> Q1(0, 0, 0);
  Vector3<Scalar> Q2(1, 0, 0);
  Vector3<Scalar> Q3(0, 0, 1);

  Vector3<Scalar> contact_points[6];
  unsigned int num_contact_points;
  Scalar penetration_depth;
  Vector3<Scalar> normal;

  auto res = detail::Intersect<Scalar>::intersect_Triangle(
        P1, P2, P3,
        Q1, Q2, Q3,
        contact_points, &num_contact_points, &penetration_depth, &normal);

  EXPECT_TRUE(res);
  EXPECT_EQ(num_contact_points, 2u);
  EXPECT_TRUE(contact_points[0].isApprox(Vector3<Scalar>::Zero()));
  EXPECT_TRUE(contact_points[1].isApprox(Vector3<Scalar>(1, 0, 0)));
  EXPECT_EQ(penetration_depth, (Scalar)0);

  res = detail::Intersect<Scalar>::intersect_Triangle(
        Q1, Q2, Q3,
        P1, P2, P3,
        contact_points, &num_contact_points, &penetration_depth, &normal);

  EXPECT_TRUE(res);
  EXPECT_EQ(num_contact_points, 2u);
  EXPECT_TRUE(contact_points[0].isApprox(Vector3<Scalar>::Zero()));
  EXPECT_TRUE(contact_points[1].isApprox(Vector3<Scalar>(1, 0, 0)));
  EXPECT_EQ(penetration_depth, (Scalar)0);
}

//==============================================================================
template <typename Scalar>
bool checkBoundingBox(const Vector3<Scalar>& min, const Vector3<Scalar>& max,
                      const Vector3<Scalar>& point, Scalar margin = 1e-12)
{
  for (auto i = 0u; i < 3u; ++i)
  {
    if (min[i] - margin > point[i] || point[i] > max[i] + margin)
      return false;
  }

  return true;
}

//==============================================================================
template <typename Scalar>
void test_collision_boxmeshboxmesh_contactpoint()
{
  const Transform3<Scalar> identity = Transform3<Scalar>::Identity();
  Transform3<Scalar> pose;

  Box<Scalar> s1(1, 1, 1);
  Box<Scalar> s2(0.5, 0.5, 0.5);

  BVHModel<AABB<Scalar>> s1_aabb;
  BVHModel<AABB<Scalar>> s2_aabb;
  BVHModel<OBB<Scalar>> s1_obb;
  BVHModel<OBB<Scalar>> s2_obb;
  BVHModel<RSS<Scalar>> s1_rss;
  BVHModel<RSS<Scalar>> s2_rss;
  BVHModel<OBBRSS<Scalar>> s1_obbrss;
  BVHModel<OBBRSS<Scalar>> s2_obbrss;

  generateBVHModel(s1_aabb, s1, identity);
  generateBVHModel(s2_aabb, s2, identity);
  generateBVHModel(s1_obb, s1, identity);
  generateBVHModel(s2_obb, s2, identity);
  generateBVHModel(s1_rss, s1, identity);
  generateBVHModel(s2_rss, s2, identity);
  generateBVHModel(s1_obbrss, s1, identity);
  generateBVHModel(s2_obbrss, s2, identity);

  CollisionRequest<Scalar> req(1e+3, true);

  CollisionResult<Scalar> res;

  //----------------------------------------------------------------------------
  // Case1: s2 is completely inside of s1.
  //
  // Primitive collision returns the maximum number of contact points, 4,
  // whereas mesh collision returns no contact.
  //----------------------------------------------------------------------------

  res.clear();
  collide(&s1, identity, &s2, identity, req, res);
  EXPECT_EQ(res.numContacts(), 4);  // maximum contact number of box-box

  res.clear();
  collide(&s1_aabb, identity, &s2_aabb, identity, req, res);
  EXPECT_EQ(res.numContacts(), 0);

  res.clear();
  collide(&s1_obb, identity, &s2_obb, identity, req, res);
  EXPECT_EQ(res.numContacts(), 0);

  res.clear();
  collide(&s1_rss, identity, &s2_rss, identity, req, res);
  EXPECT_EQ(res.numContacts(), 0);

  res.clear();
  collide(&s1_obbrss, identity, &s2_obbrss, identity, req, res);
  EXPECT_EQ(res.numContacts(), 0);

  //----------------------------------------------------------------------------
  // Case2: The left side (-y axis) of s2 touching the right side (+y axis) of
  // s1. The intersection is a face where the size is the same with the face of
  // s2 (0.5 x 0.5).
  //----------------------------------------------------------------------------

  pose = Translation3<Scalar>(Vector3<Scalar>(0, 0.75, 0));

  Vector3<Scalar> min(-0.25, 0.5, -0.25);
  Vector3<Scalar> max(0.25, 0.5, 0.25);

  Scalar margin = 0.01;
  min.array() -= margin;
  max.array() += margin;

  res.clear();
  collide(&s1, identity, &s2, pose, req, res);
  EXPECT_EQ(res.numContacts(), 4);
  for (auto i = 0u; i < res.numContacts(); ++i)
  {
    const auto& contact = res.getContact(i);
    const bool result = checkBoundingBox(min, max, contact.pos);

    EXPECT_TRUE(result);
    if (!result)
    {
      std::cout << "contact point: " << contact.pos.transpose() << std::endl;
      std::cout << "which is expected to be within ("
                << min.transpose() << ") and ("
                << max.transpose() << ")." << std::endl;
    }
  }

  res.clear();
  collide(&s1_aabb, identity, &s2_aabb, pose, req, res);
  EXPECT_TRUE(res.numContacts() > 4);
  for (auto i = 0u; i < res.numContacts(); ++i)
  {
    const auto& contact = res.getContact(i);
    const bool result = checkBoundingBox(min, max, contact.pos);

    EXPECT_TRUE(result);
    if (!result)
    {
      std::cout << "aabb contact point: " << contact.pos.transpose() << std::endl;
      std::cout << "which is expected to be within ("
                << min.transpose() << ") and ("
                << max.transpose() << ")." << std::endl;
    }
  }

  res.clear();
  collide(&s1_obb, identity, &s2_obb, pose, req, res);
  EXPECT_TRUE(res.numContacts() > 4);
  for (auto i = 0u; i < res.numContacts(); ++i)
  {
    const auto& contact = res.getContact(i);
    const bool result = checkBoundingBox(min, max, contact.pos);

    EXPECT_TRUE(result);
    if (!result)
    {
      std::cout << "obb contact point: " << contact.pos.transpose() << std::endl;
      std::cout << "which is expected to be within ("
                << min.transpose() << ") and ("
                << max.transpose() << ")." << std::endl;
    }
  }

  res.clear();
  collide(&s1_rss, identity, &s2_rss, pose, req, res);
  EXPECT_TRUE(res.numContacts() > 4);
  for (auto i = 0u; i < res.numContacts(); ++i)
  {
    const auto& contact = res.getContact(i);
    const bool result = checkBoundingBox(min, max, contact.pos);

    EXPECT_TRUE(result);
    if (!result)
    {
      std::cout << "rss contact point: " << contact.pos.transpose() << std::endl;
      std::cout << "which is expected to be within ("
                << min.transpose() << ") and ("
                << max.transpose() << ")." << std::endl;
    }
  }

  res.clear();
  collide(&s1_obbrss, identity, &s2_obbrss, pose, req, res);
  EXPECT_TRUE(res.numContacts() > 4);
  for (auto i = 0u; i < res.numContacts(); ++i)
  {
    const auto& contact = res.getContact(i);
    const bool result = checkBoundingBox(min, max, contact.pos);

    EXPECT_TRUE(result);
    if (!result)
    {
      std::cout << "obbrss contact point: " << contact.pos.transpose() << std::endl;
      std::cout << "which is expected to be within ("
                << min.transpose() << ") and ("
                << max.transpose() << ")." << std::endl;
    }
  }
}

//==============================================================================
GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, collision_triangle_triangle)
{
  test_collision_triangle_triangle<float>();
  test_collision_triangle_triangle<double>();
}

//==============================================================================
GTEST_TEST(FCL_SHAPE_MESH_CONSISTENCY, collision_boxmeshboxmesh_contactpoint)
{
  test_collision_boxmeshboxmesh_contactpoint<float>();
  test_collision_boxmeshboxmesh_contactpoint<double>();
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

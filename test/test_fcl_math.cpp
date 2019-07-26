/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
 *  Copyright (c) 2016, Toyota Research Institute
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
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
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

#include <gtest/gtest.h>

#include "fcl/broadphase/detail/morton.h"
#include "fcl/config.h"
#include "fcl/math/bv/AABB.h"
#include "fcl/math/bv/RSS.h"
#include "fcl/math/bv/utility.h"
#include "fcl/math/constants.h"

using namespace fcl;

template <typename S>
void test_vec_test_basic_vector()
{
  Vector3<S> v1(1.0, 2.0, 3.0);
  EXPECT_TRUE(v1[0] == (S)1.0);
  EXPECT_TRUE(v1[1] == (S)2.0);
  EXPECT_TRUE(v1[2] == (S)3.0);

  Vector3<S> v2 = v1;
  Vector3<S> v3(3.3, 4.3, 5.3);
  v1 += v3;
  EXPECT_TRUE(v1.isApprox(v2 + v3));
  v1 -= v3;
  EXPECT_TRUE(v1.isApprox(v2));
  v1 -= v3;
  EXPECT_TRUE(v1.isApprox(v2 - v3));
  v1 += v3;

  v1.array() *= v3.array();
  EXPECT_TRUE(v1.array().isApprox(v2.array() * v3.array()));
  v1.array() /= v3.array();
  EXPECT_TRUE(v1.isApprox(v2));
  v1.array() /= v3.array();
  EXPECT_TRUE(v1.array().isApprox(v2.array() / v3.array()));
  v1.array() *= v3.array();

  v1 *= 2.0;
  EXPECT_TRUE(v1.isApprox(v2 * 2.0));
  v1 /= 2.0;
  EXPECT_TRUE(v1.isApprox(v2));
  v1 /= 2.0;
  EXPECT_TRUE(v1.isApprox(v2 / 2.0));
  v1 *= 2.0;

  v1.array() += 2.0;
  EXPECT_TRUE(v1.array().isApprox(v2.array() + 2.0));
  v1.array() -= 2.0;
  EXPECT_TRUE(v1.isApprox(v2));
  v1.array() -= 2.0;
  EXPECT_TRUE(v1.array().isApprox(v2.array() - 2.0));
  v1.array() += 2.0;

  EXPECT_TRUE((-Vector3<S>(1.0, 2.0, 3.0)) == (Vector3<S>(-1.0, -2.0, -3.0)));

  v1 = Vector3<S>(1.0, 2.0, 3.0);
  v2 = Vector3<S>(3.0, 4.0, 5.0);
  EXPECT_TRUE((v1.cross(v2)).isApprox(Vector3<S>(-2.0, 4.0, -2.0)));
  EXPECT_TRUE(std::abs(v1.dot(v2) - 26) < 1e-5);

  v1 = Vector3<S>(3.0, 4.0, 5.0);
  EXPECT_TRUE(std::abs(v1.squaredNorm() - 50.0) < 1e-5);
  EXPECT_TRUE(std::abs(v1.norm() - sqrt(50.0)) < 1e-5);
  EXPECT_TRUE(v1.normalized().isApprox(v1 / v1.norm()));

  v1 = Vector3<S>(1.0, 2.0, 3.0);
  v2 = Vector3<S>(3.0, 4.0, 5.0);
  EXPECT_TRUE((v1.cross(v2)).isApprox(Vector3<S>(-2.0, 4.0, -2.0)));
  EXPECT_TRUE(v1.dot(v2) == 26);
}

GTEST_TEST(FCL_MATH, vec_test_basic_vector3)
{
//  test_vec_test_basic_vector<float>();
  test_vec_test_basic_vector<double>();
}

template <typename S>
void test_morton()
{
  AABB<S> bbox(Vector3<S>(0, 0, 0), Vector3<S>(1000, 1000, 1000));
  detail::morton_functor<S, std::bitset<30>> F1(bbox);
  detail::morton_functor<S, std::bitset<60>> F2(bbox);
  detail::morton_functor<S, uint64> F3(bbox); // 60 bits
  detail::morton_functor<S, uint32> F4(bbox); // 30 bits

  Vector3<S> p(254, 873, 674);

  EXPECT_TRUE(F1(p).to_ulong() == F4(p));
  EXPECT_TRUE(F2(p).to_ullong() == F3(p));
}

GTEST_TEST(FCL_MATH, morton)
{
//  test_morton<float>();
  test_morton<double>();
}

// Test fitting an RSS to various number of points
template <typename S>
void test_rss_fit()
{
  const S epsilon = constants<S>::eps_78();
  RSS<S> rss;
  Vector3<S> pn[8];
  pn[0] << 0, 0, 0;
  pn[1] << 1, 0, 0;
  // Check fitting 1 point
  fit(pn, 1, rss);
  EXPECT_EQ(rss.center(), pn[0]);
  EXPECT_EQ(rss.width(), 0.0);
  EXPECT_EQ(rss.height(), 0.0);
  EXPECT_EQ(rss.depth(), 0.0);
  EXPECT_TRUE(rss.axis.isApprox(Matrix3<S>::Identity()));
  // Check fitting 2 points for each axis
  fit(pn, 2, rss);
  EXPECT_TRUE(rss.center().isApprox(Vector3<S>(0.5, 0, 0)));
  EXPECT_EQ(rss.width(), 1.0);
  EXPECT_EQ(rss.height(), 0.0);
  EXPECT_EQ(rss.depth(), 0.0);
  EXPECT_TRUE(rss.axis.col(0).isApprox(Vector3<S>(1, 0, 0)) ||
              rss.axis.col(0).isApprox(Vector3<S>(-1, 0, 0)));
  pn[1] << 0, 1, 0;
  fit(pn, 2, rss);
  EXPECT_TRUE(rss.center().isApprox(Vector3<S>(0, 0.5, 0)));
  EXPECT_EQ(rss.width(), 1.0);
  EXPECT_EQ(rss.height(), 0.0);
  EXPECT_EQ(rss.depth(), 0.0);
  EXPECT_TRUE(rss.axis.col(0).isApprox(Vector3<S>(0, 1, 0)) ||
              rss.axis.col(0).isApprox(Vector3<S>(0, -1, 0)));
  pn[1] << 0, 0, 1;
  fit(pn, 2, rss);
  EXPECT_TRUE(rss.center().isApprox(Vector3<S>(0, 0, 0.5)));
  EXPECT_EQ(rss.width(), 1.0);
  EXPECT_EQ(rss.height(), 0.0);
  EXPECT_EQ(rss.depth(), 0.0);
  EXPECT_TRUE(rss.axis.col(0).isApprox(Vector3<S>(0, 0, 1)) ||
              rss.axis.col(0).isApprox(Vector3<S>(0, 0, -1)));
  // Check fitting 2 in opposite order
  pn[0] << 0, 0, 1;
  pn[1] << 0, 0, 0;
  fit(pn, 2, rss);
  EXPECT_TRUE(rss.center().isApprox(Vector3<S>(0, 0, 0.5)));
  EXPECT_EQ(rss.width(), 1.0);
  EXPECT_EQ(rss.height(), 0.0);
  EXPECT_EQ(rss.depth(), 0.0);
  EXPECT_TRUE(rss.axis.col(0).isApprox(Vector3<S>(0, 0, 1)) ||
              rss.axis.col(0).isApprox(Vector3<S>(0, 0, -1)));
  // Check fitting 2 not along an axis
  pn[0] << -1, 1, 0;
  pn[1] << 0, 0, 0;
  fit(pn, 2, rss);
  EXPECT_TRUE(rss.center().isApprox(Vector3<S>(-0.5, 0.5, 0)));
  EXPECT_NEAR(rss.width(), sqrt(2.0), epsilon);
  EXPECT_EQ(rss.height(), 0.0);
  EXPECT_EQ(rss.depth(), 0.0);
  EXPECT_TRUE(
      rss.axis.col(0).isApprox(Vector3<S>(-sqrt(2.0)/2.0, sqrt(2.0)/2.0, 0)) ||
      rss.axis.col(0).isApprox(Vector3<S>(sqrt(2.0)/2.0, -sqrt(2.0)/2.0, 0)));
  // Check fitting 3
  pn[0] << 0, 0, 0;
  pn[1] << 1, 0, 0;
  pn[2] << 0, 1, 0;
  fit(pn, 3, rss);
  Vector3<S> c3(0.25, 0.25, 0);
  EXPECT_TRUE(c3.isApprox(rss.center()));
  EXPECT_NEAR(rss.width(), sqrt(2.0), epsilon);
  EXPECT_NEAR(rss.height(), sqrt(2.0) / 2.0, epsilon);
  EXPECT_EQ(rss.depth(), 0.0);
  // Check fitting 8
  pn[3] << 0, 0, 1;
  pn[4] << 1, 0, 0;
  pn[5] << 1, 0, 1;
  pn[6] << 1, 1, 0;
  pn[7] << 1, 1, 1;
  fit(pn, 8, rss);
  EXPECT_GE(rss.depth(), 0.5);
}

// Test convert RSS to other bounding volumes
template <typename S>
void test_rss_conversion()
{
  const S epsilon = constants<S>::eps_78();
  RSS<S> rss;
  Vector3<S> pn[8];
  AABB<S> aabb;
  OBB<S> obb;

  pn[0] << 0, 0, 0;
  pn[1] << 1, 0, 0;
  pn[2] << 0, 1, 0;
  pn[3] << 0, 0, 1;
  pn[4] << 1, 0, 0;
  pn[5] << 1, 0, 1;
  pn[6] << 1, 1, 0;
  pn[7] << 1, 1, 1;
  fit(pn, 8, rss);

  convertBV(rss, Transform3<S>::Identity(), aabb);
  EXPECT_GE(aabb.width(), rss.width());
  EXPECT_GE(aabb.height(), rss.height());
  EXPECT_GE(aabb.depth(), rss.depth());
  EXPECT_TRUE(aabb.center().isApprox(rss.center()));
  convertBV(aabb, Transform3<S>::Identity(), rss);
  // The resulting RSS must be bigger than the AABB for it to contain it
  EXPECT_GE(rss.width() - rss.depth() + epsilon, aabb.width());
  EXPECT_GE(rss.height() - rss.depth() + epsilon, aabb.height());
  EXPECT_GE(rss.depth(), aabb.depth());
  EXPECT_TRUE(aabb.center().isApprox(rss.center()));
  convertBV(rss, Transform3<S>::Identity(), obb);
  EXPECT_GE(obb.width(), rss.width());
  EXPECT_GE(obb.height(), rss.height());
  EXPECT_GE(obb.depth(), rss.depth());
  EXPECT_TRUE(obb.center().isApprox(rss.center()));
}

// Test RSS to RSS distance function
template <typename S>
void test_rss_distance()
{
  const S epsilon = constants<S>::eps_78();
  Vector3<S> p0[3];
  Vector3<S> p1[3];
  RSS<S> rss;
  RSS<S> rss2;
  // Test RSS to RSS distance for correctness
  p0[0] << 1, 1, 1;
  p0[1] << 1, 1, -1;
  p0[2] << 0, 1, -1;
  p1[0] << 1, -1, 1;
  p1[1] << 1, -1, -1;
  p1[2] << 0, -1, -1;
  fit(p0, 3, rss);
  fit(p1, 3, rss2);
  EXPECT_NEAR(rss.distance(rss2), 2.0, epsilon);
  rss.To << 0, 0, 0.5;
  rss.axis = Matrix3<S>::Identity();
  rss.l[0] = 1;
  rss.l[1] = 1;
  rss.r = 0.5;
  rss2.To << -1, -1, 2.5;
  rss2.axis = Matrix3<S>::Identity();
  rss2.l[0] = 1;
  rss2.l[1] = 1;
  rss2.r = 0.5;
  EXPECT_NEAR(rss.distance(rss2), 1.0, epsilon);
  rss2.axis << -1, 0, 0,
                0, -1, 0,
                0, 0, -1;
  EXPECT_NEAR(rss.distance(rss2), sqrt(6) - 1.0, epsilon);
  rss2.To << 0, 0, 2.5;
  EXPECT_NEAR(rss.distance(rss2), 1.0, epsilon);
}

// Test setting RSS position
template <typename S>
void test_rss_position()
{
  // Verify setting To via the center works correctly.
  RSS<S> rss;
  rss.l[0] = 1;
  rss.l[1] = 1;
  rss.r = 0.5;
  rss.axis = Matrix3<S>::Identity();
  rss.setToFromCenter(Vector3<S>(0.5, 0.5, 0.5));
  EXPECT_TRUE(rss.To.isApprox(Vector3<S>(0.0, 0.0, 0.5)));
  rss.axis *= -1.0;
  rss.setToFromCenter(Vector3<S>(-0.5, -0.5, 2.5));
  EXPECT_TRUE(rss.To.isApprox(Vector3<S>(0.0, 0.0, 2.5)));
}

GTEST_TEST(FCL_MATH, rss_fit)
{
  test_rss_fit<double>();
}

GTEST_TEST(FCL_MATH, rss_conversion)
{
  test_rss_conversion<double>();
}

GTEST_TEST(FCL_MATH, rss_distance)
{
  test_rss_distance<double>();
}

GTEST_TEST(FCL_MATH, rss_position)
{
  test_rss_position<double>();
}

// TODO test overlap
// TODO test contain
// TODO test operator+
// TODO test size

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

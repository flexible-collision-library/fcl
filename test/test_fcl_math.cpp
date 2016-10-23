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

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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

#include "fcl/broadphase/morton.h"
#include "fcl/config.h"

using namespace fcl;

GTEST_TEST(FCL_MATH, vec_test_basic_vec32)
{
  using Vec3f32 = Vector3<float>;

  Vec3f32 v1(1.0f, 2.0f, 3.0f);
  EXPECT_TRUE(v1[0] == 1.0f);
  EXPECT_TRUE(v1[1] == 2.0f);
  EXPECT_TRUE(v1[2] == 3.0f);

  Vec3f32 v2 = v1;
  Vec3f32 v3(3.3f, 4.3f, 5.3f);
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

  v1 *= 2.0f;
  EXPECT_TRUE(v1.isApprox(v2 * 2.0f));
  v1 /= 2.0f;
  EXPECT_TRUE(v1.isApprox(v2));
  v1 /= 2.0f;
  EXPECT_TRUE(v1.isApprox(v2 / 2.0f));
  v1 *= 2.0f;

  v1.array() += 2.0f;
  EXPECT_TRUE(v1.array().isApprox(v2.array() + 2.0f));
  v1.array() -= 2.0f;
  EXPECT_TRUE(v1.isApprox(v2));
  v1.array() -= 2.0f;
  EXPECT_TRUE(v1.array().isApprox(v2.array() - 2.0f));
  v1.array() += 2.0f;

  EXPECT_TRUE((-Vec3f32(1.0f, 2.0f, 3.0f)).isApprox(Vec3f32(-1.0f, -2.0f, -3.0f)));

  v1 = Vec3f32(1.0f, 2.0f, 3.0f);
  v2 = Vec3f32(3.0f, 4.0f, 5.0f);
  EXPECT_TRUE((v1.cross(v2)).isApprox(Vec3f32(-2.0f, 4.0f, -2.0f)));
  EXPECT_TRUE(std::abs(v1.dot(v2) - 26) < 1e-5);

  v1 = Vec3f32(3.0f, 4.0f, 5.0f);
  EXPECT_TRUE(std::abs(v1.squaredNorm() - 50.0) < 1e-5);
  EXPECT_TRUE(std::abs(v1.norm() - sqrt(50.0)) < 1e-5);
  EXPECT_TRUE(v1.normalized().isApprox(v1 / v1.norm()));
}

GTEST_TEST(FCL_MATH, vec_test_basic_vec64)
{
  using Vec3f64 = Vector3<double>;

  Vec3f64 v1(1.0, 2.0, 3.0);
  EXPECT_TRUE(v1[0] == 1.0);
  EXPECT_TRUE(v1[1] == 2.0);
  EXPECT_TRUE(v1[2] == 3.0);

  Vec3f64 v2 = v1;
  Vec3f64 v3(3.3, 4.3, 5.3);
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

  EXPECT_TRUE((-Vec3f64(1.0, 2.0, 3.0)) == (Vec3f64(-1.0, -2.0, -3.0)));

  v1 = Vec3f64(1.0, 2.0, 3.0);
  v2 = Vec3f64(3.0, 4.0, 5.0);
  EXPECT_TRUE((v1.cross(v2)).isApprox(Vec3f64(-2.0, 4.0, -2.0)));
  EXPECT_TRUE(std::abs(v1.dot(v2) - 26) < 1e-5);

  v1 = Vec3f64(3.0, 4.0, 5.0);
  EXPECT_TRUE(std::abs(v1.squaredNorm() - 50.0) < 1e-5);
  EXPECT_TRUE(std::abs(v1.norm() - sqrt(50.0)) < 1e-5);
  EXPECT_TRUE(v1.normalized().isApprox(v1 / v1.norm()));

  v1 = Vec3f64(1.0, 2.0, 3.0);
  v2 = Vec3f64(3.0, 4.0, 5.0);
  EXPECT_TRUE((v1.cross(v2)).isApprox(Vec3f64(-2.0, 4.0, -2.0)));
  EXPECT_TRUE(v1.dot(v2) == 26);
}

GTEST_TEST(FCL_MATH, morton)
{
  AABB bbox(Vector3d(0, 0, 0), Vector3d(1000, 1000, 1000));
  morton_functor<std::bitset<30>> F1(bbox);
  morton_functor<std::bitset<60>> F2(bbox);
  morton_functor<FCL_UINT64> F3(bbox); // 60 bits
  morton_functor<FCL_UINT32> F4(bbox); // 30 bits

  Vector3d p(254, 873, 674);

  EXPECT_TRUE(F1(p).to_ulong() == F4(p));
  EXPECT_TRUE(F2(p).to_ullong() == F3(p));
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

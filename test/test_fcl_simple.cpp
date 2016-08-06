/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
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

#include <sstream>

#include "fcl/intersect.h"
#include "fcl/collision.h"
#include "fcl/BVH/BVH_model.h"
#include "fcl_resources/config.h"
#include "fcl/math/sampler_r.h"
#include "fcl/math/sampler_se2.h"
#include "fcl/math/sampler_se2_disk.h"
#include "fcl/math/sampler_se3_euler.h"
#include "fcl/math/sampler_se3_euler_ball.h"
#include "fcl/math/sampler_se3_quat.h"
#include "fcl/math/sampler_se3_quat_ball.h"
#include "fcl/math/geometry.h"

using namespace fcl;

template <typename Scalar>
Scalar epsilon()
{
  return 1e-6;
}

template <>
float epsilon()
{
  return 1e-4;
}

template <typename Scalar>
bool approx(Scalar x, Scalar y)
{
  return std::abs(x - y) < epsilon<Scalar>();
}

template<typename Scalar, std::size_t N>
Scalar distance_Vecnf(const VectorN<Scalar, N>& a, const VectorN<Scalar, N>& b)
{
  Scalar d = 0;
  for(std::size_t i = 0; i < N; ++i)
    d += (a[i] - b[i]) * (a[i] - b[i]);

  return d;
}

template <typename Scalar>
void test_Vec_nf_test()
{
  VectorN<Scalar, 4> a;
  VectorN<Scalar, 4> b;
  for(auto i = 0; i < a.size(); ++i)
    a[i] = i;
  for(auto i = 0; i < b.size(); ++i)
    b[i] = 1;

  std::cout << a.transpose() << std::endl;
  std::cout << b.transpose() << std::endl;
  std::cout << (a + b).transpose() << std::endl;
  std::cout << (a - b).transpose() << std::endl;
  std::cout << (a -= b).transpose() << std::endl;
  std::cout << (a += b).transpose() << std::endl;
  std::cout << (a * 2).transpose() << std::endl;
  std::cout << (a / 2).transpose() << std::endl;
  std::cout << (a *= 2).transpose() << std::endl;
  std::cout << (a /= 2).transpose() << std::endl;
  std::cout << a.dot(b) << std::endl;

  VectorN<Scalar, 8> c = combine(a, b);
  std::cout << c.transpose() << std::endl;

  VectorN<Scalar, 4> upper, lower;
  for(int i = 0; i < 4; ++i)
    upper[i] = 1;

  VectorN<Scalar, 4> aa = VectorN<Scalar, 4>(1, 2, 1, 2);
  std::cout << aa.transpose() << std::endl;

  SamplerR<Scalar, 4> sampler(lower, upper);
  for(std::size_t i = 0; i < 10; ++i)
    std::cout << sampler.sample().transpose() << std::endl;

  // Disabled broken test lines. Please see #25.
  // SamplerSE2 sampler2(0, 1, -1, 1);
  // for(std::size_t i = 0; i < 10; ++i)
  //   std::cout << sampler2.sample() << std::endl;

  SamplerSE3Euler<Scalar> sampler3(Vector3<Scalar>(0, 0, 0), Vector3<Scalar>(1, 1, 1));
  for(std::size_t i = 0; i < 10; ++i)
    std::cout << sampler3.sample().transpose() << std::endl;
  
}

GTEST_TEST(FCL_SIMPLE, Vec_nf_test)
{
  test_Vec_nf_test<float>();
  test_Vec_nf_test<double>();
}

template <typename Scalar>
void test_projection_test_line()
{
  Vector3<Scalar> v1(0, 0, 0);
  Vector3<Scalar> v2(2, 0, 0);
    
  Vector3<Scalar> p(1, 0, 0);
  auto res = Project<Scalar>::projectLine(v1, v2, p);
  EXPECT_TRUE(res.encode == 3);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)0.5));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)0.5));
    
  p = Vector3<Scalar>(-1, 0, 0);
  res = Project<Scalar>::projectLine(v1, v2, p);
  EXPECT_TRUE(res.encode == 1);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)1));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)1));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)0));

  p = Vector3<Scalar>(3, 0, 0);
  res = Project<Scalar>::projectLine(v1, v2, p);
  EXPECT_TRUE(res.encode == 2);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)1));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)1));

}

GTEST_TEST(FCL_SIMPLE, projection_test_line)
{
  test_projection_test_line<float>();
  test_projection_test_line<double>();
}

template <typename Scalar>
void test_projection_test_triangle()
{
  Vector3<Scalar> v1(0, 0, 1);
  Vector3<Scalar> v2(0, 1, 0);
  Vector3<Scalar> v3(1, 0, 0);

  Vector3<Scalar> p(1, 1, 1);
  auto res = Project<Scalar>::projectTriangle(v1, v2, v3, p);
  EXPECT_TRUE(res.encode == 7);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)(4/3.0)));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)(1/3.0)));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)(1/3.0)));
  EXPECT_TRUE(approx(res.parameterization[2], (Scalar)(1/3.0)));
  
  p = Vector3<Scalar>(0, 0, 1.5);
  res = Project<Scalar>::projectTriangle(v1, v2, v3, p);
  EXPECT_TRUE(res.encode == 1);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)0.25));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)1));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[2], (Scalar)0));

  p = Vector3<Scalar>(1.5, 0, 0);
  res = Project<Scalar>::projectTriangle(v1, v2, v3, p);
  EXPECT_TRUE(res.encode == 4);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)0.25));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[2], (Scalar)1));

  p = Vector3<Scalar>(0, 1.5, 0);
  res = Project<Scalar>::projectTriangle(v1, v2, v3, p);
  EXPECT_TRUE(res.encode == 2);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)0.25));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)1));
  EXPECT_TRUE(approx(res.parameterization[2], (Scalar)0));

  p = Vector3<Scalar>(1, 1, 0);
  res = Project<Scalar>::projectTriangle(v1, v2, v3, p);
  EXPECT_TRUE(res.encode == 6);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)0.5));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)0.5));
  EXPECT_TRUE(approx(res.parameterization[2], (Scalar)0.5));

  p = Vector3<Scalar>(1, 0, 1);
  res = Project<Scalar>::projectTriangle(v1, v2, v3, p);
  EXPECT_TRUE(res.encode == 5);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)0.5));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)0.5));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[2], (Scalar)0.5));

  p = Vector3<Scalar>(0, 1, 1);
  res = Project<Scalar>::projectTriangle(v1, v2, v3, p);
  EXPECT_TRUE(res.encode == 3);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)0.5));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)0.5));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)0.5));
  EXPECT_TRUE(approx(res.parameterization[2], (Scalar)0));
}

GTEST_TEST(FCL_SIMPLE, projection_test_triangle)
{
  test_projection_test_triangle<float>();
  test_projection_test_triangle<double>();
}

template <typename Scalar>
void test_projection_test_tetrahedron()
{
  Vector3<Scalar> v1(0, 0, 1);
  Vector3<Scalar> v2(0, 1, 0);
  Vector3<Scalar> v3(1, 0, 0);
  Vector3<Scalar> v4(1, 1, 1);

  Vector3<Scalar> p(0.5, 0.5, 0.5);
  auto res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 15);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)0.25));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)0.25));
  EXPECT_TRUE(approx(res.parameterization[2], (Scalar)0.25));
  EXPECT_TRUE(approx(res.parameterization[3], (Scalar)0.25));

  p = Vector3<Scalar>(0, 0, 0);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 7);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)(1/3.0)));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)(1/3.0)));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)(1/3.0)));
  EXPECT_TRUE(approx(res.parameterization[2], (Scalar)(1/3.0)));
  EXPECT_TRUE(approx(res.parameterization[3], (Scalar)0));

  p = Vector3<Scalar>(0, 1, 1);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 11);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)(1/3.0)));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)(1/3.0)));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)(1/3.0)));
  EXPECT_TRUE(approx(res.parameterization[2], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[3], (Scalar)(1/3.0)));

  p = Vector3<Scalar>(1, 1, 0);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 14);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)(1/3.0)));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)(1/3.0)));
  EXPECT_TRUE(approx(res.parameterization[2], (Scalar)(1/3.0)));
  EXPECT_TRUE(approx(res.parameterization[3], (Scalar)(1/3.0)));

  p = Vector3<Scalar>(1, 0, 1);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 13);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)(1/3.0)));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)(1/3.0)));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[2], (Scalar)(1/3.0)));
  EXPECT_TRUE(approx(res.parameterization[3], (Scalar)(1/3.0)));

  p = Vector3<Scalar>(1.5, 1.5, 1.5);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 8);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)0.75));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[2], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[3], (Scalar)1));

  p = Vector3<Scalar>(1.5, -0.5, -0.5);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 4);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)0.75));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[2], (Scalar)1));
  EXPECT_TRUE(approx(res.parameterization[3], (Scalar)0));

  p = Vector3<Scalar>(-0.5, -0.5, 1.5);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 1);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)0.75));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)1));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[2], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[3], (Scalar)0));

  p = Vector3<Scalar>(-0.5, 1.5, -0.5);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 2);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)0.75));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)1));
  EXPECT_TRUE(approx(res.parameterization[2], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[3], (Scalar)0));

  p = Vector3<Scalar>(0.5, -0.5, 0.5);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 5);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)0.25));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)0.5));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[2], (Scalar)0.5));
  EXPECT_TRUE(approx(res.parameterization[3], (Scalar)0));

  p = Vector3<Scalar>(0.5, 1.5, 0.5);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 10);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)0.25));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)0.5));
  EXPECT_TRUE(approx(res.parameterization[2], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[3], (Scalar)0.5));

  p = Vector3<Scalar>(1.5, 0.5, 0.5);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 12);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)0.25));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[2], (Scalar)0.5));
  EXPECT_TRUE(approx(res.parameterization[3], (Scalar)0.5));
    
  p = Vector3<Scalar>(-0.5, 0.5, 0.5);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 3);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)0.25));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)0.5));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)0.5));
  EXPECT_TRUE(approx(res.parameterization[2], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[3], (Scalar)0));

  p = Vector3<Scalar>(0.5, 0.5, 1.5);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 9);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)0.25));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)0.5));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[2], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[3], (Scalar)0.5));
    
  p = Vector3<Scalar>(0.5, 0.5, -0.5);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 6);
  EXPECT_TRUE(approx(res.sqr_distance, (Scalar)0.25));
  EXPECT_TRUE(approx(res.parameterization[0], (Scalar)0));
  EXPECT_TRUE(approx(res.parameterization[1], (Scalar)0.5));
  EXPECT_TRUE(approx(res.parameterization[2], (Scalar)0.5));
  EXPECT_TRUE(approx(res.parameterization[3], (Scalar)0));

}

GTEST_TEST(FCL_SIMPLE, projection_test_tetrahedron)
{
  test_projection_test_tetrahedron<float>();
  test_projection_test_tetrahedron<double>();
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

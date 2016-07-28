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
#include "fcl/math/sampling.h"
#include "fcl/math/geometry.h"

using namespace fcl;

static FCL_REAL epsilon = 1e-6;

static bool approx(FCL_REAL x, FCL_REAL y)
{
  return std::abs(x - y) < epsilon;
}



template<std::size_t N>
double distance_Vecnf(const VectorNd<N>& a, const VectorNd<N>& b)
{
  double d = 0;
  for(std::size_t i = 0; i < N; ++i)
    d += (a[i] - b[i]) * (a[i] - b[i]);

  return d;
}


GTEST_TEST(FCL_SIMPLE, Vec_nf_test)
{
  VectorNd<4> a;
  VectorNd<4> b;
  for(auto i = 0; i < a.size(); ++i)
    a[i] = i;
  for(auto i = 0; i < b.size(); ++i)
    b[i] = 1;

  std::cout << a << std::endl;
  std::cout << b << std::endl;
  std::cout << a + b << std::endl;
  std::cout << a - b << std::endl;
  std::cout << (a -= b) << std::endl;
  std::cout << (a += b) << std::endl;
  std::cout << a * 2 << std::endl;
  std::cout << a / 2 << std::endl;
  std::cout << (a *= 2) << std::endl;
  std::cout << (a /= 2) << std::endl;
  std::cout << a.dot(b) << std::endl;

  VectorNd<8> c = combine(a, b);
  std::cout << c << std::endl;

  VectorNd<4> upper, lower;
  for(int i = 0; i < 4; ++i)
    upper[i] = 1;

  VectorNd<4> aa = VectorNd<4>(1, 2, 1, 2);
  std::cout << aa << std::endl;

  SamplerR<4> sampler(lower, upper);
  for(std::size_t i = 0; i < 10; ++i)
    std::cout << sampler.sample() << std::endl;

  // Disabled broken test lines. Please see #25.
  // SamplerSE2 sampler2(0, 1, -1, 1);
  // for(std::size_t i = 0; i < 10; ++i)
  //   std::cout << sampler2.sample() << std::endl;

  SamplerSE3Euler sampler3(Vector3d(0, 0, 0), Vector3d(1, 1, 1));
  for(std::size_t i = 0; i < 10; ++i)
    std::cout << sampler3.sample() << std::endl;
  
}


GTEST_TEST(FCL_SIMPLE, projection_test_line)
{
  Vector3d v1(0, 0, 0);
  Vector3d v2(2, 0, 0);
    
  Vector3d p(1, 0, 0);
  Project::ProjectResult res = Project::projectLine(v1, v2, p);
  EXPECT_TRUE(res.encode == 3);
  EXPECT_TRUE(approx(res.sqr_distance, 0));
  EXPECT_TRUE(approx(res.parameterization[0], 0.5));
  EXPECT_TRUE(approx(res.parameterization[1], 0.5));
    
  p = Vector3d(-1, 0, 0);
  res = Project::projectLine(v1, v2, p);
  EXPECT_TRUE(res.encode == 1);
  EXPECT_TRUE(approx(res.sqr_distance, 1));
  EXPECT_TRUE(approx(res.parameterization[0], 1));
  EXPECT_TRUE(approx(res.parameterization[1], 0));

  p = Vector3d(3, 0, 0);
  res = Project::projectLine(v1, v2, p);
  EXPECT_TRUE(res.encode == 2);
  EXPECT_TRUE(approx(res.sqr_distance, 1));
  EXPECT_TRUE(approx(res.parameterization[0], 0));
  EXPECT_TRUE(approx(res.parameterization[1], 1));

}

GTEST_TEST(FCL_SIMPLE, projection_test_triangle)
{
  Vector3d v1(0, 0, 1);
  Vector3d v2(0, 1, 0);
  Vector3d v3(1, 0, 0);

  Vector3d p(1, 1, 1);
  Project::ProjectResult res = Project::projectTriangle(v1, v2, v3, p);
  EXPECT_TRUE(res.encode == 7);
  EXPECT_TRUE(approx(res.sqr_distance, 4/3.0));
  EXPECT_TRUE(approx(res.parameterization[0], 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[1], 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[2], 1/3.0));
  
  p = Vector3d(0, 0, 1.5);
  res = Project::projectTriangle(v1, v2, v3, p);
  EXPECT_TRUE(res.encode == 1);
  EXPECT_TRUE(approx(res.sqr_distance, 0.25));
  EXPECT_TRUE(approx(res.parameterization[0], 1));
  EXPECT_TRUE(approx(res.parameterization[1], 0));
  EXPECT_TRUE(approx(res.parameterization[2], 0));

  p = Vector3d(1.5, 0, 0);
  res = Project::projectTriangle(v1, v2, v3, p);
  EXPECT_TRUE(res.encode == 4);
  EXPECT_TRUE(approx(res.sqr_distance, 0.25));
  EXPECT_TRUE(approx(res.parameterization[0], 0));
  EXPECT_TRUE(approx(res.parameterization[1], 0));
  EXPECT_TRUE(approx(res.parameterization[2], 1));

  p = Vector3d(0, 1.5, 0);
  res = Project::projectTriangle(v1, v2, v3, p);
  EXPECT_TRUE(res.encode == 2);
  EXPECT_TRUE(approx(res.sqr_distance, 0.25));
  EXPECT_TRUE(approx(res.parameterization[0], 0));
  EXPECT_TRUE(approx(res.parameterization[1], 1));
  EXPECT_TRUE(approx(res.parameterization[2], 0));

  p = Vector3d(1, 1, 0);
  res = Project::projectTriangle(v1, v2, v3, p);
  EXPECT_TRUE(res.encode == 6);
  EXPECT_TRUE(approx(res.sqr_distance, 0.5));
  EXPECT_TRUE(approx(res.parameterization[0], 0));
  EXPECT_TRUE(approx(res.parameterization[1], 0.5));
  EXPECT_TRUE(approx(res.parameterization[2], 0.5));

  p = Vector3d(1, 0, 1);
  res = Project::projectTriangle(v1, v2, v3, p);
  EXPECT_TRUE(res.encode == 5);
  EXPECT_TRUE(approx(res.sqr_distance, 0.5));
  EXPECT_TRUE(approx(res.parameterization[0], 0.5));
  EXPECT_TRUE(approx(res.parameterization[1], 0));
  EXPECT_TRUE(approx(res.parameterization[2], 0.5));

  p = Vector3d(0, 1, 1);
  res = Project::projectTriangle(v1, v2, v3, p);
  EXPECT_TRUE(res.encode == 3);
  EXPECT_TRUE(approx(res.sqr_distance, 0.5));
  EXPECT_TRUE(approx(res.parameterization[0], 0.5));
  EXPECT_TRUE(approx(res.parameterization[1], 0.5));
  EXPECT_TRUE(approx(res.parameterization[2], 0));
}

GTEST_TEST(FCL_SIMPLE, projection_test_tetrahedron)
{
  Vector3d v1(0, 0, 1);
  Vector3d v2(0, 1, 0);
  Vector3d v3(1, 0, 0);
  Vector3d v4(1, 1, 1);

  Vector3d p(0.5, 0.5, 0.5);
  Project::ProjectResult res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 15);
  EXPECT_TRUE(approx(res.sqr_distance, 0));
  EXPECT_TRUE(approx(res.parameterization[0], 0.25));
  EXPECT_TRUE(approx(res.parameterization[1], 0.25));
  EXPECT_TRUE(approx(res.parameterization[2], 0.25));
  EXPECT_TRUE(approx(res.parameterization[3], 0.25));

  p = Vector3d(0, 0, 0);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 7);
  EXPECT_TRUE(approx(res.sqr_distance, 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[0], 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[1], 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[2], 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[3], 0));

  p = Vector3d(0, 1, 1);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 11);
  EXPECT_TRUE(approx(res.sqr_distance, 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[0], 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[1], 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[2], 0));
  EXPECT_TRUE(approx(res.parameterization[3], 1/3.0));

  p = Vector3d(1, 1, 0);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 14);
  EXPECT_TRUE(approx(res.sqr_distance, 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[0], 0));
  EXPECT_TRUE(approx(res.parameterization[1], 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[2], 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[3], 1/3.0));

  p = Vector3d(1, 0, 1);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 13);
  EXPECT_TRUE(approx(res.sqr_distance, 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[0], 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[1], 0));
  EXPECT_TRUE(approx(res.parameterization[2], 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[3], 1/3.0));

  p = Vector3d(1.5, 1.5, 1.5);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 8);
  EXPECT_TRUE(approx(res.sqr_distance, 0.75));
  EXPECT_TRUE(approx(res.parameterization[0], 0));
  EXPECT_TRUE(approx(res.parameterization[1], 0));
  EXPECT_TRUE(approx(res.parameterization[2], 0));
  EXPECT_TRUE(approx(res.parameterization[3], 1));

  p = Vector3d(1.5, -0.5, -0.5);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 4);
  EXPECT_TRUE(approx(res.sqr_distance, 0.75));
  EXPECT_TRUE(approx(res.parameterization[0], 0));
  EXPECT_TRUE(approx(res.parameterization[1], 0));
  EXPECT_TRUE(approx(res.parameterization[2], 1));
  EXPECT_TRUE(approx(res.parameterization[3], 0));

  p = Vector3d(-0.5, -0.5, 1.5);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 1);
  EXPECT_TRUE(approx(res.sqr_distance, 0.75));
  EXPECT_TRUE(approx(res.parameterization[0], 1));
  EXPECT_TRUE(approx(res.parameterization[1], 0));
  EXPECT_TRUE(approx(res.parameterization[2], 0));
  EXPECT_TRUE(approx(res.parameterization[3], 0));

  p = Vector3d(-0.5, 1.5, -0.5);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 2);
  EXPECT_TRUE(approx(res.sqr_distance, 0.75));
  EXPECT_TRUE(approx(res.parameterization[0], 0));
  EXPECT_TRUE(approx(res.parameterization[1], 1));
  EXPECT_TRUE(approx(res.parameterization[2], 0));
  EXPECT_TRUE(approx(res.parameterization[3], 0));

  p = Vector3d(0.5, -0.5, 0.5);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 5);
  EXPECT_TRUE(approx(res.sqr_distance, 0.25));
  EXPECT_TRUE(approx(res.parameterization[0], 0.5));
  EXPECT_TRUE(approx(res.parameterization[1], 0));
  EXPECT_TRUE(approx(res.parameterization[2], 0.5));
  EXPECT_TRUE(approx(res.parameterization[3], 0));

  p = Vector3d(0.5, 1.5, 0.5);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 10);
  EXPECT_TRUE(approx(res.sqr_distance, 0.25));
  EXPECT_TRUE(approx(res.parameterization[0], 0));
  EXPECT_TRUE(approx(res.parameterization[1], 0.5));
  EXPECT_TRUE(approx(res.parameterization[2], 0));
  EXPECT_TRUE(approx(res.parameterization[3], 0.5));

  p = Vector3d(1.5, 0.5, 0.5);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 12);
  EXPECT_TRUE(approx(res.sqr_distance, 0.25));
  EXPECT_TRUE(approx(res.parameterization[0], 0));
  EXPECT_TRUE(approx(res.parameterization[1], 0));
  EXPECT_TRUE(approx(res.parameterization[2], 0.5));
  EXPECT_TRUE(approx(res.parameterization[3], 0.5));
    
  p = Vector3d(-0.5, 0.5, 0.5);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 3);
  EXPECT_TRUE(approx(res.sqr_distance, 0.25));
  EXPECT_TRUE(approx(res.parameterization[0], 0.5));
  EXPECT_TRUE(approx(res.parameterization[1], 0.5));
  EXPECT_TRUE(approx(res.parameterization[2], 0));
  EXPECT_TRUE(approx(res.parameterization[3], 0));

  p = Vector3d(0.5, 0.5, 1.5);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 9);
  EXPECT_TRUE(approx(res.sqr_distance, 0.25));
  EXPECT_TRUE(approx(res.parameterization[0], 0.5));
  EXPECT_TRUE(approx(res.parameterization[1], 0));
  EXPECT_TRUE(approx(res.parameterization[2], 0));
  EXPECT_TRUE(approx(res.parameterization[3], 0.5));
    
  p = Vector3d(0.5, 0.5, -0.5);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 6);
  EXPECT_TRUE(approx(res.sqr_distance, 0.25));
  EXPECT_TRUE(approx(res.parameterization[0], 0));
  EXPECT_TRUE(approx(res.parameterization[1], 0.5));
  EXPECT_TRUE(approx(res.parameterization[2], 0.5));
  EXPECT_TRUE(approx(res.parameterization[3], 0));

}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

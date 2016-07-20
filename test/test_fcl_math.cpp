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


#define BOOST_TEST_MODULE "FCL_MATH"
#include <boost/test/unit_test.hpp>

#if FCL_HAVE_SSE
  #include "fcl/simd/math_simd_details.h"
#endif
#include "fcl/math/vec_3f.h"
#include "fcl/math/matrix_3f.h"
#include "fcl/broadphase/morton.h"
#include "fcl/config.h"

using namespace fcl;

BOOST_AUTO_TEST_CASE(vec_test_basic_vec32)
{
  typedef Vec3fX<details::Vec3Data<float> > Vec3f32;
  Vec3f32 v1(1.0f, 2.0f, 3.0f);
  BOOST_CHECK(v1[0] == 1.0f);
  BOOST_CHECK(v1[1] == 2.0f);
  BOOST_CHECK(v1[2] == 3.0f);

  Vec3f32 v2 = v1;
  Vec3f32 v3(3.3f, 4.3f, 5.3f);
  v1 += v3;
  BOOST_CHECK(v1.equal(v2 + v3));
  v1 -= v3;
  BOOST_CHECK(v1.equal(v2));
  v1 -= v3;
  BOOST_CHECK(v1.equal(v2 - v3));
  v1 += v3;

  v1 *= v3;
  BOOST_CHECK(v1.equal(v2 * v3));
  v1 /= v3;
  BOOST_CHECK(v1.equal(v2));
  v1 /= v3;
  BOOST_CHECK(v1.equal(v2 / v3));
  v1 *= v3;

  v1 *= 2.0f;
  BOOST_CHECK(v1.equal(v2 * 2.0f));
  v1 /= 2.0f;
  BOOST_CHECK(v1.equal(v2));
  v1 /= 2.0f;
  BOOST_CHECK(v1.equal(v2 / 2.0f));
  v1 *= 2.0f;

  v1 += 2.0f;
  BOOST_CHECK(v1.equal(v2 + 2.0f));
  v1 -= 2.0f;
  BOOST_CHECK(v1.equal(v2));
  v1 -= 2.0f;
  BOOST_CHECK(v1.equal(v2 - 2.0f));
  v1 += 2.0f;
  
  BOOST_CHECK((-Vec3f32(1.0f, 2.0f, 3.0f)).equal(Vec3f32(-1.0f, -2.0f, -3.0f)));

  v1 = Vec3f32(1.0f, 2.0f, 3.0f);
  v2 = Vec3f32(3.0f, 4.0f, 5.0f);
  BOOST_CHECK((v1.cross(v2)).equal(Vec3f32(-2.0f, 4.0f, -2.0f)));
  BOOST_CHECK(std::abs(v1.dot(v2) - 26) < 1e-5);

  v1 = Vec3f32(3.0f, 4.0f, 5.0f);
  BOOST_CHECK(std::abs(v1.sqrLength() - 50.0) < 1e-5);
  BOOST_CHECK(std::abs(v1.length() - sqrt(50.0)) < 1e-5);
  BOOST_CHECK(normalize(v1).equal(v1 / v1.length()));
}

BOOST_AUTO_TEST_CASE(vec_test_basic_vec64)
{
  typedef Vec3fX<details::Vec3Data<double> > Vec3f64;
  Vec3f64 v1(1.0, 2.0, 3.0);
  BOOST_CHECK(v1[0] == 1.0);
  BOOST_CHECK(v1[1] == 2.0);
  BOOST_CHECK(v1[2] == 3.0);

  Vec3f64 v2 = v1;
  Vec3f64 v3(3.3, 4.3, 5.3);
  v1 += v3;
  BOOST_CHECK(v1.equal(v2 + v3));
  v1 -= v3;
  BOOST_CHECK(v1.equal(v2));
  v1 -= v3;
  BOOST_CHECK(v1.equal(v2 - v3));
  v1 += v3;

  v1 *= v3;
  BOOST_CHECK(v1.equal(v2 * v3));
  v1 /= v3;
  BOOST_CHECK(v1.equal(v2));
  v1 /= v3;
  BOOST_CHECK(v1.equal(v2 / v3));
  v1 *= v3;

  v1 *= 2.0;
  BOOST_CHECK(v1.equal(v2 * 2.0));
  v1 /= 2.0;
  BOOST_CHECK(v1.equal(v2));
  v1 /= 2.0;
  BOOST_CHECK(v1.equal(v2 / 2.0));
  v1 *= 2.0;

  v1 += 2.0;
  BOOST_CHECK(v1.equal(v2 + 2.0));
  v1 -= 2.0;
  BOOST_CHECK(v1.equal(v2));
  v1 -= 2.0;
  BOOST_CHECK(v1.equal(v2 - 2.0));
  v1 += 2.0;

  BOOST_CHECK((-Vec3f64(1.0, 2.0, 3.0)).equal(Vec3f64(-1.0, -2.0, -3.0)));

  v1 = Vec3f64(1.0, 2.0, 3.0);
  v2 = Vec3f64(3.0, 4.0, 5.0);
  BOOST_CHECK((v1.cross(v2)).equal(Vec3f64(-2.0, 4.0, -2.0)));
  BOOST_CHECK(std::abs(v1.dot(v2) - 26) < 1e-5);

  v1 = Vec3f64(3.0, 4.0, 5.0);
  BOOST_CHECK(std::abs(v1.sqrLength() - 50.0) < 1e-5);
  BOOST_CHECK(std::abs(v1.length() - sqrt(50.0)) < 1e-5);
  BOOST_CHECK(normalize(v1).equal(v1 / v1.length()));


  v1 = Vec3f64(1.0, 2.0, 3.0);
  v2 = Vec3f64(3.0, 4.0, 5.0);
  BOOST_CHECK((v1.cross(v2)).equal(Vec3f64(-2.0, 4.0, -2.0)));
  BOOST_CHECK(v1.dot(v2) == 26);
}

#if FCL_HAVE_SSE

BOOST_AUTO_TEST_CASE(vec_test_sse_vec32)
{
  typedef Vec3fX<details::sse_meta_f4> Vec3f32;
  Vec3f32 v1(1.0f, 2.0f, 3.0f);
  BOOST_CHECK(v1[0] == 1.0f);
  BOOST_CHECK(v1[1] == 2.0f);
  BOOST_CHECK(v1[2] == 3.0f);

  Vec3f32 v2 = v1;
  Vec3f32 v3(3.3f, 4.3f, 5.3f);
  v1 += v3;
  BOOST_CHECK(v1.equal(v2 + v3));
  v1 -= v3;
  BOOST_CHECK(v1.equal(v2));
  v1 -= v3;
  BOOST_CHECK(v1.equal(v2 - v3));
  v1 += v3;

  v1 *= v3;
  BOOST_CHECK(v1.equal(v2 * v3));
  v1 /= v3;
  BOOST_CHECK(v1.equal(v2));
  v1 /= v3;
  BOOST_CHECK(v1.equal(v2 / v3));
  v1 *= v3;

  v1 *= 2.0f;
  BOOST_CHECK(v1.equal(v2 * 2.0f));
  v1 /= 2.0f;
  BOOST_CHECK(v1.equal(v2));
  v1 /= 2.0f;
  BOOST_CHECK(v1.equal(v2 / 2.0f));
  v1 *= 2.0f;

  v1 += 2.0f;
  BOOST_CHECK(v1.equal(v2 + 2.0f));
  v1 -= 2.0f;
  BOOST_CHECK(v1.equal(v2));
  v1 -= 2.0f;
  BOOST_CHECK(v1.equal(v2 - 2.0f));
  v1 += 2.0f;
  
  BOOST_CHECK((-Vec3f32(1.0f, 2.0f, 3.0f)).equal(Vec3f32(-1.0f, -2.0f, -3.0f)));

  v1 = Vec3f32(1.0f, 2.0f, 3.0f);
  v2 = Vec3f32(3.0f, 4.0f, 5.0f);
  BOOST_CHECK((v1.cross(v2)).equal(Vec3f32(-2.0f, 4.0f, -2.0f)));
  BOOST_CHECK(std::abs(v1.dot(v2) - 26) < 1e-5);

  v1 = Vec3f32(3.0f, 4.0f, 5.0f);
  BOOST_CHECK(std::abs(v1.sqrLength() - 50) < 1e-5);
  BOOST_CHECK(std::abs(v1.length() - sqrt(50)) < 1e-5);
  BOOST_CHECK(normalize(v1).equal(v1 / v1.length()));
}

BOOST_AUTO_TEST_CASE(vec_test_sse_vec64)
{
  typedef Vec3fX<details::sse_meta_d4> Vec3f64;
  Vec3f64 v1(1.0, 2.0, 3.0);
  BOOST_CHECK(v1[0] == 1.0);
  BOOST_CHECK(v1[1] == 2.0);
  BOOST_CHECK(v1[2] == 3.0);

  Vec3f64 v2 = v1;
  Vec3f64 v3(3.3, 4.3, 5.3);
  v1 += v3;
  BOOST_CHECK(v1.equal(v2 + v3));
  v1 -= v3;
  BOOST_CHECK(v1.equal(v2));
  v1 -= v3;
  BOOST_CHECK(v1.equal(v2 - v3));
  v1 += v3;

  v1 *= v3;
  BOOST_CHECK(v1.equal(v2 * v3));
  v1 /= v3;
  BOOST_CHECK(v1.equal(v2));
  v1 /= v3;
  BOOST_CHECK(v1.equal(v2 / v3));
  v1 *= v3;

  v1 *= 2.0;
  BOOST_CHECK(v1.equal(v2 * 2.0));
  v1 /= 2.0;
  BOOST_CHECK(v1.equal(v2));
  v1 /= 2.0;
  BOOST_CHECK(v1.equal(v2 / 2.0));
  v1 *= 2.0;

  v1 += 2.0;
  BOOST_CHECK(v1.equal(v2 + 2.0));
  v1 -= 2.0;
  BOOST_CHECK(v1.equal(v2));
  v1 -= 2.0;
  BOOST_CHECK(v1.equal(v2 - 2.0));
  v1 += 2.0;

  BOOST_CHECK((-Vec3f64(1.0, 2.0, 3.0)).equal(Vec3f64(-1.0, -2.0, -3.0)));

  v1 = Vec3f64(1.0, 2.0, 3.0);
  v2 = Vec3f64(3.0, 4.0, 5.0);
  BOOST_CHECK((v1.cross(v2)).equal(Vec3f64(-2.0, 4.0, -2.0)));
  BOOST_CHECK(std::abs(v1.dot(v2) - 26) < 1e-5);

  v1 = Vec3f64(3.0, 4.0, 5.0);
  BOOST_CHECK(std::abs(v1.sqrLength() - 50) < 1e-5);
  BOOST_CHECK(std::abs(v1.length() - sqrt(50)) < 1e-5);
  BOOST_CHECK(normalize(v1).equal(v1 / v1.length()));


  v1 = Vec3f64(1.0, 2.0, 3.0);
  v2 = Vec3f64(3.0, 4.0, 5.0);
  BOOST_CHECK((v1.cross(v2)).equal(Vec3f64(-2.0, 4.0, -2.0)));
  BOOST_CHECK(v1.dot(v2) == 26);
}

BOOST_AUTO_TEST_CASE(sse_mat32_consistent)
{
  typedef Vec3fX<details::Vec3Data<float> > Vec3f32;
  typedef Vec3fX<details::sse_meta_f4> Vec3f32SSE;

  typedef Matrix3fX<details::Matrix3Data<float> > Matrix3f32;
  typedef Matrix3fX<details::sse_meta_f12> Matrix3f32SSE;

  Vec3f32 v1(1, 2, 3);
  Vec3f32SSE v2(1, 2, 3);

  Matrix3f32 m1(-1, 3, -3, 0, -6, 6, -5, -3, 1);
  Matrix3f32SSE m2(-1, 3, -3, 0, -6, 6, -5, -3, 1);

  for(size_t i = 0; i < 3; ++i)
    for(size_t j = 0; j < 3; ++j)
      BOOST_CHECK((m1(i, j) - m2(i, j) < 1e-1));
  
  Matrix3f32 m3(transpose(m1));
  Matrix3f32SSE m4(transpose(m2));
        
  for(size_t i = 0; i < 3; ++i)
    for(size_t j = 0; j < 3; ++j)
      BOOST_CHECK((m3(i, j) - m4(i, j) < 1e-1));

  m3 = m1; m3.transpose();
  m4 = m2; m4.transpose();

  for(size_t i = 0; i < 3; ++i)
    for(size_t j = 0; j < 3; ++j)
      BOOST_CHECK((m3(i, j) - m4(i, j) < 1e-1));

  m3 = inverse(m1);
  m4 = inverse(m2);
  
  for(size_t i = 0; i < 3; ++i)
    for(size_t j = 0; j < 3; ++j)
      BOOST_CHECK((m3(i, j) - m4(i, j) < 1e-1));

  m3 = m1; m3.inverse();
  m4 = m2; m4.inverse();

  for(size_t i = 0; i < 3; ++i)
    for(size_t j = 0; j < 3; ++j)
      BOOST_CHECK((m3(i, j) - m4(i, j) < 1e-1));
}

BOOST_AUTO_TEST_CASE(vec_test_sse_vec32_consistent)
{
  typedef Vec3fX<details::Vec3Data<float> > Vec3f32;
  typedef Vec3fX<details::sse_meta_f4> Vec3f32SSE;

  Vec3f32 v1(3.4f, 4.2f, 10.5f), v2(3.1f, 0.1f, -50.4f);
  Vec3f32SSE v3(3.4f, 4.2f, 10.5f), v4(3.1f, 0.1f, -50.4f);
  Vec3f32 v12 = v1 + v2;
  Vec3f32SSE v34 = v3 + v4;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1 - v2;
  v34 = v3 - v4;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1 * v2;
  v34 = v3 * v4;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1 / v2;
  v34 = v3 / v4;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  float t = 1234.433f;
  v12 = v1 + t;
  v34 = v3 + t;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1 - t;
  v34 = v3 - t;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1 * t;
  v34 = v3 * t;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1 / t;
  v34 = v3 / t;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);

  v12 = v1; v12 += v2;
  v34 = v3; v34 += v4;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1; v12 -= v2;
  v34 = v3; v34 -= v4;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1; v12 *= v2;
  v34 = v3; v34 *= v4;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1; v12 /= v2;
  v34 = v3; v34 /= v4;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1; v12 += t;
  v34 = v3; v34 += t;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1; v12 -= t;
  v34 = v3; v34 -= t;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1; v12 *= t;
  v34 = v3; v34 *= t;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1; v12 /= t;
  v34 = v3; v34 /= t;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);

  v12 = -v1;
  v34 = -v3;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);

  v12 = v1.cross(v2);
  v34 = v3.cross(v4);
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);

  BOOST_CHECK(std::abs(v1.dot(v2) - v3.dot(v4)) < 1e-5);

  v12 = min(v1, v2);
  v34 = min(v3, v4);
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = max(v1, v2);
  v34 = max(v3, v4);
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);

  v12 = abs(v2);
  v34 = abs(v4);
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);

  Vec3f32 delta1(1e-9f, 1e-9f, 1e-9f);
  Vec3f32SSE delta2(1e-9f, 1e-9f, 1e-9f);
  BOOST_CHECK((v1 + delta1).equal(v1));
  BOOST_CHECK((v3 + delta2).equal(v3));

  BOOST_CHECK(std::abs(v1.length() - v3.length()) < 1e-5);
  BOOST_CHECK(std::abs(v1.sqrLength() - v3.sqrLength()) < 1e-5);
 
  v12 = v1; v12.negate();
  v34 = v3; v34.negate();
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);

  v12 = v1; v12.normalize();
  v34 = v3; v34.normalize();
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  
  v12 = normalize(v1);
  v34 = normalize(v3);
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);  
}

BOOST_AUTO_TEST_CASE(vec_test_sse_vec64_consistent)
{
  typedef Vec3fX<details::Vec3Data<double> > Vec3f64;
  typedef Vec3fX<details::sse_meta_d4> Vec3f64SSE;

  Vec3f64 v1(3.4, 4.2, 10.5), v2(3.1, 0.1, -50.4);
  Vec3f64SSE v3(3.4, 4.2, 10.5), v4(3.1, 0.1, -50.4);
  Vec3f64 v12 = v1 + v2;
  Vec3f64SSE v34 = v3 + v4;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1 - v2;
  v34 = v3 - v4;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1 * v2;
  v34 = v3 * v4;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1 / v2;
  v34 = v3 / v4;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  double t = 1234.433;
  v12 = v1 + t;
  v34 = v3 + t;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1 - t;
  v34 = v3 - t;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1 * t;
  v34 = v3 * t;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1 / t;
  v34 = v3 / t;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);

  v12 = v1; v12 += v2;
  v34 = v3; v34 += v4;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1; v12 -= v2;
  v34 = v3; v34 -= v4;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1; v12 *= v2;
  v34 = v3; v34 *= v4;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1; v12 /= v2;
  v34 = v3; v34 /= v4;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1; v12 += t;
  v34 = v3; v34 += t;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1; v12 -= t;
  v34 = v3; v34 -= t;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1; v12 *= t;
  v34 = v3; v34 *= t;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = v1; v12 /= t;
  v34 = v3; v34 /= t;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);

  v12 = -v1;
  v34 = -v3;
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);

  v12 = v1.cross(v2);
  v34 = v3.cross(v4);
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);

  BOOST_CHECK(std::abs(v1.dot(v2) - v3.dot(v4)) < 1e-5);

  v12 = min(v1, v2);
  v34 = min(v3, v4);
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  v12 = max(v1, v2);
  v34 = max(v3, v4);
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);

  v12 = abs(v2);
  v34 = abs(v4);
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);

  Vec3f64 delta1(1e-15, 1e-15, 1e-15);
  Vec3f64SSE delta2(1e-15, 1e-15, 1e-15);
  BOOST_CHECK((v1 + delta1).equal(v1));
  BOOST_CHECK((v3 + delta2).equal(v3));

  BOOST_CHECK(std::abs(v1.length() - v3.length()) < 1e-5);
  BOOST_CHECK(std::abs(v1.sqrLength() - v3.sqrLength()) < 1e-5);
 
  v12 = v1; v12.negate();
  v34 = v3; v34.negate();
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);

  v12 = v1; v12.normalize();
  v34 = v3; v34.normalize();
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);
  
  v12 = normalize(v1);
  v34 = normalize(v3);
  BOOST_CHECK(std::abs(v12[0] - v34[0]) < 1e-5);
  BOOST_CHECK(std::abs(v12[1] - v34[1]) < 1e-5);
  BOOST_CHECK(std::abs(v12[2] - v34[2]) < 1e-5);  
}

#endif

BOOST_AUTO_TEST_CASE(morton)
{
  AABB bbox(Vec3f(0, 0, 0), Vec3f(1000, 1000, 1000));
  morton_functor<std::bitset<30>> F1(bbox);
  morton_functor<std::bitset<60>> F2(bbox);
  morton_functor<FCL_UINT64> F3(bbox); // 60 bits
  morton_functor<FCL_UINT32> F4(bbox); // 30 bits

  Vec3f p(254, 873, 674);

  BOOST_CHECK(F1(p).to_ulong() == F4(p));
  BOOST_CHECK(F2(p).to_ullong() == F3(p));
}

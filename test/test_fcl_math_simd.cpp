/*
 * Software License Agreement (BSD License)
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

#include "fcl/common/types.h"
#include "fcl/math/math_simd_details.h"

using namespace fcl;
using namespace Eigen;

template <typename S>
inline fcl::Matrix3<S> gen_random_matrix3()
{
  return Eigen::Matrix<S, 3, 3>::Random();
}

template <typename S>
inline fcl::Vector3<S> gen_random_vector3()
{
  return Eigen::Matrix<S, 3, 1>::Random();
}

#ifdef FCL_SSE_ENABLED
template <typename S>
void vector3_to_simd(__m128& dst, const fcl::Vector3<S>& src)
{
  dst = _mm_setr_ps(src[0], src[1], src[2], 0.f);
}

template <typename S>
void simd_to_vector3(fcl::Vector3<S>& dst, const __m128& src)
{
  dst << src[0], src[1], src[2];
}

template <typename S>
void matrix3_to_simd(__m128 dst[3], const fcl::Matrix3<S>& src)
{
  for (auto i = 0; i < 3; i++) {
    dst[i] = _mm_setr_ps(src(i, 0), src(i, 1), src(i, 2), 0.f);
  }
}

template <typename S>
void simd_to_matrix3(fcl::Matrix3<S>& dst, const __m128 src[3])
{
  dst << src[0][0], src[0][1], src[0][2],
  src[1][0], src[1][1], src[1][2],
  src[2][0], src[2][1], src[2][2];
}
#endif

#ifdef FCL_AVX_ENABLED
template <typename S>
void vector3_to_simd(__m256d& dst, const fcl::Vector3<S>& src)
{
  dst = _mm256_setr_pd(src[0], src[1], src[2], 0);
}

template <typename S>
void simd_to_vector3(fcl::Vector3<S>& dst, const __m256d& src)
{
  dst << src[0], src[1], src[2];
}

template <typename S>
void matrix3_to_simd(__m256d dst[3], const fcl::Matrix3<S>& src)
{
  for (auto i = 0; i < 3; i++) {
    dst[i] = _mm256_setr_pd(src(i, 0), src(i, 1), src(i, 2), 0);
  }
}

template <typename S>
void simd_to_matrix3(fcl::Matrix3<S>& dst, const __m256d src[3])
{
  dst << src[0][0], src[0][1], src[0][2],
  src[1][0], src[1][1], src[1][2],
  src[2][0], src[2][1], src[2][2];
}
#endif

template <typename S>
void test_sse_abs()
{
#ifdef FCL_SSE_ENABLED
  __m128 x_simd;
  fcl::Vector3<S> x = gen_random_vector3<S>();
  vector3_to_simd(x_simd, x);

  __m128 result_simd = fcl::details::abs_ps(x_simd);

  fcl::Vector3<S> result;
  simd_to_vector3(result, result_simd);
  EXPECT_TRUE((x.cwiseAbs()).isApprox(result));
#endif
}

template <typename S>
void test_avx_abs()
{
#ifdef FCL_AVX_ENABLED
  __m256d x_simd;
  fcl::Vector3<S> x = gen_random_vector3<S>();
  vector3_to_simd(x_simd, x);

  __m256d result_simd = fcl::details::abs_pd(x_simd);

  fcl::Vector3<S> result;
  simd_to_vector3(result, result_simd);
  EXPECT_TRUE((x.cwiseAbs()).isApprox(result));
#endif
}

GTEST_TEST(FCL_MATH_SIMD, abs) {
  test_sse_abs<float>();
  test_avx_abs<float>();
  test_avx_abs<double>();
}

template <typename S>
void test_sse_allzero()
{
#ifdef FCL_SSE_ENABLED
  __m128 x_simd;
  fcl::Vector3<S> x = gen_random_vector3<S>();
  vector3_to_simd(x_simd, x);
  EXPECT_TRUE(x.isZero(0) == fcl::details::allzero_ps(x_simd));

  __m128 zero_simd = _mm_set1_ps(0.f);
  EXPECT_TRUE(fcl::details::allzero_ps(zero_simd));
#endif
}

template <typename S>
void test_avx_allzero()
{
#ifdef FCL_AVX_ENABLED
  __m256d x_simd;
  fcl::Vector3<S> x = gen_random_vector3<S>();
  vector3_to_simd(x_simd, x);
  EXPECT_TRUE(x.isZero(0) == fcl::details::allzero_pd(x_simd));

  __m256d zero_simd = _mm256_set1_pd(0);
  EXPECT_TRUE(fcl::details::allzero_pd(zero_simd));
#endif
}

GTEST_TEST(FCL_MATH_SIMD, allzero) {
  test_sse_allzero<float>();
  test_avx_allzero<float>();
  test_avx_allzero<double>();
}

template <typename S>
void test_sse_any_gt()
{
#ifdef FCL_SSE_ENABLED
  __m128 lhs_simd;
  __m128 rhs_simd;
  fcl::Vector3<S> lhs = gen_random_vector3<S>();
  vector3_to_simd(lhs_simd, lhs);
  fcl::Vector3<S> rhs = gen_random_vector3<S>();
  vector3_to_simd(rhs_simd, rhs);
  EXPECT_TRUE(((lhs - rhs).array() > 0).any() == fcl::details::any_gt_ps(lhs_simd, rhs_simd));
#endif
}

template <typename S>
void test_avx_any_gt()
{
#ifdef FCL_AVX_ENABLED
  __m256d lhs_simd;
  __m256d rhs_simd;
  fcl::Vector3<S> lhs = gen_random_vector3<S>();
  vector3_to_simd(lhs_simd, lhs);
  fcl::Vector3<S> rhs = gen_random_vector3<S>();
  vector3_to_simd(rhs_simd, rhs);
  EXPECT_TRUE(((lhs - rhs).array() > 0).any() == fcl::details::any_gt_pd(lhs_simd, rhs_simd));
#endif
}

GTEST_TEST(FCL_MATH_SIMD, any_gt) {
  test_sse_any_gt<float>();
  test_avx_any_gt<float>();
  test_avx_any_gt<double>();
}

#if defined (FCL_SSE_ENABLED) or defined(FCL_AVX_ENABLED)
template <typename S, typename T>
void test_mat3_transpose(T src[3], T dst[3])
{
  fcl::Matrix3<S> src_mat = gen_random_matrix3<S>();
  matrix3_to_simd(src, src_mat);

  fcl::details::mat3x4_transpose(src, dst);

  fcl::Matrix3<S> dst_mat;
  simd_to_matrix3(dst_mat, dst);
  EXPECT_TRUE(src_mat.transpose().isApprox(dst_mat));
}
#endif

GTEST_TEST(FCL_MATH_SIMD, mat3_transpose) {
#ifdef FCL_SSE_ENABLED
  {
    __m128 src[3];
    __m128 dst[3];
    test_mat3_transpose<float>(src, dst);
  }
#endif

#ifdef FCL_AVX_ENABLED
  {
    __m256d src[3];
    __m256d dst[3];
    test_mat3_transpose<float>(src, dst);
    test_mat3_transpose<double>(src, dst);
  }
#endif
}

#if defined (FCL_SSE_ENABLED) or defined(FCL_AVX_ENABLED)
template <typename S, typename T>
void test_mat3_mul_vec3(T matrix_simd[3], T& vector_simd)
{
  fcl::Matrix3<S> matrix = gen_random_matrix3<S>();
  matrix3_to_simd(matrix_simd, matrix);
  fcl::Vector3<S> vector = gen_random_vector3<S>();
  vector3_to_simd(vector_simd, vector);

  T result_simd = fcl::details::mat3x4_mul_vec4(matrix_simd, vector_simd);

  fcl::Vector3<S> result;
  simd_to_vector3(result, result_simd);
  EXPECT_TRUE((matrix * vector).isApprox(result));
}
#endif

GTEST_TEST(FCL_MATH_SIMD, mat3x4_mul_vec4) {
#ifdef FCL_SSE_ENABLED
  {
    __m128 matrix[3];
    __m128 vector;
    test_mat3_mul_vec3<float>(matrix, vector);
  }
#endif

#ifdef FCL_AVX_ENABLED
  {
    __m256d matrix[3];
    __m256d vector;
    test_mat3_mul_vec3<float>(matrix, vector);
    test_mat3_mul_vec3<double>(matrix, vector);
  }
#endif
}

#if defined (FCL_SSE_ENABLED) or defined(FCL_AVX_ENABLED)
template <typename S, typename T>
void test_transp_mat3_mul_vec3(T matrix_simd[3], T& vector_simd)
{
  fcl::Matrix3<S> matrix = gen_random_matrix3<S>();
  matrix3_to_simd(matrix_simd, matrix);
  fcl::Vector3<S> vector = gen_random_vector3<S>();
  vector3_to_simd(vector_simd, vector);

  T result_simd = fcl::details::transp_mat3x4_mul_vec4(matrix_simd, vector_simd);

  fcl::Vector3<S> result;
  simd_to_vector3(result, result_simd);
  EXPECT_TRUE((matrix.transpose() * vector).isApprox(result));
}
#endif

GTEST_TEST(FCL_MATH_SIMD, transp_mat3x4_mul_vec4) {
#ifdef FCL_SSE_ENABLED
  {
    __m128 matrix[3];
    __m128 vector;
    test_transp_mat3_mul_vec3<float>(matrix, vector);
  }
#endif

#ifdef FCL_AVX_ENABLED
  {
    __m256d matrix[3];
    __m256d vector;
    test_transp_mat3_mul_vec3<float>(matrix, vector);
    test_transp_mat3_mul_vec3<double>(matrix, vector);
  }
#endif
}

#if defined (FCL_SSE_ENABLED) or defined(FCL_AVX_ENABLED)
template <typename S, typename T>
void test_mat3_mul_mat3(T m1_simd[3], T m2_simd[3])
{
  fcl::Matrix3<S> m1 = gen_random_matrix3<S>();
  matrix3_to_simd(m1_simd, m1);
  fcl::Matrix3<S> m2 = gen_random_matrix3<S>();
  matrix3_to_simd(m2_simd, m2);

  T out_simd[3];
  fcl::details::mat3x3_mul_mat3x3(out_simd, m1_simd, m2_simd);

  fcl::Matrix3<S> out;
  simd_to_matrix3(out, out_simd);
  EXPECT_TRUE((m1 * m2).isApprox(out));
}
#endif

GTEST_TEST(FCL_MATH_SIMD, mat3x3_mul_mat3x3) {
#ifdef FCL_SSE_ENABLED
  {
    __m128 m1[3];
    __m128 m2[3];
    test_mat3_mul_mat3<float>(m1, m2);
  }
#endif

#ifdef FCL_AVX_ENABLED
  {
    __m256d m1[3];
    __m256d m2[3];
    test_mat3_mul_mat3<float>(m1, m2);
    test_mat3_mul_mat3<double>(m1, m2);
  }
#endif
}

#if defined (FCL_SSE_ENABLED) or defined(FCL_AVX_ENABLED)
template <typename S, typename T>
void test_transp_mat3_mul_mat3(T m1_simd[3], T m2_simd[3])
{
  fcl::Matrix3<S> m1 = gen_random_matrix3<S>();
  matrix3_to_simd(m1_simd, m1);
  fcl::Matrix3<S> m2 = gen_random_matrix3<S>();
  matrix3_to_simd(m2_simd, m2);

  T out_simd[3];
  fcl::details::transp_mat3x3_mul_mat3x3(out_simd, m1_simd, m2_simd);

  fcl::Matrix3<S> out;
  simd_to_matrix3(out, out_simd);
  EXPECT_TRUE((m1.transpose() * m2).isApprox(out));
}
#endif

GTEST_TEST(FCL_MATH_SIMD, transp_mat3x3_mul_mat3x3) {
#ifdef FCL_SSE_ENABLED
  {
    __m128 m1[3];
    __m128 m2[3];
    test_transp_mat3_mul_mat3<float>(m1, m2);
  }
#endif

#ifdef FCL_AVX_ENABLED
  {
    __m256d m1[3];
    __m256d m2[3];
    test_transp_mat3_mul_mat3<float>(m1, m2);
    test_transp_mat3_mul_mat3<double>(m1, m2);
  }
#endif
}

#if defined (FCL_SSE_ENABLED) or defined(FCL_AVX_ENABLED)
template <typename S, typename T>
void test_mat3_mul_transp_mat3(T m1_simd[3], T m2_simd[3])
{
  fcl::Matrix3<S> m1 = gen_random_matrix3<S>();
  matrix3_to_simd(m1_simd, m1);
  fcl::Matrix3<S> m2 = gen_random_matrix3<S>();
  matrix3_to_simd(m2_simd, m2);

  T out_simd[3];
  fcl::details::mat3x3_mul_transp_mat3x3(out_simd, m1_simd, m2_simd);

  fcl::Matrix3<S> out;
  simd_to_matrix3(out, out_simd);
  EXPECT_TRUE((m1 * m2.transpose()).isApprox(out));
}
#endif

GTEST_TEST(FCL_MATH_SIMD, mat3x3_mul_transp_mat3x3) {
#ifdef FCL_SSE_ENABLED
  {
    __m128 m1[3];
    __m128 m2[3];
    test_mat3_mul_transp_mat3<float>(m1, m2);
  }
#endif

#ifdef FCL_AVX_ENABLED
  {
    __m256d m1[3];
    __m256d m2[3];
    test_mat3_mul_transp_mat3<float>(m1, m2);
    test_mat3_mul_transp_mat3<double>(m1, m2);
  }
#endif
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

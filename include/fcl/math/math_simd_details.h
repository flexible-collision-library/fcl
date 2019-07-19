/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
 *  Copyright (c) 2019,      Dorabot, Inc.
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

/** @author Shuai YUAN */


#ifndef FCL_MATH_SIMD_DETAILS_H
#define FCL_MATH_SIMD_DETAILS_H

// NOTE: some compilers might not define __SSE4__ macro even if SSE4 is supported.
// Check with following command:
// $ cc -march=native -dM -E - < /dev/null | egrep "SSE|AVX"
#if not defined(__SSE4__)
#if defined(__SSE4_1__) or defined(__SSE4_2__)
#define __SSE4__
#endif
#endif

#include <immintrin.h>

#if defined (__SSE__) or defined(__SSE2__) or defined(__SSE3__) or defined(__SSE4__)
#define FCL_SSE_ENABLED
#endif

#if defined(__AVX__) or defined(__AVX2__)
#define FCL_AVX_ENABLED
#endif

#if defined(__FMA__) or defined(__AVX2__)
#define FCL_FMA_ENABLED
#endif

namespace fcl
{

namespace details
{


//==============================================================================
/// @brief Broadcast one element of a SIMD register to another SIMD register
///
/// \param[in] v: vector of 4x1
/// [ v0 v1 v2 v3 ]
///
/// \param[in] i: the index of element to be broadcased
///
/// \return a SIMD register which stores the result 4x1 vector
/// [ v[i] v[i] v[i] v[i] ]
#ifdef FCL_SSE_ENABLED
inline __m128 vec_splat_ps(const __m128 v, const int i)
{
  return _mm_set1_ps(v[i]);
}
#endif

#ifdef FCL_AVX_ENABLED
inline __m256d vec_splat_pd(const __m256d& v, const int i)
{
  return _mm256_set1_pd(v[i]);
}
#endif


//==============================================================================
#ifdef FCL_SSE_ENABLED
inline __m128 abs_ps(const __m128& x)
{
  static const __m128 sign_mask = _mm_set1_ps(-0.f); // -0.f = 1 << 31
  return _mm_andnot_ps(sign_mask, x);
}
#endif

#ifdef FCL_AVX_ENABLED
inline __m256d abs_pd(const __m256d& x)
{
  static const __m256d sign_mask = _mm256_castsi256_pd(_mm256_set1_epi64x(0x7FFFFFFFFFFFFFFF));
  return _mm256_and_pd(sign_mask, x);
}
#endif


//==============================================================================
/// @brief Fused multiply–accumulate operation
#ifdef FCL_SSE_ENABLED
inline __m128 fmadd_ps(const __m128& a, const __m128& b, const __m128& c)
{
#ifdef FCL_FMA_ENABLED
  return _mm_fmadd_ps(a, b, c);
#else
  return _mm_add_ps(_mm_mul_ps(a, b), c);
#endif
}

inline __m128 fmsub_ps(const __m128& a, const __m128& b, const __m128& c)
{
#ifdef FCL_FMA_ENABLED
  return _mm_fmsub_ps(a, b, c);
#else
  return _mm_sub_ps(_mm_mul_ps(a, b), c);
#endif
}
#endif

#ifdef FCL_AVX_ENABLED
inline __m256d fmadd_pd(const __m256d& a, const __m256d& b, const __m256d& c)
{
#ifdef FCL_FMA_ENABLED
  return _mm256_fmadd_pd(a, b, c);
#else
  return _mm256_add_pd(_mm256_mul_pd(a, b), c);
#endif
}

inline __m256d fmsub_pd(const __m256d& a, const __m256d& b, const __m256d& c)
{
#ifdef FCL_FMA_ENABLED
  return _mm256_fmsub_pd(a, b, c);
#else
  return _mm256_sub_pd(_mm256_mul_pd(a, b), c);
#endif
}
#endif


//==============================================================================
#ifdef FCL_SSE_ENABLED
inline bool allzero_ps(const __m128& x)
{
#if defined(__SSE4__)
  return _mm_testz_si128(_mm_castps_si128(x), _mm_castps_si128(x));
#else
  return (_mm_movemask_epi8(_mm_cmpeq_epi8(_mm_castps_si128(x), _mm_setzero_si128())) == 0xFFFF);
#endif
}
#endif

#ifdef FCL_AVX_ENABLED
inline bool allzero_pd(const __m256d& x)
{
  return _mm256_testz_si256(_mm256_castpd_si256((x)), _mm256_castpd_si256((x)));
}
#endif


//==============================================================================
#ifdef FCL_SSE_ENABLED
inline bool any_gt_ps(const __m128& lhs, const __m128& rhs)
{
  __m128 result = _mm_cmpgt_ps(lhs, rhs);
  return (!allzero_ps(result));
}
#endif

#ifdef FCL_AVX_ENABLED
inline bool any_gt_pd(const __m256d& lhs, const __m256d& rhs)
{
  return !allzero_pd(_mm256_cmp_pd(lhs, rhs, _CMP_GT_OQ));
}
#endif


//==============================================================================
/// @brief
/// Compute the transpose of M, where M is a 3x4 matrix denoted by an array of
/// 4 SIMD registers and its last column is a zero vector.
///
/// @param[in]  src
/// [ x0 y0 z0 0 ]
/// [ x1 y1 z1 0 ]
/// [ x2 y2 z2 0 ]
///
/// @param[out] dst
/// [ x0 x1 x2 0 ]
/// [ y0 y1 y2 0 ]
/// [ z0 z1 z2 0 ]
#ifdef FCL_SSE_ENABLED
static inline
void mat3x4_transpose(const __m128* src_r0, const __m128* src_r1, const __m128* src_r2, __m128* dst_r0, __m128* dst_r1, __m128* dst_r2)
{
  __m128 src3 = _mm_setzero_ps();
  __m128 tmp0 = _mm_unpacklo_ps(*src_r0, *src_r1);  // [ x0 x1 y0 y1 ]
  __m128 tmp2 = _mm_unpacklo_ps(*src_r2, src3);     // [ x2  0 y2  0 ]
  __m128 tmp1 = _mm_unpackhi_ps(*src_r0, *src_r1);  // [ z0 z1  0  0 ]
  __m128 tmp3 = _mm_unpackhi_ps(*src_r2, src3);     // [ z2  0  0  0 ]
  *dst_r0 = _mm_movelh_ps(tmp0, tmp2);              // [ x0 x1 x2  0 ]
  *dst_r1 = _mm_movehl_ps(tmp2, tmp0);              // [ y0 y1 y2  0 ]
  *dst_r2 = _mm_movelh_ps(tmp1, tmp3);              // [ z0 z1 z2  0 ]
}

static inline
void mat3x4_transpose(const __m128 src[3], __m128 dst[3])
{
  mat3x4_transpose(&src[0], &src[1], &src[2], &dst[0], &dst[1], &dst[2]);
}
#endif

#ifdef FCL_AVX_ENABLED
static inline
void mat3x4_transpose(const __m256d* src_r0, const __m256d* src_r1, const __m256d* src_r2, __m256d* dst_r0, __m256d* dst_r1, __m256d* dst_r2)
{
  __m256d src3 = _mm256_setzero_pd();
  __m256d tmp0 = _mm256_unpacklo_pd(*src_r0, *src_r1);  // [ x0 x1 z0 z1 ]
  __m256d tmp1 = _mm256_unpacklo_pd(*src_r2, src3);     // [ x2  0 z2  0 ]
  __m256d tmp2 = _mm256_unpackhi_pd(*src_r0, *src_r1);  // [ y0 y1  0  0 ]
  __m256d tmp3 = _mm256_unpackhi_pd(*src_r2, src3);     // [ y2  0  0  0 ]
  *dst_r0 = _mm256_permute2f128_pd(tmp0, tmp1, 0x20);   // [ x0 x1 x2  0 ]
  *dst_r1 = _mm256_permute2f128_pd(tmp2, tmp3, 0x20);   // [ y0 y1 y2  0 ]
  *dst_r2 = _mm256_permute2f128_pd(tmp0, tmp1, 0x31);   // [ z0 z1 z2  0 ]
}

static inline
void mat3x4_transpose(const __m256d src[3], __m256d dst[3])
{
  mat3x4_transpose(&src[0], &src[1], &src[2], &dst[0], &dst[1], &dst[2]);
}
#endif


//==============================================================================
/// @brief
/// Compute the product M*v, where
/// M is a 3x4 matrix denoted by an array of 4 SIMD registers and the last
/// column is a zero vector
/// v is a 4x1 vector with a zero-padding element, denoted by a SIMD register
///
/// \param[in] matrix of 3x4
/// [ x0 x1 x2 0 ]
/// [ y0 y1 y2 0 ]
/// [ z0 z1 z2 0 ]
///
/// \param[in] vector of 4x1
/// [ v0 v1 v2 0 ]
///
/// \return a SIMD register which stores the result 3x1 vector
/// and a zero-padding element which we don't care
/// [ x0v0+x1v1+x2v2 y0v0+y1v1+y2v2 z0v0+z1v1+z2v2 0 ]
#ifdef FCL_SSE_ENABLED
static inline
__m128 mat3x4_mul_vec4(const __m128* matrix_r0, const __m128* matrix_r1, const __m128* matrix_r2, const __m128* vector)
{
#if defined (__SSE4__)
  __m128 prod0 = _mm_dp_ps(*matrix_r0, *vector, 0xFF);  // [ x0v0+x1v1+x2v2 x0v0+x1v1+x2v2 x0v0+x1v1+x2v2 x0v0+x1v1+x2v2 ]
  __m128 prod1 = _mm_dp_ps(*matrix_r1, *vector, 0xFF);  // [ y0v0+y1v1+y2v2 y0v0+y1v1+y2v2 y0v0+y1v1+y2v2 y0v0+y1v1+y2v2 ]
  __m128 prod2 = _mm_dp_ps(*matrix_r2, *vector, 0xFF);  // [ z0v0+z1v1+z2v2 z0v0+z1v1+z2v2 z0v0+z1v1+z2v2 z0v0+z1v1+z2v2 ]
  __m128 prod3 = _mm_set1_ps(0.f);
  // shuffle lhs: [ x0v0+x1v1+x2v2 x0v0+x1v1+x2v2 y0v0+y1v1+y2v2 y0v0+y1v1+y2v2 ]
  // shuffle rhs: [ z0v0+z1v1+z2v2 z0v0+z1v1+z2v2 y0v0+y1v1+y2v2 y0v0+y1v1+y2v2 ]
  // shuffle result: [ x0v0+x1v1+x2v2 y0v0+y1v1+y2v2 z0v0+z1v1+z2v2 y0v0+y1v1+y2v2 ]
  return _mm_shuffle_ps(_mm_movelh_ps(prod0, prod1), _mm_movelh_ps(prod2, prod3), _MM_SHUFFLE(2, 0, 2, 0));
#else
  __m128 x = _mm_mul_ps(*matrix_r0, *vector);     // [ x0v0 x1v1 x2v2 0 ]
  __m128 y = _mm_mul_ps(*matrix_r1, *vector);     // [ y0v0 y1v1 y2v2 0 ]
  __m128 t0 = _mm_unpacklo_ps(x, y);              // [ x0v0 y0v0 x1v1 y1v1 ]
  __m128 t1 = _mm_unpackhi_ps(x, y);              // [ x2v2 y2v2 0 0 ]
  t0 = _mm_add_ps(t0, t1);                        // [ x0v0+x2v2 y0v0+y2v2 x1v1 y1v1 ]
  __m128 z = _mm_mul_ps(*matrix_r2, *vector);     // [ z0v0 z1v1 z2v2 0 ]
  __m128 w = _mm_set1_ps(0.f);
  __m128 t2 = _mm_unpacklo_ps(z, w);              // [ z0v0 0 z1v1 0 ]
  __m128 t3 = _mm_unpackhi_ps(z, w);              // [ z2v2 0 0 0 ]
  t2 = _mm_add_ps(t2, t3);                        // [ z0v0+z2v2 0 z1v1 0 ]
  // [ x0v0+x2v2 y0v0+y2v2 z0v0+z2v2 0 ] + [ x1v1 y1y1 z1v1 0 ]
  return _mm_add_ps(_mm_movelh_ps(t0, t2), _mm_movehl_ps(t2, t0));
#endif
}

static inline
__m128 mat3x4_mul_vec4(const __m128 matrix[3], const __m128 vector)
{
  return mat3x4_mul_vec4(&matrix[0], &matrix[1], &matrix[2], &vector);
}
#endif

#ifdef FCL_AVX_ENABLED
static inline
__m256d mat3x4_mul_vec4(const __m256d* matrix_r0, const __m256d* matrix_r1, const __m256d* matrix_r2, const __m256d* vector)
{
  __m256d x = _mm256_mul_pd(*matrix_r0, *vector);     // [ x0v0 x1v1 x2v2 0 ]
  __m256d y = _mm256_mul_pd(*matrix_r1, *vector);     // [ y0v0 y1v1 y2v2 0 ]
  __m256d t0 = _mm256_unpacklo_pd(x, y);              // [ x0v0 y0v0 x1v1 y1v1 ]
  __m256d t1 = _mm256_unpackhi_pd(x, y);              // [ x2v2 y2v2 0 0 ]
  t0 = _mm256_add_pd(t0, t1);                         // [ x0v0+x2v2 y0v0+y2v2 x1v1 y1v1 ]
  __m256d z = _mm256_mul_pd(*matrix_r2, *vector);     // [ z0v0 z1v1 z2v2 0 ]
  __m256d w = _mm256_set1_pd(0);
  __m256d t2 = _mm256_unpacklo_pd(z, w);              // [ z0v0 0 z1v1 0 ]
  __m256d t3 = _mm256_unpackhi_pd(z, w);              // [ z2v2 0 0 0 ]
  t2 = _mm256_add_pd(t2, t3);                         // [ z0v0+z2v2 0 z1v1 0 ]
  // [ x0v0+x2v2 y0v0+y2v2 z1v1 0 ] + [ x1v1 y1y1 z0v0+z2v2 0 ]
  return _mm256_add_pd(_mm256_blend_pd(t0, t2, 0x0C), _mm256_permute2f128_pd(t0, t2, 0x21));
}

static inline
__m256d mat3x4_mul_vec4(const __m256d matrix[3], const __m256d vector)
{
  return mat3x4_mul_vec4(&matrix[0], &matrix[1], &matrix[2], &vector);
}
#endif


//==============================================================================
/// @brief
/// Compute the product (M)^T*v,
/// M is a 3x4 matrix denoted by an array of 4 SIMD registers and the last
/// column is a zero vector
/// v is a 4x1 vector with a zero-padding element, denoted by a SIMD register
///
/// \param[in] matrix of 3x4
/// [ x0 x1 x2 0 ]
/// [ y0 y1 y2 0 ]
/// [ z0 z1 z2 0 ]
///
/// \param[in] vector of 4x1
/// [ v0 v1 v2 0 ]
///
/// \return a SIMD register which stores the result 3x1 vector
/// and a zero-padding element which we don't care
/// [ x0v0+y0v1+z0v2 x1v0+y1v1+z1v2 x2v0+y2v1+z2v2 0 ]
#ifdef FCL_SSE_ENABLED
static inline
__m128 transp_mat3x4_mul_vec4(const __m128* matrix_r0, const __m128* matrix_r1, const __m128* matrix_r2, const __m128* vector)
{
  __m128 r0 = _mm_mul_ps(vec_splat_ps(*vector, 0), *matrix_r0);
  __m128 r1 = _mm_mul_ps(vec_splat_ps(*vector, 1), *matrix_r1);
  __m128 r2 = _mm_mul_ps(vec_splat_ps(*vector, 2), *matrix_r2);
  return _mm_add_ps(_mm_add_ps(r0, r1), r2);
}

static inline
__m128 transp_mat3x4_mul_vec4(const __m128 matrix[3], const __m128 vector)
{
  return transp_mat3x4_mul_vec4(&matrix[0], &matrix[1], &matrix[2], &vector);
}
#endif

#ifdef FCL_AVX_ENABLED
static inline
__m256d transp_mat3x4_mul_vec4(const __m256d* matrix_r0, const __m256d* matrix_r1, const __m256d* matrix_r2, const __m256d* vector)
{
  __m256d r0 = _mm256_mul_pd(vec_splat_pd(*vector, 0), *matrix_r0);
  __m256d r1 = _mm256_mul_pd(vec_splat_pd(*vector, 1), *matrix_r1);
  __m256d r2 = _mm256_mul_pd(vec_splat_pd(*vector, 2), *matrix_r2);
  return _mm256_add_pd(_mm256_add_pd(r0, r1), r2);
}

static inline
__m256d transp_mat3x4_mul_vec4(const __m256d matrix[3], const __m256d vector)
{
  return transp_mat3x4_mul_vec4(&matrix[0], &matrix[1], &matrix[2], &vector);
}
#endif


//==============================================================================
/// @brief
/// Compute the product M1*M2,
/// where M1 is a 3x4 matrix denoted by an array of 4 SIMD registers,
/// and M2 is a 3x4 matrix denoted by an array of 4 SIMD registers.
/// The last columns of both M1 and M2 are zero vectors.
///
/// \param[in] matrix M1 of 3x4 (we only use 3x3 submatrix)
/// [ a00 a01 a02 0 ]
/// [ a10 a11 a12 0 ]
/// [ a20 a21 a22 0 ]
///
/// \param[in] matrix M2 of 3x4 (we only use 3x3 submatrix)
/// [ b00 b01 b02 0 ]
/// [ b10 b11 b12 0 ]
/// [ b20 b21 b22 0 ]
///
/// \param[out] matrix out of 3x4 (we only care about the 3x3 submatrix)
/// [ a00b00+a01b10+a02b20 a00b01+a01b11+a02b21 a00b02+a01b12+a02b22 0 ]
/// [ a10b00+a11b10+a12b20 a10b01+a11b11+a12b21 a10b02+a11b12+a12b22 0 ]
/// [ a20b00+a21b10+a22b20 a20b01+a21b11+a22b21 a20b02+a21b12+a22b22 0 ]
#ifdef FCL_SSE_ENABLED
static inline
void mat3x3_mul_mat3x3(__m128* out_r0, __m128* out_r1, __m128* out_r2,
    const __m128* m1_r0, const __m128* m1_r1, const __m128* m1_r2,
    const __m128* m2_r0, const __m128* m2_r1, const __m128* m2_r2)
{
  const __m128 m2_3 = _mm_setr_ps(0.f, 0.f, 0.f, 1.f);

  __m128 r0 = _mm_mul_ps(vec_splat_ps(*m1_r0, 0), *m2_r0);
  __m128 r1 = _mm_mul_ps(vec_splat_ps(*m1_r0, 1), *m2_r1);
  __m128 r2 = _mm_mul_ps(vec_splat_ps(*m1_r0, 2), *m2_r2);
  __m128 r3 = _mm_mul_ps(*m1_r0, m2_3);
  *out_r0 = _mm_add_ps(_mm_add_ps(r0, r1), _mm_add_ps(r2, r3));

  r0 = _mm_mul_ps(vec_splat_ps(*m1_r1, 0), *m2_r0);
  r1 = _mm_mul_ps(vec_splat_ps(*m1_r1, 1), *m2_r1);
  r2 = _mm_mul_ps(vec_splat_ps(*m1_r1, 2), *m2_r2);
  r3 = _mm_mul_ps(*m1_r1, m2_3);
  *out_r1 = _mm_add_ps(_mm_add_ps(r0, r1), _mm_add_ps(r2, r3));

  r0 = _mm_mul_ps(vec_splat_ps(*m1_r2, 0), *m2_r0);
  r1 = _mm_mul_ps(vec_splat_ps(*m1_r2, 1), *m2_r1);
  r2 = _mm_mul_ps(vec_splat_ps(*m1_r2, 2), *m2_r2);
  r3 = _mm_mul_ps(*m1_r2, m2_3);
  *out_r2 = _mm_add_ps(_mm_add_ps(r0, r1), _mm_add_ps(r2, r3));
}

static inline
void mat3x3_mul_mat3x3(__m128 out[3], const __m128 m1[3], const __m128 m2[3])
{
  mat3x3_mul_mat3x3(&out[0], &out[1], &out[2],
      &m1[0], &m1[1], &m1[2],
      &m2[0], &m2[1], &m2[2]);
}
#endif

#ifdef FCL_AVX_ENABLED
static inline
void mat3x3_mul_mat3x3(__m256d* out_r0, __m256d* out_r1, __m256d* out_r2,
    const __m256d* m1_r0, const __m256d* m1_r1, const __m256d* m1_r2,
    const __m256d* m2_r0, const __m256d* m2_r1, const __m256d* m2_r2)
{
  const __m256d m2_3 = _mm256_setr_pd(0, 0, 0, 1);

  __m256d r0 = _mm256_mul_pd(vec_splat_pd(*m1_r0, 0), *m2_r0);
  __m256d r1 = _mm256_mul_pd(vec_splat_pd(*m1_r0, 1), *m2_r1);
  __m256d r2 = _mm256_mul_pd(vec_splat_pd(*m1_r0, 2), *m2_r2);
  __m256d r3 = _mm256_mul_pd(*m1_r0, m2_3);
  *out_r0 = _mm256_add_pd(_mm256_add_pd(r0, r1), _mm256_add_pd(r2, r3));

  r0 = _mm256_mul_pd(vec_splat_pd(*m1_r1, 0), *m2_r0);
  r1 = _mm256_mul_pd(vec_splat_pd(*m1_r1, 1), *m2_r1);
  r2 = _mm256_mul_pd(vec_splat_pd(*m1_r1, 2), *m2_r2);
  r3 = _mm256_mul_pd(*m1_r1, m2_3);
  *out_r1 = _mm256_add_pd(_mm256_add_pd(r0, r1), _mm256_add_pd(r2, r3));

  r0 = _mm256_mul_pd(vec_splat_pd(*m1_r2, 0), *m2_r0);
  r1 = _mm256_mul_pd(vec_splat_pd(*m1_r2, 1), *m2_r1);
  r2 = _mm256_mul_pd(vec_splat_pd(*m1_r2, 2), *m2_r2);
  r3 = _mm256_mul_pd(*m1_r2, m2_3);
  *out_r2 = _mm256_add_pd(_mm256_add_pd(r0, r1), _mm256_add_pd(r2, r3));
}

static inline
void mat3x3_mul_mat3x3(__m256d out[3], const __m256d m1[3], const __m256d m2[3])
{
  mat3x3_mul_mat3x3(&out[0], &out[1], &out[2],
      &m1[0], &m1[1], &m1[2],
      &m2[0], &m2[1], &m2[2]);
}
#endif


//==============================================================================
/// @brief
/// Compute the product (M1)^T*M2,
/// where M1 is a 3x4 matrix denoted by an array of 4 SIMD registers,
/// and M2 is a 3x4 matrix denoted by an array of 4 SIMD registers.
/// The last columns of both M1 and M2 are zero vectors.
///
/// \param[in] matrix M1 of 3x4 (we only use 3x3 submatrix)
/// [ a00 a01 a02 0 ]
/// [ a10 a11 a12 0 ]
/// [ a20 a21 a22 0 ]
///
/// \param[in] matrix M2 of 3x4 (we only use 3x3 submatrix)
/// [ b00 b01 b02 0 ]
/// [ b10 b11 b12 0 ]
/// [ b20 b21 b22 0 ]
///
/// \param[out] matrix out of 3x4 (we only care about the 3x3 submatrix)
/// [ a00b00+a10b10+a20b20 a00b01+a10b11+a20b21 a00b02+a10b12+a20b22 0 ]
/// [ a01b00+a11b10+a21b20 a01b01+a11b11+a21b21 a01b02+a11b12+a21b22 0 ]
/// [ a02b00+a12b10+a22b20 a02b01+a12b11+a22b21 a02b02+a12b12+a22b22 0 ]
#ifdef FCL_SSE_ENABLED
static inline
void transp_mat3x3_mul_mat3x3(__m128* out_r0, __m128* out_r1, __m128* out_r2,
    const __m128* m1_r0, const __m128* m1_r1, const __m128* m1_r2,
    const __m128* m2_r0, const __m128* m2_r1, const __m128* m2_r2)
{
  const __m128 padding = _mm_setzero_ps();

  __m128 r0 = _mm_mul_ps(vec_splat_ps(*m1_r0, 0), *m2_r0);
  __m128 r1 = _mm_mul_ps(vec_splat_ps(*m1_r1, 0), *m2_r1);
  __m128 r2 = _mm_mul_ps(vec_splat_ps(*m1_r2, 0), *m2_r2);
  *out_r0 = _mm_add_ps(_mm_add_ps(r0, r1), _mm_add_ps(r2, padding));

  r0 = _mm_mul_ps(vec_splat_ps(*m1_r0, 1), *m2_r0);
  r1 = _mm_mul_ps(vec_splat_ps(*m1_r1, 1), *m2_r1);
  r2 = _mm_mul_ps(vec_splat_ps(*m1_r2, 1), *m2_r2);
  *out_r1 = _mm_add_ps(_mm_add_ps(r0, r1), _mm_add_ps(r2, padding));

  r0 = _mm_mul_ps(vec_splat_ps(*m1_r0, 2), *m2_r0);
  r1 = _mm_mul_ps(vec_splat_ps(*m1_r1, 2), *m2_r1);
  r2 = _mm_mul_ps(vec_splat_ps(*m1_r2, 2), *m2_r2);
  *out_r2 = _mm_add_ps(_mm_add_ps(r0, r1), _mm_add_ps(r2, padding));
}

static inline
void transp_mat3x3_mul_mat3x3(__m128 out[3], const __m128 m1[3], const __m128 m2[3])
{
  transp_mat3x3_mul_mat3x3(&out[0], &out[1], &out[2],
      &m1[0], &m1[1], &m1[2],
      &m2[0], &m2[1], &m2[2]);
}
#endif

#ifdef FCL_AVX_ENABLED
static inline
void transp_mat3x3_mul_mat3x3(__m256d* out_r0, __m256d* out_r1, __m256d* out_r2,
    const __m256d* m1_r0, const __m256d* m1_r1, const __m256d* m1_r2,
    const __m256d* m2_r0, const __m256d* m2_r1, const __m256d* m2_r2)
{
  const __m256d padding = _mm256_setzero_pd();

  __m256d r0 = _mm256_mul_pd(vec_splat_pd(*m1_r0, 0), *m2_r0);
  __m256d r1 = _mm256_mul_pd(vec_splat_pd(*m1_r1, 0), *m2_r1);
  __m256d r2 = _mm256_mul_pd(vec_splat_pd(*m1_r2, 0), *m2_r2);
  *out_r0 = _mm256_add_pd(_mm256_add_pd(r0, r1), _mm256_add_pd(r2, padding));

  r0 = _mm256_mul_pd(vec_splat_pd(*m1_r0, 1), *m2_r0);
  r1 = _mm256_mul_pd(vec_splat_pd(*m1_r1, 1), *m2_r1);
  r2 = _mm256_mul_pd(vec_splat_pd(*m1_r2, 1), *m2_r2);
  *out_r1 = _mm256_add_pd(_mm256_add_pd(r0, r1), _mm256_add_pd(r2, padding));

  r0 = _mm256_mul_pd(vec_splat_pd(*m1_r0, 2), *m2_r0);
  r1 = _mm256_mul_pd(vec_splat_pd(*m1_r1, 2), *m2_r1);
  r2 = _mm256_mul_pd(vec_splat_pd(*m1_r2, 2), *m2_r2);
  *out_r2 = _mm256_add_pd(_mm256_add_pd(r0, r1), _mm256_add_pd(r2, padding));
}

static inline
void transp_mat3x3_mul_mat3x3(__m256d out[3], const __m256d m1[3], const __m256d m2[3])
{
  transp_mat3x3_mul_mat3x3(&out[0], &out[1], &out[2],
      &m1[0], &m1[1], &m1[2],
      &m2[0], &m2[1], &m2[2]);
}
#endif


//==============================================================================
/// @brief
/// Compute the product M1*(M2)^T,
/// where M1 is a 3x4 matrix denoted by an array of 4 SIMD registers,
/// and M2 is a 3x4 matrix denoted by an array of 4 SIMD registers.
/// The last columns of both M1 and M2 are zero vectors.
///
/// \param[in] matrix M1 of 3x4 (we only use 3x3 submatrix)
/// [ a00 a01 a02 0 ]
/// [ a10 a11 a12 0 ]
/// [ a20 a21 a22 0 ]
///
/// \param[in] matrix M2 of 3x4 (we only use 3x3 submatrix)
/// [ b00 b01 b02 0 ]
/// [ b10 b11 b12 0 ]
/// [ b20 b21 b22 0 ]
///
/// \param[out] matrix out of 3x4 (we only care about the 3x3 submatrix)
/// [ a00b00+a01b01+a02b02 a00b10+a01b11+a02b12 a00b20+a01b21+a02b22 0 ]
/// [ a10b00+a11b01+a12b02 a10b10+a11b11+a12b12 a10b20+a11b21+a12b22 0 ]
/// [ a20b00+a21b01+a22b02 a20b10+a21b11+a22b12 a20b20+a21b21+a22b22 0 ]
#ifdef FCL_SSE_ENABLED
static inline
void mat3x3_mul_transp_mat3x3(__m128* out_r0, __m128* out_r1, __m128* out_r2,
    const __m128* m1_r0, const __m128* m1_r1, const __m128* m1_r2,
    const __m128* m2_r0, const __m128* m2_r1, const __m128* m2_r2)
{
#if defined (__SSE4__)
  __m128 prod0 = _mm_dp_ps(*m1_r0, *m2_r0, 0xFF);  // [ a00b00+a01b01+a02b02 a00b00+a01b01+a02b02 a00b00+a01b01+a02b02 a00b00+a01b01+a02b02 ]
  __m128 prod1 = _mm_dp_ps(*m1_r0, *m2_r1, 0xFF);  // [ a00b10+a01b11+a02b12 a00b10+a01b11+a02b12 a00b10+a01b11+a02b12 a00b10+a01b11+a02b12 ]
  __m128 prod2 = _mm_dp_ps(*m1_r0, *m2_r2, 0xFF);  // [ a00b20+a01b21+a02b22 a00b20+a01b21+a02b22 a00b20+a01b21+a02b22 a00b20+a01b21+a02b22 ]
  *out_r0 = _mm_setr_ps(prod0[0], prod1[0], prod2[0], 0.f);

  prod0 = _mm_dp_ps(*m1_r1, *m2_r0, 0xFF);         // [ a10b00+a11b01+a12b02 a10b00+a11b01+a12b02 a10b00+a11b01+a12b02 a10b00+a11b01+a12b02 ]
  prod1 = _mm_dp_ps(*m1_r1, *m2_r1, 0xFF);         // [ a10b10+a11b11+a12b12 a10b10+a11b11+a12b12 a10b10+a11b11+a12b12 a10b10+a11b11+a12b12 ]
  prod2 = _mm_dp_ps(*m1_r1, *m2_r2, 0xFF);         // [ a10b20+a11b21+a12b22 a10b20+a11b21+a12b22 a10b20+a11b21+a12b22 a10b20+a11b21+a12b22 ]
  *out_r1 = _mm_setr_ps(prod0[0], prod1[0], prod2[0], 0.f);

  prod0 = _mm_dp_ps(*m1_r2, *m2_r0, 0xFF);         // [ a10b00+a11b01+a12b02 a10b00+a11b01+a12b02 a10b00+a11b01+a12b02 a10b00+a11b01+a12b02 ]
  prod1 = _mm_dp_ps(*m1_r2, *m2_r1, 0xFF);         // [ a10b10+a11b11+a12b12 a10b10+a11b11+a12b12 a10b10+a11b11+a12b12 a10b10+a11b11+a12b12 ]
  prod2 = _mm_dp_ps(*m1_r2, *m2_r2, 0xFF);         // [ a10b20+a11b21+a12b22 a10b20+a11b21+a12b22 a10b20+a11b21+a12b22 a10b20+a11b21+a12b22 ]
  *out_r2 = _mm_setr_ps(prod0[0], prod1[0], prod2[0], 0.f);
#else
  *out_r0 = mat3x4_mul_vec4(m2_r0, m2_r1, m2_r2, m1_r0);
  *out_r1 = mat3x4_mul_vec4(m2_r0, m2_r1, m2_r2, m1_r1);
  *out_r2 = mat3x4_mul_vec4(m2_r0, m2_r1, m2_r2, m1_r2);
#endif
}

static inline
void mat3x3_mul_transp_mat3x3(__m128 out[3], const __m128 m1[3], const __m128 m2[3])
{
  mat3x3_mul_transp_mat3x3(&out[0], &out[1], &out[2],
      &m1[0], &m1[1], &m1[2],
      &m2[0], &m2[1], &m2[2]);
}
#endif

#ifdef FCL_AVX_ENABLED
static inline
void mat3x3_mul_transp_mat3x3(__m256d* out_r0, __m256d* out_r1, __m256d* out_r2,
    const __m256d* m1_r0, const __m256d* m1_r1, const __m256d* m1_r2,
    const __m256d* m2_r0, const __m256d* m2_r1, const __m256d* m2_r2)
{
  *out_r0 = mat3x4_mul_vec4(m2_r0, m2_r1, m2_r2, m1_r0);
  *out_r1 = mat3x4_mul_vec4(m2_r0, m2_r1, m2_r2, m1_r1);
  *out_r2 = mat3x4_mul_vec4(m2_r0, m2_r1, m2_r2, m1_r2);
}

static inline
void mat3x3_mul_transp_mat3x3(__m256d out[3], const __m256d m1[3], const __m256d m2[3])
{
  mat3x3_mul_transp_mat3x3(&out[0], &out[1], &out[2],
      &m1[0], &m1[1], &m1[2],
      &m2[0], &m2[1], &m2[2]);
}
#endif


} // details
} // fcl


#endif

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

/** \author Jia Pan */


#ifndef FCL_MATH_SIMD_DETAILS_H
#define FCL_MATH_SIMD_DETAILS_H

#include "fcl/common/data_types.h"

#include <xmmintrin.h>
#if defined (__SSE3__)
#include <pmmintrin.h>
#endif
#if defined (__SSE4__)
#include <smmintrin.h>
#endif


namespace fcl
{

/** @brief FCL internals. Ignore this :) unless you are God */
namespace detail
{

const __m128  xmms_0 = {0.f, 0.f, 0.f, 0.f};
const __m128d xmmd_0 = {0, 0};

static inline __m128 vec_sel(__m128 a, __m128 b, __m128 mask)
{
  return _mm_or_ps(_mm_and_ps(mask, b), _mm_andnot_ps(mask, a));
}
static inline __m128 vec_sel(__m128 a, __m128 b, const unsigned int* mask)
{
  return vec_sel(a, b, _mm_load_ps((float*)mask));
}

static inline __m128 vec_sel(__m128 a, __m128 b, unsigned int mask)
{
  return vec_sel(a, b, _mm_set1_ps(*(float*)&mask));
}

#define vec_splat(a, e) _mm_shuffle_ps((a), (a), _MM_SHUFFLE((e), (e), (e), (e)))
#define vec_splatd(a, e) _mm_shuffle_pd((a), (a), _MM_SHUFFLE2((e), (e)))

#define _mm_ror_ps(x, e) (((e) % 4) ? _mm_shuffle_ps((x), (x), _MM_SHUFFLE(((e)+3)%4, ((e)+2)%4, ((e)+1)%4, (e)%4)) : (x))

#define _mm_rol_ps(x, e) (((e) % 4) ? _mm_shuffle_ps((x), (x), _MM_SHUFFLE((7-(e))%4, (6-(e))%4, (5-(e))%4, (4-(e))%4)) : (x))

static inline __m128 newtonraphson_rsqrt4(const __m128 v)
{
  static const union { float i[4]; __m128 m; } _half4 __attribute__ ((aligned(16))) = {{.5f, .5f, .5f, .5f}};
  static const union { float i[4]; __m128 m; } _three __attribute__ ((aligned(16))) = {{3.f, 3.f, 3.f, 3.f}};
  __m128 approx = _mm_rsqrt_ps(v);
  __m128 muls = _mm_mul_ps(_mm_mul_ps(v, approx), approx);
  return _mm_mul_ps(_mm_mul_ps(_half4.m, approx), _mm_sub_ps(_three.m, muls));
}

struct sse_meta_f4
{
  typedef float meta_type;

  union {float vs[4]; __m128 v; }; 
  sse_meta_f4() : v(_mm_set1_ps(0)) {}
  sse_meta_f4(float x) : v(_mm_set1_ps(x)) {}
  sse_meta_f4(float* px) : v(_mm_load_ps(px)) {}
  sse_meta_f4(__m128 x) : v(x) {}
  sse_meta_f4(float x, float y, float z, float w = 1) : v(_mm_setr_ps(x, y, z, w)) {}
  inline void setValue(float x, float y, float z, float w = 1) { v = _mm_setr_ps(x, y, z, w); }
  inline void setValue(float x) { v = _mm_set1_ps(x); }
  inline void setValue(__m128 x) { v = x; }
  inline void negate() { v = _mm_sub_ps(xmms_0, v); }

  inline sse_meta_f4& ubound(const sse_meta_f4& u)
  {
    v = _mm_min_ps(v, u.v);
    return *this;
  }

  inline sse_meta_f4& lbound(const sse_meta_f4& l)
  {
    v = _mm_max_ps(v, l.v);
    return *this;
  }
  
  inline void* operator new [] (size_t n) { return _mm_malloc(n, 16); }
  inline void operator delete [] (void* x) { if(x) _mm_free(x); }
  inline float operator [] (size_t i) const { return vs[i]; }
  inline float& operator [] (size_t i) { return vs[i]; }

  inline sse_meta_f4 operator + (const sse_meta_f4& other) const { return sse_meta_f4(_mm_add_ps(v, other.v)); }
  inline sse_meta_f4 operator - (const sse_meta_f4& other) const { return sse_meta_f4(_mm_sub_ps(v, other.v)); }
  inline sse_meta_f4 operator * (const sse_meta_f4& other) const { return sse_meta_f4(_mm_mul_ps(v, other.v)); }
  inline sse_meta_f4 operator / (const sse_meta_f4& other) const { return sse_meta_f4(_mm_div_ps(v, other.v)); }
  inline sse_meta_f4& operator += (const sse_meta_f4& other) { v = _mm_add_ps(v, other.v); return *this; }
  inline sse_meta_f4& operator -= (const sse_meta_f4& other) { v = _mm_sub_ps(v, other.v); return *this; }
  inline sse_meta_f4& operator *= (const sse_meta_f4& other) { v = _mm_mul_ps(v, other.v); return *this; }
  inline sse_meta_f4& operator /= (const sse_meta_f4& other) { v = _mm_div_ps(v, other.v); return *this; }
  inline sse_meta_f4 operator + (float t) const { return sse_meta_f4(_mm_add_ps(v, _mm_set1_ps(t))); }
  inline sse_meta_f4 operator - (float t) const { return sse_meta_f4(_mm_sub_ps(v, _mm_set1_ps(t))); }
  inline sse_meta_f4 operator * (float t) const { return sse_meta_f4(_mm_mul_ps(v, _mm_set1_ps(t))); }
  inline sse_meta_f4 operator / (float t) const { return sse_meta_f4(_mm_div_ps(v, _mm_set1_ps(t))); }
  inline sse_meta_f4& operator += (float t) { v = _mm_add_ps(v, _mm_set1_ps(t)); return *this; }
  inline sse_meta_f4& operator -= (float t) { v = _mm_sub_ps(v, _mm_set1_ps(t)); return *this; }
  inline sse_meta_f4& operator *= (float t) { v = _mm_mul_ps(v, _mm_set1_ps(t)); return *this; }
  inline sse_meta_f4& operator /= (float t) { v = _mm_div_ps(v, _mm_set1_ps(t)); return *this; }
  inline sse_meta_f4 operator - () const 
  {
    static const union { int i[4]; __m128 m; } negativemask __attribute__ ((aligned(16))) = {{0x80000000, 0x80000000, 0x80000000, 0x80000000}};
    return sse_meta_f4(_mm_xor_ps(negativemask.m, v));                     
  }
} __attribute__ ((aligned (16)));

struct sse_meta_d4
{
  typedef double meta_type;

  union {double vs[4]; __m128d v[2]; };
  sse_meta_d4()
  {
    setValue(0.0);
  }
                    
  sse_meta_d4(double x)
  {
    setValue(x);
  }
  
  sse_meta_d4(double* px)
  {
    v[0] = _mm_load_pd(px);
    v[1] = _mm_set_pd(0, *(px + 2));
  }
  
  sse_meta_d4(__m128d x, __m128d y)
  {
    v[0] = x;
    v[1] = y;
  }

  sse_meta_d4(double x, double y, double z, double w = 0)
  {
    setValue(x, y, z, w);
  }

  inline void setValue(double x, double y, double z, double w = 0)
  {
    v[0] = _mm_setr_pd(x, y);
    v[1] = _mm_setr_pd(z, w);
  }

  inline void setValue(double x)
  {
    v[0] = _mm_set1_pd(x);
    v[1] = v[0];
  }

  inline void setValue(__m128d x, __m128d y)
  {
    v[0] = x;
    v[1] = y;
  }

  inline void negate()
  {
    v[0] = _mm_sub_pd(xmmd_0, v[0]);
    v[1] = _mm_sub_pd(xmmd_0, v[1]);
  }

  inline sse_meta_d4& ubound(const sse_meta_d4& u)
  {
    v[0] = _mm_min_pd(v[0], u.v[0]);
    v[1] = _mm_min_pd(v[1], u.v[1]);
    return *this;
  }

  inline sse_meta_d4& lbound(const sse_meta_d4& l)
  {
    v[0] = _mm_max_pd(v[0], l.v[0]);
    v[1] = _mm_max_pd(v[1], l.v[1]);
    return *this;
  }

  inline void* operator new [] (size_t n)
  {
    return _mm_malloc(n, 16);
  }

  inline void operator delete [] (void* x)
  {
    if(x) _mm_free(x);
  }

  inline double operator [] (size_t i) const { return vs[i]; }
  inline double& operator [] (size_t i) { return vs[i]; }

  inline sse_meta_d4 operator + (const sse_meta_d4& other) const { return sse_meta_d4(_mm_add_pd(v[0], other.v[0]), _mm_add_pd(v[1], other.v[1])); }
  inline sse_meta_d4 operator - (const sse_meta_d4& other) const { return sse_meta_d4(_mm_sub_pd(v[0], other.v[0]), _mm_sub_pd(v[1], other.v[1])); }
  inline sse_meta_d4 operator * (const sse_meta_d4& other) const { return sse_meta_d4(_mm_mul_pd(v[0], other.v[0]), _mm_mul_pd(v[1], other.v[1])); }
  inline sse_meta_d4 operator / (const sse_meta_d4& other) const { return sse_meta_d4(_mm_div_pd(v[0], other.v[0]), _mm_div_pd(v[1], other.v[1])); }
  inline sse_meta_d4& operator += (const sse_meta_d4& other) { v[0] = _mm_add_pd(v[0], other.v[0]); v[1] = _mm_add_pd(v[1], other.v[1]); return *this; }
  inline sse_meta_d4& operator -= (const sse_meta_d4& other) { v[0] = _mm_sub_pd(v[0], other.v[0]); v[1] = _mm_sub_pd(v[1], other.v[1]); return *this; }
  inline sse_meta_d4& operator *= (const sse_meta_d4& other) { v[0] = _mm_mul_pd(v[0], other.v[0]); v[1] = _mm_mul_pd(v[1], other.v[1]); return *this; }
  inline sse_meta_d4& operator /= (const sse_meta_d4& other) { v[0] = _mm_div_pd(v[0], other.v[0]); v[1] = _mm_div_pd(v[1], other.v[1]); return *this; }
  inline sse_meta_d4 operator + (double t) const { register __m128d d = _mm_set1_pd(t); return sse_meta_d4(_mm_add_pd(v[0], d), _mm_add_pd(v[1], d)); }
  inline sse_meta_d4 operator - (double t) const { register __m128d d = _mm_set1_pd(t); return sse_meta_d4(_mm_sub_pd(v[0], d), _mm_sub_pd(v[1], d)); }
  inline sse_meta_d4 operator * (double t) const { register __m128d d = _mm_set1_pd(t); return sse_meta_d4(_mm_mul_pd(v[0], d), _mm_mul_pd(v[1], d)); }
  inline sse_meta_d4 operator / (double t) const { register __m128d d = _mm_set1_pd(t); return sse_meta_d4(_mm_div_pd(v[0], d), _mm_div_pd(v[1], d)); }
  inline sse_meta_d4& operator += (double t) { register __m128d d = _mm_set1_pd(t); v[0] = _mm_add_pd(v[0], d); v[1] = _mm_add_pd(v[1], d); return *this; }
  inline sse_meta_d4& operator -= (double t) { register __m128d d = _mm_set1_pd(t); v[0] = _mm_sub_pd(v[0], d); v[1] = _mm_sub_pd(v[1], d); return *this; }
  inline sse_meta_d4& operator *= (double t) { register __m128d d = _mm_set1_pd(t); v[0] = _mm_mul_pd(v[0], d); v[1] = _mm_mul_pd(v[1], d); return *this; }
  inline sse_meta_d4& operator /= (double t) { register __m128d d = _mm_set1_pd(t); v[0] = _mm_div_pd(v[0], d); v[1] = _mm_div_pd(v[1], d); return *this; }
  inline sse_meta_d4 operator - () const 
  {
    static const union { FCL_INT64 i[2]; __m128d m; } negativemask __attribute__ ((aligned(16))) = {{0x8000000000000000, 0x8000000000000000}};
    return sse_meta_d4(_mm_xor_pd(v[0], negativemask.m), _mm_xor_pd(v[1], negativemask.m));
  }
} __attribute__ ((aligned (16)));



static inline __m128 cross_prod(__m128 x, __m128 y)
{
  // set to a[1][2][0][3] , b[2][0][1][3]
  // multiply
  static const int s1 = _MM_SHUFFLE(3, 0, 2, 1);
  static const int s2 = _MM_SHUFFLE(3, 1, 0, 2);
  __m128 xa = _mm_mul_ps(_mm_shuffle_ps(x, x, s1), _mm_shuffle_ps(y, y, s2));

  // set to a[2][0][1][3] , b[1][2][0][3]
  // multiply
  __m128 xb = _mm_mul_ps(_mm_shuffle_ps(x, x, s2), _mm_shuffle_ps(y, y, s1));

  // subtract
  return _mm_sub_ps(xa, xb);
}

static inline sse_meta_f4 cross_prod(const sse_meta_f4& x, const sse_meta_f4& y)
{
  return sse_meta_f4(cross_prod(x.v, y.v));
}

static inline void cross_prod(__m128d x0, __m128d x1, __m128d y0, __m128d y1, __m128d* z0, __m128d* z1)
{
  static const int s0 = _MM_SHUFFLE2(0, 0);
  static const int s1 = _MM_SHUFFLE2(0, 1);
  static const int s2 = _MM_SHUFFLE2(1, 0);
  static const int s3 = _MM_SHUFFLE2(1, 1);  
  __m128d xa1 = _mm_mul_pd(_mm_shuffle_pd(x0, x1, s1), _mm_shuffle_pd(y1, y0, s0));
  __m128d ya1 = _mm_mul_pd(_mm_shuffle_pd(x0, x1, s2), _mm_shuffle_pd(y0, y1, s3));
  
  __m128d xa2 = _mm_mul_pd(_mm_shuffle_pd(x1, x0, s0), _mm_shuffle_pd(y0, y1, s1));
  __m128d ya2 = _mm_mul_pd(_mm_shuffle_pd(x0, x1, s3), _mm_shuffle_pd(y0, y1, s2));

  *z0 = _mm_sub_pd(xa1, xa2);
  *z1 = _mm_sub_pd(ya1, ya2);
}

static inline sse_meta_d4 cross_prod(const sse_meta_d4& x, const sse_meta_d4& y)
{
  __m128d z0, z1;
  cross_prod(x.v[0], x.v[1], y.v[0], y.v[1], &z0, &z1);
  return sse_meta_d4(z0, z1);
}


static inline __m128 dot_prod3(__m128 x, __m128 y)
{
  register __m128 m = _mm_mul_ps(x, y);
  return _mm_add_ps(_mm_shuffle_ps(m, m, _MM_SHUFFLE(0, 0, 0, 0)),
                    _mm_add_ps(vec_splat(m, 1), vec_splat(m, 2)));
}

static inline float dot_prod3(const sse_meta_f4& x, const sse_meta_f4& y)
{
  return _mm_cvtss_f32(dot_prod3(x.v, y.v));
}


static inline __m128d dot_prod3(__m128d x0, __m128d x1, __m128d y0, __m128d y1)
{
  register __m128d m1 = _mm_mul_pd(x0, y0);
  register __m128d m2 = _mm_mul_pd(x1, y1);
  return _mm_add_pd(_mm_add_pd(vec_splatd(m1, 0), vec_splatd(m1, 1)), vec_splatd(m2, 0));
}

static inline double dot_prod3(const sse_meta_d4& x, const sse_meta_d4& y)
{
  double d;
  _mm_storel_pd(&d, dot_prod3(x.v[0], x.v[1], y.v[0], y.v[1]));
  return d;
}

static inline __m128 dot_prod4(__m128 x, __m128 y)
{
#if defined (__SSE4__)
  return _mm_dp_ps(x, y, 0x71);
#elif defined (__SSE3__)
  register __m128 t = _mm_mul_ps(x, y);
  t = _mm_hadd_ps(t, t);
  return _mm_hadd_ps(t, t);
#else
  register __m128 s = _mm_mul_ps(x, y);
  register __m128 r = _mm_add_ss(s, _mm_movehl_ps(s, s));
  return _mm_add_ss(r, _mm_shuffle_ps(r, r, 1));
#endif
}


static inline float dot_prod4(const sse_meta_f4& x, const sse_meta_f4& y)
{
  return _mm_cvtss_f32(dot_prod4(x.v, y.v));
}

static inline __m128d dot_prod4(__m128d x0, __m128d x1, __m128d y0, __m128d y1)
{
#if defined (__SSE4__)
  register __m128d t1 = _mm_dp_pd(x0, y0, 0x31);
  register __m128d t2 = _mm_dp_pd(x1, y1, 0x11);
  return _mm_add_pd(t1, t2);
#elif defined (__SSE3__)
  register __m128d t1 = _mm_mul_pd(x0, y0);
  register __m128d t2 = _mm_mul_pd(x1, y1);
  t1 = _mm_hadd_pd(t1, t1);
  t2 = _mm_hadd_pd(t2, t2);
  return _mm_add_pd(t1, t2);
#else 
  register __m128d t1 = _mm_mul_pd(x0, y0);
  register __m128d t2 = _mm_mul_pd(x1, y1);
  t1 = _mm_add_pd(t1, t2);
  return _mm_add_pd(t1, _mm_shuffle_pd(t1, t1, 1));
#endif
}

static inline double dot_prod4(const sse_meta_d4& x, const sse_meta_d4& y)
{
  double d;
  _mm_storel_pd(&d, dot_prod4(x.v[0], x.v[1], y.v[0], y.v[1]));
  return d;
}

static inline sse_meta_f4 min(const sse_meta_f4& x, const sse_meta_f4& y)
{
  return sse_meta_f4(_mm_min_ps(x.v, y.v));
}

static inline sse_meta_d4 min(const sse_meta_d4& x, const sse_meta_d4& y)
{
  return sse_meta_d4(_mm_min_pd(x.v[0], y.v[0]), _mm_min_pd(x.v[1], y.v[1]));
}

static inline sse_meta_f4 max(const sse_meta_f4& x, const sse_meta_f4& y)
{
  return sse_meta_f4(_mm_max_ps(x.v, y.v));
}

static inline sse_meta_d4 max(const sse_meta_d4& x, const sse_meta_d4& y)
{
  return sse_meta_d4(_mm_max_pd(x.v[0], y.v[0]), _mm_max_pd(x.v[1], y.v[1]));
}

static inline sse_meta_f4 abs(const sse_meta_f4& x)
{
  static const union { int i[4]; __m128 m; } abs4mask __attribute__ ((aligned (16))) = {{0x7fffffff, 0x7fffffff, 0x7fffffff, 0x7fffffff}};
  return sse_meta_f4(_mm_and_ps(x.v, abs4mask.m));
}

static inline sse_meta_d4 abs(const sse_meta_d4& x)
{
  static const union { FCL_INT64 i[2]; __m128d m; } abs2mask __attribute__ ((aligned (16))) = {{0x7fffffffffffffff, 0x7fffffffffffffff}};
  return sse_meta_d4(_mm_and_pd(x.v[0], abs2mask.m), _mm_and_pd(x.v[1], abs2mask.m));
}

static inline bool equal(const sse_meta_f4& x, const sse_meta_f4& y, float epsilon)
{
  register __m128 d = _mm_sub_ps(x.v, y.v);
  register __m128 e = _mm_set1_ps(epsilon);
  return ((_mm_movemask_ps(_mm_cmplt_ps(d, e)) & 0x7) == 0x7) && ((_mm_movemask_ps(_mm_cmpgt_ps(d, _mm_sub_ps(xmms_0, e))) & 0x7) == 0x7);
}

static inline bool equal(const sse_meta_d4& x, const sse_meta_d4& y, double epsilon)
{
  register __m128d d = _mm_sub_pd(x.v[0], y.v[0]);
  register __m128d e = _mm_set1_pd(epsilon);
  
  if(_mm_movemask_pd(_mm_cmplt_pd(d, e)) != 0x3) return false;
  if(_mm_movemask_pd(_mm_cmpgt_pd(d, _mm_sub_pd(xmmd_0, e))) != 0x3) return false;
  
  d = _mm_sub_pd(x.v[1], y.v[1]);
  if((_mm_movemask_pd(_mm_cmplt_pd(d, e)) & 0x1) != 0x1) return false;
  if((_mm_movemask_pd(_mm_cmpgt_pd(d, _mm_sub_pd(xmmd_0, e))) & 0x1) != 0x1) return false;
  return true;
}

static inline sse_meta_f4 normalize3(const sse_meta_f4& x)
{
  register __m128 m = _mm_mul_ps(x.v, x.v);
  __m128 r = _mm_add_ps(vec_splat(m, 0), _mm_add_ps(vec_splat(m, 1), vec_splat(m, 2)));
  return sse_meta_f4(_mm_mul_ps(x.v, newtonraphson_rsqrt4(r)));
}

static inline sse_meta_f4 normalize3_approx(const sse_meta_f4& x)
{
  register __m128 m = _mm_mul_ps(x.v, x.v);
  __m128 r = _mm_add_ps(vec_splat(m, 0), _mm_add_ps(vec_splat(m, 1), vec_splat(m, 2)));
  return sse_meta_f4(_mm_mul_ps(x.v, _mm_rsqrt_ps(r)));
}


static inline void transpose(__m128 c0, __m128 c1, __m128 c2, __m128* r0, __m128* r1, __m128* r2)
{
  static const union { unsigned int i[4]; __m128 m; } selectmask __attribute__ ((aligned(16))) = {{0, 0xffffffff, 0, 0}};
  register __m128 t0, t1;
  t0 = _mm_unpacklo_ps(c0, c2);
  t1 = _mm_unpackhi_ps(c0, c2);
  *r0 = _mm_unpacklo_ps(t0, c1);
  *r1 = _mm_shuffle_ps(t0, t0, _MM_SHUFFLE(0, 3, 2, 2));
  *r1 = vec_sel(*r1, c1, selectmask.i);
  *r2 = _mm_shuffle_ps(t1, t1, _MM_SHUFFLE(0, 1, 1, 0));
  *r2 = vec_sel(*r2, vec_splat(c1, 2), selectmask.i);
}


static inline void inverse(__m128 c0, __m128 c1, __m128 c2, __m128* i0, __m128* i1, __m128* i2)
{
  __m128 t0, t1, t2, d, invd;
  t2 = cross_prod(c0, c1);
  t0 = cross_prod(c1, c2);
  t1 = cross_prod(c2, c0);
  d = dot_prod3(t2, c2);
  d = vec_splat(d, 0);
  invd = _mm_rcp_ps(d); // approximate inverse
  transpose(t0, t1, t2, i0, i1, i2);
  *i0 = _mm_mul_ps(*i0, invd);
  *i1 = _mm_mul_ps(*i1, invd);
  *i2 = _mm_mul_ps(*i2, invd);
}


struct sse_meta_f12
{
  typedef float meta_type;
  typedef sse_meta_f4 vector_type;
  sse_meta_f4 c[3];

  sse_meta_f12() { setZero(); }

  sse_meta_f12(float xx, float xy, float xz,
               float yx, float yy, float yz,
               float zx, float zy, float zz)
  { setValue(xx, xy, xz, yx, yy, yz, zx, zy, zz); }

  sse_meta_f12(const sse_meta_f4& x, const sse_meta_f4& y, const sse_meta_f4& z)
  { setColumn(x, y, z); }

  sse_meta_f12(__m128 x, __m128 y, __m128 z)
  { setColumn(x, y, z); }

  inline void setValue(float xx, float xy, float xz,
                       float yx, float yy, float yz,
                       float zx, float zy, float zz)
  {
    c[0].setValue(xx, yx, zx, 0);
    c[1].setValue(xy, yy, zy, 0);
    c[2].setValue(xz, yz, zz, 0);
  }

  inline void setIdentity()
  {
    c[0].setValue(1, 0, 0, 0);
    c[1].setValue(0, 1, 0, 0);
    c[2].setValue(0, 0, 1, 0);
  }

  inline void setZero()
  {
    c[0].setValue(0);
    c[1].setValue(0);
    c[2].setValue(0);
  }

  inline void setColumn(const sse_meta_f4& x, const sse_meta_f4& y, const sse_meta_f4& z)
  {
    c[0] = x; c[1] = y; c[2] = z;
  }

  inline void setColumn(__m128 x, __m128 y, __m128 z)
  {
    c[0].setValue(x); c[1].setValue(y); c[2].setValue(z);
  }

  inline const sse_meta_f4& getColumn(size_t i) const
  {
    return c[i];
  }

  inline sse_meta_f4& getColumn(size_t i) 
  {
    return c[i];
  }

  inline sse_meta_f4 getRow(size_t i) const
  {
    return sse_meta_f4(c[0][i], c[1][i], c[2][i], 0);
  }

  inline float operator () (size_t i, size_t j) const
  {
    return c[j][i];
  }

  inline float& operator () (size_t i, size_t j) 
  {
    return c[j][i];
  }

  inline sse_meta_f4 operator * (const sse_meta_f4& v) const
  {
    return sse_meta_f4(_mm_add_ps(_mm_add_ps(_mm_mul_ps(c[0].v, vec_splat(v.v, 0)), _mm_mul_ps(c[1].v, vec_splat(v.v, 1))), _mm_mul_ps(c[2].v, vec_splat(v.v, 2))));
  }

  inline sse_meta_f12 operator * (const sse_meta_f12& mat) const
  {
    return sse_meta_f12((*this) * mat.c[0], (*this) * mat.c[1], (*this) * mat.c[2]);
  }

  inline sse_meta_f12 operator + (const sse_meta_f12& mat) const
  {
    return sse_meta_f12(c[0] + mat.c[0], c[1] + mat.c[1], c[2] + mat.c[2]);
  }

  inline sse_meta_f12 operator - (const sse_meta_f12& mat) const
  {
    return sse_meta_f12(c[0] - mat.c[0], c[1] - mat.c[1], c[2] - mat.c[2]);
  }

  inline sse_meta_f12 operator + (float t_) const
  {
    sse_meta_f4 t(t_);
    return sse_meta_f12(c[0] + t, c[1] + t, c[2] + t);
  }

  inline sse_meta_f12 operator - (float t_) const
  {
    sse_meta_f4 t(t_);
    return sse_meta_f12(c[0] - t, c[1] - t, c[2] - t);
  }

  inline sse_meta_f12 operator * (float t_) const
  {
    sse_meta_f4 t(t_);
    return sse_meta_f12(c[0] * t, c[1] * t, c[2] * t);
  }

  inline sse_meta_f12 operator / (float t_) const
  {
    sse_meta_f4 t(t_);
    return sse_meta_f12(c[0] / t, c[1] / t, c[2] / t);
  }

  inline sse_meta_f12& operator *= (const sse_meta_f12& mat)
  {
    setColumn((*this) * mat.c[0], (*this) * mat.c[1], (*this) * mat.c[2]);
    return *this;
  }

  inline sse_meta_f12& operator += (const sse_meta_f12& mat)
  {
    c[0] += mat.c[0];
    c[1] += mat.c[1];
    c[2] += mat.c[2];
    return *this;
  }

  inline sse_meta_f12& operator -= (const sse_meta_f12& mat)
  {
    c[0] -= mat.c[0];
    c[1] -= mat.c[1];
    c[2] -= mat.c[2];
    return *this;
  }

  inline sse_meta_f12& operator += (float t_)
  {
    sse_meta_f4 t(t_);
    c[0] += t;
    c[1] += t;
    c[2] += t;
    return *this;
  }

  inline sse_meta_f12& operator -= (float t_)
  {
    sse_meta_f4 t(t_);
    c[0] -= t;
    c[1] -= t;
    c[2] -= t;
    return *this;
  }

  inline sse_meta_f12& operator *= (float t_)
  {
    sse_meta_f4 t(t_);
    c[0] *= t;
    c[1] *= t;
    c[2] *= t;
    return *this;
  }

  inline sse_meta_f12& operator /= (float t_)
  {
    sse_meta_f4 t(t_);
    c[0] /= t;
    c[1] /= t;
    c[2] /= t;
    return *this;
  }

  inline sse_meta_f12& inverse()
  {
    __m128 inv0, inv1, inv2;
    detail::inverse(c[0].v, c[1].v, c[2].v, &inv0, &inv1, &inv2);
    setColumn(inv0, inv1, inv2);
    return *this;
  }

  inline sse_meta_f12& transpose()
  {
    __m128 r0, r1, r2;
    detail::transpose(c[0].v, c[1].v, c[2].v, &r0, &r1, &r2);
    setColumn(r0, r1, r2);
    return *this;
  }

  inline sse_meta_f12& abs()
  {
    c[0] = detail::abs(c[0]);
    c[1] = detail::abs(c[1]);
    c[2] = detail::abs(c[2]);
    return *this;
  }

  inline float determinant() const
  {
    return _mm_cvtss_f32(dot_prod3(c[2].v, cross_prod(c[0].v, c[1].v)));
  }

  inline sse_meta_f12 transposeTimes(const sse_meta_f12& other) const
  {
    return sse_meta_f12(dot_prod3(c[0], other.c[0]), dot_prod3(c[0], other.c[1]), dot_prod3(c[0], other.c[2]),
                        dot_prod3(c[1], other.c[0]), dot_prod3(c[1], other.c[1]), dot_prod3(c[1], other.c[2]),
                        dot_prod3(c[2], other.c[0]), dot_prod3(c[2], other.c[1]), dot_prod3(c[2], other.c[2]));
  }

  inline sse_meta_f12 timesTranspose(const sse_meta_f12& m) const
  {
    sse_meta_f12 tmp(m);
    return (*this) * tmp.transpose();
  }

  inline sse_meta_f4 transposeTimes(const sse_meta_f4& v) const
  {
    return sse_meta_f4(dot_prod3(c[0], v), dot_prod3(c[1], v), dot_prod3(c[2], v));
  }

  inline float transposeDot(size_t i, const sse_meta_f4& v) const
  {
    return dot_prod3(c[i], v);
  }

  inline float dot(size_t i, const sse_meta_f4& v) const
  {
    return v[0] * c[0][i] + v[1] * c[1][i] + v[2] * c[2][i];
  }

};

static inline sse_meta_f12 abs(const sse_meta_f12& mat)
{
  return sse_meta_f12(abs(mat.getColumn(0)), abs(mat.getColumn(1)), abs(mat.getColumn(2)));
}

static inline sse_meta_f12 transpose(const sse_meta_f12& mat)
{
  __m128 r0, r1, r2;
  transpose(mat.getColumn(0).v, mat.getColumn(1).v, mat.getColumn(2).v, &r0, &r1, &r2);
  return sse_meta_f12(r0, r1, r2);
}


static inline sse_meta_f12 inverse(const sse_meta_f12& mat)
{
  __m128 inv0, inv1, inv2;
  inverse(mat.getColumn(0).v, mat.getColumn(1).v, mat.getColumn(2).v, &inv0, &inv1, &inv2);
  return sse_meta_f12(inv0, inv1, inv2);
}


static inline void transpose(__m128 c0, __m128 c1, __m128 c2, __m128 c3,
                             __m128* r0, __m128* r1, __m128* r2, __m128* r3)
{
  __m128 tmp0 = _mm_unpacklo_ps(c0, c2);
  __m128 tmp1 = _mm_unpacklo_ps(c1, c3);
  __m128 tmp2 = _mm_unpackhi_ps(c0, c2);
  __m128 tmp3 = _mm_unpackhi_ps(c1, c3);
  *r0 = _mm_unpacklo_ps(tmp0, tmp1);
  *r1 = _mm_unpackhi_ps(tmp0, tmp1);
  *r2 = _mm_unpacklo_ps(tmp2, tmp3);
  *r3 = _mm_unpackhi_ps(tmp2, tmp3);
}


static inline void inverse(__m128 c0, __m128 c1, __m128 c2, __m128 c3,
                           __m128* res0, __m128* res1, __m128* res2, __m128* res3)
{
  __m128 Va, Vb, Vc;
  __m128 r1, r2, r3, tt, tt2;
  __m128 sum, Det, RDet;
  __m128 trns0, trns1, trns2, trns3;

  // Calculating the minterms for the first line.

  tt = c3; tt2 = _mm_ror_ps(c2,1); 
  Vc = _mm_mul_ps(tt2,_mm_ror_ps(tt,0));					// V3'\B7V4
  Va = _mm_mul_ps(tt2,_mm_ror_ps(tt,2));					// V3'\B7V4"
  Vb = _mm_mul_ps(tt2,_mm_ror_ps(tt,3));					// V3'\B7V4^

  r1 = _mm_sub_ps(_mm_ror_ps(Va,1),_mm_ror_ps(Vc,2));		// V3"\B7V4^ - V3^\B7V4"
  r2 = _mm_sub_ps(_mm_ror_ps(Vb,2),_mm_ror_ps(Vb,0));		// V3^\B7V4' - V3'\B7V4^
  r3 = _mm_sub_ps(_mm_ror_ps(Va,0),_mm_ror_ps(Vc,1));		// V3'\B7V4" - V3"\B7V4'

  tt = c1;
  Va = _mm_ror_ps(tt,1);		sum = _mm_mul_ps(Va,r1);
  Vb = _mm_ror_ps(tt,2);		sum = _mm_add_ps(sum,_mm_mul_ps(Vb,r2));
  Vc = _mm_ror_ps(tt,3);		sum = _mm_add_ps(sum,_mm_mul_ps(Vc,r3));

  // Calculating the determinant.
  Det = _mm_mul_ps(sum,c0);
  Det = _mm_add_ps(Det,_mm_movehl_ps(Det,Det));

  static const union { int i[4]; __m128 m; } Sign_PNPN __attribute__ ((aligned(16))) = {{0x00000000, 0x80000000, 0x00000000, 0x80000000}};
  static const union { int i[4]; __m128 m; } Sign_NPNP __attribute__ ((aligned(16))) = {{0x80000000, 0x00000000, 0x80000000, 0x00000000}};
  static const union { float i[4]; __m128 m; } ZERONE __attribute__ ((aligned(16))) = {{1.0f, 0.0f, 0.0f, 1.0f}};

  __m128 mtL1 = _mm_xor_ps(sum,Sign_PNPN.m);

  // Calculating the minterms of the second line (using previous results).
  tt = _mm_ror_ps(c0,1);		sum = _mm_mul_ps(tt,r1);
  tt = _mm_ror_ps(tt,1);		sum = _mm_add_ps(sum,_mm_mul_ps(tt,r2));
  tt = _mm_ror_ps(tt,1);		sum = _mm_add_ps(sum,_mm_mul_ps(tt,r3));
  __m128 mtL2 = _mm_xor_ps(sum,Sign_NPNP.m);

  // Testing the determinant.
  Det = _mm_sub_ss(Det,_mm_shuffle_ps(Det,Det,1));

  // Calculating the minterms of the third line.
  tt = _mm_ror_ps(c0,1);
  Va = _mm_mul_ps(tt,Vb);									// V1'\B7V2"
  Vb = _mm_mul_ps(tt,Vc);									// V1'\B7V2^
  Vc = _mm_mul_ps(tt,c1);								// V1'\B7V2

  r1 = _mm_sub_ps(_mm_ror_ps(Va,1),_mm_ror_ps(Vc,2));		// V1"\B7V2^ - V1^\B7V2"
  r2 = _mm_sub_ps(_mm_ror_ps(Vb,2),_mm_ror_ps(Vb,0));		// V1^\B7V2' - V1'\B7V2^
  r3 = _mm_sub_ps(_mm_ror_ps(Va,0),_mm_ror_ps(Vc,1));		// V1'\B7V2" - V1"\B7V2'

  tt = _mm_ror_ps(c3,1);		sum = _mm_mul_ps(tt,r1);
  tt = _mm_ror_ps(tt,1);		sum = _mm_add_ps(sum,_mm_mul_ps(tt,r2));
  tt = _mm_ror_ps(tt,1);		sum = _mm_add_ps(sum,_mm_mul_ps(tt,r3));
  __m128 mtL3 = _mm_xor_ps(sum,Sign_PNPN.m);

  // Dividing is FASTER than rcp_nr! (Because rcp_nr causes many register-memory RWs).
  RDet = _mm_div_ss(ZERONE.m, Det); // TODO: just 1.0f?
  RDet = _mm_shuffle_ps(RDet,RDet,0x00);

  // Devide the first 12 minterms with the determinant.
  mtL1 = _mm_mul_ps(mtL1, RDet);
  mtL2 = _mm_mul_ps(mtL2, RDet);
  mtL3 = _mm_mul_ps(mtL3, RDet);

  // Calculate the minterms of the forth line and devide by the determinant.
  tt = _mm_ror_ps(c2,1);		sum = _mm_mul_ps(tt,r1);
  tt = _mm_ror_ps(tt,1);		sum = _mm_add_ps(sum,_mm_mul_ps(tt,r2));
  tt = _mm_ror_ps(tt,1);		sum = _mm_add_ps(sum,_mm_mul_ps(tt,r3));
  __m128 mtL4 = _mm_xor_ps(sum,Sign_NPNP.m);
  mtL4 = _mm_mul_ps(mtL4, RDet);

  // Now we just have to transpose the minterms matrix.
  trns0 = _mm_unpacklo_ps(mtL1,mtL2);
  trns1 = _mm_unpacklo_ps(mtL3,mtL4);
  trns2 = _mm_unpackhi_ps(mtL1,mtL2);
  trns3 = _mm_unpackhi_ps(mtL3,mtL4);
  *res0 = _mm_movelh_ps(trns0,trns1);
  *res1 = _mm_movehl_ps(trns1,trns0);
  *res2 = _mm_movelh_ps(trns2,trns3);
  *res3 = _mm_movehl_ps(trns3,trns2);
}


struct sse_meta_f16
{
  typedef float meta_type;
  typedef sse_meta_f4 vector_type;
  sse_meta_f4 c[4];

  sse_meta_f16() { setZero(); }
  
  sse_meta_f16(float xx, float xy, float xz, float xw,
               float yx, float yy, float yz, float yw,
               float zx, float zy, float zz, float zw,
               float wx, float wy, float wz, float ww)
  { setValue(xx, xy, xz, xw, yz, yy, yz, yw, zx, zy, zz, zw, wx, wy, wz, ww); }

  sse_meta_f16(const sse_meta_f4& x, const sse_meta_f4& y, const sse_meta_f4& z, const sse_meta_f4& w)
  { setColumn(x, y, z, w); }

  sse_meta_f16(__m128 x, __m128 y, __m128 z, __m128 w)
  { setColumn(x, y, z, w); }

  inline void setValue(float xx, float xy, float xz, float xw,
                       float yx, float yy, float yz, float yw,
                       float zx, float zy, float zz, float zw,
                       float wx, float wy, float wz, float ww)
  {
    c[0].setValue(xx, yx, zx, wx);
    c[1].setValue(xy, yy, zy, wy);
    c[2].setValue(xz, yz, zz, wz);
    c[3].setValue(xw, yw, zw, ww);
  }

  inline void setColumn(const sse_meta_f4& x, const sse_meta_f4& y, const sse_meta_f4& z, const sse_meta_f4& w)
  {
    c[0] = x; c[1] = y; c[2] = z; c[3] = w;
  }

  inline void setColumn(__m128 x, __m128 y, __m128 z, __m128 w)
  {
    c[0].setValue(x); c[1].setValue(y); c[2].setValue(z); c[3].setValue(w);
  }

  inline void setIdentity()
  {
    c[0].setValue(1, 0, 0, 0);
    c[1].setValue(0, 1, 0, 0);
    c[2].setValue(0, 0, 1, 0);
    c[3].setValue(0, 0, 0, 1);
  }

  inline void setZero()
  {
    c[0].setValue(0);
    c[1].setValue(0);
    c[2].setValue(0);
    c[3].setValue(0);
  }

  inline const sse_meta_f4& getColumn(size_t i) const
  {
    return c[i];
  }

  inline sse_meta_f4& getColumn(size_t i) 
  {
    return c[i];
  }

  inline sse_meta_f4 getRow(size_t i) const
  {
    return sse_meta_f4(c[0][i], c[1][i], c[2][i], c[3][i]);
  }

  inline float operator () (size_t i, size_t j) const
  {
    return c[j][i];
  }

  inline float& operator () (size_t i, size_t j) 
  {
    return c[j][i];
  }

  inline sse_meta_f4 operator * (const sse_meta_f4& v) const
  {
    return sse_meta_f4(_mm_add_ps(_mm_add_ps(_mm_mul_ps(c[0].v, vec_splat(v.v, 0)), _mm_mul_ps(c[1].v, vec_splat(v.v, 1))), 
                                  _mm_add_ps(_mm_mul_ps(c[2].v, vec_splat(v.v, 2)), _mm_mul_ps(c[3].v, vec_splat(v.v, 3)))
                                  ));
  }

  inline sse_meta_f16 operator * (const sse_meta_f16& mat) const
  {
    return sse_meta_f16((*this) * mat.c[0], (*this) * mat.c[1], (*this) * mat.c[2], (*this) * mat.c[3]);
  }


  inline sse_meta_f16 operator + (const sse_meta_f16& mat) const
  {
    return sse_meta_f16(c[0] + mat.c[0], c[1] + mat.c[1], c[2] + mat.c[2], c[3] + mat.c[3]);
  }

  inline sse_meta_f16 operator - (const sse_meta_f16& mat) const
  {
    return sse_meta_f16(c[0] - mat.c[0], c[1] - mat.c[1], c[2] - mat.c[2], c[3] - mat.c[3]);
  }

  inline sse_meta_f16 operator + (float t_) const
  {
    sse_meta_f4 t(t_);
    return sse_meta_f16(c[0] + t, c[1] + t, c[2] + t, c[3] + t);
  }

  inline sse_meta_f16 operator - (float t_) const
  {
    sse_meta_f4 t(t_);
    return sse_meta_f16(c[0] - t, c[1] - t, c[2] - t, c[3] - t);
  }

  inline sse_meta_f16 operator * (float t_) const
  {
    sse_meta_f4 t(t_);
    return sse_meta_f16(c[0] * t, c[1] * t, c[2] * t, c[3] * t);
  }

  inline sse_meta_f16 operator / (float t_) const
  {
    sse_meta_f4 t(t_);
    return sse_meta_f16(c[0] / t, c[1] / t, c[2] / t, c[3] / t);
  }

  inline sse_meta_f16& operator *= (const sse_meta_f16& mat)
  {
    setColumn((*this) * mat.c[0], (*this) * mat.c[1], (*this) * mat.c[2], (*this) * mat.c[3]);
    return *this;
  }

  inline sse_meta_f16& operator += (const sse_meta_f16& mat)
  {
    c[0] += mat.c[0];
    c[1] += mat.c[1];
    c[2] += mat.c[2];
    c[3] += mat.c[3];
    return *this;
  }

  inline sse_meta_f16& operator -= (const sse_meta_f16& mat)
  {
    c[0] -= mat.c[0];
    c[1] -= mat.c[1];
    c[2] -= mat.c[2];
    c[3] -= mat.c[3];
    return *this;
  }

  inline sse_meta_f16& operator += (float t_)
  {
    sse_meta_f4 t(t_);
    c[0] += t;
    c[1] += t;
    c[2] += t;
    c[3] += t;
    return *this;
  }

  inline sse_meta_f16& operator -= (float t_)
  {
    sse_meta_f4 t(t_);
    c[0] -= t;
    c[1] -= t;
    c[2] -= t;
    c[3] -= t;
    return *this;
  }

  inline sse_meta_f16& operator *= (float t_)
  {
    sse_meta_f4 t(t_);
    c[0] *= t;
    c[1] *= t;
    c[2] *= t;
    c[3] *= t;
    return *this;
  }

  inline sse_meta_f16& operator /= (float t_)
  {
    sse_meta_f4 t(t_);
    c[0] /= t;
    c[1] /= t;
    c[2] /= t;
    c[3] /= t;
    return *this;
  }

  inline sse_meta_f16& abs()
  {
    c[0] = detail::abs(c[0]);
    c[1] = detail::abs(c[1]);
    c[2] = detail::abs(c[2]);
    c[3] = detail::abs(c[3]);
    return *this;
  }

  inline sse_meta_f16& inverse() 
  {
    __m128 r0, r1, r2, r3;
    detail::inverse(c[0].v, c[1].v, c[2].v, c[3].v, &r0, &r1, &r2, &r3);
    setColumn(r0, r1, r2, r3);
    return *this;
  }

  inline sse_meta_f16& transpose() 
  {
    __m128 r0, r1, r2, r3;
    detail::transpose(c[0].v, c[1].v, c[2].v, c[3].v, &r0, &r1, &r2, &r3);
    setColumn(r0, r1, r2, r3);
    return *this;
  }

  inline float determinant() const
  {
    __m128 Va, Vb, Vc;
    __m128 r1, r2, r3, tt, tt2;
    __m128 sum, Det;

    __m128 _L1 = c[0].v;
    __m128 _L2 = c[1].v;
    __m128 _L3 = c[2].v;
    __m128 _L4 = c[3].v;
    // Calculating the minterms for the first line.

    // _mm_ror_ps is just a macro using _mm_shuffle_ps().
    tt = _L4; tt2 = _mm_ror_ps(_L3,1); 
    Vc = _mm_mul_ps(tt2,_mm_ror_ps(tt,0));					// V3'·V4
    Va = _mm_mul_ps(tt2,_mm_ror_ps(tt,2));					// V3'·V4"
    Vb = _mm_mul_ps(tt2,_mm_ror_ps(tt,3));					// V3'·V4^

    r1 = _mm_sub_ps(_mm_ror_ps(Va,1),_mm_ror_ps(Vc,2));		// V3"·V4^ - V3^·V4"
    r2 = _mm_sub_ps(_mm_ror_ps(Vb,2),_mm_ror_ps(Vb,0));		// V3^·V4' - V3'·V4^
    r3 = _mm_sub_ps(_mm_ror_ps(Va,0),_mm_ror_ps(Vc,1));		// V3'·V4" - V3"·V4'

    tt = _L2;
    Va = _mm_ror_ps(tt,1);		sum = _mm_mul_ps(Va,r1);
    Vb = _mm_ror_ps(tt,2);		sum = _mm_add_ps(sum,_mm_mul_ps(Vb,r2));
    Vc = _mm_ror_ps(tt,3);		sum = _mm_add_ps(sum,_mm_mul_ps(Vc,r3));

    // Calculating the determinant.
    Det = _mm_mul_ps(sum,_L1);
    Det = _mm_add_ps(Det,_mm_movehl_ps(Det,Det));

    // Calculating the minterms of the second line (using previous results).
    tt = _mm_ror_ps(_L1,1);		sum = _mm_mul_ps(tt,r1);
    tt = _mm_ror_ps(tt,1);		sum = _mm_add_ps(sum,_mm_mul_ps(tt,r2));
    tt = _mm_ror_ps(tt,1);		sum = _mm_add_ps(sum,_mm_mul_ps(tt,r3));

    // Testing the determinant.
    Det = _mm_sub_ss(Det,_mm_shuffle_ps(Det,Det,1));
    return _mm_cvtss_f32(Det);
  }

  inline sse_meta_f16 transposeTimes(const sse_meta_f16& other) const
  {
    return sse_meta_f16(dot_prod4(c[0], other.c[0]), dot_prod4(c[0], other.c[1]), dot_prod4(c[0], other.c[2]), dot_prod4(c[0], other.c[3]),
                        dot_prod4(c[1], other.c[0]), dot_prod4(c[1], other.c[1]), dot_prod4(c[1], other.c[2]), dot_prod4(c[1], other.c[3]),
                        dot_prod4(c[2], other.c[0]), dot_prod4(c[2], other.c[1]), dot_prod4(c[2], other.c[2]), dot_prod4(c[2], other.c[3]),
                        dot_prod4(c[3], other.c[0]), dot_prod4(c[3], other.c[1]), dot_prod4(c[3], other.c[2]), dot_prod4(c[3], other.c[3]));
  }

  inline sse_meta_f16 timesTranspose(const sse_meta_f16& m) const
  {
    sse_meta_f16 tmp(m);
    return (*this) * tmp.transpose();
  }

  inline sse_meta_f4 transposeTimes(const sse_meta_f4& v) const
  {
    return sse_meta_f4(dot_prod4(c[0], v), dot_prod4(c[1], v), dot_prod4(c[2], v), dot_prod4(c[3], v));
  }

  inline float transposeDot(size_t i, const sse_meta_f4& v) const
  {
    return dot_prod4(c[i], v);
  }

  inline float dot(size_t i, const sse_meta_f4& v) const
  {
    return v[0] * c[0][i] + v[1] * c[1][i] + v[2] * c[2][i] + v[3] * c[3][i];
  }

};

static inline sse_meta_f16 abs(const sse_meta_f16& mat)
{
  return sse_meta_f16(abs(mat.getColumn(0)), abs(mat.getColumn(1)), abs(mat.getColumn(2)), abs(mat.getColumn(3)));
}

static inline sse_meta_f16 transpose(const sse_meta_f16& mat)
{
  __m128 r0, r1, r2, r3;
  transpose(mat.getColumn(0).v, mat.getColumn(1).v, mat.getColumn(2).v, mat.getColumn(3).v, &r0, &r1, &r2, &r3);
  return sse_meta_f16(r0, r1, r2, r3);
}


static inline sse_meta_f16 inverse(const sse_meta_f16& mat)
{
  __m128 r0, r1, r2, r3;
  inverse(mat.getColumn(0).v, mat.getColumn(1).v, mat.getColumn(2).v, mat.getColumn(3).v, &r0, &r1, &r2, &r3);
  return sse_meta_f16(r0, r1, r2, r3);
}

                                


} // detail
} // fcl


#endif

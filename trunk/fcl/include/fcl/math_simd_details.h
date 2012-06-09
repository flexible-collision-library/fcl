/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#include <xmmintrin.h>
#if defined (__SSE3__)
#include <pmmintrin.h>
#endif
#if defined (__SSE4__)
#include <smmintrin.h>
#endif


namespace fcl
{

/** \brief FCL internals. Ignore this :) unless you are God */
namespace details
{

const __m128  xmms_0 = {0.f, 0.f, 0.f, 0.f};
const __m128d xmmd_0 = {0, 0};

struct sse_meta_f4
{
  typedef float meta_type;

  union {float vs[4]; __m128 v; }; 
  sse_meta_f4() : v(_mm_set1_ps(0)) {}
  sse_meta_f4(float x) : v(_mm_set1_ps(x)) {}
  sse_meta_f4(float* px) : v(_mm_load_ps(px)) {}
  sse_meta_f4(__m128 x) : v(x) {}
  sse_meta_f4(float x, float y, float z, float w = 0) : v(_mm_setr_ps(x, y, z, w)) {}
  void setValue(float x, float y, float z, float w = 0) { v = _mm_setr_ps(x, y, z, w); }
  void setValue(float x) { v = _mm_set1_ps(x); }
  void negate() { v = _mm_sub_ps(xmms_0, v); }
  
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
  inline sse_meta_f4 operator - () const { return sse_meta_f4(_mm_sub_ps(xmms_0, v)); }
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

  inline void negate()
  {
    v[0] = _mm_sub_pd(xmmd_0, v[0]);
    v[1] = _mm_sub_pd(xmmd_0, v[1]);
  }

  inline void* operator new [] (size_t n)
  {
    return _mm_malloc(n, 16);
  }

  inline void operator delete [] (void* x)
  {
    if(x) _mm_free(x);
  }

  double operator [] (size_t i) const { return vs[i]; }
  double& operator [] (size_t i) { return vs[i]; }

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
  inline sse_meta_d4 operator - () const { return sse_meta_d4(_mm_sub_pd(xmmd_0, v[0]), _mm_sub_pd(xmmd_0, v[1])); }
} __attribute__ ((aligned (16)));


static inline sse_meta_f4 cross_prod(const sse_meta_f4& x, const sse_meta_f4& y)
{
  // set to a[1][2][0][3] , b[2][0][1][3]
  // multiply
  static const int s1 = _MM_SHUFFLE(3, 0, 2, 1);
  static const int s2 = _MM_SHUFFLE(3, 1, 0, 2);
  __m128 xa = _mm_mul_ps(_mm_shuffle_ps(x.v, x.v, s1), _mm_shuffle_ps(y.v, y.v, s2));

  // set to a[2][0][1][3] , b[1][2][0][3]
  // multiply
  __m128 xb = _mm_mul_ps(_mm_shuffle_ps(x.v, x.v, s2), _mm_shuffle_ps(y.v, y.v, s1));

  // subtract
  return sse_meta_f4(_mm_sub_ps(xa, xb));
}

static inline sse_meta_d4 cross_prod(const sse_meta_d4& x, const sse_meta_d4& y)
{
  static const int s0 = _MM_SHUFFLE2(0, 0);
  static const int s1 = _MM_SHUFFLE2(0, 1);
  static const int s2 = _MM_SHUFFLE2(1, 0);
  static const int s3 = _MM_SHUFFLE2(1, 1);  
  __m128d xa1 = _mm_mul_pd(_mm_shuffle_pd(x.v[0], x.v[1], s1), _mm_shuffle_pd(y.v[1], y.v[0], s0));
  __m128d ya1 = _mm_mul_pd(_mm_shuffle_pd(x.v[0], x.v[1], s2), _mm_shuffle_pd(y.v[0], y.v[1], s3));
  
  __m128d xa2 = _mm_mul_pd(_mm_shuffle_pd(x.v[1], x.v[0], s0), _mm_shuffle_pd(y.v[0], y.v[1], s1));
  __m128d ya2 = _mm_mul_pd(_mm_shuffle_pd(x.v[0], y.v[1], s3), _mm_shuffle_pd(y.v[0], y.v[1], s2));

  return sse_meta_d4(_mm_sub_pd(xa1, xa2), _mm_sub_pd(ya1, ya2));
}

static inline float dot_prod(const sse_meta_f4& x, const sse_meta_f4& y)
{
#if defined (__SSE4__)
  return _mm_cvtss_f32(_mm_dp_ps(x.v, y.v, 0x71));
#elif defined (__SSE3__)
  register __m128 t = _mm_mul_ps(x.v, y.v);
  t = _mm_hadd_ps(t, t);
  t = _mm_hadd_ps(t, t);
  return _mm_cvtss_f32(t);
#else
  register __m128 s = _mm_mul_ps(x.v, y.v);
  register __m128 r = _mm_add_ss(s, _mm_movehl_ps(s, s));
  r = _mm_add_ss(r, _mm_shuffle_ps(r, r, 1));
  return _mm_cvtss_f32(r);
#endif
}

static inline double dot_prod(const sse_meta_d4& x, const sse_meta_d4& y)
{
#if defined (__SSE4__)
  register __m128d t1 = _mm_dp_pd(x.v[0], y.v[0], 0x31);
  register __m128d t2 = _mm_dp_pd(x.v[1], y.v[1], 0x11);
  t1 = _mm_add_pd(t1, t2);
  double d;
  _mm_storel_pd(&d, t1);
  return d;
#elif defined (__SSE3__)
  double d;
  register __m128d t1 = _mm_mul_pd(x.v[0], y.v[0]);
  register __m128d t2 = _mm_mul_pd(x.v[1], y.v[1]);
  t1 = _mm_hadd_pd(t1, t1);
  t2 = _mm_hadd_pd(t2, t2);
  t1 = _mm_add_pd(t1, t2);
  _mm_storeh_pl(&d, t1);
  return d;
#else 
  double d;
  register __m128d t1 = _mm_mul_pd(x.v[0], y.v[0]);
  register __m128d t2 = _mm_mul_pd(x.v[1], y.v[1]);
  t1 = _mm_add_pd(t1, t2);
  t1 = _mm_add_pd(t1, _mm_shuffle_pd(t1, t1, 1));
  _mm_storel_pd(&d, t1);
  return d;
#endif
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
  static const union { int i[4]; __m128 m; } abs4mask = {0x7fffffff, 0x7fffffff, 0x7fffffff, 0x7fffffff};
  return sse_meta_f4(_mm_and_ps(x.v, abs4mask.m));
}

static inline sse_meta_d4 abs(const sse_meta_d4& x)
{
  static const union { int64_t i[2]; __m128d m; } abs2mask = {0x7fffffffffffffff, 0x7fffffffffffffff};
  return sse_meta_d4(_mm_and_pd(x.v[0], abs2mask.m), _mm_and_pd(x.v[1], abs2mask.m));
}

static inline bool equal(const sse_meta_f4& x, const sse_meta_f4& y, float epsilon)
{
  epsilon = 1e-3;
  //static const union { int i[4]; __m128 m; } abs4mask = {0x7fffffff, 0x7fffffff, 0x7fffffff, 0x7fffffff};
  //return (_mm_movemask_ps(_mm_cmplt_ps(_mm_and_ps(_mm_sub_ps(x.v, y.v), abs4mask.m), _mm_set1_ps(epsilon))) == 0x0f);
  register __m128 d = _mm_sub_ps(x.v, y.v);
  register __m128 e = _mm_set1_ps(epsilon);
  return ((_mm_movemask_ps(_mm_cmplt_ps(d, e)) & 0x7) == 0x7) && ((_mm_movemask_ps(_mm_cmpgt_ps(d, _mm_sub_ps(xmms_0, e))) & 0x7) == 0x7);
}

static inline bool equal(const sse_meta_d4& x, const sse_meta_d4& y, double epsilon)
{
  // static const union { int64 i[2]; __m128d m; } abs2mask = {0x7fffffffffffffff, 0x7fffffffffffffff};
  // return (_mm_movemask_pd(_mm_cmplt_pd(_mm_and_pd(_mm_sub_pd(x.v[0], y.v[0]), abs2mask.m), _mm_set1_pd(epsilon))) == 0x3) && (_mm_movemask_pd(_mm_cmplt_pd(_mm_and_pd(_mm_sub_pd(x.v[1], y.v[1]), abs2mask.m), _mm_set1_pd(epsilon))) == 0x3);

  register __m128d d = _mm_sub_pd(x.v[0], y.v[0]);
  register __m128d e = _mm_set1_pd(epsilon);
  
  if(_mm_movemask_pd(_mm_cmplt_pd(d, e)) != 0x3) return false;
  if(_mm_movemask_pd(_mm_cmpgt_pd(d, _mm_sub_pd(xmmd_0, e))) != 0x3) return false;
  
  d = _mm_sub_pd(x.v[1], y.v[1]);
  if((_mm_movemask_pd(_mm_cmplt_pd(d, e)) & 0x1) != 0x1) return false;
  if((_mm_movemask_pd(_mm_cmpgt_pd(d, _mm_sub_pd(xmmd_0, e))) & 0x1) != 0x1) return false;
  return true;
}



} // details
} // fcl


#endif

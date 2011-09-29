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

#ifndef COLLISION_CHECKING_VEC_3F_H
#define COLLISION_CHECKING_VEC_3F_H

#include "fcl/BVH_internal.h"
#include <cmath>
#include <cstdlib>
#include <algorithm>

/** \brief Main namespace */
namespace fcl
{


#if COLLISION_USE_SSE
#include <xmmintrin.h>
#include <pmmintrin.h>
  inline __m128 _mm_cross_ps(__m128 a , __m128 b)
  {
    // set to a[1][2][0][3] , b[2][0][1][3]
    // multiply
    __m128 xa = _mm_mul_ps(
                _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1)),
                _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 1, 0, 2)));

    // set to a[2][0][1][3] , b[1][2][0][3]
    // multiply
    __m128 xb = _mm_mul_ps(
                _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 1, 0, 2)),
                _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1 )));

    // subtract
    return _mm_sub_ps(xa, xb);
  }

  const __m128  xmms_0 = {0.f, 0.f, 0.f, 0.f};

  union ieee754_QNAN
  {
    const float f;
    struct
    {
      const unsigned int mantissa:23, exp:8, sign:1;
    };

    ieee754_QNAN() : f(0.0f), mantissa(0x7FFFFF), exp(0xFF), sign(0x0) {}
  } __attribute__ ((aligned (16)));


  class Vec3f
  {
  public:
    /** \brief vector data */
    union {float v_[4]; __m128 v4; } __attribute__ ((aligned (16)));

    Vec3f() { v4 = _mm_set1_ps(0); }

    Vec3f(const Vec3f& other)
    {
      v4 = other.v4;
    }

    Vec3f(const float* v)
    {
      v_[0] = v[0];
      v_[1] = v[1];
      v_[2] = v[2];
      v_[3] = 0.0f;
    }

    Vec3f(float x, float y, float z)
    {
      v_[0] = x;
      v_[1] = y;
      v_[2] = z;
      v_[3] = 0.0f;
    }


    Vec3f(__m128 v)
    {
      v4 = v;
    }

    virtual ~Vec3f() {}

    /** \brief Get the ith element */
    inline float operator [] (size_t i) const
    {
      return v_[i];
    }

    inline float& operator[] (size_t i)
    {
      return v_[i];
    }

    /** \brief Add the other vector */
    inline Vec3f& operator += (const Vec3f& other)
    {
      v4 = _mm_add_ps(v4, other.v4);
      return *this;
    }


    /** \brief Minus the other vector */
    inline Vec3f& operator -= (const Vec3f& other)
    {
      v4 = _mm_sub_ps(v4, other.v4);
      return *this;
    }

    inline Vec3f& operator *= (BVH_REAL t)
    {
      v4 = _mm_mul_ps(_mm_set1_ps(t), v4);
      return *this;
    }

    /** \brief Negate the vector */
    inline void negate()
    {
      v4 = _mm_sub_ps(xmms_0, v4);
    }

    /** \brief Return a negated vector */
    inline Vec3f operator - () const
    {
      return Vec3f(_mm_sub_ps(xmms_0, v4));
    }


    /** \brief Return a summation vector */
    inline Vec3f operator + (const Vec3f& other) const
    {
      return Vec3f(_mm_add_ps(v4, other.v4));
    }

    /** \brief Return a substraction vector */
    inline Vec3f operator - (const Vec3f& other) const
    {
      return Vec3f(_mm_sub_ps(v4, other.v4));
    }

    /** \brief Scale the vector by t */
    inline Vec3f operator * (float t) const
    {
      return Vec3f(_mm_mul_ps(_mm_set1_ps(t), v4));
    }

    /** \brief Return the cross product with another vector */
    inline Vec3f cross(const Vec3f& other) const
    {
      return Vec3f(_mm_cross_ps(v4, other.v4));
    }

    /** \brief Return the dot product with another vector */
    inline float dot(const Vec3f& other) const
    {
      float d;
      register __m128 t = _mm_mul_ps(other.v4, v4);
      t = _mm_hadd_ps(t, t);
      t = _mm_hadd_ps(t, t);
      _mm_store_ss(&d, t);
      return d;
    }

    /** \brief Normalization */
    inline bool normalize()
    {
      float sqr_length = sqrLength();
      if(sqr_length > EPSILON)
      {
        float inv_length = 1.0 / sqrt(sqr_length);
        v4 = _mm_mul_ps(_mm_set1_ps(inv_length), v4);
        return true;
      }
      return false;
    }

    /** \brief Return vector length */
    inline float length() const
    {
      return sqrt(sqrLength());
    }

    /** \brief Return vector square length */
    inline float sqrLength() const
    {
      float d;
      register __m128 t = _mm_mul_ps(v4, v4);
      t = _mm_hadd_ps(t, t);
      t = _mm_hadd_ps(t, t);
      _mm_store_ss(&d, t);
      return d;
    }

    /** \brief Set the vector using new values */
    inline Vec3f& setValue(float x, float y, float z)
    {
      v_[0] = x; v_[1] = y; v_[2] = z; v_[3] = 0.0f;
      return *this;
    }

    /** \brief Check whether two vectors are the same in abstracted value */
    inline bool equal(const Vec3f& other) const
    {
      return ((v_[0] - other.v_[0] < EPSILON) &&
             (v_[0] - other.v_[0] > -EPSILON) &&
             (v_[1] - other.v_[1] < EPSILON) &&
             (v_[1] - other.v_[1] > -EPSILON) &&
             (v_[2] - other.v_[2] < EPSILON) &&
             (v_[2] - other.v_[2] > -EPSILON));
    }

  private:
    /** \brief Tolerance for comparision */
    static const float EPSILON;

  };


  /** \brief The minimum of two vectors */
  inline Vec3f min(const Vec3f& a, const Vec3f& b)
  {
    Vec3f ret(_mm_min_ps(a.v4, b.v4));
    return ret;
  }

  /** \brief the maximum of two vectors */
  inline Vec3f max(const Vec3f& a, const Vec3f& b)
  {
    Vec3f ret(_mm_max_ps(a.v4, b.v4));
    return ret;
  }

  inline Vec3f abs(const Vec3f& v)
  {
    const ieee754_QNAN mask;
    __m128 abs4mask = _mm_load1_ps(&mask.f);
    return Vec3f(_mm_and_ps(abs4mask, v.v4));
  }
#else
/** \brief A class describing a three-dimensional vector */
  class Vec3f
  {
  public:
    /** \brief vector data */
    BVH_REAL v_[3];

    Vec3f() { v_[0] = 0; v_[1] = 0; v_[2] = 0; }

    Vec3f(const Vec3f& other)
    {
      v_[0] = other.v_[0];
      v_[1] = other.v_[1];
      v_[2] = other.v_[2];
    }

    Vec3f(const BVH_REAL* v)
    {
      v_[0] = v[0];
      v_[1] = v[1];
      v_[2] = v[2];
    }

    Vec3f(BVH_REAL x, BVH_REAL y, BVH_REAL z)
    {
      v_[0] = x;
      v_[1] = y;
      v_[2] = z;
    }

    virtual ~Vec3f() {}

    /** \brief Get the ith element */
    inline BVH_REAL operator [] (size_t i) const
    {
      return v_[i];
    }

    inline BVH_REAL& operator[] (size_t i)
    {
      return v_[i];
    }

    /** \brief Add the other vector */
    inline Vec3f& operator += (const Vec3f& other)
    {
      v_[0] += other.v_[0];
      v_[1] += other.v_[1];
      v_[2] += other.v_[2];
      return *this;
    }


    /** \brief Minus the other vector */
    inline Vec3f& operator -= (const Vec3f& other)
    {
      v_[0] -= other.v_[0];
      v_[1] -= other.v_[1];
      v_[2] -= other.v_[2];
      return *this;
    }

    inline Vec3f& operator *= (BVH_REAL t)
    {
      v_[0] *= t;
      v_[1] *= t;
      v_[2] *= t;
      return *this;
    }

    /** \brief Negate the vector */
    inline void negate()
    {
      v_[0] = - v_[0];
      v_[1] = - v_[1];
      v_[2] = - v_[2];
    }

    /** \brief Return a negated vector */
    inline Vec3f operator - () const
    {
      return Vec3f(-v_[0], -v_[1], -v_[2]);
    }

    /** \brief Return a summation vector */
    inline Vec3f operator + (const Vec3f& other) const
    {
      return Vec3f(v_[0] + other.v_[0], v_[1] + other.v_[1], v_[2] + other.v_[2]);
    }

    /** \brief Return a substraction vector */
    inline Vec3f operator - (const Vec3f& other) const
    {
      return Vec3f(v_[0] - other.v_[0], v_[1] - other.v_[1], v_[2] - other.v_[2]);
    }

    /** \brief Scale the vector by t */
    inline Vec3f operator * (BVH_REAL t) const
    {
      return Vec3f(v_[0] * t, v_[1] * t, v_[2] * t);
    }

    /** \brief Return the cross product with another vector */
    inline Vec3f cross(const Vec3f& other) const
    {
      return Vec3f(v_[1] * other.v_[2] - v_[2] * other.v_[1],
                    v_[2] * other.v_[0] - v_[0] * other.v_[2],
                    v_[0] * other.v_[1] - v_[1] * other.v_[0]);
    }

    /** \brief Return the dot product with another vector */
    inline BVH_REAL dot(const Vec3f& other) const
    {
      return v_[0] * other.v_[0] + v_[1] * other.v_[1] + v_[2] * other.v_[2];
    }

    /** \brief Normalization */
    inline bool normalize()
    {
      BVH_REAL sqr_length = v_[0] * v_[0] + v_[1] * v_[1] + v_[2] * v_[2];
      if(sqr_length > EPSILON * EPSILON)
      {
        BVH_REAL inv_length = (BVH_REAL)1.0 / (BVH_REAL)sqrt(sqr_length);
        v_[0] *= inv_length;
        v_[1] *= inv_length;
        v_[2] *= inv_length;
        return true;
      }
      return false;
    }

    /** \brief Return vector length */
    inline BVH_REAL length() const
    {
      return sqrt(v_[0] * v_[0] + v_[1] * v_[1] + v_[2] * v_[2]);
    }

    /** \brief Return vector square length */
    inline BVH_REAL sqrLength() const
    {
      return v_[0] * v_[0] + v_[1] * v_[1] + v_[2] * v_[2];
    }

    /** \brief Set the vector using new values */
    inline Vec3f& setValue(BVH_REAL x, BVH_REAL y, BVH_REAL z)
    {
      v_[0] = x; v_[1] = y; v_[2] = z;
      return *this;
    }

    /** \brief Check whether two vectors are the same in value */
    inline bool equal(const Vec3f& other) const
    {
      return ((v_[0] - other.v_[0] < EPSILON) &&
             (v_[0] - other.v_[0] > -EPSILON) &&
             (v_[1] - other.v_[1] < EPSILON) &&
             (v_[1] - other.v_[1] > -EPSILON) &&
             (v_[2] - other.v_[2] < EPSILON) &&
             (v_[2] - other.v_[2] > -EPSILON));
    }

  private:
    /** \brief Tolerance for comparision */
    static const BVH_REAL EPSILON;
  };


  /** \brief The minimum of two vectors */
  inline Vec3f min(const Vec3f& a, const Vec3f& b)
  {
    Vec3f ret(std::min(a[0], b[0]), std::min(a[1], b[1]), std::min(a[2], b[2]));
    return ret;
  }

  /** \brief the maximum of two vectors */
  inline Vec3f max(const Vec3f& a, const Vec3f& b)
  {
    Vec3f ret(std::max(a[0], b[0]), std::max(a[1], b[1]), std::max(a[2], b[2]));
    return ret;
  }

  inline Vec3f abs(const Vec3f& v)
  {
    BVH_REAL x = v[0] < 0 ? -v[0] : v[0];
    BVH_REAL y = v[1] < 0 ? -v[1] : v[1];
    BVH_REAL z = v[2] < 0 ? -v[2] : v[2];

    return Vec3f(x, y, z);
  }
#endif

  /** \brief M * v */
  Vec3f matMulVec(const Vec3f M[3], const Vec3f& v);

  /** \brief M' * v */
  Vec3f matTransMulVec(const Vec3f M[3], const Vec3f& v);

  /** \brief v' * M * v */
  BVH_REAL quadraticForm(const Vec3f M[3], const Vec3f& v);

  /** \brief S * M * S' */
  void tensorTransform(const Vec3f M[3], const Vec3f S[3], Vec3f newM[3]);

  /** \brief A * B */
  void matMulMat(const Vec3f M1[3], const Vec3f M2[3], Vec3f newM[3]);

  /** \brief The relative transform from (R1, T1) to (R2, T2) */
  void relativeTransform(const Vec3f R1[3], const Vec3f& T1, const Vec3f R2[3], const Vec3f& T2, Vec3f R[3], Vec3f& T);

  /** \brief compute eigen values and vectors */
  void matEigen(Vec3f a[3], BVH_REAL dout[3], Vec3f vout[3]);


}

#endif

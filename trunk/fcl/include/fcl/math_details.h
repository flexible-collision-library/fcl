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

#ifndef FCL_MATH_DETAILS_H
#define FCL_MATH_DETAILS_H


#include <cmath>
#include <algorithm>
#include <cstring>

namespace fcl
{

namespace details
{


template <typename T>
struct Vec3Data
{
  typedef T meta_type;

  T vs[3];
  Vec3Data() { setValue(0); }
  Vec3Data(T x)
  {
    setValue(x);
  }

  Vec3Data(T* x)
  {
    memcpy(vs, x, sizeof(T) * 3);
  }

  Vec3Data(T x, T y, T z)
  {
    setValue(x, y, z);
  }

  inline void setValue(T x, T y, T z)
  {
    vs[0] = x; vs[1] = y; vs[2] = z;
  }

  inline void setValue(T x)
  {
    vs[0] = x; vs[1] = x; vs[2] = x;
  }

  inline void negate()
  {
    vs[0] = -vs[0]; vs[1] = -vs[1]; vs[2] = -vs[2];
  }

  inline Vec3Data<T>& ubound(const Vec3Data<T>& u) 
  {
    vs[0] = std::min(vs[0], u.vs[0]);
    vs[1] = std::min(vs[1], u.vs[1]);
    vs[2] = std::min(vs[2], u.vs[2]);
    return *this;
  }

  inline Vec3Data<T>& lbound(const Vec3Data<T>& l)
  {
    vs[0] = std::max(vs[0], l.vs[0]);
    vs[1] = std::max(vs[1], l.vs[1]);
    vs[2] = std::max(vs[2], l.vs[2]);
    return *this;
  }

  T operator [] (size_t i) const { return vs[i]; }
  T& operator [] (size_t i) { return vs[i]; }

  inline Vec3Data<T> operator + (const Vec3Data<T>& other) const { return Vec3Data<T>(vs[0] + other.vs[0], vs[1] + other.vs[1], vs[2] + other.vs[2]); }
  inline Vec3Data<T> operator - (const Vec3Data<T>& other) const { return Vec3Data<T>(vs[0] - other.vs[0], vs[1] - other.vs[1], vs[2] - other.vs[2]); }
  inline Vec3Data<T> operator * (const Vec3Data<T>& other) const { return Vec3Data<T>(vs[0] * other.vs[0], vs[1] * other.vs[1], vs[2] * other.vs[2]); }
  inline Vec3Data<T> operator / (const Vec3Data<T>& other) const { return Vec3Data<T>(vs[0] / other.vs[0], vs[1] / other.vs[1], vs[2] / other.vs[2]); }
  inline Vec3Data<T>& operator += (const Vec3Data<T>& other) { vs[0] += other.vs[0]; vs[1] += other.vs[1]; vs[2] += other.vs[2]; return *this; }
  inline Vec3Data<T>& operator -= (const Vec3Data<T>& other) { vs[0] -= other.vs[0]; vs[1] -= other.vs[1]; vs[2] -= other.vs[2]; return *this; }
  inline Vec3Data<T>& operator *= (const Vec3Data<T>& other) { vs[0] *= other.vs[0]; vs[1] *= other.vs[1]; vs[2] *= other.vs[2]; return *this; }
  inline Vec3Data<T>& operator /= (const Vec3Data<T>& other) { vs[0] /= other.vs[0]; vs[1] /= other.vs[1]; vs[2] /= other.vs[2]; return *this; }
  inline Vec3Data<T> operator + (T t) const { return Vec3Data<T>(vs[0] + t, vs[1] + t, vs[2] + t); }
  inline Vec3Data<T> operator - (T t) const { return Vec3Data<T>(vs[0] - t, vs[1] - t, vs[2] - t); }
  inline Vec3Data<T> operator * (T t) const { return Vec3Data<T>(vs[0] * t, vs[1] * t, vs[2] * t); }
  inline Vec3Data<T> operator / (T t) const { T inv_t = 1.0 / t; return Vec3Data<T>(vs[0] * inv_t, vs[1] * inv_t, vs[2] * inv_t); }
  inline Vec3Data<T>& operator += (T t) { vs[0] += t; vs[1] += t; vs[2] += t; return *this; }
  inline Vec3Data<T>& operator -= (T t) { vs[0] -= t; vs[1] -= t; vs[2] -= t; return *this; }
  inline Vec3Data<T>& operator *= (T t) { vs[0] *= t; vs[1] *= t; vs[2] *= t; return *this; }
  inline Vec3Data<T>& operator /= (T t) { T inv_t = 1.0 / t; vs[0] *= inv_t; vs[1] *= inv_t; vs[2] *= inv_t; return *this; }
  inline Vec3Data<T> operator - () const { return Vec3Data<T>(-vs[0], -vs[1], -vs[2]); }
};


template <typename T>
static inline Vec3Data<T> cross_prod(const Vec3Data<T>& l, const Vec3Data<T>& r)
{
  return Vec3Data<T>(l.vs[1] * r.vs[2] - l.vs[2] * r.vs[1], 
                     l.vs[2] * r.vs[0] - l.vs[0] * r.vs[2],
                     l.vs[0] * r.vs[1] - l.vs[1] * r.vs[0]);
}

template <typename T>
static inline T dot_prod3(const Vec3Data<T>& l, const Vec3Data<T>& r)
{
  return l.vs[0] * r.vs[0] + l.vs[1] * r.vs[1] + l.vs[2] * r.vs[2];
}


template <typename T>
static inline Vec3Data<T> min(const Vec3Data<T>& x, const Vec3Data<T>& y)
{
  return Vec3Data<T>(std::min(x.vs[0], y.vs[0]), std::min(x.vs[1], y.vs[1]), std::min(x.vs[2], y.vs[2]));
}

template <typename T>
static inline Vec3Data<T> max(const Vec3Data<T>& x, const Vec3Data<T>& y)
{
  return Vec3Data<T>(std::max(x.vs[0], y.vs[0]), std::max(x.vs[1], y.vs[1]), std::max(x.vs[2], y.vs[2]));
}

template <typename T>
static inline Vec3Data<T> abs(const Vec3Data<T>& x)
{
  return Vec3Data<T>(std::abs(x.vs[0]), std::abs(x.vs[1]), std::abs(x.vs[2]));
}

template <typename T>
static inline bool equal(const Vec3Data<T>& x, const Vec3Data<T>& y, T epsilon)
{
  return ((x.vs[0] - y.vs[0] < epsilon) &&
          (x.vs[0] - y.vs[0] > -epsilon) &&
          (x.vs[1] - y.vs[1] < epsilon) &&
          (x.vs[1] - y.vs[1] > -epsilon) &&
          (x.vs[2] - y.vs[2] < epsilon) &&
          (x.vs[2] - y.vs[2] > -epsilon));
}

}

}

#endif

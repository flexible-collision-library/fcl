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

#ifndef FCL_VEC_3F_H
#define FCL_VEC_3F_H

#include "fcl/config.h"
#include "fcl/data_types.h"
#include "fcl/math/math_details.h"

#if FCL_HAVE_SSE
#  include "fcl/simd/math_simd_details.h"
#endif

#include <cmath>
#include <iostream>
#include <limits>


namespace fcl
{

/// @brief Vector3 class wrapper. The core data is in the template parameter class.
template <typename T>
class Vec3fX
{
public:
  typedef typename T::meta_type U;

  /// @brief interval vector3 data
  T data;

  Vec3fX() {}
  Vec3fX(const Vec3fX& other) : data(other.data) {}

  /// @brief create Vector (x, y, z)
  Vec3fX(U x, U y, U z) : data(x, y, z) {}

  /// @brief create vector (x, x, x)
  Vec3fX(U x) : data(x) {}

  /// @brief create vector using the internal data type
  Vec3fX(const T& data_) : data(data_) {}

  inline U operator [] (size_t i) const { return data[i]; }
  inline U& operator [] (size_t i) { return data[i]; }

  inline Vec3fX operator + (const Vec3fX& other) const { return Vec3fX(data + other.data); }
  inline Vec3fX operator - (const Vec3fX& other) const { return Vec3fX(data - other.data); }
  inline Vec3fX operator * (const Vec3fX& other) const { return Vec3fX(data * other.data); }
  inline Vec3fX operator / (const Vec3fX& other) const { return Vec3fX(data / other.data); }
  inline Vec3fX& operator += (const Vec3fX& other) { data += other.data; return *this; }
  inline Vec3fX& operator -= (const Vec3fX& other) { data -= other.data; return *this; }
  inline Vec3fX& operator *= (const Vec3fX& other) { data *= other.data; return *this; }
  inline Vec3fX& operator /= (const Vec3fX& other) { data /= other.data; return *this; }
  inline Vec3fX operator + (U t) const { return Vec3fX(data + t); }
  inline Vec3fX operator - (U t) const { return Vec3fX(data - t); }
  inline Vec3fX operator * (U t) const { return Vec3fX(data * t); }
  inline Vec3fX operator / (U t) const { return Vec3fX(data / t); }
  inline Vec3fX& operator += (U t) { data += t; return *this; }
  inline Vec3fX& operator -= (U t) { data -= t; return *this; }
  inline Vec3fX& operator *= (U t) { data *= t; return *this; }
  inline Vec3fX& operator /= (U t) { data /= t; return *this; }
  inline Vec3fX operator - () const { return Vec3fX(-data); }
  inline Vec3fX cross(const Vec3fX& other) const { return Vec3fX(details::cross_prod(data, other.data)); }
  inline U dot(const Vec3fX& other) const { return details::dot_prod3(data, other.data); }
  inline Vec3fX& normalize()
  {
    U sqr_length = details::dot_prod3(data, data);
    if(sqr_length > 0)
      *this /= (U)sqrt(sqr_length);
    return *this;
  }

  inline Vec3fX& normalize(bool* signal)
  {
    U sqr_length = details::dot_prod3(data, data);
    if(sqr_length > 0)
    {
      *this /= (U)sqrt(sqr_length);
      *signal = true;
    }
    else
      *signal = false;
    return *this;
  }

  inline Vec3fX& abs() 
  {
    data = abs(data);
    return *this;
  }

  inline U length() const { return sqrt(details::dot_prod3(data, data)); }
  inline U norm() const { return sqrt(details::dot_prod3(data, data)); }
  inline U sqrLength() const { return details::dot_prod3(data, data); }
  inline U squaredNorm() const { return details::dot_prod3(data, data); }
  inline void setValue(U x, U y, U z) { data.setValue(x, y, z); }
  inline void setValue(U x) { data.setValue(x); }
  inline void setZero () {data.setValue (0); }
  inline bool equal(const Vec3fX& other, U epsilon = std::numeric_limits<U>::epsilon() * 100) const { return details::equal(data, other.data, epsilon); }
  inline Vec3fX<T>& negate() { data.negate(); return *this; }

  bool operator == (const Vec3fX& other) const
  {
    return equal(other, 0);
  }

  bool operator != (const Vec3fX& other) const
  {
    return !(*this == other);
  }


  inline Vec3fX<T>& ubound(const Vec3fX<T>& u)
  {
    data.ubound(u.data);
    return *this;
  }

  inline Vec3fX<T>& lbound(const Vec3fX<T>& l)
  {
    data.lbound(l.data);
    return *this;
  }

  bool isZero() const
  {
    return (data[0] == 0) && (data[1] == 0) && (data[2] == 0);
  }

};

template<typename T>
static inline Vec3fX<T> normalize(const Vec3fX<T>& v)
{
  typename T::meta_type sqr_length = details::dot_prod3(v.data, v.data);
  if(sqr_length > 0)
    return v / (typename T::meta_type)sqrt(sqr_length);
  else
    return v;
}

template <typename T>
static inline typename T::meta_type triple(const Vec3fX<T>& x, const Vec3fX<T>& y, const Vec3fX<T>& z)
{
  return x.dot(y.cross(z));
}

template <typename T>
std::ostream& operator << (std::ostream& out, const Vec3fX<T>& x)
{
  out << x[0] << " " << x[1] << " " << x[2];
  return out;
}

template <typename T>
static inline Vec3fX<T> min(const Vec3fX<T>& x, const Vec3fX<T>& y)
{
  return Vec3fX<T>(details::min(x.data, y.data));
}

template <typename T>
static inline Vec3fX<T> max(const Vec3fX<T>& x, const Vec3fX<T>& y)
{
  return Vec3fX<T>(details::max(x.data, y.data));
}

template <typename T>
static inline Vec3fX<T> abs(const Vec3fX<T>& x)
{
  return Vec3fX<T>(details::abs(x.data));
}

template <typename T>
void generateCoordinateSystem(const Vec3fX<T>& w, Vec3fX<T>& u, Vec3fX<T>& v)
{
  typedef typename T::meta_type U;
  U inv_length;
  if(std::abs(w[0]) >= std::abs(w[1]))
  {
    inv_length = (U)1.0 / sqrt(w[0] * w[0] + w[2] * w[2]);
    u[0] = -w[2] * inv_length;
    u[1] = (U)0;
    u[2] = w[0] * inv_length;
    v[0] = w[1] * u[2];
    v[1] = w[2] * u[0] - w[0] * u[2];
    v[2] = -w[1] * u[0];
  }
  else
  {
    inv_length = (U)1.0 / sqrt(w[1] * w[1] + w[2] * w[2]);
    u[0] = (U)0;
    u[1] = w[2] * inv_length;
    u[2] = -w[1] * inv_length;
    v[0] = w[1] * u[2] - w[2] * u[1];
    v[1] = -w[0] * u[2];
    v[2] = w[0] * u[1];
  }
}

#if FCL_HAVE_SSE
  typedef Vec3fX<details::sse_meta_f4> Vec3f;
#else
  typedef Vec3fX<details::Vec3Data<FCL_REAL> > Vec3f;
#endif

static inline std::ostream& operator << (std::ostream& o, const Vec3f& v)
{
  o << "(" << v[0] << " " << v[1] << " " << v[2] << ")";
  return o;
}

 template <typename T>
   inline Vec3fX <T> operator * (const typename Vec3fX <T>::U& t,
				 const Vec3fX <T>& v)
   {
     return Vec3fX <T> (v.data * t);
   }


}


#endif

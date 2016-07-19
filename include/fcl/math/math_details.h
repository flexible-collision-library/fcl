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


template<typename T>
struct Matrix3Data
{
  typedef T meta_type;
  typedef Vec3Data<T> vector_type;
  
  Vec3Data<T> rs[3];
  Matrix3Data() {};

  Matrix3Data(T xx, T xy, T xz,
              T yx, T yy, T yz,
              T zx, T zy, T zz)
  {
    setValue(xx, xy, xz,
             yx, yy, yz,
             zx, zy, zz);
  }

  Matrix3Data(const Vec3Data<T>& v1, const Vec3Data<T>& v2, const Vec3Data<T>& v3)
  {
    rs[0] = v1;
    rs[1] = v2;
    rs[2] = v3;
  }

  Matrix3Data(const Matrix3Data<T>& other)
  {
    rs[0] = other.rs[0];
    rs[1] = other.rs[1];
    rs[2] = other.rs[2];
  }

  inline Vec3Data<T> getColumn(size_t i) const
  {
    return Vec3Data<T>(rs[0][i], rs[1][i], rs[2][i]);
  }

  inline const Vec3Data<T>& getRow(size_t i) const
  {
    return rs[i];
  }

  inline T operator() (size_t i, size_t j) const 
  {
    return rs[i][j];
  }

  inline T& operator() (size_t i, size_t j)
  {
    return rs[i][j];
  }

  inline Vec3Data<T> operator * (const Vec3Data<T>& v) const
  {
    return Vec3Data<T>(dot_prod3(rs[0], v), dot_prod3(rs[1], v), dot_prod3(rs[2], v));
  }

  inline Matrix3Data<T> operator * (const Matrix3Data<T>& other) const
  {
    return Matrix3Data<T>(other.transposeDotX(rs[0]), other.transposeDotY(rs[0]), other.transposeDotZ(rs[0]),
                          other.transposeDotX(rs[1]), other.transposeDotY(rs[1]), other.transposeDotZ(rs[1]),
                          other.transposeDotX(rs[2]), other.transposeDotY(rs[2]), other.transposeDotZ(rs[2]));
  }

  inline Matrix3Data<T> operator + (const Matrix3Data<T>& other) const
  {
    return Matrix3Data<T>(rs[0] + other.rs[0], rs[1] + other.rs[1], rs[2] + other.rs[2]);
  }

  inline Matrix3Data<T> operator - (const Matrix3Data<T>& other) const
  {
    return Matrix3Data<T>(rs[0] - other.rs[0], rs[1] - other.rs[1], rs[2] - other.rs[2]);
  }

  inline Matrix3Data<T> operator + (T c) const
  {
    return Matrix3Data<T>(rs[0] + c, rs[1] + c, rs[2] + c);
  }

  inline Matrix3Data<T> operator - (T c) const
  {
    return Matrix3Data<T>(rs[0] - c, rs[1] - c, rs[2] - c);
  }

  inline Matrix3Data<T> operator * (T c) const
  {
    return Matrix3Data<T>(rs[0] * c, rs[1] * c, rs[2] * c);
  }

  inline Matrix3Data<T> operator / (T c) const
  {
    return Matrix3Data<T>(rs[0] / c, rs[1] / c, rs[2] / c);
  }

  inline Matrix3Data<T>& operator *= (const Matrix3Data<T>& other)
  {
    rs[0].setValue(other.transposeDotX(rs[0]), other.transposeDotY(rs[0]), other.transposeDotZ(rs[0]));
    rs[1].setValue(other.transposeDotX(rs[1]), other.transposeDotY(rs[1]), other.transposeDotZ(rs[1]));
    rs[2].setValue(other.transposeDotX(rs[2]), other.transposeDotY(rs[2]), other.transposeDotZ(rs[2]));
    return *this;
  }

  inline Matrix3Data<T>& operator += (const Matrix3Data<T>& other)
  {
    rs[0] += other.rs[0];
    rs[1] += other.rs[1];
    rs[2] += other.rs[2];
    return *this;
  }

  inline Matrix3Data<T>& operator -= (const Matrix3Data<T>& other)
  {
    rs[0] -= other.rs[0];
    rs[1] -= other.rs[1];
    rs[2] -= other.rs[2];
    return *this;
  }

  inline Matrix3Data<T>& operator += (T c)
  {
    rs[0] += c;
    rs[1] += c;
    rs[2] += c;
    return *this;
  }

  inline Matrix3Data<T>& operator - (T c)
  {
    rs[0] -= c;
    rs[1] -= c;
    rs[2] -= c;
    return *this;
  }

  inline Matrix3Data<T>& operator * (T c)
  {
    rs[0] *= c;
    rs[1] *= c;
    rs[2] *= c;
    return *this;
  }

  inline Matrix3Data<T>& operator / (T c)
  {
    rs[0] /= c;
    rs[1] /= c;
    rs[2] /= c;
    return *this;
  }


  void setIdentity() 
  {
    setValue((T)1, (T)0, (T)0,
             (T)0, (T)1, (T)0,
             (T)0, (T)0, (T)1);
  }

  void setZero()
  {
    setValue((T)0);
  }

  static const Matrix3Data<T>& getIdentity()
  {
    static const Matrix3Data<T> I((T)1, (T)0, (T)0,
                                  (T)0, (T)1, (T)0,
                                  (T)0, (T)0, (T)1);
    return I;
  }

  T determinant() const
  {
    return dot_prod3(rs[0], cross_prod(rs[1], rs[2]));
  }

  Matrix3Data<T>& transpose()
  {
    register T tmp = rs[0][1];
    rs[0][1] = rs[1][0];
    rs[1][0] = tmp;
    
    tmp = rs[0][2];
    rs[0][2] = rs[2][0];
    rs[2][0] = tmp;
    
    tmp = rs[2][1];
    rs[2][1] = rs[1][2];
    rs[1][2] = tmp;
    return *this;
  }

  Matrix3Data<T>& inverse()
  {
    T det = determinant();
    register T inrsdet = 1 / det;
    
    setValue((rs[1][1] * rs[2][2] - rs[1][2] * rs[2][1]) * inrsdet,
             (rs[0][2] * rs[2][1] - rs[0][1] * rs[2][2]) * inrsdet,
             (rs[0][1] * rs[1][2] - rs[0][2] * rs[1][1]) * inrsdet,
             (rs[1][2] * rs[2][0] - rs[1][0] * rs[2][2]) * inrsdet,
             (rs[0][0] * rs[2][2] - rs[0][2] * rs[2][0]) * inrsdet,
             (rs[0][2] * rs[1][0] - rs[0][0] * rs[1][2]) * inrsdet,
             (rs[1][0] * rs[2][1] - rs[1][1] * rs[2][0]) * inrsdet,
             (rs[0][1] * rs[2][0] - rs[0][0] * rs[2][1]) * inrsdet,
             (rs[0][0] * rs[1][1] - rs[0][1] * rs[1][0]) * inrsdet);

    return *this;
  }

  Matrix3Data<T> transposeTimes(const Matrix3Data<T>& m) const
  {
    return Matrix3Data<T>(rs[0][0] * m.rs[0][0] + rs[1][0] * m.rs[1][0] + rs[2][0] * m.rs[2][0],
                          rs[0][0] * m.rs[0][1] + rs[1][0] * m.rs[1][1] + rs[2][0] * m.rs[2][1],
                          rs[0][0] * m.rs[0][2] + rs[1][0] * m.rs[1][2] + rs[2][0] * m.rs[2][2],
                          rs[0][1] * m.rs[0][0] + rs[1][1] * m.rs[1][0] + rs[2][1] * m.rs[2][0],
                          rs[0][1] * m.rs[0][1] + rs[1][1] * m.rs[1][1] + rs[2][1] * m.rs[2][1],
                          rs[0][1] * m.rs[0][2] + rs[1][1] * m.rs[1][2] + rs[2][1] * m.rs[2][2],
                          rs[0][2] * m.rs[0][0] + rs[1][2] * m.rs[1][0] + rs[2][2] * m.rs[2][0],
                          rs[0][2] * m.rs[0][1] + rs[1][2] * m.rs[1][1] + rs[2][2] * m.rs[2][1],
                          rs[0][2] * m.rs[0][2] + rs[1][2] * m.rs[1][2] + rs[2][2] * m.rs[2][2]);
  }
   
  Matrix3Data<T> timesTranspose(const Matrix3Data<T>& m) const
  {
    return Matrix3Data<T>(dot_prod3(rs[0], m[0]), dot_prod3(rs[0], m[1]), dot_prod3(rs[0], m[2]),
                          dot_prod3(rs[1], m[0]), dot_prod3(rs[1], m[1]), dot_prod3(rs[1], m[2]),
                          dot_prod3(rs[2], m[0]), dot_prod3(rs[2], m[1]), dot_prod3(rs[2], m[2]));
  }


  Vec3Data<T> transposeTimes(const Vec3Data<T>& v) const
  {
    return Vec3Data<T>(transposeDotX(v), transposeDotY(v), transposeDotZ(v));
  }

  inline T transposeDotX(const Vec3Data<T>& v) const
  {
    return rs[0][0] * v[0] + rs[1][0] * v[1] + rs[2][0] * v[2];
  }

  inline T transposeDotY(const Vec3Data<T>& v) const
  {
    return rs[0][1] * v[0] + rs[1][1] * v[1] + rs[2][1] * v[2];
  }

  inline T transposeDotZ(const Vec3Data<T>& v) const
  {
    return rs[0][2] * v[0] + rs[1][2] * v[1] + rs[2][2] * v[2];
  }

  inline T transposeDot(size_t i, const Vec3Data<T>& v) const
  {
    return rs[0][i] * v[0] + rs[1][i] * v[1] + rs[2][i] * v[2];
  }

  inline T dotX(const Vec3Data<T>& v) const
  {
    return rs[0][0] * v[0] + rs[0][1] * v[1] + rs[0][2] * v[2];
  }

  inline T dotY(const Vec3Data<T>& v) const
  {
    return rs[1][0] * v[0] + rs[1][1] * v[1] + rs[1][2] * v[2];
  }

  inline T dotZ(const Vec3Data<T>& v) const
  {
    return rs[2][0] * v[0] + rs[2][1] * v[1] + rs[2][2] * v[2];
  }

  inline T dot(size_t i, const Vec3Data<T>& v) const
  {
    return rs[i][0] * v[0] + rs[i][1] * v[1] + rs[i][2] * v[2];
  }
    
  inline void setValue(T xx, T xy, T xz,
                       T yx, T yy, T yz,
                       T zx, T zy, T zz)
  {
    rs[0].setValue(xx, xy, xz);
    rs[1].setValue(yx, yy, yz);
    rs[2].setValue(zx, zy, zz);
  }
    
  inline void setValue(T x)
  {
    rs[0].setValue(x);
    rs[1].setValue(x);
    rs[2].setValue(x);
  }
};



template<typename T>
Matrix3Data<T> abs(const Matrix3Data<T>& m)
{
  return Matrix3Data<T>(abs(m.rs[0]), abs(m.rs[1]), abs(m.rs[2]));
}

template<typename T>
Matrix3Data<T> transpose(const Matrix3Data<T>& m)
{
  return Matrix3Data<T>(m.rs[0][0], m.rs[1][0], m.rs[2][0],
                        m.rs[0][1], m.rs[1][1], m.rs[2][1],
                        m.rs[0][2], m.rs[1][2], m.rs[2][2]);
}


template<typename T>
Matrix3Data<T> inverse(const Matrix3Data<T>& m)
{
  T det = m.determinant();
  T inrsdet = 1 / det;

  return Matrix3Data<T>((m.rs[1][1] * m.rs[2][2] - m.rs[1][2] * m.rs[2][1]) * inrsdet,
                        (m.rs[0][2] * m.rs[2][1] - m.rs[0][1] * m.rs[2][2]) * inrsdet,
                        (m.rs[0][1] * m.rs[1][2] - m.rs[0][2] * m.rs[1][1]) * inrsdet,
                        (m.rs[1][2] * m.rs[2][0] - m.rs[1][0] * m.rs[2][2]) * inrsdet,
                        (m.rs[0][0] * m.rs[2][2] - m.rs[0][2] * m.rs[2][0]) * inrsdet,
                        (m.rs[0][2] * m.rs[1][0] - m.rs[0][0] * m.rs[1][2]) * inrsdet,
                        (m.rs[1][0] * m.rs[2][1] - m.rs[1][1] * m.rs[2][0]) * inrsdet,
                        (m.rs[0][1] * m.rs[2][0] - m.rs[0][0] * m.rs[2][1]) * inrsdet,
                        (m.rs[0][0] * m.rs[1][1] - m.rs[0][1] * m.rs[1][0]) * inrsdet);
}

}

}

#endif

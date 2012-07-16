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

#ifndef FCL_MATRIX_3F_H
#define FCL_MATRIX_3F_H

#include "fcl/vec_3f.h"

namespace fcl
{


template<typename T>
class Matrix3fX
{
public:
  typedef typename T::meta_type U;
  typedef typename T::vector_type S;
  T data;
  
  Matrix3fX() {}
  Matrix3fX(U xx, U xy, U xz,
            U yx, U yy, U yz,
            U zx, U zy, U zz) : data(xx, xy, xz, yx, yy, yz, zx, zy, zz)
  {}

  Matrix3fX(const Vec3fX<S>& v1, const Vec3fX<S>& v2, const Vec3fX<S>& v3)
    : data(v1.data, v2.data, v3.data) {}
  
  Matrix3fX(const Matrix3fX<T>& other) : data(other.data) {}

  Matrix3fX(const T& data_) : data(data_) {}
  
  inline Vec3fX<S> getColumn(size_t i) const
  {
    return Vec3fX<S>(data.getColumn(i));
  }

  inline Vec3fX<S> getRow(size_t i) const
  {
    return Vec3fX<S>(data.getRow(i));
  }

  inline U operator () (size_t i, size_t j) const
  {
    return data(i, j);
  }

  inline U& operator () (size_t i, size_t j)
  {
    return data(i, j);
  }

  inline Vec3fX<S> operator * (const Vec3fX<S>& v) const
  {
    return Vec3fX<S>(data * v.data);
  }

  inline Matrix3fX<T> operator * (const Matrix3fX<T>& m) const
  {
    return Matrix3fX<T>(data * m.data);
  }

  inline Matrix3fX<T> operator + (const Matrix3fX<T>& other) const
  {
    return Matrix3fX<T>(data + other.data);
  }

  inline Matrix3fX<T> operator - (const Matrix3fX<T>& other) const
  {
    return Matrix3fX<T>(data - other.data);
  }

  inline Matrix3fX<T> operator + (U c) const
  {
    return Matrix3fX<T>(data + c);
  }

  inline Matrix3fX<T> operator - (U c) const
  {
    return Matrix3fX<T>(data - c);
  }

  inline Matrix3fX<T> operator * (U c) const
  {
    return Matrix3fX<T>(data * c);
  }

  inline Matrix3fX<T> operator / (U c) const
  {
    return Matrix3fX<T>(data / c);
  }

  inline Matrix3fX<T>& operator *= (const Matrix3fX<T>& other)
  {
    data *= other.data;
    return *this;
  }

  inline Matrix3fX<T>& operator += (const Matrix3fX<T>& other)
  {
    data += other.data;
    return *this;
  }

  inline Matrix3fX<T>& operator -= (const Matrix3fX<T>& other)
  {
    data -= other.data;
    return *this;
  }

  inline Matrix3fX<T>& operator += (U c) 
  {
    data += c;
    return *this;
  }

  inline Matrix3fX<T>& operator -= (U c)
  {
    data -= c;
    return *this;
  }

  inline Matrix3fX<T>& operator *= (U c)
  {
    data *= c;
    return *this;
  }

  inline Matrix3fX<T>& operator /= (U c)
  {
    data /= c;
    return *this;
  }

  inline void setIdentity()
  {
    data.setIdentity();
  }

  inline void setZero()
  {
    data.setZero();
  }

  inline U determinant() const
  {
    return data.determinant();
  }

  Matrix3fX<T>& transpose() 
  {
    data.transpose();
    return *this;
  }

  Matrix3fX<T>& inverse()
  {
    data.inverse();
    return *this;
  }

  Matrix3fX<T>& abs()
  {
    data.abs();
    return *this;
  }

  static const Matrix3fX<T>& getIdentity()
  {
    static const Matrix3fX<T> I((U)1, (U)0, (U)0,
                                (U)0, (U)1, (U)0,
                                (U)0, (U)0, (U)1);
    return I;
  }

  Matrix3fX<T> transposeTimes(const Matrix3fX<T>& other) const
  {
    return Matrix3fX<T>(data.transposeTimes(other.data));
  }

  Matrix3fX<T> timesTranspose(const Matrix3fX<T>& other) const
  {
    return Matrix3fX<T>(data.timesTranspose(other.data));
  }

  Vec3fX<S> transposeTimes(const Vec3fX<S>& v) const
  {
    return Vec3fX<S>(data.transposeTimes(v.data));
  }

  Matrix3fX<T> tensorTransform(const Matrix3fX<T>& m) const
  {
    Matrix3fX<T> res(*this);
    res *= m;
    return res.timesTranspose(*this);
  }

  inline U transposeDotX(const Vec3fX<S>& v) const
  {
    return data.transposeDot(0, v.data);
  }

  inline U transposeDotY(const Vec3fX<S>& v) const
  {
    return data.transposeDot(1, v.data);
  }

  inline U transposeDotZ(const Vec3fX<S>& v) const
  {
    return data.transposeDot(2, v.data);
  }

  inline U transposeDot(size_t i, const Vec3fX<S>& v) const
  {
    return data.transposeDot(i, v.data);
  }

  inline U dotX(const Vec3fX<S>& v) const
  {
    return data.dot(0, v.data);
  }

  inline U dotY(const Vec3fX<S>& v) const
  {
    return data.dot(1, v.data);
  }

  inline U dotZ(const Vec3fX<S>& v) const
  {
    return data.dot(2, v.data);
  }

  inline U dot(size_t i, const Vec3fX<S>& v) const
  {
    return data.dot(i, v.data);
  }

  inline void setValue(U xx, U xy, U xz,
                       U yx, U yy, U yz,
                       U zx, U zy, U zz)
  {
    data.setValue(xx, xy, xz, 
                  yx, yy, yz,
                  zx, zy, zz);
  }

  inline void setValue(U x)
  {
    data.setValue(x);
  }
};

template<typename T>
void relativeTransform(const Matrix3fX<T>& R1, const Vec3fX<typename T::vector_type>& t1,
                       const Matrix3fX<T>& R2, const Vec3fX<typename T::vector_type>& t2,
                       Matrix3fX<T>& R, Vec3fX<typename T::vector_type>& t)
{
  R = R1.transposeTimes(R2);
  t = R1.transposeTimes(t2 - t1);
}

template<typename T>
void eigen(const Matrix3fX<T>& m, typename T::meta_type dout[3], Vec3fX<typename T::vector_type> vout[3])
{
  Matrix3fX<T> R(m);
  int n = 3;
  int j, iq, ip, i;
  typename T::meta_type tresh, theta, tau, t, sm, s, h, g, c;
  int nrot;
  typename T::meta_type b[3];
  typename T::meta_type z[3];
  typename T::meta_type v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  typename T::meta_type d[3];

  for(ip = 0; ip < n; ++ip)
  {
    b[ip] = d[ip] = R(ip, ip);
    z[ip] = 0;
  }

  nrot = 0;

  for(i = 0; i < 50; ++i)
  {
    sm = 0;
    for(ip = 0; ip < n; ++ip)
      for(iq = ip + 1; iq < n; ++iq)
        sm += std::abs(R(ip, iq));
    if(sm == 0.0)
    {
      vout[0].setValue(v[0][0], v[0][1], v[0][2]);
      vout[1].setValue(v[1][0], v[1][1], v[1][2]);
      vout[2].setValue(v[2][0], v[2][1], v[2][2]);
      dout[0] = d[0]; dout[1] = d[1]; dout[2] = d[2];
      return;
    }

    if(i < 3) tresh = 0.2 * sm / (n * n);
    else tresh = 0.0;

    for(ip = 0; ip < n; ++ip)
    {
      for(iq= ip + 1; iq < n; ++iq)
      {
        g = 100.0 * std::abs(R(ip, iq));
        if(i > 3 &&
           std::abs(d[ip]) + g == std::abs(d[ip]) &&
           std::abs(d[iq]) + g == std::abs(d[iq]))
          R(ip, iq) = 0.0;
        else if(std::abs(R(ip, iq)) > tresh)
        {
          h = d[iq] - d[ip];
          if(std::abs(h) + g == std::abs(h)) t = (R(ip, iq)) / h;
          else
          {
            theta = 0.5 * h / (R(ip, iq));
            t = 1.0 /(std::abs(theta) + std::sqrt(1.0 + theta * theta));
            if(theta < 0.0) t = -t;
          }
          c = 1.0 / std::sqrt(1 + t * t);
          s = t * c;
          tau = s / (1.0 + c);
          h = t * R(ip, iq);
          z[ip] -= h;
          z[iq] += h;
          d[ip] -= h;
          d[iq] += h;
          R(ip, iq) = 0.0;
          for(j = 0; j < ip; ++j) { g = R(j, ip); h = R(j, iq); R(j, ip) = g - s * (h + g * tau); R(j, iq) = h + s * (g - h * tau); }
          for(j = ip + 1; j < iq; ++j) { g = R(ip, j); h = R(j, iq); R(ip, j) = g - s * (h + g * tau); R(j, iq) = h + s * (g - h * tau); }
          for(j = iq + 1; j < n; ++j) { g = R(ip, j); h = R(iq, j); R(ip, j) = g - s * (h + g * tau); R(iq, j) = h + s * (g - h * tau); }
          for(j = 0; j < n; ++j) { g = v[j][ip]; h = v[j][iq]; v[j][ip] = g - s * (h + g * tau); v[j][iq] = h + s * (g - h * tau); }
          nrot++;
        }
      }
    }
    for(ip = 0; ip < n; ++ip)
    {
      b[ip] += z[ip];
      d[ip] = b[ip];
      z[ip] = 0.0;
    }
  }

  std::cerr << "eigen: too many iterations in Jacobi transform." << std::endl;

  return;
}

template<typename T>
Matrix3fX<T> abs(const Matrix3fX<T>& R) 
{
  return Matrix3fX<T>(abs(R.data));
}

template<typename T>
Matrix3fX<T> transpose(const Matrix3fX<T>& R)
{
  return Matrix3fX<T>(transpose(R.data));
}

template<typename T>
Matrix3fX<T> inverse(const Matrix3fX<T>& R)
{
  return Matrix3fX<T>(inverse(R.data));
}

template<typename T>
typename T::meta_type quadraticForm(const Matrix3fX<T>& R, const Vec3fX<typename T::vector_type>& v)
{
  return v.dot(R * v);
}


typedef Matrix3fX<details::Matrix3Data<FCL_REAL> > Matrix3f;
//typedef Matrix3fX<details::sse_meta_f12> Matrix3f;

}



#endif

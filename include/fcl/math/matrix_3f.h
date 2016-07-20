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

#ifndef FCL_MATRIX_3F_H
#define FCL_MATRIX_3F_H

#include "fcl/math/vec_3f.h"

namespace fcl
{

/// @brief Matrix2 class wrapper. the core data is in the template parameter class
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

  inline bool isIdentity () const
  {
    return
      data (0,0) == 1 && data (0,1) == 0 && data (0,2) == 0 &&
      data (1,0) == 0 && data (1,1) == 1 && data (1,2) == 0 &&
      data (2,0) == 0 && data (2,1) == 0 && data (2,2) == 1;
  }

  inline void setZero()
  {
    data.setZero();
  }

  /// @brief Set the matrix from euler angles YPR around ZYX axes
  /// @param eulerX Roll about X axis
  /// @param eulerY Pitch around Y axis
  /// @param eulerZ Yaw aboud Z axis
  ///  
  /// These angles are used to produce a rotation matrix. The euler
  /// angles are applied in ZYX order. I.e a vector is first rotated 
  /// about X then Y and then Z
  inline void setEulerZYX(FCL_REAL eulerX, FCL_REAL eulerY, FCL_REAL eulerZ)
  {
    FCL_REAL ci(cos(eulerX));
    FCL_REAL cj(cos(eulerY));
    FCL_REAL ch(cos(eulerZ));
    FCL_REAL si(sin(eulerX));
    FCL_REAL sj(sin(eulerY));
    FCL_REAL sh(sin(eulerZ));
    FCL_REAL cc = ci * ch;
    FCL_REAL cs = ci * sh;
    FCL_REAL sc = si * ch;
    FCL_REAL ss = si * sh;

    setValue(cj * ch, sj * sc - cs, sj * cc + ss,
             cj * sh, sj * ss + cc, sj * cs - sc, 
             -sj,     cj * si,      cj * ci);

  }

  /// @brief Set the matrix from euler angles using YPR around YXZ respectively
  /// @param yaw Yaw about Y axis
  /// @param pitch Pitch about X axis
  /// @param roll Roll about Z axis 
  void setEulerYPR(FCL_REAL yaw, FCL_REAL pitch, FCL_REAL roll)
  {
    setEulerZYX(roll, pitch, yaw);
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
void hat(Matrix3fX<T>& mat, const Vec3fX<typename T::vector_type>& vec)
{
  mat.setValue(0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0);
}

template<typename T>
void relativeTransform(const Matrix3fX<T>& R1, const Vec3fX<typename T::vector_type>& t1,
                       const Matrix3fX<T>& R2, const Vec3fX<typename T::vector_type>& t2,
                       Matrix3fX<T>& R, Vec3fX<typename T::vector_type>& t)
{
  R = R1.transposeTimes(R2);
  t = R1.transposeTimes(t2 - t1);
}

/// @brief compute the eigen vector and eigen vector of a matrix. dout is the eigen values, vout is the eigen vectors
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


#if FCL_HAVE_SSE
typedef Matrix3fX<details::sse_meta_f12> Matrix3f;
#else
typedef Matrix3fX<details::Matrix3Data<FCL_REAL> > Matrix3f;
#endif

static inline std::ostream& operator << (std::ostream& o, const Matrix3f& m)
{
  o << "[(" << m(0, 0) << " " << m(0, 1) << " " << m(0, 2) << ")("
    << m(1, 0) << " " << m(1, 1) << " " << m(1, 2) << ")(" 
    << m(2, 0) << " " << m(2, 1) << " " << m(2, 2) << ")]";
  return o;
}



/// @brief Class for variance matrix in 3d
class Variance3f
{
public:
  /// @brief Variation matrix
  Matrix3f Sigma;

  /// @brief Variations along the eign axes
  Matrix3f::U sigma[3];

  /// @brief Eigen axes of the variation matrix
  Vec3f axis[3];

  Variance3f() {}

  Variance3f(const Matrix3f& S) : Sigma(S)
  {
    init();
  }

  /// @brief init the Variance
  void init() 
  {
    eigen(Sigma, sigma, axis);
  }

  /// @brief Compute the sqrt of Sigma matrix based on the eigen decomposition result, this is useful when the uncertainty matrix is initialized as a square variation matrix
  Variance3f& sqrt()
  {
    for(std::size_t i = 0; i < 3; ++i)
    {
      if(sigma[i] < 0) sigma[i] = 0;
      sigma[i] = std::sqrt(sigma[i]);
    }


    Sigma.setZero();
    for(std::size_t i = 0; i < 3; ++i)
    {
      for(std::size_t j = 0; j < 3; ++j)
      {
        Sigma(i, j) += sigma[0] * axis[0][i] * axis[0][j];
        Sigma(i, j) += sigma[1] * axis[1][i] * axis[1][j];
        Sigma(i, j) += sigma[2] * axis[2][i] * axis[2][j];
      }
    }

    return *this;
  }
};

}



#endif

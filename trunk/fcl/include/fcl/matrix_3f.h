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
  class Matrix3f
  {
  public:
    Vec3f v_[3];

    /** \brief All zero matrix */
    Matrix3f() {}

    Matrix3f(BVH_REAL xx, BVH_REAL xy, BVH_REAL xz,
             BVH_REAL yx, BVH_REAL yy, BVH_REAL yz,
             BVH_REAL zx, BVH_REAL zy, BVH_REAL zz)
    {
      setValue(xx, xy, xz,
               yx, yy, yz,
               zx, zy, zz);
    }

    Matrix3f(const Matrix3f& other)
    {
      v_[0] = other.v_[0];
      v_[1] = other.v_[1];
      v_[2] = other.v_[2];
    }

    Matrix3f& operator = (const Matrix3f& other)
    {
      v_[0] = other.v_[0];
      v_[1] = other.v_[1];
      v_[2] = other.v_[2];
      return *this;
    }

    inline Vec3f getColumn(size_t i) const
    {
      return Vec3f(v_[0][i], v_[1][i], v_[2][i]);
    }

    inline const Vec3f& getRow(size_t i) const
    {
      return v_[i];
    }

    inline Vec3f& operator [](size_t i)
    {
      return v_[i];
    }

    inline const Vec3f& operator [](size_t i) const
    {
      return v_[i];
    }

    Matrix3f& operator *= (const Matrix3f& other);

    Matrix3f& operator += (BVH_REAL c);

    void setIdentity()
    {
      setValue((BVH_REAL)1.0, (BVH_REAL)0.0, (BVH_REAL)0.0,
               (BVH_REAL)0.0, (BVH_REAL)1.0, (BVH_REAL)0.0,
               (BVH_REAL)0.0, (BVH_REAL)0.0, (BVH_REAL)1.0);
    }

    void setZero()
    {
      setValue((BVH_REAL)0.0);
    }

    static const Matrix3f& getIdentity()
    {
      static const Matrix3f I((BVH_REAL)1.0, (BVH_REAL)0.0, (BVH_REAL)0.0,
                              (BVH_REAL)0.0, (BVH_REAL)1.0, (BVH_REAL)0.0,
                              (BVH_REAL)0.0, (BVH_REAL)0.0, (BVH_REAL)1.0);
      return I;
    }

    BVH_REAL determinant() const;
    Matrix3f transpose() const;
    Matrix3f inverse() const;
    Matrix3f abs() const;

    Matrix3f transposeTimes(const Matrix3f& m) const;
    Matrix3f timesTranspose(const Matrix3f& m) const;
    Matrix3f operator * (const Matrix3f& m) const;

    Vec3f operator * (const Vec3f& v) const;
    Vec3f transposeTimes(const Vec3f& v) const;

    inline BVH_REAL quadraticForm(const Vec3f& v) const
    {
      return v[0] * v[0] * v_[0][0] + v[0] * v[1] * v_[0][1] + v[0] * v[2] * v_[0][2] +
          v[1] * v[0] * v_[1][0] + v[1] * v[1] * v_[1][1] + v[1] * v[2] * v_[1][2] +
          v[2] * v[0] * v_[2][0] + v[2] * v[1] * v_[2][1] + v[2] * v[2] * v_[2][2];
    }

    /** S * M * S' */
    Matrix3f tensorTransform(const Matrix3f& m) const;

    inline BVH_REAL transposeDotX(const Vec3f& v) const
    {
      return v_[0][0] * v[0] + v_[1][0] * v[1] + v_[2][0] * v[2];
    }

    inline BVH_REAL transposeDotY(const Vec3f& v) const
    {
      return v_[0][1] * v[0] + v_[1][1] * v[1] + v_[2][1] * v[2];
    }

    inline BVH_REAL transposeDotZ(const Vec3f& v) const
    {
      return v_[0][2] * v[0] + v_[1][2] * v[1] + v_[2][2] * v[2];
    }

    inline void setValue(BVH_REAL xx, BVH_REAL xy, BVH_REAL xz,
                         BVH_REAL yx, BVH_REAL yy, BVH_REAL yz,
                         BVH_REAL zx, BVH_REAL zy, BVH_REAL zz)
    {
      v_[0].setValue(xx, xy, xz);
      v_[1].setValue(yx, yy, yz);
      v_[2].setValue(zx, zy, zz);
    }

    inline void setValue(BVH_REAL x)
    {
      v_[0].setValue(x);
      v_[1].setValue(x);
      v_[2].setValue(x);
    }
  };

  void relativeTransform(const Matrix3f& R1, const Vec3f& T1, const Matrix3f& R2, const Vec3f& T2, Matrix3f& R, Vec3f& T);

  void matEigen(const Matrix3f& R, BVH_REAL dout[3], Vec3f vout[3]);
}

#endif

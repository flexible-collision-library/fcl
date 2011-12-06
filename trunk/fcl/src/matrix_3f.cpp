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


#include "fcl/matrix_3f.h"
#include <iostream>

namespace fcl
{

Matrix3f& Matrix3f::operator *= (const Matrix3f& other)
{
  setValue(other.transposeDotX(v_[0]), other.transposeDotY(v_[0]), other.transposeDotZ(v_[0]),
           other.transposeDotX(v_[1]), other.transposeDotY(v_[1]), other.transposeDotZ(v_[1]),
           other.transposeDotX(v_[2]), other.transposeDotY(v_[2]), other.transposeDotZ(v_[2]));

  return *this;
}

Matrix3f& Matrix3f::operator += (BVH_REAL c)
{
  setValue(v_[0][0] + c, v_[0][1] + c, v_[0][2] + c,
           v_[1][0] + c, v_[1][1] + c, v_[1][2] + c,
           v_[2][0] + c, v_[2][1] + c, v_[2][2] + c);
  return *this;
}


BVH_REAL Matrix3f::determinant() const
{
  return triple(v_[0], v_[1], v_[2]);
}

Matrix3f Matrix3f::transpose() const
{
  return Matrix3f(v_[0][0], v_[1][0], v_[2][0],
                  v_[0][1], v_[1][1], v_[2][1],
                  v_[0][2], v_[1][2], v_[2][2]);
}

Matrix3f Matrix3f::abs() const
{
  return Matrix3f(fabs(v_[0][0]), fabs(v_[0][1]), fabs(v_[0][2]),
                  fabs(v_[1][0]), fabs(v_[1][1]), fabs(v_[1][2]),
                  fabs(v_[2][0]), fabs(v_[2][1]), fabs(v_[2][2]));
}

Matrix3f Matrix3f::inverse() const
{
  BVH_REAL det = determinant();
  BVH_REAL inv_det = 1.0 / det;

  return Matrix3f((v_[1][1] * v_[2][2] - v_[1][2] * v_[2][1]) * inv_det,
                  (v_[0][2] * v_[2][1] - v_[0][1] * v_[2][2]) * inv_det,
                  (v_[0][1] * v_[1][2] - v_[0][2] * v_[1][1]) * inv_det,
                  (v_[1][2] * v_[2][0] - v_[1][0] * v_[2][2]) * inv_det,
                  (v_[0][0] * v_[2][2] - v_[0][2] * v_[2][0]) * inv_det,
                  (v_[0][2] * v_[1][0] - v_[0][0] * v_[1][2]) * inv_det,
                  (v_[1][0] * v_[2][1] - v_[1][1] * v_[2][0]) * inv_det,
                  (v_[0][1] * v_[2][0] - v_[0][0] * v_[2][1]) * inv_det,
                  (v_[0][0] * v_[1][1] - v_[0][1] * v_[1][0]) * inv_det);
}


Matrix3f Matrix3f::transposeTimes(const Matrix3f& m) const
{
  return Matrix3f(
      v_[0][0] * m[0][0] + v_[1][0] * m[1][0] + v_[2][0] * m[2][0],
      v_[0][0] * m[0][1] + v_[1][0] * m[1][1] + v_[2][0] * m[2][1],
      v_[0][0] * m[0][2] + v_[1][0] * m[1][2] + v_[2][0] * m[2][2],
      v_[0][1] * m[0][0] + v_[1][1] * m[1][0] + v_[2][1] * m[2][0],
      v_[0][1] * m[0][1] + v_[1][1] * m[1][1] + v_[2][1] * m[2][1],
      v_[0][1] * m[0][2] + v_[1][1] * m[1][2] + v_[2][1] * m[2][2],
      v_[0][2] * m[0][0] + v_[1][2] * m[1][0] + v_[2][2] * m[2][0],
      v_[0][2] * m[0][1] + v_[1][2] * m[1][1] + v_[2][2] * m[2][1],
      v_[0][2] * m[0][2] + v_[1][2] * m[1][2] + v_[2][2] * m[2][2]);
}

Matrix3f Matrix3f::timesTranspose(const Matrix3f& m) const
{
  return Matrix3f(v_[0].dot(m[0]), v_[0].dot(m[1]), v_[0].dot(m[2]),
                  v_[1].dot(m[0]), v_[1].dot(m[1]), v_[1].dot(m[2]),
                  v_[2].dot(m[0]), v_[2].dot(m[1]), v_[2].dot(m[2]));
}

Vec3f Matrix3f::operator * (const Vec3f& v) const
{
  return Vec3f(v_[0].dot(v), v_[1].dot(v), v_[2].dot(v));
}

Vec3f Matrix3f::transposeTimes(const Vec3f& v) const
{
  return Vec3f(transposeDotX(v), transposeDotY(v), transposeDotZ(v));
}

Matrix3f Matrix3f::tensorTransform(const Matrix3f& m) const
{
  Matrix3f res = *this;
  res *= m;
  return res.timesTranspose(*this);
}

Matrix3f Matrix3f::operator * (const Matrix3f& m) const
{
  Matrix3f res = *this;
  return res *= m;
}

void relativeTransform(const Matrix3f& R1, const Vec3f& T1, const Matrix3f& R2, const Vec3f& T2, Matrix3f& R, Vec3f& T)
{
  R = R1.transposeTimes(R2);
  Vec3f temp = T2 - T1;
  T = R1.transposeTimes(temp);
}

void matEigen(const Matrix3f& m, BVH_REAL dout[3], Vec3f vout[3])
{
  Matrix3f R(m);
  int n = 3;
  int j, iq, ip, i;
  BVH_REAL tresh, theta, tau, t, sm, s, h, g, c;
  int nrot;
  BVH_REAL b[3];
  BVH_REAL z[3];
  BVH_REAL v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  BVH_REAL d[3];

  for(ip = 0; ip < n; ++ip)
  {
    b[ip] = R[ip][ip];
    d[ip] = R[ip][ip];
    z[ip] = 0.0;
  }

  nrot = 0;

  for(i = 0; i < 50; ++i)
  {
    sm = 0.0;
    for(ip = 0; ip < n; ++ip)
      for(iq = ip + 1; iq < n; ++iq)
        sm += fabs(R[ip][iq]);
    if(sm == 0.0)
    {
      vout[0] = Vec3f(v[0][0], v[0][1], v[0][2]);
      vout[1] = Vec3f(v[1][0], v[1][1], v[1][2]);
      vout[2] = Vec3f(v[2][0], v[2][1], v[2][2]);
      dout[0] = d[0]; dout[1] = d[1]; dout[2] = d[2];
      return;
    }

    if(i < 3) tresh = 0.2 * sm / (n * n);
    else tresh = 0.0;

    for(ip = 0; ip < n; ++ip)
    {
      for(iq= ip + 1; iq < n; ++iq)
      {
        g = 100.0 * fabs(R[ip][iq]);
        if(i > 3 &&
            fabs(d[ip]) + g == fabs(d[ip]) &&
            fabs(d[iq]) + g == fabs(d[iq]))
          R[ip][iq] = 0.0;
        else if(fabs(R[ip][iq]) > tresh)
        {
          h = d[iq] - d[ip];
          if(fabs(h) + g == fabs(h)) t = (R[ip][iq]) / h;
          else
          {
            theta = 0.5 * h / (R[ip][iq]);
            t = 1.0 /(fabs(theta) + sqrt(1.0 + theta * theta));
            if(theta < 0.0) t = -t;
          }
          c = 1.0 / sqrt(1 + t * t);
          s = t * c;
          tau = s / (1.0 + c);
          h = t * R[ip][iq];
          z[ip] -= h;
          z[iq] += h;
          d[ip] -= h;
          d[iq] += h;
          R[ip][iq] = 0.0;
          for(j = 0; j < ip; ++j) { g = R[j][ip]; h = R[j][iq]; R[j][ip] = g - s * (h + g * tau); R[j][iq] = h + s * (g - h * tau); }
          for(j = ip + 1; j < iq; ++j) { g = R[ip][j]; h = R[j][iq]; R[ip][j] = g - s * (h + g * tau); R[j][iq] = h + s * (g - h * tau); }
          for(j = iq + 1; j < n; ++j) { g = R[ip][j]; h = R[iq][j]; R[ip][j] = g - s * (h + g * tau); R[iq][j] = h + s * (g - h * tau); }
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


}

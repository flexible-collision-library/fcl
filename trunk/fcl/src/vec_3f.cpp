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

#include "fcl/vec_3f.h"
#include <iostream>

namespace fcl
{
#if COLLISION_USE_SSE
const float Vec3f::EPSILON = 1e-11;
#else
const BVH_REAL Vec3f::EPSILON = 1e-11;
#endif

Vec3f matMulVec(const Vec3f M[3], const Vec3f& v)
{
  return Vec3f(M[0].dot(v), M[1].dot(v), M[2].dot(v));
}

Vec3f matTransMulVec(const Vec3f M[3], const Vec3f& v)
{
  return M[0] * v[0] + M[1] * v[1] + M[2] * v[2];
}

BVH_REAL quadraticForm(const Vec3f M[3], const Vec3f& v)
{
  return v.dot(Vec3f(M[0].dot(v), M[1].dot(v), M[2].dot(v)));
}


void tensorTransform(const Vec3f M[3], const Vec3f S[3], Vec3f newM[3])
{
  Vec3f SMT_col[3] = {Vec3f(M[0].dot(S[0]), M[1].dot(S[0]), M[2].dot(S[0])),
                      Vec3f(M[0].dot(S[1]), M[1].dot(S[1]), M[2].dot(S[1])),
                      Vec3f(M[0].dot(S[2]), M[1].dot(S[2]), M[2].dot(S[2]))
  };

  newM[0] = Vec3f(S[0].dot(SMT_col[0]), S[1].dot(SMT_col[0]), S[2].dot(SMT_col[0]));
  newM[1] = Vec3f(S[0].dot(SMT_col[1]), S[1].dot(SMT_col[1]), S[2].dot(SMT_col[1]));
  newM[2] = Vec3f(S[0].dot(SMT_col[2]), S[1].dot(SMT_col[2]), S[2].dot(SMT_col[2]));
}

void relativeTransform(const Vec3f R1[3], const Vec3f& T1, const Vec3f R2[3], const Vec3f& T2, Vec3f R[3], Vec3f& T)
{
  R[0] = Vec3f(R1[0][0] * R2[0][0] + R1[1][0] * R2[1][0] + R1[2][0] * R2[2][0],
               R1[0][0] * R2[0][1] + R1[1][0] * R2[1][1] + R1[2][0] * R2[2][1],
               R1[0][0] * R2[0][2] + R1[1][0] * R2[1][2] + R1[2][0] * R2[2][2]);
  R[1] = Vec3f(R1[0][1] * R2[0][0] + R1[1][1] * R2[1][0] + R1[2][1] * R2[2][0],
               R1[0][1] * R2[0][1] + R1[1][1] * R2[1][1] + R1[2][1] * R2[2][1],
               R1[0][1] * R2[0][2] + R1[1][1] * R2[1][2] + R1[2][1] * R2[2][2]);
  R[2] = Vec3f(R1[0][2] * R2[0][0] + R1[1][2] * R2[1][0] + R1[2][2] * R2[2][0],
               R1[0][2] * R2[0][1] + R1[1][2] * R2[1][1] + R1[2][2] * R2[2][1],
               R1[0][2] * R2[0][2] + R1[1][2] * R2[1][2] + R1[2][2] * R2[2][2]);

  Vec3f temp = T2 - T1;
  T = Vec3f(R1[0][0] * temp[0] + R1[1][0] * temp[1] + R1[2][0] * temp[2],
            R1[0][1] * temp[0] + R1[1][1] * temp[1] + R1[2][1] * temp[2],
            R1[0][2] * temp[0] + R1[1][2] * temp[1] + R1[2][2] * temp[2]);
}

void matEigen(Vec3f a[3], BVH_REAL dout[3], Vec3f vout[3])
{
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
    b[ip] = a[ip][ip];
    d[ip] = a[ip][ip];
    z[ip] = 0.0;
  }

  nrot = 0;

  for(i = 0; i < 50; ++i)
  {
    sm = 0.0;
    for(ip = 0; ip < n; ++ip)
      for(iq = ip + 1; iq < n; ++iq)
        sm += fabs(a[ip][iq]);
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
        g = 100.0 * fabs(a[ip][iq]);
        if(i > 3 &&
            fabs(d[ip]) + g == fabs(d[ip]) &&
            fabs(d[iq]) + g == fabs(d[iq]))
          a[ip][iq] = 0.0;
        else if(fabs(a[ip][iq]) > tresh)
        {
          h = d[iq] - d[ip];
          if(fabs(h) + g == fabs(h)) t = (a[ip][iq]) / h;
          else
          {
            theta = 0.5 * h / (a[ip][iq]);
            t = 1.0 /(fabs(theta) + sqrt(1.0 + theta * theta));
            if(theta < 0.0) t = -t;
          }
          c = 1.0 / sqrt(1 + t * t);
          s = t * c;
          tau = s / (1.0 + c);
          h = t * a[ip][iq];
          z[ip] -= h;
          z[iq] += h;
          d[ip] -= h;
          d[iq] += h;
          a[ip][iq] = 0.0;
          for(j = 0; j < ip; ++j) { g = a[j][ip]; h = a[j][iq]; a[j][ip] = g - s * (h + g * tau); a[j][iq] = h + s * (g - h * tau); }
          for(j = ip + 1; j < iq; ++j) { g = a[ip][j]; h = a[j][iq]; a[ip][j] = g - s * (h + g * tau); a[j][iq] = h + s * (g - h * tau); }
          for(j = iq + 1; j < n; ++j) { g = a[ip][j]; h = a[iq][j]; a[ip][j] = g - s * (h + g * tau); a[iq][j] = h + s * (g - h * tau); }
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


void matMulMat(const Vec3f M1[3], const Vec3f M2[3], Vec3f newM[3])
{
  for(int i = 0; i < 3; ++i)
  {
    for(int j = 0; j < 3; ++j)
    {
      newM[i][j] = M1[i][0] * M2[0][j] + M1[i][1] * M2[1][j] + M1[i][2] * M2[2][j];
    }
  }
}


}

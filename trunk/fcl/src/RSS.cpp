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

#include "fcl/RSS.h"
#include "fcl/BVH_utility.h"
#include <iostream>
namespace fcl
{

bool RSS::overlap(const RSS& other) const
{
  // compute what transform [R,T] that takes us from cs1 to cs2.
  // [R,T] = [R1,T1]'[R2,T2] = [R1',-R1'T][R2,T2] = [R1'R2, R1'(T2-T1)]
  // First compute the rotation part, then translation part
  Vec3f t = other.Tr - Tr; // T2 - T1
  Vec3f T(t.dot(axis[0]), t.dot(axis[1]), t.dot(axis[2])); // R1'(T2-T1)
  Vec3f R[3];
  R[0] = Vec3f(axis[0].dot(other.axis[0]), axis[0].dot(other.axis[1]), axis[0].dot(other.axis[2]));
  R[1] = Vec3f(axis[1].dot(other.axis[0]), axis[1].dot(other.axis[1]), axis[1].dot(other.axis[2]));
  R[2] = Vec3f(axis[2].dot(other.axis[0]), axis[2].dot(other.axis[1]), axis[2].dot(other.axis[2]));

  BVH_REAL dist = rectDistance(R, T, l, other.l);
  if(dist <= (r + other.r)) return true;
  return false;
}

bool overlap(const Vec3f R0[3], const Vec3f& T0, const RSS& b1, const RSS& b2)
{
  // R0 R2
  Vec3f Rtemp_col[3];
  Rtemp_col[0] = Vec3f(R0[0].dot(b2.axis[0]), R0[1].dot(b2.axis[0]), R0[2].dot(b2.axis[0]));
  Rtemp_col[1] = Vec3f(R0[0].dot(b2.axis[1]), R0[1].dot(b2.axis[1]), R0[2].dot(b2.axis[1]));
  Rtemp_col[2] = Vec3f(R0[0].dot(b2.axis[2]), R0[1].dot(b2.axis[2]), R0[2].dot(b2.axis[2]));

  // R1'Rtemp
  Vec3f R[3];
  R[0] = Vec3f(b1.axis[0].dot(Rtemp_col[0]), b1.axis[0].dot(Rtemp_col[1]), b1.axis[0].dot(Rtemp_col[2]));
  R[1] = Vec3f(b1.axis[1].dot(Rtemp_col[0]), b1.axis[1].dot(Rtemp_col[1]), b1.axis[1].dot(Rtemp_col[2]));
  R[2] = Vec3f(b1.axis[2].dot(Rtemp_col[0]), b1.axis[2].dot(Rtemp_col[1]), b1.axis[2].dot(Rtemp_col[2]));

  Vec3f Ttemp = Vec3f(R0[0].dot(b2.Tr), R0[1].dot(b2.Tr), R0[2].dot(b2.Tr)) + T0 - b1.Tr;

  Vec3f T = Vec3f(Ttemp.dot(b1.axis[0]), Ttemp.dot(b1.axis[1]), Ttemp.dot(b1.axis[2]));

  BVH_REAL dist = RSS::rectDistance(R, T, b1.l, b2.l);
  if(dist <= (b1.r + b2.r)) return true;
  return false;
}

bool RSS::contain(const Vec3f& p) const
{
  Vec3f local_p = p - Tr;
  BVH_REAL proj0 = local_p.dot(axis[0]);
  BVH_REAL proj1 = local_p.dot(axis[1]);
  BVH_REAL proj2 = local_p.dot(axis[2]);
  BVH_REAL abs_proj2 = fabs(proj2);
  Vec3f proj(proj0, proj1, proj2);

  // projection is within the rectangle
  if((proj0 < l[0]) && (proj0 > 0) && (proj1 < l[1]) && (proj1 > 0))
  {
    if(abs_proj2 < r)
      return true;
    else
      return false;
  }
  else if((proj0 < l[0]) && (proj0 > 0) && ((proj1 < 0) || (proj1 > l[1])))
  {
    BVH_REAL y = (proj1 > 0) ? l[1] : 0;
    Vec3f v(proj0, y, 0);
    if((proj - v).sqrLength() < r * r)
      return true;
    else
      return false;
  }
  else if((proj1 < l[1]) && (proj1 > 0) && ((proj0 < 0) || (proj0 > l[0])))
  {
    BVH_REAL x = (proj0 > 0) ? l[0] : 0;
    Vec3f v(x, proj1, 0);
    if((proj - v).sqrLength() < r * r)
      return true;
    else
      return false;
  }
  else
  {
    BVH_REAL x = (proj0 > 0) ? l[0] : 0;
    BVH_REAL y = (proj1 > 0) ? l[1] : 0;
    Vec3f v(x, y, 0);
    if((proj - v).sqrLength() < r * r)
      return true;
    else
      return false;
  }
}

RSS& RSS::operator += (const Vec3f& p)
{
  Vec3f local_p = p - Tr;
  BVH_REAL proj0 = local_p.dot(axis[0]);
  BVH_REAL proj1 = local_p.dot(axis[1]);
  BVH_REAL proj2 = local_p.dot(axis[2]);
  BVH_REAL abs_proj2 = fabs(proj2);
  Vec3f proj(proj0, proj1, proj2);

  // projection is within the rectangle
  if((proj0 < l[0]) && (proj0 > 0) && (proj1 < l[1]) && (proj1 > 0))
  {
    if(abs_proj2 < r)
      ; // do nothing
    else
    {
      r = 0.5 * (r + abs_proj2); // enlarge the r
      // change RSS origin position
      if(proj2 > 0)
        Tr[2] += 0.5 * (abs_proj2 - r);
      else
        Tr[2] -= 0.5 * (abs_proj2 - r);
    }
  }
  else if((proj0 < l[0]) && (proj0 > 0) && ((proj1 < 0) || (proj1 > l[1])))
  {
    BVH_REAL y = (proj1 > 0) ? l[1] : 0;
    Vec3f v(proj0, y, 0);
    BVH_REAL new_r_sqr = (proj - v).sqrLength();
    if(new_r_sqr < r * r)
      ; // do nothing
    else
    {
      if(abs_proj2 < r)
      {
        BVH_REAL delta_y = - sqrt(r * r - proj2 * proj2) + fabs(proj1 - y);
        l[1] += delta_y;
        if(proj1 < 0)
          Tr[1] -= delta_y;
      }
      else
      {
        BVH_REAL delta_y = fabs(proj1 - y);
        l[1] += delta_y;
        if(proj1 < 0)
          Tr[1] -= delta_y;

        if(proj2 > 0)
          Tr[2] += 0.5 * (abs_proj2 - r);
        else
          Tr[2] -= 0.5 * (abs_proj2 - r);
      }
    }
  }
  else if((proj1 < l[1]) && (proj1 > 0) && ((proj0 < 0) || (proj0 > l[0])))
  {
    BVH_REAL x = (proj0 > 0) ? l[0] : 0;
    Vec3f v(x, proj1, 0);
    BVH_REAL new_r_sqr = (proj - v).sqrLength();
    if(new_r_sqr < r * r)
      ; // do nothing
    else
    {
      if(abs_proj2 < r)
      {
        BVH_REAL delta_x = - sqrt(r * r - proj2 * proj2) + fabs(proj0 - x);
        l[0] += delta_x;
        if(proj0 < 0)
          Tr[0] -= delta_x;
      }
      else
      {
        BVH_REAL delta_x = fabs(proj0 - x);
        l[0] += delta_x;
        if(proj0 < 0)
          Tr[0] -= delta_x;

        if(proj2 > 0)
          Tr[2] += 0.5 * (abs_proj2 - r);
        else
          Tr[2] -= 0.5 * (abs_proj2 - r);
      }
    }
  }
  else
  {
    BVH_REAL x = (proj0 > 0) ? l[0] : 0;
    BVH_REAL y = (proj1 > 0) ? l[1] : 0;
    Vec3f v(x, y, 0);
    BVH_REAL new_r_sqr = (proj - v).sqrLength();
    if(new_r_sqr < r * r)
      ; // do nothing
    else
    {
      if(abs_proj2 < r)
      {
        BVH_REAL diag = sqrt(new_r_sqr - proj2 * proj2);
        BVH_REAL delta_diag = - sqrt(r * r - proj2 * proj2) + diag;

        BVH_REAL delta_x = delta_diag / diag * fabs(proj0 - x);
        BVH_REAL delta_y = delta_diag / diag * fabs(proj1 - y);
        l[0] += delta_x;
        l[1] += delta_y;

        if(proj0 < 0 && proj1 < 0)
        {
          Tr[0] -= delta_x;
          Tr[1] -= delta_y;
        }
      }
      else
      {
        BVH_REAL delta_x = fabs(proj0 - x);
        BVH_REAL delta_y = fabs(proj1 - y);

        l[0] += delta_x;
        l[1] += delta_y;

        if(proj0 < 0 && proj1 < 0)
        {
          Tr[0] -= delta_x;
          Tr[1] -= delta_y;
        }

        if(proj2 > 0)
          Tr[2] += 0.5 * (abs_proj2 - r);
        else
          Tr[2] -= 0.5 * (abs_proj2 - r);
      }
    }
  }

  return *this;
}
/*
RSS RSS::operator + (const RSS& other) const
{
  RSS res = *this;

  Vec3f d0_pos = other.axis[0] * (other.l[0] + other.r);
  Vec3f d1_pos = other.axis[1] * (other.l[1] + other.r);
  Vec3f d0_neg = other.axis[0] * (-other.r);
  Vec3f d1_neg = other.axis[1] * (-other.r);
  Vec3f d2_pos = other.axis[2] * other.r;
  Vec3f d2_neg = other.axis[2] * (-other.r);

  Vec3f v[8];
  v[0] = other.Tr + d0_pos + d1_pos + d2_pos;
  v[1] = other.Tr + d0_pos + d1_pos + d2_neg;
  v[2] = other.Tr + d0_pos + d1_neg + d2_pos;
  v[3] = other.Tr + d0_pos + d1_neg + d2_neg;
  v[4] = other.Tr + d0_neg + d1_pos + d2_pos;
  v[5] = other.Tr + d0_neg + d1_pos + d2_neg;
  v[6] = other.Tr + d0_neg + d1_neg + d2_pos;
  v[7] = other.Tr + d0_neg + d1_neg + d2_neg;

  for(int i = 0; i < 8; ++i)
  {
    res += v[i];
  }
  return res;
}
*/

RSS RSS::operator + (const RSS& other) const
{
  RSS bv;

  Vec3f v[16];
  Vec3f d0_pos = other.axis[0] * (other.l[0] + other.r);
  Vec3f d1_pos = other.axis[1] * (other.l[1] + other.r);
  Vec3f d0_neg = other.axis[0] * (-other.r);
  Vec3f d1_neg = other.axis[1] * (-other.r);
  Vec3f d2_pos = other.axis[2] * other.r;
  Vec3f d2_neg = other.axis[2] * (-other.r);

  v[0] = other.Tr + d0_pos + d1_pos + d2_pos;
  v[1] = other.Tr + d0_pos + d1_pos + d2_neg;
  v[2] = other.Tr + d0_pos + d1_neg + d2_pos;
  v[3] = other.Tr + d0_pos + d1_neg + d2_neg;
  v[4] = other.Tr + d0_neg + d1_pos + d2_pos;
  v[5] = other.Tr + d0_neg + d1_pos + d2_neg;
  v[6] = other.Tr + d0_neg + d1_neg + d2_pos;
  v[7] = other.Tr + d0_neg + d1_neg + d2_neg;

  d0_pos = axis[0] * (l[0] + r);
  d1_pos = axis[1] * (l[1] + r);
  d0_neg = axis[0] * (-r);
  d1_neg = axis[1] * (-r);
  d2_pos = axis[2] * r;
  d2_neg = axis[2] * (-r);

  v[8] = Tr + d0_pos + d1_pos + d2_pos;
  v[9] = Tr + d0_pos + d1_pos + d2_neg;
  v[10] = Tr + d0_pos + d1_neg + d2_pos;
  v[11] = Tr + d0_pos + d1_neg + d2_neg;
  v[12] = Tr + d0_neg + d1_pos + d2_pos;
  v[13] = Tr + d0_neg + d1_pos + d2_neg;
  v[14] = Tr + d0_neg + d1_neg + d2_pos;
  v[15] = Tr + d0_neg + d1_neg + d2_neg;


  Vec3f M[3]; // row first matrix
  Vec3f E[3]; // row first eigen-vectors
  BVH_REAL s[3] = {0, 0, 0};

  getCovariance(v, NULL, NULL, 16, M);
  matEigen(M, s, E);

  int min, mid, max;
  if(s[0] > s[1]) { max = 0; min = 1; }
  else { min = 0; max = 1; }
  if(s[2] < s[min]) { mid = min; min = 2; }
  else if(s[2] > s[max]) { mid = max; max = 2; }
  else { mid = 2; }

  Vec3f R[3]; // column first matrix, as the axis in RSS
  R[0] = Vec3f(E[0][max], E[1][max], E[2][max]);
  R[1] = Vec3f(E[0][mid], E[1][mid], E[2][mid]);
  R[2] = Vec3f(E[1][max]*E[2][mid] - E[1][mid]*E[2][max],
               E[0][mid]*E[2][max] - E[0][max]*E[2][mid],
               E[0][max]*E[1][mid] - E[0][mid]*E[1][max]);

  // set obb axes
  bv.axis[0] = R[0];
  bv.axis[1] = R[1];
  bv.axis[2] = R[2];

  // set rss origin, rectangle size and radius
  getRadiusAndOriginAndRectangleSize(v, NULL, NULL, 16, R, bv.Tr, bv.l, bv.r);

  return bv;
}

BVH_REAL RSS::distance(const RSS& other, Vec3f* P, Vec3f* Q) const
{
  // compute what transform [R,T] that takes us from cs1 to cs2.
  // [R,T] = [R1,T1]'[R2,T2] = [R1',-R1'T][R2,T2] = [R1'R2, R1'(T2-T1)]
  // First compute the rotation part, then translation part
  Vec3f t = other.Tr - Tr; // T2 - T1
  Vec3f T(t.dot(axis[0]), t.dot(axis[1]), t.dot(axis[2])); // R1'(T2-T1)
  Vec3f R[3];
  R[0] = Vec3f(axis[0].dot(other.axis[0]), axis[0].dot(other.axis[1]), axis[0].dot(other.axis[2]));
  R[1] = Vec3f(axis[1].dot(other.axis[0]), axis[1].dot(other.axis[1]), axis[1].dot(other.axis[2]));
  R[2] = Vec3f(axis[2].dot(other.axis[0]), axis[2].dot(other.axis[1]), axis[2].dot(other.axis[2]));

  BVH_REAL dist = rectDistance(R, T, l, other.l, P, Q);
  dist -= (r + other.r);
  return (dist < (BVH_REAL)0.0) ? (BVH_REAL)0.0 : dist;
}

BVH_REAL distance(const Vec3f R0[3], const Vec3f& T0, const RSS& b1, const RSS& b2, Vec3f* P, Vec3f* Q)
{
  // R0 R2
  Vec3f Rtemp_col[3];
  Rtemp_col[0] = Vec3f(R0[0].dot(b2.axis[0]), R0[1].dot(b2.axis[0]), R0[2].dot(b2.axis[0]));
  Rtemp_col[1] = Vec3f(R0[0].dot(b2.axis[1]), R0[1].dot(b2.axis[1]), R0[2].dot(b2.axis[1]));
  Rtemp_col[2] = Vec3f(R0[0].dot(b2.axis[2]), R0[1].dot(b2.axis[2]), R0[2].dot(b2.axis[2]));

  // R1'Rtemp
  Vec3f R[3];
  R[0] = Vec3f(b1.axis[0].dot(Rtemp_col[0]), b1.axis[0].dot(Rtemp_col[1]), b1.axis[0].dot(Rtemp_col[2]));
  R[1] = Vec3f(b1.axis[1].dot(Rtemp_col[0]), b1.axis[1].dot(Rtemp_col[1]), b1.axis[1].dot(Rtemp_col[2]));
  R[2] = Vec3f(b1.axis[2].dot(Rtemp_col[0]), b1.axis[2].dot(Rtemp_col[1]), b1.axis[2].dot(Rtemp_col[2]));

  Vec3f Ttemp = Vec3f(R0[0].dot(b2.Tr), R0[1].dot(b2.Tr), R0[2].dot(b2.Tr)) + T0 - b1.Tr;

  Vec3f T = Vec3f(Ttemp.dot(b1.axis[0]), Ttemp.dot(b1.axis[1]), Ttemp.dot(b1.axis[2]));

  BVH_REAL dist = RSS::rectDistance(R, T, b1.l, b2.l, P, Q);
  dist -= (b1.r + b2.r);
  return (dist < (BVH_REAL)0.0) ? (BVH_REAL)0.0 : dist;
}


BVH_REAL RSS::rectDistance(const Vec3f Rab[3], Vec3f const& Tab, const BVH_REAL a[2], const BVH_REAL b[2], Vec3f* P, Vec3f* Q)
{
  BVH_REAL A0_dot_B0, A0_dot_B1, A1_dot_B0, A1_dot_B1;

  A0_dot_B0 = Rab[0][0];
  A0_dot_B1 = Rab[0][1];
  A1_dot_B0 = Rab[1][0];
  A1_dot_B1 = Rab[1][1];

  BVH_REAL aA0_dot_B0, aA0_dot_B1, aA1_dot_B0, aA1_dot_B1;
  BVH_REAL bA0_dot_B0, bA0_dot_B1, bA1_dot_B0, bA1_dot_B1;

  aA0_dot_B0 = a[0] * A0_dot_B0;
  aA0_dot_B1 = a[0] * A0_dot_B1;
  aA1_dot_B0 = a[1] * A1_dot_B0;
  aA1_dot_B1 = a[1] * A1_dot_B1;
  bA0_dot_B0 = b[0] * A0_dot_B0;
  bA1_dot_B0 = b[0] * A1_dot_B0;
  bA0_dot_B1 = b[1] * A0_dot_B1;
  bA1_dot_B1 = b[1] * A1_dot_B1;

  Vec3f Tba = matTransMulVec(Rab, Tab);

  Vec3f S;
  BVH_REAL t, u;

  // determine if any edge pair contains the closest points

  BVH_REAL ALL_x, ALU_x, AUL_x, AUU_x;
  BVH_REAL BLL_x, BLU_x, BUL_x, BUU_x;
  BVH_REAL LA1_lx, LA1_ux, UA1_lx, UA1_ux, LB1_lx, LB1_ux, UB1_lx, UB1_ux;

  ALL_x = -Tba[0];
  ALU_x = ALL_x + aA1_dot_B0;
  AUL_x = ALL_x + aA0_dot_B0;
  AUU_x = ALU_x + aA0_dot_B0;

  if(ALL_x < ALU_x)
  {
    LA1_lx = ALL_x;
    LA1_ux = ALU_x;
    UA1_lx = AUL_x;
    UA1_ux = AUU_x;
  }
  else
  {
    LA1_lx = ALU_x;
    LA1_ux = ALL_x;
    UA1_lx = AUU_x;
    UA1_ux = AUL_x;
  }

  BLL_x = Tab[0];
  BLU_x = BLL_x + bA0_dot_B1;
  BUL_x = BLL_x + bA0_dot_B0;
  BUU_x = BLU_x + bA0_dot_B0;

  if(BLL_x < BLU_x)
  {
    LB1_lx = BLL_x;
    LB1_ux = BLU_x;
    UB1_lx = BUL_x;
    UB1_ux = BUU_x;
  }
  else
  {
    LB1_lx = BLU_x;
    LB1_ux = BLL_x;
    UB1_lx = BUU_x;
    UB1_ux = BUL_x;
  }

  // UA1, UB1

  if((UA1_ux > b[0]) && (UB1_ux > a[0]))
  {
    if(((UA1_lx > b[0]) ||
        inVoronoi(b[1], a[1], A1_dot_B0, aA0_dot_B0 - b[0] - Tba[0],
                  A1_dot_B1, aA0_dot_B1 - Tba[1],
                  -Tab[1] - bA1_dot_B0))
            &&
            ((UB1_lx > a[0]) ||
             inVoronoi(a[1], b[1], A0_dot_B1, Tab[0] + bA0_dot_B0 - a[0],
                       A1_dot_B1, Tab[1] + bA1_dot_B0, Tba[1] - aA0_dot_B1)))
    {
      segCoords(t, u, a[1], b[1], A1_dot_B1, Tab[1] + bA1_dot_B0,
                Tba[1] - aA0_dot_B1);

      S[0] = Tab[0] + Rab[0][0] * b[0] + Rab[0][1] * u - a[0] ;
      S[1] = Tab[1] + Rab[1][0] * b[0] + Rab[1][1] * u - t;
      S[2] = Tab[2] + Rab[2][0] * b[0] + Rab[2][1] * u;

      if(P && Q)
      {
        *P = Vec3f(a[0], t, 0);
        *Q = S + (*P);
      }

      return S.length();
    }
  }


  // UA1, LB1

  if((UA1_lx < 0) && (LB1_ux > a[0]))
  {
    if(((UA1_ux < 0) ||
        inVoronoi(b[1], a[1], -A1_dot_B0, Tba[0] - aA0_dot_B0,
                  A1_dot_B1, aA0_dot_B1 - Tba[1], -Tab[1]))
        &&
        ((LB1_lx > a[0]) ||
         inVoronoi(a[1], b[1], A0_dot_B1, Tab[0] - a[0],
                   A1_dot_B1, Tab[1], Tba[1] - aA0_dot_B1)))
    {
      segCoords(t, u, a[1], b[1], A1_dot_B1, Tab[1], Tba[1] - aA0_dot_B1);

      S[0] = Tab[0] + Rab[0][1] * u - a[0];
      S[1] = Tab[1] + Rab[1][1] * u - t;
      S[2] = Tab[2] + Rab[2][1] * u;

      if(P && Q)
      {
        *P = Vec3f(a[0], t, 0);
        *Q = S + (*P);
      }

      return S.length();
    }
  }

  // LA1, UB1

  if((LA1_ux > b[0]) && (UB1_lx < 0))
  {
    if(((LA1_lx > b[0]) ||
        inVoronoi(b[1], a[1], A1_dot_B0, -Tba[0] - b[0],
                  A1_dot_B1, -Tba[1], -Tab[1] - bA1_dot_B0))
        &&
        ((UB1_ux < 0) ||
         inVoronoi(a[1], b[1], -A0_dot_B1, -Tab[0] - bA0_dot_B0,
                   A1_dot_B1, Tab[1] + bA1_dot_B0, Tba[1])))
    {
      segCoords(t, u, a[1], b[1], A1_dot_B1, Tab[1] + bA1_dot_B0, Tba[1]);

      S[0] = Tab[0] + Rab[0][0] * b[0] + Rab[0][1] * u;
      S[1] = Tab[1] + Rab[1][0] * b[0] + Rab[1][1] * u - t;
      S[2] = Tab[2] + Rab[2][0] * b[0] + Rab[2][1] * u;

      if(P && Q)
      {
        *P = Vec3f(0, t, 0);
        *Q = S + (*P);
      }

      return S.length();
    }
  }

  // LA1, LB1

  if((LA1_lx < 0) && (LB1_lx < 0))
  {
    if (((LA1_ux < 0) ||
        inVoronoi(b[1], a[1], -A1_dot_B0, Tba[0], A1_dot_B1,
                  -Tba[1], -Tab[1]))
        &&
        ((LB1_ux < 0) ||
         inVoronoi(a[1], b[1], -A0_dot_B1, -Tab[0], A1_dot_B1,
                   Tab[1], Tba[1])))
    {
      segCoords(t, u, a[1], b[1], A1_dot_B1, Tab[1], Tba[1]);

      S[0] = Tab[0] + Rab[0][1] * u;
      S[1] = Tab[1] + Rab[1][1] * u - t;
      S[2] = Tab[2] + Rab[2][1] * u;

      if(P && Q)
      {
        *P = Vec3f(0, t, 0);
        *Q = S + (*P);
      }

      return S.length();
    }
  }

  BVH_REAL ALL_y, ALU_y, AUL_y, AUU_y;

  ALL_y = -Tba[1];
  ALU_y = ALL_y + aA1_dot_B1;
  AUL_y = ALL_y + aA0_dot_B1;
  AUU_y = ALU_y + aA0_dot_B1;

  BVH_REAL LA1_ly, LA1_uy, UA1_ly, UA1_uy, LB0_lx, LB0_ux, UB0_lx, UB0_ux;

  if(ALL_y < ALU_y)
  {
    LA1_ly = ALL_y;
    LA1_uy = ALU_y;
    UA1_ly = AUL_y;
    UA1_uy = AUU_y;
  }
  else
  {
    LA1_ly = ALU_y;
    LA1_uy = ALL_y;
    UA1_ly = AUU_y;
    UA1_uy = AUL_y;
  }

  if(BLL_x < BUL_x)
  {
    LB0_lx = BLL_x;
    LB0_ux = BUL_x;
    UB0_lx = BLU_x;
    UB0_ux = BUU_x;
  }
  else
  {
    LB0_lx = BUL_x;
    LB0_ux = BLL_x;
    UB0_lx = BUU_x;
    UB0_ux = BLU_x;
  }

  // UA1, UB0

  if((UA1_uy > b[1]) && (UB0_ux > a[0]))
  {
    if(((UA1_ly > b[1]) ||
        inVoronoi(b[0], a[1], A1_dot_B1, aA0_dot_B1 - Tba[1] - b[1],
                  A1_dot_B0, aA0_dot_B0 - Tba[0], -Tab[1] - bA1_dot_B1))
        &&
        ((UB0_lx > a[0]) ||
         inVoronoi(a[1], b[0], A0_dot_B0, Tab[0] - a[0] + bA0_dot_B1,
                   A1_dot_B0, Tab[1] + bA1_dot_B1, Tba[0] - aA0_dot_B0)))
    {
      segCoords(t, u, a[1], b[0], A1_dot_B0, Tab[1] + bA1_dot_B1,
                Tba[0] - aA0_dot_B0);

      S[0] = Tab[0] + Rab[0][1] * b[1] + Rab[0][0] * u - a[0] ;
      S[1] = Tab[1] + Rab[1][1] * b[1] + Rab[1][0] * u - t;
      S[2] = Tab[2] + Rab[2][1] * b[1] + Rab[2][0] * u;

      if(P && Q)
      {
        *P = Vec3f(a[0], t, 0);
        *Q = S + (*P);
      }

      return S.length();
    }
  }

  // UA1, LB0

  if((UA1_ly < 0) && (LB0_ux > a[0]))
  {
    if(((UA1_uy < 0) ||
        inVoronoi(b[0], a[1], -A1_dot_B1, Tba[1] - aA0_dot_B1, A1_dot_B0,
                  aA0_dot_B0 - Tba[0], -Tab[1]))
        &&
        ((LB0_lx > a[0]) ||
         inVoronoi(a[1], b[0], A0_dot_B0, Tab[0] - a[0],
                   A1_dot_B0, Tab[1], Tba[0] - aA0_dot_B0)))
    {
      segCoords(t, u, a[1], b[0], A1_dot_B0, Tab[1], Tba[0] - aA0_dot_B0);

      S[0] = Tab[0] + Rab[0][0] * u - a[0];
      S[1] = Tab[1] + Rab[1][0] * u - t;
      S[2] = Tab[2] + Rab[2][0] * u;

      if(P && Q)
      {
        *P = Vec3f(a[0], t, 0);
        *Q = S + (*P);
      }

      return S.length();
    }
  }

  // LA1, UB0

  if((LA1_uy > b[1]) && (UB0_lx < 0))
  {
    if(((LA1_ly > b[1]) ||
        inVoronoi(b[0], a[1], A1_dot_B1, -Tba[1] - b[1],
                  A1_dot_B0, -Tba[0], -Tab[1] - bA1_dot_B1))
        &&

        ((UB0_ux < 0) ||
         inVoronoi(a[1], b[0], -A0_dot_B0, -Tab[0] - bA0_dot_B1, A1_dot_B0,
                   Tab[1] + bA1_dot_B1, Tba[0])))
    {
      segCoords(t, u, a[1], b[0], A1_dot_B0, Tab[1] + bA1_dot_B1, Tba[0]);

      S[0] = Tab[0] + Rab[0][1] * b[1] + Rab[0][0] * u;
      S[1] = Tab[1] + Rab[1][1] * b[1] + Rab[1][0] * u - t;
      S[2] = Tab[2] + Rab[2][1] * b[1] + Rab[2][0] * u;

      if(P && Q)
      {
        *P = Vec3f(0, t, 0);
        *Q = S + (*P);
      }


      return S.length();
    }
  }

  // LA1, LB0

  if((LA1_ly < 0) && (LB0_lx < 0))
  {
    if(((LA1_uy < 0) ||
        inVoronoi(b[0], a[1], -A1_dot_B1, Tba[1], A1_dot_B0,
                  -Tba[0], -Tab[1]))
        &&
        ((LB0_ux < 0) ||
         inVoronoi(a[1], b[0], -A0_dot_B0, -Tab[0], A1_dot_B0,
                   Tab[1], Tba[0])))
    {
      segCoords(t, u, a[1], b[0], A1_dot_B0, Tab[1], Tba[0]);

      S[0] = Tab[0] + Rab[0][0] * u;
      S[1] = Tab[1] + Rab[1][0] * u - t;
      S[2] = Tab[2] + Rab[2][0] * u;

      if(P&& Q)
      {
        *P = Vec3f(0, t, 0);
        *Q = S + (*P);
      }

      return S.length();
    }
  }

  BVH_REAL BLL_y, BLU_y, BUL_y, BUU_y;

  BLL_y = Tab[1];
  BLU_y = BLL_y + bA1_dot_B1;
  BUL_y = BLL_y + bA1_dot_B0;
  BUU_y = BLU_y + bA1_dot_B0;

  BVH_REAL LA0_lx, LA0_ux, UA0_lx, UA0_ux, LB1_ly, LB1_uy, UB1_ly, UB1_uy;

  if(ALL_x < AUL_x)
  {
    LA0_lx = ALL_x;
    LA0_ux = AUL_x;
    UA0_lx = ALU_x;
    UA0_ux = AUU_x;
  }
  else
  {
    LA0_lx = AUL_x;
    LA0_ux = ALL_x;
    UA0_lx = AUU_x;
    UA0_ux = ALU_x;
  }

  if(BLL_y < BLU_y)
  {
    LB1_ly = BLL_y;
    LB1_uy = BLU_y;
    UB1_ly = BUL_y;
    UB1_uy = BUU_y;
  }
  else
  {
    LB1_ly = BLU_y;
    LB1_uy = BLL_y;
    UB1_ly = BUU_y;
    UB1_uy = BUL_y;
  }

  // UA0, UB1

  if((UA0_ux > b[0]) && (UB1_uy > a[1]))
  {
    if(((UA0_lx > b[0]) ||
        inVoronoi(b[1], a[0], A0_dot_B0, aA1_dot_B0 - Tba[0] - b[0],
                  A0_dot_B1, aA1_dot_B1 - Tba[1], -Tab[0] - bA0_dot_B0))
        &&
        ((UB1_ly > a[1]) ||
         inVoronoi(a[0], b[1], A1_dot_B1, Tab[1] - a[1] + bA1_dot_B0,
                   A0_dot_B1, Tab[0] + bA0_dot_B0, Tba[1] - aA1_dot_B1)))
    {
      segCoords(t, u, a[0], b[1], A0_dot_B1, Tab[0] + bA0_dot_B0,
                Tba[1] - aA1_dot_B1);

      S[0] = Tab[0] + Rab[0][0] * b[0] + Rab[0][1] * u - t;
      S[1] = Tab[1] + Rab[1][0] * b[0] + Rab[1][1] * u - a[1];
      S[2] = Tab[2] + Rab[2][0] * b[0] + Rab[2][1] * u;

      if(P && Q)
      {
        *P = Vec3f(t, a[1], 0);
        *Q = S + (*P);
      }

      return S.length();
    }
  }

  // UA0, LB1

  if((UA0_lx < 0) && (LB1_uy > a[1]))
  {
    if(((UA0_ux < 0) ||
        inVoronoi(b[1], a[0], -A0_dot_B0, Tba[0] - aA1_dot_B0, A0_dot_B1,
                  aA1_dot_B1 - Tba[1], -Tab[0]))
        &&
        ((LB1_ly > a[1]) ||
         inVoronoi(a[0], b[1], A1_dot_B1, Tab[1] - a[1], A0_dot_B1, Tab[0],
                   Tba[1] - aA1_dot_B1)))
    {
      segCoords(t, u, a[0], b[1], A0_dot_B1, Tab[0], Tba[1] - aA1_dot_B1);

      S[0] = Tab[0] + Rab[0][1] * u - t;
      S[1] = Tab[1] + Rab[1][1] * u - a[1];
      S[2] = Tab[2] + Rab[2][1] * u;

      if(P && Q)
      {
        *P = Vec3f(t, a[1], 0);
        *Q = S + (*P);
      }

      return S.length();
    }
  }

  // LA0, UB1

  if((LA0_ux > b[0]) && (UB1_ly < 0))
  {
    if(((LA0_lx > b[0]) ||
        inVoronoi(b[1], a[0], A0_dot_B0, -b[0] - Tba[0], A0_dot_B1, -Tba[1],
                  -bA0_dot_B0 - Tab[0]))
        &&
        ((UB1_uy < 0) ||
         inVoronoi(a[0], b[1], -A1_dot_B1, -Tab[1] - bA1_dot_B0, A0_dot_B1,
                   Tab[0] + bA0_dot_B0, Tba[1])))
    {
      segCoords(t, u, a[0], b[1], A0_dot_B1, Tab[0] + bA0_dot_B0, Tba[1]);

      S[0] = Tab[0] + Rab[0][0] * b[0] + Rab[0][1] * u - t;
      S[1] = Tab[1] + Rab[1][0] * b[0] + Rab[1][1] * u;
      S[2] = Tab[2] + Rab[2][0] * b[0] + Rab[2][1] * u;

      if(P && Q)
      {
        *P = Vec3f(t, 0, 0);
        *Q = S + (*P);
      }

      return S.length();
    }
  }

  // LA0, LB1

  if((LA0_lx < 0) && (LB1_ly < 0))
  {
    if(((LA0_ux < 0) ||
        inVoronoi(b[1], a[0], -A0_dot_B0, Tba[0], A0_dot_B1, -Tba[1],
                  -Tab[0]))
        &&
        ((LB1_uy < 0) ||
         inVoronoi(a[0], b[1], -A1_dot_B1, -Tab[1], A0_dot_B1,
                   Tab[0], Tba[1])))
    {
      segCoords(t, u, a[0], b[1], A0_dot_B1, Tab[0], Tba[1]);

      S[0] = Tab[0] + Rab[0][1] * u - t;
      S[1] = Tab[1] + Rab[1][1] * u;
      S[2] = Tab[2] + Rab[2][1] * u;

      if(P && Q)
      {
        *P = Vec3f(t, 0, 0);
        *Q = S + (*P);
      }

      return S.length();
    }
  }

  BVH_REAL LA0_ly, LA0_uy, UA0_ly, UA0_uy, LB0_ly, LB0_uy, UB0_ly, UB0_uy;

  if(ALL_y < AUL_y)
  {
    LA0_ly = ALL_y;
    LA0_uy = AUL_y;
    UA0_ly = ALU_y;
    UA0_uy = AUU_y;
  }
  else
  {
    LA0_ly = AUL_y;
    LA0_uy = ALL_y;
    UA0_ly = AUU_y;
    UA0_uy = ALU_y;
  }

  if(BLL_y < BUL_y)
  {
    LB0_ly = BLL_y;
    LB0_uy = BUL_y;
    UB0_ly = BLU_y;
    UB0_uy = BUU_y;
  }
  else
  {
    LB0_ly = BUL_y;
    LB0_uy = BLL_y;
    UB0_ly = BUU_y;
    UB0_uy = BLU_y;
  }

  // UA0, UB0

  if((UA0_uy > b[1]) && (UB0_uy > a[1]))
  {
    if(((UA0_ly > b[1]) ||
        inVoronoi(b[0], a[0], A0_dot_B1, aA1_dot_B1 - Tba[1] - b[1],
                  A0_dot_B0, aA1_dot_B0 - Tba[0], -Tab[0] - bA0_dot_B1))
        &&
        ((UB0_ly > a[1]) ||
         inVoronoi(a[0], b[0], A1_dot_B0, Tab[1] - a[1] + bA1_dot_B1, A0_dot_B0,
                   Tab[0] + bA0_dot_B1, Tba[0] - aA1_dot_B0)))
    {
      segCoords(t, u, a[0], b[0], A0_dot_B0, Tab[0] + bA0_dot_B1,
                Tba[0] - aA1_dot_B0);

      S[0] = Tab[0] + Rab[0][1] * b[1] + Rab[0][0] * u - t;
      S[1] = Tab[1] + Rab[1][1] * b[1] + Rab[1][0] * u - a[1];
      S[2] = Tab[2] + Rab[2][1] * b[1] + Rab[2][0] * u;

      if(P && Q)
      {
        *P = Vec3f(t, a[1], 0);
        *Q = S + (*P);
      }

      return S.length();
    }
  }

  // UA0, LB0

  if((UA0_ly < 0) && (LB0_uy > a[1]))
  {
    if(((UA0_uy < 0) ||
        inVoronoi(b[0], a[0], -A0_dot_B1, Tba[1] - aA1_dot_B1, A0_dot_B0,
                  aA1_dot_B0 - Tba[0], -Tab[0]))
        &&
        ((LB0_ly > a[1]) ||
         inVoronoi(a[0], b[0], A1_dot_B0, Tab[1] - a[1],
                   A0_dot_B0, Tab[0], Tba[0] - aA1_dot_B0)))
    {
      segCoords(t, u, a[0], b[0], A0_dot_B0, Tab[0], Tba[0] - aA1_dot_B0);

      S[0] = Tab[0] + Rab[0][0] * u - t;
      S[1] = Tab[1] + Rab[1][0] * u - a[1];
      S[2] = Tab[2] + Rab[2][0] * u;

      if(P && Q)
      {
        *P = Vec3f(t, a[1], 0);
        *Q = S + (*P);
      }

      return S.length();
    }
  }

  // LA0, UB0

  if((LA0_uy > b[1]) && (UB0_ly < 0))
  {
    if(((LA0_ly > b[1]) ||
        inVoronoi(b[0], a[0], A0_dot_B1, -Tba[1] - b[1], A0_dot_B0, -Tba[0],
                  -Tab[0] - bA0_dot_B1))
        &&

        ((UB0_uy < 0) ||
         inVoronoi(a[0], b[0], -A1_dot_B0, -Tab[1] - bA1_dot_B1, A0_dot_B0,
                   Tab[0] + bA0_dot_B1, Tba[0])))
    {
      segCoords(t, u, a[0], b[0], A0_dot_B0, Tab[0] + bA0_dot_B1, Tba[0]);

       S[0] = Tab[0] + Rab[0][1] * b[1] + Rab[0][0] * u - t;
       S[1] = Tab[1] + Rab[1][1] * b[1] + Rab[1][0] * u;
       S[2] = Tab[2] + Rab[2][1] * b[1] + Rab[2][0] * u;

       if(P && Q)
       {
         *P = Vec3f(t, 0, 0);
         *Q = S + (*P);
       }

       return S.length();
    }
  }

  // LA0, LB0

  if((LA0_ly < 0) && (LB0_ly < 0))
  {
    if(((LA0_uy < 0) ||
        inVoronoi(b[0], a[0], -A0_dot_B1, Tba[1], A0_dot_B0,
                  -Tba[0], -Tab[0]))
        &&
        ((LB0_uy < 0) ||
         inVoronoi(a[0], b[0], -A1_dot_B0, -Tab[1], A0_dot_B0,
                   Tab[0], Tba[0])))
    {
      segCoords(t, u, a[0], b[0], A0_dot_B0, Tab[0], Tba[0]);

      S[0] = Tab[0] + Rab[0][0] * u - t;
      S[1] = Tab[1] + Rab[1][0] * u;
      S[2] = Tab[2] + Rab[2][0] * u;

      if(P && Q)
      {
        *P = Vec3f(t, 0, 0);
        *Q = S + (*P);
      }

      return S.length();
    }
  }

  // no edges passed, take max separation along face normals

  BVH_REAL sep1, sep2;

  if(Tab[2] > 0.0)
  {
    sep1 = Tab[2];
    if (Rab[2][0] < 0.0) sep1 += b[0] * Rab[2][0];
    if (Rab[2][1] < 0.0) sep1 += b[1] * Rab[2][1];
  }
  else
  {
    sep1 = -Tab[2];
    if (Rab[2][0] > 0.0) sep1 -= b[0] * Rab[2][0];
    if (Rab[2][1] > 0.0) sep1 -= b[1] * Rab[2][1];
  }

  if(Tba[2] < 0)
  {
    sep2 = -Tba[2];
    if (Rab[0][2] < 0.0) sep2 += a[0] * Rab[0][2];
    if (Rab[1][2] < 0.0) sep2 += a[1] * Rab[1][2];
  }
  else
  {
    sep2 = Tba[2];
    if (Rab[0][2] > 0.0) sep2 -= a[0] * Rab[0][2];
    if (Rab[1][2] > 0.0) sep2 -= a[1] * Rab[1][2];
  }

  if(sep1 >= sep2 && sep1 >= 0)
  {
    if(Tab[2] > 0)
      S = Vec3f(0, 0, sep1);
    else
      S = Vec3f(0, 0, -sep1);

    if(P && Q)
    {
      *Q = S;
      *P = Vec3f(0, 0, 0);
    }
  }

  if(sep2 >= sep1 && sep2 >= 0)
  {
    Vec3f Q_(Tab[0], Tab[1], Tab[2]);
    Vec3f P_;
    if(Tba[2] < 0)
    {
      P_[0] = Rab[0][2] * sep2 + Tab[0];
      P_[1] = Rab[1][2] * sep2 + Tab[1];
      P_[2] = Rab[2][2] * sep2 + Tab[2];
    }
    else
    {
      P_[0] = -Rab[0][2] * sep2 + Tab[0];
      P_[1] = -Rab[1][2] * sep2 + Tab[1];
      P_[2] = -Rab[2][2] * sep2 + Tab[2];
    }

    S = Q_ - P_;

    if(P && Q)
    {
      *P = P_;
      *Q = Q_;
    }
  }

  BVH_REAL sep = (sep1 > sep2 ? sep1 : sep2);
  return (sep > 0 ? sep : 0);
}


void RSS::clipToRange(BVH_REAL& val, BVH_REAL a, BVH_REAL b)
{
  if(val < a) val = a;
  else if(val > b) val = b;
}


void RSS::segCoords(BVH_REAL& t, BVH_REAL& u, BVH_REAL a, BVH_REAL b, BVH_REAL A_dot_B, BVH_REAL A_dot_T, BVH_REAL B_dot_T)
{
  BVH_REAL denom = 1 - A_dot_B * A_dot_B;

  if(denom == 0) t = 0;
  else
  {
    t = (A_dot_T - B_dot_T * A_dot_B) / denom;
    clipToRange(t, 0, a);
  }

  u = t * A_dot_B - B_dot_T;
  if(u < 0)
  {
    u = 0;
    t = A_dot_T;
    clipToRange(t, 0, a);
  }
  else if(u > b)
  {
    u = b;
    t = u * A_dot_B + A_dot_T;
    clipToRange(t, 0, a);
  }
}

bool RSS::inVoronoi(BVH_REAL a, BVH_REAL b, BVH_REAL Anorm_dot_B, BVH_REAL Anorm_dot_T, BVH_REAL A_dot_B, BVH_REAL A_dot_T, BVH_REAL B_dot_T)
{
  if(fabs(Anorm_dot_B) < 1e-7) return false;

  BVH_REAL t, u, v;

  u = -Anorm_dot_T / Anorm_dot_B;
  clipToRange(u, 0, b);

  t = u * A_dot_B + A_dot_T;
  clipToRange(t, 0, a);

  v = t * A_dot_B - B_dot_T;

  if(Anorm_dot_B > 0)
  {
    if(v > (u + 1e-7)) return true;
  }
  else
  {
    if(v < (u - 1e-7)) return true;
  }
  return false;
}





}

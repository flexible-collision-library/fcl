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

/** @author Jia Pan */

#ifndef FCL_BV_RSS_INL_H
#define FCL_BV_RSS_INL_H

#include "fcl/math/bv/RSS.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT RSS<double>;

//==============================================================================
extern template
void clipToRange(double& val, double a, double b);

//==============================================================================
extern template
void segCoords(
        double& t,
        double& u,
        double a,
        double b,
        double A_dot_B,
        double A_dot_T,
        double B_dot_T);

//==============================================================================
extern template
bool inVoronoi(
        double a,
        double b,
        double Anorm_dot_B,
        double Anorm_dot_T,
        double A_dot_B,
        double A_dot_T,
        double B_dot_T);

//==============================================================================
extern template
double rectDistance(
    const Matrix3<double>& Rab,
    const Vector3<double>& Tab,
    const double a[2],
    const double b[2],
    Vector3<double>* P,
    Vector3<double>* Q);

//==============================================================================
extern template
double rectDistance(
    const Transform3<double>& tfab,
    const double a[2],
    const double b[2],
    Vector3<double>* P,
    Vector3<double>* Q);

//==============================================================================
extern template
RSS<double> translate(const RSS<double>& bv, const Vector3<double>& t);

//==============================================================================
template <typename S>
RSS<S>::RSS()
  : axis(Matrix3<S>::Identity()), To(Vector3<S>::Zero())
{
  // Do nothing
}

//==============================================================================
template <typename S>
bool RSS<S>::overlap(const RSS<S>& other) const
{
  Vector3<S> t = other.To - To;
  Vector3<S> T(
        axis.col(0).dot(t), axis.col(1).dot(t), axis.col(2).dot(t));
  Matrix3<S> R = axis.transpose() * other.axis;

  S dist = rectDistance(R, T, l, other.l);
  return (dist <= (r + other.r));
}

//==============================================================================
template <typename S>
bool RSS<S>::overlap(const RSS<S>& other,
                          RSS<S>& /*overlap_part*/) const
{
  return overlap(other);
}

//==============================================================================
template <typename S>
bool RSS<S>::contain(const Vector3<S>& p) const
{
  Vector3<S> local_p = p - To;
  Vector3<S> proj(
      axis.col(0).dot(local_p),
      axis.col(1).dot(local_p),
      axis.col(2).dot(local_p));
  S abs_proj2 = fabs(proj[2]);

  /// projection is within the rectangle
  if((proj[0] < l[0]) && (proj[0] > 0) && (proj[1] < l[1]) && (proj[1] > 0))
  {
    return (abs_proj2 < r);
  }
  else if((proj[0] < l[0]) && (proj[0] > 0) && ((proj[1] < 0) || (proj[1] > l[1])))
  {
    S y = (proj[1] > 0) ? l[1] : 0;
    Vector3<S> v(proj[0], y, 0);
    return ((proj - v).squaredNorm() < r * r);
  }
  else if((proj[1] < l[1]) && (proj[1] > 0) && ((proj[0] < 0) || (proj[0] > l[0])))
  {
    S x = (proj[0] > 0) ? l[0] : 0;
    Vector3<S> v(x, proj[1], 0);
    return ((proj - v).squaredNorm() < r * r);
  }
  else
  {
    S x = (proj[0] > 0) ? l[0] : 0;
    S y = (proj[1] > 0) ? l[1] : 0;
    Vector3<S> v(x, y, 0);
    return ((proj - v).squaredNorm() < r * r);
  }
}

//==============================================================================
template <typename S>
RSS<S>& RSS<S>::operator +=(const Vector3<S>& p)

{
  Vector3<S> local_p = p - To;
  Vector3<S> proj(
      axis.col(0).dot(local_p),
      axis.col(1).dot(local_p),
      axis.col(2).dot(local_p));
  S abs_proj2 = fabs(proj[2]);

  // projection is within the rectangle
  if((proj[0] < l[0]) && (proj[0] > 0) && (proj[1] < l[1]) && (proj[1] > 0))
  {
    if(abs_proj2 < r)
      ; // do nothing
    else
    {
      r = 0.5 * (r + abs_proj2); // enlarge the r
      // change RSS origin position
      if(proj[2] > 0)
        To[2] += 0.5 * (abs_proj2 - r);
      else
        To[2] -= 0.5 * (abs_proj2 - r);
    }
  }
  else if((proj[0] < l[0]) && (proj[0] > 0) && ((proj[1] < 0) || (proj[1] > l[1])))
  {
    S y = (proj[1] > 0) ? l[1] : 0;
    Vector3<S> v(proj[0], y, 0);
    S new_r_sqr = (proj - v).squaredNorm();
    if(new_r_sqr < r * r)
      ; // do nothing
    else
    {
      if(abs_proj2 < r)
      {
        S delta_y = - std::sqrt(r * r - proj[2] * proj[2]) + fabs(proj[1] - y);
        l[1] += delta_y;
        if(proj[1] < 0)
          To[1] -= delta_y;
      }
      else
      {
        S delta_y = fabs(proj[1] - y);
        l[1] += delta_y;
        if(proj[1] < 0)
          To[1] -= delta_y;

        if(proj[2] > 0)
          To[2] += 0.5 * (abs_proj2 - r);
        else
          To[2] -= 0.5 * (abs_proj2 - r);
      }
    }
  }
  else if((proj[1] < l[1]) && (proj[1] > 0) && ((proj[0] < 0) || (proj[0] > l[0])))
  {
    S x = (proj[0] > 0) ? l[0] : 0;
    Vector3<S> v(x, proj[1], 0);
    S new_r_sqr = (proj - v).squaredNorm();
    if(new_r_sqr < r * r)
      ; // do nothing
    else
    {
      if(abs_proj2 < r)
      {
        S delta_x = - std::sqrt(r * r - proj[2] * proj[2]) + fabs(proj[0] - x);
        l[0] += delta_x;
        if(proj[0] < 0)
          To[0] -= delta_x;
      }
      else
      {
        S delta_x = fabs(proj[0] - x);
        l[0] += delta_x;
        if(proj[0] < 0)
          To[0] -= delta_x;

        if(proj[2] > 0)
          To[2] += 0.5 * (abs_proj2 - r);
        else
          To[2] -= 0.5 * (abs_proj2 - r);
      }
    }
  }
  else
  {
    S x = (proj[0] > 0) ? l[0] : 0;
    S y = (proj[1] > 0) ? l[1] : 0;
    Vector3<S> v(x, y, 0);
    S new_r_sqr = (proj - v).squaredNorm();
    if(new_r_sqr < r * r)
      ; // do nothing
    else
    {
      if(abs_proj2 < r)
      {
        S diag = std::sqrt(new_r_sqr - proj[2] * proj[2]);
        S delta_diag = - std::sqrt(r * r - proj[2] * proj[2]) + diag;

        S delta_x = delta_diag / diag * fabs(proj[0] - x);
        S delta_y = delta_diag / diag * fabs(proj[1] - y);
        l[0] += delta_x;
        l[1] += delta_y;

        if(proj[0] < 0 && proj[1] < 0)
        {
          To[0] -= delta_x;
          To[1] -= delta_y;
        }
      }
      else
      {
        S delta_x = fabs(proj[0] - x);
        S delta_y = fabs(proj[1] - y);

        l[0] += delta_x;
        l[1] += delta_y;

        if(proj[0] < 0 && proj[1] < 0)
        {
          To[0] -= delta_x;
          To[1] -= delta_y;
        }

        if(proj[2] > 0)
          To[2] += 0.5 * (abs_proj2 - r);
        else
          To[2] -= 0.5 * (abs_proj2 - r);
      }
    }
  }

  return *this;
}

//==============================================================================
template <typename S>
RSS<S>& RSS<S>::operator +=(const RSS<S>& other)
{
  *this = *this + other;
  return *this;
}

//==============================================================================
template <typename S>
RSS<S> RSS<S>::operator +(const RSS<S>& other) const
{
  RSS<S> bv;

  Vector3<S> v[16];

  Vector3<S> d0_pos = other.axis.col(0) * (other.l[0] + other.r);
  Vector3<S> d1_pos = other.axis.col(1) * (other.l[1] + other.r);

  Vector3<S> d0_neg = other.axis.col(0) * (-other.r);
  Vector3<S> d1_neg = other.axis.col(1) * (-other.r);

  Vector3<S> d2_pos = other.axis.col(2) * other.r;
  Vector3<S> d2_neg = other.axis.col(2) * (-other.r);

  v[0] = other.To + d0_pos + d1_pos + d2_pos;
  v[1] = other.To + d0_pos + d1_pos + d2_neg;
  v[2] = other.To + d0_pos + d1_neg + d2_pos;
  v[3] = other.To + d0_pos + d1_neg + d2_neg;
  v[4] = other.To + d0_neg + d1_pos + d2_pos;
  v[5] = other.To + d0_neg + d1_pos + d2_neg;
  v[6] = other.To + d0_neg + d1_neg + d2_pos;
  v[7] = other.To + d0_neg + d1_neg + d2_neg;

  d0_pos.noalias() = axis.col(0) * (l[0] + r);
  d1_pos.noalias() = axis.col(1) * (l[1] + r);
  d0_neg.noalias() = axis.col(0) * (-r);
  d1_neg.noalias() = axis.col(1) * (-r);
  d2_pos.noalias() = axis.col(2) * r;
  d2_neg.noalias() = axis.col(2) * (-r);

  v[8] = To + d0_pos + d1_pos + d2_pos;
  v[9] = To + d0_pos + d1_pos + d2_neg;
  v[10] = To + d0_pos + d1_neg + d2_pos;
  v[11] = To + d0_pos + d1_neg + d2_neg;
  v[12] = To + d0_neg + d1_pos + d2_pos;
  v[13] = To + d0_neg + d1_pos + d2_neg;
  v[14] = To + d0_neg + d1_neg + d2_pos;
  v[15] = To + d0_neg + d1_neg + d2_neg;


  Matrix3<S> M; // row first matrix
  Matrix3<S> E; // row first eigen-vectors
  Vector3<S> s(0, 0, 0);

  getCovariance<S>(v, nullptr, nullptr, nullptr, 16, M);
  eigen_old(M, s, E);

  int min, mid, max;
  if(s[0] > s[1]) { max = 0; min = 1; }
  else { min = 0; max = 1; }
  if(s[2] < s[min]) { mid = min; min = 2; }
  else if(s[2] > s[max]) { mid = max; max = 2; }
  else { mid = 2; }

  // column first matrix, as the axis in RSS
  bv.axis.col(0) = E.col(max);
  bv.axis.col(1) = E.col(mid);
  bv.axis.col(2).noalias() = axis.col(0).cross(axis.col(1));

  // set rss origin, rectangle size and radius
  getRadiusAndOriginAndRectangleSize<S>(v, nullptr, nullptr, nullptr, 16, bv.axis, bv.To, bv.l, bv.r);

  return bv;
}

//==============================================================================
template <typename S>
S RSS<S>::width() const
{
  return l[0] + 2 * r;
}

//==============================================================================
template <typename S>
S RSS<S>::height() const
{
  return l[1] + 2 * r;
}

//==============================================================================
template <typename S>
S RSS<S>::depth() const
{
  return 2 * r;
}

//==============================================================================
template <typename S>
S RSS<S>::volume() const
{
  return (l[0] * l[1] * 2 * r + 4 * constants<S>::pi() * r * r * r);
}

//==============================================================================
template <typename S>
S RSS<S>::size() const
{
  return (std::sqrt(l[0] * l[0] + l[1] * l[1]) + 2 * r);
}

//==============================================================================
template <typename S>
const Vector3<S> RSS<S>::center() const
{
  Vector3<S> p_ToCenter_T;
  p_ToCenter_T << l[0] * 0.5, l[1] * 0.5, 0.0;
  return p_FoTo_F() + R_FT() * p_ToCenter_T;
}

template <typename S>
void RSS<S>::setToFromCenter(const Vector3<S>& p_FoCenter_F)
{
  Vector3<S> p_ToCenter_T;
  p_ToCenter_T << l[0] * 0.5, l[1] * 0.5, 0.0;
  p_FoTo_F() = p_FoCenter_F - R_FT() * p_ToCenter_T;
}

//==============================================================================
template <typename S>
S RSS<S>::distance(
    const RSS<S>& other,
    Vector3<S>* P,
    Vector3<S>* Q) const
{
  Vector3<S> t = other.To - To;
  Vector3<S> T(
        axis.col(0).dot(t), axis.col(1).dot(t), axis.col(2).dot(t));
  Matrix3<S> R = axis.transpose() * other.axis;

  S dist = rectDistance(R, T, l, other.l, P, Q);
  dist -= (r + other.r);
  return (dist < (S)0.0) ? (S)0.0 : dist;
}

//==============================================================================
template <typename S>
void clipToRange(S& val, S a, S b)
{
  if(val < a) val = a;
  else if(val > b) val = b;
}

//==============================================================================
template <typename S>
void segCoords(S& t, S& u, S a, S b, S A_dot_B, S A_dot_T, S B_dot_T)
{
  S denom = 1 - A_dot_B * A_dot_B;

  if(denom == 0) t = 0;
  else
  {
    t = (A_dot_T - B_dot_T * A_dot_B) / denom;
    clipToRange(t, (S)0.0, a);
  }

  u = t * A_dot_B - B_dot_T;
  if(u < 0)
  {
    u = 0;
    t = A_dot_T;
    clipToRange(t, (S)0.0, a);
  }
  else if(u > b)
  {
    u = b;
    t = u * A_dot_B + A_dot_T;
    clipToRange(t, (S)0.0, a);
  }
}

//==============================================================================
template <typename S>
bool inVoronoi(S a, S b, S Anorm_dot_B, S Anorm_dot_T, S A_dot_B, S A_dot_T, S B_dot_T)
{
  if(fabs(Anorm_dot_B) < 1e-7) return false;

  S t, u, v;

  u = -Anorm_dot_T / Anorm_dot_B;
  clipToRange(u, (S)0.0, b);

  t = u * A_dot_B + A_dot_T;
  clipToRange(t, (S)0.0, a);

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

//==============================================================================
template <typename S>
S rectDistance(const Matrix3<S>& Rab, Vector3<S> const& Tab, const S a[2], const S b[2], Vector3<S>* P, Vector3<S>* Q)
{
  S A0_dot_B0, A0_dot_B1, A1_dot_B0, A1_dot_B1;

  A0_dot_B0 = Rab(0, 0);
  A0_dot_B1 = Rab(0, 1);
  A1_dot_B0 = Rab(1, 0);
  A1_dot_B1 = Rab(1, 1);

  S aA0_dot_B0, aA0_dot_B1, aA1_dot_B0, aA1_dot_B1;
  S bA0_dot_B0, bA0_dot_B1, bA1_dot_B0, bA1_dot_B1;

  aA0_dot_B0 = a[0] * A0_dot_B0;
  aA0_dot_B1 = a[0] * A0_dot_B1;
  aA1_dot_B0 = a[1] * A1_dot_B0;
  aA1_dot_B1 = a[1] * A1_dot_B1;
  bA0_dot_B0 = b[0] * A0_dot_B0;
  bA1_dot_B0 = b[0] * A1_dot_B0;
  bA0_dot_B1 = b[1] * A0_dot_B1;
  bA1_dot_B1 = b[1] * A1_dot_B1;

  Vector3<S> Tba = Rab.transpose() * Tab;

  Vector3<S> D;
  S t, u;

  // determine if any edge pair contains the closest points

  S ALL_x, ALU_x, AUL_x, AUU_x;
  S BLL_x, BLU_x, BUL_x, BUU_x;
  S LA1_lx, LA1_ux, UA1_lx, UA1_ux, LB1_lx, LB1_ux, UB1_lx, UB1_ux;

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

      D[0] = Tab[0] + Rab(0, 0) * b[0] + Rab(0, 1) * u - a[0] ;
      D[1] = Tab[1] + Rab(1, 0) * b[0] + Rab(1, 1) * u - t;
      D[2] = Tab[2] + Rab(2, 0) * b[0] + Rab(2, 1) * u;

      if(P && Q)
      {
        *P << a[0], t, 0;
        *Q = D + (*P);
      }

      return D.norm();
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

      D[0] = Tab[0] + Rab(0, 1) * u - a[0];
      D[1] = Tab[1] + Rab(1, 1) * u - t;
      D[2] = Tab[2] + Rab(2, 1) * u;

      if(P && Q)
      {
        *P << a[0], t, 0;
        *Q = D + (*P);
      }

      return D.norm();
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

      D[0] = Tab[0] + Rab(0, 0) * b[0] + Rab(0, 1) * u;
      D[1] = Tab[1] + Rab(1, 0) * b[0] + Rab(1, 1) * u - t;
      D[2] = Tab[2] + Rab(2, 0) * b[0] + Rab(2, 1) * u;

      if(P && Q)
      {
        *P << 0, t, 0;
        *Q = D + (*P);
      }

      return D.norm();
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

      D[0] = Tab[0] + Rab(0, 1) * u;
      D[1] = Tab[1] + Rab(1, 1) * u - t;
      D[2] = Tab[2] + Rab(2, 1) * u;

      if(P && Q)
      {
        *P << 0, t, 0;
        *Q = D + (*P);
      }

      return D.norm();
    }
  }

  S ALL_y, ALU_y, AUL_y, AUU_y;

  ALL_y = -Tba[1];
  ALU_y = ALL_y + aA1_dot_B1;
  AUL_y = ALL_y + aA0_dot_B1;
  AUU_y = ALU_y + aA0_dot_B1;

  S LA1_ly, LA1_uy, UA1_ly, UA1_uy, LB0_lx, LB0_ux, UB0_lx, UB0_ux;

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

      D[0] = Tab[0] + Rab(0, 1) * b[1] + Rab(0, 0) * u - a[0] ;
      D[1] = Tab[1] + Rab(1, 1) * b[1] + Rab(1, 0) * u - t;
      D[2] = Tab[2] + Rab(2, 1) * b[1] + Rab(2, 0) * u;

      if(P && Q)
      {
        *P << a[0], t, 0;
        *Q = D + (*P);
      }

      return D.norm();
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

      D[0] = Tab[0] + Rab(0, 0) * u - a[0];
      D[1] = Tab[1] + Rab(1, 0) * u - t;
      D[2] = Tab[2] + Rab(2, 0) * u;

      if(P && Q)
      {
        *P << a[0], t, 0;
        *Q = D + (*P);
      }

      return D.norm();
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

      D[0] = Tab[0] + Rab(0, 1) * b[1] + Rab(0, 0) * u;
      D[1] = Tab[1] + Rab(1, 1) * b[1] + Rab(1, 0) * u - t;
      D[2] = Tab[2] + Rab(2, 1) * b[1] + Rab(2, 0) * u;

      if(P && Q)
      {
        *P << 0, t, 0;
        *Q = D + (*P);
      }


      return D.norm();
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

      D[0] = Tab[0] + Rab(0, 0) * u;
      D[1] = Tab[1] + Rab(1, 0) * u - t;
      D[2] = Tab[2] + Rab(2, 0) * u;

      if(P&& Q)
      {
        *P << 0, t, 0;
        *Q = D + (*P);
      }

      return D.norm();
    }
  }

  S BLL_y, BLU_y, BUL_y, BUU_y;

  BLL_y = Tab[1];
  BLU_y = BLL_y + bA1_dot_B1;
  BUL_y = BLL_y + bA1_dot_B0;
  BUU_y = BLU_y + bA1_dot_B0;

  S LA0_lx, LA0_ux, UA0_lx, UA0_ux, LB1_ly, LB1_uy, UB1_ly, UB1_uy;

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

      D[0] = Tab[0] + Rab(0, 0) * b[0] + Rab(0, 1) * u - t;
      D[1] = Tab[1] + Rab(1, 0) * b[0] + Rab(1, 1) * u - a[1];
      D[2] = Tab[2] + Rab(2, 0) * b[0] + Rab(2, 1) * u;

      if(P && Q)
      {
        *P << t, a[1], 0;
        *Q = D + (*P);
      }

      return D.norm();
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

      D[0] = Tab[0] + Rab(0, 1) * u - t;
      D[1] = Tab[1] + Rab(1, 1) * u - a[1];
      D[2] = Tab[2] + Rab(2, 1) * u;

      if(P && Q)
      {
        *P << t, a[1], 0;
        *Q = D + (*P);
      }

      return D.norm();
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

      D[0] = Tab[0] + Rab(0, 0) * b[0] + Rab(0, 1) * u - t;
      D[1] = Tab[1] + Rab(1, 0) * b[0] + Rab(1, 1) * u;
      D[2] = Tab[2] + Rab(2, 0) * b[0] + Rab(2, 1) * u;

      if(P && Q)
      {
        *P << t, 0, 0;
        *Q = D + (*P);
      }

      return D.norm();
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

      D[0] = Tab[0] + Rab(0, 1) * u - t;
      D[1] = Tab[1] + Rab(1, 1) * u;
      D[2] = Tab[2] + Rab(2, 1) * u;

      if(P && Q)
      {
        *P << t, 0, 0;
        *Q = D + (*P);
      }

      return D.norm();
    }
  }

  S LA0_ly, LA0_uy, UA0_ly, UA0_uy, LB0_ly, LB0_uy, UB0_ly, UB0_uy;

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

      D[0] = Tab[0] + Rab(0, 1) * b[1] + Rab(0, 0) * u - t;
      D[1] = Tab[1] + Rab(1, 1) * b[1] + Rab(1, 0) * u - a[1];
      D[2] = Tab[2] + Rab(2, 1) * b[1] + Rab(2, 0) * u;

      if(P && Q)
      {
        *P << t, a[1], 0;
        *Q = D + (*P);
      }

      return D.norm();
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

      D[0] = Tab[0] + Rab(0, 0) * u - t;
      D[1] = Tab[1] + Rab(1, 0) * u - a[1];
      D[2] = Tab[2] + Rab(2, 0) * u;

      if(P && Q)
      {
        *P << t, a[1], 0;
        *Q = D + (*P);
      }

      return D.norm();
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

      D[0] = Tab[0] + Rab(0, 1) * b[1] + Rab(0, 0) * u - t;
      D[1] = Tab[1] + Rab(1, 1) * b[1] + Rab(1, 0) * u;
      D[2] = Tab[2] + Rab(2, 1) * b[1] + Rab(2, 0) * u;

      if(P && Q)
      {
        *P << t, 0, 0;
        *Q = D + (*P);
      }

      return D.norm();
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

      D[0] = Tab[0] + Rab(0, 0) * u - t;
      D[1] = Tab[1] + Rab(1, 0) * u;
      D[2] = Tab[2] + Rab(2, 0) * u;

      if(P && Q)
      {
        *P << t, 0, 0;
        *Q = D + (*P);
      }

      return D.norm();
    }
  }

  // no edges passed, take max separation along face normals

  S sep1, sep2;

  if(Tab[2] > 0.0)
  {
    sep1 = Tab[2];
    if (Rab(2, 0) < 0.0) sep1 += b[0] * Rab(2, 0);
    if (Rab(2, 1) < 0.0) sep1 += b[1] * Rab(2, 1);
  }
  else
  {
    sep1 = -Tab[2];
    if (Rab(2, 0) > 0.0) sep1 -= b[0] * Rab(2, 0);
    if (Rab(2, 1) > 0.0) sep1 -= b[1] * Rab(2, 1);
  }

  if(Tba[2] < 0)
  {
    sep2 = -Tba[2];
    if (Rab(0, 2) < 0.0) sep2 += a[0] * Rab(0, 2);
    if (Rab(1, 2) < 0.0) sep2 += a[1] * Rab(1, 2);
  }
  else
  {
    sep2 = Tba[2];
    if (Rab(0, 2) > 0.0) sep2 -= a[0] * Rab(0, 2);
    if (Rab(1, 2) > 0.0) sep2 -= a[1] * Rab(1, 2);
  }

  if(sep1 >= sep2 && sep1 >= 0)
  {
    if(Tab[2] > 0)
      D << 0, 0, sep1;
    else
      D << 0, 0, -sep1;

    if(P && Q)
    {
      *Q = D;
      P->setZero();
    }
  }

  if(sep2 >= sep1 && sep2 >= 0)
  {
    Vector3<S> Q_(Tab[0], Tab[1], Tab[2]);
    Vector3<S> P_;
    if(Tba[2] < 0)
    {
      P_[0] = Rab(0, 2) * sep2 + Tab[0];
      P_[1] = Rab(1, 2) * sep2 + Tab[1];
      P_[2] = Rab(2, 2) * sep2 + Tab[2];
    }
    else
    {
      P_[0] = -Rab(0, 2) * sep2 + Tab[0];
      P_[1] = -Rab(1, 2) * sep2 + Tab[1];
      P_[2] = -Rab(2, 2) * sep2 + Tab[2];
    }

    D = Q_ - P_;

    if(P && Q)
    {
      *P = P_;
      *Q = Q_;
    }
  }

  S sep = (sep1 > sep2 ? sep1 : sep2);
  return (sep > 0 ? sep : 0);
}

//==============================================================================
template <typename S>
S rectDistance(
    const Transform3<S>& tfab,
    const S a[2],
    const S b[2],
    Vector3<S>* P,
    Vector3<S>* Q)
{
  S A0_dot_B0 = tfab.linear()(0, 0);
  S A0_dot_B1 = tfab.linear()(0, 1);
  S A1_dot_B0 = tfab.linear()(1, 0);
  S A1_dot_B1 = tfab.linear()(1, 1);

  S aA0_dot_B0 = a[0] * A0_dot_B0;
  S aA0_dot_B1 = a[0] * A0_dot_B1;
  S aA1_dot_B0 = a[1] * A1_dot_B0;
  S aA1_dot_B1 = a[1] * A1_dot_B1;
  S bA0_dot_B0 = b[0] * A0_dot_B0;
  S bA1_dot_B0 = b[0] * A1_dot_B0;
  S bA0_dot_B1 = b[1] * A0_dot_B1;
  S bA1_dot_B1 = b[1] * A1_dot_B1;

  Vector3<S> Tba = tfab.linear().transpose() * tfab.translation();

  Vector3<S> D;
  S t, u;

  // determine if any edge pair contains the closest points

  S LA1_lx, LA1_ux, UA1_lx, UA1_ux, LB1_lx, LB1_ux, UB1_lx, UB1_ux;

  S ALL_x = -Tba[0];
  S ALU_x = ALL_x + aA1_dot_B0;
  S AUL_x = ALL_x + aA0_dot_B0;
  S AUU_x = ALU_x + aA0_dot_B0;

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

  S BLL_x = tfab.translation()[0];
  S BLU_x = BLL_x + bA0_dot_B1;
  S BUL_x = BLL_x + bA0_dot_B0;
  S BUU_x = BLU_x + bA0_dot_B0;

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
                  -tfab.translation()[1] - bA1_dot_B0))
       &&
       ((UB1_lx > a[0]) ||
        inVoronoi(a[1], b[1], A0_dot_B1, tfab.translation()[0] + bA0_dot_B0 - a[0],
                  A1_dot_B1, tfab.translation()[1] + bA1_dot_B0, Tba[1] - aA0_dot_B1)))
    {
      segCoords(t, u, a[1], b[1], A1_dot_B1, tfab.translation()[1] + bA1_dot_B0,
                Tba[1] - aA0_dot_B1);

      D[0] = tfab.translation()[0] + tfab.linear()(0, 0) * b[0] + tfab.linear()(0, 1) * u - a[0] ;
      D[1] = tfab.translation()[1] + tfab.linear()(1, 0) * b[0] + tfab.linear()(1, 1) * u - t;
      D[2] = tfab.translation()[2] + tfab.linear()(2, 0) * b[0] + tfab.linear()(2, 1) * u;

      if(P && Q)
      {
        *P << a[0], t, 0;
        *Q = D + (*P);
      }

      return D.norm();
    }
  }


  // UA1, LB1

  if((UA1_lx < 0) && (LB1_ux > a[0]))
  {
    if(((UA1_ux < 0) ||
        inVoronoi(b[1], a[1], -A1_dot_B0, Tba[0] - aA0_dot_B0,
                  A1_dot_B1, aA0_dot_B1 - Tba[1], -tfab.translation()[1]))
       &&
       ((LB1_lx > a[0]) ||
        inVoronoi(a[1], b[1], A0_dot_B1, tfab.translation()[0] - a[0],
                  A1_dot_B1, tfab.translation()[1], Tba[1] - aA0_dot_B1)))
    {
      segCoords(t, u, a[1], b[1], A1_dot_B1, tfab.translation()[1], Tba[1] - aA0_dot_B1);

      D[0] = tfab.translation()[0] + tfab.linear()(0, 1) * u - a[0];
      D[1] = tfab.translation()[1] + tfab.linear()(1, 1) * u - t;
      D[2] = tfab.translation()[2] + tfab.linear()(2, 1) * u;

      if(P && Q)
      {
        *P << a[0], t, 0;
        *Q = D + (*P);
      }

      return D.norm();
    }
  }

  // LA1, UB1

  if((LA1_ux > b[0]) && (UB1_lx < 0))
  {
    if(((LA1_lx > b[0]) ||
        inVoronoi(b[1], a[1], A1_dot_B0, -Tba[0] - b[0],
                  A1_dot_B1, -Tba[1], -tfab.translation()[1] - bA1_dot_B0))
       &&
       ((UB1_ux < 0) ||
        inVoronoi(a[1], b[1], -A0_dot_B1, -tfab.translation()[0] - bA0_dot_B0,
                  A1_dot_B1, tfab.translation()[1] + bA1_dot_B0, Tba[1])))
    {
      segCoords(t, u, a[1], b[1], A1_dot_B1, tfab.translation()[1] + bA1_dot_B0, Tba[1]);

      D[0] = tfab.translation()[0] + tfab.linear()(0, 0) * b[0] + tfab.linear()(0, 1) * u;
      D[1] = tfab.translation()[1] + tfab.linear()(1, 0) * b[0] + tfab.linear()(1, 1) * u - t;
      D[2] = tfab.translation()[2] + tfab.linear()(2, 0) * b[0] + tfab.linear()(2, 1) * u;

      if(P && Q)
      {
        *P << 0, t, 0;
        *Q = D + (*P);
      }

      return D.norm();
    }
  }

  // LA1, LB1

  if((LA1_lx < 0) && (LB1_lx < 0))
  {
    if (((LA1_ux < 0) ||
         inVoronoi(b[1], a[1], -A1_dot_B0, Tba[0], A1_dot_B1,
                   -Tba[1], -tfab.translation()[1]))
        &&
        ((LB1_ux < 0) ||
         inVoronoi(a[1], b[1], -A0_dot_B1, -tfab.translation()[0], A1_dot_B1,
                   tfab.translation()[1], Tba[1])))
    {
      segCoords(t, u, a[1], b[1], A1_dot_B1, tfab.translation()[1], Tba[1]);

      D[0] = tfab.translation()[0] + tfab.linear()(0, 1) * u;
      D[1] = tfab.translation()[1] + tfab.linear()(1, 1) * u - t;
      D[2] = tfab.translation()[2] + tfab.linear()(2, 1) * u;

      if(P && Q)
      {
        *P << 0, t, 0;
        *Q = D + (*P);
      }

      return D.norm();
    }
  }

  S ALL_y, ALU_y, AUL_y, AUU_y;

  ALL_y = -Tba[1];
  ALU_y = ALL_y + aA1_dot_B1;
  AUL_y = ALL_y + aA0_dot_B1;
  AUU_y = ALU_y + aA0_dot_B1;

  S LA1_ly, LA1_uy, UA1_ly, UA1_uy, LB0_lx, LB0_ux, UB0_lx, UB0_ux;

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
                  A1_dot_B0, aA0_dot_B0 - Tba[0], -tfab.translation()[1] - bA1_dot_B1))
       &&
       ((UB0_lx > a[0]) ||
        inVoronoi(a[1], b[0], A0_dot_B0, tfab.translation()[0] - a[0] + bA0_dot_B1,
                  A1_dot_B0, tfab.translation()[1] + bA1_dot_B1, Tba[0] - aA0_dot_B0)))
    {
      segCoords(t, u, a[1], b[0], A1_dot_B0, tfab.translation()[1] + bA1_dot_B1,
                Tba[0] - aA0_dot_B0);

      D[0] = tfab.translation()[0] + tfab.linear()(0, 1) * b[1] + tfab.linear()(0, 0) * u - a[0] ;
      D[1] = tfab.translation()[1] + tfab.linear()(1, 1) * b[1] + tfab.linear()(1, 0) * u - t;
      D[2] = tfab.translation()[2] + tfab.linear()(2, 1) * b[1] + tfab.linear()(2, 0) * u;

      if(P && Q)
      {
        *P << a[0], t, 0;
        *Q = D + (*P);
      }

      return D.norm();
    }
  }

  // UA1, LB0

  if((UA1_ly < 0) && (LB0_ux > a[0]))
  {
    if(((UA1_uy < 0) ||
        inVoronoi(b[0], a[1], -A1_dot_B1, Tba[1] - aA0_dot_B1, A1_dot_B0,
                  aA0_dot_B0 - Tba[0], -tfab.translation()[1]))
       &&
       ((LB0_lx > a[0]) ||
        inVoronoi(a[1], b[0], A0_dot_B0, tfab.translation()[0] - a[0],
                  A1_dot_B0, tfab.translation()[1], Tba[0] - aA0_dot_B0)))
    {
      segCoords(t, u, a[1], b[0], A1_dot_B0, tfab.translation()[1], Tba[0] - aA0_dot_B0);

      D[0] = tfab.translation()[0] + tfab.linear()(0, 0) * u - a[0];
      D[1] = tfab.translation()[1] + tfab.linear()(1, 0) * u - t;
      D[2] = tfab.translation()[2] + tfab.linear()(2, 0) * u;

      if(P && Q)
      {
        *P << a[0], t, 0;
        *Q = D + (*P);
      }

      return D.norm();
    }
  }

  // LA1, UB0

  if((LA1_uy > b[1]) && (UB0_lx < 0))
  {
    if(((LA1_ly > b[1]) ||
        inVoronoi(b[0], a[1], A1_dot_B1, -Tba[1] - b[1],
                  A1_dot_B0, -Tba[0], -tfab.translation()[1] - bA1_dot_B1))
       &&

       ((UB0_ux < 0) ||
        inVoronoi(a[1], b[0], -A0_dot_B0, -tfab.translation()[0] - bA0_dot_B1, A1_dot_B0,
                  tfab.translation()[1] + bA1_dot_B1, Tba[0])))
    {
      segCoords(t, u, a[1], b[0], A1_dot_B0, tfab.translation()[1] + bA1_dot_B1, Tba[0]);

      D[0] = tfab.translation()[0] + tfab.linear()(0, 1) * b[1] + tfab.linear()(0, 0) * u;
      D[1] = tfab.translation()[1] + tfab.linear()(1, 1) * b[1] + tfab.linear()(1, 0) * u - t;
      D[2] = tfab.translation()[2] + tfab.linear()(2, 1) * b[1] + tfab.linear()(2, 0) * u;

      if(P && Q)
      {
        *P << 0, t, 0;
        *Q = D + (*P);
      }


      return D.norm();
    }
  }

  // LA1, LB0

  if((LA1_ly < 0) && (LB0_lx < 0))
  {
    if(((LA1_uy < 0) ||
        inVoronoi(b[0], a[1], -A1_dot_B1, Tba[1], A1_dot_B0,
                  -Tba[0], -tfab.translation()[1]))
       &&
       ((LB0_ux < 0) ||
        inVoronoi(a[1], b[0], -A0_dot_B0, -tfab.translation()[0], A1_dot_B0,
                  tfab.translation()[1], Tba[0])))
    {
      segCoords(t, u, a[1], b[0], A1_dot_B0, tfab.translation()[1], Tba[0]);

      D[0] = tfab.translation()[0] + tfab.linear()(0, 0) * u;
      D[1] = tfab.translation()[1] + tfab.linear()(1, 0) * u - t;
      D[2] = tfab.translation()[2] + tfab.linear()(2, 0) * u;

      if(P&& Q)
      {
        *P << 0, t, 0;
        *Q = D + (*P);
      }

      return D.norm();
    }
  }

  S BLL_y, BLU_y, BUL_y, BUU_y;

  BLL_y = tfab.translation()[1];
  BLU_y = BLL_y + bA1_dot_B1;
  BUL_y = BLL_y + bA1_dot_B0;
  BUU_y = BLU_y + bA1_dot_B0;

  S LA0_lx, LA0_ux, UA0_lx, UA0_ux, LB1_ly, LB1_uy, UB1_ly, UB1_uy;

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
                  A0_dot_B1, aA1_dot_B1 - Tba[1], -tfab.translation()[0] - bA0_dot_B0))
       &&
       ((UB1_ly > a[1]) ||
        inVoronoi(a[0], b[1], A1_dot_B1, tfab.translation()[1] - a[1] + bA1_dot_B0,
                  A0_dot_B1, tfab.translation()[0] + bA0_dot_B0, Tba[1] - aA1_dot_B1)))
    {
      segCoords(t, u, a[0], b[1], A0_dot_B1, tfab.translation()[0] + bA0_dot_B0,
                Tba[1] - aA1_dot_B1);

      D[0] = tfab.translation()[0] + tfab.linear()(0, 0) * b[0] + tfab.linear()(0, 1) * u - t;
      D[1] = tfab.translation()[1] + tfab.linear()(1, 0) * b[0] + tfab.linear()(1, 1) * u - a[1];
      D[2] = tfab.translation()[2] + tfab.linear()(2, 0) * b[0] + tfab.linear()(2, 1) * u;

      if(P && Q)
      {
        *P << t, a[1], 0;
        *Q = D + (*P);
      }

      return D.norm();
    }
  }

  // UA0, LB1

  if((UA0_lx < 0) && (LB1_uy > a[1]))
  {
    if(((UA0_ux < 0) ||
        inVoronoi(b[1], a[0], -A0_dot_B0, Tba[0] - aA1_dot_B0, A0_dot_B1,
                  aA1_dot_B1 - Tba[1], -tfab.translation()[0]))
       &&
       ((LB1_ly > a[1]) ||
        inVoronoi(a[0], b[1], A1_dot_B1, tfab.translation()[1] - a[1], A0_dot_B1, tfab.translation()[0],
                  Tba[1] - aA1_dot_B1)))
    {
      segCoords(t, u, a[0], b[1], A0_dot_B1, tfab.translation()[0], Tba[1] - aA1_dot_B1);

      D[0] = tfab.translation()[0] + tfab.linear()(0, 1) * u - t;
      D[1] = tfab.translation()[1] + tfab.linear()(1, 1) * u - a[1];
      D[2] = tfab.translation()[2] + tfab.linear()(2, 1) * u;

      if(P && Q)
      {
        *P << t, a[1], 0;
        *Q = D + (*P);
      }

      return D.norm();
    }
  }

  // LA0, UB1

  if((LA0_ux > b[0]) && (UB1_ly < 0))
  {
    if(((LA0_lx > b[0]) ||
        inVoronoi(b[1], a[0], A0_dot_B0, -b[0] - Tba[0], A0_dot_B1, -Tba[1],
                  -bA0_dot_B0 - tfab.translation()[0]))
       &&
       ((UB1_uy < 0) ||
        inVoronoi(a[0], b[1], -A1_dot_B1, -tfab.translation()[1] - bA1_dot_B0, A0_dot_B1,
                  tfab.translation()[0] + bA0_dot_B0, Tba[1])))
    {
      segCoords(t, u, a[0], b[1], A0_dot_B1, tfab.translation()[0] + bA0_dot_B0, Tba[1]);

      D[0] = tfab.translation()[0] + tfab.linear()(0, 0) * b[0] + tfab.linear()(0, 1) * u - t;
      D[1] = tfab.translation()[1] + tfab.linear()(1, 0) * b[0] + tfab.linear()(1, 1) * u;
      D[2] = tfab.translation()[2] + tfab.linear()(2, 0) * b[0] + tfab.linear()(2, 1) * u;

      if(P && Q)
      {
        *P << t, 0, 0;
        *Q = D + (*P);
      }

      return D.norm();
    }
  }

  // LA0, LB1

  if((LA0_lx < 0) && (LB1_ly < 0))
  {
    if(((LA0_ux < 0) ||
        inVoronoi(b[1], a[0], -A0_dot_B0, Tba[0], A0_dot_B1, -Tba[1],
                  -tfab.translation()[0]))
       &&
       ((LB1_uy < 0) ||
        inVoronoi(a[0], b[1], -A1_dot_B1, -tfab.translation()[1], A0_dot_B1,
                  tfab.translation()[0], Tba[1])))
    {
      segCoords(t, u, a[0], b[1], A0_dot_B1, tfab.translation()[0], Tba[1]);

      D[0] = tfab.translation()[0] + tfab.linear()(0, 1) * u - t;
      D[1] = tfab.translation()[1] + tfab.linear()(1, 1) * u;
      D[2] = tfab.translation()[2] + tfab.linear()(2, 1) * u;

      if(P && Q)
      {
        *P << t, 0, 0;
        *Q = D + (*P);
      }

      return D.norm();
    }
  }

  S LA0_ly, LA0_uy, UA0_ly, UA0_uy, LB0_ly, LB0_uy, UB0_ly, UB0_uy;

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
                  A0_dot_B0, aA1_dot_B0 - Tba[0], -tfab.translation()[0] - bA0_dot_B1))
       &&
       ((UB0_ly > a[1]) ||
        inVoronoi(a[0], b[0], A1_dot_B0, tfab.translation()[1] - a[1] + bA1_dot_B1, A0_dot_B0,
                  tfab.translation()[0] + bA0_dot_B1, Tba[0] - aA1_dot_B0)))
    {
      segCoords(t, u, a[0], b[0], A0_dot_B0, tfab.translation()[0] + bA0_dot_B1,
                Tba[0] - aA1_dot_B0);

      D[0] = tfab.translation()[0] + tfab.linear()(0, 1) * b[1] + tfab.linear()(0, 0) * u - t;
      D[1] = tfab.translation()[1] + tfab.linear()(1, 1) * b[1] + tfab.linear()(1, 0) * u - a[1];
      D[2] = tfab.translation()[2] + tfab.linear()(2, 1) * b[1] + tfab.linear()(2, 0) * u;

      if(P && Q)
      {
        *P << t, a[1], 0;
        *Q = D + (*P);
      }

      return D.norm();
    }
  }

  // UA0, LB0

  if((UA0_ly < 0) && (LB0_uy > a[1]))
  {
    if(((UA0_uy < 0) ||
        inVoronoi(b[0], a[0], -A0_dot_B1, Tba[1] - aA1_dot_B1, A0_dot_B0,
                  aA1_dot_B0 - Tba[0], -tfab.translation()[0]))
       &&
       ((LB0_ly > a[1]) ||
        inVoronoi(a[0], b[0], A1_dot_B0, tfab.translation()[1] - a[1],
                  A0_dot_B0, tfab.translation()[0], Tba[0] - aA1_dot_B0)))
    {
      segCoords(t, u, a[0], b[0], A0_dot_B0, tfab.translation()[0], Tba[0] - aA1_dot_B0);

      D[0] = tfab.translation()[0] + tfab.linear()(0, 0) * u - t;
      D[1] = tfab.translation()[1] + tfab.linear()(1, 0) * u - a[1];
      D[2] = tfab.translation()[2] + tfab.linear()(2, 0) * u;

      if(P && Q)
      {
        *P << t, a[1], 0;
        *Q = D + (*P);
      }

      return D.norm();
    }
  }

  // LA0, UB0

  if((LA0_uy > b[1]) && (UB0_ly < 0))
  {
    if(((LA0_ly > b[1]) ||
        inVoronoi(b[0], a[0], A0_dot_B1, -Tba[1] - b[1], A0_dot_B0, -Tba[0],
                  -tfab.translation()[0] - bA0_dot_B1))
       &&

       ((UB0_uy < 0) ||
        inVoronoi(a[0], b[0], -A1_dot_B0, -tfab.translation()[1] - bA1_dot_B1, A0_dot_B0,
                  tfab.translation()[0] + bA0_dot_B1, Tba[0])))
    {
      segCoords(t, u, a[0], b[0], A0_dot_B0, tfab.translation()[0] + bA0_dot_B1, Tba[0]);

      D[0] = tfab.translation()[0] + tfab.linear()(0, 1) * b[1] + tfab.linear()(0, 0) * u - t;
      D[1] = tfab.translation()[1] + tfab.linear()(1, 1) * b[1] + tfab.linear()(1, 0) * u;
      D[2] = tfab.translation()[2] + tfab.linear()(2, 1) * b[1] + tfab.linear()(2, 0) * u;

      if(P && Q)
      {
        *P << t, 0, 0;
        *Q = D + (*P);
      }

      return D.norm();
    }
  }

  // LA0, LB0

  if((LA0_ly < 0) && (LB0_ly < 0))
  {
    if(((LA0_uy < 0) ||
        inVoronoi(b[0], a[0], -A0_dot_B1, Tba[1], A0_dot_B0,
                  -Tba[0], -tfab.translation()[0]))
       &&
       ((LB0_uy < 0) ||
        inVoronoi(a[0], b[0], -A1_dot_B0, -tfab.translation()[1], A0_dot_B0,
                  tfab.translation()[0], Tba[0])))
    {
      segCoords(t, u, a[0], b[0], A0_dot_B0, tfab.translation()[0], Tba[0]);

      D[0] = tfab.translation()[0] + tfab.linear()(0, 0) * u - t;
      D[1] = tfab.translation()[1] + tfab.linear()(1, 0) * u;
      D[2] = tfab.translation()[2] + tfab.linear()(2, 0) * u;

      if(P && Q)
      {
        *P << t, 0, 0;
        *Q = D + (*P);
      }

      return D.norm();
    }
  }

  // no edges passed, take max separation along face normals

  S sep1, sep2;

  if(tfab.translation()[2] > 0.0)
  {
    sep1 = tfab.translation()[2];
    if (tfab.linear()(2, 0) < 0.0) sep1 += b[0] * tfab.linear()(2, 0);
    if (tfab.linear()(2, 1) < 0.0) sep1 += b[1] * tfab.linear()(2, 1);
  }
  else
  {
    sep1 = -tfab.translation()[2];
    if (tfab.linear()(2, 0) > 0.0) sep1 -= b[0] * tfab.linear()(2, 0);
    if (tfab.linear()(2, 1) > 0.0) sep1 -= b[1] * tfab.linear()(2, 1);
  }

  if(Tba[2] < 0)
  {
    sep2 = -Tba[2];
    if (tfab.linear()(0, 2) < 0.0) sep2 += a[0] * tfab.linear()(0, 2);
    if (tfab.linear()(1, 2) < 0.0) sep2 += a[1] * tfab.linear()(1, 2);
  }
  else
  {
    sep2 = Tba[2];
    if (tfab.linear()(0, 2) > 0.0) sep2 -= a[0] * tfab.linear()(0, 2);
    if (tfab.linear()(1, 2) > 0.0) sep2 -= a[1] * tfab.linear()(1, 2);
  }

  if(sep1 >= sep2 && sep1 >= 0)
  {
    if(tfab.translation()[2] > 0)
      D << 0, 0, sep1;
    else
      D << 0, 0, -sep1;

    if(P && Q)
    {
      *Q = D;
      P->setZero();
    }
  }

  if(sep2 >= sep1 && sep2 >= 0)
  {
    Vector3<S> Q_(tfab.translation());
    Vector3<S> P_;
    if(Tba[2] < 0)
    {
      P_.noalias() = tfab.linear().col(2) * sep2;
      P_.noalias() += tfab.translation();
    }
    else
    {
      P_.noalias() = tfab.linear().col(2) * -sep2;
      P_.noalias() += tfab.translation();
    }

    D = Q_ - P_;

    if(P && Q)
    {
      *P = P_;
      *Q = Q_;
    }
  }

  S sep = (sep1 > sep2 ? sep1 : sep2);
  return (sep > 0 ? sep : 0);
}

//==============================================================================
template <typename S, typename DerivedA, typename DerivedB>
bool overlap(
    const Eigen::MatrixBase<DerivedA>& R0,
    const Eigen::MatrixBase<DerivedB>& T0,
    const RSS<S>& b1,
    const RSS<S>& b2)
{
  Matrix3<S> R0b2 = R0 * b2.axis;
  Matrix3<S> R = b1.axis.transpose() * R0b2;

  Vector3<S> Ttemp = R0 * b2.To + T0 - b1.To;
  Vector3<S> T = Ttemp.transpose() * b1.axis;

  S dist = rectDistance(R, T, b1.l, b2.l);
  return (dist <= (b1.r + b2.r));
}

//==============================================================================
template <typename S, typename DerivedA, typename DerivedB>
S distance(
    const Eigen::MatrixBase<DerivedA>& R0,
    const Eigen::MatrixBase<DerivedB>& T0,
    const RSS<S>& b1,
    const RSS<S>& b2,
    Vector3<S>* P,
    Vector3<S>* Q)
{
  Matrix3<S> R0b2 = R0 * b2.axis;
  Matrix3<S> R = b1.axis.transpose() * R0b2;

  Vector3<S> Ttemp = R0 * b2.To + T0 - b1.To;
  Vector3<S> T = Ttemp.transpose() * b1.axis;

  S dist = rectDistance(R, T, b1.l, b2.l, P, Q);
  dist -= (b1.r + b2.r);
  return (dist < (S)0.0) ? (S)0.0 : dist;
}

//==============================================================================
template <typename S>
RSS<S> translate(const RSS<S>& bv, const Vector3<S>& t)
{
  RSS<S> res(bv);
  res.To += t;
  return res;
}

} // namespace fcl

#endif

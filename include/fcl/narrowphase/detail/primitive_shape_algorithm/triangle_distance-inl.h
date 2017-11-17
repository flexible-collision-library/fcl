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

#ifndef FCL_NARROWPHASE_DETAIL_TRIANGLEDISTANCE_INL_H
#define FCL_NARROWPHASE_DETAIL_TRIANGLEDISTANCE_INL_H

#include "fcl/narrowphase/detail/primitive_shape_algorithm/triangle_distance.h"

namespace fcl
{

namespace detail
{

//==============================================================================
extern template
class FCL_EXPORT TriangleDistance<double>;

//==============================================================================
template <typename S>
void TriangleDistance<S>::segPoints(const Vector3<S>& P, const Vector3<S>& A, const Vector3<S>& Q, const Vector3<S>& B,
                                 Vector3<S>& VEC, Vector3<S>& X, Vector3<S>& Y)
{
  Vector3<S> T;
  S A_dot_A, B_dot_B, A_dot_B, A_dot_T, B_dot_T;
  Vector3<S> TMP;

  T = Q - P;
  A_dot_A = A.dot(A);
  B_dot_B = B.dot(B);
  A_dot_B = A.dot(B);
  A_dot_T = A.dot(T);
  B_dot_T = B.dot(T);

  // t parameterizes ray P,A
  // u parameterizes ray Q,B

  S t, u;

  // compute t for the closest point on ray P,A to
  // ray Q,B

  S denom = A_dot_A*B_dot_B - A_dot_B*A_dot_B;

  t = (A_dot_T*B_dot_B - B_dot_T*A_dot_B) / denom;

  // clamp result so t is on the segment P,A

  if((t < 0) || std::isnan(t)) t = 0; else if(t > 1) t = 1;

  // find u for point on ray Q,B closest to point at t

  u = (t*A_dot_B - B_dot_T) / B_dot_B;

  // if u is on segment Q,B, t and u correspond to
  // closest points, otherwise, clamp u, recompute and
  // clamp t

  if((u <= 0) || std::isnan(u))
  {
    Y = Q;

    t = A_dot_T / A_dot_A;

    if((t <= 0) || std::isnan(t))
    {
      X = P;
      VEC = Q - P;
    }
    else if(t >= 1)
    {
      X = P + A;
      VEC = Q - X;
    }
    else
    {
      X = P + A * t;
      TMP = T.cross(A);
      VEC = A.cross(TMP);
    }
  }
  else if (u >= 1)
  {
    Y = Q + B;

    t = (A_dot_B + A_dot_T) / A_dot_A;

    if((t <= 0) || std::isnan(t))
    {
      X = P;
      VEC = Y - P;
    }
    else if(t >= 1)
    {
      X = P + A;
      VEC = Y - X;
    }
    else
    {
      X = P + A * t;
      T = Y - P;
      TMP = T.cross(A);
      VEC= A.cross(TMP);
    }
  }
  else
  {
    Y = Q + B * u;

    if((t <= 0) || std::isnan(t))
    {
      X = P;
      TMP = T.cross(B);
      VEC = B.cross(TMP);
    }
    else if(t >= 1)
    {
      X = P + A;
      T = Q - X;
      TMP = T.cross(B);
      VEC = B.cross(TMP);
    }
    else
    {
      X = P + A * t;
      VEC = A.cross(B);
      if(VEC.dot(T) < 0)
      {
        VEC = VEC * (-1);
      }
    }
  }
}

//==============================================================================
template <typename S>
S TriangleDistance<S>::triDistance(const Vector3<S> T1[3], const Vector3<S> T2[3], Vector3<S>& P, Vector3<S>& Q)
{
  // Compute vectors along the 6 sides

  Vector3<S> Sv[3];
  Vector3<S> Tv[3];
  Vector3<S> VEC;

  Sv[0] = T1[1] - T1[0];
  Sv[1] = T1[2] - T1[1];
  Sv[2] = T1[0] - T1[2];

  Tv[0] = T2[1] - T2[0];
  Tv[1] = T2[2] - T2[1];
  Tv[2] = T2[0] - T2[2];

  // For each edge pair, the vector connecting the closest points
  // of the edges defines a slab (parallel planes at head and tail
  // enclose the slab). If we can show that the off-edge vertex of
  // each triangle is outside of the slab, then the closest points
  // of the edges are the closest points for the triangles.
  // Even if these tests fail, it may be helpful to know the closest
  // points found, and whether the triangles were shown disjoint

  Vector3<S> V;
  Vector3<S> Z;
  Vector3<S> minP = Vector3<S>::Zero();
  Vector3<S> minQ = Vector3<S>::Zero();
  S mindd;
  int shown_disjoint = 0;

  mindd = (T1[0] - T2[0]).squaredNorm() + 1; // Set first minimum safely high

  for(int i = 0; i < 3; ++i)
  {
    for(int j = 0; j < 3; ++j)
    {
      // Find closest points on edges i & j, plus the
      // vector (and distance squared) between these points
      segPoints(T1[i], Sv[i], T2[j], Tv[j], VEC, P, Q);

      V = Q - P;
      S dd = V.dot(V);

      // Verify this closest point pair only if the distance
      // squared is less than the minimum found thus far.

      if(dd <= mindd)
      {
        minP = P;
        minQ = Q;
        mindd = dd;

        Z = T1[(i+2)%3] - P;
        S a = Z.dot(VEC);
        Z = T2[(j+2)%3] - Q;
        S b = Z.dot(VEC);

        if((a <= 0) && (b >= 0)) return sqrt(dd);

        S p = V.dot(VEC);

        if(a < 0) a = 0;
        if(b > 0) b = 0;
        if((p - a + b) > 0) shown_disjoint = 1;
      }
    }
  }

  // No edge pairs contained the closest points.
  // either:
  // 1. one of the closest points is a vertex, and the
  //    other point is interior to a face.
  // 2. the triangles are overlapping.
  // 3. an edge of one triangle is parallel to the other's face. If
  //    cases 1 and 2 are not true, then the closest points from the 9
  //    edge pairs checks above can be taken as closest points for the
  //    triangles.
  // 4. possibly, the triangles were degenerate.  When the
  //    triangle points are nearly colinear or coincident, one
  //    of above tests might fail even though the edges tested
  //    contain the closest points.

  // First check for case 1

  Vector3<S> Sn;
  S Snl;

  Sn = Sv[0].cross(Sv[1]); // Compute normal to T1 triangle
  Snl = Sn.dot(Sn);        // Compute square of length of normal

  // If cross product is long enough,

  if(Snl > 1e-15)
  {
    // Get projection lengths of T2 points

    Vector3<S> Tp;

    V = T1[0] - T2[0];
    Tp[0] = V.dot(Sn);

    V = T1[0] - T2[1];
    Tp[1] = V.dot(Sn);

    V = T1[0] - T2[2];
    Tp[2] = V.dot(Sn);

    // If Sn is a separating direction,
    // find point with smallest projection

    int point = -1;
    if((Tp[0] > 0) && (Tp[1] > 0) && (Tp[2] > 0))
    {
      if(Tp[0] < Tp[1]) point = 0; else point = 1;
      if(Tp[2] < Tp[point]) point = 2;
    }
    else if((Tp[0] < 0) && (Tp[1] < 0) && (Tp[2] < 0))
    {
      if(Tp[0] > Tp[1]) point = 0; else point = 1;
      if(Tp[2] > Tp[point]) point = 2;
    }

    // If Sn is a separating direction,

    if(point >= 0)
    {
      shown_disjoint = 1;

      // Test whether the point found, when projected onto the
      // other triangle, lies within the face.

      V = T2[point] - T1[0];
      Z = Sn.cross(Sv[0]);
      if(V.dot(Z) > 0)
      {
        V = T2[point] - T1[1];
        Z = Sn.cross(Sv[1]);
        if(V.dot(Z) > 0)
        {
          V = T2[point] - T1[2];
          Z = Sn.cross(Sv[2]);
          if(V.dot(Z) > 0)
          {
            // T[point] passed the test - it's a closest point for
            // the T2 triangle; the other point is on the face of T1
            P = T2[point] + Sn * (Tp[point] / Snl);
            Q = T2[point];
            return (P - Q).norm();
          }
        }
      }
    }
  }

  Vector3<S> Tn;
  S Tnl;

  Tn = Tv[0].cross(Tv[1]);
  Tnl = Tn.dot(Tn);

  if(Tnl > 1e-15)
  {
    Vector3<S> Sp;

    V = T2[0] - T1[0];
    Sp[0] = V.dot(Tn);

    V = T2[0] - T1[1];
    Sp[1] = V.dot(Tn);

    V = T2[0] - T1[2];
    Sp[2] = V.dot(Tn);

    int point = -1;
    if((Sp[0] > 0) && (Sp[1] > 0) && (Sp[2] > 0))
    {
      if(Sp[0] < Sp[1]) point = 0; else point = 1;
      if(Sp[2] < Sp[point]) point = 2;
    }
    else if((Sp[0] < 0) && (Sp[1] < 0) && (Sp[2] < 0))
    {
      if(Sp[0] > Sp[1]) point = 0; else point = 1;
      if(Sp[2] > Sp[point]) point = 2;
    }

    if(point >= 0)
    {
      shown_disjoint = 1;

      V = T1[point] - T2[0];
      Z = Tn.cross(Tv[0]);
      if(V.dot(Z) > 0)
      {
        V = T1[point] - T2[1];
        Z = Tn.cross(Tv[1]);
        if(V.dot(Z) > 0)
        {
          V = T1[point] - T2[2];
          Z = Tn.cross(Tv[2]);
          if(V.dot(Z) > 0)
          {
            P = T1[point];
            Q = T1[point] + Tn * (Sp[point] / Tnl);
            return (P - Q).norm();
          }
        }
      }
    }
  }

  // Case 1 can't be shown.
  // If one of these tests showed the triangles disjoint,
  // we assume case 3 or 4, otherwise we conclude case 2,
  // that the triangles overlap.

  if(shown_disjoint)
  {
    P = minP;
    Q = minQ;
    return sqrt(mindd);
  }
  else return 0;
}

//==============================================================================
template <typename S>
S TriangleDistance<S>::triDistance(const Vector3<S>& S1, const Vector3<S>& S2, const Vector3<S>& S3,
                                       const Vector3<S>& T1, const Vector3<S>& T2, const Vector3<S>& T3,
                                       Vector3<S>& P, Vector3<S>& Q)
{
  Vector3<S> U[3];
  Vector3<S> T[3];
  U[0] = S1; U[1] = S2; U[2] = S3;
  T[0] = T1; T[1] = T2; T[2] = T3;

  return triDistance(U, T, P, Q);
}

//==============================================================================
template <typename S>
S TriangleDistance<S>::triDistance(const Vector3<S> T1[3], const Vector3<S> T2[3],
                                       const Matrix3<S>& R, const Vector3<S>& Tl,
                                       Vector3<S>& P, Vector3<S>& Q)
{
  Vector3<S> T_transformed[3];
  T_transformed[0] = R * T2[0] + Tl;
  T_transformed[1] = R * T2[1] + Tl;
  T_transformed[2] = R * T2[2] + Tl;

  return triDistance(T1, T_transformed, P, Q);
}

//==============================================================================
template <typename S>
S TriangleDistance<S>::triDistance(const Vector3<S> T1[3], const Vector3<S> T2[3],
                                       const Transform3<S>& tf,
                                       Vector3<S>& P, Vector3<S>& Q)
{
  Vector3<S> T_transformed[3];
  T_transformed[0] = tf * T2[0];
  T_transformed[1] = tf * T2[1];
  T_transformed[2] = tf * T2[2];

  return triDistance(T1, T_transformed, P, Q);
}

//==============================================================================
template <typename S>
S TriangleDistance<S>::triDistance(const Vector3<S>& S1, const Vector3<S>& S2, const Vector3<S>& S3,
                                       const Vector3<S>& T1, const Vector3<S>& T2, const Vector3<S>& T3,
                                       const Matrix3<S>& R, const Vector3<S>& Tl,
                                       Vector3<S>& P, Vector3<S>& Q)
{
  Vector3<S> T1_transformed = R * T1 + Tl;
  Vector3<S> T2_transformed = R * T2 + Tl;
  Vector3<S> T3_transformed = R * T3 + Tl;
  return triDistance(S1, S2, S3, T1_transformed, T2_transformed, T3_transformed, P, Q);
}

//==============================================================================
template <typename S>
S TriangleDistance<S>::triDistance(const Vector3<S>& S1, const Vector3<S>& S2, const Vector3<S>& S3,
                                       const Vector3<S>& T1, const Vector3<S>& T2, const Vector3<S>& T3,
                                       const Transform3<S>& tf,
                                       Vector3<S>& P, Vector3<S>& Q)
{
  Vector3<S> T1_transformed = tf * T1;
  Vector3<S> T2_transformed = tf * T2;
  Vector3<S> T3_transformed = tf * T3;
  return triDistance(S1, S2, S3, T1_transformed, T2_transformed, T3_transformed, P, Q);
}

} // namespace detail
} // namespace fcl

#endif

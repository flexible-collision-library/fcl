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

#ifndef FCL_NARROWPHASE_DETAIL_CAPSULECAPSULE_INL_H
#define FCL_NARROWPHASE_DETAIL_CAPSULECAPSULE_INL_H

#include "fcl/narrowphase/detail/primitive_shape_algorithm/capsule_capsule.h"

namespace fcl
{

namespace detail
{

//==============================================================================
extern template
double clamp(double n, double min, double max);

//==============================================================================
extern template
double closestPtSegmentSegment(
    Vector3d p1, Vector3d q1, Vector3d p2, Vector3d q2,
    double &s, double& t, Vector3d &c1, Vector3d &c2);

//==============================================================================
extern template
bool capsuleCapsuleDistance(
    const Capsule<double>& s1, const Transform3<double>& tf1,
    const Capsule<double>& s2, const Transform3<double>& tf2,
    double* dist, Vector3d* p1_res, Vector3d* p2_res);

//==============================================================================
template <typename S>
S clamp(S n, S min, S max)
{
  if (n < min) return min;
  if (n > max) return max;
  return n;
}

//==============================================================================
template <typename S>
S closestPtSegmentSegment(
    Vector3<S> p1, Vector3<S> q1, Vector3<S> p2, Vector3<S> q2,
    S &s, S &t, Vector3<S> &c1, Vector3<S> &c2)
{
  const S EPSILON = 0.001;
  Vector3<S> d1 = q1 - p1; // Direction vector of segment S1
  Vector3<S> d2 = q2 - p2; // Direction vector of segment S2
  Vector3<S> r = p1 - p2;
  S a = d1.dot(d1); // Squared length of segment S1, always nonnegative

  S e = d2.dot(d2); // Squared length of segment S2, always nonnegative
  S f = d2.dot(r);
  // Check if either or both segments degenerate into points
  if (a <= EPSILON && e <= EPSILON) {
    // Both segments degenerate into points
    s = t = 0.0;
    c1 = p1;
    c2 = p2;
    Vector3<S> diff = c1-c2;
    S res = diff.dot(diff);
    return res;
  }
  if (a <= EPSILON) {
    // First segment degenerates into a point
    s = 0.0;
    t = f / e; // s = 0 => t = (b*s + f) / e = f / e
    t = clamp(t, (S)0.0, (S)1.0);
  } else {
    S c = d1.dot(r);
    if (e <= EPSILON) {
      // Second segment degenerates into a point
      t = 0.0;
      s = clamp(-c / a, (S)0.0, (S)1.0); // t = 0 => s = (b*t - c) / a = -c / a
    } else {
      // The general nondegenerate case starts here
      S b = d1.dot(d2);
      S denom = a*e-b*b; // Always nonnegative
      // If segments not parallel, compute closest point on L1 to L2 and
      // clamp to segment S1. Else pick arbitrary s (here 0)
      if (denom != 0.0) {
        std::cerr << "denominator equals zero, using 0 as reference" << std::endl;
        s = clamp((b*f - c*e) / denom, (S)0.0, (S)1.0);
      } else s = 0.0;
      // Compute point on L2 closest to S1(s) using
      // t = Dot((P1 + D1*s) - P2,D2) / Dot(D2,D2) = (b*s + f) / e
      t = (b*s + f) / e;

      //
      //If t in [0,1] done. Else clamp t, recompute s for the new value
      //of t using s = Dot((P2 + D2*t) - P1,D1) / Dot(D1,D1)= (t*b - c) / a
      //and clamp s to [0, 1]
      if(t < 0.0) {
        t = 0.0;
        s = clamp(-c / a, (S)0.0, (S)1.0);
      } else if (t > 1.0) {
        t = 1.0;
        s = clamp((b - c) / a, (S)0.0, (S)1.0);
      }
    }
  }
  c1 = p1 + d1 * s;
  c2 = p2 + d2 * t;
  Vector3<S> diff = c1-c2;
  S res = diff.dot(diff);
  return res;
}

//==============================================================================
template <typename S>
bool capsuleCapsuleDistance(const Capsule<S>& s1, const Transform3<S>& tf1,
          const Capsule<S>& s2, const Transform3<S>& tf2,
          S* dist, Vector3<S>* p1_res, Vector3<S>* p2_res)
{

  Vector3<S> p1(tf1.translation());
  Vector3<S> p2(tf2.translation());

  // line segment composes two points. First point is given by the origin, second point is computed by the origin transformed along z.
  // extension along z-axis means transformation with identity matrix and translation vector z pos
  Transform3<S> transformQ1 = tf1 * Translation3<S>(Vector3<S>(0,0,s1.lz));
  Vector3<S> q1 = transformQ1.translation();

  Transform3<S> transformQ2 = tf2 * Translation3<S>(Vector3<S>(0,0,s2.lz));
  Vector3<S> q2 = transformQ2.translation();

  // s and t correspont to the length of the line segment
  S s, t;
  Vector3<S> c1, c2;

  S result = closestPtSegmentSegment(p1, q1, p2, q2, s, t, c1, c2);
  *dist = sqrt(result)-s1.radius-s2.radius;

  // getting directional unit vector
  Vector3<S> distVec = c2 -c1;
  distVec.normalize();

  // extend the point to be border of the capsule.
  // Done by following the directional unit vector for the length of the capsule radius
  *p1_res = c1 + distVec*s1.radius;

  distVec = c1-c2;
  distVec.normalize();

  *p2_res = c2 + distVec*s2.radius;

  return true;
}

} // namespace detail
} // namespace fcl

#endif

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
extern template double closestPtSegmentSegment(const Vector3d& p_FP1,
                                               const Vector3d& p_FQ1,
                                               const Vector3d& p_FP2,
                                               const Vector3d& p_FQ2, double* s,
                                               double* t, Vector3d* p_FC1,
                                               Vector3d* p_FC2);

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
S closestPtSegmentSegment(const Vector3<S>& p_FP1, const Vector3<S>& p_FQ1,
                          const Vector3<S>& p_FP2, const Vector3<S>& p_FQ2,
                          S* s, S* t, Vector3<S>* p_FC1, Vector3<S>* p_FC2) {
  // TODO(SeanCurtis-TRI): Document the match underlying this function -- the
  //  variables: a, b, c, e, and f are otherwise overly inscrutable.
  const auto kEps = constants<S>::eps_78();
  const auto kEpsSquared = kEps * kEps;

  Vector3<S> p_P1Q1 = p_FQ1 - p_FP1;  // Segment 1's displacement vector: D1.
  Vector3<S> p_P2Q2 = p_FQ2 - p_FP2;  // Segment 2's displacement vector: D2.
  Vector3<S> p_P2P1 = p_FP1 - p_FP2;

  S a = p_P1Q1.dot(p_P1Q1); // Squared length of segment S1, always nonnegative.
  S e = p_P2Q2.dot(p_P2Q2); // Squared length of segment S2, always nonnegative.
  S f = p_P2Q2.dot(p_P2P1);

  // Check if either or both segments degenerate into points.
  if (a <= kEpsSquared && e <= kEpsSquared) {
    // Both segments degenerate into points.
    *s = *t = 0.0;
    *p_FC1 = p_FP1;
    *p_FC2 = p_FP2;
    return (*p_FC1 - *p_FC2).squaredNorm();
  }
  if (a <= kEpsSquared) {
    // First segment degenerates into a point.
    *s = 0.0;
    // t = (b * s + f) / e, s = 0 --> t = f / e.
    *t = clamp(f / e, (S)0.0, (S)1.0);
  } else {
    const S c = p_P1Q1.dot(p_P2P1);
    if (e <= kEpsSquared) {
      // Second segment degenerates into a point.
      *t = 0.0;
      // s = (b*t - c) / a, t = 0 --> s = -c / a.
      *s = clamp(-c / a, (S)0.0, (S)1.0);
    } else {
      // The general non-degenerate case starts here.
      const S b = p_P1Q1.dot(p_P2Q2);
      // Mathematically, ae - b² ≥ 0, but we need to protect ourselves from
      // possible rounding error near zero that _might_ produce -epsilon.
      using std::max;
      const S denom = max(S(0), a*e-b*b);

      // If segments are not parallel, compute closest point on L1 to L2 and
      // clamp to segment S1. Else pick arbitrary s (here 0).
      if (denom > kEpsSquared) {
        *s = clamp((b*f - c*e) / denom, (S)0.0, (S)1.0);
      } else {
        *s = 0.0;
      }
      // Compute point on L2 closest to S1(s) using
      // t = Dot((P1 + D1*s) - P2, D2) / Dot(D2,D2) = (b*s + f) / e
      *t = (b*(*s) + f) / e;

      // If t in [0,1] done. Else clamp t, recompute s for the new value
      // of t using s = Dot((P2 + D2*t) - P1, D1) / Dot(D1,D1) = (t*b - c) / a
      // and clamp s to [0, 1].
      if(*t < 0.0) {
        *t = 0.0;
        *s = clamp(-c / a, (S)0.0, (S)1.0);
      } else if (*t > 1.0) {
        *t = 1.0;
        *s = clamp((b - c) / a, (S)0.0, (S)1.0);
      }
    }
  }
  *p_FC1 = p_FP1 + p_P1Q1 * (*s);
  *p_FC2 = p_FP2 + p_P2Q2 * (*t);
  return (*p_FC1 - *p_FC2).squaredNorm();
}

// Given a transform relating frames A and B, returns Bz_A, the z-axis of frame
// B, expressed in frame A.
template <typename S>
Vector3<S> z_axis(const Transform3<S>& X_AB) {
  return X_AB.matrix().template block<3, 1>(0, 2);
}

//==============================================================================
template <typename S>
bool capsuleCapsuleDistance(const Capsule<S>& s1, const Transform3<S>& X_FC1,
          const Capsule<S>& s2, const Transform3<S>& X_FC2,
          S* dist, Vector3<S>* p_FW1, Vector3<S>* p_FW2)
{
  assert(dist != nullptr);
  assert(p_FW1 != nullptr);
  assert(p_FW2 != nullptr);

  // Origin of the capsules' frames relative to F's origin.
  const Vector3<S> p_FC1o = X_FC1.translation();
  const Vector3<S> p_FC2o = X_FC2.translation();

  // A capsule is defined centered on the origin of its canonical frame C
  // and with the central line segment aligned with Cz. So, the two end points
  // of the capsule's center line segment are at `z+ = lz / 2 * Cz` and
  // `z- = -lz / 2 * Cz`, respectively. Cz_F is simply the third column of the
  // rotation matrix, R_FC. This "half arm" is the position vector from the
  // canonical frame's origin to the z+ point: p_CoZ+_F in frame F.
  auto calc_half_arm = [](const Capsule<S>& c,
                          const Transform3<S>& X_FC) -> Vector3<S> {
    const S half_length = c.lz / 2;
    const Vector3<S> Cz_F = z_axis(X_FC);
    return half_length * Cz_F;
  };

  const Vector3<S> half_arm_1_F = calc_half_arm(s1, X_FC1);
  const Vector3<S> p_FC1a = p_FC1o + half_arm_1_F;
  const Vector3<S> p_FC1b = p_FC1o - half_arm_1_F;

  const Vector3<S> half_arm_2_F = calc_half_arm(s2, X_FC2);
  const Vector3<S> p_FC2a = p_FC2o + half_arm_2_F;
  const Vector3<S> p_FC2b = p_FC2o - half_arm_2_F;

  // s and t correspond to the length of each line segment; should be s1.lz and
  // s2.lz, respectively.
  S s, t;
  // The points on the line segments closest to each other.
  Vector3<S> p_FN1, p_FN2;
  // TODO(SeanCurtis-TRI): This query isn't well tailored for this problem.
  //  By construction, we know the unit-length vectors for the two segments (and
  //  their lengths), but closestPtSegmentSegment() infers the segment direction
  //  from the end point. Furthermore, it returns the values for s and t,
  //  neither of which is required by this function. The API should be
  //  streamlined so there is less waste.
  const S squared_dist = closestPtSegmentSegment(p_FC1a, p_FC1b, p_FC2a, p_FC2b,
                                                 &s, &t, &p_FN1, &p_FN2);

  const S segment_dist = sqrt(squared_dist);
  *dist = segment_dist - s1.radius - s2.radius;
  Vector3<S> vhat_C1C2_F;
  const auto eps = constants<S>::eps_78();
  // We can only use the vector between the center-line nearest points to find
  // the witness points if they are sufficiently far apart.
  if (segment_dist > eps) {
    vhat_C1C2_F = (p_FN2 - p_FN1) / segment_dist;
  } else {
    // The points are too close. The center lines intersect. We have two cases:
    //   1. They intersect at a single point (non-parallel center lines).
    //      - The center lines span a plane and the witness points should lie
    //        above and below that plane the corresponding radius amount.
    //   2. They intersect at multiple points (parallel, overlapping center
    //      lies).
    //      - Any direction on the plane perpendicular to the common center line
    //        will suffice. We arbitrarily pick the Cx direction.
    const Vector3<S>& C1z_F = z_axis(X_FC1);
    const Vector3<S>& C2z_F = z_axis(X_FC2);
    using std::abs;
    if (abs(C1z_F.dot(C2z_F)) < 1 - eps) {
      // Vectors are sufficiently perpendicular to use cross product.
      vhat_C1C2_F = C1z_F.cross(C2z_F).normalized();
    } else {
      // Vectors are parallel, simply use Cx_F as the vector.
      vhat_C1C2_F = X_FC1.matrix().template block<3, 1>(0, 0);
    }
  }
  *p_FW1 = p_FN1 + vhat_C1C2_F * s1.radius;
  *p_FW2 = p_FN2 - vhat_C1C2_F * s2.radius;

  return true;
}

} // namespace detail
} // namespace fcl

#endif

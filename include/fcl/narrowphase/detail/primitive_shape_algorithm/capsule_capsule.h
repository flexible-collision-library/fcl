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

#ifndef FCL_NARROWPHASE_DETAIL_CAPSULECAPSULE_H
#define FCL_NARROWPHASE_DETAIL_CAPSULECAPSULE_H

#include "fcl/common/types.h"
#include "fcl/geometry/shape/capsule.h"

namespace fcl
{

namespace detail
{

// Clamp n to lie within the range [min, max]
template <typename S>
FCL_EXPORT
S clamp(S n, S min, S max);

// Computes closest points C1 and C2 of S1(s)=P1+s*(Q1-P1) and
// S2(t)=P2+t*(Q2-P2), returning s and t. Function result is squared
// distance between between S1(s) and S2(t)
template <typename S>
FCL_EXPORT
S closestPtSegmentSegment(
    Vector3<S> p1, Vector3<S> q1, Vector3<S> p2, Vector3<S> q2,
    S &s, S &t, Vector3<S> &c1, Vector3<S> &c2);

/** Computes the signed distance between two capsules `s1` and `s2` (with
 given poses `tf1` and `tf2` of the two capsules in a common frame `F`). Further
 reports the witness points to that distance (`p1_res` and `p2_res`).

 It is guaranteed that `|p_FW1 - p_FW2| = |dist|`.

 There are degenerate cases where there is no _unique_ pair of witness points.
 If the center lines of the two capsules intersect (either once or multiple
 times when the are co-linear) then distance will still be reported (a
 negative value with the maximum magnitude possible between the two capsules)
 and an arbitrary pair of witness points from the set of valid pairs.
 @param s1      The first capsule.
 @param X_FC1   The pose of the first capsule in a common frame `F`.
 @param s2      The second capsule.
 @param X_FC2   The pose of the second capsule in a common frame `F`.
 @param dist    The signed distance between the two capsules (negative indicates
                intersection.
 @param p_FW1   The first witness point: a point on the surface of the first
                capsule measured and expressed in the common frame `F`.
 @param p_FW2   The second witness point: a point on the surface of the second
                capsule measured and expressed in the common frame `F`.
 @return `true`.
 @tparam S  The scalar type for computation.
 */
template <typename S>
FCL_EXPORT
bool capsuleCapsuleDistance(const Capsule<S>& s1, const Transform3<S>& X_FC1,
          const Capsule<S>& s2, const Transform3<S>& X_FC2,
          S* dist, Vector3<S>* p_FW1, Vector3<S>* p_FW2);

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/primitive_shape_algorithm/capsule_capsule-inl.h"

#endif

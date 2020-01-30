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

/** Computes the pair of closest points `(p_FC1, p_FC2)` between two line
 segments given by point pairs `(p_FP1, p_FQ1)` and (p_FP2, p_FQ2)`. If the
 lines are parallel, there may be no _unique_ such point pair, in that case it
 computes one from the set of nearest pairs. As a by-product, it reports the
 squared distance between those points and the parameters (`s` and `t`) such
 that `p_FC1 = p_FP1 + s * (p_FQ1 - p_FP1)` for segment one, and similarly for
 `p_FC2` and `t`, with `0 ≤ s,t ≤ 1`. All points are measured and expressed in a
 common frame `F`.

 @param p_FP1   Segment 1's first point, P1.
 @param p_FQ1   Segment 1's second point, Q1.
 @param p_FP2   Segment 2's first point, P2.
 @param p_FQ2   Segment 2's second point, Q2.
 @param s       The parameter relating C1 with P1 and Q1.
 @param t       The parameter relating C2 with P2 and Q2.
 @param p_FC1   The point C1 on segment 1, nearest segment 2.
 @param p_FC2   The point C2 on segment 2, nearest segment 1.
 @return    The squared distance between points C1 and C2.
 @tparam S  The scalar type for computation.
 */
template <typename S>
FCL_EXPORT S closestPtSegmentSegment(const Vector3<S>& p_FP1,
                                     const Vector3<S>& p_FQ1,
                                     const Vector3<S>& p_FP2,
                                     const Vector3<S>& p_FQ2, S* s, S* t,
                                     Vector3<S>* p_FC1, Vector3<S>* p_FC2);

/** Computes the signed distance between two capsules `s1` and `s2` (with
 given poses `X_FC1` and `X_FC2` of the two capsules in a common frame `F`).
 Further reports the witness points to that distance (`p_FW1` and `p_FW2`).

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
                intersection).
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

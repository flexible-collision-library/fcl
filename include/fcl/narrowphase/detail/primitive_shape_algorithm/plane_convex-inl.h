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

/** @author Mike Danielczuk */

#ifndef FCL_NARROWPHASE_DETAIL_PLANECONVEX_INL_H
#define FCL_NARROWPHASE_DETAIL_PLANECONVEX_INL_H

#include "fcl/narrowphase/detail/primitive_shape_algorithm/plane_convex.h"

namespace fcl {

namespace detail {

//==============================================================================
extern template FCL_EXPORT bool planeConvexIntersect(
    const Plane<double>& s1, const Transform3<double>& tf1,
    const Convex<double>& s2, const Transform3<double>& tf2,
    std::vector<ContactPoint<double>>* contacts);

//==============================================================================
extern template FCL_EXPORT bool planeConvexDistance(
    const Plane<double>& s1, const Transform3<double>& tf1,
    const Convex<double>& s2, const Transform3<double>& tf2, double* dist,
    Vector3<double>* p1, Vector3<double>* p2);

//==============================================================================
extern template FCL_EXPORT bool planeConvexSignedDistance(
    const Plane<double>& s1, const Transform3<double>& tf1,
    const Convex<double>& s2, const Transform3<double>& tf2, double* dist,
    Vector3<double>* p1, Vector3<double>* p2);

//==============================================================================
template <typename S>
bool planeConvexIntersect(const Plane<S>& plane_P, const Transform3<S>& X_FP,
                          const Convex<S>& convex_C, const Transform3<S>& X_FC,
                          std::vector<ContactPoint<S>>* contacts) {
  Plane<S> plane_C = transform(plane_P, X_FC.inverse() * X_FP);

  Vector3<S> p_CV_deepest;
  S min_signed_distance = std::numeric_limits<S>::max();

  // TODO: Once we have an efficient "support vector" implementation for Convex
  //  (necessary to make GJK run faster with convex), this could benefit by
  //  simply asking for the support vector in the negative normal direction.
  //  That would also make computing normal_C cheaper; it could just be the
  //  product: X_FC.linear().transpose() * X_FP.linear() * plane_P.n.
  for (const auto& p_CV : convex_C.getVertices()) {
    const S signed_distance = plane_C.signedDistance(p_CV);
    if (signed_distance < min_signed_distance) {
      min_signed_distance = signed_distance;
      p_CV_deepest = p_CV;
      if (signed_distance <= 0 && contacts == nullptr) return true;
    }
  }

  const bool intersecting = min_signed_distance <= 0;

  if (intersecting && contacts) {
    const Vector3<S> normal_F = X_FP.linear() * plane_P.n;
    const Vector3<S> p_FV = X_FC * p_CV_deepest;
    // NOTE: penetration depth is defined as the negative of signed distance.
    // So, the depth reported here will always be non-negative.
    const S depth = -min_signed_distance;
    contacts->emplace_back(-normal_F, p_FV + normal_F * (0.5 * depth), depth);
  }

  return intersecting;
}

//==============================================================================
template <typename S>
bool planeConvexDistance(const Plane<S>& plane_P, const Transform3<S>& X_FP,
                         const Convex<S>& convex_C, const Transform3<S>& X_FC,
                         S* dist, Vector3<S>* p1, Vector3<S>* p2) {
  Plane<S> plane_C = transform(plane_P, X_FC.inverse() * X_FP);

  Vector3<S> p_CV_deepest;
  S min_signed_distance = std::numeric_limits<S>::max();

  // TODO: Once we have an efficient "support vector" implementation for Convex
  //  (necessary to make GJK run faster with convex), this could benefit by
  //  simply asking for the support vector in the negative normal direction.
  //  That would also make computing normal_C cheaper; it could just be the
  //  product: X_FC.linear().transpose() * X_FP.linear() * plane_P.n.
  for (const auto& p_CV : convex_C.getVertices()) {
    const S signed_distance = plane_C.signedDistance(p_CV);
    if (signed_distance < min_signed_distance) {
      min_signed_distance = signed_distance;
      p_CV_deepest = p_CV;
      if (signed_distance <= 0) {
        *dist = -1;
        return false;
      }
    }
  }

  if (dist) *dist = min_signed_distance;
  const Vector3<S> normal_F = X_FP.linear() * plane_P.n;
  const Vector3<S> p_FV = X_FC * p_CV_deepest;
  if (p1) *p1 = p_FV - min_signed_distance * normal_F;
  if (p2) *p2 = p_FV;
  return true;
}

//==============================================================================
template <typename S>
bool planeConvexSignedDistance(const Plane<S>& plane_P,
                               const Transform3<S>& X_FP,
                               const Convex<S>& convex_C,
                               const Transform3<S>& X_FC, S* dist,
                               Vector3<S>* p1, Vector3<S>* p2) {
  Plane<S> plane_C = transform(plane_P, X_FC.inverse() * X_FP);

  Vector3<S> p_CV_deepest;
  S min_signed_distance = std::numeric_limits<S>::max();

  // TODO: Once we have an efficient "support vector" implementation for Convex
  //  (necessary to make GJK run faster with convex), this could benefit by
  //  simply asking for the support vector in the negative normal direction.
  //  That would also make computing normal_C cheaper; it could just be the
  //  product: X_FC.linear().transpose() * X_FP.linear() * plane_P.n.
  for (const auto& p_CV : convex_C.getVertices()) {
    const S signed_distance = plane_C.signedDistance(p_CV);
    if (signed_distance < min_signed_distance) {
      min_signed_distance = signed_distance;
      p_CV_deepest = p_CV;
    }
  }

  if (dist) *dist = min_signed_distance;
  const Vector3<S> normal_F = X_FP.linear() * plane_P.n;
  const Vector3<S> p_FV = X_FC * p_CV_deepest;
  if (p1) *p1 = p_FV - min_signed_distance * normal_F;
  if (p2) *p2 = p_FV;
  return true;
}

}  // namespace detail
}  // namespace fcl

#endif

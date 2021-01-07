/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Toyota Research Institute.
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

/** @author Sean Curtis <sean@tri.global> (2018) */

#ifndef FCL_NARROWPHASE_DETAIL_SPHEREBOX_INL_H
#define FCL_NARROWPHASE_DETAIL_SPHEREBOX_INL_H

#include "fcl/narrowphase/detail/primitive_shape_algorithm/sphere_box.h"

namespace fcl {
namespace detail {

extern template FCL_EXPORT bool
sphereBoxIntersect(const Sphere<double>& sphere, const Transform3<double>& X_FS,
                   const Box<double>& box, const Transform3<double>& X_FB,
                   std::vector<ContactPoint<double>>* contacts);

//==============================================================================

extern template FCL_EXPORT bool
sphereBoxDistance(const Sphere<double>& sphere, const Transform3<double>& X_FS,
                  const Box<double>& box, const Transform3<double>& X_FB,
                  double* distance, Vector3<double>* p_FSb,
                  Vector3<double>* p_FBs);

//==============================================================================

// Helper function for box-sphere queries. Given a box defined in its canonical
// frame B (i.e., aligned to the axes and centered on the origin) and a query
// point Q, determines point N, the point *inside* the box nearest to Q. Note:
// this is *not* the nearest point on the surface of the box; if Q is inside
// the box, then the nearest point is Q itself.
// @param size            The size of the box to query against.
// @param p_BQ            The position vector from frame B's origin to the query
//                        point Q measured and expressed in frame B.
// @param[out] p_BN_ptr   A position vector from frame B's origin to the point N
//                        measured and expressed in frame B.
// @returns true if the nearest point is a different point than the query point.
// @pre P_BN_ptr must point to a valid Vector3<S> instance.
template <typename S>
bool nearestPointInBox(const Vector3<S>& size, const Vector3<S>& p_BQ,
                       Vector3<S>* p_BN_ptr) {
  assert(p_BN_ptr != nullptr);
  Vector3<S>& p_BN = *p_BN_ptr;
  // Clamp the point to the box. If we do *any* clamping we know the center was
  // outside. If we did *no* clamping, the point is inside the box.
  const Vector3<S> half_size = size / 2;
  // The nearest point on the box (N) to Q measured in frame B.
  bool clamped = false;
  for (int i = 0; i < 3; ++i) {
    p_BN(i) = p_BQ(i);
    if (p_BQ(i) < -half_size(i)) {
      clamped = true;
      p_BN(i) = -half_size(i);
    }
    if (p_BQ(i) > half_size(i)) {
      clamped = true;
      p_BN(i) = half_size(i);
    }
  }
  return clamped;
}
//==============================================================================

template <typename S>
FCL_EXPORT bool sphereBoxIntersect(const Sphere<S>& sphere,
                                   const Transform3<S>& X_FS, const Box<S>& box,
                                   const Transform3<S>& X_FB,
                                   std::vector<ContactPoint<S>>* contacts) {
  const S r = sphere.radius;
  // Find the sphere center C in the box's frame.
  const Transform3<S> X_BS = X_FB.inverse() * X_FS;
  const Vector3<S> p_BC = X_BS.translation();

  // Find N, the nearest point *inside* the box to the sphere center C (measured
  // and expressed in frame B)
  Vector3<S> p_BN;
  bool N_is_not_C = nearestPointInBox(box.side, p_BC, &p_BN);

  // Compute the position vector from the nearest point N to the sphere center
  // C in the common box frame B. If the center is inside the box, this will be
  // the zero vector.
  Vector3<S> p_CN_B = p_BN - p_BC;
  S squared_distance = p_CN_B.squaredNorm();
  // The nearest point to the sphere is *farther* than radius, they are *not*
  // penetrating.
  if (squared_distance > r * r)
    return false;

  // Now we know they are colliding.

  if (contacts != nullptr) {
    // Return values have been requested.
    S depth{0};
    Vector3<S> n_SB_B; // Normal pointing from sphere into box (in box's frame)
    Vector3<S> p_BP;   // Contact position (P) in the box frame.
    // We want to make sure that differences exceed machine precision -- we
    // don't want normal and contact position dominated by noise. However,
    // because we apply an arbitrary rigid transform to the sphere's center, we
    // lose bits of precision. For an arbitrary non-identity transform, 4 bits
    // is the maximum possible precision loss. So, we only consider a point to
    // be outside the box if it's distance is at least that epsilon.
    // Furthermore, in finding the *near* face, a better candidate must be more
    // than this epsilon closer to the sphere center (see the test in the
    // else branch).
    constexpr auto eps = 16 * constants<S>::eps();
    if (N_is_not_C && squared_distance > eps * eps) {
      // The center is on the outside. Normal direction is from C to N (computed
      // above) and penetration depth is r - |p_BN - p_BC|. The contact position
      // is 1/2 the penetration distance in the normal direction from p_BN.
      S distance = sqrt(squared_distance);
      n_SB_B = p_CN_B / distance;
      depth = r - distance;
      p_BP = p_BN + n_SB_B * (depth * 0.5);
    } else {
      // The center is inside. The sphere center projects onto all faces. The
      // face that is closest defines the normal direction. The penetration
      // depth is the distance to that face + radius. The position is the point
      // midway between the projection point, and the point opposite the sphere
      // center in the *negative* normal direction.
      Vector3<S> half_size = box.side / 2;
      S min_distance =
          std::numeric_limits<typename constants<S>::Real>::infinity();
      int min_axis = -1;
      for (int i = 0; i < 3; ++i) {
        S dist = p_BC(i) >= 0 ? half_size(i) - p_BC(i) : p_BC(i) + half_size(i);
        // To be closer, the face has to be more than epsilon closer.
        if (dist + eps < min_distance) {
          min_distance = dist;
          min_axis = i;
        }
      }
      n_SB_B << 0, 0, 0;
      // NOTE: This sign *may* seem counter-intuitive. A center nearest the +z
      // face produces a normal in the -z direction; this is because the normal
      // points from the sphere and into the box; and the penetration is *into*
      // the +z face (so points in the -z direction). The same logic applies to
      // all other directions.
      n_SB_B(min_axis) = p_BC(min_axis) >= 0 ? -1 : 1;
      depth = min_distance + r;
      p_BP = p_BC + n_SB_B * ((r - min_distance) / 2);
    }
    contacts->emplace_back(X_FB.linear() * n_SB_B, X_FB * p_BP, depth);
  }
  return true;
}

//==============================================================================

template <typename S>
FCL_EXPORT bool sphereBoxDistance(const Sphere<S>& sphere,
                                  const Transform3<S>& X_FS, const Box<S>& box,
                                  const Transform3<S>& X_FB, S* distance,
                                  Vector3<S>* p_FSb, Vector3<S>* p_FBs) {
  // Find the sphere center C in the box's frame.
  const Transform3<S> X_BS = X_FB.inverse() * X_FS;
  const Vector3<S> p_BC = X_BS.translation();
  const S r = sphere.radius;

  // Find N, the nearest point *inside* the box to the sphere center C (measured
  // and expressed in frame B)
  Vector3<S> p_BN;
  bool N_is_not_C = nearestPointInBox(box.side, p_BC, &p_BN);

  if (N_is_not_C) {
    // If N is not C, we know the sphere center is *outside* the box (but we
    // don't know yet if the they are completely separated).

    // Compute the position vector from the nearest point N to the sphere center
    // C in the frame B.
    Vector3<S> p_NC_B = p_BC - p_BN;
    S squared_distance = p_NC_B.squaredNorm();
    if (squared_distance > r * r) {
      // The distance to the nearest point is greater than the radius, we have
      // proven separation.
      S d{-1};
      if (distance || p_FBs || p_FSb)
        d = sqrt(squared_distance);
      if (distance != nullptr)
        *distance = d - r;
      if (p_FBs != nullptr)
        *p_FBs = X_FB * p_BN;
      if (p_FSb != nullptr) {
        const Vector3<S> p_BSb = (p_NC_B / d) * (d - r) + p_BN;
        *p_FSb = X_FB * p_BSb;
      }
      return true;
    }
  }

  // We didn't *prove* separation, so we must be in penetration.
  if (distance != nullptr) *distance = -1;
  return false;
}

} // namespace detail
} // namespace fcl

#endif // FCL_NARROWPHASE_DETAIL_SPHEREBOX_INL_H

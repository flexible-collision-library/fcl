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

#ifndef FCL_NARROWPHASE_DETAIL_SPHERECYLINDER_INL_H
#define FCL_NARROWPHASE_DETAIL_SPHERECYLINDER_INL_H

#include "fcl/narrowphase/detail/primitive_shape_algorithm/sphere_cylinder.h"

namespace fcl {
namespace detail {

extern template FCL_EXPORT bool
sphereCylinderIntersect(const Sphere<double>& sphere,
                        const Transform3<double>& X_FS,
                        const Cylinder<double>& cylinder,
                        const Transform3<double>& X_FC,
                        std::vector<ContactPoint<double>>* contacts);

//==============================================================================

extern template FCL_EXPORT bool
sphereCylinderDistance(const Sphere<double>& sphere,
                       const Transform3<double>& X_FS,
                       const Cylinder<double>& cylinder,
                       const Transform3<double>& X_FC,
                       double* distance, Vector3<double>* p_FSc,
                       Vector3<double>* p_FCs);

//==============================================================================

// Helper function for cylinder-sphere queries. Given a cylinder defined in its
// canonical frame C (i.e., centered on the origin with cylinder's height axis
// aligned to the Cz axis) and a query point Q, determines point N, the point
// *inside* the cylinder nearest to Q. Note: this is *not* necessarily the
// nearest point on the surface of the cylinder; if Q is inside the cylinder,
// then the nearest point is Q itself.
// @param height          The height of the cylinder.
// @param radius          The radius of the cylinder.
// @param p_CQ            The position vector from frame C's origin to the query
//                        point Q measured and expressed in frame C.
// @param[out] p_CN_ptr   A position vector from frame C's origin to the point N
//                        measured and expressed in frame C.
// @returns true if the nearest point is a different point than the query point.
// @pre p_CN_ptr must point to a valid Vector3<S> instance.
template <typename S>
bool nearestPointInCylinder(const S& height, const S& radius,
                            const Vector3<S>& p_CQ, Vector3<S>* p_CN_ptr) {
  assert(p_CN_ptr != nullptr);
  Vector3<S>& p_CN = *p_CN_ptr;
  p_CN = p_CQ;

  bool clamped = false;

  // Linearly clamp along the cylinders height axis.
  const S half_height = height / 2;
  if (p_CQ(2) > half_height) {
    clamped = true;
    p_CN(2) = half_height;
  } else if (p_CQ(2) < -half_height) {
    clamped = true;
    p_CN(2) = -half_height;
  }

  // Clamp according to the circular cross section.
  Vector2<S> r_CQ{p_CQ(0), p_CQ(1)};
  S squared_distance = r_CQ.dot(r_CQ);
  if (squared_distance > radius * radius) {
    // The query point lies outside the *circular* extent of the cylinder.
    clamped = true;
    r_CQ *= radius / sqrt(squared_distance);
    p_CN(0) = r_CQ(0);
    p_CN(1) = r_CQ(1);
  }

  return clamped;
}

//==============================================================================

template <typename S>
FCL_EXPORT bool sphereCylinderIntersect(
    const Sphere<S>& sphere, const Transform3<S>& X_FS,
    const Cylinder<S>& cylinder, const Transform3<S>& X_FC,
    std::vector<ContactPoint<S>>* contacts) {
  const S& r_s = sphere.radius;
  // Find the sphere center So (abbreviated as S) in the cylinder's frame.
  const Transform3<S> X_CS = X_FC.inverse() * X_FS;
  const Vector3<S> p_CS = X_CS.translation();

  // Find N, the nearest point *inside* the cylinder to the sphere center S
  // (measure and expressed in frame C).
  Vector3<S> p_CN;
  bool S_is_outside = nearestPointInCylinder(cylinder.lz, cylinder.radius, p_CS,
                                             &p_CN);

  // Compute the position vector from the sphere center S to the nearest point N
  // in the cylinder frame C. If the center is inside the cylinder, this will
  // be the zero vector.
  const Vector3<S> p_SN_C = p_CN - p_CS;
  const S p_SN_squared_dist = p_SN_C.squaredNorm();
  // The nearest point to the sphere is *farther* than radius; they are *not*
  // penetrating.
  if (p_SN_squared_dist > r_s * r_s)
    return false;

  // Now we know they are colliding.

  if (contacts != nullptr) {
    // Return values have been requested.
    S depth{0};
    // Normal pointing from sphere into cylinder (in cylinder's frame)
    Vector3<S> n_SC_C;
    // Contact position (P) in the cylinder frame.
    Vector3<S> p_CP;

    // We want to make sure that differences exceed machine precision -- we
    // don't want normal and contact position dominated by noise. However,
    // because we apply an arbitrary rigid transform to the sphere's center, we
    // lose bits of precision. For an arbitrary non-identity transform, 4 bits
    // is the maximum possible precision loss. So, we only consider a point to
    // be outside the cylinder if it's distance is at least that epsilon.
    // Furthermore, in finding the *near* face, a better candidate must be more
    // than this epsilon closer to the sphere center (see the test in the
    // else branch).
    constexpr auto eps = 16 * constants<S>::eps();
    if (S_is_outside && p_SN_squared_dist > eps * eps) {
      // The sphere center is *measurably outside* the cylinder. There are three
      // possibilities: nearest point lies on the cap face, cap edge, or barrel.
      // In all three cases, the normal points from the nearest point to the
      // sphere center. Penetration depth is the radius minus the distance
      // between the pair of points. And the contact point is simply half the
      // depth from the nearest point in the normal direction.

      // Distance from closest point (N) to sphere center (S).
      const S d_NS = sqrt(p_SN_squared_dist);
      n_SC_C = p_SN_C / d_NS;
      depth = r_s - d_NS;
      p_CP = p_CN + n_SC_C * (depth * 0.5);
    } else {
      // The center is inside. It's either nearer the barrel or the end face
      // (with preference for the end face).
      const S& h = cylinder.lz;
      const S face_distance = p_CS(2) >= 0 ? h / 2 - p_CS(2) : p_CS(2) + h / 2;
      // For the barrel to be picked over the face, it must be more than
      // epsilon closer to the sphere center.

      // The direction from the sphere to the nearest point on the barrel on
      // the z = 0 plane.
      const Vector2<S> n_SB_xy = Vector2<S>(p_CS(0), p_CS(1));
      const S d_CS_xy = n_SB_xy.norm();
      const S barrel_distance = cylinder.radius - d_CS_xy;
      // If the center is near the Voronoi boundary between the near face and
      // the barrel, then this test would be affected by the precision loss
      // inherent in computing p_CS. If we did a *strict* comparison, then
      // we would get a different answer just by changing X_FC. This requires
      // the barrel to be closer by an amount that subsumes the potential
      // precision loss.
      if (barrel_distance < face_distance - eps) {
        // Closest to the barrel.
        if (d_CS_xy > eps) {
          // Normal towards barrel
          n_SC_C << -n_SB_xy(0) / d_CS_xy, -n_SB_xy(1) / d_CS_xy, 0;
          depth = r_s + barrel_distance;
          p_CP = p_CS + n_SC_C * ((r_s - barrel_distance) / 2);
        } else {
          // Sphere center is on the central spine of the cylinder, as per
          // documentation, assume we have penetration coming in from the +x
          // direction.
          n_SC_C = -Vector3<S>::UnitX();
          depth = r_s + cylinder.radius;
          p_CP = p_CS + n_SC_C * ((r_s - barrel_distance) / 2);
        }
      } else {
        // Closest to the face.
        n_SC_C << 0, 0, 0;
        // NOTE: This sign *may* seem counter-intuitive. A center nearest the +z
        // face produces a normal in the -z direction; this is because the
        // normal points from the sphere and into the cylinder; and the
        // penetration is *into* the +z face (so points in the -z direction).
        n_SC_C(2) = p_CS(2) >= 0 ? -1 : 1;
        depth = face_distance + r_s;
        p_CP = p_CS + n_SC_C * ((r_s - face_distance) / 2);
      }
    }
    contacts->emplace_back(X_FC.linear() * n_SC_C, X_FC * p_CP, depth);
  }
  return true;
}

//==============================================================================

template <typename S>
FCL_EXPORT bool sphereCylinderDistance(const Sphere<S>& sphere,
                                       const Transform3<S>& X_FS,
                                       const Cylinder<S>& cylinder,
                                       const Transform3<S>& X_FC, S* distance,
                                       Vector3<S>* p_FSc, Vector3<S>* p_FCs) {
  // Find the sphere center S in the cylinder's frame.
  const Transform3<S> X_CS = X_FC.inverse() * X_FS;
  const Vector3<S> p_CS = X_CS.translation();
  const S r_s = sphere.radius;

  // Find N, the nearest point *inside* the cylinder to the sphere center S
  // (measured and expressed in frame C).
  Vector3<S> p_CN;
  bool S_is_outside = nearestPointInCylinder(cylinder.lz, cylinder.radius, p_CS,
                                             &p_CN);

  if (S_is_outside) {
    // If N is not S, we know the sphere center is *outside* the cylinder (but
    // we  don't know yet if the they are completely separated).

    // Compute the position vector from the nearest point N to the sphere center
    // S in the frame C.
    const Vector3<S> p_NS_C = p_CS - p_CN;
    const S p_NS_squared_dist = p_NS_C.squaredNorm();
    if (p_NS_squared_dist > r_s * r_s) {
      // The distance to the nearest point is greater than the sphere radius;
      // we have proven separation.
      S d{-1};
      if (distance || p_FCs || p_FSc)
        d = sqrt(p_NS_squared_dist);
      if (distance != nullptr)
        *distance = d - r_s;
      if (p_FCs != nullptr)
        *p_FCs = X_FC * p_CN;
      if (p_FSc != nullptr) {
        const Vector3<S> p_CSc = p_CS - (p_NS_C * r_s / d);
        *p_FSc = X_FC * p_CSc;
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

#endif // FCL_NARROWPHASE_DETAIL_SPHERECYLINDER_INL_H

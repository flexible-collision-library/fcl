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

#ifndef FCL_NARROWPHASE_DETAIL_HALFSPACE_H
#define FCL_NARROWPHASE_DETAIL_HALFSPACE_H

#include "fcl/geometry/shape/sphere.h"
#include "fcl/geometry/shape/halfspace.h"
#include "fcl/geometry/shape/ellipsoid.h"
#include "fcl/geometry/shape/box.h"
#include "fcl/geometry/shape/capsule.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/geometry/shape/cone.h"
#include "fcl/geometry/shape/convex.h"
#include "fcl/geometry/shape/plane.h"
#include "fcl/narrowphase/contact_point.h"

namespace fcl
{

namespace detail
{

template <typename S>
FCL_EXPORT
S halfspaceIntersectTolerance();

template <>
FCL_EXPORT
float halfspaceIntersectTolerance();

template <>
FCL_EXPORT
double halfspaceIntersectTolerance();

template <typename S>
FCL_EXPORT
bool sphereHalfspaceIntersect(const Sphere<S>& s1, const Transform3<S>& tf1,
                              const Halfspace<S>& s2, const Transform3<S>& tf2,
                              std::vector<ContactPoint<S>>* contacts);

template <typename S>
FCL_EXPORT
bool ellipsoidHalfspaceIntersect(const Ellipsoid<S>& s1, const Transform3<S>& tf1,
                                 const Halfspace<S>& s2, const Transform3<S>& tf2,
                                 std::vector<ContactPoint<S>>* contacts);

/// @brief box half space, a, b, c  = +/- edge size
/// n^T * (R(o + a v1 + b v2 + c v3) + T) <= d
/// so (R^T n) (a v1 + b v2 + c v3) + n * T <= d
/// check whether d - n * T - (R^T n) (a v1 + b v2 + c v3) >= 0 for some a, b, c
/// the max value of left side is d - n * T + |(R^T n) (a v1 + b v2 + c v3)|, check that is enough
template <typename S>
FCL_EXPORT
bool boxHalfspaceIntersect(const Box<S>& s1, const Transform3<S>& tf1,
                           const Halfspace<S>& s2, const Transform3<S>& tf2);

template <typename S>
FCL_EXPORT
bool boxHalfspaceIntersect(const Box<S>& s1, const Transform3<S>& tf1,
                           const Halfspace<S>& s2, const Transform3<S>& tf2,
                           std::vector<ContactPoint<S>>* contacts);

template <typename S>
FCL_EXPORT
bool capsuleHalfspaceIntersect(const Capsule<S>& s1, const Transform3<S>& tf1,
                               const Halfspace<S>& s2, const Transform3<S>& tf2,
                               std::vector<ContactPoint<S>>* contacts);

template <typename S>
FCL_EXPORT
bool cylinderHalfspaceIntersect(const Cylinder<S>& s1, const Transform3<S>& tf1,
                                const Halfspace<S>& s2, const Transform3<S>& tf2,
                                std::vector<ContactPoint<S>>* contacts);

template <typename S>
FCL_EXPORT
bool coneHalfspaceIntersect(const Cone<S>& s1, const Transform3<S>& tf1,
                            const Halfspace<S>& s2, const Transform3<S>& tf2,
                            std::vector<ContactPoint<S>>* contacts);

template <typename S>
FCL_EXPORT
bool convexHalfspaceIntersect(const Convex<S>& s1, const Transform3<S>& tf1,
                              const Halfspace<S>& s2, const Transform3<S>& tf2,
                              Vector3<S>* contact_points, S* penetration_depth, Vector3<S>* normal);

/// @brief Reports whether a convex mesh and half space are intersecting.
/// If `contacts` is not `nullptr` and the two geometries are intersecting,
/// a new ContactPoint will be added to the data.
///
/// The two geometries are considered to be free of intersection if and only if
/// all points in the Convex mesh lie strictly _outside_ the half space. Lying
/// *on* the boundary of the half space is considered intersecting.
///
/// The two geometries are each defined in their own frames, but we have
/// transforms to a common frame F.
///
/// If the two geometries are intersecting and `contacts` is not `nullptr`, the
/// new ContactPoint.normal will point in the opposite direction as the half
/// space normal (expressed in Frame F) -- it points _into_ the half space. We
/// define the point P to be a point of `convex_C` that most deeply penetrates
/// into the half space. It is not guaranteed to be unique. If it is not unique,
/// it will be arbitrarily selected from the set of all such points. The
/// ContactPoint.penetration_depth value is the depth of P. ContactPoint.pos is
/// defined as the point halfway between P and the nearest point on the boundary
/// of the half space, measured and expressed in F.
///
/// ContactPoint is documented to report contact position in the world frame W.
/// This function will only truly satisfy that requirement if F = W. It is the
/// responsibility of the caller to understand the semantics of F and confirm
/// that it satisfies that requirement.
///
/// This makes use of the
/// [Drake monogram notation](http://drake.mit.edu/doxygen_cxx/group__multibody__notation__basics.html)
/// to describe quantities (particularly the poses of shapes).
///
/// @param convex_C      The convex mesh. Its vertex positions are measured and
///                      expressed in Frame C.
/// @param X_FC          The transform relating Frame C with the common frame F.
/// @param half_space_H  The half space. The position and orientation of its
///                      boundary plane are measured and expressed in Frame H.
/// @param X_FH          The transform relating Frame H with the common frame F.
/// @param[out] contacts The optional accumulator for data characterizing the
///                      intersection.
/// @return `true` if the two geometries are intersecting.
/// @tparam S The computational scalar.
template <typename S>
FCL_EXPORT bool convexHalfspaceIntersect(
    const Convex<S>& convex_C, const Transform3<S>& X_FC,
    const Halfspace<S>& half_space_H, const Transform3<S>& X_FH,
    std::vector<ContactPoint<S>>* contacts);

template <typename S>
FCL_EXPORT
bool halfspaceTriangleIntersect(const Halfspace<S>& s1, const Transform3<S>& tf1,
                                const Vector3<S>& P1, const Vector3<S>& P2, const Vector3<S>& P3, const Transform3<S>& tf2,
                                Vector3<S>* contact_points, S* penetration_depth, Vector3<S>* normal);

/// @brief return whether plane collides with halfspace
/// if the separation plane of the halfspace is parallel with the plane
///     return code 1, if the plane's normal is the same with halfspace's normal and plane is inside halfspace, also return plane in pl
///     return code 2, if the plane's normal is oppositie to the halfspace's normal and plane is inside halfspace, also return plane in pl
///     plane is outside halfspace, collision-free
/// if not parallel
///     return the intersection ray, return code 3. ray origin is p and direction is d
template <typename S>
FCL_EXPORT
bool planeHalfspaceIntersect(const Plane<S>& s1, const Transform3<S>& tf1,
                             const Halfspace<S>& s2, const Transform3<S>& tf2,
                             Plane<S>& pl,
                             Vector3<S>& p, Vector3<S>& d,
                             S& penetration_depth,
                             int& ret);

template <typename S>
FCL_EXPORT
bool halfspacePlaneIntersect(const Halfspace<S>& s1, const Transform3<S>& tf1,
                             const Plane<S>& s2, const Transform3<S>& tf2,
                             Plane<S>& pl, Vector3<S>& p, Vector3<S>& d,
                             S& penetration_depth,
                             int& ret);

/// @brief return whether two halfspace intersect
/// if the separation planes of the two halfspaces are parallel
///    return code 1, if two halfspaces' normal are same and s1 is in s2, also return s1 in s;
///    return code 2, if two halfspaces' normal are same and s2 is in s1, also return s2 in s;
///    return code 3, if two halfspaces' normal are opposite and s1 and s2 are into each other;
///    collision free, if two halfspaces' are separate;
/// if the separation planes of the two halfspaces are not parallel, return intersection ray, return code 4. ray origin is p and direction is d
/// collision free return code 0
template <typename S>
FCL_EXPORT
bool halfspaceIntersect(const Halfspace<S>& s1, const Transform3<S>& tf1,
                        const Halfspace<S>& s2, const Transform3<S>& tf2,
                        Vector3<S>& p, Vector3<S>& d,
                        Halfspace<S>& s,
                        S& penetration_depth,
                        int& ret);

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/primitive_shape_algorithm/halfspace-inl.h"

#endif

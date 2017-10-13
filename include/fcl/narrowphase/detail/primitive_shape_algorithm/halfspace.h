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

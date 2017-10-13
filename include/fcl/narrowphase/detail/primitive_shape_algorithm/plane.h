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

#ifndef FCL_NARROWPHASE_DETAIL_PLANE_H
#define FCL_NARROWPHASE_DETAIL_PLANE_H

#include "fcl/geometry/shape/sphere.h"
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
S planeIntersectTolerance();

template <>
FCL_EXPORT
double planeIntersectTolerance();

template <>
FCL_EXPORT
float planeIntersectTolerance();

template <typename S>
FCL_EXPORT
bool spherePlaneIntersect(const Sphere<S>& s1, const Transform3<S>& tf1,
                          const Plane<S>& s2, const Transform3<S>& tf2,
                          std::vector<ContactPoint<S>>* contacts);

template <typename S>
FCL_EXPORT
bool ellipsoidPlaneIntersect(const Ellipsoid<S>& s1, const Transform3<S>& tf1,
                             const Plane<S>& s2, const Transform3<S>& tf2,
                             std::vector<ContactPoint<S>>* contacts);

/// @brief box half space, a, b, c  = +/- edge size
/// n^T * (R(o + a v1 + b v2 + c v3) + T) ~ d
/// so (R^T n) (a v1 + b v2 + c v3) + n * T ~ d
/// check whether d - n * T - (R^T n) (a v1 + b v2 + c v3) >= 0 for some a, b, c and <=0 for some a, b, c
/// so need to check whether |d - n * T| <= |(R^T n)(a v1 + b v2 + c v3)|, the reason is as follows:
/// (R^T n) (a v1 + b v2 + c v3) can get |(R^T n) (a v1 + b v2 + c v3)| for one a, b, c.
/// if |d - n * T| <= |(R^T n)(a v1 + b v2 + c v3)| then can get both positive and negative value on the right side.
template <typename S>
FCL_EXPORT
bool boxPlaneIntersect(const Box<S>& s1, const Transform3<S>& tf1,
                       const Plane<S>& s2, const Transform3<S>& tf2,
                       std::vector<ContactPoint<S>>* contacts);

template <typename S>
FCL_EXPORT
bool capsulePlaneIntersect(const Capsule<S>& s1, const Transform3<S>& tf1,
                           const Plane<S>& s2, const Transform3<S>& tf2);

template <typename S>
FCL_EXPORT
bool capsulePlaneIntersect(const Capsule<S>& s1, const Transform3<S>& tf1,
                           const Plane<S>& s2, const Transform3<S>& tf2,
                           std::vector<ContactPoint<S>>* contacts);

/// @brief cylinder-plane intersect
/// n^T (R (r * cosa * v1 + r * sina * v2 + h * v3) + T) ~ d
/// need one point to be positive and one to be negative
/// (n^T * v3) * h + n * T -d + r * (cosa * (n^T * R * v1) + sina * (n^T * R * v2)) ~ 0
/// (n^T * v3) * h + r * (cosa * (n^T * R * v1) + sina * (n^T * R * v2)) + n * T - d ~ 0
template <typename S>
FCL_EXPORT
bool cylinderPlaneIntersect(const Cylinder<S>& s1, const Transform3<S>& tf1,
                            const Plane<S>& s2, const Transform3<S>& tf2);

template <typename S>
FCL_EXPORT
bool cylinderPlaneIntersect(const Cylinder<S>& s1, const Transform3<S>& tf1,
                            const Plane<S>& s2, const Transform3<S>& tf2,
                            std::vector<ContactPoint<S>>* contacts);

template <typename S>
FCL_EXPORT
bool conePlaneIntersect(const Cone<S>& s1, const Transform3<S>& tf1,
                        const Plane<S>& s2, const Transform3<S>& tf2,
                        std::vector<ContactPoint<S>>* contacts);

template <typename S>
FCL_EXPORT
bool convexPlaneIntersect(const Convex<S>& s1, const Transform3<S>& tf1,
                          const Plane<S>& s2, const Transform3<S>& tf2,
                          Vector3<S>* contact_points, S* penetration_depth, Vector3<S>* normal);

template <typename S>
FCL_EXPORT
bool planeTriangleIntersect(const Plane<S>& s1, const Transform3<S>& tf1,
                            const Vector3<S>& P1, const Vector3<S>& P2, const Vector3<S>& P3, const Transform3<S>& tf2,
                            Vector3<S>* contact_points, S* penetration_depth, Vector3<S>* normal);

template <typename S>
FCL_EXPORT
bool planeIntersect(const Plane<S>& s1, const Transform3<S>& tf1,
                    const Plane<S>& s2, const Transform3<S>& tf2,
                    std::vector<ContactPoint<S>>* contacts);

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/primitive_shape_algorithm/plane-inl.h"

#endif

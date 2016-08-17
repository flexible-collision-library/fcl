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

#include "fcl/narrowphase/detail/primitive_shape_algorithm/plane-inl.h"

namespace fcl
{

namespace detail
{

//==============================================================================
template <>
double planeIntersectTolerance()
{
  return 0.0000001;
}

//==============================================================================
template <>
float planeIntersectTolerance()
{
  return 0.0001;
}

//==============================================================================
template
bool spherePlaneIntersect(const Sphere<double>& s1, const Transform3<double>& tf1,
                          const Plane<double>& s2, const Transform3<double>& tf2,
                          std::vector<ContactPoint<double>>* contacts);

//==============================================================================
template
bool ellipsoidPlaneIntersect(const Ellipsoid<double>& s1, const Transform3<double>& tf1,
                             const Plane<double>& s2, const Transform3<double>& tf2,
                             std::vector<ContactPoint<double>>* contacts);

//==============================================================================
template
bool boxPlaneIntersect(const Box<double>& s1, const Transform3<double>& tf1,
                       const Plane<double>& s2, const Transform3<double>& tf2,
                       std::vector<ContactPoint<double>>* contacts);

//==============================================================================
template
bool capsulePlaneIntersect(const Capsule<double>& s1, const Transform3<double>& tf1,
                           const Plane<double>& s2, const Transform3<double>& tf2);

//==============================================================================
template
bool capsulePlaneIntersect(const Capsule<double>& s1, const Transform3<double>& tf1,
                           const Plane<double>& s2, const Transform3<double>& tf2,
                           std::vector<ContactPoint<double>>* contacts);

//==============================================================================
template
bool cylinderPlaneIntersect(const Cylinder<double>& s1, const Transform3<double>& tf1,
                            const Plane<double>& s2, const Transform3<double>& tf2);

//==============================================================================
template
bool cylinderPlaneIntersect(const Cylinder<double>& s1, const Transform3<double>& tf1,
                            const Plane<double>& s2, const Transform3<double>& tf2,
                            std::vector<ContactPoint<double>>* contacts);

//==============================================================================
template
bool conePlaneIntersect(const Cone<double>& s1, const Transform3<double>& tf1,
                        const Plane<double>& s2, const Transform3<double>& tf2,
                        std::vector<ContactPoint<double>>* contacts);

//==============================================================================
template
bool convexPlaneIntersect(const Convex<double>& s1, const Transform3<double>& tf1,
                          const Plane<double>& s2, const Transform3<double>& tf2,
                          Vector3<double>* contact_points, double* penetration_depth, Vector3<double>* normal);

//==============================================================================
template
bool planeTriangleIntersect(const Plane<double>& s1, const Transform3<double>& tf1,
                            const Vector3<double>& P1, const Vector3<double>& P2, const Vector3<double>& P3, const Transform3<double>& tf2,
                            Vector3<double>* contact_points, double* penetration_depth, Vector3<double>* normal);

//==============================================================================
template
bool planeIntersect(const Plane<double>& s1, const Transform3<double>& tf1,
                    const Plane<double>& s2, const Transform3<double>& tf2,
                    std::vector<ContactPoint<double>>* contacts);

} // namespace detail
} // namespace fcl

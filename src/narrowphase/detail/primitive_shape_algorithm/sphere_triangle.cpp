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

#define FCL_NARROWPHASE_DETAIL_PRIMITIVE_SHAPE_ALGORITHM_SPHERE_TRIANGLE_BUILDING
#include "fcl/narrowphase/detail/primitive_shape_algorithm/sphere_triangle-inl.h"

namespace fcl
{

namespace detail
{

//==============================================================================
template
FCL_EXPORT
double segmentSqrDistance(const Vector3<double>& from, const Vector3<double>& to,const Vector3<double>& p, Vector3<double>& nearest);

//==============================================================================
template
FCL_EXPORT
bool projectInTriangle(const Vector3<double>& p1, const Vector3<double>& p2, const Vector3<double>& p3, const Vector3<double>& normal, const Vector3<double>& p);

//==============================================================================
template
FCL_EXPORT
bool sphereTriangleIntersect(const Sphere<double>& s, const Transform3<double>& tf,
                             const Vector3<double>& P1, const Vector3<double>& P2, const Vector3<double>& P3, Vector3<double>* contact_points, double* penetration_depth, Vector3<double>* normal_);

//==============================================================================
template
FCL_EXPORT
bool sphereTriangleDistance(const Sphere<double>& sp, const Transform3<double>& tf,
                            const Vector3<double>& P1, const Vector3<double>& P2, const Vector3<double>& P3,
                            double* dist);

//==============================================================================
template
FCL_EXPORT
bool sphereTriangleDistance(const Sphere<double>& sp, const Transform3<double>& tf,
                            const Vector3<double>& P1, const Vector3<double>& P2, const Vector3<double>& P3,
                            double* dist, Vector3<double>* p1, Vector3<double>* p2);

//==============================================================================
template
FCL_EXPORT
bool sphereTriangleDistance(const Sphere<double>& sp, const Transform3<double>& tf1,
                            const Vector3<double>& P1, const Vector3<double>& P2, const Vector3<double>& P3, const Transform3<double>& tf2,
                            double* dist, Vector3<double>* p1, Vector3<double>* p2);

} // namespace detail
} // namespace fcl

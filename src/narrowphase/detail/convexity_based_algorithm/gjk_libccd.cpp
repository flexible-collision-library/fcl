/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary formdouble, with or without
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
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIEdouble, INCLUDING, BUT NOT
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

#include "fcl/narrowphase/detail/convexity_based_algorithm/gjk_libccd-inl.h"

namespace fcl
{

namespace detail
{

//==============================================================================
template
class GJKInitializer<double, Cylinder<double>>;

//==============================================================================
template
class GJKInitializer<double, Sphere<double>>;

//==============================================================================
template
class GJKInitializer<double, Ellipsoid<double>>;

//==============================================================================
template
class GJKInitializer<double, Box<double>>;

//==============================================================================
template
class GJKInitializer<double, Capsule<double>>;

//==============================================================================
template
class GJKInitializer<double, Cone<double>>;

//==============================================================================
template
class GJKInitializer<double, Convex<double>>;

//==============================================================================
template
void* triCreateGJKObject(
    const Vector3d& P1, const Vector3d& P2, const Vector3d& P3);

//==============================================================================
template
void* triCreateGJKObject(
    const Vector3d& P1,
    const Vector3d& P2,
    const Vector3d& P3,
    const Transform3d& tf);

//==============================================================================
template
bool GJKCollide(
    void* obj1,
    ccd_support_fn supp1,
    ccd_center_fn cen1,
    void* obj2,
    ccd_support_fn supp2,
    ccd_center_fn cen2,
    unsigned int max_iterations,
    double tolerance,
    Vector3d* contact_points,
    double* penetration_depth,
    Vector3d* normal);

//==============================================================================
template
bool GJKDistance(
    void* obj1,
    ccd_support_fn supp1,
    void* obj2,
    ccd_support_fn supp2,
    unsigned int max_iterations,
    double tolerance,
    double* dist,
    Vector3d* p1,
    Vector3d* p2);

template
bool GJKSignedDistance(
    void* obj1,
    ccd_support_fn supp1,
    void* obj2,
    ccd_support_fn supp2,
    unsigned int max_iterations,
    double tolerance,
    double* dist,
    Vector3d* p1,
    Vector3d* p2);

} // namespace detail
} // namespace fcl

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

#include "fcl/geometry/shape/utility-inl.h"

namespace fcl {

//==============================================================================
template
void constructBox(const AABB<double>& bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
template
void constructBox(const OBB<double>& bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
template
void constructBox(const OBBRSS<double>& bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
template
void constructBox(const kIOS<double>& bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
template
void constructBox(const RSS<double>& bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
template
void constructBox(const KDOP<double, 16>& bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
template
void constructBox(const KDOP<double, 18>& bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
template
void constructBox(const KDOP<double, 24>& bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
template
void constructBox(const AABB<double>& bv, const Transform3<double>& tf_bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
template
void constructBox(const OBB<double>& bv, const Transform3<double>& tf_bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
template
void constructBox(const OBBRSS<double>& bv, const Transform3<double>& tf_bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
template
void constructBox(const kIOS<double>& bv, const Transform3<double>& tf_bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
template
void constructBox(const RSS<double>& bv, const Transform3<double>& tf_bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
template
void constructBox(const KDOP<double, 16>& bv, const Transform3<double>& tf_bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
template
void constructBox(const KDOP<double, 18>& bv, const Transform3<double>& tf_bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
template
void constructBox(const KDOP<double, 24>& bv, const Transform3<double>& tf_bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
namespace detail {
//==============================================================================

//==============================================================================
template
struct ComputeBVImpl<double, AABB<double>, Box<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, OBB<double>, Box<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, AABB<double>, Capsule<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, OBB<double>, Capsule<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, AABB<double>, Cone<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, OBB<double>, Cone<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, AABB<double>, Cylinder<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, OBB<double>, Cylinder<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, AABB<double>, Ellipsoid<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, OBB<double>, Ellipsoid<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, AABB<double>, Halfspace<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, OBB<double>, Halfspace<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, RSS<double>, Halfspace<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, OBBRSS<double>, Halfspace<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, kIOS<double>, Halfspace<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, KDOP<double, 16>, Halfspace<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, KDOP<double, 18>, Halfspace<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, KDOP<double, 24>, Halfspace<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, AABB<double>, Plane<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, OBB<double>, Plane<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, RSS<double>, Plane<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, OBBRSS<double>, Plane<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, kIOS<double>, Plane<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, KDOP<double, 16>, Plane<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, KDOP<double, 18>, Plane<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, KDOP<double, 24>, Plane<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, AABB<double>, Sphere<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, OBB<double>, Sphere<double>>;

//==============================================================================
template
struct ComputeBVImpl<double, AABB<double>, TriangleP<double>>;

//==============================================================================
} // namespace detail
//==============================================================================

} // namespace fcl

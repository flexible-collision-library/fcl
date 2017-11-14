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

#ifndef FCL_GEOMETRY_SHAPE_UTILITY_H
#define FCL_GEOMETRY_SHAPE_UTILITY_H

// This header shouldn't be included by any bounding volumen class (e.g.,
// AABB and OBB) nor geometric shapes (e.g., Box and Sphere).

#include "fcl/common/types.h"

#include "fcl/math/bv/AABB.h"
#include "fcl/math/bv/kDOP.h"
#include "fcl/math/bv/kIOS.h"
#include "fcl/math/bv/OBB.h"
#include "fcl/math/bv/OBBRSS.h"
#include "fcl/math/bv/RSS.h"

#include "fcl/geometry/shape/box.h"

namespace fcl
{

/// @brief calculate a bounding volume for a shape in a specific configuration
template <typename BV, typename Shape>
FCL_EXPORT
void computeBV(const Shape& s, const Transform3<typename BV::S>& tf, BV& bv);

/// @brief construct a box shape (with a configuration) from a given bounding volume
template <typename S>
FCL_EXPORT
void constructBox(const AABB<S>& bv, Box<S>& box, Transform3<S>& tf);

template <typename S>
FCL_EXPORT
void constructBox(const OBB<S>& bv, Box<S>& box, Transform3<S>& tf);

template <typename S>
FCL_EXPORT
void constructBox(const OBBRSS<S>& bv, Box<S>& box, Transform3<S>& tf);

template <typename S>
FCL_EXPORT
void constructBox(const kIOS<S>& bv, Box<S>& box, Transform3<S>& tf);

template <typename S>
FCL_EXPORT
void constructBox(const RSS<S>& bv, Box<S>& box, Transform3<S>& tf);

template <typename S>
FCL_EXPORT
void constructBox(const KDOP<S, 16>& bv, Box<S>& box, Transform3<S>& tf);

template <typename S>
FCL_EXPORT
void constructBox(const KDOP<S, 18>& bv, Box<S>& box, Transform3<S>& tf);

template <typename S>
FCL_EXPORT
void constructBox(const KDOP<S, 24>& bv, Box<S>& box, Transform3<S>& tf);

template <typename S>
FCL_EXPORT
void constructBox(const AABB<S>& bv, const Transform3<S>& tf_bv, Box<S>& box, Transform3<S>& tf);

template <typename S>
FCL_EXPORT
void constructBox(const OBB<S>& bv, const Transform3<S>& tf_bv, Box<S>& box, Transform3<S>& tf);

template <typename S>
FCL_EXPORT
void constructBox(const OBBRSS<S>& bv, const Transform3<S>& tf_bv, Box<S>& box, Transform3<S>& tf);

template <typename S>
FCL_EXPORT
void constructBox(const kIOS<S>& bv, const Transform3<S>& tf_bv, Box<S>& box, Transform3<S>& tf);

template <typename S>
FCL_EXPORT
void constructBox(const RSS<S>& bv, const Transform3<S>& tf_bv, Box<S>& box, Transform3<S>& tf);

template <typename S>
FCL_EXPORT
void constructBox(const KDOP<S, 16>& bv, const Transform3<S>& tf_bv, Box<S>& box, Transform3<S>& tf);

template <typename S>
FCL_EXPORT
void constructBox(const KDOP<S, 18>& bv, const Transform3<S>& tf_bv, Box<S>& box, Transform3<S>& tf);

template <typename S>
FCL_EXPORT
void constructBox(const KDOP<S, 24>& bv, const Transform3<S>& tf_bv, Box<S>& box, Transform3<S>& tf);

} // namespace fcl

#include "fcl/geometry/shape/utility-inl.h"

#endif

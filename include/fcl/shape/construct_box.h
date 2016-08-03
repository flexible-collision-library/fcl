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

/** \author Jia Pan */


#ifndef FCL_SHAPE_CONSTRUCT_BOX_H
#define FCL_SHAPE_CONSTRUCT_BOX_H

#include <vector>

#include "fcl/common/warning.h"
#include "fcl/BV/BV.h"
#include "fcl/shape/box.h"

namespace fcl
{

/// @brief construct a box shape (with a configuration) from a given bounding volume
template <typename Scalar>
void constructBox(const AABB<Scalar>& bv, Box<Scalar>& box, Transform3<Scalar>& tf);

template <typename Scalar>
void constructBox(const OBB<Scalar>& bv, Box<Scalar>& box, Transform3<Scalar>& tf);

template <typename Scalar>
void constructBox(const OBBRSS<Scalar>& bv, Box<Scalar>& box, Transform3<Scalar>& tf);

template <typename Scalar>
void constructBox(const kIOS<Scalar>& bv, Box<Scalar>& box, Transform3<Scalar>& tf);

template <typename Scalar>
void constructBox(const RSS<Scalar>& bv, Box<Scalar>& box, Transform3<Scalar>& tf);

template <typename Scalar>
void constructBox(const KDOP<Scalar, 16>& bv, Box<Scalar>& box, Transform3<Scalar>& tf);

template <typename Scalar>
void constructBox(const KDOP<Scalar, 18>& bv, Box<Scalar>& box, Transform3<Scalar>& tf);

template <typename Scalar>
void constructBox(const KDOP<Scalar, 24>& bv, Box<Scalar>& box, Transform3<Scalar>& tf);

template <typename Scalar>
void constructBox(const AABB<Scalar>& bv, const Transform3<Scalar>& tf_bv, Box<Scalar>& box, Transform3<Scalar>& tf);

template <typename Scalar>
void constructBox(const OBB<Scalar>& bv, const Transform3<Scalar>& tf_bv, Box<Scalar>& box, Transform3<Scalar>& tf);

template <typename Scalar>
void constructBox(const OBBRSS<Scalar>& bv, const Transform3<Scalar>& tf_bv, Box<Scalar>& box, Transform3<Scalar>& tf);

template <typename Scalar>
void constructBox(const kIOS<Scalar>& bv, const Transform3<Scalar>& tf_bv, Box<Scalar>& box, Transform3<Scalar>& tf);

template <typename Scalar>
void constructBox(const RSS<Scalar>& bv, const Transform3<Scalar>& tf_bv, Box<Scalar>& box, Transform3<Scalar>& tf);

template <typename Scalar>
void constructBox(const KDOP<Scalar, 16>& bv, const Transform3<Scalar>& tf_bv, Box<Scalar>& box, Transform3<Scalar>& tf);

template <typename Scalar>
void constructBox(const KDOP<Scalar, 18>& bv, const Transform3<Scalar>& tf_bv, Box<Scalar>& box, Transform3<Scalar>& tf);

template <typename Scalar>
void constructBox(const KDOP<Scalar, 24>& bv, const Transform3<Scalar>& tf_bv, Box<Scalar>& box, Transform3<Scalar>& tf);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
void constructBox(const AABB<Scalar>& bv, Box<Scalar>& box, Transform3<Scalar>& tf)
{
  box = Box<Scalar>(bv.max_ - bv.min_);
  tf.linear().setIdentity();
  tf.translation() = bv.center();
}

//==============================================================================
template <typename Scalar>
void constructBox(const OBB<Scalar>& bv, Box<Scalar>& box, Transform3<Scalar>& tf)
{
  box = Box<Scalar>(bv.extent * 2);
  tf.linear() = bv.axis;
  tf.translation() = bv.To;
}

//==============================================================================
template <typename Scalar>
void constructBox(const OBBRSS<Scalar>& bv, Box<Scalar>& box, Transform3<Scalar>& tf)
{
  box = Box<Scalar>(bv.obb.extent * 2);
  tf.linear() = bv.obb.axis;
  tf.translation() = bv.obb.To;
}

//==============================================================================
template <typename Scalar>
void constructBox(const kIOS<Scalar>& bv, Box<Scalar>& box, Transform3<Scalar>& tf)
{
  box = Box<Scalar>(bv.obb.extent * 2);
  tf.linear() = bv.obb.axis;
  tf.translation() = bv.obb.To;
}

//==============================================================================
template <typename Scalar>
void constructBox(const RSS<Scalar>& bv, Box<Scalar>& box, Transform3<Scalar>& tf)
{
  box = Box<Scalar>(bv.width(), bv.height(), bv.depth());
  tf.linear() = bv.axis;
  tf.translation() = bv.Tr;
}

//==============================================================================
template <typename Scalar>
void constructBox(const KDOP<Scalar, 16>& bv, Box<Scalar>& box, Transform3<Scalar>& tf)
{
  box = Box<Scalar>(bv.width(), bv.height(), bv.depth());
  tf.linear().setIdentity();
  tf.translation() = bv.center();
}

//==============================================================================
template <typename Scalar>
void constructBox(const KDOP<Scalar, 18>& bv, Box<Scalar>& box, Transform3<Scalar>& tf)
{
  box = Box<Scalar>(bv.width(), bv.height(), bv.depth());
  tf.linear().setIdentity();
  tf.translation() = bv.center();
}

//==============================================================================
template <typename Scalar>
void constructBox(const KDOP<Scalar, 24>& bv, Box<Scalar>& box, Transform3<Scalar>& tf)
{
  box = Box<Scalar>(bv.width(), bv.height(), bv.depth());
  tf.linear().setIdentity();
  tf.translation() = bv.center();
}

//==============================================================================
template <typename Scalar>
void constructBox(const AABB<Scalar>& bv, const Transform3<Scalar>& tf_bv, Box<Scalar>& box, Transform3<Scalar>& tf)
{
  FCL_SUPPRESS_UNINITIALIZED_BEGIN
  box = Box<Scalar>(bv.max_ - bv.min_);
  FCL_SUPPRESS_UNINITIALIZED_END
  tf = tf_bv * Eigen::Translation3d(bv.center());
}

//==============================================================================
template <typename Scalar>
void constructBox(const OBB<Scalar>& bv, const Transform3<Scalar>& tf_bv, Box<Scalar>& box, Transform3<Scalar>& tf)
{
  FCL_SUPPRESS_UNINITIALIZED_BEGIN
  box = Box<Scalar>(bv.extent * 2);
  FCL_SUPPRESS_UNINITIALIZED_END
  tf.linear() = bv.axis;
  tf.translation() = bv.To;
}

//==============================================================================
template <typename Scalar>
void constructBox(const OBBRSS<Scalar>& bv, const Transform3<Scalar>& tf_bv, Box<Scalar>& box, Transform3<Scalar>& tf)
{
  FCL_SUPPRESS_UNINITIALIZED_BEGIN
  box = Box<Scalar>(bv.obb.extent * 2);
  FCL_SUPPRESS_UNINITIALIZED_END
  tf.linear() = bv.obb.axis;
  tf.translation() = bv.obb.To;
  tf = tf_bv * tf;
}

//==============================================================================
template <typename Scalar>
void constructBox(const kIOS<Scalar>& bv, const Transform3<Scalar>& tf_bv, Box<Scalar>& box, Transform3<Scalar>& tf)
{
  FCL_SUPPRESS_UNINITIALIZED_BEGIN
  box = Box<Scalar>(bv.obb.extent * 2);
  FCL_SUPPRESS_UNINITIALIZED_END
  tf.linear() = bv.obb.axis;
  tf.translation() = bv.obb.To;
  tf = tf_bv * tf;
}

//==============================================================================
template <typename Scalar>
void constructBox(const RSS<Scalar>& bv, const Transform3<Scalar>& tf_bv, Box<Scalar>& box, Transform3<Scalar>& tf)
{
  FCL_SUPPRESS_UNINITIALIZED_BEGIN
  box = Box<Scalar>(bv.width(), bv.height(), bv.depth());
  FCL_SUPPRESS_UNINITIALIZED_END
  tf.linear() = bv.axis;
  tf.translation() = bv.Tr;
  tf = tf_bv * tf;
}

//==============================================================================
template <typename Scalar>
void constructBox(const KDOP<Scalar, 16>& bv, const Transform3<Scalar>& tf_bv, Box<Scalar>& box, Transform3<Scalar>& tf)
{
  FCL_SUPPRESS_UNINITIALIZED_BEGIN
  box = Box<Scalar>(bv.width(), bv.height(), bv.depth());
  FCL_SUPPRESS_UNINITIALIZED_END
  tf = tf_bv * Eigen::Translation3d(bv.center());
}

//==============================================================================
template <typename Scalar>
void constructBox(const KDOP<Scalar, 18>& bv, const Transform3<Scalar>& tf_bv, Box<Scalar>& box, Transform3<Scalar>& tf)
{
  FCL_SUPPRESS_UNINITIALIZED_BEGIN
  box = Box<Scalar>(bv.width(), bv.height(), bv.depth());
  FCL_SUPPRESS_UNINITIALIZED_END
  tf = tf_bv * Eigen::Translation3d(bv.center());
}

//==============================================================================
template <typename Scalar>
void constructBox(const KDOP<Scalar, 24>& bv, const Transform3<Scalar>& tf_bv, Box<Scalar>& box, Transform3<Scalar>& tf)
{
  FCL_SUPPRESS_UNINITIALIZED_BEGIN
  box = Box<Scalar>(bv.width(), bv.height(), bv.depth());
  FCL_SUPPRESS_UNINITIALIZED_END
  tf = tf_bv * Eigen::Translation3d(bv.center());
}

} // namespace fcl

#endif

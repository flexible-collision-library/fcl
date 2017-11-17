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

#ifndef FCL_CCD_MOTION_BASE_INL_H
#define FCL_CCD_MOTION_BASE_INL_H

#include "fcl/math/motion/motion_base.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT MotionBase<double>;

//==============================================================================
template <typename S>
MotionBase<S>::MotionBase()
  : time_interval_(std::shared_ptr<TimeInterval<S>>(new TimeInterval<S>(0, 1)))
{
  // Do nothing
}

//==============================================================================
template <typename S>
MotionBase<S>::~MotionBase() {}

//==============================================================================
template <typename S>
void MotionBase<S>::getCurrentTransform(Matrix3<S>& R, Vector3<S>& T) const
{
  Transform3<S> tf;
  getCurrentTransform(tf);
  R = tf.linear();
  T = tf.translation();
}

//==============================================================================
template <typename S>
void MotionBase<S>::getCurrentTransform(Quaternion<S>& Q, Vector3<S>& T) const
{
  Transform3<S> tf;
  getCurrentTransform(tf);
  Q = tf.linear();
  T = tf.translation();
}

//==============================================================================
template <typename S>
void MotionBase<S>::getCurrentRotation(Matrix3<S>& R) const
{
  Transform3<S> tf;
  getCurrentTransform(tf);
  R = tf.linear();
}

//==============================================================================
template <typename S>
void MotionBase<S>::getCurrentRotation(Quaternion<S>& Q) const
{
  Transform3<S> tf;
  getCurrentTransform(tf);
  Q = tf.linear();
}

//==============================================================================
template <typename S>
void MotionBase<S>::getCurrentTranslation(Vector3<S>& T) const
{
  Transform3<S> tf;
  getCurrentTransform(tf);
  T = tf.translation();
}

//==============================================================================
template <typename S>
const std::shared_ptr<TimeInterval<S> >&MotionBase<S>::getTimeInterval() const
{
  return time_interval_;
}

} // namespace fcl

#endif

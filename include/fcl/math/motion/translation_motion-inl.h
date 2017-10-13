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

#ifndef FCL_CCD_TRANSLATIONMOTION_INL_H
#define FCL_CCD_TRANSLATIONMOTION_INL_H

#include "fcl/math/motion/translation_motion.h"

#include "fcl/common/unused.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT TranslationMotion<double>;

//==============================================================================
template <typename S>
TranslationMotion<S>::TranslationMotion(
    const Transform3<S>& tf1, const Transform3<S>& tf2)
  : MotionBase<S>(),
    rot(tf1.linear()),
    trans_start(tf1.translation()),
    trans_range(tf2.translation() - tf1.translation()),
    tf(tf1)
{
  // Do nothing
}

//==============================================================================
template <typename S>
TranslationMotion<S>::TranslationMotion(
    const Matrix3<S>& R, const Vector3<S>& T1, const Vector3<S>& T2)
  : MotionBase<S>(), tf(Transform3<S>::Identity())
{
  rot = R;
  trans_start = T1;
  trans_range = T2 - T1;
  tf.linear() = R;
  tf.translation() = trans_start;
}

//==============================================================================
template <typename S>
bool TranslationMotion<S>::integrate(S dt) const
{
  if(dt > 1)
    dt = 1;

  tf.linear() = rot.toRotationMatrix(); // TODO(JS): necessary?
  tf.translation() = trans_start + trans_range * dt;

  return true;
}

//==============================================================================
template <typename S>
S TranslationMotion<S>::computeMotionBound(
    const BVMotionBoundVisitor<S>& mb_visitor) const
{
  return mb_visitor.visit(*this);
}

//==============================================================================
template <typename S>
S TranslationMotion<S>::computeMotionBound(
    const TriangleMotionBoundVisitor<S>& mb_visitor) const
{
  return mb_visitor.visit(*this);
}

//==============================================================================
template <typename S>
void TranslationMotion<S>::getCurrentTransform(Transform3<S>& tf_) const
{
  tf_ = tf;
}

//==============================================================================
template <typename S>
void TranslationMotion<S>::getTaylorModel(TMatrix3<S>& tm, TVector3<S>& tv) const
{
  FCL_UNUSED(tm);
  FCL_UNUSED(tv);

  // Do nothing
  // TODO(JS): Not implemented?
}

//==============================================================================
template <typename S>
Vector3<S> TranslationMotion<S>::getVelocity() const
{
  return trans_range;
}

} // namespace fcl

#endif

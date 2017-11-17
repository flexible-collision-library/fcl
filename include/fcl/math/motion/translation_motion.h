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

#ifndef FCL_CCD_TRANSLATIONMOTION_H
#define FCL_CCD_TRANSLATIONMOTION_H

#include "fcl/math/motion/motion_base.h"
#include "fcl/math/motion/bv_motion_bound_visitor.h"
#include "fcl/math/motion/triangle_motion_bound_visitor.h"

namespace fcl
{

template <typename S>
class FCL_EXPORT TranslationMotion : public MotionBase<S>
{
public:
  /// @brief Construct motion from intial and goal transform
  TranslationMotion(const Transform3<S>& tf1, const Transform3<S>& tf2);

  TranslationMotion(
      const Matrix3<S>& R, const Vector3<S>& T1, const Vector3<S>& T2);

  bool integrate(S dt) const override;

  S computeMotionBound(
      const BVMotionBoundVisitor<S>& mb_visitor) const override;

  S computeMotionBound(
      const TriangleMotionBoundVisitor<S>& mb_visitor) const override;

  void getCurrentTransform(Transform3<S>& tf_) const override;

  void getTaylorModel(TMatrix3<S>& tm, TVector3<S>& tv) const override;

  Vector3<S> getVelocity() const;

 private:
  /// @brief initial and goal transforms
  Quaternion<S> rot;
  Vector3<S> trans_start, trans_range;

  mutable Transform3<S> tf;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using TranslationMotionf = TranslationMotion<float>;
using TranslationMotiond = TranslationMotion<double>;

} // namespace fcl

#include "fcl/math/motion/translation_motion-inl.h"

#endif

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

#ifndef FCL_CCD_INTERPMOTION_H
#define FCL_CCD_INTERPMOTION_H

#include <iostream>
#include <vector>
#include "fcl/math/geometry.h"
#include "fcl/math/motion/motion_base.h"
#include "fcl/math/motion/bv_motion_bound_visitor.h"
#include "fcl/math/motion/triangle_motion_bound_visitor.h"

namespace fcl
{

/// @brief Linear interpolation motion
/// Each Motion is assumed to have constant linear velocity and angular velocity
/// The motion is R(t)(p - p_ref) + p_ref + T(t)
/// Therefore, R(0) = R0, R(1) = R1
///            T(0) = T0 + R0 p_ref - p_ref
///            T(1) = T1 + R1 p_ref - p_ref
template <typename S>
class FCL_EXPORT InterpMotion : public MotionBase<S>
{
public:
  /// @brief Default transformations are all identities
  InterpMotion();

  /// @brief Construct motion from the initial rotation/translation and goal rotation/translation
  InterpMotion(const Matrix3<S>& R1, const Vector3<S>& T1,
               const Matrix3<S>& R2, const Vector3<S>& T2);

  InterpMotion(const Transform3<S>& tf1_, const Transform3<S>& tf2_);

  /// @brief Construct motion from the initial rotation/translation and goal rotation/translation related to some rotation center
  InterpMotion(const Matrix3<S>& R1, const Vector3<S>& T1,
               const Matrix3<S>& R2, const Vector3<S>& T2,
               const Vector3<S>& O);

  InterpMotion(const Transform3<S>& tf1_, const Transform3<S>& tf2_, const Vector3<S>& O);

  /// @brief Integrate the motion from 0 to dt
  /// We compute the current transformation from zero point instead of from last integrate time, for precision.
  bool integrate(double dt) const;

  /// @brief Compute the motion bound for a bounding volume along a given direction n, which is defined in the visitor
  S computeMotionBound(const BVMotionBoundVisitor<S>& mb_visitor) const;

  /// @brief Compute the motion bound for a triangle along a given direction n, which is defined in the visitor 
  S computeMotionBound(const TriangleMotionBoundVisitor<S>& mb_visitor) const;

  /// @brief Get the rotation and translation in current step
  void getCurrentTransform(Transform3<S>& tf_) const;

  void getTaylorModel(TMatrix3<S>& tm, TVector3<S>& tv) const;

protected:

  void computeVelocity();

  Quaternion<S> deltaRotation(S dt) const;
  
  Quaternion<S> absoluteRotation(S dt) const;
  
  /// @brief The transformation at time 0
  Transform3<S> tf1;

  /// @brief The transformation at time 1
  Transform3<S> tf2;

  /// @brief The transformation at current time t
  mutable Transform3<S> tf;

  /// @brief Linear velocity
  Vector3<S> linear_vel;

  /// @brief Angular speed
  S angular_vel;

  /// @brief Angular velocity axis
  Vector3<S> angular_axis;

  /// @brief Reference point for the motion (in the object's local frame)
  Vector3<S> reference_p;

public:
  const Vector3<S>& getReferencePoint() const;

  const Vector3<S>& getAngularAxis() const;

  S getAngularVelocity() const;

  const Vector3<S>& getLinearVelocity() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace fcl

#include "fcl/math/motion/interp_motion-inl.h"

#endif

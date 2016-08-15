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

#include "fcl/math/geometry.h"
#include "fcl/math/motion/motion_base.h"
#include <iostream>
#include <vector>

namespace fcl
{

/// @brief Linear interpolation motion
/// Each Motion is assumed to have constant linear velocity and angular velocity
/// The motion is R(t)(p - p_ref) + p_ref + T(t)
/// Therefore, R(0) = R0, R(1) = R1
///            T(0) = T0 + R0 p_ref - p_ref
///            T(1) = T1 + R1 p_ref - p_ref
template <typename S>
class InterpMotion : public MotionBase<S>
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

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename S>
InterpMotion<S>::InterpMotion()
  : MotionBase<S>(), angular_axis(Vector3<S>::UnitX())
{
  // Default angular velocity is zero
  angular_vel = 0;

  // Default reference point is local zero point

  // Default linear velocity is zero
}

//==============================================================================
template <typename S>
InterpMotion<S>::InterpMotion(
    const Matrix3<S>& R1, const Vector3<S>& T1,
    const Matrix3<S>& R2, const Vector3<S>& T2)
  : MotionBase<S>(),
    tf1(Transform3<S>::Identity()),
    tf2(Transform3<S>::Identity())
{
  tf1.linear() = R1;
  tf1.translation() = T1;

  tf2.linear() = R2;
  tf2.translation() = T2;

  tf = tf1;

  // Compute the velocities for the motion
  computeVelocity();
}

//==============================================================================
template <typename S>
InterpMotion<S>::InterpMotion(
    const Transform3<S>& tf1_, const Transform3<S>& tf2_)
  : MotionBase<S>(), tf1(tf1_), tf2(tf2_), tf(tf1)
{
  // Compute the velocities for the motion
  computeVelocity();
}

//==============================================================================
template <typename S>
InterpMotion<S>::InterpMotion(
    const Matrix3<S>& R1,
    const Vector3<S>& T1,
    const Matrix3<S>& R2,
    const Vector3<S>& T2,
    const Vector3<S>& O)
  : MotionBase<S>(),
    tf1(Transform3<S>::Identity()),
    tf2(Transform3<S>::Identity()),
    reference_p(O)
{
  tf1.linear() = R1;
  tf1.translation() = T1;

  tf2.linear() = R2;
  tf2.translation() = T2;

  tf = tf1;

  // Compute the velocities for the motion
  computeVelocity();
}

//==============================================================================
template <typename S>
InterpMotion<S>::InterpMotion(
    const Transform3<S>& tf1_, const Transform3<S>& tf2_, const Vector3<S>& O)
  : MotionBase<S>(), tf1(tf1_), tf2(tf2_), tf(tf1), reference_p(O)
{
  // Do nothing
}

//==============================================================================
template <typename S>
bool InterpMotion<S>::integrate(double dt) const
{
  if(dt > 1) dt = 1;

  tf.linear() = absoluteRotation(dt).toRotationMatrix();
  tf.translation() = linear_vel * dt + tf1 * reference_p - tf.linear() * reference_p;

  return true;
}

//==============================================================================
template <typename S>
S InterpMotion<S>::computeMotionBound(const BVMotionBoundVisitor<S>& mb_visitor) const
{
  return mb_visitor.visit(*this);
}

//==============================================================================
template <typename S>
S InterpMotion<S>::computeMotionBound(const TriangleMotionBoundVisitor<S>& mb_visitor) const
{
  return mb_visitor.visit(*this);
}

//==============================================================================
template <typename S>
void InterpMotion<S>::getCurrentTransform(Transform3<S>& tf_) const
{
  tf_ = tf;
}

//==============================================================================
template <typename S>
void InterpMotion<S>::getTaylorModel(TMatrix3<S>& tm, TVector3<S>& tv) const
{
  Matrix3<S> hat_angular_axis;
  hat(hat_angular_axis, angular_axis);

  TaylorModel<S> cos_model(this->getTimeInterval());
  generateTaylorModelForCosFunc(cos_model, angular_vel, (S)0);
  TaylorModel<S> sin_model(this->getTimeInterval());
  generateTaylorModelForSinFunc(sin_model, angular_vel, (S)0);

  TMatrix3<S> delta_R = hat_angular_axis * sin_model
      - (hat_angular_axis * hat_angular_axis).eval() * (cos_model - 1)
      + Matrix3<S>::Identity();

  TaylorModel<S> a(this->getTimeInterval()), b(this->getTimeInterval()), c(this->getTimeInterval());
  generateTaylorModelForLinearFunc(a, (S)0, linear_vel[0]);
  generateTaylorModelForLinearFunc(b, (S)0, linear_vel[1]);
  generateTaylorModelForLinearFunc(c, (S)0, linear_vel[2]);
  TVector3<S> delta_T(a, b, c);

  tm = delta_R * tf1.linear().eval();
  tv = tf1 * reference_p
      + delta_T
      - delta_R * (tf1.linear() * reference_p).eval();
}

//==============================================================================
template <typename S>
void InterpMotion<S>::computeVelocity()
{
  linear_vel = tf2 * reference_p - tf1 * reference_p;

  const AngleAxis<S> aa(tf2.linear() * tf1.linear().transpose());
  angular_axis = aa.axis();
  angular_vel = aa.angle();

  if(angular_vel < 0)
  {
    angular_vel = -angular_vel;
    angular_axis = -angular_axis;
  }
}

//==============================================================================
template <typename S>
Quaternion<S> InterpMotion<S>::deltaRotation(S dt) const
{
  return Quaternion<S>(AngleAxis<S>((S)(dt * angular_vel), angular_axis));
}

//==============================================================================
template <typename S>
Quaternion<S> InterpMotion<S>::absoluteRotation(S dt) const
{
  Quaternion<S> delta_t = deltaRotation(dt);
  return delta_t * Quaternion<S>(tf1.linear());
}

//==============================================================================
template <typename S>
const Vector3<S>&InterpMotion<S>::getReferencePoint() const
{
  return reference_p;
}

//==============================================================================
template <typename S>
const Vector3<S>&InterpMotion<S>::getAngularAxis() const
{
  return angular_axis;
}

//==============================================================================
template <typename S>
S InterpMotion<S>::getAngularVelocity() const
{
  return angular_vel;
}

//==============================================================================
template <typename S>
const Vector3<S>&InterpMotion<S>::getLinearVelocity() const
{
  return linear_vel;
}

} // namespace fcl

#endif

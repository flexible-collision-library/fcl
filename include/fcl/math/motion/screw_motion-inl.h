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

#ifndef FCL_CCD_SCREWMOTION_INL_H
#define FCL_CCD_SCREWMOTION_INL_H

#include "fcl/math/motion/screw_motion.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT ScrewMotion<double>;

//==============================================================================
template <typename S>
ScrewMotion<S>::ScrewMotion()
  : MotionBase<S>(), axis(Vector3<S>::UnitX())
{
  // Default angular velocity is zero
  angular_vel = 0;

  // Default reference point is local zero point

  // Default linear velocity is zero
  linear_vel = 0;
}

//==============================================================================
template <typename S>
ScrewMotion<S>::ScrewMotion(
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

  computeScrewParameter();
}

//==============================================================================
template <typename S>
ScrewMotion<S>::ScrewMotion(
    const Transform3<S>& tf1_, const Transform3<S>& tf2_)
  : tf1(tf1_), tf2(tf2_), tf(tf1)
{
  computeScrewParameter();
}

//==============================================================================
template <typename S>
bool ScrewMotion<S>::integrate(double dt) const
{
  if(dt > 1) dt = 1;

  tf.linear() = absoluteRotation(dt).toRotationMatrix();

  Quaternion<S> delta_rot = deltaRotation(dt);
  tf.translation() = p + axis * (dt * linear_vel) + delta_rot * (tf1.translation() - p);

  return true;
}

//==============================================================================
template <typename S>
S ScrewMotion<S>::computeMotionBound(
    const BVMotionBoundVisitor<S>& mb_visitor) const
{
  return mb_visitor.visit(*this);
}

//==============================================================================
template <typename S>
S ScrewMotion<S>::computeMotionBound(
    const TriangleMotionBoundVisitor<S>& mb_visitor) const
{
  return mb_visitor.visit(*this);
}

//==============================================================================
template <typename S>
void ScrewMotion<S>::getCurrentTransform(Transform3<S>& tf_) const
{
  tf_ = tf;
}

//==============================================================================
template <typename S>
void ScrewMotion<S>::getTaylorModel(TMatrix3<S>& tm, TVector3<S>& tv) const
{
  Matrix3<S> hat_axis;
  hat(hat_axis, axis);

  TaylorModel<S> cos_model(this->getTimeInterval());
  generateTaylorModelForCosFunc(cos_model, angular_vel, (S)0);

  TaylorModel<S> sin_model(this->getTimeInterval());
  generateTaylorModelForSinFunc(sin_model, angular_vel, (S)0);

  TMatrix3<S> delta_R = hat_axis * sin_model
      - (hat_axis * hat_axis).eval() * (cos_model - 1)
      + Matrix3<S>::Identity();

  TaylorModel<S> a(this->getTimeInterval()), b(this->getTimeInterval()), c(this->getTimeInterval());
  generateTaylorModelForLinearFunc(a, (S)0, linear_vel * axis[0]);
  generateTaylorModelForLinearFunc(b, (S)0, linear_vel * axis[1]);
  generateTaylorModelForLinearFunc(c, (S)0, linear_vel * axis[2]);
  TVector3<S> delta_T = p - delta_R * p + TVector3<S>(a, b, c);

  tm = delta_R * tf1.linear().eval();
  tv = delta_R * tf1.translation().eval() + delta_T;
}

//==============================================================================
template <typename S>
void ScrewMotion<S>::computeScrewParameter()
{
  const AngleAxis<S> aa(tf2.linear() * tf1.linear().transpose());

  axis = aa.axis();
  angular_vel = aa.angle();

  if(angular_vel < 0)
  {
    angular_vel = -angular_vel;
    axis = -axis;
  }

  if(angular_vel < 1e-10)
  {
    angular_vel = 0;
    axis = tf2.translation() - tf1.translation();
    linear_vel = axis.norm();
    p = tf1.translation();
  }
  else
  {
    Vector3<S> o = tf2.translation() - tf1.translation();
    p = (tf1.translation() + tf2.translation() + axis.cross(o) * (1.0 / tan(angular_vel / 2.0))) * 0.5;
    linear_vel = o.dot(axis);
  }
}

//==============================================================================
template <typename S>
Quaternion<S> ScrewMotion<S>::deltaRotation(S dt) const
{
  return Quaternion<S>(AngleAxis<S>((S)(dt * angular_vel), axis));
}

//==============================================================================
template <typename S>
Quaternion<S> ScrewMotion<S>::absoluteRotation(S dt) const
{
  Quaternion<S> delta_t = deltaRotation(dt);

  return delta_t * Quaternion<S>(tf1.linear());
}

//==============================================================================
template <typename S>
S ScrewMotion<S>::getLinearVelocity() const
{
  return linear_vel;
}

//==============================================================================
template <typename S>
S ScrewMotion<S>::getAngularVelocity() const
{
  return angular_vel;
}

//==============================================================================
template <typename S>
const Vector3<S>&ScrewMotion<S>::getAxis() const
{
  return axis;
}

//==============================================================================
template <typename S>
const Vector3<S>&ScrewMotion<S>::getAxisOrigin() const
{
  return p;
}

} // namespace fcl

#endif

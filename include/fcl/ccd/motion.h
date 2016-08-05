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


#ifndef FCL_CCD_MOTION_H
#define FCL_CCD_MOTION_H

#include "fcl/math/geometry.h"
#include "fcl/ccd/motion_base.h"
#include "fcl/intersect.h"
#include <iostream>
#include <vector>

namespace fcl
{

template <typename Scalar>
class TranslationMotion : public MotionBase<Scalar>
{
public:
  /// @brief Construct motion from intial and goal transform
  TranslationMotion(const Transform3<Scalar>& tf1,
                    const Transform3<Scalar>& tf2) : MotionBase<Scalar>(),
                                              rot(tf1.linear()),
                                              trans_start(tf1.translation()),
                                              trans_range(tf2.translation() - tf1.translation()),
                                              tf(tf1)
  {
  }

  TranslationMotion(const Matrix3<Scalar>& R, const Vector3<Scalar>& T1, const Vector3<Scalar>& T2) : MotionBase<Scalar>(),
                                                                           tf(Transform3<Scalar>::Identity())
  {
    rot = R;
    trans_start = T1;
    trans_range = T2 - T1;
    tf.linear() = R;
    tf.translation() = trans_start;
  }

  bool integrate(Scalar dt) const
  {
    if(dt > 1)
      dt = 1;

    tf.linear() = rot.toRotationMatrix(); // TODO(JS): necessary?
    tf.translation() = trans_start + trans_range * dt;

    return true;
  }

  Scalar computeMotionBound(const BVMotionBoundVisitor<Scalar>& mb_visitor) const
  {
    return mb_visitor.visit(*this);
  }

  Scalar computeMotionBound(const TriangleMotionBoundVisitor<Scalar>& mb_visitor) const
  {
    return mb_visitor.visit(*this);
  }

  void getCurrentTransform(Transform3<Scalar>& tf_) const
  {
    tf_ = tf;
  }

  void getTaylorModel(TMatrix3<Scalar>& tm, TVector3<Scalar>& tv) const
  {
  }

  Vector3<Scalar> getVelocity() const
  {
    return trans_range;
  }

 private:
  /// @brief initial and goal transforms
  Quaternion3<Scalar> rot;
  Vector3<Scalar> trans_start, trans_range;

  mutable Transform3<Scalar> tf;
};

template <typename Scalar>
class SplineMotion : public MotionBase<Scalar>
{
public:
  /// @brief Construct motion from 4 deBoor points
  SplineMotion(const Vector3<Scalar>& Td0, const Vector3<Scalar>& Td1, const Vector3<Scalar>& Td2, const Vector3<Scalar>& Td3,
               const Vector3<Scalar>& Rd0, const Vector3<Scalar>& Rd1, const Vector3<Scalar>& Rd2, const Vector3<Scalar>& Rd3);

  // @brief Construct motion from initial and goal transform
  SplineMotion(const Matrix3<Scalar>& R1, const Vector3<Scalar>& T1,
               const Matrix3<Scalar>& R2, const Vector3<Scalar>& T2) : MotionBase<Scalar>()
  {
    // TODO
  }

  SplineMotion(const Transform3<Scalar>& tf1,
               const Transform3<Scalar>& tf2) : MotionBase<Scalar>()
  {
    // TODO
  }

  
  /// @brief Integrate the motion from 0 to dt
  /// We compute the current transformation from zero point instead of from last integrate time, for precision.
  bool integrate(double dt) const;

  /// @brief Compute the motion bound for a bounding volume along a given direction n, which is defined in the visitor
  Scalar computeMotionBound(const BVMotionBoundVisitor<Scalar>& mb_visitor) const
  {
    return mb_visitor.visit(*this);
  }

  /// @brief Compute the motion bound for a triangle along a given direction n, which is defined in the visitor
  Scalar computeMotionBound(const TriangleMotionBoundVisitor<Scalar>& mb_visitor) const
  {
    return mb_visitor.visit(*this);
  }

  /// @brief Get the rotation and translation in current step
  void getCurrentTransform(Transform3<Scalar>& tf_) const
  {
    tf_ = tf;
  }

  void getTaylorModel(TMatrix3<Scalar>& tm, TVector3<Scalar>& tv) const
  {
    // set tv
    Vector3<Scalar> c[4];
    c[0] = (Td[0] + Td[1] * 4 + Td[2] + Td[3]) * (1/6.0);
    c[1] = (-Td[0] + Td[2]) * (1/2.0);
    c[2] = (Td[0] - Td[1] * 2 + Td[2]) * (1/2.0);
    c[3] = (-Td[0] + Td[1] * 3 - Td[2] * 3 + Td[3]) * (1/6.0);
    tv.setTimeInterval(this->getTimeInterval());
    for(std::size_t i = 0; i < 3; ++i)
    {
      for(std::size_t j = 0; j < 4; ++j)
      {
        tv[i].coeff(j) = c[j][i];
      }
    }

    // set tm
    Matrix3<Scalar> I = Matrix3<Scalar>::Identity();
    // R(t) = R(t0) + R'(t0) (t-t0) + 1/2 R''(t0)(t-t0)^2 + 1 / 6 R'''(t0) (t-t0)^3 + 1 / 24 R''''(l)(t-t0)^4; t0 = 0.5
    /// 1. compute M(1/2)
    Vector3<Scalar> Rt0 = (Rd[0] + Rd[1] * 23 + Rd[2] * 23 + Rd[3]) * (1 / 48.0);
    Scalar Rt0_len = Rt0.norm();
    Scalar inv_Rt0_len = 1.0 / Rt0_len;
    Scalar inv_Rt0_len_3 = inv_Rt0_len * inv_Rt0_len * inv_Rt0_len;
    Scalar inv_Rt0_len_5 = inv_Rt0_len_3 * inv_Rt0_len * inv_Rt0_len;
    Scalar theta0 = Rt0_len;
    Scalar costheta0 = cos(theta0);
    Scalar sintheta0 = sin(theta0);
    
    Vector3<Scalar> Wt0 = Rt0 * inv_Rt0_len;
    Matrix3<Scalar> hatWt0;
    hat(hatWt0, Wt0);
    Matrix3<Scalar> hatWt0_sqr = hatWt0 * hatWt0;
    Matrix3<Scalar> Mt0 = I + hatWt0 * sintheta0 + hatWt0_sqr * (1 - costheta0);
    // TODO(JS): this could be improved by using exp(Wt0)

    /// 2. compute M'(1/2)
    Vector3<Scalar> dRt0 = (-Rd[0] - Rd[1] * 5 + Rd[2] * 5 + Rd[3]) * (1 / 8.0);
    Scalar Rt0_dot_dRt0 = Rt0.dot(dRt0);
    Scalar dtheta0 = Rt0_dot_dRt0 * inv_Rt0_len;
    Vector3<Scalar> dWt0 = dRt0 * inv_Rt0_len - Rt0 * (Rt0_dot_dRt0 * inv_Rt0_len_3);
    Matrix3<Scalar> hatdWt0;
    hat(hatdWt0, dWt0);
    Matrix3<Scalar> dMt0 = hatdWt0 * sintheta0 + hatWt0 * (costheta0 * dtheta0) + hatWt0_sqr * (sintheta0 * dtheta0) + (hatWt0 * hatdWt0 + hatdWt0 * hatWt0) * (1 - costheta0);

    /// 3.1. compute M''(1/2)
    Vector3<Scalar> ddRt0 = (Rd[0] - Rd[1] - Rd[2] + Rd[3]) * 0.5;
    Scalar Rt0_dot_ddRt0 = Rt0.dot(ddRt0);
    Scalar dRt0_dot_dRt0 = dRt0.squaredNorm();
    Scalar ddtheta0 = (Rt0_dot_ddRt0 + dRt0_dot_dRt0) * inv_Rt0_len - Rt0_dot_dRt0 * Rt0_dot_dRt0 * inv_Rt0_len_3;
    Vector3<Scalar> ddWt0 = ddRt0 * inv_Rt0_len - (dRt0 * (2 * Rt0_dot_dRt0) + Rt0 * (Rt0_dot_ddRt0 + dRt0_dot_dRt0)) * inv_Rt0_len_3 + (Rt0 * (3 * Rt0_dot_dRt0 * Rt0_dot_dRt0)) * inv_Rt0_len_5;
    Matrix3<Scalar> hatddWt0;
    hat(hatddWt0, ddWt0);
    Matrix3<Scalar> ddMt0 =
      hatddWt0 * sintheta0 +
      hatWt0 * (costheta0 * dtheta0 - sintheta0 * dtheta0 * dtheta0 + costheta0 * ddtheta0) +
      hatdWt0 * (costheta0 * dtheta0) +
      (hatWt0 * hatdWt0 + hatdWt0 * hatWt0) * (sintheta0 * dtheta0 * 2) +
      hatdWt0 * hatdWt0 * (2 * (1 - costheta0)) +
      hatWt0 * hatWt0 * (costheta0 * dtheta0 * dtheta0 + sintheta0 * ddtheta0) +
      (hatWt0 * hatddWt0 + hatddWt0 + hatWt0) * (1 - costheta0);


    tm.setTimeInterval(this->getTimeInterval());
    for(std::size_t i = 0; i < 3; ++i)
    {
      for(std::size_t j = 0; j < 3; ++j)
      {
        tm(i, j).coeff(0) = Mt0(i, j) - dMt0(i, j) * 0.5 + ddMt0(i, j) * 0.25 * 0.5;
        tm(i, j).coeff(1) = dMt0(i, j) - ddMt0(i, j) * 0.5;
        tm(i, j).coeff(2) = ddMt0(i, j) * 0.5;
        tm(i, j).coeff(3) = 0;

        tm(i, j).remainder() = Interval<Scalar>(-1/48.0, 1/48.0); /// not correct, should fix
      }
    } 
  }

protected:
  void computeSplineParameter()
  {
  }

  Scalar getWeight0(Scalar t) const;
  Scalar getWeight1(Scalar t) const;
  Scalar getWeight2(Scalar t) const;
  Scalar getWeight3(Scalar t) const;
  
  Vector3<Scalar> Td[4];
  Vector3<Scalar> Rd[4];

  Vector3<Scalar> TA, TB, TC;
  Vector3<Scalar> RA, RB, RC;

  Scalar Rd0Rd0, Rd0Rd1, Rd0Rd2, Rd0Rd3, Rd1Rd1, Rd1Rd2, Rd1Rd3, Rd2Rd2, Rd2Rd3, Rd3Rd3;
  //// @brief The transformation at current time t
  mutable Transform3<Scalar> tf;

  /// @brief The time related with tf
  mutable Scalar tf_t;

public:
  Scalar computeTBound(const Vector3<Scalar>& n) const;
  
  Scalar computeDWMax() const;

  Scalar getCurrentTime() const
  {
    return tf_t;
  }

};

template <typename Scalar>
class ScrewMotion : public MotionBase<Scalar>
{
public:
  /// @brief Default transformations are all identities
  ScrewMotion() : MotionBase<Scalar>(), axis(Vector3<Scalar>::UnitX())
  {
    // Default angular velocity is zero
    angular_vel = 0;

    // Default reference point is local zero point

    // Default linear velocity is zero
    linear_vel = 0;
  }

  /// @brief Construct motion from the initial rotation/translation and goal rotation/translation
  ScrewMotion(const Matrix3<Scalar>& R1, const Vector3<Scalar>& T1,
              const Matrix3<Scalar>& R2, const Vector3<Scalar>& T2) : MotionBase<Scalar>(),
                                                     tf1(Transform3<Scalar>::Identity()),
                                                     tf2(Transform3<Scalar>::Identity())
  {
    tf1.linear() = R1;
    tf1.translation() = T1;

    tf2.linear() = R2;
    tf2.translation() = T2;

    tf = tf1;

    computeScrewParameter();
  }

  /// @brief Construct motion from the initial transform and goal transform
  ScrewMotion(const Transform3<Scalar>& tf1_,
              const Transform3<Scalar>& tf2_) : tf1(tf1_),
                                         tf2(tf2_),
                                         tf(tf1)
  {
    computeScrewParameter();
  }

  /// @brief Integrate the motion from 0 to dt
  /// We compute the current transformation from zero point instead of from last integrate time, for precision.
  bool integrate(double dt) const
  {
    if(dt > 1) dt = 1;
    
    tf.linear() = absoluteRotation(dt).toRotationMatrix();
    
    Quaternion3<Scalar> delta_rot = deltaRotation(dt);
    tf.translation() = p + axis * (dt * linear_vel) + delta_rot * (tf1.translation() - p);

    return true;
  }

  /// @brief Compute the motion bound for a bounding volume along a given direction n, which is defined in the visitor
  Scalar computeMotionBound(const BVMotionBoundVisitor<Scalar>& mb_visitor) const
  {
    return mb_visitor.visit(*this);
  }

  /// @brief Compute the motion bound for a triangle along a given direction n, which is defined in the visitor
  Scalar computeMotionBound(const TriangleMotionBoundVisitor<Scalar>& mb_visitor) const
  {
    return mb_visitor.visit(*this);
  }


  /// @brief Get the rotation and translation in current step
  void getCurrentTransform(Transform3<Scalar>& tf_) const
  {
    tf_ = tf;
  }

  void getTaylorModel(TMatrix3<Scalar>& tm, TVector3<Scalar>& tv) const
  {
    Matrix3<Scalar> hat_axis;
    hat(hat_axis, axis);

    TaylorModel<Scalar> cos_model(this->getTimeInterval());
    generateTaylorModelForCosFunc(cos_model, angular_vel, 0);
    
    TaylorModel<Scalar> sin_model(this->getTimeInterval());
    generateTaylorModelForSinFunc(sin_model, angular_vel, 0);

    TMatrix3<Scalar> delta_R = hat_axis * sin_model
        - (hat_axis * hat_axis).eval() * (cos_model - 1)
        + Matrix3<Scalar>::Identity();

    TaylorModel<Scalar> a(this->getTimeInterval()), b(this->getTimeInterval()), c(this->getTimeInterval());
    generateTaylorModelForLinearFunc(a, 0, linear_vel * axis[0]);
    generateTaylorModelForLinearFunc(b, 0, linear_vel * axis[1]);
    generateTaylorModelForLinearFunc(c, 0, linear_vel * axis[2]);
    TVector3<Scalar> delta_T = p - delta_R * p + TVector3<Scalar>(a, b, c);

    tm = delta_R * tf1.linear().eval();
    tv = delta_R * tf1.translation().eval() + delta_T;
  }

protected:
  void computeScrewParameter()
  {
    const Eigen::AngleAxisd aa(tf2.linear() * tf1.linear().transpose());

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
      Vector3<Scalar> o = tf2.translation() - tf1.translation();
      p = (tf1.translation() + tf2.translation() + axis.cross(o) * (1.0 / tan(angular_vel / 2.0))) * 0.5;
      linear_vel = o.dot(axis);
    }
  }

  Quaternion3<Scalar> deltaRotation(Scalar dt) const
  {
    return Quaternion3<Scalar>(Eigen::AngleAxisd((Scalar)(dt * angular_vel), axis));
  }

  Quaternion3<Scalar> absoluteRotation(Scalar dt) const
  {
    Quaternion3<Scalar> delta_t = deltaRotation(dt);

    return delta_t * Quaternion3<Scalar>(tf1.linear());
  }

  /// @brief The transformation at time 0
  Transform3<Scalar> tf1;

  /// @brief The transformation at time 1
  Transform3<Scalar> tf2;

  /// @brief The transformation at current time t
  mutable Transform3<Scalar> tf;

  /// @brief screw axis
  Vector3<Scalar> axis;

  /// @brief A point on the axis S
  Vector3<Scalar> p;

  /// @brief linear velocity along the axis
  Scalar linear_vel;

  /// @brief angular velocity
  Scalar angular_vel;

public:

  inline Scalar getLinearVelocity() const
  {
    return linear_vel;
  }

  inline Scalar getAngularVelocity() const
  {
    return angular_vel;
  }

  inline const Vector3<Scalar>& getAxis() const
  {
    return axis;
  }

  inline const Vector3<Scalar>& getAxisOrigin() const
  {
    return p;
  }
};



/// @brief Linear interpolation motion
/// Each Motion is assumed to have constant linear velocity and angular velocity
/// The motion is R(t)(p - p_ref) + p_ref + T(t)
/// Therefore, R(0) = R0, R(1) = R1
///            T(0) = T0 + R0 p_ref - p_ref
///            T(1) = T1 + R1 p_ref - p_ref
template <typename Scalar>
class InterpMotion : public MotionBase<Scalar>
{
public:
  /// @brief Default transformations are all identities
  InterpMotion();

  /// @brief Construct motion from the initial rotation/translation and goal rotation/translation
  InterpMotion(const Matrix3<Scalar>& R1, const Vector3<Scalar>& T1,
               const Matrix3<Scalar>& R2, const Vector3<Scalar>& T2);

  InterpMotion(const Transform3<Scalar>& tf1_, const Transform3<Scalar>& tf2_);

  /// @brief Construct motion from the initial rotation/translation and goal rotation/translation related to some rotation center
  InterpMotion(const Matrix3<Scalar>& R1, const Vector3<Scalar>& T1,
               const Matrix3<Scalar>& R2, const Vector3<Scalar>& T2,
               const Vector3<Scalar>& O);

  InterpMotion(const Transform3<Scalar>& tf1_, const Transform3<Scalar>& tf2_, const Vector3<Scalar>& O);

  /// @brief Integrate the motion from 0 to dt
  /// We compute the current transformation from zero point instead of from last integrate time, for precision.
  bool integrate(double dt) const;

  /// @brief Compute the motion bound for a bounding volume along a given direction n, which is defined in the visitor
  Scalar computeMotionBound(const BVMotionBoundVisitor<Scalar>& mb_visitor) const
  {
    return mb_visitor.visit(*this);
  }

  /// @brief Compute the motion bound for a triangle along a given direction n, which is defined in the visitor 
  Scalar computeMotionBound(const TriangleMotionBoundVisitor<Scalar>& mb_visitor) const
  {
    return mb_visitor.visit(*this);
  }

  /// @brief Get the rotation and translation in current step
  void getCurrentTransform(Transform3<Scalar>& tf_) const
  {
    tf_ = tf;
  }

  void getTaylorModel(TMatrix3<Scalar>& tm, TVector3<Scalar>& tv) const
  {
    Matrix3<Scalar> hat_angular_axis;
    hat(hat_angular_axis, angular_axis);

    TaylorModel<Scalar> cos_model(this->getTimeInterval());
    generateTaylorModelForCosFunc(cos_model, angular_vel, 0);
    TaylorModel<Scalar> sin_model(this->getTimeInterval());
    generateTaylorModelForSinFunc(sin_model, angular_vel, 0);

    TMatrix3<Scalar> delta_R = hat_angular_axis * sin_model
        - (hat_angular_axis * hat_angular_axis).eval() * (cos_model - 1)
        + Matrix3<Scalar>::Identity();

    TaylorModel<Scalar> a(this->getTimeInterval()), b(this->getTimeInterval()), c(this->getTimeInterval());
    generateTaylorModelForLinearFunc(a, 0, linear_vel[0]);
    generateTaylorModelForLinearFunc(b, 0, linear_vel[1]);
    generateTaylorModelForLinearFunc(c, 0, linear_vel[2]);
    TVector3<Scalar> delta_T(a, b, c);
    
    tm = delta_R * tf1.linear().eval();
    tv = tf1 * reference_p
        + delta_T
        - delta_R * (tf1.linear() * reference_p).eval();
  }

protected:

  void computeVelocity();

  Quaternion3<Scalar> deltaRotation(Scalar dt) const;
  
  Quaternion3<Scalar> absoluteRotation(Scalar dt) const;
  
  /// @brief The transformation at time 0
  Transform3<Scalar> tf1;

  /// @brief The transformation at time 1
  Transform3<Scalar> tf2;

  /// @brief The transformation at current time t
  mutable Transform3<Scalar> tf;

  /// @brief Linear velocity
  Vector3<Scalar> linear_vel;

  /// @brief Angular speed
  Scalar angular_vel;

  /// @brief Angular velocity axis
  Vector3<Scalar> angular_axis;

  /// @brief Reference point for the motion (in the object's local frame)
  Vector3<Scalar> reference_p;

public:
  const Vector3<Scalar>& getReferencePoint() const
  {
    return reference_p;
  }

  const Vector3<Scalar>& getAngularAxis() const
  {
    return angular_axis;
  }

  Scalar getAngularVelocity() const
  {
    return angular_vel;
  }

  const Vector3<Scalar>& getLinearVelocity() const
  {
    return linear_vel;
  }
};

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
SplineMotion<Scalar>::SplineMotion(const Vector3<Scalar>& Td0, const Vector3<Scalar>& Td1, const Vector3<Scalar>& Td2, const Vector3<Scalar>& Td3,
                           const Vector3<Scalar>& Rd0, const Vector3<Scalar>& Rd1, const Vector3<Scalar>& Rd2, const Vector3<Scalar>& Rd3) : MotionBase<Scalar>()
{
  Td[0] = Td0;
  Td[1] = Td1;
  Td[2] = Td2;
  Td[3] = Td3;

  Rd[0] = Rd0;
  Rd[1] = Rd1;
  Rd[2] = Rd2;
  Rd[3] = Rd3;

  Rd0Rd0 = Rd[0].dot(Rd[0]);
  Rd0Rd1 = Rd[0].dot(Rd[1]);
  Rd0Rd2 = Rd[0].dot(Rd[2]);
  Rd0Rd3 = Rd[0].dot(Rd[3]);
  Rd1Rd1 = Rd[1].dot(Rd[1]);
  Rd1Rd2 = Rd[1].dot(Rd[2]);
  Rd1Rd3 = Rd[1].dot(Rd[3]);
  Rd2Rd2 = Rd[2].dot(Rd[2]);
  Rd2Rd3 = Rd[2].dot(Rd[3]);
  Rd3Rd3 = Rd[3].dot(Rd[3]);

  TA = Td[1] * 3 - Td[2] * 3 + Td[3] - Td[0];
  TB = (Td[0] - Td[1] * 2 + Td[2]) * 3;
  TC = (Td[2] - Td[0]) * 3;

  RA = Rd[1] * 3 - Rd[2] * 3 + Rd[3] - Rd[0];
  RB = (Rd[0] - Rd[1] * 2 + Rd[2]) * 3;
  RC = (Rd[2] - Rd[0]) * 3;

  integrate(0.0);
}

//==============================================================================
template <typename Scalar>
bool SplineMotion<Scalar>::integrate(double dt) const
{
  if(dt > 1) dt = 1;

  Vector3<Scalar> cur_T = Td[0] * getWeight0(dt) + Td[1] * getWeight1(dt) + Td[2] * getWeight2(dt) + Td[3] * getWeight3(dt);
  Vector3<Scalar> cur_w = Rd[0] * getWeight0(dt) + Rd[1] * getWeight1(dt) + Rd[2] * getWeight2(dt) + Rd[3] * getWeight3(dt);
  Scalar cur_angle = cur_w.norm();
  cur_w.normalize();

  tf.linear() = Eigen::AngleAxisd(cur_angle, cur_w).toRotationMatrix();
  tf.translation() = cur_T;

  tf_t = dt;

  return true;
}

//==============================================================================
template <typename Scalar>
Scalar SplineMotion<Scalar>::computeTBound(const Vector3<Scalar>& n) const
{
  Scalar Ta = TA.dot(n);
  Scalar Tb = TB.dot(n);
  Scalar Tc = TC.dot(n);

  std::vector<Scalar> T_potential;
  T_potential.push_back(tf_t);
  T_potential.push_back(1);
  if(Tb * Tb - 3 * Ta * Tc >= 0)
  {
    if(Ta == 0)
    {
      if(Tb != 0)
      {
        Scalar tmp = -Tc / (2 * Tb);
        if(tmp < 1 && tmp > tf_t)
          T_potential.push_back(tmp);
      }
    }
    else
    {
      Scalar tmp_delta = sqrt(Tb * Tb - 3 * Ta * Tc);
      Scalar tmp1 = (-Tb + tmp_delta) / (3 * Ta);
      Scalar tmp2 = (-Tb - tmp_delta) / (3 * Ta);
      if(tmp1 < 1 && tmp1 > tf_t)
        T_potential.push_back(tmp1);
      if(tmp2 < 1 && tmp2 > tf_t)
        T_potential.push_back(tmp2);
    }
  }

  Scalar T_bound = Ta * T_potential[0] * T_potential[0] * T_potential[0] + Tb * T_potential[0] * T_potential[0] + Tc * T_potential[0];
  for(unsigned int i = 1; i < T_potential.size(); ++i)
  {
    Scalar T_bound_tmp = Ta * T_potential[i] * T_potential[i] * T_potential[i] + Tb * T_potential[i] * T_potential[i] + Tc * T_potential[i];
    if(T_bound_tmp > T_bound) T_bound = T_bound_tmp;
  }


  Scalar cur_delta = Ta * tf_t * tf_t * tf_t + Tb * tf_t * tf_t + Tc * tf_t;

  T_bound -= cur_delta;
  T_bound /= 6.0;

  return T_bound;
}

//==============================================================================
template <typename Scalar>
Scalar SplineMotion<Scalar>::computeDWMax() const
{
  // first compute ||w'||
  int a00[5] = {1,-4,6,-4,1};
  int a01[5] = {-3,10,-11,4,0};
  int a02[5] = {3,-8,6,0,-1};
  int a03[5] = {-1,2,-1,0,0};
  int a11[5] = {9,-24,16,0,0};
  int a12[5] = {-9,18,-5,-4,0};
  int a13[5] = {3,-4,0,0,0};
  int a22[5] = {9,-12,-2,4,1};
  int a23[5] = {-3,2,1,0,0};
  int a33[5] = {1,0,0,0,0};

  Scalar a[5];

  for(int i = 0; i < 5; ++i)
  {
    a[i] = Rd0Rd0 * a00[i] + Rd0Rd1 * a01[i] + Rd0Rd2 * a02[i] + Rd0Rd3 * a03[i]
      + Rd0Rd1 * a01[i] + Rd1Rd1 * a11[i] + Rd1Rd2 * a12[i] + Rd1Rd3 * a13[i]
      + Rd0Rd2 * a02[i] + Rd1Rd2 * a12[i] + Rd2Rd2 * a22[i] + Rd2Rd3 * a23[i]
      + Rd0Rd3 * a03[i] + Rd1Rd3 * a13[i] + Rd2Rd3 * a23[i] + Rd3Rd3 * a33[i];
    a[i] /= 4.0;
  }

  // compute polynomial for ||w'||'
  int da00[4] = {4,-12,12,-4};
  int da01[4] = {-12,30,-22,4};
  int da02[4] = {12,-24,12,0};
  int da03[4] = {-4,6,-2,0};
  int da11[4] = {36,-72,32,0};
  int da12[4] = {-36,54,-10,-4};
  int da13[4] = {12,-12,0,0};
  int da22[4] = {36,-36,-4,4};
  int da23[4] = {-12,6,2,0};
  int da33[4] = {4,0,0,0};

  Scalar da[4];
  for(int i = 0; i < 4; ++i)
  {
    da[i] = Rd0Rd0 * da00[i] + Rd0Rd1 * da01[i] + Rd0Rd2 * da02[i] + Rd0Rd3 * da03[i]
      + Rd0Rd1 * da01[i] + Rd1Rd1 * da11[i] + Rd1Rd2 * da12[i] + Rd1Rd3 * da13[i]
      + Rd0Rd2 * da02[i] + Rd1Rd2 * da12[i] + Rd2Rd2 * da22[i] + Rd2Rd3 * da23[i]
      + Rd0Rd3 * da03[i] + Rd1Rd3 * da13[i] + Rd2Rd3 * da23[i] + Rd3Rd3 * da33[i];
    da[i] /= 4.0;
  }

  Scalar roots[3];

  int root_num = PolySolverd::solveCubic(da, roots);

  Scalar dWdW_max = a[0] * tf_t * tf_t * tf_t + a[1] * tf_t * tf_t * tf_t + a[2] * tf_t * tf_t + a[3] * tf_t + a[4];
  Scalar dWdW_1 = a[0] + a[1] + a[2] + a[3] + a[4];
  if(dWdW_max < dWdW_1) dWdW_max = dWdW_1;
  for(int i = 0; i < root_num; ++i)
  {
    Scalar v = roots[i];

    if(v >= tf_t && v <= 1)
    {
      Scalar value = a[0] * v * v * v * v + a[1] * v * v * v + a[2] * v * v + a[3] * v + a[4];
      if(value > dWdW_max) dWdW_max = value;
    }
  }

  return sqrt(dWdW_max);
}

//==============================================================================
template <typename Scalar>
Scalar SplineMotion<Scalar>::getWeight0(Scalar t) const
{
  return (1 - 3 * t + 3 * t * t - t * t * t) / 6.0;
}

//==============================================================================
template <typename Scalar>
Scalar SplineMotion<Scalar>::getWeight1(Scalar t) const
{
  return (4 - 6 * t * t + 3 * t * t * t) / 6.0;
}

//==============================================================================
template <typename Scalar>
Scalar SplineMotion<Scalar>::getWeight2(Scalar t) const
{
  return (1 + 3 * t + 3 * t * t - 3 * t * t * t) / 6.0;
}

//==============================================================================
template <typename Scalar>
Scalar SplineMotion<Scalar>::getWeight3(Scalar t) const
{
  return t * t * t / 6.0;
}




//==============================================================================
template <typename Scalar>
InterpMotion<Scalar>::InterpMotion() : MotionBase<Scalar>(), angular_axis(Vector3<Scalar>::UnitX())
{
  // Default angular velocity is zero
  angular_vel = 0;

  // Default reference point is local zero point

  // Default linear velocity is zero
}

//==============================================================================
template <typename Scalar>
InterpMotion<Scalar>::InterpMotion(const Matrix3<Scalar>& R1, const Vector3<Scalar>& T1,
                           const Matrix3<Scalar>& R2, const Vector3<Scalar>& T2) : MotionBase<Scalar>(),
                                                                  tf1(Transform3<Scalar>::Identity()),
                                                                  tf2(Transform3<Scalar>::Identity())
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
template <typename Scalar>
InterpMotion<Scalar>::InterpMotion(const Transform3<Scalar>& tf1_, const Transform3<Scalar>& tf2_) : MotionBase<Scalar>(),
                                                                               tf1(tf1_),
                                                                               tf2(tf2_),
                                                                               tf(tf1)
{
  // Compute the velocities for the motion
  computeVelocity();
}

//==============================================================================
template <typename Scalar>
InterpMotion<Scalar>::InterpMotion(const Matrix3<Scalar>& R1, const Vector3<Scalar>& T1,
                           const Matrix3<Scalar>& R2, const Vector3<Scalar>& T2,
                           const Vector3<Scalar>& O) : MotionBase<Scalar>(),
                                             tf1(Transform3<Scalar>::Identity()),
                                             tf2(Transform3<Scalar>::Identity()),
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
template <typename Scalar>
InterpMotion<Scalar>::InterpMotion(const Transform3<Scalar>& tf1_, const Transform3<Scalar>& tf2_, const Vector3<Scalar>& O) : MotionBase<Scalar>(),
                                                                                               tf1(tf1_),
                                                                                               tf2(tf2_),
                                                                                               tf(tf1),
                                                                                               reference_p(O)
{
}

//==============================================================================
template <typename Scalar>
bool InterpMotion<Scalar>::integrate(double dt) const
{
  if(dt > 1) dt = 1;

  tf.linear() = absoluteRotation(dt).toRotationMatrix();
  tf.translation() = linear_vel * dt + tf1 * reference_p - tf.linear() * reference_p;

  return true;
}

//==============================================================================
template <typename Scalar>
void InterpMotion<Scalar>::computeVelocity()
{
  linear_vel = tf2 * reference_p - tf1 * reference_p;

  const Eigen::AngleAxisd aa(tf2.linear() * tf1.linear().transpose());
  angular_axis = aa.axis();
  angular_vel = aa.angle();

  if(angular_vel < 0)
  {
    angular_vel = -angular_vel;
    angular_axis = -angular_axis;
  }
}

//==============================================================================
template <typename Scalar>
Quaternion3<Scalar> InterpMotion<Scalar>::deltaRotation(Scalar dt) const
{
  return Quaternion3<Scalar>(Eigen::AngleAxisd((Scalar)(dt * angular_vel), angular_axis));
}

//==============================================================================
template <typename Scalar>
Quaternion3<Scalar> InterpMotion<Scalar>::absoluteRotation(Scalar dt) const
{
  Quaternion3<Scalar> delta_t = deltaRotation(dt);
  return delta_t * Quaternion3<Scalar>(tf1.linear());
}

} // namespace fcl

#endif

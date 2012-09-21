/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#include "fcl/ccd/motion_base.h"
#include "fcl/intersect.h"
#include <iostream>
#include <vector>

namespace fcl
{

class SplineMotion : public MotionBase
{
public:
  /// @brief Construct motion from 4 deBoor points
  SplineMotion(const Vec3f& Td0, const Vec3f& Td1, const Vec3f& Td2, const Vec3f& Td3,
               const Vec3f& Rd0, const Vec3f& Rd1, const Vec3f& Rd2, const Vec3f& Rd3);

  
  /// @brief Integrate the motion from 0 to dt
  /// We compute the current transformation from zero point instead of from last integrate time, for precision.
  bool integrate(double dt);

  /// @brief Compute the motion bound for a bounding volume along a given direction n, which is defined in the visitor
  FCL_REAL computeMotionBound(const BVMotionBoundVisitor& mb_visitor) const
  {
    return mb_visitor.visit(*this);
  }

  /// @brief Compute the motion bound for a triangle along a given direction n, which is defined in the visitor
  FCL_REAL computeMotionBound(const TriangleMotionBoundVisitor& mb_visitor) const
  {
    return mb_visitor.visit(*this);
  }

  /// @brief Get the rotation and translation in current step
  void getCurrentTransform(Matrix3f& R, Vec3f& T) const
  {
    R = tf.getRotation();
    T = tf.getTranslation();
  }

  void getCurrentRotation(Matrix3f& R) const
  {
    R = tf.getRotation();
  }

  void getCurrentTranslation(Vec3f& T) const
  {
    T = tf.getTranslation();
  }

  void getCurrentTransform(Transform3f& tf_) const
  {
    tf_ = tf;
  }

protected:
  void computeSplineParameter()
  {
  }

  FCL_REAL getWeight0(FCL_REAL t) const;
  FCL_REAL getWeight1(FCL_REAL t) const;
  FCL_REAL getWeight2(FCL_REAL t) const;
  FCL_REAL getWeight3(FCL_REAL t) const;
  
  Vec3f Td[4];
  Vec3f Rd[4];

  Vec3f TA, TB, TC;
  Vec3f RA, RB, RC;

  FCL_REAL Rd0Rd0, Rd0Rd1, Rd0Rd2, Rd0Rd3, Rd1Rd1, Rd1Rd2, Rd1Rd3, Rd2Rd2, Rd2Rd3, Rd3Rd3;
  //// @brief The transformation at current time t
  Transform3f tf;

  /// @brief The time related with tf
  FCL_REAL tf_t;

public:
  FCL_REAL computeTBound(const Vec3f& n) const;
  
  FCL_REAL computeDWMax() const;

  FCL_REAL getCurrentTime() const
  {
    return tf_t;
  }

};

class ScrewMotion : public MotionBase
{
public:
  /// @brief Default transformations are all identities
  ScrewMotion()
  {
    // Default angular velocity is zero
    axis.setValue(1, 0, 0);
    angular_vel = 0;

    // Default reference point is local zero point

    // Default linear velocity is zero
    linear_vel = 0;
  }

  /// @brief Construct motion from the initial rotation/translation and goal rotation/translation
  ScrewMotion(const Matrix3f& R1, const Vec3f& T1,
              const Matrix3f& R2, const Vec3f& T2) : tf1(R1, T1),
                                                     tf2(R2, T2),
                                                     tf(tf1)
  {
    computeScrewParameter();
  }

  /// @brief Construct motion from the initial transform and goal transform
  ScrewMotion(const Transform3f& tf1_,
              const Transform3f& tf2_) : tf1(tf1_),
                                         tf2(tf2_),
                                         tf(tf1)
  {
    computeScrewParameter();
  }

  /// @brief Integrate the motion from 0 to dt
  /// We compute the current transformation from zero point instead of from last integrate time, for precision.
  bool integrate(double dt)
  {
    if(dt > 1) dt = 1;

    tf.setQuatRotation(absoluteRotation(dt));

    Quaternion3f delta_rot = deltaRotation(dt);
    tf.setTranslation(p + axis * (dt * linear_vel) + delta_rot.transform(tf1.getTranslation() - p));

    return true;
  }

  /// @brief Compute the motion bound for a bounding volume along a given direction n, which is defined in the visitor
  FCL_REAL computeMotionBound(const BVMotionBoundVisitor& mb_visitor) const
  {
    return mb_visitor.visit(*this);
  }

  /// @brief Compute the motion bound for a triangle along a given direction n, which is defined in the visitor
  FCL_REAL computeMotionBound(const TriangleMotionBoundVisitor& mb_visitor) const
  {
    return mb_visitor.visit(*this);
  }


  /// @brief Get the rotation and translation in current step
  void getCurrentTransform(Matrix3f& R, Vec3f& T) const
  {
    R = tf.getRotation();
    T = tf.getTranslation();
  }

  void getCurrentRotation(Matrix3f& R) const
  {
    R = tf.getRotation();
  }

  void getCurrentTranslation(Vec3f& T) const
  {
    T = tf.getTranslation();
  }

  void getCurrentTransform(Transform3f& tf_) const
  {
    tf_ = tf;
  }

protected:
  void computeScrewParameter()
  {
    Quaternion3f deltaq = tf2.getQuatRotation() * inverse(tf1.getQuatRotation());
    deltaq.toAxisAngle(axis, angular_vel);
    if(angular_vel < 0)
    {
      angular_vel = -angular_vel;
      axis = -axis;
    }

    if(angular_vel < 1e-10)
    {
      angular_vel = 0;
      axis = tf2.getTranslation() - tf1.getTranslation();
      linear_vel = axis.length();
      p = tf1.getTranslation();
    }
    else
    {
      Vec3f o = tf2.getTranslation() - tf1.getTranslation();
      p = (tf1.getTranslation() + tf2.getTranslation() + axis.cross(o) * (1.0 / tan(angular_vel / 2.0))) * 0.5;
      linear_vel = o.dot(axis);
    }
  }

  Quaternion3f deltaRotation(FCL_REAL dt) const
  {
    Quaternion3f res;
    res.fromAxisAngle(axis, (FCL_REAL)(dt * angular_vel));
    return res;
  }

  Quaternion3f absoluteRotation(FCL_REAL dt) const
  {
    Quaternion3f delta_t = deltaRotation(dt);
    return delta_t * tf1.getQuatRotation();
  }

  /// @brief The transformation at time 0
  Transform3f tf1;

  /// @brief The transformation at time 1
  Transform3f tf2;

  /// @brief The transformation at current time t
  Transform3f tf;

  /// @brief screw axis
  Vec3f axis;

  /// @brief A point on the axis S
  Vec3f p;

  /// @brief linear velocity along the axis
  FCL_REAL linear_vel;

  /// @brief angular velocity
  FCL_REAL angular_vel;

public:

  inline FCL_REAL getLinearVelocity() const
  {
    return linear_vel;
  }

  inline FCL_REAL getAngularVelocity() const
  {
    return angular_vel;
  }

  inline const Vec3f& getAxis() const
  {
    return axis;
  }

  inline const Vec3f& getAxisOrigin() const
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
class InterpMotion : public MotionBase
{
public:
  /// @brief Default transformations are all identities
  InterpMotion();

  /// @brief Construct motion from the initial rotation/translation and goal rotation/translation
  InterpMotion(const Matrix3f& R1, const Vec3f& T1,
               const Matrix3f& R2, const Vec3f& T2);

  InterpMotion(const Transform3f& tf1_, const Transform3f& tf2_);

  /// @brief Construct motion from the initial rotation/translation and goal rotation/translation related to some rotation center
  InterpMotion(const Matrix3f& R1, const Vec3f& T1,
               const Matrix3f& R2, const Vec3f& T2,
               const Vec3f& O);

  InterpMotion(const Transform3f& tf1_, const Transform3f& tf2_, const Vec3f& O);

  /// @brief Integrate the motion from 0 to dt
  /// We compute the current transformation from zero point instead of from last integrate time, for precision.
  bool integrate(double dt);

  /// @brief Compute the motion bound for a bounding volume along a given direction n, which is defined in the visitor
  FCL_REAL computeMotionBound(const BVMotionBoundVisitor& mb_visitor) const
  {
    return mb_visitor.visit(*this);
  }

  /// @brief Compute the motion bound for a triangle along a given direction n, which is defined in the visitor 
  FCL_REAL computeMotionBound(const TriangleMotionBoundVisitor& mb_visitor) const
  {
    return mb_visitor.visit(*this);
  }

  /// @brief Get the rotation and translation in current step
  void getCurrentTransform(Matrix3f& R, Vec3f& T) const
  {
    R = tf.getRotation();
    T = tf.getTranslation();
  }

  void getCurrentRotation(Matrix3f& R) const
  {
    R = tf.getRotation();
  }

  void getCurrentTranslation(Vec3f& T) const
  {
    T = tf.getTranslation();
  }

  void getCurrentTransform(Transform3f& tf_) const
  {
    tf_ = tf;
  }

protected:

  void computeVelocity();

  Quaternion3f deltaRotation(FCL_REAL dt) const;
  
  Quaternion3f absoluteRotation(FCL_REAL dt) const;
  
  /// @brief The transformation at time 0
  Transform3f tf1;

  /// @brief The transformation at time 1
  Transform3f tf2;

  /// @brief The transformation at current time t
  Transform3f tf;

  /// @brief Linear velocity
  Vec3f linear_vel;

  /// @brief Angular speed
  FCL_REAL angular_vel;

  /// @brief Angular velocity axis
  Vec3f angular_axis;

  /// @brief Reference point for the motion (in the object's local frame)
  Vec3f reference_p;

public:
  const Vec3f& getReferencePoint() const
  {
    return reference_p;
  }

  const Vec3f& getAngularAxis() const
  {
    return angular_axis;
  }

  FCL_REAL getAngularVelocity() const
  {
    return angular_vel;
  }

  const Vec3f& getLinearVelocity() const
  {
    return linear_vel;
  }
};



}

#endif

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


#ifndef FCL_MOTION_H
#define FCL_MOTION_H

#include "fcl/vec_3f.h"
#include "fcl/RSS.h"
#include "fcl/transform.h"
#include "fcl/motion_base.h"
#include <iostream>
namespace fcl
{

template<typename BV>
class ScrewMotion : public MotionBase<BV>
{
public:
  /** Default transformations are all identities */
  ScrewMotion()
  {
    /** Default angular velocity is zero */
    axis = Vec3f(1, 0, 0);
    angular_vel = 0;

    /** Default reference point is local zero point */

    /** Default linear velocity is zero */
    linear_vel = 0;
  }

  /** \brief Construct motion from the initial rotation/translation and goal rotation/translation */
  ScrewMotion(const Vec3f R1[3], const Vec3f& T1,
              const Vec3f R2[3], const Vec3f& T2)
  {
    t1 = SimpleTransform(R1, T1);
    t2 = SimpleTransform(R2, T2);

    /** Current time is zero, so the transformation is t1 */
    t = t1;

    computeScrewParameter();
  }

  /** \brief Integrate the motion from 0 to dt
   * We compute the current transformation from zero point instead of from last integrate time, for precision.
   */
  bool integrate(double dt)
  {
    if(dt > 1) dt = 1;

    t.setQuatRotation(absoluteRotation(dt));
    t.setTranslation(p + axis * (dt * linear_vel) - t.getQuatRotation().transform(p));

    return true;
  }

  /** \brief Compute the motion bound for a bounding volume along a given direction n
   * For general BV, not implemented so return trivial 0
   */
  BVH_REAL computeMotionBound(const BV& bv, const Vec3f& n) const { return 0.0; }

  BVH_REAL computeMotionBound(const Vec3f& a, const Vec3f& b, const Vec3f& c, const Vec3f& n) const
  {
    BVH_REAL proj_max = ((t1.getQuatRotation().transform(a) + t1.getTranslation() - p).cross(axis)).sqrLength();
    BVH_REAL tmp;
    tmp = ((t1.getQuatRotation().transform(b) + t1.getTranslation() - p).cross(axis)).sqrLength();
    if(tmp > proj_max) proj_max = tmp;
    tmp = ((t1.getQuatRotation().transform(c) + t1.getTranslation() - p).cross(axis)).sqrLength();
    if(tmp > proj_max) proj_max = tmp;

    proj_max = sqrt(proj_max);

    BVH_REAL v_dot_n = axis.dot(n) * linear_vel;
    BVH_REAL w_cross_n = (axis.cross(n)).length() * angular_vel;
    BVH_REAL mu = v_dot_n + w_cross_n * proj_max;

    return mu;
  }

  /** \brief Get the rotation and translation in current step */
  void getCurrentTransform(Vec3f R[3], Vec3f& T) const
  {
    for(int i = 0; i < 3; ++i)
    {
      R[i] = t.getRotation()[i];
    }

    T = t.getTranslation();
  }

  void getCurrentRotation(Vec3f R[3]) const
  {
    for(int i = 0; i < 3; ++i)
      R[i] = t.getRotation()[i];
  }

  void getCurrentTranslation(Vec3f& T) const
  {
    T = t.getTranslation();
  }

protected:
  void computeScrewParameter()
  {
    SimpleQuaternion deltaq = t2.getQuatRotation() * t1.getQuatRotation().inverse();
    deltaq.toAxisAngle(axis, angular_vel);
    if(angular_vel < 0)
    {
      angular_vel = -angular_vel;
      axis = -axis;
    }

    if(angular_vel < 1e-10)
    {
      angular_vel = 0;
      axis = t2.getTranslation() - t1.getTranslation();
      linear_vel = axis.length();
      p = t1.getTranslation();
    }
    else
    {
      Vec3f o = t2.getTranslation() - t1.getTranslation();
      p = (t1.getTranslation() + t2.getTranslation() + axis.cross(o) * (1.0 / tan(angular_vel / 2.0))) * 0.5;
      linear_vel = o.dot(axis);
    }
  }

  SimpleQuaternion deltaRotation(BVH_REAL dt) const
  {
    SimpleQuaternion res;
    res.fromAxisAngle(axis, (BVH_REAL)(dt * angular_vel));
    return res;
  }

  SimpleQuaternion absoluteRotation(BVH_REAL dt) const
  {
    SimpleQuaternion delta_t = deltaRotation(dt);
    return delta_t * t1.getQuatRotation();
  }

  /** \brief The transformation at time 0 */
  SimpleTransform t1;

  /** \brief The transformation at time 1 */
  SimpleTransform t2;

  /** \brief The transformation at current time t */
  SimpleTransform t;

  /** \brief screw axis */
  Vec3f axis;

  /** \brief A point on the axis S */
  Vec3f p;

  /** \brief linear velocity along the axis */
  BVH_REAL linear_vel;

  /** \brief angular velocity */
  BVH_REAL angular_vel;
};


/** \brief Compute the motion bound for a bounding volume along a given direction n
 * according to mu < |v * n| + ||w x n||(r + max(||ci*||)) where ||ci*|| = ||R0(ci) x w||. w is the angular axis (normalized)
 * and ci are the endpoints of the generator primitives of RSS.
 * Notice that all bv parameters are in the local frame of the object, but n should be in the global frame (the reason is that the motion (t1, t2 and t) is in global frame)
 */
template<>
BVH_REAL ScrewMotion<RSS>::computeMotionBound(const RSS& bv, const Vec3f& n) const;


/** \brief Linear interpolation motion
 * Each Motion is assumed to have constant linear velocity and angular velocity
 * The motion is R(t)(p - p_ref) + p_ref + T(t)
 * Therefore, R(0) = R0, R(1) = R1
 *            T(0) = T0 + R0 p_ref - p_ref
 *            T(1) = T1 + R1 p_ref - p_ref
 */
template<typename BV>
class InterpMotion : public MotionBase<BV>
{
public:
  /** \brief Default transformations are all identities */
  InterpMotion()
  {
    /** Default angular velocity is zero */
    angular_axis = Vec3f(1, 0, 0);
    angular_vel = 0;

    /** Default reference point is local zero point */

    /** Default linear velocity is zero */
  }

  /** \brief Construct motion from the initial rotation/translation and goal rotation/translation */
  InterpMotion(const Vec3f R1[3], const Vec3f& T1,
               const Vec3f R2[3], const Vec3f& T2)
  {
    t1 = SimpleTransform(R1, T1);
    t2 = SimpleTransform(R2, T2);

    /** Current time is zero, so the transformation is t1 */
    t = t1;

    /** Default reference point is local zero point */

    /** Compute the velocities for the motion */
    computeVelocity();
  }

  /** \brief Construct motion from the initial rotation/translation and goal rotation/translation related to some rotation center
   */
  InterpMotion(const Vec3f R1[3], const Vec3f& T1,
               const Vec3f R2[3], const Vec3f& T2,
               const Vec3f& O)
  {
    t1 = SimpleTransform(R1, T1);
    t2 = SimpleTransform(R2, T2);
    t = t1;

    reference_p = O;

    /** Compute the velocities for the motion */
    computeVelocity();
  }


  /** \brief Integrate the motion from 0 to dt
   * We compute the current transformation from zero point instead of from last integrate time, for precision.
   */
  bool integrate(double dt)
  {
    if(dt > 1) dt = 1;

    t.setQuatRotation(absoluteRotation(dt));
    t.setTranslation(linear_vel * dt + t1.transform(reference_p) - t.getQuatRotation().transform(reference_p));

    return true;
  }

  /** \brief Compute the motion bound for a bounding volume along a given direction n
   * For general BV, not implemented so return trivial 0
   */
  BVH_REAL computeMotionBound(const BV& bv, const Vec3f& n) const { return 0.0; }

  /** \brief Compute the motion bound for a triangle along a given direction n
   * according to mu < |v * n| + ||w x n||(max||ci*||) where ||ci*|| = ||R0(ci) x w|| / \|w\|. w is the angular velocity
   * and ci are the triangle vertex coordinates.
   * Notice that the triangle is in the local frame of the object, but n should be in the global frame (the reason is that the motion (t1, t2 and t) is in global frame)
   */
  BVH_REAL computeMotionBound(const Vec3f& a, const Vec3f& b, const Vec3f& c, const Vec3f& n) const
  {
    BVH_REAL proj_max = ((t1.getQuatRotation().transform(a - reference_p)).cross(angular_axis)).sqrLength();
    BVH_REAL tmp;
    tmp = ((t1.getQuatRotation().transform(b - reference_p)).cross(angular_axis)).sqrLength();
    if(tmp > proj_max) proj_max = tmp;
    tmp = ((t1.getQuatRotation().transform(c - reference_p)).cross(angular_axis)).sqrLength();
    if(tmp > proj_max) proj_max = tmp;

    proj_max = sqrt(proj_max);

    BVH_REAL v_dot_n = linear_vel.dot(n);
    BVH_REAL w_cross_n = (angular_axis.cross(n)).length() * angular_vel;
    BVH_REAL mu = v_dot_n + w_cross_n * proj_max;

    return mu;
  }

  /** \brief Get the rotation and translation in current step */
  void getCurrentTransform(Vec3f R[3], Vec3f& T) const
  {
    for(int i = 0; i < 3; ++i)
    {
      R[i] = t.getRotation()[i];
    }

    T = t.getTranslation();
  }

  void getCurrentRotation(Vec3f R[3]) const
  {
    for(int i = 0; i < 3; ++i)
      R[i] = t.getRotation()[i];
  }

  void getCurrentTranslation(Vec3f& T) const
  {
    T = t.getTranslation();
  }

protected:

  void computeVelocity()
  {
    linear_vel = t2.transform(reference_p) - t1.transform(reference_p);
    SimpleQuaternion deltaq = t2.getQuatRotation() * t1.getQuatRotation().inverse();
    deltaq.toAxisAngle(angular_axis, angular_vel);
    if(angular_vel < 0)
    {
      angular_vel = -angular_vel;
      angular_axis = -angular_axis;
    }
  }


  SimpleQuaternion deltaRotation(BVH_REAL dt) const
  {
    SimpleQuaternion res;
    res.fromAxisAngle(angular_axis, (BVH_REAL)(dt * angular_vel));
    return res;
  }

  SimpleQuaternion absoluteRotation(BVH_REAL dt) const
  {
    SimpleQuaternion delta_t = deltaRotation(dt);
    return delta_t * t1.getQuatRotation();
  }

  /** \brief The transformation at time 0 */
  SimpleTransform t1;

  /** \brief The transformation at time 1 */
  SimpleTransform t2;

  /** \brief The transformation at current time t */
  SimpleTransform t;

  /** \brief Linear velocity */
  Vec3f linear_vel;

  /** \brief Angular speed */
  BVH_REAL angular_vel;

  /** \brief Angular velocity axis */
  Vec3f angular_axis;

  /** \brief Reference point for the motion (in the object's local frame) */
  Vec3f reference_p;
};


/** \brief Compute the motion bound for a bounding volume along a given direction n
 * according to mu < |v * n| + ||w x n||(r + max(||ci*||)) where ||ci*|| = ||R0(ci) x w||. w is the angular axis (normalized)
 * and ci are the endpoints of the generator primitives of RSS.
 * Notice that all bv parameters are in the local frame of the object, but n should be in the global frame (the reason is that the motion (t1, t2 and t) is in global frame)
 */
template<>
BVH_REAL InterpMotion<RSS>::computeMotionBound(const RSS& bv, const Vec3f& n) const;


}

#endif

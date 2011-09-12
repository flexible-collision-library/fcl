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

namespace fcl
{

/** \brief Linear interpolation motion
 * Each Motion is assumed to have constant linear velocity and angular velocity
 */
template<typename BV>
class InterpMotion : public MotionBase<BV>
{
public:
  /** Default transformations are all identities */
  InterpMotion()
  {
    /** Default angular velocity is zero */
    angular_axis = Vec3f(1, 0, 0);
    angular_vel = 0;

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

    /** Compute the velocities for the motion */
    computeVelocity();
  }

  /** \brief Construct motion from the initial rotation/translation and goal rotation/translation related to some rotation center
   */
  InterpMotion(const Vec3f R1[3], const Vec3f& T1,
               const Vec3f R2[3], const Vec3f& T2,
               const Vec3f& O)
  {
    t1 = SimpleTransform(R1, T1 - matMulVec(R1, O));
    t2 = SimpleTransform(R2, T2 - matMulVec(R2, O));
    t = t1;

    /** Compute the velocities for the motion */
    computeVelocity();
  }


  /** \brief Integrate the motion from 0 to dt
   * We compute the current transformation from zero point instead of from last integrate time, for precision.
   */
  bool integrate(double dt)
  {
    if(dt > 1) dt = 1;

    t.T = t1.T + linear_vel * dt;

    t.q = absoluteRotation(dt);
    t.q.toRotation(t.R);

    return true;
  }

  /** \brief Compute the motion bound for a bounding volume, given the closest direction n between two query objects
   * according to mu < |v * n| + ||w x n||(r + max(||ci*||)) where ||ci*|| = ||ci x w||. w is the angular axis (normalized)
   * and ci are the endpoints of the generator primitives of RSS.
   * Notice that all bv parameters are in the local frame of the object, but n should be in the global frame (the reason is that the motion (t1, t2 and t) is in global frame)
   */
  BVH_REAL computeMotionBound(const BV& bv, const Vec3f& n) const { return 0.0; }

  /** \brief Compute the motion bound for a triangle, given the closest direction n between two query objects
   * according to mu < |v * | + ||w x n||(max||ci*||) where ||ci*|| = ||ci x w||. w is the angular axis (normalized)
   * and ci are the triangle vertex coordinates.
   * Notice that the triangle is in the local frame of the object, but n should be in the global frame (the reason is that the motion (t1, t2 and t) is in global frame)
   */
  BVH_REAL computeMotionBound(const Vec3f& a, const Vec3f& b, const Vec3f& c, const Vec3f& n) const
  {
    BVH_REAL c_proj_max = (a.cross(angular_axis)).sqrLength();
    BVH_REAL tmp;
    tmp = (b.cross(angular_axis)).sqrLength();
    if(tmp > c_proj_max) c_proj_max = tmp;
    tmp = (c.cross(angular_axis)).sqrLength();
    if(tmp > c_proj_max) c_proj_max = tmp;

    c_proj_max = sqrt(c_proj_max);

    BVH_REAL v_dot_n = linear_vel.dot(n);
    BVH_REAL w_cross_n = (angular_axis.cross(n)).length() * angular_vel;
    BVH_REAL mu = v_dot_n + w_cross_n * c_proj_max;

    return mu;
  }

  /** \brief Get the rotation and translation in current step */
  void getCurrentTransformation(Vec3f R[3], Vec3f& T) const
  {
    for(int i = 0; i < 3; ++i)
    {
      R[i] = t.R[i];
    }

    T = t.T;
  }

  void getCurrentRotation(Vec3f R[3]) const
  {
    for(int i = 0; i < 3; ++i)
      R[i] = t.R[i];
  }

  void getCurrentTranslation(Vec3f& T) const
  {
    T = t.T;
  }

protected:

  void computeVelocity()
  {
    linear_vel = t2.T - t1.T;
    SimpleQuaternion deltaq = t2.q * t1.q.inverse();
    deltaq.toAxisAngle(angular_axis, angular_vel);
  }


  SimpleQuaternion deltaRotation(BVH_REAL t) const
  {
    SimpleQuaternion res;
    res.fromAxisAngle(angular_axis, (BVH_REAL)(t * angular_vel));
    return res;
  }

  SimpleQuaternion absoluteRotation(BVH_REAL t) const
  {
    SimpleQuaternion delta_t = deltaRotation(t);
    return delta_t * t1.q;
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
};

template<>
BVH_REAL InterpMotion<RSS>::computeMotionBound(const RSS& bv, const Vec3f& n) const;


}

#endif

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
#include "fcl/intersect.h"
#include <iostream>
#include <vector>

namespace fcl
{


template<typename BV>
class SplineMotion : public MotionBase<BV>
{
public:
  /** \brief Construct motion from 4 deBoor points */
  SplineMotion(const Vec3f& Td0, const Vec3f& Td1, const Vec3f& Td2, const Vec3f& Td3,
               const Vec3f& Rd0, const Vec3f& Rd1, const Vec3f& Rd2, const Vec3f& Rd3)
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

  /** \brief Integrate the motion from 0 to dt
   * We compute the current transformation from zero point instead of from last integrate time, for precision.
   */
  bool integrate(double dt)
  {
    if(dt > 1) dt = 1;

    Vec3f cur_T = Td[0] * getWeight0(dt) + Td[1] * getWeight1(dt) + Td[2] * getWeight2(dt) + Td[3] * getWeight3(dt);
    Vec3f cur_w = Rd[0] * getWeight0(dt) + Rd[1] * getWeight1(dt) + Rd[2] * getWeight2(dt) + Rd[3] * getWeight3(dt);
    BVH_REAL cur_angle = cur_w.length();
    cur_w.normalize();

    SimpleQuaternion cur_q;
    cur_q.fromAxisAngle(cur_w, cur_angle);

    tf.setTransform(cur_q, cur_T);

    tf_t = dt;

    return true;
  }

  /** \brief Compute the motion bound for a bounding volume along a given direction n
   * For general BV, not implemented so return trivial 0
   */
  BVH_REAL computeMotionBound(const BV& bv, const Vec3f& n) const { return 0.0; }

  BVH_REAL computeMotionBound(const Vec3f& a, const Vec3f& b, const Vec3f& c, const Vec3f& n) const
  {
    BVH_REAL T_bound = computeTBound(n);

    BVH_REAL R_bound = fabs(a.dot(n)) + a.length() + (a.cross(n)).length();
    BVH_REAL R_bound_tmp = fabs(b.dot(n)) + b.length() + (b.cross(n)).length();
    if(R_bound_tmp > R_bound) R_bound = R_bound_tmp;
    R_bound_tmp = fabs(c.dot(n)) + c.length() + (c.cross(n)).length();
    if(R_bound_tmp > R_bound) R_bound = R_bound_tmp;

    BVH_REAL dWdW_max = computeDWMax();
    BVH_REAL ratio = std::min(1 - tf_t, dWdW_max);

    R_bound *= 2 * ratio;

    // std::cout << R_bound << " " << T_bound << std::endl;

    return R_bound + T_bound;
  }

  /** \brief Get the rotation and translation in current step */
  void getCurrentTransform(Vec3f R[3], Vec3f& T) const
  {
    for(int i = 0; i < 3; ++i)
    {
      R[i] = tf.getRotation()[i];
    }

    T = tf.getTranslation();
  }

  void getCurrentRotation(Vec3f R[3]) const
  {
    for(int i = 0; i < 3; ++i)
      R[i] = tf.getRotation()[i];
  }

  void getCurrentTranslation(Vec3f& T) const
  {
    T = tf.getTranslation();
  }

protected:
  void computeSplineParameter()
  {

  }

  BVH_REAL getWeight0(BVH_REAL t) const
  {
    return (1 - 3 * t + 3 * t * t - t * t * t) / 6.0;
  }

  BVH_REAL getWeight1(BVH_REAL t) const
  {
    return (4 - 6 * t * t + 3 * t * t * t) / 6.0;
  }

  BVH_REAL getWeight2(BVH_REAL t) const
  {
    return (1 + 3 * t + 3 * t * t - 3 * t * t * t) / 6.0;
  }

  BVH_REAL getWeight3(BVH_REAL t) const
  {
    return t * t * t / 6.0;
  }

  BVH_REAL computeTBound(const Vec3f& n) const
  {
    BVH_REAL Ta = TA.dot(n);
    BVH_REAL Tb = TB.dot(n);
    BVH_REAL Tc = TC.dot(n);

    std::vector<BVH_REAL> T_potential;
    T_potential.push_back(tf_t);
    T_potential.push_back(1);
    if(Tb * Tb - 3 * Ta * Tc >= 0)
    {
      if(Ta == 0)
      {
        if(Tb != 0)
        {
          BVH_REAL tmp = -Tc / (2 * Tb);
          if(tmp < 1 && tmp > tf_t)
            T_potential.push_back(tmp);
        }
      }
      else
      {
        BVH_REAL tmp_delta = sqrt(Tb * Tb - 3 * Ta * Tc);
        BVH_REAL tmp1 = (-Tb + tmp_delta) / (3 * Ta);
        BVH_REAL tmp2 = (-Tb - tmp_delta) / (3 * Ta);
        if(tmp1 < 1 && tmp1 > tf_t)
          T_potential.push_back(tmp1);
        if(tmp2 < 1 && tmp2 > tf_t)
          T_potential.push_back(tmp2);
      }
    }

    BVH_REAL T_bound = Ta * T_potential[0] * T_potential[0] * T_potential[0] + Tb * T_potential[0] * T_potential[0] + Tc * T_potential[0];
    for(unsigned int i = 1; i < T_potential.size(); ++i)
    {
      BVH_REAL T_bound_tmp = Ta * T_potential[i] * T_potential[i] * T_potential[i] + Tb * T_potential[i] * T_potential[i] + Tc * T_potential[i];
      if(T_bound_tmp > T_bound) T_bound = T_bound_tmp;
    }


    BVH_REAL cur_delta = Ta * tf_t * tf_t * tf_t + Tb * tf_t * tf_t + Tc * tf_t;

    T_bound -= cur_delta;
    T_bound /= 6.0;

    return T_bound;
  }

  BVH_REAL computeDWMax() const
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

    BVH_REAL a[5];

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

    BVH_REAL da[4];
    for(int i = 0; i < 4; ++i)
    {
      da[i] = Rd0Rd0 * da00[i] + Rd0Rd1 * da01[i] + Rd0Rd2 * da02[i] + Rd0Rd3 * da03[i]
            + Rd0Rd1 * da01[i] + Rd1Rd1 * da11[i] + Rd1Rd2 * da12[i] + Rd1Rd3 * da13[i]
            + Rd0Rd2 * da02[i] + Rd1Rd2 * da12[i] + Rd2Rd2 * da22[i] + Rd2Rd3 * da23[i]
            + Rd0Rd3 * da03[i] + Rd1Rd3 * da13[i] + Rd2Rd3 * da23[i] + Rd3Rd3 * da33[i];
      da[i] /= 4.0;
    }

    BVH_REAL roots[3];

    int root_num = PolySolver::solveCubic(da, roots);

    BVH_REAL dWdW_max = a[0] * tf_t * tf_t * tf_t + a[1] * tf_t * tf_t * tf_t + a[2] * tf_t * tf_t + a[3] * tf_t + a[4];
    BVH_REAL dWdW_1 = a[0] + a[1] + a[2] + a[3] + a[4];
    if(dWdW_max < dWdW_1) dWdW_max = dWdW_1;
    for(int i = 0; i < root_num; ++i)
    {
      BVH_REAL v = roots[i];

      if(v >= tf_t && v <= 1)
      {
        BVH_REAL value = a[0] * v * v * v * v + a[1] * v * v * v + a[2] * v * v + a[3] * v + a[4];
        if(value > dWdW_max) dWdW_max = value;
      }
    }

    return sqrt(dWdW_max);
  }

  Vec3f Td[4];
  Vec3f Rd[4];

  Vec3f TA, TB, TC;
  Vec3f RA, RB, RC;

  BVH_REAL Rd0Rd0, Rd0Rd1, Rd0Rd2, Rd0Rd3, Rd1Rd1, Rd1Rd2, Rd1Rd3, Rd2Rd2, Rd2Rd3, Rd3Rd3;
  /** \brief The transformation at current time t */
  SimpleTransform tf;

  /** \brief The time related with tf */
  BVH_REAL tf_t;
};

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
    tf1 = SimpleTransform(R1, T1);
    tf2 = SimpleTransform(R2, T2);

    /** Current time is zero, so the transformation is t1 */
    tf = tf1;

    computeScrewParameter();
  }

  /** \brief Integrate the motion from 0 to dt
   * We compute the current transformation from zero point instead of from last integrate time, for precision.
   */
  bool integrate(double dt)
  {
    if(dt > 1) dt = 1;

    tf.setQuatRotation(absoluteRotation(dt));

    SimpleQuaternion delta_rot = deltaRotation(dt);
    tf.setTranslation(p + axis * (dt * linear_vel) + delta_rot.transform(tf1.getTranslation() - p));

    return true;
  }

  /** \brief Compute the motion bound for a bounding volume along a given direction n
   * For general BV, not implemented so return trivial 0
   */
  BVH_REAL computeMotionBound(const BV& bv, const Vec3f& n) const { return 0.0; }

  BVH_REAL computeMotionBound(const Vec3f& a, const Vec3f& b, const Vec3f& c, const Vec3f& n) const
  {
    BVH_REAL proj_max = ((tf.getQuatRotation().transform(a) + tf.getTranslation() - p).cross(axis)).sqrLength();
    BVH_REAL tmp;
    tmp = ((tf.getQuatRotation().transform(b) + tf.getTranslation() - p).cross(axis)).sqrLength();
    if(tmp > proj_max) proj_max = tmp;
    tmp = ((tf.getQuatRotation().transform(c) + tf.getTranslation() - p).cross(axis)).sqrLength();
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
      R[i] = tf.getRotation()[i];
    }

    T = tf.getTranslation();
  }

  void getCurrentRotation(Vec3f R[3]) const
  {
    for(int i = 0; i < 3; ++i)
      R[i] = tf.getRotation()[i];
  }

  void getCurrentTranslation(Vec3f& T) const
  {
    T = tf.getTranslation();
  }

protected:
  void computeScrewParameter()
  {
    SimpleQuaternion deltaq = tf2.getQuatRotation() * tf1.getQuatRotation().inverse();
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

  SimpleQuaternion deltaRotation(BVH_REAL dt) const
  {
    SimpleQuaternion res;
    res.fromAxisAngle(axis, (BVH_REAL)(dt * angular_vel));
    return res;
  }

  SimpleQuaternion absoluteRotation(BVH_REAL dt) const
  {
    SimpleQuaternion delta_t = deltaRotation(dt);
    return delta_t * tf1.getQuatRotation();
  }

  /** \brief The transformation at time 0 */
  SimpleTransform tf1;

  /** \brief The transformation at time 1 */
  SimpleTransform tf2;

  /** \brief The transformation at current time t */
  SimpleTransform tf;

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
    tf1 = SimpleTransform(R1, T1);
    tf2 = SimpleTransform(R2, T2);

    /** Current time is zero, so the transformation is t1 */
    tf = tf1;

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
    tf1 = SimpleTransform(R1, T1);
    tf2 = SimpleTransform(R2, T2);
    tf = tf1;

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

    tf.setQuatRotation(absoluteRotation(dt));
    tf.setTranslation(linear_vel * dt + tf1.transform(reference_p) - tf.getQuatRotation().transform(reference_p));

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
    BVH_REAL proj_max = ((tf.getQuatRotation().transform(a - reference_p)).cross(angular_axis)).sqrLength();
    BVH_REAL tmp;
    tmp = ((tf.getQuatRotation().transform(b - reference_p)).cross(angular_axis)).sqrLength();
    if(tmp > proj_max) proj_max = tmp;
    tmp = ((tf.getQuatRotation().transform(c - reference_p)).cross(angular_axis)).sqrLength();
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
      R[i] = tf.getRotation()[i];
    }

    T = tf.getTranslation();
  }

  void getCurrentRotation(Vec3f R[3]) const
  {
    for(int i = 0; i < 3; ++i)
      R[i] = tf.getRotation()[i];
  }

  void getCurrentTranslation(Vec3f& T) const
  {
    T = tf.getTranslation();
  }

protected:

  void computeVelocity()
  {
    linear_vel = tf2.transform(reference_p) - tf1.transform(reference_p);
    SimpleQuaternion deltaq = tf2.getQuatRotation() * tf1.getQuatRotation().inverse();
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
    return delta_t * tf1.getQuatRotation();
  }

  /** \brief The transformation at time 0 */
  SimpleTransform tf1;

  /** \brief The transformation at time 1 */
  SimpleTransform tf2;

  /** \brief The transformation at current time t */
  SimpleTransform tf;

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

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


#include "fcl/ccd/motion.h"

namespace fcl
{

template<>
FCL_REAL TBVMotionBoundVisitor<RSS>::visit(const SplineMotion& motion) const
{
  FCL_REAL T_bound = motion.computeTBound(n);
  FCL_REAL tf_t = motion.getCurrentTime();

  Vec3f c1 = bv.Tr;
  Vec3f c2 = bv.Tr + bv.axis[0] * bv.l[0];
  Vec3f c3 = bv.Tr + bv.axis[1] * bv.l[1];
  Vec3f c4 = bv.Tr + bv.axis[0] * bv.l[0] + bv.axis[1] * bv.l[1];

  FCL_REAL tmp;
  // max_i |c_i * n|
  FCL_REAL cn_max = std::abs(c1.dot(n));
  tmp = std::abs(c2.dot(n));
  if(tmp > cn_max) cn_max = tmp;
  tmp = std::abs(c3.dot(n));
  if(tmp > cn_max) cn_max = tmp;
  tmp = std::abs(c4.dot(n));
  if(tmp > cn_max) cn_max = tmp;

  // max_i ||c_i||
  FCL_REAL cmax = c1.sqrLength();
  tmp = c2.sqrLength();
  if(tmp > cmax) cmax = tmp;
  tmp = c3.sqrLength();
  if(tmp > cmax) cmax = tmp;
  tmp = c4.sqrLength();
  if(tmp > cmax) cmax = tmp;
  cmax = sqrt(cmax);

  // max_i ||c_i x n||
  FCL_REAL cxn_max = (c1.cross(n)).sqrLength();
  tmp = (c2.cross(n)).sqrLength();
  if(tmp > cxn_max) cxn_max = tmp;
  tmp = (c3.cross(n)).sqrLength();
  if(tmp > cxn_max) cxn_max = tmp;
  tmp = (c4.cross(n)).sqrLength();
  if(tmp > cxn_max) cxn_max = tmp;
  cxn_max = sqrt(cxn_max);

  FCL_REAL dWdW_max = motion.computeDWMax();
  FCL_REAL ratio = std::min(1 - tf_t, dWdW_max);

  FCL_REAL R_bound = 2 * (cn_max + cmax + cxn_max + 3 * bv.r) * ratio;


  // std::cout << R_bound << " " << T_bound << std::endl;

  return R_bound + T_bound;  
}


FCL_REAL TriangleMotionBoundVisitor::visit(const SplineMotion& motion) const
{
  FCL_REAL T_bound = motion.computeTBound(n);
  FCL_REAL tf_t = motion.getCurrentTime();

  FCL_REAL R_bound = std::abs(a.dot(n)) + a.length() + (a.cross(n)).length();
  FCL_REAL R_bound_tmp = std::abs(b.dot(n)) + b.length() + (b.cross(n)).length();
  if(R_bound_tmp > R_bound) R_bound = R_bound_tmp;
  R_bound_tmp = std::abs(c.dot(n)) + c.length() + (c.cross(n)).length();
  if(R_bound_tmp > R_bound) R_bound = R_bound_tmp;

  FCL_REAL dWdW_max = motion.computeDWMax();
  FCL_REAL ratio = std::min(1 - tf_t, dWdW_max);

  R_bound *= 2 * ratio;

  // std::cout << R_bound << " " << T_bound << std::endl;

  return R_bound + T_bound;
}

SplineMotion::SplineMotion(const Vec3f& Td0, const Vec3f& Td1, const Vec3f& Td2, const Vec3f& Td3,
                           const Vec3f& Rd0, const Vec3f& Rd1, const Vec3f& Rd2, const Vec3f& Rd3) : MotionBase()
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

bool SplineMotion::integrate(double dt) const
{
  if(dt > 1) dt = 1;

  Vec3f cur_T = Td[0] * getWeight0(dt) + Td[1] * getWeight1(dt) + Td[2] * getWeight2(dt) + Td[3] * getWeight3(dt);
  Vec3f cur_w = Rd[0] * getWeight0(dt) + Rd[1] * getWeight1(dt) + Rd[2] * getWeight2(dt) + Rd[3] * getWeight3(dt);
  FCL_REAL cur_angle = cur_w.length();
  cur_w.normalize();

  Quaternion3f cur_q;
  cur_q.fromAxisAngle(cur_w, cur_angle);

  tf.setTransform(cur_q, cur_T);

  tf_t = dt;

  return true;
}


FCL_REAL SplineMotion::computeTBound(const Vec3f& n) const
{
  FCL_REAL Ta = TA.dot(n);
  FCL_REAL Tb = TB.dot(n);
  FCL_REAL Tc = TC.dot(n);

  std::vector<FCL_REAL> T_potential;
  T_potential.push_back(tf_t);
  T_potential.push_back(1);
  if(Tb * Tb - 3 * Ta * Tc >= 0)
  {
    if(Ta == 0)
    {
      if(Tb != 0)
      {
        FCL_REAL tmp = -Tc / (2 * Tb);
        if(tmp < 1 && tmp > tf_t)
          T_potential.push_back(tmp);
      }
    }
    else
    {
      FCL_REAL tmp_delta = sqrt(Tb * Tb - 3 * Ta * Tc);
      FCL_REAL tmp1 = (-Tb + tmp_delta) / (3 * Ta);
      FCL_REAL tmp2 = (-Tb - tmp_delta) / (3 * Ta);
      if(tmp1 < 1 && tmp1 > tf_t)
        T_potential.push_back(tmp1);
      if(tmp2 < 1 && tmp2 > tf_t)
        T_potential.push_back(tmp2);
    }
  }

  FCL_REAL T_bound = Ta * T_potential[0] * T_potential[0] * T_potential[0] + Tb * T_potential[0] * T_potential[0] + Tc * T_potential[0];
  for(unsigned int i = 1; i < T_potential.size(); ++i)
  {
    FCL_REAL T_bound_tmp = Ta * T_potential[i] * T_potential[i] * T_potential[i] + Tb * T_potential[i] * T_potential[i] + Tc * T_potential[i];
    if(T_bound_tmp > T_bound) T_bound = T_bound_tmp;
  }


  FCL_REAL cur_delta = Ta * tf_t * tf_t * tf_t + Tb * tf_t * tf_t + Tc * tf_t;

  T_bound -= cur_delta;
  T_bound /= 6.0;

  return T_bound;
}


FCL_REAL SplineMotion::computeDWMax() const
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

  FCL_REAL a[5];

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

  FCL_REAL da[4];
  for(int i = 0; i < 4; ++i)
  {
    da[i] = Rd0Rd0 * da00[i] + Rd0Rd1 * da01[i] + Rd0Rd2 * da02[i] + Rd0Rd3 * da03[i]
      + Rd0Rd1 * da01[i] + Rd1Rd1 * da11[i] + Rd1Rd2 * da12[i] + Rd1Rd3 * da13[i]
      + Rd0Rd2 * da02[i] + Rd1Rd2 * da12[i] + Rd2Rd2 * da22[i] + Rd2Rd3 * da23[i]
      + Rd0Rd3 * da03[i] + Rd1Rd3 * da13[i] + Rd2Rd3 * da23[i] + Rd3Rd3 * da33[i];
    da[i] /= 4.0;
  }

  FCL_REAL roots[3];

  int root_num = PolySolver::solveCubic(da, roots);

  FCL_REAL dWdW_max = a[0] * tf_t * tf_t * tf_t + a[1] * tf_t * tf_t * tf_t + a[2] * tf_t * tf_t + a[3] * tf_t + a[4];
  FCL_REAL dWdW_1 = a[0] + a[1] + a[2] + a[3] + a[4];
  if(dWdW_max < dWdW_1) dWdW_max = dWdW_1;
  for(int i = 0; i < root_num; ++i)
  {
    FCL_REAL v = roots[i];

    if(v >= tf_t && v <= 1)
    {
      FCL_REAL value = a[0] * v * v * v * v + a[1] * v * v * v + a[2] * v * v + a[3] * v + a[4];
      if(value > dWdW_max) dWdW_max = value;
    }
  }

  return sqrt(dWdW_max);
}

FCL_REAL SplineMotion::getWeight0(FCL_REAL t) const
{
  return (1 - 3 * t + 3 * t * t - t * t * t) / 6.0;
}

FCL_REAL SplineMotion::getWeight1(FCL_REAL t) const
{
  return (4 - 6 * t * t + 3 * t * t * t) / 6.0;
}

FCL_REAL SplineMotion::getWeight2(FCL_REAL t) const
{
  return (1 + 3 * t + 3 * t * t - 3 * t * t * t) / 6.0;
}

FCL_REAL SplineMotion::getWeight3(FCL_REAL t) const
{
  return t * t * t / 6.0;
}




/// @brief Compute the motion bound for a bounding volume along a given direction n
/// according to mu < |v * n| + ||w x n||(r + max(||ci*||)) where ||ci*|| = ||R0(ci) x w||. w is the angular axis (normalized)
/// and ci are the endpoints of the generator primitives of RSS.
/// Notice that all bv parameters are in the local frame of the object, but n should be in the global frame (the reason is that the motion (t1, t2 and t) is in global frame)
template<>
FCL_REAL TBVMotionBoundVisitor<RSS>::visit(const ScrewMotion& motion) const
{
  Transform3f tf;
  motion.getCurrentTransform(tf);

  const Vec3f& axis = motion.getAxis();
  FCL_REAL linear_vel = motion.getLinearVelocity();
  FCL_REAL angular_vel = motion.getAngularVelocity();
  const Vec3f& p = motion.getAxisOrigin();
    
  FCL_REAL c_proj_max = ((tf.getQuatRotation().transform(bv.Tr)).cross(axis)).sqrLength();
  FCL_REAL tmp;
  tmp = ((tf.getQuatRotation().transform(bv.Tr + bv.axis[0] * bv.l[0])).cross(axis)).sqrLength();
  if(tmp > c_proj_max) c_proj_max = tmp;
  tmp = ((tf.getQuatRotation().transform(bv.Tr + bv.axis[1] * bv.l[1])).cross(axis)).sqrLength();
  if(tmp > c_proj_max) c_proj_max = tmp;
  tmp = ((tf.getQuatRotation().transform(bv.Tr + bv.axis[0] * bv.l[0] + bv.axis[1] * bv.l[1])).cross(axis)).sqrLength();
  if(tmp > c_proj_max) c_proj_max = tmp;

  c_proj_max = sqrt(c_proj_max);

  FCL_REAL v_dot_n = axis.dot(n) * linear_vel;
  FCL_REAL w_cross_n = (axis.cross(n)).length() * angular_vel;
  FCL_REAL origin_proj = ((tf.getTranslation() - p).cross(axis)).length();

  FCL_REAL mu = v_dot_n + w_cross_n * (c_proj_max + bv.r + origin_proj);

  return mu;  
}

FCL_REAL TriangleMotionBoundVisitor::visit(const ScrewMotion& motion) const
{
  Transform3f tf;
  motion.getCurrentTransform(tf);

  const Vec3f& axis = motion.getAxis();
  FCL_REAL linear_vel = motion.getLinearVelocity();
  FCL_REAL angular_vel = motion.getAngularVelocity();
  const Vec3f& p = motion.getAxisOrigin();
  
  FCL_REAL proj_max = ((tf.getQuatRotation().transform(a) + tf.getTranslation() - p).cross(axis)).sqrLength();
  FCL_REAL tmp;
  tmp = ((tf.getQuatRotation().transform(b) + tf.getTranslation() - p).cross(axis)).sqrLength();
  if(tmp > proj_max) proj_max = tmp;
  tmp = ((tf.getQuatRotation().transform(c) + tf.getTranslation() - p).cross(axis)).sqrLength();
  if(tmp > proj_max) proj_max = tmp;

  proj_max = std::sqrt(proj_max);

  FCL_REAL v_dot_n = axis.dot(n) * linear_vel;
  FCL_REAL w_cross_n = (axis.cross(n)).length() * angular_vel;
  FCL_REAL mu = v_dot_n + w_cross_n * proj_max;

  return mu;
}





/// @brief Compute the motion bound for a bounding volume along a given direction n
/// according to mu < |v * n| + ||w x n||(r + max(||ci*||)) where ||ci*|| = ||R0(ci) x w||. w is the angular axis (normalized)
/// and ci are the endpoints of the generator primitives of RSS.
/// Notice that all bv parameters are in the local frame of the object, but n should be in the global frame (the reason is that the motion (t1, t2 and t) is in global frame)
template<>
FCL_REAL TBVMotionBoundVisitor<RSS>::visit(const InterpMotion& motion) const
{
  Transform3f tf;
  motion.getCurrentTransform(tf);

  const Vec3f& reference_p = motion.getReferencePoint();
  const Vec3f& angular_axis = motion.getAngularAxis();
  FCL_REAL angular_vel = motion.getAngularVelocity();
  const Vec3f& linear_vel = motion.getLinearVelocity();
  
  FCL_REAL c_proj_max = ((tf.getQuatRotation().transform(bv.Tr - reference_p)).cross(angular_axis)).sqrLength();
  FCL_REAL tmp;
  tmp = ((tf.getQuatRotation().transform(bv.Tr + bv.axis[0] * bv.l[0] - reference_p)).cross(angular_axis)).sqrLength();
  if(tmp > c_proj_max) c_proj_max = tmp;
  tmp = ((tf.getQuatRotation().transform(bv.Tr + bv.axis[1] * bv.l[1] - reference_p)).cross(angular_axis)).sqrLength();
  if(tmp > c_proj_max) c_proj_max = tmp;
  tmp = ((tf.getQuatRotation().transform(bv.Tr + bv.axis[0] * bv.l[0] + bv.axis[1] * bv.l[1] - reference_p)).cross(angular_axis)).sqrLength();
  if(tmp > c_proj_max) c_proj_max = tmp;

  c_proj_max = std::sqrt(c_proj_max);

  FCL_REAL v_dot_n = linear_vel.dot(n);
  FCL_REAL w_cross_n = (angular_axis.cross(n)).length() * angular_vel;
  FCL_REAL mu = v_dot_n + w_cross_n * (bv.r + c_proj_max);

  return mu;  
}

/// @brief Compute the motion bound for a triangle along a given direction n
/// according to mu < |v * n| + ||w x n||(max||ci*||) where ||ci*|| = ||R0(ci) x w|| / \|w\|. w is the angular velocity
/// and ci are the triangle vertex coordinates.
/// Notice that the triangle is in the local frame of the object, but n should be in the global frame (the reason is that the motion (t1, t2 and t) is in global frame)
FCL_REAL TriangleMotionBoundVisitor::visit(const InterpMotion& motion) const
{
  Transform3f tf;
  motion.getCurrentTransform(tf);

  const Vec3f& reference_p = motion.getReferencePoint();
  const Vec3f& angular_axis = motion.getAngularAxis();
  FCL_REAL angular_vel = motion.getAngularVelocity();
  const Vec3f& linear_vel = motion.getLinearVelocity();

  FCL_REAL proj_max = ((tf.getQuatRotation().transform(a - reference_p)).cross(angular_axis)).sqrLength();
  FCL_REAL tmp;
  tmp = ((tf.getQuatRotation().transform(b - reference_p)).cross(angular_axis)).sqrLength();
  if(tmp > proj_max) proj_max = tmp;
  tmp = ((tf.getQuatRotation().transform(c - reference_p)).cross(angular_axis)).sqrLength();
  if(tmp > proj_max) proj_max = tmp;

  proj_max = std::sqrt(proj_max);

  FCL_REAL v_dot_n = linear_vel.dot(n);
  FCL_REAL w_cross_n = (angular_axis.cross(n)).length() * angular_vel;
  FCL_REAL mu = v_dot_n + w_cross_n * proj_max;

  return mu;  
}

InterpMotion::InterpMotion() : MotionBase()
{
  // Default angular velocity is zero
  angular_axis.setValue(1, 0, 0);
  angular_vel = 0;

  // Default reference point is local zero point

  // Default linear velocity is zero
}

InterpMotion::InterpMotion(const Matrix3f& R1, const Vec3f& T1,
                           const Matrix3f& R2, const Vec3f& T2) : MotionBase(),
                                                                  tf1(R1, T1),
                                                                  tf2(R2, T2),
                                                                  tf(tf1)
{
  // Compute the velocities for the motion
  computeVelocity();
}


InterpMotion::InterpMotion(const Transform3f& tf1_, const Transform3f& tf2_) : MotionBase(),
                                                                               tf1(tf1_),
                                                                               tf2(tf2_),
                                                                               tf(tf1)
{
  // Compute the velocities for the motion
  computeVelocity();
}

InterpMotion::InterpMotion(const Matrix3f& R1, const Vec3f& T1,
                           const Matrix3f& R2, const Vec3f& T2,
                           const Vec3f& O) : MotionBase(),
                                             tf1(R1, T1),
                                             tf2(R2, T2),
                                             tf(tf1),
                                             reference_p(O)
{
  // Compute the velocities for the motion
  computeVelocity();
}

InterpMotion::InterpMotion(const Transform3f& tf1_, const Transform3f& tf2_, const Vec3f& O) : MotionBase(),
                                                                                               tf1(tf1_),
                                                                                               tf2(tf2_),
                                                                                               tf(tf1),
                                                                                               reference_p(O)
{
}

bool InterpMotion::integrate(double dt) const
{
  if(dt > 1) dt = 1;

  tf.setQuatRotation(absoluteRotation(dt));
  tf.setTranslation(linear_vel * dt + tf1.transform(reference_p) - tf.getQuatRotation().transform(reference_p));

  return true;
}


void InterpMotion::computeVelocity()
{
  linear_vel = tf2.transform(reference_p) - tf1.transform(reference_p);
  Quaternion3f deltaq = tf2.getQuatRotation() * inverse(tf1.getQuatRotation());
  deltaq.toAxisAngle(angular_axis, angular_vel);
  if(angular_vel < 0)
  {
    angular_vel = -angular_vel;
    angular_axis = -angular_axis;
  }
}


Quaternion3f InterpMotion::deltaRotation(FCL_REAL dt) const
{
  Quaternion3f res;
  res.fromAxisAngle(angular_axis, (FCL_REAL)(dt * angular_vel));
  return res;
}

Quaternion3f InterpMotion::absoluteRotation(FCL_REAL dt) const
{
  Quaternion3f delta_t = deltaRotation(dt);
  return delta_t * tf1.getQuatRotation();
}


/// @brief Compute the motion bound for a bounding volume along a given direction n
template<>
FCL_REAL TBVMotionBoundVisitor<RSS>::visit(const TranslationMotion& motion) const
{
  return motion.getVelocity().dot(n);
}

/// @brief Compute the motion bound for a triangle along a given direction n
FCL_REAL TriangleMotionBoundVisitor::visit(const TranslationMotion& motion) const
{
  return motion.getVelocity().dot(n);
}

template class TBVMotionBoundVisitor<RSS>;

}

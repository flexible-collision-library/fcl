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


#include "fcl/ccd/motion.h"

namespace fcl
{

template<>
FCL_REAL InterpMotion<RSS>::computeMotionBound(const RSS& bv, const Vec3f& n) const
{
  FCL_REAL c_proj_max = ((tf.getQuatRotation().transform(bv.Tr - reference_p)).cross(angular_axis)).sqrLength();
  FCL_REAL tmp;
  tmp = ((tf.getQuatRotation().transform(bv.Tr + bv.axis[0] * bv.l[0] - reference_p)).cross(angular_axis)).sqrLength();
  if(tmp > c_proj_max) c_proj_max = tmp;
  tmp = ((tf.getQuatRotation().transform(bv.Tr + bv.axis[1] * bv.l[1] - reference_p)).cross(angular_axis)).sqrLength();
  if(tmp > c_proj_max) c_proj_max = tmp;
  tmp = ((tf.getQuatRotation().transform(bv.Tr + bv.axis[0] * bv.l[0] + bv.axis[1] * bv.l[1] - reference_p)).cross(angular_axis)).sqrLength();
  if(tmp > c_proj_max) c_proj_max = tmp;

  c_proj_max = sqrt(c_proj_max);

  FCL_REAL v_dot_n = linear_vel.dot(n);
  FCL_REAL w_cross_n = (angular_axis.cross(n)).length() * angular_vel;
  FCL_REAL mu = v_dot_n + w_cross_n * (bv.r + c_proj_max);

  return mu;
}

template<>
FCL_REAL ScrewMotion<RSS>::computeMotionBound(const RSS& bv, const Vec3f& n) const
{
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

template<>
FCL_REAL SplineMotion<RSS>::computeMotionBound(const RSS& bv, const Vec3f& n) const
{
  FCL_REAL T_bound = computeTBound(n);

  Vec3f c1 = bv.Tr;
  Vec3f c2 = bv.Tr + bv.axis[0] * bv.l[0];
  Vec3f c3 = bv.Tr + bv.axis[1] * bv.l[1];
  Vec3f c4 = bv.Tr + bv.axis[0] * bv.l[0] + bv.axis[1] * bv.l[1];

  FCL_REAL tmp;
  // max_i |c_i * n|
  FCL_REAL cn_max = fabs(c1.dot(n));
  tmp = fabs(c2.dot(n));
  if(tmp > cn_max) cn_max = tmp;
  tmp = fabs(c3.dot(n));
  if(tmp > cn_max) cn_max = tmp;
  tmp = fabs(c4.dot(n));
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

  FCL_REAL dWdW_max = computeDWMax();
  FCL_REAL ratio = std::min(1 - tf_t, dWdW_max);

  FCL_REAL R_bound = 2 * (cn_max + cmax + cxn_max + 3 * bv.r) * ratio;


  // std::cout << R_bound << " " << T_bound << std::endl;

  return R_bound + T_bound;
}

}

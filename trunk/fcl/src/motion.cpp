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


#include "fcl/motion.h"

namespace fcl
{

template<>
BVH_REAL InterpMotion<RSS>::computeMotionBound(const RSS& bv, const Vec3f& n) const
{
  BVH_REAL c_proj_max = (bv.Tr.cross(angular_axis)).sqrLength();
  BVH_REAL tmp;
  tmp = ((bv.Tr + bv.axis[0] * bv.l[0]).cross(angular_axis)).sqrLength();
  if(tmp > c_proj_max) c_proj_max = tmp;
  tmp = ((bv.Tr + bv.axis[1] * bv.l[1]).cross(angular_axis)).sqrLength();
  if(tmp > c_proj_max) c_proj_max = tmp;
  tmp = ((bv.Tr + bv.axis[0] * bv.l[0] + bv.axis[1] * bv.l[1]).cross(angular_axis)).sqrLength();
  if(tmp > c_proj_max) c_proj_max = tmp;

  c_proj_max = sqrt(c_proj_max);

  BVH_REAL v_dot_n = linear_vel.dot(n);
  BVH_REAL w_cross_n = (angular_axis.cross(n)).length() * angular_vel;
  BVH_REAL mu = v_dot_n + w_cross_n * (bv.r + c_proj_max);

  return mu;
}

}

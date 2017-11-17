/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2014, Willow Garage, Inc.
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

#ifndef FCL_MATH_SAMPLERSE3EULERBALL_INL_H
#define FCL_MATH_SAMPLERSE3EULERBALL_INL_H

#include "fcl/math/sampler/sampler_se3_euler_ball.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT SamplerSE3Euler_ball<double>;

//==============================================================================
template <typename S>
SamplerSE3Euler_ball<S>::SamplerSE3Euler_ball()
{
  // Do nothing
}

//==============================================================================
template <typename S>
SamplerSE3Euler_ball<S>::SamplerSE3Euler_ball(S r_) : r(r_)
{
  // Do nothing
}

//==============================================================================
template <typename S>
void SamplerSE3Euler_ball<S>::setBound(const S& r_)
{
  r = r_;
}

//==============================================================================
template <typename S>
void SamplerSE3Euler_ball<S>::getBound(S& r_) const
{
  r_ = r;
}

//==============================================================================
template <typename S>
Vector6<S> SamplerSE3Euler_ball<S>::sample() const
{
  Vector6<S> q;
  S x, y, z;
  this->rng.ball(0, r, x, y, z);
  q[0] = x;
  q[1] = y;
  q[2] = z;

  S s[4];
  this->rng.quaternion(s);

  Quaternion<S> quat(s[0], s[1], s[2], s[3]);
  Vector3<S> angles = quat.toRotationMatrix().eulerAngles(0, 1, 2);
  q[3] = angles[0];
  q[4] = angles[1];
  q[5] = angles[2];

  return q;
}

} // namespace fcl

#endif

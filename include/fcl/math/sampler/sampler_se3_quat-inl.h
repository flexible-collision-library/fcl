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

#ifndef FCL_MATH_SAMPLERSE3QUAT_INL_H
#define FCL_MATH_SAMPLERSE3QUAT_INL_H

#include "fcl/math/sampler/sampler_se3_quat.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT SamplerSE3Quat<double>;

//==============================================================================
template <typename S>
SamplerSE3Quat<S>::SamplerSE3Quat()
{
  // Do nothing
}

//==============================================================================
template <typename S>
SamplerSE3Quat<S>::SamplerSE3Quat(const Vector3<S>& lower_bound_, const Vector3<S>& upper_bound_) : lower_bound(lower_bound_),
  upper_bound(upper_bound_)
{
  // Do nothing
}

//==============================================================================
template <typename S>
void SamplerSE3Quat<S>::getBound(Vector3<S>& lower_bound_, Vector3<S>& upper_bound_) const
{
  lower_bound_ = lower_bound;
  upper_bound_ = upper_bound;
}

//==============================================================================
template <typename S>
void SamplerSE3Quat<S>::setBound(const Vector3<S>& lower_bound_, const Vector3<S>& upper_bound_)

{
  lower_bound = lower_bound_;
  upper_bound = upper_bound_;
}

//==============================================================================
template <typename S>
Vector6<S> SamplerSE3Quat<S>::sample() const
{
  Vector6<S> q;
  q[0] = this->rng.uniformReal(lower_bound[0], upper_bound[0]);
  q[1] = this->rng.uniformReal(lower_bound[1], upper_bound[1]);
  q[2] = this->rng.uniformReal(lower_bound[2], upper_bound[2]);

  S s[4];
  this->rng.quaternion(s);

  q[3] = s[0];
  q[4] = s[1];
  q[5] = s[2];
  q[6] = s[3];
  return q;
}

} // namespace fcl

#endif

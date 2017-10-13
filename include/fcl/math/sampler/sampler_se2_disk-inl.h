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

#ifndef FCL_MATH_SAMPLERSE2DISK_INL_H
#define FCL_MATH_SAMPLERSE2DISK_INL_H

#include "fcl/math/sampler/sampler_se2_disk.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT SamplerSE2_disk<double>;

//==============================================================================
template <typename S>
SamplerSE2_disk<S>::SamplerSE2_disk()
{
  // Do nothing
}

//==============================================================================
template <typename S>
SamplerSE2_disk<S>::SamplerSE2_disk(S cx, S cy, S r1, S r2, S crefx, S crefy)
{
  setBound(cx, cy, r1, r2, crefx, crefy);
}

//==============================================================================
template <typename S>
void SamplerSE2_disk<S>::setBound(S cx, S cy, S r1, S r2, S crefx, S crefy)
{
  c[0] = cx; c[1] = cy;
  cref[0] = crefx; cref[1] = crefy;
  r_min = r1;
  r_max = r2;
}

//==============================================================================
template <typename S>
Vector3<S> SamplerSE2_disk<S>::sample() const
{
  Vector3<S> q;
  S x, y;
  this->rng.disk(r_min, r_max, x, y);
  q[0] = x + c[0] - cref[0];
  q[1] = y + c[1] - cref[1];
  q[2] = this->rng.uniformReal(-constants<S>::pi(), constants<S>::pi());

  return q;
}

} // namespace fcl

#endif

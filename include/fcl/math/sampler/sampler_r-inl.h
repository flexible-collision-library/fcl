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

#ifndef FCL_MATH_SAMPLERR_INL_H
#define FCL_MATH_SAMPLERR_INL_H

#include "fcl/math/sampler/sampler_r.h"

namespace fcl
{

//==============================================================================
template <typename S, std::size_t N>
SamplerR<S, N>::SamplerR()
{
  // Do nothing
}

//==============================================================================
template <typename S, std::size_t N>
SamplerR<S, N>::SamplerR(const VectorN<S, N>& lower_bound_, const VectorN<S, N>& upper_bound_)
  : lower_bound(lower_bound_), upper_bound(upper_bound_)
{
}

//==============================================================================
template <typename S, std::size_t N>
void SamplerR<S, N>::setBound(const VectorN<S, N>& lower_bound_, const VectorN<S, N>& upper_bound_)
{
  lower_bound = lower_bound_;
  upper_bound = upper_bound_;
}

//==============================================================================
template <typename S, std::size_t N>
void SamplerR<S, N>::getBound(VectorN<S, N>& lower_bound_, VectorN<S, N>& upper_bound_) const
{
  lower_bound_ = lower_bound;
  upper_bound_ = upper_bound;
}

//==============================================================================
template <typename S, std::size_t N>
VectorN<S, N> SamplerR<S, N>::sample() const
{
  VectorN<S, N> q;

  for(std::size_t i = 0; i < N; ++i)
  {
    q[i] = this->rng.uniformReal(lower_bound[i], upper_bound[i]);
  }

  return q;
}

} // namespace fcl

#endif

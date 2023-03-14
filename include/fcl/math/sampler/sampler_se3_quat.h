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

#ifndef FCL_MATH_SAMPLERSE3QUAT_H
#define FCL_MATH_SAMPLERSE3QUAT_H

#include "fcl/common/types.h"
#include "fcl/math/sampler/sampler_base.h"

namespace fcl
{

template <typename S>
class SamplerSE3Quat : public SamplerBase<S>
{
public:
  SamplerSE3Quat();

  SamplerSE3Quat(const Vector3<S>& lower_bound_,
                 const Vector3<S>& upper_bound_);

  void setBound(const Vector3<S>& lower_bound_,
                const Vector3<S>& upper_bound_);

  void getBound(Vector3<S>& lower_bound_,
                Vector3<S>& upper_bound_) const;

  Vector6<S> sample() const;

protected:
  Vector3<S> lower_bound;
  Vector3<S> upper_bound;

};

using SamplerSE3Quatf = SamplerSE3Quat<float>;
using SamplerSE3Quatd = SamplerSE3Quat<double>;

} // namespace fcl

#include "fcl/math/sampler/sampler_se3_quat-inl.h"

#endif

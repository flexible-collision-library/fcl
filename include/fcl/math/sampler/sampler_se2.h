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

#ifndef FCL_MATH_SAMPLERSE2_H
#define FCL_MATH_SAMPLERSE2_H

#include "fcl/common/types.h"
#include "fcl/math/sampler/sampler_base.h"

namespace fcl
{

template <typename S>
class SamplerSE2 : public SamplerBase<S>
{
public:
  SamplerSE2();

  SamplerSE2(const Vector2<S>& lower_bound_,
             const Vector2<S>& upper_bound_);

  SamplerSE2(S x_min, S x_max,
             S y_min, S y_max);


  void setBound(const Vector2<S>& lower_bound_,
                const Vector2<S>& upper_bound_);

  void getBound(Vector2<S>& lower_bound_,
                Vector2<S>& upper_bound_) const;


  Vector3<S> sample() const;

protected:
  Vector2<S> lower_bound;
  Vector2<S> upper_bound;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using SamplerSE2f = SamplerSE2<float>;
using SamplerSE2d = SamplerSE2<double>;

} // namespace fcl

#include "fcl/math/sampler/sampler_se2-inl.h"

#endif

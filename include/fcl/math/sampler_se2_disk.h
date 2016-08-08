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

/** \author Jia Pan */

#ifndef FCL_MATH_SAMPLERSE2DISK_H
#define FCL_MATH_SAMPLERSE2DISK_H

#include "fcl/data_types.h"
#include "fcl/math/sampler_base.h"

namespace fcl
{

template <typename Scalar>
class SamplerSE2_disk : public SamplerBase<Scalar>
{
public:
  SamplerSE2_disk();

  SamplerSE2_disk(Scalar cx, Scalar cy,
                  Scalar r1, Scalar r2,
                  Scalar crefx, Scalar crefy);

  void setBound(Scalar cx, Scalar cy,
                Scalar r1, Scalar r2,
                Scalar crefx, Scalar crefy);

  Vector3<Scalar> sample() const;

protected:
  Scalar c[2];
  Scalar cref[2];
  Scalar r_min, r_max;
};

using SamplerSE2_diskf = SamplerSE2_disk<float>;
using SamplerSE2_diskd = SamplerSE2_disk<double>;

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
SamplerSE2_disk<Scalar>::SamplerSE2_disk() {}

//==============================================================================
template <typename Scalar>
SamplerSE2_disk<Scalar>::SamplerSE2_disk(Scalar cx, Scalar cy, Scalar r1, Scalar r2, Scalar crefx, Scalar crefy)
{
  setBound(cx, cy, r1, r2, crefx, crefy);
}

//==============================================================================
template <typename Scalar>
void SamplerSE2_disk<Scalar>::setBound(Scalar cx, Scalar cy, Scalar r1, Scalar r2, Scalar crefx, Scalar crefy)
{
  c[0] = cx; c[1] = cy;
  cref[0] = crefx; cref[1] = crefy;
  r_min = r1;
  r_max = r2;
}

//==============================================================================
template <typename Scalar>
Vector3<Scalar> SamplerSE2_disk<Scalar>::sample() const
{
  Vector3<Scalar> q;
  Scalar x, y;
  this->rng.disk(r_min, r_max, x, y);
  q[0] = x + c[0] - cref[0];
  q[1] = y + c[1] - cref[1];
  q[2] = this->rng.uniformReal(-constants<Scalar>::pi(), constants<Scalar>::pi());

  return q;
}

} // namespace fcl

#endif

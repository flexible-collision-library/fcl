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

#ifndef FCL_VEC_3F_H
#define FCL_VEC_3F_H

#include "fcl/config.h"
#include "fcl/data_types.h"

#include <cmath>
#include <iostream>
#include <limits>

#include <Eigen/Dense>

namespace fcl
{

template <typename T>
using Vec3fX = Eigen::Matrix<T, 3, 1>;

using Vec3f = Vec3fX<FCL_REAL>;

inline void normalize(Vec3f& v, bool* signal)
{
  Vec3f::Scalar sqr_length = v.squaredNorm();

  if (sqr_length > 0)
  {
    v /= std::sqrt(sqr_length);
    *signal = true;
  }
  else
  {
    *signal = false;
  }
}

inline Vec3f::Scalar triple(const Vec3f& x, const Vec3f& y, const Vec3f& z)
{
  return x.dot(y.cross(z));
}

template <typename T>
void generateCoordinateSystem(const Vec3fX<T>& w, Vec3fX<T>& u, Vec3fX<T>& v)
{
  T inv_length;

  if(std::abs(w[0]) >= std::abs(w[1]))
  {
    inv_length = (T)1.0 / sqrt(w[0] * w[0] + w[2] * w[2]);
    u[0] = -w[2] * inv_length;
    u[1] = (T)0;
    u[2] =  w[0] * inv_length;
    v[0] =  w[1] * u[2];
    v[1] =  w[2] * u[0] - w[0] * u[2];
    v[2] = -w[1] * u[0];
  }
  else
  {
    inv_length = (T)1.0 / sqrt(w[1] * w[1] + w[2] * w[2]);
    u[0] = (T)0;
    u[1] =  w[2] * inv_length;
    u[2] = -w[1] * inv_length;
    v[0] =  w[1] * u[2] - w[2] * u[1];
    v[1] = -w[0] * u[2];
    v[2] =  w[0] * u[1];
  }
}

} // namespace fcl

#endif

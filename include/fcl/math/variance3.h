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

/** @author Jia Pan */

#ifndef FCL_MATH_VARIANCE3_H
#define FCL_MATH_VARIANCE3_H

#include <cmath>

#include "fcl/common/types.h"
#include "fcl/math/geometry.h"

namespace fcl
{

/// @brief Class for variance matrix in 3d
template <typename S>
class FCL_EXPORT Variance3
{
public:
  /// @brief Variation matrix
  Matrix3<S> Sigma;

  /// @brief Variations along the eign axes
  Vector3<S> sigma;

  /// @brief Matrix whose columns are eigenvectors of Sigma
  Matrix3<S> axis;

  Variance3();

  Variance3(const Matrix3<S>& sigma);

  /// @brief init the Variance
  void init();

  /// @brief Compute the sqrt of Sigma matrix based on the eigen decomposition
  /// result, this is useful when the uncertainty matrix is initialized as a
  /// square variation matrix
  Variance3<S>& sqrt();
};

using Variance3f = Variance3<float>;
using Variance3d = Variance3<double>;

} // namespace fcl

#include "fcl/math/variance3-inl.h"

#endif

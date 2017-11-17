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

#ifndef FCL_NARROWPHASE_DETAIL_POLYSOLVER_H
#define FCL_NARROWPHASE_DETAIL_POLYSOLVER_H

#include "fcl/export.h"

namespace fcl
{

namespace detail {

/// @brief A class solves polynomial degree (1,2,3) equations 
template <typename S>
class FCL_EXPORT PolySolver
{
public:
  /// @brief Solve a linear equation with coefficients c, return roots s and number of roots 
  static int solveLinear(S c[2], S s[1]);

  /// @brief Solve a quadratic function with coefficients c, return roots s and number of roots 
  static int solveQuadric(S c[3], S s[2]);

  /// @brief Solve a cubic function with coefficients c, return roots s and number of roots 
  static int solveCubic(S c[4], S s[3]);

private:
  /// @brief Check whether v is zero 
  static bool isZero(S v);

  /// @brief Compute v^{1/3} 
  static bool cbrt(S v);

  static constexpr S getNearZeroThreshold();
};

using PolySolverf = PolySolver<float>;
using PolySolverd = PolySolver<double>;

} // namespace detail
} // namespace fcl

#include "fcl/math/detail/polysolver-inl.h"

#endif

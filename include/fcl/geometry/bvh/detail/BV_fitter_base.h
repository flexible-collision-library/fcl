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

#ifndef FCL_BV_FITTERBASE_H
#define FCL_BV_FITTERBASE_H

#include "fcl/math/triangle.h"
#include "fcl/geometry/bvh/BVH_internal.h"
#include "fcl/math/bv/kIOS.h"
#include "fcl/math/bv/OBBRSS.h"
#include <iostream>

namespace fcl
{

namespace detail
{

/// @brief Interface for fitting a bv given the triangles or points inside it.
template <typename BV>
class FCL_EXPORT BVFitterBase
{
public:

  using S = typename BV::S;

  /// @brief Set the primitives to be processed by the fitter
  virtual void set(Vector3<S>* vertices_, Triangle* tri_indices_, BVHModelType type_) = 0;

  /// @brief Set the primitives to be processed by the fitter, for deformable mesh.
  virtual void set(Vector3<S>* vertices_, Vector3<S>* prev_vertices_, Triangle* tri_indices_, BVHModelType type_) = 0;

  /// @brief Compute the fitting BV
  virtual BV fit(unsigned int* primitive_indices, int num_primitives) = 0;

  /// @brief clear the temporary data generated.
  virtual void clear() = 0;
};

} // namespace detail
} // namespace fcl

#endif

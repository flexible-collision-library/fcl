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

#ifndef FCL_BV_FITTER_H
#define FCL_BV_FITTER_H

#include <iostream>
#include "fcl/math/triangle.h"
#include "fcl/math/bv/kIOS.h"
#include "fcl/math/bv/OBBRSS.h"
#include "fcl/geometry/bvh/BVH_internal.h"
#include "fcl/geometry/bvh/detail/BV_fitter_base.h"

namespace fcl
{

namespace detail
{

/// @brief The class for the default algorithm fitting a bounding volume to a set of points
template <typename BV>
class FCL_EXPORT BVFitter : public BVFitterBase<BV>
{
public:

  using S = typename BVFitterBase<BV>::S;

  /// @brief default deconstructor
  virtual ~BVFitter();

  /// @brief Prepare the geometry primitive data for fitting
  void set(
      Vector3<S>* vertices_, Triangle* tri_indices_, BVHModelType type_);

  /// @brief Prepare the geometry primitive data for fitting, for deformable mesh
  void set(
      Vector3<S>* vertices_,
      Vector3<S>* prev_vertices_,
      Triangle* tri_indices_,
      BVHModelType type_);

  /// @brief Compute a bounding volume that fits a set of primitives (points or triangles).
  /// The primitive data was set by set function and primitive_indices is the primitive index relative to the data
  BV fit(unsigned int* primitive_indices, int num_primitives);

  /// @brief Clear the geometry primitive data
  void clear();

private:

  Vector3<S>* vertices;
  Vector3<S>* prev_vertices;
  Triangle* tri_indices;
  BVHModelType type;

  template <typename, typename>
  friend struct SetImpl;

  template <typename, typename>
  friend struct FitImpl;
};

} // namespace detail
} // namespace fcl

#include "fcl/geometry/bvh/detail/BV_fitter-inl.h"

#endif

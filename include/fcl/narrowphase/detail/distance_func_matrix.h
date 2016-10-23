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

#ifndef FCL_DISTANCE_FUNC_MATRIX_H
#define FCL_DISTANCE_FUNC_MATRIX_H

#include "fcl/geometry/collision_geometry.h"
#include "fcl/narrowphase/distance_request.h"
#include "fcl/narrowphase/distance_result.h"

namespace fcl
{

namespace detail
{

/// @brief distance matrix stores the functions for distance between different
/// types of objects and provides a uniform call interface
template <typename NarrowPhaseSolver>
struct DistanceFunctionMatrix
{
  using S = typename NarrowPhaseSolver::S;

  /// @brief the uniform call interface for distance: for distance, we need know
  /// 1. two objects o1 and o2 and their configuration in world coordinate tf1
  ///    and tf2;
  /// 2. the solver for narrow phase collision, this is for distance computation
  ///    between geometric shapes;
  /// 3. the request setting for distance (e.g., whether need to return nearest
  /// points);
  using DistanceFunc = S (*)(
      const CollisionGeometry<S>* o1,
      const Transform3<S>& tf1,
      const CollisionGeometry<S>* o2,
      const Transform3<S>& tf2,
      const NarrowPhaseSolver* nsolver,
      const DistanceRequest<S>& request,
      DistanceResult<S>& result);
  
  /// @brief each item in the distance matrix is a function to handle distance
  /// between objects of type1 and type2
  DistanceFunc distance_matrix[NODE_COUNT][NODE_COUNT];

  DistanceFunctionMatrix();
};

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/distance_func_matrix-inl.h"

#endif

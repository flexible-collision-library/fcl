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

#ifndef FCL_COLLISION_FUNC_MATRIX_H
#define FCL_COLLISION_FUNC_MATRIX_H

#include "fcl/geometry/collision_geometry.h"
#include "fcl/narrowphase/collision_request.h"
#include "fcl/narrowphase/collision_result.h"

namespace fcl
{

namespace detail
{

/// @brief collision matrix stores the functions for collision between different
/// types of objects and provides a uniform call interface
template <typename NarrowPhaseSolver>
struct CollisionFunctionMatrix
{
  using S = typename NarrowPhaseSolver::S;

  /// @brief the uniform call interface for collision: for collision, we need
  /// know
  /// 1. two objects o1 and o2 and their configuration in world coordinate tf1
  ///    and tf2;
  /// 2. the solver for narrow phase collision, this is for the collision
  ///    between geometric shapes;
  /// 3. the request setting for collision (e.g., whether need to return normal
  ///    information, whether need to compute cost);
  /// 4. the structure to return collision result
  using CollisionFunc = std::size_t (*)(
      const CollisionGeometry<S>* o1,
      const Transform3<S>& tf1,
      const CollisionGeometry<S>* o2,
      const Transform3<S>& tf2,
      const NarrowPhaseSolver* nsolver,
      const CollisionRequest<S>& request,
      CollisionResult<S>& result);

  /// @brief each item in the collision matrix is a function to handle collision
  /// between objects of type1 and type2
  CollisionFunc collision_matrix[NODE_COUNT][NODE_COUNT];

  CollisionFunctionMatrix();
};

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/collision_func_matrix-inl.h"

#endif

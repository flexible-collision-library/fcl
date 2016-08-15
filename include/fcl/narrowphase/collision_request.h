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

#ifndef FCL_COLLISIONREQUEST_H
#define FCL_COLLISIONREQUEST_H

#include "fcl/common/types.h"
#include "fcl/narrowphase/gjk_solver_type.h"

namespace fcl
{

template <typename S>
struct CollisionResult;

/// @brief request to the collision algorithm
template <typename S>
struct CollisionRequest
{  
  /// @brief The maximum number of contacts will return
  size_t num_max_contacts;

  /// @brief whether the contact information (normal, penetration depth and contact position) will return
  bool enable_contact;

  /// @brief The maximum number of cost sources will return
  size_t num_max_cost_sources;

  /// @brief whether the cost sources will be computed
  bool enable_cost;

  /// @brief whether the cost computation is approximated
  bool use_approximate_cost;

  /// @brief narrow phase solver
  GJKSolverType gjk_solver_type;

  /// @brief whether enable gjk intial guess
  bool enable_cached_gjk_guess;
  
  /// @brief the gjk intial guess set by user
  Vector3<S> cached_gjk_guess;

  CollisionRequest(size_t num_max_contacts_ = 1,
                   bool enable_contact_ = false,
                   size_t num_max_cost_sources_ = 1,
                   bool enable_cost_ = false,
                   bool use_approximate_cost_ = true,
                   GJKSolverType gjk_solver_type_ = GST_LIBCCD);

  bool isSatisfied(const CollisionResult<S>& result) const;
};

using CollisionRequestf = CollisionRequest<float>;
using CollisionRequestd = CollisionRequest<double>;

} // namespace fcl

#include "fcl/narrowphase/collision_request-inl.h"

#endif

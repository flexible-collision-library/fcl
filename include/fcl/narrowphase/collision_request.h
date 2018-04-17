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

/// @brief Parameters for performing collision request.
template <typename S>
struct FCL_EXPORT CollisionRequest
{
  /// The underlying numerical representation of the request's scalar (e.g.,
  /// float or double).
  typedef typename Eigen::NumTraits<S>::Real Real;

  /// @brief The maximum number of contacts that can be returned.
  size_t num_max_contacts;

  /// @brief If true, contact information (e.g., normal, penetration depth, and
  /// contact position) will be returned.
  bool enable_contact;

  // TODO(SeanCurtis-TRI): Provide clear definitions for what "cost sources"
  // are.

  /// @brief The maximum number of cost sources that can be returned.
  size_t num_max_cost_sources;

  /// @brief If true, the cost sources will be computed.
  bool enable_cost;

  /// @brief If true, the cost computation is approximated (if computed).
  bool use_approximate_cost;

  /// @brief Enumeration indicating the GJK solver implementation to use.
  GJKSolverType gjk_solver_type;

  // TODO(SeanCurtis-TRI): Consider swapping these *two* parameters with a
  // single std::optional<Vector3<S>>.
  /// @brief If true, uses the provided initial guess for the GJK algorithm.
  bool enable_cached_gjk_guess;
  
  /// @brief The initial guess to use in the GJK algorithm.
  Vector3<S> cached_gjk_guess;

  // TODO(SeanCurtis-TRI): Document the implications of this tolerance; right
  // now it is not clear *at all* what turning this knob will do to the results.
  /// @brief Numerical tolerance to use in the GJK algorithm.
  /// NOTE: The default value is currently set as 1e-6 to provide backwards
  /// compatibility; historically it has been 1e-6. Future code should provide
  /// a value that is consistent with the precision of `S`.
  Real gjk_tolerance{1e-6};

  /// @brief Default constructor
  CollisionRequest(size_t num_max_contacts_ = 1,
                   bool enable_contact_ = false,
                   size_t num_max_cost_sources_ = 1,
                   bool enable_cost_ = false,
                   bool use_approximate_cost_ = true,
                   GJKSolverType gjk_solver_type_ = GST_LIBCCD,
                   Real gjk_tolerance_ = 1e-6);

  bool isSatisfied(const CollisionResult<S>& result) const;
};

using CollisionRequestf = CollisionRequest<float>;
using CollisionRequestd = CollisionRequest<double>;

} // namespace fcl

#include "fcl/narrowphase/collision_request-inl.h"

#endif

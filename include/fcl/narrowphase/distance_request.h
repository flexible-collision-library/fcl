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

#ifndef FCL_DISTANCEREQUEST_H
#define FCL_DISTANCEREQUEST_H

#include "fcl/common/types.h"
#include "fcl/narrowphase/gjk_solver_type.h"

namespace fcl
{

template <typename S>
struct DistanceResult;

/// @brief request to the distance computation
template <typename S>
struct FCL_EXPORT DistanceRequest
{
  /// @brief whether to return the nearest points
  bool enable_nearest_points;

  /// @brief Whether to compute exact negative distance.
  ///
  /// Basically, the distance computation routine computes only the exact
  /// positive distance when the two objects are not in collision. If the
  /// objects are in collision, the result distance is implementation defined
  /// (mostly -1).
  ///
  ///       [ Current Implementation Status ]
  /// -----------------+--------------+--------------
  ///   GJKSolverType  |  GST_LIBCCD  |  GST_INDEP
  /// -----------------+--------------+--------------
  /// primitive shapes | SD_1, NP     | SD_2, NP_X
  /// mesh and octree  | SD_2, NP_X   | SD_2, NP_X
  /// -----------------+--------------+--------------
  /// SD_1: Signed distance is computed using convexity based methods (GJK, MPA)
  /// SD_2: Positive distance is computed using convexity based mothods (GJK,
  ///       MPA), but negative distance is computed by a workaround using
  ///       penetration computation.
  /// NP  : The pair of nearest points are guaranteed to be on the surface of
  ///       objects.
  /// NP_X: The pair of nearest points are NOT guaranteed to be on the surface
  ///       of objects.
  ///
  /// If this flag is set to true, FCL will perform additional collision
  /// checking when the two objects are in collision in order to get the exact
  /// negative distance, which is the negated penetration depth. If there are
  /// multiple contact for the two objects, then the maximum penetration depth
  /// is used.
  ///
  /// If this flag is set to false, the result minimum distance is
  /// implementation defined (mostly -1).
  ///
  /// The default is false.
  ///
  /// @sa DistanceResult::min_distance
  bool enable_signed_distance;

  /// @brief error threshold for approximate distance
  S rel_err; // relative error, between 0 and 1
  S abs_err; // absoluate error

  /// @brief the threshold used in GJK algorithm to stop distance iteration
  S distance_tolerance;

  /// @brief narrow phase solver type
  GJKSolverType gjk_solver_type;

  explicit DistanceRequest(
      bool enable_nearest_points_ = false,
      bool enable_signed_distance = false,
      S rel_err_ = 0.0,
      S abs_err_ = 0.0,
      S distance_tolerance = 1e-6,
      GJKSolverType gjk_solver_type_ = GST_LIBCCD);

  bool isSatisfied(const DistanceResult<S>& result) const;
};

using DistanceRequestf = DistanceRequest<float>;
using DistanceRequestd = DistanceRequest<double>;

} // namespace fcl

#include "fcl/narrowphase/distance_request-inl.h"

#endif

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

#ifndef FCL_DISTANCERESULT_H
#define FCL_DISTANCERESULT_H

#include "fcl/common/types.h"

namespace fcl
{

template <typename>
class CollisionGeometry;

/// @brief distance result
template <typename S>
struct FCL_EXPORT DistanceResult
{
public:

  /// @brief Minimum distance between two objects.
  ///
  /// The distance is correct as a positive value when the two objects are not
  /// in collision. If they are in collision, min_distance would be a
  /// implementation defined negative value, which might not the correct
  /// negative distance. In order to get correct negative distance set
  /// DistanceRequest::enable_signed_distance to true.
  ///
  /// @sa DistanceRequest::enable_signed_distance
  S min_distance;

  /// @brief Nearest points in the world coordinates.
  ///
  /// @sa DistanceRequest::enable_nearest_points
  Vector3<S> nearest_points[2];

  /// @brief collision object 1
  const CollisionGeometry<S>* o1;

  /// @brief collision object 2
  const CollisionGeometry<S>* o2;

  /// @brief information about the nearest point in object 1
  /// if object 1 is mesh or point cloud, it is the triangle or point id
  /// if object 1 is geometry shape, it is NONE (-1),
  /// if object 1 is octree, it is the query cell id (see
  ///                OcTree::getNodeByQueryCellId)
  intptr_t b1;

  /// @brief information about the nearest point in object 2
  /// if object 2 is mesh or point cloud, it is the triangle or point id
  /// if object 2 is geometry shape, it is NONE (-1),
  /// if object 2 is octree, it is the query cell id (see
  ///                OcTree::getNodeByQueryCellId)
  intptr_t b2;

  /// @brief invalid contact primitive information
  static const int NONE = -1;
  
  DistanceResult(S min_distance_ = std::numeric_limits<S>::max());

  /// @brief add distance information into the result
  void update(S distance, const CollisionGeometry<S>* o1_, const CollisionGeometry<S>* o2_, int b1_, int b2_);

  /// @brief add distance information into the result
  void update(S distance, const CollisionGeometry<S>* o1_, const CollisionGeometry<S>* o2_, int b1_, int b2_, const Vector3<S>& p1, const Vector3<S>& p2);

  /// @brief add distance information into the result
  void update(const DistanceResult& other_result);

  /// @brief clear the result
  void clear();
};

using DistanceResultf = DistanceResult<float>;
using DistanceResultd = DistanceResult<double>;

} // namespace fcl

#include "fcl/narrowphase/distance_result-inl.h"

#endif

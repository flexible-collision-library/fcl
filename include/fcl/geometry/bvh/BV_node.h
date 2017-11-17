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

#ifndef FCL_BV_BVNODE_H
#define FCL_BV_BVNODE_H

#include <iostream>
#include "fcl/math/bv/OBB.h"
#include "fcl/math/bv/RSS.h"
#include "fcl/math/bv/OBBRSS.h"
#include "fcl/geometry/bvh/BV_node_base.h"

namespace fcl
{

/// @brief A class describing a bounding volume node. It includes the tree structure providing in BVNodeBase and also the geometry data provided in BV template parameter.
template <typename BV>
struct FCL_EXPORT BVNode : public BVNodeBase
{
  using S = typename BV::S;

  /// @brief bounding volume storing the geometry
  BV bv;

  /// @brief Check whether two BVNode collide
  bool overlap(const BVNode& other) const;

  /// @brief Compute the distance between two BVNode. P1 and P2, if not nullptr and
  /// the underlying BV supports distance, return the nearest points.
  S distance(
      const BVNode& other,
      Vector3<S>* P1 = nullptr,
      Vector3<S>* P2 = nullptr) const;

  /// @brief Access the center of the BV
  Vector3<S> getCenter() const;

  /// @brief Access the orientation of the BV
  Matrix3<S> getOrientation() const;
};

} // namespace fcl

#include "fcl/geometry/bvh/BV_node-inl.h"

#endif

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

#ifndef FCL_BVH_FRONT_H
#define FCL_BVH_FRONT_H

#include <list>
#include "fcl/export.h"

namespace fcl
{

namespace detail
{

/// @brief Front list acceleration for collision
/// Front list is a set of internal and leaf nodes in the BVTT hierarchy, where
/// the traversal terminates while performing a query during a given time
/// instance. The front list reﬂects the subset of a BVTT that is traversed for
/// that particular proximity query.
struct FCL_EXPORT BVHFrontNode
{
  /// @brief The nodes to start in the future, i.e. the wave front of the
  /// traversal tree.
  int left, right;

  /// @brief The front node is not valid when collision is detected on the front
  /// node.
  bool valid;

  BVHFrontNode(int left_, int right_);
};

/// @brief BVH front list is a list of front nodes.
using BVHFrontList = std::list<BVHFrontNode>;

/// @brief Add new front node into the front list
FCL_EXPORT
void updateFrontList(BVHFrontList* front_list, int b1, int b2);

} // namespace detail
} // namespace fcl

#endif

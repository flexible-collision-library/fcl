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

#ifndef FCL_BV_BVNODEBASE_H
#define FCL_BV_BVNODEBASE_H

#include <iostream>
#include "fcl/math/bv/OBB.h"
#include "fcl/math/bv/RSS.h"
#include "fcl/math/bv/OBBRSS.h"

namespace fcl
{

/// @brief BVNodeBase encodes the tree structure for BVH
struct FCL_EXPORT BVNodeBase
{
  /// @brief An index for first child node or primitive
  /// If the value is positive, it is the index of the first child bv node
  /// If the value is negative, it is -(primitive index + 1)
  /// Zero is not used.
  int first_child;

  /// @brief The start id the primitive belonging to the current node. The index is referred to the primitive_indices in BVHModel and from that
  /// we can obtain the primitive's index in original data indirectly.
  int first_primitive;

  /// @brief The number of primitives belonging to the current node 
  int num_primitives;

  /// @brief Whether current node is a leaf node (i.e. contains a primitive index
  bool isLeaf() const;

  /// @brief Return the primitive index. The index is referred to the original data (i.e. vertices or tri_indices) in BVHModel
  int primitiveId() const;

  /// @brief Return the index of the first child. The index is referred to the bounding volume array (i.e. bvs) in BVHModel
  int leftChild() const;

  /// @brief Return the index of the second child. The index is referred to the bounding volume array (i.e. bvs) in BVHModel
  int rightChild() const;
};

} // namespace fcl

#endif

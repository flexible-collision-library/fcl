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

/** \author Jia Pan */


#ifndef FCL_BV_NODE_H
#define FCL_BV_NODE_H

#include "fcl/math/vec_3f.h"
#include "fcl/math/matrix_3f.h"

#include "fcl/BV/BV.h"
#include <iostream>

namespace fcl
{

/// @brief BVNodeBase encodes the tree structure for BVH
struct BVNodeBase
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
  inline bool isLeaf() const { return first_child < 0; }

  /// @brief Return the primitive index. The index is referred to the original data (i.e. vertices or tri_indices) in BVHModel
  inline int primitiveId() const { return -(first_child + 1); }

  /// @brief Return the index of the first child. The index is referred to the bounding volume array (i.e. bvs) in BVHModel
  inline int leftChild() const { return first_child; }

  /// @brief Return the index of the second child. The index is referred to the bounding volume array (i.e. bvs) in BVHModel
  inline int rightChild() const { return first_child + 1; }
};

/// @brief A class describing a bounding volume node. It includes the tree structure providing in BVNodeBase and also the geometry data provided in BV template parameter.
template<typename BV>
struct BVNode : public BVNodeBase
{
  /// @brief bounding volume storing the geometry
  BV bv;

  /// @brief Check whether two BVNode collide
  bool overlap(const BVNode& other) const
  {
    return bv.overlap(other.bv);
  }

  /// @brief Compute the distance between two BVNode. P1 and P2, if not NULL and the underlying BV supports distance, return the nearest points.
  FCL_REAL distance(const BVNode& other, Vec3f* P1 = NULL, Vec3f* P2 = NULL) const
  {
    return bv.distance(other.bv, P1, P2);
  }

  /// @brief Access the center of the BV
  Vec3f getCenter() const { return bv.center(); }

  /// @brief Access the orientation of the BV
  Matrix3f getOrientation() const { return Matrix3f::getIdentity(); }
};

template<>
inline Matrix3f BVNode<OBB>::getOrientation() const 
{
  return Matrix3f(bv.axis[0][0], bv.axis[1][0], bv.axis[2][0],
                  bv.axis[0][1], bv.axis[1][1], bv.axis[2][1],
                  bv.axis[0][2], bv.axis[1][2], bv.axis[2][2]);
}

template<>
inline Matrix3f BVNode<RSS>::getOrientation() const 
{
  return Matrix3f(bv.axis[0][0], bv.axis[1][0], bv.axis[2][0],
                  bv.axis[0][1], bv.axis[1][1], bv.axis[2][1],
                  bv.axis[0][2], bv.axis[1][2], bv.axis[2][2]);
}

template<>
inline Matrix3f BVNode<OBBRSS>::getOrientation() const 
{
  return Matrix3f(bv.obb.axis[0][0], bv.obb.axis[1][0], bv.obb.axis[2][0],
                  bv.obb.axis[0][1], bv.obb.axis[1][1], bv.obb.axis[2][1],
                  bv.obb.axis[0][2], bv.obb.axis[1][2], bv.obb.axis[2][2]);
}


}

#endif

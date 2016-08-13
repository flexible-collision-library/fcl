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


#ifndef FCL_BV_BVNODE_H
#define FCL_BV_BVNODE_H

#include "fcl/BV/convert_bv.h"
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
  bool isLeaf() const;

  /// @brief Return the primitive index. The index is referred to the original data (i.e. vertices or tri_indices) in BVHModel
  int primitiveId() const;

  /// @brief Return the index of the first child. The index is referred to the bounding volume array (i.e. bvs) in BVHModel
  int leftChild() const;

  /// @brief Return the index of the second child. The index is referred to the bounding volume array (i.e. bvs) in BVHModel
  int rightChild() const;
};

/// @brief A class describing a bounding volume node. It includes the tree structure providing in BVNodeBase and also the geometry data provided in BV template parameter.
template <typename BV>
struct BVNode : public BVNodeBase
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

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
inline bool BVNodeBase::isLeaf() const
{
  return first_child < 0;
}

//==============================================================================
inline int BVNodeBase::primitiveId() const
{
  return -(first_child + 1);
}

//==============================================================================
inline int BVNodeBase::leftChild() const
{
  return first_child;
}

//==============================================================================
inline int BVNodeBase::rightChild() const
{
  return first_child + 1;
}

//==============================================================================
template <typename BV>
bool BVNode<BV>::overlap(const BVNode& other) const
{
  return bv.overlap(other.bv);
}

//==============================================================================
template <typename BV>
typename BVNode<BV>::S BVNode<BV>::distance(
    const BVNode& other, Vector3<S>* P1, Vector3<S>* P2) const
{
  return bv.distance(other.bv, P1, P2);
}

//==============================================================================
template <typename BV>
Vector3<typename BVNode<BV>::S> BVNode<BV>::getCenter() const
{
  return bv.center();
}

//==============================================================================
template <typename S, typename BV>
struct GetOrientationImpl
{
  static Matrix3<S> run(const BVNode<BV>& /*node*/)
  {
    return Matrix3<S>::Identity();
  }
};

//==============================================================================
template <typename BV>
Matrix3<typename BV::S> BVNode<BV>::getOrientation() const
{
  return GetOrientationImpl<typename BV::S, BV>::run(bv);
}

//==============================================================================
template <typename S>
struct GetOrientationImpl<S, OBB<S>>
{
  static Matrix3<S> run(const OBB<S>& bv)
  {
    return bv.axis;
  }
};

//==============================================================================
template <typename S>
struct GetOrientationImpl<S, RSS<S>>
{
  static Matrix3<S> run(const RSS<S>& bv)
  {
    return bv.axis;
  }
};

//==============================================================================
template <typename S>
struct GetOrientationImpl<S, OBBRSS<S>>
{
  static Matrix3<S> run(const OBBRSS<S>& bv)
  {
    return bv.obb.axis;
  }
};

} // namespace fcl

#endif

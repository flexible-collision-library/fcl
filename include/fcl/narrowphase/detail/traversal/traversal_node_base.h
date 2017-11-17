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

#ifndef FCL_TRAVERSAL_TRAVERSALNODEBASE_H
#define FCL_TRAVERSAL_TRAVERSALNODEBASE_H

#include "fcl/common/types.h"

namespace fcl
{

namespace detail
{

/// @brief Node structure encoding the information required for traversal.
template <typename S>
class FCL_EXPORT TraversalNodeBase
{
public:
  virtual ~TraversalNodeBase();

  virtual void preprocess();
  
  virtual void postprocess();

  /// @brief Whether b is a leaf node in the first BVH tree 
  virtual bool isFirstNodeLeaf(int b) const;

  /// @brief Whether b is a leaf node in the second BVH tree
  virtual bool isSecondNodeLeaf(int b) const;

  /// @brief Traverse the subtree of the node in the first tree first
  virtual bool firstOverSecond(int b1, int b2) const;

  /// @brief Get the left child of the node b in the first tree
  virtual int getFirstLeftChild(int b) const;

  /// @brief Get the right child of the node b in the first tree
  virtual int getFirstRightChild(int b) const;

  /// @brief Get the left child of the node b in the second tree
  virtual int getSecondLeftChild(int b) const;

  /// @brief Get the right child of the node b in the second tree
  virtual int getSecondRightChild(int b) const;

  /// @brief Enable statistics (verbose mode)
  virtual void enableStatistics(bool enable) = 0;

  /// @brief configuation of first object
  Transform3<S> tf1;

  /// @brief configuration of second object
  Transform3<S> tf2;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/traversal/traversal_node_base-inl.h"

#endif

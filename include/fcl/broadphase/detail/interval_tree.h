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

#ifndef FCL_INTERVAL_TREE_H
#define FCL_INTERVAL_TREE_H

#include <deque>
#include <limits>
#include <cstdlib>
#include <iostream>
#include "fcl/broadphase/detail/interval_tree_node.h"

namespace fcl {
namespace detail {

/// @brief Class describes the information needed when we take the
/// right branch in searching for intervals but possibly come back
/// and check the left branch as well.
template <typename S>
struct FCL_EXPORT it_recursion_node
{
public:
  IntervalTreeNode<S>* start_node;

  unsigned int parent_index;

  bool try_right_branch;
};

using it_recursion_nodef = it_recursion_node<float>;
using it_recursion_noded = it_recursion_node<double>;

extern template
struct it_recursion_node<double>;

/// @brief Interval tree
template <typename S>
class FCL_EXPORT IntervalTree
{
public:

  IntervalTree();

  ~IntervalTree();

  /// @brief Print the whole interval tree
  void print() const;

  /// @brief Delete one node of the interval tree
  SimpleInterval<S>* deleteNode(IntervalTreeNode<S>* node);

  /// @brief delete node stored a given interval
  void deleteNode(SimpleInterval<S>* ivl);

  /// @brief Insert one node of the interval tree
  IntervalTreeNode<S>* insert(SimpleInterval<S>* new_interval);

  /// @brief get the predecessor of a given node
  IntervalTreeNode<S>* getPredecessor(IntervalTreeNode<S>* node) const;

  /// @brief Get the successor of a given node
  IntervalTreeNode<S>* getSuccessor(IntervalTreeNode<S>* node) const;

  /// @brief Return result for a given query
  std::deque<SimpleInterval<S>*> query(S low, S high);

protected:

  IntervalTreeNode<S>* root;

  IntervalTreeNode<S>* nil;

  /// @brief left rotation of tree node
  void leftRotate(IntervalTreeNode<S>* node);

  /// @brief right rotation of tree node
  void rightRotate(IntervalTreeNode<S>* node);

  /// @brief Inserts node into the tree as if it were a regular binary tree
  void recursiveInsert(IntervalTreeNode<S>* node);

  /// @brief recursively print a subtree 
  void recursivePrint(IntervalTreeNode<S>* node) const;

  /// @brief recursively find the node corresponding to the interval
  IntervalTreeNode<S>* recursiveSearch(IntervalTreeNode<S>* node, SimpleInterval<S>* ivl) const;

  /// @brief Travels up to the root fixing the max_high fields after an insertion or deletion
  void fixupMaxHigh(IntervalTreeNode<S>* node);

  void deleteFixup(IntervalTreeNode<S>* node);

private:
  unsigned int recursion_node_stack_size;
  it_recursion_node<S>* recursion_node_stack;
  unsigned int current_parent;
  unsigned int recursion_node_stack_top;
};

using IntervalTreef = IntervalTree<float>;
using IntervalTreed = IntervalTree<double>;

} // namespace detail
} // namespace fcl

#include "fcl/broadphase/detail/interval_tree-inl.h"

#endif

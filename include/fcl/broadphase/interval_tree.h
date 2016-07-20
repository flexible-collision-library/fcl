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


#ifndef FCL_INTERVAL_TREE_H
#define FCL_INTERVAL_TREE_H

#include <deque>
#include <limits>

namespace fcl
{

/// @brief Interval trees implemented using red-black-trees as described in
/// the book Introduction_To_Algorithms_ by Cormen, Leisserson, and Rivest.
struct SimpleInterval
{
public:
  virtual ~SimpleInterval() {}
  
  virtual void print() {}

  /// @brief interval is defined as [low, high]
  double low, high;
};

/// @brief The node for interval tree
class IntervalTreeNode
{
  friend class IntervalTree;
public:
  /// @brief Print the interval node information: set left = nil and right = root
  void print(IntervalTreeNode* left, IntervalTreeNode* right) const;
  
  /// @brief Create an empty node
  IntervalTreeNode();

  /// @brief Create an node storing the interval
  IntervalTreeNode(SimpleInterval* new_interval);

  ~IntervalTreeNode();

protected:
  /// @brief interval stored in the node
  SimpleInterval* stored_interval;

  double key;

  double high;

  double max_high;

  /// @brief red or black node: if red = false then the node is black
  bool red;  

  IntervalTreeNode* left;

  IntervalTreeNode* right;

  IntervalTreeNode* parent;
};

struct it_recursion_node;

/// @brief Interval tree
class IntervalTree
{
public:

  IntervalTree();

  ~IntervalTree();

  /// @brief Print the whole interval tree
  void print() const;

  /// @brief Delete one node of the interval tree
  SimpleInterval* deleteNode(IntervalTreeNode* node);

  /// @brief delete node stored a given interval
  void deleteNode(SimpleInterval* ivl);

  /// @brief Insert one node of the interval tree
  IntervalTreeNode* insert(SimpleInterval* new_interval);

  /// @brief get the predecessor of a given node
  IntervalTreeNode* getPredecessor(IntervalTreeNode* node) const;

  /// @brief Get the successor of a given node
  IntervalTreeNode* getSuccessor(IntervalTreeNode* node) const;

  /// @brief Return result for a given query
  std::deque<SimpleInterval*> query(double low, double high);

protected:

  IntervalTreeNode* root;

  IntervalTreeNode* nil;

  /// @brief left rotation of tree node
  void leftRotate(IntervalTreeNode* node);

  /// @brief right rotation of tree node
  void rightRotate(IntervalTreeNode* node);

  /// @brief recursively insert a node
  void recursiveInsert(IntervalTreeNode* node);

  /// @brief recursively print a subtree 
  void recursivePrint(IntervalTreeNode* node) const;

  /// @brief recursively find the node corresponding to the interval
  IntervalTreeNode* recursiveSearch(IntervalTreeNode* node, SimpleInterval* ivl) const;

  /// @brief Travels up to the root fixing the max_high fields after an insertion or deletion
  void fixupMaxHigh(IntervalTreeNode* node);

  void deleteFixup(IntervalTreeNode* node);

private:
  unsigned int recursion_node_stack_size;
  it_recursion_node* recursion_node_stack;
  unsigned int current_parent;
  unsigned int recursion_node_stack_top;
};

}

#endif



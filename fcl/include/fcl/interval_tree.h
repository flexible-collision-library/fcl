/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

/** \brief Main namespace */
namespace fcl
{

/** \brief Interval trees implemented using red-black-trees as described in
 * the book Introduction_To_Algorithms_ by Cormen, Leisserson, and Rivest.
 * Can be replaced in part by boost::icl::interval_set, which is only supported after boost 1.46 and does not support delete node routine.
 */

struct Interval
{
public:
  Interval() {}
  virtual ~Interval() {}
  virtual void print() {}

  double low, high;
};

class IntervalTreeNode
{
  friend class IntervalTree;
public:
  /** \brief Print the interval node information: set left = nil and right = root */
  void print(IntervalTreeNode* left, IntervalTreeNode* right) const;

  IntervalTreeNode();

  IntervalTreeNode(Interval* new_interval);

  ~IntervalTreeNode();

protected:

  Interval* stored_interval;

  double key;

  double high;

  double max_high;

  bool red; /* if red = false then the node is black */

  IntervalTreeNode* left;

  IntervalTreeNode* right;

  IntervalTreeNode* parent;
};

/** \brief Class describes the information needed when we take the
 * right branch in searching for intervals but possibly come back
 * and check the left branch as well.
 */
struct it_recursion_node
{
public:
  IntervalTreeNode* start_node;

  unsigned int parent_index;

  bool try_right_branch;
};


/** \brief Interval tree */
class IntervalTree
{
public:

  IntervalTree();

  ~IntervalTree();

  /** \brief Print the whole interval tree */
  void print() const;

  /** \brief Delete one node of the interval tree */
  Interval* deleteNode(IntervalTreeNode* node);

  /** \brief Insert one node of the interval tree */
  IntervalTreeNode* insert(Interval* new_interval);

  /** \brief get the predecessor of a given node */
  IntervalTreeNode* getPredecessor(IntervalTreeNode* node) const;

  /** \brief Get the successor of a given node */
  IntervalTreeNode* getSuccessor(IntervalTreeNode* node) const;

  /** \brief Return result for a given query */
  std::deque<Interval*> query(double low, double high);

protected:

  IntervalTreeNode* root;

  IntervalTreeNode* nil;

  /** \brief left rotation of tree node */
  void leftRotate(IntervalTreeNode* node);

  /** \brief right rotation of tree node */
  void rightRotate(IntervalTreeNode* node);

  /** \brief recursively insert a node */
  void recursiveInsert(IntervalTreeNode* node);

  /** \brief recursively print a subtree */
  void recursivePrint(IntervalTreeNode* node) const;

  /** \brief Travels up to the root fixing the max_high fields after an insertion or deletion */
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



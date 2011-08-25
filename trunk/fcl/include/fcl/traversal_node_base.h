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

#ifndef FCL_TRAVERSAL_NODE_BASE_H
#define FCL_TRAVERSAL_NODE_BASE_H

#include "fcl/vec_3f.h"
#include "fcl/primitive.h"
#include "fcl/BVH_front.h"
#include "fcl/BVH_model.h"
#include <vector>


/** \brief Main namespace */
namespace fcl
{

class TraversalNodeBase
{
public:
  TraversalNodeBase() : enable_statistics(false) {}

  virtual ~TraversalNodeBase();

  /** \brief Whether b is a leaf node in the first BVH tree */
  virtual bool isFirstNodeLeaf(int b) const;

  /** \brief Whether b is a leaf node in the second BVH tree */
  virtual bool isSecondNodeLeaf(int b) const;

  /** \brief Traverse the subtree of the node in the first tree first */
  virtual bool firstOverSecond(int b1, int b2) const;

  /** \brief Get the left child of the node b in the first tree */
  virtual int getFirstLeftChild(int b) const;

  /** \brief Get the right child of the node b in the first tree */
  virtual int getFirstRightChild(int b) const;

  /** \brief Get the left child of the node b in the second tree */
  virtual int getSecondLeftChild(int b) const;

  /** \brief Get the right child of the node b in the second tree */
  virtual int getSecondRightChild(int b) const;

  void enableStatistics(bool enable) { enable_statistics = enable; }

  bool enable_statistics;
};

class CollisionTraversalNodeBase : public TraversalNodeBase
{
public:
  CollisionTraversalNodeBase() : TraversalNodeBase() {}

  virtual ~CollisionTraversalNodeBase();

  /** \brief BV test between b1 and b2 */
  virtual bool BVTesting(int b1, int b2) const;

  /** \brief Leaf test between node b1 and b2, if they are both leafs */
  virtual void leafTesting(int b1, int b2) const;

  /** \brief Check whether the traversal can stop */
  virtual bool canStop() const;
};

class DistanceTraversalNodeBase : public TraversalNodeBase
{
public:
  DistanceTraversalNodeBase() : TraversalNodeBase() {}

  virtual ~DistanceTraversalNodeBase();

  virtual BVH_REAL BVTesting(int b1, int b2) const;

  virtual void leafTesting(int b1, int b2) const;

  virtual bool canStop(BVH_REAL c) const;
};

}

#endif

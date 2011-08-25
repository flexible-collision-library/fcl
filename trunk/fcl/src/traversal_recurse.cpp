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


#include "fcl/traversal_recurse.h"

namespace fcl
{
void collisionRecurse(CollisionTraversalNodeBase* node, int b1, int b2, BVHFrontList* front_list)
{
  bool l1 = node->isFirstNodeLeaf(b1);
  bool l2 = node->isSecondNodeLeaf(b2);

  if(l1 && l2)
  {
    updateFrontList(front_list, b1, b2);

    if(node->BVTesting(b1, b2)) return;

    node->leafTesting(b1, b2);
    return;
  }

  if(node->BVTesting(b1, b2))
  {
    updateFrontList(front_list, b1, b2);
    return;
  }

  if(node->firstOverSecond(b1, b2))
  {
    int c1 = node->getFirstLeftChild(b1);
    int c2 = node->getFirstRightChild(b1);

    collisionRecurse(node, c1, b2, front_list);

    // early stop is disabled is front_list is used
    if(node->canStop() && !front_list) return;

    collisionRecurse(node, c2, b2, front_list);
  }
  else
  {
    int c1 = node->getSecondLeftChild(b2);
    int c2 = node->getSecondRightChild(b2);

    collisionRecurse(node, b1, c1, front_list);

    // early stop is disabled is front_list is used
    if(node->canStop() && !front_list) return;

    collisionRecurse(node, b1, c2, front_list);
  }
}

/** Recurse function for self collision
 * Make sure node is set correctly so that the first and second tree are the same
 */
void selfCollisionRecurse(CollisionTraversalNodeBase* node, int b, BVHFrontList* front_list)
{
  bool l = node->isFirstNodeLeaf(b);

  if(l) return;

  int c1 = node->getFirstLeftChild(b);
  int c2 = node->getFirstRightChild(b);

  selfCollisionRecurse(node, c1, front_list);

  selfCollisionRecurse(node, c2, front_list);

  collisionRecurse(node, c1, c2, front_list);
}

void distanceRecurse(DistanceTraversalNodeBase* node, int b1, int b2, BVHFrontList* front_list)
{
  bool l1 = node->isFirstNodeLeaf(b1);
  bool l2 = node->isSecondNodeLeaf(b2);

  if(l1 && l2)
  {
    updateFrontList(front_list, b1, b2);

    node->leafTesting(b1, b2);
    return;
  }

  int a1, a2, c1, c2;

  if(node->firstOverSecond(b1, b2))
  {
    a1 = node->getFirstLeftChild(b1);
    a2 = b2;
    c1 = node->getFirstRightChild(b1);
    c2 = b2;
  }
  else
  {
    a1 = b1;
    a2 = node->getSecondLeftChild(b2);
    c1 = b1;
    c2 = node->getSecondRightChild(b2);
  }

  BVH_REAL d1 = node->BVTesting(a1, a2);
  BVH_REAL d2 = node->BVTesting(c1, c2);

  if(d2 < d1)
  {
    if(!node->canStop(d2))
      distanceRecurse(node, c1, c2, front_list);
    else
      updateFrontList(front_list, c1, c2);

    if(!node->canStop(d1))
      distanceRecurse(node, a1, a2, front_list);
    else
      updateFrontList(front_list, a1, a2);
  }
  else
  {
    if(!node->canStop(d1))
      distanceRecurse(node, a1, a2, front_list);
    else
      updateFrontList(front_list, a1, a2);

    if(!node->canStop(d2))
      distanceRecurse(node, c1, c2, front_list);
    else
      updateFrontList(front_list, c1, c2);
  }
}


/** \brief Bounding volume test structure */
struct BVT
{
  /** \brief distance between bvs */
  BVH_REAL d;

  /** \brief bv indices for a pair of bvs in two models */
  int b1, b2;
};

/** \brief Comparer between two BVT */
struct BVT_Comparer
{
  bool operator() (const BVT& lhs, const BVT& rhs) const
  {
    return lhs.d > rhs.d;
  }
};

struct BVTQ
{
  BVTQ() : qsize(2) {}

  bool empty() const
  {
    return pq.empty();
  }

  size_t size() const
  {
    return pq.size();
  }

  const BVT& top() const
  {
    return pq.top();
  }

  void push(const BVT& x)
  {
    pq.push(x);
  }

  void pop()
  {
    pq.pop();
  }

  bool full() const
  {
    return ((int)pq.size() >= qsize - 1);
  }

  std::priority_queue<BVT, std::vector<BVT>, BVT_Comparer> pq;

  /** \brief Queue size */
  int qsize;
};


void distanceQueueRecurse(DistanceTraversalNodeBase* node, int b1, int b2, BVHFrontList* front_list, int qsize)
{
  BVTQ bvtq;
  bvtq.qsize = qsize;

  BVT min_test;
  min_test.b1 = b1;
  min_test.b2 = b2;

  while(1)
  {
    bool l1 = node->isFirstNodeLeaf(min_test.b1);
    bool l2 = node->isSecondNodeLeaf(min_test.b2);

    if(l1 && l2)
    {
      updateFrontList(front_list, min_test.b1, min_test.b2);

      node->leafTesting(min_test.b1, min_test.b2);
    }
    else if(bvtq.full())
    {
      // queue should not get two more tests, recur

      distanceQueueRecurse(node, min_test.b1, min_test.b2, front_list, qsize);
    }
    else
    {
      // queue capacity is not full yet
      BVT bvt1, bvt2;

      if(node->firstOverSecond(min_test.b1, min_test.b2))
      {
        int c1 = node->getFirstLeftChild(min_test.b1);
        int c2 = node->getFirstRightChild(min_test.b1);
        bvt1.b1 = c1;
        bvt1.b2 = min_test.b2;
        bvt1.d = node->BVTesting(bvt1.b1, bvt1.b2);

        bvt2.b1 = c2;
        bvt2.b2 = min_test.b2;
        bvt2.d = node->BVTesting(bvt2.b1, bvt2.b2);
      }
      else
      {
        int c1 = node->getSecondLeftChild(min_test.b2);
        int c2 = node->getSecondRightChild(min_test.b2);
        bvt1.b1 = min_test.b1;
        bvt1.b2 = c1;
        bvt1.d = node->BVTesting(bvt1.b1, bvt1.b2);

        bvt2.b1 = min_test.b1;
        bvt2.b2 = c2;
        bvt2.d = node->BVTesting(bvt2.b1, bvt2.b2);
      }

      bvtq.push(bvt1);
      bvtq.push(bvt2);
    }

    if(bvtq.empty())
      break;
    else
    {
      min_test = bvtq.top();
      bvtq.pop();

      if(node->canStop(min_test.d))
      {
        updateFrontList(front_list, min_test.b1, min_test.b2);
        break;
      }
    }
  }
}

void propagateBVHFrontListCollisionRecurse(CollisionTraversalNodeBase* node, BVHFrontList* front_list)
{
  BVHFrontList::iterator front_iter;
  BVHFrontList append;
  for(front_iter = front_list->begin(); front_iter != front_list->end(); ++front_iter)
  {
    int b1 = front_iter->left;
    int b2 = front_iter->right;
    bool l1 = node->isFirstNodeLeaf(b1);
    bool l2 = node->isSecondNodeLeaf(b2);

    if(l1 & l2)
    {
      front_iter->valid = false; // the front node is no longer valid, in collideRecurse will add again.
      collisionRecurse(node, b1, b2, &append);
    }
    else
    {
      if(!node->BVTesting(b1, b2))
      {
        front_iter->valid = false;

        if(node->firstOverSecond(b1, b2))
        {
          int c1 = node->getFirstLeftChild(b1);
          int c2 = node->getFirstRightChild(b2);

          collisionRecurse(node, c1, b2, front_list);
          collisionRecurse(node, c2, b2, front_list);
        }
        else
        {
          int c1 = node->getSecondLeftChild(b2);
          int c2 = node->getSecondRightChild(b2);

          collisionRecurse(node, b1, c1, front_list);
          collisionRecurse(node, b1, c2, front_list);
        }
      }
    }
  }


  // clean the old front list (remove invalid node)
  for(front_iter = front_list->begin(); front_iter != front_list->end();)
  {
    if(!front_iter->valid)
      front_iter = front_list->erase(front_iter);
    else
      ++front_iter;
  }

  for(front_iter = append.begin(); front_iter != append.end(); ++front_iter)
  {
    front_list->push_back(*front_iter);
  }
}


}

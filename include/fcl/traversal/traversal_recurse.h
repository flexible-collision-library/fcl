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


#ifndef FCL_TRAVERSAL_RECURSE_H
#define FCL_TRAVERSAL_RECURSE_H

#include <queue>
#include "fcl/BVH/BVH_front.h"
#include "fcl/traversal/traversal_node_base.h"
#include "fcl/traversal/collision/collision_traversal_node_base.h"
#include "fcl/traversal/collision/mesh_collision_traversal_node.h"
#include "fcl/traversal/distance/distance_traversal_node_base.h"

namespace fcl
{

/// @brief Recurse function for collision
template <typename Scalar>
void collisionRecurse(CollisionTraversalNodeBase<Scalar>* node, int b1, int b2, BVHFrontList* front_list);

/// @brief Recurse function for collision, specialized for OBBd type
template <typename Scalar>
void collisionRecurse(MeshCollisionTraversalNodeOBB<Scalar>* node, int b1, int b2, const Matrix3<Scalar>& R, const Vector3<Scalar>& T, BVHFrontList* front_list);

/// @brief Recurse function for collision, specialized for RSSd type
template <typename Scalar>
void collisionRecurse(MeshCollisionTraversalNodeRSS<Scalar>* node, int b1, int b2, const Matrix3<Scalar>& R, const Vector3<Scalar>& T, BVHFrontList* front_list);

/// @brief Recurse function for self collision. Make sure node is set correctly so that the first and second tree are the same
template <typename Scalar>
void selfCollisionRecurse(CollisionTraversalNodeBase<Scalar>* node, int b, BVHFrontList* front_list);

/// @brief Recurse function for distance
template <typename Scalar>
void distanceRecurse(DistanceTraversalNodeBase<Scalar>* node, int b1, int b2, BVHFrontList* front_list);

/// @brief Recurse function for distance, using queue acceleration
template <typename Scalar>
void distanceQueueRecurse(DistanceTraversalNodeBase<Scalar>* node, int b1, int b2, BVHFrontList* front_list, int qsize);

/// @brief Recurse function for front list propagation
template <typename Scalar>
void propagateBVHFrontListCollisionRecurse(CollisionTraversalNodeBase<Scalar>* node, BVHFrontList* front_list);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
void collisionRecurse(CollisionTraversalNodeBase<Scalar>* node, int b1, int b2, BVHFrontList* front_list)
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

//==============================================================================
template <typename Scalar>
void collisionRecurse(MeshCollisionTraversalNodeOBB<Scalar>* node, int b1, int b2, const Matrix3<Scalar>& R, const Vector3<Scalar>& T, BVHFrontList* front_list)
{
  bool l1 = node->isFirstNodeLeaf(b1);
  bool l2 = node->isSecondNodeLeaf(b2);

  if(l1 && l2)
  {
    updateFrontList(front_list, b1, b2);

    if(node->BVTesting(b1, b2, R, T)) return;

    node->leafTesting(b1, b2, R, T);
    return;
  }

  if(node->BVTesting(b1, b2, R, T))
  {
    updateFrontList(front_list, b1, b2);
    return;
  }

  Vector3<Scalar> temp;

  if(node->firstOverSecond(b1, b2))
  {
    int c1 = node->getFirstLeftChild(b1);
    int c2 = node->getFirstRightChild(b1);

    const OBB<Scalar>& bv1 = node->model1->getBV(c1).bv;

    Matrix3<Scalar> Rc = R.transpose() * bv1.frame.linear();
    temp = T - bv1.frame.translation();
    Vector3<Scalar> Tc = temp.transpose() * bv1.frame.linear();

    collisionRecurse(node, c1, b2, Rc, Tc, front_list);

    // early stop is disabled is front_list is used
    if(node->canStop() && !front_list) return;

    const OBB<Scalar>& bv2 = node->model1->getBV(c2).bv;

    Rc = R.transpose() * bv2.frame.linear();
    temp = T - bv2.frame.translation();
    Tc = temp.transpose() * bv2.frame.linear();

    collisionRecurse(node, c2, b2, Rc, Tc, front_list);
  }
  else
  {
    int c1 = node->getSecondLeftChild(b2);
    int c2 = node->getSecondRightChild(b2);

    const OBB<Scalar>& bv1 = node->model2->getBV(c1).bv;
    Matrix3<Scalar> Rc;
    temp = R * bv1.frame.linear().col(0);
    Rc(0, 0) = temp[0]; Rc(1, 0) = temp[1]; Rc(2, 0) = temp[2];
    temp = R * bv1.frame.linear().col(1);
    Rc(0, 1) = temp[0]; Rc(1, 1) = temp[1]; Rc(2, 1) = temp[2];
    temp = R * bv1.frame.linear().col(2);
    Rc(0, 2) = temp[0]; Rc(1, 2) = temp[1]; Rc(2, 2) = temp[2];
    Vector3<Scalar> Tc = R * bv1.frame.translation() + T;

    collisionRecurse(node, b1, c1, Rc, Tc, front_list);

    // early stop is disabled is front_list is used
    if(node->canStop() && !front_list) return;

    const OBB<Scalar>& bv2 = node->model2->getBV(c2).bv;
    temp = R * bv2.frame.linear().col(0);
    Rc(0, 0) = temp[0]; Rc(1, 0) = temp[1]; Rc(2, 0) = temp[2];
    temp = R * bv2.frame.linear().col(1);
    Rc(0, 1) = temp[0]; Rc(1, 1) = temp[1]; Rc(2, 1) = temp[2];
    temp = R * bv2.frame.linear().col(2);
    Rc(0, 2) = temp[0]; Rc(1, 2) = temp[1]; Rc(2, 2) = temp[2];
    Tc = R * bv2.frame.translation() + T;

    collisionRecurse(node, b1, c2, Rc, Tc, front_list);
  }
}

//==============================================================================
template <typename Scalar>
void collisionRecurse(MeshCollisionTraversalNodeRSS<Scalar>* node, int b1, int b2, const Matrix3<Scalar>& R, const Vector3<Scalar>& T, BVHFrontList* front_list)
{
  // Do nothing
}

//==============================================================================
/** Recurse function for self collision
 * Make sure node is set correctly so that the first and second tree are the same
 */
template <typename Scalar>
void selfCollisionRecurse(CollisionTraversalNodeBase<Scalar>* node, int b, BVHFrontList* front_list)
{
  bool l = node->isFirstNodeLeaf(b);

  if(l) return;

  int c1 = node->getFirstLeftChild(b);
  int c2 = node->getFirstRightChild(b);

  selfCollisionRecurse(node, c1, front_list);
  if(node->canStop() && !front_list) return;

  selfCollisionRecurse(node, c2, front_list);
  if(node->canStop() && !front_list) return;

  collisionRecurse(node, c1, c2, front_list);
}

//==============================================================================
template <typename Scalar>
void distanceRecurse(DistanceTraversalNodeBase<Scalar>* node, int b1, int b2, BVHFrontList* front_list)
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

  Scalar d1 = node->BVTesting(a1, a2);
  Scalar d2 = node->BVTesting(c1, c2);

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

//==============================================================================
/** \brief Bounding volume test structure */
template <typename Scalar>
struct BVT
{
  /** \brief distance between bvs */
  Scalar d;

  /** \brief bv indices for a pair of bvs in two models */
  int b1, b2;
};

//==============================================================================
/** \brief Comparer between two BVT */
template <typename Scalar>
struct BVT_Comparer
{
  bool operator() (const BVT<Scalar>& lhs, const BVT<Scalar>& rhs) const
  {
    return lhs.d > rhs.d;
  }
};

//==============================================================================
template <typename Scalar>
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

  const BVT<Scalar>& top() const
  {
    return pq.top();
  }

  void push(const BVT<Scalar>& x)
  {
    pq.push(x);
  }

  void pop()
  {
    pq.pop();
  }

  bool full() const
  {
    return (pq.size() + 1 >= qsize);
  }

  std::priority_queue<BVT<Scalar>, std::vector<BVT<Scalar>>, BVT_Comparer<Scalar>> pq;

  /** \brief Queue size */
  unsigned int qsize;
};

//==============================================================================
template <typename Scalar>
void distanceQueueRecurse(DistanceTraversalNodeBase<Scalar>* node, int b1, int b2, BVHFrontList* front_list, int qsize)
{
  BVTQ<Scalar> bvtq;
  bvtq.qsize = qsize;

  BVT<Scalar> min_test;
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
      BVT<Scalar> bvt1, bvt2;

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

//==============================================================================
template <typename Scalar>
void propagateBVHFrontListCollisionRecurse(CollisionTraversalNodeBase<Scalar>* node, BVHFrontList* front_list)
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
          int c2 = node->getFirstRightChild(b1);

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

} // namespace fcl

#endif

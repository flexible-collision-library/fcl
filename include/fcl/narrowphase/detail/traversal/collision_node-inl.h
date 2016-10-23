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

#ifndef FCL_COLLISION_NODE_INL_H
#define FCL_COLLISION_NODE_INL_H

#include "fcl/narrowphase/detail/traversal/collision_node.h"

/// @brief collision and distance function on traversal nodes. these functions provide a higher level abstraction for collision functions provided in collision_func_matrix
namespace fcl
{

namespace detail
{

//==============================================================================
extern template
void collide(CollisionTraversalNodeBase<double>* node, BVHFrontList* front_list);

//==============================================================================
extern template
void selfCollide(CollisionTraversalNodeBase<double>* node, BVHFrontList* front_list);

//==============================================================================
extern template
void distance(DistanceTraversalNodeBase<double>* node, BVHFrontList* front_list, int qsize);

//==============================================================================
extern template
void collide2(MeshCollisionTraversalNodeOBB<double>* node, BVHFrontList* front_list);

//==============================================================================
extern template
void collide2(MeshCollisionTraversalNodeRSS<double>* node, BVHFrontList* front_list);

//==============================================================================
template <typename S>
void collide(CollisionTraversalNodeBase<S>* node, BVHFrontList* front_list)
{
  if(front_list && front_list->size() > 0)
  {
    propagateBVHFrontListCollisionRecurse(node, front_list);
  }
  else
  {
    collisionRecurse(node, 0, 0, front_list);
  }
}

//==============================================================================
template <typename S>
void collide2(MeshCollisionTraversalNodeOBB<S>* node, BVHFrontList* front_list)
{
  if(front_list && front_list->size() > 0)
  {
    propagateBVHFrontListCollisionRecurse(node, front_list);
  }
  else
  {
    Matrix3<S> Rtemp, R;
    Vector3<S> Ttemp, T;
    Rtemp = node->R * node->model2->getBV(0).getOrientation();
    R = node->model1->getBV(0).getOrientation().transpose() * Rtemp;
    Ttemp = node->R * node->model2->getBV(0).getCenter() + node->T;
    Ttemp -= node->model1->getBV(0).getCenter();
    T = node->model1->getBV(0).getOrientation().transpose() * Ttemp;

    collisionRecurse(node, 0, 0, R, T, front_list);
  }
}

//==============================================================================
template <typename S>
void collide2(MeshCollisionTraversalNodeRSS<S>* node, BVHFrontList* front_list)
{
  if(front_list && front_list->size() > 0)
  {
    propagateBVHFrontListCollisionRecurse(node, front_list);
  }
  else
  {
    collisionRecurse(node, 0, 0, node->R, node->T, front_list);
  }
}

//==============================================================================
template <typename S>
void selfCollide(CollisionTraversalNodeBase<S>* node, BVHFrontList* front_list)
{

  if(front_list && front_list->size() > 0)
  {
    propagateBVHFrontListCollisionRecurse(node, front_list);
  }
  else
  {
    selfCollisionRecurse(node, 0, front_list);
  }
}

//==============================================================================
template <typename S>
void distance(DistanceTraversalNodeBase<S>* node, BVHFrontList* front_list, int qsize)
{
  node->preprocess();

  if(qsize <= 2)
    distanceRecurse(node, 0, 0, front_list);
  else
    distanceQueueRecurse(node, 0, 0, front_list, qsize);

  node->postprocess();
}

} // namespace detail
} // namespace fcl

#endif

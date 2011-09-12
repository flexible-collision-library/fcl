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


#include "fcl/collision_node.h"
#include "fcl/traversal_recurse.h"

namespace fcl
{

void collide(CollisionTraversalNodeBase* node, BVHFrontList* front_list)
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


void selfCollide(CollisionTraversalNodeBase* node, BVHFrontList* front_list)
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

void distance(DistanceTraversalNodeBase* node, BVHFrontList* front_list, int qsize)
{
  if(qsize <= 2)
    distanceRecurse(node, 0, 0, front_list);
  else
    distanceQueueRecurse(node, 0, 0, front_list, qsize);
}

void distance(MeshDistanceTraversalNodeRSS* node, BVHFrontList* front_list, int qsize)
{
  Triangle last_tri1 = node->tri_indices1[node->last_tri_id1];
  Triangle last_tri2 = node->tri_indices2[node->last_tri_id2];

  Vec3f last_tri1_points[3];
  Vec3f last_tri2_points[3];

  last_tri1_points[0] = node->vertices1[last_tri1[0]];
  last_tri1_points[1] = node->vertices1[last_tri1[1]];
  last_tri1_points[2] = node->vertices1[last_tri1[2]];

  last_tri2_points[0] = node->vertices2[last_tri2[0]];
  last_tri2_points[1] = node->vertices2[last_tri2[1]];
  last_tri2_points[2] = node->vertices2[last_tri2[2]];

  Vec3f last_tri_P, last_tri_Q;

  node->min_distance = TriangleDistance::triDistance(last_tri1_points[0], last_tri1_points[1], last_tri1_points[2],
                                                     last_tri2_points[0], last_tri2_points[1], last_tri2_points[2],
                                                     node->R, node->T, last_tri_P, last_tri_Q);
  node->p1 = last_tri_P;
  node->p2 = matTransMulVec(node->R, last_tri_Q - node->T);


  if(qsize <= 2)
    distanceRecurse(node, 0, 0, front_list);
  else
    distanceQueueRecurse(node, 0, 0, front_list, qsize);

  Vec3f u = node->p2 - node->T;
  node->p2 = matTransMulVec(node->R, u);
}


}

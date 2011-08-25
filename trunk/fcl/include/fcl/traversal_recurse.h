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


#ifndef FCL_TRAVERSAL_RECURSE_H
#define FCL_TRAVERSAL_RECURSE_H

#include "fcl/traversal_node_base.h"
#include "fcl/BVH_front.h"
#include <queue>

namespace fcl
{

inline void updateFrontList(BVHFrontList* front_list, int b1, int b2)
{
  if(front_list) front_list->push_back(BVHFrontNode(b1, b2));
}

void collisionRecurse(CollisionTraversalNodeBase* node, int b1, int b2, BVHFrontList* front_list);

/** Recurse function for self collision
 * Make sure node is set correctly so that the first and second tree are the same
 */
void selfCollisionRecurse(CollisionTraversalNodeBase* node, int b, BVHFrontList* front_list);

void distanceRecurse(DistanceTraversalNodeBase* node, int b1, int b2, BVHFrontList* front_list);

void distanceQueueRecurse(DistanceTraversalNodeBase* node, int b1, int b2, BVHFrontList* front_list, int qsize);

void propagateBVHFrontListCollisionRecurse(CollisionTraversalNodeBase* node, BVHFrontList* front_list);


}

#endif

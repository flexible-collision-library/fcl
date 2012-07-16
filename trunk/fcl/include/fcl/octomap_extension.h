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


#ifndef FCL_OCTOMAP_EXTENSION_H
#define FCL_OCTOMAP_EXTENSION_H

#include <set>
#include <octomap/octomap.h>
#include "fcl/broad_phase_collision.h"

namespace fcl
{


bool defaultCollisionCostFunction(CollisionObject* o1, CollisionObject* o2, void* cdata, FCL_REAL& cost);

bool defaultCollisionCostExtFunction(CollisionObject* o1, CollisionObject* o2, void* cdata, FCL_REAL& cost, std::set<octomap::OcTreeNode*>& nodes);


typedef bool (*CollisionCostCallBack)(CollisionObject* o1, CollisionObject* o2, void* cdata, FCL_REAL& cost);

typedef bool (*CollisionCostCallBackExt)(CollisionObject* o1, CollisionObject* o2, void* cdata, FCL_REAL& cost, std::set<octomap::OcTreeNode*>& nodes);


void collide(DynamicAABBTreeCollisionManager* manager, octomap::OcTree* octree, void* cdata, CollisionCallBack callback);

FCL_REAL collideCost(DynamicAABBTreeCollisionManager* manager, octomap::OcTree* octree, void* cdata, CollisionCostCallBack callback);

FCL_REAL collideCost(DynamicAABBTreeCollisionManager* manager, octomap::OcTree* octree, void* cdata, CollisionCostCallBackExt callback, std::set<octomap::OcTreeNode*>& nodes);


}

#endif

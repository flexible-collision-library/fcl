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

#ifndef FCL_COLLISION_NODE_H
#define FCL_COLLISION_NODE_H

#include "fcl/geometry/bvh/detail/BVH_front.h"
#include "fcl/narrowphase/detail/traversal/traversal_recurse.h"
#include "fcl/narrowphase/detail/traversal/collision/collision_traversal_node_base.h"
#include "fcl/narrowphase/detail/traversal/collision/mesh_collision_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/distance/distance_traversal_node_base.h"

/// @brief collision and distance function on traversal nodes. these functions provide a higher level abstraction for collision functions provided in collision_func_matrix
namespace fcl
{

namespace detail
{

/// @brief collision on collision traversal node; can use front list to accelerate
template <typename S>
FCL_EXPORT
void collide(CollisionTraversalNodeBase<S>* node, BVHFrontList* front_list = nullptr);

/// @brief self collision on collision traversal node; can use front list to accelerate
template <typename S>
FCL_EXPORT
void selfCollide(CollisionTraversalNodeBase<S>* node, BVHFrontList* front_list = nullptr);

/// @brief distance computation on distance traversal node; can use front list to accelerate
template <typename S>
FCL_EXPORT
void distance(DistanceTraversalNodeBase<S>* node, BVHFrontList* front_list = nullptr, int qsize = 2);

/// @brief special collision on OBB traversal node
template <typename S>
FCL_EXPORT
void collide2(MeshCollisionTraversalNodeOBB<S>* node, BVHFrontList* front_list = nullptr);

/// @brief special collision on RSS traversal node
template <typename S>
FCL_EXPORT
void collide2(MeshCollisionTraversalNodeRSS<S>* node, BVHFrontList* front_list = nullptr);

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/traversal/collision_node-inl.h"

#endif

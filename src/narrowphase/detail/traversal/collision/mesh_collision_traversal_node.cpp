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

#define FCL_NARROWPHASE_DETAIL_TRAVERSAL_COLLISION_MESH_COLLISION_TRAVERSAL_NODE_BUILDING
#include "fcl/narrowphase/detail/traversal/collision/mesh_collision_traversal_node-inl.h"

namespace fcl
{

namespace detail
{

//==============================================================================
template
class FCL_EXPORT MeshCollisionTraversalNodeOBB<double>;

//==============================================================================
template
FCL_EXPORT
bool initialize(
    MeshCollisionTraversalNodeOBB<double>& node,
    const BVHModel<OBB<double>>& model1,
    const Transform3<double>& tf1,
    const BVHModel<OBB<double>>& model2,
    const Transform3<double>& tf2,
    const CollisionRequest<double>& request,
    CollisionResult<double>& result);

//==============================================================================
template
class FCL_EXPORT MeshCollisionTraversalNodeRSS<double>;

//==============================================================================
template
FCL_EXPORT
bool initialize(
    MeshCollisionTraversalNodeRSS<double>& node,
    const BVHModel<RSS<double>>& model1,
    const Transform3<double>& tf1,
    const BVHModel<RSS<double>>& model2,
    const Transform3<double>& tf2,
    const CollisionRequest<double>& request,
    CollisionResult<double>& result);

//==============================================================================
template
class FCL_EXPORT MeshCollisionTraversalNodekIOS<double>;

//==============================================================================
template
FCL_EXPORT
bool initialize(
    MeshCollisionTraversalNodekIOS<double>& node,
    const BVHModel<kIOS<double>>& model1,
    const Transform3<double>& tf1,
    const BVHModel<kIOS<double>>& model2,
    const Transform3<double>& tf2,
    const CollisionRequest<double>& request,
    CollisionResult<double>& result);

//==============================================================================
template
class FCL_EXPORT MeshCollisionTraversalNodeOBBRSS<double>;

//==============================================================================
template
FCL_EXPORT
bool initialize(
    MeshCollisionTraversalNodeOBBRSS<double>& node,
    const BVHModel<OBBRSS<double>>& model1,
    const Transform3<double>& tf1,
    const BVHModel<OBBRSS<double>>& model2,
    const Transform3<double>& tf2,
    const CollisionRequest<double>& request,
    CollisionResult<double>& result);

} // namespace detail
} // namespace fcl

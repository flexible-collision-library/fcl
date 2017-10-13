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

#ifndef FCL_TRAVERSAL_MESHCONTINUOUSCOLLISIONTRAVERSALNODE_H
#define FCL_TRAVERSAL_MESHCONTINUOUSCOLLISIONTRAVERSALNODE_H

#include "fcl/narrowphase/detail/traversal/collision/bvh_collision_traversal_node.h"

namespace fcl
{

namespace detail
{

/// @brief Traversal node for continuous collision between BVH models
template <typename S>
struct FCL_EXPORT BVHContinuousCollisionPair
{
  BVHContinuousCollisionPair();

  BVHContinuousCollisionPair(int id1_, int id2_, S time);

  /// @brief The index of one in-collision primitive
  int id1;

  /// @brief The index of the other in-collision primitive
  int id2;

  /// @brief Collision time normalized in [0, 1]. The collision time out of [0, 1] means collision-free
  S collision_time;
};

/// @brief Traversal node for continuous collision between meshes
template <typename BV>
class FCL_EXPORT MeshContinuousCollisionTraversalNode
    : public BVHCollisionTraversalNode<BV>
{
public:

  using S = typename BV::S;

  MeshContinuousCollisionTraversalNode();

  /// @brief Intersection testing between leaves (two triangles)
  void leafTesting(int b1, int b2) const;

  /// @brief Whether the traversal process can stop early
  bool canStop() const;

  Vector3<S>* vertices1;
  Vector3<S>* vertices2;

  Triangle* tri_indices1;
  Triangle* tri_indices2;

  Vector3<S>* prev_vertices1;
  Vector3<S>* prev_vertices2;

  mutable int num_vf_tests;
  mutable int num_ee_tests;

  mutable std::vector<BVHContinuousCollisionPair<S>> pairs;

  mutable S time_of_contact;
};

/// @brief Initialize traversal node for continuous collision detection between
/// two meshes
template <typename BV>
FCL_EXPORT
bool initialize(
    MeshContinuousCollisionTraversalNode<BV>& node,
    const BVHModel<BV>& model1,
    const Transform3<typename BV::S>& tf1,
    const BVHModel<BV>& model2,
    const Transform3<typename BV::S>& tf2,
    const CollisionRequest<typename BV::S>& request);

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/traversal/collision/mesh_continuous_collision_traversal_node-inl.h"

#endif

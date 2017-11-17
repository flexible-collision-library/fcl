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

#ifndef FCL_TRAVERSAL_SHAPEMESHCOLLISIONTRAVERSALNODE_H
#define FCL_TRAVERSAL_SHAPEMESHCOLLISIONTRAVERSALNODE_H

#include "fcl/geometry/shape/utility.h"
#include "fcl/narrowphase/detail/traversal/collision/shape_bvh_collision_traversal_node.h"

namespace fcl
{

namespace detail
{

/// @brief Traversal node for collision between shape and mesh
template <typename Shape, typename BV, typename NarrowPhaseSolver>
class FCL_EXPORT ShapeMeshCollisionTraversalNode
    : public ShapeBVHCollisionTraversalNode<Shape, BV>
{
public:
  using S = typename BV::S;

  ShapeMeshCollisionTraversalNode();

  /// @brief Intersection testing between leaves (one shape and one triangle)
  void leafTesting(int b1, int b2) const;

  /// @brief Whether the traversal process can stop early
  bool canStop() const;

  Vector3<S>* vertices;
  Triangle* tri_indices;

  S cost_density;

  const NarrowPhaseSolver* nsolver;
};

/// @brief Initialize traversal node for collision between one mesh and one
/// shape, given current object transform
template <typename Shape, typename BV, typename NarrowPhaseSolver>
FCL_EXPORT
bool initialize(
    ShapeMeshCollisionTraversalNode<Shape, BV, NarrowPhaseSolver>& node,
    const Shape& model1,
    const Transform3<typename BV::S>& tf1,
    BVHModel<BV>& model2,
    Transform3<typename BV::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename BV::S>& request,
    CollisionResult<typename BV::S>& result,
    bool use_refit = false, bool refit_bottomup = false);

/// @brief Traversal node for shape and mesh, when mesh BVH is one of the oriented node (OBB, RSS, OBBRSS, kIOS)
template <typename Shape, typename NarrowPhaseSolver>
class FCL_EXPORT ShapeMeshCollisionTraversalNodeOBB
    : public ShapeMeshCollisionTraversalNode<
    Shape, OBB<typename Shape::S>, NarrowPhaseSolver>
{
public:
  ShapeMeshCollisionTraversalNodeOBB();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;
};

/// @brief Initialize the traversal node for collision between one mesh and one
/// shape, specialized for OBB type
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
bool initialize(
    ShapeMeshCollisionTraversalNodeOBB<Shape, NarrowPhaseSolver>& node,
    const Shape& model1,
    const Transform3<typename Shape::S>& tf1,
    const BVHModel<OBB<typename Shape::S>>& model2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename Shape::S>& request,
    CollisionResult<typename Shape::S>& result);

template <typename Shape, typename NarrowPhaseSolver>
class FCL_EXPORT ShapeMeshCollisionTraversalNodeRSS
    : public ShapeMeshCollisionTraversalNode<Shape, RSS<typename Shape::S>, NarrowPhaseSolver>
{
public:
  ShapeMeshCollisionTraversalNodeRSS();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

};

/// @brief Initialize the traversal node for collision between one mesh and one
/// shape, specialized for RSS type
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
bool initialize(
    ShapeMeshCollisionTraversalNodeRSS<Shape, NarrowPhaseSolver>& node,
    const Shape& model1,
    const Transform3<typename Shape::S>& tf1,
    const BVHModel<RSS<typename Shape::S>>& model2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename Shape::S>& request,
    CollisionResult<typename Shape::S>& result);

template <typename Shape, typename NarrowPhaseSolver>
class FCL_EXPORT ShapeMeshCollisionTraversalNodekIOS
    : public ShapeMeshCollisionTraversalNode<Shape, kIOS<typename Shape::S>, NarrowPhaseSolver>
{
public:
  ShapeMeshCollisionTraversalNodekIOS();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

};

/// @brief Initialize the traversal node for collision between one mesh and one
/// shape, specialized for kIOS type
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
bool initialize(
    ShapeMeshCollisionTraversalNodekIOS<Shape, NarrowPhaseSolver>& node,
    const Shape& model1,
    const Transform3<typename Shape::S>& tf1,
    const BVHModel<kIOS<typename Shape::S>>& model2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename Shape::S>& request,
    CollisionResult<typename Shape::S>& result);

template <typename Shape, typename NarrowPhaseSolver>
class FCL_EXPORT ShapeMeshCollisionTraversalNodeOBBRSS
    : public ShapeMeshCollisionTraversalNode<Shape, OBBRSS<typename Shape::S>, NarrowPhaseSolver>
{
public:
  ShapeMeshCollisionTraversalNodeOBBRSS();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

};

/// @brief Initialize the traversal node for collision between one mesh and one
/// shape, specialized for OBBRSS type
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
bool initialize(
    ShapeMeshCollisionTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>& node,
    const Shape& model1,
    const Transform3<typename Shape::S>& tf1,
    const BVHModel<OBBRSS<typename Shape::S>>& model2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename Shape::S>& request,
    CollisionResult<typename Shape::S>& result);

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/traversal/collision/shape_mesh_collision_traversal_node-inl.h"

#endif

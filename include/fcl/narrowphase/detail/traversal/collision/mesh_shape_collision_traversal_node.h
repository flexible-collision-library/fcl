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

#ifndef FCL_TRAVERSAL_MESHSHAPECOLLISIONTRAVERSALNODE_H
#define FCL_TRAVERSAL_MESHSHAPECOLLISIONTRAVERSALNODE_H

#include "fcl/geometry/shape/utility.h"
#include "fcl/narrowphase/detail/traversal/collision/bvh_shape_collision_traversal_node.h"

namespace fcl
{

namespace detail
{

/// @brief Traversal node for collision between mesh and shape
template <typename BV, typename Shape, typename NarrowPhaseSolver>
class FCL_EXPORT MeshShapeCollisionTraversalNode
    : public BVHShapeCollisionTraversalNode<BV, Shape>
{
public:

  using S = typename BV::S;

  MeshShapeCollisionTraversalNode();

  /// @brief Intersection testing between leaves (one triangle and one shape)
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
template <typename BV, typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
bool initialize(
    MeshShapeCollisionTraversalNode<BV, Shape, NarrowPhaseSolver>& node,
    BVHModel<BV>& model1,
    Transform3<typename BV::S>& tf1,
    const Shape& model2,
    const Transform3<typename BV::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename BV::S>& request,
    CollisionResult<typename BV::S>& result,
    bool use_refit = false, bool refit_bottomup = false);

template <typename BV, typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
void meshShapeCollisionOrientedNodeLeafTesting(
    int b1,
    int b2,
    const BVHModel<BV>* model1,
    const Shape& model2,
    Vector3<typename BV::S>* vertices,
    Triangle* tri_indices,
    const Transform3<typename BV::S>& tf1,
    const Transform3<typename BV::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    bool enable_statistics,
    typename BV::S cost_density,
    int& num_leaf_tests,
    const CollisionRequest<typename BV::S>& request,
    CollisionResult<typename BV::S>& result);

/// @brief Traversal node for mesh and shape, when mesh BVH is one of the oriented node (OBB, RSS, OBBRSS, kIOS)
template <typename Shape, typename NarrowPhaseSolver>
class FCL_EXPORT MeshShapeCollisionTraversalNodeOBB
    : public MeshShapeCollisionTraversalNode<
          OBB<typename Shape::S>, Shape, NarrowPhaseSolver>
{
public:
  MeshShapeCollisionTraversalNodeOBB();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

};

/// @brief Initialize the traversal node for collision between one mesh and one
/// shape, specialized for OBB type
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
bool initialize(
    MeshShapeCollisionTraversalNodeOBB<Shape, NarrowPhaseSolver>& node,
    const BVHModel<OBB<typename Shape::S>>& model1,
    const Transform3<typename Shape::S>& tf1,
    const Shape& model2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename Shape::S>& request,
    CollisionResult<typename Shape::S>& result);

template <typename Shape, typename NarrowPhaseSolver>
class FCL_EXPORT MeshShapeCollisionTraversalNodeRSS
    : public MeshShapeCollisionTraversalNode<
          RSS<typename Shape::S>, Shape, NarrowPhaseSolver>
{
public:
  MeshShapeCollisionTraversalNodeRSS();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

};

/// @brief Initialize the traversal node for collision between one mesh and one
/// shape, specialized for RSS type
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
bool initialize(
    MeshShapeCollisionTraversalNodeRSS<Shape, NarrowPhaseSolver>& node,
    const BVHModel<RSS<typename Shape::S>>& model1,
    const Transform3<typename Shape::S>& tf1,
    const Shape& model2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename Shape::S>& request,
    CollisionResult<typename Shape::S>& result);

template <typename Shape, typename NarrowPhaseSolver>
class FCL_EXPORT MeshShapeCollisionTraversalNodekIOS
    : public MeshShapeCollisionTraversalNode<
          kIOS<typename Shape::S>, Shape, NarrowPhaseSolver>
{
public:
  MeshShapeCollisionTraversalNodekIOS();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

};

/// @brief Initialize the traversal node for collision between one mesh and one
///  shape, specialized for kIOS type
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
bool initialize(
    MeshShapeCollisionTraversalNodekIOS<Shape, NarrowPhaseSolver>& node,
    const BVHModel<kIOS<typename Shape::S>>& model1,
    const Transform3<typename Shape::S>& tf1,
    const Shape& model2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename Shape::S>& request,
    CollisionResult<typename Shape::S>& result);

template <typename Shape, typename NarrowPhaseSolver>
class FCL_EXPORT MeshShapeCollisionTraversalNodeOBBRSS
    : public MeshShapeCollisionTraversalNode<
          OBBRSS<typename Shape::S>, Shape, NarrowPhaseSolver>
{
public:
  MeshShapeCollisionTraversalNodeOBBRSS();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

};

/// @brief Initialize the traversal node for collision between one mesh and one
/// shape, specialized for OBBRSS type
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
bool initialize(
    MeshShapeCollisionTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>& node,
    const BVHModel<OBBRSS<typename Shape::S>>& model1,
    const Transform3<typename Shape::S>& tf1,
    const Shape& model2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename Shape::S>& request,
    CollisionResult<typename Shape::S>& result);

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/traversal/collision/mesh_shape_collision_traversal_node-inl.h"

#endif

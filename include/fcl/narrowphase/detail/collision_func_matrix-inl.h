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

#ifndef FCL_COLLISION_FUNC_MATRIX_INL_H
#define FCL_COLLISION_FUNC_MATRIX_INL_H

#include "fcl/narrowphase/detail/collision_func_matrix.h"

#include "fcl/config.h"

#include "fcl/common/unused.h"

#include "fcl/narrowphase/collision_object.h"

#include "fcl/geometry/shape/box.h"
#include "fcl/geometry/shape/capsule.h"
#include "fcl/geometry/shape/cone.h"
#include "fcl/geometry/shape/convex.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/geometry/shape/ellipsoid.h"
#include "fcl/geometry/shape/halfspace.h"
#include "fcl/geometry/shape/plane.h"
#include "fcl/geometry/shape/sphere.h"
#include "fcl/geometry/shape/triangle_p.h"
#include "fcl/geometry/shape/utility.h"

#include "fcl/narrowphase/detail/traversal/collision_node.h"

#include "fcl/narrowphase/detail/traversal/collision/bvh_collision_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/collision/bvh_shape_collision_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/collision/collision_traversal_node_base.h"
#include "fcl/narrowphase/detail/traversal/collision/mesh_collision_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/collision/mesh_continuous_collision_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/collision/mesh_shape_collision_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/collision/shape_bvh_collision_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/collision/shape_collision_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/collision/shape_mesh_collision_traversal_node.h"

#if FCL_HAVE_OCTOMAP

#include "fcl/narrowphase/detail/traversal/octree/collision/mesh_octree_collision_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/octree/collision/octree_collision_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/octree/collision/octree_mesh_collision_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/octree/collision/octree_shape_collision_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/octree/collision/shape_octree_collision_traversal_node.h"

#endif // FCL_HAVE_OCTOMAP

namespace fcl
{

namespace detail
{

#if FCL_HAVE_OCTOMAP

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
std::size_t ShapeOcTreeCollide(
    const CollisionGeometry<typename Shape::S>* o1,
    const Transform3<typename Shape::S>& tf1,
    const CollisionGeometry<typename Shape::S>* o2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename Shape::S>& request,
    CollisionResult<typename Shape::S>& result)
{
  using S = typename Shape::S;

  if(request.isSatisfied(result)) return result.numContacts();

  ShapeOcTreeCollisionTraversalNode<Shape, NarrowPhaseSolver> node;
  const Shape* obj1 = static_cast<const Shape*>(o1);
  const OcTree<S>* obj2 = static_cast<const OcTree<S>*>(o2);
  OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
  collide(&node);

  return result.numContacts();
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
std::size_t OcTreeShapeCollide(
    const CollisionGeometry<typename Shape::S>* o1,
    const Transform3<typename Shape::S>& tf1,
    const CollisionGeometry<typename Shape::S>* o2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename Shape::S>& request,
    CollisionResult<typename Shape::S>& result)
{
  using S = typename Shape::S;

  if(request.isSatisfied(result)) return result.numContacts();

  OcTreeShapeCollisionTraversalNode<Shape, NarrowPhaseSolver> node;
  const OcTree<S>* obj1 = static_cast<const OcTree<S>*>(o1);
  const Shape* obj2 = static_cast<const Shape*>(o2);
  OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
  collide(&node);

  return result.numContacts();
}

//==============================================================================
template <typename NarrowPhaseSolver>
std::size_t OcTreeCollide(
    const CollisionGeometry<typename NarrowPhaseSolver::S>* o1,
    const Transform3<typename NarrowPhaseSolver::S>& tf1,
    const CollisionGeometry<typename NarrowPhaseSolver::S>* o2,
    const Transform3<typename NarrowPhaseSolver::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename NarrowPhaseSolver::S>& request,
    CollisionResult<typename NarrowPhaseSolver::S>& result)
{
  using S = typename NarrowPhaseSolver::S;

  if(request.isSatisfied(result)) return result.numContacts();

  OcTreeCollisionTraversalNode<NarrowPhaseSolver> node;
  const OcTree<S>* obj1 = static_cast<const OcTree<S>*>(o1);
  const OcTree<S>* obj2 = static_cast<const OcTree<S>*>(o2);
  OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
  collide(&node);

  return result.numContacts();
}

//==============================================================================
template <typename BV, typename NarrowPhaseSolver>
std::size_t OcTreeBVHCollide(
    const CollisionGeometry<typename BV::S>* o1,
    const Transform3<typename BV::S>& tf1,
    const CollisionGeometry<typename BV::S>* o2,
    const Transform3<typename BV::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename BV::S>& request,
    CollisionResult<typename BV::S>& result)
{
  using S = typename BV::S;

  if(request.isSatisfied(result)) return result.numContacts();

  if(request.enable_cost && request.use_approximate_cost)
  {
    CollisionRequest<S> no_cost_request(request); // request remove cost to avoid the exact but expensive cost computation between mesh and octree
    no_cost_request.enable_cost = false; // disable cost computation

    OcTreeMeshCollisionTraversalNode<BV, NarrowPhaseSolver> node;
    const OcTree<S>* obj1 = static_cast<const OcTree<S>*>(o1);
    const BVHModel<BV>* obj2 = static_cast<const BVHModel<BV>*>(o2);
    OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

    initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, no_cost_request, result);
    collide(&node);

    Box<S> box;
    Transform3<S> box_tf;
    constructBox(obj2->getBV(0).bv, tf2, box, box_tf); // compute the box for BVH's root node

    box.cost_density = obj2->cost_density;
    box.threshold_occupied = obj2->threshold_occupied;
    box.threshold_free = obj2->threshold_free;

    CollisionRequest<S> only_cost_request(result.numContacts(), false, request.num_max_cost_sources, true, false); // additional cost request, no contacts
    OcTreeShapeCollide<Box<S>, NarrowPhaseSolver>(o1, tf1, &box, box_tf, nsolver, only_cost_request, result);
  }
  else
  {
    OcTreeMeshCollisionTraversalNode<BV, NarrowPhaseSolver> node;
    const OcTree<S>* obj1 = static_cast<const OcTree<S>*>(o1);
    const BVHModel<BV>* obj2 = static_cast<const BVHModel<BV>*>(o2);
    OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

    initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
    collide(&node);
  }

  return result.numContacts();
}

//==============================================================================
template <typename BV, typename NarrowPhaseSolver>
std::size_t BVHOcTreeCollide(
    const CollisionGeometry<typename BV::S>* o1,
    const Transform3<typename BV::S>& tf1,
    const CollisionGeometry<typename BV::S>* o2,
    const Transform3<typename BV::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename BV::S>& request,
    CollisionResult<typename BV::S>& result)
{
  using S = typename BV::S;

  if(request.isSatisfied(result)) return result.numContacts();

  if(request.enable_cost && request.use_approximate_cost)
  {
    CollisionRequest<S> no_cost_request(request); // request remove cost to avoid the exact but expensive cost computation between mesh and octree
    no_cost_request.enable_cost = false; // disable cost computation

    MeshOcTreeCollisionTraversalNode<BV, NarrowPhaseSolver> node;
    const BVHModel<BV>* obj1 = static_cast<const BVHModel<BV>*>(o1);
    const OcTree<S>* obj2 = static_cast<const OcTree<S>*>(o2);
    OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

    initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, no_cost_request, result);
    collide(&node);

    Box<S> box;
    Transform3<S> box_tf;
    constructBox(obj1->getBV(0).bv, tf1, box, box_tf);

    box.cost_density = obj1->cost_density;
    box.threshold_occupied = obj1->threshold_occupied;
    box.threshold_free = obj1->threshold_free;

    CollisionRequest<S> only_cost_request(result.numContacts(), false, request.num_max_cost_sources, true, false);
    ShapeOcTreeCollide<Box<S>, NarrowPhaseSolver>(&box, box_tf, o2, tf2, nsolver, only_cost_request, result);
  }
  else
  {
    MeshOcTreeCollisionTraversalNode<BV, NarrowPhaseSolver> node;
    const BVHModel<BV>* obj1 = static_cast<const BVHModel<BV>*>(o1);
    const OcTree<S>* obj2 = static_cast<const OcTree<S>*>(o2);
    OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

    initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
    collide(&node);
  }

  return result.numContacts();
}

#endif

//==============================================================================
template <typename Shape1, typename Shape2, typename NarrowPhaseSolver>
std::size_t ShapeShapeCollide(
    const CollisionGeometry<typename Shape1::S>* o1,
    const Transform3<typename Shape1::S>& tf1,
    const CollisionGeometry<typename Shape1::S>* o2,
    const Transform3<typename Shape1::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename Shape1::S>& request,
    CollisionResult<typename Shape1::S>& result)
{
  if(request.isSatisfied(result)) return result.numContacts();

  ShapeCollisionTraversalNode<Shape1, Shape2, NarrowPhaseSolver> node;
  const Shape1* obj1 = static_cast<const Shape1*>(o1);
  const Shape2* obj2 = static_cast<const Shape2*>(o2);

  if(request.enable_cached_gjk_guess)
  {
    nsolver->enableCachedGuess(true);
    nsolver->setCachedGuess(request.cached_gjk_guess);
  }
  else
  {
    nsolver->enableCachedGuess(true);
  }

  initialize(node, *obj1, tf1, *obj2, tf2, nsolver, request, result);
  collide(&node);

  if(request.enable_cached_gjk_guess)
    result.cached_gjk_guess = nsolver->getCachedGuess();

  return result.numContacts();
}

//==============================================================================
template <typename BV, typename Shape, typename NarrowPhaseSolver>
struct BVHShapeCollider
{
  using S = typename BV::S;

  static std::size_t collide(
      const CollisionGeometry<S>* o1,
      const Transform3<S>& tf1,
      const CollisionGeometry<S>* o2,
      const Transform3<S>& tf2,
      const NarrowPhaseSolver* nsolver,
      const CollisionRequest<S>& request,
      CollisionResult<S>& result)
  {
    if(request.isSatisfied(result)) return result.numContacts();

    if(request.enable_cost && request.use_approximate_cost)
    {
      CollisionRequest<S> no_cost_request(request);
      no_cost_request.enable_cost = false;

      MeshShapeCollisionTraversalNode<BV, Shape, NarrowPhaseSolver> node;
      const BVHModel<BV>* obj1 = static_cast<const BVHModel<BV>* >(o1);
      BVHModel<BV>* obj1_tmp = new BVHModel<BV>(*obj1);
      Transform3<S> tf1_tmp = tf1;
      const Shape* obj2 = static_cast<const Shape*>(o2);

      initialize(node, *obj1_tmp, tf1_tmp, *obj2, tf2, nsolver, no_cost_request, result);
      fcl::detail::collide(&node);

      delete obj1_tmp;

      Box<S> box;
      Transform3<S> box_tf;
      constructBox(obj1->getBV(0).bv, tf1, box, box_tf);

      box.cost_density = obj1->cost_density;
      box.threshold_occupied = obj1->threshold_occupied;
      box.threshold_free = obj1->threshold_free;

      CollisionRequest<S> only_cost_request(result.numContacts(), false, request.num_max_cost_sources, true, false);
      ShapeShapeCollide<Box<S>, Shape>(&box, box_tf, o2, tf2, nsolver, only_cost_request, result);
    }
    else
    {
      MeshShapeCollisionTraversalNode<BV, Shape, NarrowPhaseSolver> node;
      const BVHModel<BV>* obj1 = static_cast<const BVHModel<BV>* >(o1);
      BVHModel<BV>* obj1_tmp = new BVHModel<BV>(*obj1);
      Transform3<S> tf1_tmp = tf1;
      const Shape* obj2 = static_cast<const Shape*>(o2);

      initialize(node, *obj1_tmp, tf1_tmp, *obj2, tf2, nsolver, request, result);
      fcl::detail::collide(&node);

      delete obj1_tmp;
    }

    return result.numContacts();
  }
};

//==============================================================================
template <typename OrientMeshShapeCollisionTraveralNode,
          typename BV, typename Shape, typename NarrowPhaseSolver>
std::size_t orientedBVHShapeCollide(
    const CollisionGeometry<typename BV::S>* o1,
    const Transform3<typename BV::S>& tf1,
    const CollisionGeometry<typename BV::S>* o2,
    const Transform3<typename BV::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename BV::S>& request,
    CollisionResult<typename BV::S>& result)
{
  using S = typename BV::S;

  if(request.isSatisfied(result)) return result.numContacts();

  if(request.enable_cost && request.use_approximate_cost)
  {
    CollisionRequest<S> no_cost_request(request);
    no_cost_request.enable_cost = false;

    OrientMeshShapeCollisionTraveralNode node;
    const BVHModel<BV>* obj1 = static_cast<const BVHModel<BV>* >(o1);
    const Shape* obj2 = static_cast<const Shape*>(o2);

    initialize(node, *obj1, tf1, *obj2, tf2, nsolver, no_cost_request, result);
    fcl::detail::collide(&node);

    Box<S> box;
    Transform3<S> box_tf;
    constructBox(obj1->getBV(0).bv, tf1, box, box_tf);

    box.cost_density = obj1->cost_density;
    box.threshold_occupied = obj1->threshold_occupied;
    box.threshold_free = obj1->threshold_free;

    CollisionRequest<S> only_cost_request(result.numContacts(), false, request.num_max_cost_sources, true, false);
    ShapeShapeCollide<Box<S>, Shape>(&box, box_tf, o2, tf2, nsolver, only_cost_request, result);
  }
  else
  {
    OrientMeshShapeCollisionTraveralNode node;
    const BVHModel<BV>* obj1 = static_cast<const BVHModel<BV>* >(o1);
    const Shape* obj2 = static_cast<const Shape*>(o2);

    initialize(node, *obj1, tf1, *obj2, tf2, nsolver, request, result);
    fcl::detail::collide(&node);
  }

  return result.numContacts();
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
struct BVHShapeCollider<
    OBB<typename Shape::S>, Shape, NarrowPhaseSolver>
{
  using S = typename Shape::S;

  static std::size_t collide(
      const CollisionGeometry<S>* o1,
      const Transform3<S>& tf1,
      const CollisionGeometry<S>* o2,
      const Transform3<S>& tf2,
      const NarrowPhaseSolver* nsolver,
      const CollisionRequest<S>& request,
      CollisionResult<S>& result)
  {
    return detail::orientedBVHShapeCollide<
        MeshShapeCollisionTraversalNodeOBB<Shape, NarrowPhaseSolver>,
        OBB<S>,
        Shape,
        NarrowPhaseSolver>(
          o1, tf1, o2, tf2, nsolver, request, result);
  }
};

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
struct BVHShapeCollider<RSS<typename Shape::S>, Shape, NarrowPhaseSolver>
{
  using S = typename Shape::S;

  static std::size_t collide(
      const CollisionGeometry<S>* o1,
      const Transform3<S>& tf1,
      const CollisionGeometry<S>* o2,
      const Transform3<S>& tf2,
      const NarrowPhaseSolver* nsolver,
      const CollisionRequest<S>& request,
      CollisionResult<S>& result)
  {
    return detail::orientedBVHShapeCollide<
        MeshShapeCollisionTraversalNodeRSS<Shape, NarrowPhaseSolver>,
        RSS<S>,
        Shape,
        NarrowPhaseSolver>(
          o1, tf1, o2, tf2, nsolver, request, result);
  }
};

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
struct BVHShapeCollider<kIOS<typename Shape::S>, Shape, NarrowPhaseSolver>
{
  using S = typename Shape::S;

  static std::size_t collide(
      const CollisionGeometry<S>* o1,
      const Transform3<S>& tf1,
      const CollisionGeometry<S>* o2,
      const Transform3<S>& tf2,
      const NarrowPhaseSolver* nsolver,
      const CollisionRequest<S>& request,
      CollisionResult<S>& result)
  {
    return detail::orientedBVHShapeCollide<
        MeshShapeCollisionTraversalNodekIOS<Shape, NarrowPhaseSolver>,
        kIOS<S>,
        Shape,
        NarrowPhaseSolver>(
          o1, tf1, o2, tf2, nsolver, request, result);
  }
};

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
struct BVHShapeCollider<OBBRSS<typename Shape::S>, Shape, NarrowPhaseSolver>
{
  using S = typename Shape::S;

  static std::size_t collide(
      const CollisionGeometry<S>* o1,
      const Transform3<S>& tf1,
      const CollisionGeometry<S>* o2,
      const Transform3<S>& tf2,
      const NarrowPhaseSolver* nsolver,
      const CollisionRequest<S>& request,
      CollisionResult<S>& result)
  {
    return detail::orientedBVHShapeCollide<
        MeshShapeCollisionTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>,
        OBBRSS<S>,
        Shape,
        NarrowPhaseSolver>(
          o1, tf1, o2, tf2, nsolver, request, result);
  }
};

//==============================================================================
template <typename S, typename BV>
struct BVHCollideImpl
{
  static std::size_t run(
      const CollisionGeometry<S>* o1,
      const Transform3<S>& tf1,
      const CollisionGeometry<S>* o2,
      const Transform3<S>& tf2,
      const CollisionRequest<S>& request,
      CollisionResult<S>& result)
  {
    if(request.isSatisfied(result)) return result.numContacts();

    MeshCollisionTraversalNode<BV> node;
    const BVHModel<BV>* obj1 = static_cast<const BVHModel<BV>* >(o1);
    const BVHModel<BV>* obj2 = static_cast<const BVHModel<BV>* >(o2);
    BVHModel<BV>* obj1_tmp = new BVHModel<BV>(*obj1);
    Transform3<S> tf1_tmp = tf1;
    BVHModel<BV>* obj2_tmp = new BVHModel<BV>(*obj2);
    Transform3<S> tf2_tmp = tf2;

    initialize(node, *obj1_tmp, tf1_tmp, *obj2_tmp, tf2_tmp, request, result);
    collide(&node);

    delete obj1_tmp;
    delete obj2_tmp;

    return result.numContacts();
  }
};

//==============================================================================
template <typename BV>
std::size_t BVHCollide(
    const CollisionGeometry<typename BV::S>* o1,
    const Transform3<typename BV::S>& tf1,
    const CollisionGeometry<typename BV::S>* o2,
    const Transform3<typename BV::S>& tf2,
    const CollisionRequest<typename BV::S>& request,
    CollisionResult<typename BV::S>& result)
{
  return BVHCollideImpl<typename BV::S, BV>::run(
        o1, tf1, o2, tf2, request, result);
}

//==============================================================================
template <typename OrientedMeshCollisionTraversalNode, typename BV>
std::size_t orientedMeshCollide(
    const CollisionGeometry<typename BV::S>* o1,
    const Transform3<typename BV::S>& tf1,
    const CollisionGeometry<typename BV::S>* o2,
    const Transform3<typename BV::S>& tf2,
    const CollisionRequest<typename BV::S>& request,
    CollisionResult<typename BV::S>& result)
{
  if(request.isSatisfied(result)) return result.numContacts();

  OrientedMeshCollisionTraversalNode node;
  const BVHModel<BV>* obj1 = static_cast<const BVHModel<BV>* >(o1);
  const BVHModel<BV>* obj2 = static_cast<const BVHModel<BV>* >(o2);

  initialize(node, *obj1, tf1, *obj2, tf2, request, result);
  collide(&node);

  return result.numContacts();
}

//==============================================================================
template <typename S>
struct BVHCollideImpl<S, OBB<S>>
{
  static std::size_t run(
      const CollisionGeometry<S>* o1,
      const Transform3<S>& tf1,
      const CollisionGeometry<S>* o2,
      const Transform3<S>& tf2,
      const CollisionRequest<S>& request,
      CollisionResult<S>& result)
  {
    return detail::orientedMeshCollide<
        MeshCollisionTraversalNodeOBB<S>, OBB<S>>(
            o1, tf1, o2, tf2, request, result);
  }
};

//==============================================================================
template <typename S>
struct BVHCollideImpl<S, OBBRSS<S>>
{
  static std::size_t run(
      const CollisionGeometry<S>* o1,
      const Transform3<S>& tf1,
      const CollisionGeometry<S>* o2,
      const Transform3<S>& tf2,
      const CollisionRequest<S>& request,
      CollisionResult<S>& result)
  {
    return detail::orientedMeshCollide<
        MeshCollisionTraversalNodeOBBRSS<S>, OBBRSS<S>>(
            o1, tf1, o2, tf2, request, result);
  }
};

//==============================================================================
template <typename S>
struct BVHCollideImpl<S, kIOS<S>>
{
  static std::size_t run(
      const CollisionGeometry<S>* o1,
      const Transform3<S>& tf1,
      const CollisionGeometry<S>* o2,
      const Transform3<S>& tf2,
      const CollisionRequest<S>& request,
      CollisionResult<S>& result)
  {
    return detail::orientedMeshCollide<
        MeshCollisionTraversalNodekIOS<S>, kIOS<S>>(
            o1, tf1, o2, tf2, request, result);
  }
};

//==============================================================================
template <typename BV, typename NarrowPhaseSolver>
std::size_t BVHCollide(
    const CollisionGeometry<typename BV::S>* o1,
    const Transform3<typename BV::S>& tf1,
    const CollisionGeometry<typename BV::S>* o2,
    const Transform3<typename BV::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename BV::S>& request,
    CollisionResult<typename BV::S>& result)
{
  FCL_UNUSED(nsolver);

  return BVHCollide<BV>(o1, tf1, o2, tf2, request, result);
}

//==============================================================================
template <typename NarrowPhaseSolver>
CollisionFunctionMatrix<NarrowPhaseSolver>::CollisionFunctionMatrix()
{
  using S = typename NarrowPhaseSolver::S;

  for(int i = 0; i < NODE_COUNT; ++i)
  {
    for(int j = 0; j < NODE_COUNT; ++j)
      collision_matrix[i][j] = nullptr;
  }

  collision_matrix[GEOM_BOX][GEOM_BOX] = &ShapeShapeCollide<Box<S>, Box<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_SPHERE] = &ShapeShapeCollide<Box<S>, Sphere<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_ELLIPSOID] = &ShapeShapeCollide<Box<S>, Ellipsoid<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_CAPSULE] = &ShapeShapeCollide<Box<S>, Capsule<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_CONE] = &ShapeShapeCollide<Box<S>, Cone<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_CYLINDER] = &ShapeShapeCollide<Box<S>, Cylinder<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_CONVEX] = &ShapeShapeCollide<Box<S>, Convex<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_PLANE] = &ShapeShapeCollide<Box<S>, Plane<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_HALFSPACE] = &ShapeShapeCollide<Box<S>, Halfspace<S>, NarrowPhaseSolver>;

  collision_matrix[GEOM_SPHERE][GEOM_BOX] = &ShapeShapeCollide<Sphere<S>, Box<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_SPHERE] = &ShapeShapeCollide<Sphere<S>, Sphere<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_ELLIPSOID] = &ShapeShapeCollide<Sphere<S>, Ellipsoid<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_CAPSULE] = &ShapeShapeCollide<Sphere<S>, Capsule<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_CONE] = &ShapeShapeCollide<Sphere<S>, Cone<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_CYLINDER] = &ShapeShapeCollide<Sphere<S>, Cylinder<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_CONVEX] = &ShapeShapeCollide<Sphere<S>, Convex<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_PLANE] = &ShapeShapeCollide<Sphere<S>, Plane<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_HALFSPACE] = &ShapeShapeCollide<Sphere<S>, Halfspace<S>, NarrowPhaseSolver>;

  collision_matrix[GEOM_ELLIPSOID][GEOM_BOX] = &ShapeShapeCollide<Ellipsoid<S>, Box<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_SPHERE] = &ShapeShapeCollide<Ellipsoid<S>, Sphere<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_ELLIPSOID] = &ShapeShapeCollide<Ellipsoid<S>, Ellipsoid<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_CAPSULE] = &ShapeShapeCollide<Ellipsoid<S>, Capsule<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_CONE] = &ShapeShapeCollide<Ellipsoid<S>, Cone<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_CYLINDER] = &ShapeShapeCollide<Ellipsoid<S>, Cylinder<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_CONVEX] = &ShapeShapeCollide<Ellipsoid<S>, Convex<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_PLANE] = &ShapeShapeCollide<Ellipsoid<S>, Plane<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_HALFSPACE] = &ShapeShapeCollide<Ellipsoid<S>, Halfspace<S>, NarrowPhaseSolver>;

  collision_matrix[GEOM_CAPSULE][GEOM_BOX] = &ShapeShapeCollide<Capsule<S>, Box<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_SPHERE] = &ShapeShapeCollide<Capsule<S>, Sphere<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_ELLIPSOID] = &ShapeShapeCollide<Capsule<S>, Ellipsoid<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_CAPSULE] = &ShapeShapeCollide<Capsule<S>, Capsule<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_CONE] = &ShapeShapeCollide<Capsule<S>, Cone<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_CYLINDER] = &ShapeShapeCollide<Capsule<S>, Cylinder<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_CONVEX] = &ShapeShapeCollide<Capsule<S>, Convex<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_PLANE] = &ShapeShapeCollide<Capsule<S>, Plane<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_HALFSPACE] = &ShapeShapeCollide<Capsule<S>, Halfspace<S>, NarrowPhaseSolver>;

  collision_matrix[GEOM_CONE][GEOM_BOX] = &ShapeShapeCollide<Cone<S>, Box<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_SPHERE] = &ShapeShapeCollide<Cone<S>, Sphere<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_ELLIPSOID] = &ShapeShapeCollide<Cone<S>, Ellipsoid<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_CAPSULE] = &ShapeShapeCollide<Cone<S>, Capsule<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_CONE] = &ShapeShapeCollide<Cone<S>, Cone<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_CYLINDER] = &ShapeShapeCollide<Cone<S>, Cylinder<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_CONVEX] = &ShapeShapeCollide<Cone<S>, Convex<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_PLANE] = &ShapeShapeCollide<Cone<S>, Plane<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_HALFSPACE] = &ShapeShapeCollide<Cone<S>, Halfspace<S>, NarrowPhaseSolver>;

  collision_matrix[GEOM_CYLINDER][GEOM_BOX] = &ShapeShapeCollide<Cylinder<S>, Box<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_SPHERE] = &ShapeShapeCollide<Cylinder<S>, Sphere<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_ELLIPSOID] = &ShapeShapeCollide<Cylinder<S>, Ellipsoid<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_CAPSULE] = &ShapeShapeCollide<Cylinder<S>, Capsule<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_CONE] = &ShapeShapeCollide<Cylinder<S>, Cone<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_CYLINDER] = &ShapeShapeCollide<Cylinder<S>, Cylinder<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_CONVEX] = &ShapeShapeCollide<Cylinder<S>, Convex<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_PLANE] = &ShapeShapeCollide<Cylinder<S>, Plane<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_HALFSPACE] = &ShapeShapeCollide<Cylinder<S>, Halfspace<S>, NarrowPhaseSolver>;

  collision_matrix[GEOM_CONVEX][GEOM_BOX] = &ShapeShapeCollide<Convex<S>, Box<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_SPHERE] = &ShapeShapeCollide<Convex<S>, Sphere<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_ELLIPSOID] = &ShapeShapeCollide<Convex<S>, Ellipsoid<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_CAPSULE] = &ShapeShapeCollide<Convex<S>, Capsule<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_CONE] = &ShapeShapeCollide<Convex<S>, Cone<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_CYLINDER] = &ShapeShapeCollide<Convex<S>, Cylinder<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_CONVEX] = &ShapeShapeCollide<Convex<S>, Convex<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_PLANE] = &ShapeShapeCollide<Convex<S>, Plane<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_HALFSPACE] = &ShapeShapeCollide<Convex<S>, Halfspace<S>, NarrowPhaseSolver>;

  collision_matrix[GEOM_PLANE][GEOM_BOX] = &ShapeShapeCollide<Plane<S>, Box<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_SPHERE] = &ShapeShapeCollide<Plane<S>, Sphere<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_ELLIPSOID] = &ShapeShapeCollide<Plane<S>, Ellipsoid<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_CAPSULE] = &ShapeShapeCollide<Plane<S>, Capsule<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_CONE] = &ShapeShapeCollide<Plane<S>, Cone<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_CYLINDER] = &ShapeShapeCollide<Plane<S>, Cylinder<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_CONVEX] = &ShapeShapeCollide<Plane<S>, Convex<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_PLANE] = &ShapeShapeCollide<Plane<S>, Plane<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_HALFSPACE] = &ShapeShapeCollide<Plane<S>, Halfspace<S>, NarrowPhaseSolver>;

  collision_matrix[GEOM_HALFSPACE][GEOM_BOX] = &ShapeShapeCollide<Halfspace<S>, Box<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_SPHERE] = &ShapeShapeCollide<Halfspace<S>, Sphere<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_ELLIPSOID] = &ShapeShapeCollide<Halfspace<S>, Ellipsoid<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CAPSULE] = &ShapeShapeCollide<Halfspace<S>, Capsule<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CONE] = &ShapeShapeCollide<Halfspace<S>, Cone<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CYLINDER] = &ShapeShapeCollide<Halfspace<S>, Cylinder<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CONVEX] = &ShapeShapeCollide<Halfspace<S>, Convex<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_PLANE] = &ShapeShapeCollide<Halfspace<S>, Plane<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_HALFSPACE] = &ShapeShapeCollide<Halfspace<S>, Halfspace<S>, NarrowPhaseSolver>;

  collision_matrix[BV_AABB][GEOM_BOX] = &BVHShapeCollider<AABB<S>, Box<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_SPHERE] = &BVHShapeCollider<AABB<S>, Sphere<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_ELLIPSOID] = &BVHShapeCollider<AABB<S>, Ellipsoid<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_CAPSULE] = &BVHShapeCollider<AABB<S>, Capsule<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_CONE] = &BVHShapeCollider<AABB<S>, Cone<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_CYLINDER] = &BVHShapeCollider<AABB<S>, Cylinder<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_CONVEX] = &BVHShapeCollider<AABB<S>, Convex<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_PLANE] = &BVHShapeCollider<AABB<S>, Plane<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_HALFSPACE] = &BVHShapeCollider<AABB<S>, Halfspace<S>, NarrowPhaseSolver>::collide;

  collision_matrix[BV_OBB][GEOM_BOX] = &BVHShapeCollider<OBB<S>, Box<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_SPHERE] = &BVHShapeCollider<OBB<S>, Sphere<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_ELLIPSOID] = &BVHShapeCollider<OBB<S>, Ellipsoid<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_CAPSULE] = &BVHShapeCollider<OBB<S>, Capsule<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_CONE] = &BVHShapeCollider<OBB<S>, Cone<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_CYLINDER] = &BVHShapeCollider<OBB<S>, Cylinder<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_CONVEX] = &BVHShapeCollider<OBB<S>, Convex<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_PLANE] = &BVHShapeCollider<OBB<S>, Plane<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_HALFSPACE] = &BVHShapeCollider<OBB<S>, Halfspace<S>, NarrowPhaseSolver>::collide;

  collision_matrix[BV_RSS][GEOM_BOX] = &BVHShapeCollider<RSS<S>, Box<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_SPHERE] = &BVHShapeCollider<RSS<S>, Sphere<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_ELLIPSOID] = &BVHShapeCollider<RSS<S>, Ellipsoid<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_CAPSULE] = &BVHShapeCollider<RSS<S>, Capsule<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_CONE] = &BVHShapeCollider<RSS<S>, Cone<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_CYLINDER] = &BVHShapeCollider<RSS<S>, Cylinder<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_CONVEX] = &BVHShapeCollider<RSS<S>, Convex<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_PLANE] = &BVHShapeCollider<RSS<S>, Plane<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_HALFSPACE] = &BVHShapeCollider<RSS<S>, Halfspace<S>, NarrowPhaseSolver>::collide;

  collision_matrix[BV_KDOP16][GEOM_BOX] = &BVHShapeCollider<KDOP<S, 16>, Box<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_SPHERE] = &BVHShapeCollider<KDOP<S, 16>, Sphere<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_ELLIPSOID] = &BVHShapeCollider<KDOP<S, 16>, Ellipsoid<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_CAPSULE] = &BVHShapeCollider<KDOP<S, 16>, Capsule<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_CONE] = &BVHShapeCollider<KDOP<S, 16>, Cone<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_CYLINDER] = &BVHShapeCollider<KDOP<S, 16>, Cylinder<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_CONVEX] = &BVHShapeCollider<KDOP<S, 16>, Convex<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_PLANE] = &BVHShapeCollider<KDOP<S, 16>, Plane<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_HALFSPACE] = &BVHShapeCollider<KDOP<S, 16>, Halfspace<S>, NarrowPhaseSolver>::collide;

  collision_matrix[BV_KDOP18][GEOM_BOX] = &BVHShapeCollider<KDOP<S, 18>, Box<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_SPHERE] = &BVHShapeCollider<KDOP<S, 18>, Sphere<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_ELLIPSOID] = &BVHShapeCollider<KDOP<S, 18>, Ellipsoid<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_CAPSULE] = &BVHShapeCollider<KDOP<S, 18>, Capsule<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_CONE] = &BVHShapeCollider<KDOP<S, 18>, Cone<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_CYLINDER] = &BVHShapeCollider<KDOP<S, 18>, Cylinder<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_CONVEX] = &BVHShapeCollider<KDOP<S, 18>, Convex<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_PLANE] = &BVHShapeCollider<KDOP<S, 18>, Plane<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_HALFSPACE] = &BVHShapeCollider<KDOP<S, 18>, Halfspace<S>, NarrowPhaseSolver>::collide;

  collision_matrix[BV_KDOP24][GEOM_BOX] = &BVHShapeCollider<KDOP<S, 24>, Box<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_SPHERE] = &BVHShapeCollider<KDOP<S, 24>, Sphere<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_ELLIPSOID] = &BVHShapeCollider<KDOP<S, 24>, Ellipsoid<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_CAPSULE] = &BVHShapeCollider<KDOP<S, 24>, Capsule<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_CONE] = &BVHShapeCollider<KDOP<S, 24>, Cone<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_CYLINDER] = &BVHShapeCollider<KDOP<S, 24>, Cylinder<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_CONVEX] = &BVHShapeCollider<KDOP<S, 24>, Convex<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_PLANE] = &BVHShapeCollider<KDOP<S, 24>, Plane<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_HALFSPACE] = &BVHShapeCollider<KDOP<S, 24>, Halfspace<S>, NarrowPhaseSolver>::collide;

  collision_matrix[BV_kIOS][GEOM_BOX] = &BVHShapeCollider<kIOS<S>, Box<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_SPHERE] = &BVHShapeCollider<kIOS<S>, Sphere<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_ELLIPSOID] = &BVHShapeCollider<kIOS<S>, Ellipsoid<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_CAPSULE] = &BVHShapeCollider<kIOS<S>, Capsule<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_CONE] = &BVHShapeCollider<kIOS<S>, Cone<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_CYLINDER] = &BVHShapeCollider<kIOS<S>, Cylinder<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_CONVEX] = &BVHShapeCollider<kIOS<S>, Convex<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_PLANE] = &BVHShapeCollider<kIOS<S>, Plane<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_HALFSPACE] = &BVHShapeCollider<kIOS<S>, Halfspace<S>, NarrowPhaseSolver>::collide;

  collision_matrix[BV_OBBRSS][GEOM_BOX] = &BVHShapeCollider<OBBRSS<S>, Box<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_SPHERE] = &BVHShapeCollider<OBBRSS<S>, Sphere<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_ELLIPSOID] = &BVHShapeCollider<OBBRSS<S>, Ellipsoid<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CAPSULE] = &BVHShapeCollider<OBBRSS<S>, Capsule<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CONE] = &BVHShapeCollider<OBBRSS<S>, Cone<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CYLINDER] = &BVHShapeCollider<OBBRSS<S>, Cylinder<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CONVEX] = &BVHShapeCollider<OBBRSS<S>, Convex<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_PLANE] = &BVHShapeCollider<OBBRSS<S>, Plane<S>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_HALFSPACE] = &BVHShapeCollider<OBBRSS<S>, Halfspace<S>, NarrowPhaseSolver>::collide;

  collision_matrix[BV_AABB][BV_AABB] = &BVHCollide<AABB<S>, NarrowPhaseSolver>;
  collision_matrix[BV_OBB][BV_OBB] = &BVHCollide<OBB<S>, NarrowPhaseSolver>;
  collision_matrix[BV_RSS][BV_RSS] = &BVHCollide<RSS<S>, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP16][BV_KDOP16] = &BVHCollide<KDOP<S, 16>, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP18][BV_KDOP18] = &BVHCollide<KDOP<S, 18>, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP24][BV_KDOP24] = &BVHCollide<KDOP<S, 24>, NarrowPhaseSolver>;
  collision_matrix[BV_kIOS][BV_kIOS] = &BVHCollide<kIOS<S>, NarrowPhaseSolver>;
  collision_matrix[BV_OBBRSS][BV_OBBRSS] = &BVHCollide<OBBRSS<S>, NarrowPhaseSolver>;

#if FCL_HAVE_OCTOMAP
  collision_matrix[GEOM_OCTREE][GEOM_BOX] = &OcTreeShapeCollide<Box<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_SPHERE] = &OcTreeShapeCollide<Sphere<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_ELLIPSOID] = &OcTreeShapeCollide<Ellipsoid<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_CAPSULE] = &OcTreeShapeCollide<Capsule<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_CONE] = &OcTreeShapeCollide<Cone<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_CYLINDER] = &OcTreeShapeCollide<Cylinder<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_CONVEX] = &OcTreeShapeCollide<Convex<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_PLANE] = &OcTreeShapeCollide<Plane<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_HALFSPACE] = &OcTreeShapeCollide<Halfspace<S>, NarrowPhaseSolver>;

  collision_matrix[GEOM_BOX][GEOM_OCTREE] = &ShapeOcTreeCollide<Box<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_OCTREE] = &ShapeOcTreeCollide<Sphere<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_OCTREE] = &ShapeOcTreeCollide<Ellipsoid<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_OCTREE] = &ShapeOcTreeCollide<Capsule<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_OCTREE] = &ShapeOcTreeCollide<Cone<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_OCTREE] = &ShapeOcTreeCollide<Cylinder<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_OCTREE] = &ShapeOcTreeCollide<Convex<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_OCTREE] = &ShapeOcTreeCollide<Plane<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_OCTREE] = &ShapeOcTreeCollide<Halfspace<S>, NarrowPhaseSolver>;

  collision_matrix[GEOM_OCTREE][GEOM_OCTREE] = &OcTreeCollide<NarrowPhaseSolver>;

  collision_matrix[GEOM_OCTREE][BV_AABB] = &OcTreeBVHCollide<AABB<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_OBB] = &OcTreeBVHCollide<OBB<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_RSS] = &OcTreeBVHCollide<RSS<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_OBBRSS] = &OcTreeBVHCollide<OBBRSS<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_kIOS] = &OcTreeBVHCollide<kIOS<S>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_KDOP16] = &OcTreeBVHCollide<KDOP<S, 16>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_KDOP18] = &OcTreeBVHCollide<KDOP<S, 18>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_KDOP24] = &OcTreeBVHCollide<KDOP<S, 24>, NarrowPhaseSolver>;

  collision_matrix[BV_AABB][GEOM_OCTREE] = &BVHOcTreeCollide<AABB<S>, NarrowPhaseSolver>;
  collision_matrix[BV_OBB][GEOM_OCTREE] = &BVHOcTreeCollide<OBB<S>, NarrowPhaseSolver>;
  collision_matrix[BV_RSS][GEOM_OCTREE] = &BVHOcTreeCollide<RSS<S>, NarrowPhaseSolver>;
  collision_matrix[BV_OBBRSS][GEOM_OCTREE] = &BVHOcTreeCollide<OBBRSS<S>, NarrowPhaseSolver>;
  collision_matrix[BV_kIOS][GEOM_OCTREE] = &BVHOcTreeCollide<kIOS<S>, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP16][GEOM_OCTREE] = &BVHOcTreeCollide<KDOP<S, 16>, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP18][GEOM_OCTREE] = &BVHOcTreeCollide<KDOP<S, 18>, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP24][GEOM_OCTREE] = &BVHOcTreeCollide<KDOP<S, 24>, NarrowPhaseSolver>;
#endif
}

} // namespace detail
} // namespace fcl

#endif

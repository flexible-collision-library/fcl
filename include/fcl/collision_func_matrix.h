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

/** \author Jia Pan */


#ifndef FCL_COLLISION_FUNC_MATRIX_H
#define FCL_COLLISION_FUNC_MATRIX_H

#include "fcl/collision_object.h"
#include "fcl/collision_data.h"
#include "fcl/collision_node.h"

#include "fcl/shape/box.h"
#include "fcl/shape/capsule.h"
#include "fcl/shape/cone.h"
#include "fcl/shape/convex.h"
#include "fcl/shape/cylinder.h"
#include "fcl/shape/ellipsoid.h"
#include "fcl/shape/halfspace.h"
#include "fcl/shape/plane.h"
#include "fcl/shape/sphere.h"
#include "fcl/shape/triangle_p.h"
#include "fcl/shape/construct_box.h"

#include "fcl/traversal/shape_collision_traversal_node.h"
#include "fcl/traversal/mesh_shape_collision_traversal_node.h"
#include "fcl/traversal/shape_mesh_collision_traversal_node.h"
#include "fcl/traversal/mesh_collision_traversal_node.h"

#include "fcl/config.h"
#if FCL_HAVE_OCTOMAP
#include "fcl/traversal/octree/shape_octree_collision_traversal_node.h"
#include "fcl/traversal/octree/octree_shape_collision_traversal_node.h"
#include "fcl/traversal/octree/octree_mesh_collision_traversal_node.h"
#include "fcl/traversal/octree/mesh_octree_collision_traversal_node.h"
#include "fcl/traversal/octree/octree_collision_traversal_node.h"
#endif

namespace fcl
{

/// @brief collision matrix stores the functions for collision between different types of objects and provides a uniform call interface
template <typename NarrowPhaseSolver>
struct CollisionFunctionMatrix
{
  using Scalar = typename NarrowPhaseSolver::Scalar;

  /// @brief the uniform call interface for collision: for collision, we need know
  /// 1. two objects o1 and o2 and their configuration in world coordinate tf1 and tf2;
  /// 2. the solver for narrow phase collision, this is for the collision between geometric shapes;
  /// 3. the request setting for collision (e.g., whether need to return normal information, whether need to compute cost);
  /// 4. the structure to return collision result
  using CollisionFunc = std::size_t (*)(
      const CollisionGeometry<Scalar>* o1,
      const Transform3<Scalar>& tf1,
      const CollisionGeometry<Scalar>* o2,
      const Transform3<Scalar>& tf2,
      const NarrowPhaseSolver* nsolver,
      const CollisionRequest<Scalar>& request,
      CollisionResult<Scalar>& result);

  /// @brief each item in the collision matrix is a function to handle collision
  /// between objects of type1 and type2
  CollisionFunc collision_matrix[NODE_COUNT][NODE_COUNT];

  CollisionFunctionMatrix();
};

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

#if FCL_HAVE_OCTOMAP

//==============================================================================
template <typename T_SH, typename NarrowPhaseSolver>
std::size_t ShapeOcTreeCollide(
    const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
    CollisionResult<typename NarrowPhaseSolver::Scalar>& result)
{
  if(request.isSatisfied(result)) return result.numContacts();

  ShapeOcTreeCollisionTraversalNode<T_SH, NarrowPhaseSolver> node;
  const T_SH* obj1 = static_cast<const T_SH*>(o1);
  const OcTree* obj2 = static_cast<const OcTree*>(o2);
  OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
  collide(&node);

  return result.numContacts();
}

//==============================================================================
template <typename T_SH, typename NarrowPhaseSolver>
std::size_t OcTreeShapeCollide(
    const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
    CollisionResult<typename NarrowPhaseSolver::Scalar>& result)
{
  if(request.isSatisfied(result)) return result.numContacts();

  OcTreeShapeCollisionTraversalNode<T_SH, NarrowPhaseSolver> node;
  const OcTree* obj1 = static_cast<const OcTree*>(o1);
  const T_SH* obj2 = static_cast<const T_SH*>(o2);
  OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
  collide(&node);

  return result.numContacts();
}

//==============================================================================
template <typename NarrowPhaseSolver>
std::size_t OcTreeCollide(
    const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
    CollisionResult<typename NarrowPhaseSolver::Scalar>& result)
{
  if(request.isSatisfied(result)) return result.numContacts();

  OcTreeCollisionTraversalNode<NarrowPhaseSolver> node;
  const OcTree* obj1 = static_cast<const OcTree*>(o1);
  const OcTree* obj2 = static_cast<const OcTree*>(o2);
  OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
  collide(&node);

  return result.numContacts();
}

//==============================================================================
template <typename T_BVH, typename NarrowPhaseSolver>
std::size_t OcTreeBVHCollide(
    const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
    CollisionResult<typename NarrowPhaseSolver::Scalar>& result)
{
  using Scalar = typename NarrowPhaseSolver::Scalar;

  if(request.isSatisfied(result)) return result.numContacts();

  if(request.enable_cost && request.use_approximate_cost)
  {
    CollisionRequest<Scalar> no_cost_request(request); // request remove cost to avoid the exact but expensive cost computation between mesh and octree
    no_cost_request.enable_cost = false; // disable cost computation

    OcTreeMeshCollisionTraversalNode<T_BVH, NarrowPhaseSolver> node;
    const OcTree* obj1 = static_cast<const OcTree*>(o1);
    const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>*>(o2);
    OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

    initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, no_cost_request, result);
    collide(&node);

    Box<Scalar> box;
    Transform3<Scalar> box_tf;
    constructBox(obj2->getBV(0).bv, tf2, box, box_tf); // compute the box for BVH's root node

    box.cost_density = obj2->cost_density;
    box.threshold_occupied = obj2->threshold_occupied;
    box.threshold_free = obj2->threshold_free;

    CollisionRequest<Scalar> only_cost_request(result.numContacts(), false, request.num_max_cost_sources, true, false); // additional cost request, no contacts
    OcTreeShapeCollide<Box<Scalar>, NarrowPhaseSolver>(o1, tf1, &box, box_tf, nsolver, only_cost_request, result);
  }
  else
  {
    OcTreeMeshCollisionTraversalNode<T_BVH, NarrowPhaseSolver> node;
    const OcTree* obj1 = static_cast<const OcTree*>(o1);
    const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>*>(o2);
    OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

    initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
    collide(&node);
  }

  return result.numContacts();
}

//==============================================================================
template <typename T_BVH, typename NarrowPhaseSolver>
std::size_t BVHOcTreeCollide(
    const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
    CollisionResult<typename NarrowPhaseSolver::Scalar>& result)
{
  using Scalar = typename NarrowPhaseSolver::Scalar;

  if(request.isSatisfied(result)) return result.numContacts();

  if(request.enable_cost && request.use_approximate_cost)
  {
    CollisionRequest<Scalar> no_cost_request(request); // request remove cost to avoid the exact but expensive cost computation between mesh and octree
    no_cost_request.enable_cost = false; // disable cost computation

    MeshOcTreeCollisionTraversalNode<T_BVH, NarrowPhaseSolver> node;
    const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>*>(o1);
    const OcTree* obj2 = static_cast<const OcTree*>(o2);
    OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

    initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, no_cost_request, result);
    collide(&node);

    Box<Scalar> box;
    Transform3<Scalar> box_tf;
    constructBox(obj1->getBV(0).bv, tf1, box, box_tf);

    box.cost_density = obj1->cost_density;
    box.threshold_occupied = obj1->threshold_occupied;
    box.threshold_free = obj1->threshold_free;

    CollisionRequest<Scalar> only_cost_request(result.numContacts(), false, request.num_max_cost_sources, true, false);
    ShapeOcTreeCollide<Box<Scalar>, NarrowPhaseSolver>(&box, box_tf, o2, tf2, nsolver, only_cost_request, result);
  }
  else
  {
    MeshOcTreeCollisionTraversalNode<T_BVH, NarrowPhaseSolver> node;
    const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>*>(o1);
    const OcTree* obj2 = static_cast<const OcTree*>(o2);
    OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

    initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
    collide(&node);
  }

  return result.numContacts();
}

#endif

//==============================================================================
template <typename T_SH1, typename T_SH2, typename NarrowPhaseSolver>
std::size_t ShapeShapeCollide(
    const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
    CollisionResult<typename NarrowPhaseSolver::Scalar>& result)
{
  if(request.isSatisfied(result)) return result.numContacts();

  ShapeCollisionTraversalNode<T_SH1, T_SH2, NarrowPhaseSolver> node;
  const T_SH1* obj1 = static_cast<const T_SH1*>(o1);
  const T_SH2* obj2 = static_cast<const T_SH2*>(o2);

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
template <typename T_BVH, typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeCollider
{
  static std::size_t collide(
      const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o1,
      const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
      const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o2,
      const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
      const NarrowPhaseSolver* nsolver,
      const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
      CollisionResult<typename NarrowPhaseSolver::Scalar>& result)
  {
    using Scalar = typename NarrowPhaseSolver::Scalar;

    if(request.isSatisfied(result)) return result.numContacts();

    if(request.enable_cost && request.use_approximate_cost)
    {
      CollisionRequest<Scalar> no_cost_request(request);
      no_cost_request.enable_cost = false;

      MeshShapeCollisionTraversalNode<T_BVH, T_SH, NarrowPhaseSolver> node;
      const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
      BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
      Transform3<Scalar> tf1_tmp = tf1;
      const T_SH* obj2 = static_cast<const T_SH*>(o2);

      initialize(node, *obj1_tmp, tf1_tmp, *obj2, tf2, nsolver, no_cost_request, result);
      fcl::collide(&node);

      delete obj1_tmp;

      Box<Scalar> box;
      Transform3<Scalar> box_tf;
      constructBox(obj1->getBV(0).bv, tf1, box, box_tf);

      box.cost_density = obj1->cost_density;
      box.threshold_occupied = obj1->threshold_occupied;
      box.threshold_free = obj1->threshold_free;

      CollisionRequest<Scalar> only_cost_request(result.numContacts(), false, request.num_max_cost_sources, true, false);
      ShapeShapeCollide<Box<Scalar>, T_SH>(&box, box_tf, o2, tf2, nsolver, only_cost_request, result);
    }
    else
    {
      MeshShapeCollisionTraversalNode<T_BVH, T_SH, NarrowPhaseSolver> node;
      const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
      BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
      Transform3<Scalar> tf1_tmp = tf1;
      const T_SH* obj2 = static_cast<const T_SH*>(o2);

      initialize(node, *obj1_tmp, tf1_tmp, *obj2, tf2, nsolver, request, result);
      fcl::collide(&node);

      delete obj1_tmp;
    }

    return result.numContacts();
  }
};

namespace details
{

//==============================================================================
template <typename OrientMeshShapeCollisionTraveralNode,
          typename T_BVH, typename T_SH, typename NarrowPhaseSolver>
std::size_t orientedBVHShapeCollide(
    const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
    CollisionResult<typename NarrowPhaseSolver::Scalar>& result)
{
  using Scalar = typename NarrowPhaseSolver::Scalar;

  if(request.isSatisfied(result)) return result.numContacts();

  if(request.enable_cost && request.use_approximate_cost)
  {
    CollisionRequest<Scalar> no_cost_request(request);
    no_cost_request.enable_cost = false;

    OrientMeshShapeCollisionTraveralNode node;
    const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    initialize(node, *obj1, tf1, *obj2, tf2, nsolver, no_cost_request, result);
    fcl::collide(&node);

    Box<Scalar> box;
    Transform3<Scalar> box_tf;
    constructBox(obj1->getBV(0).bv, tf1, box, box_tf);

    box.cost_density = obj1->cost_density;
    box.threshold_occupied = obj1->threshold_occupied;
    box.threshold_free = obj1->threshold_free;

    CollisionRequest<Scalar> only_cost_request(result.numContacts(), false, request.num_max_cost_sources, true, false);
    ShapeShapeCollide<Box<Scalar>, T_SH>(&box, box_tf, o2, tf2, nsolver, only_cost_request, result);
  }
  else
  {
    OrientMeshShapeCollisionTraveralNode node;
    const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    initialize(node, *obj1, tf1, *obj2, tf2, nsolver, request, result);
    fcl::collide(&node);
  }

  return result.numContacts();
}

} // namespace details

//==============================================================================
template <typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeCollider<
    OBB<typename NarrowPhaseSolver::Scalar>, T_SH, NarrowPhaseSolver>
{
  static std::size_t collide(
      const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o1,
      const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
      const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o2,
      const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
      const NarrowPhaseSolver* nsolver,
      const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
      CollisionResult<typename NarrowPhaseSolver::Scalar>& result)
  {
    return details::orientedBVHShapeCollide<
        MeshShapeCollisionTraversalNodeOBB<T_SH, NarrowPhaseSolver>,
        OBB<typename NarrowPhaseSolver::Scalar>,
        T_SH,
        NarrowPhaseSolver>(
          o1, tf1, o2, tf2, nsolver, request, result);
  }
};

//==============================================================================
template <typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeCollider<RSS<typename NarrowPhaseSolver::Scalar>, T_SH, NarrowPhaseSolver>
{
  static std::size_t collide(
      const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o1,
      const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
      const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o2,
      const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
      const NarrowPhaseSolver* nsolver,
      const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
      CollisionResult<typename NarrowPhaseSolver::Scalar>& result)
  {
    return details::orientedBVHShapeCollide<
        MeshShapeCollisionTraversalNodeRSS<T_SH, NarrowPhaseSolver>,
        RSS<typename NarrowPhaseSolver::Scalar>,
        T_SH,
        NarrowPhaseSolver>(
          o1, tf1, o2, tf2, nsolver, request, result);
  }
};

//==============================================================================
template <typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeCollider<kIOS<typename NarrowPhaseSolver::Scalar>, T_SH, NarrowPhaseSolver>
{
  static std::size_t collide(
      const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o1,
      const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
      const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o2,
      const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
      const NarrowPhaseSolver* nsolver,
      const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
      CollisionResult<typename NarrowPhaseSolver::Scalar>& result)
  {
    return details::orientedBVHShapeCollide<
        MeshShapeCollisionTraversalNodekIOS<T_SH, NarrowPhaseSolver>,
        kIOS<typename NarrowPhaseSolver::Scalar>,
        T_SH,
        NarrowPhaseSolver>(
          o1, tf1, o2, tf2, nsolver, request, result);
  }
};

//==============================================================================
template <typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeCollider<OBBRSS<typename NarrowPhaseSolver::Scalar>, T_SH, NarrowPhaseSolver>
{
  static std::size_t collide(
      const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o1,
      const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
      const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o2,
      const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
      const NarrowPhaseSolver* nsolver,
      const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
      CollisionResult<typename NarrowPhaseSolver::Scalar>& result)
  {
    return details::orientedBVHShapeCollide<
        MeshShapeCollisionTraversalNodeOBBRSS<T_SH, NarrowPhaseSolver>,
        OBBRSS<typename NarrowPhaseSolver::Scalar>,
        T_SH,
        NarrowPhaseSolver>(
          o1, tf1, o2, tf2, nsolver, request, result);
  }
};

//==============================================================================
template <typename Scalar, typename T_BVH>
struct BVHCollideImpl
{
  std::size_t operator()(
      const CollisionGeometry<Scalar>* o1,
      const Transform3<Scalar>& tf1,
      const CollisionGeometry<Scalar>* o2,
      const Transform3<Scalar>& tf2,
      const CollisionRequest<Scalar>& request,
      CollisionResult<Scalar>& result)
  {
    if(request.isSatisfied(result)) return result.numContacts();

    MeshCollisionTraversalNode<T_BVH> node;
    const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
    const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>* >(o2);
    BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
    Transform3<Scalar> tf1_tmp = tf1;
    BVHModel<T_BVH>* obj2_tmp = new BVHModel<T_BVH>(*obj2);
    Transform3<Scalar> tf2_tmp = tf2;

    initialize(node, *obj1_tmp, tf1_tmp, *obj2_tmp, tf2_tmp, request, result);
    collide(&node);

    delete obj1_tmp;
    delete obj2_tmp;

    return result.numContacts();
  }
};

//==============================================================================
template <typename T_BVH>
std::size_t BVHCollide(
    const CollisionGeometry<typename T_BVH::Scalar>* o1,
    const Transform3<typename T_BVH::Scalar>& tf1,
    const CollisionGeometry<typename T_BVH::Scalar>* o2,
    const Transform3<typename T_BVH::Scalar>& tf2,
    const CollisionRequest<typename T_BVH::Scalar>& request,
    CollisionResult<typename T_BVH::Scalar>& result)
{
  BVHCollideImpl<typename T_BVH::Scalar, T_BVH> tmp;
  return tmp(o1, tf1, o2, tf2, request, result);
}

namespace details
{

//==============================================================================
template <typename OrientedMeshCollisionTraversalNode, typename T_BVH>
std::size_t orientedMeshCollide(
    const CollisionGeometry<typename T_BVH::Scalar>* o1,
    const Transform3<typename T_BVH::Scalar>& tf1,
    const CollisionGeometry<typename T_BVH::Scalar>* o2,
    const Transform3<typename T_BVH::Scalar>& tf2,
    const CollisionRequest<typename T_BVH::Scalar>& request,
    CollisionResult<typename T_BVH::Scalar>& result)
{
  if(request.isSatisfied(result)) return result.numContacts();

  OrientedMeshCollisionTraversalNode node;
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
  const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>* >(o2);

  initialize(node, *obj1, tf1, *obj2, tf2, request, result);
  collide(&node);

  return result.numContacts();
}

} // namespace details

//==============================================================================
template <typename Scalar>
struct BVHCollideImpl<Scalar, OBB<Scalar>>
{
  std::size_t operator()(
      const CollisionGeometry<Scalar>* o1,
      const Transform3<Scalar>& tf1,
      const CollisionGeometry<Scalar>* o2,
      const Transform3<Scalar>& tf2,
      const CollisionRequest<Scalar>& request,
      CollisionResult<Scalar>& result)
  {
    return details::orientedMeshCollide<
        MeshCollisionTraversalNodeOBB<Scalar>, OBB<Scalar>>(
            o1, tf1, o2, tf2, request, result);
  }
};

//==============================================================================
template <typename Scalar>
struct BVHCollideImpl<Scalar, OBBRSS<Scalar>>
{
  std::size_t operator()(
      const CollisionGeometry<Scalar>* o1,
      const Transform3<Scalar>& tf1,
      const CollisionGeometry<Scalar>* o2,
      const Transform3<Scalar>& tf2,
      const CollisionRequest<Scalar>& request,
      CollisionResult<Scalar>& result)
  {
    return details::orientedMeshCollide<
        MeshCollisionTraversalNodeOBBRSS<Scalar>, OBBRSS<Scalar>>(
            o1, tf1, o2, tf2, request, result);
  }
};

//==============================================================================
template <typename Scalar>
struct BVHCollideImpl<Scalar, kIOS<Scalar>>
{
  std::size_t operator()(
      const CollisionGeometry<Scalar>* o1,
      const Transform3<Scalar>& tf1,
      const CollisionGeometry<Scalar>* o2,
      const Transform3<Scalar>& tf2,
      const CollisionRequest<Scalar>& request,
      CollisionResult<Scalar>& result)
  {
    return details::orientedMeshCollide<
        MeshCollisionTraversalNodekIOS<Scalar>, kIOS<Scalar>>(
            o1, tf1, o2, tf2, request, result);
  }
};

//==============================================================================
template <typename T_BVH, typename NarrowPhaseSolver>
std::size_t BVHCollide(
    const CollisionGeometry<typename T_BVH::Scalar>* o1,
    const Transform3<typename T_BVH::Scalar>& tf1,
    const CollisionGeometry<typename T_BVH::Scalar>* o2,
    const Transform3<typename T_BVH::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename T_BVH::Scalar>& request,
    CollisionResult<typename T_BVH::Scalar>& result)
{
  return BVHCollide<T_BVH>(o1, tf1, o2, tf2, request, result);
}

//==============================================================================
template <typename NarrowPhaseSolver>
CollisionFunctionMatrix<NarrowPhaseSolver>::CollisionFunctionMatrix()
{
  using Scalar = typename NarrowPhaseSolver::Scalar;

  for(int i = 0; i < NODE_COUNT; ++i)
  {
    for(int j = 0; j < NODE_COUNT; ++j)
      collision_matrix[i][j] = NULL;
  }

  collision_matrix[GEOM_BOX][GEOM_BOX] = &ShapeShapeCollide<Box<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_SPHERE] = &ShapeShapeCollide<Box<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_ELLIPSOID] = &ShapeShapeCollide<Box<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_CAPSULE] = &ShapeShapeCollide<Box<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_CONE] = &ShapeShapeCollide<Box<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_CYLINDER] = &ShapeShapeCollide<Box<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_CONVEX] = &ShapeShapeCollide<Box<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_PLANE] = &ShapeShapeCollide<Box<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_HALFSPACE] = &ShapeShapeCollide<Box<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  collision_matrix[GEOM_SPHERE][GEOM_BOX] = &ShapeShapeCollide<Sphere<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_SPHERE] = &ShapeShapeCollide<Sphere<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_ELLIPSOID] = &ShapeShapeCollide<Sphere<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_CAPSULE] = &ShapeShapeCollide<Sphere<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_CONE] = &ShapeShapeCollide<Sphere<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_CYLINDER] = &ShapeShapeCollide<Sphere<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_CONVEX] = &ShapeShapeCollide<Sphere<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_PLANE] = &ShapeShapeCollide<Sphere<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_HALFSPACE] = &ShapeShapeCollide<Sphere<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  collision_matrix[GEOM_ELLIPSOID][GEOM_BOX] = &ShapeShapeCollide<Ellipsoid<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_SPHERE] = &ShapeShapeCollide<Ellipsoid<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_ELLIPSOID] = &ShapeShapeCollide<Ellipsoid<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_CAPSULE] = &ShapeShapeCollide<Ellipsoid<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_CONE] = &ShapeShapeCollide<Ellipsoid<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_CYLINDER] = &ShapeShapeCollide<Ellipsoid<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_CONVEX] = &ShapeShapeCollide<Ellipsoid<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_PLANE] = &ShapeShapeCollide<Ellipsoid<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_HALFSPACE] = &ShapeShapeCollide<Ellipsoid<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  collision_matrix[GEOM_CAPSULE][GEOM_BOX] = &ShapeShapeCollide<Capsule<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_SPHERE] = &ShapeShapeCollide<Capsule<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_ELLIPSOID] = &ShapeShapeCollide<Capsule<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_CAPSULE] = &ShapeShapeCollide<Capsule<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_CONE] = &ShapeShapeCollide<Capsule<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_CYLINDER] = &ShapeShapeCollide<Capsule<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_CONVEX] = &ShapeShapeCollide<Capsule<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_PLANE] = &ShapeShapeCollide<Capsule<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_HALFSPACE] = &ShapeShapeCollide<Capsule<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  collision_matrix[GEOM_CONE][GEOM_BOX] = &ShapeShapeCollide<Cone<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_SPHERE] = &ShapeShapeCollide<Cone<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_ELLIPSOID] = &ShapeShapeCollide<Cone<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_CAPSULE] = &ShapeShapeCollide<Cone<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_CONE] = &ShapeShapeCollide<Cone<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_CYLINDER] = &ShapeShapeCollide<Cone<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_CONVEX] = &ShapeShapeCollide<Cone<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_PLANE] = &ShapeShapeCollide<Cone<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_HALFSPACE] = &ShapeShapeCollide<Cone<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  collision_matrix[GEOM_CYLINDER][GEOM_BOX] = &ShapeShapeCollide<Cylinder<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_SPHERE] = &ShapeShapeCollide<Cylinder<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_ELLIPSOID] = &ShapeShapeCollide<Cylinder<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_CAPSULE] = &ShapeShapeCollide<Cylinder<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_CONE] = &ShapeShapeCollide<Cylinder<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_CYLINDER] = &ShapeShapeCollide<Cylinder<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_CONVEX] = &ShapeShapeCollide<Cylinder<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_PLANE] = &ShapeShapeCollide<Cylinder<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_HALFSPACE] = &ShapeShapeCollide<Cylinder<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  collision_matrix[GEOM_CONVEX][GEOM_BOX] = &ShapeShapeCollide<Convex<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_SPHERE] = &ShapeShapeCollide<Convex<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_ELLIPSOID] = &ShapeShapeCollide<Convex<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_CAPSULE] = &ShapeShapeCollide<Convex<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_CONE] = &ShapeShapeCollide<Convex<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_CYLINDER] = &ShapeShapeCollide<Convex<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_CONVEX] = &ShapeShapeCollide<Convex<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_PLANE] = &ShapeShapeCollide<Convex<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_HALFSPACE] = &ShapeShapeCollide<Convex<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  collision_matrix[GEOM_PLANE][GEOM_BOX] = &ShapeShapeCollide<Plane<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_SPHERE] = &ShapeShapeCollide<Plane<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_ELLIPSOID] = &ShapeShapeCollide<Plane<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_CAPSULE] = &ShapeShapeCollide<Plane<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_CONE] = &ShapeShapeCollide<Plane<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_CYLINDER] = &ShapeShapeCollide<Plane<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_CONVEX] = &ShapeShapeCollide<Plane<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_PLANE] = &ShapeShapeCollide<Plane<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_HALFSPACE] = &ShapeShapeCollide<Plane<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  collision_matrix[GEOM_HALFSPACE][GEOM_BOX] = &ShapeShapeCollide<Halfspace<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_SPHERE] = &ShapeShapeCollide<Halfspace<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CAPSULE] = &ShapeShapeCollide<Halfspace<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CONE] = &ShapeShapeCollide<Halfspace<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CYLINDER] = &ShapeShapeCollide<Halfspace<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CONVEX] = &ShapeShapeCollide<Halfspace<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_PLANE] = &ShapeShapeCollide<Halfspace<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_HALFSPACE] = &ShapeShapeCollide<Halfspace<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  collision_matrix[BV_AABB][GEOM_BOX] = &BVHShapeCollider<AABB<Scalar>, Box<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_SPHERE] = &BVHShapeCollider<AABB<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_ELLIPSOID] = &BVHShapeCollider<AABB<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_CAPSULE] = &BVHShapeCollider<AABB<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_CONE] = &BVHShapeCollider<AABB<Scalar>, Cone<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_CYLINDER] = &BVHShapeCollider<AABB<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_CONVEX] = &BVHShapeCollider<AABB<Scalar>, Convex<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_PLANE] = &BVHShapeCollider<AABB<Scalar>, Plane<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_HALFSPACE] = &BVHShapeCollider<AABB<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>::collide;

  collision_matrix[BV_OBB][GEOM_BOX] = &BVHShapeCollider<OBB<Scalar>, Box<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_SPHERE] = &BVHShapeCollider<OBB<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_ELLIPSOID] = &BVHShapeCollider<OBB<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_CAPSULE] = &BVHShapeCollider<OBB<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_CONE] = &BVHShapeCollider<OBB<Scalar>, Cone<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_CYLINDER] = &BVHShapeCollider<OBB<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_CONVEX] = &BVHShapeCollider<OBB<Scalar>, Convex<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_PLANE] = &BVHShapeCollider<OBB<Scalar>, Plane<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_HALFSPACE] = &BVHShapeCollider<OBB<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>::collide;

  collision_matrix[BV_RSS][GEOM_BOX] = &BVHShapeCollider<RSS<Scalar>, Box<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_SPHERE] = &BVHShapeCollider<RSS<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_ELLIPSOID] = &BVHShapeCollider<RSS<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_CAPSULE] = &BVHShapeCollider<RSS<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_CONE] = &BVHShapeCollider<RSS<Scalar>, Cone<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_CYLINDER] = &BVHShapeCollider<RSS<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_CONVEX] = &BVHShapeCollider<RSS<Scalar>, Convex<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_PLANE] = &BVHShapeCollider<RSS<Scalar>, Plane<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_HALFSPACE] = &BVHShapeCollider<RSS<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>::collide;

  collision_matrix[BV_KDOP16][GEOM_BOX] = &BVHShapeCollider<KDOP<Scalar, 16>, Box<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_SPHERE] = &BVHShapeCollider<KDOP<Scalar, 16>, Sphere<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_ELLIPSOID] = &BVHShapeCollider<KDOP<Scalar, 16>, Ellipsoid<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_CAPSULE] = &BVHShapeCollider<KDOP<Scalar, 16>, Capsule<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_CONE] = &BVHShapeCollider<KDOP<Scalar, 16>, Cone<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_CYLINDER] = &BVHShapeCollider<KDOP<Scalar, 16>, Cylinder<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_CONVEX] = &BVHShapeCollider<KDOP<Scalar, 16>, Convex<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_PLANE] = &BVHShapeCollider<KDOP<Scalar, 16>, Plane<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_HALFSPACE] = &BVHShapeCollider<KDOP<Scalar, 16>, Halfspace<Scalar>, NarrowPhaseSolver>::collide;

  collision_matrix[BV_KDOP18][GEOM_BOX] = &BVHShapeCollider<KDOP<Scalar, 18>, Box<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_SPHERE] = &BVHShapeCollider<KDOP<Scalar, 18>, Sphere<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_ELLIPSOID] = &BVHShapeCollider<KDOP<Scalar, 18>, Ellipsoid<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_CAPSULE] = &BVHShapeCollider<KDOP<Scalar, 18>, Capsule<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_CONE] = &BVHShapeCollider<KDOP<Scalar, 18>, Cone<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_CYLINDER] = &BVHShapeCollider<KDOP<Scalar, 18>, Cylinder<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_CONVEX] = &BVHShapeCollider<KDOP<Scalar, 18>, Convex<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_PLANE] = &BVHShapeCollider<KDOP<Scalar, 18>, Plane<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_HALFSPACE] = &BVHShapeCollider<KDOP<Scalar, 18>, Halfspace<Scalar>, NarrowPhaseSolver>::collide;

  collision_matrix[BV_KDOP24][GEOM_BOX] = &BVHShapeCollider<KDOP<Scalar, 24>, Box<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_SPHERE] = &BVHShapeCollider<KDOP<Scalar, 24>, Sphere<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_ELLIPSOID] = &BVHShapeCollider<KDOP<Scalar, 24>, Ellipsoid<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_CAPSULE] = &BVHShapeCollider<KDOP<Scalar, 24>, Capsule<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_CONE] = &BVHShapeCollider<KDOP<Scalar, 24>, Cone<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_CYLINDER] = &BVHShapeCollider<KDOP<Scalar, 24>, Cylinder<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_CONVEX] = &BVHShapeCollider<KDOP<Scalar, 24>, Convex<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_PLANE] = &BVHShapeCollider<KDOP<Scalar, 24>, Plane<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_HALFSPACE] = &BVHShapeCollider<KDOP<Scalar, 24>, Halfspace<Scalar>, NarrowPhaseSolver>::collide;

  collision_matrix[BV_kIOS][GEOM_BOX] = &BVHShapeCollider<kIOS<Scalar>, Box<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_SPHERE] = &BVHShapeCollider<kIOS<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_ELLIPSOID] = &BVHShapeCollider<kIOS<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_CAPSULE] = &BVHShapeCollider<kIOS<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_CONE] = &BVHShapeCollider<kIOS<Scalar>, Cone<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_CYLINDER] = &BVHShapeCollider<kIOS<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_CONVEX] = &BVHShapeCollider<kIOS<Scalar>, Convex<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_PLANE] = &BVHShapeCollider<kIOS<Scalar>, Plane<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_HALFSPACE] = &BVHShapeCollider<kIOS<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>::collide;

  collision_matrix[BV_OBBRSS][GEOM_BOX] = &BVHShapeCollider<OBBRSS<Scalar>, Box<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_SPHERE] = &BVHShapeCollider<OBBRSS<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_ELLIPSOID] = &BVHShapeCollider<OBBRSS<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CAPSULE] = &BVHShapeCollider<OBBRSS<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CONE] = &BVHShapeCollider<OBBRSS<Scalar>, Cone<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CYLINDER] = &BVHShapeCollider<OBBRSS<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CONVEX] = &BVHShapeCollider<OBBRSS<Scalar>, Convex<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_PLANE] = &BVHShapeCollider<OBBRSS<Scalar>, Plane<Scalar>, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_HALFSPACE] = &BVHShapeCollider<OBBRSS<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>::collide;

  collision_matrix[BV_AABB][BV_AABB] = &BVHCollide<AABB<Scalar>, NarrowPhaseSolver>;
  collision_matrix[BV_OBB][BV_OBB] = &BVHCollide<OBB<Scalar>, NarrowPhaseSolver>;
  collision_matrix[BV_RSS][BV_RSS] = &BVHCollide<RSS<Scalar>, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP16][BV_KDOP16] = &BVHCollide<KDOP<Scalar, 16>, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP18][BV_KDOP18] = &BVHCollide<KDOP<Scalar, 18>, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP24][BV_KDOP24] = &BVHCollide<KDOP<Scalar, 24>, NarrowPhaseSolver>;
  collision_matrix[BV_kIOS][BV_kIOS] = &BVHCollide<kIOS<Scalar>, NarrowPhaseSolver>;
  collision_matrix[BV_OBBRSS][BV_OBBRSS] = &BVHCollide<OBBRSS<Scalar>, NarrowPhaseSolver>;

#if FCL_HAVE_OCTOMAP
  collision_matrix[GEOM_OCTREE][GEOM_BOX] = &OcTreeShapeCollide<Box<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_SPHERE] = &OcTreeShapeCollide<Sphere<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_ELLIPSOID] = &OcTreeShapeCollide<Ellipsoid<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_CAPSULE] = &OcTreeShapeCollide<Capsule<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_CONE] = &OcTreeShapeCollide<Cone<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_CYLINDER] = &OcTreeShapeCollide<Cylinder<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_CONVEX] = &OcTreeShapeCollide<Convex<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_PLANE] = &OcTreeShapeCollide<Plane<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_HALFSPACE] = &OcTreeShapeCollide<Halfspace<Scalar>, NarrowPhaseSolver>;

  collision_matrix[GEOM_BOX][GEOM_OCTREE] = &ShapeOcTreeCollide<Box<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_OCTREE] = &ShapeOcTreeCollide<Sphere<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_OCTREE] = &ShapeOcTreeCollide<Ellipsoid<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_OCTREE] = &ShapeOcTreeCollide<Capsule<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_OCTREE] = &ShapeOcTreeCollide<Cone<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_OCTREE] = &ShapeOcTreeCollide<Cylinder<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_OCTREE] = &ShapeOcTreeCollide<Convex<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_OCTREE] = &ShapeOcTreeCollide<Plane<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_OCTREE] = &ShapeOcTreeCollide<Halfspace<Scalar>, NarrowPhaseSolver>;

  collision_matrix[GEOM_OCTREE][GEOM_OCTREE] = &OcTreeCollide<NarrowPhaseSolver>;

  collision_matrix[GEOM_OCTREE][BV_AABB] = &OcTreeBVHCollide<AABB<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_OBB] = &OcTreeBVHCollide<OBB<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_RSS] = &OcTreeBVHCollide<RSS<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_OBBRSS] = &OcTreeBVHCollide<OBBRSS<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_kIOS] = &OcTreeBVHCollide<kIOS<Scalar>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_KDOP16] = &OcTreeBVHCollide<KDOP<Scalar, 16>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_KDOP18] = &OcTreeBVHCollide<KDOP<Scalar, 18>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_KDOP24] = &OcTreeBVHCollide<KDOP<Scalar, 24>, NarrowPhaseSolver>;

  collision_matrix[BV_AABB][GEOM_OCTREE] = &BVHOcTreeCollide<AABB<Scalar>, NarrowPhaseSolver>;
  collision_matrix[BV_OBB][GEOM_OCTREE] = &BVHOcTreeCollide<OBB<Scalar>, NarrowPhaseSolver>;
  collision_matrix[BV_RSS][GEOM_OCTREE] = &BVHOcTreeCollide<RSS<Scalar>, NarrowPhaseSolver>;
  collision_matrix[BV_OBBRSS][GEOM_OCTREE] = &BVHOcTreeCollide<OBBRSS<Scalar>, NarrowPhaseSolver>;
  collision_matrix[BV_kIOS][GEOM_OCTREE] = &BVHOcTreeCollide<kIOS<Scalar>, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP16][GEOM_OCTREE] = &BVHOcTreeCollide<KDOP<Scalar, 16>, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP18][GEOM_OCTREE] = &BVHOcTreeCollide<KDOP<Scalar, 18>, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP24][GEOM_OCTREE] = &BVHOcTreeCollide<KDOP<Scalar, 24>, NarrowPhaseSolver>;
#endif
}

} // namespace fcl

#endif

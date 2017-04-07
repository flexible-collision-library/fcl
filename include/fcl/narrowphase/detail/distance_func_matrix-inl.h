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

#ifndef FCL_DISTANCE_FUNC_MATRIX_INL_H
#define FCL_DISTANCE_FUNC_MATRIX_INL_H

#include "fcl/narrowphase/detail/distance_func_matrix.h"

#include "fcl/config.h"

#include "fcl/common/types.h"
#include "fcl/common/unused.h"

#include "fcl/narrowphase/collision_object.h"

#include "fcl/narrowphase/detail/traversal/collision_node.h"
#include "fcl/narrowphase/detail/gjk_solver_indep.h"
#include "fcl/narrowphase/detail/gjk_solver_libccd.h"

#include "fcl/narrowphase/detail/traversal/distance/bvh_distance_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/distance/bvh_shape_distance_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/distance/distance_traversal_node_base.h"
#include "fcl/narrowphase/detail/traversal/distance/mesh_distance_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/distance/mesh_conservative_advancement_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/distance/mesh_shape_distance_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/distance/mesh_shape_conservative_advancement_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/distance/shape_bvh_distance_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/distance/shape_distance_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/distance/shape_conservative_advancement_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/distance/shape_mesh_distance_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/distance/shape_mesh_conservative_advancement_traversal_node.h"

#if FCL_HAVE_OCTOMAP

#include "fcl/narrowphase/detail/traversal/octree/distance/mesh_octree_distance_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/octree/distance/octree_distance_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/octree/distance/octree_mesh_distance_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/octree/distance/octree_shape_distance_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/octree/distance/shape_octree_distance_traversal_node.h"

#endif // FCL_HAVE_OCTOMAP

namespace fcl
{

namespace detail
{

//==============================================================================
#if FCL_HAVE_OCTOMAP
template <typename Shape, typename NarrowPhaseSolver>
typename Shape::S ShapeOcTreeDistance(
    const CollisionGeometry<typename Shape::S>* o1,
    const Transform3<typename Shape::S>& tf1,
    const CollisionGeometry<typename Shape::S>* o2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename Shape::S>& request,
    DistanceResult<typename Shape::S>& result)
{
  using S = typename Shape::S;

  if(request.isSatisfied(result)) return result.min_distance;
  ShapeOcTreeDistanceTraversalNode<Shape, NarrowPhaseSolver> node;
  const Shape* obj1 = static_cast<const Shape*>(o1);
  const OcTree<S>* obj2 = static_cast<const OcTree<S>*>(o2);
  OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
  distance(&node);

  return result.min_distance;
}

template <typename Shape, typename NarrowPhaseSolver>
typename Shape::S OcTreeShapeDistance(
    const CollisionGeometry<typename Shape::S>* o1,
    const Transform3<typename Shape::S>& tf1,
    const CollisionGeometry<typename Shape::S>* o2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename Shape::S>& request,
    DistanceResult<typename Shape::S>& result)
{
  using S = typename Shape::S;

  if(request.isSatisfied(result)) return result.min_distance;
  OcTreeShapeDistanceTraversalNode<Shape, NarrowPhaseSolver> node;
  const OcTree<S>* obj1 = static_cast<const OcTree<S>*>(o1);
  const Shape* obj2 = static_cast<const Shape*>(o2);
  OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
  distance(&node);

  return result.min_distance;
}

template <typename NarrowPhaseSolver>
typename NarrowPhaseSolver::S OcTreeDistance(
    const CollisionGeometry<typename NarrowPhaseSolver::S>* o1,
    const Transform3<typename NarrowPhaseSolver::S>& tf1,
    const CollisionGeometry<typename NarrowPhaseSolver::S>* o2,
    const Transform3<typename NarrowPhaseSolver::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename NarrowPhaseSolver::S>& request,
    DistanceResult<typename NarrowPhaseSolver::S>& result)
{
  using S = typename NarrowPhaseSolver::S;

  if(request.isSatisfied(result)) return result.min_distance;
  OcTreeDistanceTraversalNode<NarrowPhaseSolver> node;
  const OcTree<S>* obj1 = static_cast<const OcTree<S>*>(o1);
  const OcTree<S>* obj2 = static_cast<const OcTree<S>*>(o2);
  OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
  distance(&node);

  return result.min_distance;
}

template <typename BV, typename NarrowPhaseSolver>
typename BV::S BVHOcTreeDistance(
    const CollisionGeometry<typename BV::S>* o1,
    const Transform3<typename BV::S>& tf1,
    const CollisionGeometry<typename BV::S>* o2,
    const Transform3<typename BV::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename BV::S>& request,
    DistanceResult<typename BV::S>& result)
{
  using S = typename BV::S;

  if(request.isSatisfied(result)) return result.min_distance;
  MeshOcTreeDistanceTraversalNode<BV, NarrowPhaseSolver> node;
  const BVHModel<BV>* obj1 = static_cast<const BVHModel<BV>*>(o1);
  const OcTree<S>* obj2 = static_cast<const OcTree<S>*>(o2);
  OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
  distance(&node);

  return result.min_distance;
}

template <typename BV, typename NarrowPhaseSolver>
typename BV::S OcTreeBVHDistance(
    const CollisionGeometry<typename BV::S>* o1,
    const Transform3<typename BV::S>& tf1,
    const CollisionGeometry<typename BV::S>* o2,
    const Transform3<typename BV::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename BV::S>& request,
    DistanceResult<typename BV::S>& result)
{
  using S = typename BV::S;

  if(request.isSatisfied(result)) return result.min_distance;
  OcTreeMeshDistanceTraversalNode<BV, NarrowPhaseSolver> node;
  const OcTree<S>* obj1 = static_cast<const OcTree<S>*>(o1);
  const BVHModel<BV>* obj2 = static_cast<const BVHModel<BV>*>(o2);
  OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
  distance(&node);

  return result.min_distance;
}

#endif

template <typename Shape1, typename Shape2, typename NarrowPhaseSolver>
typename Shape1::S ShapeShapeDistance(
    const CollisionGeometry<typename Shape1::S>* o1,
    const Transform3<typename Shape1::S>& tf1,
    const CollisionGeometry<typename Shape1::S>* o2,
    const Transform3<typename Shape1::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename Shape1::S>& request,
    DistanceResult<typename Shape1::S>& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  ShapeDistanceTraversalNode<Shape1, Shape2, NarrowPhaseSolver> node;
  const Shape1* obj1 = static_cast<const Shape1*>(o1);
  const Shape2* obj2 = static_cast<const Shape2*>(o2);

  initialize(node, *obj1, tf1, *obj2, tf2, nsolver, request, result);
  distance(&node);

  return result.min_distance;
}

template <typename BV, typename Shape, typename NarrowPhaseSolver>
struct BVHShapeDistancer
{
  using S = typename BV::S;

  static S distance(
      const CollisionGeometry<S>* o1,
      const Transform3<S>& tf1,
      const CollisionGeometry<S>* o2,
      const Transform3<S>& tf2,
      const NarrowPhaseSolver* nsolver,
      const DistanceRequest<S>& request,
      DistanceResult<S>& result)
  {
    if(request.isSatisfied(result)) return result.min_distance;
    MeshShapeDistanceTraversalNode<BV, Shape, NarrowPhaseSolver> node;
    const BVHModel<BV>* obj1 = static_cast<const BVHModel<BV>* >(o1);
    BVHModel<BV>* obj1_tmp = new BVHModel<BV>(*obj1);
    Transform3<S> tf1_tmp = tf1;
    const Shape* obj2 = static_cast<const Shape*>(o2);

    initialize(node, *obj1_tmp, tf1_tmp, *obj2, tf2, nsolver, request, result);
    ::fcl::distance(&node);

    delete obj1_tmp;
    return result.min_distance;
  }
};

template <typename OrientedMeshShapeDistanceTraversalNode,
          typename BV, typename Shape, typename NarrowPhaseSolver>
typename Shape::S orientedBVHShapeDistance(
    const CollisionGeometry<typename Shape::S>* o1,
    const Transform3<typename Shape::S>& tf1,
    const CollisionGeometry<typename Shape::S>* o2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename Shape::S>&
    request, DistanceResult<typename Shape::S>& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  OrientedMeshShapeDistanceTraversalNode node;
  const BVHModel<BV>* obj1 = static_cast<const BVHModel<BV>* >(o1);
  const Shape* obj2 = static_cast<const Shape*>(o2);

  initialize(node, *obj1, tf1, *obj2, tf2, nsolver, request, result);
  distance(&node);

  return result.min_distance;
}

template <typename Shape, typename NarrowPhaseSolver>
struct BVHShapeDistancer<RSS<typename Shape::S>, Shape, NarrowPhaseSolver>
{
  static typename Shape::S distance(
      const CollisionGeometry<typename Shape::S>* o1,
      const Transform3<typename Shape::S>& tf1,
      const CollisionGeometry<typename Shape::S>* o2,
      const Transform3<typename Shape::S>& tf2,
      const NarrowPhaseSolver* nsolver,
      const DistanceRequest<typename Shape::S>& request,
      DistanceResult<typename Shape::S>& result)
  {
    return detail::orientedBVHShapeDistance<
        MeshShapeDistanceTraversalNodeRSS<Shape, NarrowPhaseSolver>,
        RSS<typename Shape::S>,
        Shape,
        NarrowPhaseSolver>(
          o1, tf1, o2, tf2, nsolver, request, result);
  }
};

template <typename Shape, typename NarrowPhaseSolver>
struct BVHShapeDistancer<kIOS<typename Shape::S>, Shape, NarrowPhaseSolver>
{
  static typename Shape::S distance(
      const CollisionGeometry<typename Shape::S>* o1,
      const Transform3<typename Shape::S>& tf1,
      const CollisionGeometry<typename Shape::S>* o2,
      const Transform3<typename Shape::S>& tf2,
      const NarrowPhaseSolver* nsolver,
      const DistanceRequest<typename Shape::S>& request,
      DistanceResult<typename Shape::S>& result)
  {
    return detail::orientedBVHShapeDistance<
        MeshShapeDistanceTraversalNodekIOS<Shape, NarrowPhaseSolver>,
        kIOS<typename Shape::S>,
        Shape,
        NarrowPhaseSolver>(
          o1, tf1, o2, tf2, nsolver, request, result);
  }
};

template <typename Shape, typename NarrowPhaseSolver>
struct BVHShapeDistancer<OBBRSS<typename Shape::S>, Shape, NarrowPhaseSolver>
{
  static typename Shape::S distance(
      const CollisionGeometry<typename Shape::S>* o1,
      const Transform3<typename Shape::S>& tf1,
      const CollisionGeometry<typename Shape::S>* o2,
      const Transform3<typename Shape::S>& tf2,
      const NarrowPhaseSolver* nsolver,
      const DistanceRequest<typename Shape::S>& request,
      DistanceResult<typename Shape::S>& result)
  {
    return detail::orientedBVHShapeDistance<
        MeshShapeDistanceTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>,
        OBBRSS<typename Shape::S>,
        Shape,
        NarrowPhaseSolver>(
          o1, tf1, o2, tf2, nsolver, request, result);
  }
};

//==============================================================================
template <typename S, typename BV>
struct BVHDistanceImpl
{
  static S run(
      const CollisionGeometry<S>* o1,
      const Transform3<S>& tf1,
      const CollisionGeometry<S>* o2,
      const Transform3<S>& tf2,
      const DistanceRequest<S>& request,
      DistanceResult<S>& result)
  {
    if(request.isSatisfied(result)) return result.min_distance;
    MeshDistanceTraversalNode<BV> node;
    const BVHModel<BV>* obj1 = static_cast<const BVHModel<BV>* >(o1);
    const BVHModel<BV>* obj2 = static_cast<const BVHModel<BV>* >(o2);
    BVHModel<BV>* obj1_tmp = new BVHModel<BV>(*obj1);
    Transform3<S> tf1_tmp = tf1;
    BVHModel<BV>* obj2_tmp = new BVHModel<BV>(*obj2);
    Transform3<S> tf2_tmp = tf2;

    initialize(node, *obj1_tmp, tf1_tmp, *obj2_tmp, tf2_tmp, request, result);
    distance(&node);
    delete obj1_tmp;
    delete obj2_tmp;

    return result.min_distance;
  }
};

//==============================================================================
template <typename BV>
typename BV::S BVHDistance(
    const CollisionGeometry<typename BV::S>* o1,
    const Transform3<typename BV::S>& tf1,
    const CollisionGeometry<typename BV::S>* o2,
    const Transform3<typename BV::S>& tf2,
    const DistanceRequest<typename BV::S>& request,
    DistanceResult<typename BV::S>& result)
{
  return BVHDistanceImpl<typename BV::S, BV>::run(
        o1, tf1, o2, tf2, request, result);
}

template <typename OrientedMeshDistanceTraversalNode, typename BV>
typename BV::S orientedMeshDistance(
    const CollisionGeometry<typename BV::S>* o1,
    const Transform3<typename BV::S>& tf1,
    const CollisionGeometry<typename BV::S>* o2,
    const Transform3<typename BV::S>& tf2,
    const DistanceRequest<typename BV::S>& request,
    DistanceResult<typename BV::S>& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  OrientedMeshDistanceTraversalNode node;
  const BVHModel<BV>* obj1 = static_cast<const BVHModel<BV>* >(o1);
  const BVHModel<BV>* obj2 = static_cast<const BVHModel<BV>* >(o2);

  initialize(node, *obj1, tf1, *obj2, tf2, request, result);
  distance(&node);

  return result.min_distance;
}

//==============================================================================
template <typename S>
struct BVHDistanceImpl<S, RSS<S>>
{
  static S run(
      const CollisionGeometry<S>* o1,
      const Transform3<S>& tf1,
      const CollisionGeometry<S>* o2,
      const Transform3<S>& tf2,
      const DistanceRequest<S>& request,
      DistanceResult<S>& result)
  {
    return detail::orientedMeshDistance<
        MeshDistanceTraversalNodeRSS<S>, RSS<S>>(
            o1, tf1, o2, tf2, request, result);
  }
};

//==============================================================================
template <typename S>
struct BVHDistanceImpl<S, kIOS<S>>
{
  static S run(
      const CollisionGeometry<S>* o1,
      const Transform3<S>& tf1,
      const CollisionGeometry<S>* o2,
      const Transform3<S>& tf2,
      const DistanceRequest<S>& request,
      DistanceResult<S>& result)
  {
    return detail::orientedMeshDistance<
        MeshDistanceTraversalNodekIOS<S>, kIOS<S>>(
            o1, tf1, o2, tf2, request, result);
  }
};

//==============================================================================
template <typename S>
struct BVHDistanceImpl<S, OBBRSS<S>>
{
  static S run(
      const CollisionGeometry<S>* o1,
      const Transform3<S>& tf1,
      const CollisionGeometry<S>* o2,
      const Transform3<S>& tf2,
      const DistanceRequest<S>& request,
      DistanceResult<S>& result)
  {
    return detail::orientedMeshDistance<
        MeshDistanceTraversalNodeOBBRSS<S>, OBBRSS<S>>(
            o1, tf1, o2, tf2, request, result);
  }
};

//==============================================================================
template <typename BV, typename NarrowPhaseSolver>
typename BV::S BVHDistance(
    const CollisionGeometry<typename BV::S>* o1,
    const Transform3<typename BV::S>& tf1,
    const CollisionGeometry<typename BV::S>* o2,
    const Transform3<typename BV::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename BV::S>& request,
    DistanceResult<typename BV::S>& result)
{
  FCL_UNUSED(nsolver);

  return BVHDistance<BV>(o1, tf1, o2, tf2, request, result);
}

template <typename NarrowPhaseSolver>
DistanceFunctionMatrix<NarrowPhaseSolver>::DistanceFunctionMatrix()
{
  for(int i = 0; i < NODE_COUNT; ++i)
  {
    for(int j = 0; j < NODE_COUNT; ++j)
      distance_matrix[i][j] = nullptr;
  }

  distance_matrix[GEOM_BOX][GEOM_BOX] = &ShapeShapeDistance<Box<S>, Box<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_SPHERE] = &ShapeShapeDistance<Box<S>, Sphere<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_ELLIPSOID] = &ShapeShapeDistance<Box<S>, Ellipsoid<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_CAPSULE] = &ShapeShapeDistance<Box<S>, Capsule<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_CONE] = &ShapeShapeDistance<Box<S>, Cone<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_CYLINDER] = &ShapeShapeDistance<Box<S>, Cylinder<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_CONVEX] = &ShapeShapeDistance<Box<S>, Convex<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_PLANE] = &ShapeShapeDistance<Box<S>, Plane<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_HALFSPACE] = &ShapeShapeDistance<Box<S>, Halfspace<S>, NarrowPhaseSolver>;

  distance_matrix[GEOM_SPHERE][GEOM_BOX] = &ShapeShapeDistance<Sphere<S>, Box<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_SPHERE] = &ShapeShapeDistance<Sphere<S>, Sphere<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_ELLIPSOID] = &ShapeShapeDistance<Sphere<S>, Ellipsoid<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_CAPSULE] = &ShapeShapeDistance<Sphere<S>, Capsule<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_CONE] = &ShapeShapeDistance<Sphere<S>, Cone<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_CYLINDER] = &ShapeShapeDistance<Sphere<S>, Cylinder<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_CONVEX] = &ShapeShapeDistance<Sphere<S>, Convex<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_PLANE] = &ShapeShapeDistance<Sphere<S>, Plane<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_HALFSPACE] = &ShapeShapeDistance<Sphere<S>, Halfspace<S>, NarrowPhaseSolver>;

  distance_matrix[GEOM_ELLIPSOID][GEOM_BOX] = &ShapeShapeDistance<Ellipsoid<S>, Box<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_SPHERE] = &ShapeShapeDistance<Ellipsoid<S>, Sphere<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_ELLIPSOID] = &ShapeShapeDistance<Ellipsoid<S>, Ellipsoid<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_CAPSULE] = &ShapeShapeDistance<Ellipsoid<S>, Capsule<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_CONE] = &ShapeShapeDistance<Ellipsoid<S>, Cone<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_CYLINDER] = &ShapeShapeDistance<Ellipsoid<S>, Cylinder<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_CONVEX] = &ShapeShapeDistance<Ellipsoid<S>, Convex<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_PLANE] = &ShapeShapeDistance<Ellipsoid<S>, Plane<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_HALFSPACE] = &ShapeShapeDistance<Ellipsoid<S>, Halfspace<S>, NarrowPhaseSolver>;

  distance_matrix[GEOM_CAPSULE][GEOM_BOX] = &ShapeShapeDistance<Capsule<S>, Box<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_SPHERE] = &ShapeShapeDistance<Capsule<S>, Sphere<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_ELLIPSOID] = &ShapeShapeDistance<Capsule<S>, Ellipsoid<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_CAPSULE] = &ShapeShapeDistance<Capsule<S>, Capsule<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_CONE] = &ShapeShapeDistance<Capsule<S>, Cone<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_CYLINDER] = &ShapeShapeDistance<Capsule<S>, Cylinder<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_CONVEX] = &ShapeShapeDistance<Capsule<S>, Convex<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_PLANE] = &ShapeShapeDistance<Capsule<S>, Plane<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_HALFSPACE] = &ShapeShapeDistance<Capsule<S>, Halfspace<S>, NarrowPhaseSolver>;

  distance_matrix[GEOM_CONE][GEOM_BOX] = &ShapeShapeDistance<Cone<S>, Box<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_SPHERE] = &ShapeShapeDistance<Cone<S>, Sphere<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_ELLIPSOID] = &ShapeShapeDistance<Cone<S>, Ellipsoid<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_CAPSULE] = &ShapeShapeDistance<Cone<S>, Capsule<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_CONE] = &ShapeShapeDistance<Cone<S>, Cone<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_CYLINDER] = &ShapeShapeDistance<Cone<S>, Cylinder<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_CONVEX] = &ShapeShapeDistance<Cone<S>, Convex<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_PLANE] = &ShapeShapeDistance<Cone<S>, Plane<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_HALFSPACE] = &ShapeShapeDistance<Cone<S>, Halfspace<S>, NarrowPhaseSolver>;

  distance_matrix[GEOM_CYLINDER][GEOM_BOX] = &ShapeShapeDistance<Cylinder<S>, Box<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_SPHERE] = &ShapeShapeDistance<Cylinder<S>, Sphere<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_ELLIPSOID] = &ShapeShapeDistance<Cylinder<S>, Ellipsoid<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_CAPSULE] = &ShapeShapeDistance<Cylinder<S>, Capsule<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_CONE] = &ShapeShapeDistance<Cylinder<S>, Cone<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_CYLINDER] = &ShapeShapeDistance<Cylinder<S>, Cylinder<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_CONVEX] = &ShapeShapeDistance<Cylinder<S>, Convex<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_PLANE] = &ShapeShapeDistance<Cylinder<S>, Plane<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_HALFSPACE] = &ShapeShapeDistance<Cylinder<S>, Halfspace<S>, NarrowPhaseSolver>;

  distance_matrix[GEOM_CONVEX][GEOM_BOX] = &ShapeShapeDistance<Convex<S>, Box<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_SPHERE] = &ShapeShapeDistance<Convex<S>, Sphere<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_ELLIPSOID] = &ShapeShapeDistance<Convex<S>, Ellipsoid<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_CAPSULE] = &ShapeShapeDistance<Convex<S>, Capsule<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_CONE] = &ShapeShapeDistance<Convex<S>, Cone<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_CYLINDER] = &ShapeShapeDistance<Convex<S>, Cylinder<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_CONVEX] = &ShapeShapeDistance<Convex<S>, Convex<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_PLANE] = &ShapeShapeDistance<Convex<S>, Plane<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_HALFSPACE] = &ShapeShapeDistance<Convex<S>, Halfspace<S>, NarrowPhaseSolver>;

  distance_matrix[GEOM_PLANE][GEOM_BOX] = &ShapeShapeDistance<Plane<S>, Box<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_SPHERE] = &ShapeShapeDistance<Plane<S>, Sphere<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_ELLIPSOID] = &ShapeShapeDistance<Plane<S>, Ellipsoid<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_CAPSULE] = &ShapeShapeDistance<Plane<S>, Capsule<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_CONE] = &ShapeShapeDistance<Plane<S>, Cone<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_CYLINDER] = &ShapeShapeDistance<Plane<S>, Cylinder<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_CONVEX] = &ShapeShapeDistance<Plane<S>, Convex<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_PLANE] = &ShapeShapeDistance<Plane<S>, Plane<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_HALFSPACE] = &ShapeShapeDistance<Plane<S>, Halfspace<S>, NarrowPhaseSolver>;

  distance_matrix[GEOM_HALFSPACE][GEOM_BOX] = &ShapeShapeDistance<Halfspace<S>, Box<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_SPHERE] = &ShapeShapeDistance<Halfspace<S>, Sphere<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CAPSULE] = &ShapeShapeDistance<Halfspace<S>, Capsule<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CONE] = &ShapeShapeDistance<Halfspace<S>, Cone<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CYLINDER] = &ShapeShapeDistance<Halfspace<S>, Cylinder<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CONVEX] = &ShapeShapeDistance<Halfspace<S>, Convex<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_PLANE] = &ShapeShapeDistance<Halfspace<S>, Plane<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_HALFSPACE] = &ShapeShapeDistance<Halfspace<S>, Halfspace<S>, NarrowPhaseSolver>;

  /* AABB distance not implemented */
  /*
  distance_matrix[BV_AABB][GEOM_BOX] = &BVHShapeDistancer<AABB<S>, Box<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_SPHERE] = &BVHShapeDistancer<AABB<S>, Sphere<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_ELLIPSOID] = &BVHShapeDistancer<AABB<S>, Ellipsoid<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CAPSULE] = &BVHShapeDistancer<AABB<S>, Capsule<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CONE] = &BVHShapeDistancer<AABB<S>, Cone<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CYLINDER] = &BVHShapeDistancer<AABB<S>, Cylinder<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CONVEX] = &BVHShapeDistancer<AABB<S>, Convex<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_PLANE] = &BVHShapeDistancer<AABB<S>, Plane<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_HALFSPACE] = &BVHShapeDistancer<AABB<S>, Halfspace<S>, NarrowPhaseSolver>::distance;

  distance_matrix[BV_OBB][GEOM_BOX] = &BVHShapeDistancer<OBB<S>, Box<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_SPHERE] = &BVHShapeDistancer<OBB<S>, Sphere<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_ELLIPSOID] = &BVHShapeDistancer<OBB<S>, Ellipsoid<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_CAPSULE] = &BVHShapeDistancer<OBB<S>, Capsule<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_CONE] = &BVHShapeDistancer<OBB<S>, Cone<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_CYLINDER] = &BVHShapeDistancer<OBB<S>, Cylinder<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_CONVEX] = &BVHShapeDistancer<OBB<S>, Convex<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_PLANE] = &BVHShapeDistancer<OBB<S>, Plane<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_HALFSPACE] = &BVHShapeDistancer<OBB<S>, Halfspace<S>, NarrowPhaseSolver>::distance;
  */

  distance_matrix[BV_RSS][GEOM_BOX] = &BVHShapeDistancer<RSS<S>, Box<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_SPHERE] = &BVHShapeDistancer<RSS<S>, Sphere<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_ELLIPSOID] = &BVHShapeDistancer<RSS<S>, Ellipsoid<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CAPSULE] = &BVHShapeDistancer<RSS<S>, Capsule<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CONE] = &BVHShapeDistancer<RSS<S>, Cone<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CYLINDER] = &BVHShapeDistancer<RSS<S>, Cylinder<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CONVEX] = &BVHShapeDistancer<RSS<S>, Convex<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_PLANE] = &BVHShapeDistancer<RSS<S>, Plane<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_HALFSPACE] = &BVHShapeDistancer<RSS<S>, Halfspace<S>, NarrowPhaseSolver>::distance;

  /* KDOPd distance not implemented */
  /*
  distance_matrix[BV_KDOP16][GEOM_BOX] = &BVHShapeDistancer<KDOP<S, 16>, Box<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_SPHERE] = &BVHShapeDistancer<KDOP<S, 16>, Sphere<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_ELLIPSOID] = &BVHShapeDistancer<KDOP<S, 16>, Ellipsoid<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CAPSULE] = &BVHShapeDistancer<KDOP<S, 16>, Capsule<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CONE] = &BVHShapeDistancer<KDOP<S, 16>, Cone<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CYLINDER] = &BVHShapeDistancer<KDOP<S, 16>, Cylinder<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CONVEX] = &BVHShapeDistancer<KDOP<S, 16>, Convex<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_PLANE] = &BVHShapeDistancer<KDOP<S, 16>, Plane<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_HALFSPACE] = &BVHShapeDistancer<KDOP<S, 16>, Halfspace<S>, NarrowPhaseSolver>::distance;

  distance_matrix[BV_KDOP18][GEOM_BOX] = &BVHShapeDistancer<KDOP<S, 18>, Box<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_SPHERE] = &BVHShapeDistancer<KDOP<S, 18>, Sphere<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_ELLIPSOID] = &BVHShapeDistancer<KDOP<S, 18>, Ellipsoid<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CAPSULE] = &BVHShapeDistancer<KDOP<S, 18>, Capsule<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CONE] = &BVHShapeDistancer<KDOP<S, 18>, Cone<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CYLINDER] = &BVHShapeDistancer<KDOP<S, 18>, Cylinder<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CONVEX] = &BVHShapeDistancer<KDOP<S, 18>, Convex<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_PLANE] = &BVHShapeDistancer<KDOP<S, 18>, Plane<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_HALFSPACE] = &BVHShapeDistancer<KDOP<S, 18>, Halfspace<S>, NarrowPhaseSolver>::distance;

  distance_matrix[BV_KDOP24][GEOM_BOX] = &BVHShapeDistancer<KDOP<S, 24>, Box<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_SPHERE] = &BVHShapeDistancer<KDOP<S, 24>, Sphere<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_ELLIPSOID] = &BVHShapeDistancer<KDOP<S, 24>, Ellipsoid<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CAPSULE] = &BVHShapeDistancer<KDOP<S, 24>, Capsule<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CONE] = &BVHShapeDistancer<KDOP<S, 24>, Cone<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CYLINDER] = &BVHShapeDistancer<KDOP<S, 24>, Cylinder<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CONVEX] = &BVHShapeDistancer<KDOP<S, 24>, Convex<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_PLANE] = &BVHShapeDistancer<KDOP<S, 24>, Plane<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_HALFSPACE] = &BVHShapeDistancer<KDOP<S, 24>, Halfspace<S>, NarrowPhaseSolver>::distance;
  */

  distance_matrix[BV_kIOS][GEOM_BOX] = &BVHShapeDistancer<kIOS<S>, Box<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_SPHERE] = &BVHShapeDistancer<kIOS<S>, Sphere<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_ELLIPSOID] = &BVHShapeDistancer<kIOS<S>, Ellipsoid<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CAPSULE] = &BVHShapeDistancer<kIOS<S>, Capsule<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CONE] = &BVHShapeDistancer<kIOS<S>, Cone<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CYLINDER] = &BVHShapeDistancer<kIOS<S>, Cylinder<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CONVEX] = &BVHShapeDistancer<kIOS<S>, Convex<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_PLANE] = &BVHShapeDistancer<kIOS<S>, Plane<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_HALFSPACE] = &BVHShapeDistancer<kIOS<S>, Halfspace<S>, NarrowPhaseSolver>::distance;

  distance_matrix[BV_OBBRSS][GEOM_BOX] = &BVHShapeDistancer<OBBRSS<S>, Box<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_SPHERE] = &BVHShapeDistancer<OBBRSS<S>, Sphere<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_ELLIPSOID] = &BVHShapeDistancer<OBBRSS<S>, Ellipsoid<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CAPSULE] = &BVHShapeDistancer<OBBRSS<S>, Capsule<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CONE] = &BVHShapeDistancer<OBBRSS<S>, Cone<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CYLINDER] = &BVHShapeDistancer<OBBRSS<S>, Cylinder<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CONVEX] = &BVHShapeDistancer<OBBRSS<S>, Convex<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_PLANE] = &BVHShapeDistancer<OBBRSS<S>, Plane<S>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_HALFSPACE] = &BVHShapeDistancer<OBBRSS<S>, Halfspace<S>, NarrowPhaseSolver>::distance;

  distance_matrix[BV_AABB][BV_AABB] = &BVHDistance<AABB<S>, NarrowPhaseSolver>;
  distance_matrix[BV_RSS][BV_RSS] = &BVHDistance<RSS<S>, NarrowPhaseSolver>;
  distance_matrix[BV_kIOS][BV_kIOS] = &BVHDistance<kIOS<S>, NarrowPhaseSolver>;
  distance_matrix[BV_OBBRSS][BV_OBBRSS] = &BVHDistance<OBBRSS<S>, NarrowPhaseSolver>;

#if FCL_HAVE_OCTOMAP
  distance_matrix[GEOM_OCTREE][GEOM_BOX] = &OcTreeShapeDistance<Box<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_SPHERE] = &OcTreeShapeDistance<Sphere<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_ELLIPSOID] = &OcTreeShapeDistance<Ellipsoid<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_CAPSULE] = &OcTreeShapeDistance<Capsule<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_CONE] = &OcTreeShapeDistance<Cone<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_CYLINDER] = &OcTreeShapeDistance<Cylinder<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_CONVEX] = &OcTreeShapeDistance<Convex<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_PLANE] = &OcTreeShapeDistance<Plane<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_HALFSPACE] = &OcTreeShapeDistance<Halfspace<S>, NarrowPhaseSolver>;

  distance_matrix[GEOM_BOX][GEOM_OCTREE] = &ShapeOcTreeDistance<Box<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_OCTREE] = &ShapeOcTreeDistance<Sphere<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_OCTREE] = &ShapeOcTreeDistance<Ellipsoid<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_OCTREE] = &ShapeOcTreeDistance<Capsule<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_OCTREE] = &ShapeOcTreeDistance<Cone<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_OCTREE] = &ShapeOcTreeDistance<Cylinder<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_OCTREE] = &ShapeOcTreeDistance<Convex<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_OCTREE] = &ShapeOcTreeDistance<Plane<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_OCTREE] = &ShapeOcTreeDistance<Halfspace<S>, NarrowPhaseSolver>;

  distance_matrix[GEOM_OCTREE][GEOM_OCTREE] = &OcTreeDistance<NarrowPhaseSolver>;

  distance_matrix[GEOM_OCTREE][BV_AABB] = &OcTreeBVHDistance<AABB<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_OBB] = &OcTreeBVHDistance<OBB<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_RSS] = &OcTreeBVHDistance<RSS<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_OBBRSS] = &OcTreeBVHDistance<OBBRSS<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_kIOS] = &OcTreeBVHDistance<kIOS<S>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_KDOP16] = &OcTreeBVHDistance<KDOP<S, 16>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_KDOP18] = &OcTreeBVHDistance<KDOP<S, 18>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_KDOP24] = &OcTreeBVHDistance<KDOP<S, 24>, NarrowPhaseSolver>;

  distance_matrix[BV_AABB][GEOM_OCTREE] = &BVHOcTreeDistance<AABB<S>, NarrowPhaseSolver>;
  distance_matrix[BV_OBB][GEOM_OCTREE] = &BVHOcTreeDistance<OBB<S>, NarrowPhaseSolver>;
  distance_matrix[BV_RSS][GEOM_OCTREE] = &BVHOcTreeDistance<RSS<S>, NarrowPhaseSolver>;
  distance_matrix[BV_OBBRSS][GEOM_OCTREE] = &BVHOcTreeDistance<OBBRSS<S>, NarrowPhaseSolver>;
  distance_matrix[BV_kIOS][GEOM_OCTREE] = &BVHOcTreeDistance<kIOS<S>, NarrowPhaseSolver>;
  distance_matrix[BV_KDOP16][GEOM_OCTREE] = &BVHOcTreeDistance<KDOP<S, 16>, NarrowPhaseSolver>;
  distance_matrix[BV_KDOP18][GEOM_OCTREE] = &BVHOcTreeDistance<KDOP<S, 18>, NarrowPhaseSolver>;
  distance_matrix[BV_KDOP24][GEOM_OCTREE] = &BVHOcTreeDistance<KDOP<S, 24>, NarrowPhaseSolver>;
#endif

}

} // namespace detail
} // namespace fcl

#endif

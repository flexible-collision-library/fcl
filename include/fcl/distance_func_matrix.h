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

#ifndef FCL_DISTANCE_FUNC_MATRIX_H
#define FCL_DISTANCE_FUNC_MATRIX_H

#include "fcl/collision_object.h"
#include "fcl/collision_data.h"
#include "fcl/collision_node.h"
#include "fcl/narrowphase/narrowphase.h"

#include "fcl/traversal/shape_distance_traversal_node.h"
#include "fcl/traversal/mesh_shape_distance_traversal_node.h"
#include "fcl/traversal/shape_mesh_distance_traversal_node.h"
#include "fcl/traversal/mesh_distance_traversal_node.h"

#include "fcl/config.h"
#if FCL_HAVE_OCTOMAP
#include "fcl/traversal/octree/mesh_octree_distance_traversal_node.h"
#include "fcl/traversal/octree/octree_distance_traversal_node.h"
#include "fcl/traversal/octree/octree_mesh_distance_traversal_node.h"
#include "fcl/traversal/octree/octree_shape_distance_traversal_node.h"
#include "fcl/traversal/octree/octree_solver.h"
#include "fcl/traversal/octree/shape_octree_distance_traversal_node.h"
#endif

namespace fcl
{

/// @brief distance matrix stores the functions for distance between different types of objects and provides a uniform call interface
template <typename NarrowPhaseSolver>
struct DistanceFunctionMatrix
{
  using Scalar = typename NarrowPhaseSolver::Scalar;

  /// @brief the uniform call interface for distance: for distance, we need know
  /// 1. two objects o1 and o2 and their configuration in world coordinate tf1 and tf2;
  /// 2. the solver for narrow phase collision, this is for distance computation between geometric shapes;
  /// 3. the request setting for distance (e.g., whether need to return nearest points);
  using DistanceFunc = Scalar (*)(
      const CollisionGeometry<Scalar>* o1,
      const Transform3<Scalar>& tf1,
      const CollisionGeometry<Scalar>* o2,
      const Transform3<Scalar>& tf2,
      const NarrowPhaseSolver* nsolver,
      const DistanceRequest<Scalar>& request,
      DistanceResult<Scalar>& result);
  
  /// @brief each item in the distance matrix is a function to handle distance between objects of type1 and type2
  DistanceFunc distance_matrix[NODE_COUNT][NODE_COUNT];

  DistanceFunctionMatrix();
};

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
#if FCL_HAVE_OCTOMAP
template <typename T_SH, typename NarrowPhaseSolver>
typename T_SH::Scalar ShapeOcTreeDistance(
    const CollisionGeometry<typename T_SH::Scalar>* o1,
    const Transform3<typename T_SH::Scalar>& tf1,
    const CollisionGeometry<typename T_SH::Scalar>* o2,
    const Transform3<typename T_SH::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename T_SH::Scalar>& request,
    DistanceResult<typename T_SH::Scalar>& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  ShapeOcTreeDistanceTraversalNode<T_SH, NarrowPhaseSolver> node;
  const T_SH* obj1 = static_cast<const T_SH*>(o1);
  const OcTree* obj2 = static_cast<const OcTree*>(o2);
  OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
  distance(&node);

  return result.min_distance;
}

template <typename T_SH, typename NarrowPhaseSolver>
typename T_SH::Scalar OcTreeShapeDistance(
    const CollisionGeometry<typename T_SH::Scalar>* o1,
    const Transform3<typename T_SH::Scalar>& tf1,
    const CollisionGeometry<typename T_SH::Scalar>* o2,
    const Transform3<typename T_SH::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename T_SH::Scalar>& request,
    DistanceResult<typename T_SH::Scalar>& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  OcTreeShapeDistanceTraversalNode<T_SH, NarrowPhaseSolver> node;
  const OcTree* obj1 = static_cast<const OcTree*>(o1);
  const T_SH* obj2 = static_cast<const T_SH*>(o2);
  OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
  distance(&node);

  return result.min_distance;
}

template <typename NarrowPhaseSolver>
typename NarrowPhaseSolver::Scalar OcTreeDistance(
    const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename NarrowPhaseSolver::Scalar>& request,
    DistanceResult<typename NarrowPhaseSolver::Scalar>& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  OcTreeDistanceTraversalNode<NarrowPhaseSolver> node;
  const OcTree* obj1 = static_cast<const OcTree*>(o1);
  const OcTree* obj2 = static_cast<const OcTree*>(o2);
  OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
  distance(&node);

  return result.min_distance;
}

template <typename T_BVH, typename NarrowPhaseSolver>
typename T_BVH::Scalar BVHOcTreeDistance(
    const CollisionGeometry<typename T_BVH::Scalar>* o1,
    const Transform3<typename T_BVH::Scalar>& tf1,
    const CollisionGeometry<typename T_BVH::Scalar>* o2,
    const Transform3<typename T_BVH::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename T_BVH::Scalar>& request,
    DistanceResult<typename T_BVH::Scalar>& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  MeshOcTreeDistanceTraversalNode<T_BVH, NarrowPhaseSolver> node;
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>*>(o1);
  const OcTree* obj2 = static_cast<const OcTree*>(o2);
  OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
  distance(&node);

  return result.min_distance;
}

template <typename T_BVH, typename NarrowPhaseSolver>
typename T_BVH::Scalar OcTreeBVHDistance(
    const CollisionGeometry<typename T_BVH::Scalar>* o1,
    const Transform3<typename T_BVH::Scalar>& tf1,
    const CollisionGeometry<typename T_BVH::Scalar>* o2,
    const Transform3<typename T_BVH::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename T_BVH::Scalar>& request,
    DistanceResult<typename T_BVH::Scalar>& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  OcTreeMeshDistanceTraversalNode<T_BVH, NarrowPhaseSolver> node;
  const OcTree* obj1 = static_cast<const OcTree*>(o1);
  const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>*>(o2);
  OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
  distance(&node);

  return result.min_distance;
}

#endif

template <typename T_SH1, typename T_SH2, typename NarrowPhaseSolver>
typename T_SH1::Scalar ShapeShapeDistance(
    const CollisionGeometry<typename T_SH1::Scalar>* o1,
    const Transform3<typename T_SH1::Scalar>& tf1,
    const CollisionGeometry<typename T_SH1::Scalar>* o2,
    const Transform3<typename T_SH1::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename T_SH1::Scalar>& request,
    DistanceResult<typename T_SH1::Scalar>& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  ShapeDistanceTraversalNode<T_SH1, T_SH2, NarrowPhaseSolver> node;
  const T_SH1* obj1 = static_cast<const T_SH1*>(o1);
  const T_SH2* obj2 = static_cast<const T_SH2*>(o2);

  initialize(node, *obj1, tf1, *obj2, tf2, nsolver, request, result);
  distance(&node);

  return result.min_distance;
}

template <typename T_BVH, typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeDistancer
{
  using Scalar = typename T_BVH::Scalar;

  static Scalar distance(
      const CollisionGeometry<Scalar>* o1,
      const Transform3<Scalar>& tf1,
      const CollisionGeometry<Scalar>* o2,
      const Transform3<Scalar>& tf2,
      const NarrowPhaseSolver* nsolver,
      const DistanceRequest<Scalar>& request,
      DistanceResult<Scalar>& result)
  {
    if(request.isSatisfied(result)) return result.min_distance;
    MeshShapeDistanceTraversalNode<T_BVH, T_SH, NarrowPhaseSolver> node;
    const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
    BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
    Transform3<Scalar> tf1_tmp = tf1;
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    initialize(node, *obj1_tmp, tf1_tmp, *obj2, tf2, nsolver, request, result);
    ::fcl::distance(&node);

    delete obj1_tmp;
    return result.min_distance;
  }
};

namespace details
{

template <typename OrientedMeshShapeDistanceTraversalNode,
          typename T_BVH, typename T_SH, typename NarrowPhaseSolver>
typename T_SH::Scalar orientedBVHShapeDistance(
    const CollisionGeometry<typename T_SH::Scalar>* o1,
    const Transform3<typename T_SH::Scalar>& tf1,
    const CollisionGeometry<typename T_SH::Scalar>* o2,
    const Transform3<typename T_SH::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename T_SH::Scalar>&
    request, DistanceResult<typename T_SH::Scalar>& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  OrientedMeshShapeDistanceTraversalNode node;
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
  const T_SH* obj2 = static_cast<const T_SH*>(o2);

  initialize(node, *obj1, tf1, *obj2, tf2, nsolver, request, result);
  distance(&node);

  return result.min_distance;
}

} // namespace details

template <typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeDistancer<RSS<typename T_SH::Scalar>, T_SH, NarrowPhaseSolver>
{
  static typename T_SH::Scalar distance(
      const CollisionGeometry<typename T_SH::Scalar>* o1,
      const Transform3<typename T_SH::Scalar>& tf1,
      const CollisionGeometry<typename T_SH::Scalar>* o2,
      const Transform3<typename T_SH::Scalar>& tf2,
      const NarrowPhaseSolver* nsolver,
      const DistanceRequest<typename T_SH::Scalar>& request,
      DistanceResult<typename T_SH::Scalar>& result)
  {
    return details::orientedBVHShapeDistance<
        MeshShapeDistanceTraversalNodeRSS<T_SH, NarrowPhaseSolver>,
        RSS<typename T_SH::Scalar>,
        T_SH,
        NarrowPhaseSolver>(
          o1, tf1, o2, tf2, nsolver, request, result);
  }
};

template <typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeDistancer<kIOS<typename T_SH::ScalarScalar>, T_SH, NarrowPhaseSolver>
{
  static typename T_SH::ScalarScalar distance(
      const CollisionGeometry<typename T_SH::Scalar>* o1,
      const Transform3<typename T_SH::Scalar>& tf1,
      const CollisionGeometry<typename T_SH::Scalar>* o2,
      const Transform3<typename T_SH::Scalar>& tf2,
      const NarrowPhaseSolver* nsolver,
      const DistanceRequest<typename T_SH::Scalar>& request,
      DistanceResult<typename T_SH::Scalar>& result)
  {
    return details::orientedBVHShapeDistance<
        MeshShapeDistanceTraversalNodekIOS<T_SH, NarrowPhaseSolver>,
        kIOS<typename T_SH::Scalar>,
        T_SH,
        NarrowPhaseSolver>(
          o1, tf1, o2, tf2, nsolver, request, result);
  }
};

template <typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeDistancer<OBBRSS<typename T_SH::Scalar>, T_SH, NarrowPhaseSolver>
{
  static typename T_SH::Scalar distance(
      const CollisionGeometry<typename T_SH::Scalar>* o1,
      const Transform3<typename T_SH::Scalar>& tf1,
      const CollisionGeometry<typename T_SH::Scalar>* o2,
      const Transform3<typename T_SH::Scalar>& tf2,
      const NarrowPhaseSolver* nsolver,
      const DistanceRequest<typename T_SH::Scalar>& request,
      DistanceResult<typename T_SH::Scalar>& result)
  {
    return details::orientedBVHShapeDistance<
        MeshShapeDistanceTraversalNodeOBBRSS<T_SH, NarrowPhaseSolver>,
        OBBRSS<typename T_SH::Scalar>,
        T_SH,
        NarrowPhaseSolver>(
          o1, tf1, o2, tf2, nsolver, request, result);
  }
};

//==============================================================================
template <typename Scalar, typename T_BVH>
struct BVHDistanceImpl
{
  Scalar operator()(
      const CollisionGeometry<Scalar>* o1,
      const Transform3<Scalar>& tf1,
      const CollisionGeometry<Scalar>* o2,
      const Transform3<Scalar>& tf2,
      const DistanceRequest<Scalar>& request,
      DistanceResult<Scalar>& result)
  {
    if(request.isSatisfied(result)) return result.min_distance;
    MeshDistanceTraversalNode<T_BVH> node;
    const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
    const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>* >(o2);
    BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
    Transform3<Scalar> tf1_tmp = tf1;
    BVHModel<T_BVH>* obj2_tmp = new BVHModel<T_BVH>(*obj2);
    Transform3<Scalar> tf2_tmp = tf2;

    initialize(node, *obj1_tmp, tf1_tmp, *obj2_tmp, tf2_tmp, request, result);
    distance(&node);
    delete obj1_tmp;
    delete obj2_tmp;

    return result.min_distance;
  }
};

//==============================================================================
template <typename T_BVH>
typename T_BVH::Scalar BVHDistance(
    const CollisionGeometry<typename T_BVH::Scalar>* o1,
    const Transform3<typename T_BVH::Scalar>& tf1,
    const CollisionGeometry<typename T_BVH::Scalar>* o2,
    const Transform3<typename T_BVH::Scalar>& tf2,
    const DistanceRequest<typename T_BVH::Scalar>& request,
    DistanceResult<typename T_BVH::Scalar>& result)
{
  BVHDistanceImpl<typename T_BVH::Scalar, T_BVH> tmp;
  return tmp(o1, tf1, o2, tf2, request, result);
}

namespace details
{

template <typename OrientedMeshDistanceTraversalNode, typename T_BVH>
typename T_BVH::Scalar orientedMeshDistance(
    const CollisionGeometry<typename T_BVH::Scalar>* o1,
    const Transform3<typename T_BVH::Scalar>& tf1,
    const CollisionGeometry<typename T_BVH::Scalar>* o2,
    const Transform3<typename T_BVH::Scalar>& tf2,
    const DistanceRequest<typename T_BVH::Scalar>& request,
    DistanceResult<typename T_BVH::Scalar>& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  OrientedMeshDistanceTraversalNode node;
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
  const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>* >(o2);

  initialize(node, *obj1, tf1, *obj2, tf2, request, result);
  distance(&node);

  return result.min_distance;
}

} // namespace details

//==============================================================================
template <typename Scalar>
struct BVHDistanceImpl<Scalar, RSS<Scalar>>
{
  Scalar operator()(
      const CollisionGeometry<Scalar>* o1,
      const Transform3<Scalar>& tf1,
      const CollisionGeometry<Scalar>* o2,
      const Transform3<Scalar>& tf2,
      const DistanceRequest<Scalar>& request,
      DistanceResult<Scalar>& result)
  {
    return details::orientedMeshDistance<
        MeshDistanceTraversalNodeRSS<Scalar>, RSS<Scalar>>(
            o1, tf1, o2, tf2, request, result);
  }
};

//==============================================================================
template <typename Scalar>
struct BVHDistanceImpl<Scalar, kIOS<Scalar>>
{
  Scalar operator()(
      const CollisionGeometry<Scalar>* o1,
      const Transform3<Scalar>& tf1,
      const CollisionGeometry<Scalar>* o2,
      const Transform3<Scalar>& tf2,
      const DistanceRequest<Scalar>& request,
      DistanceResult<Scalar>& result)
  {
    return details::orientedMeshDistance<
        MeshDistanceTraversalNodekIOS<Scalar>, kIOS<Scalar>>(
            o1, tf1, o2, tf2, request, result);
  }
};

//==============================================================================
template <typename Scalar>
struct BVHDistanceImpl<Scalar, OBBRSS<Scalar>>
{
  Scalar operator()(
      const CollisionGeometry<Scalar>* o1,
      const Transform3<Scalar>& tf1,
      const CollisionGeometry<Scalar>* o2,
      const Transform3<Scalar>& tf2,
      const DistanceRequest<Scalar>& request,
      DistanceResult<Scalar>& result)
  {
    return details::orientedMeshDistance<
        MeshDistanceTraversalNodeOBBRSS<Scalar>, OBBRSS<Scalar>>(
            o1, tf1, o2, tf2, request, result);
  }
};

//==============================================================================
template <typename T_BVH, typename NarrowPhaseSolver>
typename T_BVH::Scalar BVHDistance(
    const CollisionGeometry<typename T_BVH::Scalar>* o1,
    const Transform3<typename T_BVH::Scalar>& tf1,
    const CollisionGeometry<typename T_BVH::Scalar>* o2,
    const Transform3<typename T_BVH::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename T_BVH::Scalar>& request,
    DistanceResult<typename T_BVH::Scalar>& result)
{
  return BVHDistance<T_BVH>(o1, tf1, o2, tf2, request, result);
}

template <typename NarrowPhaseSolver>
DistanceFunctionMatrix<NarrowPhaseSolver>::DistanceFunctionMatrix()
{
  for(int i = 0; i < NODE_COUNT; ++i)
  {
    for(int j = 0; j < NODE_COUNT; ++j)
      distance_matrix[i][j] = NULL;
  }

  distance_matrix[GEOM_BOX][GEOM_BOX] = &ShapeShapeDistance<Box<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_SPHERE] = &ShapeShapeDistance<Box<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_ELLIPSOID] = &ShapeShapeDistance<Box<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_CAPSULE] = &ShapeShapeDistance<Box<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_CONE] = &ShapeShapeDistance<Box<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_CYLINDER] = &ShapeShapeDistance<Box<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_CONVEX] = &ShapeShapeDistance<Box<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_PLANE] = &ShapeShapeDistance<Box<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_HALFSPACE] = &ShapeShapeDistance<Box<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  distance_matrix[GEOM_SPHERE][GEOM_BOX] = &ShapeShapeDistance<Sphere<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_SPHERE] = &ShapeShapeDistance<Sphere<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_ELLIPSOID] = &ShapeShapeDistance<Sphere<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_CAPSULE] = &ShapeShapeDistance<Sphere<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_CONE] = &ShapeShapeDistance<Sphere<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_CYLINDER] = &ShapeShapeDistance<Sphere<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_CONVEX] = &ShapeShapeDistance<Sphere<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_PLANE] = &ShapeShapeDistance<Sphere<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_HALFSPACE] = &ShapeShapeDistance<Sphere<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  distance_matrix[GEOM_ELLIPSOID][GEOM_BOX] = &ShapeShapeDistance<Ellipsoid<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_SPHERE] = &ShapeShapeDistance<Ellipsoid<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_ELLIPSOID] = &ShapeShapeDistance<Ellipsoid<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_CAPSULE] = &ShapeShapeDistance<Ellipsoid<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_CONE] = &ShapeShapeDistance<Ellipsoid<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_CYLINDER] = &ShapeShapeDistance<Ellipsoid<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_CONVEX] = &ShapeShapeDistance<Ellipsoid<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_PLANE] = &ShapeShapeDistance<Ellipsoid<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_HALFSPACE] = &ShapeShapeDistance<Ellipsoid<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  distance_matrix[GEOM_CAPSULE][GEOM_BOX] = &ShapeShapeDistance<Capsule<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_SPHERE] = &ShapeShapeDistance<Capsule<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_ELLIPSOID] = &ShapeShapeDistance<Capsule<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_CAPSULE] = &ShapeShapeDistance<Capsule<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_CONE] = &ShapeShapeDistance<Capsule<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_CYLINDER] = &ShapeShapeDistance<Capsule<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_CONVEX] = &ShapeShapeDistance<Capsule<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_PLANE] = &ShapeShapeDistance<Capsule<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_HALFSPACE] = &ShapeShapeDistance<Capsule<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  distance_matrix[GEOM_CONE][GEOM_BOX] = &ShapeShapeDistance<Cone<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_SPHERE] = &ShapeShapeDistance<Cone<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_ELLIPSOID] = &ShapeShapeDistance<Cone<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_CAPSULE] = &ShapeShapeDistance<Cone<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_CONE] = &ShapeShapeDistance<Cone<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_CYLINDER] = &ShapeShapeDistance<Cone<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_CONVEX] = &ShapeShapeDistance<Cone<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_PLANE] = &ShapeShapeDistance<Cone<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_HALFSPACE] = &ShapeShapeDistance<Cone<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  distance_matrix[GEOM_CYLINDER][GEOM_BOX] = &ShapeShapeDistance<Cylinder<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_SPHERE] = &ShapeShapeDistance<Cylinder<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_ELLIPSOID] = &ShapeShapeDistance<Cylinder<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_CAPSULE] = &ShapeShapeDistance<Cylinder<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_CONE] = &ShapeShapeDistance<Cylinder<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_CYLINDER] = &ShapeShapeDistance<Cylinder<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_CONVEX] = &ShapeShapeDistance<Cylinder<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_PLANE] = &ShapeShapeDistance<Cylinder<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_HALFSPACE] = &ShapeShapeDistance<Cylinder<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  distance_matrix[GEOM_CONVEX][GEOM_BOX] = &ShapeShapeDistance<Convex<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_SPHERE] = &ShapeShapeDistance<Convex<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_ELLIPSOID] = &ShapeShapeDistance<Convex<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_CAPSULE] = &ShapeShapeDistance<Convex<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_CONE] = &ShapeShapeDistance<Convex<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_CYLINDER] = &ShapeShapeDistance<Convex<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_CONVEX] = &ShapeShapeDistance<Convex<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_PLANE] = &ShapeShapeDistance<Convex<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_HALFSPACE] = &ShapeShapeDistance<Convex<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  distance_matrix[GEOM_PLANE][GEOM_BOX] = &ShapeShapeDistance<Plane<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_SPHERE] = &ShapeShapeDistance<Plane<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_ELLIPSOID] = &ShapeShapeDistance<Plane<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_CAPSULE] = &ShapeShapeDistance<Plane<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_CONE] = &ShapeShapeDistance<Plane<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_CYLINDER] = &ShapeShapeDistance<Plane<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_CONVEX] = &ShapeShapeDistance<Plane<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_PLANE] = &ShapeShapeDistance<Plane<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_HALFSPACE] = &ShapeShapeDistance<Plane<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  distance_matrix[GEOM_HALFSPACE][GEOM_BOX] = &ShapeShapeDistance<Halfspace<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_SPHERE] = &ShapeShapeDistance<Halfspace<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CAPSULE] = &ShapeShapeDistance<Halfspace<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CONE] = &ShapeShapeDistance<Halfspace<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CYLINDER] = &ShapeShapeDistance<Halfspace<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CONVEX] = &ShapeShapeDistance<Halfspace<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_PLANE] = &ShapeShapeDistance<Halfspace<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_HALFSPACE] = &ShapeShapeDistance<Halfspace<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  /* AABB distance not implemented */
  /*
  distance_matrix[BV_AABB][GEOM_BOX] = &BVHShapeDistancer<AABB<Scalar>, Box<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_SPHERE] = &BVHShapeDistancer<AABB<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_ELLIPSOID] = &BVHShapeDistancer<AABB<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CAPSULE] = &BVHShapeDistancer<AABB<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CONE] = &BVHShapeDistancer<AABB<Scalar>, Cone<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CYLINDER] = &BVHShapeDistancer<AABB<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CONVEX] = &BVHShapeDistancer<AABB<Scalar>, Convex<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_PLANE] = &BVHShapeDistancer<AABB<Scalar>, Plane<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_HALFSPACE] = &BVHShapeDistancer<AABB<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>::distance;

  distance_matrix[BV_OBB][GEOM_BOX] = &BVHShapeDistancer<OBB<Scalar>, Box<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_SPHERE] = &BVHShapeDistancer<OBB<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_ELLIPSOID] = &BVHShapeDistancer<OBB<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_CAPSULE] = &BVHShapeDistancer<OBB<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_CONE] = &BVHShapeDistancer<OBB<Scalar>, Cone<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_CYLINDER] = &BVHShapeDistancer<OBB<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_CONVEX] = &BVHShapeDistancer<OBB<Scalar>, Convex<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_PLANE] = &BVHShapeDistancer<OBB<Scalar>, Plane<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_HALFSPACE] = &BVHShapeDistancer<OBB<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>::distance;
  */

  distance_matrix[BV_RSS][GEOM_BOX] = &BVHShapeDistancer<RSS<Scalar>, Box<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_SPHERE] = &BVHShapeDistancer<RSS<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_ELLIPSOID] = &BVHShapeDistancer<RSS<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CAPSULE] = &BVHShapeDistancer<RSS<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CONE] = &BVHShapeDistancer<RSS<Scalar>, Cone<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CYLINDER] = &BVHShapeDistancer<RSS<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CONVEX] = &BVHShapeDistancer<RSS<Scalar>, Convex<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_PLANE] = &BVHShapeDistancer<RSS<Scalar>, Plane<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_HALFSPACE] = &BVHShapeDistancer<RSS<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>::distance;

  /* KDOPd distance not implemented */
  /*
  distance_matrix[BV_KDOP16][GEOM_BOX] = &BVHShapeDistancer<KDOP<Scalar, 16>, Box<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_SPHERE] = &BVHShapeDistancer<KDOP<Scalar, 16>, Sphere<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_ELLIPSOID] = &BVHShapeDistancer<KDOP<Scalar, 16>, Ellipsoid<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CAPSULE] = &BVHShapeDistancer<KDOP<Scalar, 16>, Capsule<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CONE] = &BVHShapeDistancer<KDOP<Scalar, 16>, Cone<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CYLINDER] = &BVHShapeDistancer<KDOP<Scalar, 16>, Cylinder<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CONVEX] = &BVHShapeDistancer<KDOP<Scalar, 16>, Convex<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_PLANE] = &BVHShapeDistancer<KDOP<Scalar, 16>, Plane<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_HALFSPACE] = &BVHShapeDistancer<KDOP<Scalar, 16>, Halfspace<Scalar>, NarrowPhaseSolver>::distance;

  distance_matrix[BV_KDOP18][GEOM_BOX] = &BVHShapeDistancer<KDOP<Scalar, 18>, Box<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_SPHERE] = &BVHShapeDistancer<KDOP<Scalar, 18>, Sphere<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_ELLIPSOID] = &BVHShapeDistancer<KDOP<Scalar, 18>, Ellipsoid<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CAPSULE] = &BVHShapeDistancer<KDOP<Scalar, 18>, Capsule<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CONE] = &BVHShapeDistancer<KDOP<Scalar, 18>, Cone<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CYLINDER] = &BVHShapeDistancer<KDOP<Scalar, 18>, Cylinder<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CONVEX] = &BVHShapeDistancer<KDOP<Scalar, 18>, Convex<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_PLANE] = &BVHShapeDistancer<KDOP<Scalar, 18>, Plane<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_HALFSPACE] = &BVHShapeDistancer<KDOP<Scalar, 18>, Halfspace<Scalar>, NarrowPhaseSolver>::distance;

  distance_matrix[BV_KDOP24][GEOM_BOX] = &BVHShapeDistancer<KDOP<Scalar, 24>, Box<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_SPHERE] = &BVHShapeDistancer<KDOP<Scalar, 24>, Sphere<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_ELLIPSOID] = &BVHShapeDistancer<KDOP<Scalar, 24>, Ellipsoid<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CAPSULE] = &BVHShapeDistancer<KDOP<Scalar, 24>, Capsule<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CONE] = &BVHShapeDistancer<KDOP<Scalar, 24>, Cone<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CYLINDER] = &BVHShapeDistancer<KDOP<Scalar, 24>, Cylinder<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CONVEX] = &BVHShapeDistancer<KDOP<Scalar, 24>, Convex<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_PLANE] = &BVHShapeDistancer<KDOP<Scalar, 24>, Plane<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_HALFSPACE] = &BVHShapeDistancer<KDOP<Scalar, 24>, Halfspace<Scalar>, NarrowPhaseSolver>::distance;
  */

  distance_matrix[BV_kIOS][GEOM_BOX] = &BVHShapeDistancer<kIOS<Scalar>, Box<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_SPHERE] = &BVHShapeDistancer<kIOS<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_ELLIPSOID] = &BVHShapeDistancer<kIOS<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CAPSULE] = &BVHShapeDistancer<kIOS<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CONE] = &BVHShapeDistancer<kIOS<Scalar>, Cone<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CYLINDER] = &BVHShapeDistancer<kIOS<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CONVEX] = &BVHShapeDistancer<kIOS<Scalar>, Convex<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_PLANE] = &BVHShapeDistancer<kIOS<Scalar>, Plane<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_HALFSPACE] = &BVHShapeDistancer<kIOS<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>::distance;

  distance_matrix[BV_OBBRSS][GEOM_BOX] = &BVHShapeDistancer<OBBRSS<Scalar>, Box<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_SPHERE] = &BVHShapeDistancer<OBBRSS<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_ELLIPSOID] = &BVHShapeDistancer<OBBRSS<Scalar>, Ellipsoid<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CAPSULE] = &BVHShapeDistancer<OBBRSS<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CONE] = &BVHShapeDistancer<OBBRSS<Scalar>, Cone<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CYLINDER] = &BVHShapeDistancer<OBBRSS<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CONVEX] = &BVHShapeDistancer<OBBRSS<Scalar>, Convex<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_PLANE] = &BVHShapeDistancer<OBBRSS<Scalar>, Plane<Scalar>, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_HALFSPACE] = &BVHShapeDistancer<OBBRSS<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>::distance;

  distance_matrix[BV_AABB][BV_AABB] = &BVHDistance<AABB<Scalar>, NarrowPhaseSolver>;
  distance_matrix[BV_RSS][BV_RSS] = &BVHDistance<RSS<Scalar>, NarrowPhaseSolver>;
  distance_matrix[BV_kIOS][BV_kIOS] = &BVHDistance<kIOS<Scalar>, NarrowPhaseSolver>;
  distance_matrix[BV_OBBRSS][BV_OBBRSS] = &BVHDistance<OBBRSS<Scalar>, NarrowPhaseSolver>;

#if FCL_HAVE_OCTOMAP
  distance_matrix[GEOM_OCTREE][GEOM_BOX] = &OcTreeShapeDistance<Box<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_SPHERE] = &OcTreeShapeDistance<Sphere<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_ELLIPSOID] = &OcTreeShapeDistance<Ellipsoid<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_CAPSULE] = &OcTreeShapeDistance<Capsule<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_CONE] = &OcTreeShapeDistance<Cone<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_CYLINDER] = &OcTreeShapeDistance<Cylinder<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_CONVEX] = &OcTreeShapeDistance<Convex<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_PLANE] = &OcTreeShapeDistance<Plane<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_HALFSPACE] = &OcTreeShapeDistance<Halfspace<Scalar>, NarrowPhaseSolver>;

  distance_matrix[GEOM_BOX][GEOM_OCTREE] = &ShapeOcTreeDistance<Box<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_OCTREE] = &ShapeOcTreeDistance<Sphere<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_OCTREE] = &ShapeOcTreeDistance<Ellipsoid<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_OCTREE] = &ShapeOcTreeDistance<Capsule<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_OCTREE] = &ShapeOcTreeDistance<Cone<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_OCTREE] = &ShapeOcTreeDistance<Cylinder<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_OCTREE] = &ShapeOcTreeDistance<Convex<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_OCTREE] = &ShapeOcTreeDistance<Plane<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_OCTREE] = &ShapeOcTreeDistance<Halfspace<Scalar>, NarrowPhaseSolver>;

  distance_matrix[GEOM_OCTREE][GEOM_OCTREE] = &OcTreeDistance<NarrowPhaseSolver>;

  distance_matrix[GEOM_OCTREE][BV_AABB] = &OcTreeBVHDistance<AABB<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_OBB] = &OcTreeBVHDistance<OBB<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_RSS] = &OcTreeBVHDistance<RSS<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_OBBRSS] = &OcTreeBVHDistance<OBBRSS<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_kIOS] = &OcTreeBVHDistance<kIOS<Scalar>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_KDOP16] = &OcTreeBVHDistance<KDOP<Scalar, 16>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_KDOP18] = &OcTreeBVHDistance<KDOP<Scalar, 18>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_KDOP24] = &OcTreeBVHDistance<KDOP<Scalar, 24>, NarrowPhaseSolver>;

  distance_matrix[BV_AABB][GEOM_OCTREE] = &BVHOcTreeDistance<AABB<Scalar>, NarrowPhaseSolver>;
  distance_matrix[BV_OBB][GEOM_OCTREE] = &BVHOcTreeDistance<OBB<Scalar>, NarrowPhaseSolver>;
  distance_matrix[BV_RSS][GEOM_OCTREE] = &BVHOcTreeDistance<RSS<Scalar>, NarrowPhaseSolver>;
  distance_matrix[BV_OBBRSS][GEOM_OCTREE] = &BVHOcTreeDistance<OBBRSS<Scalar>, NarrowPhaseSolver>;
  distance_matrix[BV_kIOS][GEOM_OCTREE] = &BVHOcTreeDistance<kIOS<Scalar>, NarrowPhaseSolver>;
  distance_matrix[BV_KDOP16][GEOM_OCTREE] = &BVHOcTreeDistance<KDOP<Scalar, 16>, NarrowPhaseSolver>;
  distance_matrix[BV_KDOP18][GEOM_OCTREE] = &BVHOcTreeDistance<KDOP<Scalar, 18>, NarrowPhaseSolver>;
  distance_matrix[BV_KDOP24][GEOM_OCTREE] = &BVHOcTreeDistance<KDOP<Scalar, 24>, NarrowPhaseSolver>;
#endif

}

} // namespace fcl

#endif

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
#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/narrowphase/narrowphase.h"

namespace fcl
{

/// @brief distance matrix stores the functions for distance between different types of objects and provides a uniform call interface
template <typename Scalar, typename NarrowPhaseSolver>
struct DistanceFunctionMatrix
{
  /// @brief the uniform call interface for distance: for distance, we need know
  /// 1. two objects o1 and o2 and their configuration in world coordinate tf1 and tf2;
  /// 2. the solver for narrow phase collision, this is for distance computation between geometric shapes;
  /// 3. the request setting for distance (e.g., whether need to return nearest points);
  using DistanceFunc = Scalar (*)(
      const CollisionGeometry<Scalar>* o1, const Transform3<Scalar>& tf1,
      const CollisionGeometry<Scalar>* o2, const Transform3<Scalar>& tf2,
      const NarrowPhaseSolver* nsolver,
      const DistanceRequest<Scalar>& request, DistanceResult<Scalar>& result);
  
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
template <typename Scalar, typename T_SH, typename NarrowPhaseSolver>
Scalar ShapeOcTreeDistance(
    const CollisionGeometry<Scalar>* o1, const Transform3<Scalar>& tf1,
    const CollisionGeometry<Scalar>* o2, const Transform3<Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<Scalar>& request, DistanceResult<Scalar>& result)
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

template <typename Scalar, typename T_SH, typename NarrowPhaseSolver>
Scalar OcTreeShapeDistance(const CollisionGeometry<Scalar>* o1, const Transform3<Scalar>& tf1, const CollisionGeometry<Scalar>* o2, const Transform3<Scalar>& tf2, const NarrowPhaseSolver* nsolver,
                             const DistanceRequest<Scalar>& request, DistanceResult<Scalar>& result)
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

template <typename Scalar, typename NarrowPhaseSolver>
Scalar OcTreeDistance(const CollisionGeometry<Scalar>* o1, const Transform3<Scalar>& tf1, const CollisionGeometry<Scalar>* o2, const Transform3<Scalar>& tf2, const NarrowPhaseSolver* nsolver,
                        const DistanceRequest<Scalar>& request, DistanceResult<Scalar>& result)
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

template <typename Scalar, typename T_BVH, typename NarrowPhaseSolver>
Scalar BVHOcTreeDistance(const CollisionGeometry<Scalar>* o1, const Transform3<Scalar>& tf1, const CollisionGeometry<Scalar>* o2, const Transform3<Scalar>& tf2, const NarrowPhaseSolver* nsolver,
                           const DistanceRequest<Scalar>& request, DistanceResult<Scalar>& result)
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

template <typename Scalar, typename T_BVH, typename NarrowPhaseSolver>
Scalar OcTreeBVHDistance(const CollisionGeometry<Scalar>* o1, const Transform3<Scalar>& tf1, const CollisionGeometry<Scalar>* o2, const Transform3<Scalar>& tf2, const NarrowPhaseSolver* nsolver,
                       const DistanceRequest<Scalar>& request, DistanceResult<Scalar>& result)
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

template <typename Scalar, typename T_SH1, typename T_SH2, typename NarrowPhaseSolver>
Scalar ShapeShapeDistance(const CollisionGeometry<Scalar>* o1, const Transform3<Scalar>& tf1, const CollisionGeometry<Scalar>* o2, const Transform3<Scalar>& tf2, const NarrowPhaseSolver* nsolver,
                        const DistanceRequest<Scalar>& request, DistanceResult<Scalar>& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  ShapeDistanceTraversalNode<T_SH1, T_SH2, NarrowPhaseSolver> node;
  const T_SH1* obj1 = static_cast<const T_SH1*>(o1);
  const T_SH2* obj2 = static_cast<const T_SH2*>(o2);

  initialize(node, *obj1, tf1, *obj2, tf2, nsolver, request, result);
  distance(&node);

  return result.min_distance;
}

template <typename Scalar, typename T_BVH, typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeDistancer
{
  static Scalar distance(const CollisionGeometry<Scalar>* o1, const Transform3<Scalar>& tf1, const CollisionGeometry<Scalar>* o2, const Transform3<Scalar>& tf2, const NarrowPhaseSolver* nsolver,
                           const DistanceRequest<Scalar>& request, DistanceResult<Scalar>& result)
  {
    if(request.isSatisfied(result)) return result.min_distance;
    MeshShapeDistanceTraversalNode<T_BVH, T_SH, NarrowPhaseSolver> node;
    const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
    BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
    Transform3<Scalar> tf1_tmp = tf1;
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    initialize(node, *obj1_tmp, tf1_tmp, *obj2, tf2, nsolver, request, result);
    fcl::distance(&node);

    delete obj1_tmp;
    return result.min_distance;
  }
};

namespace details
{

template <typename Scalar, typename OrientedMeshShapeDistanceTraversalNode, typename T_BVH, typename T_SH, typename NarrowPhaseSolver>
Scalar orientedBVHShapeDistance(const CollisionGeometry<Scalar>* o1, const Transform3<Scalar>& tf1, const CollisionGeometry<Scalar>* o2, const Transform3<Scalar>& tf2, const NarrowPhaseSolver* nsolver,
                                  const DistanceRequest<Scalar>& request, DistanceResult<Scalar>& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  OrientedMeshShapeDistanceTraversalNode node;
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
  const T_SH* obj2 = static_cast<const T_SH*>(o2);

  initialize(node, *obj1, tf1, *obj2, tf2, nsolver, request, result);
  fcl::distance(&node);

  return result.min_distance;
}

}

template <typename Scalar, typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeDistancer<RSSd, T_SH, NarrowPhaseSolver>
{
  static Scalar distance(const CollisionGeometry<Scalar>* o1, const Transform3<Scalar>& tf1, const CollisionGeometry<Scalar>* o2, const Transform3<Scalar>& tf2, const NarrowPhaseSolver* nsolver,
                           const DistanceRequest<Scalar>& request, DistanceResult<Scalar>& result)
  {
    return details::orientedBVHShapeDistance<MeshShapeDistanceTraversalNodeRSS<T_SH, NarrowPhaseSolver>, RSSd, T_SH, NarrowPhaseSolver>(o1, tf1, o2, tf2, nsolver, request, result);
  }
};


template <typename Scalar, typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeDistancer<kIOSd, T_SH, NarrowPhaseSolver>
{
  static Scalar distance(const CollisionGeometry<Scalar>* o1, const Transform3<Scalar>& tf1, const CollisionGeometry<Scalar>* o2, const Transform3<Scalar>& tf2, const NarrowPhaseSolver* nsolver,
                       const DistanceRequest<Scalar>& request, DistanceResult<Scalar>& result)
  {
    return details::orientedBVHShapeDistance<MeshShapeDistanceTraversalNodekIOS<T_SH, NarrowPhaseSolver>, kIOSd, T_SH, NarrowPhaseSolver>(o1, tf1, o2, tf2, nsolver, request, result);
  }
};

template <typename Scalar, typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeDistancer<OBBRSSd, T_SH, NarrowPhaseSolver>
{
  static Scalar distance(const CollisionGeometry<Scalar>* o1, const Transform3<Scalar>& tf1, const CollisionGeometry<Scalar>* o2, const Transform3<Scalar>& tf2, const NarrowPhaseSolver* nsolver,
                           const DistanceRequest<Scalar>& request, DistanceResult<Scalar>& result)
  {
    return details::orientedBVHShapeDistance<MeshShapeDistanceTraversalNodeOBBRSS<T_SH, NarrowPhaseSolver>, OBBRSSd, T_SH, NarrowPhaseSolver>(o1, tf1, o2, tf2, nsolver, request, result);
  }
};


template <typename Scalar, typename T_BVH>
Scalar BVHDistance(const CollisionGeometry<Scalar>* o1, const Transform3<Scalar>& tf1, const CollisionGeometry<Scalar>* o2, const Transform3<Scalar>& tf2,
                     const DistanceRequest<Scalar>& request, DistanceResult<Scalar>& result)
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

namespace details
{
template <typename Scalar, typename OrientedMeshDistanceTraversalNode, typename T_BVH>
Scalar orientedMeshDistance(const CollisionGeometry<Scalar>* o1, const Transform3<Scalar>& tf1, const CollisionGeometry<Scalar>* o2, const Transform3<Scalar>& tf2,
                              const DistanceRequest<Scalar>& request, DistanceResult<Scalar>& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  OrientedMeshDistanceTraversalNode node;
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
  const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>* >(o2);

  initialize(node, *obj1, tf1, *obj2, tf2, request, result);
  distance(&node);

  return result.min_distance;
}

}

template<>
Scalar BVHDistance<RSSd>(const CollisionGeometry<Scalar>* o1, const Transform3<Scalar>& tf1, const CollisionGeometry<Scalar>* o2, const Transform3<Scalar>& tf2,
                          const DistanceRequest<Scalar>& request, DistanceResult<Scalar>& result)
{
  return details::orientedMeshDistance<MeshDistanceTraversalNodeRSS, RSSd>(o1, tf1, o2, tf2, request, result);
}

template<>
Scalar BVHDistance<kIOSd>(const CollisionGeometry<Scalar>* o1, const Transform3<Scalar>& tf1, const CollisionGeometry<Scalar>* o2, const Transform3<Scalar>& tf2,
                           const DistanceRequest<Scalar>& request, DistanceResult<Scalar>& result)
{
  return details::orientedMeshDistance<MeshDistanceTraversalNodekIOS, kIOSd>(o1, tf1, o2, tf2, request, result);
}


template<>
Scalar BVHDistance<OBBRSSd>(const CollisionGeometry<Scalar>* o1, const Transform3<Scalar>& tf1, const CollisionGeometry<Scalar>* o2, const Transform3<Scalar>& tf2,
                             const DistanceRequest<Scalar>& request, DistanceResult<Scalar>& result)
{
  return details::orientedMeshDistance<MeshDistanceTraversalNodeOBBRSS, OBBRSSd>(o1, tf1, o2, tf2, request, result);
}


template<typename T_BVH, typename NarrowPhaseSolver>
Scalar BVHDistance(const CollisionGeometry<Scalar>* o1, const Transform3<Scalar>& tf1, const CollisionGeometry<Scalar>* o2, const Transform3<Scalar>& tf2,
                     const NarrowPhaseSolver* nsolver,
                     const DistanceRequest<Scalar>& request, DistanceResult<Scalar>& result)
{
  return BVHDistance<T_BVH>(o1, tf1, o2, tf2, request, result);
}

template<typename NarrowPhaseSolver>
DistanceFunctionMatrix<NarrowPhaseSolver>::DistanceFunctionMatrix()
{
  for(int i = 0; i < NODE_COUNT; ++i)
  {
    for(int j = 0; j < NODE_COUNT; ++j)
      distance_matrix[i][j] = NULL;
  }

  distance_matrix[GEOM_BOX][GEOM_BOX] = &ShapeShapeDistance<Boxd, Boxd, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_SPHERE] = &ShapeShapeDistance<Boxd, Sphered, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_ELLIPSOID] = &ShapeShapeDistance<Boxd, Ellipsoidd, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_CAPSULE] = &ShapeShapeDistance<Boxd, Capsuled, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_CONE] = &ShapeShapeDistance<Boxd, Coned, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_CYLINDER] = &ShapeShapeDistance<Boxd, Cylinderd, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_CONVEX] = &ShapeShapeDistance<Boxd, Convexd, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_PLANE] = &ShapeShapeDistance<Boxd, Planed, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_HALFSPACE] = &ShapeShapeDistance<Boxd, Halfspaced, NarrowPhaseSolver>;

  distance_matrix[GEOM_SPHERE][GEOM_BOX] = &ShapeShapeDistance<Sphered, Boxd, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_SPHERE] = &ShapeShapeDistance<Sphered, Sphered, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_ELLIPSOID] = &ShapeShapeDistance<Sphered, Ellipsoidd, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_CAPSULE] = &ShapeShapeDistance<Sphered, Capsuled, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_CONE] = &ShapeShapeDistance<Sphered, Coned, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_CYLINDER] = &ShapeShapeDistance<Sphered, Cylinderd, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_CONVEX] = &ShapeShapeDistance<Sphered, Convexd, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_PLANE] = &ShapeShapeDistance<Sphered, Planed, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_HALFSPACE] = &ShapeShapeDistance<Sphered, Halfspaced, NarrowPhaseSolver>;

  distance_matrix[GEOM_ELLIPSOID][GEOM_BOX] = &ShapeShapeDistance<Ellipsoidd, Boxd, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_SPHERE] = &ShapeShapeDistance<Ellipsoidd, Sphered, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_ELLIPSOID] = &ShapeShapeDistance<Ellipsoidd, Ellipsoidd, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_CAPSULE] = &ShapeShapeDistance<Ellipsoidd, Capsuled, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_CONE] = &ShapeShapeDistance<Ellipsoidd, Coned, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_CYLINDER] = &ShapeShapeDistance<Ellipsoidd, Cylinderd, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_CONVEX] = &ShapeShapeDistance<Ellipsoidd, Convexd, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_PLANE] = &ShapeShapeDistance<Ellipsoidd, Planed, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_HALFSPACE] = &ShapeShapeDistance<Ellipsoidd, Halfspaced, NarrowPhaseSolver>;

  distance_matrix[GEOM_CAPSULE][GEOM_BOX] = &ShapeShapeDistance<Capsuled, Boxd, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_SPHERE] = &ShapeShapeDistance<Capsuled, Sphered, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_ELLIPSOID] = &ShapeShapeDistance<Capsuled, Ellipsoidd, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_CAPSULE] = &ShapeShapeDistance<Capsuled, Capsuled, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_CONE] = &ShapeShapeDistance<Capsuled, Coned, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_CYLINDER] = &ShapeShapeDistance<Capsuled, Cylinderd, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_CONVEX] = &ShapeShapeDistance<Capsuled, Convexd, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_PLANE] = &ShapeShapeDistance<Capsuled, Planed, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_HALFSPACE] = &ShapeShapeDistance<Capsuled, Halfspaced, NarrowPhaseSolver>;

  distance_matrix[GEOM_CONE][GEOM_BOX] = &ShapeShapeDistance<Coned, Boxd, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_SPHERE] = &ShapeShapeDistance<Coned, Sphered, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_ELLIPSOID] = &ShapeShapeDistance<Coned, Ellipsoidd, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_CAPSULE] = &ShapeShapeDistance<Coned, Capsuled, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_CONE] = &ShapeShapeDistance<Coned, Coned, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_CYLINDER] = &ShapeShapeDistance<Coned, Cylinderd, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_CONVEX] = &ShapeShapeDistance<Coned, Convexd, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_PLANE] = &ShapeShapeDistance<Coned, Planed, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_HALFSPACE] = &ShapeShapeDistance<Coned, Halfspaced, NarrowPhaseSolver>;

  distance_matrix[GEOM_CYLINDER][GEOM_BOX] = &ShapeShapeDistance<Cylinderd, Boxd, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_SPHERE] = &ShapeShapeDistance<Cylinderd, Sphered, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_ELLIPSOID] = &ShapeShapeDistance<Cylinderd, Ellipsoidd, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_CAPSULE] = &ShapeShapeDistance<Cylinderd, Capsuled, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_CONE] = &ShapeShapeDistance<Cylinderd, Coned, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_CYLINDER] = &ShapeShapeDistance<Cylinderd, Cylinderd, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_CONVEX] = &ShapeShapeDistance<Cylinderd, Convexd, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_PLANE] = &ShapeShapeDistance<Cylinderd, Planed, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_HALFSPACE] = &ShapeShapeDistance<Cylinderd, Halfspaced, NarrowPhaseSolver>;

  distance_matrix[GEOM_CONVEX][GEOM_BOX] = &ShapeShapeDistance<Convexd, Boxd, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_SPHERE] = &ShapeShapeDistance<Convexd, Sphered, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_ELLIPSOID] = &ShapeShapeDistance<Convexd, Ellipsoidd, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_CAPSULE] = &ShapeShapeDistance<Convexd, Capsuled, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_CONE] = &ShapeShapeDistance<Convexd, Coned, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_CYLINDER] = &ShapeShapeDistance<Convexd, Cylinderd, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_CONVEX] = &ShapeShapeDistance<Convexd, Convexd, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_PLANE] = &ShapeShapeDistance<Convexd, Planed, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_HALFSPACE] = &ShapeShapeDistance<Convexd, Halfspaced, NarrowPhaseSolver>;

  distance_matrix[GEOM_PLANE][GEOM_BOX] = &ShapeShapeDistance<Planed, Boxd, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_SPHERE] = &ShapeShapeDistance<Planed, Sphered, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_ELLIPSOID] = &ShapeShapeDistance<Planed, Ellipsoidd, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_CAPSULE] = &ShapeShapeDistance<Planed, Capsuled, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_CONE] = &ShapeShapeDistance<Planed, Coned, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_CYLINDER] = &ShapeShapeDistance<Planed, Cylinderd, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_CONVEX] = &ShapeShapeDistance<Planed, Convexd, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_PLANE] = &ShapeShapeDistance<Planed, Planed, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_HALFSPACE] = &ShapeShapeDistance<Planed, Halfspaced, NarrowPhaseSolver>;

  distance_matrix[GEOM_HALFSPACE][GEOM_BOX] = &ShapeShapeDistance<Halfspaced, Boxd, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_SPHERE] = &ShapeShapeDistance<Halfspaced, Sphered, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CAPSULE] = &ShapeShapeDistance<Halfspaced, Capsuled, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CONE] = &ShapeShapeDistance<Halfspaced, Coned, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CYLINDER] = &ShapeShapeDistance<Halfspaced, Cylinderd, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CONVEX] = &ShapeShapeDistance<Halfspaced, Convexd, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_PLANE] = &ShapeShapeDistance<Halfspaced, Planed, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_HALFSPACE] = &ShapeShapeDistance<Halfspaced, Halfspaced, NarrowPhaseSolver>;

  /* AABBd distance not implemented */
  /*
  distance_matrix[BV_AABB][GEOM_BOX] = &BVHShapeDistancer<AABBd, Boxd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_SPHERE] = &BVHShapeDistancer<AABBd, Sphered, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_ELLIPSOID] = &BVHShapeDistancer<AABBd, Ellipsoidd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CAPSULE] = &BVHShapeDistancer<AABBd, Capsuled, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CONE] = &BVHShapeDistancer<AABBd, Coned, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CYLINDER] = &BVHShapeDistancer<AABBd, Cylinderd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CONVEX] = &BVHShapeDistancer<AABBd, Convexd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_PLANE] = &BVHShapeDistancer<AABBd, Planed, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_HALFSPACE] = &BVHShapeDistancer<AABBd, Halfspaced, NarrowPhaseSolver>::distance;

  distance_matrix[BV_OBB][GEOM_BOX] = &BVHShapeDistancer<OBBd, Boxd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_SPHERE] = &BVHShapeDistancer<OBBd, Sphered, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_ELLIPSOID] = &BVHShapeDistancer<OBBd, Ellipsoidd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_CAPSULE] = &BVHShapeDistancer<OBBd, Capsuled, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_CONE] = &BVHShapeDistancer<OBBd, Coned, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_CYLINDER] = &BVHShapeDistancer<OBBd, Cylinderd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_CONVEX] = &BVHShapeDistancer<OBBd, Convexd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_PLANE] = &BVHShapeDistancer<OBBd, Planed, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_HALFSPACE] = &BVHShapeDistancer<OBBd, Halfspaced, NarrowPhaseSolver>::distance;
  */

  distance_matrix[BV_RSS][GEOM_BOX] = &BVHShapeDistancer<RSSd, Boxd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_SPHERE] = &BVHShapeDistancer<RSSd, Sphered, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_ELLIPSOID] = &BVHShapeDistancer<RSSd, Ellipsoidd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CAPSULE] = &BVHShapeDistancer<RSSd, Capsuled, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CONE] = &BVHShapeDistancer<RSSd, Coned, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CYLINDER] = &BVHShapeDistancer<RSSd, Cylinderd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CONVEX] = &BVHShapeDistancer<RSSd, Convexd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_PLANE] = &BVHShapeDistancer<RSSd, Planed, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_HALFSPACE] = &BVHShapeDistancer<RSSd, Halfspaced, NarrowPhaseSolver>::distance;

  /* KDOPd distance not implemented */
  /*
  distance_matrix[BV_KDOP16][GEOM_BOX] = &BVHShapeDistancer<KDOPd<16>, Boxd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_SPHERE] = &BVHShapeDistancer<KDOPd<16>, Sphered, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_ELLIPSOID] = &BVHShapeDistancer<KDOPd<16>, Ellipsoidd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CAPSULE] = &BVHShapeDistancer<KDOPd<16>, Capsuled, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CONE] = &BVHShapeDistancer<KDOPd<16>, Coned, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CYLINDER] = &BVHShapeDistancer<KDOPd<16>, Cylinderd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CONVEX] = &BVHShapeDistancer<KDOPd<16>, Convexd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_PLANE] = &BVHShapeDistancer<KDOPd<16>, Planed, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_HALFSPACE] = &BVHShapeDistancer<KDOPd<16>, Halfspaced, NarrowPhaseSolver>::distance;

  distance_matrix[BV_KDOP18][GEOM_BOX] = &BVHShapeDistancer<KDOPd<18>, Boxd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_SPHERE] = &BVHShapeDistancer<KDOPd<18>, Sphered, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_ELLIPSOID] = &BVHShapeDistancer<KDOPd<18>, Ellipsoidd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CAPSULE] = &BVHShapeDistancer<KDOPd<18>, Capsuled, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CONE] = &BVHShapeDistancer<KDOPd<18>, Coned, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CYLINDER] = &BVHShapeDistancer<KDOPd<18>, Cylinderd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CONVEX] = &BVHShapeDistancer<KDOPd<18>, Convexd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_PLANE] = &BVHShapeDistancer<KDOPd<18>, Planed, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_HALFSPACE] = &BVHShapeDistancer<KDOPd<18>, Halfspaced, NarrowPhaseSolver>::distance;

  distance_matrix[BV_KDOP24][GEOM_BOX] = &BVHShapeDistancer<KDOPd<24>, Boxd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_SPHERE] = &BVHShapeDistancer<KDOPd<24>, Sphered, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_ELLIPSOID] = &BVHShapeDistancer<KDOPd<24>, Ellipsoidd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CAPSULE] = &BVHShapeDistancer<KDOPd<24>, Capsuled, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CONE] = &BVHShapeDistancer<KDOPd<24>, Coned, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CYLINDER] = &BVHShapeDistancer<KDOPd<24>, Cylinderd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CONVEX] = &BVHShapeDistancer<KDOPd<24>, Convexd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_PLANE] = &BVHShapeDistancer<KDOPd<24>, Planed, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_HALFSPACE] = &BVHShapeDistancer<KDOPd<24>, Halfspaced, NarrowPhaseSolver>::distance;
  */

  distance_matrix[BV_kIOS][GEOM_BOX] = &BVHShapeDistancer<kIOSd, Boxd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_SPHERE] = &BVHShapeDistancer<kIOSd, Sphered, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_ELLIPSOID] = &BVHShapeDistancer<kIOSd, Ellipsoidd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CAPSULE] = &BVHShapeDistancer<kIOSd, Capsuled, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CONE] = &BVHShapeDistancer<kIOSd, Coned, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CYLINDER] = &BVHShapeDistancer<kIOSd, Cylinderd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CONVEX] = &BVHShapeDistancer<kIOSd, Convexd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_PLANE] = &BVHShapeDistancer<kIOSd, Planed, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_HALFSPACE] = &BVHShapeDistancer<kIOSd, Halfspaced, NarrowPhaseSolver>::distance;

  distance_matrix[BV_OBBRSS][GEOM_BOX] = &BVHShapeDistancer<OBBRSSd, Boxd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_SPHERE] = &BVHShapeDistancer<OBBRSSd, Sphered, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_ELLIPSOID] = &BVHShapeDistancer<OBBRSSd, Ellipsoidd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CAPSULE] = &BVHShapeDistancer<OBBRSSd, Capsuled, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CONE] = &BVHShapeDistancer<OBBRSSd, Coned, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CYLINDER] = &BVHShapeDistancer<OBBRSSd, Cylinderd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CONVEX] = &BVHShapeDistancer<OBBRSSd, Convexd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_PLANE] = &BVHShapeDistancer<OBBRSSd, Planed, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_HALFSPACE] = &BVHShapeDistancer<OBBRSSd, Halfspaced, NarrowPhaseSolver>::distance;

  distance_matrix[BV_AABB][BV_AABB] = &BVHDistance<AABBd, NarrowPhaseSolver>;
  distance_matrix[BV_RSS][BV_RSS] = &BVHDistance<RSSd, NarrowPhaseSolver>;
  distance_matrix[BV_kIOS][BV_kIOS] = &BVHDistance<kIOSd, NarrowPhaseSolver>;
  distance_matrix[BV_OBBRSS][BV_OBBRSS] = &BVHDistance<OBBRSSd, NarrowPhaseSolver>;

#if FCL_HAVE_OCTOMAP
  distance_matrix[GEOM_OCTREE][GEOM_BOX] = &OcTreeShapeDistance<Boxd, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_SPHERE] = &OcTreeShapeDistance<Sphered, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_ELLIPSOID] = &OcTreeShapeDistance<Ellipsoidd, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_CAPSULE] = &OcTreeShapeDistance<Capsuled, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_CONE] = &OcTreeShapeDistance<Coned, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_CYLINDER] = &OcTreeShapeDistance<Cylinderd, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_CONVEX] = &OcTreeShapeDistance<Convexd, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_PLANE] = &OcTreeShapeDistance<Planed, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_HALFSPACE] = &OcTreeShapeDistance<Halfspaced, NarrowPhaseSolver>;

  distance_matrix[GEOM_BOX][GEOM_OCTREE] = &ShapeOcTreeDistance<Boxd, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_OCTREE] = &ShapeOcTreeDistance<Sphered, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_OCTREE] = &ShapeOcTreeDistance<Ellipsoidd, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_OCTREE] = &ShapeOcTreeDistance<Capsuled, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_OCTREE] = &ShapeOcTreeDistance<Coned, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_OCTREE] = &ShapeOcTreeDistance<Cylinderd, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_OCTREE] = &ShapeOcTreeDistance<Convexd, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_OCTREE] = &ShapeOcTreeDistance<Planed, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_OCTREE] = &ShapeOcTreeDistance<Halfspaced, NarrowPhaseSolver>;

  distance_matrix[GEOM_OCTREE][GEOM_OCTREE] = &OcTreeDistance<NarrowPhaseSolver>;

  distance_matrix[GEOM_OCTREE][BV_AABB] = &OcTreeBVHDistance<AABBd, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_OBB] = &OcTreeBVHDistance<OBBd, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_RSS] = &OcTreeBVHDistance<RSSd, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_OBBRSS] = &OcTreeBVHDistance<OBBRSSd, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_kIOS] = &OcTreeBVHDistance<kIOSd, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_KDOP16] = &OcTreeBVHDistance<KDOPd<16>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_KDOP18] = &OcTreeBVHDistance<KDOPd<18>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_KDOP24] = &OcTreeBVHDistance<KDOPd<24>, NarrowPhaseSolver>;

  distance_matrix[BV_AABB][GEOM_OCTREE] = &BVHOcTreeDistance<AABBd, NarrowPhaseSolver>;
  distance_matrix[BV_OBB][GEOM_OCTREE] = &BVHOcTreeDistance<OBBd, NarrowPhaseSolver>;
  distance_matrix[BV_RSS][GEOM_OCTREE] = &BVHOcTreeDistance<RSSd, NarrowPhaseSolver>;
  distance_matrix[BV_OBBRSS][GEOM_OCTREE] = &BVHOcTreeDistance<OBBRSSd, NarrowPhaseSolver>;
  distance_matrix[BV_kIOS][GEOM_OCTREE] = &BVHOcTreeDistance<kIOSd, NarrowPhaseSolver>;
  distance_matrix[BV_KDOP16][GEOM_OCTREE] = &BVHOcTreeDistance<KDOPd<16>, NarrowPhaseSolver>;
  distance_matrix[BV_KDOP18][GEOM_OCTREE] = &BVHOcTreeDistance<KDOPd<18>, NarrowPhaseSolver>;
  distance_matrix[BV_KDOP24][GEOM_OCTREE] = &BVHOcTreeDistance<KDOPd<24>, NarrowPhaseSolver>;
#endif


}

template struct DistanceFunctionMatrix<GJKSolver_libccd>;
template struct DistanceFunctionMatrix<GJKSolver_indep>;

} // namespace fcl

#endif

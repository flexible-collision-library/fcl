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

#include "fcl/distance_func_matrix.h"

#include "fcl/collision_node.h"
#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/narrowphase/narrowphase.h"

namespace fcl
{

#if FCL_HAVE_OCTOMAP
template<typename T_SH, typename NarrowPhaseSolver>
FCL_REAL ShapeOcTreeDistance(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2, const NarrowPhaseSolver* nsolver,
                             const DistanceRequest& request, DistanceResult& result)
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

template<typename T_SH, typename NarrowPhaseSolver>
FCL_REAL OcTreeShapeDistance(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2, const NarrowPhaseSolver* nsolver,
                             const DistanceRequest& request, DistanceResult& result)
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

template<typename NarrowPhaseSolver>
FCL_REAL OcTreeDistance(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2, const NarrowPhaseSolver* nsolver,
                        const DistanceRequest& request, DistanceResult& result)
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

template<typename T_BVH, typename NarrowPhaseSolver>
FCL_REAL BVHOcTreeDistance(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2, const NarrowPhaseSolver* nsolver,
                           const DistanceRequest& request, DistanceResult& result)
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

template<typename T_BVH, typename NarrowPhaseSolver>
FCL_REAL OcTreeBVHDistance(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2, const NarrowPhaseSolver* nsolver,
                       const DistanceRequest& request, DistanceResult& result)
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

template<typename T_SH1, typename T_SH2, typename NarrowPhaseSolver>
FCL_REAL ShapeShapeDistance(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2, const NarrowPhaseSolver* nsolver,
                        const DistanceRequest& request, DistanceResult& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  ShapeDistanceTraversalNode<T_SH1, T_SH2, NarrowPhaseSolver> node;
  const T_SH1* obj1 = static_cast<const T_SH1*>(o1);
  const T_SH2* obj2 = static_cast<const T_SH2*>(o2);

  initialize(node, *obj1, tf1, *obj2, tf2, nsolver, request, result);
  distance(&node);

  return result.min_distance;
}

template<typename T_BVH, typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeDistancer
{
  static FCL_REAL distance(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2, const NarrowPhaseSolver* nsolver,
                           const DistanceRequest& request, DistanceResult& result)
  {
    if(request.isSatisfied(result)) return result.min_distance;
    MeshShapeDistanceTraversalNode<T_BVH, T_SH, NarrowPhaseSolver> node;
    const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
    BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
    Transform3d tf1_tmp = tf1;
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    initialize(node, *obj1_tmp, tf1_tmp, *obj2, tf2, nsolver, request, result);
    fcl::distance(&node);
    
    delete obj1_tmp;
    return result.min_distance;
  }
};

namespace details
{

template<typename OrientedMeshShapeDistanceTraversalNode, typename T_BVH, typename T_SH, typename NarrowPhaseSolver>
FCL_REAL orientedBVHShapeDistance(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2, const NarrowPhaseSolver* nsolver,
                                  const DistanceRequest& request, DistanceResult& result)
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

template<typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeDistancer<RSS, T_SH, NarrowPhaseSolver>
{
  static FCL_REAL distance(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2, const NarrowPhaseSolver* nsolver,
                           const DistanceRequest& request, DistanceResult& result)
  {
    return details::orientedBVHShapeDistance<MeshShapeDistanceTraversalNodeRSS<T_SH, NarrowPhaseSolver>, RSS, T_SH, NarrowPhaseSolver>(o1, tf1, o2, tf2, nsolver, request, result);
  }
};


template<typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeDistancer<kIOS, T_SH, NarrowPhaseSolver>
{
  static FCL_REAL distance(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2, const NarrowPhaseSolver* nsolver,
                       const DistanceRequest& request, DistanceResult& result)
  {
    return details::orientedBVHShapeDistance<MeshShapeDistanceTraversalNodekIOS<T_SH, NarrowPhaseSolver>, kIOS, T_SH, NarrowPhaseSolver>(o1, tf1, o2, tf2, nsolver, request, result);
  }
};

template<typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeDistancer<OBBRSS, T_SH, NarrowPhaseSolver>
{
  static FCL_REAL distance(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2, const NarrowPhaseSolver* nsolver,
                           const DistanceRequest& request, DistanceResult& result)
  {
    return details::orientedBVHShapeDistance<MeshShapeDistanceTraversalNodeOBBRSS<T_SH, NarrowPhaseSolver>, OBBRSS, T_SH, NarrowPhaseSolver>(o1, tf1, o2, tf2, nsolver, request, result);
  }
};


template<typename T_BVH>
FCL_REAL BVHDistance(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2,
                     const DistanceRequest& request, DistanceResult& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  MeshDistanceTraversalNode<T_BVH> node;
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
  const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>* >(o2);
  BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
  Transform3d tf1_tmp = tf1;
  BVHModel<T_BVH>* obj2_tmp = new BVHModel<T_BVH>(*obj2);
  Transform3d tf2_tmp = tf2;

  initialize(node, *obj1_tmp, tf1_tmp, *obj2_tmp, tf2_tmp, request, result);
  distance(&node);
  delete obj1_tmp;
  delete obj2_tmp;
  
  return result.min_distance;
}

namespace details
{
template<typename OrientedMeshDistanceTraversalNode, typename T_BVH>
FCL_REAL orientedMeshDistance(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2,
                              const DistanceRequest& request, DistanceResult& result)
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
FCL_REAL BVHDistance<RSS>(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2,
                          const DistanceRequest& request, DistanceResult& result)
{
  return details::orientedMeshDistance<MeshDistanceTraversalNodeRSS, RSS>(o1, tf1, o2, tf2, request, result);
}

template<>
FCL_REAL BVHDistance<kIOS>(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2,
                           const DistanceRequest& request, DistanceResult& result)
{
  return details::orientedMeshDistance<MeshDistanceTraversalNodekIOS, kIOS>(o1, tf1, o2, tf2, request, result);
}


template<>
FCL_REAL BVHDistance<OBBRSS>(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2,
                             const DistanceRequest& request, DistanceResult& result)
{
  return details::orientedMeshDistance<MeshDistanceTraversalNodeOBBRSS, OBBRSS>(o1, tf1, o2, tf2, request, result);
}


template<typename T_BVH, typename NarrowPhaseSolver>
FCL_REAL BVHDistance(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2,
                     const NarrowPhaseSolver* nsolver,
                     const DistanceRequest& request, DistanceResult& result)
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

  /* AABB distance not implemented */
  /*
  distance_matrix[BV_AABB][GEOM_BOX] = &BVHShapeDistancer<AABB, Boxd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_SPHERE] = &BVHShapeDistancer<AABB, Sphered, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_ELLIPSOID] = &BVHShapeDistancer<AABB, Ellipsoidd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CAPSULE] = &BVHShapeDistancer<AABB, Capsuled, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CONE] = &BVHShapeDistancer<AABB, Coned, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CYLINDER] = &BVHShapeDistancer<AABB, Cylinderd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CONVEX] = &BVHShapeDistancer<AABB, Convexd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_PLANE] = &BVHShapeDistancer<AABB, Planed, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_HALFSPACE] = &BVHShapeDistancer<AABB, Halfspaced, NarrowPhaseSolver>::distance;

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

  distance_matrix[BV_RSS][GEOM_BOX] = &BVHShapeDistancer<RSS, Boxd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_SPHERE] = &BVHShapeDistancer<RSS, Sphered, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_ELLIPSOID] = &BVHShapeDistancer<RSS, Ellipsoidd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CAPSULE] = &BVHShapeDistancer<RSS, Capsuled, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CONE] = &BVHShapeDistancer<RSS, Coned, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CYLINDER] = &BVHShapeDistancer<RSS, Cylinderd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CONVEX] = &BVHShapeDistancer<RSS, Convexd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_PLANE] = &BVHShapeDistancer<RSS, Planed, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_HALFSPACE] = &BVHShapeDistancer<RSS, Halfspaced, NarrowPhaseSolver>::distance;

  /* KDOP distance not implemented */
  /*
  distance_matrix[BV_KDOP16][GEOM_BOX] = &BVHShapeDistancer<KDOP<16>, Boxd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_SPHERE] = &BVHShapeDistancer<KDOP<16>, Sphered, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_ELLIPSOID] = &BVHShapeDistancer<KDOP<16>, Ellipsoidd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CAPSULE] = &BVHShapeDistancer<KDOP<16>, Capsuled, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CONE] = &BVHShapeDistancer<KDOP<16>, Coned, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CYLINDER] = &BVHShapeDistancer<KDOP<16>, Cylinderd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CONVEX] = &BVHShapeDistancer<KDOP<16>, Convexd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_PLANE] = &BVHShapeDistancer<KDOP<16>, Planed, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_HALFSPACE] = &BVHShapeDistancer<KDOP<16>, Halfspaced, NarrowPhaseSolver>::distance;

  distance_matrix[BV_KDOP18][GEOM_BOX] = &BVHShapeDistancer<KDOP<18>, Boxd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_SPHERE] = &BVHShapeDistancer<KDOP<18>, Sphered, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_ELLIPSOID] = &BVHShapeDistancer<KDOP<18>, Ellipsoidd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CAPSULE] = &BVHShapeDistancer<KDOP<18>, Capsuled, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CONE] = &BVHShapeDistancer<KDOP<18>, Coned, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CYLINDER] = &BVHShapeDistancer<KDOP<18>, Cylinderd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CONVEX] = &BVHShapeDistancer<KDOP<18>, Convexd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_PLANE] = &BVHShapeDistancer<KDOP<18>, Planed, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_HALFSPACE] = &BVHShapeDistancer<KDOP<18>, Halfspaced, NarrowPhaseSolver>::distance;

  distance_matrix[BV_KDOP24][GEOM_BOX] = &BVHShapeDistancer<KDOP<24>, Boxd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_SPHERE] = &BVHShapeDistancer<KDOP<24>, Sphered, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_ELLIPSOID] = &BVHShapeDistancer<KDOP<24>, Ellipsoidd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CAPSULE] = &BVHShapeDistancer<KDOP<24>, Capsuled, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CONE] = &BVHShapeDistancer<KDOP<24>, Coned, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CYLINDER] = &BVHShapeDistancer<KDOP<24>, Cylinderd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CONVEX] = &BVHShapeDistancer<KDOP<24>, Convexd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_PLANE] = &BVHShapeDistancer<KDOP<24>, Planed, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_HALFSPACE] = &BVHShapeDistancer<KDOP<24>, Halfspaced, NarrowPhaseSolver>::distance;
  */

  distance_matrix[BV_kIOS][GEOM_BOX] = &BVHShapeDistancer<kIOS, Boxd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_SPHERE] = &BVHShapeDistancer<kIOS, Sphered, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_ELLIPSOID] = &BVHShapeDistancer<kIOS, Ellipsoidd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CAPSULE] = &BVHShapeDistancer<kIOS, Capsuled, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CONE] = &BVHShapeDistancer<kIOS, Coned, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CYLINDER] = &BVHShapeDistancer<kIOS, Cylinderd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CONVEX] = &BVHShapeDistancer<kIOS, Convexd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_PLANE] = &BVHShapeDistancer<kIOS, Planed, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_HALFSPACE] = &BVHShapeDistancer<kIOS, Halfspaced, NarrowPhaseSolver>::distance;

  distance_matrix[BV_OBBRSS][GEOM_BOX] = &BVHShapeDistancer<OBBRSS, Boxd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_SPHERE] = &BVHShapeDistancer<OBBRSS, Sphered, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_ELLIPSOID] = &BVHShapeDistancer<OBBRSS, Ellipsoidd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CAPSULE] = &BVHShapeDistancer<OBBRSS, Capsuled, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CONE] = &BVHShapeDistancer<OBBRSS, Coned, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CYLINDER] = &BVHShapeDistancer<OBBRSS, Cylinderd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CONVEX] = &BVHShapeDistancer<OBBRSS, Convexd, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_PLANE] = &BVHShapeDistancer<OBBRSS, Planed, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_HALFSPACE] = &BVHShapeDistancer<OBBRSS, Halfspaced, NarrowPhaseSolver>::distance;

  distance_matrix[BV_AABB][BV_AABB] = &BVHDistance<AABB, NarrowPhaseSolver>;
  distance_matrix[BV_RSS][BV_RSS] = &BVHDistance<RSS, NarrowPhaseSolver>;
  distance_matrix[BV_kIOS][BV_kIOS] = &BVHDistance<kIOS, NarrowPhaseSolver>;
  distance_matrix[BV_OBBRSS][BV_OBBRSS] = &BVHDistance<OBBRSS, NarrowPhaseSolver>;

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

  distance_matrix[GEOM_OCTREE][BV_AABB] = &OcTreeBVHDistance<AABB, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_OBB] = &OcTreeBVHDistance<OBBd, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_RSS] = &OcTreeBVHDistance<RSS, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_OBBRSS] = &OcTreeBVHDistance<OBBRSS, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_kIOS] = &OcTreeBVHDistance<kIOS, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_KDOP16] = &OcTreeBVHDistance<KDOP<16>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_KDOP18] = &OcTreeBVHDistance<KDOP<18>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_KDOP24] = &OcTreeBVHDistance<KDOP<24>, NarrowPhaseSolver>;

  distance_matrix[BV_AABB][GEOM_OCTREE] = &BVHOcTreeDistance<AABB, NarrowPhaseSolver>;
  distance_matrix[BV_OBB][GEOM_OCTREE] = &BVHOcTreeDistance<OBBd, NarrowPhaseSolver>;
  distance_matrix[BV_RSS][GEOM_OCTREE] = &BVHOcTreeDistance<RSS, NarrowPhaseSolver>;
  distance_matrix[BV_OBBRSS][GEOM_OCTREE] = &BVHOcTreeDistance<OBBRSS, NarrowPhaseSolver>;
  distance_matrix[BV_kIOS][GEOM_OCTREE] = &BVHOcTreeDistance<kIOS, NarrowPhaseSolver>;
  distance_matrix[BV_KDOP16][GEOM_OCTREE] = &BVHOcTreeDistance<KDOP<16>, NarrowPhaseSolver>;
  distance_matrix[BV_KDOP18][GEOM_OCTREE] = &BVHOcTreeDistance<KDOP<18>, NarrowPhaseSolver>;
  distance_matrix[BV_KDOP24][GEOM_OCTREE] = &BVHOcTreeDistance<KDOP<24>, NarrowPhaseSolver>;
#endif


}

template struct DistanceFunctionMatrix<GJKSolver_libccd>;
template struct DistanceFunctionMatrix<GJKSolver_indep>;


}

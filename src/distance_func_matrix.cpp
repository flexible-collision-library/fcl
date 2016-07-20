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
FCL_REAL ShapeOcTreeDistance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const NarrowPhaseSolver* nsolver,
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
FCL_REAL OcTreeShapeDistance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const NarrowPhaseSolver* nsolver,
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
FCL_REAL OcTreeDistance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const NarrowPhaseSolver* nsolver,
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
FCL_REAL BVHOcTreeDistance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const NarrowPhaseSolver* nsolver,
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
FCL_REAL OcTreeBVHDistance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const NarrowPhaseSolver* nsolver,
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
FCL_REAL ShapeShapeDistance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const NarrowPhaseSolver* nsolver,
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
  static FCL_REAL distance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const NarrowPhaseSolver* nsolver,
                           const DistanceRequest& request, DistanceResult& result)
  {
    if(request.isSatisfied(result)) return result.min_distance;
    MeshShapeDistanceTraversalNode<T_BVH, T_SH, NarrowPhaseSolver> node;
    const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
    BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
    Transform3f tf1_tmp = tf1;
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
FCL_REAL orientedBVHShapeDistance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const NarrowPhaseSolver* nsolver,
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
  static FCL_REAL distance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const NarrowPhaseSolver* nsolver,
                           const DistanceRequest& request, DistanceResult& result)
  {
    return details::orientedBVHShapeDistance<MeshShapeDistanceTraversalNodeRSS<T_SH, NarrowPhaseSolver>, RSS, T_SH, NarrowPhaseSolver>(o1, tf1, o2, tf2, nsolver, request, result);
  }
};


template<typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeDistancer<kIOS, T_SH, NarrowPhaseSolver>
{
  static FCL_REAL distance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const NarrowPhaseSolver* nsolver,
                       const DistanceRequest& request, DistanceResult& result)
  {
    return details::orientedBVHShapeDistance<MeshShapeDistanceTraversalNodekIOS<T_SH, NarrowPhaseSolver>, kIOS, T_SH, NarrowPhaseSolver>(o1, tf1, o2, tf2, nsolver, request, result);
  }
};

template<typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeDistancer<OBBRSS, T_SH, NarrowPhaseSolver>
{
  static FCL_REAL distance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const NarrowPhaseSolver* nsolver,
                           const DistanceRequest& request, DistanceResult& result)
  {
    return details::orientedBVHShapeDistance<MeshShapeDistanceTraversalNodeOBBRSS<T_SH, NarrowPhaseSolver>, OBBRSS, T_SH, NarrowPhaseSolver>(o1, tf1, o2, tf2, nsolver, request, result);
  }
};


template<typename T_BVH>
FCL_REAL BVHDistance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2,
                     const DistanceRequest& request, DistanceResult& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  MeshDistanceTraversalNode<T_BVH> node;
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
  const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>* >(o2);
  BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
  Transform3f tf1_tmp = tf1;
  BVHModel<T_BVH>* obj2_tmp = new BVHModel<T_BVH>(*obj2);
  Transform3f tf2_tmp = tf2;

  initialize(node, *obj1_tmp, tf1_tmp, *obj2_tmp, tf2_tmp, request, result);
  distance(&node);
  delete obj1_tmp;
  delete obj2_tmp;
  
  return result.min_distance;
}

namespace details
{
template<typename OrientedMeshDistanceTraversalNode, typename T_BVH>
FCL_REAL orientedMeshDistance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2,
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
FCL_REAL BVHDistance<RSS>(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2,
                          const DistanceRequest& request, DistanceResult& result)
{
  return details::orientedMeshDistance<MeshDistanceTraversalNodeRSS, RSS>(o1, tf1, o2, tf2, request, result);
}

template<>
FCL_REAL BVHDistance<kIOS>(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2,
                           const DistanceRequest& request, DistanceResult& result)
{
  return details::orientedMeshDistance<MeshDistanceTraversalNodekIOS, kIOS>(o1, tf1, o2, tf2, request, result);
}


template<>
FCL_REAL BVHDistance<OBBRSS>(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2,
                             const DistanceRequest& request, DistanceResult& result)
{
  return details::orientedMeshDistance<MeshDistanceTraversalNodeOBBRSS, OBBRSS>(o1, tf1, o2, tf2, request, result);
}


template<typename T_BVH, typename NarrowPhaseSolver>
FCL_REAL BVHDistance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2,
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

  distance_matrix[GEOM_BOX][GEOM_BOX] = &ShapeShapeDistance<Box, Box, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_SPHERE] = &ShapeShapeDistance<Box, Sphere, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_ELLIPSOID] = &ShapeShapeDistance<Box, Ellipsoid, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_CAPSULE] = &ShapeShapeDistance<Box, Capsule, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_CONE] = &ShapeShapeDistance<Box, Cone, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_CYLINDER] = &ShapeShapeDistance<Box, Cylinder, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_CONVEX] = &ShapeShapeDistance<Box, Convex, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_PLANE] = &ShapeShapeDistance<Box, Plane, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_HALFSPACE] = &ShapeShapeDistance<Box, Halfspace, NarrowPhaseSolver>;

  distance_matrix[GEOM_SPHERE][GEOM_BOX] = &ShapeShapeDistance<Sphere, Box, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_SPHERE] = &ShapeShapeDistance<Sphere, Sphere, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_ELLIPSOID] = &ShapeShapeDistance<Sphere, Ellipsoid, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_CAPSULE] = &ShapeShapeDistance<Sphere, Capsule, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_CONE] = &ShapeShapeDistance<Sphere, Cone, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_CYLINDER] = &ShapeShapeDistance<Sphere, Cylinder, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_CONVEX] = &ShapeShapeDistance<Sphere, Convex, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_PLANE] = &ShapeShapeDistance<Sphere, Plane, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_HALFSPACE] = &ShapeShapeDistance<Sphere, Halfspace, NarrowPhaseSolver>;

  distance_matrix[GEOM_ELLIPSOID][GEOM_BOX] = &ShapeShapeDistance<Ellipsoid, Box, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_SPHERE] = &ShapeShapeDistance<Ellipsoid, Sphere, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_ELLIPSOID] = &ShapeShapeDistance<Ellipsoid, Ellipsoid, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_CAPSULE] = &ShapeShapeDistance<Ellipsoid, Capsule, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_CONE] = &ShapeShapeDistance<Ellipsoid, Cone, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_CYLINDER] = &ShapeShapeDistance<Ellipsoid, Cylinder, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_CONVEX] = &ShapeShapeDistance<Ellipsoid, Convex, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_PLANE] = &ShapeShapeDistance<Ellipsoid, Plane, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_HALFSPACE] = &ShapeShapeDistance<Ellipsoid, Halfspace, NarrowPhaseSolver>;

  distance_matrix[GEOM_CAPSULE][GEOM_BOX] = &ShapeShapeDistance<Capsule, Box, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_SPHERE] = &ShapeShapeDistance<Capsule, Sphere, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_ELLIPSOID] = &ShapeShapeDistance<Capsule, Ellipsoid, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_CAPSULE] = &ShapeShapeDistance<Capsule, Capsule, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_CONE] = &ShapeShapeDistance<Capsule, Cone, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_CYLINDER] = &ShapeShapeDistance<Capsule, Cylinder, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_CONVEX] = &ShapeShapeDistance<Capsule, Convex, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_PLANE] = &ShapeShapeDistance<Capsule, Plane, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_HALFSPACE] = &ShapeShapeDistance<Capsule, Halfspace, NarrowPhaseSolver>;

  distance_matrix[GEOM_CONE][GEOM_BOX] = &ShapeShapeDistance<Cone, Box, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_SPHERE] = &ShapeShapeDistance<Cone, Sphere, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_ELLIPSOID] = &ShapeShapeDistance<Cone, Ellipsoid, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_CAPSULE] = &ShapeShapeDistance<Cone, Capsule, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_CONE] = &ShapeShapeDistance<Cone, Cone, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_CYLINDER] = &ShapeShapeDistance<Cone, Cylinder, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_CONVEX] = &ShapeShapeDistance<Cone, Convex, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_PLANE] = &ShapeShapeDistance<Cone, Plane, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_HALFSPACE] = &ShapeShapeDistance<Cone, Halfspace, NarrowPhaseSolver>;

  distance_matrix[GEOM_CYLINDER][GEOM_BOX] = &ShapeShapeDistance<Cylinder, Box, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_SPHERE] = &ShapeShapeDistance<Cylinder, Sphere, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_ELLIPSOID] = &ShapeShapeDistance<Cylinder, Ellipsoid, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_CAPSULE] = &ShapeShapeDistance<Cylinder, Capsule, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_CONE] = &ShapeShapeDistance<Cylinder, Cone, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_CYLINDER] = &ShapeShapeDistance<Cylinder, Cylinder, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_CONVEX] = &ShapeShapeDistance<Cylinder, Convex, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_PLANE] = &ShapeShapeDistance<Cylinder, Plane, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_HALFSPACE] = &ShapeShapeDistance<Cylinder, Halfspace, NarrowPhaseSolver>;

  distance_matrix[GEOM_CONVEX][GEOM_BOX] = &ShapeShapeDistance<Convex, Box, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_SPHERE] = &ShapeShapeDistance<Convex, Sphere, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_ELLIPSOID] = &ShapeShapeDistance<Convex, Ellipsoid, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_CAPSULE] = &ShapeShapeDistance<Convex, Capsule, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_CONE] = &ShapeShapeDistance<Convex, Cone, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_CYLINDER] = &ShapeShapeDistance<Convex, Cylinder, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_CONVEX] = &ShapeShapeDistance<Convex, Convex, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_PLANE] = &ShapeShapeDistance<Convex, Plane, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_HALFSPACE] = &ShapeShapeDistance<Convex, Halfspace, NarrowPhaseSolver>;

  distance_matrix[GEOM_PLANE][GEOM_BOX] = &ShapeShapeDistance<Plane, Box, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_SPHERE] = &ShapeShapeDistance<Plane, Sphere, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_ELLIPSOID] = &ShapeShapeDistance<Plane, Ellipsoid, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_CAPSULE] = &ShapeShapeDistance<Plane, Capsule, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_CONE] = &ShapeShapeDistance<Plane, Cone, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_CYLINDER] = &ShapeShapeDistance<Plane, Cylinder, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_CONVEX] = &ShapeShapeDistance<Plane, Convex, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_PLANE] = &ShapeShapeDistance<Plane, Plane, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_HALFSPACE] = &ShapeShapeDistance<Plane, Halfspace, NarrowPhaseSolver>;

  distance_matrix[GEOM_HALFSPACE][GEOM_BOX] = &ShapeShapeDistance<Halfspace, Box, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_SPHERE] = &ShapeShapeDistance<Halfspace, Sphere, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CAPSULE] = &ShapeShapeDistance<Halfspace, Capsule, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CONE] = &ShapeShapeDistance<Halfspace, Cone, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CYLINDER] = &ShapeShapeDistance<Halfspace, Cylinder, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CONVEX] = &ShapeShapeDistance<Halfspace, Convex, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_PLANE] = &ShapeShapeDistance<Halfspace, Plane, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_HALFSPACE] = &ShapeShapeDistance<Halfspace, Halfspace, NarrowPhaseSolver>;

  /* AABB distance not implemented */
  /*
  distance_matrix[BV_AABB][GEOM_BOX] = &BVHShapeDistancer<AABB, Box, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_SPHERE] = &BVHShapeDistancer<AABB, Sphere, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_ELLIPSOID] = &BVHShapeDistancer<AABB, Ellipsoid, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CAPSULE] = &BVHShapeDistancer<AABB, Capsule, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CONE] = &BVHShapeDistancer<AABB, Cone, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CYLINDER] = &BVHShapeDistancer<AABB, Cylinder, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CONVEX] = &BVHShapeDistancer<AABB, Convex, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_PLANE] = &BVHShapeDistancer<AABB, Plane, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_HALFSPACE] = &BVHShapeDistancer<AABB, Halfspace, NarrowPhaseSolver>::distance;

  distance_matrix[BV_OBB][GEOM_BOX] = &BVHShapeDistancer<OBB, Box, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_SPHERE] = &BVHShapeDistancer<OBB, Sphere, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_ELLIPSOID] = &BVHShapeDistancer<OBB, Ellipsoid, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_CAPSULE] = &BVHShapeDistancer<OBB, Capsule, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_CONE] = &BVHShapeDistancer<OBB, Cone, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_CYLINDER] = &BVHShapeDistancer<OBB, Cylinder, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_CONVEX] = &BVHShapeDistancer<OBB, Convex, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_PLANE] = &BVHShapeDistancer<OBB, Plane, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_HALFSPACE] = &BVHShapeDistancer<OBB, Halfspace, NarrowPhaseSolver>::distance;
  */

  distance_matrix[BV_RSS][GEOM_BOX] = &BVHShapeDistancer<RSS, Box, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_SPHERE] = &BVHShapeDistancer<RSS, Sphere, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_ELLIPSOID] = &BVHShapeDistancer<RSS, Ellipsoid, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CAPSULE] = &BVHShapeDistancer<RSS, Capsule, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CONE] = &BVHShapeDistancer<RSS, Cone, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CYLINDER] = &BVHShapeDistancer<RSS, Cylinder, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CONVEX] = &BVHShapeDistancer<RSS, Convex, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_PLANE] = &BVHShapeDistancer<RSS, Plane, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_HALFSPACE] = &BVHShapeDistancer<RSS, Halfspace, NarrowPhaseSolver>::distance;

  /* KDOP distance not implemented */
  /*
  distance_matrix[BV_KDOP16][GEOM_BOX] = &BVHShapeDistancer<KDOP<16>, Box, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_SPHERE] = &BVHShapeDistancer<KDOP<16>, Sphere, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_ELLIPSOID] = &BVHShapeDistancer<KDOP<16>, Ellipsoid, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CAPSULE] = &BVHShapeDistancer<KDOP<16>, Capsule, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CONE] = &BVHShapeDistancer<KDOP<16>, Cone, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CYLINDER] = &BVHShapeDistancer<KDOP<16>, Cylinder, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CONVEX] = &BVHShapeDistancer<KDOP<16>, Convex, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_PLANE] = &BVHShapeDistancer<KDOP<16>, Plane, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_HALFSPACE] = &BVHShapeDistancer<KDOP<16>, Halfspace, NarrowPhaseSolver>::distance;

  distance_matrix[BV_KDOP18][GEOM_BOX] = &BVHShapeDistancer<KDOP<18>, Box, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_SPHERE] = &BVHShapeDistancer<KDOP<18>, Sphere, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_ELLIPSOID] = &BVHShapeDistancer<KDOP<18>, Ellipsoid, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CAPSULE] = &BVHShapeDistancer<KDOP<18>, Capsule, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CONE] = &BVHShapeDistancer<KDOP<18>, Cone, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CYLINDER] = &BVHShapeDistancer<KDOP<18>, Cylinder, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CONVEX] = &BVHShapeDistancer<KDOP<18>, Convex, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_PLANE] = &BVHShapeDistancer<KDOP<18>, Plane, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_HALFSPACE] = &BVHShapeDistancer<KDOP<18>, Halfspace, NarrowPhaseSolver>::distance;

  distance_matrix[BV_KDOP24][GEOM_BOX] = &BVHShapeDistancer<KDOP<24>, Box, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_SPHERE] = &BVHShapeDistancer<KDOP<24>, Sphere, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_ELLIPSOID] = &BVHShapeDistancer<KDOP<24>, Ellipsoid, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CAPSULE] = &BVHShapeDistancer<KDOP<24>, Capsule, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CONE] = &BVHShapeDistancer<KDOP<24>, Cone, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CYLINDER] = &BVHShapeDistancer<KDOP<24>, Cylinder, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CONVEX] = &BVHShapeDistancer<KDOP<24>, Convex, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_PLANE] = &BVHShapeDistancer<KDOP<24>, Plane, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_HALFSPACE] = &BVHShapeDistancer<KDOP<24>, Halfspace, NarrowPhaseSolver>::distance;
  */

  distance_matrix[BV_kIOS][GEOM_BOX] = &BVHShapeDistancer<kIOS, Box, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_SPHERE] = &BVHShapeDistancer<kIOS, Sphere, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_ELLIPSOID] = &BVHShapeDistancer<kIOS, Ellipsoid, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CAPSULE] = &BVHShapeDistancer<kIOS, Capsule, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CONE] = &BVHShapeDistancer<kIOS, Cone, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CYLINDER] = &BVHShapeDistancer<kIOS, Cylinder, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CONVEX] = &BVHShapeDistancer<kIOS, Convex, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_PLANE] = &BVHShapeDistancer<kIOS, Plane, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_HALFSPACE] = &BVHShapeDistancer<kIOS, Halfspace, NarrowPhaseSolver>::distance;

  distance_matrix[BV_OBBRSS][GEOM_BOX] = &BVHShapeDistancer<OBBRSS, Box, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_SPHERE] = &BVHShapeDistancer<OBBRSS, Sphere, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_ELLIPSOID] = &BVHShapeDistancer<OBBRSS, Ellipsoid, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CAPSULE] = &BVHShapeDistancer<OBBRSS, Capsule, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CONE] = &BVHShapeDistancer<OBBRSS, Cone, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CYLINDER] = &BVHShapeDistancer<OBBRSS, Cylinder, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CONVEX] = &BVHShapeDistancer<OBBRSS, Convex, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_PLANE] = &BVHShapeDistancer<OBBRSS, Plane, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_HALFSPACE] = &BVHShapeDistancer<OBBRSS, Halfspace, NarrowPhaseSolver>::distance;

  distance_matrix[BV_AABB][BV_AABB] = &BVHDistance<AABB, NarrowPhaseSolver>;
  distance_matrix[BV_RSS][BV_RSS] = &BVHDistance<RSS, NarrowPhaseSolver>;
  distance_matrix[BV_kIOS][BV_kIOS] = &BVHDistance<kIOS, NarrowPhaseSolver>;
  distance_matrix[BV_OBBRSS][BV_OBBRSS] = &BVHDistance<OBBRSS, NarrowPhaseSolver>;

#if FCL_HAVE_OCTOMAP
  distance_matrix[GEOM_OCTREE][GEOM_BOX] = &OcTreeShapeDistance<Box, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_SPHERE] = &OcTreeShapeDistance<Sphere, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_ELLIPSOID] = &OcTreeShapeDistance<Ellipsoid, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_CAPSULE] = &OcTreeShapeDistance<Capsule, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_CONE] = &OcTreeShapeDistance<Cone, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_CYLINDER] = &OcTreeShapeDistance<Cylinder, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_CONVEX] = &OcTreeShapeDistance<Convex, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_PLANE] = &OcTreeShapeDistance<Plane, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_HALFSPACE] = &OcTreeShapeDistance<Halfspace, NarrowPhaseSolver>;

  distance_matrix[GEOM_BOX][GEOM_OCTREE] = &ShapeOcTreeDistance<Box, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_OCTREE] = &ShapeOcTreeDistance<Sphere, NarrowPhaseSolver>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_OCTREE] = &ShapeOcTreeDistance<Ellipsoid, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_OCTREE] = &ShapeOcTreeDistance<Capsule, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_OCTREE] = &ShapeOcTreeDistance<Cone, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_OCTREE] = &ShapeOcTreeDistance<Cylinder, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_OCTREE] = &ShapeOcTreeDistance<Convex, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_OCTREE] = &ShapeOcTreeDistance<Plane, NarrowPhaseSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_OCTREE] = &ShapeOcTreeDistance<Halfspace, NarrowPhaseSolver>;

  distance_matrix[GEOM_OCTREE][GEOM_OCTREE] = &OcTreeDistance<NarrowPhaseSolver>;

  distance_matrix[GEOM_OCTREE][BV_AABB] = &OcTreeBVHDistance<AABB, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_OBB] = &OcTreeBVHDistance<OBB, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_RSS] = &OcTreeBVHDistance<RSS, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_OBBRSS] = &OcTreeBVHDistance<OBBRSS, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_kIOS] = &OcTreeBVHDistance<kIOS, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_KDOP16] = &OcTreeBVHDistance<KDOP<16>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_KDOP18] = &OcTreeBVHDistance<KDOP<18>, NarrowPhaseSolver>;
  distance_matrix[GEOM_OCTREE][BV_KDOP24] = &OcTreeBVHDistance<KDOP<24>, NarrowPhaseSolver>;

  distance_matrix[BV_AABB][GEOM_OCTREE] = &BVHOcTreeDistance<AABB, NarrowPhaseSolver>;
  distance_matrix[BV_OBB][GEOM_OCTREE] = &BVHOcTreeDistance<OBB, NarrowPhaseSolver>;
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

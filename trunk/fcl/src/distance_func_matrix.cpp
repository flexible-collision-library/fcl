/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
#include "fcl/simple_setup.h"
#include "fcl/narrowphase/narrowphase.h"

namespace fcl
{

template<typename T_SH1, typename T_SH2, typename NarrowPhaseSolver>
BVH_REAL ShapeShapeDistance(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, const NarrowPhaseSolver* nsolver)
{
  ShapeDistanceTraversalNode<T_SH1, T_SH2, NarrowPhaseSolver> node;
  const T_SH1* obj1 = static_cast<const T_SH1*>(o1);
  const T_SH2* obj2 = static_cast<const T_SH2*>(o2);
  initialize(node, *obj1, tf1, *obj2, tf2, nsolver);
  distance(&node);

  return node.min_distance;
}

template<typename T_BVH, typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeDistancer
{
  static BVH_REAL distance(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, const NarrowPhaseSolver* nsolver)
  {
    MeshShapeDistanceTraversalNode<T_BVH, T_SH, NarrowPhaseSolver> node;
    const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
    BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
    SimpleTransform tf1_tmp = tf1;
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    initialize(node, *obj1_tmp, tf1_tmp, *obj2, tf2, nsolver);
    fcl::distance(&node);
    
    delete obj1_tmp;
    return node.min_distance;
  }
};

template<typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeDistancer<RSS, T_SH, NarrowPhaseSolver>
{
  static BVH_REAL distance(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, const NarrowPhaseSolver* nsolver)
  {
    MeshShapeDistanceTraversalNodeRSS<T_SH, NarrowPhaseSolver> node;
    const BVHModel<RSS>* obj1 = static_cast<const BVHModel<RSS>* >(o1);
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    initialize(node, *obj1, tf1, *obj2, tf2, nsolver);
    fcl::distance(&node);

    return node.min_distance;
  }
};


template<typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeDistancer<kIOS, T_SH, NarrowPhaseSolver>
{
  static BVH_REAL distance(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, const NarrowPhaseSolver* nsolver)
  {
    MeshShapeDistanceTraversalNodekIOS<T_SH, NarrowPhaseSolver> node;
    const BVHModel<kIOS>* obj1 = static_cast<const BVHModel<kIOS>* >(o1);
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    initialize(node, *obj1, tf1, *obj2, tf2, nsolver);
    fcl::distance(&node);

    return node.min_distance;
  }
};

template<typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeDistancer<OBBRSS, T_SH, NarrowPhaseSolver>
{
  static BVH_REAL distance(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, const NarrowPhaseSolver* nsolver)
  {
    MeshShapeDistanceTraversalNodeOBBRSS<T_SH, NarrowPhaseSolver> node;
    const BVHModel<OBBRSS>* obj1 = static_cast<const BVHModel<OBBRSS>* >(o1);
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    initialize(node, *obj1, tf1, *obj2, tf2, nsolver);
    fcl::distance(&node);

    return node.min_distance;
  }
};


template<typename T_BVH>
BVH_REAL BVHDistance(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2)
{
  MeshDistanceTraversalNode<T_BVH> node;
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
  const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>* >(o2);
  BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
  SimpleTransform tf1_tmp = tf1;
  BVHModel<T_BVH>* obj2_tmp = new BVHModel<T_BVH>(*obj2);
  SimpleTransform tf2_tmp = tf2;

  initialize(node, *obj1_tmp, tf1_tmp, *obj2_tmp, tf2_tmp);
  distance(&node);
  
  return node.min_distance;
}

template<>
BVH_REAL BVHDistance<RSS>(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2)
{
  MeshDistanceTraversalNodeRSS node;
  const BVHModel<RSS>* obj1 = static_cast<const BVHModel<RSS>* >(o1);
  const BVHModel<RSS>* obj2 = static_cast<const BVHModel<RSS>* >(o2);

  initialize(node, *obj1, tf1, *obj2, tf2);
  distance(&node);

  return node.min_distance;
}

template<>
BVH_REAL BVHDistance<kIOS>(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2)
{
  MeshDistanceTraversalNodekIOS node;
  const BVHModel<kIOS>* obj1 = static_cast<const BVHModel<kIOS>* >(o1);
  const BVHModel<kIOS>* obj2 = static_cast<const BVHModel<kIOS>* >(o2);

  initialize(node, *obj1, tf1, *obj2, tf2);
  distance(&node);

  return node.min_distance;
}


template<>
BVH_REAL BVHDistance<OBBRSS>(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2)
{
  MeshDistanceTraversalNodeOBBRSS node;
  const BVHModel<OBBRSS>* obj1 = static_cast<const BVHModel<OBBRSS>* >(o1);
  const BVHModel<OBBRSS>* obj2 = static_cast<const BVHModel<OBBRSS>* >(o2);

  initialize(node, *obj1, tf1, *obj2, tf2);
  distance(&node);

  return node.min_distance;
}


template<typename T_BVH, typename NarrowPhaseSolver>
BVH_REAL BVHDistance(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2,
                     const NarrowPhaseSolver* nsolver)
{
  return BVHDistance<T_BVH>(o1, tf1, o2, tf2);
}

template<typename NarrowPhaseSolver>
DistanceFunctionMatrix<NarrowPhaseSolver>::DistanceFunctionMatrix()
{
  for(int i = 0; i < 16; ++i)
  {
    for(int j = 0; j < 16; ++j)
      distance_matrix[i][j] = NULL;
  }

  distance_matrix[GEOM_BOX][GEOM_BOX] = &ShapeShapeDistance<Box, Box, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_SPHERE] = &ShapeShapeDistance<Box, Sphere, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_CAPSULE] = &ShapeShapeDistance<Box, Capsule, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_CONE] = &ShapeShapeDistance<Box, Cone, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_CYLINDER] = &ShapeShapeDistance<Box, Cylinder, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_CONVEX] = &ShapeShapeDistance<Box, Convex, NarrowPhaseSolver>;
  distance_matrix[GEOM_BOX][GEOM_PLANE] = &ShapeShapeDistance<Box, Plane, NarrowPhaseSolver>;

  distance_matrix[GEOM_SPHERE][GEOM_BOX] = &ShapeShapeDistance<Sphere, Box, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_SPHERE] = &ShapeShapeDistance<Sphere, Sphere, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_CAPSULE] = &ShapeShapeDistance<Sphere, Capsule, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_CONE] = &ShapeShapeDistance<Sphere, Cone, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_CYLINDER] = &ShapeShapeDistance<Sphere, Cylinder, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_CONVEX] = &ShapeShapeDistance<Sphere, Convex, NarrowPhaseSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_PLANE] = &ShapeShapeDistance<Sphere, Plane, NarrowPhaseSolver>;

  distance_matrix[GEOM_CAPSULE][GEOM_BOX] = &ShapeShapeDistance<Capsule, Box, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_SPHERE] = &ShapeShapeDistance<Capsule, Sphere, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_CAPSULE] = &ShapeShapeDistance<Capsule, Capsule, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_CONE] = &ShapeShapeDistance<Capsule, Cone, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_CYLINDER] = &ShapeShapeDistance<Capsule, Cylinder, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_CONVEX] = &ShapeShapeDistance<Capsule, Convex, NarrowPhaseSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_PLANE] = &ShapeShapeDistance<Capsule, Plane, NarrowPhaseSolver>;

  distance_matrix[GEOM_CONE][GEOM_BOX] = &ShapeShapeDistance<Cone, Box, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_SPHERE] = &ShapeShapeDistance<Cone, Sphere, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_CAPSULE] = &ShapeShapeDistance<Cone, Capsule, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_CONE] = &ShapeShapeDistance<Cone, Cone, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_CYLINDER] = &ShapeShapeDistance<Cone, Cylinder, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_CONVEX] = &ShapeShapeDistance<Cone, Convex, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONE][GEOM_PLANE] = &ShapeShapeDistance<Cone, Plane, NarrowPhaseSolver>;

  distance_matrix[GEOM_CYLINDER][GEOM_BOX] = &ShapeShapeDistance<Cylinder, Box, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_SPHERE] = &ShapeShapeDistance<Cylinder, Sphere, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_CAPSULE] = &ShapeShapeDistance<Cylinder, Capsule, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_CONE] = &ShapeShapeDistance<Cylinder, Cone, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_CYLINDER] = &ShapeShapeDistance<Cylinder, Cylinder, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_CONVEX] = &ShapeShapeDistance<Cylinder, Convex, NarrowPhaseSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_PLANE] = &ShapeShapeDistance<Cylinder, Plane, NarrowPhaseSolver>;

  distance_matrix[GEOM_CONVEX][GEOM_BOX] = &ShapeShapeDistance<Convex, Box, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_SPHERE] = &ShapeShapeDistance<Convex, Sphere, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_CAPSULE] = &ShapeShapeDistance<Convex, Capsule, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_CONE] = &ShapeShapeDistance<Convex, Cone, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_CYLINDER] = &ShapeShapeDistance<Convex, Cylinder, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_CONVEX] = &ShapeShapeDistance<Convex, Convex, NarrowPhaseSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_PLANE] = &ShapeShapeDistance<Convex, Plane, NarrowPhaseSolver>;

  distance_matrix[GEOM_PLANE][GEOM_BOX] = &ShapeShapeDistance<Plane, Box, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_SPHERE] = &ShapeShapeDistance<Plane, Sphere, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_CAPSULE] = &ShapeShapeDistance<Plane, Capsule, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_CONE] = &ShapeShapeDistance<Plane, Cone, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_CYLINDER] = &ShapeShapeDistance<Plane, Cylinder, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_CONVEX] = &ShapeShapeDistance<Plane, Convex, NarrowPhaseSolver>;
  distance_matrix[GEOM_PLANE][GEOM_PLANE] = &ShapeShapeDistance<Plane, Plane, NarrowPhaseSolver>;

  /*
  distance_matrix[BV_AABB][GEOM_BOX] = &BVHShapeDistancer<AABB, Box, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_SPHERE] = &BVHShapeDistancer<AABB, Sphere, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CAPSULE] = &BVHShapeDistancer<AABB, Capsule, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CONE] = &BVHShapeDistancer<AABB, Cone, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CYLINDER] = &BVHShapeDistancer<AABB, Cylinder, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CONVEX] = &BVHShapeDistancer<AABB, Convex, NarrowPhaseSolver>::distance;
  distance_matrix[BV_AABB][GEOM_PLANE] = &BVHShapeDistancer<AABB, Plane, NarrowPhaseSolver>::distance;

  distance_matrix[BV_OBB][GEOM_BOX] = &BVHShapeDistancer<OBB, Box, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_SPHERE] = &BVHShapeDistancer<OBB, Sphere, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_CAPSULE] = &BVHShapeDistancer<OBB, Capsule, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_CONE] = &BVHShapeDistancer<OBB, Cone, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_CYLINDER] = &BVHShapeDistancer<OBB, Cylinder, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_CONVEX] = &BVHShapeDistancer<OBB, Convex, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBB][GEOM_PLANE] = &BVHShapeDistancer<OBB, Plane, NarrowPhaseSolver>::distance;
  */

  distance_matrix[BV_RSS][GEOM_BOX] = &BVHShapeDistancer<RSS, Box, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_SPHERE] = &BVHShapeDistancer<RSS, Sphere, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CAPSULE] = &BVHShapeDistancer<RSS, Capsule, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CONE] = &BVHShapeDistancer<RSS, Cone, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CYLINDER] = &BVHShapeDistancer<RSS, Cylinder, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CONVEX] = &BVHShapeDistancer<RSS, Convex, NarrowPhaseSolver>::distance;
  distance_matrix[BV_RSS][GEOM_PLANE] = &BVHShapeDistancer<RSS, Plane, NarrowPhaseSolver>::distance;

  /*
  distance_matrix[BV_KDOP16][GEOM_BOX] = &BVHShapeDistancer<KDOP<16>, Box, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_SPHERE] = &BVHShapeDistancer<KDOP<16>, Sphere, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CAPSULE] = &BVHShapeDistancer<KDOP<16>, Capsule, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CONE] = &BVHShapeDistancer<KDOP<16>, Cone, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CYLINDER] = &BVHShapeDistancer<KDOP<16>, Cylinder, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CONVEX] = &BVHShapeDistancer<KDOP<16>, Convex, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_PLANE] = &BVHShapeDistancer<KDOP<16>, Plane, NarrowPhaseSolver>::distance;

  distance_matrix[BV_KDOP18][GEOM_BOX] = &BVHShapeDistancer<KDOP<18>, Box, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_SPHERE] = &BVHShapeDistancer<KDOP<18>, Sphere, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CAPSULE] = &BVHShapeDistancer<KDOP<18>, Capsule, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CONE] = &BVHShapeDistancer<KDOP<18>, Cone, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CYLINDER] = &BVHShapeDistancer<KDOP<18>, Cylinder, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CONVEX] = &BVHShapeDistancer<KDOP<18>, Convex, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_PLANE] = &BVHShapeDistancer<KDOP<18>, Plane, NarrowPhaseSolver>::distance;

  distance_matrix[BV_KDOP24][GEOM_BOX] = &BVHShapeDistancer<KDOP<24>, Box, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_SPHERE] = &BVHShapeDistancer<KDOP<24>, Sphere, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CAPSULE] = &BVHShapeDistancer<KDOP<24>, Capsule, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CONE] = &BVHShapeDistancer<KDOP<24>, Cone, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CYLINDER] = &BVHShapeDistancer<KDOP<24>, Cylinder, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CONVEX] = &BVHShapeDistancer<KDOP<24>, Convex, NarrowPhaseSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_PLANE] = &BVHShapeDistancer<KDOP<24>, Plane, NarrowPhaseSolver>::distance;
  */

  distance_matrix[BV_kIOS][GEOM_BOX] = &BVHShapeDistancer<kIOS, Box, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_SPHERE] = &BVHShapeDistancer<kIOS, Sphere, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CAPSULE] = &BVHShapeDistancer<kIOS, Capsule, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CONE] = &BVHShapeDistancer<kIOS, Cone, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CYLINDER] = &BVHShapeDistancer<kIOS, Cylinder, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CONVEX] = &BVHShapeDistancer<kIOS, Convex, NarrowPhaseSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_PLANE] = &BVHShapeDistancer<kIOS, Plane, NarrowPhaseSolver>::distance;

  distance_matrix[BV_OBBRSS][GEOM_BOX] = &BVHShapeDistancer<OBBRSS, Box, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_SPHERE] = &BVHShapeDistancer<OBBRSS, Sphere, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CAPSULE] = &BVHShapeDistancer<OBBRSS, Capsule, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CONE] = &BVHShapeDistancer<OBBRSS, Cone, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CYLINDER] = &BVHShapeDistancer<OBBRSS, Cylinder, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CONVEX] = &BVHShapeDistancer<OBBRSS, Convex, NarrowPhaseSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_PLANE] = &BVHShapeDistancer<OBBRSS, Plane, NarrowPhaseSolver>::distance;

  distance_matrix[BV_AABB][BV_AABB] = &BVHDistance<AABB, NarrowPhaseSolver>;
  distance_matrix[BV_RSS][BV_RSS] = &BVHDistance<RSS, NarrowPhaseSolver>;
  distance_matrix[BV_kIOS][BV_kIOS] = &BVHDistance<kIOS, NarrowPhaseSolver>;
  distance_matrix[BV_OBBRSS][BV_OBBRSS] = &BVHDistance<OBBRSS, NarrowPhaseSolver>;
}

template struct DistanceFunctionMatrix<GJKSolver_libccd>;
template struct DistanceFunctionMatrix<GJKSolver_indep>;


}

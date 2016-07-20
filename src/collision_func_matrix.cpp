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


#include "fcl/collision_func_matrix.h"

#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/collision_node.h"
#include "fcl/narrowphase/narrowphase.h"


namespace fcl
{

#if FCL_HAVE_OCTOMAP
template<typename T_SH, typename NarrowPhaseSolver>
std::size_t ShapeOcTreeCollide(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2,
                               const NarrowPhaseSolver* nsolver,
                               const CollisionRequest& request, CollisionResult& result)
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

template<typename T_SH, typename NarrowPhaseSolver>
std::size_t OcTreeShapeCollide(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2,
                               const NarrowPhaseSolver* nsolver,
                               const CollisionRequest& request, CollisionResult& result)
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

template<typename NarrowPhaseSolver>
std::size_t OcTreeCollide(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2,
                          const NarrowPhaseSolver* nsolver,
                          const CollisionRequest& request, CollisionResult& result)
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

template<typename T_BVH, typename NarrowPhaseSolver>
std::size_t OcTreeBVHCollide(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest& request, CollisionResult& result)
{
  if(request.isSatisfied(result)) return result.numContacts();

  if(request.enable_cost && request.use_approximate_cost)
  {
    CollisionRequest no_cost_request(request); // request remove cost to avoid the exact but expensive cost computation between mesh and octree
    no_cost_request.enable_cost = false; // disable cost computation

    OcTreeMeshCollisionTraversalNode<T_BVH, NarrowPhaseSolver> node;
    const OcTree* obj1 = static_cast<const OcTree*>(o1);
    const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>*>(o2);
    OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

    initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, no_cost_request, result);
    collide(&node);

    Box box;
    Transform3f box_tf;
    constructBox(obj2->getBV(0).bv, tf2, box, box_tf); // compute the box for BVH's root node

    box.cost_density = obj2->cost_density;
    box.threshold_occupied = obj2->threshold_occupied;
    box.threshold_free = obj2->threshold_free;

    CollisionRequest only_cost_request(result.numContacts(), false, request.num_max_cost_sources, true, false); // additional cost request, no contacts
    OcTreeShapeCollide<Box, NarrowPhaseSolver>(o1, tf1, &box, box_tf, nsolver, only_cost_request, result);
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

template<typename T_BVH, typename NarrowPhaseSolver>
std::size_t BVHOcTreeCollide(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest& request, CollisionResult& result)
{
  if(request.isSatisfied(result)) return result.numContacts();
 
  if(request.enable_cost && request.use_approximate_cost)
  {
    CollisionRequest no_cost_request(request); // request remove cost to avoid the exact but expensive cost computation between mesh and octree
    no_cost_request.enable_cost = false; // disable cost computation

    MeshOcTreeCollisionTraversalNode<T_BVH, NarrowPhaseSolver> node;
    const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>*>(o1);
    const OcTree* obj2 = static_cast<const OcTree*>(o2);
    OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

    initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, no_cost_request, result);
    collide(&node);

    Box box;
    Transform3f box_tf;
    constructBox(obj1->getBV(0).bv, tf1, box, box_tf);

    box.cost_density = obj1->cost_density;
    box.threshold_occupied = obj1->threshold_occupied;
    box.threshold_free = obj1->threshold_free;

    CollisionRequest only_cost_request(result.numContacts(), false, request.num_max_cost_sources, true, false);
    ShapeOcTreeCollide<Box, NarrowPhaseSolver>(&box, box_tf, o2, tf2, nsolver, only_cost_request, result);
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

template<typename T_SH1, typename T_SH2, typename NarrowPhaseSolver>
std::size_t ShapeShapeCollide(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, 
                              const NarrowPhaseSolver* nsolver,
                              const CollisionRequest& request, CollisionResult& result)
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

template<typename T_BVH, typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeCollider
{
  static std::size_t collide(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, 
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest& request, CollisionResult& result)
  {
    if(request.isSatisfied(result)) return result.numContacts();

    if(request.enable_cost && request.use_approximate_cost)
    {
      CollisionRequest no_cost_request(request);
      no_cost_request.enable_cost = false;
      
      MeshShapeCollisionTraversalNode<T_BVH, T_SH, NarrowPhaseSolver> node;
      const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
      BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
      Transform3f tf1_tmp = tf1;
      const T_SH* obj2 = static_cast<const T_SH*>(o2);

      initialize(node, *obj1_tmp, tf1_tmp, *obj2, tf2, nsolver, no_cost_request, result);
      fcl::collide(&node);

      delete obj1_tmp;

      Box box;
      Transform3f box_tf;
      constructBox(obj1->getBV(0).bv, tf1, box, box_tf);

      box.cost_density = obj1->cost_density;
      box.threshold_occupied = obj1->threshold_occupied;
      box.threshold_free = obj1->threshold_free;
      
      CollisionRequest only_cost_request(result.numContacts(), false, request.num_max_cost_sources, true, false);
      ShapeShapeCollide<Box, T_SH>(&box, box_tf, o2, tf2, nsolver, only_cost_request, result);
    }
    else
    {
      MeshShapeCollisionTraversalNode<T_BVH, T_SH, NarrowPhaseSolver> node;
      const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
      BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
      Transform3f tf1_tmp = tf1;
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

template<typename OrientMeshShapeCollisionTraveralNode, typename T_BVH, typename T_SH, typename NarrowPhaseSolver>
std::size_t orientedBVHShapeCollide(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, 
                                    const NarrowPhaseSolver* nsolver,
                                    const CollisionRequest& request, CollisionResult& result)
{
  if(request.isSatisfied(result)) return result.numContacts();

  if(request.enable_cost && request.use_approximate_cost)
  {
    CollisionRequest no_cost_request(request);
    no_cost_request.enable_cost = false;

    OrientMeshShapeCollisionTraveralNode node;
    const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    initialize(node, *obj1, tf1, *obj2, tf2, nsolver, no_cost_request, result);
    fcl::collide(&node);
   
    Box box;
    Transform3f box_tf;
    constructBox(obj1->getBV(0).bv, tf1, box, box_tf);

    box.cost_density = obj1->cost_density;
    box.threshold_occupied = obj1->threshold_occupied;
    box.threshold_free = obj1->threshold_free;
      
    CollisionRequest only_cost_request(result.numContacts(), false, request.num_max_cost_sources, true, false);
    ShapeShapeCollide<Box, T_SH>(&box, box_tf, o2, tf2, nsolver, only_cost_request, result);     
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

}


template<typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeCollider<OBB, T_SH, NarrowPhaseSolver>
{
  static std::size_t collide(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, 
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest& request, CollisionResult& result)
  {
    return details::orientedBVHShapeCollide<MeshShapeCollisionTraversalNodeOBB<T_SH, NarrowPhaseSolver>, OBB, T_SH, NarrowPhaseSolver>(o1, tf1, o2, tf2, nsolver, request, result);
  } 
};


template<typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeCollider<RSS, T_SH, NarrowPhaseSolver>
{
  static std::size_t collide(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, 
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest& request, CollisionResult& result)
  {
    return details::orientedBVHShapeCollide<MeshShapeCollisionTraversalNodeRSS<T_SH, NarrowPhaseSolver>, RSS, T_SH, NarrowPhaseSolver>(o1, tf1, o2, tf2, nsolver, request, result);
  } 
};


template<typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeCollider<kIOS, T_SH, NarrowPhaseSolver>
{
  static std::size_t collide(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, 
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest& request, CollisionResult& result)
  {
    return details::orientedBVHShapeCollide<MeshShapeCollisionTraversalNodekIOS<T_SH, NarrowPhaseSolver>, kIOS, T_SH, NarrowPhaseSolver>(o1, tf1, o2, tf2, nsolver, request, result);
  } 
};


template<typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeCollider<OBBRSS, T_SH, NarrowPhaseSolver>
{
  static std::size_t collide(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, 
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest& request, CollisionResult& result)
  {
    return details::orientedBVHShapeCollide<MeshShapeCollisionTraversalNodeOBBRSS<T_SH, NarrowPhaseSolver>, OBBRSS, T_SH, NarrowPhaseSolver>(o1, tf1, o2, tf2, nsolver, request, result);
  } 
};


template<typename T_BVH>
std::size_t BVHCollide(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const CollisionRequest& request, CollisionResult& result)
{
  if(request.isSatisfied(result)) return result.numContacts();
  
  MeshCollisionTraversalNode<T_BVH> node;
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
  const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>* >(o2);
  BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
  Transform3f tf1_tmp = tf1;
  BVHModel<T_BVH>* obj2_tmp = new BVHModel<T_BVH>(*obj2);
  Transform3f tf2_tmp = tf2;
  
  initialize(node, *obj1_tmp, tf1_tmp, *obj2_tmp, tf2_tmp, request, result);
  collide(&node);

  delete obj1_tmp;
  delete obj2_tmp;

  return result.numContacts();
}

namespace details
{
template<typename OrientedMeshCollisionTraversalNode, typename T_BVH>
std::size_t orientedMeshCollide(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const CollisionRequest& request, CollisionResult& result)
{
  if(request.isSatisfied(result)) return result.numContacts();

  OrientedMeshCollisionTraversalNode node;
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
  const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>* >(o2);

  initialize(node, *obj1, tf1, *obj2, tf2, request, result);
  collide(&node);

  return result.numContacts();
}

}

template<>
std::size_t BVHCollide<OBB>(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const CollisionRequest& request, CollisionResult& result)
{
  return details::orientedMeshCollide<MeshCollisionTraversalNodeOBB, OBB>(o1, tf1, o2, tf2, request, result);
}

template<>
std::size_t BVHCollide<OBBRSS>(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const CollisionRequest& request, CollisionResult& result)
{
  return details::orientedMeshCollide<MeshCollisionTraversalNodeOBBRSS, OBBRSS>(o1, tf1, o2, tf2, request, result);
}


template<>
std::size_t BVHCollide<kIOS>(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const CollisionRequest& request, CollisionResult& result)
{
  return details::orientedMeshCollide<MeshCollisionTraversalNodekIOS, kIOS>(o1, tf1, o2, tf2, request, result);
}


template<typename T_BVH, typename NarrowPhaseSolver>
std::size_t BVHCollide(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, 
                       const NarrowPhaseSolver* nsolver,
                       const CollisionRequest& request, CollisionResult& result)
{
  return BVHCollide<T_BVH>(o1, tf1, o2, tf2, request, result);
}


template<typename NarrowPhaseSolver>
CollisionFunctionMatrix<NarrowPhaseSolver>::CollisionFunctionMatrix()
{
  for(int i = 0; i < NODE_COUNT; ++i)
  {
    for(int j = 0; j < NODE_COUNT; ++j)
      collision_matrix[i][j] = NULL;
  }

  collision_matrix[GEOM_BOX][GEOM_BOX] = &ShapeShapeCollide<Box, Box, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_SPHERE] = &ShapeShapeCollide<Box, Sphere, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_ELLIPSOID] = &ShapeShapeCollide<Box, Ellipsoid, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_CAPSULE] = &ShapeShapeCollide<Box, Capsule, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_CONE] = &ShapeShapeCollide<Box, Cone, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_CYLINDER] = &ShapeShapeCollide<Box, Cylinder, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_CONVEX] = &ShapeShapeCollide<Box, Convex, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_PLANE] = &ShapeShapeCollide<Box, Plane, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_HALFSPACE] = &ShapeShapeCollide<Box, Halfspace, NarrowPhaseSolver>;

  collision_matrix[GEOM_SPHERE][GEOM_BOX] = &ShapeShapeCollide<Sphere, Box, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_SPHERE] = &ShapeShapeCollide<Sphere, Sphere, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_ELLIPSOID] = &ShapeShapeCollide<Sphere, Ellipsoid, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_CAPSULE] = &ShapeShapeCollide<Sphere, Capsule, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_CONE] = &ShapeShapeCollide<Sphere, Cone, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_CYLINDER] = &ShapeShapeCollide<Sphere, Cylinder, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_CONVEX] = &ShapeShapeCollide<Sphere, Convex, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_PLANE] = &ShapeShapeCollide<Sphere, Plane, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_HALFSPACE] = &ShapeShapeCollide<Sphere, Halfspace, NarrowPhaseSolver>;

  collision_matrix[GEOM_ELLIPSOID][GEOM_BOX] = &ShapeShapeCollide<Ellipsoid, Box, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_SPHERE] = &ShapeShapeCollide<Ellipsoid, Sphere, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_ELLIPSOID] = &ShapeShapeCollide<Ellipsoid, Ellipsoid, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_CAPSULE] = &ShapeShapeCollide<Ellipsoid, Capsule, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_CONE] = &ShapeShapeCollide<Ellipsoid, Cone, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_CYLINDER] = &ShapeShapeCollide<Ellipsoid, Cylinder, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_CONVEX] = &ShapeShapeCollide<Ellipsoid, Convex, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_PLANE] = &ShapeShapeCollide<Ellipsoid, Plane, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_HALFSPACE] = &ShapeShapeCollide<Ellipsoid, Halfspace, NarrowPhaseSolver>;

  collision_matrix[GEOM_CAPSULE][GEOM_BOX] = &ShapeShapeCollide<Capsule, Box, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_SPHERE] = &ShapeShapeCollide<Capsule, Sphere, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_ELLIPSOID] = &ShapeShapeCollide<Capsule, Ellipsoid, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_CAPSULE] = &ShapeShapeCollide<Capsule, Capsule, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_CONE] = &ShapeShapeCollide<Capsule, Cone, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_CYLINDER] = &ShapeShapeCollide<Capsule, Cylinder, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_CONVEX] = &ShapeShapeCollide<Capsule, Convex, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_PLANE] = &ShapeShapeCollide<Capsule, Plane, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_HALFSPACE] = &ShapeShapeCollide<Capsule, Halfspace, NarrowPhaseSolver>;

  collision_matrix[GEOM_CONE][GEOM_BOX] = &ShapeShapeCollide<Cone, Box, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_SPHERE] = &ShapeShapeCollide<Cone, Sphere, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_ELLIPSOID] = &ShapeShapeCollide<Cone, Ellipsoid, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_CAPSULE] = &ShapeShapeCollide<Cone, Capsule, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_CONE] = &ShapeShapeCollide<Cone, Cone, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_CYLINDER] = &ShapeShapeCollide<Cone, Cylinder, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_CONVEX] = &ShapeShapeCollide<Cone, Convex, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_PLANE] = &ShapeShapeCollide<Cone, Plane, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_HALFSPACE] = &ShapeShapeCollide<Cone, Halfspace, NarrowPhaseSolver>;

  collision_matrix[GEOM_CYLINDER][GEOM_BOX] = &ShapeShapeCollide<Cylinder, Box, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_SPHERE] = &ShapeShapeCollide<Cylinder, Sphere, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_ELLIPSOID] = &ShapeShapeCollide<Cylinder, Ellipsoid, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_CAPSULE] = &ShapeShapeCollide<Cylinder, Capsule, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_CONE] = &ShapeShapeCollide<Cylinder, Cone, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_CYLINDER] = &ShapeShapeCollide<Cylinder, Cylinder, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_CONVEX] = &ShapeShapeCollide<Cylinder, Convex, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_PLANE] = &ShapeShapeCollide<Cylinder, Plane, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_HALFSPACE] = &ShapeShapeCollide<Cylinder, Halfspace, NarrowPhaseSolver>;

  collision_matrix[GEOM_CONVEX][GEOM_BOX] = &ShapeShapeCollide<Convex, Box, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_SPHERE] = &ShapeShapeCollide<Convex, Sphere, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_ELLIPSOID] = &ShapeShapeCollide<Convex, Ellipsoid, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_CAPSULE] = &ShapeShapeCollide<Convex, Capsule, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_CONE] = &ShapeShapeCollide<Convex, Cone, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_CYLINDER] = &ShapeShapeCollide<Convex, Cylinder, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_CONVEX] = &ShapeShapeCollide<Convex, Convex, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_PLANE] = &ShapeShapeCollide<Convex, Plane, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_HALFSPACE] = &ShapeShapeCollide<Convex, Halfspace, NarrowPhaseSolver>;

  collision_matrix[GEOM_PLANE][GEOM_BOX] = &ShapeShapeCollide<Plane, Box, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_SPHERE] = &ShapeShapeCollide<Plane, Sphere, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_ELLIPSOID] = &ShapeShapeCollide<Plane, Ellipsoid, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_CAPSULE] = &ShapeShapeCollide<Plane, Capsule, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_CONE] = &ShapeShapeCollide<Plane, Cone, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_CYLINDER] = &ShapeShapeCollide<Plane, Cylinder, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_CONVEX] = &ShapeShapeCollide<Plane, Convex, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_PLANE] = &ShapeShapeCollide<Plane, Plane, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_HALFSPACE] = &ShapeShapeCollide<Plane, Halfspace, NarrowPhaseSolver>;

  collision_matrix[GEOM_HALFSPACE][GEOM_BOX] = &ShapeShapeCollide<Halfspace, Box, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_SPHERE] = &ShapeShapeCollide<Halfspace, Sphere, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CAPSULE] = &ShapeShapeCollide<Halfspace, Capsule, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CONE] = &ShapeShapeCollide<Halfspace, Cone, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CYLINDER] = &ShapeShapeCollide<Halfspace, Cylinder, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CONVEX] = &ShapeShapeCollide<Halfspace, Convex, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_PLANE] = &ShapeShapeCollide<Halfspace, Plane, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_HALFSPACE] = &ShapeShapeCollide<Halfspace, Halfspace, NarrowPhaseSolver>;

  collision_matrix[BV_AABB][GEOM_BOX] = &BVHShapeCollider<AABB, Box, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_SPHERE] = &BVHShapeCollider<AABB, Sphere, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_ELLIPSOID] = &BVHShapeCollider<AABB, Ellipsoid, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_CAPSULE] = &BVHShapeCollider<AABB, Capsule, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_CONE] = &BVHShapeCollider<AABB, Cone, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_CYLINDER] = &BVHShapeCollider<AABB, Cylinder, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_CONVEX] = &BVHShapeCollider<AABB, Convex, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_PLANE] = &BVHShapeCollider<AABB, Plane, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_HALFSPACE] = &BVHShapeCollider<AABB, Halfspace, NarrowPhaseSolver>::collide;

  collision_matrix[BV_OBB][GEOM_BOX] = &BVHShapeCollider<OBB, Box, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_SPHERE] = &BVHShapeCollider<OBB, Sphere, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_ELLIPSOID] = &BVHShapeCollider<OBB, Ellipsoid, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_CAPSULE] = &BVHShapeCollider<OBB, Capsule, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_CONE] = &BVHShapeCollider<OBB, Cone, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_CYLINDER] = &BVHShapeCollider<OBB, Cylinder, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_CONVEX] = &BVHShapeCollider<OBB, Convex, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_PLANE] = &BVHShapeCollider<OBB, Plane, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_HALFSPACE] = &BVHShapeCollider<OBB, Halfspace, NarrowPhaseSolver>::collide;

  collision_matrix[BV_RSS][GEOM_BOX] = &BVHShapeCollider<RSS, Box, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_SPHERE] = &BVHShapeCollider<RSS, Sphere, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_ELLIPSOID] = &BVHShapeCollider<RSS, Ellipsoid, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_CAPSULE] = &BVHShapeCollider<RSS, Capsule, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_CONE] = &BVHShapeCollider<RSS, Cone, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_CYLINDER] = &BVHShapeCollider<RSS, Cylinder, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_CONVEX] = &BVHShapeCollider<RSS, Convex, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_PLANE] = &BVHShapeCollider<RSS, Plane, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_HALFSPACE] = &BVHShapeCollider<RSS, Halfspace, NarrowPhaseSolver>::collide;

  collision_matrix[BV_KDOP16][GEOM_BOX] = &BVHShapeCollider<KDOP<16>, Box, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_SPHERE] = &BVHShapeCollider<KDOP<16>, Sphere, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_ELLIPSOID] = &BVHShapeCollider<KDOP<16>, Ellipsoid, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_CAPSULE] = &BVHShapeCollider<KDOP<16>, Capsule, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_CONE] = &BVHShapeCollider<KDOP<16>, Cone, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_CYLINDER] = &BVHShapeCollider<KDOP<16>, Cylinder, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_CONVEX] = &BVHShapeCollider<KDOP<16>, Convex, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_PLANE] = &BVHShapeCollider<KDOP<16>, Plane, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_HALFSPACE] = &BVHShapeCollider<KDOP<16>, Halfspace, NarrowPhaseSolver>::collide;

  collision_matrix[BV_KDOP18][GEOM_BOX] = &BVHShapeCollider<KDOP<18>, Box, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_SPHERE] = &BVHShapeCollider<KDOP<18>, Sphere, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_ELLIPSOID] = &BVHShapeCollider<KDOP<18>, Ellipsoid, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_CAPSULE] = &BVHShapeCollider<KDOP<18>, Capsule, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_CONE] = &BVHShapeCollider<KDOP<18>, Cone, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_CYLINDER] = &BVHShapeCollider<KDOP<18>, Cylinder, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_CONVEX] = &BVHShapeCollider<KDOP<18>, Convex, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_PLANE] = &BVHShapeCollider<KDOP<18>, Plane, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_HALFSPACE] = &BVHShapeCollider<KDOP<18>, Halfspace, NarrowPhaseSolver>::collide;

  collision_matrix[BV_KDOP24][GEOM_BOX] = &BVHShapeCollider<KDOP<24>, Box, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_SPHERE] = &BVHShapeCollider<KDOP<24>, Sphere, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_ELLIPSOID] = &BVHShapeCollider<KDOP<24>, Ellipsoid, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_CAPSULE] = &BVHShapeCollider<KDOP<24>, Capsule, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_CONE] = &BVHShapeCollider<KDOP<24>, Cone, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_CYLINDER] = &BVHShapeCollider<KDOP<24>, Cylinder, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_CONVEX] = &BVHShapeCollider<KDOP<24>, Convex, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_PLANE] = &BVHShapeCollider<KDOP<24>, Plane, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_HALFSPACE] = &BVHShapeCollider<KDOP<24>, Halfspace, NarrowPhaseSolver>::collide;

  collision_matrix[BV_kIOS][GEOM_BOX] = &BVHShapeCollider<kIOS, Box, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_SPHERE] = &BVHShapeCollider<kIOS, Sphere, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_ELLIPSOID] = &BVHShapeCollider<kIOS, Ellipsoid, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_CAPSULE] = &BVHShapeCollider<kIOS, Capsule, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_CONE] = &BVHShapeCollider<kIOS, Cone, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_CYLINDER] = &BVHShapeCollider<kIOS, Cylinder, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_CONVEX] = &BVHShapeCollider<kIOS, Convex, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_PLANE] = &BVHShapeCollider<kIOS, Plane, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_HALFSPACE] = &BVHShapeCollider<kIOS, Halfspace, NarrowPhaseSolver>::collide;

  collision_matrix[BV_OBBRSS][GEOM_BOX] = &BVHShapeCollider<OBBRSS, Box, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_SPHERE] = &BVHShapeCollider<OBBRSS, Sphere, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_ELLIPSOID] = &BVHShapeCollider<OBBRSS, Ellipsoid, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CAPSULE] = &BVHShapeCollider<OBBRSS, Capsule, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CONE] = &BVHShapeCollider<OBBRSS, Cone, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CYLINDER] = &BVHShapeCollider<OBBRSS, Cylinder, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CONVEX] = &BVHShapeCollider<OBBRSS, Convex, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_PLANE] = &BVHShapeCollider<OBBRSS, Plane, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_HALFSPACE] = &BVHShapeCollider<OBBRSS, Halfspace, NarrowPhaseSolver>::collide;

  collision_matrix[BV_AABB][BV_AABB] = &BVHCollide<AABB, NarrowPhaseSolver>;
  collision_matrix[BV_OBB][BV_OBB] = &BVHCollide<OBB, NarrowPhaseSolver>;
  collision_matrix[BV_RSS][BV_RSS] = &BVHCollide<RSS, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP16][BV_KDOP16] = &BVHCollide<KDOP<16>, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP18][BV_KDOP18] = &BVHCollide<KDOP<18>, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP24][BV_KDOP24] = &BVHCollide<KDOP<24>, NarrowPhaseSolver>;
  collision_matrix[BV_kIOS][BV_kIOS] = &BVHCollide<kIOS, NarrowPhaseSolver>;
  collision_matrix[BV_OBBRSS][BV_OBBRSS] = &BVHCollide<OBBRSS, NarrowPhaseSolver>;

#if FCL_HAVE_OCTOMAP
  collision_matrix[GEOM_OCTREE][GEOM_BOX] = &OcTreeShapeCollide<Box, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_SPHERE] = &OcTreeShapeCollide<Sphere, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_ELLIPSOID] = &OcTreeShapeCollide<Ellipsoid, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_CAPSULE] = &OcTreeShapeCollide<Capsule, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_CONE] = &OcTreeShapeCollide<Cone, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_CYLINDER] = &OcTreeShapeCollide<Cylinder, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_CONVEX] = &OcTreeShapeCollide<Convex, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_PLANE] = &OcTreeShapeCollide<Plane, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_HALFSPACE] = &OcTreeShapeCollide<Halfspace, NarrowPhaseSolver>;

  collision_matrix[GEOM_BOX][GEOM_OCTREE] = &ShapeOcTreeCollide<Box, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_OCTREE] = &ShapeOcTreeCollide<Sphere, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_OCTREE] = &ShapeOcTreeCollide<Ellipsoid, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_OCTREE] = &ShapeOcTreeCollide<Capsule, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_OCTREE] = &ShapeOcTreeCollide<Cone, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_OCTREE] = &ShapeOcTreeCollide<Cylinder, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_OCTREE] = &ShapeOcTreeCollide<Convex, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_OCTREE] = &ShapeOcTreeCollide<Plane, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_OCTREE] = &ShapeOcTreeCollide<Halfspace, NarrowPhaseSolver>;

  collision_matrix[GEOM_OCTREE][GEOM_OCTREE] = &OcTreeCollide<NarrowPhaseSolver>;

  collision_matrix[GEOM_OCTREE][BV_AABB] = &OcTreeBVHCollide<AABB, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_OBB] = &OcTreeBVHCollide<OBB, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_RSS] = &OcTreeBVHCollide<RSS, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_OBBRSS] = &OcTreeBVHCollide<OBBRSS, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_kIOS] = &OcTreeBVHCollide<kIOS, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_KDOP16] = &OcTreeBVHCollide<KDOP<16>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_KDOP18] = &OcTreeBVHCollide<KDOP<18>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_KDOP24] = &OcTreeBVHCollide<KDOP<24>, NarrowPhaseSolver>;

  collision_matrix[BV_AABB][GEOM_OCTREE] = &BVHOcTreeCollide<AABB, NarrowPhaseSolver>;
  collision_matrix[BV_OBB][GEOM_OCTREE] = &BVHOcTreeCollide<OBB, NarrowPhaseSolver>;
  collision_matrix[BV_RSS][GEOM_OCTREE] = &BVHOcTreeCollide<RSS, NarrowPhaseSolver>;
  collision_matrix[BV_OBBRSS][GEOM_OCTREE] = &BVHOcTreeCollide<OBBRSS, NarrowPhaseSolver>;
  collision_matrix[BV_kIOS][GEOM_OCTREE] = &BVHOcTreeCollide<kIOS, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP16][GEOM_OCTREE] = &BVHOcTreeCollide<KDOP<16>, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP18][GEOM_OCTREE] = &BVHOcTreeCollide<KDOP<18>, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP24][GEOM_OCTREE] = &BVHOcTreeCollide<KDOP<24>, NarrowPhaseSolver>;
#endif
}


template struct CollisionFunctionMatrix<GJKSolver_libccd>;
template struct CollisionFunctionMatrix<GJKSolver_indep>;


}

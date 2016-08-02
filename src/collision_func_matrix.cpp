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
std::size_t ShapeOcTreeCollide(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2,
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
std::size_t OcTreeShapeCollide(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2,
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
std::size_t OcTreeCollide(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2,
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
std::size_t OcTreeBVHCollide(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2,
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

    Boxd box;
    Transform3d box_tf;
    constructBox(obj2->getBV(0).bv, tf2, box, box_tf); // compute the box for BVH's root node

    box.cost_density = obj2->cost_density;
    box.threshold_occupied = obj2->threshold_occupied;
    box.threshold_free = obj2->threshold_free;

    CollisionRequest only_cost_request(result.numContacts(), false, request.num_max_cost_sources, true, false); // additional cost request, no contacts
    OcTreeShapeCollide<Boxd, NarrowPhaseSolver>(o1, tf1, &box, box_tf, nsolver, only_cost_request, result);
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
std::size_t BVHOcTreeCollide(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2,
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

    Boxd box;
    Transform3d box_tf;
    constructBox(obj1->getBV(0).bv, tf1, box, box_tf);

    box.cost_density = obj1->cost_density;
    box.threshold_occupied = obj1->threshold_occupied;
    box.threshold_free = obj1->threshold_free;

    CollisionRequest only_cost_request(result.numContacts(), false, request.num_max_cost_sources, true, false);
    ShapeOcTreeCollide<Boxd, NarrowPhaseSolver>(&box, box_tf, o2, tf2, nsolver, only_cost_request, result);
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
std::size_t ShapeShapeCollide(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2, 
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
  static std::size_t collide(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2, 
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
      Transform3d tf1_tmp = tf1;
      const T_SH* obj2 = static_cast<const T_SH*>(o2);

      initialize(node, *obj1_tmp, tf1_tmp, *obj2, tf2, nsolver, no_cost_request, result);
      fcl::collide(&node);

      delete obj1_tmp;

      Boxd box;
      Transform3d box_tf;
      constructBox(obj1->getBV(0).bv, tf1, box, box_tf);

      box.cost_density = obj1->cost_density;
      box.threshold_occupied = obj1->threshold_occupied;
      box.threshold_free = obj1->threshold_free;
      
      CollisionRequest only_cost_request(result.numContacts(), false, request.num_max_cost_sources, true, false);
      ShapeShapeCollide<Boxd, T_SH>(&box, box_tf, o2, tf2, nsolver, only_cost_request, result);
    }
    else
    {
      MeshShapeCollisionTraversalNode<T_BVH, T_SH, NarrowPhaseSolver> node;
      const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
      BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
      Transform3d tf1_tmp = tf1;
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
std::size_t orientedBVHShapeCollide(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2, 
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
   
    Boxd box;
    Transform3d box_tf;
    constructBox(obj1->getBV(0).bv, tf1, box, box_tf);

    box.cost_density = obj1->cost_density;
    box.threshold_occupied = obj1->threshold_occupied;
    box.threshold_free = obj1->threshold_free;
      
    CollisionRequest only_cost_request(result.numContacts(), false, request.num_max_cost_sources, true, false);
    ShapeShapeCollide<Boxd, T_SH>(&box, box_tf, o2, tf2, nsolver, only_cost_request, result);     
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
struct BVHShapeCollider<OBBd, T_SH, NarrowPhaseSolver>
{
  static std::size_t collide(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2, 
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest& request, CollisionResult& result)
  {
    return details::orientedBVHShapeCollide<MeshShapeCollisionTraversalNodeOBB<T_SH, NarrowPhaseSolver>, OBBd, T_SH, NarrowPhaseSolver>(o1, tf1, o2, tf2, nsolver, request, result);
  } 
};


template<typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeCollider<RSSd, T_SH, NarrowPhaseSolver>
{
  static std::size_t collide(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2, 
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest& request, CollisionResult& result)
  {
    return details::orientedBVHShapeCollide<MeshShapeCollisionTraversalNodeRSS<T_SH, NarrowPhaseSolver>, RSSd, T_SH, NarrowPhaseSolver>(o1, tf1, o2, tf2, nsolver, request, result);
  } 
};


template<typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeCollider<kIOSd, T_SH, NarrowPhaseSolver>
{
  static std::size_t collide(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2, 
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest& request, CollisionResult& result)
  {
    return details::orientedBVHShapeCollide<MeshShapeCollisionTraversalNodekIOS<T_SH, NarrowPhaseSolver>, kIOSd, T_SH, NarrowPhaseSolver>(o1, tf1, o2, tf2, nsolver, request, result);
  } 
};


template<typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeCollider<OBBRSSd, T_SH, NarrowPhaseSolver>
{
  static std::size_t collide(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2, 
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest& request, CollisionResult& result)
  {
    return details::orientedBVHShapeCollide<MeshShapeCollisionTraversalNodeOBBRSS<T_SH, NarrowPhaseSolver>, OBBRSSd, T_SH, NarrowPhaseSolver>(o1, tf1, o2, tf2, nsolver, request, result);
  } 
};


template<typename T_BVH>
std::size_t BVHCollide(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2, const CollisionRequest& request, CollisionResult& result)
{
  if(request.isSatisfied(result)) return result.numContacts();
  
  MeshCollisionTraversalNode<T_BVH> node;
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
  const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>* >(o2);
  BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
  Transform3d tf1_tmp = tf1;
  BVHModel<T_BVH>* obj2_tmp = new BVHModel<T_BVH>(*obj2);
  Transform3d tf2_tmp = tf2;
  
  initialize(node, *obj1_tmp, tf1_tmp, *obj2_tmp, tf2_tmp, request, result);
  collide(&node);

  delete obj1_tmp;
  delete obj2_tmp;

  return result.numContacts();
}

namespace details
{
template<typename OrientedMeshCollisionTraversalNode, typename T_BVH>
std::size_t orientedMeshCollide(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2, const CollisionRequest& request, CollisionResult& result)
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
std::size_t BVHCollide<OBBd>(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2, const CollisionRequest& request, CollisionResult& result)
{
  return details::orientedMeshCollide<MeshCollisionTraversalNodeOBB, OBBd>(o1, tf1, o2, tf2, request, result);
}

template<>
std::size_t BVHCollide<OBBRSSd>(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2, const CollisionRequest& request, CollisionResult& result)
{
  return details::orientedMeshCollide<MeshCollisionTraversalNodeOBBRSS, OBBRSSd>(o1, tf1, o2, tf2, request, result);
}


template<>
std::size_t BVHCollide<kIOSd>(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2, const CollisionRequest& request, CollisionResult& result)
{
  return details::orientedMeshCollide<MeshCollisionTraversalNodekIOS, kIOSd>(o1, tf1, o2, tf2, request, result);
}


template<typename T_BVH, typename NarrowPhaseSolver>
std::size_t BVHCollide(const CollisionGeometryd* o1, const Transform3d& tf1, const CollisionGeometryd* o2, const Transform3d& tf2, 
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

  collision_matrix[GEOM_BOX][GEOM_BOX] = &ShapeShapeCollide<Boxd, Boxd, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_SPHERE] = &ShapeShapeCollide<Boxd, Sphered, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_ELLIPSOID] = &ShapeShapeCollide<Boxd, Ellipsoidd, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_CAPSULE] = &ShapeShapeCollide<Boxd, Capsuled, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_CONE] = &ShapeShapeCollide<Boxd, Coned, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_CYLINDER] = &ShapeShapeCollide<Boxd, Cylinderd, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_CONVEX] = &ShapeShapeCollide<Boxd, Convexd, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_PLANE] = &ShapeShapeCollide<Boxd, Planed, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_HALFSPACE] = &ShapeShapeCollide<Boxd, Halfspaced, NarrowPhaseSolver>;

  collision_matrix[GEOM_SPHERE][GEOM_BOX] = &ShapeShapeCollide<Sphered, Boxd, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_SPHERE] = &ShapeShapeCollide<Sphered, Sphered, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_ELLIPSOID] = &ShapeShapeCollide<Sphered, Ellipsoidd, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_CAPSULE] = &ShapeShapeCollide<Sphered, Capsuled, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_CONE] = &ShapeShapeCollide<Sphered, Coned, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_CYLINDER] = &ShapeShapeCollide<Sphered, Cylinderd, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_CONVEX] = &ShapeShapeCollide<Sphered, Convexd, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_PLANE] = &ShapeShapeCollide<Sphered, Planed, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_HALFSPACE] = &ShapeShapeCollide<Sphered, Halfspaced, NarrowPhaseSolver>;

  collision_matrix[GEOM_ELLIPSOID][GEOM_BOX] = &ShapeShapeCollide<Ellipsoidd, Boxd, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_SPHERE] = &ShapeShapeCollide<Ellipsoidd, Sphered, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_ELLIPSOID] = &ShapeShapeCollide<Ellipsoidd, Ellipsoidd, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_CAPSULE] = &ShapeShapeCollide<Ellipsoidd, Capsuled, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_CONE] = &ShapeShapeCollide<Ellipsoidd, Coned, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_CYLINDER] = &ShapeShapeCollide<Ellipsoidd, Cylinderd, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_CONVEX] = &ShapeShapeCollide<Ellipsoidd, Convexd, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_PLANE] = &ShapeShapeCollide<Ellipsoidd, Planed, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_HALFSPACE] = &ShapeShapeCollide<Ellipsoidd, Halfspaced, NarrowPhaseSolver>;

  collision_matrix[GEOM_CAPSULE][GEOM_BOX] = &ShapeShapeCollide<Capsuled, Boxd, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_SPHERE] = &ShapeShapeCollide<Capsuled, Sphered, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_ELLIPSOID] = &ShapeShapeCollide<Capsuled, Ellipsoidd, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_CAPSULE] = &ShapeShapeCollide<Capsuled, Capsuled, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_CONE] = &ShapeShapeCollide<Capsuled, Coned, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_CYLINDER] = &ShapeShapeCollide<Capsuled, Cylinderd, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_CONVEX] = &ShapeShapeCollide<Capsuled, Convexd, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_PLANE] = &ShapeShapeCollide<Capsuled, Planed, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_HALFSPACE] = &ShapeShapeCollide<Capsuled, Halfspaced, NarrowPhaseSolver>;

  collision_matrix[GEOM_CONE][GEOM_BOX] = &ShapeShapeCollide<Coned, Boxd, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_SPHERE] = &ShapeShapeCollide<Coned, Sphered, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_ELLIPSOID] = &ShapeShapeCollide<Coned, Ellipsoidd, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_CAPSULE] = &ShapeShapeCollide<Coned, Capsuled, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_CONE] = &ShapeShapeCollide<Coned, Coned, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_CYLINDER] = &ShapeShapeCollide<Coned, Cylinderd, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_CONVEX] = &ShapeShapeCollide<Coned, Convexd, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_PLANE] = &ShapeShapeCollide<Coned, Planed, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_HALFSPACE] = &ShapeShapeCollide<Coned, Halfspaced, NarrowPhaseSolver>;

  collision_matrix[GEOM_CYLINDER][GEOM_BOX] = &ShapeShapeCollide<Cylinderd, Boxd, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_SPHERE] = &ShapeShapeCollide<Cylinderd, Sphered, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_ELLIPSOID] = &ShapeShapeCollide<Cylinderd, Ellipsoidd, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_CAPSULE] = &ShapeShapeCollide<Cylinderd, Capsuled, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_CONE] = &ShapeShapeCollide<Cylinderd, Coned, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_CYLINDER] = &ShapeShapeCollide<Cylinderd, Cylinderd, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_CONVEX] = &ShapeShapeCollide<Cylinderd, Convexd, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_PLANE] = &ShapeShapeCollide<Cylinderd, Planed, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_HALFSPACE] = &ShapeShapeCollide<Cylinderd, Halfspaced, NarrowPhaseSolver>;

  collision_matrix[GEOM_CONVEX][GEOM_BOX] = &ShapeShapeCollide<Convexd, Boxd, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_SPHERE] = &ShapeShapeCollide<Convexd, Sphered, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_ELLIPSOID] = &ShapeShapeCollide<Convexd, Ellipsoidd, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_CAPSULE] = &ShapeShapeCollide<Convexd, Capsuled, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_CONE] = &ShapeShapeCollide<Convexd, Coned, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_CYLINDER] = &ShapeShapeCollide<Convexd, Cylinderd, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_CONVEX] = &ShapeShapeCollide<Convexd, Convexd, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_PLANE] = &ShapeShapeCollide<Convexd, Planed, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_HALFSPACE] = &ShapeShapeCollide<Convexd, Halfspaced, NarrowPhaseSolver>;

  collision_matrix[GEOM_PLANE][GEOM_BOX] = &ShapeShapeCollide<Planed, Boxd, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_SPHERE] = &ShapeShapeCollide<Planed, Sphered, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_ELLIPSOID] = &ShapeShapeCollide<Planed, Ellipsoidd, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_CAPSULE] = &ShapeShapeCollide<Planed, Capsuled, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_CONE] = &ShapeShapeCollide<Planed, Coned, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_CYLINDER] = &ShapeShapeCollide<Planed, Cylinderd, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_CONVEX] = &ShapeShapeCollide<Planed, Convexd, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_PLANE] = &ShapeShapeCollide<Planed, Planed, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_HALFSPACE] = &ShapeShapeCollide<Planed, Halfspaced, NarrowPhaseSolver>;

  collision_matrix[GEOM_HALFSPACE][GEOM_BOX] = &ShapeShapeCollide<Halfspaced, Boxd, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_SPHERE] = &ShapeShapeCollide<Halfspaced, Sphered, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CAPSULE] = &ShapeShapeCollide<Halfspaced, Capsuled, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CONE] = &ShapeShapeCollide<Halfspaced, Coned, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CYLINDER] = &ShapeShapeCollide<Halfspaced, Cylinderd, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CONVEX] = &ShapeShapeCollide<Halfspaced, Convexd, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_PLANE] = &ShapeShapeCollide<Halfspaced, Planed, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_HALFSPACE] = &ShapeShapeCollide<Halfspaced, Halfspaced, NarrowPhaseSolver>;

  collision_matrix[BV_AABB][GEOM_BOX] = &BVHShapeCollider<AABBd, Boxd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_SPHERE] = &BVHShapeCollider<AABBd, Sphered, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_ELLIPSOID] = &BVHShapeCollider<AABBd, Ellipsoidd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_CAPSULE] = &BVHShapeCollider<AABBd, Capsuled, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_CONE] = &BVHShapeCollider<AABBd, Coned, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_CYLINDER] = &BVHShapeCollider<AABBd, Cylinderd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_CONVEX] = &BVHShapeCollider<AABBd, Convexd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_PLANE] = &BVHShapeCollider<AABBd, Planed, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_HALFSPACE] = &BVHShapeCollider<AABBd, Halfspaced, NarrowPhaseSolver>::collide;

  collision_matrix[BV_OBB][GEOM_BOX] = &BVHShapeCollider<OBBd, Boxd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_SPHERE] = &BVHShapeCollider<OBBd, Sphered, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_ELLIPSOID] = &BVHShapeCollider<OBBd, Ellipsoidd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_CAPSULE] = &BVHShapeCollider<OBBd, Capsuled, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_CONE] = &BVHShapeCollider<OBBd, Coned, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_CYLINDER] = &BVHShapeCollider<OBBd, Cylinderd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_CONVEX] = &BVHShapeCollider<OBBd, Convexd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_PLANE] = &BVHShapeCollider<OBBd, Planed, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_HALFSPACE] = &BVHShapeCollider<OBBd, Halfspaced, NarrowPhaseSolver>::collide;

  collision_matrix[BV_RSS][GEOM_BOX] = &BVHShapeCollider<RSSd, Boxd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_SPHERE] = &BVHShapeCollider<RSSd, Sphered, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_ELLIPSOID] = &BVHShapeCollider<RSSd, Ellipsoidd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_CAPSULE] = &BVHShapeCollider<RSSd, Capsuled, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_CONE] = &BVHShapeCollider<RSSd, Coned, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_CYLINDER] = &BVHShapeCollider<RSSd, Cylinderd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_CONVEX] = &BVHShapeCollider<RSSd, Convexd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_PLANE] = &BVHShapeCollider<RSSd, Planed, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_HALFSPACE] = &BVHShapeCollider<RSSd, Halfspaced, NarrowPhaseSolver>::collide;

  collision_matrix[BV_KDOP16][GEOM_BOX] = &BVHShapeCollider<KDOPd<16>, Boxd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_SPHERE] = &BVHShapeCollider<KDOPd<16>, Sphered, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_ELLIPSOID] = &BVHShapeCollider<KDOPd<16>, Ellipsoidd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_CAPSULE] = &BVHShapeCollider<KDOPd<16>, Capsuled, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_CONE] = &BVHShapeCollider<KDOPd<16>, Coned, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_CYLINDER] = &BVHShapeCollider<KDOPd<16>, Cylinderd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_CONVEX] = &BVHShapeCollider<KDOPd<16>, Convexd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_PLANE] = &BVHShapeCollider<KDOPd<16>, Planed, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_HALFSPACE] = &BVHShapeCollider<KDOPd<16>, Halfspaced, NarrowPhaseSolver>::collide;

  collision_matrix[BV_KDOP18][GEOM_BOX] = &BVHShapeCollider<KDOPd<18>, Boxd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_SPHERE] = &BVHShapeCollider<KDOPd<18>, Sphered, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_ELLIPSOID] = &BVHShapeCollider<KDOPd<18>, Ellipsoidd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_CAPSULE] = &BVHShapeCollider<KDOPd<18>, Capsuled, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_CONE] = &BVHShapeCollider<KDOPd<18>, Coned, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_CYLINDER] = &BVHShapeCollider<KDOPd<18>, Cylinderd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_CONVEX] = &BVHShapeCollider<KDOPd<18>, Convexd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_PLANE] = &BVHShapeCollider<KDOPd<18>, Planed, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_HALFSPACE] = &BVHShapeCollider<KDOPd<18>, Halfspaced, NarrowPhaseSolver>::collide;

  collision_matrix[BV_KDOP24][GEOM_BOX] = &BVHShapeCollider<KDOPd<24>, Boxd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_SPHERE] = &BVHShapeCollider<KDOPd<24>, Sphered, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_ELLIPSOID] = &BVHShapeCollider<KDOPd<24>, Ellipsoidd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_CAPSULE] = &BVHShapeCollider<KDOPd<24>, Capsuled, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_CONE] = &BVHShapeCollider<KDOPd<24>, Coned, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_CYLINDER] = &BVHShapeCollider<KDOPd<24>, Cylinderd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_CONVEX] = &BVHShapeCollider<KDOPd<24>, Convexd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_PLANE] = &BVHShapeCollider<KDOPd<24>, Planed, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_HALFSPACE] = &BVHShapeCollider<KDOPd<24>, Halfspaced, NarrowPhaseSolver>::collide;

  collision_matrix[BV_kIOS][GEOM_BOX] = &BVHShapeCollider<kIOSd, Boxd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_SPHERE] = &BVHShapeCollider<kIOSd, Sphered, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_ELLIPSOID] = &BVHShapeCollider<kIOSd, Ellipsoidd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_CAPSULE] = &BVHShapeCollider<kIOSd, Capsuled, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_CONE] = &BVHShapeCollider<kIOSd, Coned, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_CYLINDER] = &BVHShapeCollider<kIOSd, Cylinderd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_CONVEX] = &BVHShapeCollider<kIOSd, Convexd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_PLANE] = &BVHShapeCollider<kIOSd, Planed, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_HALFSPACE] = &BVHShapeCollider<kIOSd, Halfspaced, NarrowPhaseSolver>::collide;

  collision_matrix[BV_OBBRSS][GEOM_BOX] = &BVHShapeCollider<OBBRSSd, Boxd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_SPHERE] = &BVHShapeCollider<OBBRSSd, Sphered, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_ELLIPSOID] = &BVHShapeCollider<OBBRSSd, Ellipsoidd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CAPSULE] = &BVHShapeCollider<OBBRSSd, Capsuled, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CONE] = &BVHShapeCollider<OBBRSSd, Coned, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CYLINDER] = &BVHShapeCollider<OBBRSSd, Cylinderd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CONVEX] = &BVHShapeCollider<OBBRSSd, Convexd, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_PLANE] = &BVHShapeCollider<OBBRSSd, Planed, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_HALFSPACE] = &BVHShapeCollider<OBBRSSd, Halfspaced, NarrowPhaseSolver>::collide;

  collision_matrix[BV_AABB][BV_AABB] = &BVHCollide<AABBd, NarrowPhaseSolver>;
  collision_matrix[BV_OBB][BV_OBB] = &BVHCollide<OBBd, NarrowPhaseSolver>;
  collision_matrix[BV_RSS][BV_RSS] = &BVHCollide<RSSd, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP16][BV_KDOP16] = &BVHCollide<KDOPd<16>, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP18][BV_KDOP18] = &BVHCollide<KDOPd<18>, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP24][BV_KDOP24] = &BVHCollide<KDOPd<24>, NarrowPhaseSolver>;
  collision_matrix[BV_kIOS][BV_kIOS] = &BVHCollide<kIOSd, NarrowPhaseSolver>;
  collision_matrix[BV_OBBRSS][BV_OBBRSS] = &BVHCollide<OBBRSSd, NarrowPhaseSolver>;

#if FCL_HAVE_OCTOMAP
  collision_matrix[GEOM_OCTREE][GEOM_BOX] = &OcTreeShapeCollide<Boxd, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_SPHERE] = &OcTreeShapeCollide<Sphered, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_ELLIPSOID] = &OcTreeShapeCollide<Ellipsoidd, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_CAPSULE] = &OcTreeShapeCollide<Capsuled, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_CONE] = &OcTreeShapeCollide<Coned, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_CYLINDER] = &OcTreeShapeCollide<Cylinderd, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_CONVEX] = &OcTreeShapeCollide<Convexd, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_PLANE] = &OcTreeShapeCollide<Planed, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_HALFSPACE] = &OcTreeShapeCollide<Halfspaced, NarrowPhaseSolver>;

  collision_matrix[GEOM_BOX][GEOM_OCTREE] = &ShapeOcTreeCollide<Boxd, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_OCTREE] = &ShapeOcTreeCollide<Sphered, NarrowPhaseSolver>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_OCTREE] = &ShapeOcTreeCollide<Ellipsoidd, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_OCTREE] = &ShapeOcTreeCollide<Capsuled, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_OCTREE] = &ShapeOcTreeCollide<Coned, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_OCTREE] = &ShapeOcTreeCollide<Cylinderd, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_OCTREE] = &ShapeOcTreeCollide<Convexd, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_OCTREE] = &ShapeOcTreeCollide<Planed, NarrowPhaseSolver>;
  collision_matrix[GEOM_HALFSPACE][GEOM_OCTREE] = &ShapeOcTreeCollide<Halfspaced, NarrowPhaseSolver>;

  collision_matrix[GEOM_OCTREE][GEOM_OCTREE] = &OcTreeCollide<NarrowPhaseSolver>;

  collision_matrix[GEOM_OCTREE][BV_AABB] = &OcTreeBVHCollide<AABBd, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_OBB] = &OcTreeBVHCollide<OBBd, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_RSS] = &OcTreeBVHCollide<RSSd, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_OBBRSS] = &OcTreeBVHCollide<OBBRSSd, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_kIOS] = &OcTreeBVHCollide<kIOSd, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_KDOP16] = &OcTreeBVHCollide<KDOPd<16>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_KDOP18] = &OcTreeBVHCollide<KDOPd<18>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_KDOP24] = &OcTreeBVHCollide<KDOPd<24>, NarrowPhaseSolver>;

  collision_matrix[BV_AABB][GEOM_OCTREE] = &BVHOcTreeCollide<AABBd, NarrowPhaseSolver>;
  collision_matrix[BV_OBB][GEOM_OCTREE] = &BVHOcTreeCollide<OBBd, NarrowPhaseSolver>;
  collision_matrix[BV_RSS][GEOM_OCTREE] = &BVHOcTreeCollide<RSSd, NarrowPhaseSolver>;
  collision_matrix[BV_OBBRSS][GEOM_OCTREE] = &BVHOcTreeCollide<OBBRSSd, NarrowPhaseSolver>;
  collision_matrix[BV_kIOS][GEOM_OCTREE] = &BVHOcTreeCollide<kIOSd, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP16][GEOM_OCTREE] = &BVHOcTreeCollide<KDOPd<16>, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP18][GEOM_OCTREE] = &BVHOcTreeCollide<KDOPd<18>, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP24][GEOM_OCTREE] = &BVHOcTreeCollide<KDOPd<24>, NarrowPhaseSolver>;
#endif
}


template struct CollisionFunctionMatrix<GJKSolver_libccd>;
template struct CollisionFunctionMatrix<GJKSolver_indep>;


}

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

#include "fcl/ccd/conservative_advancement.h"
#include "fcl/ccd/motion.h"
#include "fcl/collision_node.h"
#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/traversal/traversal_recurse.h"
#include "fcl/collision.h"


namespace fcl
{




template<typename BV>
bool conservativeAdvancement(const BVHModel<BV>& o1,
                             const MotionBase* motion1,
                             const BVHModel<BV>& o2,
                             const MotionBase* motion2,
                             const CollisionRequest& request,
                             CollisionResult& result,
                             FCL_REAL& toc)
{
  Transform3f tf1, tf2;
  motion1->getCurrentTransform(tf1);
  motion2->getCurrentTransform(tf2);

  // whether the first start configuration is in collision
  if(collide(&o1, tf1, &o2, tf2, request, result))
  {
    toc = 0;
    return true;
  }


  BVHModel<BV>* o1_tmp = new BVHModel<BV>(o1);
  BVHModel<BV>* o2_tmp = new BVHModel<BV>(o2);
  
  
  MeshConservativeAdvancementTraversalNode<BV> node;

  node.motion1 = motion1;
  node.motion2 = motion2;

  do
  {
    // repeatedly update mesh to global coordinate, so time consuming
    initialize(node, *o1_tmp, tf1, *o2_tmp, tf2);
    
    node.delta_t = 1;
    node.min_distance = std::numeric_limits<FCL_REAL>::max();

    distanceRecurse(&node, 0, 0, NULL);

    if(node.delta_t <= node.t_err)
    {
      // std::cout << node.delta_t << " " << node.t_err << std::endl;
      break;
    }

    node.toc += node.delta_t;
    if(node.toc > 1)
    {
      node.toc = 1;
      break;
    }

    node.motion1->integrate(node.toc);
    node.motion2->integrate(node.toc);

    motion1->getCurrentTransform(tf1);
    motion2->getCurrentTransform(tf2);
  }
  while(1);

  delete o1_tmp;
  delete o2_tmp;

  toc = node.toc;

  if(node.toc < 1)
    return true;

  return false;
}

namespace details
{

template<typename BV, typename ConservativeAdvancementOrientedNode>
bool conservativeAdvancementMeshOriented(const BVHModel<BV>& o1,
                                         const MotionBase* motion1,
                                         const BVHModel<BV>& o2,
                                         const MotionBase* motion2,
                                         const CollisionRequest& request,
                                         CollisionResult& result,
                                         FCL_REAL& toc)
{
  Transform3f tf1, tf2;
  motion1->getCurrentTransform(tf1);
  motion2->getCurrentTransform(tf2);

  // whether the first start configuration is in collision
  if(collide(&o1, tf1, &o2, tf2, request, result))
  {
    toc = 0;
    return true;
  }
  
  
  ConservativeAdvancementOrientedNode node;

  initialize(node, o1, tf1, o2, tf2);

  node.motion1 = motion1;
  node.motion2 = motion2;

  do
  {
    node.motion1->getCurrentTransform(tf1);
    node.motion2->getCurrentTransform(tf2);

    // compute the transformation from 1 to 2
    Transform3f tf;
    relativeTransform(tf1, tf2, tf);
    node.R = tf.getRotation();
    node.T = tf.getTranslation();
    
    node.delta_t = 1;
    node.min_distance = std::numeric_limits<FCL_REAL>::max();

    distanceRecurse(&node, 0, 0, NULL);

    if(node.delta_t <= node.t_err)
    {
      // std::cout << node.delta_t << " " << node.t_err << std::endl;
      break;
    }

    node.toc += node.delta_t;
    if(node.toc > 1)
    {
      node.toc = 1;
      break;
    }

    node.motion1->integrate(node.toc);
    node.motion2->integrate(node.toc);
  }
  while(1);

  toc = node.toc;

  if(node.toc < 1)
    return true;

  return false;
}


}

template<>
bool conservativeAdvancement(const BVHModel<RSS>& o1,
                             const MotionBase* motion1,
                             const BVHModel<RSS>& o2,
                             const MotionBase* motion2,
                             const CollisionRequest& request,
                             CollisionResult& result,
                             FCL_REAL& toc);


template<>
bool conservativeAdvancement(const BVHModel<OBBRSS>& o1,
                             const MotionBase* motion1,
                             const BVHModel<OBBRSS>& o2,
                             const MotionBase* motion2,
                             const CollisionRequest& request,
                             CollisionResult& result,
                             FCL_REAL& toc);

template<typename S1, typename S2, typename NarrowPhaseSolver>
bool conservativeAdvancement(const S1& o1,
                             const MotionBase* motion1,
                             const S2& o2,
                             const MotionBase* motion2,
                             const NarrowPhaseSolver* solver,
                             const CollisionRequest& request,
                             CollisionResult& result,
                             FCL_REAL& toc)
{
  Transform3f tf1, tf2;
  motion1->getCurrentTransform(tf1);
  motion2->getCurrentTransform(tf2);

  // whether the first start configuration is in collision
  if(collide(&o1, tf1, &o2, tf2, request, result))
  {
    toc = 0;
    return true;
  }

  ShapeConservativeAdvancementTraversalNode<S1, S2, NarrowPhaseSolver> node;

  initialize(node, o1, tf1, o2, tf2, solver);

  node.motion1 = motion1;
  node.motion2 = motion2;

  do
  {
    motion1->getCurrentTransform(tf1);
    motion2->getCurrentTransform(tf2);
    node.tf1 = tf1;
    node.tf2 = tf2;
    
    node.delta_t = 1;
    node.min_distance = std::numeric_limits<FCL_REAL>::max();

    distanceRecurse(&node, 0, 0, NULL);

    if(node.delta_t <= node.t_err)
    {
      // std::cout << node.delta_t << " " << node.t_err << std::endl;
      break;
    }

    node.toc += node.delta_t;
    if(node.toc > 1)
    {
      node.toc = 1;
      break;
    }

    node.motion1->integrate(node.toc);
    node.motion2->integrate(node.toc);
  }
  while(1);

  toc = node.toc;

  if(node.toc < 1)
    return true;

  return false;
}

template<typename BV, typename S, typename NarrowPhaseSolver>
bool conservativeAdvancement(const BVHModel<BV>& o1,
                             const MotionBase* motion1,
                             const S& o2,
                             const MotionBase* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest& request,
                             CollisionResult& result,
                             FCL_REAL& toc)
{
  Transform3f tf1, tf2;
  motion1->getCurrentTransform(tf1);
  motion2->getCurrentTransform(tf2);

  if(collide(&o1, tf1, &o2, tf2, request, result))
  {
    toc = 0;
    return true;
  }

  BVHModel<BV>* o1_tmp = new BVHModel<BV>(o1);

  MeshShapeConservativeAdvancementTraversalNode<BV, S, NarrowPhaseSolver> node;

  node.motion1 = motion1;
  node.motion2 = motion2;

  do
  {
    // initialize update mesh to global coordinate, so time consuming
    initialize(node, *o1_tmp, tf1, o2, tf2, nsolver);

    node.delta_t = 1;
    node.min_distance = std::numeric_limits<FCL_REAL>::max();

    distanceRecurse(&node, 0, 0, NULL);

    if(node.delta_t <= node.t_err)
    {
      break;
    }

    node.toc += node.delta_t;
    if(node.toc > 1)
    {
      node.toc = 1;
      break;
    }

    node.motion1->integrate(node.toc);
    node.motion2->integrate(node.toc);

    motion1->getCurrentTransform(tf1);
    motion2->getCurrentTransform(tf2);
  }
  while(1);

  delete o1_tmp;

  toc = node.toc;

  if(node.toc < 1)
    return true;

  return false;
}

namespace details
{

template<typename BV, typename S, typename NarrowPhaseSolver, typename ConservativeAdvancementOrientedNode>
bool conservativeAdvancementMeshShapeOriented(const BVHModel<BV>& o1,
                                              const MotionBase* motion1,
                                              const S& o2,
                                              const MotionBase* motion2,
                                              const NarrowPhaseSolver* nsolver,
                                              const CollisionRequest& request,
                                              CollisionResult& result,
                                              FCL_REAL& toc)
{
  Transform3f tf1, tf2;
  motion1->getCurrentTransform(tf1);
  motion2->getCurrentTransform(tf2);

  if(collide(&o1, tf1, &o2, tf2, request, result))
  {
    toc = 0;
    return true;
  }

  ConservativeAdvancementOrientedNode node;

  initialize(node, o1, tf1, o2, tf2, nsolver);

  node.motion1 = motion1;
  node.motion2 = motion2;

  do
  {
    node.motion1->getCurrentTransform(tf1);
    node.motion2->getCurrentTransform(tf2);

    node.tf1 = tf1;
    node.tf2 = tf2;
    
    node.delta_t = 1;
    node.min_distance = std::numeric_limits<FCL_REAL>::max();

    distanceRecurse(&node, 0, 0, NULL);

    if(node.delta_t <= node.t_err)
    {
      break;
    }

    node.toc += node.delta_t;
    if(node.toc > 1)
    {
      node.toc = 1;
      break;
    }

    node.motion1->integrate(node.toc);
    node.motion2->integrate(node.toc);
  }
  while(1);

  toc = node.toc;

  if(node.toc < 1)
    return true;

  return false;
}

}


template<typename S, typename NarrowPhaseSolver>
bool conservativeAdvancement(const BVHModel<RSS>& o1,
                             const MotionBase* motion1,
                             const S& o2,
                             const MotionBase* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest& request,
                             CollisionResult& result,
                             FCL_REAL& toc)
{
  return details::conservativeAdvancementMeshShapeOriented<RSS, S, NarrowPhaseSolver, MeshShapeConservativeAdvancementTraversalNodeRSS<S, NarrowPhaseSolver> >(o1, motion1, o2, motion2, nsolver, request, result, toc);
}

template<typename S, typename NarrowPhaseSolver>
bool conservativeAdvancement(const BVHModel<OBBRSS>& o1,
                             const MotionBase* motion1,
                             const S& o2,
                             const MotionBase* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest& request,
                             CollisionResult& result,
                             FCL_REAL& toc)
{
  return details::conservativeAdvancementMeshShapeOriented<OBBRSS, S, NarrowPhaseSolver, MeshShapeConservativeAdvancementTraversalNodeOBBRSS<S, NarrowPhaseSolver> >(o1, motion1, o2, motion2, nsolver, request, result, toc);  
}
                            
template<typename S, typename BV, typename NarrowPhaseSolver>
bool conservativeAdvancement(const S& o1,
                             const MotionBase* motion1,
                             const BVHModel<BV>& o2,
                             const MotionBase* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest& request,
                             CollisionResult& result,
                             FCL_REAL& toc)
{
  Transform3f tf1, tf2;
  motion1->getCurrentTransform(tf1);
  motion2->getCurrentTransform(tf2);

  if(collide(&o1, tf1, &o2, tf2, request, result))
  {
    toc = 0;
    return true;
  }

  BVHModel<BV>* o2_tmp = new BVHModel<BV>(o2);

  ShapeMeshConservativeAdvancementTraversalNode<S, BV, NarrowPhaseSolver> node;

  node.motion1 = motion1;
  node.motion2 = motion2;

  do
  {
    // initialize update mesh to global coordinate, so time consuming
    initialize(node, o1, tf1, *o2_tmp, tf2, nsolver);

    node.delta_t = 1;
    node.min_distance = std::numeric_limits<FCL_REAL>::max();

    distanceRecurse(&node, 0, 0, NULL);

    if(node.delta_t <= node.t_err)
    {
      break;
    }

    node.toc += node.delta_t;
    if(node.toc > 1)
    {
      node.toc = 1;
      break;
    }

    node.motion1->integrate(node.toc);
    node.motion2->integrate(node.toc);

    motion1->getCurrentTransform(tf1);
    motion2->getCurrentTransform(tf2);
  }
  while(1);

  delete o2_tmp;

  toc = node.toc;

  if(node.toc < 1)
    return true;

  return false;  
}

namespace details
{

template<typename S, typename BV, typename NarrowPhaseSolver, typename ConservativeAdvancementOrientedNode>
bool conservativeAdvancementShapeMeshOriented(const S& o1,
                                              const MotionBase* motion1,
                                              const BVHModel<BV>& o2,
                                              const MotionBase* motion2,
                                              const NarrowPhaseSolver* nsolver,
                                              const CollisionRequest& request,
                                              CollisionResult& result,
                                              FCL_REAL& toc)
{
  Transform3f tf1, tf2;
  motion1->getCurrentTransform(tf1);
  motion2->getCurrentTransform(tf2);

  if(collide(&o1, tf1, &o2, tf2, request, result))
  {
    toc = 0;
    return true;
  }

  ConservativeAdvancementOrientedNode node;

  initialize(node, o1, tf1, o2, tf2, nsolver);

  node.motion1 = motion1;
  node.motion2 = motion2;

  do
  {
    node.motion1->getCurrentTransform(tf1);
    node.motion2->getCurrentTransform(tf2);

    node.tf1 = tf1;
    node.tf2 = tf2;
    
    node.delta_t = 1;
    node.min_distance = std::numeric_limits<FCL_REAL>::max();

    distanceRecurse(&node, 0, 0, NULL);

    if(node.delta_t <= node.t_err)
    {
      break;
    }

    node.toc += node.delta_t;
    if(node.toc > 1)
    {
      node.toc = 1;
      break;
    }

    node.motion1->integrate(node.toc);
    node.motion2->integrate(node.toc);
  }
  while(1);

  toc = node.toc;

  if(node.toc < 1)
    return true;

  return false;
}

}

template<typename S, typename NarrowPhaseSolver>
bool conservativeAdvancement(const S& o1,
                             const MotionBase* motion1,
                             const BVHModel<RSS>& o2,
                             const MotionBase* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest& request,
                             CollisionResult& result,
                             FCL_REAL& toc)
{
  return details::conservativeAdvancementShapeMeshOriented<S, RSS, NarrowPhaseSolver, ShapeMeshConservativeAdvancementTraversalNodeRSS<S, NarrowPhaseSolver> >(o1, motion1, o2, motion2, nsolver, request, result, toc);
}


template<typename S, typename NarrowPhaseSolver>
bool conservativeAdvancement(const S& o1,
                             const MotionBase* motion1,
                             const BVHModel<OBBRSS>& o2,
                             const MotionBase* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest& request,
                             CollisionResult& result,
                             FCL_REAL& toc)
{
  return details::conservativeAdvancementShapeMeshOriented<S, OBBRSS, NarrowPhaseSolver, ShapeMeshConservativeAdvancementTraversalNodeOBBRSS<S, NarrowPhaseSolver> >(o1, motion1, o2, motion2, nsolver, request, result, toc);
}



template<>
bool conservativeAdvancement(const BVHModel<RSS>& o1,
                             const MotionBase* motion1,
                             const BVHModel<RSS>& o2,
                             const MotionBase* motion2,
                             const CollisionRequest& request,
                             CollisionResult& result,
                             FCL_REAL& toc)
{
  return details::conservativeAdvancementMeshOriented<RSS, MeshConservativeAdvancementTraversalNodeRSS>(o1, motion1, o2, motion2, request, result, toc);
}

template<>
bool conservativeAdvancement(const BVHModel<OBBRSS>& o1,
                             const MotionBase* motion1,
                             const BVHModel<OBBRSS>& o2,
                             const MotionBase* motion2,
                             const CollisionRequest& request,
                             CollisionResult& result,
                             FCL_REAL& toc)
{
  return details::conservativeAdvancementMeshOriented<OBBRSS, MeshConservativeAdvancementTraversalNodeOBBRSS>(o1, motion1, o2, motion2, request, result, toc);
}


template<typename BV, typename NarrowPhaseSolver>
FCL_REAL BVHConservativeAdvancement(const CollisionGeometry* o1, const MotionBase* motion1, const CollisionGeometry* o2, const MotionBase* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequest& request, ContinuousCollisionResult& result)
{
  const BVHModel<BV>* obj1 = static_cast<const BVHModel<BV>*>(o1);
  const BVHModel<BV>* obj2 = static_cast<const BVHModel<BV>*>(o2);

  CollisionRequest c_request;
  CollisionResult c_result;
  FCL_REAL toc;
  bool is_collide = conservativeAdvancement(*obj1, motion1, *obj2, motion2, c_request, c_result, toc);

  result.is_collide = is_collide;
  result.time_of_contact = toc;
  
  return toc;
}

template<typename S1, typename S2, typename NarrowPhaseSolver>
FCL_REAL ShapeConservativeAdvancement(const CollisionGeometry* o1, const MotionBase* motion1, const CollisionGeometry* o2, const MotionBase* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequest& request, ContinuousCollisionResult& result)
{
  const S1* obj1 = static_cast<const S1*>(o1);
  const S2* obj2 = static_cast<const S2*>(o2);

  CollisionRequest c_request;
  CollisionResult c_result;
  FCL_REAL toc;
  bool is_collide = conservativeAdvancement(*obj1, motion1, *obj2, motion2, nsolver, c_request, c_result, toc);

  result.is_collide = is_collide;
  result.time_of_contact = toc;

  return toc;
}

template<typename S, typename BV, typename NarrowPhaseSolver>
FCL_REAL ShapeBVHConservativeAdvancement(const CollisionGeometry* o1, const MotionBase* motion1, const CollisionGeometry* o2, const MotionBase* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequest& request, ContinuousCollisionResult& result)
{
  const S* obj1 = static_cast<const S*>(o1);
  const BVHModel<BV>* obj2 = static_cast<const BVHModel<BV>*>(o2);

  CollisionRequest c_request;
  CollisionResult c_result;
  FCL_REAL toc;

  bool is_collide = conservativeAdvancement(*obj1, motion1, *obj2, motion2, nsolver, c_request, c_result, toc);

  result.is_collide = is_collide;
  result.time_of_contact = toc;

  return toc;
}

template<typename BV, typename S, typename NarrowPhaseSolver>
FCL_REAL BVHShapeConservativeAdvancement(const CollisionGeometry* o1, const MotionBase* motion1, const CollisionGeometry* o2, const MotionBase* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequest& request, ContinuousCollisionResult& result)
{
  const BVHModel<BV>* obj1 = static_cast<const BVHModel<BV>*>(o1);
  const S* obj2 = static_cast<const S*>(o2);

  CollisionRequest c_request;
  CollisionResult c_result;
  FCL_REAL toc;

  bool is_collide = conservativeAdvancement(*obj1, motion1, *obj2, motion2, nsolver, c_request, c_result, toc);

  result.is_collide = is_collide;
  result.time_of_contact = toc;

  return toc;
}

template<typename NarrowPhaseSolver>
ConservativeAdvancementFunctionMatrix<NarrowPhaseSolver>::ConservativeAdvancementFunctionMatrix()
{
  for(int i = 0; i < NODE_COUNT; ++i)
  {
    for(int j = 0; j < NODE_COUNT; ++j)
      conservative_advancement_matrix[i][j] = NULL;
  }


  conservative_advancement_matrix[GEOM_BOX][GEOM_BOX] = &ShapeConservativeAdvancement<Box, Box, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_SPHERE] = &ShapeConservativeAdvancement<Box, Sphere, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Box, Capsule, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_CONE] = &ShapeConservativeAdvancement<Box, Cone, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Box, Cylinder, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_CONVEX] = &ShapeConservativeAdvancement<Box, Convex, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_PLANE] = &ShapeConservativeAdvancement<Box, Plane, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Box, Halfspace, NarrowPhaseSolver>;
    
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_BOX] = &ShapeConservativeAdvancement<Sphere, Box, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_SPHERE] = &ShapeConservativeAdvancement<Sphere, Sphere, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Sphere, Capsule, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_CONE] = &ShapeConservativeAdvancement<Sphere, Cone, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Sphere, Cylinder, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_CONVEX] = &ShapeConservativeAdvancement<Sphere, Convex, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_PLANE] = &ShapeConservativeAdvancement<Sphere, Plane, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Sphere, Halfspace, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_BOX] = &ShapeConservativeAdvancement<Capsule, Box, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_SPHERE] = &ShapeConservativeAdvancement<Capsule, Sphere, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Capsule, Capsule, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_CONE] = &ShapeConservativeAdvancement<Capsule, Cone, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Capsule, Cylinder, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_CONVEX] = &ShapeConservativeAdvancement<Capsule, Convex, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_PLANE] = &ShapeConservativeAdvancement<Capsule, Plane, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Capsule, Halfspace, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_CONE][GEOM_BOX] = &ShapeConservativeAdvancement<Cone, Box, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_SPHERE] = &ShapeConservativeAdvancement<Cone, Sphere, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Cone, Capsule, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_CONE] = &ShapeConservativeAdvancement<Cone, Cone, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Cone, Cylinder, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_CONVEX] = &ShapeConservativeAdvancement<Cone, Convex, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_PLANE] = &ShapeConservativeAdvancement<Cone, Plane, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Cone, Halfspace, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_BOX] = &ShapeConservativeAdvancement<Cylinder, Box, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_SPHERE] = &ShapeConservativeAdvancement<Cylinder, Sphere, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Cylinder, Capsule, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_CONE] = &ShapeConservativeAdvancement<Cylinder, Cone, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Cylinder, Cylinder, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_CONVEX] = &ShapeConservativeAdvancement<Cylinder, Convex, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_PLANE] = &ShapeConservativeAdvancement<Cylinder, Plane, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Cylinder, Halfspace, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_CONVEX][GEOM_BOX] = &ShapeConservativeAdvancement<Convex, Box, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_SPHERE] = &ShapeConservativeAdvancement<Convex, Sphere, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Convex, Capsule, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_CONE] = &ShapeConservativeAdvancement<Convex, Cone, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Convex, Cylinder, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_CONVEX] = &ShapeConservativeAdvancement<Convex, Convex, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_PLANE] = &ShapeConservativeAdvancement<Convex, Plane, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Convex, Halfspace, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_PLANE][GEOM_BOX] = &ShapeConservativeAdvancement<Plane, Box, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_SPHERE] = &ShapeConservativeAdvancement<Plane, Sphere, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Plane, Capsule, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_CONE] = &ShapeConservativeAdvancement<Plane, Cone, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Plane, Cylinder, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_CONVEX] = &ShapeConservativeAdvancement<Plane, Convex, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_PLANE] = &ShapeConservativeAdvancement<Plane, Plane, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Plane, Halfspace, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_BOX] = &ShapeConservativeAdvancement<Halfspace, Box, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_SPHERE] = &ShapeConservativeAdvancement<Halfspace, Sphere, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Halfspace, Capsule, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_CONE] = &ShapeConservativeAdvancement<Halfspace, Cone, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Halfspace, Cylinder, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_CONVEX] = &ShapeConservativeAdvancement<Halfspace, Convex, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_PLANE] = &ShapeConservativeAdvancement<Halfspace, Plane, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Halfspace, Halfspace, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_AABB][GEOM_BOX] = &BVHShapeConservativeAdvancement<AABB, Box, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<AABB, Sphere, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<AABB, Capsule, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_CONE] = &BVHShapeConservativeAdvancement<AABB, Cone, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<AABB, Cylinder, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<AABB, Convex, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_PLANE] = &BVHShapeConservativeAdvancement<AABB, Plane, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<AABB, Halfspace, NarrowPhaseSolver>;
  
  conservative_advancement_matrix[BV_OBB][GEOM_BOX] = &BVHShapeConservativeAdvancement<OBB, Box, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<OBB, Sphere, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<OBB, Capsule, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_CONE] = &BVHShapeConservativeAdvancement<OBB, Cone, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<OBB, Cylinder, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<OBB, Convex, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_PLANE] = &BVHShapeConservativeAdvancement<OBB, Plane, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<OBB, Halfspace, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_OBBRSS][GEOM_BOX] = &BVHShapeConservativeAdvancement<OBBRSS, Box, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<OBBRSS, Sphere, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<OBBRSS, Capsule, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_CONE] = &BVHShapeConservativeAdvancement<OBBRSS, Cone, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<OBBRSS, Cylinder, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<OBBRSS, Convex, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_PLANE] = &BVHShapeConservativeAdvancement<OBBRSS, Plane, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<OBBRSS, Halfspace, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_RSS][GEOM_BOX] = &BVHShapeConservativeAdvancement<RSS, Box, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<RSS, Sphere, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<RSS, Capsule, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_CONE] = &BVHShapeConservativeAdvancement<RSS, Cone, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<RSS, Cylinder, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<RSS, Convex, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_PLANE] = &BVHShapeConservativeAdvancement<RSS, Plane, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<RSS, Halfspace, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_KDOP16][GEOM_BOX] = &BVHShapeConservativeAdvancement<KDOP<16>, Box, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<KDOP<16>, Sphere, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<KDOP<16>, Capsule, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_CONE] = &BVHShapeConservativeAdvancement<KDOP<16>, Cone, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<KDOP<16>, Cylinder, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<KDOP<16>, Convex, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_PLANE] = &BVHShapeConservativeAdvancement<KDOP<16>, Plane, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<KDOP<16>, Halfspace, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_KDOP18][GEOM_BOX] = &BVHShapeConservativeAdvancement<KDOP<18>, Box, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<KDOP<18>, Sphere, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<KDOP<18>, Capsule, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_CONE] = &BVHShapeConservativeAdvancement<KDOP<18>, Cone, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<KDOP<18>, Cylinder, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<KDOP<18>, Convex, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_PLANE] = &BVHShapeConservativeAdvancement<KDOP<18>, Plane, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<KDOP<18>, Halfspace, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_KDOP24][GEOM_BOX] = &BVHShapeConservativeAdvancement<KDOP<24>, Box, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<KDOP<24>, Sphere, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<KDOP<24>, Capsule, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_CONE] = &BVHShapeConservativeAdvancement<KDOP<24>, Cone, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<KDOP<24>, Cylinder, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<KDOP<24>, Convex, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_PLANE] = &BVHShapeConservativeAdvancement<KDOP<24>, Plane, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<KDOP<24>, Halfspace, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_kIOS][GEOM_BOX] = &BVHShapeConservativeAdvancement<kIOS, Box, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<kIOS, Sphere, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<kIOS, Capsule, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_CONE] = &BVHShapeConservativeAdvancement<kIOS, Cone, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<kIOS, Cylinder, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<kIOS, Convex, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_PLANE] = &BVHShapeConservativeAdvancement<kIOS, Plane, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<kIOS, Halfspace, NarrowPhaseSolver>;


  conservative_advancement_matrix[GEOM_BOX][BV_AABB] = &ShapeBVHConservativeAdvancement<Box, AABB, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_AABB] = &ShapeBVHConservativeAdvancement<Sphere, AABB, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_AABB] = &ShapeBVHConservativeAdvancement<Capsule, AABB, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_AABB] = &ShapeBVHConservativeAdvancement<Cone, AABB, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_AABB] = &ShapeBVHConservativeAdvancement<Cylinder, AABB, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_AABB] = &ShapeBVHConservativeAdvancement<Convex, AABB, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_AABB] = &ShapeBVHConservativeAdvancement<Plane, AABB, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_AABB] = &ShapeBVHConservativeAdvancement<Halfspace, AABB, NarrowPhaseSolver>;
  
  conservative_advancement_matrix[GEOM_BOX][BV_OBB] = &ShapeBVHConservativeAdvancement<Box, OBB, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_OBB] = &ShapeBVHConservativeAdvancement<Sphere, OBB, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_OBB] = &ShapeBVHConservativeAdvancement<Capsule, OBB, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_OBB] = &ShapeBVHConservativeAdvancement<Cone, OBB, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_OBB] = &ShapeBVHConservativeAdvancement<Cylinder, OBB, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_OBB] = &ShapeBVHConservativeAdvancement<Convex, OBB, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_OBB] = &ShapeBVHConservativeAdvancement<Plane, OBB, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_OBB] = &ShapeBVHConservativeAdvancement<Halfspace, OBB, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_RSS] = &ShapeBVHConservativeAdvancement<Box, RSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_RSS] = &ShapeBVHConservativeAdvancement<Sphere, RSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_RSS] = &ShapeBVHConservativeAdvancement<Capsule, RSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_RSS] = &ShapeBVHConservativeAdvancement<Cone, RSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_RSS] = &ShapeBVHConservativeAdvancement<Cylinder, RSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_RSS] = &ShapeBVHConservativeAdvancement<Convex, RSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_RSS] = &ShapeBVHConservativeAdvancement<Plane, RSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_RSS] = &ShapeBVHConservativeAdvancement<Halfspace, RSS, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Box, OBBRSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Sphere, OBBRSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Capsule, OBBRSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Cone, OBBRSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Cylinder, OBBRSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Convex, OBBRSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Plane, OBBRSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Halfspace, OBBRSS, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Box, KDOP<16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Sphere, KDOP<16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Capsule, KDOP<16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Cone, KDOP<16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Cylinder, KDOP<16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Convex, KDOP<16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Plane, KDOP<16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Halfspace, KDOP<16>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Box, KDOP<18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Sphere, KDOP<18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Capsule, KDOP<18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Cone, KDOP<18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Cylinder, KDOP<18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Convex, KDOP<18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Plane, KDOP<18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Halfspace, KDOP<18>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Box, KDOP<24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Sphere, KDOP<24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Capsule, KDOP<24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Cone, KDOP<24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Cylinder, KDOP<24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Convex, KDOP<24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Plane, KDOP<24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Halfspace, KDOP<24>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_kIOS] = &ShapeBVHConservativeAdvancement<Box, kIOS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Sphere, kIOS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Capsule, kIOS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Cone, kIOS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_kIOS] = &ShapeBVHConservativeAdvancement<Cylinder, kIOS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_kIOS] = &ShapeBVHConservativeAdvancement<Convex, kIOS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Plane, kIOS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Halfspace, kIOS, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_AABB][BV_AABB] = &BVHConservativeAdvancement<AABB, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][BV_OBB] = &BVHConservativeAdvancement<OBB, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][BV_RSS] = &BVHConservativeAdvancement<RSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][BV_OBBRSS] = &BVHConservativeAdvancement<OBBRSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][BV_KDOP16] = &BVHConservativeAdvancement<KDOP<16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][BV_KDOP18] = &BVHConservativeAdvancement<KDOP<18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][BV_KDOP24] = &BVHConservativeAdvancement<KDOP<24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][BV_kIOS] = &BVHConservativeAdvancement<kIOS, NarrowPhaseSolver>;
  
}


template struct ConservativeAdvancementFunctionMatrix<GJKSolver_libccd>;
template struct ConservativeAdvancementFunctionMatrix<GJKSolver_indep>;









}



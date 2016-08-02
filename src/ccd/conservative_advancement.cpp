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
  Transform3d tf1, tf2;
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
  Transform3d tf1, tf2;
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
    Transform3d tf = tf1.inverse() * tf2;
    node.R = tf.linear();
    node.T = tf.translation();
    
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
  Transform3d tf1, tf2;
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
  Transform3d tf1, tf2;
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
  Transform3d tf1, tf2;
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
  Transform3d tf1, tf2;
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
  Transform3d tf1, tf2;
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
FCL_REAL BVHConservativeAdvancement(const CollisionGeometryd* o1, const MotionBase* motion1, const CollisionGeometryd* o2, const MotionBase* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequest& request, ContinuousCollisionResult& result)
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
FCL_REAL ShapeConservativeAdvancement(const CollisionGeometryd* o1, const MotionBase* motion1, const CollisionGeometryd* o2, const MotionBase* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequest& request, ContinuousCollisionResult& result)
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
FCL_REAL ShapeBVHConservativeAdvancement(const CollisionGeometryd* o1, const MotionBase* motion1, const CollisionGeometryd* o2, const MotionBase* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequest& request, ContinuousCollisionResult& result)
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
FCL_REAL BVHShapeConservativeAdvancement(const CollisionGeometryd* o1, const MotionBase* motion1, const CollisionGeometryd* o2, const MotionBase* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequest& request, ContinuousCollisionResult& result)
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


  conservative_advancement_matrix[GEOM_BOX][GEOM_BOX] = &ShapeConservativeAdvancement<Boxd, Boxd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_SPHERE] = &ShapeConservativeAdvancement<Boxd, Sphered, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Boxd, Capsuled, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_CONE] = &ShapeConservativeAdvancement<Boxd, Coned, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Boxd, Cylinderd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_CONVEX] = &ShapeConservativeAdvancement<Boxd, Convexd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_PLANE] = &ShapeConservativeAdvancement<Boxd, Planed, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Boxd, Halfspaced, NarrowPhaseSolver>;
    
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_BOX] = &ShapeConservativeAdvancement<Sphered, Boxd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_SPHERE] = &ShapeConservativeAdvancement<Sphered, Sphered, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Sphered, Capsuled, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_CONE] = &ShapeConservativeAdvancement<Sphered, Coned, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Sphered, Cylinderd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_CONVEX] = &ShapeConservativeAdvancement<Sphered, Convexd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_PLANE] = &ShapeConservativeAdvancement<Sphered, Planed, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Sphered, Halfspaced, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_BOX] = &ShapeConservativeAdvancement<Capsuled, Boxd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_SPHERE] = &ShapeConservativeAdvancement<Capsuled, Sphered, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Capsuled, Capsuled, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_CONE] = &ShapeConservativeAdvancement<Capsuled, Coned, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Capsuled, Cylinderd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_CONVEX] = &ShapeConservativeAdvancement<Capsuled, Convexd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_PLANE] = &ShapeConservativeAdvancement<Capsuled, Planed, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Capsuled, Halfspaced, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_CONE][GEOM_BOX] = &ShapeConservativeAdvancement<Coned, Boxd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_SPHERE] = &ShapeConservativeAdvancement<Coned, Sphered, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Coned, Capsuled, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_CONE] = &ShapeConservativeAdvancement<Coned, Coned, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Coned, Cylinderd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_CONVEX] = &ShapeConservativeAdvancement<Coned, Convexd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_PLANE] = &ShapeConservativeAdvancement<Coned, Planed, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Coned, Halfspaced, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_BOX] = &ShapeConservativeAdvancement<Cylinderd, Boxd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_SPHERE] = &ShapeConservativeAdvancement<Cylinderd, Sphered, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Cylinderd, Capsuled, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_CONE] = &ShapeConservativeAdvancement<Cylinderd, Coned, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Cylinderd, Cylinderd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_CONVEX] = &ShapeConservativeAdvancement<Cylinderd, Convexd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_PLANE] = &ShapeConservativeAdvancement<Cylinderd, Planed, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Cylinderd, Halfspaced, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_CONVEX][GEOM_BOX] = &ShapeConservativeAdvancement<Convexd, Boxd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_SPHERE] = &ShapeConservativeAdvancement<Convexd, Sphered, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Convexd, Capsuled, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_CONE] = &ShapeConservativeAdvancement<Convexd, Coned, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Convexd, Cylinderd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_CONVEX] = &ShapeConservativeAdvancement<Convexd, Convexd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_PLANE] = &ShapeConservativeAdvancement<Convexd, Planed, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Convexd, Halfspaced, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_PLANE][GEOM_BOX] = &ShapeConservativeAdvancement<Planed, Boxd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_SPHERE] = &ShapeConservativeAdvancement<Planed, Sphered, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Planed, Capsuled, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_CONE] = &ShapeConservativeAdvancement<Planed, Coned, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Planed, Cylinderd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_CONVEX] = &ShapeConservativeAdvancement<Planed, Convexd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_PLANE] = &ShapeConservativeAdvancement<Planed, Planed, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Planed, Halfspaced, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_BOX] = &ShapeConservativeAdvancement<Halfspaced, Boxd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_SPHERE] = &ShapeConservativeAdvancement<Halfspaced, Sphered, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Halfspaced, Capsuled, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_CONE] = &ShapeConservativeAdvancement<Halfspaced, Coned, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Halfspaced, Cylinderd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_CONVEX] = &ShapeConservativeAdvancement<Halfspaced, Convexd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_PLANE] = &ShapeConservativeAdvancement<Halfspaced, Planed, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Halfspaced, Halfspaced, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_AABB][GEOM_BOX] = &BVHShapeConservativeAdvancement<AABB, Boxd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<AABB, Sphered, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<AABB, Capsuled, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_CONE] = &BVHShapeConservativeAdvancement<AABB, Coned, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<AABB, Cylinderd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<AABB, Convexd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_PLANE] = &BVHShapeConservativeAdvancement<AABB, Planed, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<AABB, Halfspaced, NarrowPhaseSolver>;
  
  conservative_advancement_matrix[BV_OBB][GEOM_BOX] = &BVHShapeConservativeAdvancement<OBBd, Boxd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<OBBd, Sphered, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<OBBd, Capsuled, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_CONE] = &BVHShapeConservativeAdvancement<OBBd, Coned, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<OBBd, Cylinderd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<OBBd, Convexd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_PLANE] = &BVHShapeConservativeAdvancement<OBBd, Planed, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<OBBd, Halfspaced, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_OBBRSS][GEOM_BOX] = &BVHShapeConservativeAdvancement<OBBRSS, Boxd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<OBBRSS, Sphered, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<OBBRSS, Capsuled, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_CONE] = &BVHShapeConservativeAdvancement<OBBRSS, Coned, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<OBBRSS, Cylinderd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<OBBRSS, Convexd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_PLANE] = &BVHShapeConservativeAdvancement<OBBRSS, Planed, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<OBBRSS, Halfspaced, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_RSS][GEOM_BOX] = &BVHShapeConservativeAdvancement<RSS, Boxd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<RSS, Sphered, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<RSS, Capsuled, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_CONE] = &BVHShapeConservativeAdvancement<RSS, Coned, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<RSS, Cylinderd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<RSS, Convexd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_PLANE] = &BVHShapeConservativeAdvancement<RSS, Planed, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<RSS, Halfspaced, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_KDOP16][GEOM_BOX] = &BVHShapeConservativeAdvancement<KDOP<16>, Boxd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<KDOP<16>, Sphered, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<KDOP<16>, Capsuled, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_CONE] = &BVHShapeConservativeAdvancement<KDOP<16>, Coned, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<KDOP<16>, Cylinderd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<KDOP<16>, Convexd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_PLANE] = &BVHShapeConservativeAdvancement<KDOP<16>, Planed, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<KDOP<16>, Halfspaced, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_KDOP18][GEOM_BOX] = &BVHShapeConservativeAdvancement<KDOP<18>, Boxd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<KDOP<18>, Sphered, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<KDOP<18>, Capsuled, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_CONE] = &BVHShapeConservativeAdvancement<KDOP<18>, Coned, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<KDOP<18>, Cylinderd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<KDOP<18>, Convexd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_PLANE] = &BVHShapeConservativeAdvancement<KDOP<18>, Planed, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<KDOP<18>, Halfspaced, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_KDOP24][GEOM_BOX] = &BVHShapeConservativeAdvancement<KDOP<24>, Boxd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<KDOP<24>, Sphered, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<KDOP<24>, Capsuled, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_CONE] = &BVHShapeConservativeAdvancement<KDOP<24>, Coned, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<KDOP<24>, Cylinderd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<KDOP<24>, Convexd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_PLANE] = &BVHShapeConservativeAdvancement<KDOP<24>, Planed, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<KDOP<24>, Halfspaced, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_kIOS][GEOM_BOX] = &BVHShapeConservativeAdvancement<kIOS, Boxd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<kIOS, Sphered, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<kIOS, Capsuled, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_CONE] = &BVHShapeConservativeAdvancement<kIOS, Coned, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<kIOS, Cylinderd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<kIOS, Convexd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_PLANE] = &BVHShapeConservativeAdvancement<kIOS, Planed, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<kIOS, Halfspaced, NarrowPhaseSolver>;


  conservative_advancement_matrix[GEOM_BOX][BV_AABB] = &ShapeBVHConservativeAdvancement<Boxd, AABB, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_AABB] = &ShapeBVHConservativeAdvancement<Sphered, AABB, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_AABB] = &ShapeBVHConservativeAdvancement<Capsuled, AABB, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_AABB] = &ShapeBVHConservativeAdvancement<Coned, AABB, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_AABB] = &ShapeBVHConservativeAdvancement<Cylinderd, AABB, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_AABB] = &ShapeBVHConservativeAdvancement<Convexd, AABB, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_AABB] = &ShapeBVHConservativeAdvancement<Planed, AABB, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_AABB] = &ShapeBVHConservativeAdvancement<Halfspaced, AABB, NarrowPhaseSolver>;
  
  conservative_advancement_matrix[GEOM_BOX][BV_OBB] = &ShapeBVHConservativeAdvancement<Boxd, OBBd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_OBB] = &ShapeBVHConservativeAdvancement<Sphered, OBBd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_OBB] = &ShapeBVHConservativeAdvancement<Capsuled, OBBd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_OBB] = &ShapeBVHConservativeAdvancement<Coned, OBBd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_OBB] = &ShapeBVHConservativeAdvancement<Cylinderd, OBBd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_OBB] = &ShapeBVHConservativeAdvancement<Convexd, OBBd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_OBB] = &ShapeBVHConservativeAdvancement<Planed, OBBd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_OBB] = &ShapeBVHConservativeAdvancement<Halfspaced, OBBd, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_RSS] = &ShapeBVHConservativeAdvancement<Boxd, RSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_RSS] = &ShapeBVHConservativeAdvancement<Sphered, RSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_RSS] = &ShapeBVHConservativeAdvancement<Capsuled, RSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_RSS] = &ShapeBVHConservativeAdvancement<Coned, RSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_RSS] = &ShapeBVHConservativeAdvancement<Cylinderd, RSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_RSS] = &ShapeBVHConservativeAdvancement<Convexd, RSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_RSS] = &ShapeBVHConservativeAdvancement<Planed, RSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_RSS] = &ShapeBVHConservativeAdvancement<Halfspaced, RSS, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Boxd, OBBRSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Sphered, OBBRSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Capsuled, OBBRSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Coned, OBBRSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Cylinderd, OBBRSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Convexd, OBBRSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Planed, OBBRSS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Halfspaced, OBBRSS, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Boxd, KDOP<16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Sphered, KDOP<16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Capsuled, KDOP<16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Coned, KDOP<16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Cylinderd, KDOP<16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Convexd, KDOP<16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Planed, KDOP<16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Halfspaced, KDOP<16>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Boxd, KDOP<18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Sphered, KDOP<18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Capsuled, KDOP<18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Coned, KDOP<18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Cylinderd, KDOP<18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Convexd, KDOP<18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Planed, KDOP<18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Halfspaced, KDOP<18>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Boxd, KDOP<24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Sphered, KDOP<24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Capsuled, KDOP<24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Coned, KDOP<24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Cylinderd, KDOP<24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Convexd, KDOP<24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Planed, KDOP<24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Halfspaced, KDOP<24>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_kIOS] = &ShapeBVHConservativeAdvancement<Boxd, kIOS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Sphered, kIOS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Capsuled, kIOS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Coned, kIOS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_kIOS] = &ShapeBVHConservativeAdvancement<Cylinderd, kIOS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_kIOS] = &ShapeBVHConservativeAdvancement<Convexd, kIOS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Planed, kIOS, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Halfspaced, kIOS, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_AABB][BV_AABB] = &BVHConservativeAdvancement<AABB, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][BV_OBB] = &BVHConservativeAdvancement<OBBd, NarrowPhaseSolver>;
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



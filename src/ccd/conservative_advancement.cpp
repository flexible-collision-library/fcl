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
#include "fcl/traversal/traversal_recurse.h"
#include "fcl/collision.h"
#include "fcl/traversal/distance/mesh_conservative_advancement_traversal_node.h"
#include "fcl/traversal/distance/shape_conservative_advancement_traversal_node.h"
#include "fcl/traversal/distance/mesh_shape_conservative_advancement_traversal_node.h"
#include "fcl/traversal/distance/shape_mesh_conservative_advancement_traversal_node.h"

namespace fcl
{




template<typename BV>
bool conservativeAdvancement(const BVHModel<BV>& o1,
                             const MotionBase* motion1,
                             const BVHModel<BV>& o2,
                             const MotionBase* motion2,
                             const CollisionRequestd& request,
                             CollisionResultd& result,
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

    distanceRecurse<double>(&node, 0, 0, NULL);

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
                                         const CollisionRequestd& request,
                                         CollisionResultd& result,
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
bool conservativeAdvancement(const BVHModel<RSSd>& o1,
                             const MotionBase* motion1,
                             const BVHModel<RSSd>& o2,
                             const MotionBase* motion2,
                             const CollisionRequestd& request,
                             CollisionResultd& result,
                             FCL_REAL& toc);


template<>
bool conservativeAdvancement(const BVHModel<OBBRSSd>& o1,
                             const MotionBase* motion1,
                             const BVHModel<OBBRSSd>& o2,
                             const MotionBase* motion2,
                             const CollisionRequestd& request,
                             CollisionResultd& result,
                             FCL_REAL& toc);

template<typename S1, typename S2, typename NarrowPhaseSolver>
bool conservativeAdvancement(const S1& o1,
                             const MotionBase* motion1,
                             const S2& o2,
                             const MotionBase* motion2,
                             const NarrowPhaseSolver* solver,
                             const CollisionRequestd& request,
                             CollisionResultd& result,
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
                             const CollisionRequestd& request,
                             CollisionResultd& result,
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

    distanceRecurse<double>(&node, 0, 0, NULL);

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
                                              const CollisionRequestd& request,
                                              CollisionResultd& result,
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
bool conservativeAdvancement(const BVHModel<RSSd>& o1,
                             const MotionBase* motion1,
                             const S& o2,
                             const MotionBase* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequestd& request,
                             CollisionResultd& result,
                             FCL_REAL& toc)
{
  return details::conservativeAdvancementMeshShapeOriented<RSSd, S, NarrowPhaseSolver, MeshShapeConservativeAdvancementTraversalNodeRSS<S, NarrowPhaseSolver> >(o1, motion1, o2, motion2, nsolver, request, result, toc);
}

template<typename S, typename NarrowPhaseSolver>
bool conservativeAdvancement(const BVHModel<OBBRSSd>& o1,
                             const MotionBase* motion1,
                             const S& o2,
                             const MotionBase* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequestd& request,
                             CollisionResultd& result,
                             FCL_REAL& toc)
{
  return details::conservativeAdvancementMeshShapeOriented<OBBRSSd, S, NarrowPhaseSolver, MeshShapeConservativeAdvancementTraversalNodeOBBRSS<S, NarrowPhaseSolver> >(o1, motion1, o2, motion2, nsolver, request, result, toc);  
}
                            
template<typename S, typename BV, typename NarrowPhaseSolver>
bool conservativeAdvancement(const S& o1,
                             const MotionBase* motion1,
                             const BVHModel<BV>& o2,
                             const MotionBase* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequestd& request,
                             CollisionResultd& result,
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
                                              const CollisionRequestd& request,
                                              CollisionResultd& result,
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
                             const BVHModel<RSSd>& o2,
                             const MotionBase* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequestd& request,
                             CollisionResultd& result,
                             FCL_REAL& toc)
{
  return details::conservativeAdvancementShapeMeshOriented<S, RSSd, NarrowPhaseSolver, ShapeMeshConservativeAdvancementTraversalNodeRSS<S, NarrowPhaseSolver> >(o1, motion1, o2, motion2, nsolver, request, result, toc);
}


template<typename S, typename NarrowPhaseSolver>
bool conservativeAdvancement(const S& o1,
                             const MotionBase* motion1,
                             const BVHModel<OBBRSSd>& o2,
                             const MotionBase* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequestd& request,
                             CollisionResultd& result,
                             FCL_REAL& toc)
{
  return details::conservativeAdvancementShapeMeshOriented<S, OBBRSSd, NarrowPhaseSolver, ShapeMeshConservativeAdvancementTraversalNodeOBBRSS<S, NarrowPhaseSolver> >(o1, motion1, o2, motion2, nsolver, request, result, toc);
}



template<>
bool conservativeAdvancement(const BVHModel<RSSd>& o1,
                             const MotionBase* motion1,
                             const BVHModel<RSSd>& o2,
                             const MotionBase* motion2,
                             const CollisionRequestd& request,
                             CollisionResultd& result,
                             FCL_REAL& toc)
{
  return details::conservativeAdvancementMeshOriented<RSSd, MeshConservativeAdvancementTraversalNodeRSSd>(o1, motion1, o2, motion2, request, result, toc);
}

template<>
bool conservativeAdvancement(const BVHModel<OBBRSSd>& o1,
                             const MotionBase* motion1,
                             const BVHModel<OBBRSSd>& o2,
                             const MotionBase* motion2,
                             const CollisionRequestd& request,
                             CollisionResultd& result,
                             FCL_REAL& toc)
{
  return details::conservativeAdvancementMeshOriented<OBBRSSd, MeshConservativeAdvancementTraversalNodeOBBRSSd>(o1, motion1, o2, motion2, request, result, toc);
}


template<typename BV, typename NarrowPhaseSolver>
FCL_REAL BVHConservativeAdvancement(const CollisionGeometryd* o1, const MotionBase* motion1, const CollisionGeometryd* o2, const MotionBase* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequestd& request, ContinuousCollisionResultd& result)
{
  const BVHModel<BV>* obj1 = static_cast<const BVHModel<BV>*>(o1);
  const BVHModel<BV>* obj2 = static_cast<const BVHModel<BV>*>(o2);

  CollisionRequestd c_request;
  CollisionResultd c_result;
  FCL_REAL toc;
  bool is_collide = conservativeAdvancement(*obj1, motion1, *obj2, motion2, c_request, c_result, toc);

  result.is_collide = is_collide;
  result.time_of_contact = toc;
  
  return toc;
}

template<typename S1, typename S2, typename NarrowPhaseSolver>
FCL_REAL ShapeConservativeAdvancement(const CollisionGeometryd* o1, const MotionBase* motion1, const CollisionGeometryd* o2, const MotionBase* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequestd& request, ContinuousCollisionResultd& result)
{
  const S1* obj1 = static_cast<const S1*>(o1);
  const S2* obj2 = static_cast<const S2*>(o2);

  CollisionRequestd c_request;
  CollisionResultd c_result;
  FCL_REAL toc;
  bool is_collide = conservativeAdvancement(*obj1, motion1, *obj2, motion2, nsolver, c_request, c_result, toc);

  result.is_collide = is_collide;
  result.time_of_contact = toc;

  return toc;
}

template<typename S, typename BV, typename NarrowPhaseSolver>
FCL_REAL ShapeBVHConservativeAdvancement(const CollisionGeometryd* o1, const MotionBase* motion1, const CollisionGeometryd* o2, const MotionBase* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequestd& request, ContinuousCollisionResultd& result)
{
  const S* obj1 = static_cast<const S*>(o1);
  const BVHModel<BV>* obj2 = static_cast<const BVHModel<BV>*>(o2);

  CollisionRequestd c_request;
  CollisionResultd c_result;
  FCL_REAL toc;

  bool is_collide = conservativeAdvancement(*obj1, motion1, *obj2, motion2, nsolver, c_request, c_result, toc);

  result.is_collide = is_collide;
  result.time_of_contact = toc;

  return toc;
}

template<typename BV, typename S, typename NarrowPhaseSolver>
FCL_REAL BVHShapeConservativeAdvancement(const CollisionGeometryd* o1, const MotionBase* motion1, const CollisionGeometryd* o2, const MotionBase* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequestd& request, ContinuousCollisionResultd& result)
{
  const BVHModel<BV>* obj1 = static_cast<const BVHModel<BV>*>(o1);
  const S* obj2 = static_cast<const S*>(o2);

  CollisionRequestd c_request;
  CollisionResultd c_result;
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

  conservative_advancement_matrix[BV_AABB][GEOM_BOX] = &BVHShapeConservativeAdvancement<AABBd, Boxd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<AABBd, Sphered, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<AABBd, Capsuled, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_CONE] = &BVHShapeConservativeAdvancement<AABBd, Coned, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<AABBd, Cylinderd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<AABBd, Convexd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_PLANE] = &BVHShapeConservativeAdvancement<AABBd, Planed, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<AABBd, Halfspaced, NarrowPhaseSolver>;
  
  conservative_advancement_matrix[BV_OBB][GEOM_BOX] = &BVHShapeConservativeAdvancement<OBBd, Boxd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<OBBd, Sphered, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<OBBd, Capsuled, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_CONE] = &BVHShapeConservativeAdvancement<OBBd, Coned, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<OBBd, Cylinderd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<OBBd, Convexd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_PLANE] = &BVHShapeConservativeAdvancement<OBBd, Planed, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<OBBd, Halfspaced, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_OBBRSS][GEOM_BOX] = &BVHShapeConservativeAdvancement<OBBRSSd, Boxd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<OBBRSSd, Sphered, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<OBBRSSd, Capsuled, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_CONE] = &BVHShapeConservativeAdvancement<OBBRSSd, Coned, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<OBBRSSd, Cylinderd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<OBBRSSd, Convexd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_PLANE] = &BVHShapeConservativeAdvancement<OBBRSSd, Planed, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<OBBRSSd, Halfspaced, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_RSS][GEOM_BOX] = &BVHShapeConservativeAdvancement<RSSd, Boxd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<RSSd, Sphered, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<RSSd, Capsuled, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_CONE] = &BVHShapeConservativeAdvancement<RSSd, Coned, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<RSSd, Cylinderd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<RSSd, Convexd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_PLANE] = &BVHShapeConservativeAdvancement<RSSd, Planed, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<RSSd, Halfspaced, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_KDOP16][GEOM_BOX] = &BVHShapeConservativeAdvancement<KDOPd<16>, Boxd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<KDOPd<16>, Sphered, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<KDOPd<16>, Capsuled, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_CONE] = &BVHShapeConservativeAdvancement<KDOPd<16>, Coned, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<KDOPd<16>, Cylinderd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<KDOPd<16>, Convexd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_PLANE] = &BVHShapeConservativeAdvancement<KDOPd<16>, Planed, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<KDOPd<16>, Halfspaced, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_KDOP18][GEOM_BOX] = &BVHShapeConservativeAdvancement<KDOPd<18>, Boxd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<KDOPd<18>, Sphered, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<KDOPd<18>, Capsuled, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_CONE] = &BVHShapeConservativeAdvancement<KDOPd<18>, Coned, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<KDOPd<18>, Cylinderd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<KDOPd<18>, Convexd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_PLANE] = &BVHShapeConservativeAdvancement<KDOPd<18>, Planed, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<KDOPd<18>, Halfspaced, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_KDOP24][GEOM_BOX] = &BVHShapeConservativeAdvancement<KDOPd<24>, Boxd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<KDOPd<24>, Sphered, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<KDOPd<24>, Capsuled, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_CONE] = &BVHShapeConservativeAdvancement<KDOPd<24>, Coned, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<KDOPd<24>, Cylinderd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<KDOPd<24>, Convexd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_PLANE] = &BVHShapeConservativeAdvancement<KDOPd<24>, Planed, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<KDOPd<24>, Halfspaced, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_kIOS][GEOM_BOX] = &BVHShapeConservativeAdvancement<kIOSd, Boxd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<kIOSd, Sphered, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<kIOSd, Capsuled, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_CONE] = &BVHShapeConservativeAdvancement<kIOSd, Coned, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<kIOSd, Cylinderd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<kIOSd, Convexd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_PLANE] = &BVHShapeConservativeAdvancement<kIOSd, Planed, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<kIOSd, Halfspaced, NarrowPhaseSolver>;


  conservative_advancement_matrix[GEOM_BOX][BV_AABB] = &ShapeBVHConservativeAdvancement<Boxd, AABBd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_AABB] = &ShapeBVHConservativeAdvancement<Sphered, AABBd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_AABB] = &ShapeBVHConservativeAdvancement<Capsuled, AABBd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_AABB] = &ShapeBVHConservativeAdvancement<Coned, AABBd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_AABB] = &ShapeBVHConservativeAdvancement<Cylinderd, AABBd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_AABB] = &ShapeBVHConservativeAdvancement<Convexd, AABBd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_AABB] = &ShapeBVHConservativeAdvancement<Planed, AABBd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_AABB] = &ShapeBVHConservativeAdvancement<Halfspaced, AABBd, NarrowPhaseSolver>;
  
  conservative_advancement_matrix[GEOM_BOX][BV_OBB] = &ShapeBVHConservativeAdvancement<Boxd, OBBd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_OBB] = &ShapeBVHConservativeAdvancement<Sphered, OBBd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_OBB] = &ShapeBVHConservativeAdvancement<Capsuled, OBBd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_OBB] = &ShapeBVHConservativeAdvancement<Coned, OBBd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_OBB] = &ShapeBVHConservativeAdvancement<Cylinderd, OBBd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_OBB] = &ShapeBVHConservativeAdvancement<Convexd, OBBd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_OBB] = &ShapeBVHConservativeAdvancement<Planed, OBBd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_OBB] = &ShapeBVHConservativeAdvancement<Halfspaced, OBBd, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_RSS] = &ShapeBVHConservativeAdvancement<Boxd, RSSd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_RSS] = &ShapeBVHConservativeAdvancement<Sphered, RSSd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_RSS] = &ShapeBVHConservativeAdvancement<Capsuled, RSSd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_RSS] = &ShapeBVHConservativeAdvancement<Coned, RSSd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_RSS] = &ShapeBVHConservativeAdvancement<Cylinderd, RSSd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_RSS] = &ShapeBVHConservativeAdvancement<Convexd, RSSd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_RSS] = &ShapeBVHConservativeAdvancement<Planed, RSSd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_RSS] = &ShapeBVHConservativeAdvancement<Halfspaced, RSSd, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Boxd, OBBRSSd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Sphered, OBBRSSd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Capsuled, OBBRSSd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Coned, OBBRSSd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Cylinderd, OBBRSSd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Convexd, OBBRSSd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Planed, OBBRSSd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Halfspaced, OBBRSSd, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Boxd, KDOPd<16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Sphered, KDOPd<16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Capsuled, KDOPd<16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Coned, KDOPd<16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Cylinderd, KDOPd<16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Convexd, KDOPd<16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Planed, KDOPd<16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Halfspaced, KDOPd<16>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Boxd, KDOPd<18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Sphered, KDOPd<18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Capsuled, KDOPd<18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Coned, KDOPd<18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Cylinderd, KDOPd<18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Convexd, KDOPd<18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Planed, KDOPd<18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Halfspaced, KDOPd<18>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Boxd, KDOPd<24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Sphered, KDOPd<24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Capsuled, KDOPd<24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Coned, KDOPd<24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Cylinderd, KDOPd<24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Convexd, KDOPd<24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Planed, KDOPd<24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Halfspaced, KDOPd<24>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_kIOS] = &ShapeBVHConservativeAdvancement<Boxd, kIOSd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Sphered, kIOSd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Capsuled, kIOSd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Coned, kIOSd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_kIOS] = &ShapeBVHConservativeAdvancement<Cylinderd, kIOSd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_kIOS] = &ShapeBVHConservativeAdvancement<Convexd, kIOSd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Planed, kIOSd, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Halfspaced, kIOSd, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_AABB][BV_AABB] = &BVHConservativeAdvancement<AABBd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][BV_OBB] = &BVHConservativeAdvancement<OBBd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][BV_RSS] = &BVHConservativeAdvancement<RSSd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][BV_OBBRSS] = &BVHConservativeAdvancement<OBBRSSd, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][BV_KDOP16] = &BVHConservativeAdvancement<KDOPd<16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][BV_KDOP18] = &BVHConservativeAdvancement<KDOPd<18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][BV_KDOP24] = &BVHConservativeAdvancement<KDOPd<24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][BV_kIOS] = &BVHConservativeAdvancement<kIOSd, NarrowPhaseSolver>;
  
}


template struct ConservativeAdvancementFunctionMatrix<GJKSolver_libccd>;
template struct ConservativeAdvancementFunctionMatrix<GJKSolver_indep>;









}



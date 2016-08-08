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

#ifndef FCL_CONSERVATIVE_ADVANCEMENT_H
#define FCL_CONSERVATIVE_ADVANCEMENT_H

#include "fcl/collision_object.h"
#include "fcl/collision_data.h"
#include "fcl/ccd/motion_base.h"
#include "fcl/BVH/BVH_model.h"
#include "fcl/shape/box.h"
#include "fcl/shape/capsule.h"
#include "fcl/shape/cone.h"
#include "fcl/shape/convex.h"
#include "fcl/shape/cylinder.h"
#include "fcl/shape/ellipsoid.h"
#include "fcl/shape/halfspace.h"
#include "fcl/shape/plane.h"
#include "fcl/shape/sphere.h"
#include "fcl/shape/triangle_p.h"
#include "fcl/traversal/traversal_nodes.h"
#include "fcl/traversal/traversal_recurse.h"

namespace fcl
{

template<typename NarrowPhaseSolver>
struct ConservativeAdvancementFunctionMatrix
{
  using Scalar = typename NarrowPhaseSolver::Scalar;

  typedef Scalar (*ConservativeAdvancementFunc)(const CollisionGeometry<Scalar>* o1, const MotionBase<Scalar>* motion1, const CollisionGeometry<Scalar>* o2, const MotionBase<Scalar>* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequest<Scalar>& request, ContinuousCollisionResult<Scalar>& result);

  ConservativeAdvancementFunc conservative_advancement_matrix[NODE_COUNT][NODE_COUNT];

  ConservativeAdvancementFunctionMatrix();
};

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template<typename BV>
bool conservativeAdvancement(const BVHModel<BV>& o1,
                             const MotionBase<typename BV::Scalar>* motion1,
                             const BVHModel<BV>& o2,
                             const MotionBase<typename BV::Scalar>* motion2,
                             const CollisionRequest<typename BV::Scalar>& request,
                             CollisionResult<typename BV::Scalar>& result,
                             typename BV::Scalar& toc)
{
  using Scalar = typename BV::Scalar;

  Transform3<Scalar> tf1, tf2;
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
    node.min_distance = std::numeric_limits<Scalar>::max();

    distanceRecurse<Scalar>(&node, 0, 0, NULL);

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
                                         const MotionBase<typename BV::Scalar>* motion1,
                                         const BVHModel<BV>& o2,
                                         const MotionBase<typename BV::Scalar>* motion2,
                                         const CollisionRequest<typename BV::Scalar>& request,
                                         CollisionResult<typename BV::Scalar>& result,
                                         typename BV::Scalar& toc)
{
  using Scalar = typename BV::Scalar;

  Transform3<Scalar> tf1, tf2;
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
    Transform3<Scalar> tf = tf1.inverse(Eigen::Isometry) * tf2;
    node.R = tf.linear();
    node.T = tf.translation();

    node.delta_t = 1;
    node.min_distance = std::numeric_limits<Scalar>::max();

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

template<typename S1, typename S2, typename NarrowPhaseSolver>
bool conservativeAdvancement(const S1& o1,
                             const MotionBase<typename NarrowPhaseSolver::Scalar>* motion1,
                             const S2& o2,
                             const MotionBase<typename NarrowPhaseSolver::Scalar>* motion2,
                             const NarrowPhaseSolver* solver,
                             const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
                             CollisionResult<typename NarrowPhaseSolver::Scalar>& result,
                             typename NarrowPhaseSolver::Scalar& toc)
{
  using Scalar = typename NarrowPhaseSolver::Scalar;

  Transform3<Scalar> tf1, tf2;
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
    node.min_distance = std::numeric_limits<Scalar>::max();

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
                             const MotionBase<typename BV::Scalar>* motion1,
                             const S& o2,
                             const MotionBase<typename BV::Scalar>* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest<typename BV::Scalar>& request,
                             CollisionResult<typename BV::Scalar>& result,
                             typename BV::Scalar& toc)
{
  using Scalar = typename BV::Scalar;

  Transform3<Scalar> tf1, tf2;
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
    node.min_distance = std::numeric_limits<Scalar>::max();

    distanceRecurse<Scalar>(&node, 0, 0, NULL);

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
                                              const MotionBase<typename BV::Scalar>* motion1,
                                              const S& o2,
                                              const MotionBase<typename BV::Scalar>* motion2,
                                              const NarrowPhaseSolver* nsolver,
                                              const CollisionRequest<typename BV::Scalar>& request,
                                              CollisionResult<typename BV::Scalar>& result,
                                              typename BV::Scalar& toc)
{
  using Scalar = typename BV::Scalar;

  Transform3<Scalar> tf1, tf2;
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
    node.min_distance = std::numeric_limits<Scalar>::max();

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
bool conservativeAdvancement(const BVHModel<RSS<typename NarrowPhaseSolver::Scalar>>& o1,
                             const MotionBase<typename NarrowPhaseSolver::Scalar>* motion1,
                             const S& o2,
                             const MotionBase<typename NarrowPhaseSolver::Scalar>* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
                             CollisionResult<typename NarrowPhaseSolver::Scalar>& result,
                             typename NarrowPhaseSolver::Scalar& toc)
{
  using Scalar = typename NarrowPhaseSolver::Scalar;

  return details::conservativeAdvancementMeshShapeOriented<RSS<Scalar>, S, NarrowPhaseSolver, MeshShapeConservativeAdvancementTraversalNodeRSS<S, NarrowPhaseSolver> >(o1, motion1, o2, motion2, nsolver, request, result, toc);
}

template<typename S, typename NarrowPhaseSolver>
bool conservativeAdvancement(const BVHModel<OBBRSS<typename NarrowPhaseSolver::Scalar>>& o1,
                             const MotionBase<typename NarrowPhaseSolver::Scalar>* motion1,
                             const S& o2,
                             const MotionBase<typename NarrowPhaseSolver::Scalar>* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
                             CollisionResult<typename NarrowPhaseSolver::Scalar>& result,
                             typename NarrowPhaseSolver::Scalar& toc)
{
  using Scalar = typename NarrowPhaseSolver::Scalar;

  return details::conservativeAdvancementMeshShapeOriented<OBBRSS<Scalar>, S, NarrowPhaseSolver, MeshShapeConservativeAdvancementTraversalNodeOBBRSS<S, NarrowPhaseSolver> >(o1, motion1, o2, motion2, nsolver, request, result, toc);
}

template<typename S, typename BV, typename NarrowPhaseSolver>
bool conservativeAdvancement(const S& o1,
                             const MotionBase<typename BV::Scalar>* motion1,
                             const BVHModel<BV>& o2,
                             const MotionBase<typename BV::Scalar>* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest<typename BV::Scalar>& request,
                             CollisionResult<typename BV::Scalar>& result,
                             typename BV::Scalar& toc)
{
  using Scalar = typename BV::Scalar;

  Transform3<Scalar> tf1, tf2;
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
    node.min_distance = std::numeric_limits<Scalar>::max();

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
                                              const MotionBase<typename BV::Scalar>* motion1,
                                              const BVHModel<BV>& o2,
                                              const MotionBase<typename BV::Scalar>* motion2,
                                              const NarrowPhaseSolver* nsolver,
                                              const CollisionRequest<typename BV::Scalar>& request,
                                              CollisionResult<typename BV::Scalar>& result,
                                              typename BV::Scalar& toc)
{
  using Scalar = typename BV::Scalar;
  Transform3<Scalar> tf1, tf2;
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
    node.min_distance = std::numeric_limits<Scalar>::max();

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

//==============================================================================
template <typename Scalar, typename S, typename NarrowPhaseSolver>
struct ConservativeAdvancementImpl1
{
  bool operator()(
      const S& o1,
      const MotionBase<Scalar>* motion1,
      const BVHModel<RSS<Scalar>>& o2,
      const MotionBase<Scalar>* motion2,
      const NarrowPhaseSolver* nsolver,
      const CollisionRequest<Scalar>& request,
      CollisionResult<Scalar>& result,
      typename NarrowPhaseSolver::Scalar& toc)
  {
    return details::conservativeAdvancementShapeMeshOriented<S, RSS<Scalar>, NarrowPhaseSolver, ShapeMeshConservativeAdvancementTraversalNodeRSS<S, NarrowPhaseSolver> >(o1, motion1, o2, motion2, nsolver, request, result, toc);
  }
};

//==============================================================================
template <typename Scalar, typename S, typename NarrowPhaseSolver>
struct ConservativeAdvancementImpl2
{
  bool operator()(
      const S& o1,
      const MotionBase<Scalar>* motion1,
      const BVHModel<OBBRSS<Scalar>>& o2,
      const MotionBase<Scalar>* motion2,
      const NarrowPhaseSolver* nsolver,
      const CollisionRequest<Scalar>& request,
      CollisionResult<Scalar>& result,
      Scalar& toc)
  {
    return details::conservativeAdvancementShapeMeshOriented<S, OBBRSS<Scalar>, NarrowPhaseSolver, ShapeMeshConservativeAdvancementTraversalNodeOBBRSS<S, NarrowPhaseSolver> >(o1, motion1, o2, motion2, nsolver, request, result, toc);
  }
};

template<typename S, typename NarrowPhaseSolver>
bool conservativeAdvancement(const S& o1,
                             const MotionBase<typename NarrowPhaseSolver::Scalar>* motion1,
                             const BVHModel<RSS<typename NarrowPhaseSolver::Scalar>>& o2,
                             const MotionBase<typename NarrowPhaseSolver::Scalar>* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
                             CollisionResult<typename NarrowPhaseSolver::Scalar>& result,
                             typename NarrowPhaseSolver::Scalar& toc)
{
  ConservativeAdvancementImpl1<typename NarrowPhaseSolver::Scalar, S, NarrowPhaseSolver> conservativeAdvancementImpl;
  return conservativeAdvancementImpl(o1, motion1, o2, motion2, nsolver, request, result, toc);
}

template<typename S, typename NarrowPhaseSolver>
bool conservativeAdvancement(const S& o1,
                             const MotionBase<typename NarrowPhaseSolver::Scalar>* motion1,
                             const BVHModel<OBBRSS<typename NarrowPhaseSolver::Scalar>>& o2,
                             const MotionBase<typename NarrowPhaseSolver::Scalar>* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
                             CollisionResult<typename NarrowPhaseSolver::Scalar>& result,
                             typename NarrowPhaseSolver::Scalar& toc)
{
  ConservativeAdvancementImpl2<typename NarrowPhaseSolver::Scalar, S, NarrowPhaseSolver> conservativeAdvancementImpl;
  return conservativeAdvancementImpl(o1, motion1, o2, motion2, nsolver, request, result, toc);
}

//==============================================================================
template <typename Scalar, typename NarrowPhaseSolver>
struct ConservativeAdvancementImpl1<Scalar, BVHModel<RSS<Scalar>>, NarrowPhaseSolver>
{
  bool operator()(
      const BVHModel<RSS<Scalar>>& o1,
      const MotionBase<Scalar>* motion1,
      const BVHModel<RSS<Scalar>>& o2,
      const MotionBase<Scalar>* motion2,
      const NarrowPhaseSolver* /*nsolver*/,
      const CollisionRequest<Scalar>& request,
      CollisionResult<Scalar>& result,
      typename NarrowPhaseSolver::Scalar& toc)
  {
    return details::conservativeAdvancementMeshOriented<RSS<Scalar>, MeshConservativeAdvancementTraversalNodeRSS<Scalar>>(o1, motion1, o2, motion2, request, result, toc);
  }
};

//==============================================================================
template <typename Scalar, typename NarrowPhaseSolver>
struct ConservativeAdvancementImpl2<Scalar, BVHModel<OBBRSS<Scalar>>, NarrowPhaseSolver>
{
  bool operator()(
      const BVHModel<OBBRSS<Scalar>>& o1,
      const MotionBase<Scalar>* motion1,
      const BVHModel<OBBRSS<Scalar>>& o2,
      const MotionBase<Scalar>* motion2,
      const NarrowPhaseSolver* /*nsolver*/,
      const CollisionRequest<Scalar>& request,
      CollisionResult<Scalar>& result,
      Scalar& toc)
  {
    return details::conservativeAdvancementMeshOriented<OBBRSS<Scalar>, MeshConservativeAdvancementTraversalNodeOBBRSS<Scalar>>(o1, motion1, o2, motion2, request, result, toc);
  }
};

template<typename BV, typename NarrowPhaseSolver>
typename BV::Scalar BVHConservativeAdvancement(const CollisionGeometry<typename BV::Scalar>* o1, const MotionBase<typename BV::Scalar>* motion1, const CollisionGeometry<typename BV::Scalar>* o2, const MotionBase<typename BV::Scalar>* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequest<typename BV::Scalar>& request, ContinuousCollisionResult<typename BV::Scalar>& result)
{
  using Scalar = typename BV::Scalar;

  const BVHModel<BV>* obj1 = static_cast<const BVHModel<BV>*>(o1);
  const BVHModel<BV>* obj2 = static_cast<const BVHModel<BV>*>(o2);

  CollisionRequest<Scalar> c_request;
  CollisionResult<Scalar> c_result;
  Scalar toc;
  bool is_collide = conservativeAdvancement(*obj1, motion1, *obj2, motion2, c_request, c_result, toc);

  result.is_collide = is_collide;
  result.time_of_contact = toc;

  return toc;
}

template<typename S1, typename S2, typename NarrowPhaseSolver>
typename NarrowPhaseSolver::Scalar ShapeConservativeAdvancement(const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o1, const MotionBase<typename NarrowPhaseSolver::Scalar>* motion1, const CollisionGeometry<typename NarrowPhaseSolver::Scalar>* o2, const MotionBase<typename NarrowPhaseSolver::Scalar>* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequest<typename NarrowPhaseSolver::Scalar>& request, ContinuousCollisionResult<typename NarrowPhaseSolver::Scalar>& result)
{
  using Scalar = typename NarrowPhaseSolver::Scalar;

  const S1* obj1 = static_cast<const S1*>(o1);
  const S2* obj2 = static_cast<const S2*>(o2);

  CollisionRequest<Scalar> c_request;
  CollisionResult<Scalar> c_result;
  Scalar toc;
  bool is_collide = conservativeAdvancement(*obj1, motion1, *obj2, motion2, nsolver, c_request, c_result, toc);

  result.is_collide = is_collide;
  result.time_of_contact = toc;

  return toc;
}

template<typename S, typename BV, typename NarrowPhaseSolver>
typename BV::Scalar ShapeBVHConservativeAdvancement(const CollisionGeometry<typename BV::Scalar>* o1, const MotionBase<typename BV::Scalar>* motion1, const CollisionGeometry<typename BV::Scalar>* o2, const MotionBase<typename BV::Scalar>* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequest<typename BV::Scalar>& request, ContinuousCollisionResult<typename BV::Scalar>& result)
{
  using Scalar = typename BV::Scalar;

  const S* obj1 = static_cast<const S*>(o1);
  const BVHModel<BV>* obj2 = static_cast<const BVHModel<BV>*>(o2);

  CollisionRequest<Scalar> c_request;
  CollisionResult<Scalar> c_result;
  Scalar toc;

  bool is_collide = conservativeAdvancement(*obj1, motion1, *obj2, motion2, nsolver, c_request, c_result, toc);

  result.is_collide = is_collide;
  result.time_of_contact = toc;

  return toc;
}

template<typename BV, typename S, typename NarrowPhaseSolver>
typename BV::Scalar BVHShapeConservativeAdvancement(const CollisionGeometry<typename BV::Scalar>* o1, const MotionBase<typename BV::Scalar>* motion1, const CollisionGeometry<typename BV::Scalar>* o2, const MotionBase<typename BV::Scalar>* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequest<typename BV::Scalar>& request, ContinuousCollisionResult<typename BV::Scalar>& result)
{
  using Scalar = typename BV::Scalar;

  const BVHModel<BV>* obj1 = static_cast<const BVHModel<BV>*>(o1);
  const S* obj2 = static_cast<const S*>(o2);

  CollisionRequest<Scalar> c_request;
  CollisionResult<Scalar> c_result;
  Scalar toc;

  bool is_collide = conservativeAdvancement(*obj1, motion1, *obj2, motion2, nsolver, c_request, c_result, toc);

  result.is_collide = is_collide;
  result.time_of_contact = toc;

  return toc;
}

template<typename NarrowPhaseSolver>
ConservativeAdvancementFunctionMatrix<NarrowPhaseSolver>::ConservativeAdvancementFunctionMatrix()
{
  using Scalar = typename NarrowPhaseSolver::Scalar;

  for(int i = 0; i < NODE_COUNT; ++i)
  {
    for(int j = 0; j < NODE_COUNT; ++j)
      conservative_advancement_matrix[i][j] = NULL;
  }


  conservative_advancement_matrix[GEOM_BOX][GEOM_BOX] = &ShapeConservativeAdvancement<Box<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_SPHERE] = &ShapeConservativeAdvancement<Box<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Box<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_CONE] = &ShapeConservativeAdvancement<Box<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Box<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_CONVEX] = &ShapeConservativeAdvancement<Box<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_PLANE] = &ShapeConservativeAdvancement<Box<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Box<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_SPHERE][GEOM_BOX] = &ShapeConservativeAdvancement<Sphere<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_SPHERE] = &ShapeConservativeAdvancement<Sphere<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Sphere<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_CONE] = &ShapeConservativeAdvancement<Sphere<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Sphere<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_CONVEX] = &ShapeConservativeAdvancement<Sphere<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_PLANE] = &ShapeConservativeAdvancement<Sphere<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Sphere<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_BOX] = &ShapeConservativeAdvancement<Capsule<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_SPHERE] = &ShapeConservativeAdvancement<Capsule<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Capsule<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_CONE] = &ShapeConservativeAdvancement<Capsule<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Capsule<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_CONVEX] = &ShapeConservativeAdvancement<Capsule<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_PLANE] = &ShapeConservativeAdvancement<Capsule<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Capsule<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_CONE][GEOM_BOX] = &ShapeConservativeAdvancement<Cone<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_SPHERE] = &ShapeConservativeAdvancement<Cone<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Cone<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_CONE] = &ShapeConservativeAdvancement<Cone<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Cone<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_CONVEX] = &ShapeConservativeAdvancement<Cone<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_PLANE] = &ShapeConservativeAdvancement<Cone<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Cone<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_BOX] = &ShapeConservativeAdvancement<Cylinder<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_SPHERE] = &ShapeConservativeAdvancement<Cylinder<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Cylinder<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_CONE] = &ShapeConservativeAdvancement<Cylinder<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Cylinder<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_CONVEX] = &ShapeConservativeAdvancement<Cylinder<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_PLANE] = &ShapeConservativeAdvancement<Cylinder<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Cylinder<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_CONVEX][GEOM_BOX] = &ShapeConservativeAdvancement<Convex<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_SPHERE] = &ShapeConservativeAdvancement<Convex<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Convex<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_CONE] = &ShapeConservativeAdvancement<Convex<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Convex<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_CONVEX] = &ShapeConservativeAdvancement<Convex<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_PLANE] = &ShapeConservativeAdvancement<Convex<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Convex<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_PLANE][GEOM_BOX] = &ShapeConservativeAdvancement<Plane<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_SPHERE] = &ShapeConservativeAdvancement<Plane<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Plane<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_CONE] = &ShapeConservativeAdvancement<Plane<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Plane<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_CONVEX] = &ShapeConservativeAdvancement<Plane<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_PLANE] = &ShapeConservativeAdvancement<Plane<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Plane<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_BOX] = &ShapeConservativeAdvancement<Halfspace<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_SPHERE] = &ShapeConservativeAdvancement<Halfspace<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Halfspace<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_CONE] = &ShapeConservativeAdvancement<Halfspace<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Halfspace<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_CONVEX] = &ShapeConservativeAdvancement<Halfspace<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_PLANE] = &ShapeConservativeAdvancement<Halfspace<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Halfspace<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_AABB][GEOM_BOX] = &BVHShapeConservativeAdvancement<AABB<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<AABB<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<AABB<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_CONE] = &BVHShapeConservativeAdvancement<AABB<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<AABB<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<AABB<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_PLANE] = &BVHShapeConservativeAdvancement<AABB<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<AABB<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_OBB][GEOM_BOX] = &BVHShapeConservativeAdvancement<OBB<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<OBB<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<OBB<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_CONE] = &BVHShapeConservativeAdvancement<OBB<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<OBB<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<OBB<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_PLANE] = &BVHShapeConservativeAdvancement<OBB<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<OBB<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_OBBRSS][GEOM_BOX] = &BVHShapeConservativeAdvancement<OBBRSS<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<OBBRSS<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<OBBRSS<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_CONE] = &BVHShapeConservativeAdvancement<OBBRSS<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<OBBRSS<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<OBBRSS<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_PLANE] = &BVHShapeConservativeAdvancement<OBBRSS<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<OBBRSS<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_RSS][GEOM_BOX] = &BVHShapeConservativeAdvancement<RSS<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<RSS<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<RSS<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_CONE] = &BVHShapeConservativeAdvancement<RSS<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<RSS<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<RSS<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_PLANE] = &BVHShapeConservativeAdvancement<RSS<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<RSS<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_KDOP16][GEOM_BOX] = &BVHShapeConservativeAdvancement<KDOP<Scalar, 16>, Box<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<KDOP<Scalar, 16>, Sphere<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<KDOP<Scalar, 16>, Capsule<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_CONE] = &BVHShapeConservativeAdvancement<KDOP<Scalar, 16>, Cone<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<KDOP<Scalar, 16>, Cylinder<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<KDOP<Scalar, 16>, Convex<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_PLANE] = &BVHShapeConservativeAdvancement<KDOP<Scalar, 16>, Plane<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<KDOP<Scalar, 16>, Halfspace<Scalar>, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_KDOP18][GEOM_BOX] = &BVHShapeConservativeAdvancement<KDOP<Scalar, 18>, Box<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<KDOP<Scalar, 18>, Sphere<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<KDOP<Scalar, 18>, Capsule<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_CONE] = &BVHShapeConservativeAdvancement<KDOP<Scalar, 18>, Cone<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<KDOP<Scalar, 18>, Cylinder<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<KDOP<Scalar, 18>, Convex<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_PLANE] = &BVHShapeConservativeAdvancement<KDOP<Scalar, 18>, Plane<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<KDOP<Scalar, 18>, Halfspace<Scalar>, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_KDOP24][GEOM_BOX] = &BVHShapeConservativeAdvancement<KDOP<Scalar, 24>, Box<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<KDOP<Scalar, 24>, Sphere<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<KDOP<Scalar, 24>, Capsule<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_CONE] = &BVHShapeConservativeAdvancement<KDOP<Scalar, 24>, Cone<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<KDOP<Scalar, 24>, Cylinder<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<KDOP<Scalar, 24>, Convex<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_PLANE] = &BVHShapeConservativeAdvancement<KDOP<Scalar, 24>, Plane<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<KDOP<Scalar, 24>, Halfspace<Scalar>, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_kIOS][GEOM_BOX] = &BVHShapeConservativeAdvancement<kIOS<Scalar>, Box<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<kIOS<Scalar>, Sphere<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<kIOS<Scalar>, Capsule<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_CONE] = &BVHShapeConservativeAdvancement<kIOS<Scalar>, Cone<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<kIOS<Scalar>, Cylinder<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<kIOS<Scalar>, Convex<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_PLANE] = &BVHShapeConservativeAdvancement<kIOS<Scalar>, Plane<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<kIOS<Scalar>, Halfspace<Scalar>, NarrowPhaseSolver>;


  conservative_advancement_matrix[GEOM_BOX][BV_AABB] = &ShapeBVHConservativeAdvancement<Box<Scalar>, AABB<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_AABB] = &ShapeBVHConservativeAdvancement<Sphere<Scalar>, AABB<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_AABB] = &ShapeBVHConservativeAdvancement<Capsule<Scalar>, AABB<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_AABB] = &ShapeBVHConservativeAdvancement<Cone<Scalar>, AABB<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_AABB] = &ShapeBVHConservativeAdvancement<Cylinder<Scalar>, AABB<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_AABB] = &ShapeBVHConservativeAdvancement<Convex<Scalar>, AABB<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_AABB] = &ShapeBVHConservativeAdvancement<Plane<Scalar>, AABB<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_AABB] = &ShapeBVHConservativeAdvancement<Halfspace<Scalar>, AABB<Scalar>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_OBB] = &ShapeBVHConservativeAdvancement<Box<Scalar>, OBB<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_OBB] = &ShapeBVHConservativeAdvancement<Sphere<Scalar>, OBB<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_OBB] = &ShapeBVHConservativeAdvancement<Capsule<Scalar>, OBB<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_OBB] = &ShapeBVHConservativeAdvancement<Cone<Scalar>, OBB<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_OBB] = &ShapeBVHConservativeAdvancement<Cylinder<Scalar>, OBB<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_OBB] = &ShapeBVHConservativeAdvancement<Convex<Scalar>, OBB<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_OBB] = &ShapeBVHConservativeAdvancement<Plane<Scalar>, OBB<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_OBB] = &ShapeBVHConservativeAdvancement<Halfspace<Scalar>, OBB<Scalar>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_RSS] = &ShapeBVHConservativeAdvancement<Box<Scalar>, RSS<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_RSS] = &ShapeBVHConservativeAdvancement<Sphere<Scalar>, RSS<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_RSS] = &ShapeBVHConservativeAdvancement<Capsule<Scalar>, RSS<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_RSS] = &ShapeBVHConservativeAdvancement<Cone<Scalar>, RSS<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_RSS] = &ShapeBVHConservativeAdvancement<Cylinder<Scalar>, RSS<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_RSS] = &ShapeBVHConservativeAdvancement<Convex<Scalar>, RSS<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_RSS] = &ShapeBVHConservativeAdvancement<Plane<Scalar>, RSS<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_RSS] = &ShapeBVHConservativeAdvancement<Halfspace<Scalar>, RSS<Scalar>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Box<Scalar>, OBBRSS<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Sphere<Scalar>, OBBRSS<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Capsule<Scalar>, OBBRSS<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Cone<Scalar>, OBBRSS<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Cylinder<Scalar>, OBBRSS<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Convex<Scalar>, OBBRSS<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Plane<Scalar>, OBBRSS<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Halfspace<Scalar>, OBBRSS<Scalar>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Box<Scalar>, KDOP<Scalar, 16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Sphere<Scalar>, KDOP<Scalar, 16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Capsule<Scalar>, KDOP<Scalar, 16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Cone<Scalar>, KDOP<Scalar, 16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Cylinder<Scalar>, KDOP<Scalar, 16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Convex<Scalar>, KDOP<Scalar, 16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Plane<Scalar>, KDOP<Scalar, 16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Halfspace<Scalar>, KDOP<Scalar, 16>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Box<Scalar>, KDOP<Scalar, 18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Sphere<Scalar>, KDOP<Scalar, 18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Capsule<Scalar>, KDOP<Scalar, 18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Cone<Scalar>, KDOP<Scalar, 18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Cylinder<Scalar>, KDOP<Scalar, 18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Convex<Scalar>, KDOP<Scalar, 18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Plane<Scalar>, KDOP<Scalar, 18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Halfspace<Scalar>, KDOP<Scalar, 18>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Box<Scalar>, KDOP<Scalar, 24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Sphere<Scalar>, KDOP<Scalar, 24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Capsule<Scalar>, KDOP<Scalar, 24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Cone<Scalar>, KDOP<Scalar, 24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Cylinder<Scalar>, KDOP<Scalar, 24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Convex<Scalar>, KDOP<Scalar, 24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Plane<Scalar>, KDOP<Scalar, 24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Halfspace<Scalar>, KDOP<Scalar, 24>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_kIOS] = &ShapeBVHConservativeAdvancement<Box<Scalar>, kIOS<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Sphere<Scalar>, kIOS<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Capsule<Scalar>, kIOS<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Cone<Scalar>, kIOS<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_kIOS] = &ShapeBVHConservativeAdvancement<Cylinder<Scalar>, kIOS<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_kIOS] = &ShapeBVHConservativeAdvancement<Convex<Scalar>, kIOS<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Plane<Scalar>, kIOS<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Halfspace<Scalar>, kIOS<Scalar>, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_AABB][BV_AABB] = &BVHConservativeAdvancement<AABB<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][BV_OBB] = &BVHConservativeAdvancement<OBB<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][BV_RSS] = &BVHConservativeAdvancement<RSS<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][BV_OBBRSS] = &BVHConservativeAdvancement<OBBRSS<Scalar>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][BV_KDOP16] = &BVHConservativeAdvancement<KDOP<Scalar, 16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][BV_KDOP18] = &BVHConservativeAdvancement<KDOP<Scalar, 18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][BV_KDOP24] = &BVHConservativeAdvancement<KDOP<Scalar, 24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][BV_kIOS] = &BVHConservativeAdvancement<kIOS<Scalar>, NarrowPhaseSolver>;

}


} // namespace fcl

#endif

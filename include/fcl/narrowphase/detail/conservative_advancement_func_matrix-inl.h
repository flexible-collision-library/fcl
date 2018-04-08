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

/** @author Jia Pan */

#ifndef FCL_CONSERVATIVE_ADVANCEMENT_INL_H
#define FCL_CONSERVATIVE_ADVANCEMENT_INL_H

#include "fcl/narrowphase/detail/conservative_advancement_func_matrix.h"

#include "fcl/common/unused.h"

#include "fcl/narrowphase/collision_object.h"

#include "fcl/math/motion/motion_base.h"

#include "fcl/geometry/bvh/BVH_model.h"
#include "fcl/geometry/shape/box.h"
#include "fcl/geometry/shape/capsule.h"
#include "fcl/geometry/shape/cone.h"
#include "fcl/geometry/shape/convex.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/geometry/shape/ellipsoid.h"
#include "fcl/geometry/shape/halfspace.h"
#include "fcl/geometry/shape/plane.h"
#include "fcl/geometry/shape/sphere.h"
#include "fcl/geometry/shape/triangle_p.h"

#include "fcl/narrowphase/detail/traversal/traversal_recurse.h"
#include "fcl/narrowphase/detail/traversal/distance/mesh_conservative_advancement_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/distance/shape_conservative_advancement_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/distance/mesh_shape_conservative_advancement_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/distance/shape_mesh_conservative_advancement_traversal_node.h"

namespace fcl
{

namespace detail
{

//==============================================================================
template<typename BV>
bool conservativeAdvancement(const BVHModel<BV>& o1,
                             const MotionBase<typename BV::S>* motion1,
                             const BVHModel<BV>& o2,
                             const MotionBase<typename BV::S>* motion2,
                             const CollisionRequest<typename BV::S>& request,
                             CollisionResult<typename BV::S>& result,
                             typename BV::S& toc)
{
  using S = typename BV::S;

  Transform3<S> tf1;
  Transform3<S> tf2;
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
    node.min_distance = std::numeric_limits<S>::max();

    distanceRecurse<S>(&node, 0, 0, nullptr);

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

template<typename BV, typename ConservativeAdvancementOrientedNode>
bool conservativeAdvancementMeshOriented(const BVHModel<BV>& o1,
                                         const MotionBase<typename BV::S>* motion1,
                                         const BVHModel<BV>& o2,
                                         const MotionBase<typename BV::S>* motion2,
                                         const CollisionRequest<typename BV::S>& request,
                                         CollisionResult<typename BV::S>& result,
                                         typename BV::S& toc)
{
  using S = typename BV::S;

  Transform3<S> tf1;
  Transform3<S> tf2;
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
    Transform3<S> tf = tf1.inverse(Eigen::Isometry) * tf2;
    node.R = tf.linear();
    node.T = tf.translation();

    node.delta_t = 1;
    node.min_distance = std::numeric_limits<S>::max();

    distanceRecurse(&node, 0, 0, nullptr);

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

template<typename Shape1, typename Shape2, typename NarrowPhaseSolver>
bool conservativeAdvancement(const Shape1& o1,
                             const MotionBase<typename Shape1::S>* motion1,
                             const Shape2& o2,
                             const MotionBase<typename Shape1::S>* motion2,
                             const NarrowPhaseSolver* solver,
                             const CollisionRequest<typename Shape1::S>& request,
                             CollisionResult<typename Shape1::S>& result,
                             typename Shape1::S& toc)
{
  using S = typename Shape1::S;

  Transform3<S> tf1;
  Transform3<S> tf2;
  motion1->getCurrentTransform(tf1);
  motion2->getCurrentTransform(tf2);

  // whether the first start configuration is in collision
  if(collide(&o1, tf1, &o2, tf2, request, result))
  {
    toc = 0;
    return true;
  }

  ShapeConservativeAdvancementTraversalNode<Shape1, Shape2, NarrowPhaseSolver> node;

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
    node.min_distance = std::numeric_limits<S>::max();

    distanceRecurse(&node, 0, 0, nullptr);

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

template<typename BV, typename Shape, typename NarrowPhaseSolver>
bool conservativeAdvancement(const BVHModel<BV>& o1,
                             const MotionBase<typename BV::S>* motion1,
                             const Shape& o2,
                             const MotionBase<typename BV::S>* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest<typename BV::S>& request,
                             CollisionResult<typename BV::S>& result,
                             typename BV::S& toc)
{
  using S = typename BV::S;

  Transform3<S> tf1;
  Transform3<S> tf2;
  motion1->getCurrentTransform(tf1);
  motion2->getCurrentTransform(tf2);

  if(collide(&o1, tf1, &o2, tf2, request, result))
  {
    toc = 0;
    return true;
  }

  BVHModel<BV>* o1_tmp = new BVHModel<BV>(o1);

  MeshShapeConservativeAdvancementTraversalNode<BV, Shape, NarrowPhaseSolver> node;

  node.motion1 = motion1;
  node.motion2 = motion2;

  do
  {
    // initialize update mesh to global coordinate, so time consuming
    initialize(node, *o1_tmp, tf1, o2, tf2, nsolver);

    node.delta_t = 1;
    node.min_distance = std::numeric_limits<S>::max();

    distanceRecurse<S>(&node, 0, 0, nullptr);

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

template<typename BV, typename Shape, typename NarrowPhaseSolver, typename ConservativeAdvancementOrientedNode>
bool conservativeAdvancementMeshShapeOriented(const BVHModel<BV>& o1,
                                              const MotionBase<typename BV::S>* motion1,
                                              const Shape& o2,
                                              const MotionBase<typename BV::S>* motion2,
                                              const NarrowPhaseSolver* nsolver,
                                              const CollisionRequest<typename BV::S>& request,
                                              CollisionResult<typename BV::S>& result,
                                              typename BV::S& toc)
{
  using S = typename BV::S;

  Transform3<S> tf1;
  Transform3<S> tf2;
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
    node.min_distance = std::numeric_limits<S>::max();

    distanceRecurse(&node, 0, 0, nullptr);

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

template<typename Shape, typename NarrowPhaseSolver>
bool conservativeAdvancement(const BVHModel<RSS<typename Shape::S>>& o1,
                             const MotionBase<typename Shape::S>* motion1,
                             const Shape& o2,
                             const MotionBase<typename Shape::S>* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest<typename Shape::S>& request,
                             CollisionResult<typename Shape::S>& result,
                             typename Shape::S& toc)
{
  using S = typename Shape::S;

  return detail::conservativeAdvancementMeshShapeOriented<RSS<S>, Shape, NarrowPhaseSolver, MeshShapeConservativeAdvancementTraversalNodeRSS<Shape, NarrowPhaseSolver> >(o1, motion1, o2, motion2, nsolver, request, result, toc);
}

template<typename Shape, typename NarrowPhaseSolver>
bool conservativeAdvancement(const BVHModel<OBBRSS<typename Shape::S>>& o1,
                             const MotionBase<typename Shape::S>* motion1,
                             const Shape& o2,
                             const MotionBase<typename Shape::S>* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest<typename Shape::S>& request,
                             CollisionResult<typename Shape::S>& result,
                             typename Shape::S& toc)
{
  using S = typename Shape::S;

  return detail::conservativeAdvancementMeshShapeOriented<OBBRSS<S>, Shape, NarrowPhaseSolver, MeshShapeConservativeAdvancementTraversalNodeOBBRSS<Shape, NarrowPhaseSolver> >(o1, motion1, o2, motion2, nsolver, request, result, toc);
}

template<typename Shape, typename BV, typename NarrowPhaseSolver>
bool conservativeAdvancement(const Shape& o1,
                             const MotionBase<typename BV::S>* motion1,
                             const BVHModel<BV>& o2,
                             const MotionBase<typename BV::S>* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest<typename BV::S>& request,
                             CollisionResult<typename BV::S>& result,
                             typename BV::S& toc)
{
  using S = typename BV::S;

  Transform3<S> tf1;
  Transform3<S> tf2;
  motion1->getCurrentTransform(tf1);
  motion2->getCurrentTransform(tf2);

  if(collide(&o1, tf1, &o2, tf2, request, result))
  {
    toc = 0;
    return true;
  }

  BVHModel<BV>* o2_tmp = new BVHModel<BV>(o2);

  ShapeMeshConservativeAdvancementTraversalNode<Shape, BV, NarrowPhaseSolver> node;

  node.motion1 = motion1;
  node.motion2 = motion2;

  do
  {
    // initialize update mesh to global coordinate, so time consuming
    initialize(node, o1, tf1, *o2_tmp, tf2, nsolver);

    node.delta_t = 1;
    node.min_distance = std::numeric_limits<S>::max();

    distanceRecurse(&node, 0, 0, nullptr);

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

template<typename Shape, typename BV, typename NarrowPhaseSolver, typename ConservativeAdvancementOrientedNode>
bool conservativeAdvancementShapeMeshOriented(const Shape& o1,
                                              const MotionBase<typename BV::S>* motion1,
                                              const BVHModel<BV>& o2,
                                              const MotionBase<typename BV::S>* motion2,
                                              const NarrowPhaseSolver* nsolver,
                                              const CollisionRequest<typename BV::S>& request,
                                              CollisionResult<typename BV::S>& result,
                                              typename BV::S& toc)
{
  using S = typename BV::S;
  Transform3<S> tf1;
  Transform3<S> tf2;
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
    node.min_distance = std::numeric_limits<S>::max();

    distanceRecurse(&node, 0, 0, nullptr);

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

//==============================================================================
template <typename S, typename Shape, typename NarrowPhaseSolver>
struct ConservativeAdvancementImpl
{
  static bool run(
      const Shape& o1,
      const MotionBase<S>* motion1,
      const BVHModel<RSS<S>>& o2,
      const MotionBase<S>* motion2,
      const NarrowPhaseSolver* nsolver,
      const CollisionRequest<S>& request,
      CollisionResult<S>& result,
      S& toc)
  {
    return detail::conservativeAdvancementShapeMeshOriented<Shape, RSS<S>, NarrowPhaseSolver, ShapeMeshConservativeAdvancementTraversalNodeRSS<Shape, NarrowPhaseSolver> >(o1, motion1, o2, motion2, nsolver, request, result, toc);
  }

  static bool run(
      const Shape& o1,
      const MotionBase<S>* motion1,
      const BVHModel<OBBRSS<S>>& o2,
      const MotionBase<S>* motion2,
      const NarrowPhaseSolver* nsolver,
      const CollisionRequest<S>& request,
      CollisionResult<S>& result,
      S& toc)
  {
    return detail::conservativeAdvancementShapeMeshOriented<Shape, OBBRSS<S>, NarrowPhaseSolver, ShapeMeshConservativeAdvancementTraversalNodeOBBRSS<Shape, NarrowPhaseSolver> >(o1, motion1, o2, motion2, nsolver, request, result, toc);
  }
};

template<typename Shape, typename NarrowPhaseSolver>
bool conservativeAdvancement(const Shape& o1,
                             const MotionBase<typename Shape::S>* motion1,
                             const BVHModel<RSS<typename Shape::S>>& o2,
                             const MotionBase<typename Shape::S>* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest<typename Shape::S>& request,
                             CollisionResult<typename Shape::S>& result,
                             typename Shape::S& toc)
{
  return ConservativeAdvancementImpl<
      typename Shape::S, Shape, NarrowPhaseSolver>::run(
        o1, motion1, o2, motion2, nsolver, request, result, toc);
}

template<typename Shape, typename NarrowPhaseSolver>
bool conservativeAdvancement(const Shape& o1,
                             const MotionBase<typename Shape::S>* motion1,
                             const BVHModel<OBBRSS<typename Shape::S>>& o2,
                             const MotionBase<typename Shape::S>* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest<typename Shape::S>& request,
                             CollisionResult<typename Shape::S>& result,
                             typename Shape::S& toc)
{
  return ConservativeAdvancementImpl<
      typename Shape::S, Shape, NarrowPhaseSolver>::run(
        o1, motion1, o2, motion2, nsolver, request, result, toc);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
struct ConservativeAdvancementImpl<S, BVHModel<RSS<S>>, NarrowPhaseSolver>
{
  static bool run(
      const BVHModel<RSS<S>>& o1,
      const MotionBase<S>* motion1,
      const BVHModel<RSS<S>>& o2,
      const MotionBase<S>* motion2,
      const NarrowPhaseSolver* /*nsolver*/,
      const CollisionRequest<S>& request,
      CollisionResult<S>& result,
      S& toc)
  {
    return detail::conservativeAdvancementMeshOriented<RSS<S>, MeshConservativeAdvancementTraversalNodeRSS<S>>(o1, motion1, o2, motion2, request, result, toc);
  }
};

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
struct ConservativeAdvancementImpl<S, BVHModel<OBBRSS<S>>, NarrowPhaseSolver>
{
  static bool run(
      const BVHModel<OBBRSS<S>>& o1,
      const MotionBase<S>* motion1,
      const BVHModel<OBBRSS<S>>& o2,
      const MotionBase<S>* motion2,
      const NarrowPhaseSolver* /*nsolver*/,
      const CollisionRequest<S>& request,
      CollisionResult<S>& result,
      S& toc)
  {
    return detail::conservativeAdvancementMeshOriented<OBBRSS<S>, MeshConservativeAdvancementTraversalNodeOBBRSS<S>>(o1, motion1, o2, motion2, request, result, toc);
  }
};

template<typename BV, typename NarrowPhaseSolver>
typename BV::S BVHConservativeAdvancement(const CollisionGeometry<typename BV::S>* o1, const MotionBase<typename BV::S>* motion1, const CollisionGeometry<typename BV::S>* o2, const MotionBase<typename BV::S>* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequest<typename BV::S>& request, ContinuousCollisionResult<typename BV::S>& result)
{
  FCL_UNUSED(nsolver);
  FCL_UNUSED(request);

  using S = typename BV::S;

  const BVHModel<BV>* obj1 = static_cast<const BVHModel<BV>*>(o1);
  const BVHModel<BV>* obj2 = static_cast<const BVHModel<BV>*>(o2);

  CollisionRequest<S> c_request;
  CollisionResult<S> c_result;
  S toc;
  bool is_collide = conservativeAdvancement(*obj1, motion1, *obj2, motion2, c_request, c_result, toc);

  result.is_collide = is_collide;
  result.time_of_contact = toc;

  return toc;
}

template<typename Shape1, typename Shape2, typename NarrowPhaseSolver>
typename Shape1::S ShapeConservativeAdvancement(const CollisionGeometry<typename Shape1::S>* o1, const MotionBase<typename Shape1::S>* motion1, const CollisionGeometry<typename Shape1::S>* o2, const MotionBase<typename Shape1::S>* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequest<typename Shape1::S>& request, ContinuousCollisionResult<typename Shape1::S>& result)
{
  FCL_UNUSED(request);

  using S = typename Shape1::S;

  const Shape1* obj1 = static_cast<const Shape1*>(o1);
  const Shape2* obj2 = static_cast<const Shape2*>(o2);

  CollisionRequest<S> c_request;
  CollisionResult<S> c_result;
  S toc;
  bool is_collide = conservativeAdvancement(*obj1, motion1, *obj2, motion2, nsolver, c_request, c_result, toc);

  result.is_collide = is_collide;
  result.time_of_contact = toc;

  return toc;
}

template<typename Shape, typename BV, typename NarrowPhaseSolver>
typename BV::S ShapeBVHConservativeAdvancement(const CollisionGeometry<typename BV::S>* o1, const MotionBase<typename BV::S>* motion1, const CollisionGeometry<typename BV::S>* o2, const MotionBase<typename BV::S>* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequest<typename BV::S>& request, ContinuousCollisionResult<typename BV::S>& result)
{
  FCL_UNUSED(request);

  using S = typename BV::S;

  const Shape* obj1 = static_cast<const Shape*>(o1);
  const BVHModel<BV>* obj2 = static_cast<const BVHModel<BV>*>(o2);

  CollisionRequest<S> c_request;
  CollisionResult<S> c_result;
  S toc;

  bool is_collide = conservativeAdvancement<Shape, BV, NarrowPhaseSolver>(*obj1, motion1, *obj2, motion2, nsolver, c_request, c_result, toc);

  result.is_collide = is_collide;
  result.time_of_contact = toc;

  return toc;
}

template<typename BV, typename Shape, typename NarrowPhaseSolver>
typename BV::S BVHShapeConservativeAdvancement(const CollisionGeometry<typename BV::S>* o1, const MotionBase<typename BV::S>* motion1, const CollisionGeometry<typename BV::S>* o2, const MotionBase<typename BV::S>* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequest<typename BV::S>& request, ContinuousCollisionResult<typename BV::S>& result)
{
  FCL_UNUSED(request);

  using S = typename BV::S;

  const BVHModel<BV>* obj1 = static_cast<const BVHModel<BV>*>(o1);
  const Shape* obj2 = static_cast<const Shape*>(o2);

  CollisionRequest<S> c_request;
  CollisionResult<S> c_result;
  S toc;

  bool is_collide = conservativeAdvancement<BV, Shape, NarrowPhaseSolver>(*obj1, motion1, *obj2, motion2, nsolver, c_request, c_result, toc);

  result.is_collide = is_collide;
  result.time_of_contact = toc;

  return toc;
}

template<typename NarrowPhaseSolver>
ConservativeAdvancementFunctionMatrix<NarrowPhaseSolver>::ConservativeAdvancementFunctionMatrix()
{
  using S = typename NarrowPhaseSolver::S;

  for(int i = 0; i < NODE_COUNT; ++i)
  {
    for(int j = 0; j < NODE_COUNT; ++j)
      conservative_advancement_matrix[i][j] = nullptr;
  }


  conservative_advancement_matrix[GEOM_BOX][GEOM_BOX] = &ShapeConservativeAdvancement<Box<S>, Box<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_SPHERE] = &ShapeConservativeAdvancement<Box<S>, Sphere<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Box<S>, Capsule<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_CONE] = &ShapeConservativeAdvancement<Box<S>, Cone<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Box<S>, Cylinder<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_CONVEX] = &ShapeConservativeAdvancement<Box<S>, Convex<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_PLANE] = &ShapeConservativeAdvancement<Box<S>, Plane<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_BOX][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Box<S>, Halfspace<S>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_SPHERE][GEOM_BOX] = &ShapeConservativeAdvancement<Sphere<S>, Box<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_SPHERE] = &ShapeConservativeAdvancement<Sphere<S>, Sphere<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Sphere<S>, Capsule<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_CONE] = &ShapeConservativeAdvancement<Sphere<S>, Cone<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Sphere<S>, Cylinder<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_CONVEX] = &ShapeConservativeAdvancement<Sphere<S>, Convex<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_PLANE] = &ShapeConservativeAdvancement<Sphere<S>, Plane<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Sphere<S>, Halfspace<S>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_BOX] = &ShapeConservativeAdvancement<Capsule<S>, Box<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_SPHERE] = &ShapeConservativeAdvancement<Capsule<S>, Sphere<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Capsule<S>, Capsule<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_CONE] = &ShapeConservativeAdvancement<Capsule<S>, Cone<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Capsule<S>, Cylinder<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_CONVEX] = &ShapeConservativeAdvancement<Capsule<S>, Convex<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_PLANE] = &ShapeConservativeAdvancement<Capsule<S>, Plane<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Capsule<S>, Halfspace<S>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_CONE][GEOM_BOX] = &ShapeConservativeAdvancement<Cone<S>, Box<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_SPHERE] = &ShapeConservativeAdvancement<Cone<S>, Sphere<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Cone<S>, Capsule<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_CONE] = &ShapeConservativeAdvancement<Cone<S>, Cone<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Cone<S>, Cylinder<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_CONVEX] = &ShapeConservativeAdvancement<Cone<S>, Convex<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_PLANE] = &ShapeConservativeAdvancement<Cone<S>, Plane<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Cone<S>, Halfspace<S>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_BOX] = &ShapeConservativeAdvancement<Cylinder<S>, Box<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_SPHERE] = &ShapeConservativeAdvancement<Cylinder<S>, Sphere<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Cylinder<S>, Capsule<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_CONE] = &ShapeConservativeAdvancement<Cylinder<S>, Cone<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Cylinder<S>, Cylinder<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_CONVEX] = &ShapeConservativeAdvancement<Cylinder<S>, Convex<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_PLANE] = &ShapeConservativeAdvancement<Cylinder<S>, Plane<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Cylinder<S>, Halfspace<S>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_CONVEX][GEOM_BOX] = &ShapeConservativeAdvancement<Convex<S>, Box<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_SPHERE] = &ShapeConservativeAdvancement<Convex<S>, Sphere<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Convex<S>, Capsule<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_CONE] = &ShapeConservativeAdvancement<Convex<S>, Cone<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Convex<S>, Cylinder<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_CONVEX] = &ShapeConservativeAdvancement<Convex<S>, Convex<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_PLANE] = &ShapeConservativeAdvancement<Convex<S>, Plane<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Convex<S>, Halfspace<S>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_PLANE][GEOM_BOX] = &ShapeConservativeAdvancement<Plane<S>, Box<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_SPHERE] = &ShapeConservativeAdvancement<Plane<S>, Sphere<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Plane<S>, Capsule<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_CONE] = &ShapeConservativeAdvancement<Plane<S>, Cone<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Plane<S>, Cylinder<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_CONVEX] = &ShapeConservativeAdvancement<Plane<S>, Convex<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_PLANE] = &ShapeConservativeAdvancement<Plane<S>, Plane<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Plane<S>, Halfspace<S>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_BOX] = &ShapeConservativeAdvancement<Halfspace<S>, Box<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_SPHERE] = &ShapeConservativeAdvancement<Halfspace<S>, Sphere<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_CAPSULE] = &ShapeConservativeAdvancement<Halfspace<S>, Capsule<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_CONE] = &ShapeConservativeAdvancement<Halfspace<S>, Cone<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_CYLINDER] = &ShapeConservativeAdvancement<Halfspace<S>, Cylinder<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_CONVEX] = &ShapeConservativeAdvancement<Halfspace<S>, Convex<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_PLANE] = &ShapeConservativeAdvancement<Halfspace<S>, Plane<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][GEOM_HALFSPACE] = &ShapeConservativeAdvancement<Halfspace<S>, Halfspace<S>, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_AABB][GEOM_BOX] = &BVHShapeConservativeAdvancement<AABB<S>, Box<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<AABB<S>, Sphere<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<AABB<S>, Capsule<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_CONE] = &BVHShapeConservativeAdvancement<AABB<S>, Cone<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<AABB<S>, Cylinder<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<AABB<S>, Convex<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_PLANE] = &BVHShapeConservativeAdvancement<AABB<S>, Plane<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_AABB][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<AABB<S>, Halfspace<S>, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_OBB][GEOM_BOX] = &BVHShapeConservativeAdvancement<OBB<S>, Box<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<OBB<S>, Sphere<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<OBB<S>, Capsule<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_CONE] = &BVHShapeConservativeAdvancement<OBB<S>, Cone<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<OBB<S>, Cylinder<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<OBB<S>, Convex<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_PLANE] = &BVHShapeConservativeAdvancement<OBB<S>, Plane<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<OBB<S>, Halfspace<S>, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_OBBRSS][GEOM_BOX] = &BVHShapeConservativeAdvancement<OBBRSS<S>, Box<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<OBBRSS<S>, Sphere<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<OBBRSS<S>, Capsule<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_CONE] = &BVHShapeConservativeAdvancement<OBBRSS<S>, Cone<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<OBBRSS<S>, Cylinder<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<OBBRSS<S>, Convex<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_PLANE] = &BVHShapeConservativeAdvancement<OBBRSS<S>, Plane<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<OBBRSS<S>, Halfspace<S>, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_RSS][GEOM_BOX] = &BVHShapeConservativeAdvancement<RSS<S>, Box<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<RSS<S>, Sphere<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<RSS<S>, Capsule<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_CONE] = &BVHShapeConservativeAdvancement<RSS<S>, Cone<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<RSS<S>, Cylinder<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<RSS<S>, Convex<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_PLANE] = &BVHShapeConservativeAdvancement<RSS<S>, Plane<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<RSS<S>, Halfspace<S>, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_KDOP16][GEOM_BOX] = &BVHShapeConservativeAdvancement<KDOP<S, 16>, Box<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<KDOP<S, 16>, Sphere<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<KDOP<S, 16>, Capsule<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_CONE] = &BVHShapeConservativeAdvancement<KDOP<S, 16>, Cone<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<KDOP<S, 16>, Cylinder<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<KDOP<S, 16>, Convex<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_PLANE] = &BVHShapeConservativeAdvancement<KDOP<S, 16>, Plane<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<KDOP<S, 16>, Halfspace<S>, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_KDOP18][GEOM_BOX] = &BVHShapeConservativeAdvancement<KDOP<S, 18>, Box<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<KDOP<S, 18>, Sphere<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<KDOP<S, 18>, Capsule<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_CONE] = &BVHShapeConservativeAdvancement<KDOP<S, 18>, Cone<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<KDOP<S, 18>, Cylinder<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<KDOP<S, 18>, Convex<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_PLANE] = &BVHShapeConservativeAdvancement<KDOP<S, 18>, Plane<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<KDOP<S, 18>, Halfspace<S>, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_KDOP24][GEOM_BOX] = &BVHShapeConservativeAdvancement<KDOP<S, 24>, Box<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<KDOP<S, 24>, Sphere<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<KDOP<S, 24>, Capsule<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_CONE] = &BVHShapeConservativeAdvancement<KDOP<S, 24>, Cone<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<KDOP<S, 24>, Cylinder<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<KDOP<S, 24>, Convex<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_PLANE] = &BVHShapeConservativeAdvancement<KDOP<S, 24>, Plane<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<KDOP<S, 24>, Halfspace<S>, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_kIOS][GEOM_BOX] = &BVHShapeConservativeAdvancement<kIOS<S>, Box<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_SPHERE] = &BVHShapeConservativeAdvancement<kIOS<S>, Sphere<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_CAPSULE] = &BVHShapeConservativeAdvancement<kIOS<S>, Capsule<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_CONE] = &BVHShapeConservativeAdvancement<kIOS<S>, Cone<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_CYLINDER] = &BVHShapeConservativeAdvancement<kIOS<S>, Cylinder<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_CONVEX] = &BVHShapeConservativeAdvancement<kIOS<S>, Convex<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_PLANE] = &BVHShapeConservativeAdvancement<kIOS<S>, Plane<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][GEOM_HALFSPACE] = &BVHShapeConservativeAdvancement<kIOS<S>, Halfspace<S>, NarrowPhaseSolver>;


  conservative_advancement_matrix[GEOM_BOX][BV_AABB] = &ShapeBVHConservativeAdvancement<Box<S>, AABB<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_AABB] = &ShapeBVHConservativeAdvancement<Sphere<S>, AABB<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_AABB] = &ShapeBVHConservativeAdvancement<Capsule<S>, AABB<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_AABB] = &ShapeBVHConservativeAdvancement<Cone<S>, AABB<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_AABB] = &ShapeBVHConservativeAdvancement<Cylinder<S>, AABB<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_AABB] = &ShapeBVHConservativeAdvancement<Convex<S>, AABB<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_AABB] = &ShapeBVHConservativeAdvancement<Plane<S>, AABB<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_AABB] = &ShapeBVHConservativeAdvancement<Halfspace<S>, AABB<S>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_OBB] = &ShapeBVHConservativeAdvancement<Box<S>, OBB<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_OBB] = &ShapeBVHConservativeAdvancement<Sphere<S>, OBB<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_OBB] = &ShapeBVHConservativeAdvancement<Capsule<S>, OBB<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_OBB] = &ShapeBVHConservativeAdvancement<Cone<S>, OBB<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_OBB] = &ShapeBVHConservativeAdvancement<Cylinder<S>, OBB<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_OBB] = &ShapeBVHConservativeAdvancement<Convex<S>, OBB<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_OBB] = &ShapeBVHConservativeAdvancement<Plane<S>, OBB<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_OBB] = &ShapeBVHConservativeAdvancement<Halfspace<S>, OBB<S>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_RSS] = &ShapeBVHConservativeAdvancement<Box<S>, RSS<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_RSS] = &ShapeBVHConservativeAdvancement<Sphere<S>, RSS<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_RSS] = &ShapeBVHConservativeAdvancement<Capsule<S>, RSS<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_RSS] = &ShapeBVHConservativeAdvancement<Cone<S>, RSS<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_RSS] = &ShapeBVHConservativeAdvancement<Cylinder<S>, RSS<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_RSS] = &ShapeBVHConservativeAdvancement<Convex<S>, RSS<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_RSS] = &ShapeBVHConservativeAdvancement<Plane<S>, RSS<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_RSS] = &ShapeBVHConservativeAdvancement<Halfspace<S>, RSS<S>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Box<S>, OBBRSS<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Sphere<S>, OBBRSS<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Capsule<S>, OBBRSS<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Cone<S>, OBBRSS<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Cylinder<S>, OBBRSS<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Convex<S>, OBBRSS<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Plane<S>, OBBRSS<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_OBBRSS] = &ShapeBVHConservativeAdvancement<Halfspace<S>, OBBRSS<S>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Box<S>, KDOP<S, 16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Sphere<S>, KDOP<S, 16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Capsule<S>, KDOP<S, 16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Cone<S>, KDOP<S, 16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Cylinder<S>, KDOP<S, 16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Convex<S>, KDOP<S, 16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Plane<S>, KDOP<S, 16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_KDOP16] = &ShapeBVHConservativeAdvancement<Halfspace<S>, KDOP<S, 16>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Box<S>, KDOP<S, 18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Sphere<S>, KDOP<S, 18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Capsule<S>, KDOP<S, 18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Cone<S>, KDOP<S, 18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Cylinder<S>, KDOP<S, 18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Convex<S>, KDOP<S, 18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Plane<S>, KDOP<S, 18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_KDOP18] = &ShapeBVHConservativeAdvancement<Halfspace<S>, KDOP<S, 18>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Box<S>, KDOP<S, 24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Sphere<S>, KDOP<S, 24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Capsule<S>, KDOP<S, 24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Cone<S>, KDOP<S, 24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Cylinder<S>, KDOP<S, 24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Convex<S>, KDOP<S, 24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Plane<S>, KDOP<S, 24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_KDOP24] = &ShapeBVHConservativeAdvancement<Halfspace<S>, KDOP<S, 24>, NarrowPhaseSolver>;

  conservative_advancement_matrix[GEOM_BOX][BV_kIOS] = &ShapeBVHConservativeAdvancement<Box<S>, kIOS<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_SPHERE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Sphere<S>, kIOS<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CAPSULE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Capsule<S>, kIOS<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Cone<S>, kIOS<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CYLINDER][BV_kIOS] = &ShapeBVHConservativeAdvancement<Cylinder<S>, kIOS<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_CONVEX][BV_kIOS] = &ShapeBVHConservativeAdvancement<Convex<S>, kIOS<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_PLANE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Plane<S>, kIOS<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[GEOM_HALFSPACE][BV_kIOS] = &ShapeBVHConservativeAdvancement<Halfspace<S>, kIOS<S>, NarrowPhaseSolver>;

  conservative_advancement_matrix[BV_AABB][BV_AABB] = &BVHConservativeAdvancement<AABB<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBB][BV_OBB] = &BVHConservativeAdvancement<OBB<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_RSS][BV_RSS] = &BVHConservativeAdvancement<RSS<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_OBBRSS][BV_OBBRSS] = &BVHConservativeAdvancement<OBBRSS<S>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP16][BV_KDOP16] = &BVHConservativeAdvancement<KDOP<S, 16>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP18][BV_KDOP18] = &BVHConservativeAdvancement<KDOP<S, 18>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_KDOP24][BV_KDOP24] = &BVHConservativeAdvancement<KDOP<S, 24>, NarrowPhaseSolver>;
  conservative_advancement_matrix[BV_kIOS][BV_kIOS] = &BVHConservativeAdvancement<kIOS<S>, NarrowPhaseSolver>;

}

} // namespace detail
} // namespace fcl

#endif

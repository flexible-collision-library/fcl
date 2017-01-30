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

#ifndef FCL_TRAVERSAL_SHAPEMESHCONSERVATIVEADVANCEMENTTRAVERSALNODE_INL_H
#define FCL_TRAVERSAL_SHAPEMESHCONSERVATIVEADVANCEMENTTRAVERSALNODE_INL_H

#include "fcl/narrowphase/detail/traversal/distance/shape_mesh_conservative_advancement_traversal_node.h"

#include "fcl/common/unused.h"

namespace fcl
{

namespace detail
{

//==============================================================================
template <typename Shape, typename BV, typename NarrowPhaseSolver>
ShapeMeshConservativeAdvancementTraversalNode<Shape, BV, NarrowPhaseSolver>::
ShapeMeshConservativeAdvancementTraversalNode(S w_)
  : ShapeMeshDistanceTraversalNode<Shape, BV, NarrowPhaseSolver>()
{
  delta_t = 1;
  toc = 0;
  t_err = (S)0.0001;

  w = w_;

  motion1 = nullptr;
  motion2 = nullptr;
}

//==============================================================================
template <typename Shape, typename BV, typename NarrowPhaseSolver>
typename BV::S
ShapeMeshConservativeAdvancementTraversalNode<Shape, BV, NarrowPhaseSolver>::
BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;
  Vector3<S> P1, P2;
  S d = this->model1_bv.distance(this->model2->getBV(b2).bv, &P1, &P2);

  stack.emplace_back(P1, P2, b1, b2, d);

  return d;
}

//==============================================================================
template <typename Shape, typename BV, typename NarrowPhaseSolver>
void ShapeMeshConservativeAdvancementTraversalNode<Shape, BV, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  FCL_UNUSED(b1);

  if(this->enable_statistics) this->num_leaf_tests++;

  const BVNode<BV>& node = this->model2->getBV(b2);

  int primitive_id = node.primitiveId();

  const Triangle& tri_id = this->tri_indices[primitive_id];

  const Vector3<S>& p1 = this->vertices[tri_id[0]];
  const Vector3<S>& p2 = this->vertices[tri_id[1]];
  const Vector3<S>& p3 = this->vertices[tri_id[2]];

  S d;
  Vector3<S> P1, P2;
  this->nsolver->shapeTriangleDistance(*(this->model1), this->tf1, p1, p2, p3, &d, &P1, &P2);

  if(d < this->min_distance)
  {
    this->min_distance = d;

    closest_p1 = P1;
    closest_p2 = P2;

    last_tri_id = primitive_id;
  }

  Vector3<S> n = P2 - this->tf1 * p1; n.normalize();
  // here n should be in global frame
  TBVMotionBoundVisitor<BV> mb_visitor1(this->model1_bv, n);
  TriangleMotionBoundVisitor<S> mb_visitor2(p1, p2, p3, -n);
  S bound1 = motion1->computeMotionBound(mb_visitor1);
  S bound2 = motion2->computeMotionBound(mb_visitor2);

  S bound = bound1 + bound2;

  S cur_delta_t;
  if(bound <= d) cur_delta_t = 1;
  else cur_delta_t = d / bound;

  if(cur_delta_t < delta_t)
    delta_t = cur_delta_t;
}

//==============================================================================
template <typename Shape, typename BV, typename NarrowPhaseSolver>
bool ShapeMeshConservativeAdvancementTraversalNode<Shape, BV, NarrowPhaseSolver>::
canStop(S c) const
{
  if((c >= w * (this->min_distance - this->abs_err)) && (c * (1 + this->rel_err) >= w * this->min_distance))
  {
    const auto& data = stack.back();

    Vector3<S> n = data.P2 - this->tf1 * data.P1; n.normalize();
    int c2 = data.c2;

    TBVMotionBoundVisitor<BV> mb_visitor1(this->model1_bv, n);
    TBVMotionBoundVisitor<BV> mb_visitor2(this->model2->getBV(c2).bv, -n);
    S bound1 = motion1->computeMotionBound(mb_visitor1);
    S bound2 = motion2->computeMotionBound(mb_visitor2);

    S bound = bound1 + bound2;

    S cur_delta_t;
    if(bound < c) cur_delta_t = 1;
    else cur_delta_t = c / bound;

    if(cur_delta_t < delta_t)
      delta_t = cur_delta_t;

    stack.pop_back();

    return true;
  }
  else
  {
    stack.pop_back();

    return false;
  }
}

//==============================================================================
template <typename Shape, typename BV, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshConservativeAdvancementTraversalNode<Shape, BV, NarrowPhaseSolver>& node,
    const Shape& model1,
    const Transform3<typename BV::S>& tf1,
    BVHModel<BV>& model2,
    const Transform3<typename BV::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    typename BV::S w,
    bool use_refit,
    bool refit_bottomup)
{
  using S = typename BV::S;

  std::vector<Vector3<S>> vertices_transformed(model2.num_vertices);
  for(int i = 0; i < model2.num_vertices; ++i)
  {
    Vector3<S>& p = model2.vertices[i];
    Vector3<S> new_v = tf2 * p;
    vertices_transformed[i] = new_v;
  }

  model2.beginReplaceModel();
  model2.replaceSubModel(vertices_transformed);
  model2.endReplaceModel(use_refit, refit_bottomup);

  node.model1 = &model1;
  node.model2 = &model2;

  node.vertices = model2.vertices;
  node.tri_indices = model2.tri_indices;

  node.tf1 = tf1;
  node.tf2 = tf2;

  node.nsolver = nsolver;
  node.w = w;

  computeBV(model1, Transform3<S>::Identity(), node.model1_bv);

  return true;
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
ShapeMeshConservativeAdvancementTraversalNodeRSS<Shape, NarrowPhaseSolver>::
ShapeMeshConservativeAdvancementTraversalNodeRSS(
    typename Shape::S w_)
  : ShapeMeshConservativeAdvancementTraversalNode<
    Shape, RSS<typename Shape::S>, NarrowPhaseSolver>(w_)
{
  // Do nothing
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
typename Shape::S
ShapeMeshConservativeAdvancementTraversalNodeRSS<Shape, NarrowPhaseSolver>::
BVTesting(int b1, int b2) const
{
  using S = typename Shape::S;

  if(this->enable_statistics) this->num_bv_tests++;
  Vector3<S> P1, P2;
  S d = distance(this->tf2.linear(), this->tf2.translation(), this->model2->getBV(b2).bv, this->model1_bv, &P2, &P1);

  this->stack.emplace_back(P1, P2, b1, b2, d);

  return d;
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void ShapeMeshConservativeAdvancementTraversalNodeRSS<Shape, NarrowPhaseSolver>::
leafTesting(int b1, int b2) const
{
  detail::meshShapeConservativeAdvancementOrientedNodeLeafTesting(
        b2,
        b1,
        this->model2,
        *(this->model1),
        this->model1_bv,
        this->vertices,
        this->tri_indices,
        this->tf2,
        this->tf1,
        this->motion2,
        this->motion1,
        this->nsolver,
        this->enable_statistics,
        this->min_distance,
        this->closest_p2,
        this->closest_p1,
        this->last_tri_id,
        this->delta_t,
        this->num_leaf_tests);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
bool ShapeMeshConservativeAdvancementTraversalNodeRSS<Shape, NarrowPhaseSolver>::
canStop(typename Shape::S c) const
{
  return detail::meshShapeConservativeAdvancementOrientedNodeCanStop(
        c,
        this->min_distance,
        this->abs_err,
        this->rel_err,
        this->w,
        this->model2,
        *(this->model1),
        this->model1_bv,
        this->motion2,
        this->motion1,
        this->stack,
        this->delta_t);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshConservativeAdvancementTraversalNodeRSS<Shape, NarrowPhaseSolver>& node,
    const Shape& model1,
    const Transform3<typename Shape::S>& tf1,
    const BVHModel<RSS<typename Shape::S>>& model2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    typename Shape::S w)
{
  using S = typename Shape::S;

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;
  node.nsolver = nsolver;

  node.w = w;

  computeBV(model1, Transform3<S>::Identity(), node.model1_bv);

  return true;
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
ShapeMeshConservativeAdvancementTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>::
ShapeMeshConservativeAdvancementTraversalNodeOBBRSS(
    typename Shape::S w_)
  : ShapeMeshConservativeAdvancementTraversalNode<
    Shape, OBBRSS<typename Shape::S>, NarrowPhaseSolver>(w_)
{
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
typename Shape::S
ShapeMeshConservativeAdvancementTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>::
BVTesting(int b1, int b2) const
{
  using S = typename Shape::S;

  if(this->enable_statistics) this->num_bv_tests++;
  Vector3<S> P1, P2;
  S d = distance(this->tf2.linear(), this->tf2.translation(), this->model2->getBV(b2).bv, this->model1_bv, &P2, &P1);

  this->stack.emplace_back(P1, P2, b1, b2, d);

  return d;
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void ShapeMeshConservativeAdvancementTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>::
leafTesting(int b1, int b2) const
{
  detail::meshShapeConservativeAdvancementOrientedNodeLeafTesting(
        b2,
        b1,
        this->model2,
        *(this->model1),
        this->model1_bv,
        this->vertices,
        this->tri_indices,
        this->tf2,
        this->tf1,
        this->motion2,
        this->motion1,
        this->nsolver,
        this->enable_statistics,
        this->min_distance,
        this->closest_p2,
        this->closest_p1,
        this->last_tri_id,
        this->delta_t,
        this->num_leaf_tests);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
bool ShapeMeshConservativeAdvancementTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>::
canStop(typename Shape::S c) const
{
  return detail::meshShapeConservativeAdvancementOrientedNodeCanStop(
        c,
        this->min_distance,
        this->abs_err,
        this->rel_err,
        this->w,
        this->model2,
        *(this->model1),
        this->model1_bv,
        this->motion2,
        this->motion1,
        this->stack,
        this->delta_t);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshConservativeAdvancementTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>& node,
    const Shape& model1,
    const Transform3<typename Shape::S>& tf1,
    const BVHModel<OBBRSS<typename Shape::S>>& model2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    typename Shape::S w)
{
  using S = typename Shape::S;

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;
  node.nsolver = nsolver;

  node.w = w;

  computeBV(model1, Transform3<S>::Identity(), node.model1_bv);

  return true;
}

} // namespace detail
} // namespace fcl

#endif

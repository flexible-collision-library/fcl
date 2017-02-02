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

#ifndef FCL_TRAVERSAL_MESHSHAPECONSERVATIVEADVANCEMENTTRAVERSALNODE_INL_H
#define FCL_TRAVERSAL_MESHSHAPECONSERVATIVEADVANCEMENTTRAVERSALNODE_INL_H

#include "fcl/narrowphase/detail/traversal/distance/mesh_shape_conservative_advancement_traversal_node.h"

#include "fcl/common/unused.h"

namespace fcl
{

namespace detail
{

//==============================================================================
template <typename BV, typename Shape, typename NarrowPhaseSolver>
MeshShapeConservativeAdvancementTraversalNode<BV, Shape, NarrowPhaseSolver>::
MeshShapeConservativeAdvancementTraversalNode(S w_) :
  MeshShapeDistanceTraversalNode<BV, Shape, NarrowPhaseSolver>()
{
  delta_t = 1;
  toc = 0;
  t_err = (S)0.0001;

  w = w_;

  motion1 = nullptr;
  motion2 = nullptr;
}

//==============================================================================
template <typename BV, typename Shape, typename NarrowPhaseSolver>
typename BV::S
MeshShapeConservativeAdvancementTraversalNode<BV, Shape, NarrowPhaseSolver>::
BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;
  Vector3<S> P1, P2;
  S d = this->model2_bv.distance(this->model1->getBV(b1).bv, &P2, &P1);

  stack.emplace_back(P1, P2, b1, b2, d);

  return d;
}

//==============================================================================
template <typename BV, typename Shape, typename NarrowPhaseSolver>
void MeshShapeConservativeAdvancementTraversalNode<BV, Shape, NarrowPhaseSolver>::
leafTesting(int b1, int b2) const
{
  FCL_UNUSED(b2);

  if(this->enable_statistics) this->num_leaf_tests++;

  const BVNode<BV>& node = this->model1->getBV(b1);

  int primitive_id = node.primitiveId();

  const Triangle& tri_id = this->tri_indices[primitive_id];

  const Vector3<S>& p1 = this->vertices[tri_id[0]];
  const Vector3<S>& p2 = this->vertices[tri_id[1]];
  const Vector3<S>& p3 = this->vertices[tri_id[2]];

  S d;
  Vector3<S> P1, P2;
  this->nsolver->shapeTriangleDistance(*(this->model2), this->tf2, p1, p2, p3, &d, &P2, &P1);

  if(d < this->min_distance)
  {
    this->min_distance = d;

    closest_p1 = P1;
    closest_p2 = P2;

    last_tri_id = primitive_id;
  }

  Vector3<S> n = this->tf2 * p2 - P1; n.normalize();
  // here n should be in global frame
  TriangleMotionBoundVisitor<S> mb_visitor1(p1, p2, p3, n);
  TBVMotionBoundVisitor<BV> mb_visitor2(this->model2_bv, -n);
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
template <typename BV, typename Shape, typename NarrowPhaseSolver>
bool MeshShapeConservativeAdvancementTraversalNode<BV, Shape, NarrowPhaseSolver>::
canStop(S c) const
{
  if((c >= w * (this->min_distance - this->abs_err))
     && (c * (1 + this->rel_err) >= w * this->min_distance))
  {
    const auto& data = stack.back();

    Vector3<S> n = this->tf2 * data.P2 - data.P1; n.normalize();
    int c1 = data.c1;

    TBVMotionBoundVisitor<BV> mb_visitor1(this->model1->getBV(c1).bv, n);
    TBVMotionBoundVisitor<BV> mb_visitor2(this->model2_bv, -n);
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
template <typename BV, typename Shape, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeConservativeAdvancementTraversalNode<BV, Shape, NarrowPhaseSolver>& node,
    BVHModel<BV>& model1,
    const Transform3<typename BV::S>& tf1,
    const Shape& model2,
    const Transform3<typename BV::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    typename BV::S w,
    bool use_refit,
    bool refit_bottomup)
{
  using S = typename BV::S;

  std::vector<Vector3<S>> vertices_transformed(model1.num_vertices);
  for(int i = 0; i < model1.num_vertices; ++i)
  {
    Vector3<S>& p = model1.vertices[i];
    Vector3<S> new_v = tf1 * p;
    vertices_transformed[i] = new_v;
  }

  model1.beginReplaceModel();
  model1.replaceSubModel(vertices_transformed);
  model1.endReplaceModel(use_refit, refit_bottomup);

  node.model1 = &model1;
  node.model2 = &model2;

  node.vertices = model1.vertices;
  node.tri_indices = model1.tri_indices;

  node.tf1 = tf1;
  node.tf2 = tf2;

  node.nsolver = nsolver;
  node.w = w;

  computeBV(model2, Transform3<S>::Identity(), node.model2_bv);

  return true;
}

//==============================================================================
template <typename BV, typename Shape, typename NarrowPhaseSolver>
void meshShapeConservativeAdvancementOrientedNodeLeafTesting(
    int b1,
    int /* b2 */,
    const BVHModel<BV>* model1,
    const Shape& model2,
    const BV& model2_bv,
    Vector3<typename BV::S>* vertices,
    Triangle* tri_indices,
    const Transform3<typename BV::S>& tf1,
    const Transform3<typename BV::S>& tf2,
    const MotionBase<typename BV::S>* motion1,
    const MotionBase<typename BV::S>* motion2,
    const NarrowPhaseSolver* nsolver,
    bool enable_statistics,
    typename BV::S& min_distance,
    Vector3<typename BV::S>& p1,
    Vector3<typename BV::S>& p2,
    int& last_tri_id,
    typename BV::S& delta_t,
    int& num_leaf_tests)
{
  using S = typename BV::S;

  if(enable_statistics) num_leaf_tests++;

  const BVNode<BV>& node = model1->getBV(b1);
  int primitive_id = node.primitiveId();

  const Triangle& tri_id = tri_indices[primitive_id];
  const Vector3<S>& t1 = vertices[tri_id[0]];
  const Vector3<S>& t2 = vertices[tri_id[1]];
  const Vector3<S>& t3 = vertices[tri_id[2]];

  S distance;
  Vector3<S> P1 = Vector3<S>::Zero();
  Vector3<S> P2 = Vector3<S>::Zero();
  nsolver->shapeTriangleDistance(model2, tf2, t1, t2, t3, tf1, &distance, &P2, &P1);

  if(distance < min_distance)
  {
    min_distance = distance;

    p1 = P1;
    p2 = P2;

    last_tri_id = primitive_id;
  }

  // n is in global frame
  Vector3<S> n = P2 - P1; n.normalize();

  TriangleMotionBoundVisitor<S> mb_visitor1(t1, t2, t3, n);
  TBVMotionBoundVisitor<BV> mb_visitor2(model2_bv, -n);
  S bound1 = motion1->computeMotionBound(mb_visitor1);
  S bound2 = motion2->computeMotionBound(mb_visitor2);

  S bound = bound1 + bound2;

  S cur_delta_t;
  if(bound <= distance) cur_delta_t = 1;
  else cur_delta_t = distance / bound;

  if(cur_delta_t < delta_t)
    delta_t = cur_delta_t;
}

//==============================================================================
template <typename BV, typename Shape>
bool meshShapeConservativeAdvancementOrientedNodeCanStop(
    typename BV::S c,
    typename BV::S min_distance,
    typename BV::S abs_err,
    typename BV::S rel_err,
    typename BV::S w,
    const BVHModel<BV>* model1,
    const Shape& model2,
    const BV& model2_bv,
    const MotionBase<typename BV::S>* motion1,
    const MotionBase<typename BV::S>* motion2,
    std::vector<ConservativeAdvancementStackData<typename BV::S>>& stack,
    typename BV::S& delta_t)
{
  FCL_UNUSED(model2);

  using S = typename BV::S;

  if((c >= w * (min_distance - abs_err)) && (c * (1 + rel_err) >= w * min_distance))
  {
    const auto& data = stack.back();
    Vector3<S> n = data.P2 - data.P1; n.normalize();
    int c1 = data.c1;

    TBVMotionBoundVisitor<BV> mb_visitor1(model1->getBV(c1).bv, n);
    TBVMotionBoundVisitor<BV> mb_visitor2(model2_bv, -n);

    S bound1 = motion1->computeMotionBound(mb_visitor1);
    S bound2 = motion2->computeMotionBound(mb_visitor2);

    S bound = bound1 + bound2;

    S cur_delta_t;
    if(bound <= c) cur_delta_t = 1;
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
template <typename Shape, typename NarrowPhaseSolver>
MeshShapeConservativeAdvancementTraversalNodeRSS<Shape, NarrowPhaseSolver>::
MeshShapeConservativeAdvancementTraversalNodeRSS(typename Shape::S w_)
  : MeshShapeConservativeAdvancementTraversalNode<
    RSS<S>, Shape, NarrowPhaseSolver>(w_)
{
  // Do nothing
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
typename Shape::S
MeshShapeConservativeAdvancementTraversalNodeRSS<Shape, NarrowPhaseSolver>::
BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;
  Vector3<S> P1, P2;
  S d = distance(this->tf1.linear(), this->tf1.translation(), this->model1->getBV(b1).bv, this->model2_bv, &P1, &P2);

  this->stack.emplace_back(P1, P2, b1, b2, d);

  return d;
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void
MeshShapeConservativeAdvancementTraversalNodeRSS<Shape, NarrowPhaseSolver>::
leafTesting(int b1, int b2) const
{
  detail::meshShapeConservativeAdvancementOrientedNodeLeafTesting(
        b1,
        b2,
        this->model1,
        *(this->model2),
        this->model2_bv,
        this->vertices,
        this->tri_indices,
        this->tf1,
        this->tf2,
        this->motion1,
        this->motion2,
        this->nsolver,
        this->enable_statistics,
        this->min_distance,
        this->closest_p1,
        this->closest_p2,
        this->last_tri_id,
        this->delta_t,
        this->num_leaf_tests);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
bool
MeshShapeConservativeAdvancementTraversalNodeRSS<Shape, NarrowPhaseSolver>::
canStop(typename Shape::S c) const
{
  return detail::meshShapeConservativeAdvancementOrientedNodeCanStop(
        c,
        this->min_distance,
        this->abs_err,
        this->rel_err,
        this->w,
        this->model1,
        *(this->model2),
        this->model2_bv,
        this->motion1,
        this->motion2,
        this->stack,
        this->delta_t);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeConservativeAdvancementTraversalNodeRSS<Shape, NarrowPhaseSolver>& node,
    const BVHModel<RSS<typename Shape::S>>& model1,
    const Transform3<typename Shape::S>& tf1,
    const Shape& model2,
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

  computeBV(model2, Transform3<S>::Identity(), node.model2_bv);

  return true;
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
MeshShapeConservativeAdvancementTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>::
MeshShapeConservativeAdvancementTraversalNodeOBBRSS(
    typename Shape::S w_)
  : MeshShapeConservativeAdvancementTraversalNode<
    OBBRSS<S>, Shape, NarrowPhaseSolver>(w_)
{
  // Do nothing
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
typename Shape::S
MeshShapeConservativeAdvancementTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>::
BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;
  Vector3<S> P1, P2;
  S d = distance(this->tf1.linear(), this->tf1.translation(), this->model1->getBV(b1).bv, this->model2_bv, &P1, &P2);

  this->stack.emplace_back(P1, P2, b1, b2, d);

  return d;
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void
MeshShapeConservativeAdvancementTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>::
leafTesting(int b1, int b2) const
{
  detail::meshShapeConservativeAdvancementOrientedNodeLeafTesting(
        b1,
        b2,
        this->model1,
        *(this->model2),
        this->model2_bv,
        this->vertices,
        this->tri_indices,
        this->tf1,
        this->tf2,
        this->motion1,
        this->motion2,
        this->nsolver,
        this->enable_statistics,
        this->min_distance,
        this->closest_p1,
        this->closest_p2,
        this->last_tri_id,
        this->delta_t,
        this->num_leaf_tests);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
bool
MeshShapeConservativeAdvancementTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>::
canStop(typename Shape::S c) const
{
  return detail::meshShapeConservativeAdvancementOrientedNodeCanStop(
        c,
        this->min_distance,
        this->abs_err,
        this->rel_err,
        this->w,
        this->model1,
        *(this->model2),
        this->model2_bv,
        this->motion1,
        this->motion2,
        this->stack,
        this->delta_t);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeConservativeAdvancementTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>& node,
    const BVHModel<OBBRSS<typename Shape::S>>& model1,
    const Transform3<typename Shape::S>& tf1,
    const Shape& model2,
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

  computeBV(model2, Transform3<S>::Identity(), node.model2_bv);

  return true;
}

} // namespace detail
} // namespace fcl

#endif

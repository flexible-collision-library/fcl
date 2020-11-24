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

#ifndef FCL_TRAVERSAL_SHAPECONSERVATIVEADVANCEMENTTRAVERSALNODE_INL_H
#define FCL_TRAVERSAL_SHAPECONSERVATIVEADVANCEMENTTRAVERSALNODE_INL_H

#include "fcl/narrowphase/detail/traversal/distance/shape_conservative_advancement_traversal_node.h"

namespace fcl
{

namespace detail
{

//==============================================================================
template <typename Shape1, typename Shape2, typename NarrowPhaseSolver>
ShapeConservativeAdvancementTraversalNode<Shape1, Shape2, NarrowPhaseSolver>::
ShapeConservativeAdvancementTraversalNode()
  : ShapeDistanceTraversalNode<Shape1, Shape2, NarrowPhaseSolver>()
{
  delta_t = 1;
  toc = 0;
  t_err = (S)0.0001;

  motion1 = nullptr;
  motion2 = nullptr;
}

//==============================================================================
template <typename Shape1, typename Shape2, typename NarrowPhaseSolver>
void ShapeConservativeAdvancementTraversalNode<Shape1, Shape2, NarrowPhaseSolver>::
leafTesting(int, int) const
{
  S distance;
  // NOTE(JS): The closest points are set to zeros in order to suppress the
  // maybe-uninitialized warning. It seems the warnings occur since
  // NarrowPhaseSolver::shapeDistance() conditionally set the closest points.
  // If this wasn't intentional then please remove the initialization of the
  // closest points, and change the function NarrowPhaseSolver::shapeDistance()
  // to always set the closest points.
  Vector3<S> closest_p1 = Vector3<S>::Zero();
  Vector3<S> closest_p2 = Vector3<S>::Zero();
  this->nsolver->shapeDistance(*(this->model1), this->tf1, *(this->model2), this->tf2, &distance, &closest_p1, &closest_p2);

  Vector3<S> n = closest_p2 - closest_p1;
  const S norm_sq = n.squaredNorm();
  if (norm_sq > 0.0)
    n /= sqrt(norm_sq);  // Normalize.
  TBVMotionBoundVisitor<RSS<S>> mb_visitor1(model1_bv, n);
  TBVMotionBoundVisitor<RSS<S>> mb_visitor2(model2_bv, -n);
  S bound1 = motion1->computeMotionBound(mb_visitor1);
  S bound2 = motion2->computeMotionBound(mb_visitor2);

  S bound = bound1 + bound2;

  S cur_delta_t;
  if(bound <= distance) cur_delta_t = 1;
  else cur_delta_t = distance / bound;

  if(cur_delta_t < delta_t)
    delta_t  = cur_delta_t;
}

//==============================================================================
template <typename Shape1, typename Shape2, typename NarrowPhaseSolver>
bool initialize(
    ShapeConservativeAdvancementTraversalNode<Shape1, Shape2, NarrowPhaseSolver>& node,
    const Shape1& shape1,
    const Transform3<typename Shape1::S>& tf1,
    const Shape2& shape2,
    const Transform3<typename Shape1::S>& tf2,
    const NarrowPhaseSolver* nsolver)
{
  using S = typename Shape1::S;

  node.model1 = &shape1;
  node.tf1 = tf1;
  node.model2 = &shape2;
  node.tf2 = tf2;
  node.nsolver = nsolver;

  computeBV(shape1, Transform3<S>::Identity(), node.model1_bv);
  computeBV(shape2, Transform3<S>::Identity(), node.model2_bv);

  return true;
}

} // namespace detail
} // namespace fcl

#endif

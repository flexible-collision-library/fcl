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

#ifndef FCL_TRAVERSAL_SHAPECONSERVATIVEADVANCEMENTTRAVERSALNODE_H
#define FCL_TRAVERSAL_SHAPECONSERVATIVEADVANCEMENTTRAVERSALNODE_H

#include "fcl/traversal/distance/shape_distance_traversal_node.h"

namespace fcl
{

template<typename S1, typename S2, typename NarrowPhaseSolver>
class ShapeConservativeAdvancementTraversalNode
    : public ShapeDistanceTraversalNode<S1, S2, NarrowPhaseSolver>
{
public:
  ShapeConservativeAdvancementTraversalNode();

  void leafTesting(int, int) const;

  mutable FCL_REAL min_distance;

  /// @brief The time from beginning point
  FCL_REAL toc;
  FCL_REAL t_err;

  /// @brief The delta_t each step
  mutable FCL_REAL delta_t;

  /// @brief Motions for the two objects in query
  const MotionBase* motion1;
  const MotionBase* motion2;

  RSSd model1_bv, model2_bv; // local bv for the two shapes
};

template <typename S1, typename S2, typename NarrowPhaseSolver>
bool initialize(
    ShapeConservativeAdvancementTraversalNode<S1, S2, NarrowPhaseSolver>& node,
    const S1& shape1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const S2& shape2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename S1, typename S2, typename NarrowPhaseSolver>
ShapeConservativeAdvancementTraversalNode<S1, S2, NarrowPhaseSolver>::
ShapeConservativeAdvancementTraversalNode()
  : ShapeDistanceTraversalNode<S1, S2, NarrowPhaseSolver>()
{
  delta_t = 1;
  toc = 0;
  t_err = (FCL_REAL)0.0001;

  motion1 = NULL;
  motion2 = NULL;
}

//==============================================================================
template <typename S1, typename S2, typename NarrowPhaseSolver>
void ShapeConservativeAdvancementTraversalNode<S1, S2, NarrowPhaseSolver>::
leafTesting(int, int) const
{
  FCL_REAL distance;
  Vector3d closest_p1, closest_p2;
  this->nsolver->shapeDistance(*(this->model1), this->tf1, *(this->model2), this->tf2, &distance, &closest_p1, &closest_p2);

  Vector3d n = this->tf2 * closest_p2 - this->tf1 * closest_p1;
  n.normalize();
  TBVMotionBoundVisitor<RSSd> mb_visitor1(model1_bv, n);
  TBVMotionBoundVisitor<RSSd> mb_visitor2(model2_bv, -n);
  FCL_REAL bound1 = motion1->computeMotionBound(mb_visitor1);
  FCL_REAL bound2 = motion2->computeMotionBound(mb_visitor2);

  FCL_REAL bound = bound1 + bound2;

  FCL_REAL cur_delta_t;
  if(bound <= distance) cur_delta_t = 1;
  else cur_delta_t = distance / bound;

  if(cur_delta_t < delta_t)
    delta_t  = cur_delta_t;
}

//==============================================================================
template <typename S1, typename S2, typename NarrowPhaseSolver>
bool initialize(
    ShapeConservativeAdvancementTraversalNode<S1, S2, NarrowPhaseSolver>& node,
    const S1& shape1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const S2& shape2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver)
{
  using Scalar = typename S1::Scalar;

  node.model1 = &shape1;
  node.tf1 = tf1;
  node.model2 = &shape2;
  node.tf2 = tf2;
  node.nsolver = nsolver;

  computeBV<Scalar, RSS<typename NarrowPhaseSolver::Scalar>, S1>(
        shape1,
        Transform3<typename NarrowPhaseSolver::Scalar>::Identity(),
        node.model1_bv);

  computeBV<Scalar, RSS<typename NarrowPhaseSolver::Scalar>, S2>(
        shape2,
        Transform3<typename NarrowPhaseSolver::Scalar>::Identity(),
        node.model2_bv);

  return true;
}

} // namespace fcl

#endif

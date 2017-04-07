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

#ifndef FCL_TRAVERSAL_SHAPEDISTANCETRAVERSALNODE_INL_H
#define FCL_TRAVERSAL_SHAPEDISTANCETRAVERSALNODE_INL_H

#include "fcl/narrowphase/detail/traversal/distance/shape_distance_traversal_node.h"

namespace fcl
{

namespace detail
{

//==============================================================================
template <typename Shape1, typename Shape2, typename NarrowPhaseSolver>
ShapeDistanceTraversalNode<Shape1, Shape2, NarrowPhaseSolver>::
ShapeDistanceTraversalNode() : DistanceTraversalNodeBase<typename Shape1::S>()
{
  model1 = nullptr;
  model2 = nullptr;

  nsolver = nullptr;
}

//==============================================================================
template <typename Shape1, typename Shape2, typename NarrowPhaseSolver>
typename Shape1::S
ShapeDistanceTraversalNode<Shape1, Shape2, NarrowPhaseSolver>::BVTesting(int, int) const
{
  return -1; // should not be used
}

//==============================================================================
template <typename Shape1, typename Shape2, typename NarrowPhaseSolver>
void ShapeDistanceTraversalNode<Shape1, Shape2, NarrowPhaseSolver>::leafTesting(
    int, int) const
{
  using S = typename Shape1::S;

  S distance;
  // NOTE(JS): The closest points are set to zeros in order to suppress the
  // maybe-uninitialized warning. It seems the warnings occur since
  // NarrowPhaseSolver::shapeDistance() conditionally set the closest points.
  // If this wasn't intentional then please remove the initialization of the
  // closest points, and change the function NarrowPhaseSolver::shapeDistance()
  // to always set the closest points.
  Vector3<S> closest_p1 = Vector3<S>::Zero();
  Vector3<S> closest_p2 = Vector3<S>::Zero();

  if (this->request.enable_signed_distance == true)
  {
    nsolver->shapeSignedDistance(*model1, this->tf1, *model2, this->tf2, &distance, &closest_p1, &closest_p2);
  }
  else
  {
    nsolver->shapeDistance(*model1, this->tf1, *model2, this->tf2, &distance, &closest_p1, &closest_p2);
  }

  this->result->update(
        distance,
        model1,
        model2,
        DistanceResult<S>::NONE,
        DistanceResult<S>::NONE,
        closest_p1,
        closest_p2);
}

//==============================================================================
template <typename Shape1, typename Shape2, typename NarrowPhaseSolver>
bool initialize(
    ShapeDistanceTraversalNode<Shape1, Shape2, NarrowPhaseSolver>& node,
    const Shape1& shape1,
    const Transform3<typename Shape1::S>& tf1,
    const Shape2& shape2,
    const Transform3<typename Shape1::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename Shape1::S>& request,
    DistanceResult<typename Shape1::S>& result)
{
  node.request = request;
  node.result = &result;

  node.model1 = &shape1;
  node.tf1 = tf1;
  node.model2 = &shape2;
  node.tf2 = tf2;
  node.nsolver = nsolver;

  return true;
}

} // namespace detail
} // namespace fcl

#endif

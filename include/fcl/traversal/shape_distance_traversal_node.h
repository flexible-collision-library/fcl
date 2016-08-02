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

#ifndef FCL_TRAVERSAL_SHAPEDISTANCETRAVERSALNODE_H
#define FCL_TRAVERSAL_SHAPEDISTANCETRAVERSALNODE_H

#include "fcl/traversal/traversal_node_base.h"
#include "fcl/traversal/distance_traversal_node_base.h"

namespace fcl
{

/// @brief Traversal node for distance between two shapes
template<typename S1, typename S2, typename NarrowPhaseSolver>
class ShapeDistanceTraversalNode
    : public DistanceTraversalNodeBase<typename NarrowPhaseSolver::Scalar>
{
public:
  ShapeDistanceTraversalNode();

  /// @brief BV culling test in one BVTT node
  FCL_REAL BVTesting(int, int) const;

  /// @brief Distance testing between leaves (two shapes)
  void leafTesting(int, int) const;

  const S1* model1;
  const S2* model2;

  const NarrowPhaseSolver* nsolver;
};

/// @brief Initialize traversal node for distance between two geometric shapes
template <typename S1, typename S2, typename NarrowPhaseSolver>
bool initialize(
    ShapeDistanceTraversalNode<S1, S2, NarrowPhaseSolver>& node,
    const S1& shape1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const S2& shape2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename NarrowPhaseSolver::Scalar>& request,
    DistanceResult<typename NarrowPhaseSolver::Scalar>& result);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename S1, typename S2, typename NarrowPhaseSolver>
ShapeDistanceTraversalNode<S1, S2, NarrowPhaseSolver>::
ShapeDistanceTraversalNode() : DistanceTraversalNodeBase<typename NarrowPhaseSolver::Scalar>()
{
  model1 = NULL;
  model2 = NULL;

  nsolver = NULL;
}

//==============================================================================
template <typename S1, typename S2, typename NarrowPhaseSolver>
FCL_REAL ShapeDistanceTraversalNode<S1, S2, NarrowPhaseSolver>::BVTesting(
    int, int) const
{
  return -1; // should not be used
}

//==============================================================================
template <typename S1, typename S2, typename NarrowPhaseSolver>
void ShapeDistanceTraversalNode<S1, S2, NarrowPhaseSolver>::leafTesting(
    int, int) const
{
  FCL_REAL distance;
  Vector3d closest_p1, closest_p2;
  nsolver->shapeDistance(
        *model1, this->tf1, *model2, this->tf2, &distance, &closest_p1, &closest_p2);
  this->result->update(
        distance,
        model1,
        model2,
        DistanceResult<typename NarrowPhaseSolver::Scalar>::NONE,
        DistanceResult<typename NarrowPhaseSolver::Scalar>::NONE,
        closest_p1,
        closest_p2);
}

//==============================================================================
template <typename S1, typename S2, typename NarrowPhaseSolver>
bool initialize(
    ShapeDistanceTraversalNode<S1, S2, NarrowPhaseSolver>& node,
    const S1& shape1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const S2& shape2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename NarrowPhaseSolver::Scalar>& request,
    DistanceResult<typename NarrowPhaseSolver::Scalar>& result)
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

} // namespace fcl

#endif

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

#ifndef FCL_TRAVERSAL_OCTREE_OCTREESHAPEDISTANCETRAVERSALNODE_H
#define FCL_TRAVERSAL_OCTREE_OCTREESHAPEDISTANCETRAVERSALNODE_H

#include "fcl/config.h"
#if not(FCL_HAVE_OCTOMAP)
#error "This header requires fcl to be compiled with octomap support"
#endif

#include "fcl/octree.h"
#include "fcl/BVH/BVH_model.h"
#include "fcl/traversal/distance_traversal_node_base.h"
#include "fcl/traversal/octree/octree_solver.h"

namespace fcl
{

/// @brief Traversal node for octree-shape distance
template <typename S, typename NarrowPhaseSolver>
class OcTreeShapeDistanceTraversalNode
    : public DistanceTraversalNodeBase<typename NarrowPhaseSolver::Scalar>
{
public:

  using Scalar = typename NarrowPhaseSolver::Scalar;

  OcTreeShapeDistanceTraversalNode();

  Scalar BVTesting(int, int) const;

  void leafTesting(int, int) const;

  const OcTree* model1;
  const S* model2;

  const OcTreeSolver<NarrowPhaseSolver>* otsolver;
};

/// @brief Initialize traversal node for distance between one octree and one
/// shape, given current object transform
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    OcTreeShapeDistanceTraversalNode<S, NarrowPhaseSolver>& node,
    const OcTree& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const S& model2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const OcTreeSolver<NarrowPhaseSolver>* otsolver,
    const DistanceRequest<typename NarrowPhaseSolver::Scalar>& request,
    DistanceResult<typename NarrowPhaseSolver::Scalar>& result);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
OcTreeShapeDistanceTraversalNode<S, NarrowPhaseSolver>::
OcTreeShapeDistanceTraversalNode()
{
  model1 = NULL;
  model2 = NULL;

  otsolver = NULL;
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
typename NarrowPhaseSolver::Scalar
OcTreeShapeDistanceTraversalNode<S, NarrowPhaseSolver>::
BVTesting(int, int) const
{
  return -1;
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void OcTreeShapeDistanceTraversalNode<S, NarrowPhaseSolver>::
leafTesting(int, int) const
{
  otsolver->OcTreeShapeDistance(
        model1, *model2, this->tf1, this->tf2, this->request, *this->result);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    OcTreeShapeDistanceTraversalNode<S, NarrowPhaseSolver>& node,
    const OcTree& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const S& model2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const OcTreeSolver<NarrowPhaseSolver>* otsolver,
    const DistanceRequest<typename NarrowPhaseSolver::Scalar>& request,
    DistanceResult<typename NarrowPhaseSolver::Scalar>& result)
{
  node.request = request;
  node.result = &result;

  node.model1 = &model1;
  node.model2 = &model2;

  node.otsolver = otsolver;

  node.tf1 = tf1;
  node.tf2 = tf2;

  return true;
}

} // namespace fcl

#endif

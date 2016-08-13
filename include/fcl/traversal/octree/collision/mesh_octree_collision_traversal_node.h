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

#ifndef FCL_TRAVERSAL_OCTREE_MESHOCTREECOLLISIONTRAVERSALNODE_H
#define FCL_TRAVERSAL_OCTREE_MESHOCTREECOLLISIONTRAVERSALNODE_H

#include "fcl/config.h"
#if not(FCL_HAVE_OCTOMAP)
#error "This header requires fcl to be compiled with octomap support"
#endif

#include "fcl/octree.h"
#include "fcl/BVH/BVH_model.h"
#include "fcl/traversal/collision/collision_traversal_node_base.h"
#include "fcl/traversal/octree/octree_solver.h"

namespace fcl
{

/// @brief Traversal node for mesh-octree collision
template <typename BV, typename NarrowPhaseSolver>
class MeshOcTreeCollisionTraversalNode
    : public CollisionTraversalNodeBase<typename BV::S>
{
public:

  using S = typename BV::S;

  MeshOcTreeCollisionTraversalNode();

  bool BVTesting(int, int) const;

  void leafTesting(int, int) const;

  const BVHModel<BV>* model1;
  const OcTree<S>* model2;

  Transform3<S> tf1, tf2;

  const OcTreeSolver<NarrowPhaseSolver>* otsolver;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @brief Initialize traversal node for collision between one mesh and one
/// octree, given current object transform
template <typename BV, typename NarrowPhaseSolver>
bool initialize(
    MeshOcTreeCollisionTraversalNode<BV, NarrowPhaseSolver>& node,
    const BVHModel<BV>& model1,
    const Transform3<typename BV::S>& tf1,
    const OcTree<typename BV::S>& model2,
    const Transform3<typename BV::S>& tf2,
    const OcTreeSolver<NarrowPhaseSolver>* otsolver,
    const CollisionRequest<typename BV::S>& request,
    CollisionResult<typename BV::S>& result);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename BV, typename NarrowPhaseSolver>
MeshOcTreeCollisionTraversalNode<BV, NarrowPhaseSolver>::
MeshOcTreeCollisionTraversalNode()
{
  model1 = nullptr;
  model2 = nullptr;

  otsolver = nullptr;
}

//==============================================================================
template <typename BV, typename NarrowPhaseSolver>
bool MeshOcTreeCollisionTraversalNode<BV, NarrowPhaseSolver>::
BVTesting(int, int) const
{
  return false;
}

//==============================================================================
template <typename BV, typename NarrowPhaseSolver>
void MeshOcTreeCollisionTraversalNode<BV, NarrowPhaseSolver>::
leafTesting(int, int) const
{
  otsolver->OcTreeMeshIntersect(
        model2, model1, tf2, tf1, this->request, *this->result);
}

//==============================================================================
template <typename BV, typename NarrowPhaseSolver>
bool initialize(
    MeshOcTreeCollisionTraversalNode<BV, NarrowPhaseSolver>& node,
    const BVHModel<BV>& model1,
    const Transform3<typename BV::S>& tf1,
    const OcTree<typename BV::S>& model2,
    const Transform3<typename BV::S>& tf2,
    const OcTreeSolver<NarrowPhaseSolver>* otsolver,
    const CollisionRequest<typename BV::S>& request,
    CollisionResult<typename BV::S>& result)
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

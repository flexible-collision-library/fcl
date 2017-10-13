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

#ifndef FCL_TRAVERSAL_SHAPEMESHCONSERVATIVEADVANCEMENTTRAVERSALNODE_H
#define FCL_TRAVERSAL_SHAPEMESHCONSERVATIVEADVANCEMENTTRAVERSALNODE_H

#include "fcl/geometry/shape/utility.h"
#include "fcl/narrowphase/detail/traversal/distance/shape_mesh_distance_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/distance/conservative_advancement_stack_data.h"

namespace fcl
{

namespace detail
{

template <typename Shape, typename BV, typename NarrowPhaseSolver>
class FCL_EXPORT ShapeMeshConservativeAdvancementTraversalNode
    : public ShapeMeshDistanceTraversalNode<Shape, BV, NarrowPhaseSolver>
{
public:

  using S = typename BV::S;

  ShapeMeshConservativeAdvancementTraversalNode(S w_ = 1);

  /// @brief BV culling test in one BVTT node
  S BVTesting(int b1, int b2) const;

  /// @brief Conservative advancement testing between leaves (one triangle and one shape)
  void leafTesting(int b1, int b2) const;

  /// @brief Whether the traversal process can stop early
  bool canStop(S c) const;

  mutable S min_distance;

  mutable Vector3<S> closest_p1, closest_p2;

  mutable int last_tri_id;
  
   /// @brief CA controlling variable: early stop for the early iterations of CA
  S w;

  /// @brief The time from beginning point
  S toc;
  S t_err;

  /// @brief The delta_t each step
  mutable S delta_t;

  /// @brief Motions for the two objects in query
  const MotionBase<S>* motion1;
  const MotionBase<S>* motion2;

  mutable std::vector<ConservativeAdvancementStackData<S>> stack;
};

template <typename Shape, typename BV, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshConservativeAdvancementTraversalNode<Shape, BV, NarrowPhaseSolver>& node,
    const Shape& model1,
    const Transform3<typename BV::S>& tf1,
    BVHModel<BV>& model2,
    const Transform3<typename BV::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    typename BV::S w = 1,
    bool use_refit = false,
    bool refit_bottomup = false);

template <typename Shape, typename NarrowPhaseSolver>
class FCL_EXPORT ShapeMeshConservativeAdvancementTraversalNodeRSS
    : public ShapeMeshConservativeAdvancementTraversalNode<
    Shape, RSS<typename Shape::S>, NarrowPhaseSolver>
{
public:
  using S = typename Shape::S;

  ShapeMeshConservativeAdvancementTraversalNodeRSS(S w_ = 1);

  S BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  bool canStop(S c) const;
};

template <typename Shape, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshConservativeAdvancementTraversalNodeRSS<Shape, NarrowPhaseSolver>& node,
    const Shape& model1,
    const Transform3<typename Shape::S>& tf1,
    const BVHModel<RSS<typename Shape::S>>& model2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    typename Shape::S w = 1);

template <typename Shape, typename NarrowPhaseSolver>
class FCL_EXPORT ShapeMeshConservativeAdvancementTraversalNodeOBBRSS
    : public ShapeMeshConservativeAdvancementTraversalNode<
    Shape, OBBRSS<typename Shape::S>, NarrowPhaseSolver>
{
public:
  using S = typename Shape::S;

  ShapeMeshConservativeAdvancementTraversalNodeOBBRSS(S w_ = 1);

  S BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  bool canStop(S c) const;
};

template <typename Shape, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshConservativeAdvancementTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>& node,
    const Shape& model1,
    const Transform3<typename Shape::S>& tf1,
    const BVHModel<OBBRSS<typename Shape::S>>& model2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    typename Shape::S w = 1);

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/traversal/distance/shape_mesh_conservative_advancement_traversal_node-inl.h"

#endif

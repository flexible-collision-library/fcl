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

#ifndef FCL_TRAVERSAL_SHAPEDISTANCETRAVERSALNODE_H
#define FCL_TRAVERSAL_SHAPEDISTANCETRAVERSALNODE_H

#include "fcl/narrowphase/detail/traversal/traversal_node_base.h"
#include "fcl/narrowphase/detail/traversal/distance/distance_traversal_node_base.h"

namespace fcl
{

namespace detail
{

/// @brief Traversal node for distance between two shapes
template<typename Shape1, typename Shape2, typename NarrowPhaseSolver>
class ShapeDistanceTraversalNode
    : public DistanceTraversalNodeBase<typename Shape1::S>
{
public:
  using S = typename Shape1::S;

  ShapeDistanceTraversalNode();

  /// @brief BV culling test in one BVTT node
  S BVTesting(int, int) const;

  /// @brief Distance testing between leaves (two shapes)
  void leafTesting(int, int) const;

  const Shape1* model1;
  const Shape2* model2;

  const NarrowPhaseSolver* nsolver;
};

/// @brief Initialize traversal node for distance between two geometric shapes
template <typename Shape1, typename Shape2, typename NarrowPhaseSolver>
bool initialize(
    ShapeDistanceTraversalNode<Shape1, Shape2, NarrowPhaseSolver>& node,
    const Shape1& shape1,
    const Transform3<typename Shape1::S>& tf1,
    const Shape2& shape2,
    const Transform3<typename Shape1::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename Shape1::S>& request,
    DistanceResult<typename Shape1::S>& result);

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/traversal/distance/shape_distance_traversal_node-inl.h"

#endif

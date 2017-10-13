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

#ifndef FCL_TRAVERSAL_SHAPEMESHDISTANCETRAVERSALNODE_H
#define FCL_TRAVERSAL_SHAPEMESHDISTANCETRAVERSALNODE_H

#include "fcl/narrowphase/detail/traversal/distance/shape_bvh_distance_traversal_node.h"
#include "fcl/geometry/bvh/BVH_model.h"

namespace fcl
{

namespace detail
{

/// @brief Traversal node for distance between shape and mesh
template <typename Shape, typename BV, typename NarrowPhaseSolver>
class FCL_EXPORT ShapeMeshDistanceTraversalNode
    : public ShapeBVHDistanceTraversalNode<Shape, BV>
{ 
public:

  using S = typename BV::S;

  ShapeMeshDistanceTraversalNode();

  /// @brief Distance testing between leaves (one shape and one triangle)
  void leafTesting(int b1, int b2) const;

  /// @brief Whether the traversal process can stop early
  bool canStop(S c) const;

  Vector3<S>* vertices;
  Triangle* tri_indices;

  S rel_err;
  S abs_err;
    
  const NarrowPhaseSolver* nsolver;
};

/// @brief Initialize traversal node for distance computation between one shape
/// and one mesh, given the current transforms
template <typename Shape, typename BV, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshDistanceTraversalNode<Shape, BV, NarrowPhaseSolver>& node,
    const Shape& model1,
    const Transform3<typename BV::S>& tf1,
    BVHModel<BV>& model2,
    Transform3<typename BV::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename BV::S>& request,
    DistanceResult<typename BV::S>& result,
    bool use_refit = false,
    bool refit_bottomup = false);

template <typename Shape, typename NarrowPhaseSolver>
class FCL_EXPORT ShapeMeshDistanceTraversalNodeRSS
    : public ShapeMeshDistanceTraversalNode<
    Shape, RSS<typename Shape::S>, NarrowPhaseSolver>
{
public:

  using S = typename Shape::S;

  ShapeMeshDistanceTraversalNodeRSS();

  void preprocess();

  void postprocess();

  S BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

};

/// @brief Initialize traversal node for distance computation between one shape
/// and one mesh, specialized for RSS type
template <typename Shape, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshDistanceTraversalNodeRSS<Shape, NarrowPhaseSolver>& node,
    const Shape& model1,
    const Transform3<typename Shape::S>& tf1,
    const BVHModel<RSS<typename Shape::S>>& model2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename Shape::S>& request,
    DistanceResult<typename Shape::S>& result);

template <typename Shape, typename NarrowPhaseSolver>
class FCL_EXPORT ShapeMeshDistanceTraversalNodekIOS
    : public ShapeMeshDistanceTraversalNode<
    Shape, kIOS<typename Shape::S>, NarrowPhaseSolver>
{
public:

  using S = typename Shape::S;

  ShapeMeshDistanceTraversalNodekIOS();

  void preprocess();

  void postprocess();

  S BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;
  
};

/// @brief Initialize traversal node for distance computation between one shape
/// and one mesh, specialized for kIOS type
template <typename Shape, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshDistanceTraversalNodekIOS<Shape, NarrowPhaseSolver>& node,
    const Shape& model1,
    const Transform3<typename Shape::S>& tf1,
    const BVHModel<kIOS<typename Shape::S>>& model2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename Shape::S>& request,
    DistanceResult<typename Shape::S>& result);

template <typename Shape, typename NarrowPhaseSolver>
class FCL_EXPORT ShapeMeshDistanceTraversalNodeOBBRSS
    : public ShapeMeshDistanceTraversalNode<
    Shape, OBBRSS<typename Shape::S>, NarrowPhaseSolver>
{
public:

  using S = typename Shape::S;

  ShapeMeshDistanceTraversalNodeOBBRSS();

  void preprocess();

  void postprocess();

  S BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;
  
};

/// @brief Initialize traversal node for distance computation between one shape
/// and one mesh, specialized for OBBRSS type
template <typename Shape, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshDistanceTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>& node,
    const Shape& model1,
    const Transform3<typename Shape::S>& tf1,
    const BVHModel<OBBRSS<typename Shape::S>>& model2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename Shape::S>& request,
    DistanceResult<typename Shape::S>& result);

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/traversal/distance/shape_mesh_distance_traversal_node-inl.h"

#endif

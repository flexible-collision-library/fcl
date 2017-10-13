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

#ifndef FCL_TRAVERSAL_MESHCONSERVATIVEADVANCEMENTTRAVERSALNODE_H
#define FCL_TRAVERSAL_MESHCONSERVATIVEADVANCEMENTTRAVERSALNODE_H

#include "fcl/math/motion/tbv_motion_bound_visitor.h"
#include "fcl/narrowphase/detail/traversal/distance/mesh_distance_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/distance/conservative_advancement_stack_data.h"

namespace fcl
{

namespace detail
{

/// @brief continuous collision node using conservative advancement. when using this default version, must refit the BVH in current configuration (R_t, T_t) into default configuration
template <typename BV>
class FCL_EXPORT MeshConservativeAdvancementTraversalNode
    : public MeshDistanceTraversalNode<BV>
{
public:

  using S = typename BV::S;

  MeshConservativeAdvancementTraversalNode(S w_ = 1);

  /// @brief BV culling test in one BVTT node
  S BVTesting(int b1, int b2) const;

  /// @brief Conservative advancement testing between leaves (two triangles)
  void leafTesting(int b1, int b2) const;

  /// @brief Whether the traversal process can stop early
  bool canStop(S c) const;

  mutable S min_distance;
 
  mutable Vector3<S> closest_p1, closest_p2;
  
  mutable int last_tri_id1, last_tri_id2;

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

  template <typename, typename>
  friend struct CanStopImpl;
};

/// @brief Initialize traversal node for conservative advancement computation
/// between two meshes, given the current transforms
template <typename BV>
FCL_EXPORT
bool initialize(
    MeshConservativeAdvancementTraversalNode<BV>& node,
    BVHModel<BV>& model1,
    const Transform3<typename BV::S>& tf1,
    BVHModel<BV>& model2,
    const Transform3<typename BV::S>& tf2,
    typename BV::S w = 1,
    bool use_refit = false,
    bool refit_bottomup = false);

template <typename S>
class FCL_EXPORT MeshConservativeAdvancementTraversalNodeRSS
    : public MeshConservativeAdvancementTraversalNode<RSS<S>>
{
public:
  MeshConservativeAdvancementTraversalNodeRSS(S w_ = 1);

  S BVTesting(int b1, int b2) const
  {
    if (this->enable_statistics)
      this->num_bv_tests++;

    Vector3<S> P1, P2;
    S d = distance(
        R,
        T,
        this->model1->getBV(b1).bv,
        this->model2->getBV(b2).bv,
        &P1,
        &P2);

    this->stack.emplace_back(P1, P2, b1, b2, d);

    return d;
  }

  void leafTesting(int b1, int b2) const;

  bool canStop(S c) const;

  Matrix3<S> R;
  Vector3<S> T;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using MeshConservativeAdvancementTraversalNodeRSSf = MeshConservativeAdvancementTraversalNodeRSS<float>;
using MeshConservativeAdvancementTraversalNodeRSSd = MeshConservativeAdvancementTraversalNodeRSS<double>;

/// @brief Initialize traversal node for conservative advancement computation
/// between two meshes, given the current transforms, specialized for RSS
template <typename S>
FCL_EXPORT
bool initialize(
    MeshConservativeAdvancementTraversalNodeRSS<S>& node,
    const BVHModel<RSS<S>>& model1,
    const Transform3<S>& tf1,
    const BVHModel<RSS<S>>& model2,
    const Transform3<S>& tf2,
    S w = 1);

template <typename S>
class FCL_EXPORT MeshConservativeAdvancementTraversalNodeOBBRSS
    : public MeshConservativeAdvancementTraversalNode<OBBRSS<S>>
{
public:
  MeshConservativeAdvancementTraversalNodeOBBRSS(S w_ = 1);

  S BVTesting(int b1, int b2) const
  {
    if (this->enable_statistics)
      this->num_bv_tests++;

    Vector3<S> P1, P2;
    S d = distance(
        R,
        T,
        this->model1->getBV(b1).bv,
        this->model2->getBV(b2).bv,
        &P1,
        &P2);

    this->stack.emplace_back(P1, P2, b1, b2, d);

    return d;
  }

  void leafTesting(int b1, int b2) const;

  bool canStop(S c) const;

  Matrix3<S> R;
  Vector3<S> T;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using MeshConservativeAdvancementTraversalNodeOBBRSSf = MeshConservativeAdvancementTraversalNodeOBBRSS<float>;
using MeshConservativeAdvancementTraversalNodeOBBRSSd = MeshConservativeAdvancementTraversalNodeOBBRSS<double>;

template <typename S>
FCL_EXPORT
bool initialize(
    MeshConservativeAdvancementTraversalNodeOBBRSS<S>& node,
    const BVHModel<OBBRSS<S>>& model1,
    const Transform3<S>& tf1,
    const BVHModel<OBBRSS<S>>& model2,
    const Transform3<S>& tf2,
    S w = 1);

template <typename S, typename BV>
FCL_EXPORT
const Vector3<S> getBVAxis(const BV& bv, int i);

template <typename BV>
FCL_EXPORT
bool meshConservativeAdvancementTraversalNodeCanStop(
    typename BV::S c,
    typename BV::S min_distance,
    typename BV::S abs_err,
    typename BV::S rel_err,
    typename BV::S w,
    const BVHModel<BV>* model1,
    const BVHModel<BV>* model2,
    const MotionBase<typename BV::S>* motion1,
    const MotionBase<typename BV::S>* motion2,
    std::vector<ConservativeAdvancementStackData<typename BV::S>>& stack,
    typename BV::S& delta_t);

template <typename BV>
FCL_EXPORT
bool meshConservativeAdvancementOrientedNodeCanStop(
    typename BV::S c,
    typename BV::S min_distance,
    typename BV::S abs_err,
    typename BV::S rel_err,
    typename BV::S w,
    const BVHModel<BV>* model1,
    const BVHModel<BV>* model2,
    const MotionBase<typename BV::S>* motion1,
    const MotionBase<typename BV::S>* motion2,
    std::vector<ConservativeAdvancementStackData<typename BV::S>>& stack,
    typename BV::S& delta_t);

template <typename BV>
FCL_EXPORT
void meshConservativeAdvancementOrientedNodeLeafTesting(
    int b1,
    int b2,
    const BVHModel<BV>* model1,
    const BVHModel<BV>* model2,
    const Triangle* tri_indices1,
    const Triangle* tri_indices2,
    const Vector3<typename BV::S>* vertices1,
    const Vector3<typename BV::S>* vertices2,
    const Matrix3<typename BV::S>& R,
    const Vector3<typename BV::S>& T,
    const MotionBase<typename BV::S>* motion1,
    const MotionBase<typename BV::S>* motion2,
    bool enable_statistics,
    typename BV::S& min_distance,
    Vector3<typename BV::S>& p1,
    Vector3<typename BV::S>& p2,
    int& last_tri_id1,
    int& last_tri_id2,
    typename BV::S& delta_t,
    int& num_leaf_tests);

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/traversal/distance/mesh_conservative_advancement_traversal_node-inl.h"

#endif

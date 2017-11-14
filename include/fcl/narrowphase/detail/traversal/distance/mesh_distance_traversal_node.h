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

#ifndef FCL_TRAVERSAL_MESHDISTANCETRAVERSALNODE_H
#define FCL_TRAVERSAL_MESHDISTANCETRAVERSALNODE_H

#include "fcl/narrowphase/detail/primitive_shape_algorithm/triangle_distance.h"
#include "fcl/math/bv/RSS.h"
#include "fcl/math/bv/OBBRSS.h"
#include "fcl/math/bv/kIOS.h"
#include "fcl/narrowphase/detail/traversal/distance/bvh_distance_traversal_node.h"

namespace fcl
{

namespace detail
{

/// @brief Traversal node for distance computation between two meshes
template <typename BV>
class FCL_EXPORT MeshDistanceTraversalNode : public BVHDistanceTraversalNode<BV>
{
public:

  using S = typename BV::S;

  MeshDistanceTraversalNode();

  /// @brief Distance testing between leaves (two triangles)
  void leafTesting(int b1, int b2) const;

  /// @brief Whether the traversal process can stop early
  bool canStop(S c) const;

  Vector3<S>* vertices1;
  Vector3<S>* vertices2;

  Triangle* tri_indices1;
  Triangle* tri_indices2;

  /// @brief relative and absolute error, default value is 0.01 for both terms
  S rel_err;
  S abs_err;
};

/// @brief Initialize traversal node for distance computation between two
/// meshes, given the current transforms
template <typename BV>
FCL_EXPORT
bool initialize(
    MeshDistanceTraversalNode<BV>& node,
    BVHModel<BV>& model1,
    Transform3<typename BV::S>& tf1,
    BVHModel<BV>& model2,
    Transform3<typename BV::S>& tf2,
    const DistanceRequest<typename BV::S>& request,
    DistanceResult<typename BV::S>& result,
    bool use_refit = false, bool refit_bottomup = false);

/// @brief Traversal node for distance computation between two meshes if their underlying BVH node is oriented node (RSS, OBBRSS, kIOS)
template <typename S>
class FCL_EXPORT MeshDistanceTraversalNodeRSS
    : public MeshDistanceTraversalNode<RSS<S>>
{
public:
  MeshDistanceTraversalNodeRSS();

  void preprocess();

  void postprocess();

  S BVTesting(int b1, int b2) const
  {
    if (this->enable_statistics) this->num_bv_tests++;

    return distance(tf.linear(), tf.translation(), this->model1->getBV(b1).bv, this->model2->getBV(b2).bv);
  }

  void leafTesting(int b1, int b2) const;

  Transform3<S> tf;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using MeshDistanceTraversalNodeRSSf = MeshDistanceTraversalNodeRSS<float>;
using MeshDistanceTraversalNodeRSSd = MeshDistanceTraversalNodeRSS<double>;

/// @brief Initialize traversal node for distance computation between two
///  meshes, specialized for RSS type
template <typename S>
FCL_EXPORT
bool initialize(
    MeshDistanceTraversalNodeRSS<S>& node,
    const BVHModel<RSS<S>>& model1,
    const Transform3<S>& tf1,
    const BVHModel<RSS<S>>& model2,
    const Transform3<S>& tf2,
    const DistanceRequest<S>& request,
    DistanceResult<S>& result);

template <typename S>
class FCL_EXPORT MeshDistanceTraversalNodekIOS
    : public MeshDistanceTraversalNode<kIOS<S>>
{
public:
  MeshDistanceTraversalNodekIOS();

  void preprocess();
  
  void postprocess();

  S BVTesting(int b1, int b2) const
  {
    if (this->enable_statistics) this->num_bv_tests++;

    return distance(tf.linear(), tf.translation(), this->model1->getBV(b1).bv, this->model2->getBV(b2).bv);
  }

  void leafTesting(int b1, int b2) const;

  Transform3<S> tf;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using MeshDistanceTraversalNodekIOSf = MeshDistanceTraversalNodekIOS<float>;
using MeshDistanceTraversalNodekIOSd = MeshDistanceTraversalNodekIOS<double>;

/// @brief Initialize traversal node for distance computation between two
///  meshes, specialized for kIOS type
template <typename S>
FCL_EXPORT
bool initialize(
    MeshDistanceTraversalNodekIOS<S>& node,
    const BVHModel<kIOS<S>>& model1,
    const Transform3<S>& tf1,
    const BVHModel<kIOS<S>>& model2,
    const Transform3<S>& tf2,
    const DistanceRequest<S>& request,
    DistanceResult<S>& result);

template <typename S>
class FCL_EXPORT MeshDistanceTraversalNodeOBBRSS
    : public MeshDistanceTraversalNode<OBBRSS<S>>
{
public:
  MeshDistanceTraversalNodeOBBRSS();

  void preprocess();

  void postprocess();

  S BVTesting(int b1, int b2) const
  {
    if (this->enable_statistics) this->num_bv_tests++;

    return distance(tf.linear(), tf.translation(), this->model1->getBV(b1).bv, this->model2->getBV(b2).bv);
  }

  void leafTesting(int b1, int b2) const;

  Transform3<S> tf;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using MeshDistanceTraversalNodeOBBRSSf = MeshDistanceTraversalNodeOBBRSS<float>;
using MeshDistanceTraversalNodeOBBRSSd = MeshDistanceTraversalNodeOBBRSS<double>;

/// @brief Initialize traversal node for distance computation between two
///  meshes, specialized for OBBRSS type
template <typename S>
FCL_EXPORT
bool initialize(
    MeshDistanceTraversalNodeOBBRSS<S>& node,
    const BVHModel<OBBRSS<S>>& model1,
    const Transform3<S>& tf1,
    const BVHModel<OBBRSS<S>>& model2,
    const Transform3<S>& tf2,
    const DistanceRequest<S>& request,
    DistanceResult<S>& result);

template <typename BV>
FCL_DEPRECATED_EXPORT
void meshDistanceOrientedNodeLeafTesting(
    int b1,
    int b2,
    const BVHModel<BV>* model1,
    const BVHModel<BV>* model2,
    Vector3<typename BV::S>* vertices1,
    Vector3<typename BV::S>* vertices2,
    Triangle* tri_indices1,
    Triangle* tri_indices2,
    const Matrix3<typename BV::S>& R,
    const Vector3<typename BV::S>& T,
    bool enable_statistics,
    int& num_leaf_tests,
    const DistanceRequest<typename BV::S>& request,
    DistanceResult<typename BV::S>& result);

template <typename BV>
FCL_EXPORT
void meshDistanceOrientedNodeLeafTesting(
    int b1,
    int b2,
    const BVHModel<BV>* model1,
    const BVHModel<BV>* model2,
    Vector3<typename BV::S>* vertices1,
    Vector3<typename BV::S>* vertices2,
    Triangle* tri_indices1,
    Triangle* tri_indices2,
    const Transform3<typename BV::S>& tf,
    bool enable_statistics,
    int& num_leaf_tests,
    const DistanceRequest<typename BV::S>& request,
    DistanceResult<typename BV::S>& result);

template <typename BV>
FCL_EXPORT
void distancePreprocessOrientedNode(
    const BVHModel<BV>* model1,
    const BVHModel<BV>* model2,
    const Vector3<typename BV::S>* vertices1,
    Vector3<typename BV::S>* vertices2,
    Triangle* tri_indices1,
    Triangle* tri_indices2,
    int init_tri_id1,
    int init_tri_id2,
    const Matrix3<typename BV::S>& R,
    const Vector3<typename BV::S>& T,
    const DistanceRequest<typename BV::S>& request,
    DistanceResult<typename BV::S>& result);

template <typename BV>
FCL_EXPORT
void distancePreprocessOrientedNode(
    const BVHModel<BV>* model1,
    const BVHModel<BV>* model2,
    const Vector3<typename BV::S>* vertices1,
    Vector3<typename BV::S>* vertices2,
    Triangle* tri_indices1,
    Triangle* tri_indices2,
    int init_tri_id1,
    int init_tri_id2,
    const Transform3<typename BV::S>& tf,
    const DistanceRequest<typename BV::S>& request,
    DistanceResult<typename BV::S>& result);

template <typename BV>
FCL_EXPORT
void distancePostprocessOrientedNode(
    const BVHModel<BV>* model1,
    const BVHModel<BV>* model2,
    const Transform3<typename BV::S>& tf1,
    const DistanceRequest<typename BV::S>& request,
    DistanceResult<typename BV::S>& result);

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/traversal/distance/mesh_distance_traversal_node-inl.h"

#endif

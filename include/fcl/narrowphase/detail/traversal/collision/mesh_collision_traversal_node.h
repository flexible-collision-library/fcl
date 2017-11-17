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

#ifndef FCL_TRAVERSAL_MESHCOLLISIONTRAVERSALNODE_H
#define FCL_TRAVERSAL_MESHCOLLISIONTRAVERSALNODE_H

#include "fcl/math/bv/OBB.h"
#include "fcl/math/bv/RSS.h"
#include "fcl/math/bv/OBBRSS.h"
#include "fcl/math/bv/kIOS.h"
#include "fcl/narrowphase/contact.h"
#include "fcl/narrowphase/cost_source.h"
#include "fcl/narrowphase/detail/traversal/collision/intersect.h"
#include "fcl/narrowphase/detail/traversal/collision/bvh_collision_traversal_node.h"

namespace fcl
{

namespace detail
{

/// @brief Traversal node for collision between two meshes
template <typename BV>
class FCL_EXPORT MeshCollisionTraversalNode : public BVHCollisionTraversalNode<BV>
{
public:

  using S = typename BV::S;

  MeshCollisionTraversalNode();

  /// @brief Intersection testing between leaves (two triangles)
  void leafTesting(int b1, int b2) const;

  /// @brief Whether the traversal process can stop early
  bool canStop() const;

  Vector3<S>* vertices1;
  Vector3<S>* vertices2;

  Triangle* tri_indices1;
  Triangle* tri_indices2;

  S cost_density;
};

/// @brief Initialize traversal node for collision between two meshes, given the
/// current transforms
template <typename BV>
FCL_EXPORT
bool initialize(
    MeshCollisionTraversalNode<BV>& node,
    BVHModel<BV>& model1,
    Transform3<typename BV::S>& tf1,
    BVHModel<BV>& model2,
    Transform3<typename BV::S>& tf2,
    const CollisionRequest<typename BV::S>& request,
    CollisionResult<typename BV::S>& result,
    bool use_refit = false,
    bool refit_bottomup = false);

/// @brief Traversal node for collision between two meshes if their underlying
/// BVH node is oriented node (OBB, RSS, OBBRSS, kIOS)
template <typename S>
class FCL_EXPORT MeshCollisionTraversalNodeOBB : public MeshCollisionTraversalNode<OBB<S>>
{
public:
  MeshCollisionTraversalNodeOBB();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  bool BVTesting(int b1, int b2, const Matrix3<S>& Rc, const Vector3<S>& Tc) const;

  bool BVTesting(int b1, int b2, const Transform3<S>& tf) const;

  void leafTesting(int b1, int b2, const Matrix3<S>& Rc, const Vector3<S>& Tc) const;

  void leafTesting(int b1, int b2, const Transform3<S>& tf) const;

  Matrix3<S> R;
  Vector3<S> T;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using MeshCollisionTraversalNodeOBBf = MeshCollisionTraversalNodeOBB<float>;
using MeshCollisionTraversalNodeOBBd = MeshCollisionTraversalNodeOBB<double>;

/// @brief Initialize traversal node for collision between two meshes,
/// specialized for OBB type
template <typename S>
FCL_EXPORT
bool initialize(
    MeshCollisionTraversalNodeOBB<S>& node,
    const BVHModel<OBB<S>>& model1,
    const Transform3<S>& tf1,
    const BVHModel<OBB<S>>& model2,
    const Transform3<S>& tf2,
    const CollisionRequest<S>& request,
    CollisionResult<S>& result);

template <typename S>
class FCL_EXPORT MeshCollisionTraversalNodeRSS : public MeshCollisionTraversalNode<RSS<S>>
{
public:
  MeshCollisionTraversalNodeRSS();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

//  FCL_DEPRECATED
//  bool BVTesting(int b1, int b2, const Matrix3<S>& Rc, const Vector3<S>& Tc) const;

  bool BVTesting(int b1, int b2, const Transform3<S>& tf) const;

//  FCL_DEPRECATED
//  void leafTesting(int b1, int b2, const Matrix3<S>& Rc, const Vector3<S>& Tc) const;

  void leafTesting(int b1, int b2, const Transform3<S>& tf) const;

  Matrix3<S> R;
  Vector3<S> T;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using MeshCollisionTraversalNodeRSSf = MeshCollisionTraversalNodeRSS<float>;
using MeshCollisionTraversalNodeRSSd = MeshCollisionTraversalNodeRSS<double>;

/// @brief Initialize traversal node for collision between two meshes,
/// specialized for RSS type
template <typename S>
FCL_EXPORT
bool initialize(
    MeshCollisionTraversalNodeRSS<S>& node,
    const BVHModel<RSS<S>>& model1,
    const Transform3<S>& tf1,
    const BVHModel<RSS<S>>& model2,
    const Transform3<S>& tf2,
    const CollisionRequest<S>& request,
    CollisionResult<S>& result);

template <typename S>
class FCL_EXPORT MeshCollisionTraversalNodekIOS : public MeshCollisionTraversalNode<kIOS<S>>
{
public:
  MeshCollisionTraversalNodekIOS();
 
  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  Matrix3<S> R;
  Vector3<S> T;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using MeshCollisionTraversalNodekIOSf = MeshCollisionTraversalNodekIOS<float>;
using MeshCollisionTraversalNodekIOSd = MeshCollisionTraversalNodekIOS<double>;

/// @brief Initialize traversal node for collision between two meshes,
/// specialized for kIOS type
template <typename S>
FCL_EXPORT
bool initialize(
    MeshCollisionTraversalNodekIOS<S>& node,
    const BVHModel<kIOS<S>>& model1,
    const Transform3<S>& tf1,
    const BVHModel<kIOS<S>>& model2,
    const Transform3<S>& tf2,
    const CollisionRequest<S>& request,
    CollisionResult<S>& result);

template <typename S>
class FCL_EXPORT MeshCollisionTraversalNodeOBBRSS : public MeshCollisionTraversalNode<OBBRSS<S>>
{
public:
  MeshCollisionTraversalNodeOBBRSS();
 

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  Matrix3<S> R;
  Vector3<S> T;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using MeshCollisionTraversalNodeOBBRSSf = MeshCollisionTraversalNodeOBBRSS<float>;
using MeshCollisionTraversalNodeOBBRSSd = MeshCollisionTraversalNodeOBBRSS<double>;

/// @brief Initialize traversal node for collision between two meshes,
/// specialized for OBBRSS type
template <typename S>
FCL_EXPORT
bool initialize(
    MeshCollisionTraversalNodeOBBRSS<S>& node,
    const BVHModel<OBBRSS<S>>& model1,
    const Transform3<S>& tf1,
    const BVHModel<OBBRSS<S>>& model2,
    const Transform3<S>& tf2,
    const CollisionRequest<S>& request,
    CollisionResult<S>& result);

template <typename BV>
FCL_EXPORT
void meshCollisionOrientedNodeLeafTesting(
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
    const Transform3<typename BV::S>& tf1,
    const Transform3<typename BV::S>& tf2,
    bool enable_statistics,
    typename BV::S cost_density,
    int& num_leaf_tests,
    const CollisionRequest<typename BV::S>& request,
    CollisionResult<typename BV::S>& result);

template <typename BV>
FCL_EXPORT
void meshCollisionOrientedNodeLeafTesting(
    int b1,
    int b2,
    const BVHModel<BV>* model1,
    const BVHModel<BV>* model2,
    Vector3<typename BV::S>* vertices1,
    Vector3<typename BV::S>* vertices2,
    Triangle* tri_indices1,
    Triangle* tri_indices2,
    const Transform3<typename BV::S>& tf,
    const Transform3<typename BV::S>& tf1,
    const Transform3<typename BV::S>& tf2,
    bool enable_statistics,
    typename BV::S cost_density,
    int& num_leaf_tests,
    const CollisionRequest<typename BV::S>& request,
    CollisionResult<typename BV::S>& result);

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/traversal/collision/mesh_collision_traversal_node-inl.h"

#endif

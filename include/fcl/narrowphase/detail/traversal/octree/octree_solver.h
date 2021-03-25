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

#ifndef FCL_TRAVERSAL_OCTREE_OCTREESOLVER_H
#define FCL_TRAVERSAL_OCTREE_OCTREESOLVER_H

#include "fcl/config.h"
#if !(FCL_HAVE_OCTOMAP)
#error "This header requires fcl to be compiled with octomap support"
#endif

#include "fcl/math/bv/utility.h"
#include "fcl/geometry/octree/octree.h"
#include "fcl/geometry/shape/utility.h"
#include "fcl/geometry/shape/box.h"

namespace fcl
{

namespace detail
{

/// @brief Algorithms for collision related with octree
template <typename NarrowPhaseSolver>
class FCL_EXPORT OcTreeSolver
{
private:

  using S = typename NarrowPhaseSolver::S;

  const NarrowPhaseSolver* solver;

  mutable const CollisionRequest<S>* crequest;
  mutable const DistanceRequest<S>* drequest;

  mutable CollisionResult<S>* cresult;
  mutable DistanceResult<S>* dresult;

public:
  OcTreeSolver(const NarrowPhaseSolver* solver_);

  /// @brief collision between two octrees
  void OcTreeIntersect(const OcTree<S>* tree1, const OcTree<S>* tree2,
                       const Transform3<S>& tf1, const Transform3<S>& tf2,
                       const CollisionRequest<S>& request_,
                       CollisionResult<S>& result_) const;

  /// @brief distance between two octrees
  void OcTreeDistance(const OcTree<S>* tree1, const OcTree<S>* tree2,
                      const Transform3<S>& tf1, const Transform3<S>& tf2,
                      const DistanceRequest<S>& request_,
                      DistanceResult<S>& result_) const;

  /// @brief collision between octree and mesh
  template <typename BV>
  void OcTreeMeshIntersect(const OcTree<S>* tree1, const BVHModel<BV>* tree2,
                           const Transform3<S>& tf1, const Transform3<S>& tf2,
                           const CollisionRequest<S>& request_,
                           CollisionResult<S>& result_) const;

  /// @brief distance between octree and mesh
  template <typename BV>
  void OcTreeMeshDistance(const OcTree<S>* tree1, const BVHModel<BV>* tree2,
                          const Transform3<S>& tf1, const Transform3<S>& tf2,
                          const DistanceRequest<S>& request_,
                          DistanceResult<S>& result_) const;

  /// @brief collision between mesh and octree
  template <typename BV>
  void MeshOcTreeIntersect(const BVHModel<BV>* tree1, const OcTree<S>* tree2,
                           const Transform3<S>& tf1, const Transform3<S>& tf2,
                           const CollisionRequest<S>& request_,
                           CollisionResult<S>& result_) const;

  /// @brief distance between mesh and octree
  template <typename BV>
  void MeshOcTreeDistance(const BVHModel<BV>* tree1, const OcTree<S>* tree2,
                          const Transform3<S>& tf1, const Transform3<S>& tf2,
                          const DistanceRequest<S>& request_,
                          DistanceResult<S>& result_) const;

  /// @brief collision between octree and shape
  template <typename Shape>
  void OcTreeShapeIntersect(const OcTree<S>* tree, const Shape& s,
                            const Transform3<S>& tf1, const Transform3<S>& tf2,
                            const CollisionRequest<S>& request_,
                            CollisionResult<S>& result_) const;

  /// @brief collision between shape and octree
  template <typename Shape>
  void ShapeOcTreeIntersect(const Shape& s, const OcTree<S>* tree,
                            const Transform3<S>& tf1, const Transform3<S>& tf2,
                            const CollisionRequest<S>& request_,
                            CollisionResult<S>& result_) const;

  /// @brief distance between octree and shape
  template <typename Shape>
  void OcTreeShapeDistance(const OcTree<S>* tree, const Shape& s,
                           const Transform3<S>& tf1, const Transform3<S>& tf2,
                           const DistanceRequest<S>& request_,
                           DistanceResult<S>& result_) const;

  /// @brief distance between shape and octree
  template <typename Shape>
  void ShapeOcTreeDistance(const Shape& s, const OcTree<S>* tree,
                           const Transform3<S>& tf1, const Transform3<S>& tf2,
                           const DistanceRequest<S>& request_,
                           DistanceResult<S>& result_) const;

private:

  template <typename Shape>
  bool OcTreeShapeDistanceRecurse(const OcTree<S>* tree1, const typename OcTree<S>::OcTreeNode* root1, const AABB<S>& bv1,
                                  const Shape& s, const AABB<S>& aabb2,
                                  const Transform3<S>& tf1, const Transform3<S>& tf2) const;

  template <typename Shape>
  bool OcTreeShapeIntersectRecurse(const OcTree<S>* tree1, const typename OcTree<S>::OcTreeNode* root1, const AABB<S>& bv1,
                                   const Shape& s, const OBB<S>& obb2,
                                   const Transform3<S>& tf1, const Transform3<S>& tf2) const;

  template <typename BV>
  bool OcTreeMeshDistanceRecurse(const OcTree<S>* tree1, const typename OcTree<S>::OcTreeNode* root1, const AABB<S>& bv1,
                                 const BVHModel<BV>* tree2, int root2,
                                 const Transform3<S>& tf1, const Transform3<S>& tf2) const;


  template <typename BV>
  bool OcTreeMeshIntersectRecurse(const OcTree<S>* tree1, const typename OcTree<S>::OcTreeNode* root1, const AABB<S>& bv1,
                                  const BVHModel<BV>* tree2, int root2,
                                  const Transform3<S>& tf1, const Transform3<S>& tf2) const;

  bool OcTreeDistanceRecurse(const OcTree<S>* tree1, const typename OcTree<S>::OcTreeNode* root1, const AABB<S>& bv1,
                             const OcTree<S>* tree2, const typename OcTree<S>::OcTreeNode* root2, const AABB<S>& bv2,
                             const Transform3<S>& tf1, const Transform3<S>& tf2) const;


  bool OcTreeIntersectRecurse(const OcTree<S>* tree1, const typename OcTree<S>::OcTreeNode* root1, const AABB<S>& bv1,
                              const OcTree<S>* tree2, const typename OcTree<S>::OcTreeNode* root2, const AABB<S>& bv2,
                              const Transform3<S>& tf1, const Transform3<S>& tf2) const;
};

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/traversal/octree/octree_solver-inl.h"

#endif

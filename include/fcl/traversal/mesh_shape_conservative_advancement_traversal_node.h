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

#ifndef FCL_TRAVERSAL_MESHSHAPECONSERVATIVEADVANCEMENTTRAVERSALNODE_H
#define FCL_TRAVERSAL_MESHSHAPECONSERVATIVEADVANCEMENTTRAVERSALNODE_H

#include "fcl/traversal/conservative_advancement_stack_data.h"
#include "fcl/traversal/mesh_shape_distance_traversal_node.h"

namespace fcl
{

/// @brief Traversal node for conservative advancement computation between BVH and shape
template<typename BV, typename S, typename NarrowPhaseSolver>
class MeshShapeConservativeAdvancementTraversalNode : public MeshShapeDistanceTraversalNode<BV, S, NarrowPhaseSolver>
{
public:
  MeshShapeConservativeAdvancementTraversalNode(FCL_REAL w_ = 1);

  /// @brief BV culling test in one BVTT node
  FCL_REAL BVTesting(int b1, int b2) const;

  /// @brief Conservative advancement testing between leaves (one triangle and one shape)
  void leafTesting(int b1, int b2) const;

  /// @brief Whether the traversal process can stop early
  bool canStop(FCL_REAL c) const;

  mutable FCL_REAL min_distance;

  mutable Vector3d closest_p1, closest_p2;

  mutable int last_tri_id;
  
  /// @brief CA controlling variable: early stop for the early iterations of CA
  FCL_REAL w;

  /// @brief The time from beginning point
  FCL_REAL toc;
  FCL_REAL t_err;

  /// @brief The delta_t each step
  mutable FCL_REAL delta_t;

  /// @brief Motions for the two objects in query
  const MotionBase* motion1;
  const MotionBase* motion2;

  mutable std::vector<ConservativeAdvancementStackData> stack;
};

template <typename BV, typename S, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeConservativeAdvancementTraversalNode<BV, S, NarrowPhaseSolver>& node,
    BVHModel<BV>& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const S& model2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    FCL_REAL w = 1,
    bool use_refit = false,
    bool refit_bottomup = false);

namespace details
{
template<typename BV, typename S, typename NarrowPhaseSolver>
void meshShapeConservativeAdvancementOrientedNodeLeafTesting(int b1, int /* b2 */,
                                                             const BVHModel<BV>* model1, const S& model2,
                                                             const BV& model2_bv,
                                                             Vector3d* vertices, Triangle* tri_indices,
                                                             const Transform3<Scalar>& tf1,
                                                             const Transform3<Scalar>& tf2,
                                                             const MotionBase* motion1, const MotionBase* motion2,
                                                             const NarrowPhaseSolver* nsolver,
                                                             bool enable_statistics,
                                                             FCL_REAL& min_distance,
                                                             Vector3d& p1, Vector3d& p2,
                                                             int& last_tri_id,
                                                             FCL_REAL& delta_t,
                                                             int& num_leaf_tests)
{
  if(enable_statistics) num_leaf_tests++;

  const BVNode<BV>& node = model1->getBV(b1);
  int primitive_id = node.primitiveId();

  const Triangle& tri_id = tri_indices[primitive_id];
  const Vector3d& t1 = vertices[tri_id[0]];
  const Vector3d& t2 = vertices[tri_id[1]];
  const Vector3d& t3 = vertices[tri_id[2]];
    
  FCL_REAL distance;
  Vector3d P1 = Vector3d::Zero();
  Vector3d P2 = Vector3d::Zero();
  nsolver->shapeTriangleDistance(model2, tf2, t1, t2, t3, tf1, &distance, &P2, &P1);

  if(distance < min_distance)
  {
    min_distance = distance;
    
    p1 = P1;
    p2 = P2;

    last_tri_id = primitive_id;
  }

  // n is in global frame
  Vector3d n = P2 - P1; n.normalize();

  TriangleMotionBoundVisitor mb_visitor1(t1, t2, t3, n);
  TBVMotionBoundVisitor<BV> mb_visitor2(model2_bv, -n);
  FCL_REAL bound1 = motion1->computeMotionBound(mb_visitor1);
  FCL_REAL bound2 = motion2->computeMotionBound(mb_visitor2);

  FCL_REAL bound = bound1 + bound2;
  
  FCL_REAL cur_delta_t;
  if(bound <= distance) cur_delta_t = 1;
  else cur_delta_t = distance / bound;

  if(cur_delta_t < delta_t)
    delta_t = cur_delta_t;  
}


template<typename BV, typename S>
bool meshShapeConservativeAdvancementOrientedNodeCanStop(FCL_REAL c,
                                                         FCL_REAL min_distance,
                                                         FCL_REAL abs_err, FCL_REAL rel_err, FCL_REAL w,
                                                         const BVHModel<BV>* model1, const S& model2,
                                                         const BV& model2_bv,
                                                         const MotionBase* motion1, const MotionBase* motion2,
                                                         std::vector<ConservativeAdvancementStackData>& stack,
                                                         FCL_REAL& delta_t)
{
  if((c >= w * (min_distance - abs_err)) && (c * (1 + rel_err) >= w * min_distance))
  {
    const ConservativeAdvancementStackData& data = stack.back();
    Vector3d n = data.P2 - data.P1; n.normalize();
    int c1 = data.c1;

    TBVMotionBoundVisitor<BV> mb_visitor1(model1->getBV(c1).bv, n);
    TBVMotionBoundVisitor<BV> mb_visitor2(model2_bv, -n);

    FCL_REAL bound1 = motion1->computeMotionBound(mb_visitor1);
    FCL_REAL bound2 = motion2->computeMotionBound(mb_visitor2);

    FCL_REAL bound = bound1 + bound2;

    FCL_REAL cur_delta_t;
    if(bound <= c) cur_delta_t = 1;
    else cur_delta_t = c / bound;

    if(cur_delta_t < delta_t)
      delta_t = cur_delta_t;

    stack.pop_back();

    return true;
  }
  else
  {
    stack.pop_back();
    return false;
  }
}


}

template<typename S, typename NarrowPhaseSolver>
class MeshShapeConservativeAdvancementTraversalNodeRSS : public MeshShapeConservativeAdvancementTraversalNode<RSSd, S, NarrowPhaseSolver>
{
public:
  MeshShapeConservativeAdvancementTraversalNodeRSS(FCL_REAL w_ = 1) : MeshShapeConservativeAdvancementTraversalNode<RSSd, S, NarrowPhaseSolver>(w_)
  {
  }

  FCL_REAL BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    Vector3d P1, P2;
    FCL_REAL d = distance(this->tf1.linear(), this->tf1.translation(), this->model1->getBV(b1).bv, this->model2_bv, &P1, &P2);

    this->stack.push_back(ConservativeAdvancementStackData(P1, P2, b1, b2, d));

    return d;
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeConservativeAdvancementOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2),
                                                                     this->model2_bv,
                                                                     this->vertices, this->tri_indices,
                                                                     this->tf1, this->tf2,
                                                                     this->motion1, this->motion2,
                                                                     this->nsolver,
                                                                     this->enable_statistics,
                                                                     this->min_distance,
                                                                     this->closest_p1, this->closest_p2,
                                                                     this->last_tri_id,
                                                                     this->delta_t,
                                                                     this->num_leaf_tests);
  }

  bool canStop(FCL_REAL c) const
  {
    return details::meshShapeConservativeAdvancementOrientedNodeCanStop(c, this->min_distance,
                                                                        this->abs_err, this->rel_err, this->w,
                                                                        this->model1, *(this->model2), this->model2_bv,
                                                                        this->motion1, this->motion2,
                                                                        this->stack, this->delta_t);
  }
};

template <typename S, typename NarrowPhaseSolver>
bool initialize(MeshShapeConservativeAdvancementTraversalNodeRSS<S, NarrowPhaseSolver>& node,
                const BVHModel<RSS<Scalar>>& model1, const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
                const S& model2, const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
                const NarrowPhaseSolver* nsolver,
                FCL_REAL w = 1);

template<typename S, typename NarrowPhaseSolver>
class MeshShapeConservativeAdvancementTraversalNodeOBBRSS : public MeshShapeConservativeAdvancementTraversalNode<OBBRSSd, S, NarrowPhaseSolver>
{
public:
  MeshShapeConservativeAdvancementTraversalNodeOBBRSS(FCL_REAL w_ = 1) : MeshShapeConservativeAdvancementTraversalNode<OBBRSSd, S, NarrowPhaseSolver>(w_)
  {
  }

  FCL_REAL BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    Vector3d P1, P2;
    FCL_REAL d = distance(this->tf1.linear(), this->tf1.translation(), this->model1->getBV(b1).bv, this->model2_bv, &P1, &P2);

    this->stack.push_back(ConservativeAdvancementStackData(P1, P2, b1, b2, d));

    return d;
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeConservativeAdvancementOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2),
                                                                     this->model2_bv,
                                                                     this->vertices, this->tri_indices,
                                                                     this->tf1, this->tf2,
                                                                     this->motion1, this->motion2,
                                                                     this->nsolver,
                                                                     this->enable_statistics,
                                                                     this->min_distance,
                                                                     this->closest_p1, this->closest_p2,
                                                                     this->last_tri_id,
                                                                     this->delta_t,
                                                                     this->num_leaf_tests);
  }

  bool canStop(FCL_REAL c) const
  {
    return details::meshShapeConservativeAdvancementOrientedNodeCanStop(c, this->min_distance,
                                                                        this->abs_err, this->rel_err, this->w,
                                                                        this->model1, *(this->model2), this->model2_bv,
                                                                        this->motion1, this->motion2,
                                                                        this->stack, this->delta_t);
  }
};

template <typename S, typename NarrowPhaseSolver>
bool initialize(MeshShapeConservativeAdvancementTraversalNodeOBBRSS<S, NarrowPhaseSolver>& node,
                const BVHModel<OBBRSS<Scalar>>& model1, const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
                const S& model2, const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
                const NarrowPhaseSolver* nsolver,
                FCL_REAL w = 1);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
MeshShapeConservativeAdvancementTraversalNode::MeshShapeConservativeAdvancementTraversalNode(FCL_REAL w_) : MeshShapeDistanceTraversalNode<BV, S, NarrowPhaseSolver>()
{
  delta_t = 1;
  toc = 0;
  t_err = (FCL_REAL)0.0001;

  w = w_;

  motion1 = NULL;
  motion2 = NULL;
}

FCL_REAL MeshShapeConservativeAdvancementTraversalNode::BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;
  Vector3d P1, P2;
  FCL_REAL d = this->model2_bv.distance(this->model1->getBV(b1).bv, &P2, &P1);

  stack.push_back(ConservativeAdvancementStackData(P1, P2, b1, b2, d));

  return d;
}

void MeshShapeConservativeAdvancementTraversalNode::leafTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_leaf_tests++;

  const BVNode<BV>& node = this->model1->getBV(b1);

  int primitive_id = node.primitiveId();

  const Triangle& tri_id = this->tri_indices[primitive_id];

  const Vector3d& p1 = this->vertices[tri_id[0]];
  const Vector3d& p2 = this->vertices[tri_id[1]];
  const Vector3d& p3 = this->vertices[tri_id[2]];

  FCL_REAL d;
  Vector3d P1, P2;
  this->nsolver->shapeTriangleDistance(*(this->model2), this->tf2, p1, p2, p3, &d, &P2, &P1);

  if(d < this->min_distance)
  {
    this->min_distance = d;

    closest_p1 = P1;
    closest_p2 = P2;

    last_tri_id = primitive_id;
  }

  Vector3d n = this->tf2 * p2 - P1; n.normalize();
  // here n should be in global frame
  TriangleMotionBoundVisitor mb_visitor1(p1, p2, p3, n);
  TBVMotionBoundVisitor<BV> mb_visitor2(this->model2_bv, -n);
  FCL_REAL bound1 = motion1->computeMotionBound(mb_visitor1);
  FCL_REAL bound2 = motion2->computeMotionBound(mb_visitor2);

  FCL_REAL bound = bound1 + bound2;

  FCL_REAL cur_delta_t;
  if(bound <= d) cur_delta_t = 1;
  else cur_delta_t = d / bound;

  if(cur_delta_t < delta_t)
    delta_t = cur_delta_t;
}

bool MeshShapeConservativeAdvancementTraversalNode::canStop(FCL_REAL c) const
{
  if((c >= w * (this->min_distance - this->abs_err)) && (c * (1 + this->rel_err) >= w * this->min_distance))
  {
    const ConservativeAdvancementStackData& data = stack.back();

    Vector3d n = this->tf2 * data.P2 - data.P1; n.normalize();
    int c1 = data.c1;

    TBVMotionBoundVisitor<BV> mb_visitor1(this->model1->getBV(c1).bv, n);
    TBVMotionBoundVisitor<BV> mb_visitor2(this->model2_bv, -n);
    FCL_REAL bound1 = motion1->computeMotionBound(mb_visitor1);
    FCL_REAL bound2 = motion2->computeMotionBound(mb_visitor2);

    FCL_REAL bound = bound1 + bound2;

    FCL_REAL cur_delta_t;
    if(bound < c) cur_delta_t = 1;
    else cur_delta_t = c / bound;

    if(cur_delta_t < delta_t)
      delta_t = cur_delta_t;

    stack.pop_back();

    return true;
  }
  else
  {
    stack.pop_back();

    return false;
  }
}

//==============================================================================
template <typename BV, typename S, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeConservativeAdvancementTraversalNode<BV, S, NarrowPhaseSolver>& node,
    BVHModel<BV>& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const S& model2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    FCL_REAL w = 1,
    bool use_refit = false,
    bool refit_bottomup = false)
{
  std::vector<Vector3d> vertices_transformed(model1.num_vertices);
  for(int i = 0; i < model1.num_vertices; ++i)
  {
    Vector3d& p = model1.vertices[i];
    Vector3d new_v = tf1 * p;
    vertices_transformed[i] = new_v;
  }

  model1.beginReplaceModel();
  model1.replaceSubModel(vertices_transformed);
  model1.endReplaceModel(use_refit, refit_bottomup);

  node.model1 = &model1;
  node.model2 = &model2;

  node.vertices = model1.vertices;
  node.tri_indices = model1.tri_indices;

  node.tf1 = tf1;
  node.tf2 = tf2;

  node.nsolver = nsolver;
  node.w = w;

  computeBV<double, BV, S>(model2, Transform3<typename NarrowPhaseSolver::Scalar>::Identity(), node.model2_bv);

  return true;
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool initialize(MeshShapeConservativeAdvancementTraversalNodeRSS<S, NarrowPhaseSolver>& node,
                const BVHModel<RSS<Scalar>>& model1, const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
                const S& model2, const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
                const NarrowPhaseSolver* nsolver,
                FCL_REAL w = 1)
{
  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;
  node.nsolver = nsolver;

  node.w = w;

  computeBV<double, RSS<Scalar>, S>(model2, Transform3<typename NarrowPhaseSolver::Scalar>::Identity(), node.model2_bv);

  return true;
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool initialize(MeshShapeConservativeAdvancementTraversalNodeOBBRSS<S, NarrowPhaseSolver>& node,
                const BVHModel<OBBRSS<Scalar>>& model1, const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
                const S& model2, const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
                const NarrowPhaseSolver* nsolver,
                FCL_REAL w = 1)
{
  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;
  node.nsolver = nsolver;

  node.w = w;

  computeBV<double, OBBRSS<Scalar>, S>(model2, Transform3<typename NarrowPhaseSolver::Scalar>::Identity(), node.model2_bv);

  return true;
}

} // namespace fcl

#endif

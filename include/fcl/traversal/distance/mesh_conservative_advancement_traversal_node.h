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

#ifndef FCL_TRAVERSAL_MESHCONSERVATIVEADVANCEMENTTRAVERSALNODE_H
#define FCL_TRAVERSAL_MESHCONSERVATIVEADVANCEMENTTRAVERSALNODE_H

#include "fcl/traversal/distance/mesh_distance_traversal_node.h"
#include "fcl/traversal/distance/conservative_advancement_stack_data.h"
#include "fcl/intersect.h"

namespace fcl
{

/// @brief continuous collision node using conservative advancement. when using this default version, must refit the BVH in current configuration (R_t, T_t) into default configuration
template <typename BV>
class MeshConservativeAdvancementTraversalNode
    : public MeshDistanceTraversalNode<BV>
{
public:

  using Scalar = typename BV::Scalar;

  MeshConservativeAdvancementTraversalNode(Scalar w_ = 1);

  /// @brief BV culling test in one BVTT node
  Scalar BVTesting(int b1, int b2) const;

  /// @brief Conservative advancement testing between leaves (two triangles)
  void leafTesting(int b1, int b2) const;

  /// @brief Whether the traversal process can stop early
  bool canStop(Scalar c) const;

  mutable Scalar min_distance;
 
  mutable Vector3<Scalar> closest_p1, closest_p2;
  
  mutable int last_tri_id1, last_tri_id2;

  /// @brief CA controlling variable: early stop for the early iterations of CA
  Scalar w;

  /// @brief The time from beginning point
  Scalar toc;
  Scalar t_err;

  /// @brief The delta_t each step
  mutable Scalar delta_t;

  /// @brief Motions for the two objects in query
  const MotionBased* motion1;
  const MotionBased* motion2;

  mutable std::vector<ConservativeAdvancementStackData<Scalar>> stack;

  template <typename, typename>
  friend struct CanStopImpl;
};

/// @brief Initialize traversal node for conservative advancement computation
/// between two meshes, given the current transforms
template <typename BV>
bool initialize(
    MeshConservativeAdvancementTraversalNode<BV>& node,
    BVHModel<BV>& model1,
    const Transform3<typename BV::Scalar>& tf1,
    BVHModel<BV>& model2,
    const Transform3<typename BV::Scalar>& tf2,
    typename BV::Scalar w = 1,
    bool use_refit = false,
    bool refit_bottomup = false);

template <typename Scalar>
class MeshConservativeAdvancementTraversalNodeRSS
    : public MeshConservativeAdvancementTraversalNode<RSS<Scalar>>
{
public:
  MeshConservativeAdvancementTraversalNodeRSS(Scalar w_ = 1);

  Scalar BVTesting(int b1, int b2) const
  {
    if (this->enable_statistics)
      this->num_bv_tests++;

    Vector3<Scalar> P1, P2;
    Scalar d = distance(
        tf,
		this->model1->getBV(b1).bv,
		this->model2->getBV(b2).bv, &P1, &P2);

    this->stack.emplace_back(P1, P2, b1, b2, d);

    return d;
  }

  void leafTesting(int b1, int b2) const;

  bool canStop(Scalar c) const;

  Transform3<Scalar> tf;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using MeshConservativeAdvancementTraversalNodeRSSf = MeshConservativeAdvancementTraversalNodeRSS<float>;
using MeshConservativeAdvancementTraversalNodeRSSd = MeshConservativeAdvancementTraversalNodeRSS<double>;

/// @brief Initialize traversal node for conservative advancement computation
/// between two meshes, given the current transforms, specialized for RSS
template <typename Scalar>
bool initialize(
    MeshConservativeAdvancementTraversalNodeRSS<Scalar>& node,
    const BVHModel<RSS<Scalar>>& model1,
    const Transform3<Scalar>& tf1,
    const BVHModel<RSS<Scalar>>& model2,
    const Transform3<Scalar>& tf2,
    Scalar w = 1);

template <typename Scalar>
class MeshConservativeAdvancementTraversalNodeOBBRSS
    : public MeshConservativeAdvancementTraversalNode<OBBRSS<Scalar>>
{
public:
  MeshConservativeAdvancementTraversalNodeOBBRSS(Scalar w_ = 1);

  Scalar BVTesting(int b1, int b2) const
  {
    if (this->enable_statistics)
      this->num_bv_tests++;

    Vector3<Scalar> P1, P2;
    Scalar d = distance(
        tf,
        this->model1->getBV(b1).bv,
        this->model2->getBV(b2).bv, &P1, &P2);

    this->stack.emplace_back(P1, P2, b1, b2, d);

    return d;
  }

  void leafTesting(int b1, int b2) const;

  bool canStop(Scalar c) const;

  Transform3<Scalar> tf;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using MeshConservativeAdvancementTraversalNodeOBBRSSf = MeshConservativeAdvancementTraversalNodeOBBRSS<float>;
using MeshConservativeAdvancementTraversalNodeOBBRSSd = MeshConservativeAdvancementTraversalNodeOBBRSS<double>;

template <typename Scalar>
bool initialize(
    MeshConservativeAdvancementTraversalNodeOBBRSS<Scalar>& node,
    const BVHModel<OBBRSS<Scalar>>& model1,
    const Transform3<Scalar>& tf1,
    const BVHModel<OBBRSS<Scalar>>& model2,
    const Transform3<Scalar>& tf2,
    Scalar w = 1);

namespace details
{

template <typename Scalar, typename BV>
const Vector3<Scalar> getBVAxis(const BV& bv, int i);

template <typename BV>
bool meshConservativeAdvancementTraversalNodeCanStop(
    typename BV::Scalar c,
    typename BV::Scalar min_distance,
    typename BV::Scalar abs_err,
    typename BV::Scalar rel_err,
    typename BV::Scalar w,
    const BVHModel<BV>* model1,
    const BVHModel<BV>* model2,
    const MotionBased* motion1,
    const MotionBased* motion2,
    std::vector<ConservativeAdvancementStackData<typename BV::Scalar>>& stack,
    typename BV::Scalar& delta_t);

template <typename BV>
bool meshConservativeAdvancementOrientedNodeCanStop(
    typename BV::Scalar c,
    typename BV::Scalar min_distance,
    typename BV::Scalar abs_err,
    typename BV::Scalar rel_err,
    typename BV::Scalar w,
    const BVHModel<BV>* model1,
    const BVHModel<BV>* model2,
    const MotionBased* motion1,
    const MotionBased* motion2,
    std::vector<ConservativeAdvancementStackData<typename BV::Scalar>>& stack,
    typename BV::Scalar& delta_t);

template <typename BV>
void meshConservativeAdvancementOrientedNodeLeafTesting(
    int b1,
    int b2,
    const BVHModel<BV>* model1,
    const BVHModel<BV>* model2,
    const Triangle* tri_indices1,
    const Triangle* tri_indices2,
    const Vector3<typename BV::Scalar>* vertices1,
    const Vector3<typename BV::Scalar>* vertices2,
    const Matrix3<typename BV::Scalar>& R,
    const Vector3<typename BV::Scalar>& T,
    const MotionBased* motion1,
    const MotionBased* motion2,
    bool enable_statistics,
    typename BV::Scalar& min_distance,
    Vector3<typename BV::Scalar>& p1,
    Vector3<typename BV::Scalar>& p2,
    int& last_tri_id1,
    int& last_tri_id2,
    typename BV::Scalar& delta_t,
    int& num_leaf_tests);

} // namespace details

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename BV>
MeshConservativeAdvancementTraversalNode<BV>::
MeshConservativeAdvancementTraversalNode(typename BV::Scalar w_)
  : MeshDistanceTraversalNode<BV>()
{
  delta_t = 1;
  toc = 0;
  t_err = (Scalar)0.00001;

  w = w_;

  motion1 = NULL;
  motion2 = NULL;
}

//==============================================================================
template <typename BV>
typename BV::Scalar
MeshConservativeAdvancementTraversalNode<BV>::BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;
  Vector3<Scalar> P1, P2;
  Scalar d = this->model1->getBV(b1).distance(this->model2->getBV(b2), &P1, &P2);

  stack.emplace_back(P1, P2, b1, b2, d);

  return d;
}

//==============================================================================
template <typename BV>
void MeshConservativeAdvancementTraversalNode<BV>::leafTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_leaf_tests++;

  const BVNode<BV>& node1 = this->model1->getBV(b1);
  const BVNode<BV>& node2 = this->model2->getBV(b2);

  int primitive_id1 = node1.primitiveId();
  int primitive_id2 = node2.primitiveId();

  const Triangle& tri_id1 = this->tri_indices1[primitive_id1];
  const Triangle& tri_id2 = this->tri_indices2[primitive_id2];

  const Vector3<Scalar>& p1 = this->vertices1[tri_id1[0]];
  const Vector3<Scalar>& p2 = this->vertices1[tri_id1[1]];
  const Vector3<Scalar>& p3 = this->vertices1[tri_id1[2]];

  const Vector3<Scalar>& q1 = this->vertices2[tri_id2[0]];
  const Vector3<Scalar>& q2 = this->vertices2[tri_id2[1]];
  const Vector3<Scalar>& q3 = this->vertices2[tri_id2[2]];

  // nearest point pair
  Vector3<Scalar> P1, P2;

  Scalar d = TriangleDistance<Scalar>::triDistance(p1, p2, p3, q1, q2, q3,
                                           P1, P2);

  if(d < this->min_distance)
  {
    this->min_distance = d;

    closest_p1 = P1;
    closest_p2 = P2;

    last_tri_id1 = primitive_id1;
    last_tri_id2 = primitive_id2;
  }

  Vector3<Scalar> n = P2 - P1;
  n.normalize();
  // here n is already in global frame as we assume the body is in original configuration (I, 0) for general BVH
  TriangleMotionBoundVisitor<Scalar> mb_visitor1(p1, p2, p3, n), mb_visitor2(q1, q2, q3, n);
  Scalar bound1 = motion1->computeMotionBound(mb_visitor1);
  Scalar bound2 = motion2->computeMotionBound(mb_visitor2);

  Scalar bound = bound1 + bound2;

  Scalar cur_delta_t;
  if(bound <= d) cur_delta_t = 1;
  else cur_delta_t = d / bound;

  if(cur_delta_t < delta_t)
    delta_t = cur_delta_t;
}

//==============================================================================
template <typename Scalar, typename BV>
struct CanStopImpl
{
  static bool run(
      const MeshConservativeAdvancementTraversalNode<BV>& node, Scalar c)
  {
    if((c >= node.w * (node.min_distance - node.abs_err))
       && (c * (1 + node.rel_err) >= node.w * node.min_distance))
    {
      const ConservativeAdvancementStackData<Scalar>& data = node.stack.back();
      Scalar d = data.d;
      Vector3<Scalar> n;
      int c1, c2;

      if(d > c)
      {
        const ConservativeAdvancementStackData<Scalar>& data2 = node.stack[node.stack.size() - 2];
        d = data2.d;
        n = data2.P2 - data2.P1; n.normalize();
        c1 = data2.c1;
        c2 = data2.c2;
        node.stack[node.stack.size() - 2] = node.stack[node.stack.size() - 1];
      }
      else
      {
        n = data.P2 - data.P1; n.normalize();
        c1 = data.c1;
        c2 = data.c2;
      }

      assert(c == d);

      TBVMotionBoundVisitor<BV> mb_visitor1(node.model1->getBV(c1).bv, n), mb_visitor2(node.model2->getBV(c2).bv, n);
      Scalar bound1 = node.motion1->computeMotionBound(mb_visitor1);
      Scalar bound2 = node.motion2->computeMotionBound(mb_visitor2);

      Scalar bound = bound1 + bound2;

      Scalar cur_delta_t;
      if(bound <= c) cur_delta_t = 1;
      else cur_delta_t = c / bound;

      if(cur_delta_t < node.delta_t)
        node.delta_t = cur_delta_t;

      node.stack.pop_back();

      return true;
    }
    else
    {
      const ConservativeAdvancementStackData<Scalar>& data = node.stack.back();
      Scalar d = data.d;

      if(d > c)
        node.stack[node.stack.size() - 2] = node.stack[node.stack.size() - 1];

      node.stack.pop_back();

      return false;
    }
  }
};

//==============================================================================
template <typename BV>
bool MeshConservativeAdvancementTraversalNode<BV>::canStop(
    typename BV::Scalar c) const
{
  return CanStopImpl<typename BV::Scalar, BV>::run(*this, c);
}

//==============================================================================
template <typename Scalar>
struct CanStopImpl<Scalar, OBB<Scalar>>
{
  static bool run(
      const MeshConservativeAdvancementTraversalNode<OBB<Scalar>>& node,
      Scalar c)
  {
    return details::meshConservativeAdvancementTraversalNodeCanStop(
          c,
          node.min_distance,
          node.abs_err,
          node.rel_err,
          node.w,
          node.model1,
          node.model2,
          node.motion1,
          node.motion2,
          node.stack,
          node.delta_t);
  }
};

//==============================================================================
template <typename Scalar>
struct CanStopImpl<Scalar, RSS<Scalar>>
{
  static bool run(
      const MeshConservativeAdvancementTraversalNode<RSS<Scalar>>& node,
      Scalar c)
  {
    return details::meshConservativeAdvancementTraversalNodeCanStop(
          c,
          node.min_distance,
          node.abs_err,
          node.rel_err,
          node.w,
          node.model1,
          node.model2,
          node.motion1,
          node.motion2,
          node.stack,
          node.delta_t);
  }
};

//==============================================================================
template <typename Scalar>
struct CanStopImpl<Scalar, OBBRSS<Scalar>>
{
  static bool run(
      const MeshConservativeAdvancementTraversalNode<OBBRSS<Scalar>>& node,
      Scalar c)
  {
    return details::meshConservativeAdvancementTraversalNodeCanStop(
          c,
          node.min_distance,
          node.abs_err,
          node.rel_err,
          node.w,
          node.model1,
          node.model2,
          node.motion1,
          node.motion2,
          node.stack,
          node.delta_t);
  }
};

//==============================================================================
template <typename BV>
bool initialize(
    MeshConservativeAdvancementTraversalNode<BV>& node,
    BVHModel<BV>& model1,
    const Transform3<typename BV::Scalar>& tf1,
    BVHModel<BV>& model2,
    const Transform3<typename BV::Scalar>& tf2,
    typename BV::Scalar w,
    bool use_refit,
    bool refit_bottomup)
{
  using Scalar = typename BV::Scalar;

  std::vector<Vector3<Scalar>> vertices_transformed1(model1.num_vertices);
  for(int i = 0; i < model1.num_vertices; ++i)
  {
    Vector3<Scalar>& p = model1.vertices[i];
    Vector3<Scalar> new_v = tf1 * p;
    vertices_transformed1[i] = new_v;
  }

  std::vector<Vector3<Scalar>> vertices_transformed2(model2.num_vertices);
  for(int i = 0; i < model2.num_vertices; ++i)
  {
    Vector3<Scalar>& p = model2.vertices[i];
    Vector3<Scalar> new_v = tf2 * p;
    vertices_transformed2[i] = new_v;
  }

  model1.beginReplaceModel();
  model1.replaceSubModel(vertices_transformed1);
  model1.endReplaceModel(use_refit, refit_bottomup);

  model2.beginReplaceModel();
  model2.replaceSubModel(vertices_transformed2);
  model2.endReplaceModel(use_refit, refit_bottomup);

  node.model1 = &model1;
  node.model2 = &model2;

  node.vertices1 = model1.vertices;
  node.vertices2 = model2.vertices;

  node.tri_indices1 = model1.tri_indices;
  node.tri_indices2 = model2.tri_indices;

  node.w = w;

  return true;
}

//==============================================================================
template <typename Scalar>
MeshConservativeAdvancementTraversalNodeRSS<Scalar>::MeshConservativeAdvancementTraversalNodeRSS(Scalar w_)
  : MeshConservativeAdvancementTraversalNode<RSS<Scalar>>(w_)
{
  tf.linear().setIdentity();
}

//==============================================================================
template <typename Scalar>
void MeshConservativeAdvancementTraversalNodeRSS<Scalar>::leafTesting(int b1, int b2) const
{
  details::meshConservativeAdvancementOrientedNodeLeafTesting(
        b1,
        b2,
        this->model1,
        this->model2,
        this->tri_indices1,
        this->tri_indices2,
        this->vertices1,
        this->vertices2,
        tf.linear(),
        tf.translation(),
        this->motion1,
        this->motion2,
        this->enable_statistics,
        this->min_distance,
        this->closest_p1,
        this->closest_p2,
        this->last_tri_id1,
        this->last_tri_id2,
        this->delta_t,
        this->num_leaf_tests);
}

//==============================================================================
template <typename Scalar>
bool MeshConservativeAdvancementTraversalNodeRSS<Scalar>::canStop(Scalar c) const
{
  return details::meshConservativeAdvancementOrientedNodeCanStop(
        c,
        this->min_distance,
        this->abs_err,
        this->rel_err,
        this->w,
        this->model1,
        this->model2,
        this->motion1,
        this->motion2,
        this->stack,
        this->delta_t);
}

//==============================================================================
template <typename Scalar>
MeshConservativeAdvancementTraversalNodeOBBRSS<Scalar>::
MeshConservativeAdvancementTraversalNodeOBBRSS(Scalar w_)
  : MeshConservativeAdvancementTraversalNode<OBBRSS<Scalar>>(w_)
{
  tf.linear().setIdentity();
}

//==============================================================================
template <typename Scalar>
void MeshConservativeAdvancementTraversalNodeOBBRSS<Scalar>::
leafTesting(int b1, int b2) const
{
  details::meshConservativeAdvancementOrientedNodeLeafTesting(
        b1,
        b2,
        this->model1,
        this->model2,
        this->tri_indices1,
        this->tri_indices2,
        this->vertices1,
        this->vertices2,
        this->tf.linear(),
        this->tf.translation(),
        this->motion1,
        this->motion2,
        this->enable_statistics,
        this->min_distance,
        this->closest_p1,
        this->closest_p2,
        this->last_tri_id1,
        this->last_tri_id2,
        this->delta_t,
        this->num_leaf_tests);
}

//==============================================================================
template <typename Scalar>
bool MeshConservativeAdvancementTraversalNodeOBBRSS<Scalar>::canStop(Scalar c) const
{
  return details::meshConservativeAdvancementOrientedNodeCanStop(
        c,
        this->min_distance,
        this->abs_err,
        this->rel_err,
        this->w,
        this->model1,
        this->model2,
        this->motion1,
        this->motion2,
        this->stack,
        this->delta_t);
}

/// @brief for OBB and RSS, there is local coordinate of BV, so normal need to be transformed
namespace details
{

//==============================================================================
template <typename Scalar, typename BV>
struct GetBVAxisImpl
{
  const Vector3<Scalar> operator()(const BV& bv, int i)
  {
    return bv.axis.col(i);
  }
};

//==============================================================================
template <typename BV>
const Vector3<typename BV::Scalar> getBVAxis(const BV& bv, int i)
{
  GetBVAxisImpl<typename BV::Scalar, BV> getBVAxisImpl;
  return getBVAxisImpl(bv, i);
}

//==============================================================================
template <typename Scalar>
struct GetBVAxisImpl<Scalar, OBBRSS<Scalar>>
{
  const Vector3<Scalar> operator()(const OBBRSS<Scalar>& bv, int i)
  {
    return bv.obb.axis.col(i);
  }
};

//==============================================================================
template <typename BV>
bool meshConservativeAdvancementTraversalNodeCanStop(
    typename BV::Scalar c,
    typename BV::Scalar min_distance,
    typename BV::Scalar abs_err,
    typename BV::Scalar rel_err,
    typename BV::Scalar w,
    const BVHModel<BV>* model1,
    const BVHModel<BV>* model2,
    const MotionBased* motion1,
    const MotionBased* motion2,
    std::vector<ConservativeAdvancementStackData<typename BV::Scalar>>& stack,
    typename BV::Scalar& delta_t)
{
  using Scalar = typename BV::Scalar;

  if((c >= w * (min_distance - abs_err)) && (c * (1 + rel_err) >= w * min_distance))
  {
    const ConservativeAdvancementStackData<Scalar>& data = stack.back();
    Scalar d = data.d;
    Vector3<Scalar> n;
    int c1, c2;

    if(d > c)
    {
      const ConservativeAdvancementStackData<Scalar>& data2 = stack[stack.size() - 2];
      d = data2.d;
      n = data2.P2 - data2.P1; n.normalize();
      c1 = data2.c1;
      c2 = data2.c2;
      stack[stack.size() - 2] = stack[stack.size() - 1];
    }
    else
    {
      n = data.P2 - data.P1; n.normalize();
      c1 = data.c1;
      c2 = data.c2;
    }

    assert(c == d);

    Vector3<Scalar> n_transformed =
        getBVAxis(model1->getBV(c1).bv, 0) * n[0] +
        getBVAxis(model1->getBV(c1).bv, 1) * n[1] +
        getBVAxis(model1->getBV(c1).bv, 2) * n[2];

    TBVMotionBoundVisitor<BV> mb_visitor1(model1->getBV(c1).bv, n_transformed), mb_visitor2(model2->getBV(c2).bv, n_transformed);
    Scalar bound1 = motion1->computeMotionBound(mb_visitor1);
    Scalar bound2 = motion2->computeMotionBound(mb_visitor2);

    Scalar bound = bound1 + bound2;

    Scalar cur_delta_t;
    if(bound <= c) cur_delta_t = 1;
    else cur_delta_t = c / bound;

    if(cur_delta_t < delta_t)
      delta_t = cur_delta_t;

    stack.pop_back();

    return true;
  }
  else
  {
    const ConservativeAdvancementStackData<Scalar>& data = stack.back();
    Scalar d = data.d;

    if(d > c)
      stack[stack.size() - 2] = stack[stack.size() - 1];

    stack.pop_back();

    return false;
  }
}

//==============================================================================
template <typename BV>
bool meshConservativeAdvancementOrientedNodeCanStop(
    typename BV::Scalar c,
    typename BV::Scalar min_distance,
    typename BV::Scalar abs_err,
    typename BV::Scalar rel_err,
    typename BV::Scalar w,
    const BVHModel<BV>* model1,
    const BVHModel<BV>* model2,
    const MotionBased* motion1,
    const MotionBased* motion2,
    std::vector<ConservativeAdvancementStackData<typename BV::Scalar>>& stack,
    typename BV::Scalar& delta_t)
{
  using Scalar = typename BV::Scalar;

  if((c >= w * (min_distance - abs_err)) && (c * (1 + rel_err) >= w * min_distance))
  {
    const ConservativeAdvancementStackData<Scalar>& data = stack.back();
    Scalar d = data.d;
    Vector3<Scalar> n;
    int c1, c2;

    if(d > c)
    {
      const ConservativeAdvancementStackData<Scalar>& data2 = stack[stack.size() - 2];
      d = data2.d;
      n = data2.P2 - data2.P1; n.normalize();
      c1 = data2.c1;
      c2 = data2.c2;
      stack[stack.size() - 2] = stack[stack.size() - 1];
    }
    else
    {
      n = data.P2 - data.P1; n.normalize();
      c1 = data.c1;
      c2 = data.c2;
    }

    assert(c == d);

    // n is in local frame of c1, so we need to turn n into the global frame
    Vector3<Scalar> n_transformed =
      getBVAxis(model1->getBV(c1).bv, 0) * n[0] +
      getBVAxis(model1->getBV(c1).bv, 1) * n[2] +  // TODO(JS): not n[1]?
      getBVAxis(model1->getBV(c1).bv, 2) * n[2];
    Quaternion<Scalar> R0;
    motion1->getCurrentRotation(R0);
    n_transformed = R0 * n_transformed;
    n_transformed.normalize();

    TBVMotionBoundVisitor<BV> mb_visitor1(model1->getBV(c1).bv, n_transformed), mb_visitor2(model2->getBV(c2).bv, -n_transformed);
    Scalar bound1 = motion1->computeMotionBound(mb_visitor1);
    Scalar bound2 = motion2->computeMotionBound(mb_visitor2);

    Scalar bound = bound1 + bound2;

    Scalar cur_delta_t;
    if(bound <= c) cur_delta_t = 1;
    else cur_delta_t = c / bound;

    if(cur_delta_t < delta_t)
      delta_t = cur_delta_t;

    stack.pop_back();

    return true;
  }
  else
  {
    const ConservativeAdvancementStackData<Scalar>& data = stack.back();
    Scalar d = data.d;

    if(d > c)
      stack[stack.size() - 2] = stack[stack.size() - 1];

    stack.pop_back();

    return false;
  }
}

//==============================================================================
template <typename BV>
void meshConservativeAdvancementOrientedNodeLeafTesting(
    int b1,
    int b2,
    const BVHModel<BV>* model1,
    const BVHModel<BV>* model2,
    const Triangle* tri_indices1,
    const Triangle* tri_indices2,
    const Vector3<typename BV::Scalar>* vertices1,
    const Vector3<typename BV::Scalar>* vertices2,
    const Matrix3<typename BV::Scalar>& R,
    const Vector3<typename BV::Scalar>& T,
    const MotionBased* motion1,
    const MotionBased* motion2,
    bool enable_statistics,
    typename BV::Scalar& min_distance,
    Vector3<typename BV::Scalar>& p1,
    Vector3<typename BV::Scalar>& p2,
    int& last_tri_id1,
    int& last_tri_id2,
    typename BV::Scalar& delta_t,
    int& num_leaf_tests)
{
  using Scalar = typename BV::Scalar;

  if(enable_statistics) num_leaf_tests++;

  const BVNode<BV>& node1 = model1->getBV(b1);
  const BVNode<BV>& node2 = model2->getBV(b2);

  int primitive_id1 = node1.primitiveId();
  int primitive_id2 = node2.primitiveId();

  const Triangle& tri_id1 = tri_indices1[primitive_id1];
  const Triangle& tri_id2 = tri_indices2[primitive_id2];

  const Vector3<Scalar>& t11 = vertices1[tri_id1[0]];
  const Vector3<Scalar>& t12 = vertices1[tri_id1[1]];
  const Vector3<Scalar>& t13 = vertices1[tri_id1[2]];

  const Vector3<Scalar>& t21 = vertices2[tri_id2[0]];
  const Vector3<Scalar>& t22 = vertices2[tri_id2[1]];
  const Vector3<Scalar>& t23 = vertices2[tri_id2[2]];

  // nearest point pair
  Vector3<Scalar> P1, P2;

  Scalar d = TriangleDistance<Scalar>::triDistance(t11, t12, t13, t21, t22, t23,
                                           R, T,
                                           P1, P2);

  if(d < min_distance)
  {
    min_distance = d;

    p1 = P1;
    p2 = P2;

    last_tri_id1 = primitive_id1;
    last_tri_id2 = primitive_id2;
  }


  /// n is the local frame of object 1, pointing from object 1 to object2
  Vector3<Scalar> n = P2 - P1;
  /// turn n into the global frame, pointing from object 1 to object 2
  Quaternion<Scalar> R0;
  motion1->getCurrentRotation(R0);
  Vector3<Scalar> n_transformed = R0 * n;
  n_transformed.normalize(); // normalized here

  TriangleMotionBoundVisitor<Scalar> mb_visitor1(t11, t12, t13, n_transformed), mb_visitor2(t21, t22, t23, -n_transformed);
  Scalar bound1 = motion1->computeMotionBound(mb_visitor1);
  Scalar bound2 = motion2->computeMotionBound(mb_visitor2);

  Scalar bound = bound1 + bound2;

  Scalar cur_delta_t;
  if(bound <= d) cur_delta_t = 1;
  else cur_delta_t = d / bound;

  if(cur_delta_t < delta_t)
    delta_t = cur_delta_t;
}

//==============================================================================
template <typename BV, typename OrientedDistanceNode>
bool setupMeshConservativeAdvancementOrientedDistanceNode(
    OrientedDistanceNode& node,
    const BVHModel<BV>& model1, const Transform3<typename BV::Scalar>& tf1,
    const BVHModel<BV>& model2, const Transform3<typename BV::Scalar>& tf2,
    typename BV::Scalar w)
{
  if(model1.getModelType() != BVH_MODEL_TRIANGLES || model2.getModelType() != BVH_MODEL_TRIANGLES)
    return false;

  node.model1 = &model1;
  node.model2 = &model2;

  node.vertices1 = model1.vertices;
  node.vertices2 = model2.vertices;

  node.tri_indices1 = model1.tri_indices;
  node.tri_indices2 = model2.tri_indices;

  node.w = w;

  node.tf = tf1.inverse(Eigen::Isometry) * tf2;

  return true;
}

} // namespace detials

//==============================================================================
template <typename Scalar>
bool initialize(
    MeshConservativeAdvancementTraversalNodeRSS<Scalar>& node,
    const BVHModel<RSS<Scalar>>& model1,
    const Transform3<Scalar>& tf1,
    const BVHModel<RSS<Scalar>>& model2,
    const Transform3<Scalar>& tf2,
    Scalar w)
{
  return details::setupMeshConservativeAdvancementOrientedDistanceNode(
        node, model1, tf1, model2, tf2, w);
}

//==============================================================================
template <typename Scalar>
bool initialize(
    MeshConservativeAdvancementTraversalNodeOBBRSS<Scalar>& node,
    const BVHModel<OBBRSS<Scalar>>& model1,
    const Transform3<Scalar>& tf1,
    const BVHModel<OBBRSS<Scalar>>& model2,
    const Transform3<Scalar>& tf2,
    Scalar w)
{
  return details::setupMeshConservativeAdvancementOrientedDistanceNode(
        node, model1, tf1, model2, tf2, w);
}

} // namespace fcl

#endif

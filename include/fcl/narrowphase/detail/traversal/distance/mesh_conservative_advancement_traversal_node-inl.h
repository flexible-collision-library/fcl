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

#ifndef FCL_TRAVERSAL_MESHCONSERVATIVEADVANCEMENTTRAVERSALNODE_INL_H
#define FCL_TRAVERSAL_MESHCONSERVATIVEADVANCEMENTTRAVERSALNODE_INL_H

#include "fcl/narrowphase/detail/traversal/distance/mesh_conservative_advancement_traversal_node.h"

#include "fcl/math/bv/RSS.h"
#include "fcl/math/motion/triangle_motion_bound_visitor.h"

namespace fcl
{

namespace detail
{

//==============================================================================
extern template
class FCL_EXPORT MeshConservativeAdvancementTraversalNodeRSS<double>;

//==============================================================================
extern template
bool initialize(
    MeshConservativeAdvancementTraversalNodeRSS<double>& node,
    const BVHModel<RSS<double>>& model1,
    const Transform3<double>& tf1,
    const BVHModel<RSS<double>>& model2,
    const Transform3<double>& tf2,
    double w);

//==============================================================================
extern template
class FCL_EXPORT MeshConservativeAdvancementTraversalNodeOBBRSS<double>;

//==============================================================================
extern template
bool initialize(
    MeshConservativeAdvancementTraversalNodeOBBRSS<double>& node,
    const BVHModel<OBBRSS<double>>& model1,
    const Transform3<double>& tf1,
    const BVHModel<OBBRSS<double>>& model2,
    const Transform3<double>& tf2,
    double w);

//==============================================================================
template <typename BV>
MeshConservativeAdvancementTraversalNode<BV>::
MeshConservativeAdvancementTraversalNode(typename BV::S w_)
  : MeshDistanceTraversalNode<BV>()
{
  delta_t = 1;
  toc = 0;
  t_err = (S)0.00001;

  w = w_;

  motion1 = nullptr;
  motion2 = nullptr;
}

//==============================================================================
template <typename BV>
typename BV::S
MeshConservativeAdvancementTraversalNode<BV>::BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;
  Vector3<S> P1, P2;
  S d = this->model1->getBV(b1).distance(this->model2->getBV(b2), &P1, &P2);

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

  const Vector3<S>& p1 = this->vertices1[tri_id1[0]];
  const Vector3<S>& p2 = this->vertices1[tri_id1[1]];
  const Vector3<S>& p3 = this->vertices1[tri_id1[2]];

  const Vector3<S>& q1 = this->vertices2[tri_id2[0]];
  const Vector3<S>& q2 = this->vertices2[tri_id2[1]];
  const Vector3<S>& q3 = this->vertices2[tri_id2[2]];

  // nearest point pair
  Vector3<S> P1, P2;

  S d = TriangleDistance<S>::triDistance(p1, p2, p3, q1, q2, q3,
                                           P1, P2);

  if(d < this->min_distance)
  {
    this->min_distance = d;

    closest_p1 = P1;
    closest_p2 = P2;

    last_tri_id1 = primitive_id1;
    last_tri_id2 = primitive_id2;
  }

  Vector3<S> n = P2 - P1;
  n.normalize();
  // here n is already in global frame as we assume the body is in original configuration (I, 0) for general BVH
  TriangleMotionBoundVisitor<S> mb_visitor1(p1, p2, p3, n), mb_visitor2(q1, q2, q3, n);
  S bound1 = motion1->computeMotionBound(mb_visitor1);
  S bound2 = motion2->computeMotionBound(mb_visitor2);

  S bound = bound1 + bound2;

  S cur_delta_t;
  if(bound <= d) cur_delta_t = 1;
  else cur_delta_t = d / bound;

  if(cur_delta_t < delta_t)
    delta_t = cur_delta_t;
}

//==============================================================================
template <typename S, typename BV>
struct CanStopImpl
{
  static bool run(
      const MeshConservativeAdvancementTraversalNode<BV>& node, S c)
  {
    if((c >= node.w * (node.min_distance - node.abs_err))
       && (c * (1 + node.rel_err) >= node.w * node.min_distance))
    {
      const ConservativeAdvancementStackData<S>& data = node.stack.back();
      S d = data.d;
      Vector3<S> n;
      int c1, c2;

      if(d > c)
      {
        const ConservativeAdvancementStackData<S>& data2 = node.stack[node.stack.size() - 2];
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
      S bound1 = node.motion1->computeMotionBound(mb_visitor1);
      S bound2 = node.motion2->computeMotionBound(mb_visitor2);

      S bound = bound1 + bound2;

      S cur_delta_t;
      if(bound <= c) cur_delta_t = 1;
      else cur_delta_t = c / bound;

      if(cur_delta_t < node.delta_t)
        node.delta_t = cur_delta_t;

      node.stack.pop_back();

      return true;
    }
    else
    {
      const ConservativeAdvancementStackData<S>& data = node.stack.back();
      S d = data.d;

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
    typename BV::S c) const
{
  return CanStopImpl<typename BV::S, BV>::run(*this, c);
}

//==============================================================================
template <typename S>
struct CanStopImpl<S, OBB<S>>
{
  static bool run(
      const MeshConservativeAdvancementTraversalNode<OBB<S>>& node,
      S c)
  {
    return detail::meshConservativeAdvancementTraversalNodeCanStop(
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
template <typename S>
struct CanStopImpl<S, RSS<S>>
{
  static bool run(
      const MeshConservativeAdvancementTraversalNode<RSS<S>>& node,
      S c)
  {
    return detail::meshConservativeAdvancementTraversalNodeCanStop(
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
template <typename S>
struct CanStopImpl<S, OBBRSS<S>>
{
  static bool run(
      const MeshConservativeAdvancementTraversalNode<OBBRSS<S>>& node,
      S c)
  {
    return detail::meshConservativeAdvancementTraversalNodeCanStop(
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
    const Transform3<typename BV::S>& tf1,
    BVHModel<BV>& model2,
    const Transform3<typename BV::S>& tf2,
    typename BV::S w,
    bool use_refit,
    bool refit_bottomup)
{
  using S = typename BV::S;

  std::vector<Vector3<S>> vertices_transformed1(model1.num_vertices);
  for(int i = 0; i < model1.num_vertices; ++i)
  {
    Vector3<S>& p = model1.vertices[i];
    Vector3<S> new_v = tf1 * p;
    vertices_transformed1[i] = new_v;
  }

  std::vector<Vector3<S>> vertices_transformed2(model2.num_vertices);
  for(int i = 0; i < model2.num_vertices; ++i)
  {
    Vector3<S>& p = model2.vertices[i];
    Vector3<S> new_v = tf2 * p;
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
template <typename S>
MeshConservativeAdvancementTraversalNodeRSS<S>::
MeshConservativeAdvancementTraversalNodeRSS(S w_)
  : MeshConservativeAdvancementTraversalNode<RSS<S>>(w_)
{
  R.setIdentity();
}

//==============================================================================
template <typename S>
void MeshConservativeAdvancementTraversalNodeRSS<S>::
leafTesting(int b1, int b2) const
{
  detail::meshConservativeAdvancementOrientedNodeLeafTesting(
        b1,
        b2,
        this->model1,
        this->model2,
        this->tri_indices1,
        this->tri_indices2,
        this->vertices1,
        this->vertices2,
        R,
        T,
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
template <typename S>
bool MeshConservativeAdvancementTraversalNodeRSS<S>::canStop(S c) const
{
  return detail::meshConservativeAdvancementOrientedNodeCanStop(
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
template <typename S>
MeshConservativeAdvancementTraversalNodeOBBRSS<S>::
MeshConservativeAdvancementTraversalNodeOBBRSS(S w_)
  : MeshConservativeAdvancementTraversalNode<OBBRSS<S>>(w_)
{
  R.setIdentity();
}

//==============================================================================
template <typename S>
void MeshConservativeAdvancementTraversalNodeOBBRSS<S>::
leafTesting(int b1, int b2) const
{
  detail::meshConservativeAdvancementOrientedNodeLeafTesting(
        b1,
        b2,
        this->model1,
        this->model2,
        this->tri_indices1,
        this->tri_indices2,
        this->vertices1,
        this->vertices2,
        this->R,
        this->T,
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
template <typename S>
bool MeshConservativeAdvancementTraversalNodeOBBRSS<S>::canStop(S c) const
{
  return detail::meshConservativeAdvancementOrientedNodeCanStop(
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

//==============================================================================
template <typename S, typename BV>
struct GetBVAxisImpl
{
  const Vector3<S> operator()(const BV& bv, int i)
  {
    return bv.axis.col(i);
  }
};

//==============================================================================
template <typename BV>
const Vector3<typename BV::S> getBVAxis(const BV& bv, int i)
{
  GetBVAxisImpl<typename BV::S, BV> getBVAxisImpl;
  return getBVAxisImpl(bv, i);
}

//==============================================================================
template <typename S>
struct GetBVAxisImpl<S, OBBRSS<S>>
{
  const Vector3<S> operator()(const OBBRSS<S>& bv, int i)
  {
    return bv.obb.axis.col(i);
  }
};

//==============================================================================
template <typename BV>
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
    typename BV::S& delta_t)
{
  using S = typename BV::S;

  if((c >= w * (min_distance - abs_err)) && (c * (1 + rel_err) >= w * min_distance))
  {
    const ConservativeAdvancementStackData<S>& data = stack.back();
    S d = data.d;
    Vector3<S> n;
    int c1, c2;

    if(d > c)
    {
      const ConservativeAdvancementStackData<S>& data2 = stack[stack.size() - 2];
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

    Vector3<S> n_transformed =
        getBVAxis(model1->getBV(c1).bv, 0) * n[0] +
        getBVAxis(model1->getBV(c1).bv, 1) * n[1] +
        getBVAxis(model1->getBV(c1).bv, 2) * n[2];

    TBVMotionBoundVisitor<BV> mb_visitor1(model1->getBV(c1).bv, n_transformed), mb_visitor2(model2->getBV(c2).bv, n_transformed);
    S bound1 = motion1->computeMotionBound(mb_visitor1);
    S bound2 = motion2->computeMotionBound(mb_visitor2);

    S bound = bound1 + bound2;

    S cur_delta_t;
    if(bound <= c) cur_delta_t = 1;
    else cur_delta_t = c / bound;

    if(cur_delta_t < delta_t)
      delta_t = cur_delta_t;

    stack.pop_back();

    return true;
  }
  else
  {
    const ConservativeAdvancementStackData<S>& data = stack.back();
    S d = data.d;

    if(d > c)
      stack[stack.size() - 2] = stack[stack.size() - 1];

    stack.pop_back();

    return false;
  }
}

//==============================================================================
template <typename BV>
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
    typename BV::S& delta_t)
{
  using S = typename BV::S;

  if((c >= w * (min_distance - abs_err)) && (c * (1 + rel_err) >= w * min_distance))
  {
    const ConservativeAdvancementStackData<S>& data = stack.back();
    S d = data.d;
    Vector3<S> n;
    int c1, c2;

    if(d > c)
    {
      const ConservativeAdvancementStackData<S>& data2 = stack[stack.size() - 2];
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
    Vector3<S> n_transformed =
      getBVAxis(model1->getBV(c1).bv, 0) * n[0] +
      getBVAxis(model1->getBV(c1).bv, 1) * n[1] +
      getBVAxis(model1->getBV(c1).bv, 2) * n[2];
    Quaternion<S> R0;
    motion1->getCurrentRotation(R0);
    n_transformed = R0 * n_transformed;
    n_transformed.normalize();

    TBVMotionBoundVisitor<BV> mb_visitor1(model1->getBV(c1).bv, n_transformed), mb_visitor2(model2->getBV(c2).bv, -n_transformed);
    S bound1 = motion1->computeMotionBound(mb_visitor1);
    S bound2 = motion2->computeMotionBound(mb_visitor2);

    S bound = bound1 + bound2;

    S cur_delta_t;
    if(bound <= c) cur_delta_t = 1;
    else cur_delta_t = c / bound;

    if(cur_delta_t < delta_t)
      delta_t = cur_delta_t;

    stack.pop_back();

    return true;
  }
  else
  {
    const ConservativeAdvancementStackData<S>& data = stack.back();
    S d = data.d;

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
    int& num_leaf_tests)
{
  using S = typename BV::S;

  if(enable_statistics) num_leaf_tests++;

  const BVNode<BV>& node1 = model1->getBV(b1);
  const BVNode<BV>& node2 = model2->getBV(b2);

  int primitive_id1 = node1.primitiveId();
  int primitive_id2 = node2.primitiveId();

  const Triangle& tri_id1 = tri_indices1[primitive_id1];
  const Triangle& tri_id2 = tri_indices2[primitive_id2];

  const Vector3<S>& t11 = vertices1[tri_id1[0]];
  const Vector3<S>& t12 = vertices1[tri_id1[1]];
  const Vector3<S>& t13 = vertices1[tri_id1[2]];

  const Vector3<S>& t21 = vertices2[tri_id2[0]];
  const Vector3<S>& t22 = vertices2[tri_id2[1]];
  const Vector3<S>& t23 = vertices2[tri_id2[2]];

  // nearest point pair
  Vector3<S> P1, P2;

  S d = TriangleDistance<S>::triDistance(t11, t12, t13, t21, t22, t23,
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
  Vector3<S> n = P2 - P1;
  /// turn n into the global frame, pointing from object 1 to object 2
  Quaternion<S> R0;
  motion1->getCurrentRotation(R0);
  Vector3<S> n_transformed = R0 * n;
  n_transformed.normalize(); // normalized here

  TriangleMotionBoundVisitor<S> mb_visitor1(t11, t12, t13, n_transformed), mb_visitor2(t21, t22, t23, -n_transformed);
  S bound1 = motion1->computeMotionBound(mb_visitor1);
  S bound2 = motion2->computeMotionBound(mb_visitor2);

  S bound = bound1 + bound2;

  S cur_delta_t;
  if(bound <= d) cur_delta_t = 1;
  else cur_delta_t = d / bound;

  if(cur_delta_t < delta_t)
    delta_t = cur_delta_t;
}

//==============================================================================
template <typename BV, typename OrientedDistanceNode>
bool setupMeshConservativeAdvancementOrientedDistanceNode(
    OrientedDistanceNode& node,
    const BVHModel<BV>& model1, const Transform3<typename BV::S>& tf1,
    const BVHModel<BV>& model2, const Transform3<typename BV::S>& tf2,
    typename BV::S w)
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

  relativeTransform(tf1.linear(), tf1.translation(), tf2.linear(), tf2.translation(), node.R, node.T);

  return true;
}

//==============================================================================
template <typename S>
bool initialize(
    MeshConservativeAdvancementTraversalNodeRSS<S>& node,
    const BVHModel<RSS<S>>& model1,
    const Transform3<S>& tf1,
    const BVHModel<RSS<S>>& model2,
    const Transform3<S>& tf2,
    S w)
{
  return detail::setupMeshConservativeAdvancementOrientedDistanceNode(
        node, model1, tf1, model2, tf2, w);
}

//==============================================================================
template <typename S>
bool initialize(
    MeshConservativeAdvancementTraversalNodeOBBRSS<S>& node,
    const BVHModel<OBBRSS<S>>& model1,
    const Transform3<S>& tf1,
    const BVHModel<OBBRSS<S>>& model2,
    const Transform3<S>& tf2,
    S w)
{
  return detail::setupMeshConservativeAdvancementOrientedDistanceNode(
        node, model1, tf1, model2, tf2, w);
}

} // namespace detail
} // namespace fcl

#endif

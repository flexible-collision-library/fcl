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

#ifndef FCL_TRAVERSAL_MESHDISTANCETRAVERSALNODE_INL_H
#define FCL_TRAVERSAL_MESHDISTANCETRAVERSALNODE_INL_H

#include "fcl/narrowphase/detail/traversal/distance/mesh_distance_traversal_node.h"

namespace fcl
{

namespace detail
{

//==============================================================================
extern template
class FCL_EXPORT MeshDistanceTraversalNodeRSS<double>;

//==============================================================================
extern template
bool initialize(
    MeshDistanceTraversalNodeRSS<double>& node,
    const BVHModel<RSS<double>>& model1,
    const Transform3<double>& tf1,
    const BVHModel<RSS<double>>& model2,
    const Transform3<double>& tf2,
    const DistanceRequest<double>& request,
    DistanceResult<double>& result);

//==============================================================================
extern template
class FCL_EXPORT MeshDistanceTraversalNodekIOS<double>;

//==============================================================================
extern template
bool initialize(
    MeshDistanceTraversalNodekIOS<double>& node,
    const BVHModel<kIOS<double>>& model1,
    const Transform3<double>& tf1,
    const BVHModel<kIOS<double>>& model2,
    const Transform3<double>& tf2,
    const DistanceRequest<double>& request,
    DistanceResult<double>& result);

//==============================================================================
extern template
class FCL_EXPORT MeshDistanceTraversalNodeOBBRSS<double>;

//==============================================================================
extern template
bool initialize(
    MeshDistanceTraversalNodeOBBRSS<double>& node,
    const BVHModel<OBBRSS<double>>& model1,
    const Transform3<double>& tf1,
    const BVHModel<OBBRSS<double>>& model2,
    const Transform3<double>& tf2,
    const DistanceRequest<double>& request,
    DistanceResult<double>& result);

//==============================================================================
template <typename BV>
MeshDistanceTraversalNode<BV>::MeshDistanceTraversalNode() : BVHDistanceTraversalNode<BV>()
{
  vertices1 = nullptr;
  vertices2 = nullptr;
  tri_indices1 = nullptr;
  tri_indices2 = nullptr;

  rel_err = this->request.rel_err;
  abs_err = this->request.abs_err;
}

//==============================================================================
template <typename BV>
void MeshDistanceTraversalNode<BV>::leafTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_leaf_tests++;

  const BVNode<BV>& node1 = this->model1->getBV(b1);
  const BVNode<BV>& node2 = this->model2->getBV(b2);

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
                                           P1, P2);

  if(this->request.enable_nearest_points)
  {
    this->result->update(d, this->model1, this->model2, primitive_id1, primitive_id2, P1, P2);
  }
  else
  {
    this->result->update(d, this->model1, this->model2, primitive_id1, primitive_id2);
  }
}

//==============================================================================
template <typename BV>
bool MeshDistanceTraversalNode<BV>::canStop(typename BV::S c) const
{
  if((c >= this->result->min_distance - abs_err) && (c * (1 + rel_err) >= this->result->min_distance))
    return true;
  return false;
}

//==============================================================================
template <typename BV>
bool initialize(
    MeshDistanceTraversalNode<BV>& node,
    BVHModel<BV>& model1,
    Transform3<typename BV::S>& tf1,
    BVHModel<BV>& model2,
    Transform3<typename BV::S>& tf2,
    const DistanceRequest<typename BV::S>& request,
    DistanceResult<typename BV::S>& result,
    bool use_refit,
    bool refit_bottomup)
{
  using S = typename BV::S;

  if(model1.getModelType() != BVH_MODEL_TRIANGLES || model2.getModelType() != BVH_MODEL_TRIANGLES)
    return false;

  if(!tf1.matrix().isIdentity())
  {
    std::vector<Vector3<S>> vertices_transformed1(model1.num_vertices);
    for(int i = 0; i < model1.num_vertices; ++i)
    {
      Vector3<S>& p = model1.vertices[i];
      Vector3<S> new_v = tf1 * p;
      vertices_transformed1[i] = new_v;
    }

    model1.beginReplaceModel();
    model1.replaceSubModel(vertices_transformed1);
    model1.endReplaceModel(use_refit, refit_bottomup);

    tf1.setIdentity();
  }

  if(!tf2.matrix().isIdentity())
  {
    std::vector<Vector3<S>> vertices_transformed2(model2.num_vertices);
    for(int i = 0; i < model2.num_vertices; ++i)
    {
      Vector3<S>& p = model2.vertices[i];
      Vector3<S> new_v = tf2 * p;
      vertices_transformed2[i] = new_v;
    }

    model2.beginReplaceModel();
    model2.replaceSubModel(vertices_transformed2);
    model2.endReplaceModel(use_refit, refit_bottomup);

    tf2.setIdentity();
  }

  node.request = request;
  node.result = &result;

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;

  node.vertices1 = model1.vertices;
  node.vertices2 = model2.vertices;

  node.tri_indices1 = model1.tri_indices;
  node.tri_indices2 = model2.tri_indices;

  return true;
}

//==============================================================================
template <typename S>
MeshDistanceTraversalNodeRSS<S>::MeshDistanceTraversalNodeRSS()
  : MeshDistanceTraversalNode<RSS<S>>(),
    tf(Transform3<S>::Identity())
{
}

//==============================================================================
template <typename S>
void MeshDistanceTraversalNodeRSS<S>::preprocess()
{
  detail::distancePreprocessOrientedNode(
        this->model1,
        this->model2,
        this->vertices1,
        this->vertices2,
        this->tri_indices1,
        this->tri_indices2,
        0,
        0,
        tf,
        this->request,
        *this->result);
}

//==============================================================================
template <typename S>
void MeshDistanceTraversalNodeRSS<S>::postprocess()
{
  detail::distancePostprocessOrientedNode(
        this->model1,
        this->model2,
        this->tf1,
        this->request,
        *this->result);
}

//==============================================================================
template <typename S>
void MeshDistanceTraversalNodeRSS<S>::leafTesting(int b1, int b2) const
{
  detail::meshDistanceOrientedNodeLeafTesting(
        b1,
        b2,
        this->model1,
        this->model2,
        this->vertices1,
        this->vertices2,
        this->tri_indices1,
        this->tri_indices2,
        tf,
        this->enable_statistics,
        this->num_leaf_tests,
        this->request,
        *this->result);
}

//==============================================================================
template <typename S>
MeshDistanceTraversalNodekIOS<S>::MeshDistanceTraversalNodekIOS()
  : MeshDistanceTraversalNode<kIOS<S>>(),
    tf(Transform3<S>::Identity())
{
  // Do nothing
}

//==============================================================================
template <typename S>
void MeshDistanceTraversalNodekIOS<S>::preprocess()
{
  detail::distancePreprocessOrientedNode(
        this->model1,
        this->model2,
        this->vertices1,
        this->vertices2,
        this->tri_indices1,
        this->tri_indices2,
        0,
        0,
        tf,
        this->request,
        *this->result);
}

//==============================================================================
template <typename S>
void MeshDistanceTraversalNodekIOS<S>::postprocess()
{
  detail::distancePostprocessOrientedNode(
        this->model1,
        this->model2,
        this->tf1,
        this->request,
        *this->result);
}

//==============================================================================
template <typename S>
void MeshDistanceTraversalNodekIOS<S>::leafTesting(int b1, int b2) const
{
  detail::meshDistanceOrientedNodeLeafTesting(
        b1,
        b2,
        this->model1,
        this->model2,
        this->vertices1,
        this->vertices2,
        this->tri_indices1,
        this->tri_indices2,
        tf,
        this->enable_statistics,
        this->num_leaf_tests,
        this->request,
        *this->result);
}

//==============================================================================
template <typename S>
MeshDistanceTraversalNodeOBBRSS<S>::MeshDistanceTraversalNodeOBBRSS()
  : MeshDistanceTraversalNode<OBBRSS<S>>(),
    tf(Transform3<S>::Identity())
{
  // Do nothing
}

//==============================================================================
template <typename S>
void MeshDistanceTraversalNodeOBBRSS<S>::preprocess()
{
  detail::distancePreprocessOrientedNode(
        this->model1,
        this->model2,
        this->vertices1,
        this->vertices2,
        this->tri_indices1,
        this->tri_indices2,
        0,
        0,
        tf,
        this->request,
        *this->result);
}

//==============================================================================
template <typename S>
void MeshDistanceTraversalNodeOBBRSS<S>::postprocess()
{
  detail::distancePostprocessOrientedNode(
        this->model1,
        this->model2,
        this->tf1,
        this->request,
        *this->result);
}

//==============================================================================
template <typename S>
void MeshDistanceTraversalNodeOBBRSS<S>::leafTesting(int b1, int b2) const
{
  detail::meshDistanceOrientedNodeLeafTesting(
        b1,
        b2,
        this->model1,
        this->model2,
        this->vertices1,
        this->vertices2,
        this->tri_indices1,
        this->tri_indices2,
        tf,
        this->enable_statistics,
        this->num_leaf_tests,
        this->request,
        *this->result);
}

//==============================================================================
template <typename BV>
void meshDistanceOrientedNodeLeafTesting(int b1,
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
    DistanceResult<typename BV::S>& result)
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

  if(request.enable_nearest_points)
    result.update(d, model1, model2, primitive_id1, primitive_id2, P1, P2);
  else
    result.update(d, model1, model2, primitive_id1, primitive_id2);
}

//==============================================================================
template <typename BV>
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
    DistanceResult<typename BV::S>& result)
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

  S d = TriangleDistance<S>::triDistance(
        t11, t12, t13, t21, t22, t23, tf, P1, P2);

  if(request.enable_nearest_points)
    result.update(d, model1, model2, primitive_id1, primitive_id2, P1, P2);
  else
    result.update(d, model1, model2, primitive_id1, primitive_id2);
}

//==============================================================================
template <typename BV>
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
    DistanceResult<typename BV::S>& result)
{
  using S = typename BV::S;

  const Triangle& init_tri1 = tri_indices1[init_tri_id1];
  const Triangle& init_tri2 = tri_indices2[init_tri_id2];

  Vector3<S> init_tri1_points[3];
  Vector3<S> init_tri2_points[3];

  init_tri1_points[0] = vertices1[init_tri1[0]];
  init_tri1_points[1] = vertices1[init_tri1[1]];
  init_tri1_points[2] = vertices1[init_tri1[2]];

  init_tri2_points[0] = vertices2[init_tri2[0]];
  init_tri2_points[1] = vertices2[init_tri2[1]];
  init_tri2_points[2] = vertices2[init_tri2[2]];

  Vector3<S> p1, p2;
  S distance = TriangleDistance<S>::triDistance(init_tri1_points[0], init_tri1_points[1], init_tri1_points[2],
      init_tri2_points[0], init_tri2_points[1], init_tri2_points[2],
      R, T, p1, p2);

  if(request.enable_nearest_points)
    result.update(distance, model1, model2, init_tri_id1, init_tri_id2, p1, p2);
  else
    result.update(distance, model1, model2, init_tri_id1, init_tri_id2);
}

//==============================================================================
template <typename BV>
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
    DistanceResult<typename BV::S>& result)
{
  using S = typename BV::S;

  const Triangle& init_tri1 = tri_indices1[init_tri_id1];
  const Triangle& init_tri2 = tri_indices2[init_tri_id2];

  Vector3<S> init_tri1_points[3];
  Vector3<S> init_tri2_points[3];

  init_tri1_points[0] = vertices1[init_tri1[0]];
  init_tri1_points[1] = vertices1[init_tri1[1]];
  init_tri1_points[2] = vertices1[init_tri1[2]];

  init_tri2_points[0] = vertices2[init_tri2[0]];
  init_tri2_points[1] = vertices2[init_tri2[1]];
  init_tri2_points[2] = vertices2[init_tri2[2]];

  Vector3<S> p1, p2;
  S distance
      = TriangleDistance<S>::triDistance(
        init_tri1_points[0], init_tri1_points[1], init_tri1_points[2],
        init_tri2_points[0], init_tri2_points[1], init_tri2_points[2],
        tf, p1, p2);

  if(request.enable_nearest_points)
    result.update(distance, model1, model2, init_tri_id1, init_tri_id2, p1, p2);
  else
    result.update(distance, model1, model2, init_tri_id1, init_tri_id2);
}

//==============================================================================
template <typename BV>
void distancePostprocessOrientedNode(
    const BVHModel<BV>* model1,
    const BVHModel<BV>* model2,
    const Transform3<typename BV::S>& tf1,
    const DistanceRequest<typename BV::S>& request,
    DistanceResult<typename BV::S>& result)
{
  /// the points obtained by triDistance are not in world space: both are in object1's local coordinate system, so we need to convert them into the world space.
  if(request.enable_nearest_points && (result.o1 == model1) && (result.o2 == model2))
  {
    result.nearest_points[0] = tf1 * result.nearest_points[0];
    result.nearest_points[1] = tf1 * result.nearest_points[1];
  }
}

template <typename BV, typename OrientedNode>
static bool setupMeshDistanceOrientedNode(
    OrientedNode& node,
    const BVHModel<BV>& model1, const Transform3<typename BV::S>& tf1,
    const BVHModel<BV>& model2, const Transform3<typename BV::S>& tf2,
    const DistanceRequest<typename BV::S>& request,
    DistanceResult<typename BV::S>& result)
{
  if(model1.getModelType() != BVH_MODEL_TRIANGLES || model2.getModelType() != BVH_MODEL_TRIANGLES)
    return false;

  node.request = request;
  node.result = &result;

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;

  node.vertices1 = model1.vertices;
  node.vertices2 = model2.vertices;

  node.tri_indices1 = model1.tri_indices;
  node.tri_indices2 = model2.tri_indices;

  node.tf = tf1.inverse(Eigen::Isometry) * tf2;

  return true;
}

//==============================================================================
template <typename S>
bool initialize(
    MeshDistanceTraversalNodeRSS<S>& node,
    const BVHModel<RSS<S>>& model1,
    const Transform3<S>& tf1,
    const BVHModel<RSS<S>>& model2,
    const Transform3<S>& tf2,
    const DistanceRequest<S>& request,
    DistanceResult<S>& result)
{
  return detail::setupMeshDistanceOrientedNode(
        node, model1, tf1, model2, tf2, request, result);
}

//==============================================================================
template <typename S>
bool initialize(
    MeshDistanceTraversalNodekIOS<S>& node,
    const BVHModel<kIOS<S>>& model1,
    const Transform3<S>& tf1,
    const BVHModel<kIOS<S>>& model2,
    const Transform3<S>& tf2,
    const DistanceRequest<S>& request,
    DistanceResult<S>& result)
{
  return detail::setupMeshDistanceOrientedNode(
        node, model1, tf1, model2, tf2, request, result);
}

//==============================================================================
template <typename S>
bool initialize(
    MeshDistanceTraversalNodeOBBRSS<S>& node,
    const BVHModel<OBBRSS<S>>& model1,
    const Transform3<S>& tf1,
    const BVHModel<OBBRSS<S>>& model2,
    const Transform3<S>& tf2,
    const DistanceRequest<S>& request,
    DistanceResult<S>& result)
{
  return detail::setupMeshDistanceOrientedNode(
        node, model1, tf1, model2, tf2, request, result);
}

} // namespace detail
} // namespace fcl

#endif

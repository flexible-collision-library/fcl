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


#ifndef FCL_TRAVERSAL_MESHDISTANCETRAVERSALNODE_H
#define FCL_TRAVERSAL_MESHDISTANCETRAVERSALNODE_H

#include "fcl/intersect.h"
#include "fcl/traversal/bvh_distance_traversal_node.h"

namespace fcl
{

/// @brief Traversal node for distance computation between two meshes
template <typename BV>
class MeshDistanceTraversalNode : public BVHDistanceTraversalNode<BV>
{
public:

  using Scalar = typename BV::Scalar;

  MeshDistanceTraversalNode();

  /// @brief Distance testing between leaves (two triangles)
  void leafTesting(int b1, int b2) const;

  /// @brief Whether the traversal process can stop early
  bool canStop(Scalar c) const;

  Vector3<Scalar>* vertices1;
  Vector3<Scalar>* vertices2;

  Triangle* tri_indices1;
  Triangle* tri_indices2;

  /// @brief relative and absolute error, default value is 0.01 for both terms
  Scalar rel_err;
  Scalar abs_err;
};

/// @brief Initialize traversal node for distance computation between two
/// meshes, given the current transforms
template <typename BV>
bool initialize(
    MeshDistanceTraversalNode<BV>& node,
    BVHModel<BV>& model1,
    Transform3<typename BV::Scalar>& tf1,
    BVHModel<BV>& model2,
    Transform3<typename BV::Scalar>& tf2,
    const DistanceRequest<typename BV::Scalar>& request,
    DistanceResult<typename BV::Scalar>& result,
    bool use_refit = false, bool refit_bottomup = false);

/// @brief Traversal node for distance computation between two meshes if their underlying BVH node is oriented node (RSS, OBBRSS, kIOS)
template <typename Scalar>
class MeshDistanceTraversalNodeRSS
    : public MeshDistanceTraversalNode<RSS<Scalar>>
{
public:
  MeshDistanceTraversalNodeRSS();

  void preprocess();

  void postprocess();

  Scalar BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  Matrix3<Scalar> R;
  Vector3<Scalar> T;
};

using MeshDistanceTraversalNodeRSSf = MeshDistanceTraversalNodeRSS<float>;
using MeshDistanceTraversalNodeRSSd = MeshDistanceTraversalNodeRSS<double>;

/// @brief Initialize traversal node for distance computation between two
///  meshes, specialized for RSS type
template <typename Scalar>
bool initialize(
    MeshDistanceTraversalNodeRSS<Scalar>& node,
    const BVHModel<RSS<Scalar>>& model1,
    const Transform3<Scalar>& tf1,
    const BVHModel<RSS<Scalar>>& model2,
    const Transform3<Scalar>& tf2,
    const DistanceRequest<Scalar>& request,
    DistanceResult<Scalar>& result);

template <typename Scalar>
class MeshDistanceTraversalNodekIOS
    : public MeshDistanceTraversalNode<kIOS<Scalar>>
{
public:
  MeshDistanceTraversalNodekIOS();

  void preprocess();
  
  void postprocess();

  Scalar BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  Matrix3<Scalar> R;
  Vector3<Scalar> T;
};

using MeshDistanceTraversalNodekIOSf = MeshDistanceTraversalNodekIOS<float>;
using MeshDistanceTraversalNodekIOSd = MeshDistanceTraversalNodekIOS<double>;

/// @brief Initialize traversal node for distance computation between two
///  meshes, specialized for kIOS type
template <typename Scalar>
bool initialize(
    MeshDistanceTraversalNodekIOS<Scalar>& node,
    const BVHModel<kIOS<Scalar>>& model1,
    const Transform3<Scalar>& tf1,
    const BVHModel<kIOS<Scalar>>& model2,
    const Transform3<Scalar>& tf2,
    const DistanceRequest<Scalar>& request,
    DistanceResult<Scalar>& result);

template <typename Scalar>
class MeshDistanceTraversalNodeOBBRSS
    : public MeshDistanceTraversalNode<OBBRSS<Scalar>>
{
public:
  MeshDistanceTraversalNodeOBBRSS();

  void preprocess();

  void postprocess();

  Scalar BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  Matrix3<Scalar> R;
  Vector3<Scalar> T;
};

using MeshDistanceTraversalNodeOBBRSSf = MeshDistanceTraversalNodeOBBRSS<float>;
using MeshDistanceTraversalNodeOBBRSSd = MeshDistanceTraversalNodeOBBRSS<double>;

/// @brief Initialize traversal node for distance computation between two
///  meshes, specialized for OBBRSS type
template <typename Scalar>
bool initialize(
    MeshDistanceTraversalNodeOBBRSS<Scalar>& node,
    const BVHModel<OBBRSS<Scalar>>& model1,
    const Transform3<Scalar>& tf1,
    const BVHModel<OBBRSS<Scalar>>& model2,
    const Transform3<Scalar>& tf2,
    const DistanceRequest<Scalar>& request,
    DistanceResult<Scalar>& result);

namespace details
{

template <typename BV>
void meshDistanceOrientedNodeLeafTesting(
    int b1,
    int b2,
    const BVHModel<BV>* model1,
    const BVHModel<BV>* model2,
    Vector3<typename BV::Scalar>* vertices1,
    Vector3<typename BV::Scalar>* vertices2,
    Triangle* tri_indices1,
    Triangle* tri_indices2,
    const Matrix3<typename BV::Scalar>& R,
    const Vector3<typename BV::Scalar>& T,
    bool enable_statistics,
    int& num_leaf_tests,
    const DistanceRequest<typename BV::Scalar>& request,
    DistanceResult<typename BV::Scalar>& result);

template <typename BV>
void distancePreprocessOrientedNode(
    const BVHModel<BV>* model1,
    const BVHModel<BV>* model2,
    const Vector3d* vertices1,
    Vector3<typename BV::Scalar>* vertices2,
    Triangle* tri_indices1,
    Triangle* tri_indices2,
    int init_tri_id1,
    int init_tri_id2,
    const Matrix3<typename BV::Scalar>& R,
    const Vector3<typename BV::Scalar>& T,
    const DistanceRequest<typename BV::Scalar>& request,
    DistanceResult<typename BV::Scalar>& result);

template <typename BV>
void distancePostprocessOrientedNode(
    const BVHModel<BV>* model1,
    const BVHModel<BV>* model2,
    const Transform3<typename BV::Scalar>& tf1,
    const DistanceRequest<typename BV::Scalar>& request,
    DistanceResult<typename BV::Scalar>& result);

} // namespace details

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename BV>
MeshDistanceTraversalNode<BV>::MeshDistanceTraversalNode() : BVHDistanceTraversalNode<BV>()
{
  vertices1 = NULL;
  vertices2 = NULL;
  tri_indices1 = NULL;
  tri_indices2 = NULL;

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

  const Vector3<Scalar>& t11 = vertices1[tri_id1[0]];
  const Vector3<Scalar>& t12 = vertices1[tri_id1[1]];
  const Vector3<Scalar>& t13 = vertices1[tri_id1[2]];

  const Vector3<Scalar>& t21 = vertices2[tri_id2[0]];
  const Vector3<Scalar>& t22 = vertices2[tri_id2[1]];
  const Vector3<Scalar>& t23 = vertices2[tri_id2[2]];

  // nearest point pair
  Vector3<Scalar> P1, P2;

  Scalar d = TriangleDistance::triDistance(t11, t12, t13, t21, t22, t23,
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
bool MeshDistanceTraversalNode<BV>::canStop(MeshDistanceTraversalNode<BV>::Scalar c) const
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
    Transform3<typename BV::Scalar>& tf1,
    BVHModel<BV>& model2,
    Transform3<typename BV::Scalar>& tf2,
    const DistanceRequest<typename BV::Scalar>& request,
    DistanceResult<typename BV::Scalar>& result,
    bool use_refit,
    bool refit_bottomup)
{
  if(model1.getModelType() != BVH_MODEL_TRIANGLES || model2.getModelType() != BVH_MODEL_TRIANGLES)
    return false;

  if(!tf1.matrix().isIdentity())
  {
    std::vector<Vector3d> vertices_transformed1(model1.num_vertices);
    for(int i = 0; i < model1.num_vertices; ++i)
    {
      Vector3d& p = model1.vertices[i];
      Vector3d new_v = tf1 * p;
      vertices_transformed1[i] = new_v;
    }

    model1.beginReplaceModel();
    model1.replaceSubModel(vertices_transformed1);
    model1.endReplaceModel(use_refit, refit_bottomup);

    tf1.setIdentity();
  }

  if(!tf2.matrix().isIdentity())
  {
    std::vector<Vector3d> vertices_transformed2(model2.num_vertices);
    for(int i = 0; i < model2.num_vertices; ++i)
    {
      Vector3d& p = model2.vertices[i];
      Vector3d new_v = tf2 * p;
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
template <typename Scalar>
MeshDistanceTraversalNodeRSS<Scalar>::MeshDistanceTraversalNodeRSS() : MeshDistanceTraversalNode<RSS<Scalar>>(), R(Matrix3<Scalar>::Identity()), T(Vector3<Scalar>::Zero())
{
}

//==============================================================================
template <typename Scalar>
void MeshDistanceTraversalNodeRSS<Scalar>::preprocess()
{
  details::distancePreprocessOrientedNode(
        this->model1,
        this->model2,
        this->vertices1,
        this->vertices2,
        this->tri_indices1,
        this->tri_indices2,
        0,
        0,
        R,
        T,
        this->request,
        *this->result);
}

//==============================================================================
template <typename Scalar>
void MeshDistanceTraversalNodeRSS<Scalar>::postprocess()
{
  details::distancePostprocessOrientedNode(
        this->model1,
        this->model2,
        this->tf1,
        this->request,
        *this->result);
}

//==============================================================================
template <typename Scalar>
Scalar MeshDistanceTraversalNodeRSS<Scalar>::BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;

  return distance(
        R,
        T,
        this->model1->getBV(b1).bv,
        this->model2->getBV(b2).bv);
}

//==============================================================================
template <typename Scalar>
void MeshDistanceTraversalNodeRSS<Scalar>::leafTesting(int b1, int b2) const
{
  details::meshDistanceOrientedNodeLeafTesting(
        b1,
        b2,
        this->model1,
        this->model2,
        this->vertices1,
        this->vertices2,
        this->tri_indices1,
        this->tri_indices2,
        R,
        T,
        this->enable_statistics,
        this->num_leaf_tests,
        this->request,
        *this->result);
}

//==============================================================================
template <typename Scalar>
MeshDistanceTraversalNodekIOS<Scalar>::MeshDistanceTraversalNodekIOS()
  : MeshDistanceTraversalNode<kIOS<Scalar>>()
{
  R.setIdentity();
}

//==============================================================================
template <typename Scalar>
void MeshDistanceTraversalNodekIOS<Scalar>::preprocess()
{
  details::distancePreprocessOrientedNode(
        this->model1,
        this->model2,
        this->vertices1,
        this->vertices2,
        this->tri_indices1,
        this->tri_indices2,
        0,
        0,
        R,
        T,
        this->request,
        *this->result);
}

//==============================================================================
template <typename Scalar>
void MeshDistanceTraversalNodekIOS<Scalar>::postprocess()
{
  details::distancePostprocessOrientedNode(
        this->model1,
        this->model2,
        this->tf1,
        this->request,
        *this->result);
}

//==============================================================================
template <typename Scalar>
Scalar MeshDistanceTraversalNodekIOS<Scalar>::BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;

  return distance(
        R,
        T,
        this->model1->getBV(b1).bv,
        this->model2->getBV(b2).bv);
}

//==============================================================================
template <typename Scalar>
void MeshDistanceTraversalNodekIOS<Scalar>::leafTesting(int b1, int b2) const
{
  details::meshDistanceOrientedNodeLeafTesting(
        b1,
        b2,
        this->model1,
        this->model2,
        this->vertices1,
        this->vertices2,
        this->tri_indices1,
        this->tri_indices2,
        R,
        T,
        this->enable_statistics,
        this->num_leaf_tests,
        this->request,
        *this->result);
}

//==============================================================================
template <typename Scalar>
MeshDistanceTraversalNodeOBBRSS<Scalar>::MeshDistanceTraversalNodeOBBRSS() : MeshDistanceTraversalNode<OBBRSS<Scalar>>()
{
  R.setIdentity();
}

//==============================================================================
template <typename Scalar>
void MeshDistanceTraversalNodeOBBRSS<Scalar>::preprocess()
{
  details::distancePreprocessOrientedNode(
        this->model1,
        this->model2,
        this->vertices1,
        this->vertices2,
        this->tri_indices1,
        this->tri_indices2,
        0,
        0,
        R,
        T,
        this->request,
        *this->result);
}

//==============================================================================
template <typename Scalar>
void MeshDistanceTraversalNodeOBBRSS<Scalar>::postprocess()
{
  details::distancePostprocessOrientedNode(
        this->model1,
        this->model2,
        this->tf1,
        this->request,
        *this->result);
}

//==============================================================================
template <typename Scalar>
Scalar MeshDistanceTraversalNodeOBBRSS<Scalar>::BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;

  return distance(
        R,
        T,
        this->model1->getBV(b1).bv,
        this->model2->getBV(b2).bv);
}

//==============================================================================
template <typename Scalar>
void MeshDistanceTraversalNodeOBBRSS<Scalar>::leafTesting(int b1, int b2) const
{
  details::meshDistanceOrientedNodeLeafTesting(
        b1,
        b2,
        this->model1,
        this->model2,
        this->vertices1,
        this->vertices2,
        this->tri_indices1,
        this->tri_indices2,
        R,
        T,
        this->enable_statistics,
        this->num_leaf_tests,
        this->request,
        *this->result);
}

namespace details
{

//==============================================================================
template <typename BV>
void meshDistanceOrientedNodeLeafTesting(int b1,
    int b2,
    const BVHModel<BV>* model1,
    const BVHModel<BV>* model2,
    Vector3<typename BV::Scalar>* vertices1,
    Vector3<typename BV::Scalar>* vertices2,
    Triangle* tri_indices1,
    Triangle* tri_indices2,
    const Matrix3<typename BV::Scalar>& R,
    const Vector3<typename BV::Scalar>& T,
    bool enable_statistics,
    int& num_leaf_tests,
    const DistanceRequest<typename BV::Scalar>& request,
    DistanceResult<typename BV::Scalar>& result)
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

  Scalar d = TriangleDistance::triDistance(t11, t12, t13, t21, t22, t23,
                                           R, T,
                                           P1, P2);

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
    const Vector3d* vertices1,
    Vector3<typename BV::Scalar>* vertices2,
    Triangle* tri_indices1,
    Triangle* tri_indices2,
    int init_tri_id1,
    int init_tri_id2,
    const Matrix3<typename BV::Scalar>& R,
    const Vector3<typename BV::Scalar>& T,
    const DistanceRequest<typename BV::Scalar>& request,
    DistanceResult<typename BV::Scalar>& result)
{
  using Scalar = typename BV::Scalar;

  const Triangle& init_tri1 = tri_indices1[init_tri_id1];
  const Triangle& init_tri2 = tri_indices2[init_tri_id2];

  Vector3<Scalar> init_tri1_points[3];
  Vector3<Scalar> init_tri2_points[3];

  init_tri1_points[0] = vertices1[init_tri1[0]];
  init_tri1_points[1] = vertices1[init_tri1[1]];
  init_tri1_points[2] = vertices1[init_tri1[2]];

  init_tri2_points[0] = vertices2[init_tri2[0]];
  init_tri2_points[1] = vertices2[init_tri2[1]];
  init_tri2_points[2] = vertices2[init_tri2[2]];

  Vector3<Scalar> p1, p2;
  Scalar distance = TriangleDistance::triDistance(init_tri1_points[0], init_tri1_points[1], init_tri1_points[2],
      init_tri2_points[0], init_tri2_points[1], init_tri2_points[2],
      R, T, p1, p2);

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
    const Transform3<typename BV::Scalar>& tf1,
    const DistanceRequest<typename BV::Scalar>& request,
    DistanceResult<typename BV::Scalar>& result)
{
  /// the points obtained by triDistance are not in world space: both are in object1's local coordinate system, so we need to convert them into the world space.
  if(request.enable_nearest_points && (result.o1 == model1) && (result.o2 == model2))
  {
    result.nearest_points[0] = tf1 * result.nearest_points[0];
    result.nearest_points[1] = tf1 * result.nearest_points[1];
  }
}

} // namespace details

namespace details
{
template <typename BV, typename OrientedNode>
static inline bool setupMeshDistanceOrientedNode(
    OrientedNode& node,
    const BVHModel<BV>& model1, const Transform3d& tf1,
    const BVHModel<BV>& model2, const Transform3d& tf2,
    const DistanceRequest<typename BV::Scalar>& request,
    DistanceResult<typename BV::Scalar>& result)
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

  relativeTransform(tf1, tf2, node.R, node.T);

  return true;
}

} // namespace details

//==============================================================================
template <typename Scalar>
bool initialize(
    MeshDistanceTraversalNodeRSS<Scalar>& node,
    const BVHModel<RSS<Scalar>>& model1,
    const Transform3<Scalar>& tf1,
    const BVHModel<RSS<Scalar>>& model2,
    const Transform3<Scalar>& tf2,
    const DistanceRequest<Scalar>& request,
    DistanceResult<Scalar>& result)
{
  return details::setupMeshDistanceOrientedNode(
        node, model1, tf1, model2, tf2, request, result);
}

//==============================================================================
template <typename Scalar>
bool initialize(
    MeshDistanceTraversalNodekIOS<Scalar>& node,
    const BVHModel<kIOS<Scalar>>& model1,
    const Transform3<Scalar>& tf1,
    const BVHModel<kIOS<Scalar>>& model2,
    const Transform3<Scalar>& tf2,
    const DistanceRequest<Scalar>& request,
    DistanceResult<Scalar>& result)
{
  return details::setupMeshDistanceOrientedNode(
        node, model1, tf1, model2, tf2, request, result);
}

//==============================================================================
template <typename Scalar>
bool initialize(
    MeshDistanceTraversalNodeOBBRSS<Scalar>& node,
    const BVHModel<OBBRSS<Scalar>>& model1,
    const Transform3<Scalar>& tf1,
    const BVHModel<OBBRSS<Scalar>>& model2,
    const Transform3<Scalar>& tf2,
    const DistanceRequest<Scalar>& request,
    DistanceResult<Scalar>& result)
{
  return details::setupMeshDistanceOrientedNode(
        node, model1, tf1, model2, tf2, request, result);
}

} // namespace fcl

#endif

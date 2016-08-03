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

#ifndef FCL_TRAVERSAL_MESHSHAPEDISTANCETRAVERSALNODE_H
#define FCL_TRAVERSAL_MESHSHAPEDISTANCETRAVERSALNODE_H

#include "fcl/shape/compute_bv.h"
#include "fcl/traversal/distance/bvh_shape_distance_traversal_node.h"

namespace fcl
{

/// @brief Traversal node for distance between mesh and shape
template <typename BV, typename S, typename NarrowPhaseSolver>
class MeshShapeDistanceTraversalNode
    : public BVHShapeDistanceTraversalNode<BV, S>
{ 
public:

  using Scalar = typename BV::Scalar;

  MeshShapeDistanceTraversalNode();

  /// @brief Distance testing between leaves (one triangle and one shape)
  void leafTesting(int b1, int b2) const;

  /// @brief Whether the traversal process can stop early
  bool canStop(Scalar c) const;

  Vector3<Scalar>* vertices;
  Triangle* tri_indices;

  Scalar rel_err;
  Scalar abs_err;
    
  const NarrowPhaseSolver* nsolver;
};

/// @cond IGNORE
namespace details
{

template <typename BV, typename S, typename NarrowPhaseSolver>
void meshShapeDistanceOrientedNodeLeafTesting(
    int b1, int /* b2 */,
    const BVHModel<BV>* model1,
    const S& model2,
    Vector3<typename BV::Scalar>* vertices,
    Triangle* tri_indices,
    const Transform3<typename BV::Scalar>& tf1,
    const Transform3<typename BV::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    bool enable_statistics,
    int & num_leaf_tests,
    const DistanceRequest<typename BV::Scalar>& /* request */,
    DistanceResult<typename BV::Scalar>& result);


template <typename BV, typename S, typename NarrowPhaseSolver>
void distancePreprocessOrientedNode(
    const BVHModel<BV>* model1,
    Vector3<typename BV::Scalar>* vertices,
    Triangle* tri_indices,
    int init_tri_id,
    const S& model2,
    const Transform3<typename BV::Scalar>& tf1,
    const Transform3<typename BV::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename BV::Scalar>& /* request */,
    DistanceResult<typename BV::Scalar>& result);

} // namespace details

/// @endcond

/// @brief Initialize traversal node for distance computation between one mesh
/// and one shape, given the current transforms
template <typename BV, typename S, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeDistanceTraversalNode<BV, S, NarrowPhaseSolver>& node,
    BVHModel<BV>& model1,
    Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const S& model2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename NarrowPhaseSolver::Scalar>& request,
    DistanceResult<typename NarrowPhaseSolver::Scalar>& result,
    bool use_refit = false, bool refit_bottomup = false);

/// @brief Traversal node for distance between mesh and shape, when mesh BVH is one of the oriented node (RSS, OBBRSS, kIOS)
template <typename S, typename NarrowPhaseSolver>
class MeshShapeDistanceTraversalNodeRSS
    : public MeshShapeDistanceTraversalNode<
    RSS<typename NarrowPhaseSolver::Scalar>, S, NarrowPhaseSolver>
{
public:
  using Scalar = typename NarrowPhaseSolver::Scalar;

  MeshShapeDistanceTraversalNodeRSS();

  void preprocess();

  void postprocess();

  Scalar BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;
};

template <typename S, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeDistanceTraversalNodeRSS<S, NarrowPhaseSolver>& node,
    const BVHModel<RSS<typename NarrowPhaseSolver::Scalar>>& model1, const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const S& model2, const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename NarrowPhaseSolver::Scalar>& request,
    DistanceResult<typename NarrowPhaseSolver::Scalar>& result);

template <typename S, typename NarrowPhaseSolver>
class MeshShapeDistanceTraversalNodekIOS
    : public MeshShapeDistanceTraversalNode<kIOS<typename NarrowPhaseSolver::Scalar>, S, NarrowPhaseSolver>
{
public:
  using Scalar = typename NarrowPhaseSolver::Scalar;

  MeshShapeDistanceTraversalNodekIOS();

  void preprocess();

  void postprocess();

  Scalar BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

};

/// @brief Initialize traversal node for distance computation between one mesh and one shape, specialized for kIOS type
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeDistanceTraversalNodekIOS<S, NarrowPhaseSolver>& node,
    const BVHModel<kIOS<typename NarrowPhaseSolver::Scalar>>& model1, const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const S& model2, const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename NarrowPhaseSolver::Scalar>& request,
    DistanceResult<typename NarrowPhaseSolver::Scalar>& result);

template <typename S, typename NarrowPhaseSolver>
class MeshShapeDistanceTraversalNodeOBBRSS
    : public MeshShapeDistanceTraversalNode<OBBRSS<typename NarrowPhaseSolver::Scalar>, S, NarrowPhaseSolver>
{
public:
  using Scalar = typename NarrowPhaseSolver::Scalar;

  MeshShapeDistanceTraversalNodeOBBRSS();

  void preprocess();

  void postprocess();

  Scalar BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;
  
};

/// @brief Initialize traversal node for distance computation between one mesh and one shape, specialized for OBBRSS type
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeDistanceTraversalNodeOBBRSS<S, NarrowPhaseSolver>& node,
    const BVHModel<OBBRSS<typename NarrowPhaseSolver::Scalar>>& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const S& model2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename NarrowPhaseSolver::Scalar>& request,
    DistanceResult<typename NarrowPhaseSolver::Scalar>& result);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename BV, typename S, typename NarrowPhaseSolver>
MeshShapeDistanceTraversalNode<BV, S, NarrowPhaseSolver>::
MeshShapeDistanceTraversalNode()
  : BVHShapeDistanceTraversalNode<BV, S>()
{
  vertices = NULL;
  tri_indices = NULL;

  rel_err = 0;
  abs_err = 0;

  nsolver = NULL;
}

//==============================================================================
template <typename BV, typename S, typename NarrowPhaseSolver>
void MeshShapeDistanceTraversalNode<BV, S, NarrowPhaseSolver>::
leafTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_leaf_tests++;

  const BVNode<BV>& node = this->model1->getBV(b1);

  int primitive_id = node.primitiveId();

  const Triangle& tri_id = tri_indices[primitive_id];

  const Vector3<Scalar>& p1 = vertices[tri_id[0]];
  const Vector3<Scalar>& p2 = vertices[tri_id[1]];
  const Vector3<Scalar>& p3 = vertices[tri_id[2]];

  Scalar d;
  Vector3<Scalar> closest_p1, closest_p2;
  nsolver->shapeTriangleDistance(*(this->model2), this->tf2, p1, p2, p3, &d, &closest_p2, &closest_p1);

  this->result->update(
        d,
        this->model1,
        this->model2,
        primitive_id,
        DistanceResult<Scalar>::NONE,
        closest_p1,
        closest_p2);
}

//==============================================================================
template <typename BV, typename S, typename NarrowPhaseSolver>
bool MeshShapeDistanceTraversalNode<BV, S, NarrowPhaseSolver>::
canStop(Scalar c) const
{
  if((c >= this->result->min_distance - abs_err) && (c * (1 + rel_err) >= this->result->min_distance))
    return true;
  return false;
}

//==============================================================================
template <typename BV, typename S, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeDistanceTraversalNode<BV, S, NarrowPhaseSolver>& node,
    BVHModel<BV>& model1,
    Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const S& model2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename NarrowPhaseSolver::Scalar>& request,
    DistanceResult<typename NarrowPhaseSolver::Scalar>& result,
    bool use_refit,
    bool refit_bottomup)
{
  using Scalar = typename BV::Scalar;

  if(model1.getModelType() != BVH_MODEL_TRIANGLES)
    return false;

  if(!tf1.matrix().isIdentity())
  {
    std::vector<Vector3<Scalar>> vertices_transformed1(model1.num_vertices);
    for(int i = 0; i < model1.num_vertices; ++i)
    {
      Vector3<Scalar>& p = model1.vertices[i];
      Vector3<Scalar> new_v = tf1 * p;
      vertices_transformed1[i] = new_v;
    }

    model1.beginReplaceModel();
    model1.replaceSubModel(vertices_transformed1);
    model1.endReplaceModel(use_refit, refit_bottomup);

    tf1.setIdentity();
  }

  node.request = request;
  node.result = &result;

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;
  node.nsolver = nsolver;

  node.vertices = model1.vertices;
  node.tri_indices = model1.tri_indices;

  computeBV(model2, tf2, node.model2_bv);

  return true;
}

/// @cond IGNORE
namespace details
{

//==============================================================================
template <typename BV, typename S, typename NarrowPhaseSolver>
void meshShapeDistanceOrientedNodeLeafTesting(
    int b1, int /* b2 */,
    const BVHModel<BV>* model1, const S& model2,
    Vector3<typename BV::Scalar>* vertices, Triangle* tri_indices,
    const Transform3<typename BV::Scalar>& tf1,
    const Transform3<typename BV::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    bool enable_statistics,
    int & num_leaf_tests,
    const DistanceRequest<typename BV::Scalar>& /* request */,
    DistanceResult<typename BV::Scalar>& result)
{
  using Scalar = typename BV::Scalar;

  if(enable_statistics) num_leaf_tests++;

  const BVNode<BV>& node = model1->getBV(b1);
  int primitive_id = node.primitiveId();

  const Triangle& tri_id = tri_indices[primitive_id];
  const Vector3<Scalar>& p1 = vertices[tri_id[0]];
  const Vector3<Scalar>& p2 = vertices[tri_id[1]];
  const Vector3<Scalar>& p3 = vertices[tri_id[2]];

  Scalar distance;
  Vector3<Scalar> closest_p1, closest_p2;
  nsolver->shapeTriangleDistance(model2, tf2, p1, p2, p3, tf1, &distance, &closest_p2, &closest_p1);

  result.update(
        distance,
        model1,
        &model2,
        primitive_id,
        DistanceResult<Scalar>::NONE,
        closest_p1,
        closest_p2);
}

//==============================================================================
template <typename BV, typename S, typename NarrowPhaseSolver>
void distancePreprocessOrientedNode(
    const BVHModel<BV>* model1,
    Vector3<typename BV::Scalar>* vertices,
    Triangle* tri_indices,
    int init_tri_id,
    const S& model2,
    const Transform3<typename BV::Scalar>& tf1,
    const Transform3<typename BV::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename BV::Scalar>& /* request */,
    DistanceResult<typename BV::Scalar>& result)
{
  using Scalar = typename BV::Scalar;

  const Triangle& init_tri = tri_indices[init_tri_id];

  const Vector3<Scalar>& p1 = vertices[init_tri[0]];
  const Vector3<Scalar>& p2 = vertices[init_tri[1]];
  const Vector3<Scalar>& p3 = vertices[init_tri[2]];

  Scalar distance;
  Vector3<Scalar> closest_p1, closest_p2;
  nsolver->shapeTriangleDistance(model2, tf2, p1, p2, p3, tf1, &distance, &closest_p2, &closest_p1);

  result.update(
        distance,
        model1,
        &model2,
        init_tri_id,
        DistanceResult<Scalar>::NONE,
        closest_p1,
        closest_p2);
}

} // namespace details

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
MeshShapeDistanceTraversalNodeRSS<S, NarrowPhaseSolver>::
MeshShapeDistanceTraversalNodeRSS()
  : MeshShapeDistanceTraversalNode<
    RSS<typename NarrowPhaseSolver::Scalar>, S, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void MeshShapeDistanceTraversalNodeRSS<S, NarrowPhaseSolver>::preprocess()
{
  details::distancePreprocessOrientedNode(
        this->model1,
        this->vertices,
        this->tri_indices,
        0,
        *(this->model2),
        this->tf1,
        this->tf2,
        this->nsolver,
        this->request,
        *(this->result));
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void MeshShapeDistanceTraversalNodeRSS<S, NarrowPhaseSolver>::postprocess()
{
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
typename NarrowPhaseSolver::Scalar
MeshShapeDistanceTraversalNodeRSS<S, NarrowPhaseSolver>::BVTesting(
    int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;
  return distance(this->tf1.linear(), this->tf1.translation(), this->model2_bv, this->model1->getBV(b1).bv);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void MeshShapeDistanceTraversalNodeRSS<S, NarrowPhaseSolver>::
leafTesting(int b1, int b2) const
{
  details::meshShapeDistanceOrientedNodeLeafTesting(
        b1,
        b2,
        this->model1,
        *(this->model2),
        this->vertices,
        this->tri_indices,
        this->tf1,
        this->tf2,
        this->nsolver,
        this->enable_statistics,
        this->num_leaf_tests,
        this->request,
        *(this->result));
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
MeshShapeDistanceTraversalNodekIOS<S, NarrowPhaseSolver>::MeshShapeDistanceTraversalNodekIOS() : MeshShapeDistanceTraversalNode<kIOS<Scalar>, S, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void MeshShapeDistanceTraversalNodekIOS<S, NarrowPhaseSolver>::preprocess()
{
  details::distancePreprocessOrientedNode(this->model1, this->vertices, this->tri_indices, 0,
                                          *(this->model2), this->tf1, this->tf2, this->nsolver, this->request, *(this->result));
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void MeshShapeDistanceTraversalNodekIOS<S, NarrowPhaseSolver>::postprocess()
{
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
typename NarrowPhaseSolver::Scalar MeshShapeDistanceTraversalNodekIOS<S, NarrowPhaseSolver>::BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;
  return distance(this->tf1.linear(), this->tf1.translation(), this->model2_bv, this->model1->getBV(b1).bv);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void MeshShapeDistanceTraversalNodekIOS<S, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  details::meshShapeDistanceOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                    this->tf1, this->tf2, this->nsolver, this->enable_statistics, this->num_leaf_tests, this->request, *(this->result));
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
MeshShapeDistanceTraversalNodeOBBRSS<S, NarrowPhaseSolver>::MeshShapeDistanceTraversalNodeOBBRSS() : MeshShapeDistanceTraversalNode<OBBRSS<Scalar>, S, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void MeshShapeDistanceTraversalNodeOBBRSS<S, NarrowPhaseSolver>::preprocess()
{
  details::distancePreprocessOrientedNode(this->model1, this->vertices, this->tri_indices, 0,
                                          *(this->model2), this->tf1, this->tf2, this->nsolver, this->request, *(this->result));
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void MeshShapeDistanceTraversalNodeOBBRSS<S, NarrowPhaseSolver>::postprocess()
{

}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
typename NarrowPhaseSolver::Scalar
MeshShapeDistanceTraversalNodeOBBRSS<S, NarrowPhaseSolver>::
BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;
  return distance(this->tf1.linear(), this->tf1.translation(), this->model2_bv, this->model1->getBV(b1).bv);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void MeshShapeDistanceTraversalNodeOBBRSS<S, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  details::meshShapeDistanceOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                    this->tf1, this->tf2, this->nsolver, this->enable_statistics, this->num_leaf_tests, this->request, *(this->result));
}

/// @cond IGNORE
namespace details
{

template <typename BV, typename S, typename NarrowPhaseSolver, template <typename, typename> class OrientedNode>
static inline bool setupMeshShapeDistanceOrientedNode(OrientedNode<S, NarrowPhaseSolver>& node,
                                                      const BVHModel<BV>& model1, const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
                                                      const S& model2, const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
                                                      const NarrowPhaseSolver* nsolver,
                                                      const DistanceRequest<typename NarrowPhaseSolver::Scalar>& request,
                                                      DistanceResult<typename NarrowPhaseSolver::Scalar>& result)
{
  if(model1.getModelType() != BVH_MODEL_TRIANGLES)
    return false;

  node.request = request;
  node.result = &result;

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;
  node.nsolver = nsolver;

  computeBV(model2, tf2, node.model2_bv);

  node.vertices = model1.vertices;
  node.tri_indices = model1.tri_indices;

  return true;
}

} // namespace details
/// @endcond

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeDistanceTraversalNodeRSS<S, NarrowPhaseSolver>& node,
    const BVHModel<RSS<typename NarrowPhaseSolver::Scalar>>& model1, const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const S& model2, const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename NarrowPhaseSolver::Scalar>& request,
    DistanceResult<typename NarrowPhaseSolver::Scalar>& result)
{
  return details::setupMeshShapeDistanceOrientedNode(node, model1, tf1, model2, tf2, nsolver, request, result);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeDistanceTraversalNodekIOS<S, NarrowPhaseSolver>& node,
    const BVHModel<kIOS<typename NarrowPhaseSolver::Scalar>>& model1, const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const S& model2, const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename NarrowPhaseSolver::Scalar>& request,
    DistanceResult<typename NarrowPhaseSolver::Scalar>& result)
{
  return details::setupMeshShapeDistanceOrientedNode(node, model1, tf1, model2, tf2, nsolver, request, result);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeDistanceTraversalNodeOBBRSS<S, NarrowPhaseSolver>& node,
    const BVHModel<OBBRSS<typename NarrowPhaseSolver::Scalar>>& model1, const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const S& model2, const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename NarrowPhaseSolver::Scalar>& request,
    DistanceResult<typename NarrowPhaseSolver::Scalar>& result)
{
  return details::setupMeshShapeDistanceOrientedNode(node, model1, tf1, model2, tf2, nsolver, request, result);
}

} // namespace fcl

#endif

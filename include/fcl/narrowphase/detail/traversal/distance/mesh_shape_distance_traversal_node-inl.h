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

#ifndef FCL_TRAVERSAL_MESHSHAPEDISTANCETRAVERSALNODE_INL_H
#define FCL_TRAVERSAL_MESHSHAPEDISTANCETRAVERSALNODE_INL_H

#include "fcl/narrowphase/detail/traversal/distance/mesh_shape_distance_traversal_node.h"

#include "fcl/common/unused.h"

namespace fcl
{

namespace detail
{

//==============================================================================
template <typename BV, typename Shape, typename NarrowPhaseSolver>
MeshShapeDistanceTraversalNode<BV, Shape, NarrowPhaseSolver>::
MeshShapeDistanceTraversalNode()
  : BVHShapeDistanceTraversalNode<BV, Shape>()
{
  vertices = nullptr;
  tri_indices = nullptr;

  rel_err = 0;
  abs_err = 0;

  nsolver = nullptr;
}

//==============================================================================
template <typename BV, typename Shape, typename NarrowPhaseSolver>
void MeshShapeDistanceTraversalNode<BV, Shape, NarrowPhaseSolver>::
leafTesting(int b1, int b2) const
{
  FCL_UNUSED(b2);

  if(this->enable_statistics) this->num_leaf_tests++;

  const BVNode<BV>& node = this->model1->getBV(b1);

  int primitive_id = node.primitiveId();

  const Triangle& tri_id = tri_indices[primitive_id];

  const Vector3<S>& p1 = vertices[tri_id[0]];
  const Vector3<S>& p2 = vertices[tri_id[1]];
  const Vector3<S>& p3 = vertices[tri_id[2]];

  S d;
  Vector3<S> closest_p1, closest_p2;
  nsolver->shapeTriangleDistance(*(this->model2), this->tf2, p1, p2, p3, &d, &closest_p2, &closest_p1);

  this->result->update(
        d,
        this->model1,
        this->model2,
        primitive_id,
        DistanceResult<S>::NONE,
        closest_p1,
        closest_p2);
}

//==============================================================================
template <typename BV, typename Shape, typename NarrowPhaseSolver>
bool MeshShapeDistanceTraversalNode<BV, Shape, NarrowPhaseSolver>::
canStop(S c) const
{
  if((c >= this->result->min_distance - abs_err) && (c * (1 + rel_err) >= this->result->min_distance))
    return true;
  return false;
}

//==============================================================================
template <typename BV, typename Shape, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeDistanceTraversalNode<BV, Shape, NarrowPhaseSolver>& node,
    BVHModel<BV>& model1,
    Transform3<typename Shape::S>& tf1,
    const Shape& model2,
    const Transform3<typename BV::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename BV::S>& request,
    DistanceResult<typename BV::S>& result,
    bool use_refit,
    bool refit_bottomup)
{
  using S = typename BV::S;

  if(model1.getModelType() != BVH_MODEL_TRIANGLES)
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

//==============================================================================
template <typename BV, typename Shape, typename NarrowPhaseSolver>
void meshShapeDistanceOrientedNodeLeafTesting(
    int b1, int /* b2 */,
    const BVHModel<BV>* model1, const Shape& model2,
    Vector3<typename BV::S>* vertices, Triangle* tri_indices,
    const Transform3<typename BV::S>& tf1,
    const Transform3<typename BV::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    bool enable_statistics,
    int & num_leaf_tests,
    const DistanceRequest<typename BV::S>& /* request */,
    DistanceResult<typename BV::S>& result)
{
  using S = typename BV::S;

  if(enable_statistics) num_leaf_tests++;

  const BVNode<BV>& node = model1->getBV(b1);
  int primitive_id = node.primitiveId();

  const Triangle& tri_id = tri_indices[primitive_id];
  const Vector3<S>& p1 = vertices[tri_id[0]];
  const Vector3<S>& p2 = vertices[tri_id[1]];
  const Vector3<S>& p3 = vertices[tri_id[2]];

  S distance;
  Vector3<S> closest_p1, closest_p2;
  nsolver->shapeTriangleDistance(model2, tf2, p1, p2, p3, tf1, &distance, &closest_p2, &closest_p1);

  result.update(
        distance,
        model1,
        &model2,
        primitive_id,
        DistanceResult<S>::NONE,
        closest_p1,
        closest_p2);
}

//==============================================================================
template <typename BV, typename Shape, typename NarrowPhaseSolver>
void distancePreprocessOrientedNode(
    const BVHModel<BV>* model1,
    Vector3<typename BV::S>* vertices,
    Triangle* tri_indices,
    int init_tri_id,
    const Shape& model2,
    const Transform3<typename BV::S>& tf1,
    const Transform3<typename BV::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename BV::S>& /* request */,
    DistanceResult<typename BV::S>& result)
{
  using S = typename BV::S;

  const Triangle& init_tri = tri_indices[init_tri_id];

  const Vector3<S>& p1 = vertices[init_tri[0]];
  const Vector3<S>& p2 = vertices[init_tri[1]];
  const Vector3<S>& p3 = vertices[init_tri[2]];

  S distance;
  Vector3<S> closest_p1, closest_p2;
  nsolver->shapeTriangleDistance(model2, tf2, p1, p2, p3, tf1, &distance, &closest_p2, &closest_p1);

  result.update(
        distance,
        model1,
        &model2,
        init_tri_id,
        DistanceResult<S>::NONE,
        closest_p1,
        closest_p2);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
MeshShapeDistanceTraversalNodeRSS<Shape, NarrowPhaseSolver>::
MeshShapeDistanceTraversalNodeRSS()
  : MeshShapeDistanceTraversalNode<
    RSS<typename Shape::S>, Shape, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void MeshShapeDistanceTraversalNodeRSS<Shape, NarrowPhaseSolver>::preprocess()
{
  detail::distancePreprocessOrientedNode(
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
template <typename Shape, typename NarrowPhaseSolver>
void MeshShapeDistanceTraversalNodeRSS<Shape, NarrowPhaseSolver>::postprocess()
{
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
typename Shape::S
MeshShapeDistanceTraversalNodeRSS<Shape, NarrowPhaseSolver>::BVTesting(
    int b1, int b2) const
{
  FCL_UNUSED(b2);

  if(this->enable_statistics) this->num_bv_tests++;

  return distance(this->tf1.linear(), this->tf1.translation(), this->model2_bv, this->model1->getBV(b1).bv);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void MeshShapeDistanceTraversalNodeRSS<Shape, NarrowPhaseSolver>::
leafTesting(int b1, int b2) const
{
  detail::meshShapeDistanceOrientedNodeLeafTesting(
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
template <typename Shape, typename NarrowPhaseSolver>
MeshShapeDistanceTraversalNodekIOS<Shape, NarrowPhaseSolver>::MeshShapeDistanceTraversalNodekIOS() : MeshShapeDistanceTraversalNode<kIOS<S>, Shape, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void MeshShapeDistanceTraversalNodekIOS<Shape, NarrowPhaseSolver>::preprocess()
{
  detail::distancePreprocessOrientedNode(this->model1, this->vertices, this->tri_indices, 0,
                                          *(this->model2), this->tf1, this->tf2, this->nsolver, this->request, *(this->result));
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void MeshShapeDistanceTraversalNodekIOS<Shape, NarrowPhaseSolver>::postprocess()
{
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
typename Shape::S MeshShapeDistanceTraversalNodekIOS<Shape, NarrowPhaseSolver>::BVTesting(int b1, int b2) const
{
  FCL_UNUSED(b2);

  if(this->enable_statistics) this->num_bv_tests++;

  return distance(this->tf1.linear(), this->tf1.translation(), this->model2_bv, this->model1->getBV(b1).bv);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void MeshShapeDistanceTraversalNodekIOS<Shape, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  detail::meshShapeDistanceOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                    this->tf1, this->tf2, this->nsolver, this->enable_statistics, this->num_leaf_tests, this->request, *(this->result));
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
MeshShapeDistanceTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>::MeshShapeDistanceTraversalNodeOBBRSS() : MeshShapeDistanceTraversalNode<OBBRSS<S>, Shape, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void MeshShapeDistanceTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>::preprocess()
{
  detail::distancePreprocessOrientedNode(this->model1, this->vertices, this->tri_indices, 0,
                                          *(this->model2), this->tf1, this->tf2, this->nsolver, this->request, *(this->result));
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void MeshShapeDistanceTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>::postprocess()
{

}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
typename Shape::S
MeshShapeDistanceTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>::
BVTesting(int b1, int b2) const
{
  FCL_UNUSED(b2);

  if(this->enable_statistics) this->num_bv_tests++;

  return distance(this->tf1.linear(), this->tf1.translation(), this->model2_bv, this->model1->getBV(b1).bv);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void MeshShapeDistanceTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  detail::meshShapeDistanceOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                    this->tf1, this->tf2, this->nsolver, this->enable_statistics, this->num_leaf_tests, this->request, *(this->result));
}

template <typename BV, typename Shape, typename NarrowPhaseSolver, template <typename, typename> class OrientedNode>
static bool setupMeshShapeDistanceOrientedNode(OrientedNode<Shape, NarrowPhaseSolver>& node,
                                                      const BVHModel<BV>& model1, const Transform3<typename BV::S>& tf1,
                                                      const Shape& model2, const Transform3<typename BV::S>& tf2,
                                                      const NarrowPhaseSolver* nsolver,
                                                      const DistanceRequest<typename BV::S>& request,
                                                      DistanceResult<typename BV::S>& result)
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

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeDistanceTraversalNodeRSS<Shape, NarrowPhaseSolver>& node,
    const BVHModel<RSS<typename Shape::S>>& model1, const Transform3<typename Shape::S>& tf1,
    const Shape& model2, const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename Shape::S>& request,
    DistanceResult<typename Shape::S>& result)
{
  return detail::setupMeshShapeDistanceOrientedNode(node, model1, tf1, model2, tf2, nsolver, request, result);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeDistanceTraversalNodekIOS<Shape, NarrowPhaseSolver>& node,
    const BVHModel<kIOS<typename Shape::S>>& model1, const Transform3<typename Shape::S>& tf1,
    const Shape& model2, const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename Shape::S>& request,
    DistanceResult<typename Shape::S>& result)
{
  return detail::setupMeshShapeDistanceOrientedNode(node, model1, tf1, model2, tf2, nsolver, request, result);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeDistanceTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>& node,
    const BVHModel<OBBRSS<typename Shape::S>>& model1, const Transform3<typename Shape::S>& tf1,
    const Shape& model2, const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename Shape::S>& request,
    DistanceResult<typename Shape::S>& result)
{
  return detail::setupMeshShapeDistanceOrientedNode(node, model1, tf1, model2, tf2, nsolver, request, result);
}

} // namespace detail
} // namespace fcl

#endif

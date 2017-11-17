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

#ifndef FCL_TRAVERSAL_SHAPEMESHDISTANCETRAVERSALNODE_INL_H
#define FCL_TRAVERSAL_SHAPEMESHDISTANCETRAVERSALNODE_INL_H

#include "fcl/narrowphase/detail/traversal/distance/shape_mesh_distance_traversal_node.h"

#include "fcl/common/unused.h"

namespace fcl
{

namespace detail
{

//==============================================================================
template <typename Shape, typename BV, typename NarrowPhaseSolver>
ShapeMeshDistanceTraversalNode<Shape, BV, NarrowPhaseSolver>::
ShapeMeshDistanceTraversalNode()
  : ShapeBVHDistanceTraversalNode<Shape, BV>()
{
  vertices = nullptr;
  tri_indices = nullptr;

  rel_err = 0;
  abs_err = 0;

  nsolver = nullptr;
}

//==============================================================================
template <typename Shape, typename BV, typename NarrowPhaseSolver>
void ShapeMeshDistanceTraversalNode<Shape, BV, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  FCL_UNUSED(b1);

  using S = typename BV::S;

  if(this->enable_statistics) this->num_leaf_tests++;

  const BVNode<BV>& node = this->model2->getBV(b2);

  int primitive_id = node.primitiveId();

  const Triangle& tri_id = tri_indices[primitive_id];

  const Vector3<S>& p1 = vertices[tri_id[0]];
  const Vector3<S>& p2 = vertices[tri_id[1]];
  const Vector3<S>& p3 = vertices[tri_id[2]];

  S distance;
  Vector3<S> closest_p1, closest_p2;
  nsolver->shapeTriangleDistance(*(this->model1), this->tf1, p1, p2, p3, &distance, &closest_p1, &closest_p2);

  this->result->update(
        distance,
        this->model1,
        this->model2,
        DistanceResult<S>::NONE,
        primitive_id,
        closest_p1,
        closest_p2);
}

//==============================================================================
template <typename Shape, typename BV, typename NarrowPhaseSolver>
bool ShapeMeshDistanceTraversalNode<Shape, BV, NarrowPhaseSolver>::canStop(S c) const
{
  if((c >= this->result->min_distance - abs_err) && (c * (1 + rel_err) >= this->result->min_distance))
    return true;
  return false;
}

//==============================================================================
template <typename Shape, typename BV, typename NarrowPhaseSolver>
FCL_EXPORT
bool initialize(
    ShapeMeshDistanceTraversalNode<Shape, BV, NarrowPhaseSolver>& node,
    const Shape& model1,
    const Transform3<typename BV::S>& tf1,
    BVHModel<BV>& model2,
    Transform3<typename BV::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename BV::S>& request,
    DistanceResult<typename BV::S>& result,
    bool use_refit,
    bool refit_bottomup)
{
  using S = typename BV::S;

  if(model2.getModelType() != BVH_MODEL_TRIANGLES)
    return false;

  if(!tf2.matrix().isIdentity())
  {
    std::vector<Vector3<S>> vertices_transformed(model2.num_vertices);
    for(int i = 0; i < model2.num_vertices; ++i)
    {
      Vector3<S>& p = model2.vertices[i];
      Vector3<S> new_v = tf2 * p;
      vertices_transformed[i] = new_v;
    }

    model2.beginReplaceModel();
    model2.replaceSubModel(vertices_transformed);
    model2.endReplaceModel(use_refit, refit_bottomup);

    tf2.setIdentity();
  }

  node.request = request;
  node.result = &result;

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;
  node.nsolver = nsolver;

  node.vertices = model2.vertices;
  node.tri_indices = model2.tri_indices;

  computeBV(model1, tf1, node.model1_bv);

  return true;
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
ShapeMeshDistanceTraversalNodeRSS<Shape, NarrowPhaseSolver>::ShapeMeshDistanceTraversalNodeRSS()
  : ShapeMeshDistanceTraversalNode<Shape, RSS<S>, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void ShapeMeshDistanceTraversalNodeRSS<Shape, NarrowPhaseSolver>::preprocess()
{
  detail::distancePreprocessOrientedNode(
        this->model2, this->vertices, this->tri_indices, 0,
        *(this->model1), this->tf2, this->tf1, this->nsolver, this->request, *(this->result));
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void ShapeMeshDistanceTraversalNodeRSS<Shape, NarrowPhaseSolver>::postprocess()
{
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
typename Shape::S
ShapeMeshDistanceTraversalNodeRSS<Shape, NarrowPhaseSolver>::
BVTesting(int b1, int b2) const
{
  FCL_UNUSED(b1);

  if(this->enable_statistics) this->num_bv_tests++;
  return distance(this->tf2.linear(), this->tf2.translation(), this->model1_bv, this->model2->getBV(b2).bv);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void ShapeMeshDistanceTraversalNodeRSS<Shape, NarrowPhaseSolver>::
leafTesting(int b1, int b2) const
{
  detail::meshShapeDistanceOrientedNodeLeafTesting(b2, b1, this->model2, *(this->model1), this->vertices, this->tri_indices,
                                                    this->tf2, this->tf1, this->nsolver, this->enable_statistics, this->num_leaf_tests, this->request, *(this->result));
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
ShapeMeshDistanceTraversalNodekIOS<Shape, NarrowPhaseSolver>::
ShapeMeshDistanceTraversalNodekIOS()
  : ShapeMeshDistanceTraversalNode<Shape, kIOS<S>, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void ShapeMeshDistanceTraversalNodekIOS<Shape, NarrowPhaseSolver>::preprocess()
{
  detail::distancePreprocessOrientedNode(
        this->model2,
        this->vertices,
        this->tri_indices,
        0,
        *(this->model1),
        this->tf2,
        this->tf1,
        this->nsolver,
        *(this->result));
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void ShapeMeshDistanceTraversalNodekIOS<Shape, NarrowPhaseSolver>::postprocess()
{
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
typename Shape::S
ShapeMeshDistanceTraversalNodekIOS<Shape, NarrowPhaseSolver>::
BVTesting(int b1, int b2) const
{
  FCL_UNUSED(b1);

  if(this->enable_statistics) this->num_bv_tests++;
  return distance(this->tf2.linear(), this->tf2.translation(), this->model1_bv, this->model2->getBV(b2).bv);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void ShapeMeshDistanceTraversalNodekIOS<Shape, NarrowPhaseSolver>::
leafTesting(int b1, int b2) const
{
  detail::meshShapeDistanceOrientedNodeLeafTesting(b2, b1, this->model2, *(this->model1), this->vertices, this->tri_indices,
                                                    this->tf2, this->tf1, this->nsolver, this->enable_statistics, this->num_leaf_tests, this->request, *(this->result));
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
ShapeMeshDistanceTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>::
ShapeMeshDistanceTraversalNodeOBBRSS() : ShapeMeshDistanceTraversalNode<Shape, OBBRSS<S>, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void ShapeMeshDistanceTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>::preprocess()
{
  detail::distancePreprocessOrientedNode(
        this->model2,
        this->vertices,
        this->tri_indices,
        0,
        *(this->model1),
        this->tf2,
        this->tf1,
        this->nsolver,
        *(this->result));
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void ShapeMeshDistanceTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>::
postprocess()
{
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
typename Shape::S
ShapeMeshDistanceTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>::
BVTesting(int b1, int b2) const
{
  FCL_UNUSED(b1);

  if(this->enable_statistics) this->num_bv_tests++;
  return distance(this->tf2.linear(), this->tf2.translation(), this->model1_bv, this->model2->getBV(b2).bv);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void ShapeMeshDistanceTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>::
leafTesting(int b1, int b2) const
{
  detail::meshShapeDistanceOrientedNodeLeafTesting(
        b2,
        b1,
        this->model2,
        *(this->model1),
        this->vertices,
        this->tri_indices,
        this->tf2,
        this->tf1,
        this->nsolver,
        this->enable_statistics,
        this->num_leaf_tests,
        this->request,
        *(this->result));
}

template <typename Shape, typename BV, typename NarrowPhaseSolver, template <typename, typename> class OrientedNode>
static bool setupShapeMeshDistanceOrientedNode(OrientedNode<Shape, NarrowPhaseSolver>& node,
                                                      const Shape& model1, const Transform3<typename BV::S>& tf1,
                                                      const BVHModel<BV>& model2, const Transform3<typename BV::S>& tf2,
                                                      const NarrowPhaseSolver* nsolver,
                                                      const DistanceRequest<typename BV::S>& request,
                                                      DistanceResult<typename BV::S>& result)
{
  if(model2.getModelType() != BVH_MODEL_TRIANGLES)
    return false;

  node.request = request;
  node.result = &result;

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;
  node.nsolver = nsolver;

  computeBV(model1, tf1, node.model1_bv);

  node.vertices = model2.vertices;
  node.tri_indices = model2.tri_indices;
  node.R = tf2.linear();
  node.T = tf2.translation();

  return true;
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
bool initialize(ShapeMeshDistanceTraversalNodeRSS<Shape, NarrowPhaseSolver>& node,
                const Shape& model1, const Transform3<typename Shape::S>& tf1,
                const BVHModel<RSS<typename Shape::S>>& model2, const Transform3<typename Shape::S>& tf2,
                const NarrowPhaseSolver* nsolver,
                const DistanceRequest<typename Shape::S>& request,
                DistanceResult<typename Shape::S>& result)
{
  return detail::setupShapeMeshDistanceOrientedNode(node, model1, tf1, model2, tf2, nsolver, request, result);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
bool initialize(ShapeMeshDistanceTraversalNodekIOS<Shape, NarrowPhaseSolver>& node,
                const Shape& model1, const Transform3<typename Shape::S>& tf1,
                const BVHModel<kIOS<typename Shape::S>>& model2, const Transform3<typename Shape::S>& tf2,
                const NarrowPhaseSolver* nsolver,
                const DistanceRequest<typename Shape::S>& request,
                DistanceResult<typename Shape::S>& result)
{
  return detail::setupShapeMeshDistanceOrientedNode(node, model1, tf1, model2, tf2, nsolver, request, result);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
bool initialize(ShapeMeshDistanceTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>& node,
                const Shape& model1, const Transform3<typename Shape::S>& tf1,
                const BVHModel<OBBRSS<typename Shape::S>>& model2, const Transform3<typename Shape::S>& tf2,
                const NarrowPhaseSolver* nsolver,
                const DistanceRequest<typename Shape::S>& request,
                DistanceResult<typename Shape::S>& result)
{
  return detail::setupShapeMeshDistanceOrientedNode(node, model1, tf1, model2, tf2, nsolver, request, result);
}

} // namespace detail
} // namespace fcl

#endif

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

#ifndef FCL_TRAVERSAL_SHAPEMESHCOLLISIONTRAVERSALNODE_INL_H
#define FCL_TRAVERSAL_SHAPEMESHCOLLISIONTRAVERSALNODE_INL_H

#include "fcl/narrowphase/detail/traversal/collision/shape_mesh_collision_traversal_node.h"

#include "fcl/common/unused.h"

namespace fcl
{

namespace detail
{

//==============================================================================
template <typename Shape, typename BV, typename NarrowPhaseSolver>
FCL_EXPORT
ShapeMeshCollisionTraversalNode<Shape, BV, NarrowPhaseSolver>::ShapeMeshCollisionTraversalNode()
  : ShapeBVHCollisionTraversalNode<Shape, BV>()
{
  vertices = nullptr;
  tri_indices = nullptr;

  nsolver = nullptr;
}

//==============================================================================
template <typename Shape, typename BV, typename NarrowPhaseSolver>
FCL_EXPORT
void ShapeMeshCollisionTraversalNode<Shape, BV, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
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

  if(this->model1->isOccupied() && this->model2->isOccupied())
  {
    bool is_intersect = false;

    if(!this->request.enable_contact)
    {
      if(nsolver->shapeTriangleIntersect(*(this->model1), this->tf1, p1, p2, p3, nullptr, nullptr, nullptr))
      {
        is_intersect = true;
        if(this->request.num_max_contacts > this->result->numContacts())
          this->result->addContact(Contact<S>(this->model1, this->model2, Contact<S>::NONE, primitive_id));
      }
    }
    else
    {
      S penetration;
      Vector3<S> normal;
      Vector3<S> contactp;

      if(nsolver->shapeTriangleIntersect(*(this->model1), this->tf1, p1, p2, p3, &contactp, &penetration, &normal))
      {
        is_intersect = true;
        if(this->request.num_max_contacts > this->result->numContacts())
          this->result->addContact(Contact<S>(this->model1, this->model2, Contact<S>::NONE, primitive_id, contactp, normal, penetration));
      }
    }

    if(is_intersect && this->request.enable_cost)
    {
      AABB<S> overlap_part;
      AABB<S> shape_aabb;
      computeBV(*(this->model1), this->tf1, shape_aabb);
      AABB<S>(p1, p2, p3).overlap(shape_aabb, overlap_part);
      this->result->addCostSource(CostSource<S>(overlap_part, cost_density), this->request.num_max_cost_sources);
    }
  }
  else if((!this->model1->isFree() && !this->model2->isFree()) && this->request.enable_cost)
  {
    if(nsolver->shapeTriangleIntersect(*(this->model1), this->tf1, p1, p2, p3, nullptr, nullptr, nullptr))
    {
      AABB<S> overlap_part;
      AABB<S> shape_aabb;
      computeBV(*(this->model1), this->tf1, shape_aabb);
      AABB<S>(p1, p2, p3).overlap(shape_aabb, overlap_part);
      this->result->addCostSource(CostSource<S>(overlap_part, cost_density), this->request.num_max_cost_sources);
    }
  }
}

//==============================================================================
template <typename Shape, typename BV, typename NarrowPhaseSolver>
FCL_EXPORT
bool ShapeMeshCollisionTraversalNode<Shape, BV, NarrowPhaseSolver>::canStop() const
{
  return this->request.isSatisfied(*(this->result));
}

//==============================================================================
template <typename Shape, typename BV, typename NarrowPhaseSolver>
FCL_EXPORT
bool initialize(
    ShapeMeshCollisionTraversalNode<Shape, BV, NarrowPhaseSolver>& node,
    const Shape& model1,
    const Transform3<typename BV::S>& tf1,
    BVHModel<BV>& model2,
    Transform3<typename BV::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename BV::S>& request,
    CollisionResult<typename BV::S>& result,
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

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;
  node.nsolver = nsolver;

  computeBV(model1, tf1, node.model1_bv);

  node.vertices = model2.vertices;
  node.tri_indices = model2.tri_indices;

  node.request = request;
  node.result = &result;

  node.cost_density = model1.cost_density * model2.cost_density;

  return true;
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
ShapeMeshCollisionTraversalNodeOBB<Shape, NarrowPhaseSolver>::ShapeMeshCollisionTraversalNodeOBB() : ShapeMeshCollisionTraversalNode<Shape, OBB<typename Shape::S>, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
bool ShapeMeshCollisionTraversalNodeOBB<Shape, NarrowPhaseSolver>::BVTesting(int b1, int b2) const
{
  FCL_UNUSED(b1);

  if(this->enable_statistics) this->num_bv_tests++;
  return !overlap(this->tf2.linear(), this->tf2.translation(), this->model1_bv, this->model2->getBV(b2).bv);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
void ShapeMeshCollisionTraversalNodeOBB<Shape, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  detail::meshShapeCollisionOrientedNodeLeafTesting(b2, b1, *(this->model2), this->model1, this->vertices, this->tri_indices,
                                                     this->tf2, this->tf1, this->nsolver, this->enable_statistics, this->cost_density, this->num_leaf_tests, this->request, *(this->request));

  // may need to change the order in pairs
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
ShapeMeshCollisionTraversalNodeRSS<Shape, NarrowPhaseSolver>::ShapeMeshCollisionTraversalNodeRSS() : ShapeMeshCollisionTraversalNode<Shape, RSS<typename Shape::S>, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
bool ShapeMeshCollisionTraversalNodeRSS<Shape, NarrowPhaseSolver>::BVTesting(int b1, int b2) const
{
  FCL_UNUSED(b1);

  if(this->enable_statistics) this->num_bv_tests++;
  return !overlap(this->tf2.linear(), this->tf2.translation(), this->model1_bv, this->model2->getBV(b2).bv);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
void ShapeMeshCollisionTraversalNodeRSS<Shape, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  detail::meshShapeCollisionOrientedNodeLeafTesting(b2, b1, *(this->model2), this->model1, this->vertices, this->tri_indices,
                                                     this->tf2, this->tf1, this->nsolver, this->enable_statistics, this->cost_density, this->num_leaf_tests, this->request, *(this->request));

  // may need to change the order in pairs
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
ShapeMeshCollisionTraversalNodekIOS<Shape, NarrowPhaseSolver>::ShapeMeshCollisionTraversalNodekIOS() : ShapeMeshCollisionTraversalNode<Shape, kIOS<typename Shape::S>, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
bool ShapeMeshCollisionTraversalNodekIOS<Shape, NarrowPhaseSolver>::BVTesting(int b1, int b2) const
{
  FCL_UNUSED(b1);

  if(this->enable_statistics) this->num_bv_tests++;
  return !overlap(this->tf2.linear(), this->tf2.translation(), this->model1_bv, this->model2->getBV(b2).bv);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
void ShapeMeshCollisionTraversalNodekIOS<Shape, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  detail::meshShapeCollisionOrientedNodeLeafTesting(b2, b1, *(this->model2), this->model1, this->vertices, this->tri_indices,
                                                     this->tf2, this->tf1, this->nsolver, this->enable_statistics, this->cost_density, this->num_leaf_tests, this->request, *(this->request));

  // may need to change the order in pairs
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
ShapeMeshCollisionTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>::ShapeMeshCollisionTraversalNodeOBBRSS() : ShapeMeshCollisionTraversalNode<Shape, OBBRSS<typename Shape::S>, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
bool ShapeMeshCollisionTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>::BVTesting(int b1, int b2) const
{
  FCL_UNUSED(b1);

  if(this->enable_statistics) this->num_bv_tests++;
  return !overlap(this->tf2.linear(), this->tf2.translation(), this->model1_bv, this->model2->getBV(b2).bv);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
void ShapeMeshCollisionTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  detail::meshShapeCollisionOrientedNodeLeafTesting(b2, b1, *(this->model2), this->model1, this->vertices, this->tri_indices,
                                                     this->tf2, this->tf1, this->nsolver, this->enable_statistics, this->cost_density, this->num_leaf_tests, this->request, *(this->request));

  // may need to change the order in pairs
}

template <typename Shape, typename BV, typename NarrowPhaseSolver, template <typename, typename> class OrientedNode>
static bool setupShapeMeshCollisionOrientedNode(OrientedNode<Shape, NarrowPhaseSolver>& node,
                                                       const Shape& model1, const Transform3<typename BV::S>& tf1,
                                                       const BVHModel<BV>& model2, const Transform3<typename BV::S>& tf2,
                                                       const NarrowPhaseSolver* nsolver,
                                                       const CollisionRequest<typename BV::S>& request,
                                                       CollisionResult<typename BV::S>& result)
{
  if(model2.getModelType() != BVH_MODEL_TRIANGLES)
    return false;

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;
  node.nsolver = nsolver;

  computeBV(model1, tf1, node.model1_bv);

  node.vertices = model2.vertices;
  node.tri_indices = model2.tri_indices;

  node.request = request;
  node.result = &result;

  node.cost_density = model1.cost_density * model2.cost_density;

  return true;
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
bool initialize(
    ShapeMeshCollisionTraversalNodeOBB<Shape, NarrowPhaseSolver>& node,
    const Shape& model1,
    const Transform3<typename Shape::S>& tf1,
    const BVHModel<OBB<typename Shape::S>>& model2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename Shape::S>& request,
    CollisionResult<typename Shape::S>& result)
{
  return detail::setupShapeMeshCollisionOrientedNode(
        node, model1, tf1, model2, tf2, nsolver, request, result);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
bool initialize(
    ShapeMeshCollisionTraversalNodeRSS<Shape, NarrowPhaseSolver>& node,
    const Shape& model1,
    const Transform3<typename Shape::S>& tf1,
    const BVHModel<RSS<typename Shape::S>>& model2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename Shape::S>& request,
    CollisionResult<typename Shape::S>& result)
{
  return detail::setupShapeMeshCollisionOrientedNode(
        node, model1, tf1, model2, tf2, nsolver, request, result);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
bool initialize(
    ShapeMeshCollisionTraversalNodekIOS<Shape, NarrowPhaseSolver>& node,
    const Shape& model1,
    const Transform3<typename Shape::S>& tf1,
    const BVHModel<kIOS<typename Shape::S>>& model2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename Shape::S>& request,
    CollisionResult<typename Shape::S>& result)
{
  return detail::setupShapeMeshCollisionOrientedNode(
        node, model1, tf1, model2, tf2, nsolver, request, result);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
FCL_EXPORT
bool initialize(
    ShapeMeshCollisionTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>& node,
    const Shape& model1,
    const Transform3<typename Shape::S>& tf1,
    const BVHModel<OBBRSS<typename Shape::S>>& model2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename Shape::S>& request,
    CollisionResult<typename Shape::S>& result)
{
  return detail::setupShapeMeshCollisionOrientedNode(
        node, model1, tf1, model2, tf2, nsolver, request, result);
}

} // namespace detail
} // namespace fcl

#endif

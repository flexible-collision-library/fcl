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

#ifndef FCL_TRAVERSAL_MESHSHAPECOLLISIONTRAVERSALNODE_INL_H
#define FCL_TRAVERSAL_MESHSHAPECOLLISIONTRAVERSALNODE_INL_H

#include "fcl/narrowphase/detail/traversal/collision/mesh_shape_collision_traversal_node.h"

#include "fcl/common/unused.h"

namespace fcl
{

namespace detail
{

//==============================================================================
template <typename BV, typename Shape, typename NarrowPhaseSolver>
MeshShapeCollisionTraversalNode<BV, Shape, NarrowPhaseSolver>::MeshShapeCollisionTraversalNode()
  : BVHShapeCollisionTraversalNode<BV, Shape>()
{
  vertices = nullptr;
  tri_indices = nullptr;

  nsolver = nullptr;
}

//==============================================================================
template <typename BV, typename Shape, typename NarrowPhaseSolver>
void MeshShapeCollisionTraversalNode<BV, Shape, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  FCL_UNUSED(b2);

  if(this->enable_statistics) this->num_leaf_tests++;
  const BVNode<BV>& node = this->model1->getBV(b1);

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
      if(nsolver->shapeTriangleIntersect(*(this->model2), this->tf2, p1, p2, p3, nullptr, nullptr, nullptr))
      {
        is_intersect = true;
        if(this->request.num_max_contacts > this->result->numContacts())
          this->result->addContact(Contact<S>(this->model1, this->model2, primitive_id, Contact<S>::NONE));
      }
    }
    else
    {
      S penetration;
      Vector3<S> normal;
      Vector3<S> contactp;

      if(nsolver->shapeTriangleIntersect(*(this->model2), this->tf2, p1, p2, p3, &contactp, &penetration, &normal))
      {
        is_intersect = true;
        if(this->request.num_max_contacts > this->result->numContacts())
          this->result->addContact(Contact<S>(this->model1, this->model2, primitive_id, Contact<S>::NONE, contactp, -normal, penetration));
      }
    }

    if(is_intersect && this->request.enable_cost)
    {
      AABB<S> overlap_part;
      AABB<S> shape_aabb;
      computeBV(*(this->model2), this->tf2, shape_aabb);
      AABB<S>(p1, p2, p3).overlap(shape_aabb, overlap_part);
      this->result->addCostSource(CostSource<S>(overlap_part, cost_density), this->request.num_max_cost_sources);
    }
  }
  if((!this->model1->isFree() && !this->model2->isFree()) && this->request.enable_cost)
  {
    if(nsolver->shapeTriangleIntersect(*(this->model2), this->tf2, p1, p2, p3, nullptr, nullptr, nullptr))
    {
      AABB<S> overlap_part;
      AABB<S> shape_aabb;
      computeBV(*(this->model2), this->tf2, shape_aabb);
      AABB<S>(p1, p2, p3).overlap(shape_aabb, overlap_part);
      this->result->addCostSource(CostSource<S>(overlap_part, cost_density), this->request.num_max_cost_sources);
    }
  }
}

//==============================================================================
template <typename BV, typename Shape, typename NarrowPhaseSolver>
bool MeshShapeCollisionTraversalNode<BV, Shape, NarrowPhaseSolver>::canStop() const
{
  return this->request.isSatisfied(*(this->result));
}

//==============================================================================
template <typename BV, typename Shape, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeCollisionTraversalNode<BV, Shape, NarrowPhaseSolver>& node,
    BVHModel<BV>& model1,
    Transform3<typename BV::S>& tf1,
    const Shape& model2,
    const Transform3<typename BV::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename BV::S>& request,
    CollisionResult<typename BV::S>& result,
    bool use_refit,
    bool refit_bottomup)
{
  using S = typename BV::S;

  if(model1.getModelType() != BVH_MODEL_TRIANGLES)
    return false;

  if(!tf1.matrix().isIdentity())
  {
    std::vector<Vector3<S>> vertices_transformed(model1.num_vertices);
    for(int i = 0; i < model1.num_vertices; ++i)
    {
      Vector3<S>& p = model1.vertices[i];
      Vector3<S> new_v = tf1 * p;
      vertices_transformed[i] = new_v;
    }

    model1.beginReplaceModel();
    model1.replaceSubModel(vertices_transformed);
    model1.endReplaceModel(use_refit, refit_bottomup);

    tf1.setIdentity();
  }

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;
  node.nsolver = nsolver;

  computeBV(model2, tf2, node.model2_bv);

  node.vertices = model1.vertices;
  node.tri_indices = model1.tri_indices;

  node.request = request;
  node.result = &result;

  node.cost_density = model1.cost_density * model2.cost_density;

  return true;
}

//==============================================================================
template <typename BV, typename Shape, typename NarrowPhaseSolver>
void meshShapeCollisionOrientedNodeLeafTesting(
    int b1, int b2,
    const BVHModel<BV>* model1, const Shape& model2,
    Vector3<typename BV::S>* vertices, Triangle* tri_indices,
    const Transform3<typename BV::S>& tf1,
    const Transform3<typename BV::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    bool enable_statistics,
    typename BV::S cost_density,
    int& num_leaf_tests,
    const CollisionRequest<typename BV::S>& request,
    CollisionResult<typename BV::S>& result)
{
  FCL_UNUSED(b2);

  using S = typename BV::S;

  if(enable_statistics) num_leaf_tests++;
  const BVNode<BV>& node = model1->getBV(b1);

  int primitive_id = node.primitiveId();

  const Triangle& tri_id = tri_indices[primitive_id];

  const Vector3<S>& p1 = vertices[tri_id[0]];
  const Vector3<S>& p2 = vertices[tri_id[1]];
  const Vector3<S>& p3 = vertices[tri_id[2]];

  if(model1->isOccupied() && model2.isOccupied())
  {
    bool is_intersect = false;

    if(!request.enable_contact) // only interested in collision or not
    {
      if(nsolver->shapeTriangleIntersect(model2, tf2, p1, p2, p3, tf1, nullptr, nullptr, nullptr))
      {
        is_intersect = true;
        if(request.num_max_contacts > result.numContacts())
          result.addContact(Contact<S>(model1, &model2, primitive_id, Contact<S>::NONE));
      }
    }
    else
    {
      S penetration;
      Vector3<S> normal;
      Vector3<S> contactp;

      if(nsolver->shapeTriangleIntersect(model2, tf2, p1, p2, p3, tf1, &contactp, &penetration, &normal))
      {
        is_intersect = true;
        if(request.num_max_contacts > result.numContacts())
          result.addContact(Contact<S>(model1, &model2, primitive_id, Contact<S>::NONE, contactp, -normal, penetration));
      }
    }

    if(is_intersect && request.enable_cost)
    {
      AABB<S> overlap_part;
      AABB<S> shape_aabb;
      computeBV(model2, tf2, shape_aabb);
      /* bool res = */ AABB<S>(tf1 * p1, tf1 * p2, tf1 * p3).overlap(shape_aabb, overlap_part);
      result.addCostSource(CostSource<S>(overlap_part, cost_density), request.num_max_cost_sources);
    }
  }
  else if((!model1->isFree() || model2.isFree()) && request.enable_cost)
  {
    if(nsolver->shapeTriangleIntersect(model2, tf2, p1, p2, p3, tf1, nullptr, nullptr, nullptr))
    {
      AABB<S> overlap_part;
      AABB<S> shape_aabb;
      computeBV(model2, tf2, shape_aabb);
      /* bool res = */ AABB<S>(tf1 * p1, tf1 * p2, tf1 * p3).overlap(shape_aabb, overlap_part);
      result.addCostSource(CostSource<S>(overlap_part, cost_density), request.num_max_cost_sources);
    }
  }
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
MeshShapeCollisionTraversalNodeOBB<Shape, NarrowPhaseSolver>::
MeshShapeCollisionTraversalNodeOBB()
  : MeshShapeCollisionTraversalNode<OBB<typename Shape::S>, Shape, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
bool MeshShapeCollisionTraversalNodeOBB<Shape, NarrowPhaseSolver>::BVTesting(int b1, int b2) const
{
  FCL_UNUSED(b2);

  if(this->enable_statistics) this->num_bv_tests++;

  return !overlap(this->tf1.linear(), this->tf1.translation(), this->model2_bv, this->model1->getBV(b1).bv);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void MeshShapeCollisionTraversalNodeOBB<Shape, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  detail::meshShapeCollisionOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                     this->tf1, this->tf2, this->nsolver, this->enable_statistics, this->cost_density, this->num_leaf_tests, this->request, *(this->result));
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
MeshShapeCollisionTraversalNodeRSS<Shape, NarrowPhaseSolver>::MeshShapeCollisionTraversalNodeRSS()
  : MeshShapeCollisionTraversalNode<RSS<typename Shape::S>, Shape, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
bool MeshShapeCollisionTraversalNodeRSS<Shape, NarrowPhaseSolver>::BVTesting(int b1, int b2) const
{
  FCL_UNUSED(b2);

  if(this->enable_statistics) this->num_bv_tests++;

  return !overlap(this->tf1.linear(), this->tf1.translation(), this->model2_bv, this->model1->getBV(b1).bv);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void MeshShapeCollisionTraversalNodeRSS<Shape, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  detail::meshShapeCollisionOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                     this->tf1, this->tf2, this->nsolver, this->enable_statistics, this->cost_density, this->num_leaf_tests, this->request, *(this->result));
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
MeshShapeCollisionTraversalNodekIOS<Shape, NarrowPhaseSolver>::
MeshShapeCollisionTraversalNodekIOS()
  : MeshShapeCollisionTraversalNode<kIOS<typename Shape::S>, Shape, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
bool MeshShapeCollisionTraversalNodekIOS<Shape, NarrowPhaseSolver>::BVTesting(int b1, int b2) const
{
  FCL_UNUSED(b2);

  if(this->enable_statistics) this->num_bv_tests++;

  return !overlap(this->tf1.linear(), this->tf1.translation(), this->model2_bv, this->model1->getBV(b1).bv);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void MeshShapeCollisionTraversalNodekIOS<Shape, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  detail::meshShapeCollisionOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                     this->tf1, this->tf2, this->nsolver, this->enable_statistics, this->cost_density, this->num_leaf_tests, this->request, *(this->result));
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
MeshShapeCollisionTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>::
MeshShapeCollisionTraversalNodeOBBRSS()
  : MeshShapeCollisionTraversalNode<OBBRSS<typename Shape::S>, Shape, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
bool MeshShapeCollisionTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>::BVTesting(int b1, int b2) const
{
  FCL_UNUSED(b2);

  if(this->enable_statistics) this->num_bv_tests++;
  return !overlap(this->tf1.linear(), this->tf1.translation(), this->model2_bv, this->model1->getBV(b1).bv);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
void MeshShapeCollisionTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  detail::meshShapeCollisionOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                     this->tf1, this->tf2, this->nsolver, this->enable_statistics, this->cost_density, this->num_leaf_tests, this->request, *(this->result));
}

template <typename BV, typename Shape, typename NarrowPhaseSolver,
          template <typename, typename> class OrientedNode>
bool setupMeshShapeCollisionOrientedNode(
    OrientedNode<Shape, NarrowPhaseSolver>& node,
    const BVHModel<BV>& model1,
    const Transform3<typename BV::S>& tf1,
    const Shape& model2, const Transform3<typename BV::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename BV::S>& request,
    CollisionResult<typename BV::S>& result)
{
  if(model1.getModelType() != BVH_MODEL_TRIANGLES)
    return false;

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;
  node.nsolver = nsolver;

  computeBV(model2, tf2, node.model2_bv);

  node.vertices = model1.vertices;
  node.tri_indices = model1.tri_indices;

  node.request = request;
  node.result = &result;

  node.cost_density = model1.cost_density * model2.cost_density;

  return true;
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeCollisionTraversalNodeOBB<Shape, NarrowPhaseSolver>& node,
    const BVHModel<OBB<typename Shape::S>>& model1,
    const Transform3<typename Shape::S>& tf1,
    const Shape& model2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename Shape::S>& request,
    CollisionResult<typename Shape::S>& result)
{
  return detail::setupMeshShapeCollisionOrientedNode(
        node, model1, tf1, model2, tf2, nsolver, request, result);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeCollisionTraversalNodeRSS<Shape, NarrowPhaseSolver>& node,
    const BVHModel<RSS<typename Shape::S>>& model1,
    const Transform3<typename Shape::S>& tf1,
    const Shape& model2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename Shape::S>& request,
    CollisionResult<typename Shape::S>& result)
{
  return detail::setupMeshShapeCollisionOrientedNode(
        node, model1, tf1, model2, tf2, nsolver, request, result);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeCollisionTraversalNodekIOS<Shape, NarrowPhaseSolver>& node,
    const BVHModel<kIOS<typename Shape::S>>& model1,
    const Transform3<typename Shape::S>& tf1,
    const Shape& model2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename Shape::S>& request,
    CollisionResult<typename Shape::S>& result)
{
  return detail::setupMeshShapeCollisionOrientedNode(
        node, model1, tf1, model2, tf2, nsolver, request, result);
}

//==============================================================================
template <typename Shape, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeCollisionTraversalNodeOBBRSS<Shape, NarrowPhaseSolver>& node,
    const BVHModel<OBBRSS<typename Shape::S>>& model1,
    const Transform3<typename Shape::S>& tf1,
    const Shape& model2,
    const Transform3<typename Shape::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename Shape::S>& request,
    CollisionResult<typename Shape::S>& result)
{
  return detail::setupMeshShapeCollisionOrientedNode(
        node, model1, tf1, model2, tf2, nsolver, request, result);
}

} // namespace detail
} // namespace fcl

#endif

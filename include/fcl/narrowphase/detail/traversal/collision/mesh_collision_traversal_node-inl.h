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

#ifndef FCL_TRAVERSAL_MESHCOLLISIONTRAVERSALNODE_INL_H
#define FCL_TRAVERSAL_MESHCOLLISIONTRAVERSALNODE_INL_H

#include "fcl/narrowphase/detail/traversal/collision/mesh_collision_traversal_node.h"

#include "fcl/common/unused.h"

#include "fcl/narrowphase/collision_result.h"

namespace fcl
{

namespace detail
{

//==============================================================================
extern template
class FCL_EXPORT MeshCollisionTraversalNodeOBB<double>;

//==============================================================================
extern template
bool initialize(
    MeshCollisionTraversalNodeOBB<double>& node,
    const BVHModel<OBB<double>>& model1,
    const Transform3<double>& tf1,
    const BVHModel<OBB<double>>& model2,
    const Transform3<double>& tf2,
    const CollisionRequest<double>& request,
    CollisionResult<double>& result);

//==============================================================================
extern template
class FCL_EXPORT MeshCollisionTraversalNodeRSS<double>;

//==============================================================================
extern template
bool initialize(
    MeshCollisionTraversalNodeRSS<double>& node,
    const BVHModel<RSS<double>>& model1,
    const Transform3<double>& tf1,
    const BVHModel<RSS<double>>& model2,
    const Transform3<double>& tf2,
    const CollisionRequest<double>& request,
    CollisionResult<double>& result);

//==============================================================================
extern template
class FCL_EXPORT MeshCollisionTraversalNodekIOS<double>;

//==============================================================================
extern template
bool initialize(
    MeshCollisionTraversalNodekIOS<double>& node,
    const BVHModel<kIOS<double>>& model1,
    const Transform3<double>& tf1,
    const BVHModel<kIOS<double>>& model2,
    const Transform3<double>& tf2,
    const CollisionRequest<double>& request,
    CollisionResult<double>& result);

//==============================================================================
extern template
class FCL_EXPORT MeshCollisionTraversalNodeOBBRSS<double>;

//==============================================================================
extern template
bool initialize(
    MeshCollisionTraversalNodeOBBRSS<double>& node,
    const BVHModel<OBBRSS<double>>& model1,
    const Transform3<double>& tf1,
    const BVHModel<OBBRSS<double>>& model2,
    const Transform3<double>& tf2,
    const CollisionRequest<double>& request,
    CollisionResult<double>& result);

//==============================================================================
template <typename BV>
MeshCollisionTraversalNode<BV>::MeshCollisionTraversalNode()
  : BVHCollisionTraversalNode<BV>()
{
  vertices1 = nullptr;
  vertices2 = nullptr;
  tri_indices1 = nullptr;
  tri_indices2 = nullptr;
}

//==============================================================================
template <typename BV>
void MeshCollisionTraversalNode<BV>::leafTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_leaf_tests++;

  const BVNode<BV>& node1 = this->model1->getBV(b1);
  const BVNode<BV>& node2 = this->model2->getBV(b2);

  int primitive_id1 = node1.primitiveId();
  int primitive_id2 = node2.primitiveId();

  const Triangle& tri_id1 = tri_indices1[primitive_id1];
  const Triangle& tri_id2 = tri_indices2[primitive_id2];

  const Vector3<S>& p1 = vertices1[tri_id1[0]];
  const Vector3<S>& p2 = vertices1[tri_id1[1]];
  const Vector3<S>& p3 = vertices1[tri_id1[2]];
  const Vector3<S>& q1 = vertices2[tri_id2[0]];
  const Vector3<S>& q2 = vertices2[tri_id2[1]];
  const Vector3<S>& q3 = vertices2[tri_id2[2]];

  if(this->model1->isOccupied() && this->model2->isOccupied())
  {
    bool is_intersect = false;

    if(!this->request.enable_contact) // only interested in collision or not
    {
      if(Intersect<S>::intersect_Triangle(p1, p2, p3, q1, q2, q3))
      {
        is_intersect = true;
        if(this->result->numContacts() < this->request.num_max_contacts)
          this->result->addContact(Contact<S>(this->model1, this->model2, primitive_id1, primitive_id2));
      }
    }
    else // need compute the contact information
    {
      S penetration;
      Vector3<S> normal;
      unsigned int n_contacts;
      Vector3<S> contacts[2];

      if(Intersect<S>::intersect_Triangle(p1, p2, p3, q1, q2, q3,
                                       contacts,
                                       &n_contacts,
                                       &penetration,
                                       &normal))
      {
        is_intersect = true;

        if(this->request.num_max_contacts < n_contacts + this->result->numContacts())
          n_contacts = (this->request.num_max_contacts >= this->result->numContacts()) ? (this->request.num_max_contacts - this->result->numContacts()) : 0;

        for(unsigned int i = 0; i < n_contacts; ++i)
        {
          this->result->addContact(Contact<S>(this->model1, this->model2, primitive_id1, primitive_id2, contacts[i], normal, penetration));
        }
      }
    }

    if(is_intersect && this->request.enable_cost)
    {
      AABB<S> overlap_part;
      AABB<S>(p1, p2, p3).overlap(AABB<S>(q1, q2, q3), overlap_part);
      this->result->addCostSource(CostSource<S>(overlap_part, cost_density), this->request.num_max_cost_sources);
    }
  }
  else if((!this->model1->isFree() && !this->model2->isFree()) && this->request.enable_cost)
  {
    if(Intersect<S>::intersect_Triangle(p1, p2, p3, q1, q2, q3))
    {
      AABB<S> overlap_part;
      AABB<S>(p1, p2, p3).overlap(AABB<S>(q1, q2, q3), overlap_part);
      this->result->addCostSource(CostSource<S>(overlap_part, cost_density), this->request.num_max_cost_sources);
    }
  }
}

//==============================================================================
template <typename BV>
bool MeshCollisionTraversalNode<BV>::canStop() const
{
  return this->request.isSatisfied(*(this->result));
}

//==============================================================================
template <typename BV>
bool initialize(
    MeshCollisionTraversalNode<BV>& node,
    BVHModel<BV>& model1,
    Transform3<typename BV::S>& tf1,
    BVHModel<BV>& model2,
    Transform3<typename BV::S>& tf2,
    const CollisionRequest<typename BV::S>& request,
    CollisionResult<typename BV::S>& result,
    bool use_refit,
    bool refit_bottomup)
{
  using S = typename BV::S;

  if(model1.getModelType() != BVH_MODEL_TRIANGLES
     || model2.getModelType() != BVH_MODEL_TRIANGLES)
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

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;

  node.vertices1 = model1.vertices;
  node.vertices2 = model2.vertices;

  node.tri_indices1 = model1.tri_indices;
  node.tri_indices2 = model2.tri_indices;

  node.request = request;
  node.result = &result;

  node.cost_density = model1.cost_density * model2.cost_density;

  return true;
}

//==============================================================================
template <typename S>
MeshCollisionTraversalNodeOBB<S>::MeshCollisionTraversalNodeOBB()
  : MeshCollisionTraversalNode<OBB<S>>(),
    R(Matrix3<S>::Identity())
{
  // Do nothing
}

//==============================================================================
template <typename S>
bool MeshCollisionTraversalNodeOBB<S>::BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;

  return !overlap(R, T, this->model1->getBV(b1).bv, this->model2->getBV(b2).bv);
}

//==============================================================================
template <typename S>
void MeshCollisionTraversalNodeOBB<S>::leafTesting(int b1, int b2) const
{
  detail::meshCollisionOrientedNodeLeafTesting(
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
        this->tf1,
        this->tf2,
        this->enable_statistics,
        this->cost_density,
        this->num_leaf_tests,
        this->request,
        *this->result);
}

//==============================================================================
template <typename S>
bool MeshCollisionTraversalNodeOBB<S>::BVTesting(
    int b1, int b2, const Matrix3<S>& Rc, const Vector3<S>& Tc) const
{
  if(this->enable_statistics) this->num_bv_tests++;

  return obbDisjoint(
        Rc, Tc,
        this->model1->getBV(b1).bv.extent,
        this->model2->getBV(b2).bv.extent);
}

//==============================================================================
template <typename S>
bool MeshCollisionTraversalNodeOBB<S>::BVTesting(
    int b1, int b2, const Transform3<S>& tf) const
{
  if(this->enable_statistics) this->num_bv_tests++;

  return obbDisjoint(tf,
        this->model1->getBV(b1).bv.extent,
        this->model2->getBV(b2).bv.extent);
}

//==============================================================================
template <typename S>
void MeshCollisionTraversalNodeOBB<S>::leafTesting(
    int b1, int b2, const Matrix3<S>& Rc, const Vector3<S>& Tc) const
{
  FCL_UNUSED(Rc);
  FCL_UNUSED(Tc);

  detail::meshCollisionOrientedNodeLeafTesting(
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
        this->tf1,
        this->tf2,
        this->enable_statistics,
        this->cost_density,
        this->num_leaf_tests,
        this->request,
        *this->result);
}

//==============================================================================
template <typename S>
void MeshCollisionTraversalNodeOBB<S>::leafTesting(
    int b1, int b2, const Transform3<S>& tf) const
{
  detail::meshCollisionOrientedNodeLeafTesting(
        b1,
        b2,
        this->model1,
        this->model2,
        this->vertices1,
        this->vertices2,
        this->tri_indices1,
        this->tri_indices2,
        tf,
        this->tf1,
        this->tf2,
        this->enable_statistics,
        this->cost_density,
        this->num_leaf_tests,
        this->request,
        *this->result);
}

//==============================================================================
template <typename S>
MeshCollisionTraversalNodeRSS<S>::MeshCollisionTraversalNodeRSS()
  : MeshCollisionTraversalNode<RSS<S>>(),
    R(Matrix3<S>::Identity())
{
  // Do nothing
}

//==============================================================================
template <typename S>
bool MeshCollisionTraversalNodeRSS<S>::BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;

  return !overlap(R, T, this->model1->getBV(b1).bv, this->model2->getBV(b2).bv);
}

//==============================================================================
template <typename S>
void MeshCollisionTraversalNodeRSS<S>::leafTesting(int b1, int b2) const
{
  detail::meshCollisionOrientedNodeLeafTesting(
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
        this->tf1,
        this->tf2,
        this->enable_statistics,
        this->cost_density,
        this->num_leaf_tests,
        this->request,
        *this->result);
}

//==============================================================================
template <typename S>
MeshCollisionTraversalNodekIOS<S>::MeshCollisionTraversalNodekIOS()
  : MeshCollisionTraversalNode<kIOS<S>>(),
    R(Matrix3<S>::Identity())
{
  // Do nothing
}

//==============================================================================
template <typename S>
bool MeshCollisionTraversalNodekIOS<S>::BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;

  return !overlap(R, T, this->model1->getBV(b1).bv, this->model2->getBV(b2).bv);
}

//==============================================================================
template <typename S>
void MeshCollisionTraversalNodekIOS<S>::leafTesting(int b1, int b2) const
{
  detail::meshCollisionOrientedNodeLeafTesting(
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
        this->tf1,
        this->tf2,
        this->enable_statistics,
        this->cost_density,
        this->num_leaf_tests,
        this->request,
        *this->result);
}

//==============================================================================
template <typename S>
MeshCollisionTraversalNodeOBBRSS<S>::MeshCollisionTraversalNodeOBBRSS()
  : MeshCollisionTraversalNode<OBBRSS<S>>(),
    R(Matrix3<S>::Identity())
{
  // Do nothing
}

//==============================================================================
template <typename S>
bool MeshCollisionTraversalNodeOBBRSS<S>::BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;

  return !overlap(R, T, this->model1->getBV(b1).bv, this->model2->getBV(b2).bv);
}

//==============================================================================
template <typename S>
void MeshCollisionTraversalNodeOBBRSS<S>::leafTesting(int b1, int b2) const
{
  detail::meshCollisionOrientedNodeLeafTesting(
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
        this->tf1,
        this->tf2,
        this->enable_statistics,
        this->cost_density,
        this->num_leaf_tests,
        this->request,
        *this->result);
}

template <typename BV>
void meshCollisionOrientedNodeLeafTesting(
    int b1, int b2,
    const BVHModel<BV>* model1,
    const BVHModel<BV>* model2,
    Vector3<typename BV::S>* vertices1,
    Vector3<typename BV::S>* vertices2,
    Triangle* tri_indices1,
    Triangle* tri_indices2,
    const Matrix3<typename BV::S>& R,
    const Vector3<typename BV::S>& T,
    const Transform3<typename BV::S>& tf1,
    const Transform3<typename BV::S>& tf2,
    bool enable_statistics,
    typename BV::S cost_density,
    int& num_leaf_tests,
    const CollisionRequest<typename BV::S>& request,
    CollisionResult<typename BV::S>& result)
{
  using S = typename BV::S;

  if(enable_statistics) num_leaf_tests++;

  const BVNode<BV>& node1 = model1->getBV(b1);
  const BVNode<BV>& node2 = model2->getBV(b2);

  int primitive_id1 = node1.primitiveId();
  int primitive_id2 = node2.primitiveId();

  const Triangle& tri_id1 = tri_indices1[primitive_id1];
  const Triangle& tri_id2 = tri_indices2[primitive_id2];

  const Vector3<S>& p1 = vertices1[tri_id1[0]];
  const Vector3<S>& p2 = vertices1[tri_id1[1]];
  const Vector3<S>& p3 = vertices1[tri_id1[2]];
  const Vector3<S>& q1 = vertices2[tri_id2[0]];
  const Vector3<S>& q2 = vertices2[tri_id2[1]];
  const Vector3<S>& q3 = vertices2[tri_id2[2]];

  if(model1->isOccupied() && model2->isOccupied())
  {
    bool is_intersect = false;

    if(!request.enable_contact) // only interested in collision or not
    {
      if(Intersect<S>::intersect_Triangle(p1, p2, p3, q1, q2, q3, R, T))
      {
        is_intersect = true;
        if(result.numContacts() < request.num_max_contacts)
          result.addContact(Contact<S>(model1, model2, primitive_id1, primitive_id2));
      }
    }
    else // need compute the contact information
    {
      S penetration;
      Vector3<S> normal;
      unsigned int n_contacts;
      Vector3<S> contacts[2];

      if(Intersect<S>::intersect_Triangle(p1, p2, p3, q1, q2, q3,
                                       R, T,
                                       contacts,
                                       &n_contacts,
                                       &penetration,
                                       &normal))
      {
        is_intersect = true;

        if(request.num_max_contacts < result.numContacts() + n_contacts)
          n_contacts = (request.num_max_contacts > result.numContacts()) ? (request.num_max_contacts - result.numContacts()) : 0;

        for(unsigned int i = 0; i < n_contacts; ++i)
        {
          result.addContact(Contact<S>(model1, model2, primitive_id1, primitive_id2, tf1 * contacts[i], tf1.linear() * normal, penetration));
        }
      }
    }

    if(is_intersect && request.enable_cost)
    {
      AABB<S> overlap_part;
      AABB<S>(tf1 * p1, tf1 * p2, tf1 * p3).overlap(AABB<S>(tf2 * q1, tf2 * q2, tf2 * q3), overlap_part);
      result.addCostSource(CostSource<S>(overlap_part, cost_density), request.num_max_cost_sources);
    }
  }
  else if((!model1->isFree() && !model2->isFree()) && request.enable_cost)
  {
    if(Intersect<S>::intersect_Triangle(p1, p2, p3, q1, q2, q3, R, T))
    {
      AABB<S> overlap_part;
      AABB<S>(tf1 * p1, tf1 * p2, tf1 * p3).overlap(AABB<S>(tf2 * q1, tf2 * q2, tf2 * q3), overlap_part);
      result.addCostSource(CostSource<S>(overlap_part, cost_density), request.num_max_cost_sources);
    }
  }
}

//==============================================================================
template <typename BV>
void meshCollisionOrientedNodeLeafTesting(
    int b1,
    int b2,
    const BVHModel<BV>* model1,
    const BVHModel<BV>* model2,
    Vector3<typename BV::S>* vertices1,
    Vector3<typename BV::S>* vertices2,
    Triangle* tri_indices1,
    Triangle* tri_indices2,
    const Transform3<typename BV::S>& tf,
    const Transform3<typename BV::S>& tf1,
    const Transform3<typename BV::S>& tf2,
    bool enable_statistics,
    typename BV::S cost_density,
    int& num_leaf_tests,
    const CollisionRequest<typename BV::S>& request,
    CollisionResult<typename BV::S>& result)
{
  using S = typename BV::S;

  if(enable_statistics) num_leaf_tests++;

  const BVNode<BV>& node1 = model1->getBV(b1);
  const BVNode<BV>& node2 = model2->getBV(b2);

  int primitive_id1 = node1.primitiveId();
  int primitive_id2 = node2.primitiveId();

  const Triangle& tri_id1 = tri_indices1[primitive_id1];
  const Triangle& tri_id2 = tri_indices2[primitive_id2];

  const Vector3<S>& p1 = vertices1[tri_id1[0]];
  const Vector3<S>& p2 = vertices1[tri_id1[1]];
  const Vector3<S>& p3 = vertices1[tri_id1[2]];
  const Vector3<S>& q1 = vertices2[tri_id2[0]];
  const Vector3<S>& q2 = vertices2[tri_id2[1]];
  const Vector3<S>& q3 = vertices2[tri_id2[2]];

  if(model1->isOccupied() && model2->isOccupied())
  {
    bool is_intersect = false;

    if(!request.enable_contact) // only interested in collision or not
    {
      if(Intersect<S>::intersect_Triangle(p1, p2, p3, q1, q2, q3, tf))
      {
        is_intersect = true;
        if(result.numContacts() < request.num_max_contacts)
          result.addContact(Contact<S>(model1, model2, primitive_id1, primitive_id2));
      }
    }
    else // need compute the contact information
    {
      S penetration;
      Vector3<S> normal;
      unsigned int n_contacts;
      Vector3<S> contacts[2];

      if(Intersect<S>::intersect_Triangle(
           p1, p2, p3, q1, q2, q3, tf, contacts, &n_contacts, &penetration, &normal))
      {
        is_intersect = true;

        if(request.num_max_contacts < result.numContacts() + n_contacts)
          n_contacts = (request.num_max_contacts > result.numContacts()) ? (request.num_max_contacts - result.numContacts()) : 0;

        for(unsigned int i = 0; i < n_contacts; ++i)
        {
          result.addContact(Contact<S>(model1, model2, primitive_id1, primitive_id2, tf1 * contacts[i], tf1.linear() * normal, penetration));
        }
      }
    }

    if(is_intersect && request.enable_cost)
    {
      AABB<S> overlap_part;
      AABB<S>(tf1 * p1, tf1 * p2, tf1 * p3).overlap(AABB<S>(tf2 * q1, tf2 * q2, tf2 * q3), overlap_part);
      result.addCostSource(CostSource<S>(overlap_part, cost_density), request.num_max_cost_sources);
    }
  }
  else if((!model1->isFree() && !model2->isFree()) && request.enable_cost)
  {
    if(Intersect<S>::intersect_Triangle(p1, p2, p3, q1, q2, q3, tf))
    {
      AABB<S> overlap_part;
      AABB<S>(tf1 * p1, tf1 * p2, tf1 * p3).overlap(AABB<S>(tf2 * q1, tf2 * q2, tf2 * q3), overlap_part);
      result.addCostSource(CostSource<S>(overlap_part, cost_density), request.num_max_cost_sources);
    }
  }
}

template<typename BV, typename OrientedNode>
bool setupMeshCollisionOrientedNode(
    OrientedNode& node,
    const BVHModel<BV>& model1, const Transform3<typename BV::S>& tf1,
    const BVHModel<BV>& model2, const Transform3<typename BV::S>& tf2,
    const CollisionRequest<typename BV::S>& request,
    CollisionResult<typename BV::S>& result)
{
  if(model1.getModelType() != BVH_MODEL_TRIANGLES || model2.getModelType() != BVH_MODEL_TRIANGLES)
    return false;

  node.vertices1 = model1.vertices;
  node.vertices2 = model2.vertices;

  node.tri_indices1 = model1.tri_indices;
  node.tri_indices2 = model2.tri_indices;

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;

  node.request = request;
  node.result = &result;

  node.cost_density = model1.cost_density * model2.cost_density;

  relativeTransform(tf1.linear(), tf1.translation(), tf2.linear(), tf2.translation(), node.R, node.T);

  return true;
}

//==============================================================================
template <typename S>
bool initialize(
    MeshCollisionTraversalNodeOBB<S>& node,
    const BVHModel<OBB<S>>& model1,
    const Transform3<S>& tf1,
    const BVHModel<OBB<S>>& model2,
    const Transform3<S>& tf2,
    const CollisionRequest<S>& request,
    CollisionResult<S>& result)
{
  return detail::setupMeshCollisionOrientedNode(
        node, model1, tf1, model2, tf2, request, result);
}

//==============================================================================
template <typename S>
bool initialize(
    MeshCollisionTraversalNodeRSS<S>& node,
    const BVHModel<RSS<S>>& model1,
    const Transform3<S>& tf1,
    const BVHModel<RSS<S>>& model2,
    const Transform3<S>& tf2,
    const CollisionRequest<S>& request,
    CollisionResult<S>& result)
{
  return detail::setupMeshCollisionOrientedNode(
        node, model1, tf1, model2, tf2, request, result);
}

//==============================================================================
template <typename S>
bool initialize(
    MeshCollisionTraversalNodekIOS<S>& node,
    const BVHModel<kIOS<S>>& model1,
    const Transform3<S>& tf1,
    const BVHModel<kIOS<S>>& model2,
    const Transform3<S>& tf2,
    const CollisionRequest<S>& request,
    CollisionResult<S>& result)
{
  return detail::setupMeshCollisionOrientedNode(
        node, model1, tf1, model2, tf2, request, result);
}

//==============================================================================
template <typename S>
bool initialize(
    MeshCollisionTraversalNodeOBBRSS<S>& node,
    const BVHModel<OBBRSS<S>>& model1,
    const Transform3<S>& tf1,
    const BVHModel<OBBRSS<S>>& model2,
    const Transform3<S>& tf2,
    const CollisionRequest<S>& request,
    CollisionResult<S>& result)
{
  return detail::setupMeshCollisionOrientedNode(
        node, model1, tf1, model2, tf2, request, result);
}

} // namespace detail
} // namespace fcl

#endif

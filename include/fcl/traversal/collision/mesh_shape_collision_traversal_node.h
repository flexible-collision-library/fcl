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

#ifndef FCL_TRAVERSAL_MESHSHAPECOLLISIONTRAVERSALNODE_H
#define FCL_TRAVERSAL_MESHSHAPECOLLISIONTRAVERSALNODE_H

#include "fcl/shape/compute_bv.h"
#include "fcl/traversal/collision/bvh_shape_collision_traversal_node.h"

namespace fcl
{

/// @brief Traversal node for collision between mesh and shape
template <typename BV, typename S, typename NarrowPhaseSolver>
class MeshShapeCollisionTraversalNode
    : public BVHShapeCollisionTraversalNode<BV, S>
{
public:

  using Scalar = typename BV::Scalar;

  MeshShapeCollisionTraversalNode();

  /// @brief Intersection testing between leaves (one triangle and one shape)
  void leafTesting(int b1, int b2) const;

  /// @brief Whether the traversal process can stop early
  bool canStop() const;

  Vector3<Scalar>* vertices;
  Triangle* tri_indices;
  
  Scalar cost_density;

  const NarrowPhaseSolver* nsolver;
};

/// @brief Initialize traversal node for collision between one mesh and one
/// shape, given current object transform
template <typename BV, typename S, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeCollisionTraversalNode<BV, S, NarrowPhaseSolver>& node,
    BVHModel<BV>& model1,
    Transform3<typename BV::Scalar>& tf1,
    const S& model2,
    const Transform3<typename BV::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename BV::Scalar>& request,
    CollisionResult<typename BV::Scalar>& result,
    bool use_refit = false, bool refit_bottomup = false);

/// @cond IGNORE
namespace details
{

template <typename BV, typename S, typename NarrowPhaseSolver>
void meshShapeCollisionOrientedNodeLeafTesting(
    int b1,
    int b2,
    const BVHModel<BV>* model1,
    const S& model2,
    Vector3<typename BV::Scalar>* vertices,
    Triangle* tri_indices,
    const Transform3<typename BV::Scalar>& tf1,
    const Transform3<typename BV::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    bool enable_statistics,
    typename BV::Scalar cost_density,
    int& num_leaf_tests,
    const CollisionRequest<typename BV::Scalar>& request,
    CollisionResult<typename BV::Scalar>& result);

} // namespace detials

/// @endcond

/// @brief Traversal node for mesh and shape, when mesh BVH is one of the oriented node (OBB, RSS, OBBRSS, kIOS)
template <typename S, typename NarrowPhaseSolver>
class MeshShapeCollisionTraversalNodeOBB
    : public MeshShapeCollisionTraversalNode<
          OBB<typename NarrowPhaseSolver::Scalar>, S, NarrowPhaseSolver>
{
public:
  MeshShapeCollisionTraversalNodeOBB();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

};

/// @brief Initialize the traversal node for collision between one mesh and one
/// shape, specialized for OBB type
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeCollisionTraversalNodeOBB<S, NarrowPhaseSolver>& node,
    const BVHModel<OBB<typename NarrowPhaseSolver::Scalar>>& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const S& model2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
    CollisionResult<typename NarrowPhaseSolver::Scalar>& result);

template <typename S, typename NarrowPhaseSolver>
class MeshShapeCollisionTraversalNodeRSS
    : public MeshShapeCollisionTraversalNode<
          RSS<typename NarrowPhaseSolver::Scalar>, S, NarrowPhaseSolver>
{
public:
  MeshShapeCollisionTraversalNodeRSS();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

};

/// @brief Initialize the traversal node for collision between one mesh and one
/// shape, specialized for RSS type
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeCollisionTraversalNodeRSS<S, NarrowPhaseSolver>& node,
    const BVHModel<RSS<typename NarrowPhaseSolver::Scalar>>& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const S& model2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
    CollisionResult<typename NarrowPhaseSolver::Scalar>& result);

template <typename S, typename NarrowPhaseSolver>
class MeshShapeCollisionTraversalNodekIOS
    : public MeshShapeCollisionTraversalNode<
          kIOS<typename NarrowPhaseSolver::Scalar>, S, NarrowPhaseSolver>
{
public:
  MeshShapeCollisionTraversalNodekIOS();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

};

/// @brief Initialize the traversal node for collision between one mesh and one
///  shape, specialized for kIOS type
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeCollisionTraversalNodekIOS<S, NarrowPhaseSolver>& node,
    const BVHModel<kIOS<typename NarrowPhaseSolver::Scalar>>& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const S& model2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
    CollisionResult<typename NarrowPhaseSolver::Scalar>& result);

template <typename S, typename NarrowPhaseSolver>
class MeshShapeCollisionTraversalNodeOBBRSS
    : public MeshShapeCollisionTraversalNode<
          OBBRSS<typename NarrowPhaseSolver::Scalar>, S, NarrowPhaseSolver>
{
public:
  MeshShapeCollisionTraversalNodeOBBRSS();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

};

/// @brief Initialize the traversal node for collision between one mesh and one
/// shape, specialized for OBBRSS type
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeCollisionTraversalNodeOBBRSS<S, NarrowPhaseSolver>& node,
    const BVHModel<OBBRSS<typename NarrowPhaseSolver::Scalar>>& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const S& model2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
    CollisionResult<typename NarrowPhaseSolver::Scalar>& result);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename BV, typename S, typename NarrowPhaseSolver>
MeshShapeCollisionTraversalNode<BV, S, NarrowPhaseSolver>::MeshShapeCollisionTraversalNode()
  : BVHShapeCollisionTraversalNode<BV, S>()
{
  vertices = NULL;
  tri_indices = NULL;

  nsolver = NULL;
}

//==============================================================================
template <typename BV, typename S, typename NarrowPhaseSolver>
void MeshShapeCollisionTraversalNode<BV, S, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_leaf_tests++;
  const BVNode<BV>& node = this->model1->getBV(b1);

  int primitive_id = node.primitiveId();

  const Triangle& tri_id = tri_indices[primitive_id];

  const Vector3<Scalar>& p1 = vertices[tri_id[0]];
  const Vector3<Scalar>& p2 = vertices[tri_id[1]];
  const Vector3<Scalar>& p3 = vertices[tri_id[2]];

  if(this->model1->isOccupied() && this->model2->isOccupied())
  {
    bool is_intersect = false;

    if(!this->request.enable_contact)
    {
      if(nsolver->shapeTriangleIntersect(*(this->model2), this->tf2, p1, p2, p3, NULL, NULL, NULL))
      {
        is_intersect = true;
        if(this->request.num_max_contacts > this->result->numContacts())
          this->result->addContact(Contact<Scalar>(this->model1, this->model2, primitive_id, Contact<Scalar>::NONE));
      }
    }
    else
    {
      Scalar penetration;
      Vector3<Scalar> normal;
      Vector3<Scalar> contactp;

      if(nsolver->shapeTriangleIntersect(*(this->model2), this->tf2, p1, p2, p3, &contactp, &penetration, &normal))
      {
        is_intersect = true;
        if(this->request.num_max_contacts > this->result->numContacts())
          this->result->addContact(Contact<Scalar>(this->model1, this->model2, primitive_id, Contact<Scalar>::NONE, contactp, -normal, penetration));
      }
    }

    if(is_intersect && this->request.enable_cost)
    {
      AABB<Scalar> overlap_part;
      AABB<Scalar> shape_aabb;
      computeBV(*(this->model2), this->tf2, shape_aabb);
      AABB<Scalar>(p1, p2, p3).overlap(shape_aabb, overlap_part);
      this->result->addCostSource(CostSource<Scalar>(overlap_part, cost_density), this->request.num_max_cost_sources);
    }
  }
  if((!this->model1->isFree() && !this->model2->isFree()) && this->request.enable_cost)
  {
    if(nsolver->shapeTriangleIntersect(*(this->model2), this->tf2, p1, p2, p3, NULL, NULL, NULL))
    {
      AABB<Scalar> overlap_part;
      AABB<Scalar> shape_aabb;
      computeBV(*(this->model2), this->tf2, shape_aabb);
      AABB<Scalar>(p1, p2, p3).overlap(shape_aabb, overlap_part);
      this->result->addCostSource(CostSource<Scalar>(overlap_part, cost_density), this->request.num_max_cost_sources);
    }
  }
}

//==============================================================================
template <typename BV, typename S, typename NarrowPhaseSolver>
bool MeshShapeCollisionTraversalNode<BV, S, NarrowPhaseSolver>::canStop() const
{
  return this->request.isSatisfied(*(this->result));
}

//==============================================================================
template <typename BV, typename S, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeCollisionTraversalNode<BV, S, NarrowPhaseSolver>& node,
    BVHModel<BV>& model1,
    Transform3<typename BV::Scalar>& tf1,
    const S& model2,
    const Transform3<typename BV::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename BV::Scalar>& request,
    CollisionResult<typename BV::Scalar>& result,
    bool use_refit,
    bool refit_bottomup)
{
  using Scalar = typename BV::Scalar;

  if(model1.getModelType() != BVH_MODEL_TRIANGLES)
    return false;

  if(!tf1.matrix().isIdentity())
  {
    std::vector<Vector3<Scalar>> vertices_transformed(model1.num_vertices);
    for(int i = 0; i < model1.num_vertices; ++i)
    {
      Vector3<Scalar>& p = model1.vertices[i];
      Vector3<Scalar> new_v = tf1 * p;
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

/// @cond IGNORE
namespace details
{

//==============================================================================
template <typename BV, typename S, typename NarrowPhaseSolver>
void meshShapeCollisionOrientedNodeLeafTesting(
    int b1, int b2,
    const BVHModel<BV>* model1, const S& model2,
    Vector3<typename BV::Scalar>* vertices, Triangle* tri_indices,
    const Transform3<typename BV::Scalar>& tf1,
    const Transform3<typename BV::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    bool enable_statistics,
    typename BV::Scalar cost_density,
    int& num_leaf_tests,
    const CollisionRequest<typename BV::Scalar>& request,
    CollisionResult<typename BV::Scalar>& result)
{
  using Scalar = typename BV::Scalar;

  if(enable_statistics) num_leaf_tests++;
  const BVNode<BV>& node = model1->getBV(b1);

  int primitive_id = node.primitiveId();

  const Triangle& tri_id = tri_indices[primitive_id];

  const Vector3<Scalar>& p1 = vertices[tri_id[0]];
  const Vector3<Scalar>& p2 = vertices[tri_id[1]];
  const Vector3<Scalar>& p3 = vertices[tri_id[2]];

  if(model1->isOccupied() && model2.isOccupied())
  {
    bool is_intersect = false;

    if(!request.enable_contact) // only interested in collision or not
    {
      if(nsolver->shapeTriangleIntersect(model2, tf2, p1, p2, p3, tf1, NULL, NULL, NULL))
      {
        is_intersect = true;
        if(request.num_max_contacts > result.numContacts())
          result.addContact(Contact<Scalar>(model1, &model2, primitive_id, Contact<Scalar>::NONE));
      }
    }
    else
    {
      Scalar penetration;
      Vector3<Scalar> normal;
      Vector3<Scalar> contactp;

      if(nsolver->shapeTriangleIntersect(model2, tf2, p1, p2, p3, tf1, &contactp, &penetration, &normal))
      {
        is_intersect = true;
        if(request.num_max_contacts > result.numContacts())
          result.addContact(Contact<Scalar>(model1, &model2, primitive_id, Contact<Scalar>::NONE, contactp, -normal, penetration));
      }
    }

    if(is_intersect && request.enable_cost)
    {
      AABB<Scalar> overlap_part;
      AABB<Scalar> shape_aabb;
      computeBV(model2, tf2, shape_aabb);
      /* bool res = */ AABB<Scalar>(tf1 * p1, tf1 * p2, tf1 * p3).overlap(shape_aabb, overlap_part);
      result.addCostSource(CostSource<Scalar>(overlap_part, cost_density), request.num_max_cost_sources);
    }
  }
  else if((!model1->isFree() || model2.isFree()) && request.enable_cost)
  {
    if(nsolver->shapeTriangleIntersect(model2, tf2, p1, p2, p3, tf1, NULL, NULL, NULL))
    {
      AABB<Scalar> overlap_part;
      AABB<Scalar> shape_aabb;
      computeBV(model2, tf2, shape_aabb);
      /* bool res = */ AABB<Scalar>(tf1 * p1, tf1 * p2, tf1 * p3).overlap(shape_aabb, overlap_part);
      result.addCostSource(CostSource<Scalar>(overlap_part, cost_density), request.num_max_cost_sources);
    }
  }
}

} // namespace detials

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
MeshShapeCollisionTraversalNodeOBB<S, NarrowPhaseSolver>::
MeshShapeCollisionTraversalNodeOBB()
  : MeshShapeCollisionTraversalNode<OBB<typename NarrowPhaseSolver::Scalar>, S, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool MeshShapeCollisionTraversalNodeOBB<S, NarrowPhaseSolver>::BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;

  return !overlap(this->tf1.linear(), this->tf1.translation(), this->model2_bv, this->model1->getBV(b1).bv);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void MeshShapeCollisionTraversalNodeOBB<S, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  details::meshShapeCollisionOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                     this->tf1, this->tf2, this->nsolver, this->enable_statistics, this->cost_density, this->num_leaf_tests, this->request, *(this->result));
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
MeshShapeCollisionTraversalNodeRSS<S, NarrowPhaseSolver>::MeshShapeCollisionTraversalNodeRSS()
  : MeshShapeCollisionTraversalNode<RSS<typename NarrowPhaseSolver::Scalar>, S, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool MeshShapeCollisionTraversalNodeRSS<S, NarrowPhaseSolver>::BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;

  return !overlap(this->tf1.linear(), this->tf1.translation(), this->model2_bv, this->model1->getBV(b1).bv);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void MeshShapeCollisionTraversalNodeRSS<S, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  details::meshShapeCollisionOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                     this->tf1, this->tf2, this->nsolver, this->enable_statistics, this->cost_density, this->num_leaf_tests, this->request, *(this->result));
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
MeshShapeCollisionTraversalNodekIOS<S, NarrowPhaseSolver>::
MeshShapeCollisionTraversalNodekIOS()
  : MeshShapeCollisionTraversalNode<kIOS<typename NarrowPhaseSolver::Scalar>, S, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool MeshShapeCollisionTraversalNodekIOS<S, NarrowPhaseSolver>::BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;

  return !overlap(this->tf1.linear(), this->tf1.translation(), this->model2_bv, this->model1->getBV(b1).bv);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void MeshShapeCollisionTraversalNodekIOS<S, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  details::meshShapeCollisionOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                     this->tf1, this->tf2, this->nsolver, this->enable_statistics, this->cost_density, this->num_leaf_tests, this->request, *(this->result));
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
MeshShapeCollisionTraversalNodeOBBRSS<S, NarrowPhaseSolver>::
MeshShapeCollisionTraversalNodeOBBRSS()
  : MeshShapeCollisionTraversalNode<OBBRSS<typename NarrowPhaseSolver::Scalar>, S, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool MeshShapeCollisionTraversalNodeOBBRSS<S, NarrowPhaseSolver>::BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;
  return !overlap(this->tf1.linear(), this->tf1.translation(), this->model2_bv, this->model1->getBV(b1).bv);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void MeshShapeCollisionTraversalNodeOBBRSS<S, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  details::meshShapeCollisionOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                     this->tf1, this->tf2, this->nsolver, this->enable_statistics, this->cost_density, this->num_leaf_tests, this->request, *(this->result));
}

/// @cond IGNORE
namespace details
{

template <typename BV, typename S, typename NarrowPhaseSolver,
          template <typename, typename> class OrientedNode>
bool setupMeshShapeCollisionOrientedNode(
    OrientedNode<S, NarrowPhaseSolver>& node,
    const BVHModel<BV>& model1,
    const Transform3<typename BV::Scalar>& tf1,
    const S& model2, const Transform3<typename BV::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename BV::Scalar>& request,
    CollisionResult<typename BV::Scalar>& result)
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

} // namespace details
/// @endcond

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeCollisionTraversalNodeOBB<S, NarrowPhaseSolver>& node,
    const BVHModel<OBB<typename NarrowPhaseSolver::Scalar>>& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const S& model2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
    CollisionResult<typename NarrowPhaseSolver::Scalar>& result)
{
  return details::setupMeshShapeCollisionOrientedNode(
        node, model1, tf1, model2, tf2, nsolver, request, result);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeCollisionTraversalNodeRSS<S, NarrowPhaseSolver>& node,
    const BVHModel<RSS<typename NarrowPhaseSolver::Scalar>>& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const S& model2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
    CollisionResult<typename NarrowPhaseSolver::Scalar>& result)
{
  return details::setupMeshShapeCollisionOrientedNode(
        node, model1, tf1, model2, tf2, nsolver, request, result);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeCollisionTraversalNodekIOS<S, NarrowPhaseSolver>& node,
    const BVHModel<kIOS<typename NarrowPhaseSolver::Scalar>>& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const S& model2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
    CollisionResult<typename NarrowPhaseSolver::Scalar>& result)
{
  return details::setupMeshShapeCollisionOrientedNode(
        node, model1, tf1, model2, tf2, nsolver, request, result);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    MeshShapeCollisionTraversalNodeOBBRSS<S, NarrowPhaseSolver>& node,
    const BVHModel<OBBRSS<typename NarrowPhaseSolver::Scalar>>& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const S& model2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
    CollisionResult<typename NarrowPhaseSolver::Scalar>& result)
{
  return details::setupMeshShapeCollisionOrientedNode(
        node, model1, tf1, model2, tf2, nsolver, request, result);
}

} // namespace fcl

#endif

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

#ifndef FCL_TRAVERSAL_SHAPEMESHCOLLISIONTRAVERSALNODE_H
#define FCL_TRAVERSAL_SHAPEMESHCOLLISIONTRAVERSALNODE_H

#include "fcl/shape/compute_bv.h"
#include "fcl/traversal/collision/shape_bvh_collision_traversal_node.h"

namespace fcl
{

/// @brief Traversal node for collision between shape and mesh
template <typename S, typename BV, typename NarrowPhaseSolver>
class ShapeMeshCollisionTraversalNode
    : public ShapeBVHCollisionTraversalNode<S, BV>
{
public:
  using Scalar = typename BV::Scalar;

  ShapeMeshCollisionTraversalNode();

  /// @brief Intersection testing between leaves (one shape and one triangle)
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
template <typename S, typename BV, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshCollisionTraversalNode<S, BV, NarrowPhaseSolver>& node,
    const S& model1,
    const Transform3<typename BV::Scalar>& tf1,
    BVHModel<BV>& model2,
    Transform3<typename BV::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename BV::Scalar>& request,
    CollisionResult<typename BV::Scalar>& result,
    bool use_refit = false, bool refit_bottomup = false);

/// @brief Traversal node for shape and mesh, when mesh BVH is one of the oriented node (OBB, RSS, OBBRSS, kIOS)
template <typename S, typename NarrowPhaseSolver>
class ShapeMeshCollisionTraversalNodeOBB
    : public ShapeMeshCollisionTraversalNode<
    S, OBB<typename NarrowPhaseSolver::Scalar>, NarrowPhaseSolver>
{
public:
  ShapeMeshCollisionTraversalNodeOBB();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;
};

/// @brief Initialize the traversal node for collision between one mesh and one
/// shape, specialized for OBB type
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshCollisionTraversalNodeOBB<S, NarrowPhaseSolver>& node,
    const S& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const BVHModel<OBB<typename NarrowPhaseSolver::Scalar>>& model2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
    CollisionResult<typename NarrowPhaseSolver::Scalar>& result);

template <typename S, typename NarrowPhaseSolver>
class ShapeMeshCollisionTraversalNodeRSS
    : public ShapeMeshCollisionTraversalNode<S, RSS<typename NarrowPhaseSolver::Scalar>, NarrowPhaseSolver>
{
public:
  ShapeMeshCollisionTraversalNodeRSS();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

};

/// @brief Initialize the traversal node for collision between one mesh and one
/// shape, specialized for RSS type
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshCollisionTraversalNodeRSS<S, NarrowPhaseSolver>& node,
    const S& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const BVHModel<RSS<typename NarrowPhaseSolver::Scalar>>& model2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
    CollisionResult<typename NarrowPhaseSolver::Scalar>& result);

template <typename S, typename NarrowPhaseSolver>
class ShapeMeshCollisionTraversalNodekIOS
    : public ShapeMeshCollisionTraversalNode<S, kIOS<typename NarrowPhaseSolver::Scalar>, NarrowPhaseSolver>
{
public:
  ShapeMeshCollisionTraversalNodekIOS();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

};

/// @brief Initialize the traversal node for collision between one mesh and one
/// shape, specialized for kIOS type
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshCollisionTraversalNodekIOS<S, NarrowPhaseSolver>& node,
    const S& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const BVHModel<kIOS<typename NarrowPhaseSolver::Scalar>>& model2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
    CollisionResult<typename NarrowPhaseSolver::Scalar>& result);

template <typename S, typename NarrowPhaseSolver>
class ShapeMeshCollisionTraversalNodeOBBRSS
    : public ShapeMeshCollisionTraversalNode<S, OBBRSS<typename NarrowPhaseSolver::Scalar>, NarrowPhaseSolver>
{
public:
  ShapeMeshCollisionTraversalNodeOBBRSS();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

};

/// @brief Initialize the traversal node for collision between one mesh and one
/// shape, specialized for OBBRSS type
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshCollisionTraversalNodeOBBRSS<S, NarrowPhaseSolver>& node,
    const S& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const BVHModel<OBBRSS<typename NarrowPhaseSolver::Scalar>>& model2,
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
template <typename S, typename BV, typename NarrowPhaseSolver>
ShapeMeshCollisionTraversalNode<S, BV, NarrowPhaseSolver>::ShapeMeshCollisionTraversalNode()
  : ShapeBVHCollisionTraversalNode<S, BV>()
{
  vertices = NULL;
  tri_indices = NULL;

  nsolver = NULL;
}

//==============================================================================
template <typename S, typename BV, typename NarrowPhaseSolver>
void ShapeMeshCollisionTraversalNode<S, BV, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  using Scalar = typename NarrowPhaseSolver::Scalar;

  if(this->enable_statistics) this->num_leaf_tests++;
  const BVNode<BV>& node = this->model2->getBV(b2);

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
      if(nsolver->shapeTriangleIntersect(*(this->model1), this->tf1, p1, p2, p3, NULL, NULL, NULL))
      {
        is_intersect = true;
        if(this->request.num_max_contacts > this->result->numContacts())
          this->result->addContact(Contact<Scalar>(this->model1, this->model2, Contact<Scalar>::NONE, primitive_id));
      }
    }
    else
    {
      Scalar penetration;
      Vector3<Scalar> normal;
      Vector3<Scalar> contactp;

      if(nsolver->shapeTriangleIntersect(*(this->model1), this->tf1, p1, p2, p3, &contactp, &penetration, &normal))
      {
        is_intersect = true;
        if(this->request.num_max_contacts > this->result->numContacts())
          this->result->addContact(Contact<Scalar>(this->model1, this->model2, Contact<Scalar>::NONE, primitive_id, contactp, normal, penetration));
      }
    }

    if(is_intersect && this->request.enable_cost)
    {
      AABB<Scalar> overlap_part;
      AABB<Scalar> shape_aabb;
      computeBV(*(this->model1), this->tf1, shape_aabb);
      AABB<Scalar>(p1, p2, p3).overlap(shape_aabb, overlap_part);
      this->result->addCostSource(CostSource<Scalar>(overlap_part, cost_density), this->request.num_max_cost_sources);
    }
  }
  else if((!this->model1->isFree() && !this->model2->isFree()) && this->request.enable_cost)
  {
    if(nsolver->shapeTriangleIntersect(*(this->model1), this->tf1, p1, p2, p3, NULL, NULL, NULL))
    {
      AABB<Scalar> overlap_part;
      AABB<Scalar> shape_aabb;
      computeBV(*(this->model1), this->tf1, shape_aabb);
      AABB<Scalar>(p1, p2, p3).overlap(shape_aabb, overlap_part);
      this->result->addCostSource(CostSource<Scalar>(overlap_part, cost_density), this->request.num_max_cost_sources);
    }
  }
}

//==============================================================================
template <typename S, typename BV, typename NarrowPhaseSolver>
bool ShapeMeshCollisionTraversalNode<S, BV, NarrowPhaseSolver>::canStop() const
{
  return this->request.isSatisfied(*(this->result));
}

//==============================================================================
template <typename S, typename BV, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshCollisionTraversalNode<S, BV, NarrowPhaseSolver>& node,
    const S& model1,
    const Transform3<typename BV::Scalar>& tf1,
    BVHModel<BV>& model2,
    Transform3<typename BV::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename BV::Scalar>& request,
    CollisionResult<typename BV::Scalar>& result,
    bool use_refit,
    bool refit_bottomup)
{
  using Scalar = typename BV::Scalar;

  if(model2.getModelType() != BVH_MODEL_TRIANGLES)
    return false;

  if(!tf2.matrix().isIdentity())
  {
    std::vector<Vector3<Scalar>> vertices_transformed(model2.num_vertices);
    for(int i = 0; i < model2.num_vertices; ++i)
    {
      Vector3<Scalar>& p = model2.vertices[i];
      Vector3<Scalar> new_v = tf2 * p;
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
template <typename S, typename NarrowPhaseSolver>
ShapeMeshCollisionTraversalNodeOBB<S, NarrowPhaseSolver>::ShapeMeshCollisionTraversalNodeOBB() : ShapeMeshCollisionTraversalNode<S, OBB<typename NarrowPhaseSolver::Scalar>, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool ShapeMeshCollisionTraversalNodeOBB<S, NarrowPhaseSolver>::BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;
  return !overlap(this->tf2.linear(), this->tf2.translation(), this->model1_bv, this->model2->getBV(b2).bv);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void ShapeMeshCollisionTraversalNodeOBB<S, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  details::meshShapeCollisionOrientedNodeLeafTesting(b2, b1, *(this->model2), this->model1, this->vertices, this->tri_indices,
                                                     this->tf2, this->tf1, this->nsolver, this->enable_statistics, this->cost_density, this->num_leaf_tests, this->request, *(this->request));

  // may need to change the order in pairs
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
ShapeMeshCollisionTraversalNodeRSS<S, NarrowPhaseSolver>::ShapeMeshCollisionTraversalNodeRSS() : ShapeMeshCollisionTraversalNode<S, RSS<typename NarrowPhaseSolver::Scalar>, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool ShapeMeshCollisionTraversalNodeRSS<S, NarrowPhaseSolver>::BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;
  return !overlap(this->tf2.linear(), this->tf2.translation(), this->model1_bv, this->model2->getBV(b2).bv);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void ShapeMeshCollisionTraversalNodeRSS<S, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  details::meshShapeCollisionOrientedNodeLeafTesting(b2, b1, *(this->model2), this->model1, this->vertices, this->tri_indices,
                                                     this->tf2, this->tf1, this->nsolver, this->enable_statistics, this->cost_density, this->num_leaf_tests, this->request, *(this->request));

  // may need to change the order in pairs
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
ShapeMeshCollisionTraversalNodekIOS<S, NarrowPhaseSolver>::ShapeMeshCollisionTraversalNodekIOS() : ShapeMeshCollisionTraversalNode<S, kIOS<typename NarrowPhaseSolver::Scalar>, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool ShapeMeshCollisionTraversalNodekIOS<S, NarrowPhaseSolver>::BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;
  return !overlap(this->tf2.linear(), this->tf2.translation(), this->model1_bv, this->model2->getBV(b2).bv);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void ShapeMeshCollisionTraversalNodekIOS<S, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  details::meshShapeCollisionOrientedNodeLeafTesting(b2, b1, *(this->model2), this->model1, this->vertices, this->tri_indices,
                                                     this->tf2, this->tf1, this->nsolver, this->enable_statistics, this->cost_density, this->num_leaf_tests, this->request, *(this->request));

  // may need to change the order in pairs
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
ShapeMeshCollisionTraversalNodeOBBRSS<S, NarrowPhaseSolver>::ShapeMeshCollisionTraversalNodeOBBRSS() : ShapeMeshCollisionTraversalNode<S, OBBRSS<typename NarrowPhaseSolver::Scalar>, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool ShapeMeshCollisionTraversalNodeOBBRSS<S, NarrowPhaseSolver>::BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;
  return !overlap(this->tf2.linear(), this->tf2.translation(), this->model1_bv, this->model2->getBV(b2).bv);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void ShapeMeshCollisionTraversalNodeOBBRSS<S, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  details::meshShapeCollisionOrientedNodeLeafTesting(b2, b1, *(this->model2), this->model1, this->vertices, this->tri_indices,
                                                     this->tf2, this->tf1, this->nsolver, this->enable_statistics, this->cost_density, this->num_leaf_tests, this->request, *(this->request));

  // may need to change the order in pairs
}

/// @cond IGNORE
namespace details
{
template <typename S, typename BV, typename NarrowPhaseSolver, template <typename, typename> class OrientedNode>
static inline bool setupShapeMeshCollisionOrientedNode(OrientedNode<S, NarrowPhaseSolver>& node,
                                                       const S& model1, const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
                                                       const BVHModel<BV>& model2, const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
                                                       const NarrowPhaseSolver* nsolver,
                                                       const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
                                                       CollisionResult<typename NarrowPhaseSolver::Scalar>& result)
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

} // namespace details
/// @endcond

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshCollisionTraversalNodeOBB<S, NarrowPhaseSolver>& node,
    const S& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const BVHModel<OBB<typename NarrowPhaseSolver::Scalar>>& model2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
    CollisionResult<typename NarrowPhaseSolver::Scalar>& result)
{
  return details::setupShapeMeshCollisionOrientedNode(
        node, model1, tf1, model2, tf2, nsolver, request, result);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshCollisionTraversalNodeRSS<S, NarrowPhaseSolver>& node,
    const S& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const BVHModel<RSS<typename NarrowPhaseSolver::Scalar>>& model2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
    CollisionResult<typename NarrowPhaseSolver::Scalar>& result)
{
  return details::setupShapeMeshCollisionOrientedNode(
        node, model1, tf1, model2, tf2, nsolver, request, result);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshCollisionTraversalNodekIOS<S, NarrowPhaseSolver>& node,
    const S& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const BVHModel<kIOS<typename NarrowPhaseSolver::Scalar>>& model2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
    CollisionResult<typename NarrowPhaseSolver::Scalar>& result)
{
  return details::setupShapeMeshCollisionOrientedNode(
        node, model1, tf1, model2, tf2, nsolver, request, result);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshCollisionTraversalNodeOBBRSS<S, NarrowPhaseSolver>& node,
    const S& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const BVHModel<OBBRSS<typename NarrowPhaseSolver::Scalar>>& model2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename NarrowPhaseSolver::Scalar>& request,
    CollisionResult<typename NarrowPhaseSolver::Scalar>& result)
{
  return details::setupShapeMeshCollisionOrientedNode(
        node, model1, tf1, model2, tf2, nsolver, request, result);
}

} // namespace fcl

#endif

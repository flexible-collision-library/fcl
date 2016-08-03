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

#ifndef FCL_TRAVERSAL_SHAPEMESHDISTANCETRAVERSALNODE_H
#define FCL_TRAVERSAL_SHAPEMESHDISTANCETRAVERSALNODE_H

#include "fcl/traversal/distance/shape_bvh_distance_traversal_node.h"
#include "fcl/BVH/BVH_model.h"

namespace fcl
{

/// @brief Traversal node for distance between shape and mesh
template <typename S, typename BV, typename NarrowPhaseSolver>
class ShapeMeshDistanceTraversalNode
    : public ShapeBVHDistanceTraversalNode<S, BV>
{ 
public:

  using Scalar = typename BV::Scalar;

  ShapeMeshDistanceTraversalNode();

  /// @brief Distance testing between leaves (one shape and one triangle)
  void leafTesting(int b1, int b2) const;

  /// @brief Whether the traversal process can stop early
  bool canStop(Scalar c) const;

  Vector3<Scalar>* vertices;
  Triangle* tri_indices;

  Scalar rel_err;
  Scalar abs_err;
    
  const NarrowPhaseSolver* nsolver;
};

/// @brief Initialize traversal node for distance computation between one shape
/// and one mesh, given the current transforms
template <typename S, typename BV, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshDistanceTraversalNode<S, BV, NarrowPhaseSolver>& node,
    const S& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    BVHModel<BV>& model2,
    Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename NarrowPhaseSolver::Scalar>& request,
    DistanceResult<typename NarrowPhaseSolver::Scalar>& result,
    bool use_refit = false,
    bool refit_bottomup = false);

template <typename S, typename NarrowPhaseSolver>
class ShapeMeshDistanceTraversalNodeRSS
    : public ShapeMeshDistanceTraversalNode<
    S, RSS<typename NarrowPhaseSolver::Scalar>, NarrowPhaseSolver>
{
public:

  using Scalar = typename NarrowPhaseSolver::Scalar;

  ShapeMeshDistanceTraversalNodeRSS();

  void preprocess();

  void postprocess();

  Scalar BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

};

/// @brief Initialize traversal node for distance computation between one shape
/// and one mesh, specialized for RSS type
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshDistanceTraversalNodeRSS<S, NarrowPhaseSolver>& node,
    const S& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const BVHModel<RSS<typename NarrowPhaseSolver::Scalar>>& model2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename NarrowPhaseSolver::Scalar>& request,
    DistanceResult<typename NarrowPhaseSolver::Scalar>& result);

template <typename S, typename NarrowPhaseSolver>
class ShapeMeshDistanceTraversalNodekIOS
    : public ShapeMeshDistanceTraversalNode<
    S, kIOS<typename NarrowPhaseSolver::Scalar>, NarrowPhaseSolver>
{
public:

  using Scalar = typename NarrowPhaseSolver::Scalar;

  ShapeMeshDistanceTraversalNodekIOS();

  void preprocess();

  void postprocess();

  Scalar BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;
  
};

/// @brief Initialize traversal node for distance computation between one shape
/// and one mesh, specialized for kIOS type
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshDistanceTraversalNodekIOS<S, NarrowPhaseSolver>& node,
    const S& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const BVHModel<kIOS<typename NarrowPhaseSolver::Scalar>>& model2,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename NarrowPhaseSolver::Scalar>& request,
    DistanceResult<typename NarrowPhaseSolver::Scalar>& result);

template <typename S, typename NarrowPhaseSolver>
class ShapeMeshDistanceTraversalNodeOBBRSS
    : public ShapeMeshDistanceTraversalNode<
    S, OBBRSS<typename NarrowPhaseSolver::Scalar>, NarrowPhaseSolver>
{
public:

  using Scalar = typename NarrowPhaseSolver::Scalar;

  ShapeMeshDistanceTraversalNodeOBBRSS();

  void preprocess();

  void postprocess();

  Scalar BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;
  
};

/// @brief Initialize traversal node for distance computation between one shape
/// and one mesh, specialized for OBBRSS type
template <typename S, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshDistanceTraversalNodeOBBRSS<S, NarrowPhaseSolver>& node,
    const S& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    const BVHModel<OBBRSS<typename NarrowPhaseSolver::Scalar>>& model2,
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
template <typename S, typename BV, typename NarrowPhaseSolver>
ShapeMeshDistanceTraversalNode<S, BV, NarrowPhaseSolver>::
ShapeMeshDistanceTraversalNode()
  : ShapeBVHDistanceTraversalNode<S, BV>()
{
  vertices = NULL;
  tri_indices = NULL;

  rel_err = 0;
  abs_err = 0;

  nsolver = NULL;
}

//==============================================================================
template <typename S, typename BV, typename NarrowPhaseSolver>
void ShapeMeshDistanceTraversalNode<S, BV, NarrowPhaseSolver>::leafTesting(int b1, int b2) const
{
  using Scalar = typename BV::Scalar;

  if(this->enable_statistics) this->num_leaf_tests++;

  const BVNode<BV>& node = this->model2->getBV(b2);

  int primitive_id = node.primitiveId();

  const Triangle& tri_id = tri_indices[primitive_id];

  const Vector3<Scalar>& p1 = vertices[tri_id[0]];
  const Vector3<Scalar>& p2 = vertices[tri_id[1]];
  const Vector3<Scalar>& p3 = vertices[tri_id[2]];

  Scalar distance;
  Vector3<Scalar> closest_p1, closest_p2;
  nsolver->shapeTriangleDistance(*(this->model1), this->tf1, p1, p2, p3, &distance, &closest_p1, &closest_p2);

  this->result->update(
        distance,
        this->model1,
        this->model2,
        DistanceResult<Scalar>::NONE,
        primitive_id,
        closest_p1,
        closest_p2);
}

//==============================================================================
template <typename S, typename BV, typename NarrowPhaseSolver>
bool ShapeMeshDistanceTraversalNode<S, BV, NarrowPhaseSolver>::canStop(Scalar c) const
{
  if((c >= this->result->min_distance - abs_err) && (c * (1 + rel_err) >= this->result->min_distance))
    return true;
  return false;
}

//==============================================================================
template <typename S, typename BV, typename NarrowPhaseSolver>
bool initialize(
    ShapeMeshDistanceTraversalNode<S, BV, NarrowPhaseSolver>& node,
    const S& model1,
    const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
    BVHModel<BV>& model2,
    Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename NarrowPhaseSolver::Scalar>& request,
    DistanceResult<typename NarrowPhaseSolver::Scalar>& result,
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
template <typename S, typename NarrowPhaseSolver>
ShapeMeshDistanceTraversalNodeRSS<S, NarrowPhaseSolver>::ShapeMeshDistanceTraversalNodeRSS()
  : ShapeMeshDistanceTraversalNode<S, RSS<Scalar>, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void ShapeMeshDistanceTraversalNodeRSS<S, NarrowPhaseSolver>::preprocess()
{
  details::distancePreprocessOrientedNode(
        this->model2, this->vertices, this->tri_indices, 0,
        *(this->model1), this->tf2, this->tf1, this->nsolver, this->request, *(this->result));
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void ShapeMeshDistanceTraversalNodeRSS<S, NarrowPhaseSolver>::postprocess()
{
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
typename NarrowPhaseSolver::Scalar
ShapeMeshDistanceTraversalNodeRSS<S, NarrowPhaseSolver>::
BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;
  return distance(this->tf2.linear(), this->tf2.translation(), this->model1_bv, this->model2->getBV(b2).bv);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void ShapeMeshDistanceTraversalNodeRSS<S, NarrowPhaseSolver>::
leafTesting(int b1, int b2) const
{
  details::meshShapeDistanceOrientedNodeLeafTesting(b2, b1, this->model2, *(this->model1), this->vertices, this->tri_indices,
                                                    this->tf2, this->tf1, this->nsolver, this->enable_statistics, this->num_leaf_tests, this->request, *(this->result));
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
ShapeMeshDistanceTraversalNodekIOS<S, NarrowPhaseSolver>::
ShapeMeshDistanceTraversalNodekIOS()
  : ShapeMeshDistanceTraversalNode<S, kIOS<Scalar>, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void ShapeMeshDistanceTraversalNodekIOS<S, NarrowPhaseSolver>::preprocess()
{
  details::distancePreprocessOrientedNode(
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
template <typename S, typename NarrowPhaseSolver>
void ShapeMeshDistanceTraversalNodekIOS<S, NarrowPhaseSolver>::postprocess()
{
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
typename NarrowPhaseSolver::Scalar
ShapeMeshDistanceTraversalNodekIOS<S, NarrowPhaseSolver>::
BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;
  return distance(this->tf2.linear(), this->tf2.translation(), this->model1_bv, this->model2->getBV(b2).bv);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void ShapeMeshDistanceTraversalNodekIOS<S, NarrowPhaseSolver>::
leafTesting(int b1, int b2) const
{
  details::meshShapeDistanceOrientedNodeLeafTesting(b2, b1, this->model2, *(this->model1), this->vertices, this->tri_indices,
                                                    this->tf2, this->tf1, this->nsolver, this->enable_statistics, this->num_leaf_tests, this->request, *(this->result));
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
ShapeMeshDistanceTraversalNodeOBBRSS<S, NarrowPhaseSolver>::
ShapeMeshDistanceTraversalNodeOBBRSS() : ShapeMeshDistanceTraversalNode<S, OBBRSS<Scalar>, NarrowPhaseSolver>()
{
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void ShapeMeshDistanceTraversalNodeOBBRSS<S, NarrowPhaseSolver>::preprocess()
{
  details::distancePreprocessOrientedNode(
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
template <typename S, typename NarrowPhaseSolver>
void ShapeMeshDistanceTraversalNodeOBBRSS<S, NarrowPhaseSolver>::
postprocess()
{
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
typename NarrowPhaseSolver::Scalar
ShapeMeshDistanceTraversalNodeOBBRSS<S, NarrowPhaseSolver>::
BVTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_bv_tests++;
  return distance(this->tf2.linear(), this->tf2.translation(), this->model1_bv, this->model2->getBV(b2).bv);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
void ShapeMeshDistanceTraversalNodeOBBRSS<S, NarrowPhaseSolver>::
leafTesting(int b1, int b2) const
{
  details::meshShapeDistanceOrientedNodeLeafTesting(
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

namespace details
{
template <typename S, typename BV, typename NarrowPhaseSolver, template <typename, typename> class OrientedNode>
static inline bool setupShapeMeshDistanceOrientedNode(OrientedNode<S, NarrowPhaseSolver>& node,
                                                      const S& model1, const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
                                                      const BVHModel<BV>& model2, const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
                                                      const NarrowPhaseSolver* nsolver,
                                                      const DistanceRequest<typename NarrowPhaseSolver::Scalar>& request,
                                                      DistanceResult<typename NarrowPhaseSolver::Scalar>& result)
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
}


//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool initialize(ShapeMeshDistanceTraversalNodeRSS<S, NarrowPhaseSolver>& node,
                const S& model1, const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
                const BVHModel<RSS<typename NarrowPhaseSolver::Scalar>>& model2, const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
                const NarrowPhaseSolver* nsolver,
                const DistanceRequest<typename NarrowPhaseSolver::Scalar>& request,
                DistanceResult<typename NarrowPhaseSolver::Scalar>& result)
{
  return details::setupShapeMeshDistanceOrientedNode(node, model1, tf1, model2, tf2, nsolver, request, result);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool initialize(ShapeMeshDistanceTraversalNodekIOS<S, NarrowPhaseSolver>& node,
                const S& model1, const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
                const BVHModel<kIOS<typename NarrowPhaseSolver::Scalar>>& model2, const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
                const NarrowPhaseSolver* nsolver,
                const DistanceRequest<typename NarrowPhaseSolver::Scalar>& request,
                DistanceResult<typename NarrowPhaseSolver::Scalar>& result)
{
  return details::setupShapeMeshDistanceOrientedNode(node, model1, tf1, model2, tf2, nsolver, request, result);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
bool initialize(ShapeMeshDistanceTraversalNodeOBBRSS<S, NarrowPhaseSolver>& node,
                const S& model1, const Transform3<typename NarrowPhaseSolver::Scalar>& tf1,
                const BVHModel<OBBRSS<typename NarrowPhaseSolver::Scalar>>& model2, const Transform3<typename NarrowPhaseSolver::Scalar>& tf2,
                const NarrowPhaseSolver* nsolver,
                const DistanceRequest<typename NarrowPhaseSolver::Scalar>& request,
                DistanceResult<typename NarrowPhaseSolver::Scalar>& result)
{
  return details::setupShapeMeshDistanceOrientedNode(node, model1, tf1, model2, tf2, nsolver, request, result);
}

} // namespace fcl

#endif

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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


#ifndef FCL_TRAVERSAL_NODE_MESH_SHAPE_H
#define FCL_TRAVERSAL_NODE_MESH_SHAPE_H

#include "fcl/collision_data.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/shape/geometric_shapes_utility.h"
#include "fcl/traversal/traversal_node_base.h"
#include "fcl/BVH/BVH_model.h"


namespace fcl
{

/// @brief Traversal node for collision between BVH and shape
template<typename BV, typename S>
class BVHShapeCollisionTraversalNode : public CollisionTraversalNodeBase
{
public:
  BVHShapeCollisionTraversalNode() : CollisionTraversalNodeBase()
  {
    model1 = NULL;
    model2 = NULL;

    num_bv_tests = 0;
    num_leaf_tests = 0;
    query_time_seconds = 0.0;
  }

  /// @brief Whether the BV node in the first BVH tree is leaf
  bool isFirstNodeLeaf(int b) const
  {
    return model1->getBV(b).isLeaf();
  }

  /// @brief Obtain the left child of BV node in the first BVH
  int getFirstLeftChild(int b) const
  {
    return model1->getBV(b).leftChild();
  }

  /// @brief Obtain the right child of BV node in the first BVH
  int getFirstRightChild(int b) const
  {
    return model1->getBV(b).rightChild();
  }

  /// @brief BV culling test in one BVTT node
  bool BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) num_bv_tests++;
    return !model1->getBV(b1).bv.overlap(model2_bv);
  }

  const BVHModel<BV>* model1;
  const S* model2;
  BV model2_bv;

  mutable int num_bv_tests;
  mutable int num_leaf_tests;
  mutable FCL_REAL query_time_seconds;
};

/// @brief Traversal node for collision between shape and BVH
template<typename S, typename BV>
class ShapeBVHCollisionTraversalNode : public CollisionTraversalNodeBase
{
public:
  ShapeBVHCollisionTraversalNode() : CollisionTraversalNodeBase()
  {
    model1 = NULL;
    model2 = NULL;

    num_bv_tests = 0;
    num_leaf_tests = 0;
    query_time_seconds = 0.0;
  }

  /// @brief Alway extend the second model, which is a BVH model
  bool firstOverSecond(int, int) const
  {
    return false;
  }

  /// @brief Whether the BV node in the second BVH tree is leaf
  bool isSecondNodeLeaf(int b) const
  {
    return model2->getBV(b).isLeaf();
  }

  /// @brief Obtain the left child of BV node in the second BVH
  int getSecondLeftChild(int b) const
  {
    return model2->getBV(b).leftChild();
  }

  /// @brief Obtain the right child of BV node in the second BVH
  int getSecondRightChild(int b) const
  {
    return model2->getBV(b).rightChild();
  }

  /// @brief BV culling test in one BVTT node
  bool BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) num_bv_tests++;
    return !model2->getBV(b2).bv.overlap(model1_bv);
  }

  const S* model1;
  const BVHModel<BV>* model2;
  BV model1_bv;

  mutable int num_bv_tests;
  mutable int num_leaf_tests;
  mutable FCL_REAL query_time_seconds;
};


/// @brief Traversal node for collision between mesh and shape
template<typename BV, typename S, typename NarrowPhaseSolver>
class MeshShapeCollisionTraversalNode : public BVHShapeCollisionTraversalNode<BV, S>
{
public:
  MeshShapeCollisionTraversalNode() : BVHShapeCollisionTraversalNode<BV, S>()
  {
    vertices = NULL;
    tri_indices = NULL;

    nsolver = NULL;
  }

  /// @brief Intersection testing between leaves (one triangle and one shape)
  void leafTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_leaf_tests++;
    const BVNode<BV>& node = this->model1->getBV(b1);

    int primitive_id = node.primitiveId();

    const Triangle& tri_id = tri_indices[primitive_id];

    const Vec3f& p1 = vertices[tri_id[0]];
    const Vec3f& p2 = vertices[tri_id[1]];
    const Vec3f& p3 = vertices[tri_id[2]];

    if(this->model1->isOccupied() && this->model2->isOccupied())
    {
      bool is_intersect = false;

      if(!this->request.enable_contact)
      {
        if(nsolver->shapeTriangleIntersect(*(this->model2), this->tf2, p1, p2, p3, NULL, NULL, NULL))
        {
          is_intersect = true;
          if(this->request.num_max_contacts > this->result->numContacts())
            this->result->addContact(Contact(this->model1, this->model2, primitive_id, Contact::NONE));
        }
      }
      else
      {
        FCL_REAL penetration;
        Vec3f normal;
        Vec3f contactp;

        if(nsolver->shapeTriangleIntersect(*(this->model2), this->tf2, p1, p2, p3, &contactp, &penetration, &normal))
        {
          is_intersect = true;
          if(this->request.num_max_contacts > this->result->numContacts())
            this->result->addContact(Contact(this->model1, this->model2, primitive_id, Contact::NONE, contactp, -normal, penetration));
        }
      }

      if(is_intersect && this->request.enable_cost)
      {
        AABB overlap_part;
        AABB shape_aabb;
        computeBV<AABB, S>(*(this->model2), this->tf2, shape_aabb);
        AABB(p1, p2, p3).overlap(shape_aabb, overlap_part);
        this->result->addCostSource(CostSource(overlap_part, cost_density), this->request.num_max_cost_sources);
      }
    }
    if((!this->model1->isFree() && !this->model2->isFree()) && this->request.enable_cost)
    {
      if(nsolver->shapeTriangleIntersect(*(this->model2), this->tf2, p1, p2, p3, NULL, NULL, NULL))
      {
        AABB overlap_part;
        AABB shape_aabb;
        computeBV<AABB, S>(*(this->model2), this->tf2, shape_aabb);
        AABB(p1, p2, p3).overlap(shape_aabb, overlap_part);
        this->result->addCostSource(CostSource(overlap_part, cost_density), this->request.num_max_cost_sources);        
      }
    }
  }

  /// @brief Whether the traversal process can stop early
  bool canStop() const
  {
    return this->request.isSatisfied(*(this->result));
  }

  Vec3f* vertices;
  Triangle* tri_indices;
  
  FCL_REAL cost_density;

  const NarrowPhaseSolver* nsolver;
};

/// @cond IGNORE
namespace details
{
template<typename BV, typename S, typename NarrowPhaseSolver>
static inline void meshShapeCollisionOrientedNodeLeafTesting(int b1, int b2,
                                                             const BVHModel<BV>* model1, const S& model2,
                                                             Vec3f* vertices, Triangle* tri_indices,
                                                             const Transform3f& tf1,
                                                             const Transform3f& tf2, 
                                                             const NarrowPhaseSolver* nsolver,
                                                             bool enable_statistics, 
                                                             FCL_REAL cost_density,
                                                             int& num_leaf_tests,
                                                             const CollisionRequest& request,
                                                             CollisionResult& result)
{
  if(enable_statistics) num_leaf_tests++;
  const BVNode<BV>& node = model1->getBV(b1);

  int primitive_id = node.primitiveId();

  const Triangle& tri_id = tri_indices[primitive_id];

  const Vec3f& p1 = vertices[tri_id[0]];
  const Vec3f& p2 = vertices[tri_id[1]];
  const Vec3f& p3 = vertices[tri_id[2]];

  if(model1->isOccupied() && model2.isOccupied())
  {
    bool is_intersect = false;

    if(!request.enable_contact) // only interested in collision or not
    {
      if(nsolver->shapeTriangleIntersect(model2, tf2, p1, p2, p3, tf1, NULL, NULL, NULL))
      {
        is_intersect = true;
        if(request.num_max_contacts > result.numContacts())
          result.addContact(Contact(model1, &model2, primitive_id, Contact::NONE));
      }
    }
    else
    {
      FCL_REAL penetration;
      Vec3f normal;
      Vec3f contactp;

      if(nsolver->shapeTriangleIntersect(model2, tf2, p1, p2, p3, tf1, &contactp, &penetration, &normal))
      {
        is_intersect = true;
        if(request.num_max_contacts > result.numContacts())
          result.addContact(Contact(model1, &model2, primitive_id, Contact::NONE, contactp, -normal, penetration));
      }
    }

    if(is_intersect && request.enable_cost)
    {
      AABB overlap_part;
      AABB shape_aabb;
      computeBV<AABB, S>(model2, tf2, shape_aabb);
      bool res = AABB(tf1.transform(p1), tf1.transform(p2), tf1.transform(p3)).overlap(shape_aabb, overlap_part);
      result.addCostSource(CostSource(overlap_part, cost_density), request.num_max_cost_sources);
    }
  }
  else if((!model1->isFree() || model2.isFree()) && request.enable_cost)
  {
    if(nsolver->shapeTriangleIntersect(model2, tf2, p1, p2, p3, tf1, NULL, NULL, NULL))
    {
      AABB overlap_part;
      AABB shape_aabb;
      computeBV<AABB, S>(model2, tf2, shape_aabb);
      bool res = AABB(tf1.transform(p1), tf1.transform(p2), tf1.transform(p3)).overlap(shape_aabb, overlap_part);
      result.addCostSource(CostSource(overlap_part, cost_density), request.num_max_cost_sources);    
    }
  }
}

}

/// @endcond


/// @brief Traversal node for mesh and shape, when mesh BVH is one of the oriented node (OBB, RSS, OBBRSS, kIOS)
template<typename S, typename NarrowPhaseSolver>
class MeshShapeCollisionTraversalNodeOBB : public MeshShapeCollisionTraversalNode<OBB, S, NarrowPhaseSolver>
{
public:
  MeshShapeCollisionTraversalNodeOBB() : MeshShapeCollisionTraversalNode<OBB, S, NarrowPhaseSolver>()
  {
  }

  bool BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return !overlap(this->tf1.getRotation(), this->tf1.getTranslation(), this->model2_bv, this->model1->getBV(b1).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeCollisionOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                       this->tf1, this->tf2, this->nsolver, this->enable_statistics, this->cost_density, this->num_leaf_tests, this->request, *(this->result));
  }

};

template<typename S, typename NarrowPhaseSolver>
class MeshShapeCollisionTraversalNodeRSS : public MeshShapeCollisionTraversalNode<RSS, S, NarrowPhaseSolver>
{
public:
  MeshShapeCollisionTraversalNodeRSS() : MeshShapeCollisionTraversalNode<RSS, S, NarrowPhaseSolver>()
  {
  }

  bool BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return !overlap(this->tf1.getRotation(), this->tf1.getTranslation(), this->model2_bv, this->model1->getBV(b1).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeCollisionOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                       this->tf1, this->tf2, this->nsolver, this->enable_statistics, this->cost_density, this->num_leaf_tests, this->request, *(this->result));
  }

};

template<typename S, typename NarrowPhaseSolver>
class MeshShapeCollisionTraversalNodekIOS : public MeshShapeCollisionTraversalNode<kIOS, S, NarrowPhaseSolver>
{
public:
  MeshShapeCollisionTraversalNodekIOS() : MeshShapeCollisionTraversalNode<kIOS, S, NarrowPhaseSolver>()
  {
  }

  bool BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return !overlap(this->tf1.getRotation(), this->tf1.getTranslation(), this->model2_bv, this->model1->getBV(b1).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeCollisionOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                       this->tf1, this->tf2, this->nsolver, this->enable_statistics, this->cost_density, this->num_leaf_tests, this->request, *(this->result));
  }

};

template<typename S, typename NarrowPhaseSolver>
class MeshShapeCollisionTraversalNodeOBBRSS : public MeshShapeCollisionTraversalNode<OBBRSS, S, NarrowPhaseSolver>
{
public:
  MeshShapeCollisionTraversalNodeOBBRSS() : MeshShapeCollisionTraversalNode<OBBRSS, S, NarrowPhaseSolver>()
  {
  }

  bool BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return !overlap(this->tf1.getRotation(), this->tf1.getTranslation(), this->model2_bv, this->model1->getBV(b1).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeCollisionOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                       this->tf1, this->tf2, this->nsolver, this->enable_statistics, this->cost_density, this->num_leaf_tests, this->request, *(this->result));
  }

};


/// @brief Traversal node for collision between shape and mesh
template<typename S, typename BV, typename NarrowPhaseSolver>
class ShapeMeshCollisionTraversalNode : public ShapeBVHCollisionTraversalNode<S, BV>
{
public:
  ShapeMeshCollisionTraversalNode() : ShapeBVHCollisionTraversalNode<S, BV>()
  {
    vertices = NULL;
    tri_indices = NULL;

    nsolver = NULL;
  }

  /// @brief Intersection testing between leaves (one shape and one triangle)
  void leafTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_leaf_tests++;
    const BVNode<BV>& node = this->model2->getBV(b2);

    int primitive_id = node.primitiveId();

    const Triangle& tri_id = tri_indices[primitive_id];

    const Vec3f& p1 = vertices[tri_id[0]];
    const Vec3f& p2 = vertices[tri_id[1]];
    const Vec3f& p3 = vertices[tri_id[2]];

    if(this->model1->isOccupied() && this->model2->isOccupied())
    {
      bool is_intersect = false;

      if(!this->request.enable_contact)
      {
        if(nsolver->shapeTriangleIntersect(*(this->model1), this->tf1, p1, p2, p3, NULL, NULL, NULL))
        {
          is_intersect = true;
          if(this->request.num_max_contacts > this->result->numContacts())
            this->result->addContact(Contact(this->model1, this->model2, Contact::NONE, primitive_id));
        }
      }
      else
      {
        FCL_REAL penetration;
        Vec3f normal;
        Vec3f contactp;

        if(nsolver->shapeTriangleIntersect(*(this->model1), this->tf1, p1, p2, p3, &contactp, &penetration, &normal))
        {
          is_intersect = true;
          if(this->request.num_max_contacts > this->result->numContacts())
            this->result->addContact(Contact(this->model1, this->model2, Contact::NONE, primitive_id, contactp, normal, penetration));
        }
      }

      if(is_intersect && this->request.enable_cost)
      {
        AABB overlap_part;
        AABB shape_aabb;
        computeBV<AABB, S>(*(this->model1), this->tf1, shape_aabb);
        AABB(p1, p2, p3).overlap(shape_aabb, overlap_part);
        this->result->addCostSource(CostSource(overlap_part, cost_density), this->request.num_max_cost_sources);
      }
    }
    else if((!this->model1->isFree() && !this->model2->isFree()) && this->request.enable_cost)
    {
      if(nsolver->shapeTriangleIntersect(*(this->model1), this->tf1, p1, p2, p3, NULL, NULL, NULL))
      {
        AABB overlap_part;
        AABB shape_aabb;
        computeBV<AABB, S>(*(this->model1), this->tf1, shape_aabb);
        AABB(p1, p2, p3).overlap(shape_aabb, overlap_part);
        this->result->addCostSource(CostSource(overlap_part, cost_density), this->request.num_max_cost_sources);
      }   
    }
  }

  /// @brief Whether the traversal process can stop early
  bool canStop() const
  {
    return this->request.isSatisfied(*(this->result));
  }

  Vec3f* vertices;
  Triangle* tri_indices;

  FCL_REAL cost_density;

  const NarrowPhaseSolver* nsolver;
};

/// @brief Traversal node for shape and mesh, when mesh BVH is one of the oriented node (OBB, RSS, OBBRSS, kIOS)
template<typename S, typename NarrowPhaseSolver>
class ShapeMeshCollisionTraversalNodeOBB : public ShapeMeshCollisionTraversalNode<S, OBB, NarrowPhaseSolver>
{
public:
  ShapeMeshCollisionTraversalNodeOBB() : ShapeMeshCollisionTraversalNode<S, OBB, NarrowPhaseSolver>()
  {
  }

  bool BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return !overlap(this->tf2.getRotation(), this->tf2.getTranslation(), this->model1_bv, this->model2->getBV(b2).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeCollisionOrientedNodeLeafTesting(b2, b1, *(this->model2), this->model1, this->vertices, this->tri_indices, 
                                                       this->tf2, this->tf1, this->nsolver, this->enable_statistics, this->cost_density, this->num_leaf_tests, this->request, *(this->request));

    // may need to change the order in pairs
  }
};


template<typename S, typename NarrowPhaseSolver>
class ShapeMeshCollisionTraversalNodeRSS : public ShapeMeshCollisionTraversalNode<S, RSS, NarrowPhaseSolver>
{
public:
  ShapeMeshCollisionTraversalNodeRSS() : ShapeMeshCollisionTraversalNode<S, RSS, NarrowPhaseSolver>()
  {
  }

  bool BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return !overlap(this->tf2.getRotation(), this->tf2.getTranslation(), this->model1_bv, this->model2->getBV(b2).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeCollisionOrientedNodeLeafTesting(b2, b1, *(this->model2), this->model1, this->vertices, this->tri_indices, 
                                                       this->tf2, this->tf1, this->nsolver, this->enable_statistics, this->cost_density, this->num_leaf_tests, this->request, *(this->request));

    // may need to change the order in pairs
  }

};


template<typename S, typename NarrowPhaseSolver>
class ShapeMeshCollisionTraversalNodekIOS : public ShapeMeshCollisionTraversalNode<S, kIOS, NarrowPhaseSolver>
{
public:
  ShapeMeshCollisionTraversalNodekIOS() : ShapeMeshCollisionTraversalNode<S, kIOS, NarrowPhaseSolver>()
  {
  }

  bool BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return !overlap(this->tf2.getRotation(), this->tf2.getTranslation(), this->model1_bv, this->model2->getBV(b2).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeCollisionOrientedNodeLeafTesting(b2, b1, *(this->model2), this->model1, this->vertices, this->tri_indices, 
                                                       this->tf2, this->tf1, this->nsolver, this->enable_statistics, this->cost_density, this->num_leaf_tests, this->request, *(this->request));

    // may need to change the order in pairs
  }

};


template<typename S, typename NarrowPhaseSolver>
class ShapeMeshCollisionTraversalNodeOBBRSS : public ShapeMeshCollisionTraversalNode<S, OBBRSS, NarrowPhaseSolver>
{
public:
  ShapeMeshCollisionTraversalNodeOBBRSS() : ShapeMeshCollisionTraversalNode<S, OBBRSS, NarrowPhaseSolver>()
  {
  }

  bool BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return !overlap(this->tf2.getRotation(), this->tf2.getTranslation(), this->model1_bv, this->model2->getBV(b2).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeCollisionOrientedNodeLeafTesting(b2, b1, *(this->model2), this->model1, this->vertices, this->tri_indices, 
                                                       this->tf2, this->tf1, this->nsolver, this->enable_statistics, this->cost_density, this->num_leaf_tests, this->request, *(this->request));

    // may need to change the order in pairs
  }

};

/// @brief Traversal node for distance computation between BVH and shape
template<typename BV, typename S>
class BVHShapeDistanceTraversalNode : public DistanceTraversalNodeBase
{
public:
  BVHShapeDistanceTraversalNode() : DistanceTraversalNodeBase()
  {
    model1 = NULL;
    model2 = NULL;
    
    num_bv_tests = 0;
    num_leaf_tests = 0;
    query_time_seconds = 0.0;
  }

  /// @brief Whether the BV node in the first BVH tree is leaf
  bool isFirstNodeLeaf(int b) const 
  {
    return model1->getBV(b).isLeaf();
  }

  /// @brief Obtain the left child of BV node in the first BVH
  int getFirstLeftChild(int b) const
  {
    return model1->getBV(b).leftChild();
  }

  /// @brief Obtain the right child of BV node in the first BVH
  int getFirstRightChild(int b) const
  {
    return model1->getBV(b).rightChild();
  }

  /// @brief BV culling test in one BVTT node
  FCL_REAL BVTesting(int b1, int b2) const
  {
    return model1->getBV(b1).bv.distance(model2_bv);
  }

  const BVHModel<BV>* model1;
  const S* model2;
  BV model2_bv;

  mutable int num_bv_tests;
  mutable int num_leaf_tests;
  mutable FCL_REAL query_time_seconds;
};

/// @brief Traversal node for distance computation between shape and BVH
template<typename S, typename BV>
class ShapeBVHDistanceTraversalNode : public DistanceTraversalNodeBase
{
public:
  ShapeBVHDistanceTraversalNode() : DistanceTraversalNodeBase()
  {
    model1 = NULL;
    model2 = NULL;
    
    num_bv_tests = 0;
    num_leaf_tests = 0;
    query_time_seconds = 0.0;
  }

  /// @brief Whether the BV node in the second BVH tree is leaf
  bool isSecondNodeLeaf(int b) const
  {
    return model2->getBV(b).isLeaf();
  }

  /// @brief Obtain the left child of BV node in the second BVH
  int getSecondLeftChild(int b) const
  {
    return model2->getBV(b).leftChild();
  }

  /// @brief Obtain the right child of BV node in the second BVH
  int getSecondRightChild(int b) const
  {
    return model2->getBV(b).rightChild();
  }

  /// @brief BV culling test in one BVTT node
  FCL_REAL BVTesting(int b1, int b2) const
  {
    return model1_bv.distance(model2->getBV(b2).bv);
  }

  const S* model1;
  const BVHModel<BV>* model2;
  BV model1_bv;
  
  mutable int num_bv_tests;
  mutable int num_leaf_tests;
  mutable FCL_REAL query_time_seconds;
};
                                  

/// @brief Traversal node for distance between mesh and shape
template<typename BV, typename S, typename NarrowPhaseSolver>
class MeshShapeDistanceTraversalNode : public BVHShapeDistanceTraversalNode<BV, S>
{ 
public:
  MeshShapeDistanceTraversalNode() : BVHShapeDistanceTraversalNode<BV, S>()
  {
    vertices = NULL;
    tri_indices = NULL;

    rel_err = 0;
    abs_err = 0;

    nsolver = NULL;
  }

  /// @brief Distance testing between leaves (one triangle and one shape)
  void leafTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_leaf_tests++;
    
    const BVNode<BV>& node = this->model1->getBV(b1);
    
    int primitive_id = node.primitiveId();
    
    const Triangle& tri_id = tri_indices[primitive_id];

    const Vec3f& p1 = vertices[tri_id[0]];
    const Vec3f& p2 = vertices[tri_id[1]];
    const Vec3f& p3 = vertices[tri_id[2]];
    
    FCL_REAL d;
    nsolver->shapeTriangleDistance(*(this->model2), this->tf2, p1, p2, p3, &d);

    this->result->update(d, this->model1, this->model2, primitive_id, DistanceResult::NONE);
  }

  /// @brief Whether the traversal process can stop early
  bool canStop(FCL_REAL c) const
  {
    if((c >= this->result->min_distance - abs_err) && (c * (1 + rel_err) >= this->result->min_distance))
      return true;
    return false;
  }

  Vec3f* vertices;
  Triangle* tri_indices;

  FCL_REAL rel_err;
  FCL_REAL abs_err;
    
  const NarrowPhaseSolver* nsolver;
};

/// @cond IGNORE
namespace details
{

template<typename BV, typename S, typename NarrowPhaseSolver>
void meshShapeDistanceOrientedNodeLeafTesting(int b1, int b2,
                                              const BVHModel<BV>* model1, const S& model2,
                                              Vec3f* vertices, Triangle* tri_indices,
                                              const Transform3f& tf1,
                                              const Transform3f& tf2,
                                              const NarrowPhaseSolver* nsolver,
                                              bool enable_statistics,
                                              int & num_leaf_tests,
                                              const DistanceRequest& request,
                                              DistanceResult& result)
{
  if(enable_statistics) num_leaf_tests++;
    
  const BVNode<BV>& node = model1->getBV(b1);
  int primitive_id = node.primitiveId();

  const Triangle& tri_id = tri_indices[primitive_id];
  const Vec3f& p1 = vertices[tri_id[0]];
  const Vec3f& p2 = vertices[tri_id[1]];
  const Vec3f& p3 = vertices[tri_id[2]];
    
  FCL_REAL distance;
  nsolver->shapeTriangleDistance(model2, tf2, p1, p2, p3, tf1, &distance);

  result.update(distance, model1, &model2, primitive_id, DistanceResult::NONE);
}


template<typename BV, typename S, typename NarrowPhaseSolver>
static inline void distancePreprocessOrientedNode(const BVHModel<BV>* model1,
                                                  Vec3f* vertices, Triangle* tri_indices, int init_tri_id,
                                                  const S& model2, const Transform3f& tf1, const Transform3f& tf2,
                                                  const NarrowPhaseSolver* nsolver,
                                                  const DistanceRequest& request,
                                                  DistanceResult& result)
{
  const Triangle& init_tri = tri_indices[init_tri_id];
  
  const Vec3f& p1 = vertices[init_tri[0]];
  const Vec3f& p2 = vertices[init_tri[1]];
  const Vec3f& p3 = vertices[init_tri[2]];
  
  FCL_REAL distance;
  nsolver->shapeTriangleDistance(model2, tf2, p1, p2, p3, tf1, &distance);

  result.update(distance, model1, &model2, init_tri_id, DistanceResult::NONE);
}


}

/// @endcond



/// @brief Traversal node for distance between mesh and shape, when mesh BVH is one of the oriented node (RSS, OBBRSS, kIOS)
template<typename S, typename NarrowPhaseSolver>
class MeshShapeDistanceTraversalNodeRSS : public MeshShapeDistanceTraversalNode<RSS, S, NarrowPhaseSolver>
{
public:
  MeshShapeDistanceTraversalNodeRSS() : MeshShapeDistanceTraversalNode<RSS, S, NarrowPhaseSolver>()
  {
  }

  void preprocess()
  {
    details::distancePreprocessOrientedNode(this->model1, this->vertices, this->tri_indices, 0, 
                                            *(this->model2), this->tf1, this->tf2, this->nsolver, this->request, *(this->result));
  }

  void postprocess()
  {
  }

  FCL_REAL BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return distance(this->tf1.getRotation(), this->tf1.getTranslation(), this->model2_bv, this->model1->getBV(b1).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeDistanceOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                      this->tf1, this->tf2, this->nsolver, this->enable_statistics, this->num_leaf_tests, this->request, *(this->result));
  }
};


template<typename S, typename NarrowPhaseSolver>
class MeshShapeDistanceTraversalNodekIOS : public MeshShapeDistanceTraversalNode<kIOS, S, NarrowPhaseSolver>
{
public:
  MeshShapeDistanceTraversalNodekIOS() : MeshShapeDistanceTraversalNode<kIOS, S, NarrowPhaseSolver>()
  {
  }

  void preprocess()
  {
    details::distancePreprocessOrientedNode(this->model1, this->vertices, this->tri_indices, 0, 
                                            *(this->model2), this->tf1, this->tf2, this->nsolver, this->request, *(this->result));
  }

  void postprocess()
  {    
  }

  FCL_REAL BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return distance(this->tf1.getRotation(), this->tf1.getTranslation(), this->model2_bv, this->model1->getBV(b1).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeDistanceOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                      this->tf1, this->tf2, this->nsolver, this->enable_statistics, this->num_leaf_tests, this->request, *(this->result));
  }

};

template<typename S, typename NarrowPhaseSolver>
class MeshShapeDistanceTraversalNodeOBBRSS : public MeshShapeDistanceTraversalNode<OBBRSS, S, NarrowPhaseSolver>
{
public:
  MeshShapeDistanceTraversalNodeOBBRSS() : MeshShapeDistanceTraversalNode<OBBRSS, S, NarrowPhaseSolver>()
  {
  }

  void preprocess()
  {
    details::distancePreprocessOrientedNode(this->model1, this->vertices, this->tri_indices, 0, 
                                            *(this->model2), this->tf1, this->tf2, this->nsolver, this->request, *(this->result));
  }

  void postprocess()
  {
    
  }

  FCL_REAL BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return distance(this->tf1.getRotation(), this->tf1.getTranslation(), this->model2_bv, this->model1->getBV(b1).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeDistanceOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                      this->tf1, this->tf2, this->nsolver, this->enable_statistics, this->num_leaf_tests, this->request, *(this->result));
  }
  
};

/// @brief Traversal node for distance between shape and mesh
template<typename S, typename BV, typename NarrowPhaseSolver>
class ShapeMeshDistanceTraversalNode : public ShapeBVHDistanceTraversalNode<S, BV>
{ 
public:
  ShapeMeshDistanceTraversalNode() : ShapeBVHDistanceTraversalNode<S, BV>()
  {
    vertices = NULL;
    tri_indices = NULL;

    rel_err = 0;
    abs_err = 0;

    nsolver = NULL;
  }

  /// @brief Distance testing between leaves (one shape and one triangle)
  void leafTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_leaf_tests++;
    
    const BVNode<BV>& node = this->model2->getBV(b2);
    
    int primitive_id = node.primitiveId();
    
    const Triangle& tri_id = tri_indices[primitive_id];

    const Vec3f& p1 = vertices[tri_id[0]];
    const Vec3f& p2 = vertices[tri_id[1]];
    const Vec3f& p3 = vertices[tri_id[2]];
    
    FCL_REAL distance;
    nsolver->shapeTriangleDistance(*(this->model1), this->tf1, p1, p2, p3, &distance);

    this->result->update(distance, this->model1, this->model2, DistanceResult::NONE, primitive_id);
  }

  /// @brief Whether the traversal process can stop early
  bool canStop(FCL_REAL c) const
  {
    if((c >= this->result->min_distance - abs_err) && (c * (1 + rel_err) >= this->result->min_distance))
      return true;
    return false;
  }

  Vec3f* vertices;
  Triangle* tri_indices;

  FCL_REAL rel_err;
  FCL_REAL abs_err;
    
  const NarrowPhaseSolver* nsolver;
};

template<typename S, typename NarrowPhaseSolver>
class ShapeMeshDistanceTraversalNodeRSS : public ShapeMeshDistanceTraversalNode<S, RSS, NarrowPhaseSolver>
{
public:
  ShapeMeshDistanceTraversalNodeRSS() : ShapeMeshDistanceTraversalNode<S, RSS, NarrowPhaseSolver>()
  {
  }

  void preprocess()
  {
    details::distancePreprocessOrientedNode(this->model2, this->vertices, this->tri_indices, 0,
                                            *(this->model1), this->tf2, this->tf1, this->nsolver, this->request, *(this->result));
  }

  void postprocess()
  {
  }

  FCL_REAL BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return distance(this->tf2.getRotation(), this->tf2.getTranslation(), this->model1_bv, this->model2->getBV(b2).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeDistanceOrientedNodeLeafTesting(b2, b1, this->model2, *(this->model1), this->vertices, this->tri_indices,
                                                      this->tf2, this->tf1, this->nsolver, this->enable_statistics, this->num_leaf_tests, this->request, *(this->result));
  }

};

template<typename S, typename NarrowPhaseSolver>
class ShapeMeshDistanceTraversalNodekIOS : public ShapeMeshDistanceTraversalNode<S, kIOS, NarrowPhaseSolver>
{
public:
  ShapeMeshDistanceTraversalNodekIOS() : ShapeMeshDistanceTraversalNode<S, kIOS, NarrowPhaseSolver>()
  {
  }

  void preprocess()
  {
    details::distancePreprocessOrientedNode(this->model2, this->vertices, this->tri_indices, 0,
                                            *(this->model1), this->tf2, this->tf1, this->nsolver, *(this->result));
  }

  void postprocess()
  {
  }

  FCL_REAL BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return distance(this->tf2.getRotation(), this->tf2.getTranslation(), this->model1_bv, this->model2->getBV(b2).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeDistanceOrientedNodeLeafTesting(b2, b1, this->model2, *(this->model1), this->vertices, this->tri_indices,
                                                      this->tf2, this->tf1, this->nsolver, this->enable_statistics, this->num_leaf_tests, this->request, *(this->result));
  }
  
};

template<typename S, typename NarrowPhaseSolver>
class ShapeMeshDistanceTraversalNodeOBBRSS : public ShapeMeshDistanceTraversalNode<S, OBBRSS, NarrowPhaseSolver>
{
public:
  ShapeMeshDistanceTraversalNodeOBBRSS() : ShapeMeshDistanceTraversalNode<S, OBBRSS, NarrowPhaseSolver>()
  {
  }

  void preprocess()
  {
    details::distancePreprocessOrientedNode(this->model2, this->vertices, this->tri_indices, 0,
                                            *(this->model1), this->tf2, this->tf1, this->nsolver, *(this->result));
  }

  void postprocess()
  {    
  }

  FCL_REAL BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return distance(this->tf2.getRotation(), this->tf2.getTranslation(), this->model1_bv, this->model2->getBV(b2).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeDistanceOrientedNodeLeafTesting(b2, b1, this->model2, *(this->model1), this->vertices, this->tri_indices,
                                                      this->tf2, this->tf1, this->nsolver, this->enable_statistics, this->num_leaf_tests, this->request, *(this->result));
  }
  
};
}

#endif

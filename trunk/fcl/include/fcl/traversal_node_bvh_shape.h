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

#include "fcl/geometric_shapes.h"
#include "fcl/traversal_node_base.h"
#include "fcl/BVH_model.h"
#include "fcl/geometric_shapes_utility.h"

namespace fcl
{

template<typename BV, typename S>
class BVHShapeCollisionTraversalNode : public CollisionTraversalNodeBase
{
public:
  BVHShapeCollisionTraversalNode()
  {
    model1 = NULL;
    model2 = NULL;

    num_bv_tests = 0;
    num_leaf_tests = 0;
    query_time_seconds = 0.0;
  }

  bool isFirstNodeLeaf(int b) const
  {
    return model1->getBV(b).isLeaf();
  }

  int getFirstLeftChild(int b) const
  {
    return model1->getBV(b).leftChild();
  }

  int getFirstRightChild(int b) const
  {
    return model1->getBV(b).rightChild();
  }

  bool BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) num_bv_tests++;
    BV bv_shape;
    computeBV(*model2, bv_shape);
    return !model1->getBV(b1).bv.overlap(bv_shape);
  }

  const BVHModel<BV>* model1;
  const S* model2;

  mutable int num_bv_tests;
  mutable int num_leaf_tests;
  mutable BVH_REAL query_time_seconds;
};


template<typename S, typename BV>
class ShapeBVHCollisionTraversalNode : public CollisionTraversalNodeBase
{
public:
  ShapeBVHCollisionTraversalNode()
  {
    model1 = NULL;
    model2 = NULL;

    num_bv_tests = 0;
    num_leaf_tests = 0;
    query_time_seconds = 0.0;
  }

  bool firstOverSecond(int, int) const
  {
    return false;
  }

  bool isSecondNodeLeaf(int b) const
  {
    return model2->getBV(b).isLeaf();
  }

  int getSecondLeftChild(int b) const
  {
    return model2->getBV(b).leftChild();
  }

  int getSecondRightChild(int b) const
  {
    return model2->getBV(b).rightChild();
  }

  bool BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) num_bv_tests++;
    BV bv_shape;
    computeBV(*model1, bv_shape);
    return !model2->getBV(b2).bv.overlap(bv_shape);
  }

  const S* model1;
  const BVHModel<BV>* model2;

  mutable int num_bv_tests;
  mutable int num_leaf_tests;
  mutable BVH_REAL query_time_seconds;
};



/** \brief The indices of in-collision primitives of objects */
struct BVHShapeCollisionPair
{
  BVHShapeCollisionPair() {}

  BVHShapeCollisionPair(int id_) : id(id_) {}

  BVHShapeCollisionPair(int id_, const Vec3f& n, const Vec3f& contactp, BVH_REAL depth) : id(id_),
      normal(n), contact_point(contactp), penetration_depth(depth) {}

  /** \brief The index of BVH's in-collision primitive */
  int id;

  /** \brief Contact normal */
  Vec3f normal;

  /** \brief Contact points */
  Vec3f contact_point;

  /** \brief Penetration depth for two triangles */
  BVH_REAL penetration_depth;
};

struct BVHShapeCollisionPairComp
{
  bool operator()(const BVHShapeCollisionPair& a, const BVHShapeCollisionPair& b)
  {
    return a.id < b.id;
  }
};


template<typename BV, typename S>
class MeshShapeCollisionTraversalNode : public BVHShapeCollisionTraversalNode<BV, S>
{
public:
  MeshShapeCollisionTraversalNode() : BVHShapeCollisionTraversalNode<BV, S>()
  {
    vertices = NULL;
    tri_indices = NULL;

    num_max_contacts = 1;
    exhaustive = false;
    enable_contact = false;
  }

  void leafTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_leaf_tests++;
    const BVNode<BV>& node = this->model1->getBV(b1);

    int primitive_id = node.primitiveId();

    const Triangle& tri_id = tri_indices[primitive_id];

    const Vec3f& p1 = vertices[tri_id[0]];
    const Vec3f& p2 = vertices[tri_id[1]];
    const Vec3f& p3 = vertices[tri_id[2]];

    BVH_REAL penetration;
    Vec3f normal;
    Vec3f contactp;

    if(!enable_contact) // only interested in collision or not
    {
      if(shapeTriangleIntersect(*(this->model2), p1, p2, p3))
      {
        pairs.push_back(BVHShapeCollisionPair(primitive_id));
      }
    }
    else
    {
      if(shapeTriangleIntersect(*(this->model2), p1, p2, p3, &contactp, &penetration, &normal))
      {
        pairs.push_back(BVHShapeCollisionPair(primitive_id, normal, contactp, penetration));
      }
    }
  }

  bool canStop() const
  {
    return (pairs.size() > 0) && (!exhaustive) && (num_max_contacts <= (int)pairs.size());
  }

  Vec3f* vertices;
  Triangle* tri_indices;

  int num_max_contacts;
  bool exhaustive;
  bool enable_contact;

  mutable std::vector<BVHShapeCollisionPair> pairs;
};

template<typename S>
class MeshShapeCollisionTraversalNodeOBB : public MeshShapeCollisionTraversalNode<OBB, S>
{
public:
  MeshShapeCollisionTraversalNodeOBB() : MeshShapeCollisionTraversalNode<OBB, S>()
  {
    R[0] = Vec3f(1, 0, 0);
    R[1] = Vec3f(0, 1, 0);
    R[2] = Vec3f(0, 0, 1);
  }

  bool BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    OBB bv_shape;
    computeBV(*this->model2, bv_shape);
    return !overlap(R, T, bv_shape, this->model1->getBV(b1).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_leaf_tests++;
    const BVNode<OBB>& node = this->model1->getBV(b1);

    int primitive_id = node.primitiveId();

    const Triangle& tri_id = this->tri_indices[primitive_id];

    const Vec3f& p1 = this->vertices[tri_id[0]];
    const Vec3f& p2 = this->vertices[tri_id[1]];
    const Vec3f& p3 = this->vertices[tri_id[2]];

    BVH_REAL penetration;
    Vec3f normal;
    Vec3f contactp;

    if(!this->enable_contact) // only interested in collision or not
    {
      if(shapeTriangleIntersect(*(this->model2), p1, p2, p3, R, T))
      {
        this->pairs.push_back(BVHShapeCollisionPair(primitive_id));
      }
    }
    else
    {
      if(shapeTriangleIntersect(*(this->model2), p1, p2, p3, R, T, &contactp, &penetration, &normal))
      {
        this->pairs.push_back(BVHShapeCollisionPair(primitive_id, normal, contactp, penetration));
      }
    }
  }

  // R, T is the transformation of bvh model
  Vec3f R[3];
  Vec3f T;
};


template<typename S, typename BV>
class ShapeMeshCollisionTraversalNode : public ShapeBVHCollisionTraversalNode<S, BV>
{
public:
  ShapeMeshCollisionTraversalNode() : ShapeBVHCollisionTraversalNode<S, BV>()
  {
    vertices = NULL;
    tri_indices = NULL;

    num_max_contacts = 1;
    exhaustive = false;
    enable_contact = false;
  }

  void leafTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_leaf_tests++;
    const BVNode<BV>& node = this->model2->getBV(b2);

    int primitive_id = node.primitiveId();

    const Triangle& tri_id = tri_indices[primitive_id];

    const Vec3f& p1 = vertices[tri_id[0]];
    const Vec3f& p2 = vertices[tri_id[1]];
    const Vec3f& p3 = vertices[tri_id[2]];

    BVH_REAL penetration;
    Vec3f normal;
    Vec3f contactp;

    if(!enable_contact) // only interested in collision or not
    {
      if(shapeTriangleIntersect(*(this->model1), p1, p2, p3))
      {
        pairs.push_back(BVHShapeCollisionPair(primitive_id));
      }
    }
    else
    {
      if(shapeTriangleIntersect(*(this->model1), p1, p2, p3, &contactp, &penetration, &normal))
      {
        pairs.push_back(BVHShapeCollisionPair(primitive_id, normal, contactp, penetration));
      }
    }
  }

  bool canStop() const
  {
    return (pairs.size() > 0) && (!exhaustive) && (num_max_contacts <= (int)pairs.size());
  }

  Vec3f* vertices;
  Triangle* tri_indices;

  int num_max_contacts;
  bool exhaustive;
  bool enable_contact;

  mutable std::vector<BVHShapeCollisionPair> pairs;
};

template<typename S>
class ShapeMeshCollisionTraversalNodeOBB : public ShapeMeshCollisionTraversalNode<S, OBB>
{
public:
  ShapeMeshCollisionTraversalNodeOBB() : ShapeMeshCollisionTraversalNode<S, OBB>()
  {
    R[0] = Vec3f(1, 0, 0);
    R[1] = Vec3f(0, 1, 0);
    R[2] = Vec3f(0, 0, 1);
  }

  bool BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    OBB bv_shape;
    computeBV(*this->model1, bv_shape);
    return !overlap(R, T, bv_shape, this->model2->getBV(b2).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_leaf_tests++;
    const BVNode<OBB>& node = this->model2->getBV(b2);

    int primitive_id = node.primitiveId();

    const Triangle& tri_id = this->tri_indices[primitive_id];

    const Vec3f& p1 = this->vertices[tri_id[0]];
    const Vec3f& p2 = this->vertices[tri_id[1]];
    const Vec3f& p3 = this->vertices[tri_id[2]];

    BVH_REAL penetration;
    Vec3f normal;
    Vec3f contactp;

    if(!this->enable_contact) // only interested in collision or not
    {
      if(shapeTriangleIntersect(*(this->model1), p1, p2, p3, R, T))
      {
        this->pairs.push_back(BVHShapeCollisionPair(primitive_id));
      }
    }
    else
    {
      if(shapeTriangleIntersect(*(this->model1), p1, p2, p3, R, T, &contactp, &penetration, &normal))
      {
        this->pairs.push_back(BVHShapeCollisionPair(primitive_id, normal, contactp, penetration));
      }
    }
  }

  // R, T is the transformation of bvh model
  Vec3f R[3];
  Vec3f T;
};




}

#endif

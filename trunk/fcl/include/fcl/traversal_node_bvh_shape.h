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
  BVHShapeCollisionTraversalNode() : CollisionTraversalNodeBase()
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
    return !model1->getBV(b1).bv.overlap(model2_bv);
  }

  const BVHModel<BV>* model1;
  const S* model2;
  BV model2_bv;

  mutable int num_bv_tests;
  mutable int num_leaf_tests;
  mutable FCL_REAL query_time_seconds;
};


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
    return !model2->getBV(b2).bv.overlap(model1_bv);
  }

  const S* model1;
  const BVHModel<BV>* model2;
  BV model1_bv;

  mutable int num_bv_tests;
  mutable int num_leaf_tests;
  mutable FCL_REAL query_time_seconds;
};



/** \brief The indices of in-collision primitives of objects */
struct BVHShapeCollisionPair
{
  BVHShapeCollisionPair() {}

  BVHShapeCollisionPair(int id_) : id(id_) {}

  BVHShapeCollisionPair(int id_, const Vec3f& n, const Vec3f& contactp, FCL_REAL depth) : id(id_),
      normal(n), contact_point(contactp), penetration_depth(depth) {}

  /** \brief The index of BVH's in-collision primitive */
  int id;

  /** \brief Contact normal */
  Vec3f normal;

  /** \brief Contact points */
  Vec3f contact_point;

  /** \brief Penetration depth for two triangles */
  FCL_REAL penetration_depth;
};

struct BVHShapeCollisionPairComp
{
  bool operator()(const BVHShapeCollisionPair& a, const BVHShapeCollisionPair& b)
  {
    return a.id < b.id;
  }
};


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

  void leafTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_leaf_tests++;
    const BVNode<BV>& node = this->model1->getBV(b1);

    int primitive_id = node.primitiveId();

    const Triangle& tri_id = tri_indices[primitive_id];

    const Vec3f& p1 = vertices[tri_id[0]];
    const Vec3f& p2 = vertices[tri_id[1]];
    const Vec3f& p3 = vertices[tri_id[2]];

    FCL_REAL penetration;
    Vec3f normal;
    Vec3f contactp;

    if(!request.enable_contact) // only interested in collision or not
    {
      if(nsolver->shapeTriangleIntersect(*(this->model2), this->tf2, p1, p2, p3, NULL, NULL, NULL))
      {
        pairs.push_back(BVHShapeCollisionPair(primitive_id));
      }
    }
    else
    {
      if(nsolver->shapeTriangleIntersect(*(this->model2), this->tf2, p1, p2, p3, &contactp, &penetration, &normal))
      {
        pairs.push_back(BVHShapeCollisionPair(primitive_id, normal, contactp, penetration));
      }
    }
  }

  bool canStop() const
  {
    return (pairs.size() > 0) && (!request.exhaustive) && (request.num_max_contacts <= (int)pairs.size());
  }

  Vec3f* vertices;
  Triangle* tri_indices;

  CollisionRequest request;

  mutable std::vector<BVHShapeCollisionPair> pairs;

  const NarrowPhaseSolver* nsolver;
};


namespace details
{
template<typename BV, typename S, typename NarrowPhaseSolver>
static inline void meshShapeCollisionOrientedNodeLeafTesting(int b1, int b2,
                                                             const BVHModel<BV>* model1, const S& model2,
                                                             Vec3f* vertices, Triangle* tri_indices,
                                                             const Matrix3f& R, const Vec3f& T,
                                                             const SimpleTransform& tf2,
                                                             const NarrowPhaseSolver* nsolver,
                                                             bool enable_statistics, 
                                                             const CollisionRequest& request,
                                                             int& num_leaf_tests,
                                                             std::vector<BVHShapeCollisionPair>& pairs)
                                                 
{
  if(enable_statistics) num_leaf_tests++;
  const BVNode<BV>& node = model1->getBV(b1);

  int primitive_id = node.primitiveId();

  const Triangle& tri_id = tri_indices[primitive_id];

  const Vec3f& p1 = vertices[tri_id[0]];
  const Vec3f& p2 = vertices[tri_id[1]];
  const Vec3f& p3 = vertices[tri_id[2]];

  FCL_REAL penetration;
  Vec3f normal;
  Vec3f contactp;

  if(!request.enable_contact) // only interested in collision or not
  {
    if(nsolver->shapeTriangleIntersect(model2, tf2, p1, p2, p3, R, T, NULL, NULL, NULL))
    {
      pairs.push_back(BVHShapeCollisionPair(primitive_id));
    }
  }
  else
  {
    if(nsolver->shapeTriangleIntersect(model2, tf2, p1, p2, p3, R, T, &contactp, &penetration, &normal))
    {
      pairs.push_back(BVHShapeCollisionPair(primitive_id, normal, contactp, penetration));
    }
  }
}

}

template<typename S, typename NarrowPhaseSolver>
class MeshShapeCollisionTraversalNodeOBB : public MeshShapeCollisionTraversalNode<OBB, S, NarrowPhaseSolver>
{
public:
  MeshShapeCollisionTraversalNodeOBB() : MeshShapeCollisionTraversalNode<OBB, S, NarrowPhaseSolver>()
  {
    R.setIdentity();
  }

  bool BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return !overlap(R, T, this->model2_bv, this->model1->getBV(b1).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeCollisionOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                       R, T, this->tf2, this->nsolver, this->enable_statistics, this->request, this->num_leaf_tests, this->pairs);
  }

  // R, T is the transform of bvh model
  Matrix3f R;
  Vec3f T;
};

template<typename S, typename NarrowPhaseSolver>
class MeshShapeCollisionTraversalNodeRSS : public MeshShapeCollisionTraversalNode<RSS, S, NarrowPhaseSolver>
{
public:
  MeshShapeCollisionTraversalNodeRSS() : MeshShapeCollisionTraversalNode<RSS, S, NarrowPhaseSolver>()
  {
    R.setIdentity();
  }

  bool BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return !overlap(R, T, this->model2_bv, this->model1->getBV(b1).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeCollisionOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                       R, T, this->tf2, this->nsolver, this->enable_statistics, this->request, this->num_leaf_tests, this->pairs);
  }

  // R, T is the transform of bvh model
  Matrix3f R;
  Vec3f T;
};

template<typename S, typename NarrowPhaseSolver>
class MeshShapeCollisionTraversalNodekIOS : public MeshShapeCollisionTraversalNode<kIOS, S, NarrowPhaseSolver>
{
public:
  MeshShapeCollisionTraversalNodekIOS() : MeshShapeCollisionTraversalNode<kIOS, S, NarrowPhaseSolver>()
  {
    R.setIdentity();
  }

  bool BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return !overlap(R, T, this->model2_bv, this->model1->getBV(b1).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeCollisionOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                       R, T, this->tf2, this->nsolver, this->enable_statistics, this->request, this->num_leaf_tests, this->pairs);
  }

  // R, T is the transform of bvh model
  Matrix3f R;
  Vec3f T;
};

template<typename S, typename NarrowPhaseSolver>
class MeshShapeCollisionTraversalNodeOBBRSS : public MeshShapeCollisionTraversalNode<OBBRSS, S, NarrowPhaseSolver>
{
public:
  MeshShapeCollisionTraversalNodeOBBRSS() : MeshShapeCollisionTraversalNode<OBBRSS, S, NarrowPhaseSolver>()
  {
    R.setIdentity();
  }

  bool BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return !overlap(R, T, this->model2_bv, this->model1->getBV(b1).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeCollisionOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                       R, T, this->tf2, this->nsolver, this->enable_statistics, this->request, this->num_leaf_tests, this->pairs);
  }

  // R, T is the transform of bvh model
  Matrix3f R;
  Vec3f T;
};

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

  void leafTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_leaf_tests++;
    const BVNode<BV>& node = this->model2->getBV(b2);

    int primitive_id = node.primitiveId();

    const Triangle& tri_id = tri_indices[primitive_id];

    const Vec3f& p1 = vertices[tri_id[0]];
    const Vec3f& p2 = vertices[tri_id[1]];
    const Vec3f& p3 = vertices[tri_id[2]];

    FCL_REAL penetration;
    Vec3f normal;
    Vec3f contactp;

    if(!request.enable_contact) // only interested in collision or not
    {
      if(nsolver->shapeTriangleIntersect(*(this->model1), this->tf1, p1, p2, p3, NULL, NULL, NULL))
      {
        pairs.push_back(BVHShapeCollisionPair(primitive_id));
      }
    }
    else
    {
      if(nsolver->shapeTriangleIntersect(*(this->model1), this->tf1, p1, p2, p3, &contactp, &penetration, &normal))
      {
        pairs.push_back(BVHShapeCollisionPair(primitive_id, normal, contactp, penetration));
      }
    }
  }

  bool canStop() const
  {
    return (pairs.size() > 0) && (!request.exhaustive) && (request.num_max_contacts <= (int)pairs.size());
  }

  Vec3f* vertices;
  Triangle* tri_indices;

  CollisionRequest request;

  mutable std::vector<BVHShapeCollisionPair> pairs;

  const NarrowPhaseSolver* nsolver;
};

template<typename S, typename NarrowPhaseSolver>
class ShapeMeshCollisionTraversalNodeOBB : public ShapeMeshCollisionTraversalNode<S, OBB, NarrowPhaseSolver>
{
public:
  ShapeMeshCollisionTraversalNodeOBB() : ShapeMeshCollisionTraversalNode<S, OBB, NarrowPhaseSolver>()
  {
    R.setIdentity();
  }

  bool BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return !overlap(R, T, this->model1_bv, this->model2->getBV(b2).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeCollisionOrientedNodeLeafTesting(b2, b1, *(this->model2), this->model1, this->vertices, this->tri_indices, 
                                                       R, T, this->tf1, this->nsolver, this->enable_statistics, this->request, this->num_leaf_tests, this->pairs);

    // may need to change the order in pairs
  }

  // R, T is the transform of bvh model
  Matrix3f R;
  Vec3f T;
};


template<typename S, typename NarrowPhaseSolver>
class ShapeMeshCollisionTraversalNodeRSS : public ShapeMeshCollisionTraversalNode<S, RSS, NarrowPhaseSolver>
{
public:
  ShapeMeshCollisionTraversalNodeRSS() : ShapeMeshCollisionTraversalNode<S, RSS, NarrowPhaseSolver>()
  {
    R.setIdentity();
  }

  bool BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return !overlap(R, T, this->model1_bv, this->model2->getBV(b2).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeCollisionOrientedNodeLeafTesting(b2, b1, *(this->model2), this->model1, this->vertices, this->tri_indices, 
                                                       R, T, this->tf1, this->nsolver, this->enable_statistics, this->request, this->num_leaf_tests, this->pairs);

    // may need to change the order in pairs
  }

  // R, T is the transform of bvh model
  Matrix3f R;
  Vec3f T;
};


template<typename S, typename NarrowPhaseSolver>
class ShapeMeshCollisionTraversalNodekIOS : public ShapeMeshCollisionTraversalNode<S, kIOS, NarrowPhaseSolver>
{
public:
  ShapeMeshCollisionTraversalNodekIOS() : ShapeMeshCollisionTraversalNode<S, kIOS, NarrowPhaseSolver>()
  {
    R.setIdentity();
  }

  bool BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return !overlap(R, T, this->model1_bv, this->model2->getBV(b2).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeCollisionOrientedNodeLeafTesting(b2, b1, *(this->model2), this->model1, this->vertices, this->tri_indices, 
                                                       R, T, this->tf1, this->nsolver, this->enable_statistics, this->request, this->num_leaf_tests, this->pairs);

    // may need to change the order in pairs
  }

  // R, T is the transform of bvh model
  Matrix3f R;
  Vec3f T;
};


template<typename S, typename NarrowPhaseSolver>
class ShapeMeshCollisionTraversalNodeOBBRSS : public ShapeMeshCollisionTraversalNode<S, OBBRSS, NarrowPhaseSolver>
{
public:
  ShapeMeshCollisionTraversalNodeOBBRSS() : ShapeMeshCollisionTraversalNode<S, OBBRSS, NarrowPhaseSolver>()
  {
    R.setIdentity();
  }

  bool BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return !overlap(R, T, this->model1_bv, this->model2->getBV(b2).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeCollisionOrientedNodeLeafTesting(b2, b1, *(this->model2), this->model1, this->vertices, this->tri_indices, 
                                                       R, T, this->tf1, this->nsolver, this->enable_statistics, this->request, this->num_leaf_tests, this->pairs);

    // may need to change the order in pairs
  }

  // R, T is the transform of bvh model
  Matrix3f R;
  Vec3f T;
};


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
                                  


template<typename BV, typename S, typename NarrowPhaseSolver>
class MeshShapeDistanceTraversalNode : public BVHShapeDistanceTraversalNode<BV, S>
{ 
public:
  MeshShapeDistanceTraversalNode() : BVHShapeDistanceTraversalNode<BV, S>()
  {
    vertices = NULL;
    tri_indices = NULL;

    last_tri_id = 0;

    rel_err = 0;
    abs_err = 0;

    min_distance = std::numeric_limits<FCL_REAL>::max();

    nsolver = NULL;
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
    
    FCL_REAL d;
    nsolver->shapeTriangleDistance(*(this->model2), this->tf2, p1, p2, p3, &d);

    if(d < min_distance)
    {
      min_distance = d;
      
      last_tri_id = primitive_id;
    }
  }

  bool canStop(FCL_REAL c) const
  {
    if((c >= min_distance - abs_err) && (c * (1 + rel_err) >= min_distance))
      return true;
    return false;
  }

  Vec3f* vertices;
  Triangle* tri_indices;

  FCL_REAL rel_err;
  FCL_REAL abs_err;
  
  mutable FCL_REAL min_distance;
  mutable int last_tri_id;
  
  const NarrowPhaseSolver* nsolver;
};

namespace details
{

template<typename BV, typename S, typename NarrowPhaseSolver>
void meshShapeDistanceOrientedNodeLeafTesting(int b1, int b2,
                                              const BVHModel<BV>* model1, const S& model2,
                                              Vec3f* vertices, Triangle* tri_indices,
                                              const Matrix3f&R, const Vec3f& T,
                                              const SimpleTransform& tf2,
                                              const NarrowPhaseSolver* nsolver,
                                              bool enable_statistics,
                                              int & num_leaf_tests,
                                              int& last_tri_id,
                                              FCL_REAL& min_distance)
{
  if(enable_statistics) num_leaf_tests++;
    
  const BVNode<BV>& node = model1->getBV(b1);
  int primitive_id = node.primitiveId();

  const Triangle& tri_id = tri_indices[primitive_id];
  const Vec3f& p1 = vertices[tri_id[0]];
  const Vec3f& p2 = vertices[tri_id[1]];
  const Vec3f& p3 = vertices[tri_id[2]];
    
  FCL_REAL dist;

  nsolver->shapeTriangleDistance(model2, tf2, p1, p2, p3, R, T, &dist);
    
  if(dist < min_distance)
  {
    min_distance = dist;

    last_tri_id = primitive_id;
  }
}

}


template<typename S, typename NarrowPhaseSolver>
static inline void distance_preprocess(Vec3f* vertices, Triangle* tri_indices, int last_tri_id,
                                       const Matrix3f& R, const Vec3f& T,
                                       const S& s, const SimpleTransform& tf,
                                       const NarrowPhaseSolver* nsolver,
                                       FCL_REAL& min_distance)
{
  const Triangle& last_tri = tri_indices[last_tri_id];
  
  const Vec3f& p1 = vertices[last_tri[0]];
  const Vec3f& p2 = vertices[last_tri[1]];
  const Vec3f& p3 = vertices[last_tri[2]];

  Vec3f last_tri_P, last_tri_Q;
  
  FCL_REAL dist;
  nsolver->shapeTriangleDistance(s, tf, p1, p2, p3, R, T, &dist);
  
  min_distance = dist;
}


static inline void distance_postprocess(const Matrix3f& R, const Vec3f& T, Vec3f& p2)
{
  Vec3f u = p2 - T;
  p2 = R.transposeTimes(u);
}


template<typename S, typename NarrowPhaseSolver>
class MeshShapeDistanceTraversalNodeRSS : public MeshShapeDistanceTraversalNode<RSS, S, NarrowPhaseSolver>
{
public:
  MeshShapeDistanceTraversalNodeRSS() : MeshShapeDistanceTraversalNode<RSS, S, NarrowPhaseSolver>()
  {
    R.setIdentity();
  }

  void preprocess()
  {
    distance_preprocess(this->vertices, this->tri_indices, this->last_tri_id, R, T, *(this->model2), this->tf2, this->nsolver, this->min_distance);
  }

  void postprocess()
  {
    
  }

  FCL_REAL BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return distance(R, T, this->model2_bv, this->model1->getBV(b1).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeDistanceOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                      R, T, this->tf2, this->nsolver, this->enable_statistics, this->num_leaf_tests, this->last_tri_id, this->min_distance);
  }
  
  Matrix3f R;
  Vec3f T;
};


template<typename S, typename NarrowPhaseSolver>
class MeshShapeDistanceTraversalNodekIOS : public MeshShapeDistanceTraversalNode<kIOS, S, NarrowPhaseSolver>
{
public:
  MeshShapeDistanceTraversalNodekIOS() : MeshShapeDistanceTraversalNode<kIOS, S, NarrowPhaseSolver>()
  {
    R.setIdentity();
  }

  void preprocess()
  {
    distance_preprocess(this->vertices, this->tri_indices, this->last_tri_id, R, T, *(this->model2), this->tf2, this->nsolver, this->min_distance);
  }

  void postprocess()
  {
    
  }

  FCL_REAL BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return distance(R, T, this->model2_bv, this->model1->getBV(b1).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeDistanceOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                      R, T, this->tf2, this->nsolver, this->enable_statistics, this->num_leaf_tests, this->last_tri_id, this->min_distance);
  }
  
  Matrix3f R;
  Vec3f T;
};

template<typename S, typename NarrowPhaseSolver>
class MeshShapeDistanceTraversalNodeOBBRSS : public MeshShapeDistanceTraversalNode<OBBRSS, S, NarrowPhaseSolver>
{
public:
  MeshShapeDistanceTraversalNodeOBBRSS() : MeshShapeDistanceTraversalNode<OBBRSS, S, NarrowPhaseSolver>()
  {
    R.setIdentity();
  }

  void preprocess()
  {
    distance_preprocess(this->vertices, this->tri_indices, this->last_tri_id, R, T, *(this->model2), this->tf2, this->nsolver, this->min_distance);
  }

  void postprocess()
  {
    
  }

  FCL_REAL BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return distance(R, T, this->model2_bv, this->model1->getBV(b1).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeDistanceOrientedNodeLeafTesting(b1, b2, this->model1, *(this->model2), this->vertices, this->tri_indices,
                                                      R, T, this->tf2, this->nsolver, this->enable_statistics, this->num_leaf_tests, this->last_tri_id, this->min_distance);
  }
  
  Matrix3f R;
  Vec3f T;
};


template<typename S, typename BV, typename NarrowPhaseSolver>
class ShapeMeshDistanceTraversalNode : public ShapeBVHDistanceTraversalNode<S, BV>
{ 
public:
  ShapeMeshDistanceTraversalNode() : ShapeBVHDistanceTraversalNode<S, BV>()
  {
    vertices = NULL;
    tri_indices = NULL;

    last_tri_id = 0;

    rel_err = 0;
    abs_err = 0;

    min_distance = std::numeric_limits<FCL_REAL>::max();

    nsolver = NULL;
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
    
    FCL_REAL d;
    nsolver->shapeTriangleDistance(*(this->model1), this->tf1, p1, p2, p3, &d);

    if(d < min_distance)
    {
      min_distance = d;
      
      last_tri_id = primitive_id;
    }
  }

  bool canStop(FCL_REAL c) const
  {
    if((c >= min_distance - abs_err) && (c * (1 + rel_err) >= min_distance))
      return true;
    return false;
  }

  Vec3f* vertices;
  Triangle* tri_indices;

  FCL_REAL rel_err;
  FCL_REAL abs_err;
  
  mutable FCL_REAL min_distance;
  mutable int last_tri_id;
  
  const NarrowPhaseSolver* nsolver;
};

template<typename S, typename NarrowPhaseSolver>
class ShapeMeshDistanceTraversalNodeRSS : public ShapeMeshDistanceTraversalNode<S, RSS, NarrowPhaseSolver>
{
public:
  ShapeMeshDistanceTraversalNodeRSS() : ShapeMeshDistanceTraversalNode<S, RSS, NarrowPhaseSolver>()
  {
    R.setIdentity();
  }

  void preprocess()
  {
    distance_preprocess(this->vertices, this->tri_indices, this->last_tri_id, R, T, *(this->model1), this->tf1, this->nsolver, this->min_distance);
  }

  void postprocess()
  {
    
  }

  FCL_REAL BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return distance(R, T, this->model1_bv, this->model2->getBV(b2).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeDistanceOrientedNodeLeafTesting(b2, b1, this->model2, *(this->model1), this->vertices, this->tri_indices,
                                                      R, T, this->tf1, this->nsolver, this->enable_statistics, this->num_leaf_tests, this->last_tri_id, this->min_distance);
  }
  
  Matrix3f R;
  Vec3f T;
};

template<typename S, typename NarrowPhaseSolver>
class ShapeMeshDistanceTraversalNodekIOS : public ShapeMeshDistanceTraversalNode<S, kIOS, NarrowPhaseSolver>
{
public:
  ShapeMeshDistanceTraversalNodekIOS() : ShapeMeshDistanceTraversalNode<S, kIOS, NarrowPhaseSolver>()
  {
    R.setIdentity();
  }

  void preprocess()
  {
    distance_preprocess(this->vertices, this->tri_indices, this->last_tri_id, R, T, *(this->model1), this->tf1, this->nsolver, this->min_distance);
  }

  void postprocess()
  {
    
  }

  FCL_REAL BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return distance(R, T, this->model1_bv, this->model2->getBV(b2).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeDistanceOrientedNodeLeafTesting(b2, b1, this->model2, *(this->model1), this->vertices, this->tri_indices,
                                                      R, T, this->tf1, this->nsolver, this->enable_statistics, this->num_leaf_tests, this->last_tri_id, this->min_distance);
  }
  
  Matrix3f R;
  Vec3f T;
};

template<typename S, typename NarrowPhaseSolver>
class ShapeMeshDistanceTraversalNodeOBBRSS : public ShapeMeshDistanceTraversalNode<S, OBBRSS, NarrowPhaseSolver>
{
public:
  ShapeMeshDistanceTraversalNodeOBBRSS() : ShapeMeshDistanceTraversalNode<S, OBBRSS, NarrowPhaseSolver>()
  {
    R.setIdentity();
  }

  void preprocess()
  {
    distance_preprocess(this->vertices, this->tri_indices, this->last_tri_id, R, T, *(this->model1), this->tf1, this->nsolver, this->min_distance);
  }

  void postprocess()
  {
    
  }

  FCL_REAL BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    return distance(R, T, this->model1_bv, this->model2->getBV(b2).bv);
  }

  void leafTesting(int b1, int b2) const
  {
    details::meshShapeDistanceOrientedNodeLeafTesting(b2, b1, this->model2, *(this->model1), this->vertices, this->tri_indices,
                                                      R, T, this->tf1, this->nsolver, this->enable_statistics, this->num_leaf_tests, this->last_tri_id, this->min_distance);
  }
  
  Matrix3f R;
  Vec3f T;
};
}

#endif

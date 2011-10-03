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


#ifndef FCL_TRAVERSAL_NODE_MESHES_H
#define FCL_TRAVERSAL_NODE_MESHES_H

#include "fcl/traversal_node_base.h"
#include "fcl/BV_node.h"
#include "fcl/BV.h"
#include "fcl/BVH_model.h"
#include "fcl/intersect.h"
#include "fcl/motion.h"

#include <boost/shared_array.hpp>
#include <boost/shared_ptr.hpp>
#include <limits>
#include <vector>
#include <cassert>

/** \brief Main namespace */
namespace fcl
{

/** \brief Traversal node for collision between BVH models */
template<typename BV>
class BVHCollisionTraversalNode : public CollisionTraversalNodeBase
{
public:
  BVHCollisionTraversalNode() : CollisionTraversalNodeBase()
  {
    model1 = NULL;
    model2 = NULL;

    num_bv_tests = 0;
    num_leaf_tests = 0;
    query_time_seconds = 0.0;
  }

  /** \brief Whether the BV node in the first BVH tree is leaf */
  bool isFirstNodeLeaf(int b) const
  {
    return model1->getBV(b).isLeaf();
  }

  /** \brief Whether the BV node in the second BVH tree is leaf */
  bool isSecondNodeLeaf(int b) const
  {
    return model2->getBV(b).isLeaf();
  }

  /** \brief Determine the traversal order, is the first BVTT subtree better */
  bool firstOverSecond(int b1, int b2) const
  {
    BVH_REAL sz1 = model1->getBV(b1).bv.size();
    BVH_REAL sz2 = model2->getBV(b2).bv.size();

    bool l1 = model1->getBV(b1).isLeaf();
    bool l2 = model2->getBV(b2).isLeaf();

    if(l2 || (!l1 && (sz1 > sz2)))
      return true;
    return false;
  }

  /** \brief Obtain the left child of BV node in the first BVH */
  int getFirstLeftChild(int b) const
  {
    return model1->getBV(b).leftChild();
  }

  /** \brief Obtain the right child of BV node in the first BVH */
  int getFirstRightChild(int b) const
  {
    return model1->getBV(b).rightChild();
  }

  /** \brief Obtain the left child of BV node in the second BVH */
  int getSecondLeftChild(int b) const
  {
    return model2->getBV(b).leftChild();
  }

  /** \brief Obtain the right child of BV node in the second BVH */
  int getSecondRightChild(int b) const
  {
    return model2->getBV(b).rightChild();
  }

  /** \brief BV culling test in one BVTT node */
  bool BVTesting(int b1, int b2) const
  {
    if(enable_statistics) num_bv_tests++;
    return !model1->getBV(b1).overlap(model2->getBV(b2));
  }
  
  /** \brief The first BVH model */
  const BVHModel<BV>* model1;
  /** \brief The second BVH model */
  const BVHModel<BV>* model2;

  /** \brief statistical information */
  mutable int num_bv_tests;
  mutable int num_leaf_tests;
  mutable BVH_REAL query_time_seconds;

};


/** \brief The collision/contact information between two primitives */
struct BVHCollisionPair
{
  BVHCollisionPair() {}

  BVHCollisionPair(int id1_, int id2_) : id1(id1_), id2(id2_) {}

  BVHCollisionPair(int id1_, int id2_, const Vec3f& n, const Vec3f& contactp, BVH_REAL depth) : id1(id1_),
      id2(id2_), normal(n), contact_point(contactp), penetration_depth(depth) {}

  /** \brief The index of one in-collision primitive */
  int id1;

  /** \brief The index of the other in-collision primitive */
  int id2;

  /** \brief Contact normal */
  Vec3f normal;

  /** \brief Contact points */
  Vec3f contact_point;

  /** \brief Penetration depth for two triangles */
  BVH_REAL penetration_depth;
};

/** \brief Sorting rule between two BVHCollisionPair, for testing */
struct BVHCollisionPairComp
{
  bool operator()(const BVHCollisionPair& a, const BVHCollisionPair& b)
  {
    if(a.id1 == b.id1)
      return a.id2 < b.id2;
    return a.id1 < b.id1;
  }
};

/** \brief Traversal node for collision between two meshes */
template<typename BV>
class MeshCollisionTraversalNode : public BVHCollisionTraversalNode<BV>
{
public:
  MeshCollisionTraversalNode() : BVHCollisionTraversalNode<BV>()
  {
    vertices1 = NULL;
    vertices2 = NULL;
    tri_indices1 = NULL;
    tri_indices2 = NULL;

    num_max_contacts = 1;
    exhaustive = false;
    enable_contact = false;
  }

  /** \brief Intersection testing between leaves (two triangles) */
  void leafTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_leaf_tests++;

    const BVNode<BV>& node1 = this->model1->getBV(b1);
    const BVNode<BV>& node2 = this->model2->getBV(b2);

    int primitive_id1 = node1.primitiveId();
    int primitive_id2 = node2.primitiveId();

    const Triangle& tri_id1 = tri_indices1[primitive_id1];
    const Triangle& tri_id2 = tri_indices2[primitive_id2];

    const Vec3f& p1 = vertices1[tri_id1[0]];
    const Vec3f& p2 = vertices1[tri_id1[1]];
    const Vec3f& p3 = vertices1[tri_id1[2]];
    const Vec3f& q1 = vertices2[tri_id2[0]];
    const Vec3f& q2 = vertices2[tri_id2[1]];
    const Vec3f& q3 = vertices2[tri_id2[2]];

    BVH_REAL penetration;
    Vec3f normal;
    int n_contacts;
    Vec3f contacts[2];


    if(!enable_contact) // only interested in collision or not
    {
      if(Intersect::intersect_Triangle(p1, p2, p3, q1, q2, q3))
      {
          pairs.push_back(BVHCollisionPair(primitive_id1, primitive_id2));
      }
    }
    else // need compute the contact information
    {
      if(Intersect::intersect_Triangle(p1, p2, p3, q1, q2, q3,
                                       contacts,
                                       (unsigned int*)&n_contacts,
                                       &penetration,
                                       &normal))
      {
        for(int i = 0; i < n_contacts; ++i)
        {
          if((!exhaustive) && (num_max_contacts <= (int)pairs.size())) break;
          pairs.push_back(BVHCollisionPair(primitive_id1, primitive_id2, contacts[i], normal, penetration));
        }
      }
    }
  }

  /** \brief Whether the traversal process can stop early */
  bool canStop() const
  {
    return (pairs.size() > 0) && (!exhaustive) && (num_max_contacts <= (int)pairs.size());
  }

  Vec3f* vertices1;
  Vec3f* vertices2;

  Triangle* tri_indices1;
  Triangle* tri_indices2;

  int num_max_contacts;
  bool exhaustive;
  bool enable_contact;

  mutable std::vector<BVHCollisionPair> pairs;
};



class MeshCollisionTraversalNodeOBB : public MeshCollisionTraversalNode<OBB>
{
public:
  MeshCollisionTraversalNodeOBB();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  Vec3f R[3];
  Vec3f T;
};


class MeshCollisionTraversalNodeRSS : public MeshCollisionTraversalNode<RSS>
{
public:
  MeshCollisionTraversalNodeRSS();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  Vec3f R[3];
  Vec3f T;
};



#if USE_SVMLIGHT

struct BVHPointCollisionPair
{
  BVHPointCollisionPair() {}

  BVHPointCollisionPair(int id1_start_, int id1_num_, int id2_start_, int id2_num_, BVH_REAL collision_prob_)
    : id1_start(id1_start_), id1_num(id1_num_), id2_start(id2_start_), id2_num(id2_num_), collision_prob(collision_prob_) {}

  int id1_start;
  int id1_num;

  int id2_start;
  int id2_num;

  BVH_REAL collision_prob;
};

struct BVHPointCollisionPairComp
{
  bool operator()(const BVHPointCollisionPair& a, const BVHPointCollisionPair& b)
  {
    if(a.id1_start == b.id1_start)
      return a.id2_start < b.id2_start;
    return a.id1_start < b.id1_start;
  }
};


template<typename BV>
class PointCloudCollisionTraversalNode : public BVHCollisionTraversalNode<BV>
{
public:
  PointCloudCollisionTraversalNode() : BVHCollisionTraversalNode<BV>()
  {
    vertices1 = NULL;
    vertices2 = NULL;

    num_max_contacts = 1;
    exhaustive = false;
    enable_contact = false;

    collision_prob_threshold = 0.5;
    max_collision_prob = 0;
    leaf_size_threshold = 1;
  }

  bool isFirstNodeLeaf(int b) const
  {
    const BVNode<BV>& node = this->model1->getBV(b);
    return ((node.num_primitives < leaf_size_threshold) || node.isLeaf());
  }

  bool isSecondNodeLeaf(int b) const
  {
    const BVNode<BV>& node = this->model2->getBV(b);
    return ((node.num_primitives < leaf_size_threshold) || node.isLeaf());
  }

  void leafTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_leaf_tests++;

    const BVNode<BV>& node1 = this->model1->getBV(b1);
    const BVNode<BV>& node2 = this->model2->getBV(b2);

    BVH_REAL collision_prob = Intersect::intersect_PointClouds(vertices1 + node1.first_primitive, uc1.get() + node1.first_primitive,
                                                               node1.num_primitives,
                                                               vertices2 + node2.first_primitive, uc2.get() + node2.first_primitive,
                                                               node2.num_primitives,
                                                               classifier_param);

    if(collision_prob > collision_prob_threshold)
      pairs.push_back(BVHPointCollisionPair(node1.first_primitive, node1.num_primitives, node2.first_primitive, node2.num_primitives, collision_prob));

    if(collision_prob > max_collision_prob)
      max_collision_prob = collision_prob;

  }

  bool canStop() const
  {
    return (pairs.size() > 0) && (!exhaustive) && (num_max_contacts <= (int)pairs.size());
  }

  Vec3f* vertices1;
  Vec3f* vertices2;

  boost::shared_array<Uncertainty> uc1;
  boost::shared_array<Uncertainty> uc2;

  int num_max_contacts;
  bool exhaustive;
  bool enable_contact;

  mutable std::vector<BVHPointCollisionPair> pairs;

  int leaf_size_threshold;

  BVH_REAL collision_prob_threshold;

  mutable BVH_REAL max_collision_prob;

  CloudClassifierParam classifier_param;
};



class PointCloudCollisionTraversalNodeOBB : public PointCloudCollisionTraversalNode<OBB>
{
public:
  PointCloudCollisionTraversalNodeOBB();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  Vec3f R[3];
  Vec3f T;
};


class PointCloudCollisionTraversalNodeRSS : public PointCloudCollisionTraversalNode<RSS>
{
public:
  PointCloudCollisionTraversalNodeRSS();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  Vec3f R[3];
  Vec3f T;
};



template<typename BV>
class PointCloudMeshCollisionTraversalNode : public BVHCollisionTraversalNode<BV>
{
public:
  PointCloudMeshCollisionTraversalNode() : BVHCollisionTraversalNode<BV>()
  {
    vertices1 = NULL;
    vertices2 = NULL;
    tri_indices2 = NULL;

    num_max_contacts = 1;
    exhaustive = false;
    enable_contact = false;

    collision_prob_threshold = 0.5;
    max_collision_prob = 0;
    leaf_size_threshold = 1;
  }

  bool isFirstNodeLeaf(int b) const
  {
    const BVNode<BV>& node = this->model1->getBV(b);
    return ((node.num_primitives < leaf_size_threshold) || node.isLeaf());
  }

  void leafTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_leaf_tests++;

    const BVNode<BV>& node1 = this->model1->getBV(b1);
    const BVNode<BV>& node2 = this->model2->getBV(b2);

    const Triangle& tri_id2 = tri_indices2[node2.primitiveId()];

    const Vec3f& q1 = vertices2[tri_id2[0]];
    const Vec3f& q2 = vertices2[tri_id2[1]];
    const Vec3f& q3 = vertices2[tri_id2[2]];

    BVH_REAL collision_prob = Intersect::intersect_PointCloudsTriangle(vertices1 + node1.first_primitive, uc1.get() + node1.first_primitive,
                                                                       node1.num_primitives,
                                                                       q1, q2, q3);

    if(collision_prob > collision_prob_threshold)
      pairs.push_back(BVHPointCollisionPair(node1.first_primitive, node1.num_primitives, node2.first_primitive, node2.num_primitives, collision_prob));

    if(collision_prob > max_collision_prob)
      max_collision_prob = collision_prob;
  }

  bool canStop() const
  {
    return (pairs.size() > 0) && (!exhaustive) && (num_max_contacts <= (int)pairs.size());
  }

  Vec3f* vertices1;
  Vec3f* vertices2;

  boost::shared_array<Uncertainty> uc1;
  Triangle* tri_indices2;

  int num_max_contacts;
  bool exhaustive;
  bool enable_contact;

  mutable std::vector<BVHPointCollisionPair> pairs;

  int leaf_size_threshold;

  BVH_REAL collision_prob_threshold;

  mutable BVH_REAL max_collision_prob;
};


class PointCloudMeshCollisionTraversalNodeOBB : public PointCloudMeshCollisionTraversalNode<OBB>
{
public:
  PointCloudMeshCollisionTraversalNodeOBB();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  Vec3f R[3];
  Vec3f T;
};

class PointCloudMeshCollisionTraversalNodeRSS : public PointCloudMeshCollisionTraversalNode<RSS>
{
public:
  PointCloudMeshCollisionTraversalNodeRSS();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  Vec3f R[3];
  Vec3f T;
};

#endif

struct BVHContinuousCollisionPair
{
  BVHContinuousCollisionPair() {}

  BVHContinuousCollisionPair(int id1_, int id2_, BVH_REAL time) : id1(id1_), id2(id2_), collision_time(time) {}

  /** \brief The index of one in-collision primitive */
  int id1;

  /** \brief The index of the other in-collision primitive */
  int id2;

  /** \brief Collision time normalized in [0, 1]. The collision time out of [0, 1] means collision-free */
  BVH_REAL collision_time;
};


template<typename BV>
class MeshContinuousCollisionTraversalNode : public BVHCollisionTraversalNode<BV>
{
public:
  MeshContinuousCollisionTraversalNode() : BVHCollisionTraversalNode<BV>()
  {
    vertices1 = NULL;
    vertices2 = NULL;
    tri_indices1 = NULL;
    tri_indices2 = NULL;
    prev_vertices1 = NULL;
    prev_vertices2 = NULL;

    num_max_contacts = 1;
    exhaustive = false;
    enable_contact = false;

    num_vf_tests = 0;
    num_ee_tests = 0;
  }

  void leafTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_leaf_tests++;

    const BVNode<BV>& node1 = this->model1->getBV(b1);
    const BVNode<BV>& node2 = this->model2->getBV(b2);

    BVH_REAL collision_time = 2;
    Vec3f collision_pos;

    int primitive_id1 = node1.primitiveId();
    int primitive_id2 = node2.primitiveId();

    const Triangle& tri_id1 = tri_indices1[primitive_id1];
    const Triangle& tri_id2 = tri_indices2[primitive_id2];

    Vec3f* S0[3];
    Vec3f* S1[3];
    Vec3f* T0[3];
    Vec3f* T1[3];


    for(int i = 0; i < 3; ++i)
    {
      S0[i] = prev_vertices1 + tri_id1[i];
      S1[i] = vertices1 + tri_id1[i];
      T0[i] = prev_vertices2 + tri_id2[i];
      T1[i] = vertices2 + tri_id2[i];
    }

    BVH_REAL tmp;
    Vec3f tmpv;

    // 6 VF checks
    for(int i = 0; i < 3; ++i)
    {
      if(this->enable_statistics) num_vf_tests++;
      if(Intersect::intersect_VF(*(S0[0]), *(S0[1]), *(S0[2]), *(T0[i]), *(S1[0]), *(S1[1]), *(S1[2]), *(T1[i]), &tmp, &tmpv))
      {
        if(collision_time > tmp)
        {
          collision_time = tmp; collision_pos = tmpv;
        }
      }

      if(this->enable_statistics) num_vf_tests++;
      if(Intersect::intersect_VF(*(T0[0]), *(T0[1]), *(T0[2]), *(S0[i]), *(T1[0]), *(T1[1]), *(T1[2]), *(S1[i]), &tmp, &tmpv))
      {
        if(collision_time > tmp)
        {
          collision_time = tmp; collision_pos = tmpv;
        }
      }
    }

    // 9 EE checks
    for(int i = 0; i < 3; ++i)
    {
      int S_id1 = i;
      int S_id2 = i + 1;
      if(S_id2 == 3) S_id2 = 0;
      for(int j = 0; j < 3; ++j)
      {
        int T_id1 = j;
        int T_id2 = j + 1;
        if(T_id2 == 3) T_id2 = 0;

        num_ee_tests++;
        if(Intersect::intersect_EE(*(S0[S_id1]), *(S0[S_id2]), *(T0[T_id1]), *(T0[T_id2]), *(S1[S_id1]), *(S1[S_id2]), *(T1[T_id1]), *(T1[T_id2]), &tmp, &tmpv))
        {
          if(collision_time > tmp)
          {
            collision_time = tmp; collision_pos = tmpv;
          }
        }
      }
    }

    if(!(collision_time > 1)) // collision happens
      pairs.push_back(BVHContinuousCollisionPair(primitive_id1, primitive_id2, collision_time));
  }

  bool canStop() const
  {
    return (pairs.size() > 0) && (!exhaustive) && (num_max_contacts <= (int)pairs.size());
  }

  Vec3f* vertices1;
  Vec3f* vertices2;

  Triangle* tri_indices1;
  Triangle* tri_indices2;

  Vec3f* prev_vertices1;
  Vec3f* prev_vertices2;

  int num_max_contacts;
  bool exhaustive;
  bool enable_contact;

  mutable int num_vf_tests;
  mutable int num_ee_tests;

  mutable std::vector<BVHContinuousCollisionPair> pairs;
};


template<typename BV>
class MeshPointCloudContinuousCollisionTraversalNode : public BVHCollisionTraversalNode<BV>
{
public:
  MeshPointCloudContinuousCollisionTraversalNode() : BVHCollisionTraversalNode<BV>()
  {
    vertices1 = NULL;
    vertices2 = NULL;
    tri_indices1 = NULL;
    prev_vertices1 = NULL;
    prev_vertices2 = NULL;

    num_max_contacts = 1;
    exhaustive = false;
    enable_contact = false;

    num_vf_tests = 0;
  }

  void leafTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_leaf_tests++;

    const BVNode<BV>& node1 = this->model1->getBV(b1);
    const BVNode<BV>& node2 = this->model2->getBV(b2);

    BVH_REAL collision_time = 2;
    Vec3f collision_pos;

    int primitive_id1 = node1.primitiveId();
    int primitive_id2 = node2.primitiveId();

    const Triangle& tri_id1 = tri_indices1[primitive_id1];
    int vertex_id2 = primitive_id2;

    Vec3f* S0[3];
    Vec3f* S1[3];

    for(int i = 0; i < 3; ++i)
    {
      S0[i] = prev_vertices1 + tri_id1[i];
      S1[i] = vertices1 + tri_id1[i];
    }
    Vec3f* T0 = prev_vertices2 + vertex_id2;
    Vec3f* T1 = vertices2 + vertex_id2;

    BVH_REAL tmp;
    Vec3f tmpv;

    // 3 VF checks
    for(int i = 0; i < 3; ++i)
    {
      num_vf_tests++;
      if(Intersect::intersect_VF(*(S0[0]), *(S0[1]), *(S0[2]), *T0, *(S1[0]), *(S1[1]), *(S1[2]), *T1, &tmp, &tmpv))
      {
        if(collision_time > tmp)
        {
          collision_time = tmp; collision_pos = tmpv;
        }
      }
    }

    if(!(collision_time > 1)) // collision happens
    {
      pairs.push_back(BVHContinuousCollisionPair(primitive_id1, primitive_id2, collision_time));
    }
  }

  bool canStop() const
  {
    return (pairs.size() > 0) && (!exhaustive) && (num_max_contacts <= (int)pairs.size());
  }

  Vec3f* vertices1;
  Vec3f* vertices2;

  Triangle* tri_indices1;

  Vec3f* prev_vertices1;
  Vec3f* prev_vertices2;

  int num_max_contacts;
  bool exhaustive;
  bool enable_contact;

  mutable int num_vf_tests;

  mutable std::vector<BVHContinuousCollisionPair> pairs;
};


template<typename BV>
class PointCloudMeshContinuousCollisionTraversalNode : public BVHCollisionTraversalNode<BV>
{
public:
  PointCloudMeshContinuousCollisionTraversalNode() : BVHCollisionTraversalNode<BV>()
  {
    vertices1 = NULL;
    vertices2 = NULL;
    tri_indices2 = NULL;
    prev_vertices1 = NULL;
    prev_vertices2 = NULL;

    num_max_contacts = 1;
    exhaustive = false;
    enable_contact = false;

    num_vf_tests = 0;
  }

  void leafTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_leaf_tests++;

    const BVNode<BV>& node1 = this->model1->getBV(b1);
    const BVNode<BV>& node2 = this->model2->getBV(b2);

    BVH_REAL collision_time = 2;
    Vec3f collision_pos;

    int primitive_id1 = node1.primitiveId();
    int primitive_id2 = node2.primitiveId();

    int vertex_id1 = primitive_id1;
    const Triangle& tri_id2 = tri_indices2[primitive_id2];

    Vec3f* S0 = prev_vertices1 + vertex_id1;
    Vec3f* S1 = vertices1 + vertex_id1;

    Vec3f* T0[3];
    Vec3f* T1[3];
    for(int i = 0; i < 3; ++i)
    {
      T0[i] = prev_vertices2 + tri_id2[i];
      T1[i] = vertices2 + tri_id2[i];
    }

    BVH_REAL tmp;
    Vec3f tmpv;

    // 3 VF checks
    for(int i = 0; i < 3; ++i)
    {
      num_vf_tests++;
      if(Intersect::intersect_VF(*(T0[0]), *(T0[1]), *(T0[2]), *S0, *(T1[0]), *(T1[1]), *(T1[2]), *S1, &tmp, &tmpv))
      {
        if(collision_time > tmp)
        {
          collision_time = tmp; collision_pos = tmpv;
        }
      }
    }

    if(!(collision_time > 1)) // collision happens
    {
      pairs.push_back(BVHContinuousCollisionPair(primitive_id1, primitive_id2, collision_time));
    }
  }

  bool canStop() const
  {
    return (pairs.size() > 0) && (!exhaustive) && (num_max_contacts <= (int)pairs.size());
  }

  Vec3f* vertices1;
  Vec3f* vertices2;

  Triangle* tri_indices2;

  Vec3f* prev_vertices1;
  Vec3f* prev_vertices2;

  int num_max_contacts;
  bool exhaustive;
  bool enable_contact;

  mutable int num_vf_tests;

  mutable std::vector<BVHContinuousCollisionPair> pairs;
};




template<typename BV>
class BVHDistanceTraversalNode : public DistanceTraversalNodeBase
{
public:
  BVHDistanceTraversalNode() : DistanceTraversalNodeBase()
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

  bool isSecondNodeLeaf(int b) const
  {
    return model2->getBV(b).isLeaf();
  }

  bool firstOverSecond(int b1, int b2) const
  {
    BVH_REAL sz1 = model1->getBV(b1).bv.size();
    BVH_REAL sz2 = model2->getBV(b2).bv.size();

    bool l1 = model1->getBV(b1).isLeaf();
    bool l2 = model2->getBV(b2).isLeaf();

    if(l2 || (!l1 && (sz1 > sz2)))
      return true;
    return false;
  }

  int getFirstLeftChild(int b) const
  {
    return model1->getBV(b).leftChild();
  }

  int getFirstRightChild(int b) const
  {
    return model1->getBV(b).rightChild();
  }

  int getSecondLeftChild(int b) const
  {
    return model2->getBV(b).leftChild();
  }

  int getSecondRightChild(int b) const
  {
    return model2->getBV(b).rightChild();
  }

  BVH_REAL BVTesting(int b1, int b2) const
  {
    if(enable_statistics) num_bv_tests++;
    return model1->getBV(b1).distance(model2->getBV(b2));
  }

  const BVHModel<BV>* model1;
  const BVHModel<BV>* model2;

  mutable int num_bv_tests;
  mutable int num_leaf_tests;
  mutable BVH_REAL query_time_seconds;

};



template<typename BV>
class MeshDistanceTraversalNode : public BVHDistanceTraversalNode<BV>
{
public:
  MeshDistanceTraversalNode() : BVHDistanceTraversalNode<BV>()
  {
    vertices1 = NULL;
    vertices2 = NULL;
    tri_indices1 = NULL;
    tri_indices2 = NULL;

    last_tri_id1 = 0;
    last_tri_id2 = 0;

    rel_err = 0;
    abs_err = 0;

    min_distance = std::numeric_limits<BVH_REAL>::max();
  }

  void leafTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_leaf_tests++;

    const BVNode<BV>& node1 = this->model1->getBV(b1);
    const BVNode<BV>& node2 = this->model2->getBV(b2);

    int primitive_id1 = node1.primitiveId();
    int primitive_id2 = node2.primitiveId();

    const Triangle& tri_id1 = tri_indices1[primitive_id1];
    const Triangle& tri_id2 = tri_indices2[primitive_id2];

    const Vec3f& t11 = vertices1[tri_id1[0]];
    const Vec3f& t12 = vertices1[tri_id1[1]];
    const Vec3f& t13 = vertices1[tri_id1[2]];

    const Vec3f& t21 = vertices2[tri_id2[0]];
    const Vec3f& t22 = vertices2[tri_id2[1]];
    const Vec3f& t23 = vertices2[tri_id2[2]];

    // nearest point pair
    Vec3f P1, P2;

    BVH_REAL d = TriangleDistance::triDistance(t11, t12, t13, t21, t22, t23,
                                               P1, P2);

    if(d < min_distance)
    {
      min_distance = d;

      p1 = P1;
      p2 = P2;

      last_tri_id1 = primitive_id1;
      last_tri_id2 = primitive_id2;
    }
  }

  bool canStop(BVH_REAL c) const
  {
    if((c >= min_distance - abs_err) && (c * (1 + rel_err) >= min_distance))
      return true;
    return false;
  }

  Vec3f* vertices1;
  Vec3f* vertices2;

  Triangle* tri_indices1;
  Triangle* tri_indices2;

  /** \brief relative and absolute error, default value is 0.01 for both terms */
  BVH_REAL rel_err;
  BVH_REAL abs_err;

  /** \brief distance and points establishing the minimum distance for the models, within the relative and absolute error bounds specified.
   * p1 is in model1's local coordinate system while p2 is in model2's local coordinate system
   */
  mutable BVH_REAL min_distance;
  mutable Vec3f p1;
  mutable Vec3f p2;

  /** \brief Remember the nearest neighbor points */
  mutable int last_tri_id1;
  mutable int last_tri_id2;
};


class MeshDistanceTraversalNodeRSS : public MeshDistanceTraversalNode<RSS>
{
public:
  MeshDistanceTraversalNodeRSS();

  BVH_REAL BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  Vec3f R[3];
  Vec3f T;
};



struct ConservativeAdvancementStackData
{
  ConservativeAdvancementStackData(const Vec3f& P1_, const Vec3f& P2_, int c1_, int c2_, BVH_REAL d_) : P1(P1_), P2(P2_), c1(c1_), c2(c2_), d(d_) {}

  Vec3f P1;
  Vec3f P2;
  int c1;
  int c2;
  BVH_REAL d;
};

// when using this default version, must refit the BVH in current configuration (R_t, T_t) into default configuration
template<typename BV>
class MeshConservativeAdvancementTraversalNode : public MeshDistanceTraversalNode<BV>
{
public:
  MeshConservativeAdvancementTraversalNode(BVH_REAL w_ = 1) : MeshDistanceTraversalNode<BV>()
  {
    delta_t = 1;
    toc = 0;
    t_err = (BVH_REAL)0.00001;

    w = w_;

    motion1 = NULL;
    motion2 = NULL;
  }

  BVH_REAL BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    Vec3f P1, P2;
    BVH_REAL d = this->model1->getBV(b1).distance(this->model2->getBV(b2), &P1, &P2);

    stack.push_back(ConservativeAdvancementStackData(P1, P2, b1, b2, d));

    return d;
  }

  void leafTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_leaf_tests++;

    const BVNode<BV>& node1 = this->model1->getBV(b1);
    const BVNode<BV>& node2 = this->model2->getBV(b2);

    int primitive_id1 = node1.primitiveId();
    int primitive_id2 = node2.primitiveId();

    const Triangle& tri_id1 = this->tri_indices1[primitive_id1];
    const Triangle& tri_id2 = this->tri_indices2[primitive_id2];

    const Vec3f& p1 = this->vertices1[tri_id1[0]];
    const Vec3f& p2 = this->vertices1[tri_id1[1]];
    const Vec3f& p3 = this->vertices1[tri_id1[2]];

    const Vec3f& q1 = this->vertices2[tri_id2[0]];
    const Vec3f& q2 = this->vertices2[tri_id2[1]];
    const Vec3f& q3 = this->vertices2[tri_id2[2]];

    // nearest point pair
    Vec3f P1, P2;

    BVH_REAL d = TriangleDistance::triDistance(p1, p2, p3, q1, q2, q3,
                                               P1, P2);

    if(d < this->min_distance)
    {
      this->min_distance = d;

      this->p1 = P1;
      this->p2 = P2;

      this->last_tri_id1 = primitive_id1;
      this->last_tri_id2 = primitive_id2;
    }


    // n is the local frame of object 1
    Vec3f n = P2 - P1;
    // here n is already in global frame as we assume the body is in original configuration (I, 0) for general BVH
    BVH_REAL bound1 = motion1->computeMotionBound(p1, p2, p3, n);
    BVH_REAL bound2 = motion2->computeMotionBound(q1, q2, q3, n);

    BVH_REAL bound = bound1 + bound2;

    BVH_REAL cur_delta_t;
    if(bound <= d) cur_delta_t = 1;
    else cur_delta_t = d / bound;

    if(cur_delta_t < delta_t)
      delta_t = cur_delta_t;
  }


  bool canStop(BVH_REAL c) const
  {
    if((c >= w * (this->min_distance - this->abs_err)) && (c * (1 + this->rel_err) >= w * this->min_distance))
    {
      const ConservativeAdvancementStackData& data = stack.back();
      BVH_REAL d = data.d;
      Vec3f n;
      int c1, c2;

      if(d > c)
      {
        const ConservativeAdvancementStackData& data2 = stack[stack.size() - 2];
        d = data2.d;
        n = data2.P2 - data2.P1;
        c1 = data2.c1;
        c2 = data2.c2;
        stack[stack.size() - 2] = stack[stack.size() - 1];
      }
      else
      {
        n = data.P2 - data.P1;
        c1 = data.c1;
        c2 = data.c2;
      }

      assert(c == d);

      BVH_REAL bound1 = motion1->computeMotionBound((this->tree1 + c1)->bv, n);
      BVH_REAL bound2 = motion2->computeMotionBound((this->tree2 + c2)->bv, n);

      BVH_REAL bound = bound1 + bound2;

      BVH_REAL cur_delta_t;
      if(bound <= c) cur_delta_t = 1;
      else cur_delta_t = c / bound;

      if(cur_delta_t < delta_t)
        delta_t = cur_delta_t;

      stack.pop_back();

      return true;
    }
    else
    {
      const ConservativeAdvancementStackData& data = stack.back();
      BVH_REAL d = data.d;

      if(d > c)
        stack[stack.size() - 2] = stack[stack.size() - 1];

      stack.pop_back();

      return false;
    }
  }


  /** \brief CA controlling variable: early stop for the early iterations of CA */
  BVH_REAL w;

  /** \brief The time from beginning point */
  BVH_REAL toc;
  BVH_REAL t_err;

  /** \brief The delta_t each step */
  mutable BVH_REAL delta_t;

  /** \brief Motions for the two objects in query */
  MotionBase<BV>* motion1;
  MotionBase<BV>* motion2;

  mutable std::vector<ConservativeAdvancementStackData> stack;
};


/** for OBB and RSS, there is local coordinate of BV, so normal need to be transformed */
template<>
bool MeshConservativeAdvancementTraversalNode<OBB>::canStop(BVH_REAL c) const;

template<>
bool MeshConservativeAdvancementTraversalNode<RSS>::canStop(BVH_REAL c) const;


class MeshConservativeAdvancementTraversalNodeRSS : public MeshConservativeAdvancementTraversalNode<RSS>
{
public:
  MeshConservativeAdvancementTraversalNodeRSS(BVH_REAL w_ = 1);

  BVH_REAL BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  bool canStop(BVH_REAL c) const;

  Vec3f R[3];
  Vec3f T;
};

}


#endif

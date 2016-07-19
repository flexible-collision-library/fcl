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


#ifndef FCL_TRAVERSAL_NODE_MESHES_H
#define FCL_TRAVERSAL_NODE_MESHES_H

#include "fcl/collision_data.h"
#include "fcl/traversal/traversal_node_base.h"
#include "fcl/BV/BV_node.h"
#include "fcl/BV/BV.h"
#include "fcl/BVH/BVH_model.h"
#include "fcl/intersect.h"
#include "fcl/ccd/motion.h"

#include <memory>
#include <limits>
#include <vector>
#include <cassert>


namespace fcl
{

/// @brief Traversal node for collision between BVH models
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

  /// @brief Whether the BV node in the first BVH tree is leaf
  bool isFirstNodeLeaf(int b) const
  {
    return model1->getBV(b).isLeaf();
  }

  /// @brief Whether the BV node in the second BVH tree is leaf
  bool isSecondNodeLeaf(int b) const
  {
    return model2->getBV(b).isLeaf();
  }

  /// @brief Determine the traversal order, is the first BVTT subtree better
  bool firstOverSecond(int b1, int b2) const
  {
    FCL_REAL sz1 = model1->getBV(b1).bv.size();
    FCL_REAL sz2 = model2->getBV(b2).bv.size();

    bool l1 = model1->getBV(b1).isLeaf();
    bool l2 = model2->getBV(b2).isLeaf();

    if(l2 || (!l1 && (sz1 > sz2)))
      return true;
    return false;
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
    if(enable_statistics) num_bv_tests++;
    return !model1->getBV(b1).overlap(model2->getBV(b2));
  }
  
  /// @brief The first BVH model
  const BVHModel<BV>* model1;
  /// @brief The second BVH model
  const BVHModel<BV>* model2;

  /// @brief statistical information
  mutable int num_bv_tests;
  mutable int num_leaf_tests;
  mutable FCL_REAL query_time_seconds;
};


/// @brief Traversal node for collision between two meshes
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
  }

  /// @brief Intersection testing between leaves (two triangles)
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
    
    if(this->model1->isOccupied() && this->model2->isOccupied())
    {
      bool is_intersect = false;

      if(!this->request.enable_contact) // only interested in collision or not
      {
        if(Intersect::intersect_Triangle(p1, p2, p3, q1, q2, q3))
        {
          is_intersect = true;
          if(this->result->numContacts() < this->request.num_max_contacts)
            this->result->addContact(Contact(this->model1, this->model2, primitive_id1, primitive_id2));
        }
      }
      else // need compute the contact information
      {
        FCL_REAL penetration;
        Vec3f normal;
        unsigned int n_contacts;
        Vec3f contacts[2];

        if(Intersect::intersect_Triangle(p1, p2, p3, q1, q2, q3,
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
            this->result->addContact(Contact(this->model1, this->model2, primitive_id1, primitive_id2, contacts[i], normal, penetration));
          }
        }
      }

      if(is_intersect && this->request.enable_cost)
      {
        AABB overlap_part;
        AABB(p1, p2, p3).overlap(AABB(q1, q2, q3), overlap_part);
        this->result->addCostSource(CostSource(overlap_part, cost_density), this->request.num_max_cost_sources);      
      }
    }   
    else if((!this->model1->isFree() && !this->model2->isFree()) && this->request.enable_cost)
    {
      if(Intersect::intersect_Triangle(p1, p2, p3, q1, q2, q3))
      {
        AABB overlap_part;
        AABB(p1, p2, p3).overlap(AABB(q1, q2, q3), overlap_part);
        this->result->addCostSource(CostSource(overlap_part, cost_density), this->request.num_max_cost_sources);      
      }
    }
  }

  /// @brief Whether the traversal process can stop early
  bool canStop() const
  {
    return this->request.isSatisfied(*(this->result));
  }

  Vec3f* vertices1;
  Vec3f* vertices2;

  Triangle* tri_indices1;
  Triangle* tri_indices2;

  FCL_REAL cost_density;
};


/// @brief Traversal node for collision between two meshes if their underlying BVH node is oriented node (OBB, RSS, OBBRSS, kIOS)
class MeshCollisionTraversalNodeOBB : public MeshCollisionTraversalNode<OBB>
{
public:
  MeshCollisionTraversalNodeOBB();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  bool BVTesting(int b1, int b2, const Matrix3f& Rc, const Vec3f& Tc) const;

  void leafTesting(int b1, int b2, const Matrix3f& Rc, const Vec3f& Tc) const;

  Matrix3f R;
  Vec3f T;
};

class MeshCollisionTraversalNodeRSS : public MeshCollisionTraversalNode<RSS>
{
public:
  MeshCollisionTraversalNodeRSS();

  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  bool BVTesting(int b1, int b2, const Matrix3f& Rc, const Vec3f& Tc) const;

  void leafTesting(int b1, int b2, const Matrix3f& Rc, const Vec3f& Tc) const;

  Matrix3f R;
  Vec3f T;
};

class MeshCollisionTraversalNodekIOS : public MeshCollisionTraversalNode<kIOS>
{
public:
  MeshCollisionTraversalNodekIOS();
 
  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  Matrix3f R;
  Vec3f T;
};

class MeshCollisionTraversalNodeOBBRSS : public MeshCollisionTraversalNode<OBBRSS>
{
public:
  MeshCollisionTraversalNodeOBBRSS();
 
  bool BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  Matrix3f R;
  Vec3f T;
};

/// @brief Traversal node for continuous collision between BVH models
struct BVHContinuousCollisionPair
{
  BVHContinuousCollisionPair() {}

  BVHContinuousCollisionPair(int id1_, int id2_, FCL_REAL time) : id1(id1_), id2(id2_), collision_time(time) {}

  /// @brief The index of one in-collision primitive
  int id1;

  /// @brief The index of the other in-collision primitive
  int id2;

  /// @brief Collision time normalized in [0, 1]. The collision time out of [0, 1] means collision-free
  FCL_REAL collision_time;
};

/// @brief Traversal node for continuous collision between meshes
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

    num_vf_tests = 0;
    num_ee_tests = 0;
    time_of_contact = 1;
  }

  /// @brief Intersection testing between leaves (two triangles)
  void leafTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_leaf_tests++;

    const BVNode<BV>& node1 = this->model1->getBV(b1);
    const BVNode<BV>& node2 = this->model2->getBV(b2);

    FCL_REAL collision_time = 2;
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

    FCL_REAL tmp;
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
    {
      pairs.push_back(BVHContinuousCollisionPair(primitive_id1, primitive_id2, collision_time));
      time_of_contact = std::min(time_of_contact, collision_time);
    }
  }

  /// @brief Whether the traversal process can stop early
  bool canStop() const
  {
    return (pairs.size() > 0) && (this->request.num_max_contacts <= pairs.size());
  }

  Vec3f* vertices1;
  Vec3f* vertices2;

  Triangle* tri_indices1;
  Triangle* tri_indices2;

  Vec3f* prev_vertices1;
  Vec3f* prev_vertices2;

  mutable int num_vf_tests;
  mutable int num_ee_tests;

  mutable std::vector<BVHContinuousCollisionPair> pairs;

  mutable FCL_REAL time_of_contact;
};



/// @brief Traversal node for distance computation between BVH models
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

  /// @brief Whether the BV node in the first BVH tree is leaf
  bool isFirstNodeLeaf(int b) const
  {
    return model1->getBV(b).isLeaf();
  }

  /// @brief Whether the BV node in the second BVH tree is leaf
  bool isSecondNodeLeaf(int b) const
  {
    return model2->getBV(b).isLeaf();
  }

  /// @brief Determine the traversal order, is the first BVTT subtree better
  bool firstOverSecond(int b1, int b2) const
  {
    FCL_REAL sz1 = model1->getBV(b1).bv.size();
    FCL_REAL sz2 = model2->getBV(b2).bv.size();

    bool l1 = model1->getBV(b1).isLeaf();
    bool l2 = model2->getBV(b2).isLeaf();

    if(l2 || (!l1 && (sz1 > sz2)))
      return true;
    return false;
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
    if(enable_statistics) num_bv_tests++;
    return model1->getBV(b1).distance(model2->getBV(b2));
  }

  /// @brief The first BVH model
  const BVHModel<BV>* model1;
  /// @brief The second BVH model
  const BVHModel<BV>* model2;

  /// @brief statistical information
  mutable int num_bv_tests;
  mutable int num_leaf_tests;
  mutable FCL_REAL query_time_seconds;
};


/// @brief Traversal node for distance computation between two meshes
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

    rel_err = this->request.rel_err;
    abs_err = this->request.abs_err;
  }

  /// @brief Distance testing between leaves (two triangles)
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

    FCL_REAL d = TriangleDistance::triDistance(t11, t12, t13, t21, t22, t23,
                                               P1, P2);

    if(this->request.enable_nearest_points)
    {
      this->result->update(d, this->model1, this->model2, primitive_id1, primitive_id2, P1, P2);
    }
    else
    {
      this->result->update(d, this->model1, this->model2, primitive_id1, primitive_id2);
    }
  }

  /// @brief Whether the traversal process can stop early
  bool canStop(FCL_REAL c) const
  {
    if((c >= this->result->min_distance - abs_err) && (c * (1 + rel_err) >= this->result->min_distance))
      return true;
    return false;
  }

  Vec3f* vertices1;
  Vec3f* vertices2;

  Triangle* tri_indices1;
  Triangle* tri_indices2;

  /// @brief relative and absolute error, default value is 0.01 for both terms
  FCL_REAL rel_err;
  FCL_REAL abs_err;
};

/// @brief Traversal node for distance computation between two meshes if their underlying BVH node is oriented node (RSS, OBBRSS, kIOS)
class MeshDistanceTraversalNodeRSS : public MeshDistanceTraversalNode<RSS>
{
public:
  MeshDistanceTraversalNodeRSS();

  void preprocess();

  void postprocess();

  FCL_REAL BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  Matrix3f R;
  Vec3f T;
};


class MeshDistanceTraversalNodekIOS : public MeshDistanceTraversalNode<kIOS>
{
public:
  MeshDistanceTraversalNodekIOS();

  void preprocess();
  
  void postprocess();

  FCL_REAL BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  Matrix3f R;
  Vec3f T;
};

class MeshDistanceTraversalNodeOBBRSS : public MeshDistanceTraversalNode<OBBRSS>
{
public:
  MeshDistanceTraversalNodeOBBRSS();

  void preprocess();

  void postprocess();

  FCL_REAL BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  Matrix3f R;
  Vec3f T;
};



/// @brief continuous collision node using conservative advancement. when using this default version, must refit the BVH in current configuration (R_t, T_t) into default configuration
template<typename BV>
class MeshConservativeAdvancementTraversalNode : public MeshDistanceTraversalNode<BV>
{
public:
  MeshConservativeAdvancementTraversalNode(FCL_REAL w_ = 1) : MeshDistanceTraversalNode<BV>()
  {
    delta_t = 1;
    toc = 0;
    t_err = (FCL_REAL)0.00001;

    w = w_;

    motion1 = NULL;
    motion2 = NULL;
  }

  /// @brief BV culling test in one BVTT node
  FCL_REAL BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    Vec3f P1, P2;
    FCL_REAL d = this->model1->getBV(b1).distance(this->model2->getBV(b2), &P1, &P2);

    stack.push_back(ConservativeAdvancementStackData(P1, P2, b1, b2, d));

    return d;
  }

  /// @brief Conservative advancement testing between leaves (two triangles)
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

    FCL_REAL d = TriangleDistance::triDistance(p1, p2, p3, q1, q2, q3,
                                               P1, P2);

    if(d < this->min_distance)
    {
      this->min_distance = d;

      closest_p1 = P1;
      closest_p2 = P2;
      
      last_tri_id1 = primitive_id1;
      last_tri_id2 = primitive_id2;
    }


    Vec3f n = P2 - P1;
    n.normalize();
    // here n is already in global frame as we assume the body is in original configuration (I, 0) for general BVH
    TriangleMotionBoundVisitor mb_visitor1(p1, p2, p3, n), mb_visitor2(q1, q2, q3, n);
    FCL_REAL bound1 = motion1->computeMotionBound(mb_visitor1);
    FCL_REAL bound2 = motion2->computeMotionBound(mb_visitor2);

    FCL_REAL bound = bound1 + bound2;

    FCL_REAL cur_delta_t;
    if(bound <= d) cur_delta_t = 1;
    else cur_delta_t = d / bound;

    if(cur_delta_t < delta_t)
      delta_t = cur_delta_t;
  }

  /// @brief Whether the traversal process can stop early
  bool canStop(FCL_REAL c) const
  {
    if((c >= w * (this->min_distance - this->abs_err)) && (c * (1 + this->rel_err) >= w * this->min_distance))
    {
      const ConservativeAdvancementStackData& data = stack.back();
      FCL_REAL d = data.d;
      Vec3f n;
      int c1, c2;

      if(d > c)
      {
        const ConservativeAdvancementStackData& data2 = stack[stack.size() - 2];
        d = data2.d;
        n = data2.P2 - data2.P1; n.normalize();
        c1 = data2.c1;
        c2 = data2.c2;
        stack[stack.size() - 2] = stack[stack.size() - 1];
      }
      else
      {
        n = data.P2 - data.P1; n.normalize();
        c1 = data.c1;
        c2 = data.c2;
      }

      assert(c == d);

      TBVMotionBoundVisitor<BV> mb_visitor1(this->model1->getBV(c1).bv, n), mb_visitor2(this->model2->getBV(c2).bv, n);
      FCL_REAL bound1 = motion1->computeMotionBound(mb_visitor1);
      FCL_REAL bound2 = motion2->computeMotionBound(mb_visitor2);

      FCL_REAL bound = bound1 + bound2;

      FCL_REAL cur_delta_t;
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
      FCL_REAL d = data.d;

      if(d > c)
        stack[stack.size() - 2] = stack[stack.size() - 1];

      stack.pop_back();

      return false;
    }
  }

  mutable FCL_REAL min_distance;
 
  mutable Vec3f closest_p1, closest_p2;
  
  mutable int last_tri_id1, last_tri_id2;


  /// @brief CA controlling variable: early stop for the early iterations of CA
  FCL_REAL w;

  /// @brief The time from beginning point
  FCL_REAL toc;
  FCL_REAL t_err;

  /// @brief The delta_t each step
  mutable FCL_REAL delta_t;

  /// @brief Motions for the two objects in query
  const MotionBase* motion1;
  const MotionBase* motion2;

  mutable std::vector<ConservativeAdvancementStackData> stack;
};


/// @brief for OBB and RSS, there is local coordinate of BV, so normal need to be transformed
namespace details
{

template<typename BV>
const Vec3f& getBVAxis(const BV& bv, int i)
{
  return bv.axis[i];
}

template<>
inline const Vec3f& getBVAxis<OBBRSS>(const OBBRSS& bv, int i)
{
  return bv.obb.axis[i];
}


template<typename BV>
bool meshConservativeAdvancementTraversalNodeCanStop(FCL_REAL c,
                                                     FCL_REAL min_distance,
                                                     FCL_REAL abs_err, FCL_REAL rel_err, FCL_REAL w,
                                                     const BVHModel<BV>* model1, const BVHModel<BV>* model2,
                                                     const MotionBase* motion1, const MotionBase* motion2,
                                                     std::vector<ConservativeAdvancementStackData>& stack,
                                                     FCL_REAL& delta_t)
{
  if((c >= w * (min_distance - abs_err)) && (c * (1 + rel_err) >= w * min_distance))
  {
    const ConservativeAdvancementStackData& data = stack.back();
    FCL_REAL d = data.d;
    Vec3f n;
    int c1, c2;

    if(d > c)
    {
      const ConservativeAdvancementStackData& data2 = stack[stack.size() - 2];
      d = data2.d;
      n = data2.P2 - data2.P1; n.normalize();
      c1 = data2.c1;
      c2 = data2.c2;
      stack[stack.size() - 2] = stack[stack.size() - 1];
    }
    else
    {
      n = data.P2 - data.P1; n.normalize();
      c1 = data.c1;
      c2 = data.c2;
    }

    assert(c == d);

    Vec3f n_transformed =
      getBVAxis(model1->getBV(c1).bv, 0) * n[0] +
      getBVAxis(model1->getBV(c1).bv, 1) * n[1] +
      getBVAxis(model1->getBV(c1).bv, 2) * n[2];

    TBVMotionBoundVisitor<BV> mb_visitor1(model1->getBV(c1).bv, n_transformed), mb_visitor2(model2->getBV(c2).bv, n_transformed);
    FCL_REAL bound1 = motion1->computeMotionBound(mb_visitor1);
    FCL_REAL bound2 = motion2->computeMotionBound(mb_visitor2);

    FCL_REAL bound = bound1 + bound2;

    FCL_REAL cur_delta_t;
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
    FCL_REAL d = data.d;

    if(d > c)
      stack[stack.size() - 2] = stack[stack.size() - 1];

    stack.pop_back();

    return false;
  }
}

}

/// for OBB, RSS and OBBRSS, there is local coordinate of BV, so normal need to be transformed
template<>
inline bool MeshConservativeAdvancementTraversalNode<OBB>::canStop(FCL_REAL c) const
{
  return details::meshConservativeAdvancementTraversalNodeCanStop(c, this->min_distance,
                                                                  this->abs_err, this->rel_err, w,
                                                                  this->model1, this->model2,
                                                                  motion1, motion2,
                                                                  stack, delta_t);
}

template<>
inline bool MeshConservativeAdvancementTraversalNode<RSS>::canStop(FCL_REAL c) const
{
  return details::meshConservativeAdvancementTraversalNodeCanStop(c, this->min_distance,
                                                                  this->abs_err, this->rel_err, w,
                                                                  this->model1, this->model2,
                                                                  motion1, motion2,
                                                                  stack, delta_t);
}

template<>
inline bool MeshConservativeAdvancementTraversalNode<OBBRSS>::canStop(FCL_REAL c) const
{
  return details::meshConservativeAdvancementTraversalNodeCanStop(c, this->min_distance,
                                                                  this->abs_err, this->rel_err, w,
                                                                  this->model1, this->model2,
                                                                  motion1, motion2,
                                                                  stack, delta_t);
}


class MeshConservativeAdvancementTraversalNodeRSS : public MeshConservativeAdvancementTraversalNode<RSS>
{
public:
  MeshConservativeAdvancementTraversalNodeRSS(FCL_REAL w_ = 1);

  FCL_REAL BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  bool canStop(FCL_REAL c) const;

  Matrix3f R;
  Vec3f T;
};

class MeshConservativeAdvancementTraversalNodeOBBRSS : public MeshConservativeAdvancementTraversalNode<OBBRSS>
{
public:
  MeshConservativeAdvancementTraversalNodeOBBRSS(FCL_REAL w_ = 1);

  FCL_REAL BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  bool canStop(FCL_REAL c) const;

  Matrix3f R;
  Vec3f T;
};
}


#endif

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

#ifndef FCL_TRAVERSAL_MESHCONTINUOUSCOLLISIONTRAVERSALNODE_INL_H
#define FCL_TRAVERSAL_MESHCONTINUOUSCOLLISIONTRAVERSALNODE_INL_H

#include "fcl/narrowphase/detail/traversal/collision/mesh_continuous_collision_traversal_node.h"

#include "fcl/narrowphase/detail/traversal/collision/intersect.h"

namespace fcl
{

namespace detail
{

//==============================================================================
extern template
struct BVHContinuousCollisionPair<double>;

//==============================================================================
template <typename S>
BVHContinuousCollisionPair<S>::BVHContinuousCollisionPair()
{
  // Do nothing
}

//==============================================================================
template <typename S>
BVHContinuousCollisionPair<S>::BVHContinuousCollisionPair(
    int id1_, int id2_, S time)
  : id1(id1_), id2(id2_), collision_time(time)
{
  // Do nothing
}

//==============================================================================
template <typename BV>
MeshContinuousCollisionTraversalNode<BV>::MeshContinuousCollisionTraversalNode()
  : BVHCollisionTraversalNode<BV>()
{
  vertices1 = nullptr;
  vertices2 = nullptr;
  tri_indices1 = nullptr;
  tri_indices2 = nullptr;
  prev_vertices1 = nullptr;
  prev_vertices2 = nullptr;

  num_vf_tests = 0;
  num_ee_tests = 0;
  time_of_contact = 1;
}

//==============================================================================
template <typename BV>
void MeshContinuousCollisionTraversalNode<BV>::leafTesting(int b1, int b2) const
{
  if(this->enable_statistics) this->num_leaf_tests++;

  const BVNode<BV>& node1 = this->model1->getBV(b1);
  const BVNode<BV>& node2 = this->model2->getBV(b2);

  S collision_time = 2;
  Vector3<S> collision_pos;

  int primitive_id1 = node1.primitiveId();
  int primitive_id2 = node2.primitiveId();

  const Triangle& tri_id1 = tri_indices1[primitive_id1];
  const Triangle& tri_id2 = tri_indices2[primitive_id2];

  Vector3<S>* S0[3];
  Vector3<S>* S1[3];
  Vector3<S>* T0[3];
  Vector3<S>* T1[3];

  for(int i = 0; i < 3; ++i)
  {
    S0[i] = prev_vertices1 + tri_id1[i];
    S1[i] = vertices1 + tri_id1[i];
    T0[i] = prev_vertices2 + tri_id2[i];
    T1[i] = vertices2 + tri_id2[i];
  }

  S tmp;
  Vector3<S> tmpv;

  // 6 VF checks
  for(int i = 0; i < 3; ++i)
  {
    if(this->enable_statistics) num_vf_tests++;
    if(Intersect<S>::intersect_VF(*(S0[0]), *(S0[1]), *(S0[2]), *(T0[i]), *(S1[0]), *(S1[1]), *(S1[2]), *(T1[i]), &tmp, &tmpv))
    {
      if(collision_time > tmp)
      {
        collision_time = tmp; collision_pos = tmpv;
      }
    }

    if(this->enable_statistics) num_vf_tests++;
    if(Intersect<S>::intersect_VF(*(T0[0]), *(T0[1]), *(T0[2]), *(S0[i]), *(T1[0]), *(T1[1]), *(T1[2]), *(S1[i]), &tmp, &tmpv))
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
      if(Intersect<S>::intersect_EE(*(S0[S_id1]), *(S0[S_id2]), *(T0[T_id1]), *(T0[T_id2]), *(S1[S_id1]), *(S1[S_id2]), *(T1[T_id1]), *(T1[T_id2]), &tmp, &tmpv))
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
    pairs.emplace_back(primitive_id1, primitive_id2, collision_time);
    time_of_contact = std::min(time_of_contact, collision_time);
  }
}

//==============================================================================
template <typename BV>
bool MeshContinuousCollisionTraversalNode<BV>::canStop() const
{
  return (pairs.size() > 0) && (this->request.num_max_contacts <= pairs.size());
}

//==============================================================================
template <typename BV>
bool initialize(
    MeshContinuousCollisionTraversalNode<BV>& node,
    const BVHModel<BV>& model1,
    const Transform3<typename BV::S>& tf1,
    const BVHModel<BV>& model2,
    const Transform3<typename BV::S>& tf2,
    const CollisionRequest<typename BV::S>& request)
{
  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;

  node.vertices1 = model1.vertices;
  node.vertices2 = model2.vertices;

  node.tri_indices1 = model1.tri_indices;
  node.tri_indices2 = model2.tri_indices;

  node.prev_vertices1 = model1.prev_vertices;
  node.prev_vertices2 = model2.prev_vertices;

  node.request = request;

  return true;
}

} // namespace detail
} // namespace fcl

#endif

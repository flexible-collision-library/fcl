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

#include "fcl/broadphase/hierarchy_tree.h"

namespace fcl
{

template<>
size_t select(const NodeBase<AABBd>& node, const NodeBase<AABBd>& node1, const NodeBase<AABBd>& node2)
{
  const AABBd& bv = node.bv;
  const AABBd& bv1 = node1.bv;
  const AABBd& bv2 = node2.bv;
  Vector3d v = bv.min_ + bv.max_;
  Vector3d v1 = v - (bv1.min_ + bv1.max_);
  Vector3d v2 = v - (bv2.min_ + bv2.max_);
  FCL_REAL d1 = fabs(v1[0]) + fabs(v1[1]) + fabs(v1[2]);
  FCL_REAL d2 = fabs(v2[0]) + fabs(v2[1]) + fabs(v2[2]);
  return (d1 < d2) ? 0 : 1;
}

template<>
size_t select(const AABBd& query, const NodeBase<AABBd>& node1, const NodeBase<AABBd>& node2)
{
  const AABBd& bv = query;
  const AABBd& bv1 = node1.bv;
  const AABBd& bv2 = node2.bv;
  Vector3d v = bv.min_ + bv.max_;
  Vector3d v1 = v - (bv1.min_ + bv1.max_);
  Vector3d v2 = v - (bv2.min_ + bv2.max_);
  FCL_REAL d1 = fabs(v1[0]) + fabs(v1[1]) + fabs(v1[2]);
  FCL_REAL d2 = fabs(v2[0]) + fabs(v2[1]) + fabs(v2[2]);
  return (d1 < d2) ? 0 : 1;
}

template<>
bool HierarchyTree<AABBd>::update(NodeBase<AABBd>* leaf, const AABBd& bv_, const Vector3d& vel, FCL_REAL margin)
{
  AABBd bv(bv_);
  if(leaf->bv.contain(bv)) return false;
  Vector3d marginv = Vector3d::Constant(margin);
  bv.min_ -= marginv;
  bv.max_ += marginv;
  if(vel[0] > 0) bv.max_[0] += vel[0];
  else bv.min_[0] += vel[0];
  if(vel[1] > 0) bv.max_[1] += vel[1];
  else bv.min_[1] += vel[1];
  if(vel[2] > 0) bv.max_[2] += vel[2];
  else bv.max_[2] += vel[2];
  update(leaf, bv);
  return true;
}

template<>
bool HierarchyTree<AABBd>::update(NodeBase<AABBd>* leaf, const AABBd& bv_, const Vector3d& vel)
{
  AABBd bv(bv_);
  if(leaf->bv.contain(bv)) return false;
  if(vel[0] > 0) bv.max_[0] += vel[0];
  else bv.min_[0] += vel[0];
  if(vel[1] > 0) bv.max_[1] += vel[1];
  else bv.min_[1] += vel[1];
  if(vel[2] > 0) bv.max_[2] += vel[2];
  else bv.max_[2] += vel[2];
  update(leaf, bv);
  return true;
}

namespace implementation_array
{
template<>
size_t select(size_t query, size_t node1, size_t node2, NodeBase<AABBd>* nodes)
{
  const AABBd& bv = nodes[query].bv;
  const AABBd& bv1 = nodes[node1].bv;
  const AABBd& bv2 = nodes[node2].bv;
  Vector3d v = bv.min_ + bv.max_;
  Vector3d v1 = v - (bv1.min_ + bv1.max_);
  Vector3d v2 = v - (bv2.min_ + bv2.max_);
  FCL_REAL d1 = fabs(v1[0]) + fabs(v1[1]) + fabs(v1[2]);
  FCL_REAL d2 = fabs(v2[0]) + fabs(v2[1]) + fabs(v2[2]);
  return (d1 < d2) ? 0 : 1;
}

template<>
size_t select(const AABBd& query, size_t node1, size_t node2, NodeBase<AABBd>* nodes)
{
  const AABBd& bv = query;
  const AABBd& bv1 = nodes[node1].bv;
  const AABBd& bv2 = nodes[node2].bv;
  Vector3d v = bv.min_ + bv.max_;
  Vector3d v1 = v - (bv1.min_ + bv1.max_);
  Vector3d v2 = v - (bv2.min_ + bv2.max_);
  FCL_REAL d1 = fabs(v1[0]) + fabs(v1[1]) + fabs(v1[2]);
  FCL_REAL d2 = fabs(v2[0]) + fabs(v2[1]) + fabs(v2[2]);
  return (d1 < d2) ? 0 : 1;
}

}

}

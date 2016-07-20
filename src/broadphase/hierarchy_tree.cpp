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
size_t select(const NodeBase<AABB>& node, const NodeBase<AABB>& node1, const NodeBase<AABB>& node2)
{
  const AABB& bv = node.bv;
  const AABB& bv1 = node1.bv;
  const AABB& bv2 = node2.bv;
  Vec3f v = bv.min_ + bv.max_;
  Vec3f v1 = v - (bv1.min_ + bv1.max_);
  Vec3f v2 = v - (bv2.min_ + bv2.max_);
  FCL_REAL d1 = fabs(v1[0]) + fabs(v1[1]) + fabs(v1[2]);
  FCL_REAL d2 = fabs(v2[0]) + fabs(v2[1]) + fabs(v2[2]);
  return (d1 < d2) ? 0 : 1;
}

template<>
size_t select(const AABB& query, const NodeBase<AABB>& node1, const NodeBase<AABB>& node2)
{
  const AABB& bv = query;
  const AABB& bv1 = node1.bv;
  const AABB& bv2 = node2.bv;
  Vec3f v = bv.min_ + bv.max_;
  Vec3f v1 = v - (bv1.min_ + bv1.max_);
  Vec3f v2 = v - (bv2.min_ + bv2.max_);
  FCL_REAL d1 = fabs(v1[0]) + fabs(v1[1]) + fabs(v1[2]);
  FCL_REAL d2 = fabs(v2[0]) + fabs(v2[1]) + fabs(v2[2]);
  return (d1 < d2) ? 0 : 1;
}

template<>
bool HierarchyTree<AABB>::update(NodeBase<AABB>* leaf, const AABB& bv_, const Vec3f& vel, FCL_REAL margin)
{
  AABB bv(bv_);
  if(leaf->bv.contain(bv)) return false;
  Vec3f marginv(margin);
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
bool HierarchyTree<AABB>::update(NodeBase<AABB>* leaf, const AABB& bv_, const Vec3f& vel)
{
  AABB bv(bv_);
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
size_t select(size_t query, size_t node1, size_t node2, NodeBase<AABB>* nodes)
{
  const AABB& bv = nodes[query].bv;
  const AABB& bv1 = nodes[node1].bv;
  const AABB& bv2 = nodes[node2].bv;
  Vec3f v = bv.min_ + bv.max_;
  Vec3f v1 = v - (bv1.min_ + bv1.max_);
  Vec3f v2 = v - (bv2.min_ + bv2.max_);
  FCL_REAL d1 = fabs(v1[0]) + fabs(v1[1]) + fabs(v1[2]);
  FCL_REAL d2 = fabs(v2[0]) + fabs(v2[1]) + fabs(v2[2]);
  return (d1 < d2) ? 0 : 1;
}

template<>
size_t select(const AABB& query, size_t node1, size_t node2, NodeBase<AABB>* nodes)
{
  const AABB& bv = query;
  const AABB& bv1 = nodes[node1].bv;
  const AABB& bv2 = nodes[node2].bv;
  Vec3f v = bv.min_ + bv.max_;
  Vec3f v1 = v - (bv1.min_ + bv1.max_);
  Vec3f v2 = v - (bv2.min_ + bv2.max_);
  FCL_REAL d1 = fabs(v1[0]) + fabs(v1[1]) + fabs(v1[2]);
  FCL_REAL d2 = fabs(v2[0]) + fabs(v2[1]) + fabs(v2[2]);
  return (d1 < d2) ? 0 : 1;
}

}

}

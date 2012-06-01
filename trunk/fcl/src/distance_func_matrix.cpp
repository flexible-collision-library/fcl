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

#include "fcl/distance_func_matrix.h"

#include "fcl/collision_node.h"
#include "fcl/simple_setup.h"

namespace fcl
{

template<typename T_BVH>
BVH_REAL BVHDistance(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2)
{
  MeshDistanceTraversalNode<T_BVH> node;
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
  const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>* >(o2);
  BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
  SimpleTransform tf1_tmp = tf1;
  BVHModel<T_BVH>* obj2_tmp = new BVHModel<T_BVH>(*obj2);
  SimpleTransform tf2_tmp = tf2;

  initialize(node, *obj1_tmp, tf1_tmp, *obj2_tmp, tf2_tmp);
  distance(&node);
  
  return node.min_distance;
}

template<>
BVH_REAL BVHDistance<RSS>(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2)
{
  MeshDistanceTraversalNodeRSS node;
  const BVHModel<RSS>* obj1 = static_cast<const BVHModel<RSS>* >(o1);
  const BVHModel<RSS>* obj2 = static_cast<const BVHModel<RSS>* >(o2);

  initialize(node, *obj1, tf1, *obj2, tf2);
  distance(&node);

  return node.min_distance;
}

template<>
BVH_REAL BVHDistance<kIOS>(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2)
{
  MeshDistanceTraversalNodekIOS node;
  const BVHModel<kIOS>* obj1 = static_cast<const BVHModel<kIOS>* >(o1);
  const BVHModel<kIOS>* obj2 = static_cast<const BVHModel<kIOS>* >(o2);

  initialize(node, *obj1, tf1, *obj2, tf2);
  distance(&node);

  return node.min_distance;
}


template<>
BVH_REAL BVHDistance<OBBRSS>(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2)
{
  MeshDistanceTraversalNodeOBBRSS node;
  const BVHModel<OBBRSS>* obj1 = static_cast<const BVHModel<OBBRSS>* >(o1);
  const BVHModel<OBBRSS>* obj2 = static_cast<const BVHModel<OBBRSS>* >(o2);

  initialize(node, *obj1, tf1, *obj2, tf2);
  distance(&node);

  return node.min_distance;
}

DistanceFunctionMatrix::DistanceFunctionMatrix()
{
  for(int i = 0; i < 16; ++i)
  {
    for(int j = 0; j < 16; ++j)
      distance_matrix[i][j] = NULL;
  }

  distance_matrix[BV_AABB][BV_AABB] = &BVHDistance<AABB>;
  distance_matrix[BV_RSS][BV_RSS] = &BVHDistance<RSS>;
  distance_matrix[BV_kIOS][BV_kIOS] = &BVHDistance<kIOS>;
  distance_matrix[BV_OBBRSS][BV_OBBRSS] = &BVHDistance<OBBRSS>;
}


}

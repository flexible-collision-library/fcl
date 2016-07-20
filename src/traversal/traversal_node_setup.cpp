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


#include "fcl/traversal/traversal_node_setup.h"

namespace fcl
{


namespace details
{
template<typename BV, typename OrientedNode>
static inline bool setupMeshCollisionOrientedNode(OrientedNode& node,
                                                  const BVHModel<BV>& model1, const Transform3f& tf1,
                                                  const BVHModel<BV>& model2, const Transform3f& tf2,
                                                  const CollisionRequest& request,
                                                  CollisionResult& result)
{
  if(model1.getModelType() != BVH_MODEL_TRIANGLES || model2.getModelType() != BVH_MODEL_TRIANGLES)
    return false;

  node.vertices1 = model1.vertices;
  node.vertices2 = model2.vertices;

  node.tri_indices1 = model1.tri_indices;
  node.tri_indices2 = model2.tri_indices;

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;

  node.request = request;
  node.result = &result;

  node.cost_density = model1.cost_density * model2.cost_density;

  relativeTransform(tf1.getRotation(), tf1.getTranslation(), tf2.getRotation(), tf2.getTranslation(), node.R, node.T);

  return true;
}

}


bool initialize(MeshCollisionTraversalNodeOBB& node,
                const BVHModel<OBB>& model1, const Transform3f& tf1,
                const BVHModel<OBB>& model2, const Transform3f& tf2,
                const CollisionRequest& request,
                CollisionResult& result)
{
  return details::setupMeshCollisionOrientedNode(node, model1, tf1, model2, tf2, request, result);
}


bool initialize(MeshCollisionTraversalNodeRSS& node,
                const BVHModel<RSS>& model1, const Transform3f& tf1,
                const BVHModel<RSS>& model2, const Transform3f& tf2,
                const CollisionRequest& request,
                CollisionResult& result)
{
  return details::setupMeshCollisionOrientedNode(node, model1, tf1, model2, tf2, request, result);
}


bool initialize(MeshCollisionTraversalNodekIOS& node,
                const BVHModel<kIOS>& model1, const Transform3f& tf1,
                const BVHModel<kIOS>& model2, const Transform3f& tf2,
                const CollisionRequest& request,
                CollisionResult& result)
{
  return details::setupMeshCollisionOrientedNode(node, model1, tf1, model2, tf2, request, result);
}

bool initialize(MeshCollisionTraversalNodeOBBRSS& node,
                const BVHModel<OBBRSS>& model1, const Transform3f& tf1,
                const BVHModel<OBBRSS>& model2, const Transform3f& tf2,
                const CollisionRequest& request,
                CollisionResult& result)
{
  return details::setupMeshCollisionOrientedNode(node, model1, tf1, model2, tf2, request, result);
}


namespace details
{
template<typename BV, typename OrientedNode>
static inline bool setupMeshDistanceOrientedNode(OrientedNode& node,
                                                 const BVHModel<BV>& model1, const Transform3f& tf1,
                                                 const BVHModel<BV>& model2, const Transform3f& tf2,
                                                 const DistanceRequest& request,
                                                 DistanceResult& result)
{
  if(model1.getModelType() != BVH_MODEL_TRIANGLES || model2.getModelType() != BVH_MODEL_TRIANGLES)
    return false;

  node.request = request;
  node.result = &result;

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;

  node.vertices1 = model1.vertices;
  node.vertices2 = model2.vertices;

  node.tri_indices1 = model1.tri_indices;
  node.tri_indices2 = model2.tri_indices;

  relativeTransform(tf1.getRotation(), tf1.getTranslation(), tf2.getRotation(), tf2.getTranslation(), node.R, node.T);

  return true;
}


}

bool initialize(MeshDistanceTraversalNodeRSS& node,
                const BVHModel<RSS>& model1, const Transform3f& tf1,
                const BVHModel<RSS>& model2, const Transform3f& tf2,
                const DistanceRequest& request,
                DistanceResult& result)
{
  return details::setupMeshDistanceOrientedNode(node, model1, tf1, model2, tf2, request, result);
}


bool initialize(MeshDistanceTraversalNodekIOS& node,
                const BVHModel<kIOS>& model1, const Transform3f& tf1,
                const BVHModel<kIOS>& model2, const Transform3f& tf2,
                const DistanceRequest& request,
                DistanceResult& result)
{
  return details::setupMeshDistanceOrientedNode(node, model1, tf1, model2, tf2, request, result);
}

bool initialize(MeshDistanceTraversalNodeOBBRSS& node,
                const BVHModel<OBBRSS>& model1, const Transform3f& tf1,
                const BVHModel<OBBRSS>& model2, const Transform3f& tf2,
                const DistanceRequest& request,
                DistanceResult& result)
{
  return details::setupMeshDistanceOrientedNode(node, model1, tf1, model2, tf2, request, result);
}

namespace details
{


template<typename BV, typename OrientedDistanceNode>
static inline bool setupMeshConservativeAdvancementOrientedDistanceNode(OrientedDistanceNode& node,
                                                                        const BVHModel<BV>& model1, const Transform3f& tf1,
                                                                        const BVHModel<BV>& model2, const Transform3f& tf2,
                                                                        FCL_REAL w)
{
  if(model1.getModelType() != BVH_MODEL_TRIANGLES || model2.getModelType() != BVH_MODEL_TRIANGLES)
    return false;

  node.model1 = &model1;
  node.model2 = &model2;

  node.vertices1 = model1.vertices;
  node.vertices2 = model2.vertices;

  node.tri_indices1 = model1.tri_indices;
  node.tri_indices2 = model2.tri_indices;

  node.w = w;

  relativeTransform(tf1.getRotation(), tf1.getTranslation(), tf2.getRotation(), tf2.getTranslation(), node.R, node.T);

  return true;
}

}


bool initialize(MeshConservativeAdvancementTraversalNodeRSS& node,
                const BVHModel<RSS>& model1, const Transform3f& tf1,
                const BVHModel<RSS>& model2, const Transform3f& tf2,
                FCL_REAL w)
{
  return details::setupMeshConservativeAdvancementOrientedDistanceNode(node, model1, tf1, model2, tf2, w);
}


bool initialize(MeshConservativeAdvancementTraversalNodeOBBRSS& node,
                const BVHModel<OBBRSS>& model1, const Transform3f& tf1,
                const BVHModel<OBBRSS>& model2, const Transform3f& tf2,
                FCL_REAL w)
{
  return details::setupMeshConservativeAdvancementOrientedDistanceNode(node, model1, tf1, model2, tf2, w);
}




}

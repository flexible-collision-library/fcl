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


#include "fcl/simple_setup.h"

namespace fcl
{


namespace details
{
template<typename BV, typename OrientedNode>
static inline bool setupMeshCollisionOrientedNode(OrientedNode& node,
                                                  const BVHModel<BV>& model1, const SimpleTransform& tf1,
                                                  const BVHModel<BV>& model2, const SimpleTransform& tf2,
                                                  const CollisionRequest& request)
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

  relativeTransform(tf1.getRotation(), tf1.getTranslation(), tf2.getRotation(), tf2.getTranslation(), node.R, node.T);

  return true;
}

}


bool initialize(MeshCollisionTraversalNodeOBB& node,
                const BVHModel<OBB>& model1, const SimpleTransform& tf1,
                const BVHModel<OBB>& model2, const SimpleTransform& tf2,
                const CollisionRequest& request)
{
  return details::setupMeshCollisionOrientedNode(node, model1, tf1, model2, tf2, request);
}


bool initialize(MeshCollisionTraversalNodeRSS& node,
                const BVHModel<RSS>& model1, const SimpleTransform& tf1,
                const BVHModel<RSS>& model2, const SimpleTransform& tf2,
                const CollisionRequest& request)
{
  return details::setupMeshCollisionOrientedNode(node, model1, tf1, model2, tf2, request);
}


bool initialize(MeshCollisionTraversalNodekIOS& node,
                const BVHModel<kIOS>& model1, const SimpleTransform& tf1,
                const BVHModel<kIOS>& model2, const SimpleTransform& tf2,
                const CollisionRequest& request)
{
  return details::setupMeshCollisionOrientedNode(node, model1, tf1, model2, tf2, request);
}

bool initialize(MeshCollisionTraversalNodeOBBRSS& node,
                const BVHModel<OBBRSS>& model1, const SimpleTransform& tf1,
                const BVHModel<OBBRSS>& model2, const SimpleTransform& tf2,
                const CollisionRequest& request)
{
  return details::setupMeshCollisionOrientedNode(node, model1, tf1, model2, tf2, request);
}


#if USE_SVMLIGHT

namespace details
{
template<typename BV, typename OrientedNode>
static inline bool setupPointCloudCollisionOrientedNode(OrientedNode& node, 
                                                        BVHModel<BV>& model1, const SimpleTransform& tf1,
                                                        BVHModel<BV>& model2, const SimpleTransform& tf2,
                                                        const CollisionRequest& request,
                                                        FCL_REAL collision_prob_threshold,
                                                        int leaf_size_threshold,
                                                        FCL_REAL expand_r)
{
  if(!(model1.getModelType() == BVH_MODEL_TRIANGLES || model1.getModelType() == BVH_MODEL_POINTCLOUD)
      || !(model2.getModelType() == BVH_MODEL_TRIANGLES || model2.getModelType() == BVH_MODEL_POINTCLOUD))
    return false;

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;

  node.vertices1 = model1.vertices;
  node.vertices2 = model2.vertices;

  node.uc1.reset(new Uncertainty[model1.num_vertices]);
  node.uc2.reset(new Uncertainty[model2.num_vertices]);

  estimateSamplingUncertainty(model1.vertices, model1.num_vertices, node.uc1.get());
  estimateSamplingUncertainty(model2.vertices, model2.num_vertices, node.uc2.get());

  BVHExpand(model1, node.uc1.get(), expand_r);
  BVHExpand(model2, node.uc2.get(), expand_r);

  node.request = request;
  node.collision_prob_threshold = collision_prob_threshold;
  node.leaf_size_threshold = leaf_size_threshold;

  relativeTransform(tf1.getRotation(), tf1.getTranslation(), tf2.getRotation(), tf2.getTranslation(), node.R, node.T);

  return true;  
}

}

bool initialize(PointCloudCollisionTraversalNodeOBB& node,
                BVHModel<OBB>& model1, const SimpleTransform& tf1,
                BVHModel<OBB>& model2, const SimpleTransform& tf2,
                const CollisionRequest& request,
                FCL_REAL collision_prob_threshold,
                int leaf_size_threshold,
                FCL_REAL expand_r)
{
  return details::setupPointCloudCollisionOrientedNode(node, model1, tf1, model2, tf2, request, collision_prob_threshold, leaf_size_threshold, expand_r);
}


bool initialize(PointCloudCollisionTraversalNodeRSS& node,
                BVHModel<RSS>& model1, const SimpleTransform& tf1,
                BVHModel<RSS>& model2, const SimpleTransform& tf2,
                const CollisionRequest& request,
                FCL_REAL collision_prob_threshold,
                int leaf_size_threshold,
                FCL_REAL expand_r)
{
  return details::setupPointCloudCollisionOrientedNode(node, model1, tf1, model2, tf2, request, collision_prob_threshold, leaf_size_threshold, expand_r);
}

namespace details
{

template<typename BV, typename OrientedNode>
static inline bool setupPointCloudMeshCollisionOrientedNode(OrientedNode& node,
                                                            BVHModel<BV>& model1, const SimpleTransform& tf1,
                                                            const BVHModel<BV>& model2, const SimpleTransform& tf2,
                                                            const CollisionRequest& request,
                                                            FCL_REAL collision_prob_threshold,
                                                            int leaf_size_threshold,
                                                            FCL_REAL expand_r)
{
  if(!(model1.getModelType() == BVH_MODEL_TRIANGLES || model1.getModelType() == BVH_MODEL_POINTCLOUD) || model2.getModelType() != BVH_MODEL_TRIANGLES)
    return false;

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;

  node.vertices1 = model1.vertices;
  node.vertices2 = model2.vertices;

  node.tri_indices2 = model2.tri_indices;
  node.uc1.reset(new Uncertainty[model1.num_vertices]);

  estimateSamplingUncertainty(model1.vertices, model1.num_vertices, node.uc1.get());

  BVHExpand(model1, node.uc1.get(), expand_r);

  node.request = request;
  node.collision_prob_threshold = collision_prob_threshold;
  node.leaf_size_threshold = leaf_size_threshold;

  relativeTransform(tf1.getRotation(), tf1.getTranslation(), tf2.getRotation(), tf2.getTranslation(), node.R, node.T);

  return true;
}                                                           
}

bool initialize(PointCloudMeshCollisionTraversalNodeOBB& node,
                BVHModel<OBB>& model1, const SimpleTransform& tf1,
                const BVHModel<OBB>& model2, const SimpleTransform& tf2,
                const CollisionRequest& request,
                FCL_REAL collision_prob_threshold,
                int leaf_size_threshold,
                FCL_REAL expand_r)
{
  return details::setupPointCloudMeshCollisionOrientedNode(node, model1, tf1, model2, tf2, request, collision_prob_threshold, leaf_size_threshold, expand_r);
}


bool initialize(PointCloudMeshCollisionTraversalNodeRSS& node,
                BVHModel<RSS>& model1, const SimpleTransform& tf1,
                const BVHModel<RSS>& model2, const SimpleTransform& tf2,
                const CollisionRequest& request,
                FCL_REAL collision_prob_threshold,
                int leaf_size_threshold,
                FCL_REAL expand_r)
{
  return details::setupPointCloudMeshCollisionOrientedNode(node, model1, tf1, model2, tf2, request, collision_prob_threshold, leaf_size_threshold, expand_r);
}

#endif


namespace details
{
template<typename BV, typename OrientedNode>
static inline bool setupMeshDistanceOrientedNode(OrientedNode& node,
                                                 const BVHModel<BV>& model1, const SimpleTransform& tf1,
                                                 const BVHModel<BV>& model2, const SimpleTransform& tf2,
                                                 const DistanceRequest& request)
{
  if(model1.getModelType() != BVH_MODEL_TRIANGLES || model2.getModelType() != BVH_MODEL_TRIANGLES)
    return false;

  node.request = request;

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
                const BVHModel<RSS>& model1, const SimpleTransform& tf1,
                const BVHModel<RSS>& model2, const SimpleTransform& tf2,
                const DistanceRequest& request)
{
  return details::setupMeshDistanceOrientedNode(node, model1, tf1, model2, tf2, request);
}


bool initialize(MeshDistanceTraversalNodekIOS& node,
                const BVHModel<kIOS>& model1, const SimpleTransform& tf1,
                const BVHModel<kIOS>& model2, const SimpleTransform& tf2,
                const DistanceRequest& request)
{
  return details::setupMeshDistanceOrientedNode(node, model1, tf1, model2, tf2, request);
}

bool initialize(MeshDistanceTraversalNodeOBBRSS& node,
                const BVHModel<OBBRSS>& model1, const SimpleTransform& tf1,
                const BVHModel<OBBRSS>& model2, const SimpleTransform& tf2,
                const DistanceRequest& request)
{
  return details::setupMeshDistanceOrientedNode(node, model1, tf1, model2, tf2, request);
}


}

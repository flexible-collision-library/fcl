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

bool initialize(MeshCollisionTraversalNodeOBB& node, const BVHModel<OBB>& model1, const BVHModel<OBB>& model2,
                int num_max_contacts, bool exhaustive, bool enable_contact)
{
  if(model1.getModelType() != BVH_MODEL_TRIANGLES || model2.getModelType() != BVH_MODEL_TRIANGLES)
    return false;

  node.vertices1 = model1.vertices;
  node.vertices2 = model2.vertices;

  node.tri_indices1 = model1.tri_indices;
  node.tri_indices2 = model2.tri_indices;

  node.model1 = &model1;
  node.model2 = &model2;

  node.num_max_contacts = num_max_contacts;
  node.exhaustive = exhaustive;
  node.enable_contact = enable_contact;

  relativeTransform(model1.getRotation(), model1.getTranslation(), model2.getRotation(), model2.getTranslation(), node.R, node.T);

  return true;
}


bool initialize(MeshCollisionTraversalNodeRSS& node, const BVHModel<RSS>& model1, const BVHModel<RSS>& model2,
                int num_max_contacts, bool exhaustive, bool enable_contact)
{
  if(model1.getModelType() != BVH_MODEL_TRIANGLES || model2.getModelType() != BVH_MODEL_TRIANGLES)
    return false;

  node.vertices1 = model1.vertices;
  node.vertices2 = model2.vertices;

  node.tri_indices1 = model1.tri_indices;
  node.tri_indices2 = model2.tri_indices;

  node.model1 = &model1;
  node.model2 = &model2;

  node.num_max_contacts = num_max_contacts;
  node.exhaustive = exhaustive;
  node.enable_contact = enable_contact;

  relativeTransform(model1.getRotation(), model1.getTranslation(), model2.getRotation(), model2.getTranslation(), node.R, node.T);

  return true;
}

#if USE_SVMLIGHT

bool initialize(PointCloudCollisionTraversalNodeOBB& node, BVHModel<OBB>& model1, BVHModel<OBB>& model2,
                BVH_REAL collision_prob_threshold,
                int leaf_size_threshold,
                int num_max_contacts,
                bool exhaustive,
                bool enable_contact,
                BVH_REAL expand_r)
{
  if(!(model1.getModelType() == BVH_MODEL_TRIANGLES || model1.getModelType() == BVH_MODEL_POINTCLOUD)
      || !(model2.getModelType() == BVH_MODEL_TRIANGLES || model2.getModelType() == BVH_MODEL_POINTCLOUD))
    return false;

  node.model1 = &model1;
  node.model2 = &model2;

  node.vertices1 = model1.vertices;
  node.vertices2 = model2.vertices;

  node.uc1.reset(new Uncertainty[model1.num_vertices]);
  node.uc2.reset(new Uncertainty[model2.num_vertices]);

  estimateSamplingUncertainty(model1.vertices, model1.num_vertices, node.uc1.get());
  estimateSamplingUncertainty(model2.vertices, model2.num_vertices, node.uc2.get());

  BVHExpand(model1, node.uc1.get(), expand_r);
  BVHExpand(model2, node.uc2.get(), expand_r);

  node.num_max_contacts = num_max_contacts;
  node.exhaustive = exhaustive;
  node.enable_contact = enable_contact;
  node.collision_prob_threshold = collision_prob_threshold;
  node.leaf_size_threshold = leaf_size_threshold;

  relativeTransform(model1.getRotation(), model1.getTranslation(), model2.getRotation(), model2.getTranslation(), node.R, node.T);

  return true;
}


bool initialize(PointCloudCollisionTraversalNodeRSS& node, BVHModel<RSS>& model1, BVHModel<RSS>& model2,
                BVH_REAL collision_prob_threshold,
                int leaf_size_threshold,
                int num_max_contacts,
                bool exhaustive,
                bool enable_contact,
                BVH_REAL expand_r)
{
  if(!(model1.getModelType() == BVH_MODEL_TRIANGLES || model1.getModelType() == BVH_MODEL_POINTCLOUD)
      || !(model2.getModelType() == BVH_MODEL_TRIANGLES || model2.getModelType() == BVH_MODEL_POINTCLOUD))
    return false;

  node.model1 = &model1;
  node.model2 = &model2;

  node.vertices1 = model1.vertices;
  node.vertices2 = model2.vertices;

  node.uc1.reset(new Uncertainty[model1.num_vertices]);
  node.uc2.reset(new Uncertainty[model2.num_vertices]);

  estimateSamplingUncertainty(model1.vertices, model1.num_vertices, node.uc1.get());
  estimateSamplingUncertainty(model2.vertices, model2.num_vertices, node.uc2.get());

  BVHExpand(model1, node.uc1.get(), expand_r);
  BVHExpand(model2, node.uc2.get(), expand_r);

  node.num_max_contacts = num_max_contacts;
  node.exhaustive = exhaustive;
  node.enable_contact = enable_contact;
  node.collision_prob_threshold = collision_prob_threshold;
  node.leaf_size_threshold = leaf_size_threshold;

  relativeTransform(model1.getRotation(), model1.getTranslation(), model2.getRotation(), model2.getTranslation(), node.R, node.T);

  return true;
}

bool initialize(PointCloudMeshCollisionTraversalNodeOBB& node, BVHModel<OBB>& model1, const BVHModel<OBB>& model2,
                BVH_REAL collision_prob_threshold,
                int leaf_size_threshold,
                int num_max_contacts,
                bool exhaustive,
                bool enable_contact,
                BVH_REAL expand_r)
{
  if(!(model1.getModelType() == BVH_MODEL_TRIANGLES || model1.getModelType() == BVH_MODEL_POINTCLOUD) || model2.getModelType() != BVH_MODEL_TRIANGLES)
    return false;

  node.model1 = &model1;
  node.model2 = &model2;

  node.vertices1 = model1.vertices;
  node.vertices2 = model2.vertices;

  node.tri_indices2 = model2.tri_indices;
  node.uc1.reset(new Uncertainty[model1.num_vertices]);

  estimateSamplingUncertainty(model1.vertices, model1.num_vertices, node.uc1.get());

  BVHExpand(model1, node.uc1.get(), expand_r);

  node.num_max_contacts = num_max_contacts;
  node.exhaustive = exhaustive;
  node.enable_contact = enable_contact;
  node.collision_prob_threshold = collision_prob_threshold;
  node.leaf_size_threshold = leaf_size_threshold;

  relativeTransform(model1.getRotation(), model1.getTranslation(), model2.getRotation(), model2.getTranslation(), node.R, node.T);

  return true;
}


bool initialize(PointCloudMeshCollisionTraversalNodeRSS& node, BVHModel<RSS>& model1, const BVHModel<RSS>& model2,
                BVH_REAL collision_prob_threshold,
                int leaf_size_threshold,
                int num_max_contacts,
                bool exhaustive,
                bool enable_contact,
                BVH_REAL expand_r)
{
  if(!(model1.getModelType() == BVH_MODEL_TRIANGLES || model1.getModelType() == BVH_MODEL_POINTCLOUD) || model2.getModelType() != BVH_MODEL_TRIANGLES)
    return false;

  node.model1 = &model1;
  node.model2 = &model2;

  node.vertices1 = model1.vertices;
  node.vertices2 = model2.vertices;

  node.tri_indices2 = model2.tri_indices;
  node.uc1.reset(new Uncertainty[model1.num_vertices]);

  estimateSamplingUncertainty(model1.vertices, model1.num_vertices, node.uc1.get());

  BVHExpand(model1, node.uc1.get(), expand_r);

  node.num_max_contacts = num_max_contacts;
  node.exhaustive = exhaustive;
  node.enable_contact = enable_contact;
  node.collision_prob_threshold = collision_prob_threshold;
  node.leaf_size_threshold = leaf_size_threshold;

  relativeTransform(model1.getRotation(), model1.getTranslation(), model2.getRotation(), model2.getTranslation(), node.R, node.T);

  return true;
}

#endif

bool initialize(MeshDistanceTraversalNodeRSS& node, const BVHModel<RSS>& model1, const BVHModel<RSS>& model2)
{
  if(model1.getModelType() != BVH_MODEL_TRIANGLES || model2.getModelType() != BVH_MODEL_TRIANGLES)
    return false;

  node.model1 = &model1;
  node.model2 = &model2;

  node.vertices1 = model1.vertices;
  node.vertices2 = model2.vertices;

  node.tri_indices1 = model1.tri_indices;
  node.tri_indices2 = model2.tri_indices;

  relativeTransform(model1.getRotation(), model1.getTranslation(), model2.getRotation(), model2.getTranslation(), node.R, node.T);

  return true;
}


}

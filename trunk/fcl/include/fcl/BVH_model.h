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

#ifndef FCL_BVH_MODEL_H
#define FCL_BVH_MODEL_H

#include "fcl/collision_object.h"
#include "fcl/BVH_internal.h"
#include "fcl/BV.h"
#include "fcl/BV_node.h"
#include "fcl/primitive.h"
#include "fcl/vec_3f.h"
#include "fcl/BV_splitter.h"
#include "fcl/BV_fitter.h"
#include <vector>
#include <boost/shared_ptr.hpp>

/** \brief Main namespace */
namespace fcl
{

/** \brief A class describing the bounding hierarchy of a mesh model */
template<typename BV>
class BVHModel : public CollisionObject
{
private:
  int num_tris_allocated;
  int num_vertices_allocated;
  int num_bvs_allocated;
  int num_vertex_updated; // for ccd vertex update
  unsigned int* primitive_indices;

public:

  /** \brief The state of BVH building process */
  BVHBuildState build_state;

  /** \brief Split rule to split one BV node into two children */
  boost::shared_ptr<BVSplitterBase<BV> > bv_splitter;

  /** \brief Fitting rule to fit a BV node to a set of geometry primitives */
  boost::shared_ptr<BVFitterBase<BV> > bv_fitter;

  /** \brief Model type */
  BVHModelType getModelType() const
  {
    if(num_tris && num_vertices)
      return BVH_MODEL_TRIANGLES;
    else if(num_vertices)
      return BVH_MODEL_POINTCLOUD;
    else
      return BVH_MODEL_UNKNOWN;
  }

  /** \brief Constructing an empty BVH */
  BVHModel()
  {
    vertices = NULL;
    tri_indices = NULL;
    bvs = NULL;

    num_tris = 0;
    num_tris_allocated = 0;
    num_vertices = 0;
    num_vertices_allocated = 0;
    num_bvs = 0;
    num_bvs_allocated = 0;

    prev_vertices = NULL;

    primitive_indices = NULL;

    build_state = BVH_BUILD_STATE_EMPTY;

    bv_splitter.reset(new BVSplitter<BV>(SPLIT_METHOD_MEAN));
    bv_fitter.reset(new BVFitter<BV>());
  }

  BVHModel(const BVHModel& other);


  ~BVHModel()
  {
    delete [] vertices;
    delete [] tri_indices;
    delete [] bvs;

    delete [] prev_vertices;

    delete [] primitive_indices;
  }

  /** \brief We provide getBV() and getNumBVs() because BVH may be compressed (in future), so
      we must provide some flexibility here */
  
  /** \brief Get the BV of index id */
  const BVNode<BV>& getBV(int id) const
  {
    return bvs[id];
  }

  /** \brief Get the BV of index id */
  BVNode<BV>& getBV(int id)
  {
    return bvs[id];
  }

  /** \brief Get the number of BVs */
  int getNumBVs() const
  {
    return num_bvs;
  }

  /** \brief Get the object type: it is a BVH */
  OBJECT_TYPE getObjectType() const { return OT_BVH; }

  /** \brief Get the BV type */
  NODE_TYPE getNodeType() const { return BV_UNKNOWN; }

  /** \brief Compute the AABB for the BVH, used for broad-phase collision */
  void computeLocalAABB();

  /** \brief Geometry point data */
  Vec3f* vertices;

  /** \brief Geometry triangle index data, will be NULL for point clouds */
  Triangle* tri_indices;

  /** \brief Geometry point data in previous frame */
  Vec3f* prev_vertices;

  /** \brief Number of triangles */
  int num_tris;

  /** \brief Number of points */
  int num_vertices;

  /** \brief Begin a new BVH model */
  int beginModel(int num_tris = 0, int num_vertices = 0);

  /** \brief Add one point in the new BVH model */
  int addVertex(const Vec3f& p);

  /** \brief Add one triangle in the new BVH model */
  int addTriangle(const Vec3f& p1, const Vec3f& p2, const Vec3f& p3);

  /** \brief Add a set of triangles in the new BVH model */
  int addSubModel(const std::vector<Vec3f>& ps, const std::vector<Triangle>& ts);

  /** \brief Add a set of points in the new BVH model */
  int addSubModel(const std::vector<Vec3f>& ps);

  /** \brief End BVH model construction, will build the bounding volume hierarchy */
  int endModel();


  /** \brief Replace the geometry information of current frame (i.e. should have the same mesh topology with the previous frame) */
  int beginReplaceModel();

  /** \brief Replace one point in the old BVH model */
  int replaceVertex(const Vec3f& p);

  /** \brief Replace one triangle in the old BVH model */
  int replaceTriangle(const Vec3f& p1, const Vec3f& p2, const Vec3f& p3);

  /** \brief Replace a set of points in the old BVH model */
  int replaceSubModel(const std::vector<Vec3f>& ps);

  /** \brief End BVH model replacement, will also refit or rebuild the bounding volume hierarchy */
  int endReplaceModel(bool refit = true, bool bottomup = true);


  /** \brief Replace the geometry information of current frame (i.e. should have the same mesh topology with the previous frame).
   *  The current frame will be saved as the previous frame in prev_vertices.
   *  */
  int beginUpdateModel();

  /** \brief Update one point in the old BVH model */
  int updateVertex(const Vec3f& p);

  /** \brief Update one triangle in the old BVH model */
  int updateTriangle(const Vec3f& p1, const Vec3f& p2, const Vec3f& p3);

  /** \brief Update a set of points in the old BVH model */
  int updateSubModel(const std::vector<Vec3f>& ps);

  /** \brief End BVH model update, will also refit or rebuild the bounding volume hierarchy */
  int endUpdateModel(bool refit = true, bool bottomup = true);

  /** \brief Check the number of memory used */
  int memUsage(int msg) const;

private:

  /** \brief Bounding volume hierarchy */
  BVNode<BV>* bvs;

  /** \brief Number of BV nodes in bounding volume hierarchy */
  int num_bvs;

  /** \brief Build the bounding volume hierarchy */
  int buildTree();

  /** \brief Refit the bounding volume hierarchy */
  int refitTree(bool bottomup);

  /** \brief Refit the bounding volume hierarchy in a top-down way (slow but more compact)*/
  int refitTree_topdown();

  /** \brief Refit the bounding volume hierarchy in a bottom-up way (fast but less compact) */
  int refitTree_bottomup();

  /** \brief Recursive kernel for hierarchy construction */
  int recursiveBuildTree(int bv_id, int first_primitive, int num_primitives);

  /** \brief Recursive kernel for bottomup refitting */
  int recursiveRefitTree_bottomup(int bv_id);

};


/** Specialization of getNodeType() for BVHModel with different BV types */
template<>
NODE_TYPE BVHModel<AABB>::getNodeType() const;

template<>
NODE_TYPE BVHModel<OBB>::getNodeType() const;

template<>
NODE_TYPE BVHModel<RSS>::getNodeType() const;

template<>
NODE_TYPE BVHModel<KDOP<16> >::getNodeType() const;

template<>
NODE_TYPE BVHModel<KDOP<18> >::getNodeType() const;

template<>
NODE_TYPE BVHModel<KDOP<24> >::getNodeType() const;

}

#endif

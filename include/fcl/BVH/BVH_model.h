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

#ifndef FCL_BVH_MODEL_H
#define FCL_BVH_MODEL_H

#include <vector>
#include <memory>

#include "fcl/collision_geometry.h"
#include "fcl/BV/BV_node.h"
#include "fcl/BV/OBB.h"
#include "fcl/BVH/BVH_internal.h"
#include "fcl/BVH/BV_splitter.h"
#include "fcl/BVH/BV_fitter.h"

namespace fcl
{

/// @brief A class describing the bounding hierarchy of a mesh model or a point cloud model (which is viewed as a degraded version of mesh)
template <typename BV>
class BVHModel : public CollisionGeometry<typename BV::Scalar>
{
public:

  using Scalar = typename BV::Scalar;

  /// @brief Model type described by the instance
  BVHModelType getModelType() const;

  /// @brief Constructing an empty BVH
  BVHModel();

  /// @brief copy from another BVH
  BVHModel(const BVHModel& other);

  /// @brief deconstruction, delete mesh data related.
  ~BVHModel();

  /// @brief We provide getBV() and getNumBVs() because BVH may be compressed
  /// (in future), so we must provide some flexibility here
  
  /// @brief Access the bv giving the its index
  const BVNode<BV>& getBV(int id) const;

  /// @brief Access the bv giving the its index
  BVNode<BV>& getBV(int id);

  /// @brief Get the number of bv in the BVH
  int getNumBVs() const;

  /// @brief Get the object type: it is a BVH
  OBJECT_TYPE getObjectType() const;

  /// @brief Get the BV type: default is unknown
  NODE_TYPE getNodeType() const;

  /// @brief Compute the AABB<Scalar> for the BVH, used for broad-phase collision
  void computeLocalAABB() override;

  /// @brief Begin a new BVH model
  int beginModel(int num_tris = 0, int num_vertices = 0);

  /// @brief Add one point in the new BVH model
  int addVertex(const Vector3<Scalar>& p);

  /// @brief Add one triangle in the new BVH model
  int addTriangle(const Vector3<Scalar>& p1, const Vector3<Scalar>& p2, const Vector3<Scalar>& p3);

  /// @brief Add a set of triangles in the new BVH model
  int addSubModel(const std::vector<Vector3<Scalar>>& ps, const std::vector<Triangle>& ts);

  /// @brief Add a set of points in the new BVH model
  int addSubModel(const std::vector<Vector3<Scalar>>& ps);

  /// @brief End BVH model construction, will build the bounding volume hierarchy
  int endModel();


  /// @brief Replace the geometry information of current frame (i.e. should have the same mesh topology with the previous frame)
  int beginReplaceModel();

  /// @brief Replace one point in the old BVH model
  int replaceVertex(const Vector3<Scalar>& p);

  /// @brief Replace one triangle in the old BVH model
  int replaceTriangle(const Vector3<Scalar>& p1, const Vector3<Scalar>& p2, const Vector3<Scalar>& p3);

  /// @brief Replace a set of points in the old BVH model
  int replaceSubModel(const std::vector<Vector3<Scalar>>& ps);

  /// @brief End BVH model replacement, will also refit or rebuild the bounding volume hierarchy
  int endReplaceModel(bool refit = true, bool bottomup = true);


  /// @brief Replace the geometry information of current frame (i.e. should have the same mesh topology with the previous frame).
  /// The current frame will be saved as the previous frame in prev_vertices.
  int beginUpdateModel();

  /// @brief Update one point in the old BVH model
  int updateVertex(const Vector3<Scalar>& p);

  /// @brief Update one triangle in the old BVH model
  int updateTriangle(const Vector3<Scalar>& p1, const Vector3<Scalar>& p2, const Vector3<Scalar>& p3);

  /// @brief Update a set of points in the old BVH model
  int updateSubModel(const std::vector<Vector3<Scalar>>& ps);

  /// @brief End BVH model update, will also refit or rebuild the bounding volume hierarchy
  int endUpdateModel(bool refit = true, bool bottomup = true);

  /// @brief Check the number of memory used
  int memUsage(int msg) const;

  /// @brief This is a special acceleration: BVH_model default stores the BV's transform in world coordinate. However, we can also store each BV's transform related to its parent 
  /// BV node. When traversing the BVH, this can save one matrix transformation.
  void makeParentRelative();

  Vector3<Scalar> computeCOM() const;

  Scalar computeVolume() const;

  Matrix3<Scalar> computeMomentofInertia() const;

public:
  /// @brief Geometry point data
  Vector3<Scalar>* vertices;

  /// @brief Geometry triangle index data, will be NULL for point clouds
  Triangle* tri_indices;

  /// @brief Geometry point data in previous frame
  Vector3<Scalar>* prev_vertices;

  /// @brief Number of triangles
  int num_tris;

  /// @brief Number of points
  int num_vertices;

  /// @brief The state of BVH building process
  BVHBuildState build_state;

  /// @brief Split rule to split one BV node into two children
  std::shared_ptr<BVSplitterBase<BV>> bv_splitter;

  /// @brief Fitting rule to fit a BV node to a set of geometry primitives
  std::shared_ptr<BVFitterBase<BV>> bv_fitter;


private:

  int num_tris_allocated;
  int num_vertices_allocated;
  int num_bvs_allocated;
  int num_vertex_updated; /// for ccd vertex update
  unsigned int* primitive_indices;


  /// @brief Bounding volume hierarchy
  BVNode<BV>* bvs;

  /// @brief Number of BV nodes in bounding volume hierarchy
  int num_bvs;

  /// @brief Build the bounding volume hierarchy
  int buildTree();

  /// @brief Refit the bounding volume hierarchy
  int refitTree(bool bottomup);

  /// @brief Refit the bounding volume hierarchy in a top-down way (slow but more compact)
  int refitTree_topdown();

  /// @brief Refit the bounding volume hierarchy in a bottom-up way (fast but less compact)
  int refitTree_bottomup();

  /// @brief Recursive kernel for hierarchy construction
  int recursiveBuildTree(int bv_id, int first_primitive, int num_primitives);

  /// @brief Recursive kernel for bottomup refitting 
  int recursiveRefitTree_bottomup(int bv_id);

  /// @recursively compute each bv's transform related to its parent. For
  /// default BV, only the translation works. For oriented BV (OBB, RSS,
  /// OBBRSS), special implementation is provided.
  void makeParentRelativeRecurse(
      int bv_id,
      const Matrix3<Scalar>& parent_axis,
      const Vector3<Scalar>& parent_c);

  template <typename, typename>
  friend struct MakeParentRelativeRecurseImpl;
};

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename BV>
BVHModelType BVHModel<BV>::getModelType() const
{
  if(num_tris && num_vertices)
    return BVH_MODEL_TRIANGLES;
  else if(num_vertices)
    return BVH_MODEL_POINTCLOUD;
  else
    return BVH_MODEL_UNKNOWN;
}

//==============================================================================
template <typename BV>
BVHModel<BV>::BVHModel() : vertices(NULL),
  tri_indices(NULL),
  prev_vertices(NULL),
  num_tris(0),
  num_vertices(0),
  build_state(BVH_BUILD_STATE_EMPTY),
  bv_splitter(new BVSplitter<BV>(SPLIT_METHOD_MEAN)),
  bv_fitter(new BVFitter<BV>()),
  num_tris_allocated(0),
  num_vertices_allocated(0),
  num_bvs_allocated(0),
  num_vertex_updated(0),
  primitive_indices(NULL),
  bvs(NULL),
  num_bvs(0)
{
  // Do nothing
}

//==============================================================================
template <typename BV>
BVHModel<BV>::BVHModel(const BVHModel<BV>& other)
  : CollisionGeometry<Scalar>(other),
    num_tris(other.num_tris),
    num_vertices(other.num_vertices),
    build_state(other.build_state),
    bv_splitter(other.bv_splitter),
    bv_fitter(other.bv_fitter),
    num_tris_allocated(other.num_tris),
    num_vertices_allocated(other.num_vertices)
{
  if(other.vertices)
  {
    vertices = new Vector3<Scalar>[num_vertices];
    memcpy(vertices, other.vertices, sizeof(Vector3<Scalar>) * num_vertices);
  }
  else
    vertices = NULL;

  if(other.tri_indices)
  {
    tri_indices = new Triangle[num_tris];
    memcpy(tri_indices, other.tri_indices, sizeof(Triangle) * num_tris);
  }
  else
    tri_indices = NULL;

  if(other.prev_vertices)
  {
    prev_vertices = new Vector3<Scalar>[num_vertices];
    memcpy(prev_vertices, other.prev_vertices, sizeof(Vector3<Scalar>) * num_vertices);
  }
  else
    prev_vertices = NULL;

  if(other.primitive_indices)
  {
    int num_primitives = 0;
    switch(other.getModelType())
    {
      case BVH_MODEL_TRIANGLES:
        num_primitives = num_tris;
        break;
      case BVH_MODEL_POINTCLOUD:
        num_primitives = num_vertices;
        break;
      default:
        ;
    }

    primitive_indices = new unsigned int[num_primitives];
    memcpy(primitive_indices, other.primitive_indices, sizeof(unsigned int) * num_primitives);
  }
  else
    primitive_indices = NULL;

  num_bvs = num_bvs_allocated = other.num_bvs;
  if(other.bvs)
  {
    bvs = new BVNode<BV>[num_bvs];
    memcpy(bvs, other.bvs, sizeof(BVNode<BV>) * num_bvs);
  }
  else
    bvs = NULL;
}

//==============================================================================
template <typename BV>
BVHModel<BV>::~BVHModel()
{
  delete [] vertices;
  delete [] tri_indices;
  delete [] bvs;

  delete [] prev_vertices;
  delete [] primitive_indices;
}

//==============================================================================
template <typename BV>
const BVNode<BV>& BVHModel<BV>::getBV(int id) const
{
  return bvs[id];
}

//==============================================================================
template <typename BV>
BVNode<BV>& BVHModel<BV>::getBV(int id)
{
  return bvs[id];
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::getNumBVs() const
{
  return num_bvs;
}

//==============================================================================
template <typename BV>
OBJECT_TYPE BVHModel<BV>::getObjectType() const
{
  return OT_BVH;
}

//==============================================================================
template <typename BV>
struct GetNodeTypeImpl
{
  NODE_TYPE operator()()
  {
    return BV_UNKNOWN;
  }
};

//==============================================================================
template <typename BV>
NODE_TYPE BVHModel<BV>::getNodeType() const
{
  GetNodeTypeImpl<BV> getNodeTypeImpl;
  return getNodeTypeImpl();
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::beginModel(int num_tris_, int num_vertices_)
{
  if(build_state != BVH_BUILD_STATE_EMPTY)
  {
    delete [] vertices; vertices = NULL;
    delete [] tri_indices; tri_indices = NULL;
    delete [] bvs; bvs = NULL;
    delete [] prev_vertices; prev_vertices = NULL;
    delete [] primitive_indices; primitive_indices = NULL;

    num_vertices_allocated = num_vertices = num_tris_allocated = num_tris = num_bvs_allocated = num_bvs = 0;
  }

  if(num_tris_ <= 0) num_tris_ = 8;
  if(num_vertices_ <= 0) num_vertices_ = 8;

  num_vertices_allocated = num_vertices_;
  num_tris_allocated = num_tris_;

  tri_indices = new Triangle[num_tris_allocated];
  vertices = new Vector3<Scalar>[num_vertices_allocated];

  if(!tri_indices)
  {
    std::cerr << "BVH Error! Out of memory for tri_indices array on BeginModel() call!" << std::endl;
    return BVH_ERR_MODEL_OUT_OF_MEMORY;
  }
  if(!vertices)
  {
    std::cerr << "BVH Error! Out of memory for vertices array on BeginModel() call!" << std::endl;
    return BVH_ERR_MODEL_OUT_OF_MEMORY;
  }

  if(build_state != BVH_BUILD_STATE_EMPTY)
  {
    std::cerr << "BVH Warning! Call beginModel() on a BVHModel that is not empty. This model was cleared and previous triangles/vertices were lost." << std::endl;
    build_state = BVH_BUILD_STATE_EMPTY;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  build_state = BVH_BUILD_STATE_BEGUN;

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::addVertex(const Vector3<Scalar>& p)
{
  if(build_state != BVH_BUILD_STATE_BEGUN)
  {
    std::cerr << "BVH Warning! Call addVertex() in a wrong order. addVertex() was ignored. Must do a beginModel() to clear the model for addition of new vertices." << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  if(num_vertices >= num_vertices_allocated)
  {
    Vector3<Scalar>* temp = new Vector3<Scalar>[num_vertices_allocated * 2];
    if(!temp)
    {
      std::cerr << "BVH Error! Out of memory for vertices array on addVertex() call!" << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }

    memcpy(temp, vertices, sizeof(Vector3<Scalar>) * num_vertices);
    delete [] vertices;
    vertices = temp;
    num_vertices_allocated *= 2;
  }

  vertices[num_vertices] = p;
  num_vertices += 1;

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::addTriangle(const Vector3<Scalar>& p1, const Vector3<Scalar>& p2, const Vector3<Scalar>& p3)
{
  if(build_state == BVH_BUILD_STATE_PROCESSED)
  {
    std::cerr << "BVH Warning! Call addTriangle() in a wrong order. addTriangle() was ignored. Must do a beginModel() to clear the model for addition of new triangles." << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  if(num_vertices + 2 >= num_vertices_allocated)
  {
    Vector3<Scalar>* temp = new Vector3<Scalar>[num_vertices_allocated * 2 + 2];
    if(!temp)
    {
      std::cerr << "BVH Error! Out of memory for vertices array on addTriangle() call!" << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }

    memcpy(temp, vertices, sizeof(Vector3<Scalar>) * num_vertices);
    delete [] vertices;
    vertices = temp;
    num_vertices_allocated = num_vertices_allocated * 2 + 2;
  }

  int offset = num_vertices;

  vertices[num_vertices] = p1;
  num_vertices++;
  vertices[num_vertices] = p2;
  num_vertices++;
  vertices[num_vertices] = p3;
  num_vertices++;

  if(num_tris >= num_tris_allocated)
  {
    Triangle* temp = new Triangle[num_tris_allocated * 2];
    if(!temp)
    {
      std::cerr << "BVH Error! Out of memory for tri_indices array on addTriangle() call!" << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }

    memcpy(temp, tri_indices, sizeof(Triangle) * num_tris);
    delete [] tri_indices;
    tri_indices = temp;
    num_tris_allocated *= 2;
  }

  tri_indices[num_tris].set(offset, offset + 1, offset + 2);
  num_tris++;

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::addSubModel(const std::vector<Vector3<Scalar>>& ps)
{
  if(build_state == BVH_BUILD_STATE_PROCESSED)
  {
    std::cerr << "BVH Warning! Call addSubModel() in a wrong order. addSubModel() was ignored. Must do a beginModel() to clear the model for addition of new vertices." << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  int num_vertices_to_add = ps.size();

  if(num_vertices + num_vertices_to_add - 1 >= num_vertices_allocated)
  {
    Vector3<Scalar>* temp = new Vector3<Scalar>[num_vertices_allocated * 2 + num_vertices_to_add - 1];
    if(!temp)
    {
      std::cerr << "BVH Error! Out of memory for vertices array on addSubModel() call!" << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }

    memcpy(temp, vertices, sizeof(Vector3<Scalar>) * num_vertices);
    delete [] vertices;
    vertices = temp;
    num_vertices_allocated = num_vertices_allocated * 2 + num_vertices_to_add - 1;
  }

  for(int i = 0; i < num_vertices_to_add; ++i)
  {
    vertices[num_vertices] = ps[i];
    num_vertices++;
  }

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::addSubModel(const std::vector<Vector3<Scalar>>& ps, const std::vector<Triangle>& ts)
{
  if(build_state == BVH_BUILD_STATE_PROCESSED)
  {
    std::cerr << "BVH Warning! Call addSubModel() in a wrong order. addSubModel() was ignored. Must do a beginModel() to clear the model for addition of new vertices." << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  int num_vertices_to_add = ps.size();

  if(num_vertices + num_vertices_to_add - 1 >= num_vertices_allocated)
  {
    Vector3<Scalar>* temp = new Vector3<Scalar>[num_vertices_allocated * 2 + num_vertices_to_add - 1];
    if(!temp)
    {
      std::cerr << "BVH Error! Out of memory for vertices array on addSubModel() call!" << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }

    memcpy(temp, vertices, sizeof(Vector3<Scalar>) * num_vertices);
    delete [] vertices;
    vertices = temp;
    num_vertices_allocated = num_vertices_allocated * 2 + num_vertices_to_add - 1;
  }

  int offset = num_vertices;

  for(int i = 0; i < num_vertices_to_add; ++i)
  {
    vertices[num_vertices] = ps[i];
    num_vertices++;
  }


  int num_tris_to_add = ts.size();

  if(num_tris + num_tris_to_add - 1 >= num_tris_allocated)
  {
    Triangle* temp = new Triangle[num_tris_allocated * 2 + num_tris_to_add - 1];
    if(!temp)
    {
      std::cerr << "BVH Error! Out of memory for tri_indices array on addSubModel() call!" << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }

    memcpy(temp, tri_indices, sizeof(Triangle) * num_tris);
    delete [] tri_indices;
    tri_indices = temp;
    num_tris_allocated = num_tris_allocated * 2 + num_tris_to_add - 1;
  }

  for(int i = 0; i < num_tris_to_add; ++i)
  {
    const Triangle& t = ts[i];
    tri_indices[num_tris].set(t[0] + offset, t[1] + offset, t[2] + offset);
    num_tris++;
  }

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::endModel()
{
  if(build_state != BVH_BUILD_STATE_BEGUN)
  {
    std::cerr << "BVH Warning! Call endModel() in wrong order. endModel() was ignored." << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  if(num_tris == 0 && num_vertices == 0)
  {
    std::cerr << "BVH Error! endModel() called on model with no triangles and vertices." << std::endl;
    return BVH_ERR_BUILD_EMPTY_MODEL;
  }

  if(num_tris_allocated > num_tris)
  {
    Triangle* new_tris = new Triangle[num_tris];
    if(!new_tris)
    {
      std::cerr << "BVH Error! Out of memory for tri_indices array in endModel() call!" << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }
    memcpy(new_tris, tri_indices, sizeof(Triangle) * num_tris);
    delete [] tri_indices;
    tri_indices = new_tris;
    num_tris_allocated = num_tris;
  }

  if(num_vertices_allocated > num_vertices)
  {
    Vector3<Scalar>* new_vertices = new Vector3<Scalar>[num_vertices];
    if(!new_vertices)
    {
      std::cerr << "BVH Error! Out of memory for vertices array in endModel() call!" << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }
    memcpy(new_vertices, vertices, sizeof(Vector3<Scalar>) * num_vertices);
    delete [] vertices;
    vertices = new_vertices;
    num_vertices_allocated = num_vertices;
  }


  // construct BVH tree
  int num_bvs_to_be_allocated = 0;
  if(num_tris == 0)
    num_bvs_to_be_allocated = 2 * num_vertices - 1;
  else
    num_bvs_to_be_allocated = 2 * num_tris - 1;


  bvs = new BVNode<BV> [num_bvs_to_be_allocated];
  primitive_indices = new unsigned int [num_bvs_to_be_allocated];
  if(!bvs || !primitive_indices)
  {
    std::cerr << "BVH Error! Out of memory for BV array in endModel()!" << std::endl;
    return BVH_ERR_MODEL_OUT_OF_MEMORY;
  }
  num_bvs_allocated = num_bvs_to_be_allocated;
  num_bvs = 0;

  buildTree();

  // finish constructing
  build_state = BVH_BUILD_STATE_PROCESSED;

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::beginReplaceModel()
{
  if(build_state != BVH_BUILD_STATE_PROCESSED)
  {
    std::cerr << "BVH Error! Call beginReplaceModel() on a BVHModel that has no previous frame." << std::endl;
    return BVH_ERR_BUILD_EMPTY_PREVIOUS_FRAME;
  }

  if(prev_vertices) delete [] prev_vertices; prev_vertices = NULL;

  num_vertex_updated = 0;

  build_state = BVH_BUILD_STATE_REPLACE_BEGUN;

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::replaceVertex(const Vector3<Scalar>& p)
{
  if(build_state != BVH_BUILD_STATE_REPLACE_BEGUN)
  {
    std::cerr << "BVH Warning! Call replaceVertex() in a wrong order. replaceVertex() was ignored. Must do a beginReplaceModel() for initialization." << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  vertices[num_vertex_updated] = p;
  num_vertex_updated++;

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::replaceTriangle(const Vector3<Scalar>& p1, const Vector3<Scalar>& p2, const Vector3<Scalar>& p3)
{
  if(build_state != BVH_BUILD_STATE_REPLACE_BEGUN)
  {
    std::cerr << "BVH Warning! Call replaceTriangle() in a wrong order. replaceTriangle() was ignored. Must do a beginReplaceModel() for initialization." << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  vertices[num_vertex_updated] = p1; num_vertex_updated++;
  vertices[num_vertex_updated] = p2; num_vertex_updated++;
  vertices[num_vertex_updated] = p3; num_vertex_updated++;
  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::replaceSubModel(const std::vector<Vector3<Scalar>>& ps)
{
  if(build_state != BVH_BUILD_STATE_REPLACE_BEGUN)
  {
    std::cerr << "BVH Warning! Call replaceSubModel() in a wrong order. replaceSubModel() was ignored. Must do a beginReplaceModel() for initialization." << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  for(unsigned int i = 0; i < ps.size(); ++i)
  {
    vertices[num_vertex_updated] = ps[i];
    num_vertex_updated++;
  }
  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::endReplaceModel(bool refit, bool bottomup)
{
  if(build_state != BVH_BUILD_STATE_REPLACE_BEGUN)
  {
    std::cerr << "BVH Warning! Call endReplaceModel() in a wrong order. endReplaceModel() was ignored. " << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  if(num_vertex_updated != num_vertices)
  {
    std::cerr << "BVH Error! The replaced model should have the same number of vertices as the old model." << std::endl;
    return BVH_ERR_INCORRECT_DATA;
  }

  if(refit)  // refit, do not change BVH structure
  {
    refitTree(bottomup);
  }
  else // reconstruct bvh tree based on current frame data
  {
    buildTree();
  }

  build_state = BVH_BUILD_STATE_PROCESSED;

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::beginUpdateModel()
{
  if(build_state != BVH_BUILD_STATE_PROCESSED && build_state != BVH_BUILD_STATE_UPDATED)
  {
    std::cerr << "BVH Error! Call beginUpdatemodel() on a BVHModel that has no previous frame." << std::endl;
    return BVH_ERR_BUILD_EMPTY_PREVIOUS_FRAME;
  }

  if(prev_vertices)
  {
    Vector3<Scalar>* temp = prev_vertices;
    prev_vertices = vertices;
    vertices = temp;
  }
  else
  {
    prev_vertices = vertices;
    vertices = new Vector3<Scalar>[num_vertices];
  }

  num_vertex_updated = 0;

  build_state = BVH_BUILD_STATE_UPDATE_BEGUN;

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::updateVertex(const Vector3<Scalar>& p)
{
  if(build_state != BVH_BUILD_STATE_UPDATE_BEGUN)
  {
    std::cerr << "BVH Warning! Call updateVertex() in a wrong order. updateVertex() was ignored. Must do a beginUpdateModel() for initialization." << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  vertices[num_vertex_updated] = p;
  num_vertex_updated++;

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::updateTriangle(const Vector3<Scalar>& p1, const Vector3<Scalar>& p2, const Vector3<Scalar>& p3)
{
  if(build_state != BVH_BUILD_STATE_UPDATE_BEGUN)
  {
    std::cerr << "BVH Warning! Call updateTriangle() in a wrong order. updateTriangle() was ignored. Must do a beginUpdateModel() for initialization." << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  vertices[num_vertex_updated] = p1; num_vertex_updated++;
  vertices[num_vertex_updated] = p2; num_vertex_updated++;
  vertices[num_vertex_updated] = p3; num_vertex_updated++;
  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::updateSubModel(const std::vector<Vector3<Scalar>>& ps)
{
  if(build_state != BVH_BUILD_STATE_UPDATE_BEGUN)
  {
    std::cerr << "BVH Warning! Call updateSubModel() in a wrong order. updateSubModel() was ignored. Must do a beginUpdateModel() for initialization." << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  for(unsigned int i = 0; i < ps.size(); ++i)
  {
    vertices[num_vertex_updated] = ps[i];
    num_vertex_updated++;
  }
  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::endUpdateModel(bool refit, bool bottomup)
{
  if(build_state != BVH_BUILD_STATE_UPDATE_BEGUN)
  {
    std::cerr << "BVH Warning! Call endUpdateModel() in a wrong order. endUpdateModel() was ignored. " << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  if(num_vertex_updated != num_vertices)
  {
    std::cerr << "BVH Error! The updated model should have the same number of vertices as the old model." << std::endl;
    return BVH_ERR_INCORRECT_DATA;
  }

  if(refit)  // refit, do not change BVH structure
  {
    refitTree(bottomup);
  }
  else // reconstruct bvh tree based on current frame data
  {
    buildTree();

    // then refit

    refitTree(bottomup);
  }


  build_state = BVH_BUILD_STATE_UPDATED;

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::memUsage(int msg) const
{
  int mem_bv_list = sizeof(BV) * num_bvs;
  int mem_tri_list = sizeof(Triangle) * num_tris;
  int mem_vertex_list = sizeof(Vector3<Scalar>) * num_vertices;

  int total_mem = mem_bv_list + mem_tri_list + mem_vertex_list + sizeof(BVHModel<BV>);
  if(msg)
  {
    std::cerr << "Total for model " << total_mem << " bytes." << std::endl;
    std::cerr << "BVs: " << num_bvs << " allocated." << std::endl;
    std::cerr << "Tris: " << num_tris << " allocated." << std::endl;
    std::cerr << "Vertices: " << num_vertices << " allocated." << std::endl;
  }

  return BVH_OK;
}

//==============================================================================
template <typename BV>
void BVHModel<BV>::makeParentRelative()
{
  makeParentRelativeRecurse(
        0, Matrix3<Scalar>::Identity(), Vector3<Scalar>::Zero());
}

//==============================================================================
template <typename BV>
Vector3<typename BV::Scalar> BVHModel<BV>::computeCOM() const
{
  Scalar vol = 0;
  Vector3<Scalar> com = Vector3<Scalar>::Zero();
  for(int i = 0; i < num_tris; ++i)
  {
    const Triangle& tri = tri_indices[i];
    Scalar d_six_vol = (vertices[tri[0]].cross(vertices[tri[1]])).dot(vertices[tri[2]]);
    vol += d_six_vol;
    com += (vertices[tri[0]] + vertices[tri[1]] + vertices[tri[2]]) * d_six_vol;
  }

  return com / (vol * 4);
}

//==============================================================================
template <typename BV>
typename BV::Scalar BVHModel<BV>::computeVolume() const
{
  Scalar vol = 0;
  for(int i = 0; i < num_tris; ++i)
  {
    const Triangle& tri = tri_indices[i];
    Scalar d_six_vol = (vertices[tri[0]].cross(vertices[tri[1]])).dot(vertices[tri[2]]);
    vol += d_six_vol;
  }

  return vol / 6;
}

//==============================================================================
template <typename BV>
Matrix3<typename BV::Scalar> BVHModel<BV>::computeMomentofInertia() const
{
  Matrix3<Scalar> C = Matrix3<Scalar>::Zero();

  Matrix3<Scalar> C_canonical;
  C_canonical << 1/ 60.0, 1/120.0, 1/120.0,
      1/120.0, 1/ 60.0, 1/120.0,
      1/120.0, 1/120.0, 1/ 60.0;

  for(int i = 0; i < num_tris; ++i)
  {
    const Triangle& tri = tri_indices[i];
    const Vector3<Scalar>& v1 = vertices[tri[0]];
    const Vector3<Scalar>& v2 = vertices[tri[1]];
    const Vector3<Scalar>& v3 = vertices[tri[2]];
    Scalar d_six_vol = (v1.cross(v2)).dot(v3);
    Matrix3<Scalar> A;
    A.row(0) = v1;
    A.row(1) = v2;
    A.row(2) = v3;
    C += A.transpose() * C_canonical * A * d_six_vol;
  }

  Scalar trace_C = C(0, 0) + C(1, 1) + C(2, 2);

  Matrix3<Scalar> m;
  m << trace_C - C(0, 0), -C(0, 1), -C(0, 2),
      -C(1, 0), trace_C - C(1, 1), -C(1, 2),
      -C(2, 0), -C(2, 1), trace_C - C(2, 2);

  return m;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::buildTree()
{
  // set BVFitter
  bv_fitter->set(vertices, tri_indices, getModelType());
  // set SplitRule
  bv_splitter->set(vertices, tri_indices, getModelType());

  num_bvs = 1;

  int num_primitives = 0;
  switch(getModelType())
  {
  case BVH_MODEL_TRIANGLES:
    num_primitives = num_tris;
    break;
  case BVH_MODEL_POINTCLOUD:
    num_primitives = num_vertices;
    break;
  default:
    std::cerr << "BVH Error: Model type not supported!" << std::endl;
    return BVH_ERR_UNSUPPORTED_FUNCTION;
  }

  for(int i = 0; i < num_primitives; ++i)
    primitive_indices[i] = i;
  recursiveBuildTree(0, 0, num_primitives);

  bv_fitter->clear();
  bv_splitter->clear();

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::recursiveBuildTree(int bv_id, int first_primitive, int num_primitives)
{
  BVHModelType type = getModelType();
  BVNode<BV>* bvnode = bvs + bv_id;
  unsigned int* cur_primitive_indices = primitive_indices + first_primitive;

  // constructing BV
  BV bv = bv_fitter->fit(cur_primitive_indices, num_primitives);
  bv_splitter->computeRule(bv, cur_primitive_indices, num_primitives);

  bvnode->bv = bv;
  bvnode->first_primitive = first_primitive;
  bvnode->num_primitives = num_primitives;

  if(num_primitives == 1)
  {
    bvnode->first_child = -((*cur_primitive_indices) + 1);
  }
  else
  {
    bvnode->first_child = num_bvs;
    num_bvs += 2;

    int c1 = 0;
    for(int i = 0; i < num_primitives; ++i)
    {
      Vector3<Scalar> p;
      if(type == BVH_MODEL_POINTCLOUD) p = vertices[cur_primitive_indices[i]];
      else if(type == BVH_MODEL_TRIANGLES)
      {
        const Triangle& t = tri_indices[cur_primitive_indices[i]];
        const Vector3<Scalar>& p1 = vertices[t[0]];
        const Vector3<Scalar>& p2 = vertices[t[1]];
        const Vector3<Scalar>& p3 = vertices[t[2]];
        p = (p1 + p2 + p3) / 3.0;
      }
      else
      {
        std::cerr << "BVH Error: Model type not supported!" << std::endl;
        return BVH_ERR_UNSUPPORTED_FUNCTION;
      }


      // loop invariant: up to (but not including) index c1 in group 1,
      // then up to (but not including) index i in group 2
      //
      //  [1] [1] [1] [1] [2] [2] [2] [x] [x] ... [x]
      //                   c1          i
      //
      if(bv_splitter->apply(p)) // in the right side
      {
        // do nothing
      }
      else
      {
        std::swap(cur_primitive_indices[i], cur_primitive_indices[c1]);
        c1++;
      }
    }


    if((c1 == 0) || (c1 == num_primitives)) c1 = num_primitives / 2;

    int num_first_half = c1;

    recursiveBuildTree(bvnode->leftChild(), first_primitive, num_first_half);
    recursiveBuildTree(bvnode->rightChild(), first_primitive + num_first_half, num_primitives - num_first_half);
  }

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::refitTree(bool bottomup)
{
  if(bottomup)
    return refitTree_bottomup();
  else
    return refitTree_topdown();
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::refitTree_bottomup()
{
  int res = recursiveRefitTree_bottomup(0);

  return res;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::recursiveRefitTree_bottomup(int bv_id)
{
  BVNode<BV>* bvnode = bvs + bv_id;
  if(bvnode->isLeaf())
  {
    BVHModelType type = getModelType();
    int primitive_id = -(bvnode->first_child + 1);
    if(type == BVH_MODEL_POINTCLOUD)
    {
      BV bv;

      if(prev_vertices)
      {
        Vector3<Scalar> v[2];
        v[0] = prev_vertices[primitive_id];
        v[1] = vertices[primitive_id];
        fit(v, 2, bv);
      }
      else
        fit(vertices + primitive_id, 1, bv);

      bvnode->bv = bv;
    }
    else if(type == BVH_MODEL_TRIANGLES)
    {
      BV bv;
      const Triangle& triangle = tri_indices[primitive_id];

      if(prev_vertices)
      {
        Vector3<Scalar> v[6];
        for(int i = 0; i < 3; ++i)
        {
          v[i] = prev_vertices[triangle[i]];
          v[i + 3] = vertices[triangle[i]];
        }

        fit(v, 6, bv);
      }
      else
      {
        Vector3<Scalar> v[3];
        for(int i = 0; i < 3; ++i)
        {
          v[i] = vertices[triangle[i]];
        }

        fit(v, 3, bv);
      }

      bvnode->bv = bv;
    }
    else
    {
      std::cerr << "BVH Error: Model type not supported!" << std::endl;
      return BVH_ERR_UNSUPPORTED_FUNCTION;
    }
  }
  else
  {
    recursiveRefitTree_bottomup(bvnode->leftChild());
    recursiveRefitTree_bottomup(bvnode->rightChild());
    bvnode->bv = bvs[bvnode->leftChild()].bv + bvs[bvnode->rightChild()].bv;
  }

  return BVH_OK;
}

//==============================================================================
template <typename Scalar, typename BV>
struct MakeParentRelativeRecurseImpl
{
  void operator()(BVHModel<BV>& model,
                  int bv_id,
                  const Matrix3<Scalar>& parent_axis,
                  const Vector3<Scalar>& parent_c)
  {
    if(!model.bvs[bv_id].isLeaf())
    {
      MakeParentRelativeRecurseImpl<Scalar, BV> tmp1;
      tmp1(model, model.bvs[bv_id].first_child, parent_axis, model.bvs[bv_id].getCenter());

      MakeParentRelativeRecurseImpl<Scalar, BV> tmp2;
      tmp2(model, model.bvs[bv_id].first_child + 1, parent_axis, model.bvs[bv_id].getCenter());
    }

    model.bvs[bv_id].bv = translate(model.bvs[bv_id].bv, -parent_c);
  }
};

//==============================================================================
template <typename BV>
void BVHModel<BV>::makeParentRelativeRecurse(
    int bv_id,
    const Matrix3<Scalar>& parent_axis,
    const Vector3<Scalar>& parent_c)
{
  MakeParentRelativeRecurseImpl<typename BV::Scalar, BV> tmp;
  tmp(*this, bv_id, parent_axis, parent_c);
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::refitTree_topdown()
{
  bv_fitter->set(vertices, prev_vertices, tri_indices, getModelType());
  for(int i = 0; i < num_bvs; ++i)
  {
    BV bv = bv_fitter->fit(primitive_indices + bvs[i].first_primitive, bvs[i].num_primitives);
    bvs[i].bv = bv;
  }

  bv_fitter->clear();

  return BVH_OK;
}

//==============================================================================
template <typename BV>
void BVHModel<BV>::computeLocalAABB()
{
  AABB<Scalar> aabb_;
  for(int i = 0; i < num_vertices; ++i)
  {
    aabb_ += vertices[i];
  }

  this->aabb_center = aabb_.center();

  this->aabb_radius = 0;
  for(int i = 0; i < num_vertices; ++i)
  {
    Scalar r = (this->aabb_center - vertices[i]).squaredNorm();
    if(r > this->aabb_radius) this->aabb_radius = r;
  }

  this->aabb_radius = sqrt(this->aabb_radius);

  this->aabb_local = aabb_;
}

//==============================================================================
template <typename Scalar>
struct MakeParentRelativeRecurseImpl<Scalar, OBB<Scalar>>
{
  void operator()(BVHModel<OBB<Scalar>>& model,
                  int bv_id,
                  const Matrix3<Scalar>& parent_axis,
                  const Vector3<Scalar>& parent_c)
  {
    OBB<Scalar>& obb = model.bvs[bv_id].bv;
    if(!model.bvs[bv_id].isLeaf())
    {
      MakeParentRelativeRecurseImpl<Scalar, OBB<Scalar>> tmp1;
      tmp1(model, model.bvs[bv_id].first_child, obb.axis, obb.To);

      MakeParentRelativeRecurseImpl<Scalar, OBB<Scalar>> tmp2;
      tmp2(model, model.bvs[bv_id].first_child + 1, obb.axis, obb.To);
    }

    // make self parent relative
    obb.axis = parent_axis.transpose() * obb.axis;
    obb.To = (obb.To - parent_c).transpose() * parent_axis;
  }
};

//==============================================================================
template <typename Scalar>
struct MakeParentRelativeRecurseImpl<Scalar, RSS<Scalar>>
{
  void operator()(BVHModel<RSS<Scalar>>& model,
                  int bv_id,
                  const Matrix3<Scalar>& parent_axis,
                  const Vector3<Scalar>& parent_c)
  {
    RSS<Scalar>& rss = model.bvs[bv_id].bv;
    if(!model.bvs[bv_id].isLeaf())
    {
      MakeParentRelativeRecurseImpl<Scalar, RSS<Scalar>> tmp1;
      tmp1(model, model.bvs[bv_id].first_child, rss.axis, rss.Tr);

      MakeParentRelativeRecurseImpl<Scalar, RSS<Scalar>> tmp2;
      tmp2(model, model.bvs[bv_id].first_child + 1, rss.axis, rss.Tr);
    }

    // make self parent relative
    rss.axis = parent_axis.transpose() * rss.axis;
    rss.Tr = (rss.Tr - parent_c).transpose() * parent_axis;
  }
};

//==============================================================================
template <typename Scalar>
struct MakeParentRelativeRecurseImpl<Scalar, OBBRSS<Scalar>>
{
  void operator()(BVHModel<OBBRSS<Scalar>>& model,
                  int bv_id,
                  const Matrix3<Scalar>& parent_axis,
                  const Vector3<Scalar>& parent_c)
  {
    OBB<Scalar>& obb = model.bvs[bv_id].bv.obb;
    RSS<Scalar>& rss = model.bvs[bv_id].bv.rss;
    if(!model.bvs[bv_id].isLeaf())
    {
      MakeParentRelativeRecurseImpl<Scalar, RSS<Scalar>> tmp1;
      tmp1(model, model.bvs[bv_id].first_child, obb.axis, obb.To);

      MakeParentRelativeRecurseImpl<Scalar, RSS<Scalar>> tmp2;
      tmp2(model, model.bvs[bv_id].first_child + 1, obb.axis, obb.To);
    }

    // make self parent relative
    obb.axis = parent_axis.transpose() * obb.axis;
    rss.axis = obb.axis;

    obb.To = (obb.To - parent_c).transpose() * parent_axis;
    rss.Tr = obb.To;
  }
};

//==============================================================================
template <typename Scalar>
struct GetNodeTypeImpl<AABB<Scalar>>
{
  NODE_TYPE operator()()
  {
    return BV_AABB;
  }
};

//==============================================================================
template <typename Scalar>
struct GetNodeTypeImpl<OBB<Scalar>>
{
  NODE_TYPE operator()()
  {
    return BV_OBB;
  }
};

//==============================================================================
template <typename Scalar>
struct GetNodeTypeImpl<RSS<Scalar>>
{
  NODE_TYPE operator()()
  {
    return BV_RSS;
  }
};

//==============================================================================
template <typename Scalar>
struct GetNodeTypeImpl<kIOS<Scalar>>
{
  NODE_TYPE operator()()
  {
    return BV_kIOS;
  }
};

//==============================================================================
template <typename Scalar>
struct GetNodeTypeImpl<OBBRSS<Scalar>>
{
  NODE_TYPE operator()()
  {
    return BV_OBBRSS;
  }
};

//==============================================================================
template <typename Scalar>
struct GetNodeTypeImpl<KDOP<Scalar, 16>>
{
  NODE_TYPE operator()()
  {
    return BV_KDOP16;
  }
};

//==============================================================================
template <typename Scalar>
struct GetNodeTypeImpl<KDOP<Scalar, 18>>
{
  NODE_TYPE operator()()
  {
    return BV_KDOP18;
  }
};

//==============================================================================
template <typename Scalar>
struct GetNodeTypeImpl<KDOP<Scalar, 24>>
{
  NODE_TYPE operator()()
  {
    return BV_KDOP24;
  }
};

} // namespace fcl

#endif

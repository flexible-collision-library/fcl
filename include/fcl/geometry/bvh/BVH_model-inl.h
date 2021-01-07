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

#ifndef FCL_BVH_MODEL_INL_H
#define FCL_BVH_MODEL_INL_H

#include "fcl/geometry/bvh/BVH_model.h"
#include <new>
#include <algorithm>

namespace fcl
{

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
BVHModel<BV>::BVHModel() : vertices(nullptr),
  tri_indices(nullptr),
  prev_vertices(nullptr),
  num_tris(0),
  num_vertices(0),
  build_state(BVH_BUILD_STATE_EMPTY),
  bv_splitter(new detail::BVSplitter<BV>(detail::SPLIT_METHOD_MEAN)),
  bv_fitter(new detail::BVFitter<BV>()),
  num_tris_allocated(0),
  num_vertices_allocated(0),
  num_bvs_allocated(0),
  num_vertex_updated(0),
  primitive_indices(nullptr),
  bvs(nullptr),
  num_bvs(0)
{
  // Do nothing
}

//==============================================================================
template <typename BV>
BVHModel<BV>::BVHModel(const BVHModel<BV>& other)
  : CollisionGeometry<S>(other),
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
    vertices = new Vector3<S>[num_vertices];
    std::copy(other.vertices, other.vertices + num_vertices, vertices);
  }
  else
    vertices = nullptr;

  if(other.tri_indices)
  {
    tri_indices = new Triangle[num_tris];
    std::copy(other.tri_indices, other.tri_indices + num_tris, tri_indices);
  }
  else
    tri_indices = nullptr;

  if(other.prev_vertices)
  {
    prev_vertices = new Vector3<S>[num_vertices];
    std::copy(other.prev_vertices, other.prev_vertices + num_vertices, prev_vertices);
  }
  else
    prev_vertices = nullptr;

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
    std::copy(other.primitive_indices, other.primitive_indices + num_primitives, primitive_indices);
  }
  else
    primitive_indices = nullptr;

  num_bvs = num_bvs_allocated = other.num_bvs;
  if(other.bvs)
  {
    bvs = new BVNode<BV>[num_bvs];
    std::copy(other.bvs, other.bvs + num_bvs, bvs);
  }
  else
    bvs = nullptr;
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
  static NODE_TYPE run()
  {
    return BV_UNKNOWN;
  }
};

//==============================================================================
template <typename BV>
NODE_TYPE BVHModel<BV>::getNodeType() const
{
  return GetNodeTypeImpl<BV>::run();
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::beginModel(int num_tris_, int num_vertices_)
{
  if(build_state != BVH_BUILD_STATE_EMPTY)
  {
    delete [] vertices; vertices = nullptr;
    delete [] tri_indices; tri_indices = nullptr;
    delete [] bvs; bvs = nullptr;
    delete [] prev_vertices; prev_vertices = nullptr;
    delete [] primitive_indices; primitive_indices = nullptr;

    num_vertices_allocated = num_vertices = num_tris_allocated = num_tris = num_bvs_allocated = num_bvs = 0;
  }

  if(num_tris_ < 0) num_tris_ = 8;
  if(num_vertices_ <= 0) num_vertices_ = 8;

  num_vertices_allocated = num_vertices_;
  num_tris_allocated = num_tris_;

  if(num_tris_ > 0)
  {
    tri_indices = new(std::nothrow) Triangle[num_tris_allocated];
    if(!tri_indices)
    {
      std::cerr << "BVH Error! Out of memory for tri_indices array on BeginModel() call!" << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }
  }

  vertices = new Vector3<S>[num_vertices_allocated];
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
int BVHModel<BV>::addVertex(const Vector3<S>& p)
{
  if(build_state != BVH_BUILD_STATE_BEGUN)
  {
    std::cerr << "BVH Warning! Call addVertex() in a wrong order. addVertex() was ignored. Must do a beginModel() to clear the model for addition of new vertices." << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  if(num_vertices >= num_vertices_allocated)
  {
    Vector3<S>* temp = new Vector3<S>[num_vertices_allocated * 2];
    if(!temp)
    {
      std::cerr << "BVH Error! Out of memory for vertices array on addVertex() call!" << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }

    std::copy(vertices, vertices + num_vertices, temp);
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
int BVHModel<BV>::addTriangle(const Vector3<S>& p1, const Vector3<S>& p2, const Vector3<S>& p3)
{
  if(build_state == BVH_BUILD_STATE_PROCESSED)
  {
    std::cerr << "BVH Warning! Call addTriangle() in a wrong order. addTriangle() was ignored. Must do a beginModel() to clear the model for addition of new triangles." << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  if(num_vertices + 2 >= num_vertices_allocated)
  {
    Vector3<S>* temp = new Vector3<S>[num_vertices_allocated * 2 + 2];
    if(!temp)
    {
      std::cerr << "BVH Error! Out of memory for vertices array on addTriangle() call!" << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }

    std::copy(vertices, vertices + num_vertices, temp);
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
    if(num_tris_allocated == 0)
    {
      num_tris_allocated = 1;
    }
    Triangle* temp = new Triangle[num_tris_allocated * 2];
    if(!temp)
    {
      std::cerr << "BVH Error! Out of memory for tri_indices array on addTriangle() call!" << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }

    std::copy(tri_indices, tri_indices + num_tris, temp);
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
int BVHModel<BV>::addSubModel(const std::vector<Vector3<S>>& ps)
{
  if(build_state == BVH_BUILD_STATE_PROCESSED)
  {
    std::cerr << "BVH Warning! Call addSubModel() in a wrong order. addSubModel() was ignored. Must do a beginModel() to clear the model for addition of new vertices." << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  int num_vertices_to_add = ps.size();

  if(num_vertices + num_vertices_to_add - 1 >= num_vertices_allocated)
  {
    Vector3<S>* temp = new Vector3<S>[num_vertices_allocated * 2 + num_vertices_to_add - 1];
    if(!temp)
    {
      std::cerr << "BVH Error! Out of memory for vertices array on addSubModel() call!" << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }

    std::copy(vertices, vertices + num_vertices, temp);
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
int BVHModel<BV>::addSubModel(const std::vector<Vector3<S>>& ps, const std::vector<Triangle>& ts)
{
  if(build_state == BVH_BUILD_STATE_PROCESSED)
  {
    std::cerr << "BVH Warning! Call addSubModel() in a wrong order. addSubModel() was ignored. Must do a beginModel() to clear the model for addition of new vertices." << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  int num_vertices_to_add = ps.size();

  if(num_vertices + num_vertices_to_add - 1 >= num_vertices_allocated)
  {
    Vector3<S>* temp = new Vector3<S>[num_vertices_allocated * 2 + num_vertices_to_add - 1];
    if(!temp)
    {
      std::cerr << "BVH Error! Out of memory for vertices array on addSubModel() call!" << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }

    std::copy(vertices, vertices + num_vertices, temp);
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
    if(num_tris_allocated == 0)
    {
      num_tris_allocated = 1;
    }
    Triangle* temp = new(std::nothrow) Triangle[num_tris_allocated * 2 + num_tris_to_add - 1];
    if(!temp)
    {
      std::cerr << "BVH Error! Out of memory for tri_indices array on addSubModel() call!" << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }

    std::copy(tri_indices, tri_indices + num_tris, temp);
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
    Triangle* new_tris = new(std::nothrow) Triangle[num_tris];
    if(!new_tris)
    {
      std::cerr << "BVH Error! Out of memory for tri_indices array in endModel() call!" << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }
    std::copy(tri_indices, tri_indices + num_tris, new_tris);
    delete [] tri_indices;
    tri_indices = new_tris;
    num_tris_allocated = num_tris;
  }

  if(num_vertices_allocated > num_vertices)
  {
    Vector3<S>* new_vertices = new Vector3<S>[num_vertices];
    if(!new_vertices)
    {
      std::cerr << "BVH Error! Out of memory for vertices array in endModel() call!" << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }
    std::copy(vertices, vertices + num_vertices, new_vertices);
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


  bvs = new(std::nothrow) BVNode<BV> [num_bvs_to_be_allocated];
  primitive_indices = new(std::nothrow) unsigned int [num_bvs_to_be_allocated];
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

  if(prev_vertices)
  {
    delete [] prev_vertices;
    prev_vertices = nullptr;
  }

  num_vertex_updated = 0;

  build_state = BVH_BUILD_STATE_REPLACE_BEGUN;

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::replaceVertex(const Vector3<S>& p)
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
int BVHModel<BV>::replaceTriangle(const Vector3<S>& p1, const Vector3<S>& p2, const Vector3<S>& p3)
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
int BVHModel<BV>::replaceSubModel(const std::vector<Vector3<S>>& ps)
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
    Vector3<S>* temp = prev_vertices;
    prev_vertices = vertices;
    vertices = temp;
  }
  else
  {
    prev_vertices = vertices;
    vertices = new Vector3<S>[num_vertices];
  }

  num_vertex_updated = 0;

  build_state = BVH_BUILD_STATE_UPDATE_BEGUN;

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::updateVertex(const Vector3<S>& p)
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
int BVHModel<BV>::updateTriangle(const Vector3<S>& p1, const Vector3<S>& p2, const Vector3<S>& p3)
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
int BVHModel<BV>::updateSubModel(const std::vector<Vector3<S>>& ps)
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
  int mem_vertex_list = sizeof(Vector3<S>) * num_vertices;

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
        0, Matrix3<S>::Identity(), Vector3<S>::Zero());
}

//==============================================================================
template <typename BV>
Vector3<typename BV::S> BVHModel<BV>::computeCOM() const
{
  S vol = 0;
  Vector3<S> com = Vector3<S>::Zero();
  for(int i = 0; i < num_tris; ++i)
  {
    const Triangle& tri = tri_indices[i];
    S d_six_vol = (vertices[tri[0]].cross(vertices[tri[1]])).dot(vertices[tri[2]]);
    vol += d_six_vol;
    com.noalias() += (vertices[tri[0]] + vertices[tri[1]] + vertices[tri[2]]) * d_six_vol;
  }

  return com / (vol * 4);
}

//==============================================================================
template <typename BV>
typename BV::S BVHModel<BV>::computeVolume() const
{
  S vol = 0;
  for(int i = 0; i < num_tris; ++i)
  {
    const Triangle& tri = tri_indices[i];
    S d_six_vol = (vertices[tri[0]].cross(vertices[tri[1]])).dot(vertices[tri[2]]);
    vol += d_six_vol;
  }

  return vol / 6;
}

//==============================================================================
template <typename BV>
Matrix3<typename BV::S> BVHModel<BV>::computeMomentofInertia() const
{
  Matrix3<S> C = Matrix3<S>::Zero();

  Matrix3<S> C_canonical;
  C_canonical << 1/ 60.0, 1/120.0, 1/120.0,
      1/120.0, 1/ 60.0, 1/120.0,
      1/120.0, 1/120.0, 1/ 60.0;

  for(int i = 0; i < num_tris; ++i)
  {
    const Triangle& tri = tri_indices[i];
    const Vector3<S>& v1 = vertices[tri[0]];
    const Vector3<S>& v2 = vertices[tri[1]];
    const Vector3<S>& v3 = vertices[tri[2]];
    S d_six_vol = (v1.cross(v2)).dot(v3);
    Matrix3<S> A;
    A.row(0) = v1;
    A.row(1) = v2;
    A.row(2) = v3;
    C.noalias() += A.transpose() * C_canonical * A * d_six_vol;
  }

  S trace_C = C(0, 0) + C(1, 1) + C(2, 2);

  Matrix3<S> m;
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
      Vector3<S> p;
      if(type == BVH_MODEL_POINTCLOUD) p = vertices[cur_primitive_indices[i]];
      else if(type == BVH_MODEL_TRIANGLES)
      {
        const Triangle& t = tri_indices[cur_primitive_indices[i]];
        const Vector3<S>& p1 = vertices[t[0]];
        const Vector3<S>& p2 = vertices[t[1]];
        const Vector3<S>& p3 = vertices[t[2]];
        p.noalias() = (p1 + p2 + p3) / 3.0;
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
        Vector3<S> v[2];
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
        Vector3<S> v[6];
        for(int i = 0; i < 3; ++i)
        {
          v[i] = prev_vertices[triangle[i]];
          v[i + 3] = vertices[triangle[i]];
        }

        fit(v, 6, bv);
      }
      else
      {
        Vector3<S> v[3];
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
template <typename S, typename BV>
struct MakeParentRelativeRecurseImpl
{
  static void run(BVHModel<BV>& model,
                  int bv_id,
                  const Matrix3<S>& parent_axis,
                  const Vector3<S>& parent_c)
  {
    if(!model.bvs[bv_id].isLeaf())
    {
      MakeParentRelativeRecurseImpl<S, BV> tmp1;
      tmp1(model, model.bvs[bv_id].first_child, parent_axis, model.bvs[bv_id].getCenter());

      MakeParentRelativeRecurseImpl<S, BV> tmp2;
      tmp2(model, model.bvs[bv_id].first_child + 1, parent_axis, model.bvs[bv_id].getCenter());
    }

    model.bvs[bv_id].bv = translate(model.bvs[bv_id].bv, -parent_c);
  }
};

//==============================================================================
template <typename BV>
void BVHModel<BV>::makeParentRelativeRecurse(
    int bv_id,
    const Matrix3<S>& parent_axis,
    const Vector3<S>& parent_c)
{
  MakeParentRelativeRecurseImpl<typename BV::S, BV>::run(
        *this, bv_id, parent_axis, parent_c);
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
  AABB<S> aabb_;
  for(int i = 0; i < num_vertices; ++i)
  {
    aabb_ += vertices[i];
  }

  this->aabb_center = aabb_.center();

  this->aabb_radius = 0;
  for(int i = 0; i < num_vertices; ++i)
  {
    S r = (this->aabb_center - vertices[i]).squaredNorm();
    if(r > this->aabb_radius) this->aabb_radius = r;
  }

  this->aabb_radius = sqrt(this->aabb_radius);

  this->aabb_local = aabb_;
}

//==============================================================================
template <typename S>
struct MakeParentRelativeRecurseImpl<S, OBB<S>>
{
  static void run(BVHModel<OBB<S>>& model,
                  int bv_id,
                  const Matrix3<S>& parent_axis,
                  const Vector3<S>& parent_c)
  {
    OBB<S>& obb = model.bvs[bv_id].bv;
    if(!model.bvs[bv_id].isLeaf())
    {
      MakeParentRelativeRecurseImpl<S, OBB<S>> tmp1;
      tmp1(model, model.bvs[bv_id].first_child, obb.axis, obb.To);

      MakeParentRelativeRecurseImpl<S, OBB<S>> tmp2;
      tmp2(model, model.bvs[bv_id].first_child + 1, obb.axis, obb.To);
    }

    // make self parent relative
    obb.axis = parent_axis.transpose() * obb.axis;
    obb.To = (obb.To - parent_c).transpose() * parent_axis;
  }
};

//==============================================================================
template <typename S>
struct MakeParentRelativeRecurseImpl<S, RSS<S>>
{
  static void run(BVHModel<RSS<S>>& model,
                  int bv_id,
                  const Matrix3<S>& parent_axis,
                  const Vector3<S>& parent_c)
  {
    RSS<S>& rss = model.bvs[bv_id].bv;
    if(!model.bvs[bv_id].isLeaf())
    {
      MakeParentRelativeRecurseImpl<S, RSS<S>> tmp1;
      tmp1(model, model.bvs[bv_id].first_child, rss.axis, rss.To);

      MakeParentRelativeRecurseImpl<S, RSS<S>> tmp2;
      tmp2(model, model.bvs[bv_id].first_child + 1, rss.axis, rss.To);
    }

    // make self parent relative
    rss.axis = parent_axis.transpose() * rss.axis;
    rss.To = (rss.To - parent_c).transpose() * parent_axis;
  }
};

//==============================================================================
template <typename S>
struct MakeParentRelativeRecurseImpl<S, OBBRSS<S>>
{
  static void run(BVHModel<OBBRSS<S>>& model,
                  int bv_id,
                  const Matrix3<S>& parent_axis,
                  const Vector3<S>& parent_c)
  {
    OBB<S>& obb = model.bvs[bv_id].bv.obb;
    RSS<S>& rss = model.bvs[bv_id].bv.rss;
    if(!model.bvs[bv_id].isLeaf())
    {
      MakeParentRelativeRecurseImpl<S, RSS<S>> tmp1;
      tmp1(model, model.bvs[bv_id].first_child, obb.axis, obb.To);

      MakeParentRelativeRecurseImpl<S, RSS<S>> tmp2;
      tmp2(model, model.bvs[bv_id].first_child + 1, obb.axis, obb.To);
    }

    // make self parent relative
    obb.axis = parent_axis.transpose() * obb.axis;
    rss.axis = obb.axis;

    obb.To = (obb.To - parent_c).transpose() * parent_axis;
    rss.To = obb.To;
  }
};

//==============================================================================
template <typename S>
struct GetNodeTypeImpl<AABB<S>>
{
  static NODE_TYPE run()
  {
    return BV_AABB;
  }
};

//==============================================================================
template <typename S>
struct GetNodeTypeImpl<OBB<S>>
{
  static NODE_TYPE run()
  {
    return BV_OBB;
  }
};

//==============================================================================
template <typename S>
struct GetNodeTypeImpl<RSS<S>>
{
  static NODE_TYPE run()
  {
    return BV_RSS;
  }
};

//==============================================================================
template <typename S>
struct GetNodeTypeImpl<kIOS<S>>
{
  static NODE_TYPE run()
  {
    return BV_kIOS;
  }
};

//==============================================================================
template <typename S>
struct GetNodeTypeImpl<OBBRSS<S>>
{
  static NODE_TYPE run()
  {
    return BV_OBBRSS;
  }
};

//==============================================================================
template <typename S>
struct GetNodeTypeImpl<KDOP<S, 16>>
{
  static NODE_TYPE run()
  {
    return BV_KDOP16;
  }
};

//==============================================================================
template <typename S>
struct GetNodeTypeImpl<KDOP<S, 18>>
{
  static NODE_TYPE run()
  {
    return BV_KDOP18;
  }
};

//==============================================================================
template <typename S>
struct GetNodeTypeImpl<KDOP<S, 24>>
{
  static NODE_TYPE run()
  {
    return BV_KDOP24;
  }
};

} // namespace fcl

#endif

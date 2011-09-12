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

#include "fcl/BVH_model.h"
#include "fcl/BV.h"
#include <iostream>
#include <string.h>

namespace fcl
{

template<typename BV>
BVHModel<BV>::BVHModel(const BVHModel<BV>& other) : CollisionObject(other)
{
  num_tris = num_tris_allocated = other.num_tris;
  num_vertices = num_vertices_allocated = other.num_vertices;
  if(other.vertices)
  {
    vertices = new Vec3f[num_vertices];
    memcpy(vertices, other.vertices, sizeof(Vec3f) * num_vertices);
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
    prev_vertices = new Vec3f[num_vertices];
    memcpy(prev_vertices, other.prev_vertices, sizeof(Vec3f) * num_vertices);
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

  build_state = other.build_state;

  bv_splitter = other.bv_splitter;

  bv_fitter = other.bv_fitter;
}


template<typename BV>
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
  vertices = new Vec3f[num_vertices_allocated];

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


template<typename BV>
int BVHModel<BV>::addVertex(const Vec3f& p)
{
  if(build_state != BVH_BUILD_STATE_UPDATE_BEGUN)
  {
    std::cerr << "BVH Warning! Call addVertex() in a wrong order. addVertex() was ignored. Must do a beginModel() to clear the model for addition of new vertices." << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  if(num_vertices >= num_vertices_allocated)
  {
    Vec3f* temp = new Vec3f[num_vertices_allocated * 2];
    if(!temp)
    {
      std::cerr << "BVH Error! Out of memory for vertices array on addVertex() call!" << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }

    memcpy(temp, vertices, sizeof(Vec3f) * num_vertices);
    delete [] vertices;
    vertices = temp;
    num_vertices_allocated *= 2;
  }

  vertices[num_vertices] = p;
  num_vertices += 1;

  return BVH_OK;
}

template<typename BV>
int BVHModel<BV>::addTriangle(const Vec3f& p1, const Vec3f& p2, const Vec3f& p3)
{
  if(build_state == BVH_BUILD_STATE_PROCESSED)
  {
    std::cerr << "BVH Warning! Call addTriangle() in a wrong order. addTriangle() was ignored. Must do a beginModel() to clear the model for addition of new triangles." << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  if(num_vertices + 2 >= num_vertices_allocated)
  {
    Vec3f* temp = new Vec3f[num_vertices_allocated * 2 + 2];
    if(!temp)
    {
      std::cerr << "BVH Error! Out of memory for vertices array on addTriangle() call!" << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }

    memcpy(temp, vertices, sizeof(Vec3f) * num_vertices);
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

  tri_indices[num_tris] = Triangle(offset, offset + 1, offset + 2);

  return BVH_OK;
}

template<typename BV>
int BVHModel<BV>::addSubModel(const std::vector<Vec3f>& ps)
{
  if(build_state == BVH_BUILD_STATE_PROCESSED)
  {
    std::cerr << "BVH Warning! Call addSubModel() in a wrong order. addSubModel() was ignored. Must do a beginModel() to clear the model for addition of new vertices." << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  int num_vertices_to_add = ps.size();

  if(num_vertices + num_vertices_to_add - 1 >= num_vertices_allocated)
  {
    Vec3f* temp = new Vec3f[num_vertices_allocated * 2 + num_vertices_to_add - 1];
    if(!temp)
    {
      std::cerr << "BVH Error! Out of memory for vertices array on addSubModel() call!" << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }

    memcpy(temp, vertices, sizeof(Vec3f) * num_vertices);
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

template<typename BV>
int BVHModel<BV>::addSubModel(const std::vector<Vec3f>& ps, const std::vector<Triangle>& ts)
{
  if(build_state == BVH_BUILD_STATE_PROCESSED)
  {
    std::cerr << "BVH Warning! Call addSubModel() in a wrong order. addSubModel() was ignored. Must do a beginModel() to clear the model for addition of new vertices." << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  int num_vertices_to_add = ps.size();

  if(num_vertices + num_vertices_to_add - 1 >= num_vertices_allocated)
  {
    Vec3f* temp = new Vec3f[num_vertices_allocated * 2 + num_vertices_to_add - 1];
    if(!temp)
    {
      std::cerr << "BVH Error! Out of memory for vertices array on addSubModel() call!" << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }

    memcpy(temp, vertices, sizeof(Vec3f) * num_vertices);
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
    tri_indices[num_tris] = Triangle(t[0] + offset, t[1] + offset, t[2] + offset);
    num_tris++;
  }

  return BVH_OK;
}

template<typename BV>
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
    Vec3f* new_vertices = new Vec3f[num_vertices];
    if(!new_vertices)
    {
      std::cerr << "BVH Error! Out of memory for vertices array in endModel() call!" << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }
    memcpy(new_vertices, vertices, sizeof(Vec3f) * num_vertices);
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



template<typename BV>
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

template<typename BV>
int BVHModel<BV>::replaceVertex(const Vec3f& p)
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

template<typename BV>
int BVHModel<BV>::replaceTriangle(const Vec3f& p1, const Vec3f& p2, const Vec3f& p3)
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

template<typename BV>
int BVHModel<BV>::replaceSubModel(const std::vector<Vec3f>& ps)
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

template<typename BV>
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





template<typename BV>
int BVHModel<BV>::beginUpdateModel()
{
  if(build_state != BVH_BUILD_STATE_PROCESSED && build_state != BVH_BUILD_STATE_UPDATED)
  {
    std::cerr << "BVH Error! Call beginUpdatemodel() on a BVHModel that has no previous frame." << std::endl;
    return BVH_ERR_BUILD_EMPTY_PREVIOUS_FRAME;
  }

  if(prev_vertices)
  {
    Vec3f* temp = prev_vertices;
    prev_vertices = vertices;
    vertices = temp;
  }
  else
  {
    prev_vertices = vertices;
    vertices = new Vec3f[num_vertices];
  }

  num_vertex_updated = 0;

  build_state = BVH_BUILD_STATE_UPDATE_BEGUN;

  return BVH_OK;
}

template<typename BV>
int BVHModel<BV>::updateVertex(const Vec3f& p)
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

template<typename BV>
int BVHModel<BV>::updateTriangle(const Vec3f& p1, const Vec3f& p2, const Vec3f& p3)
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

template<typename BV>
int BVHModel<BV>::updateSubModel(const std::vector<Vec3f>& ps)
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

template<typename BV>
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




template<typename BV>
int BVHModel<BV>::memUsage(int msg) const
{
  int mem_bv_list = sizeof(BV) * num_bvs;
  int mem_tri_list = sizeof(Triangle) * num_tris;
  int mem_vertex_list = sizeof(Vec3f) * num_vertices;

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


template<typename BV>
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

template<typename BV>
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
      Vec3f p;
      if(type == BVH_MODEL_POINTCLOUD) p = vertices[cur_primitive_indices[i]];
      else if(type == BVH_MODEL_TRIANGLES)
      {
        const Triangle& t = tri_indices[cur_primitive_indices[i]];
        const Vec3f& p1 = vertices[t[0]];
        const Vec3f& p2 = vertices[t[1]];
        const Vec3f& p3 = vertices[t[2]];
        BVH_REAL x = (p1[0] + p2[0] + p3[0]) / 3;
        BVH_REAL y = (p1[1] + p2[1] + p3[1]) / 3;
        BVH_REAL z = (p1[2] + p2[2] + p3[2]) / 3;
        p = Vec3f(x, y, z);
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
        unsigned int temp = cur_primitive_indices[i];
        cur_primitive_indices[i] = cur_primitive_indices[c1];
        cur_primitive_indices[c1] = temp;
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

template<typename BV>
int BVHModel<BV>::refitTree(bool bottomup)
{
  if(bottomup)
    return refitTree_bottomup();
  else
    return refitTree_topdown();
}

template<typename BV>
int BVHModel<BV>::refitTree_bottomup()
{
  int res = recursiveRefitTree_bottomup(0);

  return res;
}


template<typename BV>
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
        Vec3f v[2];
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
        Vec3f v[6];
        for(int i = 0; i < 3; ++i)
        {
          v[i] = prev_vertices[triangle[i]];
          v[i + 3] = vertices[triangle[i]];
        }

        fit(v, 6, bv);
      }
      else
      {
        Vec3f v[3];
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

template<typename BV>
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

template<typename BV>
void BVHModel<BV>::computeLocalAABB()
{
  AABB aabb_;
  for(int i = 0; i < num_vertices; ++i)
  {
    aabb_ += vertices[i];
  }

  aabb_local = aabb_;

  aabb = aabb_local;

  aabb_center = aabb_local.center();

  aabb_radius = 0;
  for(int i = 0; i < num_vertices; ++i)
  {
    BVH_REAL r = (aabb_center - vertices[i]).sqrLength();
    if(r > aabb_radius) aabb_radius = r;
  }

  aabb_radius = sqrt(aabb_radius);
}

template<>
NODE_TYPE BVHModel<AABB>::getNodeType() const
{
  return BV_AABB;
}

template<>
NODE_TYPE BVHModel<OBB>::getNodeType() const
{
  return BV_OBB;
}

template<>
NODE_TYPE BVHModel<RSS>::getNodeType() const
{
  return BV_RSS;
}

template<>
NODE_TYPE BVHModel<KDOP<16> >::getNodeType() const
{
  return BV_KDOP16;
}

template<>
NODE_TYPE BVHModel<KDOP<18> >::getNodeType() const
{
  return BV_KDOP18;
}

template<>
NODE_TYPE BVHModel<KDOP<24> >::getNodeType() const
{
  return BV_KDOP24;
}



template class BVHModel<KDOP<16> >;
template class BVHModel<KDOP<18> >;
template class BVHModel<KDOP<24> >;
template class BVHModel<OBB>;
template class BVHModel<AABB>;
template class BVHModel<RSS>;

}

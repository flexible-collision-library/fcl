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

#ifndef FCL_BV_SPLITTER_H
#define FCL_BV_SPLITTER_H


#include "fcl/BVH_internal.h"
#include "fcl/primitive.h"
#include "fcl/vec_3f.h"
#include "fcl/OBB.h"
#include <vector>
#include <iostream>

/** \brief Main namespace */
namespace fcl
{

/** \brief Base class for BV splitting operation */
template<typename BV>
class BVSplitterBase
{
public:
  /** \brief Set the geometry data needed by the split rule */
  virtual void set(Vec3f* vertices_, Triangle* tri_indices_, BVHModelType type_) = 0;

  /** \brief Compute the split rule according to a subset of geometry and the corresponding BV node */
  virtual void computeRule(const BV& bv, unsigned int* primitive_indices, int num_primitives) = 0;

  /** \brief Apply the split rule on a given point */
  virtual bool apply(const Vec3f& q) const = 0;

  /** \brief Clear the geometry data set before */
  virtual void clear() = 0;
};


enum SplitMethodType {SPLIT_METHOD_MEAN, SPLIT_METHOD_MEDIAN, SPLIT_METHOD_BV_CENTER};


/** \brief A class describing the split rule that splits each BV node */
template<typename BV>
class BVSplitter : public BVSplitterBase<BV>
{
public:

  BVSplitter(SplitMethodType method)
  {
    split_method = method;
  }

  /** \brief Set the geometry data needed by the split rule */
  void set(Vec3f* vertices_, Triangle* tri_indices_, BVHModelType type_)
  {
    vertices = vertices_;
    tri_indices = tri_indices_;
    type = type_;
  }

  /** \brief Compute the split rule according to a subset of geometry and the corresponding BV node */
  void computeRule(const BV& bv, unsigned int* primitive_indices, int num_primitives)
  {
    switch(split_method)
    {
      case SPLIT_METHOD_MEAN:
        computeRule_mean(bv, primitive_indices, num_primitives);
        break;
      case SPLIT_METHOD_MEDIAN:
        computeRule_median(bv, primitive_indices, num_primitives);
        break;
      case SPLIT_METHOD_BV_CENTER:
        computeRule_bvcenter(bv, primitive_indices, num_primitives);
        break;
      default:
        std::cerr << "Split method not supported" << std::endl;
    }
  }

  /** \brief Apply the split rule on a given point */
  bool apply(const Vec3f& q) const
  {
    return q[split_axis] > split_value;
  }

  /** \brief Clear the geometry data set before */
  void clear()
  {
    vertices = NULL;
    tri_indices = NULL;
    type = BVH_MODEL_UNKNOWN;
  }

private:

  /** \brief The split axis */
  int split_axis;

  /** \brief The split threshold */
  BVH_REAL split_value;

  Vec3f* vertices;
  Triangle* tri_indices;
  BVHModelType type;
  SplitMethodType split_method;

  /** \brief Split the node from center */
  void computeRule_bvcenter(const BV& bv, unsigned int* primitive_indices, int num_primitives)
  {
    Vec3f center = bv.center();
    int axis = 2;

    if(bv.width() >= bv.height() && bv.width() >= bv.depth())
      axis = 0;
    else if(bv.height() >= bv.width() && bv.height() >= bv.depth())
      axis = 1;

    split_axis = axis;
    split_value = center[axis];
  }

  /** \brief Split the node according to the mean of the data contained */
  void computeRule_mean(const BV& bv, unsigned int* primitive_indices, int num_primitives)
  {
    int axis = 2;

    if(bv.width() >= bv.height() && bv.width() >= bv.depth())
      axis = 0;
    else if(bv.height() >= bv.width() && bv.height() >= bv.depth())
      axis = 1;

    split_axis = axis;
    BVH_REAL sum = 0;

    if(type == BVH_MODEL_TRIANGLES)
    {
      for(int i = 0; i < num_primitives; ++i)
      {
        const Triangle& t = tri_indices[primitive_indices[i]];
        sum += ((vertices[t[0]][split_axis] + vertices[t[1]][split_axis] + vertices[t[2]][split_axis]) / 3);
      }
    }
    else if(type == BVH_MODEL_POINTCLOUD)
    {
      for(int i = 0; i < num_primitives; ++i)
      {
        sum += vertices[primitive_indices[i]][split_axis];
      }
    }

    split_value = sum / num_primitives;
  }

  /** \brief Split the node according to the median of the data contained */
  void computeRule_median(const BV& bv, unsigned int* primitive_indices, int num_primitives)
  {
    int axis = 2;

    if(bv.width() >= bv.height() && bv.width() >= bv.depth())
      axis = 0;
    else if(bv.height() >= bv.width() && bv.height() >= bv.depth())
      axis = 1;

    split_axis = axis;
    std::vector<BVH_REAL> proj(num_primitives);

    if(type == BVH_MODEL_TRIANGLES)
    {
      for(int i = 0; i < num_primitives; ++i)
      {
        const Triangle& t = tri_indices[primitive_indices[i]];
        proj[i] = (vertices[t[0]][split_axis] + vertices[t[1]][split_axis] + vertices[t[2]][split_axis]) / 3;
      }
    }
    else if(type == BVH_MODEL_POINTCLOUD)
    {
      for(int i = 0; i < num_primitives; ++i)
        proj[i] = vertices[primitive_indices[i]][split_axis];
    }

    std::sort(proj.begin(), proj.end());

    if(num_primitives % 2 == 1)
    {
      split_value = proj[(num_primitives - 1) / 2];
    }
    else
    {
      split_value = (proj[num_primitives / 2] + proj[num_primitives / 2 - 1]) / 2;
    }
  }
};



/** \brief BVHSplitRule specialization for OBB */
template<>
class BVSplitter<OBB> : public BVSplitterBase<OBB>
{
public:

  BVSplitter(SplitMethodType method)
  {
    split_method = method;
  }

  /** \brief Set the geometry data needed by the split rule */
  void set(Vec3f* vertices_, Triangle* tri_indices_, BVHModelType type_)
  {
    vertices = vertices_;
    tri_indices = tri_indices_;
    type = type_;
  }

  /** \brief Compute the split rule according to a subset of geometry and the corresponding BV node */
  void computeRule(const OBB& bv, unsigned int* primitive_indices, int num_primitives)
  {
    Vec3f center = bv.center();
    split_vector = bv.axis[0];
    split_value = center[0];
  }

  /** \brief Apply the split rule on a given point */
  bool apply(const Vec3f& q) const
  {
    return split_vector.dot(Vec3f(q[0], q[1], q[2])) > split_value;
  }

  /** \brief Clear the geometry data set before */
  void clear()
  {
    vertices = NULL;
    tri_indices = NULL;
    type = BVH_MODEL_UNKNOWN;
  }

private:

  /** \brief Split the node from center */
  void computeRule_bvcenter(const OBB& bv, unsigned int* primitive_indices, int num_primitives);

  /** \brief Split the node according to the mean of the data contained */
  void computeRule_mean(const OBB& bv, unsigned int* primitive_indices, int num_primitives);

  /** \brief Split the node according to the median of the data contained */
  void computeRule_median(const OBB& bv, unsigned int* primitive_indices, int num_primitives);

  BVH_REAL split_value;
  Vec3f split_vector;

  Vec3f* vertices;
  Triangle* tri_indices;
  BVHModelType type;
  SplitMethodType split_method;
};

}

#endif

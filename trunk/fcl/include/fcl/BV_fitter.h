
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

#ifndef FCL_BV_FITTER_H
#define FCL_BV_FITTER_H

#include "fcl/BVH_internal.h"
#include "fcl/primitive.h"
#include "fcl/vec_3f.h"
#include "fcl/OBB.h"
#include "fcl/RSS.h"
#include <iostream>

/** \brief Main namespace */
namespace fcl
{

/** \brief Compute a bounding volume that fits a set of n points. */
template<typename BV>
void fit(Vec3f* ps, int n, BV& bv)
{
  for(int i = 0; i < n; ++i)
  {
    bv += ps[i];
  }
}

template<>
void fit<OBB>(Vec3f* ps, int n, OBB& bv);

template<>
void fit<RSS>(Vec3f* ps, int n, RSS& bv);


template<typename BV>
class BVFitterBase
{
public:
  virtual void set(Vec3f* vertices_, Triangle* tri_indices_, BVHModelType type_) = 0;

  virtual void set(Vec3f* vertices_, Vec3f* prev_vertices_, Triangle* tri_indices_, BVHModelType type_) = 0;

  virtual BV fit(unsigned int* primitive_indices, int num_primitives) = 0;

  virtual void clear() = 0;
};

/** \brief A class for fitting a bounding volume to a set of points  */
template<typename BV>
class BVFitter : public BVFitterBase<BV>
{
public:
  /** \brief Prepare the geometry primitive data for fitting */
  void set(Vec3f* vertices_, Triangle* tri_indices_, BVHModelType type_)
  {
    vertices = vertices_;
    prev_vertices = NULL;
    tri_indices = tri_indices_;
    type = type_;
  }

  /** \brief Prepare the geometry primitive data for fitting */
  void set(Vec3f* vertices_, Vec3f* prev_vertices_, Triangle* tri_indices_, BVHModelType type_)
  {
    vertices = vertices_;
    prev_vertices = prev_vertices_;
    tri_indices = tri_indices_;
    type = type_;
  }

  /** \brief Compute a bounding volume that fits a set of primitives (points or triangles).
   * The primitive data was set by set function and primitive_indices is the primitive index relative to the data
   */
  BV fit(unsigned int* primitive_indices, int num_primitives)
  {
    BV bv;

    if(type == BVH_MODEL_TRIANGLES)             /* The primitive is triangle */
    {
      for(int i = 0; i < num_primitives; ++i)
      {
        Triangle t = tri_indices[primitive_indices[i]];
        bv += vertices[t[0]];
        bv += vertices[t[1]];
        bv += vertices[t[2]];

        if(prev_vertices) /* When fitting both current and previous frame */
        {
          bv += prev_vertices[t[0]];
          bv += prev_vertices[t[1]];
          bv += prev_vertices[t[2]];
        }
      }
    }
    else if(type == BVH_MODEL_POINTCLOUD)       /* The primitive is point */
    {
      for(int i = 0; i < num_primitives; ++i)
      {
        bv += vertices[primitive_indices[i]];

        if(prev_vertices) /* When fitting both current and previous frame */
        {
          bv += prev_vertices[primitive_indices[i]];
        }
      }
    }

    return bv;
  }

  /** \brief Clear the geometry primitive data */
  void clear()
  {
    vertices = NULL;
    prev_vertices = NULL;
    tri_indices = NULL;
    type = BVH_MODEL_UNKNOWN;
  }

private:

  Vec3f* vertices;
  Vec3f* prev_vertices;
  Triangle* tri_indices;
  BVHModelType type;
};


/** \brief Specification of BVFitter for OBB bounding volume */
template<>
class BVFitter<OBB> : public BVFitterBase<OBB>
{
public:
  /** \brief Prepare the geometry primitive data for fitting */
  void set(Vec3f* vertices_, Triangle* tri_indices_, BVHModelType type_)
  {
    vertices = vertices_;
    prev_vertices = NULL;
    tri_indices = tri_indices_;
    type = type_;
  }

  /** \brief Prepare the geometry primitive data for fitting */
  void set(Vec3f* vertices_, Vec3f* prev_vertices_, Triangle* tri_indices_, BVHModelType type_)
  {
    vertices = vertices_;
    prev_vertices = prev_vertices_;
    tri_indices = tri_indices_;
    type = type_;
  }

  /** \brief Compute a bounding volume that fits a set of primitives (points or triangles).
   * The primitive data was set by set function and primitive_indices is the primitive index relative to the data.
   */
  OBB fit(unsigned int* primitive_indices, int num_primitives);

  /** \brief Clear the geometry primitive data */
  void clear()
  {
    vertices = NULL;
    prev_vertices = NULL;
    tri_indices = NULL;
    type = BVH_MODEL_UNKNOWN;
  }

private:

  Vec3f* vertices;
  Vec3f* prev_vertices;
  Triangle* tri_indices;
  BVHModelType type;
};


/** \brief Specification of BVFitter for RSS bounding volume */
template<>
class BVFitter<RSS> : public BVFitterBase<RSS>
{
public:
  /** \brief Prepare the geometry primitive data for fitting */
  void set(Vec3f* vertices_, Triangle* tri_indices_, BVHModelType type_)
  {
    vertices = vertices_;
    prev_vertices = NULL;
    tri_indices = tri_indices_;
    type = type_;
  }

  /** \brief Prepare the geometry primitive data for fitting */
  void set(Vec3f* vertices_, Vec3f* prev_vertices_, Triangle* tri_indices_, BVHModelType type_)
  {
    vertices = vertices_;
    prev_vertices = prev_vertices_;
    tri_indices = tri_indices_;
    type = type_;
  }

  /** \brief Compute a bounding volume that fits a set of primitives (points or triangles).
   * The primitive data was set by set function and primitive_indices is the primitive index relative to the data.
   */
  RSS fit(unsigned int* primitive_indices, int num_primitives);

  /** \brief Clear the geometry primitive data */
  void clear()
  {
    vertices = NULL;
    prev_vertices = NULL;
    tri_indices = NULL;
    type = BVH_MODEL_UNKNOWN;
  }

private:

  Vec3f* vertices;
  Vec3f* prev_vertices;
  Triangle* tri_indices;
  BVHModelType type;
};

}

#endif

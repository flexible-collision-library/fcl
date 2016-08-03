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

#ifndef FCL_BV_FITTER_H
#define FCL_BV_FITTER_H

#include "fcl/math/triangle.h"
#include "fcl/BVH/BVH_internal.h"
#include "fcl/BV/kIOS.h"
#include "fcl/BV/OBBRSS.h"
#include <iostream>

namespace fcl
{

/// @brief Interface for fitting a bv given the triangles or points inside it.
template <typename BV>
class BVFitterBase
{
public:

  using Scalar = typename BV::Scalar;

  /// @brief Set the primitives to be processed by the fitter
  virtual void set(Vector3<Scalar>* vertices_, Triangle* tri_indices_, BVHModelType type_) = 0;

  /// @brief Set the primitives to be processed by the fitter, for deformable mesh.
  virtual void set(Vector3<Scalar>* vertices_, Vector3<Scalar>* prev_vertices_, Triangle* tri_indices_, BVHModelType type_) = 0;

  /// @brief Compute the fitting BV
  virtual BV fit(unsigned int* primitive_indices, int num_primitives) = 0;

  /// @brief clear the temporary data generated.
  virtual void clear() = 0;
};

/// @brief The class for the default algorithm fitting a bounding volume to a set of points
template <typename BV>
class BVFitter : public BVFitterBase<BV>
{
public:

  using Scalar = typename BVFitterBase<BV>::Scalar;

  /// @brief default deconstructor
  virtual ~BVFitter();

  /// @brief Prepare the geometry primitive data for fitting
  void set(
      Vector3<Scalar>* vertices_, Triangle* tri_indices_, BVHModelType type_);

  /// @brief Prepare the geometry primitive data for fitting, for deformable mesh
  void set(
      Vector3<Scalar>* vertices_,
      Vector3<Scalar>* prev_vertices_,
      Triangle* tri_indices_,
      BVHModelType type_);

  /// @brief Compute a bounding volume that fits a set of primitives (points or triangles).
  /// The primitive data was set by set function and primitive_indices is the primitive index relative to the data
  BV fit(unsigned int* primitive_indices, int num_primitives);

  /// @brief Clear the geometry primitive data
  void clear();

private:

  Vector3<Scalar>* vertices;
  Vector3<Scalar>* prev_vertices;
  Triangle* tri_indices;
  BVHModelType type;
};


/// @brief Specification of BVFitter for OBB bounding volume
template <typename Scalar>
class BVFitter<OBB<Scalar>> : public BVFitterBase<OBB<Scalar>>
{
public:
  /// @brief Prepare the geometry primitive data for fitting
  void set(Vector3<Scalar>* vertices_, Triangle* tri_indices_, BVHModelType type_);

  /// @brief Prepare the geometry primitive data for fitting, for deformable mesh
  void set(Vector3<Scalar>* vertices_, Vector3<Scalar>* prev_vertices_, Triangle* tri_indices_, BVHModelType type_);

  /// @brief Compute a bounding volume that fits a set of primitives (points or triangles).
  /// The primitive data was set by set function and primitive_indices is the primitive index relative to the data.
  OBB<Scalar> fit(unsigned int* primitive_indices, int num_primitives);

  /// brief Clear the geometry primitive data
  void clear();

private:

  Vector3<Scalar>* vertices;
  Vector3<Scalar>* prev_vertices;
  Triangle* tri_indices;
  BVHModelType type;
};

/// @brief Specification of BVFitter for RSS bounding volume
template <typename Scalar>
class BVFitter<RSS<Scalar>> : public BVFitterBase<RSS<Scalar>>
{
public:
  /// brief Prepare the geometry primitive data for fitting
  void set(Vector3<Scalar>* vertices_, Triangle* tri_indices_, BVHModelType type_);

  /// @brief Prepare the geometry primitive data for fitting, for deformable mesh
  void set(Vector3<Scalar>* vertices_, Vector3<Scalar>* prev_vertices_, Triangle* tri_indices_, BVHModelType type_);

  /// @brief Compute a bounding volume that fits a set of primitives (points or triangles).
  /// The primitive data was set by set function and primitive_indices is the primitive index relative to the data.
  RSS<Scalar> fit(unsigned int* primitive_indices, int num_primitives);

  /// @brief Clear the geometry primitive data
  void clear();

private:

  Vector3<Scalar>* vertices;
  Vector3<Scalar>* prev_vertices;
  Triangle* tri_indices;
  BVHModelType type;
};

/// @brief Specification of BVFitter for kIOS bounding volume
template <typename Scalar>
class BVFitter<kIOS<Scalar>> : public BVFitterBase<kIOS<Scalar>>
{
public:
  /// @brief Prepare the geometry primitive data for fitting
  void set(Vector3<Scalar>* vertices_, Triangle* tri_indices_, BVHModelType type_);

  /// @brief Prepare the geometry primitive data for fitting
  void set(Vector3<Scalar>* vertices_, Vector3<Scalar>* prev_vertices_, Triangle* tri_indices_, BVHModelType type_);

  /// @brief Compute a bounding volume that fits a set of primitives (points or triangles).
  /// The primitive data was set by set function and primitive_indices is the primitive index relative to the data.
  kIOS<Scalar> fit(unsigned int* primitive_indices, int num_primitives);

  /// @brief Clear the geometry primitive data
  void clear();

private:
  Vector3<Scalar>* vertices;
  Vector3<Scalar>* prev_vertices;
  Triangle* tri_indices;
  BVHModelType type;
};

/// @brief Specification of BVFitter for OBBRSS bounding volume
template <typename Scalar>
class BVFitter<OBBRSS<Scalar>> : public BVFitterBase<OBBRSS<Scalar>>
{
public:
  /// @brief Prepare the geometry primitive data for fitting
  void set(Vector3<Scalar>* vertices_, Triangle* tri_indices_, BVHModelType type_);

  /// @brief Prepare the geometry primitive data for fitting
  void set(Vector3<Scalar>* vertices_, Vector3<Scalar>* prev_vertices_, Triangle* tri_indices_, BVHModelType type_);

  /// @brief Compute a bounding volume that fits a set of primitives (points or triangles).
  /// The primitive data was set by set function and primitive_indices is the primitive index relative to the data.
  OBBRSS<Scalar> fit(unsigned int* primitive_indices, int num_primitives);

  /// @brief Clear the geometry primitive data
  void clear();

private:

  Vector3<Scalar>* vertices;
  Vector3<Scalar>* prev_vertices;
  Triangle* tri_indices;
  BVHModelType type;
};

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename BV>
BVFitter<BV>::~BVFitter()
{
  // Do nothing
}

//==============================================================================
template <typename BV>
void BVFitter<BV>::set(
    Vector3<typename BVFitter<BV>::Scalar>* vertices_,
    Triangle* tri_indices_,
    BVHModelType type_)
{
  vertices = vertices_;
  prev_vertices = NULL;
  tri_indices = tri_indices_;
  type = type_;
}

//==============================================================================
template <typename BV>
void BVFitter<BV>::set(
    Vector3<typename BVFitter<BV>::Scalar>* vertices_,
    Vector3<typename BVFitter<BV>::Scalar>* prev_vertices_,
    Triangle* tri_indices_,
    BVHModelType type_)
{
  vertices = vertices_;
  prev_vertices = prev_vertices_;
  tri_indices = tri_indices_;
  type = type_;
}

//==============================================================================
template <typename BV>
BV BVFitter<BV>::fit(unsigned int* primitive_indices, int num_primitives)
{
  BV bv;

  if(type == BVH_MODEL_TRIANGLES)             /// The primitive is triangle
  {
    for(int i = 0; i < num_primitives; ++i)
    {
      Triangle t = tri_indices[primitive_indices[i]];
      bv += vertices[t[0]];
      bv += vertices[t[1]];
      bv += vertices[t[2]];

      if(prev_vertices)                      /// can fitting both current and previous frame
      {
        bv += prev_vertices[t[0]];
        bv += prev_vertices[t[1]];
        bv += prev_vertices[t[2]];
      }
    }
  }
  else if(type == BVH_MODEL_POINTCLOUD)       /// The primitive is point
  {
    for(int i = 0; i < num_primitives; ++i)
    {
      bv += vertices[primitive_indices[i]];

      if(prev_vertices)                       /// can fitting both current and previous frame
      {
        bv += prev_vertices[primitive_indices[i]];
      }
    }
  }

  return bv;
}

//==============================================================================
template <typename BV>
void BVFitter<BV>::clear()
{
  vertices = NULL;
  prev_vertices = NULL;
  tri_indices = NULL;
  type = BVH_MODEL_UNKNOWN;
}

//==============================================================================
template <typename Scalar>
void BVFitter<OBB<Scalar>>::set(
    Vector3<Scalar>* vertices_, Triangle* tri_indices_, BVHModelType type_)
{
  vertices = vertices_;
  prev_vertices = NULL;
  tri_indices = tri_indices_;
  type = type_;
}

//==============================================================================
template <typename Scalar>
void BVFitter<OBB<Scalar>>::set(
    Vector3<Scalar>* vertices_,
    Vector3<Scalar>* prev_vertices_,
    Triangle* tri_indices_, BVHModelType type_)
{
  vertices = vertices_;
  prev_vertices = prev_vertices_;
  tri_indices = tri_indices_;
  type = type_;
}

//==============================================================================
template <typename Scalar>
OBB<Scalar> BVFitter<OBB<Scalar>>::fit(
    unsigned int* primitive_indices, int num_primitives)
{
  OBB<Scalar> bv;

  Matrix3<Scalar> M; // row first matrix
  Matrix3<Scalar> E; // row first eigen-vectors
  Vector3<Scalar> s; // three eigen values

  getCovariance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, M);
  eigen(M, s, E);

  axisFromEigen(E, s, bv.axis);

  // set obb centers and extensions
  getExtentAndCenter(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, bv.axis, bv.To, bv.extent);

  return bv;
}

//==============================================================================
template <typename Scalar>
void BVFitter<OBB<Scalar>>::clear()
{
  vertices = NULL;
  prev_vertices = NULL;
  tri_indices = NULL;
  type = BVH_MODEL_UNKNOWN;
}

//==============================================================================
template <typename Scalar>
void BVFitter<RSS<Scalar>>::set(
    Vector3<Scalar>* vertices_, Triangle* tri_indices_, BVHModelType type_)
{
  vertices = vertices_;
  prev_vertices = NULL;
  tri_indices = tri_indices_;
  type = type_;
}

//==============================================================================
template <typename Scalar>
void BVFitter<RSS<Scalar>>::set(
    Vector3<Scalar>* vertices_,
    Vector3<Scalar>* prev_vertices_,
    Triangle* tri_indices_,
    BVHModelType type_)
{
  vertices = vertices_;
  prev_vertices = prev_vertices_;
  tri_indices = tri_indices_;
  type = type_;
}

//==============================================================================
template <typename Scalar>
RSS<Scalar> BVFitter<RSS<Scalar>>::fit(
    unsigned int* primitive_indices, int num_primitives)
{
  RSS<Scalar> bv;

  Matrix3<Scalar> M; // row first matrix
  Matrix3<Scalar> E; // row first eigen-vectors
  Vector3<Scalar> s; // three eigen values
  getCovariance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, M);
  eigen(M, s, E);
  axisFromEigen(E, s, bv.axis);

  // set rss origin, rectangle size and radius

  Vector3<Scalar> origin;
  Scalar l[2];
  Scalar r;
  getRadiusAndOriginAndRectangleSize(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, bv.axis, origin, l, r);

  bv.Tr = origin;
  bv.l[0] = l[0];
  bv.l[1] = l[1];
  bv.r = r;


  return bv;
}

//==============================================================================
template <typename Scalar>
void BVFitter<RSS<Scalar>>::clear()
{
  vertices = NULL;
  prev_vertices = NULL;
  tri_indices = NULL;
  type = BVH_MODEL_UNKNOWN;
}

//==============================================================================
template <typename Scalar>
void BVFitter<kIOS<Scalar>>::set(
    Vector3<Scalar>* vertices_,
    Vector3<Scalar>* prev_vertices_,
    Triangle* tri_indices_,
    BVHModelType type_)
{
  vertices = vertices_;
  prev_vertices = prev_vertices_;
  tri_indices = tri_indices_;
  type = type_;
}

//==============================================================================
template <typename Scalar>
void BVFitter<kIOS<Scalar>>::set(
    Vector3<Scalar>* vertices_, Triangle* tri_indices_, BVHModelType type_)
{
  vertices = vertices_;
  prev_vertices = NULL;
  tri_indices = tri_indices_;
  type = type_;
}

//==============================================================================
template <typename Scalar>
kIOS<Scalar> BVFitter<kIOS<Scalar>>::fit(
    unsigned int* primitive_indices, int num_primitives)
{
  kIOS<Scalar> bv;

  Matrix3<Scalar> M; // row first matrix
  Matrix3<Scalar> E; // row first eigen-vectors
  Vector3<Scalar> s;

  getCovariance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, M);
  eigen(M, s, E);

  axisFromEigen(E, s, bv.obb.axis);

  // get centers and extensions
  getExtentAndCenter(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, bv.obb.axis, bv.obb.To, bv.obb.extent);

  const Vector3<Scalar>& center = bv.obb.To;
  const Vector3<Scalar>& extent = bv.obb.extent;
  Scalar r0 = maximumDistance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, center);

  // decide k in kIOS
  if(extent[0] > kIOS<Scalar>::ratio() * extent[2])
  {
    if(extent[0] > kIOS<Scalar>::ratio() * extent[1]) bv.num_spheres = 5;
    else bv.num_spheres = 3;
  }
  else bv.num_spheres = 1;

  bv.spheres[0].o = center;
  bv.spheres[0].r = r0;

  if(bv.num_spheres >= 3)
  {
    Scalar r10 = sqrt(r0 * r0 - extent[2] * extent[2]) * kIOS<Scalar>::invSinA();
    Vector3<Scalar> delta = bv.obb.axis.col(2) * (r10 * kIOS<Scalar>::cosA() - extent[2]);
    bv.spheres[1].o = center - delta;
    bv.spheres[2].o = center + delta;

    Scalar r11 = maximumDistance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, bv.spheres[1].o);
    Scalar r12 = maximumDistance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, bv.spheres[2].o);

    bv.spheres[1].o += bv.obb.axis.col(2) * (-r10 + r11);
    bv.spheres[2].o += bv.obb.axis.col(2) * (r10 - r12);

    bv.spheres[1].r = r10;
    bv.spheres[2].r = r10;
  }

  if(bv.num_spheres >= 5)
  {
    Scalar r10 = bv.spheres[1].r;
    Vector3<Scalar> delta = bv.obb.axis.col(1) * (sqrt(r10 * r10 - extent[0] * extent[0] - extent[2] * extent[2]) - extent[1]);
    bv.spheres[3].o = bv.spheres[0].o - delta;
    bv.spheres[4].o = bv.spheres[0].o + delta;

    Scalar r21 = 0, r22 = 0;
    r21 = maximumDistance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, bv.spheres[3].o);
    r22 = maximumDistance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, bv.spheres[4].o);

    bv.spheres[3].o += bv.obb.axis.col(1) * (-r10 + r21);
    bv.spheres[4].o += bv.obb.axis.col(1) * (r10 - r22);

    bv.spheres[3].r = r10;
    bv.spheres[4].r = r10;
  }

  return bv;
}

//==============================================================================
template <typename Scalar>
void BVFitter<kIOS<Scalar>>::clear()
{
  vertices = NULL;
  prev_vertices = NULL;
  tri_indices = NULL;
  type = BVH_MODEL_UNKNOWN;
}

//==============================================================================
template <typename Scalar>
void BVFitter<OBBRSS<Scalar>>::set(
    Vector3<Scalar>* vertices_, Triangle* tri_indices_, BVHModelType type_)
{
  vertices = vertices_;
  prev_vertices = NULL;
  tri_indices = tri_indices_;
  type = type_;
}

//==============================================================================
template <typename Scalar>
void BVFitter<OBBRSS<Scalar>>::set(
    Vector3<Scalar>* vertices_,
    Vector3<Scalar>* prev_vertices_,
    Triangle* tri_indices_,
    BVHModelType type_)
{
  vertices = vertices_;
  prev_vertices = prev_vertices_;
  tri_indices = tri_indices_;
  type = type_;
}

//==============================================================================
template <typename Scalar>
OBBRSS<Scalar> BVFitter<OBBRSS<Scalar>>::fit(
    unsigned int* primitive_indices, int num_primitives)
{
  OBBRSS<Scalar> bv;
  Matrix3<Scalar> M;
  Matrix3<Scalar> E;
  Vector3<Scalar> s;

  getCovariance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, M);
  eigen(M, s, E);

  axisFromEigen(E, s, bv.obb.axis);
  bv.rss.axis = bv.obb.axis;

  getExtentAndCenter(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, bv.obb.axis, bv.obb.To, bv.obb.extent);

  Vector3<Scalar> origin;
  Scalar l[2];
  Scalar r;
  getRadiusAndOriginAndRectangleSize(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, bv.rss.axis, origin, l, r);

  bv.rss.Tr = origin;
  bv.rss.l[0] = l[0];
  bv.rss.l[1] = l[1];
  bv.rss.r = r;

  return bv;
}

//==============================================================================
template <typename Scalar>
void BVFitter<OBBRSS<Scalar>>::clear()
{
  vertices = NULL;
  prev_vertices = NULL;
  tri_indices = NULL;
  type = BVH_MODEL_UNKNOWN;
}

} // namespace fcl

#endif

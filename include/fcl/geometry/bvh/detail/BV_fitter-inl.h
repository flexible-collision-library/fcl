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

#ifndef FCL_BV_FITTER_INL_H
#define FCL_BV_FITTER_INL_H

#include "fcl/geometry/bvh/detail/BV_fitter.h"

namespace fcl
{

namespace detail
{

//==============================================================================
template <typename BV>
BVFitter<BV>::~BVFitter()
{
  // Do nothing
}

//==============================================================================
template <typename S, typename BV>
struct SetImpl;

//==============================================================================
template <typename BV>
void BVFitter<BV>::set(
    Vector3<typename BVFitter<BV>::S>* vertices_,
    Triangle* tri_indices_,
    BVHModelType type_)
{
  SetImpl<typename BV::S, BV>::run(*this, vertices_, tri_indices_, type_);
}

//==============================================================================
template <typename BV>
void BVFitter<BV>::set(
    Vector3<typename BVFitter<BV>::S>* vertices_,
    Vector3<typename BVFitter<BV>::S>* prev_vertices_,
    Triangle* tri_indices_,
    BVHModelType type_)
{
  SetImpl<typename BV::S, BV>::run(
        *this, vertices_, prev_vertices_, tri_indices_, type_);
}

//==============================================================================
template <typename S, typename BV>
struct FitImpl;

//==============================================================================
template <typename BV>
BV BVFitter<BV>::fit(unsigned int* primitive_indices, int num_primitives)
{
  return FitImpl<typename BV::S, BV>::run(
        *this, primitive_indices, num_primitives);
}

//==============================================================================
template <typename BV>
void BVFitter<BV>::clear()
{
  vertices = nullptr;
  prev_vertices = nullptr;
  tri_indices = nullptr;
  type = BVH_MODEL_UNKNOWN;
}

//==============================================================================
template <typename S, typename BV>
struct SetImpl
{
  static void run(
      BVFitter<BV>& fitter,
      Vector3<S>* vertices_,
      Triangle* tri_indices_,
      BVHModelType type_)
  {
    fitter.vertices = vertices_;
    fitter.prev_vertices = nullptr;
    fitter.tri_indices = tri_indices_;
    fitter.type = type_;
  }

  static void run(
      BVFitter<BV>& fitter,
      Vector3<S>* vertices_,
      Vector3<S>* prev_vertices_,
      Triangle* tri_indices_,
      BVHModelType type_)
  {
    fitter.vertices = vertices_;
    fitter.prev_vertices = prev_vertices_;
    fitter.tri_indices = tri_indices_;
    fitter.type = type_;
  }
};

//==============================================================================
template <typename S>
struct SetImpl<S, OBB<S>>
{
  static void run(
      BVFitter<OBB<S>>& fitter,
      Vector3<S>* vertices_,
      Triangle* tri_indices_,
      BVHModelType type_)
  {
    fitter.vertices = vertices_;
    fitter.prev_vertices = nullptr;
    fitter.tri_indices = tri_indices_;
    fitter.type = type_;
  }

  static void run(
      BVFitter<OBB<S>>& fitter,
      Vector3<S>* vertices_,
      Vector3<S>* prev_vertices_,
      Triangle* tri_indices_,
      BVHModelType type_)
  {
    fitter.vertices = vertices_;
    fitter.prev_vertices = prev_vertices_;
    fitter.tri_indices = tri_indices_;
    fitter.type = type_;
  }
};

//==============================================================================
template <typename S>
struct SetImpl<S, RSS<S>>
{
  static void run(
      BVFitter<RSS<S>>& fitter,
      Vector3<S>* vertices_,
      Triangle* tri_indices_,
      BVHModelType type_)
  {
    fitter.vertices = vertices_;
    fitter.prev_vertices = nullptr;
    fitter.tri_indices = tri_indices_;
    fitter.type = type_;
  }

  static void run(
      BVFitter<RSS<S>>& fitter,
      Vector3<S>* vertices_,
      Vector3<S>* prev_vertices_,
      Triangle* tri_indices_,
      BVHModelType type_)
  {
    fitter.vertices = vertices_;
    fitter.prev_vertices = prev_vertices_;
    fitter.tri_indices = tri_indices_;
    fitter.type = type_;
  }
};

//==============================================================================
template <typename S>
struct SetImpl<S, kIOS<S>>
{
  static void run(
      BVFitter<kIOS<S>>& fitter,
      Vector3<S>* vertices_,
      Triangle* tri_indices_,
      BVHModelType type_)
  {
    fitter.vertices = vertices_;
    fitter.prev_vertices = nullptr;
    fitter.tri_indices = tri_indices_;
    fitter.type = type_;
  }

  static void run(
      BVFitter<kIOS<S>>& fitter,
      Vector3<S>* vertices_,
      Vector3<S>* prev_vertices_,
      Triangle* tri_indices_,
      BVHModelType type_)
  {
    fitter.vertices = vertices_;
    fitter.prev_vertices = prev_vertices_;
    fitter.tri_indices = tri_indices_;
    fitter.type = type_;
  }
};

//==============================================================================
template <typename S>
struct SetImpl<S, OBBRSS<S>>
{
  static void run(
      BVFitter<OBBRSS<S>>& fitter,
      Vector3<S>* vertices_,
      Triangle* tri_indices_,
      BVHModelType type_)
  {
    fitter.vertices = vertices_;
    fitter.prev_vertices = nullptr;
    fitter.tri_indices = tri_indices_;
    fitter.type = type_;
  }

  static void run(
      BVFitter<OBBRSS<S>>& fitter,
      Vector3<S>* vertices_,
      Vector3<S>* prev_vertices_,
      Triangle* tri_indices_,
      BVHModelType type_)
  {
    fitter.vertices = vertices_;
    fitter.prev_vertices = prev_vertices_;
    fitter.tri_indices = tri_indices_;
    fitter.type = type_;
  }
};

//==============================================================================
template <typename S, typename BV>
struct FitImpl
{
  static BV run(
      const BVFitter<BV>& fitter,
      unsigned int* primitive_indices,
      int num_primitives)
  {
    BV bv;

    if(fitter.type == BVH_MODEL_TRIANGLES)             /// The primitive is triangle
    {
      for(int i = 0; i < num_primitives; ++i)
      {
        Triangle t = fitter.tri_indices[primitive_indices[i]];
        bv += fitter.vertices[t[0]];
        bv += fitter.vertices[t[1]];
        bv += fitter.vertices[t[2]];

        if(fitter.prev_vertices)                      /// can fitting both current and previous frame
        {
          bv += fitter.prev_vertices[t[0]];
          bv += fitter.prev_vertices[t[1]];
          bv += fitter.prev_vertices[t[2]];
        }
      }
    }
    else if(fitter.type == BVH_MODEL_POINTCLOUD)       /// The primitive is point
    {
      for(int i = 0; i < num_primitives; ++i)
      {
        bv += fitter.vertices[primitive_indices[i]];

        if(fitter.prev_vertices)                       /// can fitting both current and previous frame
        {
          bv += fitter.prev_vertices[primitive_indices[i]];
        }
      }
    }

    return bv;
  }
};

//==============================================================================
template <typename S>
struct FitImpl<S, OBB<S>>
{
  static OBB<S> run(
      const BVFitter<OBB<S>>& fitter,
      unsigned int* primitive_indices,
      int num_primitives)
  {
    OBB<S> bv;

    Matrix3<S> M; // row first matrix
    Matrix3<S> E; // row first eigen-vectors
    Vector3<S> s; // three eigen values
    getCovariance(
          fitter.vertices, fitter.prev_vertices, fitter.tri_indices,
          primitive_indices, num_primitives, M);
    eigen_old(M, s, E);
    axisFromEigen(E, s, bv.axis);

    // set obb centers and extensions
    getExtentAndCenter(
          fitter.vertices, fitter.prev_vertices, fitter.tri_indices,
          primitive_indices, num_primitives,
          bv.axis, bv.To, bv.extent);

    return bv;
  }
};

//==============================================================================
template <typename S>
struct FitImpl<S, RSS<S>>
{
  static RSS<S> run(
      const BVFitter<RSS<S>>& fitter,
      unsigned int* primitive_indices,
      int num_primitives)
  {
    RSS<S> bv;

    Matrix3<S> M; // row first matrix
    Matrix3<S> E; // row first eigen-vectors
    Vector3<S> s; // three eigen values
    getCovariance(
          fitter.vertices, fitter.prev_vertices, fitter.tri_indices,
          primitive_indices, num_primitives, M);
    eigen_old(M, s, E);
    axisFromEigen(E, s, bv.axis);

    // set rss origin, rectangle size and radius
    getRadiusAndOriginAndRectangleSize(
          fitter.vertices, fitter.prev_vertices, fitter.tri_indices,
          primitive_indices, num_primitives, bv.axis, bv.To, bv.l, bv.r);

    return bv;
  }
};

//==============================================================================
template <typename S>
struct FitImpl<S, kIOS<S>>
{
  static kIOS<S> run(
      const BVFitter<kIOS<S>>& fitter,
      unsigned int* primitive_indices,
      int num_primitives)
  {
    kIOS<S> bv;

    Matrix3<S> M; // row first matrix
    Matrix3<S> E; // row first eigen-vectors
    Vector3<S> s;
    getCovariance(
          fitter.vertices, fitter.prev_vertices, fitter.tri_indices,
          primitive_indices, num_primitives, M);
    eigen_old(M, s, E);
    axisFromEigen(E, s, bv.obb.axis);

    // get centers and extensions
    getExtentAndCenter(
          fitter.vertices, fitter.prev_vertices, fitter.tri_indices,
          primitive_indices, num_primitives, bv.obb.axis, bv.obb.To, bv.obb.extent);

    const Vector3<S>& center = bv.obb.To;
    const Vector3<S>& extent = bv.obb.extent;
    S r0 = maximumDistance(
          fitter.vertices, fitter.prev_vertices, fitter.tri_indices,
          primitive_indices, num_primitives, center);

    // decide k in kIOS
    if(extent[0] > kIOS<S>::ratio() * extent[2])
    {
      if(extent[0] > kIOS<S>::ratio() * extent[1]) bv.num_spheres = 5;
      else bv.num_spheres = 3;
    }
    else bv.num_spheres = 1;

    bv.spheres[0].o = center;
    bv.spheres[0].r = r0;

    if(bv.num_spheres >= 3)
    {
      S r10 = sqrt(r0 * r0 - extent[2] * extent[2]) * kIOS<S>::invSinA();
      Vector3<S> delta = bv.obb.axis.col(2) * (r10 * kIOS<S>::cosA() - extent[2]);
      bv.spheres[1].o = center - delta;
      bv.spheres[2].o = center + delta;

      S r11 = maximumDistance(
            fitter.vertices, fitter.prev_vertices, fitter.tri_indices,
            primitive_indices, num_primitives, bv.spheres[1].o);
      S r12 = maximumDistance(
            fitter.vertices, fitter.prev_vertices, fitter.tri_indices,
            primitive_indices, num_primitives, bv.spheres[2].o);

      bv.spheres[1].o.noalias() += bv.obb.axis.col(2) * (-r10 + r11);
      bv.spheres[2].o.noalias() += bv.obb.axis.col(2) * (r10 - r12);

      bv.spheres[1].r = r10;
      bv.spheres[2].r = r10;
    }

    if(bv.num_spheres >= 5)
    {
      S r10 = bv.spheres[1].r;
      Vector3<S> delta = bv.obb.axis.col(1) * (sqrt(r10 * r10 - extent[0] * extent[0] - extent[2] * extent[2]) - extent[1]);
      bv.spheres[3].o = bv.spheres[0].o - delta;
      bv.spheres[4].o = bv.spheres[0].o + delta;

      S r21 = 0, r22 = 0;
      r21 = maximumDistance(
            fitter.vertices, fitter.prev_vertices, fitter.tri_indices,
            primitive_indices, num_primitives, bv.spheres[3].o);
      r22 = maximumDistance(
            fitter.vertices, fitter.prev_vertices, fitter.tri_indices,
            primitive_indices, num_primitives, bv.spheres[4].o);

      bv.spheres[3].o.noalias() += bv.obb.axis.col(1) * (-r10 + r21);
      bv.spheres[4].o.noalias() += bv.obb.axis.col(1) * (r10 - r22);

      bv.spheres[3].r = r10;
      bv.spheres[4].r = r10;
    }

    return bv;
  }
};

//==============================================================================
template <typename S>
struct FitImpl<S, OBBRSS<S>>
{
  static OBBRSS<S> run(
      const BVFitter<OBBRSS<S>>& fitter,
      unsigned int* primitive_indices,
      int num_primitives)
  {
    OBBRSS<S> bv;
    Matrix3<S> M;
    Matrix3<S> E;
    Vector3<S> s;
    getCovariance(
          fitter.vertices, fitter.prev_vertices, fitter.tri_indices,
          primitive_indices, num_primitives, M);
    eigen_old(M, s, E);
    axisFromEigen(E, s, bv.obb.axis);
    bv.rss.axis = bv.obb.axis;

    getExtentAndCenter(
          fitter.vertices, fitter.prev_vertices, fitter.tri_indices,
          primitive_indices, num_primitives, bv.obb.axis, bv.obb.To, bv.obb.extent);

    getRadiusAndOriginAndRectangleSize(
          fitter.vertices, fitter.prev_vertices, fitter.tri_indices,
          primitive_indices, num_primitives,
          bv.rss.axis, bv.rss.To, bv.rss.l, bv.rss.r);

    return bv;
  }
};

} // namespace detail
} // namespace fcl

#endif

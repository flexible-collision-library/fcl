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

#include "fcl/BVH/BV_fitter.h"
#include "fcl/BVH/BVH_utility.h"
#include <limits>

namespace fcl
{

OBBd BVFitter<OBBd>::fit(unsigned int* primitive_indices, int num_primitives)
{
  OBBd bv;

  Matrix3d M; // row first matrix
  Matrix3d E; // row first eigen-vectors
  Vector3d s; // three eigen values

  getCovariance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, M);
  eigen(M, s, E);

  axisFromEigen(E, s, bv.axis);

  // set obb centers and extensions
  getExtentAndCenter(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, bv.axis, bv.To, bv.extent);

  return bv;
}

OBBRSS BVFitter<OBBRSS>::fit(unsigned int* primitive_indices, int num_primitives)
{
  OBBRSS bv;
  Matrix3d M;
  Matrix3d E;
  Vector3d s;

  getCovariance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, M);
  eigen(M, s, E);

  axisFromEigen(E, s, bv.obb.axis);
  bv.rss.axis = bv.obb.axis;

  getExtentAndCenter(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, bv.obb.axis, bv.obb.To, bv.obb.extent);

  Vector3d origin;
  FCL_REAL l[2];
  FCL_REAL r;
  getRadiusAndOriginAndRectangleSize(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, bv.rss.axis, origin, l, r);

  bv.rss.Tr = origin;
  bv.rss.l[0] = l[0];
  bv.rss.l[1] = l[1];
  bv.rss.r = r;

  return bv;
}

RSS BVFitter<RSS>::fit(unsigned int* primitive_indices, int num_primitives)
{
  RSS bv;

  Matrix3d M; // row first matrix
  Matrix3d E; // row first eigen-vectors
  Vector3d s; // three eigen values
  getCovariance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, M);
  eigen(M, s, E);
  axisFromEigen(E, s, bv.axis);

  // set rss origin, rectangle size and radius

  Vector3d origin;
  FCL_REAL l[2];
  FCL_REAL r;
  getRadiusAndOriginAndRectangleSize(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, bv.axis, origin, l, r);

  bv.Tr = origin;
  bv.l[0] = l[0];
  bv.l[1] = l[1];
  bv.r = r;


  return bv;
}


kIOS BVFitter<kIOS>::fit(unsigned int* primitive_indices, int num_primitives)
{
  kIOS bv;

  Matrix3d M; // row first matrix
  Matrix3d E; // row first eigen-vectors
  Vector3d s;
  
  getCovariance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, M);
  eigen(M, s, E);

  axisFromEigen(E, s, bv.obb.axis);

  // get centers and extensions
  getExtentAndCenter(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, bv.obb.axis, bv.obb.To, bv.obb.extent);

  const Vector3d& center = bv.obb.To;
  const Vector3d& extent = bv.obb.extent;
  FCL_REAL r0 = maximumDistance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, center);

  // decide k in kIOS
  if(extent[0] > kIOS::ratio() * extent[2])
  {
    if(extent[0] > kIOS::ratio() * extent[1]) bv.num_spheres = 5;
    else bv.num_spheres = 3;
  }
  else bv.num_spheres = 1;

  bv.spheres[0].o = center;
  bv.spheres[0].r = r0;

  if(bv.num_spheres >= 3)
  {
    FCL_REAL r10 = sqrt(r0 * r0 - extent[2] * extent[2]) * kIOS::invSinA();
    Vector3d delta = bv.obb.axis.col(2) * (r10 * kIOS::cosA() - extent[2]);
    bv.spheres[1].o = center - delta;
    bv.spheres[2].o = center + delta;

    FCL_REAL r11 = maximumDistance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, bv.spheres[1].o);
    FCL_REAL r12 = maximumDistance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, bv.spheres[2].o);

    bv.spheres[1].o += bv.obb.axis.col(2) * (-r10 + r11);
    bv.spheres[2].o += bv.obb.axis.col(2) * (r10 - r12);

    bv.spheres[1].r = r10;
    bv.spheres[2].r = r10;
  }

  if(bv.num_spheres >= 5)
  {
    FCL_REAL r10 = bv.spheres[1].r;
    Vector3d delta = bv.obb.axis.col(1) * (sqrt(r10 * r10 - extent[0] * extent[0] - extent[2] * extent[2]) - extent[1]);
    bv.spheres[3].o = bv.spheres[0].o - delta;
    bv.spheres[4].o = bv.spheres[0].o + delta;
    
    FCL_REAL r21 = 0, r22 = 0;
    r21 = maximumDistance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, bv.spheres[3].o);
    r22 = maximumDistance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, bv.spheres[4].o);

    bv.spheres[3].o += bv.obb.axis.col(1) * (-r10 + r21);
    bv.spheres[4].o += bv.obb.axis.col(1) * (r10 - r22);
    
    bv.spheres[3].r = r10;
    bv.spheres[4].r = r10;
  }

  return bv;
}


}

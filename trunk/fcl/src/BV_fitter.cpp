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

#include "fcl/BV_fitter.h"
#include "fcl/BVH_utility.h"
#include <limits>
#include <boost/math/constants/constants.hpp>

namespace fcl
{


namespace OBB_fit_functions
{

void fit1(Vec3f* ps, OBB& bv)
{
  bv.To = ps[0];
  bv.axis[0].setValue(1, 0, 0);
  bv.axis[1].setValue(0, 1, 0);
  bv.axis[2].setValue(0, 0, 1);
  bv.extent.setValue(0);
}

void fit2(Vec3f* ps, OBB& bv)
{
  const Vec3f& p1 = ps[0];
  const Vec3f& p2 = ps[1];
  Vec3f p1p2 = p1 - p2;
  BVH_REAL len_p1p2 = p1p2.length();
  p1p2.normalize();

  bv.axis[0] = p1p2;
  generateCoordinateSystem(bv.axis[0], bv.axis[1], bv.axis[2]);

  bv.extent.setValue(len_p1p2 * 0.5, 0, 0);
  bv.To.setValue(0.5 * (p1[0] + p2[0]),
                 0.5 * (p1[1] + p2[1]),
                 0.5 * (p1[2] + p2[2]));

}

void fit3(Vec3f* ps, OBB& bv)
{
  const Vec3f& p1 = ps[0];
  const Vec3f& p2 = ps[1];
  const Vec3f& p3 = ps[2];
  Vec3f e[3];
  e[0] = p1 - p2;
  e[1] = p2 - p3;
  e[2] = p3 - p1;
  BVH_REAL len[3];
  len[0] = e[0].sqrLength();
  len[1] = e[1].sqrLength();
  len[2] = e[2].sqrLength();

  int imax = 0;
  if(len[1] > len[0]) imax = 1;
  if(len[2] > len[imax]) imax = 2;

  Vec3f& u = bv.axis[0];
  Vec3f& v = bv.axis[1];
  Vec3f& w = bv.axis[2];

  w = e[0].cross(e[1]);
  w.normalize();
  u = e[imax];
  u.normalize();
  v = w.cross(u);

  getExtentAndCenter(ps, NULL, NULL, 3, bv.axis, bv.To, bv.extent);
}

void fit6(Vec3f* ps, OBB& bv)
{
  OBB bv1, bv2;
  fit3(ps, bv1);
  fit3(ps + 3, bv2);
  bv = bv1 + bv2;
}


void fitn(Vec3f* ps, int n, OBB& bv)
{
  Matrix3f M;
  Vec3f E[3];
  BVH_REAL s[3] = {0, 0, 0}; // three eigen values

  getCovariance(ps, NULL, NULL, n, M);
  matEigen(M, s, E);

  int min, mid, max;
  if(s[0] > s[1]) { max = 0; min = 1; }
  else { min = 0; max = 1; }
  if(s[2] < s[min]) { mid = min; min = 2; }
  else if(s[2] > s[max]) { mid = max; max = 2; }
  else { mid = 2; }

  bv.axis[0].setValue(E[0][max], E[1][max], E[2][max]);
  bv.axis[1].setValue(E[0][mid], E[1][mid], E[2][mid]);
  bv.axis[2].setValue(E[1][max]*E[2][mid] - E[1][mid]*E[2][max],
                      E[0][mid]*E[2][max] - E[0][max]*E[2][mid],
                      E[0][max]*E[1][mid] - E[0][mid]*E[1][max]);

  // set obb centers and extensions
  Vec3f center, extent;
  getExtentAndCenter(ps, NULL, NULL, n, bv.axis, center, extent);

  bv.To = center;
  bv.extent = extent;
}

}


namespace RSS_fit_functions
{
void fit1(Vec3f* ps, RSS& bv)
{
  bv.Tr = ps[0];
  bv.axis[0].setValue(1, 0, 0);
  bv.axis[1].setValue(0, 1, 0);
  bv.axis[2].setValue(0, 0, 1);
  bv.l[0] = 0;
  bv.l[1] = 0;
  bv.r = 0;
}

void fit2(Vec3f* ps, RSS& bv)
{
  const Vec3f& p1 = ps[0];
  const Vec3f& p2 = ps[1];
  Vec3f p1p2 = p1 - p2;
  BVH_REAL len_p1p2 = p1p2.length();
  p1p2.normalize();

  bv.axis[0] = p1p2;
  generateCoordinateSystem(bv.axis[0], bv.axis[1], bv.axis[2]);
  bv.l[0] = len_p1p2;
  bv.l[1] = 0;

  bv.Tr = p2;
  bv.r = 0;
}

void fit3(Vec3f* ps, RSS& bv)
{
  const Vec3f& p1 = ps[0];
  const Vec3f& p2 = ps[1];
  const Vec3f& p3 = ps[2];
  Vec3f e[3];
  e[0] = p1 - p2;
  e[1] = p2 - p3;
  e[2] = p3 - p1;
  BVH_REAL len[3];
  len[0] = e[0].sqrLength();
  len[1] = e[1].sqrLength();
  len[2] = e[2].sqrLength();

  int imax = 0;
  if(len[1] > len[0]) imax = 1;
  if(len[2] > len[imax]) imax = 2;

  Vec3f& u = bv.axis[0];
  Vec3f& v = bv.axis[1];
  Vec3f& w = bv.axis[2];

  w = e[0].cross(e[1]);
  w.normalize();
  u = e[imax];
  u.normalize();
  v = w.cross(u);

  getRadiusAndOriginAndRectangleSize(ps, NULL, NULL, 3, bv.axis, bv.Tr, bv.l, bv.r);
}

void fit6(Vec3f* ps, RSS& bv)
{
  RSS bv1, bv2;
  fit3(ps, bv1);
  fit3(ps + 3, bv2);
  bv = bv1 + bv2;
}

void fitn(Vec3f* ps, int n, RSS& bv)
{
  Matrix3f M; // row first matrix
  Vec3f E[3]; // row first eigen-vectors
  BVH_REAL s[3] = {0, 0, 0};

  getCovariance(ps, NULL, NULL, n, M);
  matEigen(M, s, E);

  int min, mid, max;
  if(s[0] > s[1]) { max = 0; min = 1; }
  else { min = 0; max = 1; }
  if(s[2] < s[min]) { mid = min; min = 2; }
  else if(s[2] > s[max]) { mid = max; max = 2; }
  else { mid = 2; }

  bv.axis[0].setValue(E[0][max], E[1][max], E[2][max]);
  bv.axis[1].setValue(E[0][mid], E[1][mid], E[2][mid]);
  bv.axis[2].setValue(E[1][max]*E[2][mid] - E[1][mid]*E[2][max],
                      E[0][mid]*E[2][max] - E[0][max]*E[2][mid],
                      E[0][max]*E[1][mid] - E[0][mid]*E[1][max]);

  // set rss origin, rectangle size and radius
  getRadiusAndOriginAndRectangleSize(ps, NULL, NULL, n, bv.axis, bv.Tr, bv.l, bv.r);
}

}

namespace kIOS_fit_functions
{

void fit1(Vec3f* ps, kIOS& bv)
{
  bv.num_spheres = 1;
  bv.spheres[0].o = ps[0];
  bv.spheres[0].r = 0;
}

void fit2(Vec3f* ps, kIOS& bv)
{
  bv.num_spheres = 5;

  const Vec3f& p1 = ps[0];
  const Vec3f& p2 = ps[1];
  Vec3f p1p2 = p1 - p2;
  BVH_REAL len_p1p2 = p1p2.length();
  p1p2.normalize();
    
  Vec3f axis[3];
  axis[0] = p1p2;
  generateCoordinateSystem(axis[0], axis[1], axis[2]);
    
  BVH_REAL r0 = len_p1p2 / 2.0;
  BVH_REAL r0cos30 = r0 * cos(boost::math::constants::pi<BVH_REAL>() / 6);
  bv.spheres[0].o = (p1 + p2) * 0.5;
  bv.spheres[0].r = r0;

  BVH_REAL twor0 = 2 * r0;

  bv.spheres[1].r = twor0;
  bv.spheres[2].r = twor0;
  Vec3f delta = axis[1] * r0cos30;
  bv.spheres[1].o = bv.spheres[0].o - delta;
  bv.spheres[2].o = bv.spheres[0].o + delta;

  bv.spheres[3].r = twor0;
  bv.spheres[4].r = twor0;
  delta = axis[2] * r0cos30;
  bv.spheres[3].o = bv.spheres[0].o - delta;
  bv.spheres[4].o = bv.spheres[0].o + delta;
}

void fit3(Vec3f* ps, kIOS& bv)
{
  bv.num_spheres = 3;
    
  const Vec3f& p1 = ps[0];
  const Vec3f& p2 = ps[1];
  const Vec3f& p3 = ps[2];
  Vec3f e[3];
  e[0] = p1 - p2;
  e[1] = p2 - p3;
  e[2] = p3 - p1;
  BVH_REAL len[3];
  len[0] = e[0].sqrLength();
  len[1] = e[1].sqrLength();
  len[2] = e[2].sqrLength();
    
  int imax = 0;
  if(len[1] > len[0]) imax = 1;
  if(len[2] > len[1]) imax = 2;
    
  Vec3f axis[3];
    
  axis[2] = e[0].cross(e[1]);
  axis[2].normalize();
  axis[0] = e[imax];
  axis[0].normalize();
  axis[1] = axis[2].cross(axis[0]);


  // compute radius and center
  BVH_REAL r0;
  Vec3f center;
  circumCircleComputation(p1, p2, p3, center, r0);

  bv.spheres[0].o = center;
  bv.spheres[0].r = r0;

  BVH_REAL twor0 = 2 * r0;
  Vec3f delta = axis[2] * r0 * cos(boost::math::constants::pi<BVH_REAL>() / 6);
  
  bv.spheres[1].r = twor0;
  bv.spheres[1].o = center - delta;
  bv.spheres[2].r = twor0;
  bv.spheres[2].o = center + delta;
}

void fit6(Vec3f* ps, kIOS& bv)
{
  kIOS bv1, bv2;
  fit3(ps, bv1);
  fit3(ps + 3, bv2);
  bv = bv1 + bv2;
}

void fitn(Vec3f* ps, int n, kIOS& bv)
{
  Matrix3f M;
  Vec3f E[3];
  BVH_REAL s[3] = {0, 0, 0}; // three eigen values;

  getCovariance(ps, NULL, NULL, n, M);
  matEigen(M, s, E);
  
  int min, mid, max;
  if(s[0] > s[1]) { max = 0; min = 1; }
  else { min = 0; max = 1; }
  if(s[2] < s[min]) { mid = min; min = 2; }
  else if(s[2] > s[max]) { mid = max; max = 2; }
  else { mid = 2; }
  
  Vec3f axis[3];
  axis[0].setValue(E[0][max], E[1][max], E[2][max]);
  axis[1].setValue(E[0][mid], E[1][mid], E[2][mid]);
  axis[2].setValue(E[1][max]*E[2][mid] - E[1][mid]*E[2][max],
                   E[0][mid]*E[2][max] - E[0][max]*E[2][mid],
                   E[0][max]*E[1][mid] - E[0][mid]*E[1][max]);

  // get center and extension
  Vec3f center, extent;
  getExtentAndCenter(ps, NULL, NULL, n, axis, center, extent);

  BVH_REAL r0 = maximumDistance(ps, NULL, NULL, n, center);
  
  // decide the k in kIOS
  if(extent[0] > 1.5 * extent[2])
  {
    if(extent[0] > 1.5 * extent[1]) bv.num_spheres = 5;
    else bv.num_spheres = 3;
  }
  else bv.num_spheres = 1;

  if(bv.num_spheres >= 1)
  {
    bv.spheres[0].o = center;
    bv.spheres[0].r = r0;
  }
  
  BVH_REAL cos30 = cos(boost::math::constants::pi<BVH_REAL>() / 6);

  if(bv.num_spheres >= 3)
  {
    BVH_REAL r10 = sqrt(r0 * r0 - extent[2] * extent[2]) * 2;
    Vec3f delta = axis[2] * (r10 * cos30 - extent[2]);
    bv.spheres[1].o = center - delta;
    bv.spheres[2].o = center + delta;
   
    BVH_REAL r11 = 0, r12 = 0;
    r11 = maximumDistance(ps, NULL, NULL, n, bv.spheres[1].o);
    r12 = maximumDistance(ps, NULL, NULL, n, bv.spheres[2].o);
    bv.spheres[1].o += axis[2] * (-r10 + r11);
    bv.spheres[2].o += axis[2] * (r10 - r12);

    bv.spheres[1].r = r10;
    bv.spheres[2].r = r10;
  }

  if(bv.num_spheres >= 5)
  {
    BVH_REAL r20 = sqrt(r0 * r0 - extent[1] * extent[1]) * 2;
    Vec3f delta = axis[1] * (r20 * cos30 - extent[1]);
    bv.spheres[3].o = center - delta;
    bv.spheres[4].o = center + delta;
    
    BVH_REAL r21 = 0, r22 = 0;
    r21 = maximumDistance(ps, NULL, NULL, n, bv.spheres[3].o);
    r22 = maximumDistance(ps, NULL, NULL, n, bv.spheres[4].o);
    bv.spheres[3].o += axis[3] * (-r20 + r21);
    bv.spheres[4].o += axis[3] * (r20 - r22);

    bv.spheres[3].r = r20;
    bv.spheres[4].r = r20;
  }
}

}



template<>
void fit(Vec3f* ps, int n, OBB& bv)
{
  switch(n)
  {
    case 1:
      OBB_fit_functions::fit1(ps, bv);
      break;
    case 2:
      OBB_fit_functions::fit2(ps, bv);
      break;
    case 3:
      OBB_fit_functions::fit3(ps, bv);
      break;
    case 6:
      OBB_fit_functions::fit6(ps, bv);
      break;
    default:
      OBB_fit_functions::fitn(ps, n, bv);
  }
}


template<>
void fit(Vec3f* ps, int n, RSS& bv)
{
  switch(n)
  {
    case 1:
      RSS_fit_functions::fit1(ps, bv);
      break;
    case 2:
      RSS_fit_functions::fit2(ps, bv);
      break;
    case 3:
      RSS_fit_functions::fit3(ps, bv);
      break;
    default:
      RSS_fit_functions::fitn(ps, n, bv);
  }
}

template<>
void fit(Vec3f* ps, int n, kIOS& bv)
{
  switch(n)
  {
  case 1:
    kIOS_fit_functions::fit1(ps, bv);
    break;
  case 2:
    kIOS_fit_functions::fit2(ps, bv);
    break;
  case 3:
    kIOS_fit_functions::fit3(ps, bv);
    break;
  case 6:
    kIOS_fit_functions::fit6(ps, bv);
    break;
  default:
    kIOS_fit_functions::fitn(ps, n, bv);
  }
}


OBB BVFitter<OBB>::fit(unsigned int* primitive_indices, int num_primitives)
{
  OBB bv;

  Matrix3f M; // row first matrix
  Vec3f E[3]; // row first eigen-vectors
  BVH_REAL s[3]; // three eigen values

  if(type == BVH_MODEL_TRIANGLES)
  {
    getCovariance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, M);
    matEigen(M, s, E);
  }
  else if(type == BVH_MODEL_POINTCLOUD)
  {
    getCovariance(vertices, prev_vertices, primitive_indices, num_primitives, M);
    matEigen(M, s, E);
  }

  int min, mid, max;
  if(s[0] > s[1]) { max = 0; min = 1; }
  else { min = 0; max = 1; }
  if(s[2] < s[min]) { mid = min; min = 2; }
  else if(s[2] > s[max]) { mid = max; max = 2; }
  else { mid = 2; }

  bv.axis[0].setValue(E[0][max], E[1][max], E[2][max]);
  bv.axis[1].setValue(E[0][mid], E[1][mid], E[2][mid]);
  bv.axis[2].setValue(E[1][max]*E[2][mid] - E[1][mid]*E[2][max],
                      E[0][mid]*E[2][max] - E[0][max]*E[2][mid],
                      E[0][max]*E[1][mid] - E[0][mid]*E[1][max]);

  // set obb centers and extensions
  Vec3f center, extent;
  if(type == BVH_MODEL_TRIANGLES)
  {
    getExtentAndCenter(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, bv.axis, center, extent);
  }
  else if(type == BVH_MODEL_POINTCLOUD)
  {
    getExtentAndCenter(vertices, prev_vertices, primitive_indices, num_primitives, bv.axis, center, extent);
  }

  bv.To = center;
  bv.extent = extent;

  return bv;
}

kIOS BVFitter<kIOS>::fit(unsigned int* primitive_indices, int num_primitives)
{
  kIOS bv;

  Matrix3f M; // row first matrix
  Vec3f E[3]; // row first eigen-vectors
  BVH_REAL s[3];
  
  if(type == BVH_MODEL_TRIANGLES)
  {
    getCovariance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, M);
    matEigen(M, s, E);
  }
  else if(type == BVH_MODEL_POINTCLOUD)
  {
    getCovariance(vertices, prev_vertices, primitive_indices, num_primitives, M);
    matEigen(M, s, E);
  }

  int min, mid, max;
  if(s[0] > s[1]) { max = 0; min = 1; }
  else { min = 0; max = 1; }
  if(s[2] < s[min]) { mid = min; min = 2; }
  else if(s[2] > s[max]) { mid = max; max = 2; }
  else mid = 2;

  Vec3f axis[3];
  axis[0].setValue(E[0][max], E[1][max], E[2][max]);
  axis[1].setValue(E[0][mid], E[1][mid], E[2][mid]);
  axis[2].setValue(E[1][max]*E[2][mid] - E[1][mid]*E[2][max],
                   E[0][mid]*E[2][max] - E[0][max]*E[2][mid],
                   E[0][max]*E[1][mid] - E[0][mid]*E[1][max]);

  // get centers and extensions
  Vec3f center, extent;
  if(type == BVH_MODEL_TRIANGLES)
  {
    getExtentAndCenter(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, axis, center, extent);
  }
  else if(type == BVH_MODEL_POINTCLOUD)
  {
    getExtentAndCenter(vertices, prev_vertices, primitive_indices, num_primitives, axis, center, extent);
  }

  BVH_REAL r0;
  if(type == BVH_MODEL_TRIANGLES)
    r0 = maximumDistance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, center);
  else if(type == BVH_MODEL_POINTCLOUD) 
    r0 = maximumDistance(vertices, prev_vertices, primitive_indices, num_primitives, center);

  // decide k in kIOS
  if(extent[0] > 1.5 * extent[2])
  {
    if(extent[0] > 1.5 * extent[1]) bv.num_spheres = 5;
    else bv.num_spheres = 3;
  }
  else bv.num_spheres = 1;

  if(bv.num_spheres >= 1)
  {
    bv.spheres[0].o = center;
    bv.spheres[0].r = r0;
  }

  BVH_REAL cos30 = cos(boost::math::constants::pi<BVH_REAL>() / 6);

  if(bv.num_spheres >= 3)
  {
    BVH_REAL r10 = sqrt(r0 * r0 - extent[2] * extent[2]) * 2;
    Vec3f delta = axis[2] * (r10 * cos30 - extent[2]);
    bv.spheres[1].o = center - delta;
    bv.spheres[2].o = center + delta;

    BVH_REAL r11 = 0, r12 = 0;
    if(type == BVH_MODEL_TRIANGLES)
    {
      r11 = maximumDistance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, bv.spheres[1].o);
      r12 = maximumDistance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, bv.spheres[2].o);
    }
    else if(type == BVH_MODEL_POINTCLOUD)
    {
      r11 = maximumDistance(vertices, prev_vertices, primitive_indices, num_primitives, bv.spheres[1].o);
      r12 = maximumDistance(vertices, prev_vertices, primitive_indices, num_primitives, bv.spheres[2].o);
    }
    bv.spheres[1].o += axis[2] * (-r10 + r11);
    bv.spheres[2].o += axis[2] * (r10 - r12);

    bv.spheres[1].r = r10;
    bv.spheres[2].r = r10;
  }

  if(bv.num_spheres >= 5)
  {
    BVH_REAL r20 = sqrt(r0 * r0 - extent[1] * extent[1]) * 2;
    Vec3f delta = axis[1] * (r20 * cos30 - extent[1]);
    bv.spheres[3].o = center - delta;
    bv.spheres[4].o = center + delta;

    BVH_REAL r21 = 0, r22 = 0;
    if(type == BVH_MODEL_TRIANGLES)
    {
      r21 = maximumDistance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, bv.spheres[3].o);
      r22 = maximumDistance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, bv.spheres[4].o);
    }
    else if(type == BVH_MODEL_POINTCLOUD)
    {
      r21 = maximumDistance(vertices, prev_vertices, primitive_indices, num_primitives, bv.spheres[3].o);
      r22 = maximumDistance(vertices, prev_vertices, primitive_indices, num_primitives, bv.spheres[4].o);
    }
    bv.spheres[3].o += axis[1] * (-r20 + r21);
    bv.spheres[4].o += axis[1] * (r20 - r22);

    bv.spheres[3].r = r20;
    bv.spheres[4].r = r20;
  }

  return bv;
}


RSS BVFitter<RSS>::fit(unsigned int* primitive_indices, int num_primitives)
{
  RSS bv;

  Matrix3f M; // row first matrix
  Vec3f E[3]; // row first eigen-vectors
  BVH_REAL s[3]; // three eigen values

  if(type == BVH_MODEL_TRIANGLES)
  {
    getCovariance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, M);
    matEigen(M, s, E);
  }
  else if(type == BVH_MODEL_POINTCLOUD)
  {
    getCovariance(vertices, prev_vertices, primitive_indices, num_primitives, M);
    matEigen(M, s, E);
  }

  int min, mid, max;
  if(s[0] > s[1]) { max = 0; min = 1; }
  else { min = 0; max = 1; }
  if(s[2] < s[min]) { mid = min; min = 2; }
  else if(s[2] > s[max]) { mid = max; max = 2; }
  else { mid = 2; }

  bv.axis[0].setValue(E[0][max], E[1][max], E[2][max]);
  bv.axis[1].setValue(E[0][mid], E[1][mid], E[2][mid]);
  bv.axis[2].setValue(E[1][max]*E[2][mid] - E[1][mid]*E[2][max],
                      E[0][mid]*E[2][max] - E[0][max]*E[2][mid],
                      E[0][max]*E[1][mid] - E[0][mid]*E[1][max]);

  // set rss origin, rectangle size and radius

  Vec3f origin;
  BVH_REAL l[2];
  BVH_REAL r;

  if(type == BVH_MODEL_TRIANGLES)
  {
    getRadiusAndOriginAndRectangleSize(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, bv.axis, origin, l, r);
  }
  else if(type == BVH_MODEL_POINTCLOUD)
  {
    getRadiusAndOriginAndRectangleSize(vertices, prev_vertices, primitive_indices, num_primitives, bv.axis, origin, l, r);
  }

  bv.Tr = origin;
  bv.l[0] = l[0];
  bv.l[1] = l[1];
  bv.r = r;


  return bv;
}




}

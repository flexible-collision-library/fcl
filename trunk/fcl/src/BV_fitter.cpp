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

namespace fcl
{


namespace OBB_fit_functions
{

void fit1(Vec3f* ps, OBB& bv)
{
  bv.To = ps[0];
  bv.axis[0] = Vec3f(1, 0, 0);
  bv.axis[1] = Vec3f(0, 1, 0);
  bv.axis[2] = Vec3f(0, 0, 1);
  bv.extent = Vec3f(0, 0, 0);
}

void fit2(Vec3f* ps, OBB& bv)
{
  Vec3f p1(ps[0][0], ps[0][1], ps[0][2]);
  Vec3f p2(ps[1][0], ps[1][1], ps[1][2]);
  Vec3f p1p2 = p1 - p2;
  float len_p1p2 = p1p2.length();
  Vec3f w = p1p2;
  w.normalize();

  // then generate other two axes orthonormal to w
  Vec3f u, v;
  float inv_length;
  if(fabs(w[0]) >= fabs(w[1]))
  {
    inv_length = 1.0 / sqrt(w[0] * w[0] + w[2] * w[2]);
    u[0] = -w[2] * inv_length;
    u[1] = 0;
    u[2] = w[0] * inv_length;
    v[0] = w[1] * u[2];
    v[1] = w[2] * u[0] - w[0] * u[2];
    v[2] = -w[1] * u[0];
  }
  else
  {
    inv_length = 1.0 / sqrt(w[1] * w[1] + w[2] * w[2]);
    u[0] = 0;
    u[1] = w[2] * inv_length;
    u[2] = -w[1] * inv_length;
    v[0] = w[1] * u[2] - w[2] * u[1];
    v[1] = -w[0] * u[2];
    v[2] = w[0] * u[1];
  }

  bv.axis[0] = w;
  bv.axis[1] = u;
  bv.axis[2] = v;

  bv.extent = Vec3f(len_p1p2 * 0.5, 0, 0);
  bv.To = Vec3f(0.5 * (p1[0] + p2[0]),
                    0.5 * (p1[1] + p2[1]),
                    0.5 * (p1[2] + p2[2]));

}

void fit3(Vec3f* ps, OBB& bv)
{
  Vec3f p1(ps[0][0], ps[0][1], ps[0][2]);
  Vec3f p2(ps[1][0], ps[1][1], ps[1][2]);
  Vec3f p3(ps[2][0], ps[2][1], ps[2][2]);
  Vec3f e[3];
  e[0] = p1 - p2;
  e[1] = p2 - p3;
  e[2] = p3 - p1;
  float len[3];
  len[0] = e[0].sqrLength();
  len[1] = e[1].sqrLength();
  len[2] = e[2].sqrLength();

  int imax = 0;
  if(len[1] > len[0]) imax = 1;
  if(len[2] > len[imax]) imax = 2;

  Vec3f w = e[0].cross(e[1]);
  w.normalize();
  Vec3f u = e[imax];
  u.normalize();
  Vec3f v = w.cross(u);
  bv.axis[0] = u;
  bv.axis[1] = v;
  bv.axis[2] = w;

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
  Vec3f M[3]; // row first matrix
  Vec3f E[3]; // row first eigen-vectors
  BVH_REAL s[3] = {0, 0, 0}; // three eigen values

  getCovariance(ps, NULL, NULL, n, M);
  matEigen(M, s, E);

  int min, mid, max;
  if(s[0] > s[1]) { max = 0; min = 1; }
  else { min = 0; max = 1; }
  if(s[2] < s[min]) { mid = min; min = 2; }
  else if(s[2] > s[max]) { mid = max; max = 2; }
  else { mid = 2; }

  Vec3f R[3]; // column first matrix, as the axis in OBB
  R[0] = Vec3f(E[0][max], E[1][max], E[2][max]);
  R[1] = Vec3f(E[0][mid], E[1][mid], E[2][mid]);
  R[2] = Vec3f(E[1][max]*E[2][mid] - E[1][mid]*E[2][max],
               E[0][mid]*E[2][max] - E[0][max]*E[2][mid],
               E[0][max]*E[1][mid] - E[0][mid]*E[1][max]);


  // set obb axes
  bv.axis[0] = R[0];
  bv.axis[1] = R[1];
  bv.axis[2] = R[2];

  // set obb centers and extensions
  Vec3f center, extent;
  getExtentAndCenter(ps, NULL, NULL, n, R, center, extent);

  bv.To = center;
  bv.extent = extent;
}

}


namespace RSS_fit_functions
{
void fit1(Vec3f* ps, RSS& bv)
{
  bv.Tr = ps[0];
  bv.axis[0] = Vec3f(1, 0, 0);
  bv.axis[1] = Vec3f(0, 1, 0);
  bv.axis[2] = Vec3f(0, 0, 1);
  bv.l[0] = 0;
  bv.l[1] = 0;
  bv.r = 0;
}

void fit2(Vec3f* ps, RSS& bv)
{
  Vec3f p1(ps[0][0], ps[0][1], ps[0][2]);
  Vec3f p2(ps[1][0], ps[1][1], ps[1][2]);
  Vec3f p1p2 = p1 - p2;
  float len_p1p2 = p1p2.length();
  Vec3f w = p1p2;
  w.normalize();

  // then generate other two axes orthonormal to w
  Vec3f u, v;
  float inv_length;
  if(fabs(w[0]) >= fabs(w[1]))
  {
    inv_length = 1.0 / sqrt(w[0] * w[0] + w[2] * w[2]);
    u[0] = -w[2] * inv_length;
    u[1] = 0;
    u[2] = w[0] * inv_length;
    v[0] = w[1] * u[2];
    v[1] = w[2] * u[0] - w[0] * u[2];
    v[2] = -w[1] * u[0];
  }
  else
  {
    inv_length = 1.0 / sqrt(w[1] * w[1] + w[2] * w[2]);
    u[0] = 0;
    u[1] = w[2] * inv_length;
    u[2] = -w[1] * inv_length;
    v[0] = w[1] * u[2] - w[2] * u[1];
    v[1] = -w[0] * u[2];
    v[2] = w[0] * u[1];
  }

  bv.axis[0] = w;
  bv.axis[1] = u;
  bv.axis[2] = v;

  bv.l[0] = len_p1p2;
  bv.l[1] = 0;

  bv.Tr = p2;
  bv.r = 0;
}

void fit3(Vec3f* ps, RSS& bv)
{
  Vec3f p1(ps[0][0], ps[0][1], ps[0][2]);
  Vec3f p2(ps[1][0], ps[1][1], ps[1][2]);
  Vec3f p3(ps[2][0], ps[2][1], ps[2][2]);
  Vec3f e[3];
  e[0] = p1 - p2;
  e[1] = p2 - p3;
  e[2] = p3 - p1;
  float len[3];
  len[0] = e[0].sqrLength();
  len[1] = e[1].sqrLength();
  len[2] = e[2].sqrLength();

  int imax = 0;
  if(len[1] > len[0]) imax = 1;
  if(len[2] > len[imax]) imax = 2;

  Vec3f w = e[0].cross(e[1]);
  w.normalize();
  Vec3f u = e[imax];
  u.normalize();
  Vec3f v = w.cross(u);
  bv.axis[0] = u;
  bv.axis[1] = v;
  bv.axis[2] = w;

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
  Vec3f M[3]; // row first matrix
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

  Vec3f R[3]; // column first matrix, as the axis in RSS
  R[0] = Vec3f(E[0][max], E[1][max], E[2][max]);
  R[1] = Vec3f(E[0][mid], E[1][mid], E[2][mid]);
  R[2] = Vec3f(E[1][max]*E[2][mid] - E[1][mid]*E[2][max],
               E[0][mid]*E[2][max] - E[0][max]*E[2][mid],
               E[0][max]*E[1][mid] - E[0][mid]*E[1][max]);

  // set obb axes
  bv.axis[0] = R[0];
  bv.axis[1] = R[1];
  bv.axis[2] = R[2];

  // set rss origin, rectangle size and radius
  getRadiusAndOriginAndRectangleSize(ps, NULL, NULL, n, R, bv.Tr, bv.l, bv.r);
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


OBB BVFitter<OBB>::fit(unsigned int* primitive_indices, int num_primitives)
{
  OBB bv;

  Vec3f M[3]; // row first matrix
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

  Vec3f R[3]; // column first matrix, as the axis in OBB
  R[0] = Vec3f(E[0][max], E[1][max], E[2][max]);
  R[1] = Vec3f(E[0][mid], E[1][mid], E[2][mid]);
  R[2] = Vec3f(E[1][max]*E[2][mid] - E[1][mid]*E[2][max],
               E[0][mid]*E[2][max] - E[0][max]*E[2][mid],
               E[0][max]*E[1][mid] - E[0][mid]*E[1][max]);


  // set obb axes
  bv.axis[0] = R[0];
  bv.axis[1] = R[1];
  bv.axis[2] = R[2];

  // set obb centers and extensions
  Vec3f center, extent;
  if(type == BVH_MODEL_TRIANGLES)
  {
    getExtentAndCenter(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, R, center, extent);
  }
  else if(type == BVH_MODEL_POINTCLOUD)
  {
    getExtentAndCenter(vertices, prev_vertices, primitive_indices, num_primitives, R, center, extent);
  }

  bv.To = center;
  bv.extent = extent;

  return bv;
}


RSS BVFitter<RSS>::fit(unsigned int* primitive_indices, int num_primitives)
{
  RSS bv;

  Vec3f M[3]; // row first matrix
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

  Vec3f R[3]; // column first matrix, as the axis in OBB
  R[0] = Vec3f(E[0][max], E[1][max], E[2][max]);
  R[1] = Vec3f(E[0][mid], E[1][mid], E[2][mid]);
  R[2] = Vec3f(E[1][max]*E[2][mid] - E[1][mid]*E[2][max],
               E[0][mid]*E[2][max] - E[0][max]*E[2][mid],
               E[0][max]*E[1][mid] - E[0][mid]*E[1][max]);

  // set rss axes
  bv.axis[0] = R[0];
  bv.axis[1] = R[1];
  bv.axis[2] = R[2];

  // set rss origin, rectangle size and radius

  Vec3f origin;
  BVH_REAL l[2];
  BVH_REAL r;

  if(type == BVH_MODEL_TRIANGLES)
  {
    getRadiusAndOriginAndRectangleSize(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, R, origin, l, r);
  }
  else if(type == BVH_MODEL_POINTCLOUD)
  {
    getRadiusAndOriginAndRectangleSize(vertices, prev_vertices, primitive_indices, num_primitives, R, origin, l, r);
  }

  bv.Tr = origin;
  bv.l[0] = l[0];
  bv.l[1] = l[1];
  bv.r = r;


  return bv;
}




}

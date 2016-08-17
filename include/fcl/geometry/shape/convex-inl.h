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

#ifndef FCL_SHAPE_CONVEX_INL_H
#define FCL_SHAPE_CONVEX_INL_H

#include "fcl/geometry/shape/convex.h"

namespace fcl
{

//==============================================================================
extern template
class Convex<double>;

//==============================================================================
template <typename S>
Convex<S>::Convex(
    Vector3<S>* plane_normals, S* plane_dis, int num_planes_,
    Vector3<S>* points, int num_points_, int* polygons_)
  : ShapeBase<S>()
{
  plane_normals = plane_normals;
  plane_dis = plane_dis;
  num_planes = num_planes_;
  points = points;
  num_points = num_points_;
  polygons = polygons_;
  edges = nullptr;

  Vector3<S> sum = Vector3<S>::Zero();
  for(int i = 0; i < num_points; ++i)
  {
    sum += points[i];
  }

  center = sum * (S)(1.0 / num_points);

  fillEdges();
}

//==============================================================================
template <typename S>
Convex<S>::Convex(const Convex& other)
  : ShapeBase<S>(other)
{
  plane_normals = other.plane_normals;
  plane_dis = other.plane_dis;
  num_planes = other.num_planes;
  points = other.points;
  polygons = other.polygons;
  edges = new Edge[other.num_edges];
  memcpy(edges, other.edges, sizeof(Edge) * num_edges);
}

//==============================================================================
template <typename S>
Convex<S>::~Convex()
{
  delete [] edges;
}

//==============================================================================
template <typename S>
void Convex<S>::computeLocalAABB()
{
  this->aabb_local.min_.setConstant(-std::numeric_limits<S>::max());
  this->aabb_local.max_.setConstant(std::numeric_limits<S>::max());
  for(int i = 0; i < num_points; ++i)
    this->aabb_local += points[i];

  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
}

//==============================================================================
template <typename S>
NODE_TYPE Convex<S>::getNodeType() const
{
  return GEOM_CONVEX;
}

//==============================================================================
template <typename S>
Matrix3<S> Convex<S>::computeMomentofInertia() const
{
  Matrix3<S> C = Matrix3<S>::Zero();

  Matrix3<S> C_canonical;
  C_canonical << 1/ 60.0, 1/120.0, 1/120.0,
      1/120.0, 1/ 60.0, 1/120.0,
      1/120.0, 1/120.0, 1/ 60.0;

  int* points_in_poly = polygons;
  int* index = polygons + 1;
  for(int i = 0; i < num_planes; ++i)
  {
    Vector3<S> plane_center = Vector3<S>::Zero();

    // compute the center of the polygon
    for(int j = 0; j < *points_in_poly; ++j)
      plane_center += points[index[j]];
    plane_center = plane_center * (1.0 / *points_in_poly);

    // compute the volume of tetrahedron making by neighboring two points, the plane center and the reference point (zero) of the convex shape
    const Vector3<S>& v3 = plane_center;
    for(int j = 0; j < *points_in_poly; ++j)
    {
      int e_first = index[j];
      int e_second = index[(j+1)%*points_in_poly];
      const Vector3<S>& v1 = points[e_first];
      const Vector3<S>& v2 = points[e_second];
      S d_six_vol = (v1.cross(v2)).dot(v3);
      Matrix3<S> A; // this is A' in the original document
      A.row(0) = v1;
      A.row(1) = v2;
      A.row(2) = v3;
      C += A.transpose() * C_canonical * A * d_six_vol; // change accordingly
    }

    points_in_poly += (*points_in_poly + 1);
    index = points_in_poly + 1;
  }

  S trace_C = C(0, 0) + C(1, 1) + C(2, 2);

  Matrix3<S> m;
  m << trace_C - C(0, 0), -C(0, 1), -C(0, 2),
      -C(1, 0), trace_C - C(1, 1), -C(1, 2),
      -C(2, 0), -C(2, 1), trace_C - C(2, 2);

  return m;
}

//==============================================================================
template <typename S>
Vector3<S> Convex<S>::computeCOM() const
{
  Vector3<S> com = Vector3<S>::Zero();
  S vol = 0;
  int* points_in_poly = polygons;
  int* index = polygons + 1;
  for(int i = 0; i < num_planes; ++i)
  {
    Vector3<S> plane_center = Vector3<S>::Zero();

    // compute the center of the polygon
    for(int j = 0; j < *points_in_poly; ++j)
      plane_center += points[index[j]];
    plane_center = plane_center * (1.0 / *points_in_poly);

    // compute the volume of tetrahedron making by neighboring two points, the plane center and the reference point (zero) of the convex shape
    const Vector3<S>& v3 = plane_center;
    for(int j = 0; j < *points_in_poly; ++j)
    {
      int e_first = index[j];
      int e_second = index[(j+1)%*points_in_poly];
      const Vector3<S>& v1 = points[e_first];
      const Vector3<S>& v2 = points[e_second];
      S d_six_vol = (v1.cross(v2)).dot(v3);
      vol += d_six_vol;
      com += (points[e_first] + points[e_second] + plane_center) * d_six_vol;
    }

    points_in_poly += (*points_in_poly + 1);
    index = points_in_poly + 1;
  }

  return com / (vol * 4); // here we choose zero as the reference
}

//==============================================================================
template <typename S>
S Convex<S>::computeVolume() const
{
  S vol = 0;
  int* points_in_poly = polygons;
  int* index = polygons + 1;
  for(int i = 0; i < num_planes; ++i)
  {
    Vector3<S> plane_center = Vector3<S>::Zero();

    // compute the center of the polygon
    for(int j = 0; j < *points_in_poly; ++j)
      plane_center += points[index[j]];
    plane_center = plane_center * (1.0 / *points_in_poly);

    // compute the volume of tetrahedron making by neighboring two points, the plane center and the reference point (zero point) of the convex shape
    const Vector3<S>& v3 = plane_center;
    for(int j = 0; j < *points_in_poly; ++j)
    {
      int e_first = index[j];
      int e_second = index[(j+1)%*points_in_poly];
      const Vector3<S>& v1 = points[e_first];
      const Vector3<S>& v2 = points[e_second];
      S d_six_vol = (v1.cross(v2)).dot(v3);
      vol += d_six_vol;
    }

    points_in_poly += (*points_in_poly + 1);
    index = points_in_poly + 1;
  }

  return vol / 6;
}

//==============================================================================
template <typename S>
void Convex<S>::fillEdges()
{
  int* points_in_poly = polygons;
  if(edges) delete [] edges;

  int num_edges_alloc = 0;
  for(int i = 0; i < num_planes; ++i)
  {
    num_edges_alloc += *points_in_poly;
    points_in_poly += (*points_in_poly + 1);
  }

  edges = new Edge[num_edges_alloc];

  points_in_poly = polygons;
  int* index = polygons + 1;
  num_edges = 0;
  Edge e;
  bool isinset;
  for(int i = 0; i < num_planes; ++i)
  {
    for(int j = 0; j < *points_in_poly; ++j)
    {
      e.first = std::min(index[j], index[(j+1)%*points_in_poly]);
      e.second = std::max(index[j], index[(j+1)%*points_in_poly]);
      isinset = false;
      for(int k = 0; k < num_edges; ++k)
      {
        if((edges[k].first == e.first) && (edges[k].second == e.second))
        {
          isinset = true;
          break;
        }
      }

      if(!isinset)
      {
        edges[num_edges].first = e.first;
        edges[num_edges].second = e.second;
        ++num_edges;
      }
    }

    points_in_poly += (*points_in_poly + 1);
    index = points_in_poly + 1;
  }

  if(num_edges < num_edges_alloc)
  {
    Edge* tmp = new Edge[num_edges];
    memcpy(tmp, edges, num_edges * sizeof(Edge));
    delete [] edges;
    edges = tmp;
  }
}

//==============================================================================
template <typename S>
std::vector<Vector3<S>> Convex<S>::getBoundVertices(
    const Transform3<S>& tf) const
{
  std::vector<Vector3<S>> result(num_points);
  for(int i = 0; i < num_points; ++i)
  {
    result[i] = tf * points[i];
  }

  return result;
}

} // namespace fcl

#endif

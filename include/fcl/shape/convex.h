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


#ifndef FCL_SHAPE_CONVEX_H
#define FCL_SHAPE_CONVEX_H

#include "fcl/shape/shape_base.h"
#include "fcl/shape/compute_bv.h"
#include "fcl/BV/OBB.h"

namespace fcl
{

/// @brief Convex polytope
template <typename ScalarT>
class Convex : public ShapeBase<ScalarT>
{
public:

  using Scalar = ScalarT;

  /// @brief Constructing a convex, providing normal and offset of each polytype surface, and the points and shape topology information 
  Convex(Vector3<ScalarT>* plane_normals,
         ScalarT* plane_dis,
         int num_planes,
         Vector3<ScalarT>* points,
         int num_points,
         int* polygons);

  /// @brief Copy constructor 
  Convex(const Convex& other);

  ~Convex();

  /// @brief Compute AABB<ScalarT>
  void computeLocalAABB() override;

  /// @brief Get node type: a conex polytope 
  NODE_TYPE getNodeType() const override;

  
  Vector3<ScalarT>* plane_normals;
  ScalarT* plane_dis;

  /// @brief An array of indices to the points of each polygon, it should be the number of vertices
  /// followed by that amount of indices to "points" in counter clockwise order
  int* polygons;

  Vector3<ScalarT>* points;
  int num_points;
  int num_edges;
  int num_planes;

  struct Edge
  {
    int first, second;
  };

  Edge* edges;

  /// @brief center of the convex polytope, this is used for collision: center is guaranteed in the internal of the polytope (as it is convex) 
  Vector3<ScalarT> center;

  /// based on http://number-none.com/blow/inertia/bb_inertia.doc
  Matrix3<ScalarT> computeMomentofInertia() const override;

  // Documentation inherited
  Vector3<ScalarT> computeCOM() const override;

  // Documentation inherited
  ScalarT computeVolume() const override;

  /// @brief get the vertices of some convex shape which can bound this shape in
  /// a specific configuration
  std::vector<Vector3<ScalarT>> getBoundVertices(
      const Transform3<ScalarT>& tf) const;

protected:

  /// @brief Get edge information 
  void fillEdges();
};

using Convexf = Convex<float>;
using Convexd = Convex<double>;

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename ScalarT>
Convex<ScalarT>::Convex(
    Vector3<ScalarT>* plane_normals, ScalarT* plane_dis, int num_planes,
    Vector3<ScalarT>* points, int num_points, int* polygons)
  : ShapeBase<ScalarT>()
{
  plane_normals = plane_normals;
  plane_dis = plane_dis;
  num_planes = num_planes;
  points = points;
  num_points = num_points;
  polygons = polygons;
  edges = NULL;

  Vector3<ScalarT> sum = Vector3<ScalarT>::Zero();
  for(int i = 0; i < num_points; ++i)
  {
    sum += points[i];
  }

  center = sum * (ScalarT)(1.0 / num_points);

  fillEdges();
}

//==============================================================================
template <typename ScalarT>
Convex<ScalarT>::Convex(const Convex& other)
  : ShapeBase<ScalarT>(other)
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
template <typename ScalarT>
Convex<ScalarT>::~Convex()
{
  delete [] edges;
}

//==============================================================================
template <typename ScalarT>
void Convex<ScalarT>::computeLocalAABB()
{
  computeBV(*this, Transform3<Scalar>::Identity(), this->aabb_local);
  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
}

//==============================================================================
template <typename ScalarT>
NODE_TYPE Convex<ScalarT>::getNodeType() const
{
  return GEOM_CONVEX;
}

//==============================================================================
template <typename ScalarT>
Matrix3<ScalarT> Convex<ScalarT>::computeMomentofInertia() const
{
  Matrix3<ScalarT> C = Matrix3<ScalarT>::Zero();

  Matrix3<ScalarT> C_canonical;
  C_canonical << 1/ 60.0, 1/120.0, 1/120.0,
      1/120.0, 1/ 60.0, 1/120.0,
      1/120.0, 1/120.0, 1/ 60.0;

  int* points_in_poly = polygons;
  int* index = polygons + 1;
  for(int i = 0; i < num_planes; ++i)
  {
    Vector3<ScalarT> plane_center = Vector3<ScalarT>::Zero();

    // compute the center of the polygon
    for(int j = 0; j < *points_in_poly; ++j)
      plane_center += points[index[j]];
    plane_center = plane_center * (1.0 / *points_in_poly);

    // compute the volume of tetrahedron making by neighboring two points, the plane center and the reference point (zero) of the convex shape
    const Vector3<ScalarT>& v3 = plane_center;
    for(int j = 0; j < *points_in_poly; ++j)
    {
      int e_first = index[j];
      int e_second = index[(j+1)%*points_in_poly];
      const Vector3<ScalarT>& v1 = points[e_first];
      const Vector3<ScalarT>& v2 = points[e_second];
      ScalarT d_six_vol = (v1.cross(v2)).dot(v3);
      Matrix3<ScalarT> A; // this is A' in the original document
      A.row(0) = v1;
      A.row(1) = v2;
      A.row(2) = v3;
      C += A.transpose() * C_canonical * A * d_six_vol; // change accordingly
    }

    points_in_poly += (*points_in_poly + 1);
    index = points_in_poly + 1;
  }

  ScalarT trace_C = C(0, 0) + C(1, 1) + C(2, 2);

  Matrix3<ScalarT> m;
  m << trace_C - C(0, 0), -C(0, 1), -C(0, 2),
      -C(1, 0), trace_C - C(1, 1), -C(1, 2),
      -C(2, 0), -C(2, 1), trace_C - C(2, 2);

  return m;
}

//==============================================================================
template <typename ScalarT>
Vector3<ScalarT> Convex<ScalarT>::computeCOM() const
{
  Vector3<ScalarT> com = Vector3<ScalarT>::Zero();
  ScalarT vol = 0;
  int* points_in_poly = polygons;
  int* index = polygons + 1;
  for(int i = 0; i < num_planes; ++i)
  {
    Vector3<ScalarT> plane_center = Vector3<ScalarT>::Zero();

    // compute the center of the polygon
    for(int j = 0; j < *points_in_poly; ++j)
      plane_center += points[index[j]];
    plane_center = plane_center * (1.0 / *points_in_poly);

    // compute the volume of tetrahedron making by neighboring two points, the plane center and the reference point (zero) of the convex shape
    const Vector3<Scalar>& v3 = plane_center;
    for(int j = 0; j < *points_in_poly; ++j)
    {
      int e_first = index[j];
      int e_second = index[(j+1)%*points_in_poly];
      const Vector3<ScalarT>& v1 = points[e_first];
      const Vector3<ScalarT>& v2 = points[e_second];
      ScalarT d_six_vol = (v1.cross(v2)).dot(v3);
      vol += d_six_vol;
      com += (points[e_first] + points[e_second] + plane_center) * d_six_vol;
    }

    points_in_poly += (*points_in_poly + 1);
    index = points_in_poly + 1;
  }

  return com / (vol * 4); // here we choose zero as the reference
}

//==============================================================================
template <typename ScalarT>
ScalarT Convex<ScalarT>::computeVolume() const
{
  ScalarT vol = 0;
  int* points_in_poly = polygons;
  int* index = polygons + 1;
  for(int i = 0; i < num_planes; ++i)
  {
    Vector3<ScalarT> plane_center = Vector3<ScalarT>::Zero();

    // compute the center of the polygon
    for(int j = 0; j < *points_in_poly; ++j)
      plane_center += points[index[j]];
    plane_center = plane_center * (1.0 / *points_in_poly);

    // compute the volume of tetrahedron making by neighboring two points, the plane center and the reference point (zero point) of the convex shape
    const Vector3<Scalar>& v3 = plane_center;
    for(int j = 0; j < *points_in_poly; ++j)
    {
      int e_first = index[j];
      int e_second = index[(j+1)%*points_in_poly];
      const Vector3<ScalarT>& v1 = points[e_first];
      const Vector3<ScalarT>& v2 = points[e_second];
      ScalarT d_six_vol = (v1.cross(v2)).dot(v3);
      vol += d_six_vol;
    }

    points_in_poly += (*points_in_poly + 1);
    index = points_in_poly + 1;
  }

  return vol / 6;
}

//==============================================================================
template <typename ScalarT>
void Convex<ScalarT>::fillEdges()
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
template <typename ScalarT>
std::vector<Vector3<ScalarT>> Convex<ScalarT>::getBoundVertices(
    const Transform3<ScalarT>& tf) const
{
  std::vector<Vector3<ScalarT>> result(num_points);
  for(int i = 0; i < num_points; ++i)
  {
    result[i] = tf * points[i];
  }

  return result;
}

} // namespace fcl

#include "fcl/shape/detail/bv_computer_convex.h"

#endif

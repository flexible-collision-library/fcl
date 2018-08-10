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
/** @author Sean Curtis (2018) Modify API and correct implementation bugs. */

#ifndef FCL_SHAPE_CONVEX_INL_H
#define FCL_SHAPE_CONVEX_INL_H

#include "fcl/geometry/shape/convex.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT Convex<double>;

//==============================================================================
template <typename S>
Convex<S>::Convex(int num_vertices, Vector3<S>* vertices,
                  int num_faces, int* faces)
  : ShapeBase<S>(),
    num_vertices(num_vertices),
    vertices(vertices),
    num_faces(num_faces),
    faces(faces) {
  assert(vertices != nullptr);
  assert(faces != nullptr);
  // Compute an interior point. We're computing the mean point and *not* some
  // alternative such as the centroid or bounding box center.
  Vector3<S> sum = Vector3<S>::Zero();
  for(int i = 0; i < num_vertices; ++i) {
    sum += vertices[i];
  }
  interior_point = sum * (S)(1.0 / num_vertices);
}

//==============================================================================
template <typename S>
void Convex<S>::computeLocalAABB() {
  this->aabb_local.min_.setConstant(std::numeric_limits<S>::max());
  this->aabb_local.max_.setConstant(-std::numeric_limits<S>::max());
  for(int i = 0; i < num_vertices; ++i)
    this->aabb_local += vertices[i];

  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
}

//==============================================================================
template <typename S>
NODE_TYPE Convex<S>::getNodeType() const {
  return GEOM_CONVEX;
}

//==============================================================================
// TODO(SeanCurtis-TRI): When revisiting these, consider the following
// resources:
//  https://www.geometrictools.com/Documentation/PolyhedralMassProperties.pdf
//  http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.56.127&rep=rep1&type=pdf
//  http://number-none.com/blow/inertia/bb_inertia.doc
template <typename S>
Matrix3<S> Convex<S>::computeMomentofInertia() const {
  Matrix3<S> C = Matrix3<S>::Zero();

  Matrix3<S> C_canonical;
  C_canonical << 1/ 60.0, 1/120.0, 1/120.0,
      1/120.0, 1/ 60.0, 1/120.0,
      1/120.0, 1/120.0, 1/ 60.0;

  S vol_times_six = 0;
  int* face_encoding = faces;
  int* index = faces + 1;
  for(int i = 0; i < num_faces; ++i) {
    const int vertex_count = *face_encoding;
    Vector3<S> face_center = Vector3<S>::Zero();

    // Compute the center of the face.
    for(int j = 0; j < vertex_count; ++j)
      face_center += vertices[index[j]];
    face_center = face_center * (1.0 / vertex_count);

    // Compute the volume of tetrahedron formed by the vertices on one of the
    // polygon's edges, the center point, and the shape's frame's origin.
    const Vector3<S>& v3 = face_center;
    for(int j = 0; j < vertex_count; ++j) {
      int e_first = index[j];
      int e_second = index[(j + 1) % vertex_count];
      const Vector3<S>& v1 = vertices[e_first];
      const Vector3<S>& v2 = vertices[e_second];
      S d_six_vol = (v1.cross(v2)).dot(v3);
      Matrix3<S> A; // this is A' in the original document
      A.row(0) = v1;
      A.row(1) = v2;
      A.row(2) = v3;
      C += A.transpose() * C_canonical * A * d_six_vol; // change accordingly
      vol_times_six += d_six_vol;
    }

    face_encoding += vertex_count + 1;
    index = face_encoding + 1;
  }

  S trace_C = C(0, 0) + C(1, 1) + C(2, 2);

  Matrix3<S> m;
  m << trace_C - C(0, 0), -C(0, 1), -C(0, 2),
      -C(1, 0), trace_C - C(1, 1), -C(1, 2),
      -C(2, 0), -C(2, 1), trace_C - C(2, 2);

  return m * (6 / vol_times_six);
}

//==============================================================================
template <typename S>
Vector3<S> Convex<S>::computeCOM() const {
  Vector3<S> com = Vector3<S>::Zero();
  S vol = 0;
  int* face_encoding = faces;
  int* index = faces + 1;
  for(int i = 0; i < num_faces; ++i) {
    const int vertex_count = *face_encoding;
    Vector3<S> face_center = Vector3<S>::Zero();

    // TODO(SeanCurtis-TRI): See note in computeVolume() on the efficiency of
    // this approach.

    // Compute the center of the polygon.
    for(int j = 0; j < vertex_count; ++j)
      face_center += vertices[index[j]];
    face_center = face_center * (1.0 / vertex_count);

    // Compute the volume of tetrahedron formed by the vertices on one of the
    // polygon's edges, the center point, and the shape's frame's origin.
    const Vector3<S>& v3 = face_center;
    for(int j = 0; j < vertex_count; ++j) {
      int e_first = index[j];
      int e_second = index[(j + 1) % vertex_count];
      const Vector3<S>& v1 = vertices[e_first];
      const Vector3<S>& v2 = vertices[e_second];
      S d_six_vol = (v1.cross(v2)).dot(v3);
      vol += d_six_vol;
      com += (vertices[e_first] + vertices[e_second] + face_center) * d_six_vol;
    }

    face_encoding += vertex_count + 1;
    index = face_encoding + 1;
  }

  return com / (vol * 4); // here we choose zero as the reference
}

//==============================================================================
template <typename S> S Convex<S>::computeVolume() const {
  S vol = 0;
  int *face_encoding = faces;
  int *index = faces + 1;
  for(int i = 0; i < num_faces; ++i) {
    const int vertex_count = *face_encoding;
    Vector3<S> face_center = Vector3<S>::Zero();

    // TODO(SeanCurtis-TRI): While this is general, this is inefficient. If the
    // face happens to be a triangle, this does 3X the requisite work.
    // If the face is a 4-gon, then this does 2X the requisite work.
    // As N increases in the N-gon this approach's inherent relative penalty
    // shrinks. Ideally, this should at least key on 3-gon and 4-gon before
    // falling through to this.

    // Compute the center of the polygon.
    for(int j = 0; j < vertex_count; ++j)
      face_center += vertices[index[j]];
    face_center = face_center * (1.0 / vertex_count);

    // TODO(SeanCurtis-TRI): Because volume serves as the weights for
    // center-of-mass an inertia computations, it should be refactored into its
    // own function that can be invoked by providing three vertices (the fourth
    // being the origin).

    // Compute the volume of tetrahedron formed by the vertices on one of the
    // polygon's edges, the center point, and the shape's frame's origin.
    const Vector3<S>& v3 = face_center;
    for(int j = 0; j < vertex_count; ++j) {
      int e_first = index[j];
      int e_second = index[(j + 1) % vertex_count];
      const Vector3<S>& v1 = vertices[e_first];
      const Vector3<S>& v2 = vertices[e_second];
      S d_six_vol = (v1.cross(v2)).dot(v3);
      vol += d_six_vol;
    }

    face_encoding += vertex_count + 1;
    index = face_encoding + 1;
  }

  return vol / 6;
}

//==============================================================================
template <typename S>
std::vector<Vector3<S>> Convex<S>::getBoundVertices(
    const Transform3<S>& tf) const {
  std::vector<Vector3<S>> result(num_vertices);
  for(int i = 0; i < num_vertices; ++i)
  {
    result[i] = tf * vertices[i];
  }

  return result;
}

} // namespace fcl

#endif

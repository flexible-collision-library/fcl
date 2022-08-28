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

#include <map>
#include <set>
#include <utility>

#include "fcl/geometry/shape/convex.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT Convex<double>;

//==============================================================================
template <typename S>
Convex<S>::Convex(
    const std::shared_ptr<const std::vector<Vector3<S>>>& vertices,
    int num_faces, const std::shared_ptr<const std::vector<int>>& faces,
    bool throw_if_invalid)
    : ShapeBase<S>(),
      vertices_(vertices),
      num_faces_(num_faces),
      faces_(faces),
      find_extreme_via_neighbors_{vertices->size() >
                                  kMinVertCountForEdgeWalking} {
  assert(vertices != nullptr);
  assert(faces != nullptr);
  // Compute an interior point. We're computing the mean point and *not* some
  // alternative such as the centroid or bounding box center.
  Vector3<S> sum = Vector3<S>::Zero();
  for (const auto& vertex : *vertices_) {
    sum += vertex;
  }
  interior_point_ = sum * (S)(1.0 / vertices_->size());
  FindVertexNeighbors();
  ValidateMesh(throw_if_invalid);
}

//==============================================================================
template <typename S>
void Convex<S>::computeLocalAABB() {
  this->aabb_local.min_.setConstant(std::numeric_limits<S>::max());
  this->aabb_local.max_.setConstant(-std::numeric_limits<S>::max());

  for (const auto& v : *vertices_) {
    this->aabb_local += v;
  }

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
  const std::vector<Vector3<S>>& vertices = *vertices_;
  const std::vector<int>& faces = *faces_;
  Matrix3<S> C = Matrix3<S>::Zero();

  Matrix3<S> C_canonical;
  C_canonical << 1/ 60.0, 1/120.0, 1/120.0,
      1/120.0, 1/ 60.0, 1/120.0,
      1/120.0, 1/120.0, 1/ 60.0;

  S vol_times_six = 0;
  int face_index = 0;
  for (int i = 0; i < num_faces_; ++i) {
    const int vertex_count = faces[face_index];
    Vector3<S> face_center = Vector3<S>::Zero();

    // Compute the center of the face.
    for (int j = 1; j <= vertex_count; ++j) {
      face_center += vertices[faces[face_index + j]];
    }
    face_center = face_center * (1.0 / vertex_count);

    // Compute the volume of tetrahedron formed by the vertices on one of the
    // polygon's edges, the center point, and the shape's frame's origin.
    const Vector3<S>& v3 = face_center;
    const int vertex_base = face_index + 1;
    for (int j = 0; j < vertex_count; ++j) {
      int e_first = faces[vertex_base + j];
      int e_second = faces[vertex_base + (j + 1) % vertex_count];
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

    face_index += vertex_count + 1;
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
  const std::vector<Vector3<S>>& vertices = *vertices_;
  const std::vector<int>& faces = *faces_;
  Vector3<S> com = Vector3<S>::Zero();
  S vol = 0;
  int face_index = 0;
  for (int i = 0; i < num_faces_; ++i) {
    const int vertex_count = faces[face_index];
    Vector3<S> face_center = Vector3<S>::Zero();

    // TODO(SeanCurtis-TRI): See note in computeVolume() on the efficiency of
    // this approach.

    // Compute the center of the polygon.
    for (int j = 1; j <= vertex_count; ++j) {
      face_center += vertices[faces[face_index + j]];
    }
    face_center = face_center * (1.0 / vertex_count);

    // Compute the volume of tetrahedron formed by the vertices on one of the
    // polygon's edges, the center point, and the shape's frame's origin.
    const Vector3<S>& v3 = face_center;
    for (int j = 1; j <= vertex_count; ++j) {
      int e_first = faces[face_index + j];
      int e_second = faces[face_index + (j % vertex_count) + 1];
      const Vector3<S>& v1 = vertices[e_first];
      const Vector3<S>& v2 = vertices[e_second];
      S d_six_vol = (v1.cross(v2)).dot(v3);
      vol += d_six_vol;
      com += (v1 + v2 + face_center) * d_six_vol;
    }

    face_index += vertex_count + 1;
  }

  return com / (vol * 4); // here we choose zero as the reference
}

//==============================================================================
template <typename S> S Convex<S>::computeVolume() const {
  const std::vector<Vector3<S>>& vertices = *vertices_;
  const std::vector<int>& faces = *faces_;
  S vol = 0;
  int face_index = 0;
  for (int i = 0; i < num_faces_; ++i) {
    const int vertex_count = faces[face_index];
    Vector3<S> face_center = Vector3<S>::Zero();

    // TODO(SeanCurtis-TRI): While this is general, this is inefficient. If the
    // face happens to be a triangle, this does 3X the requisite work.
    // If the face is a 4-gon, then this does 2X the requisite work.
    // As N increases in the N-gon this approach's inherent relative penalty
    // shrinks. Ideally, this should at least key on 3-gon and 4-gon before
    // falling through to this.

    // Compute the center of the polygon.
    for (int j = 1; j <= vertex_count; ++j) {
      face_center += vertices[faces[face_index + j]];
    }
    face_center = face_center * (1.0 / vertex_count);

    // TODO(SeanCurtis-TRI): Because volume serves as the weights for
    // center-of-mass and inertia computations, it should be refactored into its
    // own function that can be invoked by providing three vertices (the fourth
    // being the origin).

    // Compute the volume of tetrahedron formed by the vertices on one of the
    // polygon's edges, the center point, and the shape's frame's origin.
    const Vector3<S>& v3 = face_center;
    for (int j = 1; j <= vertex_count; ++j) {
      int e_first = faces[face_index + j];
      int e_second = faces[face_index + (j % vertex_count) + 1];
      const Vector3<S>& v1 = vertices[e_first];
      const Vector3<S>& v2 = vertices[e_second];
      S d_six_vol = (v1.cross(v2)).dot(v3);
      vol += d_six_vol;
    }

    face_index += vertex_count + 1;
  }

  return vol / 6;
}

//==============================================================================
template <typename S>
std::vector<Vector3<S>> Convex<S>::getBoundVertices(
    const Transform3<S>& tf) const {
  std::vector<Vector3<S>> result;
  result.reserve(vertices_->size());

  for (const auto& v : *vertices_) {
    result.push_back(tf * v);
  }

  return result;
}

//==============================================================================
template <typename S>
const Vector3<S>& Convex<S>::findExtremeVertex(const Vector3<S>& v_C) const {
  // TODO(SeanCurtis-TRI): Create an override of this that seeds the search with
  //  the last extremal vertex index (assuming some kind of coherency in the
  //  evaluation sequence).
  const std::vector<Vector3<S>>& vertices = *vertices_;
  // Note: A small amount of empirical testing suggests that a vector of int8_t
  // yields a slight performance improvement over int and bool. This is *not*
  // definitive.
  std::vector<int8_t> visited(vertices.size(), 0);
  int extreme_index = 0;
  S extreme_value = v_C.dot(vertices[extreme_index]);

  if (find_extreme_via_neighbors_) {
    bool keep_searching = true;
    while (keep_searching) {
      keep_searching = false;
      const int neighbor_start = neighbors_[extreme_index];
      const int neighbor_count = neighbors_[neighbor_start];
      for (int n_index = neighbor_start + 1;
           n_index <= neighbor_start + neighbor_count; ++n_index) {
        const int neighbor_index = neighbors_[n_index];
        if (visited[neighbor_index]) continue;
        visited[neighbor_index] = 1;
        const S neighbor_value = v_C.dot(vertices[neighbor_index]);
        // N.B. Testing >= (instead of >) protects us from the (very rare) case
        // where the *starting* vertex is co-planar with all of its neighbors
        // *and* the query direction v_C is perpendicular to that plane. With >
        // we wouldn't walk away from the start vertex. With >= we will walk
        // away and continue around (although it won't necessarily be the
        // fastest walk down).
        if (neighbor_value >= extreme_value) {
          keep_searching = true;
          extreme_index = neighbor_index;
          extreme_value = neighbor_value;
        }
      }
    }
  } else {
    // Simple linear search.
    for (int i = 1; i < static_cast<int>(vertices.size()); ++i) {
      S value = v_C.dot(vertices[i]);
      if (value > extreme_value) {
        extreme_index = i;
        extreme_value = value;
      }
    }
  }
  return vertices[extreme_index];
}

//==============================================================================
template <typename S>
void Convex<S>::ValidateMesh(bool throw_on_error) {
  ValidateTopology(throw_on_error);
  // TODO(SeanCurtis-TRI) Implement the missing "all-faces-are-planar" test.
  // TODO(SeanCurtis-TRI) Implement the missing "really-is-convex" test.
}

//==============================================================================
template <typename S>
void Convex<S>::ValidateTopology(bool throw_on_error) {
  // Computing the vertex neighbors is a pre-requisite to determining validity.
  assert(neighbors_.size() > vertices_->size());

  std::stringstream ss;
  ss << "Found errors in the Convex mesh:";

  // To simplify the code, we define an edge as a pair of ints (A, B) such that
  // A < B must be true.
  auto make_edge = [](int v0, int v1) {
      if (v0 > v1) std::swap(v0, v1);
      return std::make_pair(v0, v1);
  };

  bool all_connected = true;
  // Build a map from each unique edge to the _number_ of adjacent faces (see
  // the definition of make_edge for the encoding of an edge).
  std::map<std::pair<int, int>, int> per_edge_face_count;
  // First, pre-populate all the edges found in the vertex neighbor calculation.
  for (int v = 0; v < static_cast<int>(vertices_->size()); ++v) {
    const int neighbor_start = neighbors_[v];
    const int neighbor_count = neighbors_[neighbor_start];
    if (neighbor_count == 0) {
      if (all_connected) {
        ss << "\n Not all vertices are connected.";
        all_connected = false;
      }
      ss << "\n  Vertex " << v << " is not included in any faces.";
    }
    for (int n_index = neighbor_start + 1;
         n_index <= neighbor_start + neighbor_count; ++n_index) {
      const int n = neighbors_[n_index];
        per_edge_face_count[make_edge(v, n)] = 0;
    }
  }

  // To count adjacent faces, we must iterate through the faces; we can't infer
  // it from how many times an edge appears in the neighbor list. In the
  // degenerate case where three faces share an edge, that edge would only
  // appear twice. So, we must explicitly examine each face.
  const std::vector<int>& faces = *faces_;
  int face_index = 0;
  for (int f = 0; f < num_faces_; ++f) {
      const int vertex_count = faces[face_index];
      int prev_v = faces[face_index + vertex_count];
      for (int i = face_index + 1; i <= face_index + vertex_count; ++i) {
          const int v = faces[i];
          ++per_edge_face_count[make_edge(v, prev_v)];
          prev_v = v;
      }
      face_index += vertex_count + 1;
  }

  // Now examine the results.
  bool is_watertight = true;
  for (const auto& key_value_pair : per_edge_face_count) {
    const auto& edge = key_value_pair.first;
    const int count = key_value_pair.second;
    if (count != 2) {
      if (is_watertight) {
        is_watertight = false;
        ss << "\n The mesh is not watertight.";
      }
      ss << "\n  Edge between vertices " << edge.first << " and " << edge.second
         << " is shared by " << count << " faces (should be 2).";
    }
  }
  // We can't trust walking the edges on a mesh that isn't watertight.
  const bool has_error = !(is_watertight && all_connected);
  // Note: find_extreme_via_neighbors_ may already be false because the mesh
  // is too small. Don't indirectly re-enable it just because the mesh doesn't
  // have any errors.
  find_extreme_via_neighbors_ = find_extreme_via_neighbors_ && !has_error;
  if (has_error && throw_on_error) {
    throw std::runtime_error(ss.str());
  }
}

//==============================================================================
template <typename S>
void Convex<S>::FindVertexNeighbors() {
  // We initially build it using sets. Two faces with a common edge will
  // independently want to report that the edge's vertices are neighbors. So,
  // we rely on the set to eliminate the redundant declaration and then dump
  // the results to a more compact representation: the vector.
  const int v_count = static_cast<int>(vertices_->size());
  std::vector<std::set<int>> neighbors(v_count);
  const std::vector<int>& faces = *faces_;
  int face_index = 0;
  for (int f = 0; f < num_faces_; ++f) {
    const int vertex_count = faces[face_index];
    int prev_v = faces[face_index + vertex_count];
    for (int i = face_index + 1; i <= face_index + vertex_count; ++i) {
      const int v = faces[i];
      neighbors[v].insert(prev_v);
      neighbors[prev_v].insert(v);
      prev_v = v;
    }
    face_index += vertex_count + 1;
  }

  // Now build the encoded adjacency graph as documented.
  neighbors_.resize(v_count);
  for (int v = 0; v < v_count; ++v) {
    const std::set<int>& v_neighbors = neighbors[v];
    neighbors_[v] = static_cast<int>(neighbors_.size());
    neighbors_.push_back(static_cast<int>(v_neighbors.size()));
    neighbors_.insert(neighbors_.end(), v_neighbors.begin(), v_neighbors.end());
  }
}

template <typename S>
Vector3<S> Convex<S>::localGetSupportingVertex(const Vector3<S>& vec) const
{
  const Vector3<S> v = findExtremeVertex(vec);
  return Vector3<S>(v[0], v[1], v[2]);
}

} // namespace fcl

#endif

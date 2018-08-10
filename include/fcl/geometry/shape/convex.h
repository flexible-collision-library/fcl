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
/** @author Sean Curtis (2018)  Streamline API and document. */

#ifndef FCL_SHAPE_CONVEX_H
#define FCL_SHAPE_CONVEX_H

#include "fcl/geometry/shape/shape_base.h"

namespace fcl
{

/// @brief A convex polytope
///
/// The %Convex class represents a subset of general meshes: convex meshes.
/// The class represents its underlying mesh quite simply: an ordered list of
/// vertices, `V = [v₀, v₁, ..., vₙ₋₁]`, and an ordered list of faces,
/// `F = [f₀, f₁, ..., fₘ₋₁]`, built on those vertices (via vertex _indices_).
///
/// A mesh is only compatible with %Convex if it satisfies the following
/// properties:
///   - Each face, fᵢ, must be planar and completely lies on a supporting
///     plane πᵢ. The ordered sets `F = [f₀, f₁, ..., fₙ₋₁]` and
///     `Π = [π₀, π₁, ..., πₙ₋₁]` are defined such that face fᵢ has supporting
///     plane πᵢ.
///   - A face fᵢ is defined by an ordered list of vertex indices (e.g.,
///     `fᵢ = [iᵢ₀, iᵢ₁, ..., iᵢₖ₋₁]` (for a face with k vertices and k edges).
///     The _ordering_ of the vertex indices must visit the face's vertices in a
///     _counter-clockwise_ order when viewed from outside the mesh and the
///     vertices must all lie on the face's supporting plane.
///   - Define functions `πᵢ(x)` which report the signed distance from point `x`
///     to plane `πᵢ`. To be convex, it must be true that
///     `π(v) ≤ 0, ∀ π ∈ Π ∧ v ∈ V`.
///
/// If those requirements are satisfied, for a given convex region, the mesh
/// with the smallest number of faces *may* have non-triangular faces. In fact,
/// they could be polygons with an arbitrary number of edges/vertices. The
/// %Convex class supports compact representation. But it *also* supports
/// representations that decompose a large n-gon into a set of smaller polygons
/// or triangles. In this case, each of the triangles supporting planes would
/// be the same and the requirements listed above would still be satisfied.
///
/// @tparam S_  The scalar type; must be a valid Eigen scalar.
template <typename S_>
class FCL_EXPORT Convex : public ShapeBase<S_>
{
public:

  using S = S_;

  // TODO(SeanCurtis-TRI): A huge shopping list of issues with this class are
  // enumerated in https://github.com/flexible-collision-library/fcl/issues/326.

  /// @brief Constructor
  ///
  /// @note: The %Convex geometry does *not* take ownership of any of the data
  /// provided. The data must remain valid for as long as the %Convex instance
  /// and must be cleaned up explicitly.
  ///
  /// @warning: The %Convex class does *not* validate the input; it trusts that
  /// the inputs truly represent a coherent convex polytope.
  ///
  /// @param num_vertices   The number of vertices defined in `vertices`.
  /// @param vertices       The positions of the polytope vertices.
  /// @param num_faces      The number of faces defined in `faces`.
  /// @param faces          Encoding of the polytope faces. Must encode
  ///                       `num_faces` number of faces. See member
  ///                       documentation for details on encoding.
  Convex(int num_vertices, Vector3<S>* vertices, int num_faces, int* faces);

  /// @brief Copy constructor 
  Convex(const Convex& other) = default;

  ~Convex() = default;

  /// @brief Compute AABB<S> in the geometry's canonical frame.
  void computeLocalAABB() override;

  /// @brief Get node type: a convex polytope.
  NODE_TYPE getNodeType() const override;

  /// @brief The total number of vertices in the convex mesh.
  int num_vertices;

  /// @brief The vertex positions in the geometry's frame G.
  Vector3<S>* vertices;

  /// @brief The total number of faces in the convex mesh.
  int num_faces;

  /// @brief The representation of the *faces* of the convex hull.
  ///
  /// The array is the concatenation of an integer-based representations of each
  /// face. A single face is encoded as a sub-array of ints where the first int
  /// is the *number* n of vertices in the face, and the following n values
  /// are ordered indices into `vertices` which visit the vertex values in a
  /// *counter-clockwise* order (viewed from the outside).
  ///
  /// For a well-formed face `f` consisting of indices [i₀, i₁, ..., iₘ₋₁], it
  /// should be the case that:
  ///
  ///    `eⱼ × eⱼ₊₁ · n̂ₚ = |eⱼ × eⱼ₊₁|, ∀ 0 ≤ j < m, j ∈ ℤ`, where
  ///    `n̂ₚ` is the normal for plane `π` on which face `f` lies.
  ///    `eⱼ = vertices[iⱼ] - vertices[iⱼ₋₁]` is the edge vector of face `f`
  ///    defined by adjacent vertex indices at jᵗʰ vertex (wrapping around such
  ///    that `j - 1 = m - 1` for `j = 0`).
  ///
  /// Satisfying this condition implies the following:
  ///    1. vertices are not coincident and
  ///    3. the indices of the face correspond to a proper counter-clockwise
  ///       ordering.
  int* faces;

  /// @brief A point guaranteed to be on the interior of the convex polytope,
  /// used for collision.
  Vector3<S> interior_point;

  // Documentation inherited.
  Matrix3<S> computeMomentofInertia() const override;

  // Documentation inherited.
  Vector3<S> computeCOM() const override;

  // Documentation inherited.
  S computeVolume() const override;

  /// @brief get the vertices of some convex shape which can bound this shape in
  /// a specific configuration
  std::vector<Vector3<S>> getBoundVertices(const Transform3<S>& tf) const;
};

using Convexf = Convex<float>;
using Convexd = Convex<double>;

} // namespace fcl

#include "fcl/geometry/shape/convex-inl.h"

#endif

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

#include <iostream>
#include <memory>
#include <vector>

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
  /// @note: The %Convex geometry assumes that the input data %vertices and
  /// %faces do not change through the life of the object.
  ///
  /// @warning: The %Convex class only partially validates the input; it
  /// generally trusts that the inputs truly represent a coherent convex
  /// polytope. For those aspects that *are* validated, the constructor will
  /// throw for failed tests if `throw_if_invalid` is set to `true`.
  ///
  /// @param vertices           The positions of the polytope vertices.
  /// @param num_faces          The number of faces defined in `faces`.
  /// @param faces              Encoding of the polytope faces. Must encode
  ///                           `num_faces` number of faces. See member
  ///                           documentation for details on encoding.
  /// @param throw_if_invalid   If `true`, failed attempts to validate the mesh
  ///                           will throw an exception.
  Convex(const std::shared_ptr<const std::vector<Vector3<S>>>& vertices,
         int num_faces, const std::shared_ptr<const std::vector<int>>& faces,
         bool throw_if_invalid = false);

  /// @brief Copy constructor 
  Convex(const Convex& other);

  ~Convex() = default;

  /// @brief Computes AABB<S> in the geometry's canonical frame.
  void computeLocalAABB() override;

  /// @brief Gets node type: a convex polytope.
  NODE_TYPE getNodeType() const override;

  /// @brief Gets the vertex positions in the geometry's frame G.
  const std::vector<Vector3<S>>& getVertices() const { return *vertices_; }

  /// @brief Gets the total number of faces in the convex mesh.
  int getFaceCount() const { return num_faces_; }

  /// @brief Gets the representation of the *faces* of the convex hull.
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
  const std::vector<int>& getFaces() const { return *faces_; }

  /// @brief A point guaranteed to be on the interior of the convex polytope,
  /// used for collision.
  const Vector3<S>& getInteriorPoint() const { return interior_point_; }

  // Documentation inherited.
  Matrix3<S> computeMomentofInertia() const override;

  // Documentation inherited.
  Vector3<S> computeCOM() const override;

  // Documentation inherited.
  S computeVolume() const override;

  /// @brief Gets the vertices of some convex shape which can bound this shape in
  /// a specific configuration
  std::vector<Vector3<S>> getBoundVertices(const Transform3<S>& tf) const;

  /// @brief Reports a vertex in this convex polytope that lies farthest in the
  /// given direction v_C. The direction vector must be expressed in the same
  /// frame C as the vertices of `this` %Convex polytope.
  /// @retval p_CE the position vector from Frame C's origin to the extreme
  ///         vertex E. It is guaranteed that v_C⋅p_CE ≥ v_C⋅p_CF for all F ≠ E
  ///         in the  %Convex polytope's set of vertices.
  const Vector3<S>& findExtremeVertex(const Vector3<S>& v_C) const;

  virtual Vector3<S> localGetSupportingVertex(const Vector3<S>& vec) const override;

  friend
  std::ostream& operator<<(std::ostream& out, const Convex& convex) {
    out << "Convex(v count: " << convex.vertices_->size() << ", f count: "
        << convex.getFaceCount() << ")";
    return out;
  }

 private:
  // Test utility to examine Convex internal state.
  friend class ConvexTester;

  // TODO(SeanCurtis-TRI): Complete the validation.
  // *Partially* validate the mesh. The following properties are validated:
  //   - Confirms mesh is water tight (see IsWaterTight).
  //   - Confirms that all vertices are included in a face.
  // The following properties still need to be validated:
  //   - There are at least four vertices (the minimum to enclose a *volume*.)
  //   - the vertices associated with each face are planar.
  //   - For each face, all vertices *not* forming the face lie "on" or "behind"
  //     the face.
  //
  // Invoking this *can* have side effects. Internal configuration can change
  // based on failed validation tests. For example, the performance of
  // findExtremeVertex() depends on the mesh being "valid" -- an invalid mesh
  // can be used, but will use a slower algorithm as a result of being found
  // invalid.
  //
  // @param throw_on_error  If `true` and the convex is shown to be invalid, an
  //                        exception is thrown.
  void ValidateMesh(bool throw_on_error);

  // Reports if the mesh is watertight and that every vertex is part of a face.
  // The mesh is watertight if every edge is shared by two different faces.
  //
  // As a side effect, if this fails, find_extreme_via_neighbors_ is set to
  // false because a water tight mesh is a prerequisite to being able to find
  // extremal points by following edges.
  //
  // @param throw_on_error  If `true` and the convex is shown to be invalid, an
  //                        exception is thrown.
  // @pre FindVertexNeighbors() must have been called already.
  void ValidateTopology(bool throw_on_error);

  // Analyzes the convex specification and builds the map of vertex ->
  // neighboring vertices.
  void FindVertexNeighbors();

  const std::shared_ptr<const std::vector<Vector3<S>>> vertices_;
  const int num_faces_;
  const std::shared_ptr<const std::vector<int>> faces_;
  Vector3<S> interior_point_;

  /* The encoding of vertex adjacency in the mesh. The encoding is as follows:
   Entries [0, V-1] encode the location in `neighbors_` where the adjacency
   data for the ith vertex lies in the array.
   Following those offsets is the compact representation of each vertex's
   neighbors as: number of neighbors, n0, n1, ....

   The figure below shows that for vertex j, entry j tells us to jump into
   the vector at neighbors_[j]. The value mⱼ -- the number of vertices adjacent
   to j -- is stored at that location. The next mⱼ entries starting at
   neighbors_[j] + 1 are the *indices* of those vertices adjacent to vertex j.

               Index where
               vertex j's     Vertex j's
               data lies      neighbor count
                    ↓          ↓
           |_|...|_|_|_|......┃mⱼ┃n₁┃n₂┃...┃nₘ┃mⱼ₊₁|...|
            0 ...   j             ↑   ↑ ... ↑
                                 Indices of vertex j's
                                 neighboring vertices.

   A modicum testing indicated that this compact representation led to
   measurably improved performance for findExtremeVertex() -- initial
   hypothesis attributes it to improved cache hits. */
  std::vector<int> neighbors_;

  // If true, findExtremeVertex() can reliably use neighbor_vertices_ to walk
  // along the surface of the mesh. If false, it must linearly search.
  bool find_extreme_via_neighbors_{false};

  // Empirical evidence suggests that finding the extreme vertex by walking the
  // edges of the mesh is only more efficient if there are more than 32
  // vertices.
  static constexpr int kMinVertCountForEdgeWalking = 32;
};

// Workaround for https://gcc.gnu.org/bugzilla/show_bug.cgi?id=57728 which
// should be moved back into the class definition once we no longer need to
// support GCC versions prior to 6.3.
template <typename S>
Convex<S>::Convex(const Convex<S>& other) = default;

using Convexf = Convex<float>;
using Convexd = Convex<double>;

} // namespace fcl

#include "fcl/geometry/shape/convex-inl.h"

#endif

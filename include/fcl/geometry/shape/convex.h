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

#ifndef FCL_SHAPE_CONVEX_H
#define FCL_SHAPE_CONVEX_H

#include "fcl/geometry/shape/shape_base.h"

namespace fcl
{

/// @brief Convex polytope
template <typename S_>
class FCL_EXPORT Convex : public ShapeBase<S_>
{
public:

  using S = S_;

  // TODO(SeanCurtis-TRI): Figure out what the ownership of this data is. It
  // *seems* that this struct only owns the edges.
  /// @brief Definition of a convex polytope.
  ///
  /// This requires *two* coordinated representations of the convex hull:
  ///   - The set of half spaces that bound the hull region and
  ///   - the polytope mesh formed by the intersection of the half spaces.
  ///
  /// The half spaces are defined by the implicit equation of a plane:
  /// `Π(x, y, z) = Ax + By + Cz + d = 0` where `n̂ = [A, B, C]` and |n̂| = 1.
  /// The normal points *outside* of the convex region. There should be *no*
  /// redundant planes in the set.
  ///
  /// For each plane Π there should corresponding face f that lies on that plane
  /// and whose edges and vertices are defined by the intersection of all other
  /// planes with plane Π.
  ///
  /// @note: The %Convex geometry does *not* take ownership of any of the data
  /// provided. The data must remain valid for as long as the %Convex instance
  /// and must be cleaned up explicitly.
  ///
  /// @param plane_normals  For m planes, the normals of plane i:
  ///                       `[n̂₀, n̂₁, ..., n̂ₘ₋₁]`.
  /// @param plane_dis      For m planes, the offset values of plane i:
  ///                       `[d₀, d₁, ..., dₘ₋₁]`.
  /// @param num_planes     The number of planes `m`.
  /// @param points         All unique vertices formed by the intersection of
  ///                       three or more planes.
  /// @param num_points     The number of vertices in `points`.
  /// @param polygons       Encoding of the faces for each plane. See member
  ///                       documentation for details on encoding.
  Convex(Vector3<S>* plane_normals,
         S* plane_dis,
         int num_planes,
         Vector3<S>* points,
         int num_points,
         int* polygons);

  /// @brief Copy constructor 
  Convex(const Convex& other);

  ~Convex();

  /// @brief Compute AABB<S>
  void computeLocalAABB() override;

  /// @brief Get node type: a convex polytope
  NODE_TYPE getNodeType() const override;

  /// @brief
  Vector3<S>* plane_normals;
  S* plane_dis;

  /// @brief The representation of the *faces* of the convex hull.
  ///
  /// The array is the concatenation of integer-based representations of each
  /// face. A single face is encoded as a sub-array of ints where the first int
  /// is the *number* n of vertices in the face, and the following n values
  /// are ordered indices into `points` of the vertices in a *counter-clockwise*
  /// order (viewed from the outside).
  ///
  /// For a well-formed face `f` consisting of indices [v₀, v₁, ..., vₘ₋₁], it
  /// should be the case that:
  ///
  ///    `rᵢ × rᵢ₊₁ · n̂ₚ = |rᵢ × rᵢ₊₁|, ∀ 0 ≤ i < m, i ∈ ℤ`, where
  ///    `n̂ₚ` is the normal for plane `p` on which the face `f` lies.
  ///    `rᵢ = points[vᵢ] - points[vᵢ₋₁]` is the displacement of the edge of
  ///    face `f` defined by adjacent vertex indices at iᵗʰ vertex (wrapping
  ///    around such that i - 1 = m - 1 for i = 0).
  ///
  /// Satisfying this condition implies the following:
  ///    1. Vertices are not coincident,
  ///    2. The nᵗʰ encoded polygon corresponds with the nᵗʰ plane normal,
  ///    3. The indices of the face correspond to a proper counter-clockwise
  ///       ordering.
  int* polygons;

  Vector3<S>* points;
  int num_points;
  int num_edges;
  int num_planes;

  struct Edge
  {
    int first, second;
  };

  Edge* edges;

  /// @brief The mean point of the convex polytope, used for collision. This
  /// point is guaranteed to be inside the convex hull.
  /// note: The name "center" is misleading; it is neither the centroid nor the
  /// center of mass.
  Vector3<S> center;

  /// based on http://number-none.com/blow/inertia/bb_inertia.doc
  Matrix3<S> computeMomentofInertia() const override;

  // Documentation inherited
  Vector3<S> computeCOM() const override;

  // Documentation inherited
  S computeVolume() const override;

  /// @brief get the vertices of some convex shape which can bound this shape in
  /// a specific configuration
  std::vector<Vector3<S>> getBoundVertices(const Transform3<S>& tf) const;

protected:

  /// @brief Get edge information 
  void fillEdges();
};

using Convexf = Convex<float>;
using Convexd = Convex<double>;

} // namespace fcl

#include "fcl/geometry/shape/convex-inl.h"

#endif

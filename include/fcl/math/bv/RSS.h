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

#ifndef FCL_BV_RSS_H
#define FCL_BV_RSS_H

#include "fcl/math/constants.h"
#include "fcl/math/geometry.h"

namespace fcl
{

/// @brief A class for rectangle sphere-swept bounding volume
template <typename S_>
class FCL_VISIBLE RSS
{
public:

  using S = S_;

  /// @brief Orientation of RSS. The axes of the rotation matrix are the
  /// principle directions of the box. We assume that the first column
  /// corresponds to the axis with the longest box edge, second column
  /// corresponds to the shorter one and the third coulumn corresponds to the
  /// shortest one.
  Matrix3<S> axis;

  /// @brief Origin of the rectangle in RSS
  Vector3<S> To;

  /// @brief Side lengths of rectangle
  S l[2];

  /// @brief Radius of sphere summed with rectangle to form RSS
  S r;

  /// Constructor
  RSS();

  /// @brief Check collision between two RSS
  bool overlap(const RSS<S>& other) const;

  /// @brief Check collision between two RSS and return the overlap part.
  /// For RSS, we return nothing, as the overlap part of two RSSs usually is not a RSS.
  bool overlap(const RSS<S>& other, RSS<S>& overlap_part) const;

  /// @brief Check whether the RSS contains a point
  bool contain(const Vector3<S>& p) const;

  /// @brief A simple way to merge the RSS and a point, not compact.
  /// @todo This function may have some bug.
  RSS<S>& operator += (const Vector3<S>& p);

  /// @brief Merge the RSS and another RSS
  RSS<S>& operator += (const RSS<S>& other);

  /// @brief Return the merged RSS of current RSS and the other one
  RSS<S> operator + (const RSS<S>& other) const;

  /// @brief Width of the RSS
  S width() const;

  /// @brief Height of the RSS
  S height() const;

  /// @brief Depth of the RSS
  S depth() const;

  /// @brief Volume of the RSS
  S volume() const;

  /// @brief Size of the RSS (used in BV_Splitter to order two RSSs)
  S size() const;

  /// @brief The RSS center
  const Vector3<S> center() const;

  /// @brief the distance between two RSS; P and Q, if not nullptr, return the nearest points
  S distance(const RSS<S>& other,
             Vector3<S>* P = nullptr,
             Vector3<S>* Q = nullptr) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

using RSSf = RSS<float>;
using RSSd = RSS<double>;

/// @brief Clip value between a and b
template <typename S>
void clipToRange(S& val, S a, S b);

/// @brief Finds the parameters t & u corresponding to the two closest points on
/// a pair of line segments. The first segment is defined as
/// Pa + A*t, 0 <= t <= a,  where "Pa" is one endpoint of the segment, "A" is a
/// unit vector pointing to the other endpoint, and t is a scalar that produces
/// all the points between the two endpoints. Since "A" is a unit vector, "a" is
/// the segment's length. The second segment is defined as
/// Pb + B*u, 0 <= u <= b. Many of the terms needed by the algorithm are already
/// computed for other purposes,so we just pass these terms into the function
/// instead of complete specifications of each segment. "T" in the dot products
/// is the vector betweeen Pa and Pb.
/// Reference: "On fast computation of distance between line segments." Vladimir
/// J. Lumelsky, in Information Processing Letters, no. 21, pages 55-61, 1985.
template <typename S>
void segCoords(S& t, S& u, S a, S b,
               S A_dot_B, S A_dot_T, S B_dot_T);

/// @brief Returns whether the nearest point on rectangle edge
/// Pb + B*u, 0 <= u <= b, to the rectangle edge,
/// Pa + A*t, 0 <= t <= a, is within the half space
/// determined by the point Pa and the direction Anorm.
/// A,B, and Anorm are unit vectors. T is the vector between Pa and Pb.
template <typename S>
bool inVoronoi(S a, S b,
               S Anorm_dot_B, S Anorm_dot_T,
               S A_dot_B, S A_dot_T, S B_dot_T);

/// @brief Distance between two oriented rectangles; P and Q (optional return
/// values) are the closest points in the rectangles, both are in the local
/// frame of the first rectangle.
template <typename S>
S rectDistance(
    const Matrix3<S>& Rab,
    const Vector3<S>& Tab,
    const S a[2],
    const S b[2],
    Vector3<S>* P = nullptr,
    Vector3<S>* Q = nullptr);

/// @brief Distance between two oriented rectangles; P and Q (optional return
/// values) are the closest points in the rectangles, both are in the local
/// frame of the first rectangle.
template <typename S>
S rectDistance(
    const Transform3<S>& tfab,
    const S a[2],
    const S b[2],
    Vector3<S>* P = nullptr,
    Vector3<S>* Q = nullptr);

/// @brief distance between two RSS bounding volumes
/// P and Q (optional return values) are the closest points in the rectangles,
/// not the RSS. But the direction P - Q is the correct direction for cloest
/// points. Notice that P and Q are both in the local frame of the first RSS
/// (not global frame and not even the local frame of object 1)
template <typename S, typename DerivedA, typename DerivedB>
S distance(
    const Eigen::MatrixBase<DerivedA>& R0,
    const Eigen::MatrixBase<DerivedB>& T0,
    const RSS<S>& b1,
    const RSS<S>& b2,
    Vector3<S>* P = nullptr,
    Vector3<S>* Q = nullptr);

/// @brief Check collision between two RSSs, b1 is in configuration (R0, T0) and
/// b2 is in identity.
template <typename S, typename DerivedA, typename DerivedB>
bool overlap(
    const Eigen::MatrixBase<DerivedA>& R0,
    const Eigen::MatrixBase<DerivedB>& T0,
    const RSS<S>& b1,
    const RSS<S>& b2);

/// @brief Translate the RSS bv
template <typename S>
RSS<S> translate(const RSS<S>& bv, const Vector3<S>& t);

} // namespace fcl

#include "fcl/math/bv/RSS-inl.h"

#endif

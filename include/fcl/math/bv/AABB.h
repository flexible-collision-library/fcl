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

#ifndef FCL_BV_AABB_H
#define FCL_BV_AABB_H

#include "fcl/common/types.h"

namespace fcl
{

/// @brief A class describing the AABB collision structure, which is a box in 3D
/// space determined by two diagonal points
template <typename S_>
class FCL_EXPORT AABB
{
public:

  using S = S_;

  /// @brief The min point in the AABB
  Vector3<S> min_;

  /// @brief The max point in the AABB
  Vector3<S> max_;

  /// @brief Creating an AABB with zero size (low bound +inf, upper bound -inf)
  AABB();

  /// @brief Creating an AABB at position v with zero size
  AABB(const Vector3<S>& v);

  /// @brief Creating an AABB with two endpoints a and b
  AABB(const Vector3<S>& a, const Vector3<S>&b);

  /// @brief Creating an AABB centered as core and is of half-dimension delta
  AABB(const AABB<S>& core, const Vector3<S>& delta);

  /// @brief Creating an AABB contains three points
  AABB(const Vector3<S>& a, const Vector3<S>& b, const Vector3<S>& c);

  /// @brief Check whether two AABB are overlap
  bool overlap(const AABB<S>& other) const;

  /// @brief Check whether the AABB contains another AABB
  bool contain(const AABB<S>& other) const;

  /// @brief Check whether two AABB are overlapped along specific axis
  bool axisOverlap(const AABB<S>& other, int axis_id) const;

  /// @brief Check whether two AABB are overlap and return the overlap part
  bool overlap(const AABB<S>& other, AABB<S>& overlap_part) const;

  /// @brief Check whether the AABB contains a point
  bool contain(const Vector3<S>& p) const;

  /// @brief Merge the AABB and a point
  AABB<S>& operator += (const Vector3<S>& p);

  /// @brief Merge the AABB and another AABB
  AABB<S>& operator += (const AABB<S>& other);

  /// @brief Return the merged AABB of current AABB and the other one
  AABB<S> operator + (const AABB<S>& other) const;

  /// @brief Width of the AABB
  S width() const;

  /// @brief Height of the AABB
  S height() const;

  /// @brief Depth of the AABB
  S depth() const;

  /// @brief Volume of the AABB
  S volume() const;

  /// @brief Size of the AABB (used in BV_Splitter to order two AABBs)
  S size() const;

  /// @brief Radius of the AABB
  S radius() const;

  /// @brief Center of the AABB
  Vector3<S> center() const;

  /// @brief Distance between two AABBs; P and Q, should not be nullptr, return
  /// the nearest points
  S distance(const AABB<S>& other, Vector3<S>* P, Vector3<S>* Q) const;

  /// @brief Distance between two AABBs
  S distance(const AABB<S>& other) const;

  /// @brief whether two AABB are equal
  bool equal(const AABB<S>& other) const;

  /// @brief expand the half size of the AABB by delta, and keep the center
  /// unchanged.
  AABB<S>& expand(const Vector3<S>& delta);

  /// @brief expand the aabb by increase the thickness of the plate by a ratio
  AABB<S>& expand(const AABB<S>& core, S ratio);
};

using AABBf = AABB<float>;
using AABBd = AABB<double>;

/// @brief translate the center of AABB by t
template <typename S, typename Derived>
AABB<S> translate(
    const AABB<S>& aabb, const Eigen::MatrixBase<Derived>& t);

} // namespace fcl

#include "fcl/math/bv/AABB-inl.h"

#endif

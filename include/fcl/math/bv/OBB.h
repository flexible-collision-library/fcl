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

#ifndef FCL_BV_OBB_H
#define FCL_BV_OBB_H

#include <iostream>

#include "fcl/common/types.h"
#include "fcl/math/geometry.h"

namespace fcl
{

/// @brief Oriented bounding box class
template <typename S_>
class FCL_EXPORT OBB
{
public:

  using S = S_;

  /// @brief Orientation of OBB. The axes of the rotation matrix are the
  /// principle directions of the box. We assume that the first column
  /// corresponds to the axis with the longest box edge, second column
  /// corresponds to the shorter one and the third coulumn corresponds to the
  /// shortest one.
  Matrix3<S> axis;

  /// @brief Center of OBB
  Vector3<S> To;

  /// @brief Half dimensions of OBB
  Vector3<S> extent;

  /// @brief Constructor
  OBB();

  /// @brief Constructor
  OBB(const Matrix3<S>& axis,
      const Vector3<S>& center,
      const Vector3<S>& extent);

  /// @brief Check collision between two OBB, return true if collision happens. 
  bool overlap(const OBB<S>& other) const;
  
  /// @brief Check collision between two OBB and return the overlap part. For OBB, the overlap_part return value is NOT used as the overlap part of two obbs usually is not an obb. 
  bool overlap(const OBB<S>& other, OBB<S>& overlap_part) const;

  /// @brief Check whether the OBB contains a point.
  bool contain(const Vector3<S>& p) const;

  /// @brief A simple way to merge the OBB and a point (the result is not compact).
  OBB<S>& operator +=(const Vector3<S>& p);

  /// @brief Merge the OBB and another OBB (the result is not compact).
  OBB<S>& operator += (const OBB<S>& other);

  /// @brief Return the merged OBB of current OBB and the other one (the result is not compact).
  OBB<S> operator + (const OBB<S>& other) const;

  /// @brief Width of the OBB.
  S width() const;

  /// @brief Height of the OBB.
  S height() const;

  /// @brief Depth of the OBB
  S depth() const;

  /// @brief Volume of the OBB
  S volume() const;

  /// @brief Size of the OBB (used in BV_Splitter to order two OBBs)
  S size() const;

  /// @brief Center of the OBB
  const Vector3<S> center() const;

  /// @brief Distance between two OBBs, not implemented.
  S distance(const OBB& other, Vector3<S>* P = nullptr,
                  Vector3<S>* Q = nullptr) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

using OBBf = OBB<float>;
using OBBd = OBB<double>;

/// @brief Compute the 8 vertices of a OBB
template <typename S>
FCL_EXPORT
void computeVertices(const OBB<S>& b, Vector3<S> vertices[8]);

/// @brief OBB merge method when the centers of two smaller OBB are far away
template <typename S>
FCL_EXPORT
OBB<S> merge_largedist(const OBB<S>& b1, const OBB<S>& b2);

/// @brief OBB merge method when the centers of two smaller OBB are close
template <typename S>
FCL_EXPORT
OBB<S> merge_smalldist(const OBB<S>& b1, const OBB<S>& b2);

/// @brief Translate the OBB bv
template <typename S, typename Derived>
FCL_EXPORT
OBB<S> translate(
    const OBB<S>& bv, const Eigen::MatrixBase<Derived>& t);

/// @brief Check collision between two obbs, b1 is in configuration (R0, T0) and
/// b2 is in identity.
template <typename S, typename DerivedA, typename DerivedB>
FCL_EXPORT
bool overlap(const Eigen::MatrixBase<DerivedA>& R0,
             const Eigen::MatrixBase<DerivedB>& T0,
             const OBB<S>& b1, const OBB<S>& b2);

/// @brief Check collision between two boxes: the first box is in configuration
/// (R, T) and its half dimension is set by a; the second box is in identity
/// configuration and its half dimension is set by b.
template <typename S>
FCL_EXPORT
bool obbDisjoint(
    const Matrix3<S>& B,
    const Vector3<S>& T,
    const Vector3<S>& a,
    const Vector3<S>& b);

/// @brief Check collision between two boxes: the first box is in configuration
/// (R, T) and its half dimension is set by a; the second box is in identity
/// configuration and its half dimension is set by b.
template <typename S>
FCL_EXPORT
bool obbDisjoint(
    const Transform3<S>& tf,
    const Vector3<S>& a,
    const Vector3<S>& b);

} // namespace fcl

#include "fcl/math/bv/OBB-inl.h"

#endif

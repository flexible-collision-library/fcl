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

#ifndef FCL_BV_KIOS_H
#define FCL_BV_KIOS_H

#include "fcl/math/bv/OBB.h"

namespace fcl
{
 
/// @brief A class describing the kIOS collision structure, which is a set of spheres.
template <typename S_>
class FCL_EXPORT kIOS
{
  /// @brief One sphere in kIOS
  struct kIOS_Sphere
  {
    Vector3<S_> o;
    S_ r;
  };

  /// @brief generate one sphere enclosing two spheres
  static kIOS_Sphere encloseSphere(
      const kIOS_Sphere& s0, const kIOS_Sphere& s1);

public:

  using S = S_;

  /// @brief The (at most) five spheres for intersection
  kIOS_Sphere spheres[5];

  /// @brief The number of spheres, no larger than 5
  unsigned int num_spheres;

  /// @brief OBB related with kIOS
  OBB<S> obb;

  /// @brief Check collision between two kIOS
  bool overlap(const kIOS<S>& other) const;

  /// @brief Check collision between two kIOS and return the overlap part.
  /// For kIOS, we return nothing, as the overlappart of two kIOS usually is not
  /// an kIOS
  /// @todo Not efficient. It first checks the sphere collisions and then use
  /// OBB for further culling.
  bool overlap(const kIOS<S>& other, kIOS<S>& overlap_part) const;

  /// @brief Check whether the kIOS contains a point
  bool contain(const Vector3<S>& p) const;

  /// @brief A simple way to merge the kIOS and a point
  kIOS<S>& operator += (const Vector3<S>& p);

  /// @brief Merge the kIOS and another kIOS
  kIOS<S>& operator += (const kIOS<S>& other);

  /// @brief Return the merged kIOS of current kIOS and the other one
  kIOS<S> operator + (const kIOS<S>& other) const;

  /// @brief Center of the kIOS
  const Vector3<S>& center() const;

  /// @brief Width of the kIOS
  S width() const;

  /// @brief Height of the kIOS
  S height() const;

  /// @brief Depth of the kIOS
  S depth() const;

  /// @brief Volume of the kIOS
  S volume() const;

  /// @brief size of the kIOS (used in BV_Splitter to order two kIOSs)
  S size() const;

  /// @brief The distance between two kIOS
  S distance(
      const kIOS<S>& other,
      Vector3<S>* P = nullptr, Vector3<S>* Q = nullptr) const;

  static constexpr S ratio() { return 1.5; }
  static constexpr S invSinA() { return 2; }
  static S cosA() { return std::sqrt(3.0) / 2.0; }
};

using kIOSf = kIOS<float>;
using kIOSd = kIOS<double>;

/// @brief Translate the kIOS BV
template <typename S, typename Derived>
FCL_EXPORT
kIOS<S> translate(
    const kIOS<S>& bv, const Eigen::MatrixBase<Derived>& t);

/// @brief Check collision between two kIOSs, b1 is in configuration (R0, T0)
/// and b2 is in identity.
/// @todo Not efficient
template <typename S, typename DerivedA, typename DerivedB>
FCL_EXPORT
bool overlap(
    const Eigen::MatrixBase<DerivedA>& R0,
    const Eigen::MatrixBase<DerivedB>& T0,
    const kIOS<S>& b1, const kIOS<S>& b2);

/// @brief Check collision between two kIOSs, b1 is in configuration (R0, T0)
/// and b2 is in identity.
/// @todo Not efficient
template <typename S>
FCL_EXPORT
bool overlap(
    const Transform3<S>& tf,
    const kIOS<S>& b1,
    const kIOS<S>& b2);

/// @brief Approximate distance between two kIOS bounding volumes
/// @todo P and Q is not returned, need implementation
template <typename S, typename DerivedA, typename DerivedB>
FCL_EXPORT
S distance(
    const Eigen::MatrixBase<DerivedA>& R0,
    const Eigen::MatrixBase<DerivedB>& T0,
    const kIOS<S>& b1,
    const kIOS<S>& b2,
    Vector3<S>* P = nullptr,
    Vector3<S>* Q = nullptr);

/// @brief Approximate distance between two kIOS bounding volumes
/// @todo P and Q is not returned, need implementation
template <typename S>
FCL_EXPORT
S distance(
    const Transform3<S>& tf,
    const kIOS<S>& b1,
    const kIOS<S>& b2,
    Vector3<S>* P = nullptr,
    Vector3<S>* Q = nullptr);

} // namespace fcl

#include "fcl/math/bv/kIOS-inl.h"

#endif

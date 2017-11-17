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

#ifndef FCL_BV_OBBRSS_H
#define FCL_BV_OBBRSS_H

#include "fcl/math/bv/OBB.h"
#include "fcl/math/bv/RSS.h"

namespace fcl
{

/// @brief Class merging the OBB and RSS, can handle collision and distance
/// simultaneously
template <typename S_>
class FCL_EXPORT OBBRSS
{
public:

  using S = S_;

  /// @brief OBB member, for rotation
  OBB<S> obb;

  /// @brief RSS member, for distance
  RSS<S> rss;

  /// @brief Check collision between two OBBRSS
  bool overlap(const OBBRSS<S>& other) const;

  /// @brief Check collision between two OBBRSS and return the overlap part.
  bool overlap(const OBBRSS<S>& other, OBBRSS<S>& overlap_part) const;

  /// @brief Check whether the OBBRSS contains a point
  bool contain(const Vector3<S>& p) const;

  /// @brief Merge the OBBRSS and a point
  OBBRSS<S>& operator += (const Vector3<S>& p);

  /// @brief Merge two OBBRSS
  OBBRSS<S>& operator += (const OBBRSS<S>& other);

  /// @brief Merge two OBBRSS
  OBBRSS<S> operator + (const OBBRSS<S>& other) const;

  /// @brief Width of the OBRSS
  S width() const;

  /// @brief Height of the OBBRSS
  S height() const;

  /// @brief Depth of the OBBRSS
  S depth() const;

  /// @brief Volume of the OBBRSS
  S volume() const;

  /// @brief Size of the OBBRSS (used in BV_Splitter to order two OBBRSS)
  S size() const;

  /// @brief Center of the OBBRSS
  const Vector3<S> center() const;

  /// @brief Distance between two OBBRSS; P and Q , is not nullptr, returns the nearest points
  S distance(const OBBRSS<S>& other,
                  Vector3<S>* P = nullptr, Vector3<S>* Q = nullptr) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

using OBBRSSf = OBBRSS<float>;
using OBBRSSd = OBBRSS<double>;

/// @brief Translate the OBBRSS bv
template <typename S>
FCL_EXPORT
OBBRSS<S> translate(const OBBRSS<S>& bv, const Vector3<S>& t);

/// @brief Check collision between two OBBRSS, b1 is in configuration (R0, T0)
/// and b2 is in indentity
template <typename S, typename DerivedA, typename DerivedB>
FCL_EXPORT
bool overlap(const Eigen::MatrixBase<DerivedA>& R0,
             const Eigen::MatrixBase<DerivedB>& T0,
             const OBBRSS<S>& b1, const OBBRSS<S>& b2);

/// @brief Check collision between two OBBRSS, b1 is in configuration (R0, T0)
/// and b2 is in indentity
template <typename S>
FCL_EXPORT
bool overlap(
    const Transform3<S>& tf,
    const OBBRSS<S>& b1,
    const OBBRSS<S>& b2);

/// @brief Computate distance between two OBBRSS, b1 is in configuation (R0, T0)
/// and b2 is in indentity; P and Q, is not nullptr, returns the nearest points
template <typename S, typename DerivedA, typename DerivedB>
FCL_EXPORT
S distance(
    const Eigen::MatrixBase<DerivedA>& R0,
    const Eigen::MatrixBase<DerivedB>& T0,
    const OBBRSS<S>& b1, const OBBRSS<S>& b2,
    Vector3<S>* P = nullptr, Vector3<S>* Q = nullptr);

/// @brief Computate distance between two OBBRSS, b1 is in configuation (R0, T0)
/// and b2 is in indentity; P and Q, is not nullptr, returns the nearest points
template <typename S>
FCL_EXPORT
S distance(
    const Transform3<S>& tf,
    const OBBRSS<S>& b1,
    const OBBRSS<S>& b2,
    Vector3<S>* P = nullptr,
    Vector3<S>* Q = nullptr);

} // namespace fcl

#include "fcl/math/bv/OBBRSS-inl.h"

#endif

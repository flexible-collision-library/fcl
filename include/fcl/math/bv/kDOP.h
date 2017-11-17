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

#ifndef FCL_BV_KDOP_H
#define FCL_BV_KDOP_H

#include <cstddef>
#include <iostream>

#include "fcl/common/types.h"

namespace fcl
{

/// @brief KDOP class describes the KDOP collision structures. K is set as the template parameter, which should be 16, 18, or 24
///  The KDOP structure is defined by some pairs of parallel planes defined by some axes. 
/// For K = 16, the planes are 6 AABB planes and 10 diagonal planes that cut off some space of the edges:
/// (-1,0,0) and (1,0,0)  -> indices 0 and 8
/// (0,-1,0) and (0,1,0)  -> indices 1 and 9
/// (0,0,-1) and (0,0,1)  -> indices 2 and 10
/// (-1,-1,0) and (1,1,0) -> indices 3 and 11
/// (-1,0,-1) and (1,0,1) -> indices 4 and 12
/// (0,-1,-1) and (0,1,1) -> indices 5 and 13
/// (-1,1,0) and (1,-1,0) -> indices 6 and 14
/// (-1,0,1) and (1,0,-1) -> indices 7 and 15
/// For K = 18, the planes are 6 AABB planes and 12 diagonal planes that cut off some space of the edges:
/// (-1,0,0) and (1,0,0)  -> indices 0 and 9
/// (0,-1,0) and (0,1,0)  -> indices 1 and 10
/// (0,0,-1) and (0,0,1)  -> indices 2 and 11
/// (-1,-1,0) and (1,1,0) -> indices 3 and 12
/// (-1,0,-1) and (1,0,1) -> indices 4 and 13
/// (0,-1,-1) and (0,1,1) -> indices 5 and 14
/// (-1,1,0) and (1,-1,0) -> indices 6 and 15
/// (-1,0,1) and (1,0,-1) -> indices 7 and 16
/// (0,-1,1) and (0,1,-1) -> indices 8 and 17
/// For K = 18, the planes are 6 AABB planes and 18 diagonal planes that cut off some space of the edges:
/// (-1,0,0) and (1,0,0)  -> indices 0 and 12
/// (0,-1,0) and (0,1,0)  -> indices 1 and 13
/// (0,0,-1) and (0,0,1)  -> indices 2 and 14
/// (-1,-1,0) and (1,1,0) -> indices 3 and 15
/// (-1,0,-1) and (1,0,1) -> indices 4 and 16
/// (0,-1,-1) and (0,1,1) -> indices 5 and 17
/// (-1,1,0) and (1,-1,0) -> indices 6 and 18
/// (-1,0,1) and (1,0,-1) -> indices 7 and 19
/// (0,-1,1) and (0,1,-1) -> indices 8 and 20
/// (-1, -1, 1) and (1, 1, -1) --> indices 9 and 21
/// (-1, 1, -1) and (1, -1, 1) --> indices 10 and 22
/// (1, -1, -1) and (-1, 1, 1) --> indices 11 and 23
template <typename S_, std::size_t N>
class FCL_EXPORT KDOP
{
public:

  using S = S_;

  /// @brief Creating kDOP containing nothing
  KDOP();

  /// @brief Creating kDOP containing only one point
  KDOP(const Vector3<S>& v);

  /// @brief Creating kDOP containing two points
  KDOP(const Vector3<S>& a, const Vector3<S>& b);
  
  /// @brief Check whether two KDOPs are overlapped
  bool overlap(const KDOP<S, N>& other) const;

  //// @brief Check whether one point is inside the KDOP
  bool inside(const Vector3<S>& p) const;

  /// @brief Merge the point and the KDOP
  KDOP<S, N>& operator += (const Vector3<S>& p);

  /// @brief Merge two KDOPs
  KDOP<S, N>& operator += (const KDOP<S, N>& other);

  /// @brief Create a KDOP by mergin two KDOPs
  KDOP<S, N> operator + (const KDOP<S, N>& other) const;

  /// @brief The (AABB) width
  S width() const;

  /// @brief The (AABB) height
  S height() const;

  /// @brief The (AABB) depth
  S depth() const;

  /// @brief The (AABB) volume
  S volume() const;

  /// @brief Size of the kDOP (used in BV_Splitter to order two kDOPs)
  S size() const;

  /// @brief The (AABB) center
  Vector3<S> center() const;

  /// @brief The distance between two KDOP<S, N>. Not implemented.
  S distance(
      const KDOP<S, N>& other,
      Vector3<S>* P = nullptr, Vector3<S>* Q = nullptr) const;

private:
  /// @brief Origin's distances to N KDOP planes
  S dist_[N];

public:
  S dist(std::size_t i) const;

  S& dist(std::size_t i);

};

template <std::size_t N>
using KDOPf = KDOP<float, N>;
template <std::size_t N>
using KDOPd = KDOP<double, N>;

/// @brief Find the smaller and larger one of two values
template <typename S>
FCL_EXPORT
void minmax(S a, S b, S& minv, S& maxv);

/// @brief Merge the interval [minv, maxv] and value p/
template <typename S>
FCL_EXPORT
void minmax(S p, S& minv, S& maxv);

/// @brief Compute the distances to planes with normals from KDOP vectors except
/// those of AABB face planes
template <typename S, std::size_t N>
FCL_EXPORT
void getDistances(const Vector3<S>& p, S* d);

/// @brief translate the KDOP BV
template <typename S, std::size_t N, typename Derived>
FCL_EXPORT
KDOP<S, N> translate(
    const KDOP<S, N>& bv, const Eigen::MatrixBase<Derived>& t);

} // namespace fcl

#include "fcl/math/bv/kDOP-inl.h"

#endif

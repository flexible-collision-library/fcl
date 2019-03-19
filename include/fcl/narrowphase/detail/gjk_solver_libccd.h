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

#ifndef FCL_NARROWPHASE_GJKSOLVERLIBCCD_H
#define FCL_NARROWPHASE_GJKSOLVERLIBCCD_H

#include <iostream>

#include "fcl/common/types.h"
#include "fcl/narrowphase/contact_point.h"

namespace fcl
{

namespace detail
{

/// @brief collision and distance solver based on libccd library.
template <typename S_>
struct FCL_EXPORT GJKSolver_libccd
{
  using S = S_;

  /// @brief intersection checking between two shapes
  /// @deprecated use shapeIntersect(const Shape1&, const Transform3<S>&, const Shape2&, const Transform3<S>&, std::vector<ContactPoint<S>>*) const
  template<typename Shape1, typename Shape2>
  FCL_DEPRECATED
  bool shapeIntersect(
      const Shape1& s1,
      const Transform3<S>& tf1,
      const Shape2& s2,
      const Transform3<S>& tf2,
      Vector3<S>* contact_points,
      S* penetration_depth,
      Vector3<S>* normal) const;

  /// @brief intersection checking between two shapes
  template<typename Shape1, typename Shape2>
  bool shapeIntersect(
      const Shape1& s1,
      const Transform3<S>& tf1,
      const Shape2& s2,
      const Transform3<S>& tf2,
      std::vector<ContactPoint<S>>* contacts = nullptr) const;

  /// @brief intersection checking between one shape and a triangle
  template<typename Shape>
  bool shapeTriangleIntersect(
      const Shape& s,
      const Transform3<S>& tf,
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      Vector3<S>* contact_points = nullptr,
      S* penetration_depth = nullptr,
      Vector3<S>* normal = nullptr) const;

  //// @brief intersection checking between one shape and a triangle with transformation
  template<typename Shape>
  bool shapeTriangleIntersect(
      const Shape& s,
      const Transform3<S>& tf1,
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      const Transform3<S>& tf2,
      Vector3<S>* contact_points = nullptr,
      S* penetration_depth = nullptr,
      Vector3<S>* normal = nullptr) const;

  /// @brief distance computation between two shapes
  template<typename Shape1, typename Shape2>
  bool shapeDistance(
      const Shape1& s1,
      const Transform3<S>& tf1,
      const Shape2& s2,
      const Transform3<S>& tf2,
      S* dist = nullptr,
      Vector3<S>* p1 = nullptr,
      Vector3<S>* p2 = nullptr) const;


  template<typename Shape1, typename Shape2>
  bool shapeSignedDistance(
      const Shape1& s1,
      const Transform3<S>& tf1,
      const Shape2& s2,
      const Transform3<S>& tf2,
      S* dist = nullptr,
      Vector3<S>* p1 = nullptr,
      Vector3<S>* p2 = nullptr) const;

  /// @brief distance computation between one shape and a triangle
  template<typename Shape>
  bool shapeTriangleDistance(
      const Shape& s,
      const Transform3<S>& tf,
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      S* dist = nullptr,
      Vector3<S>* p1 = nullptr,
      Vector3<S>* p2 = nullptr) const;

  /// @brief distance computation between one shape and a triangle with transformation
  template<typename Shape>
  bool shapeTriangleDistance(
      const Shape& s,
      const Transform3<S>& tf1,
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      const Transform3<S>& tf2,
      S* dist = nullptr,
      Vector3<S>* p1 = nullptr,
      Vector3<S>* p2 = nullptr) const;

  /// @brief default setting for GJK algorithm
  GJKSolver_libccd();

  void enableCachedGuess(bool if_enable) const;

  void setCachedGuess(const Vector3<S>& guess) const;

  Vector3<S> getCachedGuess() const;

  /// @brief maximum number of iterations used in GJK algorithm for collision
  unsigned int max_collision_iterations;

  /// @brief maximum number of iterations used in GJK algorithm for distance
  unsigned int max_distance_iterations;

  /// @brief the threshold used in GJK algorithm to stop collision iteration
  S collision_tolerance;

  /// @brief the threshold used in GJK algorithm to stop distance iteration
  S distance_tolerance;

  friend
  std::ostream& operator<<(std::ostream& out, const GJKSolver_libccd& solver) {
    out << "GjkSolver_libccd"
        << "\n    collision_tolerance:      " << solver.collision_tolerance
        << "\n    max collision iterations: " << solver.max_collision_iterations
        << "\n    distance tolerance:       " << solver.distance_tolerance
        << "\n    max distance iterations:  " << solver.max_distance_iterations;
    // NOTE: Cached guesses are not supported.
    return out;
  }

};

using GJKSolver_libccdf = GJKSolver_libccd<float>;
using GJKSolver_libccdd = GJKSolver_libccd<double>;

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/gjk_solver_libccd-inl.h"

#endif

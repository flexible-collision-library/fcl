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

#ifndef FCL_NARROWPHASE_DETAIL_GJK_H
#define FCL_NARROWPHASE_DETAIL_GJK_H

#include "fcl/common/types.h"
#include "fcl/narrowphase/detail/convexity_based_algorithm/minkowski_diff.h"

namespace fcl
{

namespace detail
{

/// @brief class for GJK algorithm
template <typename S>
struct FCL_EXPORT GJK
{
  struct SimplexV
  {
    /// @brief support direction
    Vector3<S> d;
    /// @brieg support vector (i.e., the furthest point on the shape along the support direction)
    Vector3<S> w;
  };

  struct Simplex
  {
    /// @brief simplex vertex
    SimplexV* c[4];
    /// @brief weight 
    S p[4];
    /// @brief size of simplex (number of vertices)
    size_t rank;

    Simplex();
  };

  enum Status {Valid, Inside, Failed};

  MinkowskiDiff<S> shape;
  Vector3<S> ray;
  S distance;
  Simplex simplices[2];

  GJK(unsigned int max_iterations_, S tolerance_);
  
  void initialize();

  /// @brief GJK algorithm, given the initial value guess
  Status evaluate(const MinkowskiDiff<S>& shape_, const Vector3<S>& guess);

  /// @brief apply the support function along a direction, the result is return in sv
  void getSupport(const Vector3<S>& d, SimplexV& sv) const;

  /// @brief apply the support function along a direction, the result is return is sv, here shape0 is translating at velocity v
  void getSupport(const Vector3<S>& d, const Vector3<S>& v, SimplexV& sv) const;

  /// @brief discard one vertex from the simplex
  void removeVertex(Simplex& simplex);

  /// @brief append one vertex to the simplex
  void appendVertex(Simplex& simplex, const Vector3<S>& v);

  /// @brief whether the simplex enclose the origin
  bool encloseOrigin();

  /// @brief get the underlying simplex using in GJK, can be used for cache in next iteration
  Simplex* getSimplex() const;

  /// @brief get the guess from current simplex
  Vector3<S> getGuessFromSimplex() const;

private:
  SimplexV store_v[4];
  SimplexV* free_v[4];
  size_t nfree;
  size_t current;
  Simplex* simplex;
  Status status;

  unsigned int max_iterations;
  S tolerance;

};

using GJKf = GJK<float>;
using GJKd = GJK<double>;

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/convexity_based_algorithm/gjk-inl.h"

#endif

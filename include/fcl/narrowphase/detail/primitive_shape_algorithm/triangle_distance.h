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

#ifndef FCL_NARROWPHASE_DETAIL_TRIANGLEDISTANCE_H
#define FCL_NARROWPHASE_DETAIL_TRIANGLEDISTANCE_H

#include "fcl/common/types.h"

namespace fcl
{

namespace detail
{

/// @brief Triangle distance functions
template <typename S>
class FCL_EXPORT TriangleDistance
{
public:

  /// @brief Returns closest points between an segment pair.
  /// The first segment is P + t * A
  /// The second segment is Q + t * B
  /// X, Y are the closest points on the two segments
  /// VEC is the vector between X and Y
  static void segPoints(const Vector3<S>& P, const Vector3<S>& A, const Vector3<S>& Q, const Vector3<S>& B,
                        Vector3<S>& VEC, Vector3<S>& X, Vector3<S>& Y);

  /// @brief Compute the closest points on two triangles given their absolute coordinate, and returns the distance between them
  /// T1 and T2 are two triangles
  /// If the triangles are disjoint, P and Q give the closet points of T1 and T2 respectively. However,
  /// if the triangles overlap, P and Q are basically a random pair of points from the triangles, not
  /// coincident points on the intersection of the triangles, as might be expected.
  static S triDistance(const Vector3<S> T1[3], const Vector3<S> T2[3], Vector3<S>& P, Vector3<S>& Q);

  static S triDistance(const Vector3<S>& S1, const Vector3<S>& S2, const Vector3<S>& S3,
                              const Vector3<S>& T1, const Vector3<S>& T2, const Vector3<S>& T3,
                              Vector3<S>& P, Vector3<S>& Q);

  /// @brief Compute the closest points on two triangles given the relative transform between them, and returns the distance between them
  /// T1 and T2 are two triangles
  /// If the triangles are disjoint, P and Q give the closet points of T1 and T2 respectively. However,
  /// if the triangles overlap, P and Q are basically a random pair of points from the triangles, not
  /// coincident points on the intersection of the triangles, as might be expected.
  /// The returned P and Q are both in the coordinate of the first triangle's coordinate
  static S triDistance(const Vector3<S> T1[3], const Vector3<S> T2[3],
                              const Matrix3<S>& R, const Vector3<S>& Tl,
                              Vector3<S>& P, Vector3<S>& Q);

  static S triDistance(const Vector3<S> T1[3], const Vector3<S> T2[3],
                              const Transform3<S>& tf,
                              Vector3<S>& P, Vector3<S>& Q);

  static S triDistance(const Vector3<S>& S1, const Vector3<S>& S2, const Vector3<S>& S3,
                              const Vector3<S>& T1, const Vector3<S>& T2, const Vector3<S>& T3,
                              const Matrix3<S>& R, const Vector3<S>& Tl,
                              Vector3<S>& P, Vector3<S>& Q);

  static S triDistance(
      const Vector3<S>& S1,
      const Vector3<S>& S2,
      const Vector3<S>& S3,
      const Vector3<S>& T1,
      const Vector3<S>& T2,
      const Vector3<S>& T3,
      const Transform3<S>& tf,
      Vector3<S>& P,
      Vector3<S>& Q);

};

using TriangleDistancef = TriangleDistance<float>;
using TriangleDistanced = TriangleDistance<double>;

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/primitive_shape_algorithm/triangle_distance-inl.h"

#endif

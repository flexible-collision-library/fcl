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

/** \author Jia Pan */


#ifndef FCL_SHAPE_GET_BOUND_VERTICES_H
#define FCL_SHAPE_GET_BOUND_VERTICES_H

#include <vector>

#include "fcl/shape/box.h"
#include "fcl/shape/capsule.h"
#include "fcl/shape/cone.h"
#include "fcl/shape/convex.h"
#include "fcl/shape/cylinder.h"
#include "fcl/shape/ellipsoid.h"
#include "fcl/shape/halfspace.h"
#include "fcl/shape/plane.h"
#include "fcl/shape/sphere.h"
#include "fcl/shape/triangle_p.h"

namespace fcl
{

/// @cond IGNORE
namespace details
{
/// @brief get the vertices of some convex shape which can bound the given shape
/// in a specific configuration
template <typename Scalar>
std::vector<Vector3<Scalar>> getBoundVertices(
    const Box<Scalar>& box, const Transform3<Scalar>& tf);

template <typename Scalar>
std::vector<Vector3<Scalar>> getBoundVertices(
    const Sphere<Scalar>& sphere, const Transform3<Scalar>& tf);

template <typename Scalar>
std::vector<Vector3<Scalar>> getBoundVertices(
    const Ellipsoid<Scalar>& ellipsoid, const Transform3<Scalar>& tf);

template <typename Scalar>
std::vector<Vector3<Scalar>> getBoundVertices(
    const Capsule<Scalar>& capsule, const Transform3<Scalar>& tf);

template <typename Scalar>
std::vector<Vector3<Scalar>> getBoundVertices(
    const Cone<Scalar>& cone, const Transform3<Scalar>& tf);

template <typename Scalar>
std::vector<Vector3<Scalar>> getBoundVertices(
    const Cylinder<Scalar>& cylinder, const Transform3<Scalar>& tf);

template <typename Scalar>
std::vector<Vector3<Scalar>> getBoundVertices(
    const Convex<Scalar>& convex, const Transform3<Scalar>& tf);

template <typename Scalar>
std::vector<Vector3<Scalar>> getBoundVertices(
    const TriangleP<Scalar>& triangle, const Transform3<Scalar>& tf);
} 
/// @endcond

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

namespace details
{

//==============================================================================
template <typename Scalar>
std::vector<Vector3<Scalar>> getBoundVertices(
    const Box<Scalar>& box, const Transform3<Scalar>& tf)
{
  std::vector<Vector3<Scalar>> result(8);
  auto a = box.side[0] / 2;
  auto b = box.side[1] / 2;
  auto c = box.side[2] / 2;
  result[0] = tf * Vector3<Scalar>(a, b, c);
  result[1] = tf * Vector3<Scalar>(a, b, -c);
  result[2] = tf * Vector3<Scalar>(a, -b, c);
  result[3] = tf * Vector3<Scalar>(a, -b, -c);
  result[4] = tf * Vector3<Scalar>(-a, b, c);
  result[5] = tf * Vector3<Scalar>(-a, b, -c);
  result[6] = tf * Vector3<Scalar>(-a, -b, c);
  result[7] = tf * Vector3<Scalar>(-a, -b, -c);

  return result;
}

//==============================================================================
template <typename Scalar>
std::vector<Vector3<Scalar>> getBoundVertices(
    const Sphere<Scalar>& sphere, const Transform3<Scalar>& tf)
{
  // we use icosahedron to bound the sphere

  std::vector<Vector3<Scalar>> result(12);
  const auto m = (1 + std::sqrt(5.0)) / 2.0;
  auto edge_size = sphere.radius * 6 / (std::sqrt(27.0) + std::sqrt(15.0));

  auto a = edge_size;
  auto b = m * edge_size;
  result[0] = tf * Vector3<Scalar>(0, a, b);
  result[1] = tf * Vector3<Scalar>(0, -a, b);
  result[2] = tf * Vector3<Scalar>(0, a, -b);
  result[3] = tf * Vector3<Scalar>(0, -a, -b);
  result[4] = tf * Vector3<Scalar>(a, b, 0);
  result[5] = tf * Vector3<Scalar>(-a, b, 0);
  result[6] = tf * Vector3<Scalar>(a, -b, 0);
  result[7] = tf * Vector3<Scalar>(-a, -b, 0);
  result[8] = tf * Vector3<Scalar>(b, 0, a);
  result[9] = tf * Vector3<Scalar>(b, 0, -a);
  result[10] = tf * Vector3<Scalar>(-b, 0, a);
  result[11] = tf * Vector3<Scalar>(-b, 0, -a);

  return result;
}

//==============================================================================
template <typename Scalar>
std::vector<Vector3<Scalar>> getBoundVertices(
    const Ellipsoid<Scalar>& ellipsoid, const Transform3<Scalar>& tf)
{
  // we use scaled icosahedron to bound the ellipsoid

  std::vector<Vector3<Scalar>> result(12);

  const auto phi = (1.0 + std::sqrt(5.0)) / 2.0;  // golden ratio

  const auto a = std::sqrt(3.0) / (phi * phi);
  const auto b = phi * a;

  const auto& A = ellipsoid.radii[0];
  const auto& B = ellipsoid.radii[1];
  const auto& C = ellipsoid.radii[2];

  const auto Aa = A * a;
  const auto Ab = A * b;
  const auto Ba = B * a;
  const auto Bb = B * b;
  const auto Ca = C * a;
  const auto Cb = C * b;

  result[0] = tf * Vector3<Scalar>(0, Ba, Cb);
  result[1] = tf * Vector3<Scalar>(0, -Ba, Cb);
  result[2] = tf * Vector3<Scalar>(0, Ba, -Cb);
  result[3] = tf * Vector3<Scalar>(0, -Ba, -Cb);
  result[4] = tf * Vector3<Scalar>(Aa, Bb, 0);
  result[5] = tf * Vector3<Scalar>(-Aa, Bb, 0);
  result[6] = tf * Vector3<Scalar>(Aa, -Bb, 0);
  result[7] = tf * Vector3<Scalar>(-Aa, -Bb, 0);
  result[8] = tf * Vector3<Scalar>(Ab, 0, Ca);
  result[9] = tf * Vector3<Scalar>(Ab, 0, -Ca);
  result[10] = tf * Vector3<Scalar>(-Ab, 0, Ca);
  result[11] = tf * Vector3<Scalar>(-Ab, 0, -Ca);

  return result;
}

//==============================================================================
template <typename Scalar>
std::vector<Vector3<Scalar>> getBoundVertices(
    const Capsule<Scalar>& capsule, const Transform3<Scalar>& tf)
{
  std::vector<Vector3<Scalar>> result(36);
  const auto m = (1 + std::sqrt(5.0)) / 2.0;

  auto hl = capsule.lz * 0.5;
  auto edge_size = capsule.radius * 6 / (std::sqrt(27.0) + std::sqrt(15.0));
  auto a = edge_size;
  auto b = m * edge_size;
  auto r2 = capsule.radius * 2 / std::sqrt(3.0);

  result[0] = tf * Vector3<Scalar>(0, a, b + hl);
  result[1] = tf * Vector3<Scalar>(0, -a, b + hl);
  result[2] = tf * Vector3<Scalar>(0, a, -b + hl);
  result[3] = tf * Vector3<Scalar>(0, -a, -b + hl);
  result[4] = tf * Vector3<Scalar>(a, b, hl);
  result[5] = tf * Vector3<Scalar>(-a, b, hl);
  result[6] = tf * Vector3<Scalar>(a, -b, hl);
  result[7] = tf * Vector3<Scalar>(-a, -b, hl);
  result[8] = tf * Vector3<Scalar>(b, 0, a + hl);
  result[9] = tf * Vector3<Scalar>(b, 0, -a + hl);
  result[10] = tf * Vector3<Scalar>(-b, 0, a + hl);
  result[11] = tf * Vector3<Scalar>(-b, 0, -a + hl);

  result[12] = tf * Vector3<Scalar>(0, a, b - hl);
  result[13] = tf * Vector3<Scalar>(0, -a, b - hl);
  result[14] = tf * Vector3<Scalar>(0, a, -b - hl);
  result[15] = tf * Vector3<Scalar>(0, -a, -b - hl);
  result[16] = tf * Vector3<Scalar>(a, b, -hl);
  result[17] = tf * Vector3<Scalar>(-a, b, -hl);
  result[18] = tf * Vector3<Scalar>(a, -b, -hl);
  result[19] = tf * Vector3<Scalar>(-a, -b, -hl);
  result[20] = tf * Vector3<Scalar>(b, 0, a - hl);
  result[21] = tf * Vector3<Scalar>(b, 0, -a - hl);
  result[22] = tf * Vector3<Scalar>(-b, 0, a - hl);
  result[23] = tf * Vector3<Scalar>(-b, 0, -a - hl);

  auto c = 0.5 * r2;
  auto d = capsule.radius;
  result[24] = tf * Vector3<Scalar>(r2, 0, hl);
  result[25] = tf * Vector3<Scalar>(c, d, hl);
  result[26] = tf * Vector3<Scalar>(-c, d, hl);
  result[27] = tf * Vector3<Scalar>(-r2, 0, hl);
  result[28] = tf * Vector3<Scalar>(-c, -d, hl);
  result[29] = tf * Vector3<Scalar>(c, -d, hl);

  result[30] = tf * Vector3<Scalar>(r2, 0, -hl);
  result[31] = tf * Vector3<Scalar>(c, d, -hl);
  result[32] = tf * Vector3<Scalar>(-c, d, -hl);
  result[33] = tf * Vector3<Scalar>(-r2, 0, -hl);
  result[34] = tf * Vector3<Scalar>(-c, -d, -hl);
  result[35] = tf * Vector3<Scalar>(c, -d, -hl);

  return result;
}

//==============================================================================
template <typename Scalar>
std::vector<Vector3<Scalar>> getBoundVertices(
    const Cone<Scalar>& cone, const Transform3<Scalar>& tf)
{
  std::vector<Vector3<Scalar>> result(7);

  auto hl = cone.lz * 0.5;
  auto r2 = cone.radius * 2 / std::sqrt(3.0);
  auto a = 0.5 * r2;
  auto b = cone.radius;

  result[0] = tf * Vector3<Scalar>(r2, 0, -hl);
  result[1] = tf * Vector3<Scalar>(a, b, -hl);
  result[2] = tf * Vector3<Scalar>(-a, b, -hl);
  result[3] = tf * Vector3<Scalar>(-r2, 0, -hl);
  result[4] = tf * Vector3<Scalar>(-a, -b, -hl);
  result[5] = tf * Vector3<Scalar>(a, -b, -hl);

  result[6] = tf * Vector3<Scalar>(0, 0, hl);

  return result;
}

//==============================================================================
template <typename Scalar>
std::vector<Vector3<Scalar>> getBoundVertices(
    const Cylinder<Scalar>& cylinder, const Transform3<Scalar>& tf)
{
  std::vector<Vector3<Scalar>> result(12);

  auto hl = cylinder.lz * 0.5;
  auto r2 = cylinder.radius * 2 / std::sqrt(3.0);
  auto a = 0.5 * r2;
  auto b = cylinder.radius;

  result[0] = tf * Vector3<Scalar>(r2, 0, -hl);
  result[1] = tf * Vector3<Scalar>(a, b, -hl);
  result[2] = tf * Vector3<Scalar>(-a, b, -hl);
  result[3] = tf * Vector3<Scalar>(-r2, 0, -hl);
  result[4] = tf * Vector3<Scalar>(-a, -b, -hl);
  result[5] = tf * Vector3<Scalar>(a, -b, -hl);

  result[6] = tf * Vector3<Scalar>(r2, 0, hl);
  result[7] = tf * Vector3<Scalar>(a, b, hl);
  result[8] = tf * Vector3<Scalar>(-a, b, hl);
  result[9] = tf * Vector3<Scalar>(-r2, 0, hl);
  result[10] = tf * Vector3<Scalar>(-a, -b, hl);
  result[11] = tf * Vector3<Scalar>(a, -b, hl);

  return result;
}

//==============================================================================
template <typename Scalar>
std::vector<Vector3<Scalar>> getBoundVertices(
    const Convex<Scalar>& convex, const Transform3<Scalar>& tf)
{
  std::vector<Vector3<Scalar>> result(convex.num_points);
  for(int i = 0; i < convex.num_points; ++i)
  {
    result[i] = tf * convex.points[i];
  }

  return result;
}

//==============================================================================
template <typename Scalar>
std::vector<Vector3<Scalar>> getBoundVertices(
    const TriangleP<Scalar>& triangle, const Transform3<Scalar>& tf)
{
  std::vector<Vector3<Scalar>> result(3);
  result[0] = tf * triangle.a;
  result[1] = tf * triangle.b;
  result[2] = tf * triangle.c;

  return result;
}

} // end detail


}

#endif

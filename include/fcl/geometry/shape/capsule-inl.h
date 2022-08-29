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

#ifndef FCL_SHAPE_CAPSULE_INL_H
#define FCL_SHAPE_CAPSULE_INL_H

#include "fcl/geometry/shape/capsule.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT Capsule<double>;

//==============================================================================
template <typename S>
Capsule<S>::Capsule(S radius, S lz)
  : ShapeBase<S>(), radius(radius), lz(lz), height(0.5 * lz)
{
  // Do nothing
}

//==============================================================================
template <typename S>
void Capsule<S>::computeLocalAABB()
{
  const Vector3<S> v_delta(radius, radius, height + radius);
  this->aabb_local.max_ = v_delta;
  this->aabb_local.min_ = -v_delta;

  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
}

//==============================================================================
template <typename S>
NODE_TYPE Capsule<S>::getNodeType() const
{
  return GEOM_CAPSULE;
}

//==============================================================================
template <typename S>
S Capsule<S>::computeVolume() const
{
  return constants<S>::pi() * radius * radius *(lz + radius * 4.0/3.0);
}

//==============================================================================
// Compare https://www.gamedev.net/articles/programming/math-and-physics/capsule-inertia-tensor-r3856/
template <typename S>
Matrix3<S> Capsule<S>::computeMomentofInertia() const
{
  S l2 = lz * lz;
  S r2 = radius * radius;

  S v_cyl = r2 * lz * constants<S>::pi();
  S v_sph = r2 * radius * constants<S>::pi() * 4 / 3.0;

  S ix = v_cyl * (l2 / 12. + r2 / 4.) + v_sph * (0.4 * r2 + 0.25 * l2 + 3. * radius * lz / 8.);
  S iz = (0.5 * v_cyl + 0.4 * v_sph) * r2;

  return Vector3<S>(ix, ix, iz).asDiagonal();
}

//==============================================================================
template <typename S>
std::vector<Vector3<S>> Capsule<S>::getBoundVertices(
    const Transform3<S>& tf) const
{
  std::vector<Vector3<S>> result(36);
  const auto m = (1 + std::sqrt(5.0)) / 2.0;

  auto hl = height;
  auto edge_size = radius * 6 / (std::sqrt(27.0) + std::sqrt(15.0));
  auto a = edge_size;
  auto b = m * edge_size;
  auto r2 = radius * 2 / std::sqrt(3.0);

  result[0] = tf * Vector3<S>(0, a, b + hl);
  result[1] = tf * Vector3<S>(0, -a, b + hl);
  result[2] = tf * Vector3<S>(0, a, -b + hl);
  result[3] = tf * Vector3<S>(0, -a, -b + hl);
  result[4] = tf * Vector3<S>(a, b, hl);
  result[5] = tf * Vector3<S>(-a, b, hl);
  result[6] = tf * Vector3<S>(a, -b, hl);
  result[7] = tf * Vector3<S>(-a, -b, hl);
  result[8] = tf * Vector3<S>(b, 0, a + hl);
  result[9] = tf * Vector3<S>(b, 0, -a + hl);
  result[10] = tf * Vector3<S>(-b, 0, a + hl);
  result[11] = tf * Vector3<S>(-b, 0, -a + hl);

  result[12] = tf * Vector3<S>(0, a, b - hl);
  result[13] = tf * Vector3<S>(0, -a, b - hl);
  result[14] = tf * Vector3<S>(0, a, -b - hl);
  result[15] = tf * Vector3<S>(0, -a, -b - hl);
  result[16] = tf * Vector3<S>(a, b, -hl);
  result[17] = tf * Vector3<S>(-a, b, -hl);
  result[18] = tf * Vector3<S>(a, -b, -hl);
  result[19] = tf * Vector3<S>(-a, -b, -hl);
  result[20] = tf * Vector3<S>(b, 0, a - hl);
  result[21] = tf * Vector3<S>(b, 0, -a - hl);
  result[22] = tf * Vector3<S>(-b, 0, a - hl);
  result[23] = tf * Vector3<S>(-b, 0, -a - hl);

  auto c = 0.5 * r2;
  auto d = radius;
  result[24] = tf * Vector3<S>(r2, 0, hl);
  result[25] = tf * Vector3<S>(c, d, hl);
  result[26] = tf * Vector3<S>(-c, d, hl);
  result[27] = tf * Vector3<S>(-r2, 0, hl);
  result[28] = tf * Vector3<S>(-c, -d, hl);
  result[29] = tf * Vector3<S>(c, -d, hl);

  result[30] = tf * Vector3<S>(r2, 0, -hl);
  result[31] = tf * Vector3<S>(c, d, -hl);
  result[32] = tf * Vector3<S>(-c, d, -hl);
  result[33] = tf * Vector3<S>(-r2, 0, -hl);
  result[34] = tf * Vector3<S>(-c, -d, -hl);
  result[35] = tf * Vector3<S>(c, -d, -hl);

  return result;
}

template <typename S>
Vector3<S> Capsule<S>::localGetSupportingVertex(const Vector3<S>& vec) const
{
  Vector3<S> v = radius * vec;
  if (vec[2] >= 0.0)
      v[2] += height;
  else
      v[2] -= height;
  return v;
}
} // namespace fcl

#endif

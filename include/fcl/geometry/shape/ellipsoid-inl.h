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

#ifndef FCL_SHAPE_ELLIPSOID_INL_H
#define FCL_SHAPE_ELLIPSOID_INL_H

#include "fcl/geometry/shape/ellipsoid.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT Ellipsoid<double>;

//==============================================================================
template <typename S>
Ellipsoid<S>::Ellipsoid(S a, S b, S c)
  : ShapeBase<S>(), radii(a, b, c)
{
  // Do nothing
}

//==============================================================================
template <typename S>
Ellipsoid<S>::Ellipsoid(const Vector3<S>& radii)
  : ShapeBase<S>(), radii(radii)
{
  // Do nothing
}

//==============================================================================
template <typename S>
void Ellipsoid<S>::computeLocalAABB()
{
  this->aabb_local.max_ = radii;
  this->aabb_local.min_ = -radii;

  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
}

//==============================================================================
template <typename S>
NODE_TYPE Ellipsoid<S>::getNodeType() const
{
  return GEOM_ELLIPSOID;
}

//==============================================================================
template <typename S>
Matrix3<S> Ellipsoid<S>::computeMomentofInertia() const
{
  const S V = computeVolume();

  const S a2 = radii[0] * radii[0] * V;
  const S b2 = radii[1] * radii[1] * V;
  const S c2 = radii[2] * radii[2] * V;

  return Vector3<S>(0.2 * (b2 + c2), 0.2 * (a2 + c2), 0.2 * (a2 + b2)).asDiagonal();
}

//==============================================================================
template <typename S>
S Ellipsoid<S>::computeVolume() const
{
  const S pi = constants<S>::pi();
  return 4.0 * pi * radii[0] * radii[1] * radii[2] / 3.0;
}

//==============================================================================
template <typename S>
std::vector<Vector3<S>> Ellipsoid<S>::getBoundVertices(
    const Transform3<S>& tf) const
{
  // we use scaled icosahedron to bound the ellipsoid

  std::vector<Vector3<S>> result(12);

  const auto phi = (1.0 + std::sqrt(5.0)) / 2.0;  // golden ratio

  const auto a = std::sqrt(3.0) / (phi * phi);
  const auto b = phi * a;

  const auto& A = radii[0];
  const auto& B = radii[1];
  const auto& C = radii[2];

  const auto Aa = A * a;
  const auto Ab = A * b;
  const auto Ba = B * a;
  const auto Bb = B * b;
  const auto Ca = C * a;
  const auto Cb = C * b;

  result[0] = tf * Vector3<S>(0, Ba, Cb);
  result[1] = tf * Vector3<S>(0, -Ba, Cb);
  result[2] = tf * Vector3<S>(0, Ba, -Cb);
  result[3] = tf * Vector3<S>(0, -Ba, -Cb);
  result[4] = tf * Vector3<S>(Aa, Bb, 0);
  result[5] = tf * Vector3<S>(-Aa, Bb, 0);
  result[6] = tf * Vector3<S>(Aa, -Bb, 0);
  result[7] = tf * Vector3<S>(-Aa, -Bb, 0);
  result[8] = tf * Vector3<S>(Ab, 0, Ca);
  result[9] = tf * Vector3<S>(Ab, 0, -Ca);
  result[10] = tf * Vector3<S>(-Ab, 0, Ca);
  result[11] = tf * Vector3<S>(-Ab, 0, -Ca);

  return result;
}

} // namespace fcl

#endif

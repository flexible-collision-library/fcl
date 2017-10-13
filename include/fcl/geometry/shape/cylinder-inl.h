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

#ifndef FCL_SHAPE_CYLINDER_INL_H
#define FCL_SHAPE_CYLINDER_INL_H

#include "fcl/geometry/shape/cylinder.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT Cylinder<double>;

//==============================================================================
template <typename S>
Cylinder<S>::Cylinder(S radius, S lz)
  : ShapeBase<S>(), radius(radius), lz(lz)
{
  // Do nothing
}

//==============================================================================
template <typename S>
void Cylinder<S>::computeLocalAABB()
{
  const Vector3<S> v_delta(radius, radius, 0.5 * lz);
  this->aabb_local.max_ = v_delta;
  this->aabb_local.min_ = -v_delta;

  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
}

//==============================================================================
template <typename S>
NODE_TYPE Cylinder<S>::getNodeType() const
{
  return GEOM_CYLINDER;
}

//==============================================================================
template <typename S>
S Cylinder<S>::computeVolume() const
{
  return constants<S>::pi() * radius * radius * lz;
}

//==============================================================================
template <typename S>
Matrix3<S> Cylinder<S>::computeMomentofInertia() const
{
  S V = computeVolume();
  S ix = V * (3 * radius * radius + lz * lz) / 12;
  S iz = V * radius * radius / 2;

  return Vector3<S>(ix, ix, iz).asDiagonal();
}

//==============================================================================
template <typename S>
std::vector<Vector3<S>> Cylinder<S>::getBoundVertices(
    const Transform3<S>& tf) const
{
  std::vector<Vector3<S>> result(12);

  auto hl = lz * 0.5;
  auto r2 = radius * 2 / std::sqrt(3.0);
  auto a = 0.5 * r2;
  auto b = radius;

  result[0] = tf * Vector3<S>(r2, 0, -hl);
  result[1] = tf * Vector3<S>(a, b, -hl);
  result[2] = tf * Vector3<S>(-a, b, -hl);
  result[3] = tf * Vector3<S>(-r2, 0, -hl);
  result[4] = tf * Vector3<S>(-a, -b, -hl);
  result[5] = tf * Vector3<S>(a, -b, -hl);

  result[6] = tf * Vector3<S>(r2, 0, hl);
  result[7] = tf * Vector3<S>(a, b, hl);
  result[8] = tf * Vector3<S>(-a, b, hl);
  result[9] = tf * Vector3<S>(-r2, 0, hl);
  result[10] = tf * Vector3<S>(-a, -b, hl);
  result[11] = tf * Vector3<S>(a, -b, hl);

  return result;
}

} // namespace fcl

#endif

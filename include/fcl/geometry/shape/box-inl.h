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

#ifndef FCL_SHAPE_BOX_INL_H
#define FCL_SHAPE_BOX_INL_H

#include "fcl/geometry/shape/box.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT Box<double>;

//==============================================================================
template <typename S>
Box<S>::Box(S x, S y, S z)
  : ShapeBase<S>(), side(x, y, z), h_side(0.5 * x, 0.5 * y, 0.5 * z)
{
  // Do nothing
}

//==============================================================================
template <typename S>
Box<S>::Box(const Vector3<S>& side_)
  : ShapeBase<S>(), side(side_), h_side(0.5 * side_)
{
  // Do nothing
}

//==============================================================================
template <typename S>
Box<S>::Box()
  : ShapeBase<S>(), side(Vector3<S>::Zero()), h_side(Vector3<S>::Zero())
{
  // Do nothing
}

//==============================================================================
template <typename S>
void Box<S>::computeLocalAABB()
{
  this->aabb_local.max_ = h_side;
  this->aabb_local.min_ = -h_side;

  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
}

//==============================================================================
template <typename S>
NODE_TYPE Box<S>::getNodeType() const
{
  return GEOM_BOX;
}

//==============================================================================
template <typename S>
S Box<S>::computeVolume() const
{
  return side.prod();
}

//==============================================================================
template <typename S>
Matrix3<S> Box<S>::computeMomentofInertia() const
{
  S V = computeVolume();

  S a2 = side[0] * side[0] * V;
  S b2 = side[1] * side[1] * V;
  S c2 = side[2] * side[2] * V;

  Vector3<S> I((b2 + c2) / 12.0, (a2 + c2) / 12.0, (a2 + b2) / 12.0);

  return I.asDiagonal();
}

//==============================================================================
template <typename S>
std::vector<Vector3<S>> Box<S>::getBoundVertices(
    const Transform3<S>& tf) const
{
  std::vector<Vector3<S>> result(8);
  auto a = h_side[0];
  auto b = h_side[1];
  auto c = h_side[2];
  result[0] = tf * Vector3<S>(a, b, c);
  result[1] = tf * Vector3<S>(a, b, -c);
  result[2] = tf * Vector3<S>(a, -b, c);
  result[3] = tf * Vector3<S>(a, -b, -c);
  result[4] = tf * Vector3<S>(-a, b, c);
  result[5] = tf * Vector3<S>(-a, b, -c);
  result[6] = tf * Vector3<S>(-a, -b, c);
  result[7] = tf * Vector3<S>(-a, -b, -c);

  return result;
}

template <typename S>
Vector3<S> Box<S>::localGetSupportingVertex(const Vector3<S>& vec) const
{
  // Use a customized sign function, so that the support of the box always
  // appears in one of the box vertices.
  // Picking support vertices on the interior of faces/edges can lead to
  // degenerate triangles in the EPA algorithm and are no more correct than just
  // picking box corners.
  auto sign = [](S x) -> S {
    return x >= S(0.0) ? S(1.0) : S(-1.0);
  };
  return Vector3<S>(sign(vec[0]) * h_side[0], sign(vec[1]) * h_side[1], sign(vec[2]) * h_side[2]);
}

} // namespace fcl

#endif

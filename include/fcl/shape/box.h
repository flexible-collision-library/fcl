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

#ifndef FCL_SHAPE_BOX_H
#define FCL_SHAPE_BOX_H

#include "fcl/shape/shape_base.h"
#include "fcl/shape/detail/bv_computer.h"

namespace fcl
{

/// @brief Center at zero point, axis aligned box
template <typename S_>
class Box : public ShapeBase<S_>
{
public:

  using S = S_;

  /// @brief Constructor
  Box(S x, S y, S z);

  /// @brief Constructor
  Box(const Vector3<S>& side);

  /// @brief Constructor
  Box();

  /// @brief box side length
  Vector3<S> side;

  /// @brief Compute AABBd
  void computeLocalAABB() override;

  /// @brief Get node type: a box
  NODE_TYPE getNodeType() const override;

  // Documentation inherited
  S computeVolume() const override;

  // Documentation inherited
  Matrix3<S> computeMomentofInertia() const override;

  /// @brief get the vertices of some convex shape which can bound this shape in
  /// a specific configuration
  std::vector<Vector3<S>> getBoundVertices(
      const Transform3<S>& tf) const;
};

using Boxf = Box<float>;
using Boxd = Box<double>;

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename S>
Box<S>::Box(S x, S y, S z)
  : ShapeBase<S>(), side(x, y, z)
{
  // Do nothing
}

//==============================================================================
template <typename S>
Box<S>::Box(const Vector3<S>& side_)
  : ShapeBase<S>(), side(side_)
{
  // Do nothing
}

//==============================================================================
template <typename S>
Box<S>::Box()
  : ShapeBase<S>(), side(Vector3<S>::Zero())
{
  // Do nothing
}

//==============================================================================
template <typename S>
void Box<S>::computeLocalAABB()
{
  computeBV(*this, Transform3<S>::Identity(), this->aabb_local);
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

  Vector3<S> I((b2 + c2) / 12, (a2 + c2) / 12, (a2 + b2) / 12);

  return I.asDiagonal();
}

//==============================================================================
template <typename S>
std::vector<Vector3<S>> Box<S>::getBoundVertices(
    const Transform3<S>& tf) const
{
  std::vector<Vector3<S>> result(8);
  auto a = side[0] / 2;
  auto b = side[1] / 2;
  auto c = side[2] / 2;
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

} // namespace fcl

#include "fcl/shape/detail/bv_computer_box.h"

#endif

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
#include "fcl/shape/geometric_shapes_utility.h"

namespace fcl
{

/// @brief Center at zero point, axis aligned box
template <typename Scalar>
class Box : public ShapeBase<Scalar>
{
public:
  /// @brief Constructor
  Box(Scalar x, Scalar y, Scalar z);

  /// @brief Constructor
  Box(const Vector3<Scalar>& side);

  /// @brief Constructor
  Box();

  /// @brief box side length
  Vector3<Scalar> side;

  /// @brief Compute AABB
  void computeLocalAABB() override;

  /// @brief Get node type: a box
  NODE_TYPE getNodeType() const override;

  // Documentation inherited
  Scalar computeVolume() const override;

  // Documentation inherited
  Matrix3<Scalar> computeMomentofInertia() const override;
};

using Boxf = Box<float>;
using Boxd = Box<double>;

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
Box<Scalar>::Box(Scalar x, Scalar y, Scalar z)
  : ShapeBase<Scalar>(), side(x, y, z)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
Box<Scalar>::Box(const Vector3<Scalar>& side_) : ShapeBase<Scalar>(), side(side_)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
Box<Scalar>::Box() : ShapeBase<Scalar>(), side(Vector3<Scalar>::Zero())
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
void Box<Scalar>::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3d::Identity(), this->aabb_local);
  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
}

//==============================================================================
template <typename Scalar>
NODE_TYPE Box<Scalar>::getNodeType() const
{
  return GEOM_BOX;
}

//==============================================================================
template <typename Scalar>
Scalar Box<Scalar>::computeVolume() const
{
  return side[0] * side[1] * side[2];
}

//==============================================================================
template <typename Scalar>
Matrix3<Scalar> Box<Scalar>::computeMomentofInertia() const
{
  Scalar V = computeVolume();

  Scalar a2 = side[0] * side[0] * V;
  Scalar b2 = side[1] * side[1] * V;
  Scalar c2 = side[2] * side[2] * V;

  Vector3<Scalar> I((b2 + c2) / 12, (a2 + c2) / 12, (a2 + b2) / 12);

  return I.asDiagonal();
}

}

#endif

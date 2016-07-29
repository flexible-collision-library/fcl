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


#ifndef FCL_SHAPE_ELLIPSOID_H
#define FCL_SHAPE_ELLIPSOID_H

#include "fcl/shape/shape_base.h"
#include "fcl/shape/geometric_shapes_utility.h"

namespace fcl
{

/// @brief Center at zero point ellipsoid
template <typename Scalar>
class Ellipsoid : public ShapeBase<Scalar>
{
public:
  /// @brief Constructor
  Ellipsoid(Scalar a, Scalar b, Scalar c);

  /// @brief Constructor
  Ellipsoid(const Vector3<Scalar>& radii);

  /// @brief Radii of the ellipsoid
  Vector3<Scalar> radii;

  /// @brief Compute AABB
  void computeLocalAABB() override;

  /// @brief Get node type: a sphere
  NODE_TYPE getNodeType() const override;

  // Documentation inherited
  Matrix3<Scalar> computeMomentofInertia() const override;

  // Documentation inherited
  Scalar computeVolume() const override;
};

using Ellipsoidf = Ellipsoid<float>;
using Ellipsoidd = Ellipsoid<double>;

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
Ellipsoid<Scalar>::Ellipsoid(Scalar a, Scalar b, Scalar c)
  : ShapeBase<Scalar>(), radii(a, b, c)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
Ellipsoid<Scalar>::Ellipsoid(const Vector3<Scalar>& radii)
  : ShapeBase<Scalar>(), radii(radii)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
void Ellipsoid<Scalar>::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3d::Identity(), this->aabb_local);
  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
}

//==============================================================================
template <typename Scalar>
NODE_TYPE Ellipsoid<Scalar>::getNodeType() const
{
  return GEOM_ELLIPSOID;
}

//==============================================================================
template <typename Scalar>
Matrix3<Scalar> Ellipsoid<Scalar>::computeMomentofInertia() const
{
  const Scalar V = computeVolume();

  const Scalar a2 = radii[0] * radii[0] * V;
  const Scalar b2 = radii[1] * radii[1] * V;
  const Scalar c2 = radii[2] * radii[2] * V;

  return Vector3<Scalar>(0.2 * (b2 + c2), 0.2 * (a2 + c2), 0.2 * (a2 + b2)).asDiagonal();
}

//==============================================================================
template <typename Scalar>
Scalar Ellipsoid<Scalar>::computeVolume() const
{
  const Scalar pi = constants::pi;
  return 4.0 * pi * radii[0] * radii[1] * radii[2] / 3.0;
}

}

#endif

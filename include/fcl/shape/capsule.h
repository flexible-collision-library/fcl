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

#ifndef FCL_SHAPE_CAPSULE_H
#define FCL_SHAPE_CAPSULE_H

#include "fcl/shape/shape_base.h"
#include "fcl/shape/geometric_shapes_utility.h"

namespace fcl
{

/// @brief Center at zero point capsule 
template <typename Scalar>
class Capsule : public ShapeBase<Scalar>
{
public:
  /// @brief Constructor
  Capsule(Scalar radius, Scalar lz);

  /// @brief Radius of capsule 
  Scalar radius;

  /// @brief Length along z axis 
  Scalar lz;

  /// @brief Compute AABB 
  void computeLocalAABB() override;

  /// @brief Get node type: a capsule 
  NODE_TYPE getNodeType() const override;

  // Documentation inherited
  Scalar computeVolume() const override;

  // Documentation inherited
  Matrix3<Scalar> computeMomentofInertia() const override;
  
};

using Capsulef = Capsule<float>;
using Capsuled = Capsule<double>;

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
Capsule<Scalar>::Capsule(Scalar radius, Scalar lz)
  : ShapeBased(), radius(radius), lz(lz)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
void Capsule<Scalar>::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3d::Identity(), this->aabb_local);
  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
}

//==============================================================================
template <typename Scalar>
NODE_TYPE Capsule<Scalar>::getNodeType() const
{
  return GEOM_CAPSULE;
}

//==============================================================================
template <typename Scalar>
Scalar Capsule<Scalar>::computeVolume() const
{
  return constants::pi * radius * radius *(lz + radius * 4/3.0);
}

//==============================================================================
template <typename Scalar>
Matrix3<Scalar> Capsule<Scalar>::computeMomentofInertia() const
{
  Scalar v_cyl = radius * radius * lz * constants::pi;
  Scalar v_sph = radius * radius * radius * constants::pi * 4 / 3.0;

  Scalar ix = v_cyl * lz * lz / 12.0 + 0.25 * v_cyl * radius + 0.4 * v_sph * radius * radius + 0.25 * v_sph * lz * lz;
  Scalar iz = (0.5 * v_cyl + 0.4 * v_sph) * radius * radius;

  return Vector3<Scalar>(ix, ix, iz).asDiagonal();
}

}

#endif

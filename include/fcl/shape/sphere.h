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

#ifndef FCL_SHAPE_SPHERE_H
#define FCL_SHAPE_SPHERE_H

#include "fcl/shape/shape_base.h"
#include "fcl/shape/geometric_shapes_utility.h"

namespace fcl
{

/// @brief Center at zero point sphere
template <typename Scalar>
class Sphere : public ShapeBase<Scalar>
{
public:
  Sphere(Scalar radius);

  /// @brief Radius of the sphere
  Scalar radius;

  /// @brief Compute AABB
  void computeLocalAABB() override;

  /// @brief Get node type: a sphere
  NODE_TYPE getNodeType() const override;

  Matrix3<Scalar> computeMomentofInertia() const override;

  Scalar computeVolume() const override;
};

using Spheref = Sphere<float>;
using Sphered = Sphere<double>;

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
Sphere<Scalar>::Sphere(Scalar radius) : ShapeBased(), radius(radius)
{
}

//==============================================================================
template <typename Scalar>
void Sphere<Scalar>::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3d::Identity(), this->aabb_local);
  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = radius;
}

//==============================================================================
template <typename Scalar>
NODE_TYPE Sphere<Scalar>::getNodeType() const
{
  return GEOM_SPHERE; }

//==============================================================================
template <typename Scalar>
Matrix3<Scalar> Sphere<Scalar>::computeMomentofInertia() const
{
  Scalar I = 0.4 * radius * radius * computeVolume();

  return Vector3<Scalar>::Constant(I).asDiagonal();
}

//==============================================================================
template <typename Scalar>
Scalar Sphere<Scalar>::computeVolume() const
{
  return 4.0 * constants::pi * radius * radius * radius / 3.0;
}

}

#endif

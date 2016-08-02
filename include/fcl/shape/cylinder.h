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

#ifndef FCL_SHAPE_CYLINDER_H
#define FCL_SHAPE_CYLINDER_H

#include "fcl/shape/shape_base.h"
#include "fcl/shape/compute_bv.h"
#include "fcl/BV/OBB.h"

namespace fcl
{

/// @brief Center at zero cylinder 
template <typename Scalar>
class Cylinder : public ShapeBase<Scalar>
{
public:
  /// @brief Constructor
  Cylinder(Scalar radius, Scalar lz);
  
  /// @brief Radius of the cylinder 
  Scalar radius;

  /// @brief Length along z axis 
  Scalar lz;

  /// @brief Compute AABBd 
  void computeLocalAABB() override;

  /// @brief Get node type: a cylinder 
  NODE_TYPE getNodeType() const override;

  // Documentation inherited
  Scalar computeVolume() const override;

  // Documentation inherited
  Matrix3<Scalar> computeMomentofInertia() const override;

  std::vector<Vector3<Scalar>> getBoundVertices(
      const Transform3<Scalar>& tf) const
  {
    std::vector<Vector3<Scalar>> result(12);

    auto hl = lz * 0.5;
    auto r2 = radius * 2 / std::sqrt(3.0);
    auto a = 0.5 * r2;
    auto b = radius;

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
};

using Cylinderf = Cylinder<float>;
using Cylinderd = Cylinder<double>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, AABBd, Cylinder<Scalar>>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, OBB<Scalar>, Cylinder<Scalar>>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, AABBd, Cylinder<Scalar>>
{
  void operator()(const Cylinder<Scalar>& s, const Transform3<Scalar>& tf, AABBd& bv)
  {
    const Matrix3d& R = tf.linear();
    const Vector3d& T = tf.translation();

    FCL_REAL x_range = fabs(R(0, 0) * s.radius) + fabs(R(0, 1) * s.radius) + 0.5 * fabs(R(0, 2) * s.lz);
    FCL_REAL y_range = fabs(R(1, 0) * s.radius) + fabs(R(1, 1) * s.radius) + 0.5 * fabs(R(1, 2) * s.lz);
    FCL_REAL z_range = fabs(R(2, 0) * s.radius) + fabs(R(2, 1) * s.radius) + 0.5 * fabs(R(2, 2) * s.lz);

    Vector3d v_delta(x_range, y_range, z_range);
    bv.max_ = T + v_delta;
    bv.min_ = T - v_delta;
  }
};

template <typename Scalar>
struct ComputeBVImpl<Scalar, OBB<Scalar>, Cylinder<Scalar>>
{
  void operator()(const Cylinder<Scalar>& s, const Transform3<Scalar>& tf, OBB<Scalar>& bv)
  {
    bv.To = tf.translation();
    bv.axis = tf.linear();
    bv.extent << s.radius, s.radius, s.lz / 2;
  }
};

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
Cylinder<Scalar>::Cylinder(Scalar radius, Scalar lz)
  : ShapeBase<Scalar>(), radius(radius), lz(lz)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
void Cylinder<Scalar>::computeLocalAABB()
{
  computeBV<Scalar, AABBd>(*this, Transform3d::Identity(), this->aabb_local);
  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
}

//==============================================================================
template <typename Scalar>
NODE_TYPE Cylinder<Scalar>::getNodeType() const
{
  return GEOM_CYLINDER;
}

//==============================================================================
template <typename Scalar>
Scalar Cylinder<Scalar>::computeVolume() const
{
  return constants::pi * radius * radius * lz;
}

//==============================================================================
template <typename Scalar>
Matrix3<Scalar> Cylinder<Scalar>::computeMomentofInertia() const
{
  Scalar V = computeVolume();
  Scalar ix = V * (3 * radius * radius + lz * lz) / 12;
  Scalar iz = V * radius * radius / 2;

  return Vector3<Scalar>(ix, ix, iz).asDiagonal();
}

}

#endif

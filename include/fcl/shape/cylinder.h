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
template <typename ScalarT>
class Cylinder : public ShapeBase<ScalarT>
{
public:

  using Scalar = ScalarT;

  /// @brief Constructor
  Cylinder(ScalarT radius, ScalarT lz);
  
  /// @brief Radius of the cylinder 
  ScalarT radius;

  /// @brief Length along z axis 
  ScalarT lz;

  /// @brief Compute AABBd 
  void computeLocalAABB() override;

  /// @brief Get node type: a cylinder 
  NODE_TYPE getNodeType() const override;

  // Documentation inherited
  ScalarT computeVolume() const override;

  // Documentation inherited
  Matrix3<ScalarT> computeMomentofInertia() const override;

  std::vector<Vector3<ScalarT>> getBoundVertices(
      const Transform3<ScalarT>& tf) const
  {
    std::vector<Vector3<ScalarT>> result(12);

    auto hl = lz * 0.5;
    auto r2 = radius * 2 / std::sqrt(3.0);
    auto a = 0.5 * r2;
    auto b = radius;

    result[0] = tf * Vector3<ScalarT>(r2, 0, -hl);
    result[1] = tf * Vector3<ScalarT>(a, b, -hl);
    result[2] = tf * Vector3<ScalarT>(-a, b, -hl);
    result[3] = tf * Vector3<ScalarT>(-r2, 0, -hl);
    result[4] = tf * Vector3<ScalarT>(-a, -b, -hl);
    result[5] = tf * Vector3<ScalarT>(a, -b, -hl);

    result[6] = tf * Vector3<ScalarT>(r2, 0, hl);
    result[7] = tf * Vector3<ScalarT>(a, b, hl);
    result[8] = tf * Vector3<ScalarT>(-a, b, hl);
    result[9] = tf * Vector3<ScalarT>(-r2, 0, hl);
    result[10] = tf * Vector3<ScalarT>(-a, -b, hl);
    result[11] = tf * Vector3<ScalarT>(a, -b, hl);

    return result;
  }
};

using Cylinderf = Cylinder<float>;
using Cylinderd = Cylinder<double>;

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, AABBd, Cylinder<ScalarT>>;

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, OBB<ScalarT>, Cylinder<ScalarT>>;

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, AABBd, Cylinder<ScalarT>>
{
  void operator()(const Cylinder<ScalarT>& s, const Transform3<ScalarT>& tf, AABBd& bv)
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

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, OBB<ScalarT>, Cylinder<ScalarT>>
{
  void operator()(const Cylinder<ScalarT>& s, const Transform3<ScalarT>& tf, OBB<ScalarT>& bv)
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
template <typename ScalarT>
Cylinder<ScalarT>::Cylinder(ScalarT radius, ScalarT lz)
  : ShapeBase<ScalarT>(), radius(radius), lz(lz)
{
  // Do nothing
}

//==============================================================================
template <typename ScalarT>
void Cylinder<ScalarT>::computeLocalAABB()
{
  computeBV<ScalarT, AABBd>(*this, Transform3d::Identity(), this->aabb_local);
  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
}

//==============================================================================
template <typename ScalarT>
NODE_TYPE Cylinder<ScalarT>::getNodeType() const
{
  return GEOM_CYLINDER;
}

//==============================================================================
template <typename ScalarT>
ScalarT Cylinder<ScalarT>::computeVolume() const
{
  return constants::pi * radius * radius * lz;
}

//==============================================================================
template <typename ScalarT>
Matrix3<ScalarT> Cylinder<ScalarT>::computeMomentofInertia() const
{
  ScalarT V = computeVolume();
  ScalarT ix = V * (3 * radius * radius + lz * lz) / 12;
  ScalarT iz = V * radius * radius / 2;

  return Vector3<ScalarT>(ix, ix, iz).asDiagonal();
}

}

#endif

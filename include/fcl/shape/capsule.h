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
#include "fcl/shape/compute_bv.h"
#include "fcl/BV/OBB.h"

namespace fcl
{

/// @brief Center at zero point capsule 
template <typename ScalarT>
class Capsule : public ShapeBase<ScalarT>
{
public:

  using Scalar = ScalarT;

  /// @brief Constructor
  Capsule(ScalarT radius, ScalarT lz);

  /// @brief Radius of capsule 
  ScalarT radius;

  /// @brief Length along z axis 
  ScalarT lz;

  /// @brief Compute AABB<ScalarT>
  void computeLocalAABB() override;

  /// @brief Get node type: a capsule 
  NODE_TYPE getNodeType() const override;

  // Documentation inherited
  ScalarT computeVolume() const override;

  // Documentation inherited
  Matrix3<ScalarT> computeMomentofInertia() const override;

  /// @brief get the vertices of some convex shape which can bound this shape in
  /// a specific configuration
  std::vector<Vector3<ScalarT>> getBoundVertices(
      const Transform3<ScalarT>& tf) const
  {
    std::vector<Vector3<ScalarT>> result(36);
    const auto m = (1 + std::sqrt(5.0)) / 2.0;

    auto hl = lz * 0.5;
    auto edge_size = radius * 6 / (std::sqrt(27.0) + std::sqrt(15.0));
    auto a = edge_size;
    auto b = m * edge_size;
    auto r2 = radius * 2 / std::sqrt(3.0);

    result[0] = tf * Vector3<ScalarT>(0, a, b + hl);
    result[1] = tf * Vector3<ScalarT>(0, -a, b + hl);
    result[2] = tf * Vector3<ScalarT>(0, a, -b + hl);
    result[3] = tf * Vector3<ScalarT>(0, -a, -b + hl);
    result[4] = tf * Vector3<ScalarT>(a, b, hl);
    result[5] = tf * Vector3<ScalarT>(-a, b, hl);
    result[6] = tf * Vector3<ScalarT>(a, -b, hl);
    result[7] = tf * Vector3<ScalarT>(-a, -b, hl);
    result[8] = tf * Vector3<ScalarT>(b, 0, a + hl);
    result[9] = tf * Vector3<ScalarT>(b, 0, -a + hl);
    result[10] = tf * Vector3<ScalarT>(-b, 0, a + hl);
    result[11] = tf * Vector3<ScalarT>(-b, 0, -a + hl);

    result[12] = tf * Vector3<ScalarT>(0, a, b - hl);
    result[13] = tf * Vector3<ScalarT>(0, -a, b - hl);
    result[14] = tf * Vector3<ScalarT>(0, a, -b - hl);
    result[15] = tf * Vector3<ScalarT>(0, -a, -b - hl);
    result[16] = tf * Vector3<ScalarT>(a, b, -hl);
    result[17] = tf * Vector3<ScalarT>(-a, b, -hl);
    result[18] = tf * Vector3<ScalarT>(a, -b, -hl);
    result[19] = tf * Vector3<ScalarT>(-a, -b, -hl);
    result[20] = tf * Vector3<ScalarT>(b, 0, a - hl);
    result[21] = tf * Vector3<ScalarT>(b, 0, -a - hl);
    result[22] = tf * Vector3<ScalarT>(-b, 0, a - hl);
    result[23] = tf * Vector3<ScalarT>(-b, 0, -a - hl);

    auto c = 0.5 * r2;
    auto d = radius;
    result[24] = tf * Vector3<ScalarT>(r2, 0, hl);
    result[25] = tf * Vector3<ScalarT>(c, d, hl);
    result[26] = tf * Vector3<ScalarT>(-c, d, hl);
    result[27] = tf * Vector3<ScalarT>(-r2, 0, hl);
    result[28] = tf * Vector3<ScalarT>(-c, -d, hl);
    result[29] = tf * Vector3<ScalarT>(c, -d, hl);

    result[30] = tf * Vector3<ScalarT>(r2, 0, -hl);
    result[31] = tf * Vector3<ScalarT>(c, d, -hl);
    result[32] = tf * Vector3<ScalarT>(-c, d, -hl);
    result[33] = tf * Vector3<ScalarT>(-r2, 0, -hl);
    result[34] = tf * Vector3<ScalarT>(-c, -d, -hl);
    result[35] = tf * Vector3<ScalarT>(c, -d, -hl);

    return result;
  }
};

using Capsulef = Capsule<float>;
using Capsuled = Capsule<double>;

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, AABB<ScalarT>, Capsule<ScalarT>>;

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, OBB<ScalarT>, Capsule<ScalarT>>;

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, AABB<ScalarT>, Capsule<ScalarT>>
{
  void operator()(const Capsule<ScalarT>& s, const Transform3<ScalarT>& tf, AABB<ScalarT>& bv)
  {
    const Matrix3<ScalarT>& R = tf.linear();
    const Vector3<ScalarT>& T = tf.translation();

    ScalarT x_range = 0.5 * fabs(R(0, 2) * s.lz) + s.radius;
    ScalarT y_range = 0.5 * fabs(R(1, 2) * s.lz) + s.radius;
    ScalarT z_range = 0.5 * fabs(R(2, 2) * s.lz) + s.radius;

    Vector3<ScalarT> v_delta(x_range, y_range, z_range);
    bv.max_ = T + v_delta;
    bv.min_ = T - v_delta;
  }
};

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, OBB<ScalarT>, Capsule<ScalarT>>
{
  void operator()(const Capsule<ScalarT>& s, const Transform3<ScalarT>& tf, OBB<ScalarT>& bv)
  {
    bv.To = tf.translation();
    bv.axis = tf.linear();
    bv.extent << s.radius, s.radius, s.lz / 2 + s.radius;
  }
};

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename ScalarT>
Capsule<ScalarT>::Capsule(ScalarT radius, ScalarT lz)
  : ShapeBase<ScalarT>(), radius(radius), lz(lz)
{
  // Do nothing
}

//==============================================================================
template <typename ScalarT>
void Capsule<ScalarT>::computeLocalAABB()
{
  computeBV<ScalarT, AABB<ScalarT>>(*this, Transform3<ScalarT>::Identity(), this->aabb_local);
  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
}

//==============================================================================
template <typename ScalarT>
NODE_TYPE Capsule<ScalarT>::getNodeType() const
{
  return GEOM_CAPSULE;
}

//==============================================================================
template <typename ScalarT>
ScalarT Capsule<ScalarT>::computeVolume() const
{
  return constants<Scalar>::pi() * radius * radius *(lz + radius * 4/3.0);
}

//==============================================================================
template <typename ScalarT>
Matrix3<ScalarT> Capsule<ScalarT>::computeMomentofInertia() const
{
  ScalarT v_cyl = radius * radius * lz * constants<Scalar>::pi();
  ScalarT v_sph = radius * radius * radius * constants<Scalar>::pi() * 4 / 3.0;

  ScalarT ix = v_cyl * lz * lz / 12.0 + 0.25 * v_cyl * radius + 0.4 * v_sph * radius * radius + 0.25 * v_sph * lz * lz;
  ScalarT iz = (0.5 * v_cyl + 0.4 * v_sph) * radius * radius;

  return Vector3<ScalarT>(ix, ix, iz).asDiagonal();
}

}

#endif

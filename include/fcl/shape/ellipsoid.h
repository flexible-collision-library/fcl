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
#include "fcl/shape/compute_bv.h"
#include "fcl/BV/OBB.h"

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

  /// @brief Compute AABBd
  void computeLocalAABB() override;

  /// @brief Get node type: a sphere
  NODE_TYPE getNodeType() const override;

  // Documentation inherited
  Matrix3<Scalar> computeMomentofInertia() const override;

  // Documentation inherited
  Scalar computeVolume() const override;

  std::vector<Vector3<Scalar>> getBoundVertices(
      const Transform3<Scalar>& tf) const
  {
    // we use scaled icosahedron to bound the ellipsoid

    std::vector<Vector3<Scalar>> result(12);

    const auto phi = (1.0 + std::sqrt(5.0)) / 2.0;  // golden ratio

    const auto a = std::sqrt(3.0) / (phi * phi);
    const auto b = phi * a;

    const auto& A = radii[0];
    const auto& B = radii[1];
    const auto& C = radii[2];

    const auto Aa = A * a;
    const auto Ab = A * b;
    const auto Ba = B * a;
    const auto Bb = B * b;
    const auto Ca = C * a;
    const auto Cb = C * b;

    result[0] = tf * Vector3<Scalar>(0, Ba, Cb);
    result[1] = tf * Vector3<Scalar>(0, -Ba, Cb);
    result[2] = tf * Vector3<Scalar>(0, Ba, -Cb);
    result[3] = tf * Vector3<Scalar>(0, -Ba, -Cb);
    result[4] = tf * Vector3<Scalar>(Aa, Bb, 0);
    result[5] = tf * Vector3<Scalar>(-Aa, Bb, 0);
    result[6] = tf * Vector3<Scalar>(Aa, -Bb, 0);
    result[7] = tf * Vector3<Scalar>(-Aa, -Bb, 0);
    result[8] = tf * Vector3<Scalar>(Ab, 0, Ca);
    result[9] = tf * Vector3<Scalar>(Ab, 0, -Ca);
    result[10] = tf * Vector3<Scalar>(-Ab, 0, Ca);
    result[11] = tf * Vector3<Scalar>(-Ab, 0, -Ca);

    return result;
  }
};

using Ellipsoidf = Ellipsoid<float>;
using Ellipsoidd = Ellipsoid<double>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, AABBd, Ellipsoid<Scalar>>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, OBB<Scalar>, Ellipsoid<Scalar>>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, AABBd, Ellipsoid<Scalar>>
{
  void operator()(const Ellipsoid<Scalar>& s, const Transform3<Scalar>& tf, AABBd& bv)
  {
    const Matrix3d& R = tf.linear();
    const Vector3d& T = tf.translation();

    FCL_REAL x_range = (fabs(R(0, 0) * s.radii[0]) + fabs(R(0, 1) * s.radii[1]) + fabs(R(0, 2) * s.radii[2]));
    FCL_REAL y_range = (fabs(R(1, 0) * s.radii[0]) + fabs(R(1, 1) * s.radii[1]) + fabs(R(1, 2) * s.radii[2]));
    FCL_REAL z_range = (fabs(R(2, 0) * s.radii[0]) + fabs(R(2, 1) * s.radii[1]) + fabs(R(2, 2) * s.radii[2]));

    Vector3d v_delta(x_range, y_range, z_range);
    bv.max_ = T + v_delta;
    bv.min_ = T - v_delta;
  }
};

template <typename Scalar>
struct ComputeBVImpl<Scalar, OBB<Scalar>, Ellipsoid<Scalar>>
{
  void operator()(const Ellipsoid<Scalar>& s, const Transform3<Scalar>& tf, OBB<Scalar>& bv)
  {
    bv.To = tf.translation();
    bv.axis = tf.linear();
    bv.extent = s.radii;
  }
};

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
  computeBV<Scalar, AABBd>(*this, Transform3d::Identity(), this->aabb_local);
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

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

#include "fcl/BVH/BV_fitter.h"
#include "fcl/shape/shape_base.h"
#include "fcl/shape/compute_bv.h"

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

  /// @brief Compute AABBd
  void computeLocalAABB() override;

  /// @brief Get node type: a box
  NODE_TYPE getNodeType() const override;

  // Documentation inherited
  Scalar computeVolume() const override;

  // Documentation inherited
  Matrix3<Scalar> computeMomentofInertia() const override;

  /// @brief get the vertices of some convex shape which can bound this shape in
  /// a specific configuration
  std::vector<Vector3<Scalar>> getBoundVertices(
      const Transform3<Scalar>& tf) const;
};

using Boxf = Box<float>;
using Boxd = Box<double>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, AABBd, Box<Scalar>>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, OBBd, Box<Scalar>>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, AABBd, Box<Scalar>>
{
  void operator()(const Box<Scalar>& s, const Transform3<Scalar>& tf, AABBd& bv)
  {
    const Matrix3d& R = tf.linear();
    const Vector3d& T = tf.translation();

    FCL_REAL x_range = 0.5 * (fabs(R(0, 0) * s.side[0]) + fabs(R(0, 1) * s.side[1]) + fabs(R(0, 2) * s.side[2]));
    FCL_REAL y_range = 0.5 * (fabs(R(1, 0) * s.side[0]) + fabs(R(1, 1) * s.side[1]) + fabs(R(1, 2) * s.side[2]));
    FCL_REAL z_range = 0.5 * (fabs(R(2, 0) * s.side[0]) + fabs(R(2, 1) * s.side[1]) + fabs(R(2, 2) * s.side[2]));

    Vector3d v_delta(x_range, y_range, z_range);
    bv.max_ = T + v_delta;
    bv.min_ = T - v_delta;
  }
};

template <typename Scalar>
struct ComputeBVImpl<Scalar, OBBd, Box<Scalar>>
{
  void operator()(const Box<Scalar>& s, const Transform3<Scalar>& tf, OBBd& bv)
  {
    bv.To = tf.translation();
    bv.axis = tf.linear();
    bv.extent = s.side * (FCL_REAL)0.5;
  }
};

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
  computeBV(*this, Transform3<Scalar>::Identity(), this->aabb_local);
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
  return side.prod();
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

//==============================================================================
template <typename Scalar>
std::vector<Vector3<Scalar>> Box<Scalar>::getBoundVertices(
    const Transform3<Scalar>& tf) const
{
  std::vector<Vector3<Scalar>> result(8);
  auto a = side[0] / 2;
  auto b = side[1] / 2;
  auto c = side[2] / 2;
  result[0] = tf * Vector3<Scalar>(a, b, c);
  result[1] = tf * Vector3<Scalar>(a, b, -c);
  result[2] = tf * Vector3<Scalar>(a, -b, c);
  result[3] = tf * Vector3<Scalar>(a, -b, -c);
  result[4] = tf * Vector3<Scalar>(-a, b, c);
  result[5] = tf * Vector3<Scalar>(-a, b, -c);
  result[6] = tf * Vector3<Scalar>(-a, -b, c);
  result[7] = tf * Vector3<Scalar>(-a, -b, -c);

  return result;
}

} // namespace fcl

#endif

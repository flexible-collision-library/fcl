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
#include "fcl/shape/compute_bv.h"
#include "fcl/BV/OBB.h"

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

  /// @brief Compute AABBd
  void computeLocalAABB() override;

  /// @brief Get node type: a sphere
  NODE_TYPE getNodeType() const override;

  Matrix3<Scalar> computeMomentofInertia() const override;

  Scalar computeVolume() const override;

  std::vector<Vector3<Scalar>> getBoundVertices(
      const Transform3<Scalar>& tf) const
  {
    // we use icosahedron to bound the sphere

    std::vector<Vector3<Scalar>> result(12);
    const auto m = (1 + std::sqrt(5.0)) / 2.0;
    auto edge_size = radius * 6 / (std::sqrt(27.0) + std::sqrt(15.0));

    auto a = edge_size;
    auto b = m * edge_size;
    result[0] = tf * Vector3<Scalar>(0, a, b);
    result[1] = tf * Vector3<Scalar>(0, -a, b);
    result[2] = tf * Vector3<Scalar>(0, a, -b);
    result[3] = tf * Vector3<Scalar>(0, -a, -b);
    result[4] = tf * Vector3<Scalar>(a, b, 0);
    result[5] = tf * Vector3<Scalar>(-a, b, 0);
    result[6] = tf * Vector3<Scalar>(a, -b, 0);
    result[7] = tf * Vector3<Scalar>(-a, -b, 0);
    result[8] = tf * Vector3<Scalar>(b, 0, a);
    result[9] = tf * Vector3<Scalar>(b, 0, -a);
    result[10] = tf * Vector3<Scalar>(-b, 0, a);
    result[11] = tf * Vector3<Scalar>(-b, 0, -a);

    return result;
  }
};

using Spheref = Sphere<float>;
using Sphered = Sphere<double>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, AABBd, Sphere<Scalar>>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, OBB<Scalar>, Sphere<Scalar>>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, AABBd, Sphere<Scalar>>
{
  void operator()(const Sphere<Scalar>& s, const Transform3<Scalar>& tf, AABBd& bv)
  {
    const Vector3d& T = tf.translation();

    Vector3d v_delta = Vector3d::Constant(s.radius);
    bv.max_ = T + v_delta;
    bv.min_ = T - v_delta;
  }
};

template <typename Scalar>
struct ComputeBVImpl<Scalar, OBB<Scalar>, Sphere<Scalar>>
{
  void operator()(const Sphere<Scalar>& s, const Transform3<Scalar>& tf, OBB<Scalar>& bv)
  {
    bv.To = tf.translation();
    bv.axis.setIdentity();
    bv.extent.setConstant(s.radius);
  }
};

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
  computeBV<Scalar, AABBd>(*this, Transform3d::Identity(), this->aabb_local);
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

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
template <typename ScalarT>
class Sphere : public ShapeBase<ScalarT>
{
public:

  using Scalar = ScalarT;

  Sphere(ScalarT radius);

  /// @brief Radius of the sphere
  ScalarT radius;

  /// @brief Compute AABB<ScalarT>
  void computeLocalAABB() override;

  /// @brief Get node type: a sphere
  NODE_TYPE getNodeType() const override;

  Matrix3<ScalarT> computeMomentofInertia() const override;

  ScalarT computeVolume() const override;

  std::vector<Vector3<ScalarT>> getBoundVertices(
      const Transform3<ScalarT>& tf) const
  {
    // we use icosahedron to bound the sphere

    std::vector<Vector3<ScalarT>> result(12);
    const auto m = (1 + std::sqrt(5.0)) / 2.0;
    auto edge_size = radius * 6 / (std::sqrt(27.0) + std::sqrt(15.0));

    auto a = edge_size;
    auto b = m * edge_size;
    result[0] = tf * Vector3<ScalarT>(0, a, b);
    result[1] = tf * Vector3<ScalarT>(0, -a, b);
    result[2] = tf * Vector3<ScalarT>(0, a, -b);
    result[3] = tf * Vector3<ScalarT>(0, -a, -b);
    result[4] = tf * Vector3<ScalarT>(a, b, 0);
    result[5] = tf * Vector3<ScalarT>(-a, b, 0);
    result[6] = tf * Vector3<ScalarT>(a, -b, 0);
    result[7] = tf * Vector3<ScalarT>(-a, -b, 0);
    result[8] = tf * Vector3<ScalarT>(b, 0, a);
    result[9] = tf * Vector3<ScalarT>(b, 0, -a);
    result[10] = tf * Vector3<ScalarT>(-b, 0, a);
    result[11] = tf * Vector3<ScalarT>(-b, 0, -a);

    return result;
  }
};

using Spheref = Sphere<float>;
using Sphered = Sphere<double>;

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, AABB<ScalarT>, Sphere<ScalarT>>;

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, OBB<ScalarT>, Sphere<ScalarT>>;

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, AABB<ScalarT>, Sphere<ScalarT>>
{
  void operator()(const Sphere<ScalarT>& s, const Transform3<ScalarT>& tf, AABB<ScalarT>& bv)
  {
    const Vector3<ScalarT> v_delta = Vector3<ScalarT>::Constant(s.radius);
    bv.max_ = tf.translation() + v_delta;
    bv.min_ = tf.translation() - v_delta;
  }
};

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, OBB<ScalarT>, Sphere<ScalarT>>
{
  void operator()(const Sphere<ScalarT>& s, const Transform3<ScalarT>& tf, OBB<ScalarT>& bv)
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
template <typename ScalarT>
Sphere<ScalarT>::Sphere(ScalarT radius) : ShapeBase<ScalarT>(), radius(radius)
{
}

//==============================================================================
template <typename ScalarT>
void Sphere<ScalarT>::computeLocalAABB()
{
  computeBV<ScalarT, AABB<ScalarT>>(*this, Transform3<ScalarT>::Identity(), this->aabb_local);
  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = radius;
}

//==============================================================================
template <typename ScalarT>
NODE_TYPE Sphere<ScalarT>::getNodeType() const
{
  return GEOM_SPHERE; }

//==============================================================================
template <typename ScalarT>
Matrix3<ScalarT> Sphere<ScalarT>::computeMomentofInertia() const
{
  ScalarT I = 0.4 * radius * radius * computeVolume();

  return Vector3<ScalarT>::Constant(I).asDiagonal();
}

//==============================================================================
template <typename ScalarT>
ScalarT Sphere<ScalarT>::computeVolume() const
{
  return 4.0 * constants<Scalar>::pi() * radius * radius * radius / 3.0;
}

}

#endif

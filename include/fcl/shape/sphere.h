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
template <typename S_>
class Sphere : public ShapeBase<S_>
{
public:

  using S = S_;

  Sphere(S radius);

  /// @brief Radius of the sphere
  S radius;

  /// @brief Compute AABB<S>
  void computeLocalAABB() override;

  /// @brief Get node type: a sphere
  NODE_TYPE getNodeType() const override;

  Matrix3<S> computeMomentofInertia() const override;

  S computeVolume() const override;

  /// @brief get the vertices of some convex shape which can bound this shape in
  /// a specific configuration
  std::vector<Vector3<S>> getBoundVertices(
      const Transform3<S>& tf) const;
};

using Spheref = Sphere<float>;
using Sphered = Sphere<double>;

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename S>
Sphere<S>::Sphere(S radius) : ShapeBase<S>(), radius(radius)
{
}

//==============================================================================
template <typename S>
void Sphere<S>::computeLocalAABB()
{
  computeBV(*this, Transform3<S>::Identity(), this->aabb_local);
  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = radius;
}

//==============================================================================
template <typename S>
NODE_TYPE Sphere<S>::getNodeType() const
{
  return GEOM_SPHERE; }

//==============================================================================
template <typename S>
Matrix3<S> Sphere<S>::computeMomentofInertia() const
{
  S I = (S)0.4 * radius * radius * computeVolume();

  return Vector3<S>::Constant(I).asDiagonal();
}

//==============================================================================
template <typename S>
S Sphere<S>::computeVolume() const
{
  return (S)4.0 * constants<S>::pi() * radius * radius * radius / (S)3.0;
}

//==============================================================================
template <typename S>
std::vector<Vector3<S>> Sphere<S>::getBoundVertices(
    const Transform3<S>& tf) const
{
  // we use icosahedron to bound the sphere

  std::vector<Vector3<S>> result(12);
  const auto m = (1 + std::sqrt(5.0)) / 2.0;
  auto edge_size = radius * 6 / (std::sqrt(27.0) + std::sqrt(15.0));

  auto a = edge_size;
  auto b = m * edge_size;
  result[0] = tf * Vector3<S>(0, a, b);
  result[1] = tf * Vector3<S>(0, -a, b);
  result[2] = tf * Vector3<S>(0, a, -b);
  result[3] = tf * Vector3<S>(0, -a, -b);
  result[4] = tf * Vector3<S>(a, b, 0);
  result[5] = tf * Vector3<S>(-a, b, 0);
  result[6] = tf * Vector3<S>(a, -b, 0);
  result[7] = tf * Vector3<S>(-a, -b, 0);
  result[8] = tf * Vector3<S>(b, 0, a);
  result[9] = tf * Vector3<S>(b, 0, -a);
  result[10] = tf * Vector3<S>(-b, 0, a);
  result[11] = tf * Vector3<S>(-b, 0, -a);

  return result;
}

} // namespace fcl

#include "fcl/shape/detail/bv_computer_sphere.h"

#endif

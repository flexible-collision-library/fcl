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

/** @author Jia Pan */

#ifndef FCL_SHAPE_PLANE_INL_H
#define FCL_SHAPE_PLANE_INL_H

#include <iomanip>
#include <sstream>

#include "fcl/geometry/shape/plane.h"
#include "fcl/geometry/shape/representation.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT Plane<double>;

//==============================================================================
extern template
Plane<double> transform(const Plane<double>& a, const Transform3<double>& tf);

//==============================================================================
template <typename S>
Plane<S>::Plane(const Vector3<S>& n, S d)
  : ShapeBase<S>(), n(n), d(d)
{
  unitNormalTest();
}

//==============================================================================
template <typename S>
Plane<S>::Plane(S a, S b, S c, S d)
  : ShapeBase<S>(), n(a, b, c), d(d)
{
  unitNormalTest();
}

//==============================================================================
template <typename S>
Plane<S>::Plane() : ShapeBase<S>(), n(1, 0, 0), d(0)
{
  // Do nothing
}

//==============================================================================
template <typename S>
S Plane<S>::signedDistance(const Vector3<S>& p) const
{
  return n.dot(p) - d;
}

//==============================================================================
template <typename S>
S Plane<S>::distance(const Vector3<S>& p) const
{
  return std::abs(n.dot(p) - d);
}

//==============================================================================
template <typename S>
void Plane<S>::computeLocalAABB()
{
  this->aabb_local.min_.setConstant(-std::numeric_limits<S>::max());
  this->aabb_local.max_.setConstant(std::numeric_limits<S>::max());
  if(n[1] == (S)0.0 && n[2] == (S)0.0)
  {
    // normal aligned with x axis
    if(n[0] < 0)
      this->aabb_local.min_[0] = this->aabb_local.max_[0] = -d;
    else if(n[0] > 0)
      this->aabb_local.min_[0] = this->aabb_local.max_[0] = d;
  }
  else if(n[0] == (S)0.0 && n[2] == (S)0.0)
  {
    // normal aligned with y axis
    if(n[1] < 0)
      this->aabb_local.min_[1] = this->aabb_local.max_[1] = -d;
    else if(n[1] > 0)
      this->aabb_local.min_[1] = this->aabb_local.max_[1] = d;
  }
  else if(n[0] == (S)0.0 && n[1] == (S)0.0)
  {
    // normal aligned with z axis
    if(n[2] < 0)
      this->aabb_local.min_[2] = this->aabb_local.max_[2] = -d;
    else if(n[2] > 0)
      this->aabb_local.min_[2] = this->aabb_local.max_[2] = d;
  }

  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
}

//==============================================================================
template <typename S>
NODE_TYPE Plane<S>::getNodeType() const
{
  return GEOM_PLANE;
}

//==============================================================================
template <typename S>
std::string Plane<S>::representation(int precision) const {
  const char* S_str = detail::ScalarRepr<S>::value();
  std::stringstream ss;
  ss << std::setprecision(precision);
  ss << "Plane<" << S_str << ">(" << n[0] << ", " << n[1] << ", " << n[2]
     << ", " << d << ");";
  return ss.str();
}

//==============================================================================
template <typename S>
void Plane<S>::unitNormalTest()
{
  S l = n.norm();
  if(l > 0)
  {
    S inv_l = 1.0 / l;
    n *= inv_l;
    d *= inv_l;
  }
  else
  {
    n << 1, 0, 0;
    d = 0;
  }
}

//==============================================================================
template <typename S>
Plane<S> transform(const Plane<S>& a, const Transform3<S>& tf)
{
  /// suppose the initial halfspace is n * x <= d
  /// after transform (R, T), x --> x' = R x + T
  /// and the new half space becomes n' * x' <= d'
  /// where n' = R * n
  ///   and d' = d + n' * T

  Vector3<S> n = tf.linear() * a.n;
  S d = a.d + n.dot(tf.translation());

  return Plane<S>(n, d);
}

} // namespace fcl

#endif

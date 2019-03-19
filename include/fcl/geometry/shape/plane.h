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

#ifndef FCL_SHAPE_PLANE_H
#define FCL_SHAPE_PLANE_H

#include <iostream>

#include "fcl/geometry/shape/shape_base.h"

namespace fcl
{

/// @brief Infinite plane 
template <typename S_>
class FCL_EXPORT Plane : public ShapeBase<S_>
{
public:

  using S = S_;

  /// @brief Construct a plane with normal direction and offset 
  Plane(const Vector3<S>& n, S d);
  
  /// @brief Construct a plane with normal direction and offset 
  Plane(S a, S b, S c, S d);

  Plane();

  S signedDistance(const Vector3<S>& p) const;

  S distance(const Vector3<S>& p) const;

  /// @brief Compute AABB
  void computeLocalAABB() override;

  /// @brief Get node type: a plane 
  NODE_TYPE getNodeType() const override;

  /// @brief Plane normal 
  Vector3<S> n;

  /// @brief Plane offset 
  S d;

  friend
  std::ostream& operator<<(std::ostream& out, const Plane& plane) {
    out << "Plane(n: " << plane.n.transpose() << ", d: " << plane.d << ")";
    return out;
  }

protected:
  
  /// @brief Turn non-unit normal into unit 
  void unitNormalTest();
};

using Planef = Plane<float>;
using Planed = Plane<double>;

template <typename S>
FCL_EXPORT
Plane<S> transform(const Plane<S>& a, const Transform3<S>& tf);

} // namespace fcl

#include "fcl/geometry/shape/plane-inl.h"

#endif

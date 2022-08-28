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

#ifndef FCL_SHAPE_TRIANGLE_P_H
#define FCL_SHAPE_TRIANGLE_P_H

#include <iostream>

#include "fcl/geometry/shape/shape_base.h"

namespace fcl
{

/// @brief Triangle stores the points instead of only indices of points
template <typename S_>
class FCL_EXPORT TriangleP : public ShapeBase<S_>
{
public:

  using S = S_;

  TriangleP(const Vector3<S>& a,
            const Vector3<S>& b,
            const Vector3<S>& c);

  /// @brief virtual function of compute AABB<S> in local coordinate
  void computeLocalAABB() override;
  
  // Documentation inherited
  NODE_TYPE getNodeType() const override;

  Vector3<S> a;
  Vector3<S> b;
  Vector3<S> c;
  Vector3<S> center;

  /// @brief get the vertices of some convex shape which can bound this shape in
  /// a specific configuration
  std::vector<Vector3<S>> getBoundVertices(const Transform3<S>& tf) const;

  virtual Vector3<S> localGetSupportingVertex(const Vector3<S>& vec) const override;

  friend
  std::ostream& operator<<(std::ostream& out, const TriangleP& tri) {
    out << "TriangleP(a: " << tri.a.transpose()
        << ", b: " << tri.b.transpose() << ", c: " << tri.c.transpose() << ")";
    return out;
  }
};

using TrianglePf = TriangleP<float>;
using TrianglePd = TriangleP<double>;

} // namespace fcl

#include "fcl/geometry/shape/triangle_p-inl.h"

#endif

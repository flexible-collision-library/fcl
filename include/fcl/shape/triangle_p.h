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

#ifndef FCL_SHAPE_TRIANGLE_P_H
#define FCL_SHAPE_TRIANGLE_P_H

#include "fcl/shape/shape_base.h"
#include "fcl/shape/compute_bv.h"

namespace fcl
{

/// @brief Triangle stores the points instead of only indices of points
template <typename ScalarT>
class TriangleP : public ShapeBase<ScalarT>
{
public:

  using Scalar = ScalarT;

  TriangleP(const Vector3<ScalarT>& a,
            const Vector3<ScalarT>& b,
            const Vector3<ScalarT>& c);

  /// @brief virtual function of compute AABB<ScalarT> in local coordinate
  void computeLocalAABB() override;
  
  // Documentation inherited
  NODE_TYPE getNodeType() const override;

  Vector3<ScalarT> a;
  Vector3<ScalarT> b;
  Vector3<ScalarT> c;

  /// @brief get the vertices of some convex shape which can bound this shape in
  /// a specific configuration
  std::vector<Vector3<ScalarT>> getBoundVertices(
      const Transform3<ScalarT>& tf) const;
};

using TrianglePf = TriangleP<float>;
using TrianglePd = TriangleP<double>;

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename ScalarT>
TriangleP<ScalarT>::TriangleP(
    const Vector3<ScalarT>& a,
    const Vector3<ScalarT>& b,
    const Vector3<ScalarT>& c)
  : ShapeBase<ScalarT>(), a(a), b(b), c(c)
{
  // Do nothing
}

//==============================================================================
template <typename ScalarT>
void TriangleP<ScalarT>::computeLocalAABB()
{
  computeBV(*this, Transform3<ScalarT>::Identity(), this->aabb_local);
  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
}

//==============================================================================
template <typename ScalarT>
NODE_TYPE TriangleP<ScalarT>::getNodeType() const
{
  return GEOM_TRIANGLE;
}

//==============================================================================
template <typename ScalarT>
std::vector<Vector3<ScalarT>> TriangleP<ScalarT>::getBoundVertices(
    const Transform3<ScalarT>& tf) const
{
  std::vector<Vector3<ScalarT>> result(3);
  result[0] = tf * a;
  result[1] = tf * b;
  result[2] = tf * c;

  return result;
}

} // namespace fcl

#include "fcl/shape/detail/bv_computer_triangle_p.h"

#endif

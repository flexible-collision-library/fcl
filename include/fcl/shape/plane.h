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

#ifndef FCL_SHAPE_PLANE_H
#define FCL_SHAPE_PLANE_H

#include "fcl/shape/shape_base.h"
#include "fcl/shape/geometric_shapes_utility.h"

namespace fcl
{

/// @brief Infinite plane 
template <typename Scalar>
class Plane : public ShapeBase<Scalar>
{
public:
  /// @brief Construct a plane with normal direction and offset 
  Plane(const Vector3<Scalar>& n, Scalar d);
  
  /// @brief Construct a plane with normal direction and offset 
  Plane(Scalar a, Scalar b, Scalar c, Scalar d);

  Plane();

  Scalar signedDistance(const Vector3<Scalar>& p) const;

  Scalar distance(const Vector3<Scalar>& p) const;

  /// @brief Compute AABB 
  void computeLocalAABB() override;

  /// @brief Get node type: a plane 
  NODE_TYPE getNodeType() const override;

  /// @brief Plane normal 
  Vector3<Scalar> n;

  /// @brief Plane offset 
  Scalar d;

protected:
  
  /// @brief Turn non-unit normal into unit 
  void unitNormalTest();
};

using Planef = Plane<float>;
using Planed = Plane<double>;

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
Plane<Scalar>::Plane(const Vector3<Scalar>& n, Scalar d)
  : ShapeBase<Scalar>(), n(n), d(d)
{
  unitNormalTest();
}

//==============================================================================
template <typename Scalar>
Plane<Scalar>::Plane(Scalar a, Scalar b, Scalar c, Scalar d)
  : ShapeBase<Scalar>(), n(a, b, c), d(d)
{
  unitNormalTest();
}

//==============================================================================
template <typename Scalar>
Plane<Scalar>::Plane() : ShapeBased(), n(1, 0, 0), d(0)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
Scalar Plane<Scalar>::signedDistance(const Vector3<Scalar>& p) const
{
  return n.dot(p) - d;
}

//==============================================================================
template <typename Scalar>
Scalar Plane<Scalar>::distance(const Vector3<Scalar>& p) const
{
  return std::abs(n.dot(p) - d);
}

//==============================================================================
template <typename Scalar>
void Plane<Scalar>::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3d::Identity(), this->aabb_local);
  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
}

//==============================================================================
template <typename Scalar>
NODE_TYPE Plane<Scalar>::getNodeType() const
{
  return GEOM_PLANE;
}

//==============================================================================
template <typename Scalar>
void Plane<Scalar>::unitNormalTest()
{
  Scalar l = n.norm();
  if(l > 0)
  {
    Scalar inv_l = 1.0 / l;
    n *= inv_l;
    d *= inv_l;
  }
  else
  {
    n << 1, 0, 0;
    d = 0;
  }
}


}

#endif

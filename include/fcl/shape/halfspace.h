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

#ifndef FCL_SHAPE_HALFSPACE_H
#define FCL_SHAPE_HALFSPACE_H

#include "fcl/shape/shape_base.h"
#include "fcl/shape/geometric_shapes_utility.h"

namespace fcl
{

/// @brief Half Space: this is equivalent to the Planed in ODE. The separation plane is defined as n * x = d;
/// Points in the negative side of the separation plane (i.e. {x | n * x < d}) are inside the half space and points
/// in the positive side of the separation plane (i.e. {x | n * x > d}) are outside the half space
template <typename Scalar>
class Halfspace : public ShapeBase<Scalar>
{
public:
  /// @brief Construct a half space with normal direction and offset
  Halfspace(const Vector3<Scalar>& n, Scalar d);

  /// @brief Construct a plane with normal direction and offset
  Halfspace(Scalar a, Scalar b, Scalar c, Scalar d_);

  Halfspace();

  Scalar signedDistance(const Vector3<Scalar>& p) const;

  Scalar distance(const Vector3<Scalar>& p) const;

  /// @brief Compute AABB
  void computeLocalAABB() override;

  /// @brief Get node type: a half space
  NODE_TYPE getNodeType() const override;
  
  /// @brief Planed normal
  Vector3<Scalar> n;
  
  /// @brief Planed offset
  Scalar d;

protected:

  /// @brief Turn non-unit normal into unit
  void unitNormalTest();
};

using Halfspacef = Halfspace<float>;
using Halfspaced = Halfspace<double>;

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
Halfspace<Scalar>::Halfspace(const Vector3<Scalar>& n, Scalar d)
  : ShapeBase<Scalar>(), n(n), d(d)
{
  unitNormalTest();
}

//==============================================================================
template <typename Scalar>
Halfspace<Scalar>::Halfspace(Scalar a, Scalar b, Scalar c, Scalar d)
  : ShapeBase<Scalar>(), n(a, b, c), d(d)
{
  unitNormalTest();
}

//==============================================================================
template <typename Scalar>
Halfspace<Scalar>::Halfspace() : ShapeBase<Scalar>(), n(1, 0, 0), d(0)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
Scalar Halfspace<Scalar>::signedDistance(const Vector3<Scalar>& p) const
{
  return n.dot(p) - d;
}

//==============================================================================
template <typename Scalar>
Scalar Halfspace<Scalar>::distance(const Vector3<Scalar>& p) const
{
  return std::abs(n.dot(p) - d);
}

//==============================================================================
template <typename Scalar>
void Halfspace<Scalar>::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3d::Identity(), this->aabb_local);
  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
}

//==============================================================================
template <typename Scalar>
NODE_TYPE Halfspace<Scalar>::getNodeType() const
{
  return GEOM_HALFSPACE;
}

//==============================================================================
template <typename Scalar>
void Halfspace<Scalar>::unitNormalTest()
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

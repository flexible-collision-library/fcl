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

#include "fcl/geometry/shape/halfspace.h"

namespace fcl
{

//==============================================================================
template <typename S>
Halfspace<S>::Halfspace(const Vector3<S>& n, S d)
  : ShapeBase<S>(), n(n), d(d)
{
  unitNormalTest();
}

//==============================================================================
template <typename S>
Halfspace<S>::Halfspace(S a, S b, S c, S d)
  : ShapeBase<S>(), n(a, b, c), d(d)
{
  unitNormalTest();
}

//==============================================================================
template <typename S>
Halfspace<S>::Halfspace() : ShapeBase<S>(), n(1, 0, 0), d(0)
{
  // Do nothing
}

//==============================================================================
template <typename S>
S Halfspace<S>::signedDistance(const Vector3<S>& p) const
{
  return n.dot(p) - d;
}

//==============================================================================
template <typename S>
S Halfspace<S>::distance(const Vector3<S>& p) const
{
  return std::abs(n.dot(p) - d);
}

//==============================================================================
template <typename S>
void Halfspace<S>::computeLocalAABB()
{
  computeBV(*this, Transform3<S>::Identity(), this->aabb_local);
  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
}

//==============================================================================
template <typename S>
NODE_TYPE Halfspace<S>::getNodeType() const
{
  return GEOM_HALFSPACE;
}

//==============================================================================
template <typename S>
void Halfspace<S>::unitNormalTest()
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

} // namespace fcl

#include "fcl/geometry/shape/detail/bv_computer_halfspace.h"

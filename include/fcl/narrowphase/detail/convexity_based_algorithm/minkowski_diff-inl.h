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

#ifndef FCL_NARROWPHASE_DETAIL_MINKOWSKIDIFF_INL_H
#define FCL_NARROWPHASE_DETAIL_MINKOWSKIDIFF_INL_H

#include "fcl/narrowphase/detail/convexity_based_algorithm/minkowski_diff.h"

#include "fcl/geometry/shape/box.h"
#include "fcl/geometry/shape/capsule.h"
#include "fcl/geometry/shape/cone.h"
#include "fcl/geometry/shape/convex.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/geometry/shape/ellipsoid.h"
#include "fcl/geometry/shape/halfspace.h"
#include "fcl/geometry/shape/plane.h"
#include "fcl/geometry/shape/sphere.h"
#include "fcl/geometry/shape/triangle_p.h"

namespace fcl
{

namespace detail
{

//==============================================================================
extern template
struct MinkowskiDiff<double>;

//==============================================================================
template <typename S>
MinkowskiDiff<S>::MinkowskiDiff()
{
  // Do nothing
}

//==============================================================================
template <typename S>
Vector3<S> MinkowskiDiff<S>::support0(const Vector3<S>& d) const
{
  return shapes[0]->localGetSupportingVertex(d);
}

//==============================================================================
template <typename S>
Vector3<S> MinkowskiDiff<S>::support1(const Vector3<S>& d) const
{
  return toshape0 * shapes[1]->localGetSupportingVertex(toshape1 * d);
}

//==============================================================================
template <typename S>
Vector3<S> MinkowskiDiff<S>::support(const Vector3<S>& d) const
{
  return support0(d) - support1(-d);
}

//==============================================================================
template <typename S>
Vector3<S> MinkowskiDiff<S>::support(const Vector3<S>& d, size_t index) const
{
  if (index)
    return support1(d);
  else
    return support0(d);
}

//==============================================================================
template <typename S>
Vector3<S> MinkowskiDiff<S>::support0(const Vector3<S>& d, const Vector3<S>& v) const
{
  if (d.dot(v) <= 0)
    return shapes[0]->localGetSupportingVertex(d);
  else
    return shapes[0]->localGetSupportingVertex(d) + v;
}

//==============================================================================
template <typename S>
Vector3<S> MinkowskiDiff<S>::support(const Vector3<S>& d, const Vector3<S>& v) const
{
  return support0(d, v) - support1(-d);
}

//==============================================================================
template <typename S>
Vector3<S> MinkowskiDiff<S>::support(const Vector3<S>& d, const Vector3<S>& v, size_t index) const
{
  if (index)
    return support1(d);
  else
    return support0(d, v);
}

//==============================================================================
template <typename S>
Support<S> MinkowskiDiff<S>::supportS(const Vector3<S>& d) const
{
  Support<S> s;
  s.v1 = support0(d); s.v2 = support1(-d); s.v = s.v1 - s.v2;
  return s;
}

//==============================================================================
template <typename S>
Support<S> MinkowskiDiff<S>::supportS(const Vector3<S>& d, const Vector3<S>& v) const
{
  Support<S> s;
  s.v1 = support0(d, v); s.v2 = support1(-d); s.v = s.v1 - s.v2;
  return s;
}

} // namespace detail
} // namespace fcl

#endif

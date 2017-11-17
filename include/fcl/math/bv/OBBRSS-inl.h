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

#ifndef FCL_BV_OBBRSS_INL_H
#define FCL_BV_OBBRSS_INL_H

#include "fcl/math/bv/OBBRSS.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT OBBRSS<double>;

//==============================================================================
extern template
OBBRSS<double> translate(const OBBRSS<double>& bv, const Vector3<double>& t);

//==============================================================================
template <typename S>
bool OBBRSS<S>::overlap(const OBBRSS<S>& other) const
{
  return obb.overlap(other.obb);
}

//==============================================================================
template <typename S>
bool OBBRSS<S>::overlap(const OBBRSS<S>& other,
                             OBBRSS<S>& /*overlap_part*/) const
{
  return overlap(other);
}

//==============================================================================
template <typename S>
bool OBBRSS<S>::contain(const Vector3<S>& p) const
{
  return obb.contain(p);
}

//==============================================================================
template <typename S>
OBBRSS<S>& OBBRSS<S>::operator +=(const Vector3<S>& p)
{
  obb += p;
  rss += p;
  return *this;
}

//==============================================================================
template <typename S>
OBBRSS<S>& OBBRSS<S>::operator +=(const OBBRSS<S>& other)
{
  *this = *this + other;
  return *this;
}

//==============================================================================
template <typename S>
OBBRSS<S> OBBRSS<S>::operator +(const OBBRSS<S>& other) const
{
  OBBRSS<S> result;
  result.obb = obb + other.obb;
  result.rss = rss + other.rss;
  return result;
}

//==============================================================================
template <typename S>
S OBBRSS<S>::width() const
{
  return obb.width();
}

//==============================================================================
template <typename S>
S OBBRSS<S>::height() const
{
  return obb.height();
}

//==============================================================================
template <typename S>
S OBBRSS<S>::depth() const
{
  return obb.depth();
}

//==============================================================================
template <typename S>
S OBBRSS<S>::volume() const
{
  return obb.volume();
}

//==============================================================================
template <typename S>
S OBBRSS<S>::size() const
{
  return obb.size();
}

//==============================================================================
template <typename S>
const Vector3<S> OBBRSS<S>::center() const
{
  return obb.center();
}

//==============================================================================
template <typename S>
S OBBRSS<S>::distance(const OBBRSS<S>& other,
                                Vector3<S>* P, Vector3<S>* Q) const
{
  return rss.distance(other.rss, P, Q);
}

//==============================================================================
template <typename S, typename DerivedA, typename DerivedB>
bool overlap(const Eigen::MatrixBase<DerivedA>& R0,
             const Eigen::MatrixBase<DerivedB>& T0,
             const OBBRSS<S>& b1, const OBBRSS<S>& b2)
{
  return overlap(R0, T0, b1.obb, b2.obb);
}

//==============================================================================
template <typename S, typename DerivedA, typename DerivedB>
S distance(
    const Eigen::MatrixBase<DerivedA>& R0,
    const Eigen::MatrixBase<DerivedB>& T0,
    const OBBRSS<S>& b1, const OBBRSS<S>& b2,
    Vector3<S>* P, Vector3<S>* Q)
{
  return distance(R0, T0, b1.rss, b2.rss, P, Q);
}

//==============================================================================
template <typename S>
OBBRSS<S> translate(const OBBRSS<S>& bv, const Vector3<S>& t)
{
  OBBRSS<S> res(bv);
  res.obb.To += t;
  res.rss.To += t;
  return res;
}

} // namespace fcl

#endif

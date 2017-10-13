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

#ifndef FCL_BV_AABB_INL_H
#define FCL_BV_AABB_INL_H

#include "fcl/math/bv/AABB.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT AABB<double>;

//==============================================================================
template <typename S>
AABB<S>::AABB()
  : min_(Vector3<S>::Constant(std::numeric_limits<S>::max())),
    max_(Vector3<S>::Constant(-std::numeric_limits<S>::max()))
{
  // Do nothing
}

//==============================================================================
template <typename S>
AABB<S>::AABB(const Vector3<S>& v) : min_(v), max_(v)
{
  // Do nothing
}

//==============================================================================
template <typename S>
AABB<S>::AABB(const Vector3<S>& a, const Vector3<S>& b)
  : min_(a.cwiseMin(b)),
    max_(a.cwiseMax(b))
{
  // Do nothing
}

//==============================================================================
template <typename S>
AABB<S>::AABB(const AABB<S>& core, const Vector3<S>& delta)
  : min_(core.min_ - delta),
    max_(core.max_ + delta)
{
  // Do nothing
}

//==============================================================================
template <typename S>
AABB<S>::AABB(
    const Vector3<S>& a,
    const Vector3<S>& b,
    const Vector3<S>& c)
  : min_(a.cwiseMin(b).cwiseMin(c)),
    max_(a.cwiseMax(b).cwiseMax(c))
{
  // Do nothing
}

//==============================================================================
template <typename S>
bool AABB<S>::overlap(const AABB<S>& other) const
{
  if ((min_.array() > other.max_.array()).any())
    return false;

  if ((max_.array() < other.min_.array()).any())
    return false;

  return true;
}

//==============================================================================
template <typename S>
bool AABB<S>::contain(const AABB<S>& other) const
{
  if ((min_.array() > other.min_.array()).any())
    return false;

  if ((max_.array() < other.max_.array()).any())
    return false;

  return true;
}

//==============================================================================
template <typename S>
bool AABB<S>::axisOverlap(const AABB<S>& other, int axis_id) const
{
  if(min_[axis_id] > other.max_[axis_id]) return false;

  if(max_[axis_id] < other.min_[axis_id]) return false;

  return true;
}

//==============================================================================
template <typename S>
bool AABB<S>::overlap(const AABB<S>& other, AABB<S>& overlap_part) const
{
  if(!overlap(other))
  {
    return false;
  }

  overlap_part.min_ = min_.cwiseMax(other.min_);
  overlap_part.max_ = max_.cwiseMin(other.max_);
  return true;
}

//==============================================================================
template <typename S>
bool AABB<S>::contain(const Vector3<S>& p) const
{
  if ((min_.array() > p.array()).any())
    return false;

  if ((max_.array() < p.array()).any())
    return false;

  return true;
}

//==============================================================================
template <typename S>
AABB<S>& AABB<S>::operator +=(const Vector3<S>& p)
{
  min_ = min_.cwiseMin(p);
  max_ = max_.cwiseMax(p);
  return *this;
}

//==============================================================================
template <typename S>
AABB<S>& AABB<S>::operator +=(const AABB<S>& other)
{
  min_ = min_.cwiseMin(other.min_);
  max_ = max_.cwiseMax(other.max_);
  return *this;
}

//==============================================================================
template <typename S>
AABB<S> AABB<S>::operator +(const AABB<S>& other) const
{
  AABB res(*this);
  return res += other;
}

//==============================================================================
template <typename S>
S AABB<S>::width() const
{
  return max_[0] - min_[0];
}

//==============================================================================
template <typename S>
S AABB<S>::height() const
{
  return max_[1] - min_[1];
}

//==============================================================================
template <typename S>
S AABB<S>::depth() const
{
  return max_[2] - min_[2];
}

//==============================================================================
template <typename S>
S AABB<S>::volume() const
{
  return width() * height() * depth();
}

//==============================================================================
template <typename S>
S AABB<S>::size() const
{
  return (max_ - min_).squaredNorm();
}

//==============================================================================
template <typename S>
S AABB<S>::radius() const
{
  return (max_ - min_).norm() / 2;
}

//==============================================================================
template <typename S>
Vector3<S> AABB<S>::center() const
{
  return (min_ + max_) * 0.5;
}

//==============================================================================
template <typename S>
S AABB<S>::distance(const AABB<S>& other, Vector3<S>* P, Vector3<S>* Q) const
{
  S result = 0;
  for(std::size_t i = 0; i < 3; ++i)
  {
    const S& amin = min_[i];
    const S& amax = max_[i];
    const S& bmin = other.min_[i];
    const S& bmax = other.max_[i];

    if(amin > bmax)
    {
      S delta = bmax - amin;
      result += delta * delta;
      if(P && Q)
      {
        (*P)[i] = amin;
        (*Q)[i] = bmax;
      }
    }
    else if(bmin > amax)
    {
      S delta = amax - bmin;
      result += delta * delta;
      if(P && Q)
      {
        (*P)[i] = amax;
        (*Q)[i] = bmin;
      }
    }
    else
    {
      if(P && Q)
      {
        if(bmin >= amin)
        {
          S t = 0.5 * (amax + bmin);
          (*P)[i] = t;
          (*Q)[i] = t;
        }
        else
        {
          S t = 0.5 * (amin + bmax);
          (*P)[i] = t;
          (*Q)[i] = t;
        }
      }
    }
  }

  return std::sqrt(result);
}

//==============================================================================
template <typename S>
S AABB<S>::distance(const AABB<S>& other) const
{
  S result = 0;
  for(std::size_t i = 0; i < 3; ++i)
  {
    const S& amin = min_[i];
    const S& amax = max_[i];
    const S& bmin = other.min_[i];
    const S& bmax = other.max_[i];

    if(amin > bmax)
    {
      S delta = bmax - amin;
      result += delta * delta;
    }
    else if(bmin > amax)
    {
      S delta = amax - bmin;
      result += delta * delta;
    }
  }

  return std::sqrt(result);
}

//==============================================================================
template <typename S>
bool AABB<S>::equal(const AABB<S>& other) const
{
  return min_.isApprox(other.min_, std::numeric_limits<S>::epsilon() * 100)
      && max_.isApprox(other.max_, std::numeric_limits<S>::epsilon() * 100);
}

//==============================================================================
template <typename S>
AABB<S>& AABB<S>::expand(const Vector3<S>& delta)
{
  min_ -= delta;
  max_ += delta;
  return *this;
}

//==============================================================================
template <typename S>
AABB<S>& AABB<S>::expand(const AABB<S>& core, S ratio)
{
  min_ = min_ * ratio - core.min_;
  max_ = max_ * ratio - core.max_;
  return *this;
}

//==============================================================================
template <typename S, typename Derived>
AABB<S> translate(
    const AABB<S>& aabb, const Eigen::MatrixBase<Derived>& t)
{
  AABB<S> res(aabb);
  res.min_ += t;
  res.max_ += t;
  return res;
}

} // namespace fcl

#endif

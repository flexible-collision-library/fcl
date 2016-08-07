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

#ifndef FCL_BV_AABB_H
#define FCL_BV_AABB_H

#include "fcl/data_types.h"

namespace fcl
{

/// @brief A class describing the AABB collision structure, which is a box in 3D
/// space determined by two diagonal points
template <typename ScalarT>
class AABB
{
public:

  using Scalar = ScalarT;

  /// @brief The min point in the AABB
  Vector3<Scalar> min_;

  /// @brief The max point in the AABB
  Vector3<Scalar> max_;

  /// @brief Creating an AABB with zero size (low bound +inf, upper bound -inf)
  AABB();

  /// @brief Creating an AABB at position v with zero size
  AABB(const Vector3<Scalar>& v);

  /// @brief Creating an AABB with two endpoints a and b
  AABB(const Vector3<Scalar>& a, const Vector3<Scalar>&b);

  /// @brief Creating an AABB centered as core and is of half-dimension delta
  AABB(const AABB<ScalarT>& core, const Vector3<Scalar>& delta);

  /// @brief Creating an AABB contains three points
  AABB(const Vector3<Scalar>& a, const Vector3<Scalar>& b, const Vector3<Scalar>& c);

  /// @brief Check whether two AABB are overlap
  bool overlap(const AABB<ScalarT>& other) const;

  /// @brief Check whether the AABB contains another AABB
  bool contain(const AABB<ScalarT>& other) const;

  /// @brief Check whether two AABB are overlapped along specific axis
  bool axisOverlap(const AABB<ScalarT>& other, int axis_id) const;

  /// @brief Check whether two AABB are overlap and return the overlap part
  bool overlap(const AABB<ScalarT>& other, AABB<ScalarT>& overlap_part) const;

  /// @brief Check whether the AABB contains a point
  bool contain(const Vector3<Scalar>& p) const;

  /// @brief Merge the AABB and a point
  AABB<ScalarT>& operator += (const Vector3<Scalar>& p);

  /// @brief Merge the AABB and another AABB
  AABB<ScalarT>& operator += (const AABB<ScalarT>& other);

  /// @brief Return the merged AABB of current AABB and the other one
  AABB<ScalarT> operator + (const AABB<ScalarT>& other) const;

  /// @brief Width of the AABB
  Scalar width() const;

  /// @brief Height of the AABB
  Scalar height() const;

  /// @brief Depth of the AABB
  Scalar depth() const;

  /// @brief Volume of the AABB
  Scalar volume() const;

  /// @brief Size of the AABB (used in BV_Splitter to order two AABBs)
  Scalar size() const;

  /// @brief Radius of the AABB
  Scalar radius() const;

  /// @brief Center of the AABB
  Vector3<ScalarT> center() const;

  /// @brief Distance between two AABBs; P and Q, should not be NULL, return the nearest points 
  Scalar distance(
      const AABB<ScalarT>& other, Vector3<Scalar>* P, Vector3<Scalar>* Q) const;

  /// @brief Distance between two AABBs
  Scalar distance(const AABB<ScalarT>& other) const;

  /// @brief whether two AABB are equal
  bool equal(const AABB<ScalarT>& other) const;

  /// @brief expand the half size of the AABB by delta, and keep the center unchanged.
  AABB<ScalarT>& expand(const Vector3<Scalar>& delta);

  /// @brief expand the aabb by increase the thickness of the plate by a ratio
  AABB<ScalarT>& expand(const AABB<ScalarT>& core, Scalar ratio);
};

using AABBf = AABB<float>;
using AABBd = AABB<double>;

/// @brief translate the center of AABB by t
template <typename ScalarT, typename Derived>
AABB<ScalarT> translate(
    const AABB<ScalarT>& aabb, const Eigen::MatrixBase<Derived>& t);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename ScalarT>
AABB<ScalarT>::AABB()
  : min_(Vector3<Scalar>::Constant(std::numeric_limits<Scalar>::max())),
    max_(Vector3<Scalar>::Constant(-std::numeric_limits<Scalar>::max()))
{
  // Do nothing
}

//==============================================================================
template <typename ScalarT>
AABB<ScalarT>::AABB(const Vector3<Scalar>& v) : min_(v), max_(v)
{
  // Do nothing
}

//==============================================================================
template <typename ScalarT>
AABB<ScalarT>::AABB(const Vector3<Scalar>& a, const Vector3<Scalar>& b)
  : min_(a.cwiseMin(b)),
    max_(a.cwiseMax(b))
{
  // Do nothing
}

//==============================================================================
template <typename ScalarT>
AABB<ScalarT>::AABB(const AABB<ScalarT>& core, const Vector3<Scalar>& delta)
  : min_(core.min_ - delta),
    max_(core.max_ + delta)
{
  // Do nothing
}

//==============================================================================
template <typename ScalarT>
AABB<ScalarT>::AABB(
    const Vector3<Scalar>& a,
    const Vector3<Scalar>& b,
    const Vector3<Scalar>& c)
  : min_(a.cwiseMin(b).cwiseMin(c)),
    max_(a.cwiseMax(b).cwiseMax(c))
{
  // Do nothing
}

//==============================================================================
template <typename ScalarT>
bool AABB<ScalarT>::overlap(const AABB<ScalarT>& other) const
{
  if ((min_.array() > other.max_.array()).any())
    return false;

  if ((max_.array() < other.min_.array()).any())
    return false;

  return true;
}

//==============================================================================
template <typename ScalarT>
bool AABB<ScalarT>::contain(const AABB<ScalarT>& other) const
{
  if ((min_.array() > other.min_.array()).any())
    return false;

  if ((max_.array() < other.max_.array()).any())
    return false;

  return true;
}

//==============================================================================
template <typename ScalarT>
bool AABB<ScalarT>::axisOverlap(const AABB<ScalarT>& other, int axis_id) const
{
  if(min_[axis_id] > other.max_[axis_id]) return false;

  if(max_[axis_id] < other.min_[axis_id]) return false;

  return true;
}

//==============================================================================
template <typename ScalarT>
bool AABB<ScalarT>::overlap(const AABB<ScalarT>& other, AABB<ScalarT>& overlap_part) const
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
template <typename ScalarT>
bool AABB<ScalarT>::contain(const Vector3<Scalar>& p) const
{
  if ((min_.array() > p.array()).any())
    return false;

  if ((max_.array() < p.array()).any())
    return false;

  return true;
}

//==============================================================================
template <typename ScalarT>
AABB<ScalarT>& AABB<ScalarT>::operator +=(const Vector3<Scalar>& p)
{
  min_ = min_.cwiseMin(p);
  max_ = max_.cwiseMax(p);
  return *this;
}

//==============================================================================
template <typename ScalarT>
AABB<ScalarT>& AABB<ScalarT>::operator +=(const AABB<ScalarT>& other)
{
  min_ = min_.cwiseMin(other.min_);
  max_ = max_.cwiseMax(other.max_);
  return *this;
}

//==============================================================================
template <typename ScalarT>
AABB<ScalarT> AABB<ScalarT>::operator +(const AABB<ScalarT>& other) const
{
  AABB res(*this);
  return res += other;
}

//==============================================================================
template <typename ScalarT>
ScalarT AABB<ScalarT>::width() const
{
  return max_[0] - min_[0];
}

//==============================================================================
template <typename ScalarT>
ScalarT AABB<ScalarT>::height() const
{
  return max_[1] - min_[1];
}

//==============================================================================
template <typename ScalarT>
ScalarT AABB<ScalarT>::depth() const
{
  return max_[2] - min_[2];
}

//==============================================================================
template <typename ScalarT>
ScalarT AABB<ScalarT>::volume() const
{
  return width() * height() * depth();
}

//==============================================================================
template <typename ScalarT>
ScalarT AABB<ScalarT>::size() const
{
  return (max_ - min_).squaredNorm();
}

//==============================================================================
template <typename ScalarT>
ScalarT AABB<ScalarT>::radius() const
{
  return (max_ - min_).norm() / 2;
}

//==============================================================================
template <typename ScalarT>
Vector3<ScalarT> AABB<ScalarT>::center() const
{
  return (min_ + max_) * 0.5;
}

//==============================================================================
template <typename ScalarT>
ScalarT AABB<ScalarT>::distance(const AABB<ScalarT>& other, Vector3<Scalar>* P, Vector3<Scalar>* Q) const
{
  Scalar result = 0;
  for(std::size_t i = 0; i < 3; ++i)
  {
    const Scalar& amin = min_[i];
    const Scalar& amax = max_[i];
    const Scalar& bmin = other.min_[i];
    const Scalar& bmax = other.max_[i];

    if(amin > bmax)
    {
      Scalar delta = bmax - amin;
      result += delta * delta;
      if(P && Q)
      {
        (*P)[i] = amin;
        (*Q)[i] = bmax;
      }
    }
    else if(bmin > amax)
    {
      Scalar delta = amax - bmin;
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
          Scalar t = 0.5 * (amax + bmin);
          (*P)[i] = t;
          (*Q)[i] = t;
        }
        else
        {
          Scalar t = 0.5 * (amin + bmax);
          (*P)[i] = t;
          (*Q)[i] = t;
        }
      }
    }
  }

  return std::sqrt(result);
}

//==============================================================================
template <typename ScalarT>
ScalarT AABB<ScalarT>::distance(const AABB<ScalarT>& other) const
{
  Scalar result = 0;
  for(std::size_t i = 0; i < 3; ++i)
  {
    const Scalar& amin = min_[i];
    const Scalar& amax = max_[i];
    const Scalar& bmin = other.min_[i];
    const Scalar& bmax = other.max_[i];

    if(amin > bmax)
    {
      Scalar delta = bmax - amin;
      result += delta * delta;
    }
    else if(bmin > amax)
    {
      Scalar delta = amax - bmin;
      result += delta * delta;
    }
  }

  return std::sqrt(result);
}

//==============================================================================
template <typename ScalarT>
bool AABB<ScalarT>::equal(const AABB<ScalarT>& other) const
{
  return min_.isApprox(other.min_, std::numeric_limits<Scalar>::epsilon() * 100)
      && max_.isApprox(other.max_, std::numeric_limits<Scalar>::epsilon() * 100);
}

//==============================================================================
template <typename ScalarT>
AABB<ScalarT>& AABB<ScalarT>::expand(const Vector3<Scalar>& delta)
{
  min_ -= delta;
  max_ += delta;
  return *this;
}

//==============================================================================
template <typename ScalarT>
AABB<ScalarT>& AABB<ScalarT>::expand(const AABB<ScalarT>& core, Scalar ratio)
{
  min_ = min_ * ratio - core.min_;
  max_ = max_ * ratio - core.max_;
  return *this;
}

//==============================================================================
template <typename ScalarT, typename Derived>
AABB<ScalarT> translate(
    const AABB<ScalarT>& aabb, const Eigen::MatrixBase<Derived>& t)
{
  AABB<ScalarT> res(aabb);
  res.min_ += t;
  res.max_ += t;
  return res;
}

} // namespace fcl

#endif

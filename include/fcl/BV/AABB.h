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

#ifndef FCL_AABB_H
#define FCL_AABB_H

#include "fcl/data_types.h"

namespace fcl
{

/// @brief A class describing the AABB collision structure, which is a box in 3D space determined by two diagonal points
class AABB
{
public:
  /// @brief The min point in the AABB
  Vector3d min_;
  /// @brief The max point in the AABB
  Vector3d max_;

  /// @brief Creating an AABB with zero size (low bound +inf, upper bound -inf)
  AABB();

  /// @brief Creating an AABB at position v with zero size
  AABB(const Vector3d& v) : min_(v), max_(v)
  {
  }

  /// @brief Creating an AABB with two endpoints a and b
  AABB(const Vector3d& a, const Vector3d&b) : min_(a.cwiseMin(b)),
                                        max_(a.cwiseMax(b))
  {
  }

  /// @brief Creating an AABB centered as core and is of half-dimension delta
  AABB(const AABB& core, const Vector3d& delta) : min_(core.min_ - delta),
                                               max_(core.max_ + delta)
  {
  }

  /// @brief Creating an AABB contains three points
  AABB(const Vector3d& a, const Vector3d& b, const Vector3d& c) : min_(a.cwiseMin(b).cwiseMin(c)),
                                                         max_(a.cwiseMax(b).cwiseMax(c))
  {
  }

  /// @brief Check whether two AABB are overlap
  inline bool overlap(const AABB& other) const
  {
    if ((min_.array() > other.max_.array()).any())
      return false;

    if ((max_.array() < other.min_.array()).any())
      return false;

    return true;
  }    

  /// @brief Check whether the AABB contains another AABB
  inline bool contain(const AABB& other) const
  {
    if ((min_.array() > other.min_.array()).any())
      return false;

    if ((max_.array() < other.max_.array()).any())
      return false;

    return true;
  }


  /// @brief Check whether two AABB are overlapped along specific axis
  inline bool axisOverlap(const AABB& other, int axis_id) const
  {
    if(min_[axis_id] > other.max_[axis_id]) return false;

    if(max_[axis_id] < other.min_[axis_id]) return false;

    return true;
  }

  /// @brief Check whether two AABB are overlap and return the overlap part
  inline bool overlap(const AABB& other, AABB& overlap_part) const
  {
    if(!overlap(other))
    {
      return false;
    }
    
    overlap_part.min_ = min_.cwiseMax(other.min_);
    overlap_part.max_ = max_.cwiseMin(other.max_);
    return true;
  }


  /// @brief Check whether the AABB contains a point
  inline bool contain(const Vector3d& p) const
  {
    if ((min_.array() > p.array()).any())
      return false;

    if ((max_.array() < p.array()).any())
      return false;

    return true;
  }

  /// @brief Merge the AABB and a point
  inline AABB& operator += (const Vector3d& p)
  {
    min_ = min_.cwiseMin(p);
    max_ = max_.cwiseMax(p);
    return *this;
  }

  /// @brief Merge the AABB and another AABB
  inline AABB& operator += (const AABB& other)
  {
    min_ = min_.cwiseMin(other.min_);
    max_ = max_.cwiseMax(other.max_);
    return *this;
  }

  /// @brief Return the merged AABB of current AABB and the other one
  inline AABB operator + (const AABB& other) const
  {
    AABB res(*this);
    return res += other;
  }

  /// @brief Width of the AABB
  inline FCL_REAL width() const
  {
    return max_[0] - min_[0];
  }

  /// @brief Height of the AABB
  inline FCL_REAL height() const
  {
    return max_[1] - min_[1];
  }

  /// @brief Depth of the AABB
  inline FCL_REAL depth() const
  {
    return max_[2] - min_[2];
  }

  /// @brief Volume of the AABB
  inline FCL_REAL volume() const
  {
    return width() * height() * depth();
  }  

  /// @brief Size of the AABB (used in BV_Splitter to order two AABBs)
  inline FCL_REAL size() const
  {
    return (max_ - min_).squaredNorm();
  }

  /// @brief Radius of the AABB
  inline FCL_REAL radius() const
  {
    return (max_ - min_).norm() / 2;
  }

  /// @brief Center of the AABB
  inline  Vector3d center() const
  {
    return (min_ + max_) * 0.5;
  }

  /// @brief Distance between two AABBs; P and Q, should not be NULL, return the nearest points 
  FCL_REAL distance(const AABB& other, Vector3d* P, Vector3d* Q) const;

  /// @brief Distance between two AABBs
  FCL_REAL distance(const AABB& other) const;

  /// @brief whether two AABB are equal
  inline bool equal(const AABB& other) const
  {
    return min_.isApprox(other.min_, std::numeric_limits<Vector3d::Scalar>::epsilon() * 100)
        && max_.isApprox(other.max_, std::numeric_limits<Vector3d::Scalar>::epsilon() * 100);
  }

  /// @brief expand the half size of the AABB by delta, and keep the center unchanged.
  inline AABB& expand(const Vector3d& delta)
  {
    min_ -= delta;
    max_ += delta;
    return *this;
  }

  /// @brief expand the aabb by increase the thickness of the plate by a ratio
  inline AABB& expand(const AABB& core, FCL_REAL ratio)
  {
    min_ = min_ * ratio - core.min_;
    max_ = max_ * ratio - core.max_;
    return *this;
  }  
};

/// @brief translate the center of AABB by t
static inline AABB translate(const AABB& aabb, const Vector3d& t)
{
  AABB res(aabb);
  res.min_ += t;
  res.max_ += t;
  return res;
}

}

#endif

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


#include "fcl/math/vec_3f.h"

namespace fcl
{

/// @brief A class describing the AABB collision structure, which is a box in 3D space determined by two diagonal points
class AABB
{
public:
  /// @brief The min point in the AABB
  Vec3f min_;
  /// @brief The max point in the AABB
  Vec3f max_;

  /// @brief Creating an AABB with zero size (low bound +inf, upper bound -inf)
  AABB();

  /// @brief Creating an AABB at position v with zero size
  AABB(const Vec3f& v) : min_(v), max_(v)
  {
  }

  /// @brief Creating an AABB with two endpoints a and b
  AABB(const Vec3f& a, const Vec3f&b) : min_(min(a, b)),
                                        max_(max(a, b))
  {
  }

  /// @brief Creating an AABB centered as core and is of half-dimension delta
  AABB(const AABB& core, const Vec3f& delta) : min_(core.min_ - delta),
                                               max_(core.max_ + delta)
  {
  }

  /// @brief Creating an AABB contains three points
  AABB(const Vec3f& a, const Vec3f& b, const Vec3f& c) : min_(min(min(a, b), c)),
                                                         max_(max(max(a, b), c))
  {
  }

  /// @brief Check whether two AABB are overlap
  inline bool overlap(const AABB& other) const
  {
    if(min_[0] > other.max_[0]) return false;
    if(min_[1] > other.max_[1]) return false;
    if(min_[2] > other.max_[2]) return false;

    if(max_[0] < other.min_[0]) return false;
    if(max_[1] < other.min_[1]) return false;
    if(max_[2] < other.min_[2]) return false;

    return true;
  }    

  /// @brief Check whether the AABB contains another AABB
  inline bool contain(const AABB& other) const
  {
    return (other.min_[0] >= min_[0]) && (other.max_[0] <= max_[0]) && (other.min_[1] >= min_[1]) && (other.max_[1] <= max_[1]) && (other.min_[2] >= min_[2]) && (other.max_[2] <= max_[2]);
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
    
    overlap_part.min_ = max(min_, other.min_);
    overlap_part.max_ = min(max_, other.max_);
    return true;
  }


  /// @brief Check whether the AABB contains a point
  inline bool contain(const Vec3f& p) const
  {
    if(p[0] < min_[0] || p[0] > max_[0]) return false;
    if(p[1] < min_[1] || p[1] > max_[1]) return false;
    if(p[2] < min_[2] || p[2] > max_[2]) return false;

    return true;
  }

  /// @brief Merge the AABB and a point
  inline AABB& operator += (const Vec3f& p)
  {
    min_.ubound(p);
    max_.lbound(p);
    return *this;
  }

  /// @brief Merge the AABB and another AABB
  inline AABB& operator += (const AABB& other)
  {
    min_.ubound(other.min_);
    max_.lbound(other.max_);
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
    return (max_ - min_).sqrLength();
  }

  /// @brief Radius of the AABB
  inline FCL_REAL radius() const
  {
    return (max_ - min_).length() / 2;
  }

  /// @brief Center of the AABB
  inline  Vec3f center() const
  {
    return (min_ + max_) * 0.5;
  }

  /// @brief Distance between two AABBs; P and Q, should not be NULL, return the nearest points 
  FCL_REAL distance(const AABB& other, Vec3f* P, Vec3f* Q) const;

  /// @brief Distance between two AABBs
  FCL_REAL distance(const AABB& other) const;

  /// @brief whether two AABB are equal
  inline bool equal(const AABB& other) const
  {
    return min_.equal(other.min_) && max_.equal(other.max_);
  }

  /// @brief expand the half size of the AABB by delta, and keep the center unchanged.
  inline AABB& expand(const Vec3f& delta)
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
static inline AABB translate(const AABB& aabb, const Vec3f& t)
{
  AABB res(aabb);
  res.min_ += t;
  res.max_ += t;
  return res;
}

}

#endif

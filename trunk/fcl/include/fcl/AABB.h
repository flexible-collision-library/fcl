/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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


#include "fcl/BVH_internal.h"
#include "fcl/vec_3f.h"

/** \brief Main namespace */
namespace fcl
{

/** \brief A class describing the AABB collision structure, which is a box in 3D space determined by two diagonal points */
class AABB
{
public:
  /** \brief The min point in the AABB */
  Vec3f min_;
  /** \brief The max point in the AABB */
  Vec3f max_;

  /** \brief Constructor creating an AABB with infinite size */
  AABB();

  /** \brief Constructor creating an AABB at position v with zero size */
  AABB(const Vec3f& v) : min_(v), max_(v)
  {
  }

  /** \brief Constructor creating an AABB with two endpoints a and b */
  AABB(const Vec3f& a, const Vec3f&b)
  {
    min_ = min(a, b);
    max_ = max(a, b);
  }

  /** \brief Check whether two AABB are overlap */
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

  /** \brief Check whether two AABB are overlapped along specific axis */
  inline bool axisOverlap(const AABB& other, int axis_id) const
  {
    if(min_[axis_id] > other.max_[axis_id]) return false;

    if(max_[axis_id] < other.min_[axis_id]) return false;

    return true;
  }

  /** \brief Check whether two AABB are overlap and return the overlap part */
  inline bool overlap(const AABB& other, AABB& overlap_part) const
  {
    if(!overlap(other))
      return false;
    
    overlap_part.min_ = max(min_, other.min_);
    overlap_part.max_ = min(max_, other.max_);
    return true;
  }


  /** \brief Check whether the AABB contains a point */
  inline bool contain(const Vec3f& p) const
  {
    if(p[0] < min_[0] || p[0] > max_[0]) return false;
    if(p[1] < min_[1] || p[1] > max_[1]) return false;
    if(p[2] < min_[2] || p[2] > max_[2]) return false;

    return true;
  }

  /** \brief Merge the AABB and a point */
  inline AABB& operator += (const Vec3f& p)
  {
    min_ = min(min_, p);
    max_ = max(max_, p);
    return *this;
  }

  /** \brief Merge the AABB and another AABB */
  inline AABB& operator += (const AABB& other)
  {
    min_ = min(min_, other.min_);
    max_ = max(max_, other.max_);
    return *this;
  }

  /** \brief Return the merged AABB of current AABB and the other one */
  inline AABB operator + (const AABB& other) const
  {
    AABB res(*this);
    return res += other;
  }

  /** \brief Width of the AABB */
  inline BVH_REAL width() const
  {
    return max_[0] - min_[0];
  }

  /** \brief Height of the AABB */
  inline BVH_REAL height() const
  {
    return max_[1] - min_[1];
  }

  /** \brief Depth of the AABB */
  inline BVH_REAL depth() const
  {
    return max_[2] - min_[2];
  }

  /** \brief Volume of the AABB */
  inline BVH_REAL volume() const
  {
    return width() * height() * depth();
  }  

  /** \brief Size of the AABB, for split order */
  inline BVH_REAL size() const
  {
    return (max_ - min_).sqrLength();
  }

  /** \brief Center of the AABB */
  inline  Vec3f center() const
  {
    return (min_ + max_) * 0.5;
  }

  /** \brief The distance between two AABB 
   * Not implemented.
   */
  BVH_REAL distance(const AABB& other, Vec3f* P = NULL, Vec3f* Q = NULL) const;
};

}

#endif

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

// This code is based on code developed by Stephane Redon at UNC and Inria for the CATCH library: http://graphics.ewha.ac.kr/CATCH/
/** \author Jia Pan */


#ifndef FCL_CCD_INTERVAL_H
#define FCL_CCD_INTERVAL_H

#include "fcl/data_types.h"

namespace fcl
{

/// @brief Interval class for [a, b]
struct Interval
{
  FCL_REAL i_[2];

  Interval() { i_[0] = i_[1] = 0; }

  explicit Interval(FCL_REAL v)
  {
    i_[0] = i_[1] = v;
  }

  /// @brief construct interval [left, right]
  Interval(FCL_REAL left, FCL_REAL right)
  {
    i_[0] = left; i_[1] = right;
  }

  /// @brief construct interval [left, right]
  inline void setValue(FCL_REAL a, FCL_REAL b)
  {
    i_[0] = a; i_[1] = b;
  }

  /// @brief construct zero interval [x, x]
  inline void setValue(FCL_REAL x)
  {
    i_[0] = i_[1] = x;
  }

  /// @brief access the interval endpoints: 0 for left, 1 for right end
  inline FCL_REAL operator [] (size_t i) const
  {
    return i_[i];
  }

  /// @brief access the interval endpoints: 0 for left, 1 for right end
  inline FCL_REAL& operator [] (size_t i)
  {
    return i_[i];
  }

  /// @brief whether two intervals are the same
  inline bool operator == (const Interval& other) const
  {
    if(i_[0] != other.i_[0]) return false;
    if(i_[1] != other.i_[1]) return false;
    return true;
  }

  /// @brief add two intervals
  inline Interval operator + (const Interval& other) const
  {
    return Interval(i_[0] + other.i_[0], i_[1] + other.i_[1]);
  }

  /// @brief minus another interval
  inline Interval operator - (const Interval& other) const
  {
    return Interval(i_[0] - other.i_[1], i_[1] - other.i_[0]);
  }

  inline Interval& operator += (const Interval& other)
  {
    i_[0] += other.i_[0];
    i_[1] += other.i_[1];
    return *this;
  }

  inline Interval& operator -= (const Interval& other)
  {
    i_[0] -= other.i_[1];
    i_[1] -= other.i_[0];
    return *this;
  }

  Interval operator * (const Interval& other) const;

  Interval& operator *= (const Interval& other);

  inline Interval operator * (FCL_REAL d) const
  {
    if(d >= 0) return Interval(i_[0] * d, i_[1] * d);
    return Interval(i_[1] * d, i_[0] * d);
  }

  inline Interval& operator *= (FCL_REAL d)
  {
    if(d >= 0)
    {
      i_[0] *= d;
      i_[1] *= d;
    }
    else
    {
      FCL_REAL tmp = i_[0];
      i_[0] = i_[1] * d;
      i_[1] = tmp * d;
    }

    return *this;
  }

  /// @brief other must not contain 0
  Interval operator / (const Interval& other) const;

  Interval& operator /= (const Interval& other);

  /// @brief determine whether the intersection between intervals is empty
  inline bool overlap(const Interval& other) const
  {
    if(i_[1] < other.i_[0]) return false;
    if(i_[0] > other.i_[1]) return false;
    return true;
  }

  inline bool intersect(const Interval& other)
  {
    if(i_[1] < other.i_[0]) return false;
    if(i_[0] > other.i_[1]) return false;
    if(i_[1] > other.i_[1]) i_[1] = other.i_[1];
    if(i_[0] < other.i_[0]) i_[0] = other.i_[0];
    return true;
  }

  inline Interval operator - () const
  {
    return Interval(-i_[1], -i_[0]);
  }

  /// @brief Return the nearest distance for points within the interval to zero
  inline FCL_REAL getAbsLower() const
  {
    if(i_[0] >= 0) return i_[0];
    if(i_[1] >= 0) return 0;
    return -i_[1];
  }

  /// @brief Return the farthest distance for points within the interval to zero
  inline FCL_REAL getAbsUpper() const
  {
    if(i_[0] + i_[1] >= 0) return i_[1];
    return i_[0];
  }


  inline bool contains(FCL_REAL v) const
  {
    if(v < i_[0]) return false;
    if(v > i_[1]) return false;
    return true;
  }

  /// @brief Compute the minimum interval contains v and original interval
  inline Interval& bound(FCL_REAL v)
  {
    if(v < i_[0]) i_[0] = v;
    if(v > i_[1]) i_[1] = v;
    return *this;
  }


  /// @brief Compute the minimum interval contains other and original interval
  inline Interval& bound(const Interval& other)
  {
    if(other.i_[0] < i_[0]) i_[0] = other.i_[0];
    if(other.i_[1] > i_[1]) i_[1] = other.i_[1];
    return *this;
  }


  void print() const;
  inline FCL_REAL center() const { return 0.5 * (i_[0] + i_[1]); }
  inline FCL_REAL diameter() const { return i_[1] -i_[0]; }
};

Interval bound(const Interval& i, FCL_REAL v);

Interval bound(const Interval& i, const Interval& other);

}
#endif

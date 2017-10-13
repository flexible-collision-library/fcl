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
/** @author Jia Pan */

#ifndef FCL_CCD_INTERVAL_H
#define FCL_CCD_INTERVAL_H

#include <iostream>
#include "fcl/common/types.h"

namespace fcl
{

/// @brief Interval class for [a, b]
template <typename S>
struct FCL_EXPORT Interval
{
  S i_[2];

  Interval();

  explicit Interval(S v);

  /// @brief construct interval [left, right]
  Interval(S left, S right);

  /// @brief construct interval [left, right]
  void setValue(S a, S b);

  /// @brief construct zero interval [x, x]
  void setValue(S x);

  /// @brief access the interval endpoints: 0 for left, 1 for right end
  S operator [] (size_t i) const;

  /// @brief access the interval endpoints: 0 for left, 1 for right end
  S& operator [] (size_t i);

  /// @brief whether two intervals are the same
  bool operator == (const Interval& other) const;

  /// @brief add two intervals
  Interval operator + (const Interval& other) const;

  /// @brief minus another interval
  Interval operator - (const Interval& other) const;

  Interval& operator += (const Interval& other);

  Interval& operator -= (const Interval& other);

  Interval operator * (const Interval& other) const;

  Interval& operator *= (const Interval& other);

  Interval operator * (S d) const;

  Interval& operator *= (S d);

  /// @brief other must not contain 0
  Interval operator / (const Interval& other) const;

  Interval& operator /= (const Interval& other);

  /// @brief determine whether the intersection between intervals is empty
  bool overlap(const Interval& other) const;

  bool intersect(const Interval& other);

  Interval operator - () const;

  /// @brief Return the nearest distance for points within the interval to zero
  S getAbsLower() const;

  /// @brief Return the farthest distance for points within the interval to zero
  S getAbsUpper() const;

  bool contains(S v) const;

  /// @brief Compute the minimum interval contains v and original interval
  Interval& bound(S v);

  /// @brief Compute the minimum interval contains other and original interval
  Interval& bound(const Interval& other);

  void print() const;

  S center() const;

  S diameter() const;
};

template <typename S>
FCL_EXPORT
Interval<S> bound(const Interval<S>& i, S v);

template <typename S>
FCL_EXPORT
Interval<S> bound(const Interval<S>& i, const Interval<S>& other);

} // namespace fcl

#include "fcl/math/motion/taylor_model/interval-inl.h"

#endif

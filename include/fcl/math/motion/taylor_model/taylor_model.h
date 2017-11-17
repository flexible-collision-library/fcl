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

// This code is based on code developed by Stephane Redon at UNC and Inria for
// the CATCH library: http://graphics.ewha.ac.kr/CATCH/

/** @author Jia Pan */

#ifndef FCL_CCD_TAYLOR_MODEL_H
#define FCL_CCD_TAYLOR_MODEL_H

#include <memory>
#include <iostream>
#include "fcl/math/constants.h"
#include "fcl/math/motion/taylor_model/interval.h"
#include "fcl/math/motion/taylor_model/time_interval.h"

namespace fcl
{

/// @brief TaylorModel implements a third order Taylor model, i.e., a cubic
/// approximation of a function over a time interval, with an interval
/// remainder. All the operations on two Taylor models assume their time
/// intervals are the same.
template <typename S>
class FCL_EXPORT TaylorModel
{
  /// @brief time interval
  std::shared_ptr<TimeInterval<S>> time_interval_;

  /// @brief Coefficients of the cubic polynomial approximation
  S coeffs_[4];

  /// @brief interval remainder
  Interval<S> r_;

public:

  void setTimeInterval(S l, S r);
  
  void setTimeInterval(const std::shared_ptr<TimeInterval<S>>& time_interval);

  const std::shared_ptr<TimeInterval<S>>& getTimeInterval() const;

  S coeff(std::size_t i) const;
  S& coeff(std::size_t i);
  const Interval<S>& remainder() const;
  Interval<S>& remainder();
  
  TaylorModel();
  TaylorModel(const std::shared_ptr<TimeInterval<S>>& time_interval);
  TaylorModel(S coeff, const std::shared_ptr<TimeInterval<S>>& time_interval);
  TaylorModel(S coeffs[3], const Interval<S>& r, const std::shared_ptr<TimeInterval<S>>& time_interval);
  TaylorModel(S c0, S c1, S c2, S c3, const Interval<S>& r, const std::shared_ptr<TimeInterval<S>>& time_interval);

  TaylorModel operator + (const TaylorModel& other) const;
  TaylorModel& operator += (const TaylorModel& other);

  TaylorModel operator - (const TaylorModel& other) const;
  TaylorModel& operator -= (const TaylorModel& other);

  TaylorModel operator + (S d) const;
  TaylorModel& operator += (S d);

  TaylorModel operator - (S d) const;
  TaylorModel& operator -= (S d);

  TaylorModel operator * (const TaylorModel& other) const;
  TaylorModel operator * (S d) const;
  TaylorModel& operator *= (const TaylorModel& other);
  TaylorModel& operator *= (S d);

  TaylorModel operator - () const;

  void print() const;

  Interval<S> getBound() const;
  Interval<S> getBound(S l, S r) const;

  Interval<S> getTightBound() const;
  Interval<S> getTightBound(S l, S r) const;

  Interval<S> getBound(S t) const;

  void setZero();
};

template <typename S>
FCL_EXPORT
TaylorModel<S> operator * (S d, const TaylorModel<S>& a);

template <typename S>
FCL_EXPORT
TaylorModel<S> operator + (S d, const TaylorModel<S>& a);

template <typename S>
FCL_EXPORT
TaylorModel<S> operator - (S d, const TaylorModel<S>& a);

/// @brief Generate Taylor model for cos(w t + q0)
template <typename S>
FCL_EXPORT
void generateTaylorModelForCosFunc(TaylorModel<S>& tm, S w, S q0);

/// @brief Generate Taylor model for sin(w t + q0)
template <typename S>
FCL_EXPORT
void generateTaylorModelForSinFunc(TaylorModel<S>& tm, S w, S q0);

/// @brief Generate Taylor model for p + v t
template <typename S>
FCL_EXPORT
void generateTaylorModelForLinearFunc(TaylorModel<S>& tm, S p, S v);

} // namespace fcl

#include "fcl/math/motion/taylor_model/taylor_model-inl.h"

#endif

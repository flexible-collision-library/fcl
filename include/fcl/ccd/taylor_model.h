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

#ifndef FCL_CCD_TAYLOR_MODEL_H
#define FCL_CCD_TAYLOR_MODEL_H

#include "fcl/ccd/interval.h"
#include <memory>

namespace fcl
{

struct TimeInterval
{
  /// @brief time interval and different powers
  Interval t_; // [t1, t2]
  Interval t2_; // [t1, t2]^2
  Interval t3_; // [t1, t2]^3
  Interval t4_; // [t1, t2]^4
  Interval t5_; // [t1, t2]^5
  Interval t6_; // [t1, t2]^6

  TimeInterval() {}
  TimeInterval(FCL_REAL l, FCL_REAL r)
  {
    setValue(l, r);
  }

  void setValue(FCL_REAL l, FCL_REAL r)
  {
    t_.setValue(l, r);
    t2_.setValue(l * t_[0], r * t_[1]);
    t3_.setValue(l * t2_[0], r * t2_[1]);
    t4_.setValue(l * t3_[0], r * t3_[1]);
    t5_.setValue(l * t4_[0], r * t4_[1]);
    t6_.setValue(l * t5_[0], r * t5_[1]);    
  }
};

/// @brief TaylorModel implements a third order Taylor model, i.e., a cubic approximation of a function
/// over a time interval, with an interval remainder.
/// All the operations on two Taylor models assume their time intervals are the same.
class TaylorModel
{
  /// @brief time interval
  std::shared_ptr<TimeInterval> time_interval_;

  /// @brief Coefficients of the cubic polynomial approximation
  FCL_REAL coeffs_[4];

  /// @brief interval remainder
  Interval r_;

public:

  void setTimeInterval(FCL_REAL l, FCL_REAL r)
  {
    time_interval_->setValue(l, r);
  }
  
  void setTimeInterval(const std::shared_ptr<TimeInterval>& time_interval)
  {
    time_interval_ = time_interval;
  }

  const std::shared_ptr<TimeInterval>& getTimeInterval() const
  {
    return time_interval_;
  }

  FCL_REAL coeff(std::size_t i) const { return coeffs_[i]; }
  FCL_REAL& coeff(std::size_t i) { return coeffs_[i]; }
  const Interval& remainder() const { return r_; }
  Interval& remainder() { return r_; }
  

  TaylorModel();
  TaylorModel(const std::shared_ptr<TimeInterval>& time_interval);
  TaylorModel(FCL_REAL coeff, const std::shared_ptr<TimeInterval>& time_interval);
  TaylorModel(FCL_REAL coeffs[3], const Interval& r, const std::shared_ptr<TimeInterval>& time_interval);
  TaylorModel(FCL_REAL c0, FCL_REAL c1, FCL_REAL c2, FCL_REAL c3, const Interval& r, const std::shared_ptr<TimeInterval>& time_interval);

  TaylorModel operator + (const TaylorModel& other) const;
  TaylorModel& operator += (const TaylorModel& other);

  TaylorModel operator - (const TaylorModel& other) const;
  TaylorModel& operator -= (const TaylorModel& other);

  TaylorModel operator + (FCL_REAL d) const;
  TaylorModel& operator += (FCL_REAL d);

  TaylorModel operator - (FCL_REAL d) const;
  TaylorModel& operator -= (FCL_REAL d);

  TaylorModel operator * (const TaylorModel& other) const;
  TaylorModel operator * (FCL_REAL d) const;
  TaylorModel& operator *= (const TaylorModel& other);
  TaylorModel& operator *= (FCL_REAL d);

  TaylorModel operator - () const;

  void print() const;

  Interval getBound() const;
  Interval getBound(FCL_REAL l, FCL_REAL r) const;

  Interval getTightBound() const;
  Interval getTightBound(FCL_REAL l, FCL_REAL r) const;

  Interval getBound(FCL_REAL t) const;

  void setZero();
};

TaylorModel operator * (FCL_REAL d, const TaylorModel& a);
TaylorModel operator + (FCL_REAL d, const TaylorModel& a);
TaylorModel operator - (FCL_REAL d, const TaylorModel& a);

/// @brief Generate Taylor model for cos(w t + q0)
void generateTaylorModelForCosFunc(TaylorModel& tm, FCL_REAL w, FCL_REAL q0);

/// @brief Generate Taylor model for sin(w t + q0)
void generateTaylorModelForSinFunc(TaylorModel& tm, FCL_REAL w, FCL_REAL q0);

/// @brief Generate Taylor model for p + v t
void generateTaylorModelForLinearFunc(TaylorModel& tm, FCL_REAL p, FCL_REAL v);

}

#endif

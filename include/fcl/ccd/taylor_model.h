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

#include <memory>
#include <iostream>
#include "fcl/math/constants.h"
#include "fcl/ccd/interval.h"

namespace fcl
{

template <typename Scalar>
struct TimeInterval
{
  /// @brief time Interval<Scalar> and different powers
  Interval<Scalar> t_; // [t1, t2]
  Interval<Scalar> t2_; // [t1, t2]^2
  Interval<Scalar> t3_; // [t1, t2]^3
  Interval<Scalar> t4_; // [t1, t2]^4
  Interval<Scalar> t5_; // [t1, t2]^5
  Interval<Scalar> t6_; // [t1, t2]^6

  TimeInterval<Scalar>() {}
  TimeInterval<Scalar>(Scalar l, Scalar r)
  {
    setValue(l, r);
  }

  void setValue(Scalar l, Scalar r)
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
template <typename Scalar>
class TaylorModel
{
  /// @brief time interval
  std::shared_ptr<TimeInterval<Scalar>> time_interval_;

  /// @brief Coefficients of the cubic polynomial approximation
  Scalar coeffs_[4];

  /// @brief interval remainder
  Interval<Scalar> r_;

public:

  void setTimeInterval(Scalar l, Scalar r)
  {
    time_interval_->setValue(l, r);
  }
  
  void setTimeInterval(const std::shared_ptr<TimeInterval<Scalar>>& time_interval)
  {
    time_interval_ = time_interval;
  }

  const std::shared_ptr<TimeInterval<Scalar>>& getTimeInterval() const
  {
    return time_interval_;
  }

  Scalar coeff(std::size_t i) const { return coeffs_[i]; }
  Scalar& coeff(std::size_t i) { return coeffs_[i]; }
  const Interval<Scalar>& remainder() const { return r_; }
  Interval<Scalar>& remainder() { return r_; }
  

  TaylorModel();
  TaylorModel(const std::shared_ptr<TimeInterval<Scalar>>& time_interval);
  TaylorModel(Scalar coeff, const std::shared_ptr<TimeInterval<Scalar>>& time_interval);
  TaylorModel(Scalar coeffs[3], const Interval<Scalar>& r, const std::shared_ptr<TimeInterval<Scalar>>& time_interval);
  TaylorModel(Scalar c0, Scalar c1, Scalar c2, Scalar c3, const Interval<Scalar>& r, const std::shared_ptr<TimeInterval<Scalar>>& time_interval);

  TaylorModel operator + (const TaylorModel& other) const;
  TaylorModel& operator += (const TaylorModel& other);

  TaylorModel operator - (const TaylorModel& other) const;
  TaylorModel& operator -= (const TaylorModel& other);

  TaylorModel operator + (Scalar d) const;
  TaylorModel& operator += (Scalar d);

  TaylorModel operator - (Scalar d) const;
  TaylorModel& operator -= (Scalar d);

  TaylorModel operator * (const TaylorModel& other) const;
  TaylorModel operator * (Scalar d) const;
  TaylorModel& operator *= (const TaylorModel& other);
  TaylorModel& operator *= (Scalar d);

  TaylorModel operator - () const;

  void print() const;

  Interval<Scalar> getBound() const;
  Interval<Scalar> getBound(Scalar l, Scalar r) const;

  Interval<Scalar> getTightBound() const;
  Interval<Scalar> getTightBound(Scalar l, Scalar r) const;

  Interval<Scalar> getBound(Scalar t) const;

  void setZero();
};

template <typename Scalar>
TaylorModel<Scalar> operator * (Scalar d, const TaylorModel<Scalar>& a);

template <typename Scalar>
TaylorModel<Scalar> operator + (Scalar d, const TaylorModel<Scalar>& a);

template <typename Scalar>
TaylorModel<Scalar> operator - (Scalar d, const TaylorModel<Scalar>& a);

/// @brief Generate Taylor model for cos(w t + q0)
template <typename Scalar>
void generateTaylorModelForCosFunc(TaylorModel<Scalar>& tm, Scalar w, Scalar q0);

/// @brief Generate Taylor model for sin(w t + q0)
template <typename Scalar>
void generateTaylorModelForSinFunc(TaylorModel<Scalar>& tm, Scalar w, Scalar q0);

/// @brief Generate Taylor model for p + v t
template <typename Scalar>
void generateTaylorModelForLinearFunc(TaylorModel<Scalar>& tm, Scalar p, Scalar v);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
TaylorModel<Scalar>::TaylorModel()
{
  coeffs_[0] = coeffs_[1] = coeffs_[2] = coeffs_[3] = 0;
}

//==============================================================================
template <typename Scalar>
TaylorModel<Scalar>::TaylorModel(const std::shared_ptr<TimeInterval<Scalar>>& time_interval) : time_interval_(time_interval)
{
  coeffs_[0] = coeffs_[1] = coeffs_[2] = coeffs_[3] = 0;
}

//==============================================================================
template <typename Scalar>
TaylorModel<Scalar>::TaylorModel(Scalar coeff, const std::shared_ptr<TimeInterval<Scalar>>& time_interval) : time_interval_(time_interval)
{
  coeffs_[0] = coeff;
  coeffs_[1] = coeffs_[2] = coeffs_[3] = r_[0] = r_[1] = 0;
}

//==============================================================================
template <typename Scalar>
TaylorModel<Scalar>::TaylorModel(Scalar coeffs[3], const Interval<Scalar>& r, const std::shared_ptr<TimeInterval<Scalar>>& time_interval) : time_interval_(time_interval)
{
  coeffs_[0] = coeffs[0];
  coeffs_[1] = coeffs[1];
  coeffs_[2] = coeffs[2];
  coeffs_[3] = coeffs[3];

  r_ = r;
}

//==============================================================================
template <typename Scalar>
TaylorModel<Scalar>::TaylorModel(Scalar c0, Scalar c1, Scalar c2, Scalar c3, const Interval<Scalar>& r, const std::shared_ptr<TimeInterval<Scalar>>& time_interval) : time_interval_(time_interval)
{
  coeffs_[0] = c0;
  coeffs_[1] = c1;
  coeffs_[2] = c2;
  coeffs_[3] = c3;

  r_ = r;
}

//==============================================================================
template <typename Scalar>
TaylorModel<Scalar> TaylorModel<Scalar>::operator + (Scalar d) const
{
  return TaylorModel(coeffs_[0] + d, coeffs_[1], coeffs_[2], coeffs_[3], r_, time_interval_);
}

//==============================================================================
template <typename Scalar>
TaylorModel<Scalar>& TaylorModel<Scalar>::operator += (Scalar d)
{
  coeffs_[0] += d;
  return *this;
}

//==============================================================================
template <typename Scalar>
TaylorModel<Scalar> TaylorModel<Scalar>::operator - (Scalar d) const
{
  return TaylorModel(coeffs_[0] - d, coeffs_[1], coeffs_[2], coeffs_[3], r_, time_interval_);
}

//==============================================================================
template <typename Scalar>
TaylorModel<Scalar>& TaylorModel<Scalar>::operator -= (Scalar d)
{
  coeffs_[0] -= d;
  return *this;
}

//==============================================================================
template <typename Scalar>
TaylorModel<Scalar> TaylorModel<Scalar>::operator + (const TaylorModel<Scalar>& other) const
{
  assert(other.time_interval_ == time_interval_);
  return TaylorModel(coeffs_[0] + other.coeffs_[0], coeffs_[1] + other.coeffs_[1], coeffs_[2] + other.coeffs_[2], coeffs_[3] + other.coeffs_[3], r_ + other.r_, time_interval_);
}

//==============================================================================
template <typename Scalar>
TaylorModel<Scalar> TaylorModel<Scalar>::operator - (const TaylorModel<Scalar>& other) const
{
  assert(other.time_interval_ == time_interval_);
  return TaylorModel(coeffs_[0] - other.coeffs_[0], coeffs_[1] - other.coeffs_[1], coeffs_[2] - other.coeffs_[2], coeffs_[3] - other.coeffs_[3], r_ - other.r_, time_interval_);
}

//==============================================================================
template <typename Scalar>
TaylorModel<Scalar>& TaylorModel<Scalar>::operator += (const TaylorModel<Scalar>& other)
{
  assert(other.time_interval_ == time_interval_);
  coeffs_[0] += other.coeffs_[0];
  coeffs_[1] += other.coeffs_[1];
  coeffs_[2] += other.coeffs_[2];
  coeffs_[3] += other.coeffs_[3];
  r_ += other.r_;
  return *this;
}

//==============================================================================
template <typename Scalar>
TaylorModel<Scalar>& TaylorModel<Scalar>::operator -= (const TaylorModel<Scalar>& other)
{
  assert(other.time_interval_ == time_interval_);
  coeffs_[0] -= other.coeffs_[0];
  coeffs_[1] -= other.coeffs_[1];
  coeffs_[2] -= other.coeffs_[2];
  coeffs_[3] -= other.coeffs_[3];
  r_ -= other.r_;
  return *this;
}

//==============================================================================
/// @brief Taylor model multiplication:
/// f(t) = c0+c1*t+c2*t^2+c3*t^3+[a,b]
/// g(t) = c0'+c1'*t+c2'*t^2+c3'*t^2+[c,d]
/// f(t)g(t)= c0c0'+
///           (c0c1'+c1c0')t+
///           (c0c2'+c1c1'+c2c0')t^2+
///           (c0c3'+c1c2'+c2c1'+c3c0')t^3+
///           [a,b][c,d]+
///           (c1c3'+c2c2'+c3c1')t^4+
///           (c2c3'+c3c2')t^5+
///           (c3c3')t^6+
///           (c0+c1*t+c2*t^2+c3*t^3)[c,d]+
///           (c0'+c1'*t+c2'*t^2+c3'*c^3)[a,b]
template <typename Scalar>
TaylorModel<Scalar> TaylorModel<Scalar>::operator * (const TaylorModel<Scalar>& other) const
{
  TaylorModel res(*this);
  res *= other;
  return res;
}

//==============================================================================
template <typename Scalar>
TaylorModel<Scalar> TaylorModel<Scalar>::operator * (Scalar d) const
{
  return TaylorModel(coeffs_[0] * d, coeffs_[1] * d, coeffs_[2] * d, coeffs_[3] * d,  r_ * d, time_interval_);
}

//==============================================================================
template <typename Scalar>
TaylorModel<Scalar>& TaylorModel<Scalar>::operator *= (const TaylorModel<Scalar>& other)
{
  assert(other.time_interval_ == time_interval_);
  register Scalar c0, c1, c2, c3;
  register Scalar c0b = other.coeffs_[0], c1b = other.coeffs_[1], c2b = other.coeffs_[2], c3b = other.coeffs_[3];

  const Interval<Scalar>& rb = other.r_;

  c0 = coeffs_[0] * c0b;
  c1 = coeffs_[0] * c1b + coeffs_[1] * c0b;
  c2 = coeffs_[0] * c2b + coeffs_[1] * c1b + coeffs_[2] * c0b;
  c3 = coeffs_[0] * c3b + coeffs_[1] * c2b + coeffs_[2] * c1b + coeffs_[3] * c0b;

  Interval<Scalar> remainder(r_ * rb);
  register Scalar tempVal = coeffs_[1] * c3b + coeffs_[2] * c2b + coeffs_[3] * c1b;
  remainder += time_interval_->t4_ * tempVal;

  tempVal = coeffs_[2] * c3b + coeffs_[3] * c2b;
  remainder += time_interval_->t5_ * tempVal;

  tempVal = coeffs_[3] * c3b;
  remainder += time_interval_->t6_ * tempVal;

  remainder += ((Interval<Scalar>(coeffs_[0]) + time_interval_->t_ * coeffs_[1] + time_interval_->t2_ * coeffs_[2] + time_interval_->t3_ * coeffs_[3]) * rb +
                (Interval<Scalar>(c0b) + time_interval_->t_ * c1b + time_interval_->t2_ * c2b + time_interval_->t3_ * c3b) * r_);

  coeffs_[0] = c0;
  coeffs_[1] = c1;
  coeffs_[2] = c2;
  coeffs_[3] = c3;

  r_ = remainder;

  return *this;
}

//==============================================================================
template <typename Scalar>
TaylorModel<Scalar>& TaylorModel<Scalar>::operator *= (Scalar d)
{
  coeffs_[0] *= d;
  coeffs_[1] *= d;
  coeffs_[2] *= d;
  coeffs_[3] *= d;
  r_ *= d;
  return *this;
}

//==============================================================================
template <typename Scalar>
TaylorModel<Scalar> TaylorModel<Scalar>::operator - () const
{
  return TaylorModel(-coeffs_[0], -coeffs_[1], -coeffs_[2], -coeffs_[3], -r_, time_interval_);
}

//==============================================================================
template <typename Scalar>
void TaylorModel<Scalar>::print() const
{
  std::cout << coeffs_[0] << "+" << coeffs_[1] << "*t+" << coeffs_[2] << "*t^2+" << coeffs_[3] << "*t^3+[" << r_[0] << "," << r_[1] << "]" << std::endl;
}

//==============================================================================
template <typename Scalar>
Interval<Scalar> TaylorModel<Scalar>::getBound(Scalar t) const
{
  return Interval<Scalar>(coeffs_[0] + t * (coeffs_[1] + t * (coeffs_[2] + t * coeffs_[3]))) + r_;
}

//==============================================================================
template <typename Scalar>
Interval<Scalar> TaylorModel<Scalar>::getBound(Scalar t0, Scalar t1) const
{
  Interval<Scalar> t(t0, t1);
  Interval<Scalar> t2(t0 * t0, t1 * t1);
  Interval<Scalar> t3(t0 * t2[0], t1 * t2[1]);

  return Interval<Scalar>(coeffs_[0]) + t * coeffs_[1] + t2 * coeffs_[2] + t3 * coeffs_[3] + r_;
}

//==============================================================================
template <typename Scalar>
Interval<Scalar> TaylorModel<Scalar>::getBound() const
{
  return Interval<Scalar>(coeffs_[0] + r_[0], coeffs_[1] + r_[1]) + time_interval_->t_ * coeffs_[1] + time_interval_->t2_ * coeffs_[2] + time_interval_->t3_ * coeffs_[3];
}

//==============================================================================
template <typename Scalar>
Interval<Scalar> TaylorModel<Scalar>::getTightBound(Scalar t0, Scalar t1) const
{
  if(t0 < time_interval_->t_[0]) t0 = time_interval_->t_[0];
  if(t1 > time_interval_->t_[1]) t1 = time_interval_->t_[1];

  if(coeffs_[3] == 0)
  {
    register Scalar a = -coeffs_[1] / (2 * coeffs_[2]);
    Interval<Scalar> polybounds;
    if(a <= t1 && a >= t0)
    {
      Scalar AQ = coeffs_[0] + a * (coeffs_[1] + a * coeffs_[2]);
      register Scalar t = t0;
      Scalar LQ = coeffs_[0] + t * (coeffs_[1] + t * coeffs_[2]);
      t = t1;
      Scalar RQ = coeffs_[0] + t * (coeffs_[1] + t * coeffs_[2]);

      Scalar minQ = LQ, maxQ = RQ;
      if(LQ > RQ)
      {
        minQ = RQ;
        maxQ = LQ;
      }

      if(minQ > AQ) minQ = AQ;
      if(maxQ < AQ) maxQ = AQ;

      polybounds.setValue(minQ, maxQ);
    }
    else
    {
      register Scalar t = t0;
      Scalar LQ = coeffs_[0] + t * (coeffs_[1] + t * coeffs_[2]);
      t = t1;
      Scalar RQ = coeffs_[0] + t * (coeffs_[1] + t * coeffs_[2]);

      if(LQ > RQ) polybounds.setValue(RQ, LQ);
      else polybounds.setValue(LQ, RQ);
    }

    return polybounds + r_;
  }
  else
  {
    register Scalar t = t0;
    Scalar LQ = coeffs_[0] + t * (coeffs_[1] + t * (coeffs_[2] + t * coeffs_[3]));
    t = t1;
    Scalar RQ = coeffs_[0] + t * (coeffs_[1] + t * (coeffs_[2] + t * coeffs_[3]));

    if(LQ > RQ)
    {
      Scalar tmp = LQ;
      LQ = RQ;
      RQ = tmp;
    }

    // derivative: c1+2*c2*t+3*c3*t^2

    Scalar delta = coeffs_[2] * coeffs_[2] - 3 * coeffs_[1] * coeffs_[3];
    if(delta < 0)
      return Interval<Scalar>(LQ, RQ) + r_;

    Scalar r1 = (-coeffs_[2]-sqrt(delta))/(3*coeffs_[3]);
    Scalar r2 = (-coeffs_[2]+sqrt(delta))/(3*coeffs_[3]);

    if(r1 <= t1 && r1 >= t0)
    {
      Scalar Q = coeffs_[0] + r1 * (coeffs_[1] + r1 * (coeffs_[2] + r1 * coeffs_[3]));
      if(Q < LQ) LQ = Q;
      else if(Q > RQ) RQ = Q;
    }

    if(r2 <= t1 && r2 >= t0)
    {
      Scalar Q = coeffs_[0] + r2 * (coeffs_[1] + r2 * (coeffs_[2] + r2 * coeffs_[3]));
      if(Q < LQ) LQ = Q;
      else if(Q > RQ) RQ = Q;
    }

    return Interval<Scalar>(LQ, RQ) + r_;
  }
}

//==============================================================================
template <typename Scalar>
Interval<Scalar> TaylorModel<Scalar>::getTightBound() const
{
  return getTightBound(time_interval_->t_[0], time_interval_->t_[1]);
}

//==============================================================================
template <typename Scalar>
void TaylorModel<Scalar>::setZero()
{
  coeffs_[0] = coeffs_[1] = coeffs_[2] = coeffs_[3] = 0;
  r_.setValue(0);
}

//==============================================================================
template <typename Scalar>
TaylorModel<Scalar> operator * (Scalar d, const TaylorModel<Scalar>& a)
{
  TaylorModel<Scalar> res(a);
  res.coeff(0) *= d;
  res.coeff(1) *= d;
  res.coeff(2) *= d;
  res.coeff(3) *= d;
  res.remainder() *= d;
  return res;
}

//==============================================================================
template <typename Scalar>
TaylorModel<Scalar> operator + (Scalar d, const TaylorModel<Scalar>& a)
{
  return a + d;
}

//==============================================================================
template <typename Scalar>
TaylorModel<Scalar> operator - (Scalar d, const TaylorModel<Scalar>& a)
{
  return -a + d;
}

//==============================================================================
template <typename Scalar>
void generateTaylorModelForCosFunc(TaylorModel<Scalar>& tm, Scalar w, Scalar q0)
{
  Scalar a = tm.getTimeInterval()->t_.center();
  Scalar t = w * a + q0;
  Scalar w2 = w * w;
  Scalar fa = cos(t);
  Scalar fda = -w*sin(t);
  Scalar fdda = -w2*fa;
  Scalar fddda = -w2*fda;

  tm.coeff(0) = fa-a*(fda-0.5*a*(fdda-1.0/3.0*a*fddda));
  tm.coeff(1) = fda-a*fdda+0.5*a*a*fddda;
  tm.coeff(2) = 0.5*(fdda-a*fddda);
  tm.coeff(3) = 1.0/6.0*fddda;

  // compute bounds for w^3 cos(wt+q0)/16, t \in [t0, t1]
  Interval<Scalar> fddddBounds;
  if(w == 0) fddddBounds.setValue(0);
  else
  {
    Scalar cosQL = cos(tm.getTimeInterval()->t_[0] * w + q0);
    Scalar cosQR = cos(tm.getTimeInterval()->t_[1] * w + q0);

    if(cosQL < cosQR) fddddBounds.setValue(cosQL, cosQR);
    else fddddBounds.setValue(cosQR, cosQL);

    // enlarge to handle round-off errors
    fddddBounds[0] -= 1e-15;
    fddddBounds[1] += 1e-15;

    // cos reaches maximum if there exists an integer k in [(w*t0+q0)/2pi, (w*t1+q0)/2pi];
    // cos reaches minimum if there exists an integer k in [(w*t0+q0-pi)/2pi, (w*t1+q0-pi)/2pi]

    Scalar k1 = (tm.getTimeInterval()->t_[0] * w + q0) / (2 * constants<Scalar>::pi());
    Scalar k2 = (tm.getTimeInterval()->t_[1] * w + q0) / (2 * constants<Scalar>::pi());


    if(w > 0)
    {
      if(ceil(k2) - floor(k1) > 1) fddddBounds[1] = 1;
      k1 -= 0.5;
      k2 -= 0.5;
      if(ceil(k2) - floor(k1) > 1) fddddBounds[0] = -1;
    }
    else
    {
      if(ceil(k1) - floor(k2) > 1) fddddBounds[1] = 1;
      k1 -= 0.5;
      k2 -= 0.5;
      if(ceil(k1) - floor(k2) > 1) fddddBounds[0] = -1;
    }
  }

  Scalar w4 = w2 * w2;
  fddddBounds *= w4;

  Scalar midSize = 0.5 * (tm.getTimeInterval()->t_[1] - tm.getTimeInterval()->t_[0]);
  Scalar midSize2 = midSize * midSize;
  Scalar midSize4 = midSize2 * midSize2;

  // [0, midSize4] * fdddBounds
  if(fddddBounds[0] > 0)
    tm.remainder().setValue(0, fddddBounds[1] * midSize4 * (1.0 / 24));
  else if(fddddBounds[0] < 0)
    tm.remainder().setValue(fddddBounds[0] * midSize4 * (1.0 / 24), 0);
  else
    tm.remainder().setValue(fddddBounds[0] * midSize4 * (1.0 / 24), fddddBounds[1] * midSize4 * (1.0 / 24));
}

//==============================================================================
template <typename Scalar>
void generateTaylorModelForSinFunc(TaylorModel<Scalar>& tm, Scalar w, Scalar q0)
{
  Scalar a = tm.getTimeInterval()->t_.center();
  Scalar t = w * a + q0;
  Scalar w2 = w * w;
  Scalar fa = sin(t);
  Scalar fda = w*cos(t);
  Scalar fdda = -w2*fa;
  Scalar fddda = -w2*fda;

  tm.coeff(0) = fa-a*(fda-0.5*a*(fdda-1.0/3.0*a*fddda));
  tm.coeff(1) = fda-a*fdda+0.5*a*a*fddda;
  tm.coeff(2) = 0.5*(fdda-a*fddda);
  tm.coeff(3) = 1.0/6.0*fddda;

  // compute bounds for w^3 sin(wt+q0)/16, t \in [t0, t1]

  Interval<Scalar> fddddBounds;

  if(w == 0) fddddBounds.setValue(0);
  else
  {
    Scalar sinQL = sin(w * tm.getTimeInterval()->t_[0] + q0);
    Scalar sinQR = sin(w * tm.getTimeInterval()->t_[1] + q0);

    if(sinQL < sinQR) fddddBounds.setValue(sinQL, sinQR);
    else fddddBounds.setValue(sinQR, sinQL);

    // enlarge to handle round-off errors
    fddddBounds[0] -= 1e-15;
    fddddBounds[1] += 1e-15;

    // sin reaches maximum if there exists an integer k in [(w*t0+q0-pi/2)/2pi, (w*t1+q0-pi/2)/2pi];
    // sin reaches minimum if there exists an integer k in [(w*t0+q0-pi-pi/2)/2pi, (w*t1+q0-pi-pi/2)/2pi]

    Scalar k1 = (tm.getTimeInterval()->t_[0] * w + q0) / (2 * constants<Scalar>::pi()) - 0.25;
    Scalar k2 = (tm.getTimeInterval()->t_[1] * w + q0) / (2 * constants<Scalar>::pi()) - 0.25;

    if(w > 0)
    {
      if(ceil(k2) - floor(k1) > 1) fddddBounds[1] = 1;
      k1 -= 0.5;
      k2 -= 0.5;
      if(ceil(k2) - floor(k1) > 1) fddddBounds[0] = -1;
    }
    else
    {
      if(ceil(k1) - floor(k2) > 1) fddddBounds[1] = 1;
      k1 -= 0.5;
      k2 -= 0.5;
      if(ceil(k1) - floor(k2) > 1) fddddBounds[0] = -1;
    }

    Scalar w4 = w2 * w2;
    fddddBounds *= w4;

    Scalar midSize = 0.5 * (tm.getTimeInterval()->t_[1] - tm.getTimeInterval()->t_[0]);
    Scalar midSize2 = midSize * midSize;
    Scalar midSize4 = midSize2 * midSize2;

    // [0, midSize4] * fdddBounds
    if(fddddBounds[0] > 0)
      tm.remainder().setValue(0, fddddBounds[1] * midSize4 * (1.0 / 24));
    else if(fddddBounds[0] < 0)
      tm.remainder().setValue(fddddBounds[0] * midSize4 * (1.0 / 24), 0);
    else
      tm.remainder().setValue(fddddBounds[0] * midSize4 * (1.0 / 24), fddddBounds[1] * midSize4 * (1.0 / 24));
  }
}

//==============================================================================
template <typename Scalar>
void generateTaylorModelForLinearFunc(TaylorModel<Scalar>& tm, Scalar p, Scalar v)
{
  tm.coeff(0) = p;
  tm.coeff(1) = v;
  tm.coeff(2) = 0;
  tm.coeff(3) = 0;
  tm.remainder()[0] = 0;
  tm.remainder()[1] = 0;
}

} // namespace fcl

#endif

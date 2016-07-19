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

#include "fcl/math/constants.h"
#include "fcl/ccd/taylor_model.h"
#include <cassert>
#include <iostream>
#include <cmath>

namespace fcl
{

TaylorModel::TaylorModel()
{
  coeffs_[0] = coeffs_[1] = coeffs_[2] = coeffs_[3] = 0;
}

TaylorModel::TaylorModel(const std::shared_ptr<TimeInterval>& time_interval) : time_interval_(time_interval)
{
  coeffs_[0] = coeffs_[1] = coeffs_[2] = coeffs_[3] = 0;
}

TaylorModel::TaylorModel(FCL_REAL coeff, const std::shared_ptr<TimeInterval>& time_interval) : time_interval_(time_interval)
{
  coeffs_[0] = coeff;
  coeffs_[1] = coeffs_[2] = coeffs_[3] = r_[0] = r_[1] = 0;
}

TaylorModel::TaylorModel(FCL_REAL coeffs[3], const Interval& r, const std::shared_ptr<TimeInterval>& time_interval) : time_interval_(time_interval)
{
  coeffs_[0] = coeffs[0];
  coeffs_[1] = coeffs[1];
  coeffs_[2] = coeffs[2];
  coeffs_[3] = coeffs[3];

  r_ = r;
}

TaylorModel::TaylorModel(FCL_REAL c0, FCL_REAL c1, FCL_REAL c2, FCL_REAL c3, const Interval& r, const std::shared_ptr<TimeInterval>& time_interval) : time_interval_(time_interval)
{
  coeffs_[0] = c0;
  coeffs_[1] = c1;
  coeffs_[2] = c2;
  coeffs_[3] = c3;

  r_ = r;
}

TaylorModel TaylorModel::operator + (FCL_REAL d) const
{
  return TaylorModel(coeffs_[0] + d, coeffs_[1], coeffs_[2], coeffs_[3], r_, time_interval_);
}

TaylorModel& TaylorModel::operator += (FCL_REAL d)
{
  coeffs_[0] += d;
  return *this;
}

TaylorModel TaylorModel::operator - (FCL_REAL d) const
{
  return TaylorModel(coeffs_[0] - d, coeffs_[1], coeffs_[2], coeffs_[3], r_, time_interval_);
}

TaylorModel& TaylorModel::operator -= (FCL_REAL d)
{
  coeffs_[0] -= d;
  return *this;
}


TaylorModel TaylorModel::operator + (const TaylorModel& other) const
{
  assert(other.time_interval_ == time_interval_);
  return TaylorModel(coeffs_[0] + other.coeffs_[0], coeffs_[1] + other.coeffs_[1], coeffs_[2] + other.coeffs_[2], coeffs_[3] + other.coeffs_[3], r_ + other.r_, time_interval_);
}

TaylorModel TaylorModel::operator - (const TaylorModel& other) const
{
  assert(other.time_interval_ == time_interval_);
  return TaylorModel(coeffs_[0] - other.coeffs_[0], coeffs_[1] - other.coeffs_[1], coeffs_[2] - other.coeffs_[2], coeffs_[3] - other.coeffs_[3], r_ - other.r_, time_interval_);
}
TaylorModel& TaylorModel::operator += (const TaylorModel& other)
{
  assert(other.time_interval_ == time_interval_);
  coeffs_[0] += other.coeffs_[0];
  coeffs_[1] += other.coeffs_[1];
  coeffs_[2] += other.coeffs_[2];
  coeffs_[3] += other.coeffs_[3];
  r_ += other.r_;
  return *this;
}

TaylorModel& TaylorModel::operator -= (const TaylorModel& other)
{
  assert(other.time_interval_ == time_interval_);
  coeffs_[0] -= other.coeffs_[0];
  coeffs_[1] -= other.coeffs_[1];
  coeffs_[2] -= other.coeffs_[2];
  coeffs_[3] -= other.coeffs_[3];
  r_ -= other.r_;
  return *this;
}

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
TaylorModel TaylorModel::operator * (const TaylorModel& other) const
{
  TaylorModel res(*this);
  res *= other;
  return res;
}

TaylorModel TaylorModel::operator * (FCL_REAL d) const
{
  return TaylorModel(coeffs_[0] * d, coeffs_[1] * d, coeffs_[2] * d, coeffs_[3] * d,  r_ * d, time_interval_);
}

TaylorModel& TaylorModel::operator *= (const TaylorModel& other)
{
  assert(other.time_interval_ == time_interval_);
  register FCL_REAL c0, c1, c2, c3;
  register FCL_REAL c0b = other.coeffs_[0], c1b = other.coeffs_[1], c2b = other.coeffs_[2], c3b = other.coeffs_[3];

  const Interval& rb = other.r_;

  c0 = coeffs_[0] * c0b;
  c1 = coeffs_[0] * c1b + coeffs_[1] * c0b;
  c2 = coeffs_[0] * c2b + coeffs_[1] * c1b + coeffs_[2] * c0b;
  c3 = coeffs_[0] * c3b + coeffs_[1] * c2b + coeffs_[2] * c1b + coeffs_[3] * c0b;

  Interval remainder(r_ * rb);
  register FCL_REAL tempVal = coeffs_[1] * c3b + coeffs_[2] * c2b + coeffs_[3] * c1b;
  remainder += time_interval_->t4_ * tempVal;

  tempVal = coeffs_[2] * c3b + coeffs_[3] * c2b;
  remainder += time_interval_->t5_ * tempVal;

  tempVal = coeffs_[3] * c3b;
  remainder += time_interval_->t6_ * tempVal;

  remainder += ((Interval(coeffs_[0]) + time_interval_->t_ * coeffs_[1] + time_interval_->t2_ * coeffs_[2] + time_interval_->t3_ * coeffs_[3]) * rb +
                (Interval(c0b) + time_interval_->t_ * c1b + time_interval_->t2_ * c2b + time_interval_->t3_ * c3b) * r_);

  coeffs_[0] = c0;
  coeffs_[1] = c1;
  coeffs_[2] = c2;
  coeffs_[3] = c3;

  r_ = remainder;

  return *this;
}

TaylorModel& TaylorModel::operator *= (FCL_REAL d)
{
  coeffs_[0] *= d;
  coeffs_[1] *= d;
  coeffs_[2] *= d;
  coeffs_[3] *= d;
  r_ *= d;
  return *this;
}


TaylorModel TaylorModel::operator - () const
{
  return TaylorModel(-coeffs_[0], -coeffs_[1], -coeffs_[2], -coeffs_[3], -r_, time_interval_);
}

void TaylorModel::print() const
{
  std::cout << coeffs_[0] << "+" << coeffs_[1] << "*t+" << coeffs_[2] << "*t^2+" << coeffs_[3] << "*t^3+[" << r_[0] << "," << r_[1] << "]" << std::endl;
}

Interval TaylorModel::getBound(FCL_REAL t) const
{
  return Interval(coeffs_[0] + t * (coeffs_[1] + t * (coeffs_[2] + t * coeffs_[3]))) + r_;
}

Interval TaylorModel::getBound(FCL_REAL t0, FCL_REAL t1) const
{
  Interval t(t0, t1);
  Interval t2(t0 * t0, t1 * t1);
  Interval t3(t0 * t2[0], t1 * t2[1]);

  return Interval(coeffs_[0]) + t * coeffs_[1] + t2 * coeffs_[2] + t3 * coeffs_[3] + r_;
}

Interval TaylorModel::getBound() const
{
  return Interval(coeffs_[0] + r_[0], coeffs_[1] + r_[1]) + time_interval_->t_ * coeffs_[1] + time_interval_->t2_ * coeffs_[2] + time_interval_->t3_ * coeffs_[3];
}

Interval TaylorModel::getTightBound(FCL_REAL t0, FCL_REAL t1) const
{
  if(t0 < time_interval_->t_[0]) t0 = time_interval_->t_[0];
  if(t1 > time_interval_->t_[1]) t1 = time_interval_->t_[1];

  if(coeffs_[3] == 0)
  {
    register FCL_REAL a = -coeffs_[1] / (2 * coeffs_[2]);
    Interval polybounds;
    if(a <= t1 && a >= t0)
    {
      FCL_REAL AQ = coeffs_[0] + a * (coeffs_[1] + a * coeffs_[2]);
      register FCL_REAL t = t0;
      FCL_REAL LQ = coeffs_[0] + t * (coeffs_[1] + t * coeffs_[2]);
      t = t1;
      FCL_REAL RQ = coeffs_[0] + t * (coeffs_[1] + t * coeffs_[2]);

      FCL_REAL minQ = LQ, maxQ = RQ;
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
      register FCL_REAL t = t0;
      FCL_REAL LQ = coeffs_[0] + t * (coeffs_[1] + t * coeffs_[2]);
      t = t1;
      FCL_REAL RQ = coeffs_[0] + t * (coeffs_[1] + t * coeffs_[2]);

      if(LQ > RQ) polybounds.setValue(RQ, LQ);
      else polybounds.setValue(LQ, RQ);
    }

    return polybounds + r_;
  }
  else
  {
    register FCL_REAL t = t0;
    FCL_REAL LQ = coeffs_[0] + t * (coeffs_[1] + t * (coeffs_[2] + t * coeffs_[3]));
    t = t1;
    FCL_REAL RQ = coeffs_[0] + t * (coeffs_[1] + t * (coeffs_[2] + t * coeffs_[3]));

    if(LQ > RQ)
    {
      FCL_REAL tmp = LQ;
      LQ = RQ;
      RQ = tmp;
    }

    // derivative: c1+2*c2*t+3*c3*t^2

    FCL_REAL delta = coeffs_[2] * coeffs_[2] - 3 * coeffs_[1] * coeffs_[3];
    if(delta < 0)
      return Interval(LQ, RQ) + r_;

    FCL_REAL r1 = (-coeffs_[2]-sqrt(delta))/(3*coeffs_[3]);
    FCL_REAL r2 = (-coeffs_[2]+sqrt(delta))/(3*coeffs_[3]);

    if(r1 <= t1 && r1 >= t0)
    {
      FCL_REAL Q = coeffs_[0] + r1 * (coeffs_[1] + r1 * (coeffs_[2] + r1 * coeffs_[3]));
      if(Q < LQ) LQ = Q;
      else if(Q > RQ) RQ = Q;
    }

    if(r2 <= t1 && r2 >= t0)
    {
      FCL_REAL Q = coeffs_[0] + r2 * (coeffs_[1] + r2 * (coeffs_[2] + r2 * coeffs_[3]));
      if(Q < LQ) LQ = Q;
      else if(Q > RQ) RQ = Q;
    }

    return Interval(LQ, RQ) + r_;
  }
}

Interval TaylorModel::getTightBound() const
{
  return getTightBound(time_interval_->t_[0], time_interval_->t_[1]);
}

void TaylorModel::setZero()
{
  coeffs_[0] = coeffs_[1] = coeffs_[2] = coeffs_[3] = 0;
  r_.setValue(0);
}

TaylorModel operator * (FCL_REAL d, const TaylorModel& a)
{
  TaylorModel res(a);
  res.coeff(0) *= d;
  res.coeff(1) *= d;
  res.coeff(2) *= d;
  res.coeff(3) *= d;
  res.remainder() *= d;
  return res;
}

TaylorModel operator + (FCL_REAL d, const TaylorModel& a)
{
  return a + d;
}

TaylorModel operator - (FCL_REAL d, const TaylorModel& a)
{
  return -a + d;
}


void generateTaylorModelForCosFunc(TaylorModel& tm, FCL_REAL w, FCL_REAL q0)
{
  FCL_REAL a = tm.getTimeInterval()->t_.center();
  FCL_REAL t = w * a + q0;
  FCL_REAL w2 = w * w;
  FCL_REAL fa = cos(t);
  FCL_REAL fda = -w*sin(t);
  FCL_REAL fdda = -w2*fa;
  FCL_REAL fddda = -w2*fda;

  tm.coeff(0) = fa-a*(fda-0.5*a*(fdda-1.0/3.0*a*fddda));
  tm.coeff(1) = fda-a*fdda+0.5*a*a*fddda;
  tm.coeff(2) = 0.5*(fdda-a*fddda);
  tm.coeff(3) = 1.0/6.0*fddda;

  // compute bounds for w^3 cos(wt+q0)/16, t \in [t0, t1]
  Interval fddddBounds;
  if(w == 0) fddddBounds.setValue(0);
  else
  {
    FCL_REAL cosQL = cos(tm.getTimeInterval()->t_[0] * w + q0);
    FCL_REAL cosQR = cos(tm.getTimeInterval()->t_[1] * w + q0);

    if(cosQL < cosQR) fddddBounds.setValue(cosQL, cosQR);
    else fddddBounds.setValue(cosQR, cosQL);

    // enlarge to handle round-off errors
    fddddBounds[0] -= 1e-15;
    fddddBounds[1] += 1e-15;

    // cos reaches maximum if there exists an integer k in [(w*t0+q0)/2pi, (w*t1+q0)/2pi];
    // cos reaches minimum if there exists an integer k in [(w*t0+q0-pi)/2pi, (w*t1+q0-pi)/2pi]

    FCL_REAL k1 = (tm.getTimeInterval()->t_[0] * w + q0) / (2 * constants::pi);
    FCL_REAL k2 = (tm.getTimeInterval()->t_[1] * w + q0) / (2 * constants::pi);


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

  FCL_REAL w4 = w2 * w2;
  fddddBounds *= w4;

  FCL_REAL midSize = 0.5 * (tm.getTimeInterval()->t_[1] - tm.getTimeInterval()->t_[0]);
  FCL_REAL midSize2 = midSize * midSize;
  FCL_REAL midSize4 = midSize2 * midSize2;

  // [0, midSize4] * fdddBounds
  if(fddddBounds[0] > 0)
    tm.remainder().setValue(0, fddddBounds[1] * midSize4 * (1.0 / 24));
  else if(fddddBounds[0] < 0)
    tm.remainder().setValue(fddddBounds[0] * midSize4 * (1.0 / 24), 0);
  else
    tm.remainder().setValue(fddddBounds[0] * midSize4 * (1.0 / 24), fddddBounds[1] * midSize4 * (1.0 / 24));
}

void generateTaylorModelForSinFunc(TaylorModel& tm, FCL_REAL w, FCL_REAL q0)
{
  FCL_REAL a = tm.getTimeInterval()->t_.center();
  FCL_REAL t = w * a + q0;
  FCL_REAL w2 = w * w;
  FCL_REAL fa = sin(t);
  FCL_REAL fda = w*cos(t);
  FCL_REAL fdda = -w2*fa;
  FCL_REAL fddda = -w2*fda;

  tm.coeff(0) = fa-a*(fda-0.5*a*(fdda-1.0/3.0*a*fddda));
  tm.coeff(1) = fda-a*fdda+0.5*a*a*fddda;
  tm.coeff(2) = 0.5*(fdda-a*fddda);
  tm.coeff(3) = 1.0/6.0*fddda;

  // compute bounds for w^3 sin(wt+q0)/16, t \in [t0, t1]

  Interval fddddBounds;

  if(w == 0) fddddBounds.setValue(0);
  else
  {
    FCL_REAL sinQL = sin(w * tm.getTimeInterval()->t_[0] + q0);
    FCL_REAL sinQR = sin(w * tm.getTimeInterval()->t_[1] + q0);

    if(sinQL < sinQR) fddddBounds.setValue(sinQL, sinQR);
    else fddddBounds.setValue(sinQR, sinQL);

    // enlarge to handle round-off errors
    fddddBounds[0] -= 1e-15;
    fddddBounds[1] += 1e-15;

    // sin reaches maximum if there exists an integer k in [(w*t0+q0-pi/2)/2pi, (w*t1+q0-pi/2)/2pi];
    // sin reaches minimum if there exists an integer k in [(w*t0+q0-pi-pi/2)/2pi, (w*t1+q0-pi-pi/2)/2pi]

    FCL_REAL k1 = (tm.getTimeInterval()->t_[0] * w + q0) / (2 * constants::pi) - 0.25;
    FCL_REAL k2 = (tm.getTimeInterval()->t_[1] * w + q0) / (2 * constants::pi) - 0.25;

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

    FCL_REAL w4 = w2 * w2;
    fddddBounds *= w4;

    FCL_REAL midSize = 0.5 * (tm.getTimeInterval()->t_[1] - tm.getTimeInterval()->t_[0]);
    FCL_REAL midSize2 = midSize * midSize;
    FCL_REAL midSize4 = midSize2 * midSize2;

    // [0, midSize4] * fdddBounds
    if(fddddBounds[0] > 0)
      tm.remainder().setValue(0, fddddBounds[1] * midSize4 * (1.0 / 24));
    else if(fddddBounds[0] < 0)
      tm.remainder().setValue(fddddBounds[0] * midSize4 * (1.0 / 24), 0);
    else
      tm.remainder().setValue(fddddBounds[0] * midSize4 * (1.0 / 24), fddddBounds[1] * midSize4 * (1.0 / 24));
  }
}

void generateTaylorModelForLinearFunc(TaylorModel& tm, FCL_REAL p, FCL_REAL v)
{
  tm.coeff(0) = p;
  tm.coeff(1) = v;
  tm.coeff(2) = 0;
  tm.coeff(3) = 0;
  tm.remainder()[0] = 0;
  tm.remainder()[1] = 0;
}

}

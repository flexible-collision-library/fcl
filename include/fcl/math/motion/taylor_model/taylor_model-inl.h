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

#ifndef FCL_CCD_TAYLOR_MODEL_INL_H
#define FCL_CCD_TAYLOR_MODEL_INL_H

#include "fcl/math/motion/taylor_model/taylor_model.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT TaylorModel<double>;

//==============================================================================
extern template
TaylorModel<double> operator * (double d, const TaylorModel<double>& a);

//==============================================================================
extern template
TaylorModel<double> operator + (double d, const TaylorModel<double>& a);

//==============================================================================
extern template
TaylorModel<double> operator - (double d, const TaylorModel<double>& a);

//==============================================================================
extern template
void generateTaylorModelForCosFunc(TaylorModel<double>& tm, double w, double q0);

//==============================================================================
extern template
void generateTaylorModelForSinFunc(TaylorModel<double>& tm, double w, double q0);

//==============================================================================
extern template
void generateTaylorModelForLinearFunc(TaylorModel<double>& tm, double p, double v);

//==============================================================================
template <typename S>
void TaylorModel<S>::setTimeInterval(S l, S r)
{
  time_interval_->setValue(l, r);
}

//==============================================================================
template <typename S>
void TaylorModel<S>::setTimeInterval(const std::shared_ptr<TimeInterval<S> >& time_interval)
{
  time_interval_ = time_interval;
}

//==============================================================================
template <typename S>
const std::shared_ptr<TimeInterval<S> >&TaylorModel<S>::getTimeInterval() const
{
  return time_interval_;
}

//==============================================================================
template <typename S>
S TaylorModel<S>::coeff(std::size_t i) const
{
  return coeffs_[i];
}

//==============================================================================
template <typename S>
S&TaylorModel<S>::coeff(std::size_t i)
{
  return coeffs_[i];
}

//==============================================================================
template <typename S>
const Interval<S>&TaylorModel<S>::remainder() const
{
  return r_;
}

//==============================================================================
template <typename S>
Interval<S>&TaylorModel<S>::remainder()
{
  return r_;
}

//==============================================================================
template <typename S>
TaylorModel<S>::TaylorModel()
{
  coeffs_[0] = coeffs_[1] = coeffs_[2] = coeffs_[3] = 0;
}

//==============================================================================
template <typename S>
TaylorModel<S>::TaylorModel(const std::shared_ptr<TimeInterval<S>>& time_interval) : time_interval_(time_interval)
{
  coeffs_[0] = coeffs_[1] = coeffs_[2] = coeffs_[3] = 0;
}

//==============================================================================
template <typename S>
TaylorModel<S>::TaylorModel(S coeff, const std::shared_ptr<TimeInterval<S>>& time_interval) : time_interval_(time_interval)
{
  coeffs_[0] = coeff;
  coeffs_[1] = coeffs_[2] = coeffs_[3] = r_[0] = r_[1] = 0;
}

//==============================================================================
template <typename S>
TaylorModel<S>::TaylorModel(S coeffs[3], const Interval<S>& r, const std::shared_ptr<TimeInterval<S>>& time_interval) : time_interval_(time_interval)
{
  coeffs_[0] = coeffs[0];
  coeffs_[1] = coeffs[1];
  coeffs_[2] = coeffs[2];
  coeffs_[3] = coeffs[3];

  r_ = r;
}

//==============================================================================
template <typename S>
TaylorModel<S>::TaylorModel(S c0, S c1, S c2, S c3, const Interval<S>& r, const std::shared_ptr<TimeInterval<S>>& time_interval) : time_interval_(time_interval)
{
  coeffs_[0] = c0;
  coeffs_[1] = c1;
  coeffs_[2] = c2;
  coeffs_[3] = c3;

  r_ = r;
}

//==============================================================================
template <typename S>
TaylorModel<S> TaylorModel<S>::operator + (S d) const
{
  return TaylorModel(coeffs_[0] + d, coeffs_[1], coeffs_[2], coeffs_[3], r_, time_interval_);
}

//==============================================================================
template <typename S>
TaylorModel<S>& TaylorModel<S>::operator += (S d)
{
  coeffs_[0] += d;
  return *this;
}

//==============================================================================
template <typename S>
TaylorModel<S> TaylorModel<S>::operator - (S d) const
{
  return TaylorModel(coeffs_[0] - d, coeffs_[1], coeffs_[2], coeffs_[3], r_, time_interval_);
}

//==============================================================================
template <typename S>
TaylorModel<S>& TaylorModel<S>::operator -= (S d)
{
  coeffs_[0] -= d;
  return *this;
}

//==============================================================================
template <typename S>
TaylorModel<S> TaylorModel<S>::operator + (const TaylorModel<S>& other) const
{
  assert(other.time_interval_ == time_interval_);
  return TaylorModel(coeffs_[0] + other.coeffs_[0], coeffs_[1] + other.coeffs_[1], coeffs_[2] + other.coeffs_[2], coeffs_[3] + other.coeffs_[3], r_ + other.r_, time_interval_);
}

//==============================================================================
template <typename S>
TaylorModel<S> TaylorModel<S>::operator - (const TaylorModel<S>& other) const
{
  assert(other.time_interval_ == time_interval_);
  return TaylorModel(coeffs_[0] - other.coeffs_[0], coeffs_[1] - other.coeffs_[1], coeffs_[2] - other.coeffs_[2], coeffs_[3] - other.coeffs_[3], r_ - other.r_, time_interval_);
}

//==============================================================================
template <typename S>
TaylorModel<S>& TaylorModel<S>::operator += (const TaylorModel<S>& other)
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
template <typename S>
TaylorModel<S>& TaylorModel<S>::operator -= (const TaylorModel<S>& other)
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
template <typename S>
TaylorModel<S> TaylorModel<S>::operator * (const TaylorModel<S>& other) const
{
  TaylorModel res(*this);
  res *= other;
  return res;
}

//==============================================================================
template <typename S>
TaylorModel<S> TaylorModel<S>::operator * (S d) const
{
  return TaylorModel(coeffs_[0] * d, coeffs_[1] * d, coeffs_[2] * d, coeffs_[3] * d,  r_ * d, time_interval_);
}

//==============================================================================
template <typename S>
TaylorModel<S>& TaylorModel<S>::operator *= (const TaylorModel<S>& other)
{
  assert(other.time_interval_ == time_interval_);
  S c0, c1, c2, c3;
  S c0b = other.coeffs_[0], c1b = other.coeffs_[1], c2b = other.coeffs_[2], c3b = other.coeffs_[3];

  const Interval<S>& rb = other.r_;

  c0 = coeffs_[0] * c0b;
  c1 = coeffs_[0] * c1b + coeffs_[1] * c0b;
  c2 = coeffs_[0] * c2b + coeffs_[1] * c1b + coeffs_[2] * c0b;
  c3 = coeffs_[0] * c3b + coeffs_[1] * c2b + coeffs_[2] * c1b + coeffs_[3] * c0b;

  Interval<S> remainder(r_ * rb);
  S tempVal = coeffs_[1] * c3b + coeffs_[2] * c2b + coeffs_[3] * c1b;
  remainder += time_interval_->t4_ * tempVal;

  tempVal = coeffs_[2] * c3b + coeffs_[3] * c2b;
  remainder += time_interval_->t5_ * tempVal;

  tempVal = coeffs_[3] * c3b;
  remainder += time_interval_->t6_ * tempVal;

  remainder += ((Interval<S>(coeffs_[0]) + time_interval_->t_ * coeffs_[1] + time_interval_->t2_ * coeffs_[2] + time_interval_->t3_ * coeffs_[3]) * rb +
                (Interval<S>(c0b) + time_interval_->t_ * c1b + time_interval_->t2_ * c2b + time_interval_->t3_ * c3b) * r_);

  coeffs_[0] = c0;
  coeffs_[1] = c1;
  coeffs_[2] = c2;
  coeffs_[3] = c3;

  r_ = remainder;

  return *this;
}

//==============================================================================
template <typename S>
TaylorModel<S>& TaylorModel<S>::operator *= (S d)
{
  coeffs_[0] *= d;
  coeffs_[1] *= d;
  coeffs_[2] *= d;
  coeffs_[3] *= d;
  r_ *= d;
  return *this;
}

//==============================================================================
template <typename S>
TaylorModel<S> TaylorModel<S>::operator - () const
{
  return TaylorModel(-coeffs_[0], -coeffs_[1], -coeffs_[2], -coeffs_[3], -r_, time_interval_);
}

//==============================================================================
template <typename S>
void TaylorModel<S>::print() const
{
  std::cout << coeffs_[0] << "+" << coeffs_[1] << "*t+" << coeffs_[2] << "*t^2+" << coeffs_[3] << "*t^3+[" << r_[0] << "," << r_[1] << "]" << std::endl;
}

//==============================================================================
template <typename S>
Interval<S> TaylorModel<S>::getBound(S t) const
{
  return Interval<S>(coeffs_[0] + t * (coeffs_[1] + t * (coeffs_[2] + t * coeffs_[3]))) + r_;
}

//==============================================================================
template <typename S>
Interval<S> TaylorModel<S>::getBound(S t0, S t1) const
{
  Interval<S> t(t0, t1);
  Interval<S> t2(t0 * t0, t1 * t1);
  Interval<S> t3(t0 * t2[0], t1 * t2[1]);

  return Interval<S>(coeffs_[0]) + t * coeffs_[1] + t2 * coeffs_[2] + t3 * coeffs_[3] + r_;
}

//==============================================================================
template <typename S>
Interval<S> TaylorModel<S>::getBound() const
{
  return Interval<S>(coeffs_[0] + r_[0], coeffs_[1] + r_[1]) + time_interval_->t_ * coeffs_[1] + time_interval_->t2_ * coeffs_[2] + time_interval_->t3_ * coeffs_[3];
}

//==============================================================================
template <typename S>
Interval<S> TaylorModel<S>::getTightBound(S t0, S t1) const
{
  if(t0 < time_interval_->t_[0]) t0 = time_interval_->t_[0];
  if(t1 > time_interval_->t_[1]) t1 = time_interval_->t_[1];

  if(coeffs_[3] == 0)
  {
    S a = -coeffs_[1] / (2 * coeffs_[2]);
    Interval<S> polybounds;
    if(a <= t1 && a >= t0)
    {
      S AQ = coeffs_[0] + a * (coeffs_[1] + a * coeffs_[2]);
      S t = t0;
      S LQ = coeffs_[0] + t * (coeffs_[1] + t * coeffs_[2]);
      t = t1;
      S RQ = coeffs_[0] + t * (coeffs_[1] + t * coeffs_[2]);

      S minQ = LQ, maxQ = RQ;
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
      S t = t0;
      S LQ = coeffs_[0] + t * (coeffs_[1] + t * coeffs_[2]);
      t = t1;
      S RQ = coeffs_[0] + t * (coeffs_[1] + t * coeffs_[2]);

      if(LQ > RQ) polybounds.setValue(RQ, LQ);
      else polybounds.setValue(LQ, RQ);
    }

    return polybounds + r_;
  }
  else
  {
    S t = t0;
    S LQ = coeffs_[0] + t * (coeffs_[1] + t * (coeffs_[2] + t * coeffs_[3]));
    t = t1;
    S RQ = coeffs_[0] + t * (coeffs_[1] + t * (coeffs_[2] + t * coeffs_[3]));

    if(LQ > RQ)
    {
      S tmp = LQ;
      LQ = RQ;
      RQ = tmp;
    }

    // derivative: c1+2*c2*t+3*c3*t^2

    S delta = coeffs_[2] * coeffs_[2] - 3 * coeffs_[1] * coeffs_[3];
    if(delta < 0)
      return Interval<S>(LQ, RQ) + r_;

    S r1 = (-coeffs_[2]-sqrt(delta))/(3*coeffs_[3]);
    S r2 = (-coeffs_[2]+sqrt(delta))/(3*coeffs_[3]);

    if(r1 <= t1 && r1 >= t0)
    {
      S Q = coeffs_[0] + r1 * (coeffs_[1] + r1 * (coeffs_[2] + r1 * coeffs_[3]));
      if(Q < LQ) LQ = Q;
      else if(Q > RQ) RQ = Q;
    }

    if(r2 <= t1 && r2 >= t0)
    {
      S Q = coeffs_[0] + r2 * (coeffs_[1] + r2 * (coeffs_[2] + r2 * coeffs_[3]));
      if(Q < LQ) LQ = Q;
      else if(Q > RQ) RQ = Q;
    }

    return Interval<S>(LQ, RQ) + r_;
  }
}

//==============================================================================
template <typename S>
Interval<S> TaylorModel<S>::getTightBound() const
{
  return getTightBound(time_interval_->t_[0], time_interval_->t_[1]);
}

//==============================================================================
template <typename S>
void TaylorModel<S>::setZero()
{
  coeffs_[0] = coeffs_[1] = coeffs_[2] = coeffs_[3] = 0;
  r_.setValue(0);
}

//==============================================================================
template <typename S>
TaylorModel<S> operator * (S d, const TaylorModel<S>& a)
{
  TaylorModel<S> res(a);
  res.coeff(0) *= d;
  res.coeff(1) *= d;
  res.coeff(2) *= d;
  res.coeff(3) *= d;
  res.remainder() *= d;
  return res;
}

//==============================================================================
template <typename S>
TaylorModel<S> operator + (S d, const TaylorModel<S>& a)
{
  return a + d;
}

//==============================================================================
template <typename S>
TaylorModel<S> operator - (S d, const TaylorModel<S>& a)
{
  return -a + d;
}

//==============================================================================
template <typename S>
void generateTaylorModelForCosFunc(TaylorModel<S>& tm, S w, S q0)
{
  S a = tm.getTimeInterval()->t_.center();
  S t = w * a + q0;
  S w2 = w * w;
  S fa = cos(t);
  S fda = -w*sin(t);
  S fdda = -w2*fa;
  S fddda = -w2*fda;

  tm.coeff(0) = fa-a*(fda-0.5*a*(fdda-1.0/3.0*a*fddda));
  tm.coeff(1) = fda-a*fdda+0.5*a*a*fddda;
  tm.coeff(2) = 0.5*(fdda-a*fddda);
  tm.coeff(3) = 1.0/6.0*fddda;

  // compute bounds for w^3 cos(wt+q0)/16, t \in [t0, t1]
  Interval<S> fddddBounds;
  if(w == 0) fddddBounds.setValue(0);
  else
  {
    S cosQL = cos(tm.getTimeInterval()->t_[0] * w + q0);
    S cosQR = cos(tm.getTimeInterval()->t_[1] * w + q0);

    if(cosQL < cosQR) fddddBounds.setValue(cosQL, cosQR);
    else fddddBounds.setValue(cosQR, cosQL);

    // enlarge to handle round-off errors
    fddddBounds[0] -= 1e-15;
    fddddBounds[1] += 1e-15;

    // cos reaches maximum if there exists an integer k in [(w*t0+q0)/2pi, (w*t1+q0)/2pi];
    // cos reaches minimum if there exists an integer k in [(w*t0+q0-pi)/2pi, (w*t1+q0-pi)/2pi]

    S k1 = (tm.getTimeInterval()->t_[0] * w + q0) / (2 * constants<S>::pi());
    S k2 = (tm.getTimeInterval()->t_[1] * w + q0) / (2 * constants<S>::pi());


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

  S w4 = w2 * w2;
  fddddBounds *= w4;

  S midSize = 0.5 * (tm.getTimeInterval()->t_[1] - tm.getTimeInterval()->t_[0]);
  S midSize2 = midSize * midSize;
  S midSize4 = midSize2 * midSize2;

  // [0, midSize4] * fdddBounds
  if(fddddBounds[0] > 0)
    tm.remainder().setValue(0, fddddBounds[1] * midSize4 * (1.0 / 24));
  else if(fddddBounds[0] < 0)
    tm.remainder().setValue(fddddBounds[0] * midSize4 * (1.0 / 24), 0);
  else
    tm.remainder().setValue(fddddBounds[0] * midSize4 * (1.0 / 24), fddddBounds[1] * midSize4 * (1.0 / 24));
}

//==============================================================================
template <typename S>
void generateTaylorModelForSinFunc(TaylorModel<S>& tm, S w, S q0)
{
  S a = tm.getTimeInterval()->t_.center();
  S t = w * a + q0;
  S w2 = w * w;
  S fa = sin(t);
  S fda = w*cos(t);
  S fdda = -w2*fa;
  S fddda = -w2*fda;

  tm.coeff(0) = fa-a*(fda-0.5*a*(fdda-1.0/3.0*a*fddda));
  tm.coeff(1) = fda-a*fdda+0.5*a*a*fddda;
  tm.coeff(2) = 0.5*(fdda-a*fddda);
  tm.coeff(3) = 1.0/6.0*fddda;

  // compute bounds for w^3 sin(wt+q0)/16, t \in [t0, t1]

  Interval<S> fddddBounds;

  if(w == 0) fddddBounds.setValue(0);
  else
  {
    S sinQL = sin(w * tm.getTimeInterval()->t_[0] + q0);
    S sinQR = sin(w * tm.getTimeInterval()->t_[1] + q0);

    if(sinQL < sinQR) fddddBounds.setValue(sinQL, sinQR);
    else fddddBounds.setValue(sinQR, sinQL);

    // enlarge to handle round-off errors
    fddddBounds[0] -= 1e-15;
    fddddBounds[1] += 1e-15;

    // sin reaches maximum if there exists an integer k in [(w*t0+q0-pi/2)/2pi, (w*t1+q0-pi/2)/2pi];
    // sin reaches minimum if there exists an integer k in [(w*t0+q0-pi-pi/2)/2pi, (w*t1+q0-pi-pi/2)/2pi]

    S k1 = (tm.getTimeInterval()->t_[0] * w + q0) / (2 * constants<S>::pi()) - 0.25;
    S k2 = (tm.getTimeInterval()->t_[1] * w + q0) / (2 * constants<S>::pi()) - 0.25;

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

    S w4 = w2 * w2;
    fddddBounds *= w4;

    S midSize = 0.5 * (tm.getTimeInterval()->t_[1] - tm.getTimeInterval()->t_[0]);
    S midSize2 = midSize * midSize;
    S midSize4 = midSize2 * midSize2;

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
template <typename S>
void generateTaylorModelForLinearFunc(TaylorModel<S>& tm, S p, S v)
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

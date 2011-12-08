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

#include "fcl/taylor_model.h"
#include <cassert>
#include <iostream>
#include <cmath>

namespace fcl
{

const BVH_REAL TaylorModel::PI_ = 3.1415626535;

TaylorModel::TaylorModel() {}

TaylorModel::TaylorModel(BVH_REAL coeff)
{
  coeffs_[0] = coeff;
  coeffs_[1] = coeffs_[2] = coeffs_[3] = r_[0] = r_[1] = 0;
}

TaylorModel::TaylorModel(BVH_REAL coeffs[3], const Interval& r)
{
  coeffs_[0] = coeffs[0];
  coeffs_[1] = coeffs[1];
  coeffs_[2] = coeffs[2];
  coeffs_[3] = coeffs[3];

  r_ = r;
}

TaylorModel::TaylorModel(BVH_REAL c0, BVH_REAL c1, BVH_REAL c2, BVH_REAL c3, const Interval& r)
{
  coeffs_[0] = c0;
  coeffs_[1] = c1;
  coeffs_[2] = c2;
  coeffs_[3] = c3;

  r_ = r;
}

void TaylorModel::setTimeInterval(BVH_REAL l, BVH_REAL r)
{
  t_.setValue(l, r);
  t2_.setValue(l * t_[0], r * t_[1]);
  t3_.setValue(l * t2_[0], r * t2_[1]);
  t4_.setValue(l * t3_[0], r * t3_[1]);
  t5_.setValue(l * t4_[0], r * t4_[1]);
  t6_.setValue(l * t5_[0], r * t5_[1]);
}

TaylorModel TaylorModel::operator + (const TaylorModel& other) const
{
  assert(other.t_ == t_);
  return TaylorModel(coeffs_[0] + other.coeffs_[0], coeffs_[1] + other.coeffs_[1], coeffs_[2] + other.coeffs_[2], coeffs_[3] + other.coeffs_[3], r_ + other.r_);
}

TaylorModel TaylorModel::operator - (const TaylorModel& other) const
{
  assert(other.t_ == t_);
  return TaylorModel(coeffs_[0] - other.coeffs_[0], coeffs_[1] - other.coeffs_[1], coeffs_[2] - other.coeffs_[2], coeffs_[3] - other.coeffs_[3], r_ - other.r_);
}
TaylorModel& TaylorModel::operator += (const TaylorModel& other)
{
  assert(other.t_ == t_);
  coeffs_[0] += other.coeffs_[0];
  coeffs_[1] += other.coeffs_[1];
  coeffs_[2] += other.coeffs_[2];
  coeffs_[3] += other.coeffs_[3];
  r_ += other.r_;
  return *this;
}

TaylorModel& TaylorModel::operator -= (const TaylorModel& other)
{
  assert(other.t_ == t_);
  coeffs_[0] -= other.coeffs_[0];
  coeffs_[1] -= other.coeffs_[1];
  coeffs_[2] -= other.coeffs_[2];
  coeffs_[3] -= other.coeffs_[3];
  r_ -= other.r_;
  return *this;
}

/** \brief Taylor model multiplication:
 * f(t) = c0+c1*t+c2*t^2+c3*t^3+[a,b]
 * g(t) = c0'+c1'*t+c2'*t^2+c3'*t^2+[c,d]
 * f(t)g(t)= c0c0'+
 *           (c0c1'+c1c0')t+
 *           (c0c2'+c1c1'+c2c0')t^2+
 *           (c0c3'+c1c2'+c2c1'+c3c0')t^3+
 *           [a,b][c,d]+
 *           (c1c3'+c2c2'+c3c1')t^4+
 *           (c2c3'+c3c2')t^5+
 *           (c3c3')t^6+
 *           (c0+c1*t+c2*t^2+c3*t^3)[c,d]+
 *           (c0'+c1'*t+c2'*t^2+c3'*c^3)[a,b]
 */

TaylorModel TaylorModel::operator * (const TaylorModel& other) const
{
  assert(other.t_ == t_);
  register BVH_REAL c0, c1, c2, c3;
  register BVH_REAL c0b = other.coeffs_[0], c1b = other.coeffs_[1], c2b = other.coeffs_[2], c3b = other.coeffs_[3];

  const Interval& rb = other.r_;

  c0 = coeffs_[0] * c0b;
  c1 = coeffs_[0] * c1b + coeffs_[1] * c0b;
  c2 = coeffs_[0] * c2b + coeffs_[1] * c1b + coeffs_[2] * c0b;
  c3 = coeffs_[0] * c3b + coeffs_[1] * c2b + coeffs_[2] * c1b + coeffs_[3] * c0b;

  Interval remainder(r_ * rb);
  register BVH_REAL tempVal = coeffs_[1] * c3b + coeffs_[2] * c2b + coeffs_[3] * c1b;
  remainder += t4_ * tempVal;

  tempVal = coeffs_[2] * c3b + coeffs_[3] * c2b;
  remainder += t5_ * tempVal;

  tempVal = coeffs_[3] * c3b;
  remainder += t6_ * tempVal;

  remainder += ((Interval(coeffs_[0]) + t_ * coeffs_[1] + t2_ * coeffs_[2] + t3_ * coeffs_[3]) * rb +
      (Interval(c0b) + t_ * c1b + t2_ * c2b + t3_ * c3b) * r_);

  return TaylorModel(c0, c1, c2, c3, remainder);
}

TaylorModel TaylorModel::operator * (BVH_REAL d) const
{
  return TaylorModel(coeffs_[0] * d, coeffs_[1] * d, coeffs_[2] * d, coeffs_[3] * d,  r_ * d);
}


TaylorModel TaylorModel::operator - () const
{
  return TaylorModel(-coeffs_[0], -coeffs_[1], -coeffs_[2], -coeffs_[3], -r_);
}

void TaylorModel::print() const
{
  std::cout << coeffs_[0] << "+" << coeffs_[1] << "*t+" << coeffs_[2] << "*t^2+" << coeffs_[3] << "*t^3+[" << r_[0] << "," << r_[1] << "]" << std::endl;
}

Interval TaylorModel::getBound(BVH_REAL t) const
{
  return Interval(coeffs_[0] + t * (coeffs_[1] + t * (coeffs_[2] + t * coeffs_[3]))) + r_;
}

Interval TaylorModel::getBound(BVH_REAL t0, BVH_REAL t1) const
{
  Interval t(t0, t1);
  Interval t2(t0 * t0, t1 * t1);
  Interval t3(t0 * t2[0], t1 * t2[1]);

  return Interval(coeffs_[0]) + t * coeffs_[1] + t2 * coeffs_[2] + t3 * coeffs_[3] + r_;
}

Interval TaylorModel::getBound() const
{
  return Interval(coeffs_[0] + r_[0], coeffs_[1] + r_[1]) + t_ * coeffs_[1] + t2_ * coeffs_[2] + t3_ * coeffs_[3];
}

Interval TaylorModel::getTightBound(BVH_REAL t0, BVH_REAL t1) const
{

  if(t0 < t_[0]) t0 = t_[0];
  if(t1 > t_[1]) t1 = t_[1];

  if(coeffs_[3] == 0)
  {
    register BVH_REAL a = -coeffs_[1] / (2 * coeffs_[2]);
    Interval polybounds;
    if(a <= t1 && a >= t0)
    {
      BVH_REAL AQ = coeffs_[0] + a * (coeffs_[1] + a * coeffs_[2]);
      register BVH_REAL t = t0;
      BVH_REAL LQ = coeffs_[0] + t * (coeffs_[1] + t * coeffs_[2]);
      t = t1;
      BVH_REAL RQ = coeffs_[0] + t * (coeffs_[1] + t * coeffs_[2]);

      BVH_REAL minQ = LQ, maxQ = RQ;
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
      register BVH_REAL t = t0;
      BVH_REAL LQ = coeffs_[0] + t * (coeffs_[1] + t * coeffs_[2]);
      t = t1;
      BVH_REAL RQ = coeffs_[0] + t * (coeffs_[1] + t * coeffs_[2]);

      if(LQ > RQ) polybounds.setValue(RQ, LQ);
      else polybounds.setValue(LQ, RQ);
    }

    return polybounds + r_;
  }
  else
  {
    register BVH_REAL t = t0;
    BVH_REAL LQ = coeffs_[0] + t * (coeffs_[1] + t * (coeffs_[2] + t * coeffs_[3]));
    t = t1;
    BVH_REAL RQ = coeffs_[0] + t * (coeffs_[1] + t * (coeffs_[2] + t * coeffs_[3]));

    if(LQ > RQ)
    {
      BVH_REAL tmp = LQ;
      LQ = RQ;
      RQ = tmp;
    }

    // derivative: c1+2*c2*t+3*c3*t^2

    BVH_REAL delta = coeffs_[2] * coeffs_[2] - 3 * coeffs_[1] * coeffs_[3];
    if(delta < 0)
      return Interval(LQ, RQ) + r_;

    BVH_REAL r1=(-coeffs_[2]-sqrt(delta))/(3*coeffs_[3]);
    BVH_REAL r2=(-coeffs_[2]+sqrt(delta))/(3*coeffs_[3]);

    if(r1 <= t1 && r1 >= t0)
    {
      BVH_REAL Q = coeffs_[0] + r1 * (coeffs_[1] + r1 * (coeffs_[2] + r1 * coeffs_[3]));
      if(Q < LQ) LQ = Q;
      else if(Q > RQ) RQ = Q;
    }

    if(r2 <= t1 && r2 >= t0)
    {
      BVH_REAL Q = coeffs_[0] + r2 * (coeffs_[1] + r2 * (coeffs_[2] + r2 * coeffs_[3]));
      if(Q < LQ) LQ = Q;
      else if(Q > RQ) RQ = Q;
    }

    return Interval(LQ, RQ) + r_;
  }
}

Interval TaylorModel::getTightBound() const
{
  return getTightBound(t_[0], t_[1]);
}

void TaylorModel::setZero()
{
  coeffs_[0] = coeffs_[1] = coeffs_[2] = coeffs_[3] = 0;
  r_.setValue(0);
}


void generateTaylorModelForCosFunc(TaylorModel& tm, BVH_REAL w, BVH_REAL q0)
{
  BVH_REAL a = tm.t_.center();
  BVH_REAL t = w * a + q0;
  BVH_REAL w2 = w * w;
  BVH_REAL fa = cos(t);
  BVH_REAL fda = -w*sin(t);
  BVH_REAL fdda = -w2*fa;
  BVH_REAL fddda = -w2*fda;

  tm.coeffs_[0] = fa-a*(fda-0.5*a*(fdda-1.0/3.0*a*fddda));
  tm.coeffs_[1] = fda-a*fdda+0.5*a*a*fddda;
  tm.coeffs_[2] = 0.5*(fdda-a*fddda);
  tm.coeffs_[3] = 1.0/6.0*fddda;

  // compute bounds for w^3 cos(wt+q0)/16, t \in [t0, t1]
  Interval fddddBounds;
  if(w == 0) fddddBounds.setValue(0);
  else
  {
    BVH_REAL cosQL = cos(tm.t_[0] * w + q0);
    BVH_REAL cosQR = cos(tm.t_[1] * w + q0);

    if(cosQL < cosQR) fddddBounds.setValue(cosQL, cosQR);
    else fddddBounds.setValue(cosQR, cosQL);

    // enlarge to handle round-off errors
    fddddBounds[0] -= 1e-15;
    fddddBounds[1] += 1e-15;

    // cos reaches maximum if there exists an integer k in [(w*t0+q0)/2pi, (w*t1+q0)/2pi];
    // cos reaches minimum if there exists an integer k in [(w*t0+q0-pi)/2pi, (w*t1+q0-pi)/2pi]

    BVH_REAL k1 = (tm.t_[0] * w + q0) / (2 * TaylorModel::PI_);
    BVH_REAL k2 = (tm.t_[1] * w + q0) / (2 * TaylorModel::PI_);


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

  BVH_REAL w4 = w2 * w2;
  fddddBounds *= w4;

  BVH_REAL midSize = 0.5 * (tm.t_[1] - tm.t_[0]);
  BVH_REAL midSize2 = midSize * midSize;
  BVH_REAL midSize4 = midSize2 * midSize2;

  // [0, midSize4] * fdddBounds
  if(fddddBounds[0] > 0)
    tm.r_.setValue(0, fddddBounds[1] * midSize4 * (1.0 / 24));
  else if(fddddBounds[0] < 0)
    tm.r_.setValue(fddddBounds[0] * midSize4 * (1.0 / 24), 0);
  else
    tm.r_.setValue(fddddBounds[0] * midSize4 * (1.0 / 24), fddddBounds[1] * midSize4 * (1.0 / 24));
}

void generateTaylorModelForSinFunc(TaylorModel& tm, BVH_REAL w, BVH_REAL q0)
{
  BVH_REAL a = tm.t_.center();
  BVH_REAL t = w * a + q0;
  BVH_REAL w2 = w * w;
  BVH_REAL fa = sin(t);
  BVH_REAL fda = w*cos(t);
  BVH_REAL fdda = -w2*fa;
  BVH_REAL fddda = -w2*fda;

  tm.coeffs_[0] = fa-a*(fda-0.5*a*(fdda-1.0/3.0*a*fddda));
  tm.coeffs_[1] = fda-a*fdda+0.5*a*a*fddda;
  tm.coeffs_[2] = 0.5*(fdda-a*fddda);
  tm.coeffs_[3] = 1.0/6.0*fddda;

  // compute bounds for w^3 sin(wt+q0)/16, t \in [t0, t1]

  Interval fddddBounds;

  if(w == 0) fddddBounds.setValue(0);
  else
  {
    BVH_REAL sinQL = sin(w * tm.t_[0] + q0);
    BVH_REAL sinQR = sin(w * tm.t_[1] + q0);

    if(sinQL < sinQR) fddddBounds.setValue(sinQL, sinQR);
    else fddddBounds.setValue(sinQR, sinQL);

    // enlarge to handle round-off errors
    fddddBounds[0] -= 1e-15;
    fddddBounds[1] += 1e-15;

    // sin reaches maximum if there exists an integer k in [(w*t0+q0-pi/2)/2pi, (w*t1+q0-pi/2)/2pi];
    // sin reaches minimum if there exists an integer k in [(w*t0+q0-pi-pi/2)/2pi, (w*t1+q0-pi-pi/2)/2pi]

    BVH_REAL k1 = (tm.t_[0] * w + q0) / (2 * TaylorModel::PI_) - 0.25;
    BVH_REAL k2 = (tm.t_[1] * w + q0) / (2 * TaylorModel::PI_) - 0.25;

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

    BVH_REAL w4 = w2 * w2;
    fddddBounds *= w4;

    BVH_REAL midSize = 0.5 * (tm.t_[1] - tm.t_[0]);
    BVH_REAL midSize2 = midSize * midSize;
    BVH_REAL midSize4 = midSize2 * midSize2;

    // [0, midSize4] * fdddBounds
    if(fddddBounds[0] > 0)
      tm.r_.setValue(0, fddddBounds[1] * midSize4 * (1.0 / 24));
    else if(fddddBounds[0] < 0)
      tm.r_.setValue(fddddBounds[0] * midSize4 * (1.0 / 24), 0);
    else
      tm.r_.setValue(fddddBounds[0] * midSize4 * (1.0 / 24), fddddBounds[1] * midSize4 * (1.0 / 24));
  }
}

void generateTaylorModelForLinearFunc(TaylorModel& tm, BVH_REAL p, BVH_REAL v)
{
  tm.coeffs_[0] = p;
  tm.coeffs_[1] = v;
  tm.coeffs_[2] = tm.coeffs_[3] = tm.r_[0] = tm.r_[1] = 0;
}

}

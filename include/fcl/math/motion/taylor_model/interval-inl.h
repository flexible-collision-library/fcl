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

#ifndef FCL_CCD_INTERVAL_INL_H
#define FCL_CCD_INTERVAL_INL_H

#include "fcl/math/motion/taylor_model/interval.h"

namespace fcl
{

//==============================================================================
extern template
struct Interval<double>;

//==============================================================================
extern template
Interval<double> bound(const Interval<double>& i, double v);

//==============================================================================
extern template
Interval<double> bound(const Interval<double>& i, const Interval<double>& other);

//==============================================================================
template <typename S>
Interval<S>::Interval()
{
  i_[0] = i_[1] = 0;
}

//==============================================================================
template <typename S>
Interval<S>::Interval(S v)
{
  i_[0] = i_[1] = v;
}

//==============================================================================
template <typename S>
Interval<S>::Interval(S left, S right)
{
  i_[0] = left; i_[1] = right;
}

//==============================================================================
template <typename S>
void Interval<S>::setValue(S a, S b)
{
  i_[0] = a; i_[1] = b;
}

//==============================================================================
template <typename S>
void Interval<S>::setValue(S x)
{
  i_[0] = i_[1] = x;
}

//==============================================================================
template <typename S>
S Interval<S>::operator [] (size_t i) const
{
  return i_[i];
}

//==============================================================================
template <typename S>
S& Interval<S>::operator [] (size_t i)
{
  return i_[i];
}

//==============================================================================
template <typename S>
bool Interval<S>::operator == (const Interval& other) const
{
  if(i_[0] != other.i_[0]) return false;
  if(i_[1] != other.i_[1]) return false;
  return true;
}

//==============================================================================
template <typename S>
Interval<S> Interval<S>::operator + (const Interval<S>& other) const
{
  return Interval(i_[0] + other.i_[0], i_[1] + other.i_[1]);
}

//==============================================================================
template <typename S>
Interval<S> Interval<S>::operator - (const Interval<S>& other) const
{
  return Interval(i_[0] - other.i_[1], i_[1] - other.i_[0]);
}

//==============================================================================
template <typename S>
Interval<S>& Interval<S>::operator += (const Interval<S>& other)
{
  i_[0] += other.i_[0];
  i_[1] += other.i_[1];
  return *this;
}

//==============================================================================
template <typename S>
Interval<S>& Interval<S>::operator -= (const Interval<S>& other)
{
  i_[0] -= other.i_[1];
  i_[1] -= other.i_[0];
  return *this;
}

//==============================================================================
template <typename S>
Interval<S> Interval<S>::operator * (const Interval<S>& other) const
{
  if(other.i_[0] >= 0)
  {
    if(i_[0] >= 0) return Interval<S>(i_[0] * other.i_[0], i_[1] * other.i_[1]);
    if(i_[1] <= 0) return Interval<S>(i_[0] * other.i_[1], i_[1] * other.i_[0]);
    return Interval<S>(i_[0] * other.i_[1], i_[1] * other.i_[1]);
  }
  if(other.i_[1] <= 0)
  {
    if(i_[0] >= 0) return Interval<S>(i_[1] * other.i_[0], i_[0] * other.i_[1]);
    if(i_[1] <= 0) return Interval<S>(i_[1] * other.i_[1], i_[0] * other.i_[0]);
    return Interval<S>(i_[1] * other.i_[0], i_[0] * other.i_[0]);
  }

  if(i_[0] >= 0) return Interval<S>(i_[1] * other.i_[0], i_[1] * other.i_[1]);
  if(i_[1] <= 0) return Interval<S>(i_[0] * other.i_[1], i_[0] * other.i_[0]);

  S v00 = i_[0] * other.i_[0];
  S v11 = i_[1] * other.i_[1];
  if(v00 <= v11)
  {
    S v01 = i_[0] * other.i_[1];
    S v10 = i_[1] * other.i_[0];
    if(v01 < v10) return Interval<S>(v01, v11);
    return Interval<S>(v10, v11);
  }

  S v01 = i_[0] * other.i_[1];
  S v10 = i_[1] * other.i_[0];
  if(v01 < v10) return Interval<S>(v01, v00);
  return Interval<S>(v10, v00);
}

//==============================================================================
template <typename S>
Interval<S>& Interval<S>::operator *= (const Interval<S>& other)
{
  if(other.i_[0] >= 0)
  {
    if(i_[0] >= 0)
    {
      i_[0] *= other.i_[0];
      i_[1] *= other.i_[1];
    }
    else if(i_[1] <= 0)
    {
      i_[0] *= other.i_[1];
      i_[1] *= other.i_[0];
    }
    else
    {
      i_[0] *= other.i_[1];
      i_[1] *= other.i_[1];
    }
    return *this;
  }

  if(other.i_[1] <= 0)
  {
    if(i_[0] >= 0)
    {
      S tmp = i_[0];
      i_[0] = i_[1] * other.i_[0];
      i_[1] = tmp * other.i_[1];
    }
    else if(i_[1] <= 0)
    {
      S tmp = i_[0];
      i_[0] = i_[1] * other.i_[1];
      i_[1] = tmp * other.i_[0];
    }
    else
    {
      S tmp = i_[0];
      i_[0] = i_[1] * other.i_[0];
      i_[1] = tmp * other.i_[0];
    }
    return *this;
  }

  if(i_[0] >= 0)
  {
    i_[0] = i_[1] * other.i_[0];
    i_[1] *= other.i_[1];
    return *this;
  }

  if(i_[1] <= 0)
  {
    i_[1] = i_[0] * other.i_[0];
    i_[0] *= other.i_[1];
    return *this;
  }

  S v00 = i_[0] * other.i_[0];
  S v11 = i_[1] * other.i_[1];
  if(v00 <= v11)
  {
    S v01 = i_[0] * other.i_[1];
    S v10 = i_[1] * other.i_[0];
    if(v01 < v10)
    {
      i_[0] = v01;
      i_[1] = v11;
    }
    else
    {
      i_[0] = v10;
      i_[1] = v11;
    }
    return *this;
  }

  S v01 = i_[0] * other.i_[1];
  S v10 = i_[1] * other.i_[0];
  if(v01 < v10)
  {
    i_[0] = v01;
    i_[1] = v00;
  }
  else
  {
    i_[0] = v10;
    i_[1] = v00;
  }

  return *this;
}

//==============================================================================
template <typename S>
Interval<S> Interval<S>::operator * (S d) const
{
  if(d >= 0) return Interval(i_[0] * d, i_[1] * d);
  return Interval(i_[1] * d, i_[0] * d);
}

//==============================================================================
template <typename S>
Interval<S>& Interval<S>::operator *= (S d)
{
  if(d >= 0)
  {
    i_[0] *= d;
    i_[1] *= d;
  }
  else
  {
    S tmp = i_[0];
    i_[0] = i_[1] * d;
    i_[1] = tmp * d;
  }

  return *this;
}

//==============================================================================
template <typename S>
Interval<S> Interval<S>::operator / (const Interval<S>& other) const
{
  return *this * Interval(1.0 / other.i_[1], 1.0 / other.i_[0]);
}

//==============================================================================
template <typename S>
Interval<S>& Interval<S>::operator /= (const Interval<S>& other)
{
  *this *= Interval(1.0 / other.i_[1], 1.0 / other.i_[0]);
  return *this;
}

//==============================================================================
template <typename S>
bool Interval<S>::overlap(const Interval<S>& other) const
{
  if(i_[1] < other.i_[0]) return false;
  if(i_[0] > other.i_[1]) return false;
  return true;
}

//==============================================================================
template <typename S>
bool Interval<S>::intersect(const Interval<S>& other)
{
  if(i_[1] < other.i_[0]) return false;
  if(i_[0] > other.i_[1]) return false;
  if(i_[1] > other.i_[1]) i_[1] = other.i_[1];
  if(i_[0] < other.i_[0]) i_[0] = other.i_[0];
  return true;
}

//==============================================================================
template <typename S>
Interval<S> Interval<S>::operator -() const
{
  return Interval<S>(-i_[1], -i_[0]);
}

//==============================================================================
template <typename S>
S Interval<S>::getAbsLower() const
{
  if(i_[0] >= 0) return i_[0];
  if(i_[1] >= 0) return 0;
  return -i_[1];
}

//==============================================================================
template <typename S>
S Interval<S>::getAbsUpper() const
{
  if(i_[0] + i_[1] >= 0) return i_[1];
  return i_[0];
}

//==============================================================================
template <typename S>
bool Interval<S>::contains(S v) const
{
  if(v < i_[0]) return false;
  if(v > i_[1]) return false;
  return true;
}

//==============================================================================
template <typename S>
Interval<S>& Interval<S>::bound(S v)
{
  if(v < i_[0]) i_[0] = v;
  if(v > i_[1]) i_[1] = v;
  return *this;
}

//==============================================================================
template <typename S>
Interval<S>& Interval<S>::bound(const Interval<S>& other)
{
  if(other.i_[0] < i_[0]) i_[0] = other.i_[0];
  if(other.i_[1] > i_[1]) i_[1] = other.i_[1];
  return *this;
}

//==============================================================================
template <typename S>
void Interval<S>::print() const
{
  std::cout << "[" << i_[0] << ", " << i_[1] << "]" << std::endl;
}

//==============================================================================
template <typename S>
S Interval<S>::center() const
{
  return 0.5 * (i_[0] + i_[1]);
}

//==============================================================================
template <typename S>
S Interval<S>::diameter() const
{
  return i_[1] -i_[0];
}

//==============================================================================
template <typename S>
Interval<S> bound(const Interval<S>& i, S v)
{
  Interval<S> res = i;
  if(v < res.i_[0]) res.i_[0] = v;
  if(v > res.i_[1]) res.i_[1] = v;
  return res;
}

//==============================================================================
template <typename S>
Interval<S> bound(const Interval<S>& i, const Interval<S>& other)
{
  Interval<S> res = i;
  if(other.i_[0] < res.i_[0]) res.i_[0] = other.i_[0];
  if(other.i_[1] > res.i_[1]) res.i_[1] = other.i_[1];
  return res;
}

} // namespace fcl

#endif

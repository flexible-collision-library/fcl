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

#include <iostream>
#include "fcl/data_types.h"

namespace fcl
{

/// @brief Interval class for [a, b]
template <typename Scalar>
struct Interval
{
  Scalar i_[2];

  Interval();

  explicit Interval(Scalar v);

  /// @brief construct interval [left, right]
  Interval(Scalar left, Scalar right);

  /// @brief construct interval [left, right]
  void setValue(Scalar a, Scalar b);

  /// @brief construct zero interval [x, x]
  void setValue(Scalar x);

  /// @brief access the interval endpoints: 0 for left, 1 for right end
  Scalar operator [] (size_t i) const;

  /// @brief access the interval endpoints: 0 for left, 1 for right end
  Scalar& operator [] (size_t i);

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

  Interval operator * (Scalar d) const;

  Interval& operator *= (Scalar d);

  /// @brief other must not contain 0
  Interval operator / (const Interval& other) const;

  Interval& operator /= (const Interval& other);

  /// @brief determine whether the intersection between intervals is empty
  bool overlap(const Interval& other) const;

  bool intersect(const Interval& other);

  Interval operator - () const;

  /// @brief Return the nearest distance for points within the interval to zero
  Scalar getAbsLower() const;

  /// @brief Return the farthest distance for points within the interval to zero
  Scalar getAbsUpper() const;

  bool contains(Scalar v) const;

  /// @brief Compute the minimum interval contains v and original interval
  Interval& bound(Scalar v);

  /// @brief Compute the minimum interval contains other and original interval
  Interval& bound(const Interval& other);

  void print() const;

  Scalar center() const;

  Scalar diameter() const;
};

template <typename Scalar>
Interval<Scalar> bound(const Interval<Scalar>& i, Scalar v);

template <typename Scalar>
Interval<Scalar> bound(const Interval<Scalar>& i, const Interval<Scalar>& other);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
Interval<Scalar>::Interval()
{
  i_[0] = i_[1] = 0;
}

//==============================================================================
template <typename Scalar>
Interval<Scalar>::Interval(Scalar v)
{
  i_[0] = i_[1] = v;
}

//==============================================================================
template <typename Scalar>
Interval<Scalar>::Interval(Scalar left, Scalar right)
{
  i_[0] = left; i_[1] = right;
}

//==============================================================================
template <typename Scalar>
void Interval<Scalar>::setValue(Scalar a, Scalar b)
{
  i_[0] = a; i_[1] = b;
}

//==============================================================================
template <typename Scalar>
void Interval<Scalar>::setValue(Scalar x)
{
  i_[0] = i_[1] = x;
}

//==============================================================================
template <typename Scalar>
Scalar Interval<Scalar>::operator [] (size_t i) const
{
  return i_[i];
}

//==============================================================================
template <typename Scalar>
Scalar& Interval<Scalar>::operator [] (size_t i)
{
  return i_[i];
}

//==============================================================================
template <typename Scalar>
bool Interval<Scalar>::operator == (const Interval& other) const
{
  if(i_[0] != other.i_[0]) return false;
  if(i_[1] != other.i_[1]) return false;
  return true;
}

//==============================================================================
template <typename Scalar>
Interval<Scalar> Interval<Scalar>::operator + (const Interval<Scalar>& other) const
{
  return Interval(i_[0] + other.i_[0], i_[1] + other.i_[1]);
}

//==============================================================================
template <typename Scalar>
Interval<Scalar> Interval<Scalar>::operator - (const Interval<Scalar>& other) const
{
  return Interval(i_[0] - other.i_[1], i_[1] - other.i_[0]);
}

//==============================================================================
template <typename Scalar>
Interval<Scalar>& Interval<Scalar>::operator += (const Interval<Scalar>& other)
{
  i_[0] += other.i_[0];
  i_[1] += other.i_[1];
  return *this;
}

//==============================================================================
template <typename Scalar>
Interval<Scalar>& Interval<Scalar>::operator -= (const Interval<Scalar>& other)
{
  i_[0] -= other.i_[1];
  i_[1] -= other.i_[0];
  return *this;
}

//==============================================================================
template <typename Scalar>
Interval<Scalar> Interval<Scalar>::operator * (const Interval<Scalar>& other) const
{
  if(other.i_[0] >= 0)
  {
    if(i_[0] >= 0) return Interval<Scalar>(i_[0] * other.i_[0], i_[1] * other.i_[1]);
    if(i_[1] <= 0) return Interval<Scalar>(i_[0] * other.i_[1], i_[1] * other.i_[0]);
    return Interval<Scalar>(i_[0] * other.i_[1], i_[1] * other.i_[1]);
  }
  if(other.i_[1] <= 0)
  {
    if(i_[0] >= 0) return Interval<Scalar>(i_[1] * other.i_[0], i_[0] * other.i_[1]);
    if(i_[1] <= 0) return Interval<Scalar>(i_[1] * other.i_[1], i_[0] * other.i_[0]);
    return Interval<Scalar>(i_[1] * other.i_[0], i_[0] * other.i_[0]);
  }

  if(i_[0] >= 0) return Interval<Scalar>(i_[1] * other.i_[0], i_[1] * other.i_[1]);
  if(i_[1] <= 0) return Interval<Scalar>(i_[0] * other.i_[1], i_[0] * other.i_[0]);

  Scalar v00 = i_[0] * other.i_[0];
  Scalar v11 = i_[1] * other.i_[1];
  if(v00 <= v11)
  {
    Scalar v01 = i_[0] * other.i_[1];
    Scalar v10 = i_[1] * other.i_[0];
    if(v01 < v10) return Interval<Scalar>(v01, v11);
    return Interval<Scalar>(v10, v11);
  }

  Scalar v01 = i_[0] * other.i_[1];
  Scalar v10 = i_[1] * other.i_[0];
  if(v01 < v10) return Interval<Scalar>(v01, v00);
  return Interval<Scalar>(v10, v00);
}

//==============================================================================
template <typename Scalar>
Interval<Scalar>& Interval<Scalar>::operator *= (const Interval<Scalar>& other)
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
      Scalar tmp = i_[0];
      i_[0] = i_[1] * other.i_[0];
      i_[1] = tmp * other.i_[1];
    }
    else if(i_[1] <= 0)
    {
      Scalar tmp = i_[0];
      i_[0] = i_[1] * other.i_[1];
      i_[1] = tmp * other.i_[0];
    }
    else
    {
      Scalar tmp = i_[0];
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

  Scalar v00 = i_[0] * other.i_[0];
  Scalar v11 = i_[1] * other.i_[1];
  if(v00 <= v11)
  {
    Scalar v01 = i_[0] * other.i_[1];
    Scalar v10 = i_[1] * other.i_[0];
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

  Scalar v01 = i_[0] * other.i_[1];
  Scalar v10 = i_[1] * other.i_[0];
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
template <typename Scalar>
Interval<Scalar> Interval<Scalar>::operator * (Scalar d) const
{
  if(d >= 0) return Interval(i_[0] * d, i_[1] * d);
  return Interval(i_[1] * d, i_[0] * d);
}

//==============================================================================
template <typename Scalar>
Interval<Scalar>& Interval<Scalar>::operator *= (Scalar d)
{
  if(d >= 0)
  {
    i_[0] *= d;
    i_[1] *= d;
  }
  else
  {
    Scalar tmp = i_[0];
    i_[0] = i_[1] * d;
    i_[1] = tmp * d;
  }

  return *this;
}

//==============================================================================
template <typename Scalar>
Interval<Scalar> Interval<Scalar>::operator / (const Interval<Scalar>& other) const
{
  return *this * Interval(1.0 / other.i_[1], 1.0 / other.i_[0]);
}

//==============================================================================
template <typename Scalar>
Interval<Scalar>& Interval<Scalar>::operator /= (const Interval<Scalar>& other)
{
  *this *= Interval(1.0 / other.i_[1], 1.0 / other.i_[0]);
  return *this;
}

//==============================================================================
template <typename Scalar>
bool Interval<Scalar>::overlap(const Interval<Scalar>& other) const
{
  if(i_[1] < other.i_[0]) return false;
  if(i_[0] > other.i_[1]) return false;
  return true;
}

//==============================================================================
template <typename Scalar>
bool Interval<Scalar>::intersect(const Interval<Scalar>& other)
{
  if(i_[1] < other.i_[0]) return false;
  if(i_[0] > other.i_[1]) return false;
  if(i_[1] > other.i_[1]) i_[1] = other.i_[1];
  if(i_[0] < other.i_[0]) i_[0] = other.i_[0];
  return true;
}

//==============================================================================
template <typename Scalar>
Interval<Scalar> Interval<Scalar>::operator -() const
{
  return Interval<Scalar>(-i_[1], -i_[0]);
}

//==============================================================================
template <typename Scalar>
Scalar Interval<Scalar>::getAbsLower() const
{
  if(i_[0] >= 0) return i_[0];
  if(i_[1] >= 0) return 0;
  return -i_[1];
}

//==============================================================================
template <typename Scalar>
Scalar Interval<Scalar>::getAbsUpper() const
{
  if(i_[0] + i_[1] >= 0) return i_[1];
  return i_[0];
}

//==============================================================================
template <typename Scalar>
bool Interval<Scalar>::contains(Scalar v) const
{
  if(v < i_[0]) return false;
  if(v > i_[1]) return false;
  return true;
}

//==============================================================================
template <typename Scalar>
Interval<Scalar>& Interval<Scalar>::bound(Scalar v)
{
  if(v < i_[0]) i_[0] = v;
  if(v > i_[1]) i_[1] = v;
  return *this;
}

//==============================================================================
template <typename Scalar>
Interval<Scalar>& Interval<Scalar>::bound(const Interval<Scalar>& other)
{
  if(other.i_[0] < i_[0]) i_[0] = other.i_[0];
  if(other.i_[1] > i_[1]) i_[1] = other.i_[1];
  return *this;
}

//==============================================================================
template <typename Scalar>
void Interval<Scalar>::print() const
{
  std::cout << "[" << i_[0] << ", " << i_[1] << "]" << std::endl;
}

//==============================================================================
template <typename Scalar>
Scalar Interval<Scalar>::center() const
{
  return 0.5 * (i_[0] + i_[1]);
}

//==============================================================================
template <typename Scalar>
Scalar Interval<Scalar>::diameter() const
{
  return i_[1] -i_[0];
}

//==============================================================================
template <typename Scalar>
Interval<Scalar> bound(const Interval<Scalar>& i, Scalar v)
{
  Interval<Scalar> res = i;
  if(v < res.i_[0]) res.i_[0] = v;
  if(v > res.i_[1]) res.i_[1] = v;
  return res;
}

//==============================================================================
template <typename Scalar>
Interval<Scalar> bound(const Interval<Scalar>& i, const Interval<Scalar>& other)
{
  Interval<Scalar> res = i;
  if(other.i_[0] < res.i_[0]) res.i_[0] = other.i_[0];
  if(other.i_[1] > res.i_[1]) res.i_[1] = other.i_[1];
  return res;
}

} // namespace fcl

#endif

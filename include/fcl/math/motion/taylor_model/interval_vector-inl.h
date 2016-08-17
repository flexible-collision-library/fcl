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

#ifndef FCL_CCD_INTERVAL_VECTOR_INL_H
#define FCL_CCD_INTERVAL_VECTOR_INL_H

#include "fcl/math/motion/taylor_model/interval_vector.h"

namespace fcl
{

//==============================================================================
extern template
struct IVector3<double>;

//==============================================================================
extern template
IVector3<double> bound(const IVector3<double>& i, const Vector3<double>& v);

//==============================================================================
extern template
IVector3<double> bound(const IVector3<double>& i, const IVector3<double>& v);

//==============================================================================
template <typename S>
IVector3<S>::IVector3()
{
  // Do nothing
}

//==============================================================================
template <typename S>
IVector3<S>::IVector3(S v)
{
  setValue(v);
}

//==============================================================================
template <typename S>
IVector3<S>::IVector3(S x, S y, S z)
{
  setValue(x, y, z);
}

//==============================================================================
template <typename S>
IVector3<S>::IVector3(S xl, S xu, S yl, S yu, S zl, S zu)
{
  setValue(xl, xu, yl, yu, zl, zu);
}

//==============================================================================
template <typename S>
IVector3<S>::IVector3(S v[3][2])
{
  setValue(v);
}

//==============================================================================
template <typename S>
IVector3<S>::IVector3(Interval<S> v[3])
{
  setValue(v);
}

//==============================================================================
template <typename S>
IVector3<S>::IVector3(const Interval<S>& v1, const Interval<S>& v2, const Interval<S>& v3)
{
  setValue(v1, v2, v3);
}

//==============================================================================
template <typename S>
IVector3<S>::IVector3(const Vector3<S>& v)
{
  setValue(v);
}

//==============================================================================
template <typename S>
void IVector3<S>::setValue(S v)
{
  i_[0].setValue(v);
  i_[1].setValue(v);
  i_[2].setValue(v);
}

//==============================================================================
template <typename S>
void IVector3<S>::setValue(S x, S y, S z)
{
  i_[0].setValue(x);
  i_[1].setValue(y);
  i_[2].setValue(z);
}

//==============================================================================
template <typename S>
void IVector3<S>::setValue(S xl, S xu, S yl, S yu, S zl, S zu)
{
  i_[0].setValue(xl, xu);
  i_[1].setValue(yl, yu);
  i_[2].setValue(zl, zu);
}

//==============================================================================
template <typename S>
void IVector3<S>::setValue(S v[3][2])
{
  i_[0].setValue(v[0][0], v[0][1]);
  i_[1].setValue(v[1][0], v[1][1]);
  i_[2].setValue(v[2][0], v[2][1]);
}

//==============================================================================
template <typename S>
void IVector3<S>::setValue(Interval<S> v[3])
{
  i_[0] = v[0];
  i_[1] = v[1];
  i_[2] = v[2];
}

//==============================================================================
template <typename S>
void IVector3<S>::setValue(const Interval<S>& v1, const Interval<S>& v2, const Interval<S>& v3)
{
  i_[0] = v1;
  i_[1] = v2;
  i_[2] = v3;
}

//==============================================================================
template <typename S>
void IVector3<S>::setValue(const Vector3<S>& v)
{
  i_[0].setValue(v[0]);
  i_[1].setValue(v[1]);
  i_[2].setValue(v[2]);
}

//==============================================================================
template <typename S>
void IVector3<S>::setValue(S v[])
{
  i_[0].setValue(v[0]);
  i_[1].setValue(v[1]);
  i_[2].setValue(v[2]);
}

//==============================================================================
template <typename S>
void IVector3<S>::setZero()
{
  setValue((S)0.0);
}

//==============================================================================
template <typename S>
IVector3<S> IVector3<S>::operator + (const IVector3<S>& other) const
{
  return IVector3(i_[0] + other.i_[0], i_[1] + other.i_[1], i_[2] + other.i_[2]);
}

//==============================================================================
template <typename S>
IVector3<S>& IVector3<S>::operator += (const IVector3<S>& other)
{
  i_[0] += other[0];
  i_[1] += other[1];
  i_[2] += other[2];
  return *this;
}

//==============================================================================
template <typename S>
IVector3<S> IVector3<S>::operator - (const IVector3<S>& other) const
{
  return IVector3(i_[0] - other.i_[0], i_[1] - other.i_[1], i_[2] - other.i_[2]);
}

//==============================================================================
template <typename S>
IVector3<S>& IVector3<S>::operator -= (const IVector3<S>& other)
{
  i_[0] -= other[0];
  i_[1] -= other[1];
  i_[2] -= other[2];
  return *this;
}

//==============================================================================
template <typename S>
Interval<S> IVector3<S>::dot(const IVector3& other) const
{
  return i_[0] * other.i_[0] + i_[1] * other.i_[1] + i_[2] * other.i_[2];
}

//==============================================================================
template <typename S>
IVector3<S> IVector3<S>::cross(const IVector3<S>& other) const
{
  return IVector3(i_[1] * other.i_[2] - i_[2] * other.i_[1],
                  i_[2] * other.i_[0] - i_[0] * other.i_[2],
                  i_[0] * other.i_[1] - i_[1] * other.i_[0]);
}

//==============================================================================
template <typename S>
Interval<S> IVector3<S>::dot(const Vector3<S>& other) const
{
  return i_[0] * other[0] + i_[1] * other[1] + i_[2] * other[2];
}

//==============================================================================
template <typename S>
const Interval<S>&IVector3<S>::operator [](size_t i) const
{
  return i_[i];
}

//==============================================================================
template <typename S>
Interval<S>&IVector3<S>::operator [](size_t i)
{
  return i_[i];
}

//==============================================================================
template <typename S>
Vector3<S> IVector3<S>::getLow() const
{
  return Vector3<S>(i_[0][0], i_[1][0], i_[2][0]);
}

//==============================================================================
template <typename S>
Vector3<S> IVector3<S>::getHigh() const
{
  return Vector3<S>(i_[0][1], i_[1][1], i_[2][1]);
}

//==============================================================================
template <typename S>
IVector3<S> IVector3<S>::cross(const Vector3<S>& other) const
{
  return IVector3(i_[1] * other[2] - i_[2] * other[1],
                  i_[2] * other[0] - i_[0] * other[2],
                  i_[0] * other[1] - i_[1] * other[0]);
}

//==============================================================================
template <typename S>
S IVector3<S>::volumn() const
{
  return i_[0].diameter() * i_[1].diameter() * i_[2].diameter();
}

//==============================================================================
template <typename S>
void IVector3<S>::print() const
{
  std::cout << "[" << i_[0][0] << "," << i_[0][1] << "]" << std::endl;
  std::cout << "[" << i_[1][0] << "," << i_[1][1] << "]" << std::endl;
  std::cout << "[" << i_[2][0] << "," << i_[2][1] << "]" << std::endl;
}

//==============================================================================
template <typename S>
Vector3<S> IVector3<S>::center() const
{
  return Vector3<S>(i_[0].center(), i_[1].center(), i_[2].center());
}

//==============================================================================
template <typename S>
void IVector3<S>::bound(const IVector3& v)
{
  if(v[0][0] < i_[0][0]) i_[0][0] = v[0][0];
  if(v[1][0] < i_[1][0]) i_[1][0] = v[1][0];
  if(v[2][0] < i_[2][0]) i_[2][0] = v[2][0];

  if(v[0][1] > i_[0][1]) i_[0][1] = v[0][1];
  if(v[1][1] > i_[1][1]) i_[1][1] = v[1][1];
  if(v[2][1] > i_[2][1]) i_[2][1] = v[2][1];
}

//==============================================================================
template <typename S>
void IVector3<S>::bound(const Vector3<S>& v)
{
  if(v[0] < i_[0][0]) i_[0][0] = v[0];
  if(v[1] < i_[1][0]) i_[1][0] = v[1];
  if(v[2] < i_[2][0]) i_[2][0] = v[2];

  if(v[0] > i_[0][1]) i_[0][1] = v[0];
  if(v[1] > i_[1][1]) i_[1][1] = v[1];
  if(v[2] > i_[2][1]) i_[2][1] = v[2];
}

//==============================================================================
template <typename S>
IVector3<S> bound(const IVector3<S>& i, const IVector3<S>& v)
{
  IVector3<S> res(i);
  if(v[0][0] < res.i_[0][0]) res.i_[0][0] = v[0][0];
  if(v[1][0] < res.i_[1][0]) res.i_[1][0] = v[1][0];
  if(v[2][0] < res.i_[2][0]) res.i_[2][0] = v[2][0];

  if(v[0][1] > res.i_[0][1]) res.i_[0][1] = v[0][1];
  if(v[1][1] > res.i_[1][1]) res.i_[1][1] = v[1][1];
  if(v[2][1] > res.i_[2][1]) res.i_[2][1] = v[2][1];

  return res;
}

//==============================================================================
template <typename S>
IVector3<S> bound(const IVector3<S>& i, const Vector3<S>& v)
{
  IVector3<S> res(i);
  if(v[0] < res.i_[0][0]) res.i_[0][0] = v[0];
  if(v[1] < res.i_[1][0]) res.i_[1][0] = v[1];
  if(v[2] < res.i_[2][0]) res.i_[2][0] = v[2];

  if(v[0] > res.i_[0][1]) res.i_[0][1] = v[0];
  if(v[1] > res.i_[1][1]) res.i_[1][1] = v[1];
  if(v[2] > res.i_[2][1]) res.i_[2][1] = v[2];

  return res;
}

//==============================================================================
template <typename S>
bool IVector3<S>::overlap(const IVector3& v) const
{
  if(v[0][1] < i_[0][0]) return false;
  if(v[1][1] < i_[1][0]) return false;
  if(v[2][1] < i_[2][0]) return false;

  if(v[0][0] > i_[0][1]) return false;
  if(v[1][0] > i_[1][1]) return false;
  if(v[2][0] > i_[2][1]) return false;

  return true;
}

//==============================================================================
template <typename S>
bool IVector3<S>::contain(const IVector3& v) const
{
  if(v[0][0] < i_[0][0]) return false;
  if(v[1][0] < i_[1][0]) return false;
  if(v[2][0] < i_[2][0]) return false;

  if(v[0][1] > i_[0][1]) return false;
  if(v[1][1] > i_[1][1]) return false;
  if(v[2][1] > i_[2][1]) return false;

  return true;
}

} // namespace fcl

#endif

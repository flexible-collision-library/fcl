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


#ifndef FCL_CCD_INTERVAL_VECTOR_H
#define FCL_CCD_INTERVAL_VECTOR_H

#include "fcl/ccd/interval.h"

namespace fcl
{

template <typename Scalar>
struct IVector3
{
  Interval<Scalar> i_[3];

  IVector3();
  IVector3(Scalar v);
  IVector3(Scalar x, Scalar y, Scalar z);
  IVector3(Scalar xl, Scalar xu, Scalar yl, Scalar yu, Scalar zl, Scalar zu);
  IVector3(Interval<Scalar> v[3]);
  IVector3(Scalar v[3][2]);
  IVector3(const Interval<Scalar>& v1, const Interval<Scalar>& v2, const Interval<Scalar>& v3);
  IVector3(const Vector3<Scalar>& v);

  void setValue(Scalar v);

  void setValue(Scalar x, Scalar y, Scalar z);

  void setValue(Scalar xl, Scalar xu, Scalar yl, Scalar yu, Scalar zl, Scalar zu);

  void setValue(Scalar v[3][2]);

  void setValue(Interval<Scalar> v[3]);

  void setValue(const Interval<Scalar>& v1, const Interval<Scalar>& v2, const Interval<Scalar>& v3);

  void setValue(const Vector3<Scalar>& v);

  void setValue(Scalar v[3]);
  
  IVector3 operator + (const IVector3& other) const;
  IVector3& operator += (const IVector3& other);

  IVector3 operator - (const IVector3& other) const;
  IVector3& operator -= (const IVector3& other);

  Interval<Scalar> dot(const IVector3& other) const;
  IVector3 cross(const IVector3& other) const;

  Interval<Scalar> dot(const Vector3<Scalar>& other) const;
  IVector3 cross(const Vector3<Scalar>& other) const;

  const Interval<Scalar>& operator [] (size_t i) const;

  Interval<Scalar>& operator [] (size_t i);

  Vector3<Scalar> getLow() const;
  
  Vector3<Scalar> getHigh() const;

  void print() const;
  Vector3<Scalar> center() const;
  Scalar volumn() const;
  void setZero();

  void bound(const Vector3<Scalar>& v);
  void bound(const IVector3& v);

  bool overlap(const IVector3& v) const;
  bool contain(const IVector3& v) const;
};

template <typename Scalar>
IVector3<Scalar> bound(const IVector3<Scalar>& i, const Vector3<Scalar>& v);

template <typename Scalar>
IVector3<Scalar> bound(const IVector3<Scalar>& i, const IVector3<Scalar>& v);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
IVector3<Scalar>::IVector3()
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
IVector3<Scalar>::IVector3(Scalar v)
{
  setValue(v);
}

//==============================================================================
template <typename Scalar>
IVector3<Scalar>::IVector3(Scalar x, Scalar y, Scalar z)
{
  setValue(x, y, z);
}

//==============================================================================
template <typename Scalar>
IVector3<Scalar>::IVector3(Scalar xl, Scalar xu, Scalar yl, Scalar yu, Scalar zl, Scalar zu)
{
  setValue(xl, xu, yl, yu, zl, zu);
}

//==============================================================================
template <typename Scalar>
IVector3<Scalar>::IVector3(Scalar v[3][2])
{
  setValue(v);
}

//==============================================================================
template <typename Scalar>
IVector3<Scalar>::IVector3(Interval<Scalar> v[3])
{
  setValue(v);
}

//==============================================================================
template <typename Scalar>
IVector3<Scalar>::IVector3(const Interval<Scalar>& v1, const Interval<Scalar>& v2, const Interval<Scalar>& v3)
{
  setValue(v1, v2, v3);
}

//==============================================================================
template <typename Scalar>
IVector3<Scalar>::IVector3(const Vector3<Scalar>& v)
{
  setValue(v);
}

//==============================================================================
template <typename Scalar>
void IVector3<Scalar>::setValue(Scalar v)
{
  i_[0].setValue(v);
  i_[1].setValue(v);
  i_[2].setValue(v);
}

//==============================================================================
template <typename Scalar>
void IVector3<Scalar>::setValue(Scalar x, Scalar y, Scalar z)
{
  i_[0].setValue(x);
  i_[1].setValue(y);
  i_[2].setValue(z);
}

//==============================================================================
template <typename Scalar>
void IVector3<Scalar>::setValue(Scalar xl, Scalar xu, Scalar yl, Scalar yu, Scalar zl, Scalar zu)
{
  i_[0].setValue(xl, xu);
  i_[1].setValue(yl, yu);
  i_[2].setValue(zl, zu);
}

//==============================================================================
template <typename Scalar>
void IVector3<Scalar>::setValue(Scalar v[3][2])
{
  i_[0].setValue(v[0][0], v[0][1]);
  i_[1].setValue(v[1][0], v[1][1]);
  i_[2].setValue(v[2][0], v[2][1]);
}

//==============================================================================
template <typename Scalar>
void IVector3<Scalar>::setValue(Interval<Scalar> v[3])
{
  i_[0] = v[0];
  i_[1] = v[1];
  i_[2] = v[2];
}

//==============================================================================
template <typename Scalar>
void IVector3<Scalar>::setValue(const Interval<Scalar>& v1, const Interval<Scalar>& v2, const Interval<Scalar>& v3)
{
  i_[0] = v1;
  i_[1] = v2;
  i_[2] = v3;
}

//==============================================================================
template <typename Scalar>
void IVector3<Scalar>::setValue(const Vector3<Scalar>& v)
{
  i_[0].setValue(v[0]);
  i_[1].setValue(v[1]);
  i_[2].setValue(v[2]);
}

//==============================================================================
template <typename Scalar>
void IVector3<Scalar>::setValue(Scalar v[])
{
  i_[0].setValue(v[0]);
  i_[1].setValue(v[1]);
  i_[2].setValue(v[2]);
}

//==============================================================================
template <typename Scalar>
void IVector3<Scalar>::setZero()
{
  setValue((Scalar)0.0);
}

//==============================================================================
template <typename Scalar>
IVector3<Scalar> IVector3<Scalar>::operator + (const IVector3<Scalar>& other) const
{
  return IVector3(i_[0] + other.i_[0], i_[1] + other.i_[1], i_[2] + other.i_[2]);
}

//==============================================================================
template <typename Scalar>
IVector3<Scalar>& IVector3<Scalar>::operator += (const IVector3<Scalar>& other)
{
  i_[0] += other[0];
  i_[1] += other[1];
  i_[2] += other[2];
  return *this;
}

//==============================================================================
template <typename Scalar>
IVector3<Scalar> IVector3<Scalar>::operator - (const IVector3<Scalar>& other) const
{
  return IVector3(i_[0] - other.i_[0], i_[1] - other.i_[1], i_[2] - other.i_[2]);
}

//==============================================================================
template <typename Scalar>
IVector3<Scalar>& IVector3<Scalar>::operator -= (const IVector3<Scalar>& other)
{
  i_[0] -= other[0];
  i_[1] -= other[1];
  i_[2] -= other[2];
  return *this;
}

//==============================================================================
template <typename Scalar>
Interval<Scalar> IVector3<Scalar>::dot(const IVector3& other) const
{
  return i_[0] * other.i_[0] + i_[1] * other.i_[1] + i_[2] * other.i_[2];
}

//==============================================================================
template <typename Scalar>
IVector3<Scalar> IVector3<Scalar>::cross(const IVector3<Scalar>& other) const
{
  return IVector3(i_[1] * other.i_[2] - i_[2] * other.i_[1],
                  i_[2] * other.i_[0] - i_[0] * other.i_[2],
                  i_[0] * other.i_[1] - i_[1] * other.i_[0]);
}

//==============================================================================
template <typename Scalar>
Interval<Scalar> IVector3<Scalar>::dot(const Vector3<Scalar>& other) const
{
  return i_[0] * other[0] + i_[1] * other[1] + i_[2] * other[2];
}

//==============================================================================
template <typename Scalar>
const Interval<Scalar>&IVector3<Scalar>::operator [](size_t i) const
{
  return i_[i];
}

//==============================================================================
template <typename Scalar>
Interval<Scalar>&IVector3<Scalar>::operator [](size_t i)
{
  return i_[i];
}

//==============================================================================
template <typename Scalar>
Vector3<Scalar> IVector3<Scalar>::getLow() const
{
  return Vector3<Scalar>(i_[0][0], i_[1][0], i_[2][0]);
}

//==============================================================================
template <typename Scalar>
Vector3<Scalar> IVector3<Scalar>::getHigh() const
{
  return Vector3<Scalar>(i_[0][1], i_[1][1], i_[2][1]);
}

//==============================================================================
template <typename Scalar>
IVector3<Scalar> IVector3<Scalar>::cross(const Vector3<Scalar>& other) const
{
  return IVector3(i_[1] * other[2] - i_[2] * other[1],
                  i_[2] * other[0] - i_[0] * other[2],
                  i_[0] * other[1] - i_[1] * other[0]);
}

//==============================================================================
template <typename Scalar>
Scalar IVector3<Scalar>::volumn() const
{
  return i_[0].diameter() * i_[1].diameter() * i_[2].diameter();
}

//==============================================================================
template <typename Scalar>
void IVector3<Scalar>::print() const
{
  std::cout << "[" << i_[0][0] << "," << i_[0][1] << "]" << std::endl;
  std::cout << "[" << i_[1][0] << "," << i_[1][1] << "]" << std::endl;
  std::cout << "[" << i_[2][0] << "," << i_[2][1] << "]" << std::endl;
}

//==============================================================================
template <typename Scalar>
Vector3<Scalar> IVector3<Scalar>::center() const
{
  return Vector3<Scalar>(i_[0].center(), i_[1].center(), i_[2].center());
}

//==============================================================================
template <typename Scalar>
void IVector3<Scalar>::bound(const IVector3& v)
{
  if(v[0][0] < i_[0][0]) i_[0][0] = v[0][0];
  if(v[1][0] < i_[1][0]) i_[1][0] = v[1][0];
  if(v[2][0] < i_[2][0]) i_[2][0] = v[2][0];

  if(v[0][1] > i_[0][1]) i_[0][1] = v[0][1];
  if(v[1][1] > i_[1][1]) i_[1][1] = v[1][1];
  if(v[2][1] > i_[2][1]) i_[2][1] = v[2][1];
}

//==============================================================================
template <typename Scalar>
void IVector3<Scalar>::bound(const Vector3<Scalar>& v)
{
  if(v[0] < i_[0][0]) i_[0][0] = v[0];
  if(v[1] < i_[1][0]) i_[1][0] = v[1];
  if(v[2] < i_[2][0]) i_[2][0] = v[2];

  if(v[0] > i_[0][1]) i_[0][1] = v[0];
  if(v[1] > i_[1][1]) i_[1][1] = v[1];
  if(v[2] > i_[2][1]) i_[2][1] = v[2];
}

//==============================================================================
template <typename Scalar>
IVector3<Scalar> bound(const IVector3<Scalar>& i, const IVector3<Scalar>& v)
{
  IVector3<Scalar> res(i);
  if(v[0][0] < res.i_[0][0]) res.i_[0][0] = v[0][0];
  if(v[1][0] < res.i_[1][0]) res.i_[1][0] = v[1][0];
  if(v[2][0] < res.i_[2][0]) res.i_[2][0] = v[2][0];

  if(v[0][1] > res.i_[0][1]) res.i_[0][1] = v[0][1];
  if(v[1][1] > res.i_[1][1]) res.i_[1][1] = v[1][1];
  if(v[2][1] > res.i_[2][1]) res.i_[2][1] = v[2][1];

  return res;
}

//==============================================================================
template <typename Scalar>
IVector3<Scalar> bound(const IVector3<Scalar>& i, const Vector3<Scalar>& v)
{
  IVector3<Scalar> res(i);
  if(v[0] < res.i_[0][0]) res.i_[0][0] = v[0];
  if(v[1] < res.i_[1][0]) res.i_[1][0] = v[1];
  if(v[2] < res.i_[2][0]) res.i_[2][0] = v[2];

  if(v[0] > res.i_[0][1]) res.i_[0][1] = v[0];
  if(v[1] > res.i_[1][1]) res.i_[1][1] = v[1];
  if(v[2] > res.i_[2][1]) res.i_[2][1] = v[2];

  return res;
}

//==============================================================================
template <typename Scalar>
bool IVector3<Scalar>::overlap(const IVector3& v) const
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
template <typename Scalar>
bool IVector3<Scalar>::contain(const IVector3& v) const
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

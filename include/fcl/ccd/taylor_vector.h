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

#ifndef FCL_CCD_TAYLOR_VECTOR_H
#define FCL_CCD_TAYLOR_VECTOR_H

#include "fcl/ccd/interval_vector.h"
#include "fcl/ccd/taylor_model.h"

namespace fcl
{

template <typename Scalar>
class TVector3
{
  TaylorModel<Scalar> i_[3];

public:
  
  TVector3();
  TVector3(const std::shared_ptr<TimeInterval<Scalar>>& time_interval);
  TVector3(TaylorModel<Scalar> v[3]);
  TVector3(const TaylorModel<Scalar>& v0, const TaylorModel<Scalar>& v1, const TaylorModel<Scalar>& v2);
  TVector3(const Vector3<Scalar>& v, const std::shared_ptr<TimeInterval<Scalar>>& time_interval);
  
  TVector3 operator + (const TVector3& other) const;
  TVector3& operator += (const TVector3& other);

  TVector3 operator + (const Vector3<Scalar>& other) const;
  TVector3& operator += (const Vector3<Scalar>& other);

  TVector3 operator - (const TVector3& other) const;
  TVector3& operator -= (const TVector3& other);

  TVector3 operator - (const Vector3<Scalar>& other) const;
  TVector3& operator -= (const Vector3<Scalar>& other);

  TVector3 operator - () const;

  TVector3 operator * (const TaylorModel<Scalar>& d) const;
  TVector3& operator *= (const TaylorModel<Scalar>& d);
  TVector3 operator * (Scalar d) const;
  TVector3& operator *= (Scalar d);

  const TaylorModel<Scalar>& operator [] (size_t i) const;
  TaylorModel<Scalar>& operator [] (size_t i);

  TaylorModel<Scalar> dot(const TVector3& other) const;
  TVector3 cross(const TVector3& other) const;
  TaylorModel<Scalar> dot(const Vector3<Scalar>& other) const;
  TVector3 cross(const Vector3<Scalar>& other) const;

  IVector3<Scalar> getBound() const;
  IVector3<Scalar> getBound(Scalar l, Scalar r) const;
  IVector3<Scalar> getBound(Scalar t) const;

  IVector3<Scalar> getTightBound() const;
  IVector3<Scalar> getTightBound(Scalar l, Scalar r) const;

  void print() const;
  Scalar volumn() const;
  void setZero();

  TaylorModel<Scalar> squareLength() const;

  void setTimeInterval(const std::shared_ptr<TimeInterval<Scalar>>& time_interval);
  void setTimeInterval(Scalar l, Scalar r);

  const std::shared_ptr<TimeInterval<Scalar>>& getTimeInterval() const;
};

template <typename Scalar>
void generateTVector3ForLinearFunc(TVector3<Scalar>& v, const Vector3<Scalar>& position, const Vector3<Scalar>& velocity);

template <typename Scalar>
TVector3<Scalar> operator * (const Vector3<Scalar>& v, const TaylorModel<Scalar>& a);

template <typename Scalar>
TVector3<Scalar> operator + (const Vector3<Scalar>& v1, const TVector3<Scalar>& v2);

template <typename Scalar>
TVector3<Scalar> operator - (const Vector3<Scalar>& v1, const TVector3<Scalar>& v2);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
TVector3<Scalar>::TVector3()
{
}

//==============================================================================
template <typename Scalar>
TVector3<Scalar>::TVector3(const std::shared_ptr<TimeInterval<Scalar>>& time_interval)
{
  setTimeInterval(time_interval);
}

//==============================================================================
template <typename Scalar>
TVector3<Scalar>::TVector3(TaylorModel<Scalar> v[3])
{
  i_[0] = v[0];
  i_[1] = v[1];
  i_[2] = v[2];
}

//==============================================================================
template <typename Scalar>
TVector3<Scalar>::TVector3(const TaylorModel<Scalar>& v1, const TaylorModel<Scalar>& v2, const TaylorModel<Scalar>& v3)
{
  i_[0] = v1;
  i_[1] = v2;
  i_[2] = v3;
}

//==============================================================================
template <typename Scalar>
TVector3<Scalar>::TVector3(const Vector3<Scalar>& v, const std::shared_ptr<TimeInterval<Scalar>>& time_interval)
{
  i_[0] = TaylorModel<Scalar>(v[0], time_interval);
  i_[1] = TaylorModel<Scalar>(v[1], time_interval);
  i_[2] = TaylorModel<Scalar>(v[2], time_interval);
}

//==============================================================================
template <typename Scalar>
void TVector3<Scalar>::setZero()
{
  i_[0].setZero();
  i_[1].setZero();
  i_[2].setZero();
}

//==============================================================================
template <typename Scalar>
TVector3<Scalar> TVector3<Scalar>::operator + (const TVector3<Scalar>& other) const
{
  return TVector3(i_[0] + other.i_[0], i_[1] + other.i_[1], i_[2] + other.i_[2]);
}

//==============================================================================
template <typename Scalar>
TVector3<Scalar> TVector3<Scalar>::operator - (const TVector3<Scalar>& other) const
{
  return TVector3(i_[0] - other.i_[0], i_[1] - other.i_[1], i_[2] - other.i_[2]);
}

//==============================================================================
template <typename Scalar>
TVector3<Scalar> TVector3<Scalar>::operator - () const
{
  return TVector3(-i_[0], -i_[1], -i_[2]);
}

//==============================================================================
template <typename Scalar>
TVector3<Scalar>& TVector3<Scalar>::operator += (const TVector3<Scalar>& other)
{
  i_[0] += other.i_[0];
  i_[1] += other.i_[1];
  i_[2] += other.i_[2];
  return *this;
}

//==============================================================================
template <typename Scalar>
TVector3<Scalar>& TVector3<Scalar>::operator -= (const TVector3<Scalar>& other)
{
  i_[0] -= other.i_[0];
  i_[1] -= other.i_[1];
  i_[2] -= other.i_[2];
  return *this;
}

//==============================================================================
template <typename Scalar>
TVector3<Scalar> TVector3<Scalar>::operator + (const Vector3<Scalar>& other) const
{
  return TVector3(i_[0] + other[0], i_[1] + other[1], i_[2] + other[2]);
}

//==============================================================================
template <typename Scalar>
TVector3<Scalar>& TVector3<Scalar>::operator += (const Vector3<Scalar>& other)
{
  i_[0] += other[0];
  i_[1] += other[1];
  i_[2] += other[2];
  return *this;
}

//==============================================================================
template <typename Scalar>
TVector3<Scalar> TVector3<Scalar>::operator - (const Vector3<Scalar>& other) const
{
  return TVector3(i_[0] - other[0], i_[1] - other[1], i_[2] - other[2]);
}

//==============================================================================
template <typename Scalar>
TVector3<Scalar>& TVector3<Scalar>::operator -= (const Vector3<Scalar>& other)
{
  i_[0] -= other[0];
  i_[1] -= other[1];
  i_[2] -= other[2];
  return *this;
}

//==============================================================================
template <typename Scalar>
TVector3<Scalar> TVector3<Scalar>::operator * (const TaylorModel<Scalar>& d) const
{
  return TVector3(i_[0] * d, i_[1] * d, i_[2] * d);
}

//==============================================================================
template <typename Scalar>
TVector3<Scalar>& TVector3<Scalar>::operator *= (const TaylorModel<Scalar>& d)
{
  i_[0] *= d;
  i_[1] *= d;
  i_[2] *= d;
  return *this;
}

//==============================================================================
template <typename Scalar>
TVector3<Scalar> TVector3<Scalar>::operator * (Scalar d) const
{
  return TVector3(i_[0] * d, i_[1] * d, i_[2] * d);
}

//==============================================================================
template <typename Scalar>
TVector3<Scalar>& TVector3<Scalar>::operator *= (Scalar d)
{
  i_[0] *= d;
  i_[1] *= d;
  i_[2] *= d;
  return *this;
}

//==============================================================================
template <typename Scalar>
const TaylorModel<Scalar>& TVector3<Scalar>::operator [] (size_t i) const
{
  return i_[i];
}

//==============================================================================
template <typename Scalar>
TaylorModel<Scalar>& TVector3<Scalar>::operator [] (size_t i)
{
  return i_[i];
}

//==============================================================================
template <typename Scalar>
TaylorModel<Scalar> TVector3<Scalar>::dot(const TVector3& other) const
{
  return i_[0] * other.i_[0] + i_[1] * other.i_[1] + i_[2] * other.i_[2];
}

//==============================================================================
template <typename Scalar>
TVector3<Scalar> TVector3<Scalar>::cross(const TVector3<Scalar>& other) const
{
  return TVector3<Scalar>(i_[1] * other.i_[2] - i_[2] * other.i_[1],
                  i_[2] * other.i_[0] - i_[0] * other.i_[2],
                  i_[0] * other.i_[1] - i_[1] * other.i_[0]);
}

//==============================================================================
template <typename Scalar>
TaylorModel<Scalar> TVector3<Scalar>::dot(const Vector3<Scalar>& other) const
{
  return i_[0] * other[0] + i_[1] * other[1] + i_[2] * other[2];
}

//==============================================================================
template <typename Scalar>
TVector3<Scalar> TVector3<Scalar>::cross(const Vector3<Scalar>& other) const
{
  return TVector3<Scalar>(i_[1] * other[2] - i_[2] * other[1],
                  i_[2] * other[0] - i_[0] * other[2],
                  i_[0] * other[1] - i_[1] * other[0]);
}

//==============================================================================
template <typename Scalar>
Scalar TVector3<Scalar>::volumn() const
{
  return i_[0].getBound().diameter() * i_[1].getBound().diameter() * i_[2].getBound().diameter();
}

//==============================================================================
template <typename Scalar>
IVector3<Scalar> TVector3<Scalar>::getBound() const
{
  return IVector3<Scalar>(i_[0].getBound(), i_[1].getBound(), i_[2].getBound());
}

//==============================================================================
template <typename Scalar>
IVector3<Scalar> TVector3<Scalar>::getBound(Scalar l, Scalar r) const
{
  return IVector3<Scalar>(i_[0].getBound(l, r), i_[1].getBound(l, r), i_[2].getBound(l, r));
}

//==============================================================================
template <typename Scalar>
IVector3<Scalar> TVector3<Scalar>::getBound(Scalar t) const
{
  return IVector3<Scalar>(i_[0].getBound(t), i_[1].getBound(t), i_[2].getBound(t));
}

//==============================================================================
template <typename Scalar>
IVector3<Scalar> TVector3<Scalar>::getTightBound() const
{
  return IVector3<Scalar>(i_[0].getTightBound(), i_[1].getTightBound(), i_[2].getTightBound());
}

//==============================================================================
template <typename Scalar>
IVector3<Scalar> TVector3<Scalar>::getTightBound(Scalar l, Scalar r) const
{
  return IVector3<Scalar>(i_[0].getTightBound(l, r), i_[1].getTightBound(l, r), i_[2].getTightBound(l, r));
}

//==============================================================================
template <typename Scalar>
void TVector3<Scalar>::print() const
{
  i_[0].print();
  i_[1].print();
  i_[2].print();
}

//==============================================================================
template <typename Scalar>
TaylorModel<Scalar> TVector3<Scalar>::squareLength() const
{
  return i_[0] * i_[0] + i_[1] * i_[1] + i_[2] * i_[2];
}

//==============================================================================
template <typename Scalar>
void TVector3<Scalar>::setTimeInterval(const std::shared_ptr<TimeInterval<Scalar>>& time_interval)
{
  i_[0].setTimeInterval(time_interval);
  i_[1].setTimeInterval(time_interval);
  i_[2].setTimeInterval(time_interval);
}

//==============================================================================
template <typename Scalar>
void TVector3<Scalar>::setTimeInterval(Scalar l, Scalar r)
{
  i_[0].setTimeInterval(l, r);
  i_[1].setTimeInterval(l, r);
  i_[2].setTimeInterval(l, r);
}

//==============================================================================
template <typename Scalar>
const std::shared_ptr<TimeInterval<Scalar>>& TVector3<Scalar>::getTimeInterval() const
{
  return i_[0].getTimeInterval();
}

//==============================================================================
template <typename Scalar>
void generateTVector3ForLinearFunc(TVector3<Scalar>& v, const Vector3<Scalar>& position, const Vector3<Scalar>& velocity)
{
  generateTaylorModelForLinearFunc(v[0], position[0], velocity[0]);
  generateTaylorModelForLinearFunc(v[1], position[1], velocity[1]);
  generateTaylorModelForLinearFunc(v[2], position[2], velocity[2]);
}

//==============================================================================
template <typename Scalar>
TVector3<Scalar> operator * (const Vector3<Scalar>& v, const TaylorModel<Scalar>& a)
{
  TVector3<Scalar> res(a.getTimeInterval());
  res[0] = a * v[0];
  res[1] = a * v[1];
  res[2] = a * v[2];

  return res;
}

//==============================================================================
template <typename Scalar>
TVector3<Scalar> operator + (const Vector3<Scalar>& v1, const TVector3<Scalar>& v2)
{
  return v2 + v1;
}

//==============================================================================
template <typename Scalar>
TVector3<Scalar> operator - (const Vector3<Scalar>& v1, const TVector3<Scalar>& v2)
{
  return -v2 + v1;
}

} // namespace fcl

#endif

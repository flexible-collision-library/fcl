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

#ifndef FCL_CCD_TAYLOR_VECTOR_INL_H
#define FCL_CCD_TAYLOR_VECTOR_INL_H

#include "fcl/math/motion/taylor_model/taylor_vector.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT TVector3<double>;

//==============================================================================
extern template
void generateTVector3ForLinearFunc(TVector3<double>& v, const Vector3<double>& position, const Vector3<double>& velocity);

//==============================================================================
extern template
TVector3<double> operator * (const Vector3<double>& v, const TaylorModel<double>& a);

//==============================================================================
extern template
TVector3<double> operator + (const Vector3<double>& v1, const TVector3<double>& v2);

//==============================================================================
extern template
TVector3<double> operator - (const Vector3<double>& v1, const TVector3<double>& v2);

//==============================================================================
template <typename S>
TVector3<S>::TVector3()
{
  // Do nothing
}

//==============================================================================
template <typename S>
TVector3<S>::TVector3(const std::shared_ptr<TimeInterval<S>>& time_interval)
{
  setTimeInterval(time_interval);
}

//==============================================================================
template <typename S>
TVector3<S>::TVector3(TaylorModel<S> v[3])
{
  i_[0] = v[0];
  i_[1] = v[1];
  i_[2] = v[2];
}

//==============================================================================
template <typename S>
TVector3<S>::TVector3(const TaylorModel<S>& v1, const TaylorModel<S>& v2, const TaylorModel<S>& v3)
{
  i_[0] = v1;
  i_[1] = v2;
  i_[2] = v3;
}

//==============================================================================
template <typename S>
TVector3<S>::TVector3(const Vector3<S>& v, const std::shared_ptr<TimeInterval<S>>& time_interval)
{
  i_[0] = TaylorModel<S>(v[0], time_interval);
  i_[1] = TaylorModel<S>(v[1], time_interval);
  i_[2] = TaylorModel<S>(v[2], time_interval);
}

//==============================================================================
template <typename S>
void TVector3<S>::setZero()
{
  i_[0].setZero();
  i_[1].setZero();
  i_[2].setZero();
}

//==============================================================================
template <typename S>
TVector3<S> TVector3<S>::operator + (const TVector3<S>& other) const
{
  return TVector3(i_[0] + other.i_[0], i_[1] + other.i_[1], i_[2] + other.i_[2]);
}

//==============================================================================
template <typename S>
TVector3<S> TVector3<S>::operator - (const TVector3<S>& other) const
{
  return TVector3(i_[0] - other.i_[0], i_[1] - other.i_[1], i_[2] - other.i_[2]);
}

//==============================================================================
template <typename S>
TVector3<S> TVector3<S>::operator - () const
{
  return TVector3(-i_[0], -i_[1], -i_[2]);
}

//==============================================================================
template <typename S>
TVector3<S>& TVector3<S>::operator += (const TVector3<S>& other)
{
  i_[0] += other.i_[0];
  i_[1] += other.i_[1];
  i_[2] += other.i_[2];
  return *this;
}

//==============================================================================
template <typename S>
TVector3<S>& TVector3<S>::operator -= (const TVector3<S>& other)
{
  i_[0] -= other.i_[0];
  i_[1] -= other.i_[1];
  i_[2] -= other.i_[2];
  return *this;
}

//==============================================================================
template <typename S>
TVector3<S> TVector3<S>::operator + (const Vector3<S>& other) const
{
  return TVector3(i_[0] + other[0], i_[1] + other[1], i_[2] + other[2]);
}

//==============================================================================
template <typename S>
TVector3<S>& TVector3<S>::operator += (const Vector3<S>& other)
{
  i_[0] += other[0];
  i_[1] += other[1];
  i_[2] += other[2];
  return *this;
}

//==============================================================================
template <typename S>
TVector3<S> TVector3<S>::operator - (const Vector3<S>& other) const
{
  return TVector3(i_[0] - other[0], i_[1] - other[1], i_[2] - other[2]);
}

//==============================================================================
template <typename S>
TVector3<S>& TVector3<S>::operator -= (const Vector3<S>& other)
{
  i_[0] -= other[0];
  i_[1] -= other[1];
  i_[2] -= other[2];
  return *this;
}

//==============================================================================
template <typename S>
TVector3<S> TVector3<S>::operator * (const TaylorModel<S>& d) const
{
  return TVector3(i_[0] * d, i_[1] * d, i_[2] * d);
}

//==============================================================================
template <typename S>
TVector3<S>& TVector3<S>::operator *= (const TaylorModel<S>& d)
{
  i_[0] *= d;
  i_[1] *= d;
  i_[2] *= d;
  return *this;
}

//==============================================================================
template <typename S>
TVector3<S> TVector3<S>::operator * (S d) const
{
  return TVector3(i_[0] * d, i_[1] * d, i_[2] * d);
}

//==============================================================================
template <typename S>
TVector3<S>& TVector3<S>::operator *= (S d)
{
  i_[0] *= d;
  i_[1] *= d;
  i_[2] *= d;
  return *this;
}

//==============================================================================
template <typename S>
const TaylorModel<S>& TVector3<S>::operator [] (size_t i) const
{
  return i_[i];
}

//==============================================================================
template <typename S>
TaylorModel<S>& TVector3<S>::operator [] (size_t i)
{
  return i_[i];
}

//==============================================================================
template <typename S>
TaylorModel<S> TVector3<S>::dot(const TVector3& other) const
{
  return i_[0] * other.i_[0] + i_[1] * other.i_[1] + i_[2] * other.i_[2];
}

//==============================================================================
template <typename S>
TVector3<S> TVector3<S>::cross(const TVector3<S>& other) const
{
  return TVector3<S>(i_[1] * other.i_[2] - i_[2] * other.i_[1],
                  i_[2] * other.i_[0] - i_[0] * other.i_[2],
                  i_[0] * other.i_[1] - i_[1] * other.i_[0]);
}

//==============================================================================
template <typename S>
TaylorModel<S> TVector3<S>::dot(const Vector3<S>& other) const
{
  return i_[0] * other[0] + i_[1] * other[1] + i_[2] * other[2];
}

//==============================================================================
template <typename S>
TVector3<S> TVector3<S>::cross(const Vector3<S>& other) const
{
  return TVector3<S>(i_[1] * other[2] - i_[2] * other[1],
                  i_[2] * other[0] - i_[0] * other[2],
                  i_[0] * other[1] - i_[1] * other[0]);
}

//==============================================================================
template <typename S>
S TVector3<S>::volumn() const
{
  return i_[0].getBound().diameter() * i_[1].getBound().diameter() * i_[2].getBound().diameter();
}

//==============================================================================
template <typename S>
IVector3<S> TVector3<S>::getBound() const
{
  return IVector3<S>(i_[0].getBound(), i_[1].getBound(), i_[2].getBound());
}

//==============================================================================
template <typename S>
IVector3<S> TVector3<S>::getBound(S l, S r) const
{
  return IVector3<S>(i_[0].getBound(l, r), i_[1].getBound(l, r), i_[2].getBound(l, r));
}

//==============================================================================
template <typename S>
IVector3<S> TVector3<S>::getBound(S t) const
{
  return IVector3<S>(i_[0].getBound(t), i_[1].getBound(t), i_[2].getBound(t));
}

//==============================================================================
template <typename S>
IVector3<S> TVector3<S>::getTightBound() const
{
  return IVector3<S>(i_[0].getTightBound(), i_[1].getTightBound(), i_[2].getTightBound());
}

//==============================================================================
template <typename S>
IVector3<S> TVector3<S>::getTightBound(S l, S r) const
{
  return IVector3<S>(i_[0].getTightBound(l, r), i_[1].getTightBound(l, r), i_[2].getTightBound(l, r));
}

//==============================================================================
template <typename S>
void TVector3<S>::print() const
{
  i_[0].print();
  i_[1].print();
  i_[2].print();
}

//==============================================================================
template <typename S>
TaylorModel<S> TVector3<S>::squareLength() const
{
  return i_[0] * i_[0] + i_[1] * i_[1] + i_[2] * i_[2];
}

//==============================================================================
template <typename S>
void TVector3<S>::setTimeInterval(const std::shared_ptr<TimeInterval<S>>& time_interval)
{
  i_[0].setTimeInterval(time_interval);
  i_[1].setTimeInterval(time_interval);
  i_[2].setTimeInterval(time_interval);
}

//==============================================================================
template <typename S>
void TVector3<S>::setTimeInterval(S l, S r)
{
  i_[0].setTimeInterval(l, r);
  i_[1].setTimeInterval(l, r);
  i_[2].setTimeInterval(l, r);
}

//==============================================================================
template <typename S>
const std::shared_ptr<TimeInterval<S>>& TVector3<S>::getTimeInterval() const
{
  return i_[0].getTimeInterval();
}

//==============================================================================
template <typename S>
void generateTVector3ForLinearFunc(TVector3<S>& v, const Vector3<S>& position, const Vector3<S>& velocity)
{
  generateTaylorModelForLinearFunc(v[0], position[0], velocity[0]);
  generateTaylorModelForLinearFunc(v[1], position[1], velocity[1]);
  generateTaylorModelForLinearFunc(v[2], position[2], velocity[2]);
}

//==============================================================================
template <typename S>
TVector3<S> operator * (const Vector3<S>& v, const TaylorModel<S>& a)
{
  TVector3<S> res(a.getTimeInterval());
  res[0] = a * v[0];
  res[1] = a * v[1];
  res[2] = a * v[2];

  return res;
}

//==============================================================================
template <typename S>
TVector3<S> operator + (const Vector3<S>& v1, const TVector3<S>& v2)
{
  return v2 + v1;
}

//==============================================================================
template <typename S>
TVector3<S> operator - (const Vector3<S>& v1, const TVector3<S>& v2)
{
  return -v2 + v1;
}

} // namespace fcl

#endif

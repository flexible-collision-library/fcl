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

#ifndef FCL_CCD_TAYLOR_VECTOR_H
#define FCL_CCD_TAYLOR_VECTOR_H

#include "fcl/math/motion/taylor_model/interval_vector.h"
#include "fcl/math/motion/taylor_model/taylor_model.h"

namespace fcl
{

template <typename S>
class FCL_EXPORT TVector3
{
  TaylorModel<S> i_[3];

public:
  
  TVector3();
  TVector3(const std::shared_ptr<TimeInterval<S>>& time_interval);
  TVector3(TaylorModel<S> v[3]);
  TVector3(const TaylorModel<S>& v0, const TaylorModel<S>& v1, const TaylorModel<S>& v2);
  TVector3(const Vector3<S>& v, const std::shared_ptr<TimeInterval<S>>& time_interval);
  
  TVector3 operator + (const TVector3& other) const;
  TVector3& operator += (const TVector3& other);

  TVector3 operator + (const Vector3<S>& other) const;
  TVector3& operator += (const Vector3<S>& other);

  TVector3 operator - (const TVector3& other) const;
  TVector3& operator -= (const TVector3& other);

  TVector3 operator - (const Vector3<S>& other) const;
  TVector3& operator -= (const Vector3<S>& other);

  TVector3 operator - () const;

  TVector3 operator * (const TaylorModel<S>& d) const;
  TVector3& operator *= (const TaylorModel<S>& d);
  TVector3 operator * (S d) const;
  TVector3& operator *= (S d);

  const TaylorModel<S>& operator [] (size_t i) const;
  TaylorModel<S>& operator [] (size_t i);

  TaylorModel<S> dot(const TVector3& other) const;
  TVector3 cross(const TVector3& other) const;
  TaylorModel<S> dot(const Vector3<S>& other) const;
  TVector3 cross(const Vector3<S>& other) const;

  IVector3<S> getBound() const;
  IVector3<S> getBound(S l, S r) const;
  IVector3<S> getBound(S t) const;

  IVector3<S> getTightBound() const;
  IVector3<S> getTightBound(S l, S r) const;

  void print() const;
  S volumn() const;
  void setZero();

  TaylorModel<S> squareLength() const;

  void setTimeInterval(const std::shared_ptr<TimeInterval<S>>& time_interval);
  void setTimeInterval(S l, S r);

  const std::shared_ptr<TimeInterval<S>>& getTimeInterval() const;
};

template <typename S>
FCL_EXPORT
void generateTVector3ForLinearFunc(TVector3<S>& v, const Vector3<S>& position, const Vector3<S>& velocity);

template <typename S>
FCL_EXPORT
TVector3<S> operator * (const Vector3<S>& v, const TaylorModel<S>& a);

template <typename S>
FCL_EXPORT
TVector3<S> operator + (const Vector3<S>& v1, const TVector3<S>& v2);

template <typename S>
FCL_EXPORT
TVector3<S> operator - (const Vector3<S>& v1, const TVector3<S>& v2);

} // namespace fcl

#include "fcl/math/motion/taylor_model/taylor_vector-inl.h"

#endif

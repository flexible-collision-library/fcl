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

#ifndef FCL_CCD_TAYLOR_MATRIX_H
#define FCL_CCD_TAYLOR_MATRIX_H

#include "fcl/math/motion/taylor_model/taylor_vector.h"
#include "fcl/math/motion/taylor_model/interval_matrix.h"

namespace fcl
{

template <typename S>
class TMatrix3
{
  TVector3<S> v_[3];
  
public:
  TMatrix3();
  TMatrix3(const std::shared_ptr<TimeInterval<S>>& time_interval);
  TMatrix3(TaylorModel<S> m[3][3]);
  TMatrix3(const TVector3<S>& v1, const TVector3<S>& v2, const TVector3<S>& v3);
  TMatrix3(const Matrix3<S>& m, const std::shared_ptr<TimeInterval<S>>& time_interval);

  TVector3<S> getColumn(size_t i) const;
  const TVector3<S>& getRow(size_t i) const;

  const TaylorModel<S>& operator () (size_t i, size_t j) const;
  TaylorModel<S>& operator () (size_t i, size_t j);

  TVector3<S> operator * (const Vector3<S>& v) const;
  TVector3<S> operator * (const TVector3<S>& v) const;
  TMatrix3 operator * (const Matrix3<S>& m) const;
  TMatrix3 operator * (const TMatrix3& m) const;
  TMatrix3 operator * (const TaylorModel<S>& d) const;
  TMatrix3 operator * (S d) const;

  TMatrix3& operator *= (const Matrix3<S>& m);
  TMatrix3& operator *= (const TMatrix3& m);
  TMatrix3& operator *= (const TaylorModel<S>& d);
  TMatrix3& operator *= (S d);

  TMatrix3 operator + (const TMatrix3& m) const;
  TMatrix3& operator += (const TMatrix3& m);
  TMatrix3 operator + (const Matrix3<S>& m) const;
  TMatrix3& operator += (const Matrix3<S>& m);

  TMatrix3 operator - (const TMatrix3& m) const;
  TMatrix3& operator -= (const TMatrix3& m);
  TMatrix3 operator - (const Matrix3<S>& m) const;
  TMatrix3& operator -= (const Matrix3<S>& m);
  TMatrix3 operator - () const;

  IMatrix3<S> getBound() const;
  IMatrix3<S> getBound(S l, S r) const;
  IMatrix3<S> getBound(S t) const;

  IMatrix3<S> getTightBound() const;
  IMatrix3<S> getTightBound(S l, S r) const;

  void print() const;
  void setIdentity();
  void setZero();
  S diameter() const;

  void setTimeInterval(const std::shared_ptr<TimeInterval<S>>& time_interval);
  void setTimeInterval(S l, S r);

  const std::shared_ptr<TimeInterval<S>>& getTimeInterval() const;

  TMatrix3& rotationConstrain();
};

template <typename S>
TMatrix3<S> rotationConstrain(const TMatrix3<S>& m);

template <typename S>
TMatrix3<S> operator * (const Matrix3<S>& m, const TaylorModel<S>& a);

template <typename S>
TMatrix3<S> operator * (const TaylorModel<S>& a, const Matrix3<S>& m);

template <typename S>
TMatrix3<S> operator * (const TaylorModel<S>& a, const TMatrix3<S>& m);

template <typename S>
TMatrix3<S> operator * (S d, const TMatrix3<S>& m);

template <typename S>
TMatrix3<S> operator + (const Matrix3<S>& m1, const TMatrix3<S>& m2);

template <typename S>
TMatrix3<S> operator - (const Matrix3<S>& m1, const TMatrix3<S>& m2);

} // namespace fcl

#include "fcl/math/motion/taylor_model/taylor_matrix-inl.h"

#endif

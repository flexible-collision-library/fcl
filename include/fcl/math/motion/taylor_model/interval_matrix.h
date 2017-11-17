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

#ifndef FCL_CCD_INTERVAL_MATRIX_H
#define FCL_CCD_INTERVAL_MATRIX_H

#include "fcl/math/motion/taylor_model/interval.h"
#include "fcl/math/motion/taylor_model/interval_vector.h"

namespace fcl
{

template <typename S>
struct FCL_EXPORT IMatrix3
{
  IVector3<S> v_[3];

  IMatrix3();
  IMatrix3(S v);
  IMatrix3(const Matrix3<S>& m);
  IMatrix3(S m[3][3][2]);
  IMatrix3(S m[3][3]);
  IMatrix3(Interval<S> m[3][3]);
  IMatrix3(const IVector3<S>& v1, const IVector3<S>& v2, const IVector3<S>& v3);

  void setIdentity();

  IVector3<S> getColumn(size_t i) const;
  const IVector3<S>& getRow(size_t i) const;

  Vector3<S> getColumnLow(size_t i) const;
  Vector3<S> getRowLow(size_t i) const;

  Vector3<S> getColumnHigh(size_t i) const;
  Vector3<S> getRowHigh(size_t i) const;

  Matrix3<S> getLow() const;
  Matrix3<S> getHigh() const;

  const Interval<S>& operator () (size_t i, size_t j) const;

  Interval<S>& operator () (size_t i, size_t j);

  IMatrix3 operator + (const IMatrix3& m) const;
  IMatrix3& operator += (const IMatrix3& m);

  IMatrix3 operator - (const IMatrix3& m) const;
  IMatrix3& operator -= (const IMatrix3& m);

  IVector3<S> operator * (const Vector3<S>& v) const;
  IVector3<S> operator * (const IVector3<S>& v) const;
  IMatrix3 operator * (const IMatrix3& m) const;
  IMatrix3 operator * (const Matrix3<S>& m) const;

  IMatrix3& operator *= (const IMatrix3& m);
  IMatrix3& operator *= (const Matrix3<S>& m);

  IMatrix3& rotationConstrain();

  void print() const;
};

template <typename S>
FCL_EXPORT
IMatrix3<S> rotationConstrain(const IMatrix3<S>& m);

} // namespace fcl

#include "fcl/math/motion/taylor_model/interval_matrix-inl.h"

#endif

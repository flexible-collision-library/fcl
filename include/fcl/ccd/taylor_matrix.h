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

#ifndef FCL_CCD_TAYLOR_MATRIX_H
#define FCL_CCD_TAYLOR_MATRIX_H


#include "fcl/math/matrix_3f.h"
#include "fcl/ccd/taylor_vector.h"
#include "fcl/ccd/interval_matrix.h"

namespace fcl
{

class TMatrix3
{
  TVector3 v_[3];
  
public:
  TMatrix3();
  TMatrix3(const std::shared_ptr<TimeInterval>& time_interval);
  TMatrix3(TaylorModel m[3][3]);
  TMatrix3(const TVector3& v1, const TVector3& v2, const TVector3& v3);
  TMatrix3(const Matrix3f& m, const std::shared_ptr<TimeInterval>& time_interval);

  TVector3 getColumn(size_t i) const;
  const TVector3& getRow(size_t i) const;

  const TaylorModel& operator () (size_t i, size_t j) const;
  TaylorModel& operator () (size_t i, size_t j);

  TVector3 operator * (const Vec3f& v) const;
  TVector3 operator * (const TVector3& v) const;
  TMatrix3 operator * (const Matrix3f& m) const;
  TMatrix3 operator * (const TMatrix3& m) const;
  TMatrix3 operator * (const TaylorModel& d) const;
  TMatrix3 operator * (FCL_REAL d) const;

  TMatrix3& operator *= (const Matrix3f& m);
  TMatrix3& operator *= (const TMatrix3& m);
  TMatrix3& operator *= (const TaylorModel& d);
  TMatrix3& operator *= (FCL_REAL d);

  TMatrix3 operator + (const TMatrix3& m) const;
  TMatrix3& operator += (const TMatrix3& m);
  TMatrix3 operator + (const Matrix3f& m) const;
  TMatrix3& operator += (const Matrix3f& m);

  TMatrix3 operator - (const TMatrix3& m) const;
  TMatrix3& operator -= (const TMatrix3& m);
  TMatrix3 operator - (const Matrix3f& m) const;
  TMatrix3& operator -= (const Matrix3f& m);
  TMatrix3 operator - () const;

  IMatrix3 getBound() const;
  IMatrix3 getBound(FCL_REAL l, FCL_REAL r) const;
  IMatrix3 getBound(FCL_REAL t) const;

  IMatrix3 getTightBound() const;
  IMatrix3 getTightBound(FCL_REAL l, FCL_REAL r) const;


  void print() const;
  void setIdentity();
  void setZero();
  FCL_REAL diameter() const;

  void setTimeInterval(const std::shared_ptr<TimeInterval>& time_interval);
  void setTimeInterval(FCL_REAL l, FCL_REAL r);

  const std::shared_ptr<TimeInterval>& getTimeInterval() const;

  TMatrix3& rotationConstrain();
};

TMatrix3 rotationConstrain(const TMatrix3& m);

TMatrix3 operator * (const Matrix3f& m, const TaylorModel& a);
TMatrix3 operator * (const TaylorModel& a, const Matrix3f& m);
TMatrix3 operator * (const TaylorModel& a, const TMatrix3& m);
TMatrix3 operator * (FCL_REAL d, const TMatrix3& m);
TMatrix3 operator + (const Matrix3f& m1, const TMatrix3& m2);
TMatrix3 operator - (const Matrix3f& m1, const TMatrix3& m2);

}

#endif

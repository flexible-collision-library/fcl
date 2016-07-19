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

class TVector3
{
  TaylorModel i_[3];

public:
  
  TVector3();
  TVector3(const std::shared_ptr<TimeInterval>& time_interval);
  TVector3(TaylorModel v[3]);
  TVector3(const TaylorModel& v0, const TaylorModel& v1, const TaylorModel& v2);
  TVector3(const Vec3f& v, const std::shared_ptr<TimeInterval>& time_interval);
  
  TVector3 operator + (const TVector3& other) const;
  TVector3& operator += (const TVector3& other);

  TVector3 operator + (const Vec3f& other) const;
  TVector3& operator += (const Vec3f& other);

  TVector3 operator - (const TVector3& other) const;
  TVector3& operator -= (const TVector3& other);

  TVector3 operator - (const Vec3f& other) const;
  TVector3& operator -= (const Vec3f& other);

  TVector3 operator - () const;

  TVector3 operator * (const TaylorModel& d) const;
  TVector3& operator *= (const TaylorModel& d);
  TVector3 operator * (FCL_REAL d) const;
  TVector3& operator *= (FCL_REAL d);

  const TaylorModel& operator [] (size_t i) const;
  TaylorModel& operator [] (size_t i);

  TaylorModel dot(const TVector3& other) const;
  TVector3 cross(const TVector3& other) const;
  TaylorModel dot(const Vec3f& other) const;
  TVector3 cross(const Vec3f& other) const;

  IVector3 getBound() const;
  IVector3 getBound(FCL_REAL l, FCL_REAL r) const;
  IVector3 getBound(FCL_REAL t) const;

  IVector3 getTightBound() const;
  IVector3 getTightBound(FCL_REAL l, FCL_REAL r) const;

  void print() const;
  FCL_REAL volumn() const;
  void setZero();

  TaylorModel squareLength() const;

  void setTimeInterval(const std::shared_ptr<TimeInterval>& time_interval);
  void setTimeInterval(FCL_REAL l, FCL_REAL r);

  const std::shared_ptr<TimeInterval>& getTimeInterval() const;
};

void generateTVector3ForLinearFunc(TVector3& v, const Vec3f& position, const Vec3f& velocity);


TVector3 operator * (const Vec3f& v, const TaylorModel& a);
TVector3 operator + (const Vec3f& v1, const TVector3& v2);
TVector3 operator - (const Vec3f& v1, const TVector3& v2);

}

#endif

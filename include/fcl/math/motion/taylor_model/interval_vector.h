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

#ifndef FCL_CCD_INTERVAL_VECTOR_H
#define FCL_CCD_INTERVAL_VECTOR_H

#include "fcl/math/motion/taylor_model/interval.h"

namespace fcl
{

template <typename S>
struct FCL_EXPORT IVector3
{
  Interval<S> i_[3];

  IVector3();
  IVector3(S v);
  IVector3(S x, S y, S z);
  IVector3(S xl, S xu, S yl, S yu, S zl, S zu);
  IVector3(Interval<S> v[3]);
  IVector3(S v[3][2]);
  IVector3(const Interval<S>& v1, const Interval<S>& v2, const Interval<S>& v3);
  IVector3(const Vector3<S>& v);

  void setValue(S v);

  void setValue(S x, S y, S z);

  void setValue(S xl, S xu, S yl, S yu, S zl, S zu);

  void setValue(S v[3][2]);

  void setValue(Interval<S> v[3]);

  void setValue(const Interval<S>& v1, const Interval<S>& v2, const Interval<S>& v3);

  void setValue(const Vector3<S>& v);

  void setValue(S v[3]);
  
  IVector3 operator + (const IVector3& other) const;
  IVector3& operator += (const IVector3& other);

  IVector3 operator - (const IVector3& other) const;
  IVector3& operator -= (const IVector3& other);

  Interval<S> dot(const IVector3& other) const;
  IVector3 cross(const IVector3& other) const;

  Interval<S> dot(const Vector3<S>& other) const;
  IVector3 cross(const Vector3<S>& other) const;

  const Interval<S>& operator [] (size_t i) const;

  Interval<S>& operator [] (size_t i);

  Vector3<S> getLow() const;
  
  Vector3<S> getHigh() const;

  void print() const;
  Vector3<S> center() const;
  S volumn() const;
  void setZero();

  void bound(const Vector3<S>& v);
  void bound(const IVector3& v);

  bool overlap(const IVector3& v) const;
  bool contain(const IVector3& v) const;
};

template <typename S>
FCL_EXPORT
IVector3<S> bound(const IVector3<S>& i, const Vector3<S>& v);

template <typename S>
FCL_EXPORT
IVector3<S> bound(const IVector3<S>& i, const IVector3<S>& v);

} // namespace fcl

#include "fcl/math/motion/taylor_model/interval_vector-inl.h"

#endif

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
#include "fcl/math/vec_3f.h"

namespace fcl
{

struct IVector3
{
  Interval i_[3];

  IVector3();
  IVector3(FCL_REAL v);
  IVector3(FCL_REAL x, FCL_REAL y, FCL_REAL z);
  IVector3(FCL_REAL xl, FCL_REAL xu, FCL_REAL yl, FCL_REAL yu, FCL_REAL zl, FCL_REAL zu);
  IVector3(Interval v[3]);
  IVector3(FCL_REAL v[3][2]);
  IVector3(const Interval& v1, const Interval& v2, const Interval& v3);
  IVector3(const Vec3f& v);

  inline void setValue(FCL_REAL v)
  {
    i_[0].setValue(v);
    i_[1].setValue(v);
    i_[2].setValue(v);
  }

  inline void setValue(FCL_REAL x, FCL_REAL y, FCL_REAL z)
  {
    i_[0].setValue(x);
    i_[1].setValue(y);
    i_[2].setValue(z);
  }

  inline void setValue(FCL_REAL xl, FCL_REAL xu, FCL_REAL yl, FCL_REAL yu, FCL_REAL zl, FCL_REAL zu)
  {
    i_[0].setValue(xl, xu);
    i_[1].setValue(yl, yu);
    i_[2].setValue(zl, zu);
  }

  inline void setValue(FCL_REAL v[3][2])
  {
    i_[0].setValue(v[0][0], v[0][1]);
    i_[1].setValue(v[1][0], v[1][1]);
    i_[2].setValue(v[2][0], v[2][1]);
  }

  inline void setValue(Interval v[3])
  {
    i_[0] = v[0];
    i_[1] = v[1];
    i_[2] = v[2];
  }

  inline void setValue(const Interval& v1, const Interval& v2, const Interval& v3)
  {
    i_[0] = v1;
    i_[1] = v2;
    i_[2] = v3;
  }

  inline void setValue(const Vec3f& v)
  {
    i_[0].setValue(v[0]);
    i_[1].setValue(v[1]);
    i_[2].setValue(v[2]);
  }

  inline void setValue(FCL_REAL v[3])
  {
    i_[0].setValue(v[0]);
    i_[1].setValue(v[1]);
    i_[2].setValue(v[2]);
  }
  
  IVector3 operator + (const IVector3& other) const;
  IVector3& operator += (const IVector3& other);

  IVector3 operator - (const IVector3& other) const;
  IVector3& operator -= (const IVector3& other);

  Interval dot(const IVector3& other) const;
  IVector3 cross(const IVector3& other) const;

  Interval dot(const Vec3f& other) const;
  IVector3 cross(const Vec3f& other) const;

  inline const Interval& operator [] (size_t i) const
  {
    return i_[i];
  }

  inline Interval& operator [] (size_t i)
  {
    return i_[i];
  }

  inline Vec3f getLow() const 
  {
    return Vec3f(i_[0][0], i_[1][0], i_[2][0]);
  }
  
  inline Vec3f getHigh() const
  {
    return Vec3f(i_[0][1], i_[1][1], i_[2][1]);
  }

  void print() const;
  Vec3f center() const;
  FCL_REAL volumn() const;
  void setZero();

  void bound(const Vec3f& v);
  void bound(const IVector3& v);

  bool overlap(const IVector3& v) const;
  bool contain(const IVector3& v) const;
};

IVector3 bound(const IVector3& i, const Vec3f& v);

IVector3 bound(const IVector3& i, const IVector3& v);

}

#endif

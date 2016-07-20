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

/** \author Jia Pan */

#include "fcl/ccd/interval_vector.h"
#include <iostream>

namespace fcl
{


IVector3::IVector3() {} 

IVector3::IVector3(FCL_REAL v) { setValue(v); }

IVector3::IVector3(FCL_REAL x, FCL_REAL y, FCL_REAL z) { setValue(x, y, z); }

IVector3::IVector3(FCL_REAL xl, FCL_REAL xu, FCL_REAL yl, FCL_REAL yu, FCL_REAL zl, FCL_REAL zu)
{
  setValue(xl, xu, yl, yu, zl, zu);
}

IVector3::IVector3(FCL_REAL v[3][2]) { setValue(v); }

IVector3::IVector3(Interval v[3]) { setValue(v); }

IVector3::IVector3(const Interval& v1, const Interval& v2, const Interval& v3) { setValue(v1, v2, v3); }

IVector3::IVector3(const Vec3f& v) { setValue(v); }

void IVector3::setZero() { setValue((FCL_REAL)0.0); }

IVector3 IVector3::operator + (const IVector3& other) const
{
  return IVector3(i_[0] + other.i_[0], i_[1] + other.i_[1], i_[2] + other.i_[2]);
}

IVector3& IVector3::operator += (const IVector3& other)
{
  i_[0] += other[0];
  i_[1] += other[1];
  i_[2] += other[2];
  return *this;
}

IVector3 IVector3::operator - (const IVector3& other) const
{
  return IVector3(i_[0] - other.i_[0], i_[1] - other.i_[1], i_[2] - other.i_[2]);
}

IVector3& IVector3::operator -= (const IVector3& other)
{
  i_[0] -= other[0];
  i_[1] -= other[1];
  i_[2] -= other[2];
  return *this;
}

Interval IVector3::dot(const IVector3& other) const
{
  return i_[0] * other.i_[0] + i_[1] * other.i_[1] + i_[2] * other.i_[2];
}

IVector3 IVector3::cross(const IVector3& other) const
{
  return IVector3(i_[1] * other.i_[2] - i_[2] * other.i_[1], 
                  i_[2] * other.i_[0] - i_[0] * other.i_[2],
                  i_[0] * other.i_[1] - i_[1] * other.i_[0]);
}

Interval IVector3::dot(const Vec3f& other) const
{
  return i_[0] * other[0] + i_[1] * other[1] + i_[2] * other[2];
}

IVector3 IVector3::cross(const Vec3f& other) const
{
  return IVector3(i_[1] * other[2] - i_[2] * other[1], 
                  i_[2] * other[0] - i_[0] * other[2],
                  i_[0] * other[1] - i_[1] * other[0]);
}

FCL_REAL IVector3::volumn() const
{
  return i_[0].diameter() * i_[1].diameter() * i_[2].diameter();
}

void IVector3::print() const
{
  std::cout << "[" << i_[0][0] << "," << i_[0][1] << "]" << std::endl;
  std::cout << "[" << i_[1][0] << "," << i_[1][1] << "]" << std::endl;
  std::cout << "[" << i_[2][0] << "," << i_[2][1] << "]" << std::endl;
}

Vec3f IVector3::center() const
{
  return Vec3f(i_[0].center(), i_[1].center(), i_[2].center());
}

void IVector3::bound(const IVector3& v)
{
  if(v[0][0] < i_[0][0]) i_[0][0] = v[0][0];
  if(v[1][0] < i_[1][0]) i_[1][0] = v[1][0];
  if(v[2][0] < i_[2][0]) i_[2][0] = v[2][0];

  if(v[0][1] > i_[0][1]) i_[0][1] = v[0][1];
  if(v[1][1] > i_[1][1]) i_[1][1] = v[1][1];
  if(v[2][1] > i_[2][1]) i_[2][1] = v[2][1];
}

void IVector3::bound(const Vec3f& v)
{
  if(v[0] < i_[0][0]) i_[0][0] = v[0];
  if(v[1] < i_[1][0]) i_[1][0] = v[1];
  if(v[2] < i_[2][0]) i_[2][0] = v[2];

  if(v[0] > i_[0][1]) i_[0][1] = v[0];
  if(v[1] > i_[1][1]) i_[1][1] = v[1];
  if(v[2] > i_[2][1]) i_[2][1] = v[2];
}


IVector3 bound(const IVector3& i, const IVector3& v)
{
  IVector3 res(i);
  if(v[0][0] < res.i_[0][0]) res.i_[0][0] = v[0][0];
  if(v[1][0] < res.i_[1][0]) res.i_[1][0] = v[1][0];
  if(v[2][0] < res.i_[2][0]) res.i_[2][0] = v[2][0];

  if(v[0][1] > res.i_[0][1]) res.i_[0][1] = v[0][1];
  if(v[1][1] > res.i_[1][1]) res.i_[1][1] = v[1][1];
  if(v[2][1] > res.i_[2][1]) res.i_[2][1] = v[2][1];

  return res;
}

IVector3 bound(const IVector3& i, const Vec3f& v)
{
  IVector3 res(i);
  if(v[0] < res.i_[0][0]) res.i_[0][0] = v[0];
  if(v[1] < res.i_[1][0]) res.i_[1][0] = v[1];
  if(v[2] < res.i_[2][0]) res.i_[2][0] = v[2];

  if(v[0] > res.i_[0][1]) res.i_[0][1] = v[0];
  if(v[1] > res.i_[1][1]) res.i_[1][1] = v[1];
  if(v[2] > res.i_[2][1]) res.i_[2][1] = v[2];

  return res;
}

bool IVector3::overlap(const IVector3& v) const
{
  if(v[0][1] < i_[0][0]) return false;
  if(v[1][1] < i_[1][0]) return false;
  if(v[2][1] < i_[2][0]) return false;

  if(v[0][0] > i_[0][1]) return false;
  if(v[1][0] > i_[1][1]) return false;
  if(v[2][0] > i_[2][1]) return false;

  return true;
}

bool IVector3::contain(const IVector3& v) const
{
  if(v[0][0] < i_[0][0]) return false;
  if(v[1][0] < i_[1][0]) return false;
  if(v[2][0] < i_[2][0]) return false;

  if(v[0][1] > i_[0][1]) return false;
  if(v[1][1] > i_[1][1]) return false;
  if(v[2][1] > i_[2][1]) return false;

  return true;
}



}

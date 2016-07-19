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

#include "fcl/ccd/taylor_vector.h"

namespace fcl
{

TVector3::TVector3()
{
}

TVector3::TVector3(const std::shared_ptr<TimeInterval>& time_interval)
{
  setTimeInterval(time_interval);
}

TVector3::TVector3(TaylorModel v[3])
{
  i_[0] = v[0];
  i_[1] = v[1];
  i_[2] = v[2];
}

TVector3::TVector3(const TaylorModel& v1, const TaylorModel& v2, const TaylorModel& v3)
{
  i_[0] = v1;
  i_[1] = v2;
  i_[2] = v3;
}

TVector3::TVector3(const Vec3f& v, const std::shared_ptr<TimeInterval>& time_interval)
{
  i_[0] = TaylorModel(v[0], time_interval);
  i_[1] = TaylorModel(v[1], time_interval);
  i_[2] = TaylorModel(v[2], time_interval);
}

void TVector3::setZero()
{
  i_[0].setZero();
  i_[1].setZero();
  i_[2].setZero();
}

TVector3 TVector3::operator + (const TVector3& other) const
{
  return TVector3(i_[0] + other.i_[0], i_[1] + other.i_[1], i_[2] + other.i_[2]);
}

TVector3 TVector3::operator - (const TVector3& other) const
{
  return TVector3(i_[0] - other.i_[0], i_[1] - other.i_[1], i_[2] - other.i_[2]);
}

TVector3 TVector3::operator - () const
{
  return TVector3(-i_[0], -i_[1], -i_[2]);
}

TVector3& TVector3::operator += (const TVector3& other)
{
  i_[0] += other.i_[0];
  i_[1] += other.i_[1];
  i_[2] += other.i_[2];
  return *this;
}

TVector3& TVector3::operator -= (const TVector3& other)
{
  i_[0] -= other.i_[0];
  i_[1] -= other.i_[1];
  i_[2] -= other.i_[2];
  return *this;
}

TVector3 TVector3::operator + (const Vec3f& other) const
{
  return TVector3(i_[0] + other[0], i_[1] + other[1], i_[2] + other[2]);
}

TVector3& TVector3::operator += (const Vec3f& other)
{
  i_[0] += other[0];
  i_[1] += other[1];
  i_[2] += other[2];
  return *this;
}

TVector3 TVector3::operator - (const Vec3f& other) const
{
  return TVector3(i_[0] - other[0], i_[1] - other[1], i_[2] - other[2]);
}

TVector3& TVector3::operator -= (const Vec3f& other)
{
  i_[0] -= other[0];
  i_[1] -= other[1];
  i_[2] -= other[2];
  return *this;
}


TVector3 TVector3::operator * (const TaylorModel& d) const
{
  return TVector3(i_[0] * d, i_[1] * d, i_[2] * d);
}

TVector3& TVector3::operator *= (const TaylorModel& d)
{
  i_[0] *= d;
  i_[1] *= d;
  i_[2] *= d;
  return *this;
}

TVector3 TVector3::operator * (FCL_REAL d) const
{
  return TVector3(i_[0] * d, i_[1] * d, i_[2] * d);
}

TVector3& TVector3::operator *= (FCL_REAL d)
{
  i_[0] *= d;
  i_[1] *= d;
  i_[2] *= d;
  return *this;
}

const TaylorModel& TVector3::operator [] (size_t i) const
{
  return i_[i];
}

TaylorModel& TVector3::operator [] (size_t i)
{
  return i_[i];
}

TaylorModel TVector3::dot(const TVector3& other) const
{
  return i_[0] * other.i_[0] + i_[1] * other.i_[1] + i_[2] * other.i_[2];
}

TVector3 TVector3::cross(const TVector3& other) const
{
  return TVector3(i_[1] * other.i_[2] - i_[2] * other.i_[1], 
                  i_[2] * other.i_[0] - i_[0] * other.i_[2],
                  i_[0] * other.i_[1] - i_[1] * other.i_[0]);
}

TaylorModel TVector3::dot(const Vec3f& other) const
{
  return i_[0] * other[0] + i_[1] * other[1] + i_[2] * other[2];
}

TVector3 TVector3::cross(const Vec3f& other) const
{
  return TVector3(i_[1] * other[2] - i_[2] * other[1], 
                  i_[2] * other[0] - i_[0] * other[2],
                  i_[0] * other[1] - i_[1] * other[0]);
}

FCL_REAL TVector3::volumn() const
{
  return i_[0].getBound().diameter() * i_[1].getBound().diameter() * i_[2].getBound().diameter();
}

IVector3 TVector3::getBound() const
{
  return IVector3(i_[0].getBound(), i_[1].getBound(), i_[2].getBound());
}

IVector3 TVector3::getBound(FCL_REAL l, FCL_REAL r) const
{
  return IVector3(i_[0].getBound(l, r), i_[1].getBound(l, r), i_[2].getBound(l, r));
}

IVector3 TVector3::getBound(FCL_REAL t) const
{
  return IVector3(i_[0].getBound(t), i_[1].getBound(t), i_[2].getBound(t));
}

IVector3 TVector3::getTightBound() const
{
  return IVector3(i_[0].getTightBound(), i_[1].getTightBound(), i_[2].getTightBound());
}

IVector3 TVector3::getTightBound(FCL_REAL l, FCL_REAL r) const
{
  return IVector3(i_[0].getTightBound(l, r), i_[1].getTightBound(l, r), i_[2].getTightBound(l, r));
}


void TVector3::print() const
{
  i_[0].print();
  i_[1].print();
  i_[2].print();
}


TaylorModel TVector3::squareLength() const
{
  return i_[0] * i_[0] + i_[1] * i_[1] + i_[2] * i_[2];
}

void TVector3::setTimeInterval(const std::shared_ptr<TimeInterval>& time_interval)
{
  i_[0].setTimeInterval(time_interval);
  i_[1].setTimeInterval(time_interval);
  i_[2].setTimeInterval(time_interval);
}

void TVector3::setTimeInterval(FCL_REAL l, FCL_REAL r)
{
  i_[0].setTimeInterval(l, r);
  i_[1].setTimeInterval(l, r);
  i_[2].setTimeInterval(l, r);
}

const std::shared_ptr<TimeInterval>& TVector3::getTimeInterval() const
{
  return i_[0].getTimeInterval();
}

void generateTVector3ForLinearFunc(TVector3& v, const Vec3f& position, const Vec3f& velocity)
{
  generateTaylorModelForLinearFunc(v[0], position[0], velocity[0]);
  generateTaylorModelForLinearFunc(v[1], position[1], velocity[1]);
  generateTaylorModelForLinearFunc(v[2], position[2], velocity[2]);
}


TVector3 operator * (const Vec3f& v, const TaylorModel& a)
{
  TVector3 res(a.getTimeInterval());
  res[0] = a * v[0];
  res[1] = a * v[1];
  res[2] = a * v[2];

  return res;
}

TVector3 operator + (const Vec3f& v1, const TVector3& v2)
{
  return v2 + v1;
}

TVector3 operator - (const Vec3f& v1, const TVector3& v2)
{
  return -v2 + v1;
}



}

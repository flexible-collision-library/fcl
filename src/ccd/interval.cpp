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

#include "fcl/ccd/interval.h"
#include <iostream>

namespace fcl
{

Interval bound(const Interval& i, FCL_REAL v)
{
  Interval res = i;
  if(v < res.i_[0]) res.i_[0] = v;
  if(v > res.i_[1]) res.i_[1] = v;
  return res;
}

Interval bound(const Interval& i, const Interval& other)
{
  Interval res = i;
  if(other.i_[0] < res.i_[0]) res.i_[0] = other.i_[0];
  if(other.i_[1] > res.i_[1]) res.i_[1] = other.i_[1];
  return res;
}

Interval Interval::operator * (const Interval& other) const
{
  if(other.i_[0] >= 0)
  {
    if(i_[0] >= 0) return Interval(i_[0] * other.i_[0], i_[1] * other.i_[1]);
    if(i_[1] <= 0) return Interval(i_[0] * other.i_[1], i_[1] * other.i_[0]);
    return Interval(i_[0] * other.i_[1], i_[1] * other.i_[1]);
  }
  if(other.i_[1] <= 0)
  {
    if(i_[0] >= 0) return Interval(i_[1] * other.i_[0], i_[0] * other.i_[1]);
    if(i_[1] <= 0) return Interval(i_[1] * other.i_[1], i_[0] * other.i_[0]);
    return Interval(i_[1] * other.i_[0], i_[0] * other.i_[0]);
  }

  if(i_[0] >= 0) return Interval(i_[1] * other.i_[0], i_[1] * other.i_[1]);
  if(i_[1] <= 0) return Interval(i_[0] * other.i_[1], i_[0] * other.i_[0]);

  FCL_REAL v00 = i_[0] * other.i_[0];
  FCL_REAL v11 = i_[1] * other.i_[1];
  if(v00 <= v11)
  {
    FCL_REAL v01 = i_[0] * other.i_[1];
    FCL_REAL v10 = i_[1] * other.i_[0];
    if(v01 < v10) return Interval(v01, v11);
    return Interval(v10, v11);
  }

  FCL_REAL v01 = i_[0] * other.i_[1];
  FCL_REAL v10 = i_[1] * other.i_[0];
  if(v01 < v10) return Interval(v01, v00);
  return Interval(v10, v00);
}

Interval& Interval::operator *= (const Interval& other)
{
  if(other.i_[0] >= 0)
  {
    if(i_[0] >= 0) 
    {
      i_[0] *= other.i_[0];
      i_[1] *= other.i_[1];
    }
    else if(i_[1] <= 0)
    {
      i_[0] *= other.i_[1];
      i_[1] *= other.i_[0];
    }
    else
    {
      i_[0] *= other.i_[1];
      i_[1] *= other.i_[1];
    }
    return *this;
  }

  if(other.i_[1] <= 0)
  {
    if(i_[0] >= 0)
    {
      FCL_REAL tmp = i_[0];
      i_[0] = i_[1] * other.i_[0];
      i_[1] = tmp * other.i_[1];
    }
    else if(i_[1] <= 0)
    {
      FCL_REAL tmp = i_[0];
      i_[0] = i_[1] * other.i_[1];
      i_[1] = tmp * other.i_[0];
    }
    else
    {
      FCL_REAL tmp = i_[0];
      i_[0] = i_[1] * other.i_[0];
      i_[1] = tmp * other.i_[0];  
    }
    return *this;
  }

  if(i_[0] >= 0) 
  {
    i_[0] = i_[1] * other.i_[0];
    i_[1] *= other.i_[1];
    return *this;
  }

  if(i_[1] <= 0) 
  {
    i_[1] = i_[0] * other.i_[0];
    i_[0] *= other.i_[1];
    return *this;
  }

  FCL_REAL v00 = i_[0] * other.i_[0];
  FCL_REAL v11 = i_[1] * other.i_[1];
  if(v00 <= v11)
  {
    FCL_REAL v01 = i_[0] * other.i_[1];
    FCL_REAL v10 = i_[1] * other.i_[0];
    if(v01 < v10)
    {
      i_[0] = v01;
      i_[1] = v11;
    }
    else
    {
      i_[0] = v10;
      i_[1] = v11;
    }
    return *this;
  }

  FCL_REAL v01 = i_[0] * other.i_[1];
  FCL_REAL v10 = i_[1] * other.i_[0];
  if(v01 < v10)
  {
    i_[0] = v01;
    i_[1] = v00;
  }
  else
  {
    i_[0] = v10;
    i_[1] = v00;
  }

  return *this;
}

Interval Interval::operator / (const Interval& other) const
{
  return *this * Interval(1.0 / other.i_[1], 1.0 / other.i_[0]);
}

Interval& Interval::operator /= (const Interval& other)
{
  *this *= Interval(1.0 / other.i_[1], 1.0 / other.i_[0]);
  return *this;
}

void Interval::print() const
{
  std::cout << "[" << i_[0] << ", " << i_[1] << "]" << std::endl;
}

}

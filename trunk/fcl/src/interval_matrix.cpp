/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#include "fcl/interval_matrix.h"
#include <iostream>

namespace fcl
{

IMatrix3::IMatrix3()
{
  i_[0][0] = i_[0][1] = i_[0][2] = i_[1][0] = i_[1][1] = i_[1][2] = i_[2][0] = i_[2][1] = i_[2][2] = 0;
}

IMatrix3::IMatrix3(BVH_REAL v)
{
  i_[0][0] = i_[0][1] = i_[0][2] = i_[1][0] = i_[1][1] = i_[1][2] = i_[2][0] = i_[2][1] = i_[2][2] = v;
}

IMatrix3::IMatrix3(const Matrix3f& m)
{
  i_[0][0] = m[0][0];
  i_[0][1] = m[0][1];
  i_[0][2] = m[0][2];

  i_[1][0] = m[1][0];
  i_[1][1] = m[1][1];
  i_[1][2] = m[1][2];

  i_[2][0] = m[2][0];
  i_[2][1] = m[2][1];
  i_[2][2] = m[2][2];
}

IMatrix3::IMatrix3(BVH_REAL m[3][3][2])
{
  i_[0][0].setValue(m[0][0][0], m[0][0][1]);
  i_[0][1].setValue(m[0][1][0], m[0][1][1]);
  i_[0][2].setValue(m[0][2][0], m[0][2][1]);

  i_[1][0].setValue(m[1][0][0], m[1][0][1]);
  i_[1][1].setValue(m[1][1][0], m[1][1][1]);
  i_[1][2].setValue(m[1][2][0], m[1][2][1]);

  i_[2][0].setValue(m[2][0][0], m[2][0][1]);
  i_[2][1].setValue(m[2][1][0], m[2][1][1]);
  i_[2][2].setValue(m[2][2][0], m[2][2][1]);
}

IMatrix3::IMatrix3(BVH_REAL m[3][3])
{
  i_[0][0].setValue(m[0][0]);
  i_[0][1].setValue(m[0][1]);
  i_[0][2].setValue(m[0][2]);

  i_[1][0].setValue(m[1][0]);
  i_[1][1].setValue(m[1][1]);
  i_[1][2].setValue(m[1][2]);

  i_[2][0].setValue(m[2][0]);
  i_[2][1].setValue(m[2][1]);
  i_[2][2].setValue(m[2][2]);
}

IMatrix3::IMatrix3(Interval m[3][3])
{
  i_[0][0] = m[0][0];
  i_[0][1] = m[0][1];
  i_[0][2] = m[0][2];

  i_[1][0] = m[1][0];
  i_[1][1] = m[1][1];
  i_[1][2] = m[1][2];

  i_[2][0] = m[2][0];
  i_[2][1] = m[2][1];
  i_[2][2] = m[2][2];
}

void IMatrix3::setIdentity()
{
  i_[0][0] = 1;
  i_[0][1] = 0;
  i_[0][2] = 0;

  i_[1][0] = 0;
  i_[1][1] = 1;
  i_[1][2] = 0;

  i_[2][0] = 0;
  i_[2][1] = 0;
  i_[2][2] = 1;
}

IVector3 IMatrix3::getColumn(size_t i) const
{
  return IVector3(i_[0][i], i_[1][i], i_[2][i]);
}

IVector3 IMatrix3::getRow(size_t i) const
{
  return IVector3(i_[i][0], i_[i][1], i_[i][2]);
}

Vec3f IMatrix3::getRealColumn(size_t i) const
{
  return Vec3f(i_[0][i][0], i_[1][i][0], i_[2][i][0]);
}

Vec3f IMatrix3::getRealRow(size_t i) const
{
  return Vec3f(i_[i][0][0], i_[i][1][0], i_[i][2][0]);
}

IMatrix3 IMatrix3::operator * (const Matrix3f& m) const
{
  BVH_REAL res[3][3][2];

  if(m[0][0] < 0)
  {
    res[0][0][0] = m[0][0] * i_[0][0][1];
    res[0][0][1] = m[0][0] * i_[0][0][0];
    res[1][0][0] = m[0][0] * i_[1][0][1];
    res[1][0][1] = m[0][0] * i_[1][0][0];
    res[2][0][0] = m[0][0] * i_[2][0][1];
    res[2][0][1] = m[0][0] * i_[2][0][0];
  }
  else
  {
    res[0][0][0] = m[0][0] * i_[0][0][0];
    res[0][0][1] = m[0][0] * i_[0][0][1];
    res[1][0][0] = m[0][0] * i_[1][0][0];
    res[1][0][1] = m[0][0] * i_[1][0][1];
    res[2][0][0] = m[0][0] * i_[2][0][0];
    res[2][0][1] = m[0][0] * i_[2][0][1];
  }

  if(m[0][1] < 0)
  {
    res[0][1][0] = m[0][1] * i_[0][0][1];
    res[0][1][1] = m[0][1] * i_[0][0][0];
    res[1][1][0] = m[0][1] * i_[1][0][1];
    res[1][1][1] = m[0][1] * i_[1][0][0];
    res[2][1][0] = m[0][1] * i_[2][0][1];
    res[2][1][1] = m[0][1] * i_[2][0][0];
  }
  else
  {
    res[0][1][0] = m[0][1] * i_[0][0][0];
    res[0][1][1] = m[0][1] * i_[0][0][1];
    res[1][1][0] = m[0][1] * i_[1][0][0];
    res[1][1][1] = m[0][1] * i_[1][0][1];
    res[2][1][0] = m[0][1] * i_[2][0][0];
    res[2][1][1] = m[0][1] * i_[2][0][1];
  }

  if(m[0][2] < 0)
  {
    res[0][2][0] = m[0][2] * i_[0][0][1];
    res[0][2][1] = m[0][2] * i_[0][0][0];
    res[1][2][0] = m[0][2] * i_[1][0][1];
    res[1][2][1] = m[0][2] * i_[1][0][0];
    res[2][2][0] = m[0][2] * i_[2][0][1];
    res[2][2][1] = m[0][2] * i_[2][0][0];
  }
  else
  {
    res[0][2][0] = m[0][2] * i_[0][0][0];
    res[0][2][1] = m[0][2] * i_[0][0][1];
    res[1][2][0] = m[0][2] * i_[1][0][0];
    res[1][2][1] = m[0][2] * i_[1][0][1];
    res[2][2][0] = m[0][2] * i_[2][0][0];
    res[2][2][1] = m[0][2] * i_[2][0][1];
  }

  if(m[1][0] < 0)
  {
    res[0][0][0] += m[1][0] * i_[0][1][1];
    res[0][0][1] += m[1][0] * i_[0][1][0];
    res[1][0][0] += m[1][0] * i_[1][1][1];
    res[1][0][1] += m[1][0] * i_[1][1][0];
    res[2][0][0] += m[1][0] * i_[2][1][1];
    res[2][0][1] += m[1][0] * i_[2][1][0];
  }
  else
  {
    res[0][0][0] += m[1][0] * i_[0][1][0];
    res[0][0][1] += m[1][0] * i_[0][1][1];
    res[1][0][0] += m[1][0] * i_[1][1][0];
    res[1][0][1] += m[1][0] * i_[1][1][1];
    res[2][0][0] += m[1][0] * i_[2][1][0];
    res[2][0][1] += m[1][0] * i_[2][1][1];
  }

  if(m[1][1] < 0)
  {
    res[0][1][0] += m[1][1] * i_[0][1][1];
    res[0][1][1] += m[1][1] * i_[0][1][0];
    res[1][1][0] += m[1][1] * i_[1][1][1];
    res[1][1][1] += m[1][1] * i_[1][1][0];
    res[2][1][0] += m[1][1] * i_[2][1][1];
    res[2][1][1] += m[1][1] * i_[2][1][0];
  }
  else
  {
    res[0][1][0] += m[1][1] * i_[0][1][0];
    res[0][1][1] += m[1][1] * i_[0][1][1];
    res[1][1][0] += m[1][1] * i_[1][1][0];
    res[1][1][1] += m[1][1] * i_[1][1][1];
    res[2][1][0] += m[1][1] * i_[2][1][0];
    res[2][1][1] += m[1][1] * i_[2][1][1];
  }

  if(m[1][2] < 0)
  {
    res[0][2][0] += m[1][2] * i_[0][1][1];
    res[0][2][1] += m[1][2] * i_[0][1][0];
    res[1][2][0] += m[1][2] * i_[1][1][1];
    res[1][2][1] += m[1][2] * i_[1][1][0];
    res[2][2][0] += m[1][2] * i_[2][1][1];
    res[2][2][1] += m[1][2] * i_[2][1][0];
  }
  else
  {
    res[0][2][0] += m[1][2] * i_[0][1][0];
    res[0][2][1] += m[1][2] * i_[0][1][1];
    res[1][2][0] += m[1][2] * i_[1][1][0];
    res[1][2][1] += m[1][2] * i_[1][1][1];
    res[2][2][0] += m[1][2] * i_[2][1][0];
    res[2][2][1] += m[1][2] * i_[2][1][1];
  }

  if(m[2][0] < 0)
  {
    res[0][0][0] += m[2][0] * i_[0][2][1];
    res[0][0][1] += m[2][0] * i_[0][2][0];
    res[1][0][0] += m[2][0] * i_[1][2][1];
    res[1][0][1] += m[2][0] * i_[1][2][0];
    res[2][0][0] += m[2][0] * i_[2][2][1];
    res[2][0][1] += m[2][0] * i_[2][2][0];
  }
  else
  {
    res[0][0][0] += m[2][0] * i_[0][2][0];
    res[0][0][1] += m[2][0] * i_[0][2][1];
    res[1][0][0] += m[2][0] * i_[1][2][0];
    res[1][0][1] += m[2][0] * i_[1][2][1];
    res[2][0][0] += m[2][0] * i_[2][2][0];
    res[2][0][1] += m[2][0] * i_[2][2][1];
  }

  if(m[2][1] < 0)
  {
    res[0][1][0] += m[2][1] * i_[0][2][1];
    res[0][1][1] += m[2][1] * i_[0][2][0];
    res[1][1][0] += m[2][1] * i_[1][2][1];
    res[1][1][1] += m[2][1] * i_[1][2][0];
    res[2][1][0] += m[2][1] * i_[2][2][1];
    res[2][1][1] += m[2][1] * i_[2][2][0];
  }
  else
  {
    res[0][1][0] += m[2][1] * i_[0][2][0];
    res[0][1][1] += m[2][1] * i_[0][2][1];
    res[1][1][0] += m[2][1] * i_[1][2][0];
    res[1][1][1] += m[2][1] * i_[1][2][1];
    res[2][1][0] += m[2][1] * i_[2][2][0];
    res[2][1][1] += m[2][1] * i_[2][2][1];
  }

  if(m[2][2] < 0)
  {
    res[0][2][0] += m[2][2] * i_[0][2][1];
    res[0][2][1] += m[2][2] * i_[0][2][0];
    res[1][2][0] += m[2][2] * i_[1][2][1];
    res[1][2][1] += m[2][2] * i_[1][2][0];
    res[2][2][0] += m[2][2] * i_[2][2][1];
    res[2][2][1] += m[2][2] * i_[2][2][0];
  }
  else
  {
    res[0][2][0] += m[2][2] * i_[0][2][0];
    res[0][2][1] += m[2][2] * i_[0][2][1];
    res[1][2][0] += m[2][2] * i_[1][2][0];
    res[1][2][1] += m[2][2] * i_[1][2][1];
    res[2][2][0] += m[2][2] * i_[2][2][0];
    res[2][2][1] += m[2][2] * i_[2][2][1];
  }

  return IMatrix3(res);
}

IVector3 IMatrix3::operator * (const Vec3f& v) const
{
  BVH_REAL res[3][2];

  if(v[0] < 0)
  {
    res[0][0] = v[0] * i_[0][0][1];
    res[0][1] = v[0] * i_[0][0][0];
    res[1][0] = v[0] * i_[1][0][1];
    res[1][1] = v[0] * i_[1][0][0];
    res[2][0] = v[0] * i_[2][0][1];
    res[2][1] = v[0] * i_[2][0][0];
  }
  else
  {
    res[0][0] = v[0] * i_[0][0][0];
    res[0][1] = v[0] * i_[0][0][1];
    res[1][0] = v[0] * i_[1][0][0];
    res[1][1] = v[0] * i_[1][0][1];
    res[2][0] = v[0] * i_[2][0][0];
    res[2][1] = v[0] * i_[2][0][1];
  }

  if(v[1] < 0)
  {
    res[0][0] += v[1] * i_[0][1][1];
    res[0][1] += v[1] * i_[0][1][0];
    res[1][0] += v[1] * i_[1][1][1];
    res[1][1] += v[1] * i_[1][1][0];
    res[2][0] += v[1] * i_[2][1][1];
    res[2][1] += v[1] * i_[2][1][0];
  }
  else
  {
    res[0][0] += v[1] * i_[0][1][0];
    res[0][1] += v[1] * i_[0][1][1];
    res[1][0] += v[1] * i_[1][1][0];
    res[1][1] += v[1] * i_[1][1][1];
    res[2][0] += v[1] * i_[2][1][0];
    res[2][1] += v[1] * i_[2][1][1];
  }

  if(v[2] < 0)
  {
    res[0][0] += v[2] * i_[0][2][1];
    res[0][1] += v[2] * i_[0][2][0];
    res[1][0] += v[2] * i_[1][2][1];
    res[1][1] += v[2] * i_[1][2][0];
    res[2][0] += v[2] * i_[2][2][1];
    res[2][1] += v[2] * i_[2][2][0];
  }
  else
  {
    res[0][0] += v[2] * i_[0][2][0];
    res[0][1] += v[2] * i_[0][2][1];
    res[1][0] += v[2] * i_[1][2][0];
    res[1][1] += v[2] * i_[1][2][1];
    res[2][0] += v[2] * i_[2][2][0];
    res[2][1] += v[2] * i_[2][2][1];
  }

  return IVector3(res);
}


IMatrix3 IMatrix3::nonIntervalAddMatrix(const IMatrix3& m) const
{
  BVH_REAL res[3][3];

  res[0][0] = i_[0][0][0] + m.i_[0][0][0];
  res[0][1] = i_[0][1][0] + m.i_[0][1][0];
  res[0][2] = i_[0][2][0] + m.i_[0][2][0];

  res[1][0] = i_[1][0][0] + m.i_[1][0][0];
  res[1][1] = i_[1][1][0] + m.i_[1][1][0];
  res[1][2] = i_[1][2][0] + m.i_[1][2][0];

  res[2][0] = i_[2][0][0] + m.i_[2][0][0];
  res[2][1] = i_[2][1][0] + m.i_[2][1][0];
  res[2][2] = i_[2][2][0] + m.i_[2][2][0];

  return IMatrix3(res);
}

IVector3 IMatrix3::operator * (const IVector3& v) const
{
  BVH_REAL xl, xu, yl, yu, zl, zu;
  register BVH_REAL temp, vmin, vmax;

  // r.v.i_[0]
  vmin = vmax = i_[0][0][0] * v.i_[0][0];
  temp = i_[0][0][1] * v.i_[0][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][0][1] * v.i_[0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  xl = vmin; xu = vmax;

  vmin = vmax = i_[0][1][0] * v.i_[1][0];
  temp = i_[0][1][0] * v.i_[1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][1][1] * v.i_[1][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][1][1] * v.i_[1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  xl += vmin; xu += vmax;

  vmin = vmax = i_[0][2][0] * v.i_[2][0];
  temp = i_[0][2][0] * v.i_[2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][2][1] * v.i_[2][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][2][1] * v.i_[2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  xl += vmin; xu += vmax;

  // r.v.i_[1]

  vmin = vmax = i_[1][0][0] * v.i_[0][0];
  temp = i_[1][0][0] * v.i_[0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][0][1] * v.i_[0][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][0][1] * v.i_[0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  yl = vmin; yu = vmax;

  vmin = vmax = i_[1][1][0] * v.i_[1][0];
  temp = i_[1][1][0] * v.i_[1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][1][1] * v.i_[1][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][1][1] * v.i_[1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  yl += vmin; yu += vmax;

  vmin = vmax = i_[1][2][0] * v.i_[2][0];
  temp = i_[1][2][0] * v.i_[2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][2][1] * v.i_[2][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][2][1] * v.i_[2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  yl += vmin; yu += vmax;

  // r.v.i_[2]

  vmin = vmax = i_[2][0][0] * v.i_[0][0];
  temp = i_[2][0][0] * v.i_[0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][0][1] * v.i_[0][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][0][1] * v.i_[0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  zl = vmin; zu = vmax;

  vmin = vmax = i_[2][1][0] * v.i_[1][0];
  temp = i_[2][1][0] * v.i_[1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][1][1] * v.i_[1][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][1][1] * v.i_[1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  zl += vmin; zu += vmax;

  vmin = vmax = i_[2][2][0] * v.i_[2][0];
  temp = i_[2][2][0] * v.i_[2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][2][1] * v.i_[2][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][2][1] * v.i_[2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  zl += vmin; zu += vmax;      vmin = vmax = i_[0][0][0] * v.i_[0][0];
  temp = i_[0][0][0] * v.i_[0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][0][1] * v.i_[0][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][0][1] * v.i_[0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  xl = vmin; xu = vmax;

  vmin = vmax = i_[0][1][0] * v.i_[1][0];
  temp = i_[0][1][0] * v.i_[1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][1][1] * v.i_[1][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][1][1] * v.i_[1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  xl += vmin; xu += vmax;

  vmin = vmax = i_[0][2][0] * v.i_[2][0];
  temp = i_[0][2][0] * v.i_[2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][2][1] * v.i_[2][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][2][1] * v.i_[2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  xl += vmin; xu += vmax;

  // r.v.i_[1]

  vmin = vmax = i_[1][0][0] * v.i_[0][0];
  temp = i_[1][0][0] * v.i_[0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][0][1] * v.i_[0][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][0][1] * v.i_[0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  yl = vmin; yu = vmax;

  vmin = vmax = i_[1][1][0] * v.i_[1][0];
  temp = i_[1][1][0] * v.i_[1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][1][1] * v.i_[1][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][1][1] * v.i_[1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  yl += vmin; yu += vmax;

  vmin = vmax = i_[1][2][0] * v.i_[2][0];
  temp = i_[1][2][0] * v.i_[2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][2][1] * v.i_[2][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][2][1] * v.i_[2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  yl += vmin; yu += vmax;

  // r.v.i_[2]

  vmin = vmax = i_[2][0][0] * v.i_[0][0];
  temp = i_[2][0][0] * v.i_[0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][0][1] * v.i_[0][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][0][1] * v.i_[0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  zl = vmin; zu = vmax;

  vmin = vmax = i_[2][1][0] * v.i_[1][0];
  temp = i_[2][1][0] * v.i_[1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][1][1] * v.i_[1][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][1][1] * v.i_[1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  zl += vmin; zu += vmax;

  vmin = vmax = i_[2][2][0] * v.i_[2][0];
  temp = i_[2][2][0] * v.i_[2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][2][1] * v.i_[2][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][2][1] * v.i_[2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  zl += vmin; zu += vmax;

  return IVector3(xl, xu, yl, yu, zl, zu);
}

IMatrix3 IMatrix3::operator * (const IMatrix3& m) const
{

  register BVH_REAL temp, vmin, vmax;
  BVH_REAL res[3][3][2];

  // res[0][0]

  vmin = vmax = i_[0][0][0] * m.i_[0][0][0];
  temp = i_[0][0][0] * m.i_[0][0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][0][1] * m.i_[0][0][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][0][1] * m.i_[0][0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[0][0][0] = vmin; res[0][0][1] = vmax;

  vmin = vmax = i_[0][1][0] * m.i_[1][0][0];
  temp = i_[0][1][0] * m.i_[1][0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][1][1] * m.i_[1][0][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][1][1] * m.i_[1][0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[0][0][0] += vmin; res[0][0][1] += vmax;

  vmin = vmax = i_[0][2][0] * m.i_[2][0][0];
  temp = i_[0][2][0] * m.i_[2][0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][2][1] * m.i_[2][0][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][2][1] * m.i_[2][0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[0][0][0] += vmin; res[0][0][1] += vmax;

  // res[1][0]

  vmin = vmax = i_[1][0][0] * m.i_[0][0][0];
  temp = i_[1][0][0] * m.i_[0][0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][0][1] * m.i_[0][0][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][0][1] * m.i_[0][0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[1][0][0] = vmin; res[1][0][1] = vmax;

  vmin = vmax = i_[1][1][0] * m.i_[1][0][0];
  temp = i_[1][1][0] * m.i_[1][0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][1][1] * m.i_[1][0][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][1][1] * m.i_[1][0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[1][0][0] += vmin; res[1][0][1] += vmax;

  vmin = vmax = i_[1][2][0] * m.i_[2][0][0];
  temp = i_[1][2][0] * m.i_[2][0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][2][1] * m.i_[2][0][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][2][1] * m.i_[2][0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[1][0][0] += vmin; res[1][0][1] += vmax;

  // res[2][0]

  vmin = vmax = i_[2][0][0] * m.i_[0][0][0];
  temp = i_[2][0][0] * m.i_[0][0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][0][1] * m.i_[0][0][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][0][1] * m.i_[0][0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[2][0][0] = vmin; res[2][0][1] = vmax;

  vmin = vmax = i_[2][1][0] * m.i_[1][0][0];
  temp = i_[2][1][0] * m.i_[1][0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][1][1] * m.i_[1][0][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][1][1] * m.i_[1][0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[2][0][0] += vmin; res[2][0][1] += vmax;

  vmin = vmax = i_[2][2][0] * m.i_[2][0][0];
  temp = i_[2][2][0] * m.i_[2][0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][2][1] * m.i_[2][0][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][2][1] * m.i_[2][0][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[2][0][0] += vmin; res[2][0][1] += vmax;

  // res[0][1]

  vmin = vmax = i_[0][0][0] * m.i_[0][1][0];
  temp = i_[0][0][0] * m.i_[0][1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][0][1] * m.i_[0][1][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][0][1] * m.i_[0][1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[0][1][0] = vmin; res[0][1][1] = vmax;

  vmin = vmax = i_[0][1][0] * m.i_[1][1][0];
  temp = i_[0][1][0] * m.i_[1][1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][1][1] * m.i_[1][1][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][1][1] * m.i_[1][1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[0][1][0] += vmin; res[0][1][1] += vmax;

  vmin = vmax = i_[0][2][0] * m.i_[2][1][0];
  temp = i_[0][2][0] * m.i_[2][1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][2][1] * m.i_[2][1][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][2][1] * m.i_[2][1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[0][1][0] += vmin; res[0][1][1] += vmax;

  // res[1][1]

  vmin = vmax = i_[1][0][0] * m.i_[0][1][0];
  temp = i_[1][0][0] * m.i_[0][1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][0][1] * m.i_[0][1][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][0][1] * m.i_[0][1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[1][1][0] = vmin; res[1][1][1] = vmax;

  vmin = vmax = i_[1][1][0] * m.i_[1][1][0];
  temp = i_[1][1][0] * m.i_[1][1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][1][1] * m.i_[1][1][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][1][1] * m.i_[1][1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[1][1][0] += vmin; res[1][1][1] += vmax;

  vmin = vmax = i_[1][2][0] * m.i_[2][1][0];
  temp = i_[1][2][0] * m.i_[2][1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][2][1] * m.i_[2][1][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][2][1] * m.i_[2][1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[1][1][0] += vmin; res[1][1][1] += vmax;

  // res[2][1]

  vmin = vmax = i_[2][0][0] * m.i_[0][1][0];
  temp = i_[2][0][0] * m.i_[0][1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][0][1] * m.i_[0][1][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][0][1] * m.i_[0][1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[2][1][0] = vmin; res[2][1][1] = vmax;

  vmin = vmax = i_[2][1][0] * m.i_[1][1][0];
  temp = i_[2][1][0] * m.i_[1][1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][1][1] * m.i_[1][1][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][1][1] * m.i_[1][1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[2][1][0] += vmin; res[2][1][1] += vmax;

  vmin = vmax = i_[2][2][0] * m.i_[2][1][0];
  temp = i_[2][2][0] * m.i_[2][1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][2][1] * m.i_[2][1][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][2][1] * m.i_[2][1][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[2][1][0] += vmin; res[2][1][1] += vmax;

  // res[0][2]

  vmin = vmax = i_[0][0][0] * m.i_[0][2][0];
  temp = i_[0][0][0] * m.i_[0][2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][0][1] * m.i_[0][2][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][0][1] * m.i_[0][2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[0][2][0] = vmin; res[0][2][1] = vmax;

  vmin = vmax = i_[0][1][0] * m.i_[1][2][0];
  temp = i_[0][1][0] * m.i_[1][2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][1][1] * m.i_[1][2][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][1][1] * m.i_[1][2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[0][2][0] += vmin; res[0][2][1] += vmax;

  vmin = vmax = i_[0][2][0] * m.i_[2][2][0];
  temp = i_[0][2][0] * m.i_[2][2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][2][1] * m.i_[2][2][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[0][2][1] * m.i_[2][2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[0][2][0] += vmin; res[0][2][1] += vmax;

  // res[1][2]

  vmin = vmax = i_[1][0][0] * m.i_[0][2][0];
  temp = i_[1][0][0] * m.i_[0][2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][0][1] * m.i_[0][2][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][0][1] * m.i_[0][2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[1][2][0] = vmin; res[1][2][1] = vmax;

  vmin = vmax = i_[1][1][0] * m.i_[1][2][0];
  temp = i_[1][1][0] * m.i_[1][2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][1][1] * m.i_[1][2][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][1][1] * m.i_[1][2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[1][2][0] += vmin; res[1][2][1] += vmax;

  vmin = vmax = i_[1][2][0] * m.i_[2][2][0];
  temp = i_[1][2][0] * m.i_[2][2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][2][1] * m.i_[2][2][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[1][2][1] * m.i_[2][2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[1][2][0] += vmin; res[1][2][1] += vmax;

  // res[2][2]

  vmin = vmax = i_[2][0][0] * m.i_[0][2][0];
  temp = i_[2][0][0] * m.i_[0][2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][0][1] * m.i_[0][2][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][0][1] * m.i_[0][2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[2][2][0] = vmin; res[2][2][1] = vmax;

  vmin = vmax = i_[2][1][0] * m.i_[1][2][0];
  temp = i_[2][1][0] * m.i_[1][2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][1][1] * m.i_[1][2][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][1][1] * m.i_[1][2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[2][2][0] += vmin; res[2][2][1] += vmax;

  vmin = vmax = i_[2][2][0] * m.i_[2][2][0];
  temp = i_[2][2][0] * m.i_[2][2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][2][1] * m.i_[2][2][0]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  temp = i_[2][2][1] * m.i_[2][2][1]; if(temp < vmin) vmin = temp; else if(temp > vmax) vmax = temp;
  res[2][2][0] += vmin; res[2][2][1] += vmax;

  return IMatrix3(res);
}

IMatrix3 IMatrix3::operator + (const IMatrix3& m) const
{
  BVH_REAL res[3][3][2];
  res[0][0][0] = i_[0][0][0] + m.i_[0][0][0]; res[0][0][1] = i_[0][0][1] + m.i_[0][0][1]; res[0][1][0] = i_[0][1][0] + m.i_[0][1][0]; res[0][1][1] = i_[0][1][1] + m.i_[0][1][1]; res[0][2][0] = i_[0][2][0] + m.i_[0][2][0]; res[0][2][1] = i_[0][2][1] + m.i_[0][2][1];
  res[1][0][0] = i_[1][0][0] + m.i_[1][0][0]; res[1][0][1] = i_[1][0][1] + m.i_[1][0][1]; res[1][1][0] = i_[1][1][0] + m.i_[1][1][0]; res[1][1][1] = i_[1][1][1] + m.i_[1][1][1]; res[1][2][0] = i_[1][2][0] + m.i_[1][2][0]; res[1][2][1] = i_[1][2][1] + m.i_[1][2][1];
  res[2][0][0] = i_[2][0][0] + m.i_[2][0][0]; res[2][0][1] = i_[2][0][1] + m.i_[2][0][1]; res[2][1][0] = i_[2][1][0] + m.i_[2][1][0]; res[2][1][1] = i_[2][1][1] + m.i_[2][1][1]; res[2][2][0] = i_[2][2][0] + m.i_[2][2][0]; res[2][2][1] = i_[2][2][1] + m.i_[2][2][1];

  return IMatrix3(res);
}

IMatrix3& IMatrix3::operator += (const IMatrix3& m)
{
  i_[0][0][0] += m.i_[0][0][0]; i_[0][0][1] += m.i_[0][0][1]; i_[0][1][0] += m.i_[0][1][0]; i_[0][1][1] += m.i_[0][1][1]; i_[0][2][0] += m.i_[0][2][0]; i_[0][2][1] += m.i_[0][2][1];
  i_[1][0][0] += m.i_[1][0][0]; i_[1][0][1] += m.i_[1][0][1]; i_[1][1][0] += m.i_[1][1][0]; i_[1][1][1] += m.i_[1][1][1]; i_[1][2][0] += m.i_[1][2][0]; i_[1][2][1] += m.i_[1][2][1];
  i_[2][0][0] += m.i_[2][0][0]; i_[2][0][1] += m.i_[2][0][1]; i_[2][1][0] += m.i_[2][1][0]; i_[2][1][1] += m.i_[2][1][1]; i_[2][2][0] += m.i_[2][2][0]; i_[2][2][1] += m.i_[2][2][1];

  return *this;
}

IVector3 IMatrix3::nonIntervalTimesVector(const Vec3f& v) const
{
  return IVector3(i_[0][0][0] * v[0] + i_[0][1][0] * v[1] + i_[0][2][0] * v[2],
                  i_[1][0][0] * v[0] + i_[1][1][0] * v[1] + i_[1][2][0] * v[2],
                  i_[2][0][0] * v[0] + i_[2][1][0] * v[1] + i_[2][2][0] * v[2]);
}

IVector3 IMatrix3::nonIntervalTimesVector(const IVector3& v) const
{
  return IVector3(i_[0][0][0] * v[0][0] + i_[0][1][0] * v[1][0] + i_[0][2][0] * v[2][0],
                  i_[1][0][0] * v[0][0] + i_[1][1][0] * v[1][0] + i_[1][2][0] * v[2][0],
                  i_[2][0][0] * v[0][0] + i_[2][1][0] * v[1][0] + i_[2][2][0] * v[2][0]);
}

const Interval& IMatrix3::operator () (size_t i, size_t j) const
{
  return i_[i][j];
}

Interval& IMatrix3::operator () (size_t i, size_t j)
{
  return i_[i][j];
}

void IMatrix3::print() const
{
  std::cout << "[" << i_[0][0][0] << "," << i_[0][0][1] << "]" << " [" << i_[0][1][0] << "," << i_[0][1][1] << "]" << " [" << i_[0][2][0] << "," << i_[0][2][1] << "]" << std::endl;
  std::cout << "[" << i_[1][0][0] << "," << i_[1][0][1] << "]" << " [" << i_[1][1][0] << "," << i_[1][1][1] << "]" << " [" << i_[1][2][0] << "," << i_[1][2][1] << "]" << std::endl;
  std::cout << "[" << i_[2][0][0] << "," << i_[2][0][1] << "]" << " [" << i_[2][1][0] << "," << i_[2][1][1] << "]" << " [" << i_[2][2][0] << "," << i_[2][2][1] << "]" << std::endl;
}

}

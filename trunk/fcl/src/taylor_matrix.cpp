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

#include "fcl/taylor_matrix.h"

namespace fcl
{

TMatrix3::TMatrix3() {}
TMatrix3::TMatrix3(TaylorModel m[3][3])
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


TMatrix3::TMatrix3(const Matrix3f& m)
{
  setZero();
  i_[0][0].coeffs_[0] = m(0, 0);
  i_[0][1].coeffs_[0] = m(0, 1);
  i_[0][2].coeffs_[0] = m(0, 2);

  i_[1][0].coeffs_[0] = m(1, 0);
  i_[1][1].coeffs_[0] = m(1, 1);
  i_[1][2].coeffs_[0] = m(1, 2);

  i_[2][0].coeffs_[0] = m(2, 0);
  i_[2][1].coeffs_[0] = m(2, 1);
  i_[2][2].coeffs_[0] = m(2, 2);
}

void TMatrix3::setIdentity()
{
  setZero();
  i_[0][0].coeffs_[0] = 1;
  i_[1][0].coeffs_[0] = 1;
  i_[2][2].coeffs_[0] = 1;

}

void TMatrix3::setZero()
{
  i_[0][0].setZero();
  i_[0][1].setZero();
  i_[0][2].setZero();
  i_[1][0].setZero();
  i_[1][1].setZero();
  i_[1][2].setZero();
  i_[2][0].setZero();
  i_[2][1].setZero();
  i_[2][2].setZero();
}

TVector3 TMatrix3::getColumn(size_t i) const
{
  return TVector3(i_[0][i], i_[1][i], i_[2][i]);
}

TVector3 TMatrix3::getRow(size_t i) const
{
  return TVector3(i_[i][0], i_[i][1], i_[i][2]);
}

const TaylorModel& TMatrix3::operator () (size_t i, size_t j) const
{
  return i_[i][j];
}

TaylorModel& TMatrix3::operator () (size_t i, size_t j)
{
  return i_[i][j];
}

TMatrix3 TMatrix3::operator * (const Matrix3f& m) const
{
  TaylorModel res[3][3];
  register BVH_REAL temp;

  temp = m[0][0];
  res[0][0].coeffs_[0] = i_[0][0].coeffs_[0] * temp; res[0][0].coeffs_[1] = i_[0][0].coeffs_[1] * temp; res[0][0].coeffs_[2] = i_[0][0].coeffs_[2] * temp; res[0][0].coeffs_[3] = i_[0][0].coeffs_[3] * temp; res[1][0].coeffs_[0] = i_[1][0].coeffs_[0] * temp; res[1][0].coeffs_[1] = i_[1][0].coeffs_[1] * temp; res[1][0].coeffs_[2] = i_[1][0].coeffs_[2] * temp; res[1][0].coeffs_[3] = i_[1][0].coeffs_[3] * temp; res[2][0].coeffs_[0] = i_[2][0].coeffs_[0] * temp; res[2][0].coeffs_[1] = i_[2][0].coeffs_[1] * temp; res[2][0].coeffs_[2] = i_[2][0].coeffs_[2] * temp; res[2][0].coeffs_[3] = i_[2][0].coeffs_[3] * temp;
  if(temp >= 0)
  {
    res[0][0].r_[0] = i_[0][0].r_[0] * temp; res[0][0].r_[1] = i_[0][0].r_[1] * temp; res[1][0].r_[0] = i_[1][0].r_[0] * temp; res[1][0].r_[1] = i_[1][0].r_[1] * temp; res[2][0].r_[0] = i_[2][0].r_[0] * temp; res[2][0].r_[1] = i_[2][0].r_[1] * temp;
  }
  else
  {
    res[0][0].r_[0] = i_[0][0].r_[1] * temp; res[0][0].r_[1] = i_[0][0].r_[0] * temp; res[1][0].r_[0] = i_[1][0].r_[1] * temp; res[1][0].r_[1] = i_[1][0].r_[0] * temp; res[2][0].r_[0] = i_[2][0].r_[1] * temp; res[2][0].r_[1] = i_[2][0].r_[0] * temp;
  }

  temp = m[0][1];
  res[0][1].coeffs_[0] = i_[0][0].coeffs_[0] * temp; res[0][1].coeffs_[1] = i_[0][0].coeffs_[1] * temp; res[0][1].coeffs_[2] = i_[0][0].coeffs_[2] * temp; res[0][1].coeffs_[3] = i_[0][0].coeffs_[3] * temp; res[1][1].coeffs_[0] = i_[1][0].coeffs_[0] * temp; res[1][1].coeffs_[1] = i_[1][0].coeffs_[1] * temp; res[1][1].coeffs_[2] = i_[1][0].coeffs_[2] * temp; res[1][1].coeffs_[3] = i_[1][0].coeffs_[3] * temp; res[2][1].coeffs_[0] = i_[2][0].coeffs_[0] * temp; res[2][1].coeffs_[1] = i_[2][0].coeffs_[1] * temp; res[2][1].coeffs_[2] = i_[2][0].coeffs_[2] * temp; res[2][1].coeffs_[3] = i_[2][0].coeffs_[3] * temp;
  if(temp >= 0)
  {
    res[0][1].r_[0] = i_[0][0].r_[0] * temp; res[0][1].r_[1] = i_[0][0].r_[1] * temp; res[1][1].r_[0] = i_[1][0].r_[0] * temp; res[1][1].r_[1] = i_[1][0].r_[1] * temp; res[2][1].r_[0] = i_[2][0].r_[0] * temp; res[2][1].r_[1] = i_[2][0].r_[1] * temp;
  }
  else
  {
    res[0][1].r_[0] = i_[0][0].r_[1] * temp; res[0][1].r_[1] = i_[0][0].r_[0] * temp; res[1][1].r_[0] = i_[1][0].r_[1] * temp; res[1][1].r_[1] = i_[1][0].r_[0] * temp; res[2][1].r_[0] = i_[2][0].r_[1] * temp; res[2][1].r_[1] = i_[2][0].r_[0] * temp;
  }

  temp = m[0][2];
  res[0][2].coeffs_[0] = i_[0][0].coeffs_[0] * temp; res[0][2].coeffs_[1] = i_[0][0].coeffs_[1] * temp; res[0][2].coeffs_[2] = i_[0][0].coeffs_[2] * temp; res[0][2].coeffs_[3] = i_[0][0].coeffs_[3] * temp; res[1][2].coeffs_[0] = i_[1][0].coeffs_[0] * temp; res[1][2].coeffs_[1] = i_[1][0].coeffs_[1] * temp; res[1][2].coeffs_[2] = i_[1][0].coeffs_[2] * temp; res[1][2].coeffs_[3] = i_[1][0].coeffs_[3] * temp; res[2][2].coeffs_[0] = i_[2][0].coeffs_[0] * temp; res[2][2].coeffs_[1] = i_[2][0].coeffs_[1] * temp; res[2][2].coeffs_[2] = i_[2][0].coeffs_[2] * temp; res[2][2].coeffs_[3] = i_[2][0].coeffs_[3] * temp;
  if(temp >= 0)
  {
    res[0][2].r_[0] = i_[0][0].r_[0] * temp; res[0][2].r_[1] = i_[0][0].r_[1] * temp; res[1][2].r_[0] = i_[1][0].r_[0] * temp; res[1][2].r_[1] = i_[1][0].r_[1] * temp; res[2][2].r_[0] = i_[2][0].r_[0] * temp; res[2][2].r_[1] = i_[2][0].r_[1] * temp;
  }
  else
  {
    res[0][2].r_[0] = i_[0][0].r_[1] * temp; res[0][2].r_[1] = i_[0][0].r_[0] * temp; res[1][2].r_[0] = i_[1][0].r_[1] * temp; res[1][2].r_[1] = i_[1][0].r_[0] * temp; res[2][2].r_[0] = i_[2][0].r_[1] * temp; res[2][2].r_[1] = i_[2][0].r_[0] * temp;
  }

  temp = m[1][0];
  res[0][0].coeffs_[0] += i_[0][1].coeffs_[0] * temp; res[0][0].coeffs_[1] += i_[0][1].coeffs_[1] * temp; res[0][0].coeffs_[2] += i_[0][1].coeffs_[2] * temp; res[0][0].coeffs_[3] += i_[0][1].coeffs_[3] * temp; res[1][0].coeffs_[0] += i_[1][1].coeffs_[0] * temp; res[1][0].coeffs_[1] += i_[1][1].coeffs_[1] * temp; res[1][0].coeffs_[2] += i_[1][1].coeffs_[2] * temp; res[1][0].coeffs_[3] += i_[1][1].coeffs_[3] * temp; res[2][0].coeffs_[0] += i_[2][1].coeffs_[0] * temp; res[2][0].coeffs_[1] += i_[2][1].coeffs_[1] * temp; res[2][0].coeffs_[2] += i_[2][1].coeffs_[2] * temp; res[2][0].coeffs_[3] += i_[2][1].coeffs_[3] * temp;
  if(temp >= 0)
  {
    res[0][0].r_[0] += i_[0][1].r_[0] * temp; res[0][0].r_[1] += i_[0][1].r_[1] * temp; res[1][0].r_[0] += i_[1][1].r_[0] * temp; res[1][0].r_[1] += i_[1][1].r_[1] * temp; res[2][0].r_[0] += i_[2][1].r_[0] * temp; res[2][0].r_[1] += i_[2][1].r_[1] * temp;
  }
  else
  {
    res[0][0].r_[0] += i_[0][1].r_[1] * temp; res[0][0].r_[1] += i_[0][1].r_[0] * temp; res[1][0].r_[0] += i_[1][1].r_[1] * temp; res[1][0].r_[1] += i_[1][1].r_[0] * temp; res[2][0].r_[0] += i_[2][1].r_[1] * temp; res[2][0].r_[1] += i_[2][1].r_[0] * temp;
  }

  temp = m[1][1];
  res[0][1].coeffs_[0] += i_[0][1].coeffs_[0] * temp; res[0][1].coeffs_[1] += i_[0][1].coeffs_[1] * temp; res[0][1].coeffs_[2] += i_[0][1].coeffs_[2] * temp; res[0][1].coeffs_[3] += i_[0][1].coeffs_[3] * temp; res[1][1].coeffs_[0] += i_[1][1].coeffs_[0] * temp; res[1][1].coeffs_[1] += i_[1][1].coeffs_[1] * temp; res[1][1].coeffs_[2] += i_[1][1].coeffs_[2] * temp; res[1][1].coeffs_[3] += i_[1][1].coeffs_[3] * temp; res[2][1].coeffs_[0] += i_[2][1].coeffs_[0] * temp; res[2][1].coeffs_[1] += i_[2][1].coeffs_[1] * temp; res[2][1].coeffs_[2] += i_[2][1].coeffs_[2] * temp; res[2][1].coeffs_[3] += i_[2][1].coeffs_[3] * temp;
  if(temp >= 0)
  {
    res[0][1].r_[0] += i_[0][1].r_[0] * temp; res[0][1].r_[1] += i_[0][1].r_[1] * temp; res[1][1].r_[0] += i_[1][1].r_[0] * temp; res[1][1].r_[1] += i_[1][1].r_[1] * temp; res[2][1].r_[0] += i_[2][1].r_[0] * temp; res[2][1].r_[1] += i_[2][1].r_[1] * temp;
  }
  else
  {
    res[0][1].r_[0] += i_[0][1].r_[1] * temp; res[0][1].r_[1] += i_[0][1].r_[0] * temp; res[1][1].r_[0] += i_[1][1].r_[1] * temp; res[1][1].r_[1] += i_[1][1].r_[0] * temp; res[2][1].r_[0] += i_[2][1].r_[1] * temp; res[2][1].r_[1] += i_[2][1].r_[0] * temp;
  }

  temp = m[1][2];
  res[0][2].coeffs_[0] += i_[0][1].coeffs_[0] * temp; res[0][2].coeffs_[1] += i_[0][1].coeffs_[1] * temp; res[0][2].coeffs_[2] += i_[0][1].coeffs_[2] * temp; res[0][2].coeffs_[3] += i_[0][1].coeffs_[3] * temp; res[1][2].coeffs_[0] += i_[1][1].coeffs_[0] * temp; res[1][2].coeffs_[1] += i_[1][1].coeffs_[1] * temp; res[1][2].coeffs_[2] += i_[1][1].coeffs_[2] * temp; res[1][2].coeffs_[3] += i_[1][1].coeffs_[3] * temp; res[2][2].coeffs_[0] += i_[2][1].coeffs_[0] * temp; res[2][2].coeffs_[1] += i_[2][1].coeffs_[1] * temp; res[2][2].coeffs_[2] += i_[2][1].coeffs_[2] * temp; res[2][2].coeffs_[3] += i_[2][1].coeffs_[3] * temp;
  if(temp >= 0)
  {
    res[0][2].r_[0] += i_[0][1].r_[0] * temp; res[0][2].r_[1] += i_[0][1].r_[1] * temp; res[1][2].r_[0] += i_[1][1].r_[0] * temp; res[1][2].r_[1] += i_[1][1].r_[1] * temp; res[2][2].r_[0] += i_[2][1].r_[0] * temp; res[2][2].r_[1] += i_[2][1].r_[1] * temp;
  }
  else
  {
    res[0][2].r_[0] += i_[0][1].r_[1] * temp; res[0][2].r_[1] += i_[0][1].r_[0] * temp; res[1][2].r_[0] += i_[1][1].r_[1] * temp; res[1][2].r_[1] += i_[1][1].r_[0] * temp; res[2][2].r_[0] += i_[2][1].r_[1] * temp; res[2][2].r_[1] += i_[2][1].r_[0] * temp;
  }

  temp = m[2][0];
  res[0][0].coeffs_[0] += i_[0][2].coeffs_[0] * temp; res[0][0].coeffs_[1] += i_[0][2].coeffs_[1] * temp; res[0][0].coeffs_[2] += i_[0][2].coeffs_[2] * temp; res[0][0].coeffs_[3] += i_[0][2].coeffs_[3] * temp; res[1][0].coeffs_[0] += i_[1][2].coeffs_[0] * temp; res[1][0].coeffs_[1] += i_[1][2].coeffs_[1] * temp; res[1][0].coeffs_[2] += i_[1][2].coeffs_[2] * temp; res[1][0].coeffs_[3] += i_[1][2].coeffs_[3] * temp; res[2][0].coeffs_[0] += i_[2][2].coeffs_[0] * temp; res[2][0].coeffs_[1] += i_[2][2].coeffs_[1] * temp; res[2][0].coeffs_[2] += i_[2][2].coeffs_[2] * temp; res[2][0].coeffs_[3] += i_[2][2].coeffs_[3] * temp;
  if(temp >= 0)
  {
    res[0][0].r_[0] += i_[0][2].r_[0] * temp; res[0][0].r_[1] += i_[0][2].r_[1] * temp; res[1][0].r_[0] += i_[1][2].r_[0] * temp; res[1][0].r_[1] += i_[1][2].r_[1] * temp; res[2][0].r_[0] += i_[2][2].r_[0] * temp; res[2][0].r_[1] += i_[2][2].r_[1] * temp;
  }
  else
  {
    res[0][0].r_[0] += i_[0][2].r_[1] * temp; res[0][0].r_[1] += i_[0][2].r_[0] * temp; res[1][0].r_[0] += i_[1][2].r_[1] * temp; res[1][0].r_[1] += i_[1][2].r_[0] * temp; res[2][0].r_[0] += i_[2][2].r_[1] * temp; res[2][0].r_[1] += i_[2][2].r_[0] * temp;
  }

  temp = m[2][1];
  res[0][1].coeffs_[0] += i_[0][2].coeffs_[0] * temp; res[0][1].coeffs_[1] += i_[0][2].coeffs_[1] * temp; res[0][1].coeffs_[2] += i_[0][2].coeffs_[2] * temp; res[0][1].coeffs_[3] += i_[0][2].coeffs_[3] * temp; res[1][1].coeffs_[0] += i_[1][2].coeffs_[0] * temp; res[1][1].coeffs_[1] += i_[1][2].coeffs_[1] * temp; res[1][1].coeffs_[2] += i_[1][2].coeffs_[2] * temp; res[1][1].coeffs_[3] += i_[1][2].coeffs_[3] * temp; res[2][1].coeffs_[0] += i_[2][2].coeffs_[0] * temp; res[2][1].coeffs_[1] += i_[2][2].coeffs_[1] * temp; res[2][1].coeffs_[2] += i_[2][2].coeffs_[2] * temp; res[2][1].coeffs_[3] += i_[2][2].coeffs_[3] * temp;
  if(temp >= 0)
  {
    res[0][1].r_[0] += i_[0][2].r_[0] * temp; res[0][1].r_[1] += i_[0][2].r_[1] * temp; res[1][1].r_[0] += i_[1][2].r_[0] * temp; res[1][1].r_[1] += i_[1][2].r_[1] * temp; res[2][1].r_[0] += i_[2][2].r_[0] * temp; res[2][1].r_[1] += i_[2][2].r_[1] * temp;
  }
  else
  {
    res[0][1].r_[0] += i_[0][2].r_[1] * temp; res[0][1].r_[1] += i_[0][2].r_[0] * temp; res[1][1].r_[0] += i_[1][2].r_[1] * temp; res[1][1].r_[1] += i_[1][2].r_[0] * temp; res[2][1].r_[0] += i_[2][2].r_[1] * temp; res[2][1].r_[1] += i_[2][2].r_[0] * temp;
  }

  temp = m[2][2];
  res[0][2].coeffs_[0] += i_[0][2].coeffs_[0] * temp; res[0][2].coeffs_[1] += i_[0][2].coeffs_[1] * temp; res[0][2].coeffs_[2] += i_[0][2].coeffs_[2] * temp; res[0][2].coeffs_[3] += i_[0][2].coeffs_[3] * temp; res[1][2].coeffs_[0] += i_[1][2].coeffs_[0] * temp; res[1][2].coeffs_[1] += i_[1][2].coeffs_[1] * temp; res[1][2].coeffs_[2] += i_[1][2].coeffs_[2] * temp; res[1][2].coeffs_[3] += i_[1][2].coeffs_[3] * temp; res[2][2].coeffs_[0] += i_[2][2].coeffs_[0] * temp; res[2][2].coeffs_[1] += i_[2][2].coeffs_[1] * temp; res[2][2].coeffs_[2] += i_[2][2].coeffs_[2] * temp; res[2][2].coeffs_[3] += i_[2][2].coeffs_[3] * temp;
  if(temp >= 0)
  {
    res[0][2].r_[0] += i_[0][2].r_[0] * temp; res[0][2].r_[1] += i_[0][2].r_[1] * temp; res[1][2].r_[0] += i_[1][2].r_[0] * temp; res[1][2].r_[1] += i_[1][2].r_[1] * temp; res[2][2].r_[0] += i_[2][2].r_[0] * temp; res[2][2].r_[1] += i_[2][2].r_[1] * temp;
  }
  else
  {
    res[0][2].r_[0] += i_[0][2].r_[1] * temp; res[0][2].r_[1] += i_[0][2].r_[0] * temp; res[1][2].r_[0] += i_[1][2].r_[1] * temp; res[1][2].r_[1] += i_[1][2].r_[0] * temp; res[2][2].r_[0] += i_[2][2].r_[1] * temp; res[2][2].r_[1] += i_[2][2].r_[0] * temp;
  }

  return TMatrix3(res);
}


TVector3 TMatrix3::operator * (const Vec3f& v) const
{
  TaylorModel res[3];

  register BVH_REAL temp;

  temp = v[0];
  res[0].coeffs_[0] = i_[0][0].coeffs_[0] * temp; res[0].coeffs_[1] = i_[0][0].coeffs_[1] * temp; res[0].coeffs_[2] = i_[0][0].coeffs_[2] * temp; res[0].coeffs_[3] = i_[0][0].coeffs_[3] * temp; res[1].coeffs_[0] = i_[1][0].coeffs_[0] * temp; res[1].coeffs_[1] = i_[1][0].coeffs_[1] * temp; res[1].coeffs_[2] = i_[1][0].coeffs_[2] * temp; res[1].coeffs_[3] = i_[1][0].coeffs_[3] * temp; res[2].coeffs_[0] = i_[2][0].coeffs_[0] * temp; res[2].coeffs_[1] = i_[2][0].coeffs_[1] * temp; res[2].coeffs_[2] = i_[2][0].coeffs_[2] * temp; res[2].coeffs_[3] = i_[2][0].coeffs_[3] * temp;
  if(temp >= 0)
  {
    res[0].r_[0] = i_[0][0].r_[0] * temp; res[0].r_[1] = i_[0][0].r_[1] * temp; res[1].r_[0] = i_[1][0].r_[0] * temp; res[1].r_[1] = i_[1][0].r_[1] * temp; res[2].r_[0] = i_[2][0].r_[0] * temp; res[2].r_[1] = i_[2][0].r_[1] * temp;
  }
  else
  {
    res[0].r_[0] = i_[0][0].r_[1] * temp; res[0].r_[1] = i_[0][0].r_[0] * temp; res[1].r_[0] = i_[1][0].r_[1] * temp; res[1].r_[1] = i_[1][0].r_[0] * temp; res[2].r_[0] = i_[2][0].r_[1] * temp; res[2].r_[1] = i_[2][0].r_[0] * temp;
  }

  temp = v[1];
  res[0].coeffs_[0] += i_[0][1].coeffs_[0] * temp; res[0].coeffs_[1] += i_[0][1].coeffs_[1] * temp; res[0].coeffs_[2] += i_[0][1].coeffs_[2] * temp; res[0].coeffs_[3] += i_[0][1].coeffs_[3] * temp; res[1].coeffs_[0] += i_[1][1].coeffs_[0] * temp; res[1].coeffs_[1] += i_[1][1].coeffs_[1] * temp; res[1].coeffs_[2] += i_[1][1].coeffs_[2] * temp; res[1].coeffs_[3] += i_[1][1].coeffs_[3] * temp; res[2].coeffs_[0] += i_[2][1].coeffs_[0] * temp; res[2].coeffs_[1] += i_[2][1].coeffs_[1] * temp; res[2].coeffs_[2] += i_[2][1].coeffs_[2] * temp; res[2].coeffs_[3] += i_[2][1].coeffs_[3] * temp;
  if(temp >= 0)
  {
    res[0].r_[0] += i_[0][1].r_[0] * temp; res[0].r_[1] += i_[0][1].r_[1] * temp; res[1].r_[0] += i_[1][1].r_[0] * temp; res[1].r_[1] += i_[1][1].r_[1] * temp; res[2].r_[0] += i_[2][1].r_[0] * temp; res[2].r_[1] += i_[2][1].r_[1] * temp;
  }
  else
  {
    res[0].r_[0] += i_[0][1].r_[1] * temp; res[0].r_[1] += i_[0][1].r_[0] * temp; res[1].r_[0] += i_[1][1].r_[1] * temp; res[1].r_[1] += i_[1][1].r_[0] * temp; res[2].r_[0] += i_[2][1].r_[1] * temp; res[2].r_[1] += i_[2][1].r_[0] * temp;  \
  }

  temp = v[2];
  res[0].coeffs_[0] += i_[0][2].coeffs_[0] * temp; res[0].coeffs_[1] += i_[0][2].coeffs_[1] * temp; res[0].coeffs_[2] += i_[0][2].coeffs_[2] * temp; res[0].coeffs_[3] += i_[0][2].coeffs_[3] * temp; res[1].coeffs_[0] += i_[1][2].coeffs_[0] * temp; res[1].coeffs_[1] += i_[1][2].coeffs_[1] * temp; res[1].coeffs_[2] += i_[1][2].coeffs_[2] * temp; res[1].coeffs_[3] += i_[1][2].coeffs_[3] * temp; res[2].coeffs_[0] += i_[2][2].coeffs_[0] * temp; res[2].coeffs_[1] += i_[2][2].coeffs_[1] * temp; res[2].coeffs_[2] += i_[2][2].coeffs_[2] * temp; res[2].coeffs_[3] += i_[2][2].coeffs_[3] * temp;
  if(temp >= 0)
  {
    res[0].r_[0] += i_[0][2].r_[0] * temp; res[0].r_[1] += i_[0][2].r_[1] * temp; res[1].r_[0] += i_[1][2].r_[0] * temp; res[1].r_[1] += i_[1][2].r_[1] * temp; res[2].r_[0] += i_[2][2].r_[0] * temp; res[2].r_[1] += i_[2][2].r_[1] * temp;
  }
  else
  {
    res[0].r_[0] += i_[0][2].r_[1] * temp; res[0].r_[1] += i_[0][2].r_[0] * temp; res[1].r_[0] += i_[1][2].r_[1] * temp; res[1].r_[1] += i_[1][2].r_[0] * temp; res[2].r_[0] += i_[2][2].r_[1] * temp; res[2].r_[1] += i_[2][2].r_[0] * temp;
  }

  return TVector3(res);
}

TVector3 TMatrix3::operator * (const TVector3& v) const
{
  TaylorModel res[3];

  res[0] = i_[0][0] * v[0] + i_[0][1] * v[1] + i_[0][2] * v[2];
  res[1] = i_[1][0] * v[0] + i_[1][1] * v[1] + i_[1][2] * v[2];
  res[2] = i_[2][0] * v[0] + i_[2][1] * v[1] + i_[2][2] * v[2];

  return TVector3(res);
}

TMatrix3 TMatrix3::operator * (const TMatrix3& m) const
{
  TaylorModel res[3][3];

  res[0][0] = i_[0][0] * m.i_[0][0] + i_[0][1] * m.i_[1][0] + i_[0][2] * m.i_[2][0];
  res[0][1] = i_[0][0] * m.i_[0][1] + i_[0][1] * m.i_[1][1] + i_[0][2] * m.i_[2][1];
  res[0][2] = i_[0][0] * m.i_[0][2] + i_[0][1] * m.i_[1][2] + i_[0][2] * m.i_[2][2];

  res[1][0] = i_[1][0] * m.i_[0][0] + i_[1][1] * m.i_[1][0] + i_[1][2] * m.i_[2][0];
  res[1][1] = i_[1][0] * m.i_[0][1] + i_[1][1] * m.i_[1][1] + i_[1][2] * m.i_[2][1];
  res[1][2] = i_[1][0] * m.i_[0][2] + i_[1][1] * m.i_[1][2] + i_[1][2] * m.i_[2][2];

  res[2][0] = i_[2][0] * m.i_[0][0] + i_[2][1] * m.i_[1][0] + i_[2][2] * m.i_[2][0];
  res[2][1] = i_[2][0] * m.i_[0][1] + i_[2][1] * m.i_[1][1] + i_[2][2] * m.i_[2][1];
  res[2][2] = i_[2][0] * m.i_[0][2] + i_[2][1] * m.i_[1][2] + i_[2][2] * m.i_[2][2];

  return TMatrix3(res);
}

TMatrix3 TMatrix3::operator + (const TMatrix3& m) const
{
  TaylorModel res[3][3];

  res[0][0] = i_[0][0] + m.i_[0][0];
  res[0][1] = i_[0][1] + m.i_[0][1];
  res[0][2] = i_[0][2] + m.i_[0][2];

  res[1][0] = i_[1][0] + m.i_[1][0];
  res[1][1] = i_[1][1] + m.i_[1][1];
  res[1][2] = i_[1][2] + m.i_[1][2];

  res[2][0] = i_[2][0] + m.i_[2][0];
  res[2][1] = i_[2][1] + m.i_[2][1];
  res[2][2] = i_[2][2] + m.i_[2][2];

  return TMatrix3(res);
}

TMatrix3& TMatrix3::operator += (const TMatrix3& m)
{
  i_[0][0] += m.i_[0][0];
  i_[0][1] += m.i_[0][1];
  i_[0][2] += m.i_[0][2];

  i_[1][0] += m.i_[1][0];
  i_[1][1] += m.i_[1][1];
  i_[1][2] += m.i_[1][2];

  i_[2][0] += m.i_[2][0];
  i_[2][1] += m.i_[2][1];
  i_[2][2] += m.i_[2][2];

  return *this;
}

void TMatrix3::print() const
{
  getColumn(0).print();
  getColumn(1).print();
  getColumn(2).print();
}

IMatrix3 TMatrix3::getBound() const
{
  Interval res[3][3];

  res[0][0] = i_[0][0].getBound();
  res[0][1] = i_[0][1].getBound();
  res[0][2] = i_[0][2].getBound();

  res[1][0] = i_[1][0].getBound();
  res[1][1] = i_[1][1].getBound();
  res[1][2] = i_[1][2].getBound();

  res[2][0] = i_[2][0].getBound();
  res[2][1] = i_[2][1].getBound();
  res[2][2] = i_[2][2].getBound();

  return IMatrix3(res);
}

BVH_REAL TMatrix3::diameter() const
{
  BVH_REAL d = 0;


  BVH_REAL tmp = i_[0][0].r_.diameter();
  if(tmp > d) d = tmp;
  tmp = i_[0][1].r_.diameter();
  if(tmp > d) d = tmp;
  tmp = i_[0][2].r_.diameter();
  if(tmp > d) d = tmp;

  tmp = i_[1][0].r_.diameter();
  if(tmp > d) d = tmp;
  tmp = i_[1][1].r_.diameter();
  if(tmp > d) d = tmp;
  tmp = i_[1][2].r_.diameter();
  if(tmp > d) d = tmp;

  tmp = i_[2][0].r_.diameter();
  if(tmp > d) d = tmp;
  tmp = i_[2][1].r_.diameter();
  if(tmp > d) d = tmp;
  tmp = i_[2][2].r_.diameter();
  if(tmp > d) d = tmp;

  return d;
}

}

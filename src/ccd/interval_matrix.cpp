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

#include "fcl/ccd/interval_matrix.h"
#include <iostream>

namespace fcl
{

IMatrix3::IMatrix3() {}

IMatrix3::IMatrix3(FCL_REAL v)
{
  v_[0].setValue(v);
  v_[1].setValue(v);
  v_[2].setValue(v);
}

IMatrix3::IMatrix3(const Matrix3f& m)
{
  v_[0] = m.getRow(0);
  v_[1] = m.getRow(1);
  v_[2] = m.getRow(2);
}

IMatrix3::IMatrix3(FCL_REAL m[3][3][2])
{
  v_[0].setValue(m[0]);
  v_[1].setValue(m[1]);
  v_[2].setValue(m[2]);
}

IMatrix3::IMatrix3(FCL_REAL m[3][3])
{
  v_[0].setValue(m[0]);
  v_[1].setValue(m[1]);
  v_[2].setValue(m[2]);
}

IMatrix3::IMatrix3(Interval m[3][3])
{
  v_[0].setValue(m[0]);
  v_[1].setValue(m[1]);
  v_[2].setValue(m[2]);
}

IMatrix3::IMatrix3(const IVector3& v1, const IVector3& v2, const IVector3& v3)
{
  v_[0] = v1;
  v_[1] = v2;
  v_[2] = v3;
}

void IMatrix3::setIdentity()
{
  v_[0].setValue(1, 0, 0);
  v_[1].setValue(0, 1, 0);
  v_[2].setValue(0, 0, 1);
}

IVector3 IMatrix3::getColumn(size_t i) const
{
  return IVector3(v_[0][i], v_[1][i], v_[2][i]);
}

const IVector3& IMatrix3::getRow(size_t i) const
{
  return v_[i];
}

Vec3f IMatrix3::getColumnLow(size_t i) const
{
  return Vec3f(v_[0][i][0], v_[1][i][0], v_[2][i][0]);
}

Vec3f IMatrix3::getRowLow(size_t i) const
{
  return Vec3f(v_[i][0][0], v_[i][1][0], v_[i][2][0]);
}

Vec3f IMatrix3::getColumnHigh(size_t i) const
{
  return Vec3f(v_[0][i][1], v_[1][i][1], v_[2][i][1]);
}

Vec3f IMatrix3::getRowHigh(size_t i) const
{
  return Vec3f(v_[i][0][1], v_[i][1][1], v_[i][2][1]);
}

Matrix3f IMatrix3::getLow() const
{
  return Matrix3f(v_[0][0][0], v_[0][1][0], v_[0][2][0],
                  v_[1][0][0], v_[1][1][0], v_[1][2][0],
                  v_[2][0][0], v_[2][1][0], v_[2][2][0]);
}

Matrix3f IMatrix3::getHigh() const
{
  return Matrix3f(v_[0][0][1], v_[0][1][1], v_[0][2][1],
                  v_[1][0][1], v_[1][1][1], v_[1][2][1],
                  v_[2][0][1], v_[2][1][1], v_[2][2][1]);
}

IMatrix3 IMatrix3::operator * (const Matrix3f& m) const
{
  const Vec3f& mc0 = m.getColumn(0);
  const Vec3f& mc1 = m.getColumn(1);
  const Vec3f& mc2 = m.getColumn(2);

  return IMatrix3(IVector3(v_[0].dot(mc0), v_[0].dot(mc1), v_[0].dot(mc2)),
                  IVector3(v_[1].dot(mc0), v_[1].dot(mc1), v_[1].dot(mc2)),
                  IVector3(v_[2].dot(mc0), v_[2].dot(mc1), v_[2].dot(mc2)));
}

IVector3 IMatrix3::operator * (const Vec3f& v) const
{
  return IVector3(v_[0].dot(v), v_[1].dot(v), v_[2].dot(v));
}

IVector3 IMatrix3::operator * (const IVector3& v) const
{
  return IVector3(v_[0].dot(v), v_[1].dot(v), v_[2].dot(v));
}

IMatrix3 IMatrix3::operator * (const IMatrix3& m) const
{
  const IVector3& mc0 = m.getColumn(0);
  const IVector3& mc1 = m.getColumn(1);
  const IVector3& mc2 = m.getColumn(2);

  return IMatrix3(IVector3(v_[0].dot(mc0), v_[0].dot(mc1), v_[0].dot(mc2)),
                  IVector3(v_[1].dot(mc0), v_[1].dot(mc1), v_[1].dot(mc2)),
                  IVector3(v_[2].dot(mc0), v_[2].dot(mc1), v_[2].dot(mc2)));
}

IMatrix3& IMatrix3::operator *= (const Matrix3f& m)
{
  const Vec3f& mc0 = m.getColumn(0);
  const Vec3f& mc1 = m.getColumn(1);
  const Vec3f& mc2 = m.getColumn(2);

  v_[0].setValue(v_[0].dot(mc0), v_[0].dot(mc1), v_[0].dot(mc2));
  v_[1].setValue(v_[1].dot(mc0), v_[1].dot(mc1), v_[1].dot(mc2));
  v_[2].setValue(v_[2].dot(mc0), v_[2].dot(mc1), v_[2].dot(mc2));
  return *this;
}


IMatrix3& IMatrix3::operator *= (const IMatrix3& m)
{
  const IVector3& mc0 = m.getColumn(0);
  const IVector3& mc1 = m.getColumn(1);
  const IVector3& mc2 = m.getColumn(2);

  v_[0].setValue(v_[0].dot(mc0), v_[0].dot(mc1), v_[0].dot(mc2));
  v_[1].setValue(v_[1].dot(mc0), v_[1].dot(mc1), v_[1].dot(mc2));
  v_[2].setValue(v_[2].dot(mc0), v_[2].dot(mc1), v_[2].dot(mc2));
  return *this;
}

IMatrix3 IMatrix3::operator + (const IMatrix3& m) const
{
  return IMatrix3(v_[0] + m.v_[0], v_[1] + m.v_[1], v_[2] + m.v_[2]);
}

IMatrix3& IMatrix3::operator += (const IMatrix3& m)
{
  v_[0] += m.v_[0];
  v_[1] += m.v_[1];
  v_[2] += m.v_[2];
  return *this;
}

IMatrix3 IMatrix3::operator - (const IMatrix3& m) const
{
  return IMatrix3(v_[0] - m.v_[0], v_[1] - m.v_[1], v_[2] - m.v_[2]);
}

IMatrix3& IMatrix3::operator -= (const IMatrix3& m)
{
  v_[0] -= m.v_[0];
  v_[1] -= m.v_[1];
  v_[2] -= m.v_[2];
  return *this;
}

IMatrix3& IMatrix3::rotationConstrain()
{
  for(std::size_t i = 0; i < 3; ++i)
  {
    for(std::size_t j = 0; j < 3; ++j)
    {
      if(v_[i][j][0] < -1) v_[i][j][0] = -1;
      else if(v_[i][j][0] > 1) v_[i][j][0] = 1;

      if(v_[i][j][1] < -1) v_[i][j][1] = -1;
      else if(v_[i][j][1] > 1) v_[i][j][1] = 1;
    }
  }

  return *this;
}

void IMatrix3::print() const
{
  std::cout << "[" << v_[0][0][0] << "," << v_[0][0][1] << "]" << " [" << v_[0][1][0] << "," << v_[0][1][1] << "]" << " [" << v_[0][2][0] << "," << v_[0][2][1] << "]" << std::endl;
  std::cout << "[" << v_[1][0][0] << "," << v_[1][0][1] << "]" << " [" << v_[1][1][0] << "," << v_[1][1][1] << "]" << " [" << v_[1][2][0] << "," << v_[1][2][1] << "]" << std::endl;
  std::cout << "[" << v_[2][0][0] << "," << v_[2][0][1] << "]" << " [" << v_[2][1][0] << "," << v_[2][1][1] << "]" << " [" << v_[2][2][0] << "," << v_[2][2][1] << "]" << std::endl;
}

IMatrix3 rotationConstrain(const IMatrix3& m)
{
  IMatrix3 res;
  for(std::size_t i = 0; i < 3; ++i)
  {
    for(std::size_t j = 0; j < 3; ++j)
    {
      if(m(i, j)[0] < -1) res(i, j)[0] = -1;
      else if(m(i, j)[0] > 1) res(i, j)[0] = 1;

      if(m(i, j)[1] < -1) res(i, j)[1] = -1;
      else if(m(i, j)[1] > 1) res(i, j)[1] = 1;      
    }
  }

  return res;
}

}

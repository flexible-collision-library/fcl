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

#include "fcl/ccd/taylor_matrix.h"

namespace fcl
{

TMatrix3::TMatrix3()
{
}

TMatrix3::TMatrix3(const std::shared_ptr<TimeInterval>& time_interval)
{
  setTimeInterval(time_interval);
}

TMatrix3::TMatrix3(TaylorModel m[3][3])
{
  v_[0] = TVector3(m[0]);
  v_[1] = TVector3(m[1]);
  v_[2] = TVector3(m[2]);
}

TMatrix3::TMatrix3(const TVector3& v1, const TVector3& v2, const TVector3& v3)
{
  v_[0] = v1;
  v_[1] = v2;
  v_[2] = v3;
}


TMatrix3::TMatrix3(const Matrix3f& m, const std::shared_ptr<TimeInterval>& time_interval)
{
  v_[0] = TVector3(m.getRow(0), time_interval);
  v_[1] = TVector3(m.getRow(1), time_interval);
  v_[2] = TVector3(m.getRow(2), time_interval);
}

void TMatrix3::setIdentity()
{
  setZero();
  v_[0][0].coeff(0) = 1;
  v_[1][1].coeff(0) = 1;
  v_[2][2].coeff(0) = 1;

}

void TMatrix3::setZero()
{
  v_[0].setZero();
  v_[1].setZero();
  v_[2].setZero();
}

TVector3 TMatrix3::getColumn(size_t i) const
{
  return TVector3(v_[0][i], v_[1][i], v_[2][i]);
}

const TVector3& TMatrix3::getRow(size_t i) const
{
  return v_[i];
}

const TaylorModel& TMatrix3::operator () (size_t i, size_t j) const
{
  return v_[i][j];
}

TaylorModel& TMatrix3::operator () (size_t i, size_t j)
{
  return v_[i][j];
}

TMatrix3 TMatrix3::operator * (const Matrix3f& m) const
{
  const Vec3f& mc0 = m.getColumn(0);
  const Vec3f& mc1 = m.getColumn(1);
  const Vec3f& mc2 = m.getColumn(2);

  return TMatrix3(TVector3(v_[0].dot(mc0), v_[0].dot(mc1), v_[0].dot(mc2)),
                  TVector3(v_[1].dot(mc0), v_[1].dot(mc1), v_[1].dot(mc2)),
                  TVector3(v_[2].dot(mc0), v_[2].dot(mc1), v_[2].dot(mc2)));
}


TVector3 TMatrix3::operator * (const Vec3f& v) const
{
  return TVector3(v_[0].dot(v), v_[1].dot(v), v_[2].dot(v));
}

TVector3 TMatrix3::operator * (const TVector3& v) const
{
  return TVector3(v_[0].dot(v), v_[1].dot(v), v_[2].dot(v));
}

TMatrix3 TMatrix3::operator * (const TMatrix3& m) const
{
  const TVector3& mc0 = m.getColumn(0);
  const TVector3& mc1 = m.getColumn(1);
  const TVector3& mc2 = m.getColumn(2);

  return TMatrix3(TVector3(v_[0].dot(mc0), v_[0].dot(mc1), v_[0].dot(mc2)),
                  TVector3(v_[1].dot(mc0), v_[1].dot(mc1), v_[1].dot(mc2)),
                  TVector3(v_[2].dot(mc0), v_[2].dot(mc1), v_[2].dot(mc2)));
}

TMatrix3 TMatrix3::operator * (const TaylorModel& d) const
{
  return TMatrix3(v_[0] * d, v_[1] * d, v_[2] * d);
}

TMatrix3 TMatrix3::operator * (FCL_REAL d) const
{
  return TMatrix3(v_[0] * d, v_[1] * d, v_[2] * d);
}


TMatrix3& TMatrix3::operator *= (const Matrix3f& m)
{
  const Vec3f& mc0 = m.getColumn(0);
  const Vec3f& mc1 = m.getColumn(1);
  const Vec3f& mc2 = m.getColumn(2);
  
  v_[0] = TVector3(v_[0].dot(mc0), v_[0].dot(mc1), v_[0].dot(mc2));
  v_[1] = TVector3(v_[1].dot(mc0), v_[1].dot(mc1), v_[1].dot(mc2));
  v_[2] = TVector3(v_[2].dot(mc0), v_[2].dot(mc1), v_[2].dot(mc2));
  return *this;
}

TMatrix3& TMatrix3::operator *= (const TMatrix3& m)
{
  const TVector3& mc0 = m.getColumn(0);
  const TVector3& mc1 = m.getColumn(1);
  const TVector3& mc2 = m.getColumn(2);
  
  v_[0] = TVector3(v_[0].dot(mc0), v_[0].dot(mc1), v_[0].dot(mc2));
  v_[1] = TVector3(v_[1].dot(mc0), v_[1].dot(mc1), v_[1].dot(mc2));
  v_[2] = TVector3(v_[2].dot(mc0), v_[2].dot(mc1), v_[2].dot(mc2));
  return *this;
}

TMatrix3& TMatrix3::operator *= (const TaylorModel& d)
{
  v_[0] *= d;
  v_[1] *= d;
  v_[2] *= d;
  return *this;
}

TMatrix3& TMatrix3::operator *= (FCL_REAL d)
{
  v_[0] *= d;
  v_[1] *= d;
  v_[2] *= d;
  return *this;
}

TMatrix3 TMatrix3::operator + (const TMatrix3& m) const
{
  return TMatrix3(v_[0] + m.v_[0], v_[1] + m.v_[1], v_[2] + m.v_[2]);
}

TMatrix3& TMatrix3::operator += (const TMatrix3& m)
{
  v_[0] += m.v_[0];
  v_[1] += m.v_[1];
  v_[2] += m.v_[2];
  return *this;
}

TMatrix3 TMatrix3::operator + (const Matrix3f& m) const
{
  TMatrix3 res = *this;
  res += m;
  return res;
}

TMatrix3& TMatrix3::operator += (const Matrix3f& m)
{
  for(std::size_t i = 0; i < 3; ++i)
  {
    for(std::size_t j = 0; j < 3; ++j)
      v_[i][j] += m(i, j);
  }

  return *this;
}

TMatrix3 TMatrix3::operator - (const TMatrix3& m) const
{
  return TMatrix3(v_[0] - m.v_[0], v_[1] - m.v_[1], v_[2] - m.v_[2]);
}

TMatrix3& TMatrix3::operator -= (const TMatrix3& m)
{
  v_[0] -= m.v_[0];
  v_[1] -= m.v_[1];
  v_[2] -= m.v_[2];
  return *this;
}

TMatrix3 TMatrix3::operator - (const Matrix3f& m) const
{
  TMatrix3 res = *this;
  res -= m;
  return res;
}

TMatrix3& TMatrix3::operator -= (const Matrix3f& m)
{
  for(std::size_t i = 0; i < 3; ++i)
  {
    for(std::size_t j = 0; j < 3; ++j)
      v_[i][j] -= m(i, j);
  }

  return *this;
}

TMatrix3 TMatrix3::operator - () const
{
  return TMatrix3(-v_[0], -v_[1], -v_[2]);
}


void TMatrix3::print() const
{
  getColumn(0).print();
  getColumn(1).print();
  getColumn(2).print();
}

IMatrix3 TMatrix3::getBound() const
{
  return IMatrix3(v_[0].getBound(), v_[1].getBound(), v_[2].getBound());
}

IMatrix3 TMatrix3::getBound(FCL_REAL l, FCL_REAL r) const
{
  return IMatrix3(v_[0].getBound(l, r), v_[1].getBound(l, r), v_[2].getBound(l, r));
}

IMatrix3 TMatrix3::getBound(FCL_REAL t) const
{
  return IMatrix3(v_[0].getBound(t), v_[1].getBound(t), v_[2].getBound(t));
}

IMatrix3 TMatrix3::getTightBound() const
{
  return IMatrix3(v_[0].getTightBound(), v_[1].getTightBound(), v_[2].getTightBound());
}

IMatrix3 TMatrix3::getTightBound(FCL_REAL l, FCL_REAL r) const
{
  return IMatrix3(v_[0].getTightBound(l, r), v_[1].getTightBound(l, r), v_[2].getTightBound(l, r));
}


FCL_REAL TMatrix3::diameter() const
{
  FCL_REAL d = 0;

  FCL_REAL tmp = v_[0][0].remainder().diameter();
  if(tmp > d) d = tmp;
  tmp = v_[0][1].remainder().diameter();
  if(tmp > d) d = tmp;
  tmp = v_[0][2].remainder().diameter();
  if(tmp > d) d = tmp;

  tmp = v_[1][0].remainder().diameter();
  if(tmp > d) d = tmp;
  tmp = v_[1][1].remainder().diameter();
  if(tmp > d) d = tmp;
  tmp = v_[1][2].remainder().diameter();
  if(tmp > d) d = tmp;

  tmp = v_[2][0].remainder().diameter();
  if(tmp > d) d = tmp;
  tmp = v_[2][1].remainder().diameter();
  if(tmp > d) d = tmp;
  tmp = v_[2][2].remainder().diameter();
  if(tmp > d) d = tmp;

  return d;
}

void TMatrix3::setTimeInterval(const std::shared_ptr<TimeInterval>& time_interval)
{
  v_[0].setTimeInterval(time_interval);
  v_[1].setTimeInterval(time_interval);
  v_[2].setTimeInterval(time_interval);
}

void TMatrix3::setTimeInterval(FCL_REAL l, FCL_REAL r)
{
  v_[0].setTimeInterval(l, r);
  v_[1].setTimeInterval(l, r);
  v_[2].setTimeInterval(l, r);
}

const std::shared_ptr<TimeInterval>& TMatrix3::getTimeInterval() const
{
  return v_[0].getTimeInterval();
}

TMatrix3& TMatrix3::rotationConstrain()
{
  for(std::size_t i = 0; i < 3; ++i)
  {
    for(std::size_t j = 0; j < 3; ++j)
    {
      if(v_[i][j].remainder()[0] < -1) v_[i][j].remainder()[0] = -1;
      else if(v_[i][j].remainder()[0] > 1) v_[i][j].remainder()[0] = 1;

      if(v_[i][j].remainder()[1] < -1) v_[i][j].remainder()[1] = -1;
      else if(v_[i][j].remainder()[1] > 1) v_[i][j].remainder()[1] = 1;

      if((v_[i][j].remainder()[0] == -1) && (v_[i][j].remainder()[1] == 1))
      {
        v_[i][j].coeff(0) = 0;
        v_[i][j].coeff(1) = 0;
        v_[i][j].coeff(2) = 0;
        v_[i][j].coeff(3) = 0;
      }
    }
  }

  return *this;
}


TMatrix3 rotationConstrain(const TMatrix3& m)
{
  TMatrix3 res;

  for(std::size_t i = 0; i < 3; ++i)
  {
    for(std::size_t j = 0; j < 3; ++j)
    {
      if(m(i, j).remainder()[0] < -1) res(i, j).remainder()[0] = -1;
      else if(m(i, j).remainder()[0] > 1) res(i, j).remainder()[0] = 1;

      if(m(i, j).remainder()[1] < -1) res(i, j).remainder()[1] = -1;
      else if(m(i, j).remainder()[1] > 1) res(i, j).remainder()[1] = 1;

      if((m(i, j).remainder()[0] == -1) && (m(i, j).remainder()[1] == 1))
      {
        res(i, j).coeff(0) = 0;
        res(i, j).coeff(1) = 0;
        res(i, j).coeff(2) = 0;
        res(i, j).coeff(3) = 0;
      }
    }
  }

  return res;
}

TMatrix3 operator * (const Matrix3f& m, const TaylorModel& a)
{
  TMatrix3 res(a.getTimeInterval());
  res(0, 0) = a * m(0, 0);
  res(0, 1) = a * m(0, 1);
  res(0, 1) = a * m(0, 2);

  res(1, 0) = a * m(1, 0);
  res(1, 1) = a * m(1, 1);
  res(1, 1) = a * m(1, 2);

  res(2, 0) = a * m(2, 0);
  res(2, 1) = a * m(2, 1);
  res(2, 1) = a * m(2, 2);

  return res;
}

TMatrix3 operator * (const TaylorModel& a, const Matrix3f& m)
{
  return m * a;
}

TMatrix3 operator * (const TaylorModel& a, const TMatrix3& m)
{
  return m * a;
}

TMatrix3 operator * (FCL_REAL d, const TMatrix3& m)
{
  return m * d;
}

TMatrix3 operator + (const Matrix3f& m1, const TMatrix3& m2)
{
  return m2 + m1;
}

TMatrix3 operator - (const Matrix3f& m1, const TMatrix3& m2)
{
  return -m2 + m1;
}


}

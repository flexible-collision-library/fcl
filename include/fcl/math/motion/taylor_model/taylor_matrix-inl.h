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

#ifndef FCL_CCD_TAYLOR_MATRIX_INL_H
#define FCL_CCD_TAYLOR_MATRIX_INL_H

#include "fcl/math/motion/taylor_model/taylor_matrix.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT TMatrix3<double>;

//==============================================================================
extern template
TMatrix3<double> rotationConstrain(const TMatrix3<double>& m);

//==============================================================================
extern template
TMatrix3<double> operator * (const Matrix3<double>& m, const TaylorModel<double>& a);

//==============================================================================
extern template
TMatrix3<double> operator * (const TaylorModel<double>& a, const Matrix3<double>& m);

//==============================================================================
extern template
TMatrix3<double> operator * (const TaylorModel<double>& a, const TMatrix3<double>& m);

//==============================================================================
extern template
TMatrix3<double> operator * (double d, const TMatrix3<double>& m);

//==============================================================================
extern template
TMatrix3<double> operator + (const Matrix3<double>& m1, const TMatrix3<double>& m2);

//==============================================================================
extern template
TMatrix3<double> operator - (const Matrix3<double>& m1, const TMatrix3<double>& m2);

//==============================================================================
template <typename S>
TMatrix3<S>::TMatrix3()
{
  // Do nothing
}

//==============================================================================
template <typename S>
TMatrix3<S>::TMatrix3(const std::shared_ptr<TimeInterval<S>>& time_interval)
{
  setTimeInterval(time_interval);
}

//==============================================================================
template <typename S>
TMatrix3<S>::TMatrix3(TaylorModel<S> m[3][3])
{
  v_[0] = TVector3<S>(m[0]);
  v_[1] = TVector3<S>(m[1]);
  v_[2] = TVector3<S>(m[2]);
}

//==============================================================================
template <typename S>
TMatrix3<S>::TMatrix3(const TVector3<S>& v1, const TVector3<S>& v2, const TVector3<S>& v3)
{
  v_[0] = v1;
  v_[1] = v2;
  v_[2] = v3;
}

//==============================================================================
template <typename S>
TMatrix3<S>::TMatrix3(const Matrix3<S>& m, const std::shared_ptr<TimeInterval<S>>& time_interval)
{
  v_[0] = TVector3<S>(m.row(0), time_interval);
  v_[1] = TVector3<S>(m.row(1), time_interval);
  v_[2] = TVector3<S>(m.row(2), time_interval);
}

//==============================================================================
template <typename S>
void TMatrix3<S>::setIdentity()
{
  setZero();
  v_[0][0].coeff(0) = 1;
  v_[1][1].coeff(0) = 1;
  v_[2][2].coeff(0) = 1;

}

//==============================================================================
template <typename S>
void TMatrix3<S>::setZero()
{
  v_[0].setZero();
  v_[1].setZero();
  v_[2].setZero();
}

//==============================================================================
template <typename S>
TVector3<S> TMatrix3<S>::getColumn(size_t i) const
{
  return TVector3<S>(v_[0][i], v_[1][i], v_[2][i]);
}

//==============================================================================
template <typename S>
const TVector3<S>& TMatrix3<S>::getRow(size_t i) const
{
  return v_[i];
}

//==============================================================================
template <typename S>
const TaylorModel<S>& TMatrix3<S>::operator () (size_t i, size_t j) const
{
  return v_[i][j];
}

//==============================================================================
template <typename S>
TaylorModel<S>& TMatrix3<S>::operator () (size_t i, size_t j)
{
  return v_[i][j];
}

//==============================================================================
template <typename S>
TMatrix3<S> TMatrix3<S>::operator * (const Matrix3<S>& m) const
{
  const Vector3<S>& mc0 = m.col(0);
  const Vector3<S>& mc1 = m.col(1);
  const Vector3<S>& mc2 = m.col(2);

  return TMatrix3(TVector3<S>(v_[0].dot(mc0), v_[0].dot(mc1), v_[0].dot(mc2)),
                  TVector3<S>(v_[1].dot(mc0), v_[1].dot(mc1), v_[1].dot(mc2)),
                  TVector3<S>(v_[2].dot(mc0), v_[2].dot(mc1), v_[2].dot(mc2)));
}

//==============================================================================
template <typename S>
TVector3<S> TMatrix3<S>::operator * (const Vector3<S>& v) const
{
  return TVector3<S>(v_[0].dot(v), v_[1].dot(v), v_[2].dot(v));
}

//==============================================================================
template <typename S>
TVector3<S> TMatrix3<S>::operator * (const TVector3<S>& v) const
{
  return TVector3<S>(v_[0].dot(v), v_[1].dot(v), v_[2].dot(v));
}

//==============================================================================
template <typename S>
TMatrix3<S> TMatrix3<S>::operator * (const TMatrix3<S>& m) const
{
  const TVector3<S>& mc0 = m.getColumn(0);
  const TVector3<S>& mc1 = m.getColumn(1);
  const TVector3<S>& mc2 = m.getColumn(2);

  return TMatrix3(TVector3<S>(v_[0].dot(mc0), v_[0].dot(mc1), v_[0].dot(mc2)),
                  TVector3<S>(v_[1].dot(mc0), v_[1].dot(mc1), v_[1].dot(mc2)),
                  TVector3<S>(v_[2].dot(mc0), v_[2].dot(mc1), v_[2].dot(mc2)));
}

//==============================================================================
template <typename S>
TMatrix3<S> TMatrix3<S>::operator * (const TaylorModel<S>& d) const
{
  return TMatrix3(v_[0] * d, v_[1] * d, v_[2] * d);
}

//==============================================================================
template <typename S>
TMatrix3<S> TMatrix3<S>::operator * (S d) const
{
  return TMatrix3(v_[0] * d, v_[1] * d, v_[2] * d);
}

//==============================================================================
template <typename S>
TMatrix3<S>& TMatrix3<S>::operator *= (const Matrix3<S>& m)
{
  const Vector3<S>& mc0 = m.col(0);
  const Vector3<S>& mc1 = m.col(1);
  const Vector3<S>& mc2 = m.col(2);

  v_[0] = TVector3<S>(v_[0].dot(mc0), v_[0].dot(mc1), v_[0].dot(mc2));
  v_[1] = TVector3<S>(v_[1].dot(mc0), v_[1].dot(mc1), v_[1].dot(mc2));
  v_[2] = TVector3<S>(v_[2].dot(mc0), v_[2].dot(mc1), v_[2].dot(mc2));
  return *this;
}

//==============================================================================
template <typename S>
TMatrix3<S>& TMatrix3<S>::operator *= (const TMatrix3<S>& m)
{
  const TVector3<S>& mc0 = m.getColumn(0);
  const TVector3<S>& mc1 = m.getColumn(1);
  const TVector3<S>& mc2 = m.getColumn(2);

  v_[0] = TVector3<S>(v_[0].dot(mc0), v_[0].dot(mc1), v_[0].dot(mc2));
  v_[1] = TVector3<S>(v_[1].dot(mc0), v_[1].dot(mc1), v_[1].dot(mc2));
  v_[2] = TVector3<S>(v_[2].dot(mc0), v_[2].dot(mc1), v_[2].dot(mc2));

  return *this;
}

//==============================================================================
template <typename S>
TMatrix3<S>& TMatrix3<S>::operator *= (const TaylorModel<S>& d)
{
  v_[0] *= d;
  v_[1] *= d;
  v_[2] *= d;
  return *this;
}

//==============================================================================
template <typename S>
TMatrix3<S>& TMatrix3<S>::operator *= (S d)
{
  v_[0] *= d;
  v_[1] *= d;
  v_[2] *= d;
  return *this;
}

//==============================================================================
template <typename S>
TMatrix3<S> TMatrix3<S>::operator + (const TMatrix3<S>& m) const
{
  return TMatrix3(v_[0] + m.v_[0], v_[1] + m.v_[1], v_[2] + m.v_[2]);
}

//==============================================================================
template <typename S>
TMatrix3<S>& TMatrix3<S>::operator += (const TMatrix3<S>& m)
{
  v_[0] += m.v_[0];
  v_[1] += m.v_[1];
  v_[2] += m.v_[2];
  return *this;
}

//==============================================================================
template <typename S>
TMatrix3<S> TMatrix3<S>::operator + (const Matrix3<S>& m) const
{
  TMatrix3 res = *this;
  res += m;
  return res;
}

//==============================================================================
template <typename S>
TMatrix3<S>& TMatrix3<S>::operator += (const Matrix3<S>& m)
{
  for(std::size_t i = 0; i < 3; ++i)
  {
    for(std::size_t j = 0; j < 3; ++j)
      v_[i][j] += m(i, j);
  }

  return *this;
}

//==============================================================================
template <typename S>
TMatrix3<S> TMatrix3<S>::operator - (const TMatrix3<S>& m) const
{
  return TMatrix3(v_[0] - m.v_[0], v_[1] - m.v_[1], v_[2] - m.v_[2]);
}

//==============================================================================
template <typename S>
TMatrix3<S>& TMatrix3<S>::operator -= (const TMatrix3<S>& m)
{
  v_[0] -= m.v_[0];
  v_[1] -= m.v_[1];
  v_[2] -= m.v_[2];
  return *this;
}

//==============================================================================
template <typename S>
TMatrix3<S> TMatrix3<S>::operator - (const Matrix3<S>& m) const
{
  TMatrix3 res = *this;
  res -= m;
  return res;
}

//==============================================================================
template <typename S>
TMatrix3<S>& TMatrix3<S>::operator -= (const Matrix3<S>& m)
{
  for(std::size_t i = 0; i < 3; ++i)
  {
    for(std::size_t j = 0; j < 3; ++j)
      v_[i][j] -= m(i, j);
  }

  return *this;
}

//==============================================================================
template <typename S>
TMatrix3<S> TMatrix3<S>::operator - () const
{
  return TMatrix3<S>(-v_[0], -v_[1], -v_[2]);
}

//==============================================================================
template <typename S>
void TMatrix3<S>::print() const
{
  getColumn(0).print();
  getColumn(1).print();
  getColumn(2).print();
}

//==============================================================================
template <typename S>
IMatrix3<S> TMatrix3<S>::getBound() const
{
  return IMatrix3<S>(v_[0].getBound(), v_[1].getBound(), v_[2].getBound());
}

//==============================================================================
template <typename S>
IMatrix3<S> TMatrix3<S>::getBound(S l, S r) const
{
  return IMatrix3<S>(v_[0].getBound(l, r), v_[1].getBound(l, r), v_[2].getBound(l, r));
}

//==============================================================================
template <typename S>
IMatrix3<S> TMatrix3<S>::getBound(S t) const
{
  return IMatrix3<S>(v_[0].getBound(t), v_[1].getBound(t), v_[2].getBound(t));
}

//==============================================================================
template <typename S>
IMatrix3<S> TMatrix3<S>::getTightBound() const
{
  return IMatrix3<S>(v_[0].getTightBound(), v_[1].getTightBound(), v_[2].getTightBound());
}

//==============================================================================
template <typename S>
IMatrix3<S> TMatrix3<S>::getTightBound(S l, S r) const
{
  return IMatrix3<S>(v_[0].getTightBound(l, r), v_[1].getTightBound(l, r), v_[2].getTightBound(l, r));
}

//==============================================================================
template <typename S>
S TMatrix3<S>::diameter() const
{
  S d = 0;

  S tmp = v_[0][0].remainder().diameter();
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

//==============================================================================
template <typename S>
void TMatrix3<S>::setTimeInterval(const std::shared_ptr<TimeInterval<S>>& time_interval)
{
  v_[0].setTimeInterval(time_interval);
  v_[1].setTimeInterval(time_interval);
  v_[2].setTimeInterval(time_interval);
}

//==============================================================================
template <typename S>
void TMatrix3<S>::setTimeInterval(S l, S r)
{
  v_[0].setTimeInterval(l, r);
  v_[1].setTimeInterval(l, r);
  v_[2].setTimeInterval(l, r);
}

//==============================================================================
template <typename S>
const std::shared_ptr<TimeInterval<S>>& TMatrix3<S>::getTimeInterval() const
{
  return v_[0].getTimeInterval();
}

//==============================================================================
template <typename S>
TMatrix3<S>& TMatrix3<S>::rotationConstrain()
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

//==============================================================================
template <typename S>
TMatrix3<S> rotationConstrain(const TMatrix3<S>& m)
{
  TMatrix3<S> res;

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

//==============================================================================
template <typename S>
TMatrix3<S> operator * (const Matrix3<S>& m, const TaylorModel<S>& a)
{
  TMatrix3<S> res(a.getTimeInterval());
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

//==============================================================================
template <typename S>
TMatrix3<S> operator * (const TaylorModel<S>& a, const Matrix3<S>& m)
{
  return m * a;
}

//==============================================================================
template <typename S>
TMatrix3<S> operator * (const TaylorModel<S>& a, const TMatrix3<S>& m)
{
  return m * a;
}

//==============================================================================
template <typename S>
TMatrix3<S> operator * (S d, const TMatrix3<S>& m)
{
  return m * d;
}

//==============================================================================
template <typename S>
TMatrix3<S> operator + (const Matrix3<S>& m1, const TMatrix3<S>& m2)
{
  return m2 + m1;
}

//==============================================================================
template <typename S>
TMatrix3<S> operator - (const Matrix3<S>& m1, const TMatrix3<S>& m2)
{
  return -m2 + m1;
}

} // namespace fcl

#endif

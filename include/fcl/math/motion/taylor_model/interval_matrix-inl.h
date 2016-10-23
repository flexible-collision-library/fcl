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

#ifndef FCL_CCD_INTERVAL_MATRIX_INL_H
#define FCL_CCD_INTERVAL_MATRIX_INL_H

#include "fcl/math/motion/taylor_model/interval_matrix.h"

namespace fcl
{

//==============================================================================
extern template
struct IMatrix3<double>;

//==============================================================================
extern template
IMatrix3<double> rotationConstrain(const IMatrix3<double>& m);

//==============================================================================
template <typename S>
IMatrix3<S>::IMatrix3() {}

//==============================================================================
template <typename S>
IMatrix3<S>::IMatrix3(S v)
{
  v_[0].setValue(v);
  v_[1].setValue(v);
  v_[2].setValue(v);
}

//==============================================================================
template <typename S>
IMatrix3<S>::IMatrix3(const Matrix3<S>& m)
{
  v_[0].setValue(m.row(0)[0], m.row(0)[1], m.row(0)[2]);
  v_[1].setValue(m.row(1)[0], m.row(1)[1], m.row(1)[2]);
  v_[2].setValue(m.row(2)[0], m.row(2)[1], m.row(2)[2]);
}

//==============================================================================
template <typename S>
IMatrix3<S>::IMatrix3(S m[3][3][2])
{
  v_[0].setValue(m[0]);
  v_[1].setValue(m[1]);
  v_[2].setValue(m[2]);
}

//==============================================================================
template <typename S>
IMatrix3<S>::IMatrix3(S m[3][3])
{
  v_[0].setValue(m[0]);
  v_[1].setValue(m[1]);
  v_[2].setValue(m[2]);
}

//==============================================================================
template <typename S>
IMatrix3<S>::IMatrix3(Interval<S> m[3][3])
{
  v_[0].setValue(m[0]);
  v_[1].setValue(m[1]);
  v_[2].setValue(m[2]);
}

//==============================================================================
template <typename S>
IMatrix3<S>::IMatrix3(const IVector3<S>& v1, const IVector3<S>& v2, const IVector3<S>& v3)
{
  v_[0] = v1;
  v_[1] = v2;
  v_[2] = v3;
}

//==============================================================================
template <typename S>
void IMatrix3<S>::setIdentity()
{
  v_[0].setValue(1, 0, 0);
  v_[1].setValue(0, 1, 0);
  v_[2].setValue(0, 0, 1);
}

//==============================================================================
template <typename S>
IVector3<S> IMatrix3<S>::getColumn(size_t i) const
{
  return IVector3<S>(v_[0][i], v_[1][i], v_[2][i]);
}

//==============================================================================
template <typename S>
const IVector3<S>& IMatrix3<S>::getRow(size_t i) const
{
  return v_[i];
}

//==============================================================================
template <typename S>
Vector3<S> IMatrix3<S>::getColumnLow(size_t i) const
{
  return Vector3<S>(v_[0][i][0], v_[1][i][0], v_[2][i][0]);
}

//==============================================================================
template <typename S>
Vector3<S> IMatrix3<S>::getRowLow(size_t i) const
{
  return Vector3<S>(v_[i][0][0], v_[i][1][0], v_[i][2][0]);
}

//==============================================================================
template <typename S>
Vector3<S> IMatrix3<S>::getColumnHigh(size_t i) const
{
  return Vector3<S>(v_[0][i][1], v_[1][i][1], v_[2][i][1]);
}

//==============================================================================
template <typename S>
Vector3<S> IMatrix3<S>::getRowHigh(size_t i) const
{
  return Vector3<S>(v_[i][0][1], v_[i][1][1], v_[i][2][1]);
}

//==============================================================================
template <typename S>
Matrix3<S> IMatrix3<S>::getLow() const
{
  Matrix3<S> m;
  m << v_[0][0][1], v_[0][1][1], v_[0][2][1],
       v_[1][0][1], v_[1][1][1], v_[1][2][1],
       v_[2][0][1], v_[2][1][1], v_[2][2][1];
  return m;
}

//==============================================================================
template <typename S>
Matrix3<S> IMatrix3<S>::getHigh() const
{
  Matrix3<S> m;
  m << v_[0][0][1], v_[0][1][1], v_[0][2][1],
       v_[1][0][1], v_[1][1][1], v_[1][2][1],
       v_[2][0][1], v_[2][1][1], v_[2][2][1];
  return m;
}

//==============================================================================
template <typename S>
const Interval<S>& IMatrix3<S>::operator ()(size_t i, size_t j) const
{
  return v_[i][j];
}

//==============================================================================
template <typename S>
Interval<S>& IMatrix3<S>::operator ()(size_t i, size_t j)
{
  return v_[i][j];
}

//==============================================================================
template <typename S>
IMatrix3<S> IMatrix3<S>::operator * (const Matrix3<S>& m) const
{
  return IMatrix3(IVector3<S>(v_[0].dot(m.col(0)), v_[0].dot(m.col(1)), v_[0].dot(m.col(2))),
                  IVector3<S>(v_[1].dot(m.col(0)), v_[1].dot(m.col(1)), v_[1].dot(m.col(2))),
                  IVector3<S>(v_[2].dot(m.col(0)), v_[2].dot(m.col(1)), v_[2].dot(m.col(2))));
}

//==============================================================================
template <typename S>
IVector3<S> IMatrix3<S>::operator * (const Vector3<S>& v) const
{
  return IVector3<S>(v_[0].dot(v), v_[1].dot(v), v_[2].dot(v));
}

//==============================================================================
template <typename S>
IVector3<S> IMatrix3<S>::operator * (const IVector3<S>& v) const
{
  return IVector3<S>(v_[0].dot(v), v_[1].dot(v), v_[2].dot(v));
}

//==============================================================================
template <typename S>
IMatrix3<S> IMatrix3<S>::operator * (const IMatrix3<S>& m) const
{
  const IVector3<S>& mc0 = m.getColumn(0);
  const IVector3<S>& mc1 = m.getColumn(1);
  const IVector3<S>& mc2 = m.getColumn(2);

  return IMatrix3(IVector3<S>(v_[0].dot(mc0), v_[0].dot(mc1), v_[0].dot(mc2)),
                  IVector3<S>(v_[1].dot(mc0), v_[1].dot(mc1), v_[1].dot(mc2)),
                  IVector3<S>(v_[2].dot(mc0), v_[2].dot(mc1), v_[2].dot(mc2)));
}

//==============================================================================
template <typename S>
IMatrix3<S>& IMatrix3<S>::operator *= (const Matrix3<S>& m)
{
  v_[0].setValue(v_[0].dot(m.col(0)), v_[0].dot(m.col(1)), v_[0].dot(m.col(2)));
  v_[1].setValue(v_[1].dot(m.col(0)), v_[1].dot(m.col(1)), v_[1].dot(m.col(2)));
  v_[2].setValue(v_[2].dot(m.col(0)), v_[2].dot(m.col(1)), v_[2].dot(m.col(2)));
  return *this;
}

//==============================================================================
template <typename S>
IMatrix3<S>& IMatrix3<S>::operator *= (const IMatrix3<S>& m)
{
  const IVector3<S>& mc0 = m.getColumn(0);
  const IVector3<S>& mc1 = m.getColumn(1);
  const IVector3<S>& mc2 = m.getColumn(2);

  v_[0].setValue(v_[0].dot(mc0), v_[0].dot(mc1), v_[0].dot(mc2));
  v_[1].setValue(v_[1].dot(mc0), v_[1].dot(mc1), v_[1].dot(mc2));
  v_[2].setValue(v_[2].dot(mc0), v_[2].dot(mc1), v_[2].dot(mc2));
  return *this;
}

//==============================================================================
template <typename S>
IMatrix3<S> IMatrix3<S>::operator + (const IMatrix3<S>& m) const
{
  return IMatrix3(v_[0] + m.v_[0], v_[1] + m.v_[1], v_[2] + m.v_[2]);
}

//==============================================================================
template <typename S>
IMatrix3<S>& IMatrix3<S>::operator += (const IMatrix3<S>& m)
{
  v_[0] += m.v_[0];
  v_[1] += m.v_[1];
  v_[2] += m.v_[2];
  return *this;
}

//==============================================================================
template <typename S>
IMatrix3<S> IMatrix3<S>::operator - (const IMatrix3<S>& m) const
{
  return IMatrix3(v_[0] - m.v_[0], v_[1] - m.v_[1], v_[2] - m.v_[2]);
}

//==============================================================================
template <typename S>
IMatrix3<S>& IMatrix3<S>::operator -= (const IMatrix3<S>& m)
{
  v_[0] -= m.v_[0];
  v_[1] -= m.v_[1];
  v_[2] -= m.v_[2];
  return *this;
}

//==============================================================================
template <typename S>
IMatrix3<S>& IMatrix3<S>::rotationConstrain()
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

//==============================================================================
template <typename S>
void IMatrix3<S>::print() const
{
  std::cout << "[" << v_[0][0][0] << "," << v_[0][0][1] << "]" << " [" << v_[0][1][0] << "," << v_[0][1][1] << "]" << " [" << v_[0][2][0] << "," << v_[0][2][1] << "]" << std::endl;
  std::cout << "[" << v_[1][0][0] << "," << v_[1][0][1] << "]" << " [" << v_[1][1][0] << "," << v_[1][1][1] << "]" << " [" << v_[1][2][0] << "," << v_[1][2][1] << "]" << std::endl;
  std::cout << "[" << v_[2][0][0] << "," << v_[2][0][1] << "]" << " [" << v_[2][1][0] << "," << v_[2][1][1] << "]" << " [" << v_[2][2][0] << "," << v_[2][2][1] << "]" << std::endl;
}

//==============================================================================
template <typename S>
IMatrix3<S> rotationConstrain(const IMatrix3<S>& m)
{
  IMatrix3<S> res;
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

} // namespace fcl

#endif

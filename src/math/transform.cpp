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

#include "fcl/math/constants.h"
#include "fcl/math/transform.h"
#include <cassert>

namespace fcl
{

void Quaternion3f::fromRotation(const Matrix3f& R)
{
  const int next[3] = {1, 2, 0};

  FCL_REAL trace = R(0, 0) + R(1, 1) + R(2, 2);
  FCL_REAL root;

  if(trace > 0.0)
  {
    // |w| > 1/2, may as well choose w > 1/2
    root = sqrt(trace + 1.0);  // 2w
    data[0] = 0.5 * root;
    root = 0.5 / root;  // 1/(4w)
    data[1] = (R(2, 1) - R(1, 2))*root;
    data[2] = (R(0, 2) - R(2, 0))*root;
    data[3] = (R(1, 0) - R(0, 1))*root;
  }
  else
  {
    // |w| <= 1/2
    int i = 0;
    if(R(1, 1) > R(0, 0))
    {
      i = 1;
    }
    if(R(2, 2) > R(i, i))
    {
      i = 2;
    }
    int j = next[i];
    int k = next[j];

    root = sqrt(R(i, i) - R(j, j) - R(k, k) + 1.0);
    FCL_REAL* quat[3] = { &data[1], &data[2], &data[3] };
    *quat[i] = 0.5 * root;
    root = 0.5 / root;
    data[0] = (R(k, j) - R(j, k)) * root;
    *quat[j] = (R(j, i) + R(i, j)) * root;
    *quat[k] = (R(k, i) + R(i, k)) * root;
  }
}

void Quaternion3f::toRotation(Matrix3f& R) const
{
  assert (.99 < data [0]*data [0] + data [1]*data [1] +
	  data [2]*data [2] + data [3]*data [3]);
  assert (data [0]*data [0] + data [1]*data [1] +
	  data [2]*data [2] + data [3]*data [3] < 1.01);
  FCL_REAL twoX  = 2.0*data[1];
  FCL_REAL twoY  = 2.0*data[2];
  FCL_REAL twoZ  = 2.0*data[3];
  FCL_REAL twoWX = twoX*data[0];
  FCL_REAL twoWY = twoY*data[0];
  FCL_REAL twoWZ = twoZ*data[0];
  FCL_REAL twoXX = twoX*data[1];
  FCL_REAL twoXY = twoY*data[1];
  FCL_REAL twoXZ = twoZ*data[1];
  FCL_REAL twoYY = twoY*data[2];
  FCL_REAL twoYZ = twoZ*data[2];
  FCL_REAL twoZZ = twoZ*data[3];

  R.setValue(1.0 - (twoYY + twoZZ), twoXY - twoWZ, twoXZ + twoWY,
             twoXY + twoWZ, 1.0 - (twoXX + twoZZ), twoYZ - twoWX,
             twoXZ - twoWY, twoYZ + twoWX, 1.0 - (twoXX + twoYY));
}


void Quaternion3f::fromAxes(const Vec3f axis[3])
{
  // Algorithm in Ken Shoemake's article in 1987 SIGGRAPH course notes
  // article "Quaternion Calculus and Fast Animation".

  const int next[3] = {1, 2, 0};

  FCL_REAL trace = axis[0][0] + axis[1][1] + axis[2][2];
  FCL_REAL root;

  if(trace > 0.0)
  {
    // |w| > 1/2, may as well choose w > 1/2
    root = sqrt(trace + 1.0);  // 2w
    data[0] = 0.5 * root;
    root = 0.5 / root;  // 1/(4w)
    data[1] = (axis[1][2] - axis[2][1])*root;
    data[2] = (axis[2][0] - axis[0][2])*root;
    data[3] = (axis[0][1] - axis[1][0])*root;
  }
  else
  {
    // |w| <= 1/2
    int i = 0;
    if(axis[1][1] > axis[0][0])
    {
      i = 1;
    }
    if(axis[2][2] > axis[i][i])
    {
      i = 2;
    }
    int j = next[i];
    int k = next[j];

    root = sqrt(axis[i][i] - axis[j][j] - axis[k][k] + 1.0);
    FCL_REAL* quat[3] = { &data[1], &data[2], &data[3] };
    *quat[i] = 0.5 * root;
    root = 0.5 / root;
    data[0] = (axis[j][k] - axis[k][j]) * root;
    *quat[j] = (axis[i][j] + axis[j][i]) * root;
    *quat[k] = (axis[i][k] + axis[k][i]) * root;
  }
}

void Quaternion3f::toAxes(Vec3f axis[3]) const
{
  FCL_REAL twoX  = 2.0*data[1];
  FCL_REAL twoY  = 2.0*data[2];
  FCL_REAL twoZ  = 2.0*data[3];
  FCL_REAL twoWX = twoX*data[0];
  FCL_REAL twoWY = twoY*data[0];
  FCL_REAL twoWZ = twoZ*data[0];
  FCL_REAL twoXX = twoX*data[1];
  FCL_REAL twoXY = twoY*data[1];
  FCL_REAL twoXZ = twoZ*data[1];
  FCL_REAL twoYY = twoY*data[2];
  FCL_REAL twoYZ = twoZ*data[2];
  FCL_REAL twoZZ = twoZ*data[3];

  axis[0].setValue(1.0 - (twoYY + twoZZ), twoXY + twoWZ, twoXZ - twoWY);
  axis[1].setValue(twoXY - twoWZ, 1.0 - (twoXX + twoZZ), twoYZ + twoWX);
  axis[2].setValue(twoXZ + twoWY, twoYZ - twoWX, 1.0 - (twoXX + twoYY));
}


void Quaternion3f::fromAxisAngle(const Vec3f& axis, FCL_REAL angle)
{
  FCL_REAL half_angle = 0.5 * angle;
  FCL_REAL sn = sin((double)half_angle);
  data[0] = cos((double)half_angle);
  data[1] = sn * axis[0];
  data[2] = sn * axis[1];
  data[3] = sn * axis[2];
}

void Quaternion3f::toAxisAngle(Vec3f& axis, FCL_REAL& angle) const
{
  double sqr_length = data[1] * data[1] + data[2] * data[2] + data[3] * data[3];
  if(sqr_length > 0)
  {
    angle = 2.0 * acos((double)data[0]);
    double inv_length = 1.0 / sqrt(sqr_length);
    axis[0] = inv_length * data[1];
    axis[1] = inv_length * data[2];
    axis[2] = inv_length * data[3];
  }
  else
  {
    angle = 0;
    axis[0] = 1;
    axis[1] = 0;
    axis[2] = 0;
  }
}

FCL_REAL Quaternion3f::dot(const Quaternion3f& other) const
{
  return data[0] * other.data[0] + data[1] * other.data[1] + data[2] * other.data[2] + data[3] * other.data[3];
}

Quaternion3f Quaternion3f::operator + (const Quaternion3f& other) const
{
  return Quaternion3f(data[0] + other.data[0], data[1] + other.data[1],
                      data[2] + other.data[2], data[3] + other.data[3]);
}

const Quaternion3f& Quaternion3f::operator += (const Quaternion3f& other)
{
  data[0] += other.data[0];
  data[1] += other.data[1];
  data[2] += other.data[2];
  data[3] += other.data[3];

  return *this;
}

Quaternion3f Quaternion3f::operator - (const Quaternion3f& other) const
{
  return Quaternion3f(data[0] - other.data[0], data[1] - other.data[1],
                      data[2] - other.data[2], data[3] - other.data[3]);
}

const Quaternion3f& Quaternion3f::operator -= (const Quaternion3f& other)
{
  data[0] -= other.data[0];
  data[1] -= other.data[1];
  data[2] -= other.data[2];
  data[3] -= other.data[3];

  return *this;
}

Quaternion3f Quaternion3f::operator * (const Quaternion3f& other) const
{
  return Quaternion3f(data[0] * other.data[0] - data[1] * other.data[1] - data[2] * other.data[2] - data[3] * other.data[3],
                      data[0] * other.data[1] + data[1] * other.data[0] + data[2] * other.data[3] - data[3] * other.data[2],
                      data[0] * other.data[2] - data[1] * other.data[3] + data[2] * other.data[0] + data[3] * other.data[1],
                      data[0] * other.data[3] + data[1] * other.data[2] - data[2] * other.data[1] + data[3] * other.data[0]);
}


const Quaternion3f& Quaternion3f::operator *= (const Quaternion3f& other)
{
  FCL_REAL a = data[0] * other.data[0] - data[1] * other.data[1] - data[2] * other.data[2] - data[3] * other.data[3];
  FCL_REAL b = data[0] * other.data[1] + data[1] * other.data[0] + data[2] * other.data[3] - data[3] * other.data[2];
  FCL_REAL c = data[0] * other.data[2] - data[1] * other.data[3] + data[2] * other.data[0] + data[3] * other.data[1];
  FCL_REAL d = data[0] * other.data[3] + data[1] * other.data[2] - data[2] * other.data[1] + data[3] * other.data[0];

  data[0] = a;
  data[1] = b;
  data[2] = c;
  data[3] = d;
  return *this;
}

Quaternion3f Quaternion3f::operator - () const
{
  return Quaternion3f(-data[0], -data[1], -data[2], -data[3]);
}

Quaternion3f Quaternion3f::operator * (FCL_REAL t) const
{
  return Quaternion3f(data[0] * t, data[1] * t, data[2] * t, data[3] * t);
}

const Quaternion3f& Quaternion3f::operator *= (FCL_REAL t)
{
  data[0] *= t;
  data[1] *= t;
  data[2] *= t;
  data[3] *= t;

  return *this;
}


Quaternion3f& Quaternion3f::conj()
{
  data[1] = -data[1];
  data[2] = -data[2];
  data[3] = -data[3];
  return *this;
}

Quaternion3f& Quaternion3f::inverse()
{
  FCL_REAL sqr_length = data[0] * data[0] + data[1] * data[1] + data[2] * data[2] + data[3] * data[3];
  if(sqr_length > 0)
  {
    FCL_REAL inv_length = 1 / std::sqrt(sqr_length);
    data[0] *= inv_length;
    data[1] *= (-inv_length);
    data[2] *= (-inv_length);
    data[3] *= (-inv_length);
  }
  else
  {
    data[1] = -data[1];
    data[2] = -data[2];
    data[3] = -data[3];
  }

  return *this;
}

Vec3f Quaternion3f::transform(const Vec3f& v) const
{
  Vec3f u(getX(), getY(), getZ());
  double s = getW();
  Vec3f vprime = 2*u.dot(v)*u + (s*s - u.dot(u))*v + 2*s*u.cross(v);
  return vprime;
}

Quaternion3f conj(const Quaternion3f& q)
{
  Quaternion3f r(q);
  return r.conj();
}

Quaternion3f inverse(const Quaternion3f& q)
{
  Quaternion3f res(q);
  return res.inverse();
}

void Quaternion3f::fromEuler(FCL_REAL a, FCL_REAL b, FCL_REAL c)
{
  Matrix3f R;
  R.setEulerYPR(a, b, c);

  fromRotation(R);
}

void Quaternion3f::toEuler(FCL_REAL& a, FCL_REAL& b, FCL_REAL& c) const
{
  Matrix3f R;
  toRotation(R);
  a = atan2(R(1, 0), R(0, 0));
  b = asin(-R(2, 0));
  c = atan2(R(2, 1), R(2, 2));

  if(b == constants::pi * 0.5)
  {
    if(a > 0)
      a -= constants::pi;
    else 
      a += constants::pi;

    if(c > 0)
      c -= constants::pi;
    else
      c += constants::pi;
  }
}


Vec3f Quaternion3f::getColumn(std::size_t i) const
{
  switch(i)
  {
  case 0:
    return Vec3f(data[0] * data[0] + data[1] * data[1] - data[2] * data[2] - data[3] * data[3],
                 2 * (- data[0] * data[3] + data[1] * data[2]),
                 2 * (data[1] * data[3] + data[0] * data[2]));
  case 1:
    return Vec3f(2 * (data[1] * data[2] + data[0] * data[3]),
                 data[0] * data[0] - data[1] * data[1] + data[2] * data[2] - data[3] * data[3],
                 2 * (data[2] * data[3] - data[0] * data[1]));
  case 2:
    return Vec3f(2 * (data[1] * data[3] - data[0] * data[2]),
                 2 * (data[2] * data[3] + data[0] * data[1]),
                 data[0] * data[0] - data[1] * data[1] - data[2] * data[2] + data[3] * data[3]);
  default:
    return Vec3f();
  }
}

Vec3f Quaternion3f::getRow(std::size_t i) const
{
  switch(i)
  {
  case 0:
    return Vec3f(data[0] * data[0] + data[1] * data[1] - data[2] * data[2] - data[3] * data[3],
                 2 * (data[0] * data[3] + data[1] * data[2]),
                 2 * (data[1] * data[3] - data[0] * data[2]));
  case 1:
    return Vec3f(2 * (data[1] * data[2] - data[0] * data[3]),
                 data[0] * data[0] - data[1] * data[1] + data[2] * data[2] - data[3] * data[3],
                 2 * (data[2] * data[3] + data[0] * data[1]));
  case 2:
    return Vec3f(2 * (data[1] * data[3] + data[0] * data[2]),
                 2 * (data[2] * data[3] - data[0] * data[1]),
                 data[0] * data[0] - data[1] * data[1] - data[2] * data[2] + data[3] * data[3]);
  default:
    return Vec3f();
  }
}


const Matrix3f& Transform3f::getRotationInternal() const
{
  std::unique_lock<std::mutex> slock(const_cast<std::mutex&>(lock_));
  if(!matrix_set)
  {
    q.toRotation(R);
    matrix_set = true;
  }

  return R;
}


Transform3f inverse(const Transform3f& tf)
{
  Transform3f res(tf);
  return res.inverse();
}

void relativeTransform(const Transform3f& tf1, const Transform3f& tf2,
                       Transform3f& tf)
{
  const Quaternion3f& q1_inv = fcl::conj(tf1.getQuatRotation());
  tf = Transform3f(q1_inv * tf2.getQuatRotation(), q1_inv.transform(tf2.getTranslation() - tf1.getTranslation()));
}

void relativeTransform2(const Transform3f& tf1, const Transform3f& tf2,
                       Transform3f& tf)
{
  const Quaternion3f& q1inv = fcl::conj(tf1.getQuatRotation());
  const Quaternion3f& q2_q1inv = tf2.getQuatRotation() * q1inv;
  tf = Transform3f(q2_q1inv, tf2.getTranslation() - q2_q1inv.transform(tf1.getTranslation()));
}



}

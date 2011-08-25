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


#include "fcl/transform.h"

namespace fcl
{

void SimpleQuaternion::fromRotation(const Vec3f R[3])
{
  const int next[3] = {1, 2, 0};

  BVH_REAL trace = R[0][0] + R[1][1] + R[2][2];
  BVH_REAL root;

  if(trace > 0.0)
  {
    // |w| > 1/2, may as well choose w > 1/2
    root = sqrt(trace + 1.0);  // 2w
    data[0] = 0.5 * root;
    root = 0.5 / root;  // 1/(4w)
    data[1] = (R[2][1] - R[1][2])*root;
    data[2] = (R[0][2] - R[2][0])*root;
    data[3] = (R[1][0] - R[0][1])*root;
  }
  else
  {
    // |w| <= 1/2
    int i = 0;
    if(R[1][1] > R[0][0])
    {
        i = 1;
    }
    if(R[2][2] > R[i][i])
    {
        i = 2;
    }
    int j = next[i];
    int k = next[j];

    root = sqrt(R[i][i] - R[j][j] - R[k][k] + 1.0);
    BVH_REAL* quat[3] = { &data[1], &data[2], &data[3] };
    *quat[i] = 0.5 * root;
    root = 0.5 / root;
    data[0] = (R[k][j] - R[j][k]) * root;
    *quat[j] = (R[j][i] + R[i][j]) * root;
    *quat[k] = (R[k][i] + R[i][k]) * root;
  }
}

void SimpleQuaternion::toRotation(Vec3f R[3]) const
{
  BVH_REAL twoX  = 2.0*data[1];
  BVH_REAL twoY  = 2.0*data[2];
  BVH_REAL twoZ  = 2.0*data[3];
  BVH_REAL twoWX = twoX*data[0];
  BVH_REAL twoWY = twoY*data[0];
  BVH_REAL twoWZ = twoZ*data[0];
  BVH_REAL twoXX = twoX*data[1];
  BVH_REAL twoXY = twoY*data[1];
  BVH_REAL twoXZ = twoZ*data[1];
  BVH_REAL twoYY = twoY*data[2];
  BVH_REAL twoYZ = twoZ*data[2];
  BVH_REAL twoZZ = twoZ*data[3];

  R[0] = Vec3f(1.0 - (twoYY + twoZZ), twoXY - twoWZ, twoXZ + twoWY);
  R[1] = Vec3f(twoXY + twoWZ, 1.0 - (twoXX + twoZZ), twoYZ - twoWX);
  R[2] = Vec3f(twoXZ - twoWY, twoYZ + twoWX, 1.0 - (twoXX + twoYY));
}


void SimpleQuaternion::fromAxes(const Vec3f axis[3])
{
  // Algorithm in Ken Shoemake's article in 1987 SIGGRAPH course notes
  // article "Quaternion Calculus and Fast Animation".

  const int next[3] = {1, 2, 0};

  BVH_REAL trace = axis[0][0] + axis[1][1] + axis[2][2];
  BVH_REAL root;

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
    BVH_REAL* quat[3] = { &data[1], &data[2], &data[3] };
    *quat[i] = 0.5 * root;
    root = 0.5 / root;
    data[0] = (axis[j][k] - axis[k][j]) * root;
    *quat[j] = (axis[i][j] + axis[j][i]) * root;
    *quat[k] = (axis[i][k] + axis[k][i]) * root;
  }
}

void SimpleQuaternion::toAxes(Vec3f axis[3]) const
{
  BVH_REAL twoX  = 2.0*data[1];
  BVH_REAL twoY  = 2.0*data[2];
  BVH_REAL twoZ  = 2.0*data[3];
  BVH_REAL twoWX = twoX*data[0];
  BVH_REAL twoWY = twoY*data[0];
  BVH_REAL twoWZ = twoZ*data[0];
  BVH_REAL twoXX = twoX*data[1];
  BVH_REAL twoXY = twoY*data[1];
  BVH_REAL twoXZ = twoZ*data[1];
  BVH_REAL twoYY = twoY*data[2];
  BVH_REAL twoYZ = twoZ*data[2];
  BVH_REAL twoZZ = twoZ*data[3];

  axis[0] = Vec3f(1.0 - (twoYY + twoZZ), twoXY + twoWZ, twoXZ - twoWY);
  axis[1] = Vec3f(twoXY - twoWZ, 1.0 - (twoXX + twoZZ), twoYZ + twoWX);
  axis[2] = Vec3f(twoXZ + twoWY, twoYZ - twoWX, 1.0 - (twoXX + twoYY));
}


void SimpleQuaternion::fromAxisAngle(const Vec3f& axis, BVH_REAL angle)
{
  BVH_REAL half_angle = 0.5 * angle;
  BVH_REAL sn = sin((double)half_angle);
  data[0] = cos((double)half_angle);
  data[1] = sn * axis[0];
  data[2] = sn * axis[1];
  data[3] = sn * axis[2];
}

void SimpleQuaternion::toAxisAngle(Vec3f& axis, BVH_REAL& angle) const
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

BVH_REAL SimpleQuaternion::dot(const SimpleQuaternion& other) const
{
  return data[0] * other.data[0] + data[1] * other.data[1] + data[2] * other.data[2] + data[3] * other.data[3];
}

SimpleQuaternion SimpleQuaternion::operator + (const SimpleQuaternion& other) const
{
  return SimpleQuaternion(data[0] + other.data[0], data[1] + other.data[1],
                          data[2] + other.data[2], data[3] + other.data[3]);
}

SimpleQuaternion SimpleQuaternion::operator - (const SimpleQuaternion& other) const
{
  return SimpleQuaternion(data[0] - other.data[0], data[1] - other.data[1],
                          data[2] - other.data[2], data[3] - other.data[3]);
}

SimpleQuaternion SimpleQuaternion::operator * (const SimpleQuaternion& other) const
{
  return SimpleQuaternion(data[0] * other.data[0] - data[1] * other.data[1] - data[2] * other.data[2] - data[3] * other.data[3],
                          data[0] * other.data[1] + data[1] * other.data[0] + data[2] * other.data[3] - data[3] * other.data[2],
                          data[0] * other.data[2] - data[1] * other.data[3] + data[2] * other.data[0] + data[3] * other.data[1],
                          data[0] * other.data[3] + data[1] * other.data[2] - data[2] * other.data[1] + data[3] * other.data[0]);
}

SimpleQuaternion SimpleQuaternion::operator - () const
{
  return SimpleQuaternion(-data[0], -data[1], -data[2], -data[3]);
}

SimpleQuaternion SimpleQuaternion::operator * (BVH_REAL t) const
{
  return SimpleQuaternion(data[0] * t, data[1] * t, data[2] * t, data[3] * t);
}

SimpleQuaternion SimpleQuaternion::conj() const
{
  return SimpleQuaternion(data[0], -data[1], -data[2], -data[3]);
}

SimpleQuaternion SimpleQuaternion::inverse() const
{
  double sqr_length = data[0] * data[0] + data[1] * data[1] + data[2] * data[2] + data[3] * data[3];
  if(sqr_length > 0)
  {
    double inv_length = 1.0 / sqrt(sqr_length);
    return SimpleQuaternion(data[0] * inv_length, -data[1] * inv_length, -data[2] * inv_length, -data[3] * inv_length);
  }
  else
  {
    return SimpleQuaternion(data[0], -data[1], -data[2], -data[3]);
  }
}

Vec3f SimpleQuaternion::transform(const Vec3f& v) const
{
  SimpleQuaternion r = (*this) * SimpleQuaternion(0, v[0], v[1], v[2]) * (this->conj());
  return Vec3f(r.data[1], r.data[2], r.data[3]);
}


}

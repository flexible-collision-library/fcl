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


#ifndef FCL_INTERVAL_MATRIX_H
#define FCL_INTERVAL_MATRIX_H

#include "fcl/interval.h"
#include "fcl/matrix_3f.h"
#include "fcl/interval_vector.h"

namespace fcl
{

struct IMatrix3
{
  Interval i_[3][3];

  IMatrix3();
  IMatrix3(BVH_REAL v);
  IMatrix3(const Matrix3f& m);
  IMatrix3(BVH_REAL m[3][3][2]);
  IMatrix3(BVH_REAL m[3][3]);

  void setIdentity();

  IVector3 getColumn(size_t i) const;
  IVector3 getRow(size_t i) const;

  Vec3f getRealColumn(size_t i) const;
  Vec3f getRealRow(size_t i) const;

  IVector3 operator * (const Vec3f& v) const;
  IVector3 operator * (const IVector3& v) const;
  IMatrix3 operator * (const IMatrix3& m) const;
  IMatrix3 operator * (const Matrix3f& m) const;
  IMatrix3 operator + (const IMatrix3& m) const;
  IMatrix3& operator += (const IMatrix3& m);

  void print() const;

  IMatrix3 nonIntervalAddMatrix(const IMatrix3& m) const;
  IVector3 nonIntervalTimesVector(const IVector3& v) const;
  IVector3 nonIntervalTimesVector(const Vec3f& v) const;

};

}

#endif

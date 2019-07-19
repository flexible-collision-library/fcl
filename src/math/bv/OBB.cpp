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

/** @author Jia Pan */

#include "fcl/math/bv/OBB-inl.h"

namespace fcl
{

//==============================================================================
template
class OBB<double>;

//==============================================================================
template
void computeVertices(const OBB<double>& b, Vector3<double> vertices[8]);

//==============================================================================
template
OBB<double> merge_largedist(const OBB<double>& b1, const OBB<double>& b2);

//==============================================================================
template
OBB<double> merge_smalldist(const OBB<double>& b1, const OBB<double>& b2);

//==============================================================================
template
bool obbDisjoint(
    const Matrix3<double>& B,
    const Vector3<double>& T,
    const Vector3<double>& a,
    const Vector3<double>& b);

//==============================================================================
template
bool obbDisjoint(
    const Transform3<double>& tf,
    const Vector3<double>& a,
    const Vector3<double>& b);

//==============================================================================
template <>
bool obbDisjoint(const Matrix3<float>& B, const Vector3<float>& T,
                 const Vector3<float>& a, const Vector3<float>& b)
{
  const float reps = 1e-6;

  Matrix3<float> Bf = B.cwiseAbs();
  Bf.array() += reps;

  // Test the three major axes of the OBB a.
  if (((T.cwiseAbs() - (a + Bf * b)).array() > 0).any()) {
    return true;
  }

  // Test the three major axes of the OBB b.
  if ((((B.transpose() * T).cwiseAbs() - (b + Bf.transpose() * a)).array() > 0).any()) {
    return true;
  }

  // Test the 9 different cross-axes.
  Matrix3<float> symmetric_matrix;
  symmetric_matrix <<    0, b[2], b[1],
                      b[2],    0, b[0],
                      b[1], b[0],    0;

  // A0 x B0
  // A0 x B1
  // A0 x B2
  if ((((T[2] * B.row(1) - T[1] * B.row(2)).cwiseAbs() - (Bf.row(2) * a[1] + Bf.row(1) * a[2] + Bf.row(0) * symmetric_matrix)).array() > 0).any()) {
    return true;
  }

  // A1 x B0
  // A1 x B1
  // A1 x B2
  if ((((T[0] * B.row(2) - T[2] * B.row(0)).cwiseAbs() - (Bf.row(2) * a[0] + Bf.row(0) * a[2] + Bf.row(1) * symmetric_matrix)).array() > 0).any()) {
    return true;
  }

  // A2 x B0
  // A2 x B1
  // A2 x B2
  if ((((T[1] * B.row(0) - T[0] * B.row(1)).cwiseAbs() - (Bf.row(1) * a[0] + Bf.row(0) * a[1] + Bf.row(2) * symmetric_matrix)).array() > 0).any()) {
    return true;
  }

  return false;
}

} // namespace fcl

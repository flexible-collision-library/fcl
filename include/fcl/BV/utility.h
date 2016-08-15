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


#ifndef FCL_BV_UTILITY_H
#define FCL_BV_UTILITY_H

#include "fcl/data_types.h"
#include "fcl/math/triangle.h"
#include "fcl/BV/detail/utility.h"

namespace fcl
{

/// @brief Compute the bounding volume extent and center for a set or subset of points, given the BV axises.
template <typename S>
void getExtentAndCenter(
    Vector3<S>* ps,
    Vector3<S>* ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    const Matrix3<S>& axis,
    Vector3<S>& center,
    Vector3<S>& extent);

/// @brief Compute the bounding volume extent and center for a set or subset of
/// points, given the BV axises.
template <typename S>
void getExtentAndCenter(
    Vector3<S>* ps,
    Vector3<S>* ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    Transform3<S>& tf,
    Vector3<S>& extent);

/// @brief Compute the covariance matrix for a set or subset of points. if
/// ts = null, then indices refer to points directly; otherwise refer to
/// triangles
template <typename S>
void getCovariance(
    Vector3<S>* ps,
    Vector3<S>* ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    Matrix3<S>& M);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename S>
void getExtentAndCenter(
    Vector3<S>* ps,
    Vector3<S>* ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    const Matrix3<S>& axis,
    Vector3<S>& center,
    Vector3<S>& extent)
{
  if(ts)
    detail::getExtentAndCenter_mesh(ps, ps2, ts, indices, n, axis, center, extent);
  else
    detail::getExtentAndCenter_pointcloud(ps, ps2, indices, n, axis, center, extent);
}

//==============================================================================
template <typename S>
void getExtentAndCenter(
    Vector3<S>* ps,
    Vector3<S>* ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    Transform3<S>& tf,
    Vector3<S>& extent)
{
  if(ts)
    detail::getExtentAndCenter_mesh(ps, ps2, ts, indices, n, tf, extent);
  else
    detail::getExtentAndCenter_pointcloud(ps, ps2, indices, n, tf, extent);
}

//==============================================================================
template <typename S>
void getCovariance(Vector3<S>* ps,
    Vector3<S>* ps2,
    Triangle* ts,
    unsigned int* indices,
    int n, Matrix3<S>& M)
{
  Vector3<S> S1 = Vector3<S>::Zero();
  Vector3<S> S2[3] = {
    Vector3<S>::Zero(), Vector3<S>::Zero(), Vector3<S>::Zero()
  };

  if(ts)
  {
    for(int i = 0; i < n; ++i)
    {
      const Triangle& t = (indices) ? ts[indices[i]] : ts[i];

      const Vector3<S>& p1 = ps[t[0]];
      const Vector3<S>& p2 = ps[t[1]];
      const Vector3<S>& p3 = ps[t[2]];

      S1 += (p1 + p2 + p3).eval();

      S2[0][0] += (p1[0] * p1[0] + p2[0] * p2[0] + p3[0] * p3[0]);
      S2[1][1] += (p1[1] * p1[1] + p2[1] * p2[1] + p3[1] * p3[1]);
      S2[2][2] += (p1[2] * p1[2] + p2[2] * p2[2] + p3[2] * p3[2]);
      S2[0][1] += (p1[0] * p1[1] + p2[0] * p2[1] + p3[0] * p3[1]);
      S2[0][2] += (p1[0] * p1[2] + p2[0] * p2[2] + p3[0] * p3[2]);
      S2[1][2] += (p1[1] * p1[2] + p2[1] * p2[2] + p3[1] * p3[2]);

      if(ps2)
      {
        const Vector3<S>& p1 = ps2[t[0]];
        const Vector3<S>& p2 = ps2[t[1]];
        const Vector3<S>& p3 = ps2[t[2]];

        S1 += (p1 + p2 + p3).eval();

        S2[0][0] += (p1[0] * p1[0] + p2[0] * p2[0] + p3[0] * p3[0]);
        S2[1][1] += (p1[1] * p1[1] + p2[1] * p2[1] + p3[1] * p3[1]);
        S2[2][2] += (p1[2] * p1[2] + p2[2] * p2[2] + p3[2] * p3[2]);
        S2[0][1] += (p1[0] * p1[1] + p2[0] * p2[1] + p3[0] * p3[1]);
        S2[0][2] += (p1[0] * p1[2] + p2[0] * p2[2] + p3[0] * p3[2]);
        S2[1][2] += (p1[1] * p1[2] + p2[1] * p2[2] + p3[1] * p3[2]);
      }
    }
  }
  else
  {
    for(int i = 0; i < n; ++i)
    {
      const Vector3<S>& p = (indices) ? ps[indices[i]] : ps[i];
      S1 += p;
      S2[0][0] += (p[0] * p[0]);
      S2[1][1] += (p[1] * p[1]);
      S2[2][2] += (p[2] * p[2]);
      S2[0][1] += (p[0] * p[1]);
      S2[0][2] += (p[0] * p[2]);
      S2[1][2] += (p[1] * p[2]);

      if(ps2) // another frame
      {
        const Vector3<S>& p = (indices) ? ps2[indices[i]] : ps2[i];
        S1 += p;
        S2[0][0] += (p[0] * p[0]);
        S2[1][1] += (p[1] * p[1]);
        S2[2][2] += (p[2] * p[2]);
        S2[0][1] += (p[0] * p[1]);
        S2[0][2] += (p[0] * p[2]);
        S2[1][2] += (p[1] * p[2]);
      }
    }
  }

  int n_points = ((ps2) ? 2 : 1) * ((ts) ? 3 : 1) * n;

  M(0, 0) = S2[0][0] - S1[0]*S1[0] / n_points;
  M(1, 1) = S2[1][1] - S1[1]*S1[1] / n_points;
  M(2, 2) = S2[2][2] - S1[2]*S1[2] / n_points;
  M(0, 1) = S2[0][1] - S1[0]*S1[1] / n_points;
  M(1, 2) = S2[1][2] - S1[1]*S1[2] / n_points;
  M(0, 2) = S2[0][2] - S1[0]*S1[2] / n_points;
  M(1, 0) = M(0, 1);
  M(2, 0) = M(0, 2);
  M(2, 1) = M(1, 2);
}

} // namespace fcl

#endif

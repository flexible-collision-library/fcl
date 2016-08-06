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

namespace fcl
{

/// @brief Compute the bounding volume extent and center for a set or subset of points, given the BV axises.
template <typename Scalar>
void getExtentAndCenter(
    Vector3<Scalar>* ps,
    Vector3<Scalar>* ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    const Matrix3<Scalar>& axis,
    Vector3<Scalar>& center,
    Vector3<Scalar>& extent);

/// @brief Compute the covariance matrix for a set or subset of points. if
/// ts = null, then indices refer to points directly; otherwise refer to
/// triangles
template <typename Scalar>
void getCovariance(
    Vector3<Scalar>* ps,
    Vector3<Scalar>* ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    Matrix3<Scalar>& M);

namespace detail {

/// \brief Compute the bounding volume extent and center for a set or subset of
/// points. The bounding volume axes are known.
template <typename Scalar>
void getExtentAndCenter_pointcloud(
    Vector3<Scalar>* ps, Vector3<Scalar>* ps2,
    Triangle* ts, unsigned int* indices, int n,
    const Matrix3<Scalar>& axis, Vector3<Scalar>& center, Vector3<Scalar>& extent);

/// \brief Compute the bounding volume extent and center for a set or subset of
/// points. The bounding volume axes are known.
template <typename Scalar>
void getExtentAndCenter_mesh(
    Vector3<Scalar>* ps, Vector3<Scalar>* ps2,
    Triangle* ts, unsigned int* indices, int n,
    const Matrix3<Scalar>& axis, Vector3<Scalar>& center,
    Vector3<Scalar>& extent);

} // namespace detail

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

namespace detail {

//==============================================================================
template <typename Scalar>
void getExtentAndCenter_pointcloud(
    Vector3<Scalar>* ps,
    Vector3<Scalar>* ps2,
    unsigned int* indices,
    int n,
    const Matrix3<Scalar>& axis,
    Vector3<Scalar>& center,
    Vector3<Scalar>& extent)
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  auto real_max = std::numeric_limits<Scalar>::max();

  Vector3<Scalar> min_coord = Vector3<Scalar>::Constant(real_max);
  Vector3<Scalar> max_coord = Vector3<Scalar>::Constant(-real_max);

  for(int i = 0; i < n; ++i)
  {
    int index = indirect_index ? indices[i] : i;

    const Vector3<Scalar>& p = ps[index];
    Vector3<Scalar> proj = p.transpose() * axis;

    for(int j = 0; j < 3; ++j)
    {
      if(proj[j] > max_coord[j])
        max_coord[j] = proj[j];

      if(proj[j] < min_coord[j])
        min_coord[j] = proj[j];
    }

    if(ps2)
    {
      const Vector3<Scalar>& v = ps2[index];
      proj = v.transpose() * axis;

      for(int j = 0; j < 3; ++j)
      {
        if(proj[j] > max_coord[j])
          max_coord[j] = proj[j];

        if(proj[j] < min_coord[j])
          min_coord[j] = proj[j];
      }
    }
  }

  const Vector3<Scalar> o = (max_coord + min_coord) / 2;
  center.noalias() = axis * o;
  extent.noalias() = (max_coord - min_coord) / 2;
}

//==============================================================================
template <typename Scalar>
void getExtentAndCenter_mesh(Vector3<Scalar>* ps,
    Vector3<Scalar>* ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    const Matrix3<Scalar>& axis,
    Vector3<Scalar>& center,
    Vector3<Scalar>& extent)
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  auto real_max = std::numeric_limits<Scalar>::max();

  Vector3<Scalar> min_coord = Vector3<Scalar>::Constant(real_max);
  Vector3<Scalar> max_coord = Vector3<Scalar>::Constant(-real_max);

  for(int i = 0; i < n; ++i)
  {
    unsigned int index = indirect_index? indices[i] : i;
    const Triangle& t = ts[index];

    for(int j = 0; j < 3; ++j)
    {
      int point_id = t[j];
      const Vector3<Scalar>& p = ps[point_id];
      const Vector3<Scalar> proj = p.transpose() * axis;

      for(int k = 0; k < 3; ++k)
      {
        if(proj[k] > max_coord[k])
          max_coord[k] = proj[k];

        if(proj[k] < min_coord[k])
          min_coord[k] = proj[k];
      }
    }

    if(ps2)
    {
      for(int j = 0; j < 3; ++j)
      {
        int point_id = t[j];
        const Vector3<Scalar>& p = ps2[point_id];
        const Vector3<Scalar> proj = p.transpose() * axis;

        for(int k = 0; k < 3; ++k)
        {
          if(proj[k] > max_coord[k])
            max_coord[k] = proj[k];

          if(proj[k] < min_coord[k])
            min_coord[k] = proj[k];
        }
      }
    }
  }

  const Vector3<Scalar> o = (max_coord + min_coord) / 2;
  center.noalias() = axis * o;
  extent.noalias() = (max_coord - min_coord) / 2;
}

} // namespace detail

//==============================================================================
template <typename Scalar>
void getExtentAndCenter(
    Vector3<Scalar>* ps,
    Vector3<Scalar>* ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    const Matrix3<Scalar>& axis,
    Vector3<Scalar>& center,
    Vector3<Scalar>& extent)
{
  if(ts)
    detail::getExtentAndCenter_mesh(ps, ps2, ts, indices, n, axis, center, extent);
  else
    detail::getExtentAndCenter_pointcloud(ps, ps2, indices, n, axis, center, extent);
}

//==============================================================================
template <typename Scalar>
void getCovariance(Vector3<Scalar>* ps,
    Vector3<Scalar>* ps2,
    Triangle* ts,
    unsigned int* indices,
    int n, Matrix3<Scalar>& M)
{
  Vector3<Scalar> S1 = Vector3<Scalar>::Zero();
  Vector3<Scalar> S2[3] = {
    Vector3<Scalar>::Zero(), Vector3<Scalar>::Zero(), Vector3<Scalar>::Zero()
  };

  if(ts)
  {
    for(int i = 0; i < n; ++i)
    {
      const Triangle& t = (indices) ? ts[indices[i]] : ts[i];

      const Vector3<Scalar>& p1 = ps[t[0]];
      const Vector3<Scalar>& p2 = ps[t[1]];
      const Vector3<Scalar>& p3 = ps[t[2]];

      S1[0] += (p1[0] + p2[0] + p3[0]);
      S1[1] += (p1[1] + p2[1] + p3[1]);
      S1[2] += (p1[2] + p2[2] + p3[2]);
      S2[0][0] += (p1[0] * p1[0] + p2[0] * p2[0] + p3[0] * p3[0]);
      S2[1][1] += (p1[1] * p1[1] + p2[1] * p2[1] + p3[1] * p3[1]);
      S2[2][2] += (p1[2] * p1[2] + p2[2] * p2[2] + p3[2] * p3[2]);
      S2[0][1] += (p1[0] * p1[1] + p2[0] * p2[1] + p3[0] * p3[1]);
      S2[0][2] += (p1[0] * p1[2] + p2[0] * p2[2] + p3[0] * p3[2]);
      S2[1][2] += (p1[1] * p1[2] + p2[1] * p2[2] + p3[1] * p3[2]);

      if(ps2)
      {
        const Vector3<Scalar>& p1 = ps2[t[0]];
        const Vector3<Scalar>& p2 = ps2[t[1]];
        const Vector3<Scalar>& p3 = ps2[t[2]];

        S1[0] += (p1[0] + p2[0] + p3[0]);
        S1[1] += (p1[1] + p2[1] + p3[1]);
        S1[2] += (p1[2] + p2[2] + p3[2]);

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
      const Vector3<Scalar>& p = (indices) ? ps[indices[i]] : ps[i];
      S1 += p;
      S2[0][0] += (p[0] * p[0]);
      S2[1][1] += (p[1] * p[1]);
      S2[2][2] += (p[2] * p[2]);
      S2[0][1] += (p[0] * p[1]);
      S2[0][2] += (p[0] * p[2]);
      S2[1][2] += (p[1] * p[2]);

      if(ps2) // another frame
      {
        const Vector3<Scalar>& p = (indices) ? ps2[indices[i]] : ps2[i];
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

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

#ifndef FCL_BV_DETAIL_FITTEROBB_H
#define FCL_BV_DETAIL_FITTEROBB_H

#include "fcl/math/bv/OBB.h"

namespace fcl {
namespace detail {
namespace OBB_fit_functions {

template <typename S>
void fit1(Vector3<S>* ps, OBB<S>& bv);

template <typename S>
void fit2(Vector3<S>* ps, OBB<S>& bv);

template <typename S>
void fit3(Vector3<S>* ps, OBB<S>& bv);

template <typename S>
void fit6(Vector3<S>* ps, OBB<S>& bv);

template <typename S>
void fitn(Vector3<S>* ps, int n, OBB<S>& bv);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename S>
void fit1(Vector3<S>* ps, OBB<S>& bv)
{
  bv.To = ps[0];
  bv.axis.setIdentity();
  bv.extent.setZero();
}

//==============================================================================
template <typename S>
void fit2(Vector3<S>* ps, OBB<S>& bv)
{
  const Vector3<S>& p1 = ps[0];
  const Vector3<S>& p2 = ps[1];
  Vector3<S> p1p2 = p1 - p2;
  S len_p1p2 = p1p2.norm();
  p1p2.normalize();

  bv.axis.col(0) = p1p2;
  generateCoordinateSystem(bv.axis);

  bv.extent << len_p1p2 * 0.5, 0, 0;
  bv.To.noalias() = 0.5 * (p1 + p2);
}

//==============================================================================
template <typename S>
void fit3(Vector3<S>* ps, OBB<S>& bv)
{
  const Vector3<S>& p1 = ps[0];
  const Vector3<S>& p2 = ps[1];
  const Vector3<S>& p3 = ps[2];
  Vector3<S> e[3];
  e[0] = p1 - p2;
  e[1] = p2 - p3;
  e[2] = p3 - p1;
  S len[3];
  len[0] = e[0].squaredNorm();
  len[1] = e[1].squaredNorm();
  len[2] = e[2].squaredNorm();

  int imax = 0;
  if(len[1] > len[0]) imax = 1;
  if(len[2] > len[imax]) imax = 2;

  bv.axis.col(2).noalias() = e[0].cross(e[1]);
  bv.axis.col(2).normalize();
  bv.axis.col(0) = e[imax];
  bv.axis.col(0).normalize();
  bv.axis.col(1).noalias() = bv.axis.col(2).cross(bv.axis.col(0));

  getExtentAndCenter<S>(ps, nullptr, nullptr, nullptr, 3, bv.axis, bv.To, bv.extent);
}

//==============================================================================
template <typename S>
void fit6(Vector3<S>* ps, OBB<S>& bv)
{
  OBB<S> bv1, bv2;
  fit3(ps, bv1);
  fit3(ps + 3, bv2);
  bv = bv1 + bv2;
}

//==============================================================================
template <typename S>
void fitn(Vector3<S>* ps, int n, OBB<S>& bv)
{
  Matrix3<S> M;
  Matrix3<S> E;
  Vector3<S> s = Vector3<S>::Zero(); // three eigen values

  getCovariance<S>(ps, nullptr, nullptr, nullptr, n, M);
  eigen_old(M, s, E);
  axisFromEigen(E, s, bv.axis);

  // set obb centers and extensions
  getExtentAndCenter<S>(ps, nullptr, nullptr, nullptr, n, bv.axis, bv.To, bv.extent);
}

} // namespace OBB_fit_functions
} // namespace detail
} // namespace fcl

#endif

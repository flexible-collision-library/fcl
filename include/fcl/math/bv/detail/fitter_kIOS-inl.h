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

#ifndef FCL_BV_DETAIL_FITTERKIOS_INL_H
#define FCL_BV_DETAIL_FITTERKIOS_INL_H

#include "fcl/math/bv/detail/fitter_kIOS.h"

namespace fcl {
namespace detail {
namespace kIOS_fit_functions {

//==============================================================================
template <typename S>
void fit1(Vector3<S>* ps, kIOS<S>& bv)
{
  bv.num_spheres = 1;
  bv.spheres[0].o = ps[0];
  bv.spheres[0].r = 0;

  bv.obb.axis.setIdentity();
  bv.obb.extent.setZero();
  bv.obb.To = ps[0];
}

template <typename S>
void fit2(Vector3<S>* ps, kIOS<S>& bv)
{
  bv.num_spheres = 5;

  const Vector3<S>& p1 = ps[0];
  const Vector3<S>& p2 = ps[1];
  Vector3<S> p1p2 = p1 - p2;
  S len_p1p2 = p1p2.norm();
  p1p2.normalize();

  bv.obb.axis.col(0) = p1p2;
  generateCoordinateSystem(bv.obb.axis);

  S r0 = len_p1p2 * 0.5;
  bv.obb.extent << r0, 0, 0;
  bv.obb.To.noalias() = (p1 + p2) * 0.5;

  bv.spheres[0].o = bv.obb.To;
  bv.spheres[0].r = r0;

  S r1 = r0 * kIOS<S>::invSinA();
  S r1cosA = r1 * kIOS<S>::cosA();
  bv.spheres[1].r = r1;
  bv.spheres[2].r = r1;
  Vector3<S> delta = bv.obb.axis.col(1) * r1cosA;
  bv.spheres[1].o = bv.spheres[0].o - delta;
  bv.spheres[2].o = bv.spheres[0].o + delta;

  bv.spheres[3].r = r1;
  bv.spheres[4].r = r1;
  delta.noalias() = bv.obb.axis.col(2) * r1cosA;
  bv.spheres[3].o = bv.spheres[0].o - delta;
  bv.spheres[4].o = bv.spheres[0].o + delta;
}

template <typename S>
void fit3(Vector3<S>* ps, kIOS<S>& bv)
{
  bv.num_spheres = 3;

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

  bv.obb.axis.col(2).noalias() = e[0].cross(e[1]).normalized();
  bv.obb.axis.col(0) = e[imax].normalized();
  bv.obb.axis.col(1).noalias() = bv.obb.axis.col(2).cross(bv.obb.axis.col(0));

  getExtentAndCenter<S>(ps, nullptr, nullptr, nullptr, 3, bv.obb.axis, bv.obb.To, bv.obb.extent);

  // compute radius and center
  S r0;
  Vector3<S> center;
  circumCircleComputation(p1, p2, p3, center, r0);

  bv.spheres[0].o = center;
  bv.spheres[0].r = r0;

  S r1 = r0 * kIOS<S>::invSinA();
  Vector3<S> delta = bv.obb.axis.col(2) * (r1 * kIOS<S>::cosA());

  bv.spheres[1].r = r1;
  bv.spheres[1].o = center - delta;
  bv.spheres[2].r = r1;
  bv.spheres[2].o = center + delta;
}

template <typename S>
void fitn(Vector3<S>* ps, int n, kIOS<S>& bv)
{
  Matrix3<S> M;
  Matrix3<S> E;
  Vector3<S> s = Vector3<S>::Zero(); // three eigen values;

  getCovariance<S>(ps, nullptr, nullptr, nullptr, n, M);
  eigen_old(M, s, E);
  axisFromEigen(E, s, bv.obb.axis);

  getExtentAndCenter<S>(ps, nullptr, nullptr, nullptr, n, bv.obb.axis, bv.obb.To, bv.obb.extent);

  // get center and extension
  const Vector3<S>& center = bv.obb.To;
  const Vector3<S>& extent = bv.obb.extent;
  S r0 = maximumDistance<S>(ps, nullptr, nullptr, nullptr, n, center);

  // decide the k in kIOS<S>
  if(extent[0] > kIOS<S>::ratio() * extent[2])
  {
    if(extent[0] > kIOS<S>::ratio() * extent[1]) bv.num_spheres = 5;
    else bv.num_spheres = 3;
  }
  else bv.num_spheres = 1;


  bv.spheres[0].o = center;
  bv.spheres[0].r = r0;

  if(bv.num_spheres >= 3)
  {
    S r10 = sqrt(r0 * r0 - extent[2] * extent[2]) * kIOS<S>::invSinA();
    Vector3<S> delta = bv.obb.axis.col(2) * (r10 * kIOS<S>::cosA() - extent[2]);
    bv.spheres[1].o = center - delta;
    bv.spheres[2].o = center + delta;

    S r11 = 0, r12 = 0;
    r11 = maximumDistance<S>(ps, nullptr, nullptr, nullptr, n, bv.spheres[1].o);
    r12 = maximumDistance<S>(ps, nullptr, nullptr, nullptr, n, bv.spheres[2].o);
    bv.spheres[1].o.noalias() += bv.obb.axis.col(2) * (-r10 + r11);
    bv.spheres[2].o.noalias() += bv.obb.axis.col(2) * (r10 - r12);

    bv.spheres[1].r = r10;
    bv.spheres[2].r = r10;
  }

  if(bv.num_spheres >= 5)
  {
    S r10 = bv.spheres[1].r;
    Vector3<S> delta = bv.obb.axis.col(1) * (sqrt(r10 * r10 - extent[0] * extent[0] - extent[2] * extent[2]) - extent[1]);
    bv.spheres[3].o = bv.spheres[0].o - delta;
    bv.spheres[4].o = bv.spheres[0].o + delta;

    S r21 = 0, r22 = 0;
    r21 = maximumDistance<S>(ps, nullptr, nullptr, nullptr, n, bv.spheres[3].o);
    r22 = maximumDistance<S>(ps, nullptr, nullptr, nullptr, n, bv.spheres[4].o);

    bv.spheres[3].o.noalias() += bv.obb.axis.col(1) * (-r10 + r21);
    bv.spheres[4].o.noalias() += bv.obb.axis.col(1) * (r10 - r22);

    bv.spheres[3].r = r10;
    bv.spheres[4].r = r10;
  }
}

} // namespace kIOS_fit_functions
} // namespace detail
} // namespace fcl

#endif

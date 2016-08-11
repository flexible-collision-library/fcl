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

#ifndef FCL_BV_DETAIL_FITTER_H
#define FCL_BV_DETAIL_FITTER_H

#include "fcl/BV/OBB.h"
#include "fcl/BV/RSS.h"
#include "fcl/BV/OBBRSS.h"
#include "fcl/BV/kIOS.h"

namespace fcl {
namespace detail {

//==============================================================================
template <typename Scalar, typename BV>
struct Fitter
{
  static void fit(Vector3<Scalar>* ps, int n, BV& bv)
  {
    for(int i = 0; i < n; ++i)
      bv += ps[i];
  }
};

namespace OBB_fit_functions
{

template <typename Scalar>
void fit1(Vector3<Scalar>* ps, OBB<Scalar>& bv)
{
  bv.To = ps[0];
  bv.axis.setIdentity();
  bv.extent.setZero();
}

template <typename Scalar>
void fit2(Vector3<Scalar>* ps, OBB<Scalar>& bv)
{
  const Vector3<Scalar>& p1 = ps[0];
  const Vector3<Scalar>& p2 = ps[1];
  Vector3<Scalar> p1p2 = p1 - p2;
  Scalar len_p1p2 = p1p2.norm();
  p1p2.normalize();

  bv.axis.col(0) = p1p2;
  generateCoordinateSystem(bv.axis);

  bv.extent << len_p1p2 * 0.5, 0, 0;
  bv.To.noalias() = 0.5 * (p1 + p2);
}

template <typename Scalar>
void fit3(Vector3<Scalar>* ps, OBB<Scalar>& bv)
{
  const Vector3<Scalar>& p1 = ps[0];
  const Vector3<Scalar>& p2 = ps[1];
  const Vector3<Scalar>& p3 = ps[2];
  Vector3<Scalar> e[3];
  e[0] = p1 - p2;
  e[1] = p2 - p3;
  e[2] = p3 - p1;
  Scalar len[3];
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

  getExtentAndCenter<Scalar>(ps, NULL, NULL, NULL, 3, bv.axis, bv.To, bv.extent);
}

template <typename Scalar>
void fit6(Vector3<Scalar>* ps, OBB<Scalar>& bv)
{
  OBB<Scalar> bv1, bv2;
  fit3(ps, bv1);
  fit3(ps + 3, bv2);
  bv = bv1 + bv2;
}

template <typename Scalar>
void fitn(Vector3<Scalar>* ps, int n, OBB<Scalar>& bv)
{
  Matrix3<Scalar> M;
  Matrix3<Scalar> E;
  Vector3<Scalar> s = Vector3<Scalar>::Zero(); // three eigen values

  getCovariance<Scalar>(ps, NULL, NULL, NULL, n, M);
  eigen_old(M, s, E);
  axisFromEigen(E, s, bv.axis);

  // set obb centers and extensions
  getExtentAndCenter<Scalar>(ps, NULL, NULL, NULL, n, bv.axis, bv.To, bv.extent);
}

} // namespace OBB_fit_functions


namespace RSS_fit_functions {

template <typename Scalar>
void fit1(Vector3<Scalar>* ps, RSS<Scalar>& bv)
{
  bv.To = ps[0];
  bv.axis.setIdentity();
  bv.l[0] = 0;
  bv.l[1] = 0;
  bv.r = 0;
}

template <typename Scalar>
void fit2(Vector3<Scalar>* ps, RSS<Scalar>& bv)
{
  const Vector3<Scalar>& p1 = ps[0];
  const Vector3<Scalar>& p2 = ps[1];
  Vector3<Scalar> p1p2 = p1 - p2;
  Scalar len_p1p2 = p1p2.norm();
  p1p2.normalize();

  bv.axis.col(0) = p1p2;
  generateCoordinateSystem(bv.axis);
  bv.l[0] = len_p1p2;
  bv.l[1] = 0;

  bv.To = p2;
  bv.r = 0;
}

template <typename Scalar>
void fit3(Vector3<Scalar>* ps, RSS<Scalar>& bv)
{
  const Vector3<Scalar>& p1 = ps[0];
  const Vector3<Scalar>& p2 = ps[1];
  const Vector3<Scalar>& p3 = ps[2];
  Vector3<Scalar> e[3];
  e[0] = p1 - p2;
  e[1] = p2 - p3;
  e[2] = p3 - p1;
  Scalar len[3];
  len[0] = e[0].squaredNorm();
  len[1] = e[1].squaredNorm();
  len[2] = e[2].squaredNorm();

  int imax = 0;
  if(len[1] > len[0]) imax = 1;
  if(len[2] > len[imax]) imax = 2;

  bv.axis.col(2).noalias() = e[0].cross(e[1]).normalized();
  bv.axis.col(0).noalias() = e[imax].normalized();
  bv.axis.col(1).noalias() = bv.axis.col(2).cross(bv.axis.col(0));

  getRadiusAndOriginAndRectangleSize<Scalar>(ps, NULL, NULL, NULL, 3, bv.axis, bv.To, bv.l, bv.r);
}

template <typename Scalar>
void fit6(Vector3<Scalar>* ps, RSS<Scalar>& bv)
{
  RSS<Scalar> bv1, bv2;
  fit3(ps, bv1);
  fit3(ps + 3, bv2);
  bv = bv1 + bv2;
}

template <typename Scalar>
void fitn(Vector3<Scalar>* ps, int n, RSS<Scalar>& bv)
{
  Matrix3<Scalar> M; // row first matrix
  Matrix3<Scalar> E; // row first eigen-vectors
  Vector3<Scalar> s = Vector3<Scalar>::Zero();

  getCovariance<Scalar>(ps, NULL, NULL, NULL, n, M);
  eigen_old(M, s, E);
  axisFromEigen(E, s, bv.axis);

  // set rss origin, rectangle size and radius
  getRadiusAndOriginAndRectangleSize<Scalar>(ps, NULL, NULL, NULL, n, bv.axis, bv.To, bv.l, bv.r);
}

} // namespace RSS_fit_functions

namespace kIOS_fit_functions {

template <typename Scalar>
void fit1(Vector3<Scalar>* ps, kIOS<Scalar>& bv)
{
  bv.num_spheres = 1;
  bv.spheres[0].o = ps[0];
  bv.spheres[0].r = 0;

  bv.obb.axis.setIdentity();
  bv.obb.extent.setZero();
  bv.obb.To = ps[0];
}

template <typename Scalar>
void fit2(Vector3<Scalar>* ps, kIOS<Scalar>& bv)
{
  bv.num_spheres = 5;

  const Vector3<Scalar>& p1 = ps[0];
  const Vector3<Scalar>& p2 = ps[1];
  Vector3<Scalar> p1p2 = p1 - p2;
  Scalar len_p1p2 = p1p2.norm();
  p1p2.normalize();

  bv.obb.axis.col(0) = p1p2;
  generateCoordinateSystem(bv.obb.axis);

  Scalar r0 = len_p1p2 * 0.5;
  bv.obb.extent << r0, 0, 0;
  bv.obb.To.noalias() = (p1 + p2) * 0.5;

  bv.spheres[0].o = bv.obb.To;
  bv.spheres[0].r = r0;

  Scalar r1 = r0 * kIOS<Scalar>::invSinA();
  Scalar r1cosA = r1 * kIOS<Scalar>::cosA();
  bv.spheres[1].r = r1;
  bv.spheres[2].r = r1;
  Vector3<Scalar> delta = bv.obb.axis.col(1) * r1cosA;
  bv.spheres[1].o = bv.spheres[0].o - delta;
  bv.spheres[2].o = bv.spheres[0].o + delta;

  bv.spheres[3].r = r1;
  bv.spheres[4].r = r1;
  delta.noalias() = bv.obb.axis.col(2) * r1cosA;
  bv.spheres[3].o = bv.spheres[0].o - delta;
  bv.spheres[4].o = bv.spheres[0].o + delta;
}

template <typename Scalar>
void fit3(Vector3<Scalar>* ps, kIOS<Scalar>& bv)
{
  bv.num_spheres = 3;

  const Vector3<Scalar>& p1 = ps[0];
  const Vector3<Scalar>& p2 = ps[1];
  const Vector3<Scalar>& p3 = ps[2];
  Vector3<Scalar> e[3];
  e[0] = p1 - p2;
  e[1] = p2 - p3;
  e[2] = p3 - p1;
  Scalar len[3];
  len[0] = e[0].squaredNorm();
  len[1] = e[1].squaredNorm();
  len[2] = e[2].squaredNorm();

  int imax = 0;
  if(len[1] > len[0]) imax = 1;
  if(len[2] > len[imax]) imax = 2;

  bv.obb.axis.col(2).noalias() = e[0].cross(e[1]).normalized();
  bv.obb.axis.col(0) = e[imax].normalized();
  bv.obb.axis.col(1).noalias() = bv.obb.axis.col(2).cross(bv.obb.axis.col(0));

  getExtentAndCenter<Scalar>(ps, NULL, NULL, NULL, 3, bv.obb.axis, bv.obb.To, bv.obb.extent);

  // compute radius and center
  Scalar r0;
  Vector3<Scalar> center;
  circumCircleComputation(p1, p2, p3, center, r0);

  bv.spheres[0].o = center;
  bv.spheres[0].r = r0;

  Scalar r1 = r0 * kIOS<Scalar>::invSinA();
  Vector3<Scalar> delta = bv.obb.axis.col(2) * (r1 * kIOS<Scalar>::cosA());

  bv.spheres[1].r = r1;
  bv.spheres[1].o = center - delta;
  bv.spheres[2].r = r1;
  bv.spheres[2].o = center + delta;
}

template <typename Scalar>
void fitn(Vector3<Scalar>* ps, int n, kIOS<Scalar>& bv)
{
  Matrix3<Scalar> M;
  Matrix3<Scalar> E;
  Vector3<Scalar> s = Vector3<Scalar>::Zero(); // three eigen values;

  getCovariance<Scalar>(ps, NULL, NULL, NULL, n, M);
  eigen_old(M, s, E);
  axisFromEigen(E, s, bv.obb.axis);

  getExtentAndCenter<Scalar>(ps, NULL, NULL, NULL, n, bv.obb.axis, bv.obb.To, bv.obb.extent);

  // get center and extension
  const Vector3<Scalar>& center = bv.obb.To;
  const Vector3<Scalar>& extent = bv.obb.extent;
  Scalar r0 = maximumDistance<Scalar>(ps, NULL, NULL, NULL, n, center);

  // decide the k in kIOS<Scalar>
  if(extent[0] > kIOS<Scalar>::ratio() * extent[2])
  {
    if(extent[0] > kIOS<Scalar>::ratio() * extent[1]) bv.num_spheres = 5;
    else bv.num_spheres = 3;
  }
  else bv.num_spheres = 1;


  bv.spheres[0].o = center;
  bv.spheres[0].r = r0;

  if(bv.num_spheres >= 3)
  {
    Scalar r10 = sqrt(r0 * r0 - extent[2] * extent[2]) * kIOS<Scalar>::invSinA();
    Vector3<Scalar> delta = bv.obb.axis.col(2) * (r10 * kIOS<Scalar>::cosA() - extent[2]);
    bv.spheres[1].o = center - delta;
    bv.spheres[2].o = center + delta;

    Scalar r11 = 0, r12 = 0;
    r11 = maximumDistance<Scalar>(ps, NULL, NULL, NULL, n, bv.spheres[1].o);
    r12 = maximumDistance<Scalar>(ps, NULL, NULL, NULL, n, bv.spheres[2].o);
    bv.spheres[1].o.noalias() += bv.obb.axis.col(2) * (-r10 + r11);
    bv.spheres[2].o.noalias() += bv.obb.axis.col(2) * (r10 - r12);

    bv.spheres[1].r = r10;
    bv.spheres[2].r = r10;
  }

  if(bv.num_spheres >= 5)
  {
    Scalar r10 = bv.spheres[1].r;
    Vector3<Scalar> delta = bv.obb.axis.col(1) * (sqrt(r10 * r10 - extent[0] * extent[0] - extent[2] * extent[2]) - extent[1]);
    bv.spheres[3].o = bv.spheres[0].o - delta;
    bv.spheres[4].o = bv.spheres[0].o + delta;

    Scalar r21 = 0, r22 = 0;
    r21 = maximumDistance<Scalar>(ps, NULL, NULL, NULL, n, bv.spheres[3].o);
    r22 = maximumDistance<Scalar>(ps, NULL, NULL, NULL, n, bv.spheres[4].o);

    bv.spheres[3].o.noalias() += bv.obb.axis.col(1) * (-r10 + r21);
    bv.spheres[4].o.noalias() += bv.obb.axis.col(1) * (r10 - r22);

    bv.spheres[3].r = r10;
    bv.spheres[4].r = r10;
  }
}

} // namespace kIOS_fit_functions

namespace OBBRSS_fit_functions {

template <typename Scalar>
void fit1(Vector3<Scalar>* ps, OBBRSS<Scalar>& bv)
{
  OBB_fit_functions::fit1(ps, bv.obb);
  RSS_fit_functions::fit1(ps, bv.rss);
}

template <typename Scalar>
void fit2(Vector3<Scalar>* ps, OBBRSS<Scalar>& bv)
{
  OBB_fit_functions::fit2(ps, bv.obb);
  RSS_fit_functions::fit2(ps, bv.rss);
}

template <typename Scalar>
void fit3(Vector3<Scalar>* ps, OBBRSS<Scalar>& bv)
{
  OBB_fit_functions::fit3(ps, bv.obb);
  RSS_fit_functions::fit3(ps, bv.rss);
}

template <typename Scalar>
void fitn(Vector3<Scalar>* ps, int n, OBBRSS<Scalar>& bv)
{
  OBB_fit_functions::fitn(ps, n, bv.obb);
  RSS_fit_functions::fitn(ps, n, bv.rss);
}

} // namespace OBBRSS_fit_functions

//==============================================================================
template <typename Scalar>
struct Fitter<Scalar, OBB<Scalar>>
{
  static void fit(Vector3<Scalar>* ps, int n, OBB<Scalar>& bv)
  {
    switch(n)
    {
    case 1:
      OBB_fit_functions::fit1(ps, bv);
      break;
    case 2:
      OBB_fit_functions::fit2(ps, bv);
      break;
    case 3:
      OBB_fit_functions::fit3(ps, bv);
      break;
    case 6:
      OBB_fit_functions::fit6(ps, bv);
      break;
    default:
      OBB_fit_functions::fitn(ps, n, bv);
    }
  }
};

//==============================================================================
template <typename Scalar>
struct Fitter<Scalar, RSS<Scalar>>
{
  static void fit(Vector3<Scalar>* ps, int n, RSS<Scalar>& bv)
  {
    switch(n)
    {
    case 1:
      RSS_fit_functions::fit1(ps, bv);
      break;
    case 2:
      RSS_fit_functions::fit2(ps, bv);
      break;
    case 3:
      RSS_fit_functions::fit3(ps, bv);
      break;
    default:
      RSS_fit_functions::fitn(ps, n, bv);
    }
  }
};

//==============================================================================
template <typename Scalar>
struct Fitter<Scalar, kIOS<Scalar>>
{
  static void fit(Vector3<Scalar>* ps, int n, kIOS<Scalar>& bv)
  {
    switch(n)
    {
    case 1:
      kIOS_fit_functions::fit1(ps, bv);
      break;
    case 2:
      kIOS_fit_functions::fit2(ps, bv);
      break;
    case 3:
      kIOS_fit_functions::fit3(ps, bv);
      break;
    default:
      kIOS_fit_functions::fitn(ps, n, bv);
    }
  }
};

//==============================================================================
template <typename Scalar>
struct Fitter<Scalar, OBBRSS<Scalar>>
{
  static void fit(Vector3<Scalar>* ps, int n, OBBRSS<Scalar>& bv)
  {
    switch(n)
    {
    case 1:
      OBBRSS_fit_functions::fit1(ps, bv);
      break;
    case 2:
      OBBRSS_fit_functions::fit2(ps, bv);
      break;
    case 3:
      OBBRSS_fit_functions::fit3(ps, bv);
      break;
    default:
      OBBRSS_fit_functions::fitn(ps, n, bv);
    }
  }
};

} // namespace detail
} // namespace fcl

#endif

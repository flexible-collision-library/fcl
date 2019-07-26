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

#ifndef FCL_MATH_BV_UTILITY_INL_H
#define FCL_MATH_BV_UTILITY_INL_H

#include "fcl/math/bv/utility.h"

#include "fcl/common/unused.h"

#include "fcl/math/bv/AABB.h"
#include "fcl/math/bv/kDOP.h"
#include "fcl/math/bv/kIOS.h"
#include "fcl/math/bv/OBB.h"
#include "fcl/math/bv/OBBRSS.h"
#include "fcl/math/bv/RSS.h"

/** \brief Main namespace */
namespace fcl {

//==============================================================================
namespace detail {
//==============================================================================

//==============================================================================
namespace OBB_fit_functions {
//==============================================================================

//==============================================================================
template <typename S>
FCL_EXPORT
void fit1(const Vector3<S>* const ps, OBB<S>& bv)
{
  bv.To = ps[0];
  bv.axis.setIdentity();
  bv.extent.setZero();
}

//==============================================================================
template <typename S>
FCL_EXPORT
void fit2(const Vector3<S>* const ps, OBB<S>& bv)
{
  const Vector3<S>& p1 = ps[0];
  const Vector3<S>& p2 = ps[1];
  const Vector3<S> p1p2 = p1 - p2;
  const S len_p1p2 = p1p2.norm();

  bv.axis = generateCoordinateSystem(p1p2);

  bv.extent << len_p1p2 * 0.5, 0, 0;
  bv.To.noalias() = 0.5 * (p1 + p2);
}

//==============================================================================
template <typename S>
FCL_EXPORT
void fit3(const Vector3<S>* const ps, OBB<S>& bv)
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
FCL_EXPORT
void fit6(const Vector3<S>* const ps, OBB<S>& bv)
{
  OBB<S> bv1, bv2;
  fit3(ps, bv1);
  fit3(ps + 3, bv2);
  bv = bv1 + bv2;
}

//==============================================================================
template <typename S>
FCL_EXPORT
void fitn(const Vector3<S>* const ps, int n, OBB<S>& bv)
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

//==============================================================================
extern template
void fit1(const Vector3d* const ps, OBB<double>& bv);

//==============================================================================
extern template
void fit2(const Vector3d* const ps, OBB<double>& bv);

//==============================================================================
extern template
void fit3(const Vector3d* const ps, OBB<double>& bv);

//==============================================================================
extern template
void fit6(const Vector3d* const ps, OBB<double>& bv);

//==============================================================================
extern template
void fitn(const Vector3d* const ps, int n, OBB<double>& bv);

//==============================================================================
} // namespace OBB_fit_functions
//==============================================================================

//==============================================================================
namespace RSS_fit_functions {
//==============================================================================

//==============================================================================
template <typename S>
FCL_EXPORT
void fit1(const Vector3<S>* const ps, RSS<S>& bv)
{
  bv.To = ps[0];
  bv.axis.setIdentity();
  bv.l[0] = 0;
  bv.l[1] = 0;
  bv.r = 0;
}

//==============================================================================
template <typename S>
FCL_EXPORT
void fit2(const Vector3<S>* const ps, RSS<S>& bv)
{
  const Vector3<S>& p1 = ps[0];
  const Vector3<S>& p2 = ps[1];
  const Vector3<S> p1p2 = p1 - p2;
  const S len_p1p2 = p1p2.norm();

  bv.axis = generateCoordinateSystem(p1p2);
  bv.l[0] = len_p1p2;
  bv.l[1] = 0;

  bv.To = p2;
  bv.r = 0;
}

//==============================================================================
template <typename S>
FCL_EXPORT
void fit3(const Vector3<S>* const ps, RSS<S>& bv)
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

  bv.axis.col(2).noalias() = e[0].cross(e[1]).normalized();
  bv.axis.col(0).noalias() = e[imax].normalized();
  bv.axis.col(1).noalias() = bv.axis.col(2).cross(bv.axis.col(0));

  getRadiusAndOriginAndRectangleSize<S>(ps, nullptr, nullptr, nullptr, 3, bv.axis, bv.To, bv.l, bv.r);
}

//==============================================================================
template <typename S>
FCL_EXPORT
void fit6(const Vector3<S>* const ps, RSS<S>& bv)
{
  RSS<S> bv1, bv2;
  fit3(ps, bv1);
  fit3(ps + 3, bv2);
  bv = bv1 + bv2;
}

//==============================================================================
template <typename S>
FCL_EXPORT
void fitn(const Vector3<S>* const ps, int n, RSS<S>& bv)
{
  Matrix3<S> M; // row first matrix
  Matrix3<S> E; // row first eigen-vectors
  Vector3<S> s = Vector3<S>::Zero();

  getCovariance<S>(ps, nullptr, nullptr, nullptr, n, M);
  eigen_old(M, s, E);
  axisFromEigen(E, s, bv.axis);

  // set rss origin, rectangle size and radius
  getRadiusAndOriginAndRectangleSize<S>(ps, nullptr, nullptr, nullptr, n, bv.axis, bv.To, bv.l, bv.r);
}

//==============================================================================
extern template
FCL_EXPORT
void fit1(const Vector3d* const ps, RSS<double>& bv);

//==============================================================================
extern template
FCL_EXPORT
void fit2(const Vector3d* const ps, RSS<double>& bv);

//==============================================================================
extern template
FCL_EXPORT
void fit3(const Vector3d* const ps, RSS<double>& bv);

//==============================================================================
extern template
FCL_EXPORT
void fit6(const Vector3d* const ps, RSS<double>& bv);

//==============================================================================
extern template
FCL_EXPORT
void fitn(const Vector3d* const ps, int n, RSS<double>& bv);

//==============================================================================
} // namespace RSS_fit_functions
//==============================================================================

//==============================================================================
namespace kIOS_fit_functions {
//==============================================================================

//==============================================================================
template <typename S>
FCL_EXPORT
void fit1(const Vector3<S>* const ps, kIOS<S>& bv)
{
  bv.num_spheres = 1;
  bv.spheres[0].o = ps[0];
  bv.spheres[0].r = 0;

  bv.obb.axis.setIdentity();
  bv.obb.extent.setZero();
  bv.obb.To = ps[0];
}

//==============================================================================
template <typename S>
FCL_EXPORT
void fit2(const Vector3<S>* const ps, kIOS<S>& bv)
{
  bv.num_spheres = 5;

  const Vector3<S>& p1 = ps[0];
  const Vector3<S>& p2 = ps[1];
  const Vector3<S> p1p2 = p1 - p2;
  const S len_p1p2 = p1p2.norm();

  bv.obb.axis = generateCoordinateSystem(p1p2);

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

//==============================================================================
template <typename S>
FCL_EXPORT
void fit3(const Vector3<S>* const ps, kIOS<S>& bv)
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

//==============================================================================
template <typename S>
FCL_EXPORT
void fitn(const Vector3<S>* const ps, int n, kIOS<S>& bv)
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

//==============================================================================
extern template
FCL_EXPORT
void fit1(const Vector3d* const ps, kIOS<double>& bv);

//==============================================================================
extern template
FCL_EXPORT
void fit2(const Vector3d* const ps, kIOS<double>& bv);

//==============================================================================
extern template
FCL_EXPORT
void fit3(const Vector3d* const ps, kIOS<double>& bv);

//==============================================================================
extern template
FCL_EXPORT
void fitn(const Vector3d* const ps, int n, kIOS<double>& bv);

//==============================================================================
} // namespace kIOS_fit_functions
//==============================================================================

//==============================================================================
namespace OBBRSS_fit_functions {
//==============================================================================

//==============================================================================
template <typename S>
FCL_EXPORT
void fit1(const Vector3<S>* const ps, OBBRSS<S>& bv)
{
  OBB_fit_functions::fit1(ps, bv.obb);
  RSS_fit_functions::fit1(ps, bv.rss);
}

//==============================================================================
template <typename S>
FCL_EXPORT
void fit2(const Vector3<S>* const ps, OBBRSS<S>& bv)
{
  OBB_fit_functions::fit2(ps, bv.obb);
  RSS_fit_functions::fit2(ps, bv.rss);
}

//==============================================================================
template <typename S>
FCL_EXPORT
void fit3(const Vector3<S>* const ps, OBBRSS<S>& bv)
{
  OBB_fit_functions::fit3(ps, bv.obb);
  RSS_fit_functions::fit3(ps, bv.rss);
}

//==============================================================================
template <typename S>
FCL_EXPORT
void fitn(const Vector3<S>* const ps, int n, OBBRSS<S>& bv)
{
  OBB_fit_functions::fitn(ps, n, bv.obb);
  RSS_fit_functions::fitn(ps, n, bv.rss);
}

//==============================================================================
extern template
void fit1(const Vector3d* const ps, OBBRSS<double>& bv);

//==============================================================================
extern template
void fit2(const Vector3d* const ps, OBBRSS<double>& bv);

//==============================================================================
extern template
void fit3(const Vector3d* const ps, OBBRSS<double>& bv);

//==============================================================================
extern template
void fitn(const Vector3d* const ps, int n, OBBRSS<double>& bv);

//==============================================================================
} // namespace OBBRSS_fit_functions
//==============================================================================

//==============================================================================
template <typename S, typename BV>
struct FCL_EXPORT Fitter
{
  static void fit(const Vector3<S>* const ps, int n, BV& bv)
  {
    for(int i = 0; i < n; ++i)
      bv += ps[i];
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT Fitter<S, OBB<S>>
{
  static void fit(const Vector3<S>* const ps, int n, OBB<S>& bv)
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
template <typename S>
struct FCL_EXPORT Fitter<S, RSS<S>>
{
  static void fit(const Vector3<S>* const ps, int n, RSS<S>& bv)
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
template <typename S>
struct FCL_EXPORT Fitter<S, kIOS<S>>
{
  static void fit(const Vector3<S>* const ps, int n, kIOS<S>& bv)
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
template <typename S>
struct FCL_EXPORT Fitter<S, OBBRSS<S>>
{
  static void fit(const Vector3<S>* const ps, int n, OBBRSS<S>& bv)
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

//==============================================================================
extern template
struct Fitter<double, OBB<double>>;

//==============================================================================
extern template
struct Fitter<double, RSS<double>>;

//==============================================================================
extern template
struct Fitter<double, kIOS<double>>;

//==============================================================================
extern template
struct Fitter<double, OBBRSS<double>>;

//==============================================================================
} // namespace detail
//==============================================================================

//==============================================================================
template <typename BV>
FCL_EXPORT
void fit(const Vector3<typename BV::S>* const ps, int n, BV& bv)
{
  detail::Fitter<typename BV::S, BV>::fit(ps, n, bv);
}

//==============================================================================
namespace detail {
//==============================================================================

/// @brief Convert a bounding volume of type BV1 in configuration tf1 to a
/// bounding volume of type BV2 in I configuration.
template <typename S, typename BV1, typename BV2>
class FCL_EXPORT ConvertBVImpl
{
private:
  static void run(const BV1& bv1, const Transform3<S>& tf1, BV2& bv2)
  {
    FCL_UNUSED(bv1);
    FCL_UNUSED(tf1);
    FCL_UNUSED(bv2);

    // should only use the specialized version, so it is private.
  }
};

//==============================================================================
/// @brief Convert from AABB to AABB, not very tight but is fast.
template <typename S>
class FCL_EXPORT ConvertBVImpl<S, AABB<S>, AABB<S>>
{
public:
  static void run(const AABB<S>& bv1, const Transform3<S>& tf1, AABB<S>& bv2)
  {
    const Vector3<S> center = bv1.center();
    S r = (bv1.max_ - bv1.min_).norm() * 0.5;
    Vector3<S> center2 = tf1 * center;
    Vector3<S> delta(r, r, r);
    bv2.min_ = center2 - delta;
    bv2.max_ = center2 + delta;
  }
};

//==============================================================================
template <typename S>
class FCL_EXPORT ConvertBVImpl<S, AABB<S>, OBB<S>>
{
public:
  static void run(const AABB<S>& bv1, const Transform3<S>& tf1, OBB<S>& bv2)
  {
    /*
      bv2.To = tf1 * bv1.center());

      /// Sort the AABB edges so that AABB extents are ordered.
      S d[3] = {bv1.width(), bv1.height(), bv1.depth() };
      std::size_t id[3] = {0, 1, 2};

      for(std::size_t i = 1; i < 3; ++i)
      {
        for(std::size_t j = i; j > 0; --j)
        {
          if(d[j] > d[j-1])
          {
            {
              S tmp = d[j];
              d[j] = d[j-1];
              d[j-1] = tmp;
            }
            {
              std::size_t tmp = id[j];
              id[j] = id[j-1];
              id[j-1] = tmp;
            }
          }
        }
      }

      Vector3<S> extent = (bv1.max_ - bv1.min_) * 0.5;
      bv2.extent = Vector3<S>(extent[id[0]], extent[id[1]], extent[id[2]]);
      const Matrix3<S>& R = tf1.linear();
      bool left_hand = (id[0] == (id[1] + 1) % 3);
      bv2.axis[0] = left_hand ? -R.col(id[0]) : R.col(id[0]);
      bv2.axis[1] = R.col(id[1]);
      bv2.axis[2] = R.col(id[2]);
      */

    bv2.To.noalias() = tf1 * bv1.center();
    bv2.extent.noalias() = (bv1.max_ - bv1.min_) * 0.5;
    bv2.axis = tf1.linear();
  }
};

//==============================================================================
template <typename S>
class FCL_EXPORT ConvertBVImpl<S, OBB<S>, OBB<S>>
{
public:
  static void run(const OBB<S>& bv1, const Transform3<S>& tf1, OBB<S>& bv2)
  {
    bv2.extent = bv1.extent;
    bv2.To.noalias() = tf1 * bv1.To;
    bv2.axis.noalias() = tf1.linear() * bv1.axis;
  }
};

//==============================================================================
template <typename S>
class FCL_EXPORT ConvertBVImpl<S, OBBRSS<S>, OBB<S>>
{
public:
  static void run(const OBBRSS<S>& bv1, const Transform3<S>& tf1, OBB<S>& bv2)
  {
    ConvertBVImpl<S, OBB<S>, OBB<S>>::run(bv1.obb, tf1, bv2);
  }
};

//==============================================================================
template <typename S>
class FCL_EXPORT ConvertBVImpl<S, RSS<S>, OBB<S>>
{
public:
  static void run(const RSS<S>& bv1, const Transform3<S>& tf1, OBB<S>& bv2)
  {
    bv2.extent << bv1.l[0] * 0.5 + bv1.r, bv1.l[1] * 0.5 + bv1.r, bv1.r;
    bv2.To.noalias() = tf1 * bv1.center();
    bv2.axis.noalias() = tf1.linear() * bv1.axis;
  }
};

//==============================================================================
template <typename S, typename BV1>
class FCL_EXPORT ConvertBVImpl<S, BV1, AABB<S>>
{
public:
  static void run(const BV1& bv1, const Transform3<S>& tf1, AABB<S>& bv2)
  {
    const Vector3<S> center = bv1.center();
    S r = Vector3<S>(bv1.width(), bv1.height(), bv1.depth()).norm() * 0.5;
    Vector3<S> delta(r, r, r);
    Vector3<S> center2 = tf1 * center;
    bv2.min_ = center2 - delta;
    bv2.max_ = center2 + delta;
  }
};

//==============================================================================
template <typename S, typename BV1>
class FCL_EXPORT ConvertBVImpl<S, BV1, OBB<S>>
{
public:
  static void run(const BV1& bv1, const Transform3<S>& tf1, OBB<S>& bv2)
  {
    AABB<S> bv;
    ConvertBVImpl<S, BV1, AABB<S>>::run(bv1, Transform3<S>::Identity(), bv);
    ConvertBVImpl<S, AABB<S>, OBB<S>>::run(bv, tf1, bv2);
  }
};

//==============================================================================
template <typename S>
class FCL_EXPORT ConvertBVImpl<S, OBB<S>, RSS<S>>
{
public:
  static void run(const OBB<S>& bv1, const Transform3<S>& tf1, RSS<S>& bv2)
  {
    // OBB's rotation matrix in axis is required to be lined up with the
    // x-axis along the longest edge, the y-axis on the next longest edge
    // and the z-axis on the shortest edge. RSS requires the longest edge
    // of the rectangle to be the x-axis and the next longest the y-axis.
    // This maps perfectly from OBB to RSS so simply transform the rotation
    // axis of the OBB into the RSS.
    bv2.axis.noalias() = tf1.linear() * bv1.axis;

    // Set longest rectangle side for RSS to longest dimension of OBB.
    bv2.l[0] = 2 * (bv1.extent[0]);
    // Set shortest rectangle side for RSS to next-longest dimension of OBB.
    bv2.l[1] = 2 * (bv1.extent[1]);
    // Set radius for RSS to the smallest dimension of OBB.
    bv2.r = bv1.extent[2];

    // OBB's To is at its center while RSS's To is at a corner.
    bv2.setToFromCenter(tf1 * bv1.center());
  }
};

//==============================================================================
template <typename S>
class FCL_EXPORT ConvertBVImpl<S, RSS<S>, RSS<S>>
{
public:
  static void run(const RSS<S>& bv1, const Transform3<S>& tf1, RSS<S>& bv2)
  {
    bv2.To.noalias() = tf1 * bv1.To;
    bv2.axis.noalias() = tf1.linear() * bv1.axis;

    bv2.r = bv1.r;
    bv2.l[0] = bv1.l[0];
    bv2.l[1] = bv1.l[1];
  }
};

//==============================================================================
template <typename S>
class FCL_EXPORT ConvertBVImpl<S, OBBRSS<S>, RSS<S>>
{
public:
  static void run(const OBBRSS<S>& bv1, const Transform3<S>& tf1, RSS<S>& bv2)
  {
    ConvertBVImpl<S, RSS<S>, RSS<S>>::run(bv1.rss, tf1, bv2);
  }
};

//==============================================================================
template <typename S>
class FCL_EXPORT ConvertBVImpl<S, AABB<S>, RSS<S>>
{
public:
  static void run(const AABB<S>& bv1, const Transform3<S>& tf1, RSS<S>& bv2)
  {
    /// Sort the AABB edges so that AABB extents are ordered.
    S d[3] = {bv1.width(), bv1.height(), bv1.depth() };
    std::size_t id[3] = {0, 1, 2};

    for(std::size_t i = 1; i < 3; ++i)
    {
      for(std::size_t j = i; j > 0; --j)
      {
        if(d[j] > d[j-1])
        {
          {
            S tmp = d[j];
            d[j] = d[j-1];
            d[j-1] = tmp;
          }
          {
            std::size_t tmp = id[j];
            id[j] = id[j-1];
            id[j-1] = tmp;
          }
        }
      }
    }

    Vector3<S> extent = (bv1.max_ - bv1.min_) * 0.5;
    bv2.r = extent[id[2]];
    bv2.l[0] = (extent[id[0]]) * 2;
    bv2.l[1] = (extent[id[1]]) * 2;

    const Matrix3<S>& R = tf1.linear();
    bool left_hand = (id[0] == (id[1] + 1) % 3);
    if (left_hand)
      bv2.axis.col(0) = -R.col(id[0]);
    else
      bv2.axis.col(0) = R.col(id[0]);
    bv2.axis.col(1) = R.col(id[1]);
    bv2.axis.col(2) = R.col(id[2]);
    bv2.setToFromCenter(tf1 * bv1.center());
  }
};

//==============================================================================
extern template
class FCL_EXPORT ConvertBVImpl<double, AABB<double>, AABB<double>>;

//==============================================================================
extern template
class FCL_EXPORT ConvertBVImpl<double, AABB<double>, OBB<double>>;

//==============================================================================
extern template
class FCL_EXPORT ConvertBVImpl<double, OBB<double>, OBB<double>>;

//==============================================================================
extern template
class FCL_EXPORT ConvertBVImpl<double, OBBRSS<double>, OBB<double>>;

//==============================================================================
extern template
class FCL_EXPORT ConvertBVImpl<double, RSS<double>, OBB<double>>;

//==============================================================================
extern template
class FCL_EXPORT ConvertBVImpl<double, OBB<double>, RSS<double>>;

//==============================================================================
extern template
class FCL_EXPORT ConvertBVImpl<double, RSS<double>, RSS<double>>;

//==============================================================================
extern template
class FCL_EXPORT ConvertBVImpl<double, OBBRSS<double>, RSS<double>>;

//==============================================================================
extern template
class FCL_EXPORT ConvertBVImpl<double, AABB<double>, RSS<double>>;

//==============================================================================
} // namespace detail
//==============================================================================

//==============================================================================
template <typename BV1, typename BV2>
FCL_EXPORT
void convertBV(
    const BV1& bv1, const Transform3<typename BV1::S>& tf1, BV2& bv2)
{
  static_assert(std::is_same<typename BV1::S, typename BV2::S>::value,
                "The scalar type of BV1 and BV2 should be the same");

  detail::ConvertBVImpl<typename BV1::S, BV1, BV2>::run(bv1, tf1, bv2);
}

} // namespace fcl

#endif

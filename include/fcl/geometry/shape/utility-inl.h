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

#ifndef FCL_GEOMETRY_SHAPE_UTILITY_INL_H
#define FCL_GEOMETRY_SHAPE_UTILITY_INL_H

#include "fcl/geometry/shape/utility.h"

#include "fcl/common/unused.h"

#include "fcl/math/bv/utility.h"

#include "fcl/geometry/shape/capsule.h"
#include "fcl/geometry/shape/cone.h"
#include "fcl/geometry/shape/convex.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/geometry/shape/ellipsoid.h"
#include "fcl/geometry/shape/halfspace.h"
#include "fcl/geometry/shape/plane.h"
#include "fcl/geometry/shape/sphere.h"
#include "fcl/geometry/shape/triangle_p.h"

namespace fcl {

//==============================================================================
extern template
void constructBox(const OBB<double>& bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
extern template
void constructBox(const OBBRSS<double>& bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
extern template
void constructBox(const kIOS<double>& bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
extern template
void constructBox(const RSS<double>& bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
extern template
void constructBox(const KDOP<double, 16>& bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
extern template
void constructBox(const KDOP<double, 18>& bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
extern template
void constructBox(const KDOP<double, 24>& bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
extern template
void constructBox(const AABB<double>& bv, const Transform3<double>& tf_bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
extern template
void constructBox(const OBB<double>& bv, const Transform3<double>& tf_bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
extern template
void constructBox(const OBBRSS<double>& bv, const Transform3<double>& tf_bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
extern template
void constructBox(const kIOS<double>& bv, const Transform3<double>& tf_bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
extern template
void constructBox(const RSS<double>& bv, const Transform3<double>& tf_bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
extern template
void constructBox(const KDOP<double, 16>& bv, const Transform3<double>& tf_bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
extern template
void constructBox(const KDOP<double, 18>& bv, const Transform3<double>& tf_bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
extern template
void constructBox(const KDOP<double, 24>& bv, const Transform3<double>& tf_bv, Box<double>& box, Transform3<double>& tf);

//==============================================================================
namespace detail {
//==============================================================================

//==============================================================================
template <typename S, typename BV, typename Shape>
struct FCL_EXPORT ComputeBVImpl
{
  static void run(const Shape& s, const Transform3<S>& tf, BV& bv)
  {
    std::vector<Vector3<S>> convex_bound_vertices = s.getBoundVertices(tf);
    fit(convex_bound_vertices.data(),
        static_cast<int>(convex_bound_vertices.size()), bv);
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, AABB<S>, Box<S>>
{
  static void run(const Box<S>& s, const Transform3<S>& tf, AABB<S>& bv)
  {
    const Matrix3<S>& R = tf.linear();
    const Vector3<S>& T = tf.translation();

    S x_range = 0.5 * (fabs(R(0, 0) * s.side[0]) + fabs(R(0, 1) * s.side[1]) + fabs(R(0, 2) * s.side[2]));
    S y_range = 0.5 * (fabs(R(1, 0) * s.side[0]) + fabs(R(1, 1) * s.side[1]) + fabs(R(1, 2) * s.side[2]));
    S z_range = 0.5 * (fabs(R(2, 0) * s.side[0]) + fabs(R(2, 1) * s.side[1]) + fabs(R(2, 2) * s.side[2]));

    Vector3<S> v_delta(x_range, y_range, z_range);
    bv.max_ = T + v_delta;
    bv.min_ = T - v_delta;
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, OBB<S>, Box<S>>
{
  static void run(const Box<S>& s, const Transform3<S>& tf, OBB<S>& bv)
  {
    bv.axis = tf.linear();
    bv.To = tf.translation();
    bv.extent = s.side * (S)0.5;
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, AABB<S>, Capsule<S>>
{
  static void run(const Capsule<S>& s, const Transform3<S>& tf, AABB<S>& bv)
  {
    const Matrix3<S>& R = tf.linear();
    const Vector3<S>& T = tf.translation();

    S x_range = 0.5 * fabs(R(0, 2) * s.lz) + s.radius;
    S y_range = 0.5 * fabs(R(1, 2) * s.lz) + s.radius;
    S z_range = 0.5 * fabs(R(2, 2) * s.lz) + s.radius;

    Vector3<S> v_delta(x_range, y_range, z_range);
    bv.max_ = T + v_delta;
    bv.min_ = T - v_delta;
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, OBB<S>, Capsule<S>>
{
  static void run(const Capsule<S>& s, const Transform3<S>& tf, OBB<S>& bv)
  {
    bv.axis = tf.linear();
    bv.To = tf.translation();
    bv.extent << s.radius, s.radius, s.lz / 2 + s.radius;
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, AABB<S>, Cone<S>>
{
  static void run(const Cone<S>& s, const Transform3<S>& tf, AABB<S>& bv)
  {
    const Matrix3<S>& R = tf.linear();
    const Vector3<S>& T = tf.translation();

    S x_range = fabs(R(0, 0) * s.radius) + fabs(R(0, 1) * s.radius) + 0.5 * fabs(R(0, 2) * s.lz);
    S y_range = fabs(R(1, 0) * s.radius) + fabs(R(1, 1) * s.radius) + 0.5 * fabs(R(1, 2) * s.lz);
    S z_range = fabs(R(2, 0) * s.radius) + fabs(R(2, 1) * s.radius) + 0.5 * fabs(R(2, 2) * s.lz);

    Vector3<S> v_delta(x_range, y_range, z_range);
    bv.max_ = T + v_delta;
    bv.min_ = T - v_delta;
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, OBB<S>, Cone<S>>
{
  static void run(const Cone<S>& s, const Transform3<S>& tf, OBB<S>& bv)
  {
    bv.axis = tf.linear();
    bv.To = tf.translation();
    bv.extent << s.radius, s.radius, s.lz / 2;
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, AABB<S>, Convex<S>>
{
  static void run(const Convex<S>& s, const Transform3<S>& tf, AABB<S>& bv)
  {
    const Matrix3<S>& R = tf.linear();
    const Vector3<S>& T = tf.translation();

    AABB<S> bv_;
    for (const auto& vertex : s.getVertices())
    {
      Vector3<S> new_p = R * vertex + T;
      bv_ += new_p;
    }

    bv = bv_;
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, OBB<S>, Convex<S>>
{
  static void run(const Convex<S>& s, const Transform3<S>& tf, OBB<S>& bv)
  {
    fit(s.getVertices().data(), static_cast<int>(s.getVertices().size()), bv);

    bv.axis = tf.linear();
    bv.To = tf * bv.To;
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, AABB<S>, Cylinder<S>>
{
  static void run(const Cylinder<S>& s, const Transform3<S>& tf, AABB<S>& bv)
  {
    const Matrix3<S>& R = tf.linear();
    const Vector3<S>& T = tf.translation();

    S x_range = fabs(R(0, 0) * s.radius) + fabs(R(0, 1) * s.radius) + 0.5 * fabs(R(0, 2) * s.lz);
    S y_range = fabs(R(1, 0) * s.radius) + fabs(R(1, 1) * s.radius) + 0.5 * fabs(R(1, 2) * s.lz);
    S z_range = fabs(R(2, 0) * s.radius) + fabs(R(2, 1) * s.radius) + 0.5 * fabs(R(2, 2) * s.lz);

    Vector3<S> v_delta(x_range, y_range, z_range);
    bv.max_ = T + v_delta;
    bv.min_ = T - v_delta;
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, OBB<S>, Cylinder<S>>
{
  static void run(const Cylinder<S>& s, const Transform3<S>& tf, OBB<S>& bv)
  {
    bv.axis = tf.linear();
    bv.To = tf.translation();
    bv.extent << s.radius, s.radius, s.lz / 2;
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, AABB<S>, Ellipsoid<S>>
{
  static void run(const Ellipsoid<S>& s, const Transform3<S>& tf, AABB<S>& bv)
  {
    const Matrix3<S>& R = tf.linear();
    const Vector3<S>& T = tf.translation();

    S x_range = (fabs(R(0, 0) * s.radii[0]) + fabs(R(0, 1) * s.radii[1]) + fabs(R(0, 2) * s.radii[2]));
    S y_range = (fabs(R(1, 0) * s.radii[0]) + fabs(R(1, 1) * s.radii[1]) + fabs(R(1, 2) * s.radii[2]));
    S z_range = (fabs(R(2, 0) * s.radii[0]) + fabs(R(2, 1) * s.radii[1]) + fabs(R(2, 2) * s.radii[2]));

    Vector3<S> v_delta(x_range, y_range, z_range);
    bv.max_ = T + v_delta;
    bv.min_ = T - v_delta;
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, OBB<S>, Ellipsoid<S>>
{
  static void run(const Ellipsoid<S>& s, const Transform3<S>& tf, OBB<S>& bv)
  {
    bv.axis = tf.linear();
    bv.To = tf.translation();
    bv.extent = s.radii;
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, AABB<S>, Halfspace<S>>
{
  static void run(const Halfspace<S>& s, const Transform3<S>& tf, AABB<S>& bv)
  {
    Halfspace<S> new_s = transform(s, tf);
    const Vector3<S>& n = new_s.n;
    const S& d = new_s.d;

    AABB<S> bv_;
    bv_.min_ = Vector3<S>::Constant(-std::numeric_limits<S>::max());
    bv_.max_ = Vector3<S>::Constant(std::numeric_limits<S>::max());
    if(n[1] == (S)0.0 && n[2] == (S)0.0)
    {
      // normal aligned with x axis
      if(n[0] < 0) bv_.min_[0] = -d;
      else if(n[0] > 0) bv_.max_[0] = d;
    }
    else if(n[0] == (S)0.0 && n[2] == (S)0.0)
    {
      // normal aligned with y axis
      if(n[1] < 0) bv_.min_[1] = -d;
      else if(n[1] > 0) bv_.max_[1] = d;
    }
    else if(n[0] == (S)0.0 && n[1] == (S)0.0)
    {
      // normal aligned with z axis
      if(n[2] < 0) bv_.min_[2] = -d;
      else if(n[2] > 0) bv_.max_[2] = d;
    }

    bv = bv_;
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, OBB<S>, Halfspace<S>>
{
  static void run(const Halfspace<S>& s, const Transform3<S>& tf, OBB<S>& bv)
  {
    FCL_UNUSED(s);
    FCL_UNUSED(tf);

    /// Half space can only have very rough OBB
    bv.axis.setIdentity();
    bv.To.setZero();
    bv.extent.setConstant(std::numeric_limits<S>::max());
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, RSS<S>, Halfspace<S>>
{
  static void run(const Halfspace<S>& s, const Transform3<S>& tf, RSS<S>& bv)
  {
    FCL_UNUSED(s);
    FCL_UNUSED(tf);

    /// Half space can only have very rough RSS
    bv.axis.setIdentity();
    bv.To.setZero();
    bv.l[0] = bv.l[1] = bv.r = std::numeric_limits<S>::max();
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, OBBRSS<S>, Halfspace<S>>
{
  static void run(const Halfspace<S>& s, const Transform3<S>& tf, OBBRSS<S>& bv)
  {
    computeBV(s, tf, bv.obb);
    computeBV(s, tf, bv.rss);
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, kIOS<S>, Halfspace<S>>
{
  static void run(const Halfspace<S>& s, const Transform3<S>& tf, kIOS<S>& bv)
  {
    bv.num_spheres = 1;
    computeBV(s, tf, bv.obb);
    bv.spheres[0].o.setZero();
    bv.spheres[0].r = std::numeric_limits<S>::max();
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, KDOP<S, 16>, Halfspace<S>>
{
  static void run(const Halfspace<S>& s, const Transform3<S>& tf, KDOP<S, 16>& bv)
  {
    Halfspace<S> new_s = transform(s, tf);
    const Vector3<S>& n = new_s.n;
    const S& d = new_s.d;

    const std::size_t D = 8;
    for(std::size_t i = 0; i < D; ++i)
      bv.dist(i) = -std::numeric_limits<S>::max();
    for(std::size_t i = D; i < 2 * D; ++i)
      bv.dist(i) = std::numeric_limits<S>::max();

    if(n[1] == (S)0.0 && n[2] == (S)0.0)
    {
      if(n[0] > 0) bv.dist(D) = d;
      else bv.dist(0) = -d;
    }
    else if(n[0] == (S)0.0 && n[2] == (S)0.0)
    {
      if(n[1] > 0) bv.dist(D + 1) = d;
      else bv.dist(1) = -d;
    }
    else if(n[0] == (S)0.0 && n[1] == (S)0.0)
    {
      if(n[2] > 0) bv.dist(D + 2) = d;
      else bv.dist(2) = -d;
    }
    else if(n[2] == (S)0.0 && n[0] == n[1])
    {
      if(n[0] > 0) bv.dist(D + 3) = n[0] * d * 2;
      else bv.dist(3) = n[0] * d * 2;
    }
    else if(n[1] == (S)0.0 && n[0] == n[2])
    {
      if(n[1] > 0) bv.dist(D + 4) = n[0] * d * 2;
      else bv.dist(4) = n[0] * d * 2;
    }
    else if(n[0] == (S)0.0 && n[1] == n[2])
    {
      if(n[1] > 0) bv.dist(D + 5) = n[1] * d * 2;
      else bv.dist(5) = n[1] * d * 2;
    }
    else if(n[2] == (S)0.0 && n[0] + n[1] == (S)0.0)
    {
      if(n[0] > 0) bv.dist(D + 6) = n[0] * d * 2;
      else bv.dist(6) = n[0] * d * 2;
    }
    else if(n[1] == (S)0.0 && n[0] + n[2] == (S)0.0)
    {
      if(n[0] > 0) bv.dist(D + 7) = n[0] * d * 2;
      else bv.dist(7) = n[0] * d * 2;
    }
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, KDOP<S, 18>, Halfspace<S>>
{
  static void run(const Halfspace<S>& s, const Transform3<S>& tf, KDOP<S, 18>& bv)
  {
    Halfspace<S> new_s = transform(s, tf);
    const Vector3<S>& n = new_s.n;
    const S& d = new_s.d;

    const std::size_t D = 9;

    for(std::size_t i = 0; i < D; ++i)
      bv.dist(i) = -std::numeric_limits<S>::max();
    for(std::size_t i = D; i < 2 * D; ++i)
      bv.dist(i) = std::numeric_limits<S>::max();

    if(n[1] == (S)0.0 && n[2] == (S)0.0)
    {
      if(n[0] > 0) bv.dist(D) = d;
      else bv.dist(0) = -d;
    }
    else if(n[0] == (S)0.0 && n[2] == (S)0.0)
    {
      if(n[1] > 0) bv.dist(D + 1) = d;
      else bv.dist(1) = -d;
    }
    else if(n[0] == (S)0.0 && n[1] == (S)0.0)
    {
      if(n[2] > 0) bv.dist(D + 2) = d;
      else bv.dist(2) = -d;
    }
    else if(n[2] == (S)0.0 && n[0] == n[1])
    {
      if(n[0] > 0) bv.dist(D + 3) = n[0] * d * 2;
      else bv.dist(3) = n[0] * d * 2;
    }
    else if(n[1] == (S)0.0 && n[0] == n[2])
    {
      if(n[1] > 0) bv.dist(D + 4) = n[0] * d * 2;
      else bv.dist(4) = n[0] * d * 2;
    }
    else if(n[0] == (S)0.0 && n[1] == n[2])
    {
      if(n[1] > 0) bv.dist(D + 5) = n[1] * d * 2;
      else bv.dist(5) = n[1] * d * 2;
    }
    else if(n[2] == (S)0.0 && n[0] + n[1] == (S)0.0)
    {
      if(n[0] > 0) bv.dist(D + 6) = n[0] * d * 2;
      else bv.dist(6) = n[0] * d * 2;
    }
    else if(n[1] == (S)0.0 && n[0] + n[2] == (S)0.0)
    {
      if(n[0] > 0) bv.dist(D + 7) = n[0] * d * 2;
      else bv.dist(7) = n[0] * d * 2;
    }
    else if(n[0] == (S)0.0 && n[1] + n[2] == (S)0.0)
    {
      if(n[1] > 0) bv.dist(D + 8) = n[1] * d * 2;
      else bv.dist(8) = n[1] * d * 2;
    }
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, KDOP<S, 24>, Halfspace<S>>
{
  static void run(const Halfspace<S>& s, const Transform3<S>& tf, KDOP<S, 24>& bv)
  {
    Halfspace<S> new_s = transform(s, tf);
    const Vector3<S>& n = new_s.n;
    const S& d = new_s.d;

    const std::size_t D = 12;

    for(std::size_t i = 0; i < D; ++i)
      bv.dist(i) = -std::numeric_limits<S>::max();
    for(std::size_t i = D; i < 2 * D; ++i)
      bv.dist(i) = std::numeric_limits<S>::max();

    if(n[1] == (S)0.0 && n[2] == (S)0.0)
    {
      if(n[0] > 0) bv.dist(D) = d;
      else bv.dist(0) = -d;
    }
    else if(n[0] == (S)0.0 && n[2] == (S)0.0)
    {
      if(n[1] > 0) bv.dist(D + 1) = d;
      else bv.dist(1) = -d;
    }
    else if(n[0] == (S)0.0 && n[1] == (S)0.0)
    {
      if(n[2] > 0) bv.dist(D + 2) = d;
      else bv.dist(2) = -d;
    }
    else if(n[2] == (S)0.0 && n[0] == n[1])
    {
      if(n[0] > 0) bv.dist(D + 3) = n[0] * d * 2;
      else bv.dist(3) = n[0] * d * 2;
    }
    else if(n[1] == (S)0.0 && n[0] == n[2])
    {
      if(n[1] > 0) bv.dist(D + 4) = n[0] * d * 2;
      else bv.dist(4) = n[0] * d * 2;
    }
    else if(n[0] == (S)0.0 && n[1] == n[2])
    {
      if(n[1] > 0) bv.dist(D + 5) = n[1] * d * 2;
      else bv.dist(5) = n[1] * d * 2;
    }
    else if(n[2] == (S)0.0 && n[0] + n[1] == (S)0.0)
    {
      if(n[0] > 0) bv.dist(D + 6) = n[0] * d * 2;
      else bv.dist(6) = n[0] * d * 2;
    }
    else if(n[1] == (S)0.0 && n[0] + n[2] == (S)0.0)
    {
      if(n[0] > 0) bv.dist(D + 7) = n[0] * d * 2;
      else bv.dist(7) = n[0] * d * 2;
    }
    else if(n[0] == (S)0.0 && n[1] + n[2] == (S)0.0)
    {
      if(n[1] > 0) bv.dist(D + 8) = n[1] * d * 2;
      else bv.dist(8) = n[1] * d * 2;
    }
    else if(n[0] + n[2] == (S)0.0 && n[0] + n[1] == (S)0.0)
    {
      if(n[0] > 0) bv.dist(D + 9) = n[0] * d * 3;
      else bv.dist(9) = n[0] * d * 3;
    }
    else if(n[0] + n[1] == (S)0.0 && n[1] + n[2] == (S)0.0)
    {
      if(n[0] > 0) bv.dist(D + 10) = n[0] * d * 3;
      else bv.dist(10) = n[0] * d * 3;
    }
    else if(n[0] + n[1] == (S)0.0 && n[0] + n[2] == (S)0.0)
    {
      if(n[1] > 0) bv.dist(D + 11) = n[1] * d * 3;
      else bv.dist(11) = n[1] * d * 3;
    }
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, AABB<S>, Plane<S>>
{
  static void run(const Plane<S>& s, const Transform3<S>& tf, AABB<S>& bv)
  {
    Plane<S> new_s = transform(s, tf);
    const Vector3<S>& n = new_s.n;
    const S& d = new_s.d;

    AABB<S> bv_;
    bv_.min_ = Vector3<S>::Constant(-std::numeric_limits<S>::max());
    bv_.max_ = Vector3<S>::Constant(std::numeric_limits<S>::max());
    if(n[1] == (S)0.0 && n[2] == (S)0.0)
    {
      // normal aligned with x axis
      if(n[0] < 0) { bv_.min_[0] = bv_.max_[0] = -d; }
      else if(n[0] > 0) { bv_.min_[0] = bv_.max_[0] = d; }
    }
    else if(n[0] == (S)0.0 && n[2] == (S)0.0)
    {
      // normal aligned with y axis
      if(n[1] < 0) { bv_.min_[1] = bv_.max_[1] = -d; }
      else if(n[1] > 0) { bv_.min_[1] = bv_.max_[1] = d; }
    }
    else if(n[0] == (S)0.0 && n[1] == (S)0.0)
    {
      // normal aligned with z axis
      if(n[2] < 0) { bv_.min_[2] = bv_.max_[2] = -d; }
      else if(n[2] > 0) { bv_.min_[2] = bv_.max_[2] = d; }
    }

    bv = bv_;
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, OBB<S>, Plane<S>>
{
  static void run(const Plane<S>& s, const Transform3<S>& tf, OBB<S>& bv)
  {
    const Vector3<S> n = tf.linear() * s.n;
    bv.axis = generateCoordinateSystem(n);

    bv.extent << 0, std::numeric_limits<S>::max(), std::numeric_limits<S>::max();

    Vector3<S> p = s.n * s.d;
    bv.To = tf * p; /// n'd' = R * n * (d + (R * n) * T) = R * (n * d) + T
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, RSS<S>, Plane<S>>
{
  static void run(const Plane<S>& s, const Transform3<S>& tf, RSS<S>& bv)
  {
    const Vector3<S> n = tf.linear() * s.n;
    bv.axis = generateCoordinateSystem(n);

    bv.l[0] = std::numeric_limits<S>::max();
    bv.l[1] = std::numeric_limits<S>::max();

    bv.r = 0;

    Vector3<S> p = s.n * s.d;
    bv.To = tf * p;
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, OBBRSS<S>, Plane<S>>
{
  static void run(const Plane<S>& s, const Transform3<S>& tf, OBBRSS<S>& bv)
  {
    computeBV(s, tf, bv.obb);
    computeBV(s, tf, bv.rss);
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, kIOS<S>, Plane<S>>
{
  static void run(const Plane<S>& s, const Transform3<S>& tf, kIOS<S>& bv)
  {
    bv.num_spheres = 1;
    computeBV(s, tf, bv.obb);
    bv.spheres[0].o.setZero();
    bv.spheres[0].r = std::numeric_limits<S>::max();
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, KDOP<S, 16>, Plane<S>>
{
  static void run(const Plane<S>& s, const Transform3<S>& tf, KDOP<S, 16>& bv)
  {
    Plane<S> new_s = transform(s, tf);
    const Vector3<S>& n = new_s.n;
    const S& d = new_s.d;

    const std::size_t D = 8;

    for(std::size_t i = 0; i < D; ++i)
      bv.dist(i) = -std::numeric_limits<S>::max();
    for(std::size_t i = D; i < 2 * D; ++i)
      bv.dist(i) = std::numeric_limits<S>::max();

    if(n[1] == (S)0.0 && n[2] == (S)0.0)
    {
      if(n[0] > 0) bv.dist(0) = bv.dist(D) = d;
      else bv.dist(0) = bv.dist(D) = -d;
    }
    else if(n[0] == (S)0.0 && n[2] == (S)0.0)
    {
      if(n[1] > 0) bv.dist(1) = bv.dist(D + 1) = d;
      else bv.dist(1) = bv.dist(D + 1) = -d;
    }
    else if(n[0] == (S)0.0 && n[1] == (S)0.0)
    {
      if(n[2] > 0) bv.dist(2) = bv.dist(D + 2) = d;
      else bv.dist(2) = bv.dist(D + 2) = -d;
    }
    else if(n[2] == (S)0.0 && n[0] == n[1])
    {
      bv.dist(3) = bv.dist(D + 3) = n[0] * d * 2;
    }
    else if(n[1] == (S)0.0 && n[0] == n[2])
    {
      bv.dist(4) = bv.dist(D + 4) = n[0] * d * 2;
    }
    else if(n[0] == (S)0.0 && n[1] == n[2])
    {
      bv.dist(6) = bv.dist(D + 5) = n[1] * d * 2;
    }
    else if(n[2] == (S)0.0 && n[0] + n[1] == (S)0.0)
    {
      bv.dist(6) = bv.dist(D + 6) = n[0] * d * 2;
    }
    else if(n[1] == (S)0.0 && n[0] + n[2] == (S)0.0)
    {
      bv.dist(7) = bv.dist(D + 7) = n[0] * d * 2;
    }
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, KDOP<S, 18>, Plane<S>>
{
  static void run(const Plane<S>& s, const Transform3<S>& tf, KDOP<S, 18>& bv)
  {
    Plane<S> new_s = transform(s, tf);
    const Vector3<S>& n = new_s.n;
    const S& d = new_s.d;

    const std::size_t D = 9;

    for(std::size_t i = 0; i < D; ++i)
      bv.dist(i) = -std::numeric_limits<S>::max();
    for(std::size_t i = D; i < 2 * D; ++i)
      bv.dist(i) = std::numeric_limits<S>::max();

    if(n[1] == (S)0.0 && n[2] == (S)0.0)
    {
      if(n[0] > 0) bv.dist(0) = bv.dist(D) = d;
      else bv.dist(0) = bv.dist(D) = -d;
    }
    else if(n[0] == (S)0.0 && n[2] == (S)0.0)
    {
      if(n[1] > 0) bv.dist(1) = bv.dist(D + 1) = d;
      else bv.dist(1) = bv.dist(D + 1) = -d;
    }
    else if(n[0] == (S)0.0 && n[1] == (S)0.0)
    {
      if(n[2] > 0) bv.dist(2) = bv.dist(D + 2) = d;
      else bv.dist(2) = bv.dist(D + 2) = -d;
    }
    else if(n[2] == (S)0.0 && n[0] == n[1])
    {
      bv.dist(3) = bv.dist(D + 3) = n[0] * d * 2;
    }
    else if(n[1] == (S)0.0 && n[0] == n[2])
    {
      bv.dist(4) = bv.dist(D + 4) = n[0] * d * 2;
    }
    else if(n[0] == (S)0.0 && n[1] == n[2])
    {
      bv.dist(5) = bv.dist(D + 5) = n[1] * d * 2;
    }
    else if(n[2] == (S)0.0 && n[0] + n[1] == (S)0.0)
    {
      bv.dist(6) = bv.dist(D + 6) = n[0] * d * 2;
    }
    else if(n[1] == (S)0.0 && n[0] + n[2] == (S)0.0)
    {
      bv.dist(7) = bv.dist(D + 7) = n[0] * d * 2;
    }
    else if(n[0] == (S)0.0 && n[1] + n[2] == (S)0.0)
    {
      bv.dist(8) = bv.dist(D + 8) = n[1] * d * 2;
    }
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, KDOP<S, 24>, Plane<S>>
{
  static void run(const Plane<S>& s, const Transform3<S>& tf, KDOP<S, 24>& bv)
  {
    Plane<S> new_s = transform(s, tf);
    const Vector3<S>& n = new_s.n;
    const S& d = new_s.d;

    const std::size_t D = 12;

    for(std::size_t i = 0; i < D; ++i)
      bv.dist(i) = -std::numeric_limits<S>::max();
    for(std::size_t i = D; i < 2 * D; ++i)
      bv.dist(i) = std::numeric_limits<S>::max();

    if(n[1] == (S)0.0 && n[2] == (S)0.0)
    {
      if(n[0] > 0) bv.dist(0) = bv.dist(D) = d;
      else bv.dist(0) = bv.dist(D) = -d;
    }
    else if(n[0] == (S)0.0 && n[2] == (S)0.0)
    {
      if(n[1] > 0) bv.dist(1) = bv.dist(D + 1) = d;
      else bv.dist(1) = bv.dist(D + 1) = -d;
    }
    else if(n[0] == (S)0.0 && n[1] == (S)0.0)
    {
      if(n[2] > 0) bv.dist(2) = bv.dist(D + 2) = d;
      else bv.dist(2) = bv.dist(D + 2) = -d;
    }
    else if(n[2] == (S)0.0 && n[0] == n[1])
    {
      bv.dist(3) = bv.dist(D + 3) = n[0] * d * 2;
    }
    else if(n[1] == (S)0.0 && n[0] == n[2])
    {
      bv.dist(4) = bv.dist(D + 4) = n[0] * d * 2;
    }
    else if(n[0] == (S)0.0 && n[1] == n[2])
    {
      bv.dist(5) = bv.dist(D + 5) = n[1] * d * 2;
    }
    else if(n[2] == (S)0.0 && n[0] + n[1] == (S)0.0)
    {
      bv.dist(6) = bv.dist(D + 6) = n[0] * d * 2;
    }
    else if(n[1] == (S)0.0 && n[0] + n[2] == (S)0.0)
    {
      bv.dist(7) = bv.dist(D + 7) = n[0] * d * 2;
    }
    else if(n[0] == (S)0.0 && n[1] + n[2] == (S)0.0)
    {
      bv.dist(8) = bv.dist(D + 8) = n[1] * d * 2;
    }
    else if(n[0] + n[2] == (S)0.0 && n[0] + n[1] == (S)0.0)
    {
      bv.dist(9) = bv.dist(D + 9) = n[0] * d * 3;
    }
    else if(n[0] + n[1] == (S)0.0 && n[1] + n[2] == (S)0.0)
    {
      bv.dist(10) = bv.dist(D + 10) = n[0] * d * 3;
    }
    else if(n[0] + n[1] == (S)0.0 && n[0] + n[2] == (S)0.0)
    {
      bv.dist(11) = bv.dist(D + 11) = n[1] * d * 3;
    }
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, AABB<S>, Sphere<S>>
{
  static void run(const Sphere<S>& s, const Transform3<S>& tf, AABB<S>& bv)
  {
    const Vector3<S> v_delta = Vector3<S>::Constant(s.radius);
    bv.max_ = tf.translation() + v_delta;
    bv.min_ = tf.translation() - v_delta;
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, OBB<S>, Sphere<S>>
{
  static void run(const Sphere<S>& s, const Transform3<S>& tf, OBB<S>& bv)
  {
    bv.To = tf.translation();
    bv.axis.setIdentity();
    bv.extent.setConstant(s.radius);
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT ComputeBVImpl<S, AABB<S>, TriangleP<S>>
{
  static void run(const TriangleP<S>& s, const Transform3<S>& tf, AABB<S>& bv)
  {
    bv = AABB<S>(tf * s.a, tf * s.b, tf * s.c);
  }
};

//==============================================================================
extern template
struct ComputeBVImpl<double, AABB<double>, Box<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, OBB<double>, Box<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, AABB<double>, Capsule<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, OBB<double>, Capsule<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, AABB<double>, Cone<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, OBB<double>, Cone<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, AABB<double>, Cylinder<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, OBB<double>, Cylinder<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, AABB<double>, Ellipsoid<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, OBB<double>, Ellipsoid<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, AABB<double>, Halfspace<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, OBB<double>, Halfspace<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, RSS<double>, Halfspace<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, OBBRSS<double>, Halfspace<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, kIOS<double>, Halfspace<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, KDOP<double, 16>, Halfspace<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, KDOP<double, 18>, Halfspace<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, KDOP<double, 24>, Halfspace<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, AABB<double>, Plane<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, OBB<double>, Plane<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, RSS<double>, Plane<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, OBBRSS<double>, Plane<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, kIOS<double>, Plane<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, KDOP<double, 16>, Plane<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, KDOP<double, 18>, Plane<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, KDOP<double, 24>, Plane<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, AABB<double>, Sphere<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, OBB<double>, Sphere<double>>;

//==============================================================================
extern template
struct ComputeBVImpl<double, AABB<double>, TriangleP<double>>;

//==============================================================================
} // namespace detail
//==============================================================================

//==============================================================================
template <typename BV, typename Shape>
FCL_EXPORT
void computeBV(const Shape& s, const Transform3<typename BV::S>& tf, BV& bv)
{
  using S = typename BV::S;

  detail::ComputeBVImpl<S, BV, Shape>::run(s, tf, bv);
}

//==============================================================================
template <typename S>
void constructBox(const AABB<S>& bv, Box<S>& box, Transform3<S>& tf)
{
  box = Box<S>(bv.max_ - bv.min_);
  tf.linear().setIdentity();
  tf.translation() = bv.center();
}

//==============================================================================
template <typename S>
void constructBox(const OBB<S>& bv, Box<S>& box, Transform3<S>& tf)
{
  box = Box<S>(bv.extent * 2);
  tf.linear() = bv.axis;
  tf.translation() = bv.To;
}

//==============================================================================
template <typename S>
void constructBox(const OBBRSS<S>& bv, Box<S>& box, Transform3<S>& tf)
{
  box = Box<S>(bv.obb.extent * 2);
  tf.linear() = bv.obb.axis;
  tf.translation() = bv.obb.To;
}

//==============================================================================
template <typename S>
void constructBox(const kIOS<S>& bv, Box<S>& box, Transform3<S>& tf)
{
  box = Box<S>(bv.obb.extent * 2);
  tf.linear() = bv.obb.axis;
  tf.translation() = bv.obb.To;
}

//==============================================================================
template <typename S>
void constructBox(const RSS<S>& bv, Box<S>& box, Transform3<S>& tf)
{
  box = Box<S>(bv.width(), bv.height(), bv.depth());
  tf.linear() = bv.axis;
  tf.translation() = bv.To;
}

//==============================================================================
template <typename S>
void constructBox(const KDOP<S, 16>& bv, Box<S>& box, Transform3<S>& tf)
{
  box = Box<S>(bv.width(), bv.height(), bv.depth());
  tf.linear().setIdentity();
  tf.translation() = bv.center();
}

//==============================================================================
template <typename S>
void constructBox(const KDOP<S, 18>& bv, Box<S>& box, Transform3<S>& tf)
{
  box = Box<S>(bv.width(), bv.height(), bv.depth());
  tf.linear().setIdentity();
  tf.translation() = bv.center();
}

//==============================================================================
template <typename S>
void constructBox(const KDOP<S, 24>& bv, Box<S>& box, Transform3<S>& tf)
{
  box = Box<S>(bv.width(), bv.height(), bv.depth());
  tf.linear().setIdentity();
  tf.translation() = bv.center();
}

//==============================================================================
template <typename S>
void constructBox(const AABB<S>& bv, const Transform3<S>& tf_bv, Box<S>& box, Transform3<S>& tf)
{
  box = Box<S>(bv.max_ - bv.min_);
  tf = tf_bv * Translation3<S>(bv.center());
}

//==============================================================================
template <typename S>
void constructBox(const OBB<S>& bv, const Transform3<S>& tf_bv, Box<S>& box, Transform3<S>& tf)
{
  FCL_UNUSED(tf_bv);

  box = Box<S>(bv.extent * 2);
  tf.linear() = bv.axis;
  tf.translation() = bv.To;
}

//==============================================================================
template <typename S>
void constructBox(const OBBRSS<S>& bv, const Transform3<S>& tf_bv, Box<S>& box, Transform3<S>& tf)
{
  box = Box<S>(bv.obb.extent * 2);
  tf.linear() = bv.obb.axis;
  tf.translation() = bv.obb.To;
  tf = tf_bv * tf;
}

//==============================================================================
template <typename S>
void constructBox(const kIOS<S>& bv, const Transform3<S>& tf_bv, Box<S>& box, Transform3<S>& tf)
{
  FCL_UNUSED(tf_bv);

  box = Box<S>(bv.obb.extent * 2);
  tf.linear() = bv.obb.axis;
  tf.translation() = bv.obb.To;
}

//==============================================================================
template <typename S>
void constructBox(const RSS<S>& bv, const Transform3<S>& tf_bv, Box<S>& box, Transform3<S>& tf)
{
  box = Box<S>(bv.width(), bv.height(), bv.depth());
  tf.linear() = bv.axis;
  tf.translation() = bv.To;
  tf = tf_bv * tf;
}

//==============================================================================
template <typename S>
void constructBox(const KDOP<S, 16>& bv, const Transform3<S>& tf_bv, Box<S>& box, Transform3<S>& tf)
{
  box = Box<S>(bv.width(), bv.height(), bv.depth());
  tf = tf_bv * Translation3<S>(bv.center());
}

//==============================================================================
template <typename S>
void constructBox(const KDOP<S, 18>& bv, const Transform3<S>& tf_bv, Box<S>& box, Transform3<S>& tf)
{
  box = Box<S>(bv.width(), bv.height(), bv.depth());
  tf = tf_bv * Translation3<S>(bv.center());
}

//==============================================================================
template <typename S>
void constructBox(const KDOP<S, 24>& bv, const Transform3<S>& tf_bv, Box<S>& box, Transform3<S>& tf)
{
  box = Box<S>(bv.width(), bv.height(), bv.depth());
  tf = tf_bv * Translation3<S>(bv.center());
}

} // namespace fcl

#endif

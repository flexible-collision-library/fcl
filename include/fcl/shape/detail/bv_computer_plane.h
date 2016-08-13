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

#ifndef FCL_SHAPE_DETAIL_BVCOMPUTERPLANE_H
#define FCL_SHAPE_DETAIL_BVCOMPUTERPLANE_H

#include "fcl/BV/AABB.h"
#include "fcl/BV/OBB.h"
#include "fcl/BV/RSS.h"
#include "fcl/BV/OBBRSS.h"
#include "fcl/BV/kDOP.h"
#include "fcl/BV/kIOS.h"

namespace fcl
{
namespace detail
{

template <typename S>
struct BVComputer<S, AABB<S>, Plane<S>>;

template <typename S>
struct BVComputer<S, OBB<S>, Plane<S>>;

template <typename S>
struct BVComputer<S, RSS<S>, Plane<S>>;

template <typename S>
struct BVComputer<S, OBBRSS<S>, Plane<S>>;

template <typename S>
struct BVComputer<S, kIOS<S>, Plane<S>>;

template <typename S>
struct BVComputer<S, KDOP<S, 16>, Plane<S>>;

template <typename S>
struct BVComputer<S, KDOP<S, 18>, Plane<S>>;

template <typename S>
struct BVComputer<S, KDOP<S, 24>, Plane<S>>;

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename S>
struct BVComputer<S, AABB<S>, Plane<S>>
{
  static void compute(const Plane<S>& s, const Transform3<S>& tf, AABB<S>& bv)
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
struct BVComputer<S, OBB<S>, Plane<S>>
{
  static void compute(const Plane<S>& s, const Transform3<S>& tf, OBB<S>& bv)
  {
    Vector3<S> n = tf.linear() * s.n;
    bv.axis.col(0) = n;
    generateCoordinateSystem(bv.axis);

    bv.extent << 0, std::numeric_limits<S>::max(), std::numeric_limits<S>::max();

    Vector3<S> p = s.n * s.d;
    bv.To = tf * p; /// n'd' = R * n * (d + (R * n) * T) = R * (n * d) + T
  }
};

//==============================================================================
template <typename S>
struct BVComputer<S, RSS<S>, Plane<S>>
{
  static void compute(const Plane<S>& s, const Transform3<S>& tf, RSS<S>& bv)
  {
    Vector3<S> n = tf.linear() * s.n;

    bv.axis.col(0) = n;
    generateCoordinateSystem(bv.axis);

    bv.l[0] = std::numeric_limits<S>::max();
    bv.l[1] = std::numeric_limits<S>::max();

    bv.r = 0;

    Vector3<S> p = s.n * s.d;
    bv.To = tf * p;
  }
};

//==============================================================================
template <typename S>
struct BVComputer<S, OBBRSS<S>, Plane<S>>
{
  static void compute(const Plane<S>& s, const Transform3<S>& tf, OBBRSS<S>& bv)
  {
    computeBV(s, tf, bv.obb);
    computeBV(s, tf, bv.rss);
  }
};

//==============================================================================
template <typename S>
struct BVComputer<S, kIOS<S>, Plane<S>>
{
  static void compute(const Plane<S>& s, const Transform3<S>& tf, kIOS<S>& bv)
  {
    bv.num_spheres = 1;
    computeBV(s, tf, bv.obb);
    bv.spheres[0].o.setZero();
    bv.spheres[0].r = std::numeric_limits<S>::max();
  }
};

//==============================================================================
template <typename S>
struct BVComputer<S, KDOP<S, 16>, Plane<S>>
{
  static void compute(const Plane<S>& s, const Transform3<S>& tf, KDOP<S, 16>& bv)
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
struct BVComputer<S, KDOP<S, 18>, Plane<S>>
{
  static void compute(const Plane<S>& s, const Transform3<S>& tf, KDOP<S, 18>& bv)
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
struct BVComputer<S, KDOP<S, 24>, Plane<S>>
{
  static void compute(const Plane<S>& s, const Transform3<S>& tf, KDOP<S, 24>& bv)
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

} // namespace detail
} // namespace fcl

#endif

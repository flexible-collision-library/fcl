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

#ifndef FCL_BV_DETAIL_CONVERTER_H
#define FCL_BV_DETAIL_CONVERTER_H

#include "fcl/BV/kDOP.h"
#include "fcl/BV/AABB.h"
#include "fcl/BV/OBB.h"
#include "fcl/BV/RSS.h"
#include "fcl/BV/OBBRSS.h"
#include "fcl/BV/kIOS.h"

/** \brief Main namespace */
namespace fcl
{

/// @cond IGNORE
namespace detail
{

/// @brief Convert a bounding volume of type BV1 in configuration tf1 to a bounding volume of type BV2 in I configuration.
template <typename S, typename BV1, typename BV2>
class Converter
{
private:
  static void convert(const BV1& bv1, const Transform3<S>& tf1, BV2& bv2)
  {
    // should only use the specialized version, so it is private.
  }
};

/// @brief Convert from AABB to AABB, not very tight but is fast.
template <typename S>
class Converter<S, AABB<S>, AABB<S>>
{
public:
  static void convert(const AABB<S>& bv1, const Transform3<S>& tf1, AABB<S>& bv2)
  {
    const Vector3<S> center = bv1.center();
    S r = (bv1.max_ - bv1.min_).norm() * 0.5;
    Vector3<S> center2 = tf1 * center;
    Vector3<S> delta(r, r, r);
    bv2.min_ = center2 - delta;
    bv2.max_ = center2 + delta;
  }
};

template <typename S>
class Converter<S, AABB<S>, OBB<S>>
{
public:
  static void convert(const AABB<S>& bv1, const Transform3<S>& tf1, OBB<S>& bv2)
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

template <typename S>
class Converter<S, OBB<S>, OBB<S>>
{
public:
  static void convert(const OBB<S>& bv1, const Transform3<S>& tf1, OBB<S>& bv2)
  {
    bv2.extent = bv1.extent;
    bv2.To.noalias() = tf1 * bv1.To;
    bv2.axis.noalias() = tf1.linear() * bv1.axis;
  }
};

template <typename S>
class Converter<S, OBBRSS<S>, OBB<S>>
{
public:
  static void convert(const OBBRSS<S>& bv1, const Transform3<S>& tf1, OBB<S>& bv2)
  {
    Converter<S, OBB<S>, OBB<S>>::convert(bv1.obb, tf1, bv2);
  }
};

template <typename S>
class Converter<S, RSS<S>, OBB<S>>
{
public:
  static void convert(const RSS<S>& bv1, const Transform3<S>& tf1, OBB<S>& bv2)
  {
    bv2.extent << bv1.l[0] * 0.5 + bv1.r, bv1.l[1] * 0.5 + bv1.r, bv1.r;
    bv2.To.noalias() = tf1 * bv1.To;
    bv2.axis.noalias() = tf1.linear() * bv1.axis;
  }
};


template <typename S, typename BV1>
class Converter<S, BV1, AABB<S>>
{
public:
  static void convert(const BV1& bv1, const Transform3<S>& tf1, AABB<S>& bv2)
  {
    const Vector3<S> center = bv1.center();
    S r = Vector3<S>(bv1.width(), bv1.height(), bv1.depth()).norm() * 0.5;
    Vector3<S> delta(r, r, r);
    Vector3<S> center2 = tf1 * center;
    bv2.min_ = center2 - delta;
    bv2.max_ = center2 + delta;
  }
};

template <typename S, typename BV1>
class Converter<S, BV1, OBB<S>>
{
public:
  static void convert(const BV1& bv1, const Transform3<S>& tf1, OBB<S>& bv2)
  {
    AABB<S> bv;
    Converter<S, BV1, AABB<S>>::convert(bv1, Transform3<S>::Identity(), bv);
    Converter<S, AABB<S>, OBB<S>>::convert(bv, tf1, bv2);
  }
};

template <typename S>
class Converter<S, OBB<S>, RSS<S>>
{
public:
  static void convert(const OBB<S>& bv1, const Transform3<S>& tf1, RSS<S>& bv2)
  {
    bv2.To.noalias() = tf1 * bv1.To;
    bv2.axis.noalias() = tf1.linear() * bv1.axis;

    bv2.r = bv1.extent[2];
    bv2.l[0] = 2 * (bv1.extent[0] - bv2.r);
    bv2.l[1] = 2 * (bv1.extent[1] - bv2.r);
  }
};

template <typename S>
class Converter<S, RSS<S>, RSS<S>>
{
public:
  static void convert(const RSS<S>& bv1, const Transform3<S>& tf1, RSS<S>& bv2)
  {
    bv2.To.noalias() = tf1 * bv1.To;
    bv2.axis.noalias() = tf1.linear() * bv1.axis;

    bv2.r = bv1.r;
    bv2.l[0] = bv1.l[0];
    bv2.l[1] = bv1.l[1];
  }
};

template <typename S>
class Converter<S, OBBRSS<S>, RSS<S>>
{
public:
  static void convert(const OBBRSS<S>& bv1, const Transform3<S>& tf1, RSS<S>& bv2)
  {
    Converter<S, RSS<S>, RSS<S>>::convert(bv1.rss, tf1, bv2);
  }
};

template <typename S>
class Converter<S, AABB<S>, RSS<S>>
{
public:
  static void convert(const AABB<S>& bv1, const Transform3<S>& tf1, RSS<S>& bv2)
  {
    bv2.To.noalias() = tf1 * bv1.center();

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
    bv2.l[0] = (extent[id[0]] - bv2.r) * 2;
    bv2.l[1] = (extent[id[1]] - bv2.r) * 2;

    const Matrix3<S>& R = tf1.linear();
    bool left_hand = (id[0] == (id[1] + 1) % 3);
    if (left_hand)
      bv2.axis.col(0) = -R.col(id[0]);
    else
      bv2.axis.col(0) = R.col(id[0]);
    bv2.axis.col(1) = R.col(id[1]);
    bv2.axis.col(2) = R.col(id[2]);
  }
};

} // namespace detail

/// @endcond 

} // namespace fcl

#endif

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

#ifndef FCL_BV_H
#define FCL_BV_H


#include "fcl/BV/kDOP.h"
#include "fcl/BV/AABB.h"
#include "fcl/BV/OBB.h"
#include "fcl/BV/RSS.h"
#include "fcl/BV/OBBRSS.h"
#include "fcl/BV/kIOS.h"
#include "fcl/math/transform.h"

/** \brief Main namespace */
namespace fcl
{

/// @cond IGNORE
namespace details
{

/// @brief Convert a bounding volume of type BV1 in configuration tf1 to a bounding volume of type BV2 in I configuration.
template<typename BV1, typename BV2>
class Converter
{
private:
  static void convert(const BV1& bv1, const Transform3f& tf1, BV2& bv2)
  {
    // should only use the specialized version, so it is private.
  }
};


/// @brief Convert from AABB to AABB, not very tight but is fast.
template<>
class Converter<AABB, AABB>
{
public:
  static void convert(const AABB& bv1, const Transform3f& tf1, AABB& bv2)
  {
    const Vec3f& center = bv1.center();
    FCL_REAL r = (bv1.max_ - bv1.min_).length() * 0.5;
    Vec3f center2 = tf1.transform(center);
    Vec3f delta(r, r, r);
    bv2.min_ = center2 - delta;
    bv2.max_ = center2 + delta;
  }
};

template<>
class Converter<AABB, OBB>
{
public:
  static void convert(const AABB& bv1, const Transform3f& tf1, OBB& bv2)
  {    
    /*
    bv2.To = tf1.transform(bv1.center());

    /// Sort the AABB edges so that AABB extents are ordered.
    FCL_REAL d[3] = {bv1.width(), bv1.height(), bv1.depth() };
    std::size_t id[3] = {0, 1, 2};

    for(std::size_t i = 1; i < 3; ++i)
    {
      for(std::size_t j = i; j > 0; --j)
      {
        if(d[j] > d[j-1])
        {
          {
            FCL_REAL tmp = d[j];
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

    Vec3f extent = (bv1.max_ - bv1.min_) * 0.5;
    bv2.extent = Vec3f(extent[id[0]], extent[id[1]], extent[id[2]]);
    const Matrix3f& R = tf1.getRotation();
    bool left_hand = (id[0] == (id[1] + 1) % 3);
    bv2.axis[0] = left_hand ? -R.getColumn(id[0]) : R.getColumn(id[0]);
    bv2.axis[1] = R.getColumn(id[1]);
    bv2.axis[2] = R.getColumn(id[2]);
    */

    bv2.To = tf1.transform(bv1.center());
    bv2.extent = (bv1.max_ - bv1.min_) * 0.5;
    const Matrix3f& R = tf1.getRotation();
    bv2.axis[0] = R.getColumn(0);
    bv2.axis[1] = R.getColumn(1);
    bv2.axis[2] = R.getColumn(2);    
  }
};

template<>
class Converter<OBB, OBB>
{
public:
  static void convert(const OBB& bv1, const Transform3f& tf1, OBB& bv2)
  {
    bv2.extent = bv1.extent;
    bv2.To = tf1.transform(bv1.To);
    bv2.axis[0] = tf1.getQuatRotation().transform(bv1.axis[0]);
    bv2.axis[1] = tf1.getQuatRotation().transform(bv1.axis[1]);
    bv2.axis[2] = tf1.getQuatRotation().transform(bv1.axis[2]);
  }
};

template<>
class Converter<OBBRSS, OBB>
{
public:
  static void convert(const OBBRSS& bv1, const Transform3f& tf1, OBB& bv2)
  {
    Converter<OBB, OBB>::convert(bv1.obb, tf1, bv2);
  }
};

template<>
class Converter<RSS, OBB>
{
public:
  static void convert(const RSS& bv1, const Transform3f& tf1, OBB& bv2)
  {
    bv2.extent = Vec3f(bv1.l[0] * 0.5 + bv1.r, bv1.l[1] * 0.5 + bv1.r, bv1.r);
    bv2.To = tf1.transform(bv1.Tr);
    bv2.axis[0] = tf1.getQuatRotation().transform(bv1.axis[0]);
    bv2.axis[1] = tf1.getQuatRotation().transform(bv1.axis[1]);
    bv2.axis[2] = tf1.getQuatRotation().transform(bv1.axis[2]);    
  }
};


template<typename BV1>
class Converter<BV1, AABB>
{
public:
  static void convert(const BV1& bv1, const Transform3f& tf1, AABB& bv2)
  {
    const Vec3f& center = bv1.center();
    FCL_REAL r = Vec3f(bv1.width(), bv1.height(), bv1.depth()).length() * 0.5;
    Vec3f delta(r, r, r);
    Vec3f center2 = tf1.transform(center);
    bv2.min_ = center2 - delta;
    bv2.max_ = center2 + delta;
  }
};

template<typename BV1>
class Converter<BV1, OBB>
{
public:
  static void convert(const BV1& bv1, const Transform3f& tf1, OBB& bv2)
  {
    AABB bv;
    Converter<BV1, AABB>::convert(bv1, Transform3f(), bv);
    Converter<AABB, OBB>::convert(bv, tf1, bv2);
  }
};

template<>
class Converter<OBB, RSS>
{
public:
  static void convert(const OBB& bv1, const Transform3f& tf1, RSS& bv2)
  {
    bv2.Tr = tf1.transform(bv1.To);
    bv2.axis[0] = tf1.getQuatRotation().transform(bv1.axis[0]);
    bv2.axis[1] = tf1.getQuatRotation().transform(bv1.axis[1]);
    bv2.axis[2] = tf1.getQuatRotation().transform(bv1.axis[2]);
 
    bv2.r = bv1.extent[2];
    bv2.l[0] = 2 * (bv1.extent[0] - bv2.r);
    bv2.l[1] = 2 * (bv1.extent[1] - bv2.r);
  }
};

template<>
class Converter<RSS, RSS>
{
public:
  static void convert(const RSS& bv1, const Transform3f& tf1, RSS& bv2)
  {
    bv2.Tr = tf1.transform(bv1.Tr);
    bv2.axis[0] = tf1.getQuatRotation().transform(bv1.axis[0]);
    bv2.axis[1] = tf1.getQuatRotation().transform(bv1.axis[1]);
    bv2.axis[2] = tf1.getQuatRotation().transform(bv1.axis[2]);

    bv2.r = bv1.r;
    bv2.l[0] = bv1.l[0];
    bv2.l[1] = bv1.l[1];
  }
};

template<>
class Converter<OBBRSS, RSS>
{
public:
  static void convert(const OBBRSS& bv1, const Transform3f& tf1, RSS& bv2)
  {
    Converter<RSS, RSS>::convert(bv1.rss, tf1, bv2);
  }
};

template<>
class Converter<AABB, RSS>
{
public:
  static void convert(const AABB& bv1, const Transform3f& tf1, RSS& bv2)
  {
    bv2.Tr = tf1.transform(bv1.center());

    /// Sort the AABB edges so that AABB extents are ordered.
    FCL_REAL d[3] = {bv1.width(), bv1.height(), bv1.depth() };
    std::size_t id[3] = {0, 1, 2};

    for(std::size_t i = 1; i < 3; ++i)
    {
      for(std::size_t j = i; j > 0; --j)
      {
        if(d[j] > d[j-1])
        {
          {
            FCL_REAL tmp = d[j];
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

    Vec3f extent = (bv1.max_ - bv1.min_) * 0.5;
    bv2.r = extent[id[2]];
    bv2.l[0] = (extent[id[0]] - bv2.r) * 2;
    bv2.l[1] = (extent[id[1]] - bv2.r) * 2;

    const Matrix3f& R = tf1.getRotation();
    bool left_hand = (id[0] == (id[1] + 1) % 3);
    bv2.axis[0] = left_hand ? -R.getColumn(id[0]) : R.getColumn(id[0]);
    bv2.axis[1] = R.getColumn(id[1]);
    bv2.axis[2] = R.getColumn(id[2]);    
  }
};

}

/// @endcond 


/// @brief Convert a bounding volume of type BV1 in configuration tf1 to bounding volume of type BV2 in identity configuration.
template<typename BV1, typename BV2>
static inline void convertBV(const BV1& bv1, const Transform3f& tf1, BV2& bv2) 
{
  details::Converter<BV1, BV2>::convert(bv1, tf1, bv2);
}

}

#endif

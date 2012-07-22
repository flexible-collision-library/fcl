/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
#include "fcl/transform.h"

/** \brief Main namespace */
namespace fcl
{


template<typename BV1, typename BV2>
class Converter
{
private:
  static void convert(const BV1& bv1, const SimpleTransform& tf1, BV2& bv2)
  {
    // should only use the specialized version, so it is private.
  }
};

template<>
class Converter<AABB, AABB>
{
public:
  static void convert(const AABB& bv1, const SimpleTransform& tf1, AABB& bv2)
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
  static void convert(const AABB& bv1, const SimpleTransform& tf1, OBB& bv2)
  {
    bv2.extent = (bv1.max_ - bv1.min_) * 0.5;
    bv2.To = tf1.transform(bv1.center());
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
  static void convert(const OBB& bv1, const SimpleTransform& tf1, OBB& bv2)
  {
    bv2.extent = bv1.extent;
    bv2.To = tf1.transform(bv1.To);
    bv2.axis[0] = tf1.transform(bv1.axis[0]);
    bv2.axis[1] = tf1.transform(bv1.axis[1]);
    bv2.axis[2] = tf1.transform(bv1.axis[2]);
  }
};

template<>
class Converter<OBBRSS, OBB>
{
public:
  static void convert(const OBBRSS& bv1, const SimpleTransform& tf1, OBB& bv2)
  {
    Converter<OBB, OBB>::convert(bv1.obb, tf1, bv2);
  }
};

template<>
class Converter<RSS, OBB>
{
public:
  static void convert(const RSS& bv1, const SimpleTransform& tf1, OBB& bv2)
  {
    bv2.extent = Vec3f(bv1.l[0] * 0.5 + bv1.r, bv1.l[1] * 0.5 + bv1.r, bv1.r);
    bv2.To = tf1.transform(bv1.Tr);
    bv2.axis[0] = tf1.transform(bv1.axis[0]);
    bv2.axis[1] = tf1.transform(bv1.axis[1]);
    bv2.axis[2] = tf1.transform(bv1.axis[2]);    
  }
};


template<typename BV1>
class Converter<BV1, AABB>
{
public:
  static void convert(const BV1& bv1, const SimpleTransform& tf1, AABB& bv2)
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
  static void convert(const BV1& bv1, const SimpleTransform& tf1, OBB& bv2)
  {
    AABB bv;
    Converter<BV1, AABB>::convert(bv1, SimpleTransform(), bv);
    Converter<AABB, OBB>::convert(bv, tf1, bv2);
  }
};




template<typename BV1, typename BV2>
static inline void convertBV(const BV1& bv1, const SimpleTransform& tf1, BV2& bv2) 
{
  Converter<BV1, BV2>::convert(bv1, tf1, bv2);
}

}

#endif

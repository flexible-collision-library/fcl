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

#ifndef FCL_BV_DETAIL_CONVERTER_H
#define FCL_BV_DETAIL_CONVERTER_H

#include "fcl/math/bv/kDOP.h"
#include "fcl/math/bv/AABB.h"
#include "fcl/math/bv/OBB.h"
#include "fcl/math/bv/RSS.h"
#include "fcl/math/bv/OBBRSS.h"
#include "fcl/math/bv/kIOS.h"

/** @brief Main namespace */
namespace fcl
{

/// @cond IGNORE
namespace detail
{

/// @brief Convert a bounding volume of type BV1 in configuration tf1 to a
/// bounding volume of type BV2 in I configuration.
template <typename S, typename BV1, typename BV2>
class Converter
{
private:
  static void convert(const BV1& bv1, const Transform3<S>& tf1, BV2& bv2);
};

/// @brief Convert from AABB to AABB, not very tight but is fast.
template <typename S>
class Converter<S, AABB<S>, AABB<S>>
{
public:
  static void convert(const AABB<S>& bv1, const Transform3<S>& tf1, AABB<S>& bv2);
};

template <typename S>
class Converter<S, AABB<S>, OBB<S>>
{
public:
  static void convert(const AABB<S>& bv1, const Transform3<S>& tf1, OBB<S>& bv2);
};

template <typename S>
class Converter<S, OBB<S>, OBB<S>>
{
public:
  static void convert(const OBB<S>& bv1, const Transform3<S>& tf1, OBB<S>& bv2);
};

template <typename S>
class Converter<S, OBBRSS<S>, OBB<S>>
{
public:
  static void convert(const OBBRSS<S>& bv1, const Transform3<S>& tf1, OBB<S>& bv2);
};

template <typename S>
class Converter<S, RSS<S>, OBB<S>>
{
public:
  static void convert(const RSS<S>& bv1, const Transform3<S>& tf1, OBB<S>& bv2);
};

template <typename S, typename BV1>
class Converter<S, BV1, AABB<S>>
{
public:
  static void convert(const BV1& bv1, const Transform3<S>& tf1, AABB<S>& bv2);
};

template <typename S, typename BV1>
class Converter<S, BV1, OBB<S>>
{
public:
  static void convert(const BV1& bv1, const Transform3<S>& tf1, OBB<S>& bv2);
};

template <typename S>
class Converter<S, OBB<S>, RSS<S>>
{
public:
  static void convert(const OBB<S>& bv1, const Transform3<S>& tf1, RSS<S>& bv2);
};

template <typename S>
class Converter<S, RSS<S>, RSS<S>>
{
public:
  static void convert(const RSS<S>& bv1, const Transform3<S>& tf1, RSS<S>& bv2);
};

template <typename S>
class Converter<S, OBBRSS<S>, RSS<S>>
{
public:
  static void convert(const OBBRSS<S>& bv1, const Transform3<S>& tf1, RSS<S>& bv2);
};

template <typename S>
class Converter<S, AABB<S>, RSS<S>>
{
public:
  static void convert(const AABB<S>& bv1, const Transform3<S>& tf1, RSS<S>& bv2);
};

} // namespace detail

/// @endcond 

} // namespace fcl

#include "fcl/math/bv/detail/converter-inl.h"

#endif

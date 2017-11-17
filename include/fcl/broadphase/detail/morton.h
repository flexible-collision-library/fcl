/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
 *  Copyright (c) 2016, Toyota Research Institute
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

#ifndef FCL_MORTON_H
#define FCL_MORTON_H

#include "fcl/common/types.h"
#include "fcl/math/bv/AABB.h"

#include <bitset>

namespace fcl
{

/// @cond IGNORE
namespace detail
{

template <typename S>
FCL_EXPORT
uint32 quantize(S x, uint32 n);

/// @brief compute 30 bit morton code
FCL_EXPORT
uint32 morton_code(uint32 x, uint32 y, uint32 z);

/// @brief compute 60 bit morton code
FCL_EXPORT
uint64 morton_code60(uint32 x, uint32 y, uint32 z);

/// @brief Functor to compute the morton code for a given AABB
/// This is specialized for 32- and 64-bit unsigned integers giving
/// a 30- or 60-bit code, respectively, and for `std::bitset<N>` where
/// N is the length of the code and must be a multiple of 3.
template<typename S, typename T>
struct FCL_EXPORT morton_functor {};

/// @brief Functor to compute 30 bit morton code for a given AABB<S>
template<typename S>
struct FCL_EXPORT morton_functor<S, uint32>
{
  morton_functor(const AABB<S>& bbox);

  uint32 operator() (const Vector3<S>& point) const;

  const Vector3<S> base;
  const Vector3<S> inv;

  static constexpr size_t bits();
};

using morton_functoru32f = morton_functor<float, uint32>;
using morton_functoru32d = morton_functor<double, uint32>;

/// @brief Functor to compute 60 bit morton code for a given AABB<S>
template<typename S>
struct FCL_EXPORT morton_functor<S, uint64>
{
  morton_functor(const AABB<S>& bbox);

  uint64 operator() (const Vector3<S>& point) const;

  const Vector3<S> base;
  const Vector3<S> inv;

  static constexpr size_t bits();
};

using morton_functoru64f = morton_functor<float, uint64>;
using morton_functoru64d = morton_functor<double, uint64>;

/// @brief Functor to compute N bit morton code for a given AABB<S>
/// N must be a multiple of 3.
template<typename S, size_t N>
struct FCL_EXPORT morton_functor<S, std::bitset<N>>
{
  static_assert(N%3==0, "Number of bits must be a multiple of 3");

  morton_functor(const AABB<S>& bbox);

  std::bitset<N> operator() (const Vector3<S>& point) const;

  const Vector3<S> base;
  const Vector3<S> inv;

  static constexpr size_t bits();
};

} // namespace detail
/// @endcond
} // namespace fcl

#include "fcl/broadphase/detail/morton-inl.h"

#endif

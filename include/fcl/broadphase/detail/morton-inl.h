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

#ifndef FCL_MORTON_INL_H
#define FCL_MORTON_INL_H

#include "fcl/broadphase/detail/morton.h"

namespace fcl {
/// @cond IGNORE
namespace detail {

//==============================================================================
extern template
uint32 quantize(double x, uint32 n);

//==============================================================================
extern template
struct morton_functor<double, uint32>;

//==============================================================================
extern template
struct morton_functor<double, uint64>;

//==============================================================================
template <typename S>
uint32 quantize(S x, uint32 n)
{
  return std::max(std::min((uint32)(x * (S)n), uint32(n-1)), uint32(0));
}

//==============================================================================
template<typename S>
morton_functor<S, uint32>::morton_functor(const AABB<S>& bbox)
  : base(bbox.min_),
    inv(1.0 / (bbox.max_[0] - bbox.min_[0]),
    1.0 / (bbox.max_[1] - bbox.min_[1]),
    1.0 / (bbox.max_[2] - bbox.min_[2]))
{
  // Do nothing
}

//==============================================================================
template<typename S>
uint32 morton_functor<S, uint32>::operator()(const Vector3<S>& point) const
{
  uint32 x = detail::quantize((point[0] - base[0]) * inv[0], 1024u);
  uint32 y = detail::quantize((point[1] - base[1]) * inv[1], 1024u);
  uint32 z = detail::quantize((point[2] - base[2]) * inv[2], 1024u);

  return detail::morton_code(x, y, z);
}

//==============================================================================
template<typename S>
morton_functor<S, uint64>::morton_functor(const AABB<S>& bbox)
  : base(bbox.min_),
    inv(1.0 / (bbox.max_[0] - bbox.min_[0]),
    1.0 / (bbox.max_[1] - bbox.min_[1]),
    1.0 / (bbox.max_[2] - bbox.min_[2]))
{
  // Do nothing
}

//==============================================================================
template<typename S>
uint64 morton_functor<S, uint64>::operator()(const Vector3<S>& point) const
{
  uint32 x = detail::quantize((point[0] - base[0]) * inv[0], 1u << 20);
  uint32 y = detail::quantize((point[1] - base[1]) * inv[1], 1u << 20);
  uint32 z = detail::quantize((point[2] - base[2]) * inv[2], 1u << 20);

  return detail::morton_code60(x, y, z);
}

//==============================================================================
template<typename S>
constexpr size_t morton_functor<S, uint64>::bits()
{
  return 60;
}

//==============================================================================
template<typename S>
constexpr size_t morton_functor<S, uint32>::bits()
{
  return 30;
}

//==============================================================================
template<typename S, size_t N>
morton_functor<S, std::bitset<N>>::morton_functor(const AABB<S>& bbox)
  : base(bbox.min_),
    inv(1.0 / (bbox.max_[0] - bbox.min_[0]),
        1.0 / (bbox.max_[1] - bbox.min_[1]),
        1.0 / (bbox.max_[2] - bbox.min_[2]))
{
  // Do nothing
}

//==============================================================================
template<typename S, size_t N>
std::bitset<N> morton_functor<S, std::bitset<N>>::operator()(
    const Vector3<S>& point) const
{
  S x = (point[0] - base[0]) * inv[0];
  S y = (point[1] - base[1]) * inv[1];
  S z = (point[2] - base[2]) * inv[2];
  int start_bit = bits() - 1;
  std::bitset<N> bset;

  x *= 2;
  y *= 2;
  z *= 2;

  for(size_t i = 0; i < bits()/3; ++i)
  {
    bset[start_bit--] = ((z < 1) ? 0 : 1);
    bset[start_bit--] = ((y < 1) ? 0 : 1);
    bset[start_bit--] = ((x < 1) ? 0 : 1);
    x = ((x >= 1) ? 2*(x-1) : 2*x);
    y = ((y >= 1) ? 2*(y-1) : 2*y);
    z = ((z >= 1) ? 2*(z-1) : 2*z);
  }

  return bset;
}

//==============================================================================
template<typename S, size_t N>
constexpr size_t morton_functor<S, std::bitset<N>>::bits()
{
  return N;
}

} // namespace detail
/// @endcond
} // namespace fcl

#endif

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

/** \author Jia Pan */

#ifndef FCL_MORTON_H
#define FCL_MORTON_H

#include "fcl/data_types.h"
#include "fcl/BV/AABB.h"

#include <bitset>

namespace fcl
{

/// @cond IGNORE
namespace details
{

static inline FCL_UINT32 quantize(FCL_REAL x, FCL_UINT32 n)
{
  return std::max(std::min((FCL_UINT32)(x * (FCL_REAL)n), FCL_UINT32(n-1)), FCL_UINT32(0));
}

/// @brief compute 30 bit morton code
static inline FCL_UINT32 morton_code(FCL_UINT32 x, FCL_UINT32 y, FCL_UINT32 z)
{
  x = (x | (x << 16)) & 0x030000FF; 
  x = (x | (x <<  8)) & 0x0300F00F; 
  x = (x | (x <<  4)) & 0x030C30C3; 
  x = (x | (x <<  2)) & 0x09249249; 

  y = (y | (y << 16)) & 0x030000FF; 
  y = (y | (y <<  8)) & 0x0300F00F; 
  y = (y | (y <<  4)) & 0x030C30C3; 
  y = (y | (y <<  2)) & 0x09249249; 

  z = (z | (z << 16)) & 0x030000FF; 
  z = (z | (z <<  8)) & 0x0300F00F; 
  z = (z | (z <<  4)) & 0x030C30C3; 
  z = (z | (z <<  2)) & 0x09249249; 

  return x | (y << 1) | (z << 2);
}

/// @brief compute 60 bit morton code
static inline FCL_UINT64 morton_code60(FCL_UINT32 x, FCL_UINT32 y, FCL_UINT32 z)
{
  FCL_UINT32 lo_x = x & 1023u;
  FCL_UINT32 lo_y = y & 1023u;
  FCL_UINT32 lo_z = z & 1023u;
  FCL_UINT32 hi_x = x >> 10u;
  FCL_UINT32 hi_y = y >> 10u;
  FCL_UINT32 hi_z = z >> 10u;

  return (FCL_UINT64(morton_code(hi_x, hi_y, hi_z)) << 30) | FCL_UINT64(morton_code(lo_x, lo_y, lo_z));
}

}
/// @endcond


/// @brief Functor to compute the morton code for a given AABB
/// This is specialized for 32- and 64-bit unsigned integers giving
/// a 30- or 60-bit code, respectively, and for `std::bitset<N>` where
/// N is the length of the code and must be a multiple of 3.
template<typename T>
struct morton_functor {};


/// @brief Functor to compute 30 bit morton code for a given AABB
template<>
struct morton_functor<FCL_UINT32>
{
  morton_functor(const AABB& bbox) : base(bbox.min_), 
                                     inv(1.0 / (bbox.max_[0] - bbox.min_[0]),
                                         1.0 / (bbox.max_[1] - bbox.min_[1]),
                                         1.0 / (bbox.max_[2] - bbox.min_[2]))
  {}

  FCL_UINT32 operator() (const Vec3f& point) const
  {
    FCL_UINT32 x = details::quantize((point[0] - base[0]) * inv[0], 1024u);
    FCL_UINT32 y = details::quantize((point[1] - base[1]) * inv[1], 1024u);
    FCL_UINT32 z = details::quantize((point[2] - base[2]) * inv[2], 1024u);
    
    return details::morton_code(x, y, z);
  }

  const Vec3f base;
  const Vec3f inv;

  static constexpr size_t bits() { return 30; }
};


/// @brief Functor to compute 60 bit morton code for a given AABB
template<>
struct morton_functor<FCL_UINT64>
{
  morton_functor(const AABB& bbox) : base(bbox.min_),
                                     inv(1.0 / (bbox.max_[0] - bbox.min_[0]),
                                         1.0 / (bbox.max_[1] - bbox.min_[1]),
                                         1.0 / (bbox.max_[2] - bbox.min_[2]))
  {}

  FCL_UINT64 operator() (const Vec3f& point) const
  {
    FCL_UINT32 x = details::quantize((point[0] - base[0]) * inv[0], 1u << 20);
    FCL_UINT32 y = details::quantize((point[1] - base[1]) * inv[1], 1u << 20);
    FCL_UINT32 z = details::quantize((point[2] - base[2]) * inv[2], 1u << 20);

    return details::morton_code60(x, y, z);
  }

  const Vec3f base;
  const Vec3f inv;

  static constexpr size_t bits() { return 60; }
};


/// @brief Functor to compute N bit morton code for a given AABB
/// N must be a multiple of 3.
template<size_t N>
struct morton_functor<std::bitset<N>> 
{
  static_assert(N%3==0, "Number of bits must be a multiple of 3");

  morton_functor(const AABB& bbox) : base(bbox.min_),
                                     inv(1.0 / (bbox.max_[0] - bbox.min_[0]),
                                         1.0 / (bbox.max_[1] - bbox.min_[1]),
                                         1.0 / (bbox.max_[2] - bbox.min_[2]))
  {}

  std::bitset<N> operator() (const Vec3f& point) const
  {
    FCL_REAL x = (point[0] - base[0]) * inv[0];
    FCL_REAL y = (point[1] - base[1]) * inv[1];
    FCL_REAL z = (point[2] - base[2]) * inv[2];
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

  const Vec3f base;
  const Vec3f inv;

  static constexpr size_t bits() { return N; }
};

}

#endif

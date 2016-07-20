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

#ifndef FCL_DATA_TYPES_H
#define FCL_DATA_TYPES_H

#include <cstddef>
#include <cstdint>

namespace fcl
{

typedef double FCL_REAL;
typedef std::int64_t  FCL_INT64;
typedef std::uint64_t FCL_UINT64;
typedef std::int32_t  FCL_INT32;
typedef std::uint32_t FCL_UINT32;

/// @brief Triangle with 3 indices for points
class Triangle
{
  /// @brief indices for each vertex of triangle
  std::size_t vids[3];

public:
  /// @brief Default constructor
  Triangle() {}

  /// @brief Create a triangle with given vertex indices
  Triangle(std::size_t p1, std::size_t p2, std::size_t p3)
  {
    set(p1, p2, p3);
  }

  /// @brief Set the vertex indices of the triangle
  inline void set(std::size_t p1, std::size_t p2, std::size_t p3)
  {
    vids[0] = p1; vids[1] = p2; vids[2] = p3;
  }

  /// @access the triangle index
  inline std::size_t operator[](int i) const { return vids[i]; }

  inline std::size_t& operator[](int i) { return vids[i]; }
};

}

#endif

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

#ifndef FCL_KDOP_H
#define FCL_KDOP_H


#include "fcl/math/vec_3f.h"

namespace fcl
{


/// @brief KDOP class describes the KDOP collision structures. K is set as the template parameter, which should be 16, 18, or 24
///  The KDOP structure is defined by some pairs of parallel planes defined by some axes. 
/// For K = 16, the planes are 6 AABB planes and 10 diagonal planes that cut off some space of the edges:
/// (-1,0,0) and (1,0,0)  -> indices 0 and 8
/// (0,-1,0) and (0,1,0)  -> indices 1 and 9
/// (0,0,-1) and (0,0,1)  -> indices 2 and 10
/// (-1,-1,0) and (1,1,0) -> indices 3 and 11
/// (-1,0,-1) and (1,0,1) -> indices 4 and 12
/// (0,-1,-1) and (0,1,1) -> indices 5 and 13
/// (-1,1,0) and (1,-1,0) -> indices 6 and 14
/// (-1,0,1) and (1,0,-1) -> indices 7 and 15
/// For K = 18, the planes are 6 AABB planes and 12 diagonal planes that cut off some space of the edges:
/// (-1,0,0) and (1,0,0)  -> indices 0 and 9
/// (0,-1,0) and (0,1,0)  -> indices 1 and 10
/// (0,0,-1) and (0,0,1)  -> indices 2 and 11
/// (-1,-1,0) and (1,1,0) -> indices 3 and 12
/// (-1,0,-1) and (1,0,1) -> indices 4 and 13
/// (0,-1,-1) and (0,1,1) -> indices 5 and 14
/// (-1,1,0) and (1,-1,0) -> indices 6 and 15
/// (-1,0,1) and (1,0,-1) -> indices 7 and 16
/// (0,-1,1) and (0,1,-1) -> indices 8 and 17
/// For K = 18, the planes are 6 AABB planes and 18 diagonal planes that cut off some space of the edges:
/// (-1,0,0) and (1,0,0)  -> indices 0 and 12
/// (0,-1,0) and (0,1,0)  -> indices 1 and 13
/// (0,0,-1) and (0,0,1)  -> indices 2 and 14
/// (-1,-1,0) and (1,1,0) -> indices 3 and 15
/// (-1,0,-1) and (1,0,1) -> indices 4 and 16
/// (0,-1,-1) and (0,1,1) -> indices 5 and 17
/// (-1,1,0) and (1,-1,0) -> indices 6 and 18
/// (-1,0,1) and (1,0,-1) -> indices 7 and 19
/// (0,-1,1) and (0,1,-1) -> indices 8 and 20
/// (-1, -1, 1) and (1, 1, -1) --> indices 9 and 21
/// (-1, 1, -1) and (1, -1, 1) --> indices 10 and 22
/// (1, -1, -1) and (-1, 1, 1) --> indices 11 and 23
template<size_t N>
class KDOP
{
public:

  /// @brief Creating kDOP containing nothing
  KDOP();

  /// @brief Creating kDOP containing only one point
  KDOP(const Vec3f& v);

  /// @brief Creating kDOP containing two points
  KDOP(const Vec3f& a, const Vec3f& b);
  
  /// @brief Check whether two KDOPs are overlapped
  bool overlap(const KDOP<N>& other) const;

  //// @brief Check whether one point is inside the KDOP
  bool inside(const Vec3f& p) const;

  /// @brief Merge the point and the KDOP
  KDOP<N>& operator += (const Vec3f& p);

  /// @brief Merge two KDOPs
  KDOP<N>& operator += (const KDOP<N>& other);

  /// @brief Create a KDOP by mergin two KDOPs
  KDOP<N> operator + (const KDOP<N>& other) const;

  /// @brief The (AABB) width
  inline FCL_REAL width() const
  {
    return dist_[N / 2] - dist_[0];
  }

  /// @brief The (AABB) height
  inline FCL_REAL height() const
  {
    return dist_[N / 2 + 1] - dist_[1];
  }

  /// @brief The (AABB) depth
  inline FCL_REAL depth() const
  {
    return dist_[N / 2 + 2] - dist_[2];
  }

  /// @brief The (AABB) volume
  inline FCL_REAL volume() const
  {
    return width() * height() * depth();
  }

  /// @brief Size of the kDOP (used in BV_Splitter to order two kDOPs)
  inline FCL_REAL size() const
  {
    return width() * width() + height() * height() + depth() * depth();
  }

  /// @brief The (AABB) center
  inline Vec3f center() const
  {
    return Vec3f(dist_[0] + dist_[N / 2], dist_[1] + dist_[N / 2 + 1], dist_[2] + dist_[N / 2 + 2]) * 0.5;
  }

  /// @brief The distance between two KDOP<N>. Not implemented.
  FCL_REAL distance(const KDOP<N>& other, Vec3f* P = NULL, Vec3f* Q = NULL) const;

private:
  /// @brief Origin's distances to N KDOP planes
  FCL_REAL dist_[N];

public:
  inline FCL_REAL dist(std::size_t i) const
  {
    return dist_[i];
  }

  inline FCL_REAL& dist(std::size_t i)
  {
    return dist_[i];
  }


};


/// @brief translate the KDOP BV
template<size_t N>
KDOP<N> translate(const KDOP<N>& bv, const Vec3f& t);

}

#endif

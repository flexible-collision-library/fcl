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

#ifndef FCL_RSS_H
#define FCL_RSS_H

#include "fcl/math/constants.h"
#include "fcl/math/vec_3f.h"
#include "fcl/math/matrix_3f.h"


namespace fcl
{

/// @brief A class for rectangle sphere-swept bounding volume
class RSS
{
public:
  /// @brief Orientation of RSS. axis[i] is the ith column of the orientation matrix for the RSS; it is also the i-th principle direction of the RSS.
  /// We assume that axis[0] corresponds to the axis with the longest length, axis[1] corresponds to the shorter one and axis[2] corresponds to the shortest one.
  Vec3f axis[3];

  /// @brief Origin of the rectangle in RSS
  Vec3f Tr;

  /// @brief Side lengths of rectangle
  FCL_REAL l[2];

  /// @brief Radius of sphere summed with rectangle to form RSS
  FCL_REAL r;

  /// @brief Check collision between two RSS
  bool overlap(const RSS& other) const;

  /// @brief Check collision between two RSS and return the overlap part.
  /// For RSS, we return nothing, as the overlap part of two RSSs usually is not a RSS.
  bool overlap(const RSS& other, RSS& overlap_part) const
  {
    return overlap(other);
  }

  /// @brief Check whether the RSS contains a point
  inline bool contain(const Vec3f& p) const;

  /// @brief A simple way to merge the RSS and a point, not compact.
  /// @todo This function may have some bug.
  RSS& operator += (const Vec3f& p);

  /// @brief Merge the RSS and another RSS
  inline RSS& operator += (const RSS& other)
  {
    *this = *this + other;
    return *this;
  }

  /// @brief Return the merged RSS of current RSS and the other one
  RSS operator + (const RSS& other) const;

  /// @brief Width of the RSS
  inline FCL_REAL width() const
  {
    return l[0] + 2 * r;
  }

  /// @brief Height of the RSS
  inline FCL_REAL height() const
  {
    return l[1] + 2 * r;
  }

  /// @brief Depth of the RSS
  inline FCL_REAL depth() const
  {
    return 2 * r;
  }

  /// @brief Volume of the RSS
  inline FCL_REAL volume() const
  {
    return (l[0] * l[1] * 2 * r + 4 * constants::pi * r * r * r);
  }

  /// @brief Size of the RSS (used in BV_Splitter to order two RSSs)
  inline FCL_REAL size() const
  {
    return (std::sqrt(l[0] * l[0] + l[1] * l[1]) + 2 * r);
  }

  /// @brief The RSS center
  inline const Vec3f& center() const
  {
    return Tr;
  }

  /// @brief the distance between two RSS; P and Q, if not NULL, return the nearest points
  FCL_REAL distance(const RSS& other, Vec3f* P = NULL, Vec3f* Q = NULL) const;

};


/// @brief Translate the RSS bv
RSS translate(const RSS& bv, const Vec3f& t);

/// @brief distance between two RSS bounding volumes
/// P and Q (optional return values) are the closest points in the rectangles, not the RSS. But the direction P - Q is the correct direction for cloest points
/// Notice that P and Q are both in the local frame of the first RSS (not global frame and not even the local frame of object 1)
FCL_REAL distance(const Matrix3f& R0, const Vec3f& T0, const RSS& b1, const RSS& b2, Vec3f* P = NULL, Vec3f* Q = NULL);


/// @brief Check collision between two RSSs, b1 is in configuration (R0, T0) and b2 is in identity.
bool overlap(const Matrix3f& R0, const Vec3f& T0, const RSS& b1, const RSS& b2);


}


#endif

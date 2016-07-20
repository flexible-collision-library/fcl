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

#ifndef FCL_OBBRSS_H
#define FCL_OBBRSS_H


#include "fcl/BV/OBB.h"
#include "fcl/BV/RSS.h"

namespace fcl
{


/// @brief Class merging the OBB and RSS, can handle collision and distance simultaneously
class OBBRSS
{
public:

  /// @brief OBB member, for rotation
  OBB obb;

  /// @brief RSS member, for distance
  RSS rss;

  /// @brief Check collision between two OBBRSS
  bool overlap(const OBBRSS& other) const
  {
    return obb.overlap(other.obb);
  }

  /// @brief Check collision between two OBBRSS and return the overlap part.
  bool overlap(const OBBRSS& other, OBBRSS& overlap_part) const
  {
    return overlap(other);
  }

  /// @brief Check whether the OBBRSS contains a point
  inline bool contain(const Vec3f& p) const
  {
    return obb.contain(p);
  }

  /// @brief Merge the OBBRSS and a point
  OBBRSS& operator += (const Vec3f& p) 
  {
    obb += p;
    rss += p;
    return *this;
  }

  /// @brief Merge two OBBRSS
  OBBRSS& operator += (const OBBRSS& other)
  {
    *this = *this + other;
    return *this;
  }

  /// @brief Merge two OBBRSS
  OBBRSS operator + (const OBBRSS& other) const
  {
    OBBRSS result;
    result.obb = obb + other.obb;
    result.rss = rss + other.rss;
    return result;
  }

  /// @brief Width of the OBRSS
  inline FCL_REAL width() const
  {
    return obb.width();
  }

  /// @brief Height of the OBBRSS
  inline FCL_REAL height() const
  {
    return obb.height();
  }

  /// @brief Depth of the OBBRSS
  inline FCL_REAL depth() const
  {
    return obb.depth();
  }

  /// @brief Volume of the OBBRSS
  inline FCL_REAL volume() const
  {
    return obb.volume();
  }

  /// @brief Size of the OBBRSS (used in BV_Splitter to order two OBBRSS)
  inline FCL_REAL size() const
  {
    return obb.size();
  }

  /// @brief Center of the OBBRSS
  inline const Vec3f& center() const
  {
    return obb.center();
  }

  /// @brief Distance between two OBBRSS; P and Q , is not NULL, returns the nearest points
  FCL_REAL distance(const OBBRSS& other, Vec3f* P = NULL, Vec3f* Q = NULL) const
  {
    return rss.distance(other.rss, P, Q);
  }
};

/// @brief Translate the OBBRSS bv
OBBRSS translate(const OBBRSS& bv, const Vec3f& t);

/// @brief Check collision between two OBBRSS, b1 is in configuration (R0, T0) and b2 is in indentity
bool overlap(const Matrix3f& R0, const Vec3f& T0, const OBBRSS& b1, const OBBRSS& b2);

/// @brief Computate distance between two OBBRSS, b1 is in configuation (R0, T0) and b2 is in indentity; P and Q, is not NULL, returns the nearest points
FCL_REAL distance(const Matrix3f& R0, const Vec3f& T0, const OBBRSS& b1, const OBBRSS& b2, Vec3f* P = NULL, Vec3f* Q = NULL);

}


#endif

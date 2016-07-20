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

#ifndef FCL_OBB_H
#define FCL_OBB_H


#include "fcl/math/vec_3f.h"
#include "fcl/math/matrix_3f.h"

namespace fcl
{

/// @brief Oriented bounding box class
class OBB
{
public:
  /// @brief Orientation of OBB. axis[i] is the ith column of the orientation matrix for the box; it is also the i-th principle direction of the box. 
  /// We assume that axis[0] corresponds to the axis with the longest box edge, axis[1] corresponds to the shorter one and axis[2] corresponds to the shortest one.
  Vec3f axis[3];

  /// @brief Center of OBB
  Vec3f To;
  
  /// @brief Half dimensions of OBB
  Vec3f extent;

  /// @brief Check collision between two OBB, return true if collision happens. 
  bool overlap(const OBB& other) const;

  
  /// @brief Check collision between two OBB and return the overlap part. For OBB, the overlap_part return value is NOT used as the overlap part of two obbs usually is not an obb. 
  bool overlap(const OBB& other, OBB& overlap_part) const
  {
    return overlap(other);
  }

  /// @brief Check whether the OBB contains a point.
  bool contain(const Vec3f& p) const;

  /// @brief A simple way to merge the OBB and a point (the result is not compact).
  OBB& operator += (const Vec3f& p);

  /// @brief Merge the OBB and another OBB (the result is not compact).
  OBB& operator += (const OBB& other)
  {
     *this = *this + other;
     return *this;
  }

  /// @brief Return the merged OBB of current OBB and the other one (the result is not compact).
  OBB operator + (const OBB& other) const;

  /// @brief Width of the OBB.
  inline FCL_REAL width() const
  {
    return 2 * extent[0];
  }

  /// @brief Height of the OBB.
  inline FCL_REAL height() const
  {
    return 2 * extent[1];
  }

  /// @brief Depth of the OBB
  inline FCL_REAL depth() const
  {
    return 2 * extent[2];
  }

  /// @brief Volume of the OBB
  inline FCL_REAL volume() const
  {
    return width() * height() * depth();
  }

  /// @brief Size of the OBB (used in BV_Splitter to order two OBBs)
  inline FCL_REAL size() const
  {
    return extent.sqrLength();
  }

  /// @brief Center of the OBB
  inline const Vec3f& center() const
  {
    return To;
  }


  /// @brief Distance between two OBBs, not implemented.
  FCL_REAL distance(const OBB& other, Vec3f* P = NULL, Vec3f* Q = NULL) const;
};


/// @brief Translate the OBB bv
OBB translate(const OBB& bv, const Vec3f& t);

/// @brief Check collision between two obbs, b1 is in configuration (R0, T0) and b2 is in identity.
bool overlap(const Matrix3f& R0, const Vec3f& T0, const OBB& b1, const OBB& b2);


/// @brief Check collision between two boxes: the first box is in configuration (R, T) and its half dimension is set by a;
/// the second box is in identity configuration and its half dimension is set by b.
bool obbDisjoint(const Matrix3f& B, const Vec3f& T, const Vec3f& a, const Vec3f& b);
}

#endif

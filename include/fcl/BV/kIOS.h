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

#ifndef FCL_KIOS_H
#define FCL_KIOS_H

#include "fcl/BV/OBB.h"


namespace fcl
{
 
/// @brief A class describing the kIOS collision structure, which is a set of spheres.
class kIOS
{
  /// @brief One sphere in kIOS
  struct kIOS_Sphere
  {
    Vec3f o;
    FCL_REAL r;
  };

  /// @brief generate one sphere enclosing two spheres
  static kIOS_Sphere encloseSphere(const kIOS_Sphere& s0, const kIOS_Sphere& s1)
  {
    Vec3f d = s1.o - s0.o;
    FCL_REAL dist2 = d.sqrLength();
    FCL_REAL diff_r = s1.r - s0.r;
      
    /** The sphere with the larger radius encloses the other */
    if(diff_r * diff_r >= dist2)
    {
      if(s1.r > s0.r) return s1;
      else return s0;
    }
    else /** spheres partially overlapping or disjoint */
    {
      float dist = std::sqrt(dist2);
      kIOS_Sphere s;
      s.r = dist + s0.r + s1.r;
      if(dist > 0)
        s.o = s0.o + d * ((s.r - s0.r) / dist);
      else
        s.o = s0.o;
      return s;
    }
  }
public:
    
  /// @brief The (at most) five spheres for intersection
  kIOS_Sphere spheres[5];

  /// @brief The number of spheres, no larger than 5
  unsigned int num_spheres;

  /// @ OBB related with kIOS
  OBB obb;

  /// @brief Check collision between two kIOS
  bool overlap(const kIOS& other) const;

  /// @brief Check collision between two kIOS and return the overlap part.
  /// For kIOS, we return nothing, as the overlappart of two kIOS usually is not an kIOS
  /// @todo Not efficient. It first checks the sphere collisions and then use OBB for further culling.
  bool overlap(const kIOS& other, kIOS& overlap_part) const
  {
    return overlap(other);
  }

  /// @brief Check whether the kIOS contains a point
  inline bool contain(const Vec3f& p) const;

  /// @brief A simple way to merge the kIOS and a point
  kIOS& operator += (const Vec3f& p);

  /// @brief Merge the kIOS and another kIOS
  kIOS& operator += (const kIOS& other)
  {
    *this = *this + other;
    return *this;
  }

  /// @brief Return the merged kIOS of current kIOS and the other one
  kIOS operator + (const kIOS& other) const;

  /// @brief Center of the kIOS
  const Vec3f& center() const
  {
    return spheres[0].o;
  }

  /// @brief Width of the kIOS
  FCL_REAL width() const;

  /// @brief Height of the kIOS
  FCL_REAL height() const;

  /// @brief Depth of the kIOS
  FCL_REAL depth() const;

  /// @brief Volume of the kIOS
  FCL_REAL volume() const;

  /// @brief size of the kIOS (used in BV_Splitter to order two kIOSs)
  FCL_REAL size() const;

  /// @brief The distance between two kIOS
  FCL_REAL distance(const kIOS& other, Vec3f* P = NULL, Vec3f* Q = NULL) const;
};


/// @brief Translate the kIOS BV
kIOS translate(const kIOS& bv, const Vec3f& t);

/// @brief Check collision between two kIOSs, b1 is in configuration (R0, T0) and b2 is in identity.
/// @todo Not efficient
bool overlap(const Matrix3f& R0, const Vec3f& T0, const kIOS& b1, const kIOS& b2);

/// @brief Approximate distance between two kIOS bounding volumes
/// @todo P and Q is not returned, need implementation
FCL_REAL distance(const Matrix3f& R0, const Vec3f& T0, const kIOS& b1, const kIOS& b2, Vec3f* P = NULL, Vec3f* Q = NULL);

}

#endif

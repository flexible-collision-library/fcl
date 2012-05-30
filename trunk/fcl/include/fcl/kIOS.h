/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#include "fcl/BVH_internal.h"
#include "fcl/vec_3f.h"
#include "fcl/matrix_3f.h"
#include "fcl/OBB.h"

/** \brief Main namespace */
namespace fcl
{
 
/** \brief kIOS class */
class kIOS
{
  struct kIOS_Sphere
  {
    Vec3f o;
    BVH_REAL r;
  };

  static kIOS_Sphere encloseSphere(const kIOS_Sphere& s0, const kIOS_Sphere& s1)
  {
    Vec3f d = s1.o - s0.o;
    BVH_REAL dist2 = d.sqrLength();
    BVH_REAL diff_r = s1.r - s0.r;
      
    /** The sphere with the larger radius encloses the other */
    if(diff_r * diff_r >= dist2)
    {
      if(s1.r > s0.r) return s1;
      else return s0;
    }
    else /** spheres partially overlapping or disjoint */
    {
      float dist = sqrt(dist2);
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
    
  /** \brief The (at most) five spheres for intersection */
  kIOS_Sphere spheres[5];

  unsigned int num_spheres; // num_spheres <= 5

  OBB obb_bv;

  kIOS() {}

  /** \brief Check collision between two kIOS */
  bool overlap(const kIOS& other) const;

  /** \brief Check collision between two kIOS and return the overlap part.
   * For kIOS, we return nothing, as the overlappart of two kIOS usually is not an kIOS
   */
  bool overlap(const kIOS& other, kIOS& overlap_part) const
  {
    return overlap(other);
  }

  /** \brief Check whether the kIOS contains a point */
  inline bool contain(const Vec3f& p) const;

  /** \brief A simple way to merge the kIOS and a point */
  kIOS& operator += (const Vec3f& p);

  /** \brief Merge the kIOS and another kIOS */
  kIOS& operator += (const kIOS& other)
  {
    *this = *this + other;
    return *this;
  }

  /** \brief Return the merged kIOS of current kIOS and the other one */
  kIOS operator + (const kIOS& other) const;

  /** \brief Center of the kIOS */
  const Vec3f& center() const
  {
    return spheres[0].o;
  }

  /** \brief width of the kIOS */
  BVH_REAL width() const;

  /** \brief height of the kIOS */
  BVH_REAL height() const;

  /** \brief depth of the kIOS */
  BVH_REAL depth() const;

  /** \brief volume of the kIOS */
  BVH_REAL volume() const;

  /** \brief size of the kIOS, for split order */
  BVH_REAL size() const;

  /** \brief The distance between two kIOS */
  BVH_REAL distance(const kIOS& other, Vec3f* P = NULL, Vec3f* Q = NULL) const;

private:    
    
};

bool overlap(const Matrix3f& R0, const Vec3f& T0, const kIOS& b1, const kIOS& b2);

BVH_REAL distance(const Matrix3f& R0, const Vec3f& T0, const kIOS& b1, const kIOS& b2, Vec3f* P = NULL, Vec3f* Q = NULL);

}

#endif

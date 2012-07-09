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

#ifndef FCL_OBBRSS_H
#define FCL_OBBRSS_H

#include "fcl/BVH_internal.h"
#include "fcl/vec_3f.h"
#include "fcl/matrix_3f.h"
#include "fcl/BV/OBB.h"
#include "fcl/BV/RSS.h"

namespace fcl
{

class OBBRSS
{
public:

  OBB obb;
  RSS rss;
  
  OBBRSS() {}
  
  bool overlap(const OBBRSS& other) const
  {
    return obb.overlap(other.obb);
  }

  bool overlap(const OBBRSS& other, OBBRSS& overlap_part) const
  {
    return overlap(other);
  }

  inline bool contain(const Vec3f& p) const
  {
    return obb.contain(p);
  }

  OBBRSS& operator += (const Vec3f& p) 
  {
    obb += p;
    rss += p;
    return *this;
  }

  OBBRSS& operator += (const OBBRSS& other)
  {
    *this = *this + other;
    return *this;
  }

  OBBRSS operator + (const OBBRSS& other) const
  {
    OBBRSS result;
    result.obb = obb + other.obb;
    result.rss = rss + other.rss;
    return result;
  }

  inline FCL_REAL width() const
  {
    return obb.width();
  }

  inline FCL_REAL height() const
  {
    return obb.height();
  }

  inline FCL_REAL depth() const
  {
    return obb.depth();
  }

  inline FCL_REAL volume() const
  {
    return obb.volume();
  }

  inline FCL_REAL size() const
  {
    return obb.size();
  }

  inline const Vec3f& center() const
  {
    return obb.center();
  }

  FCL_REAL distance(const OBBRSS& other, Vec3f* P = NULL, Vec3f* Q = NULL) const
  {
    return rss.distance(other.rss, P, Q);
  }
};


bool overlap(const Matrix3f& R0, const Vec3f& T0, const OBBRSS& b1, const OBBRSS& b2);

FCL_REAL distance(const Matrix3f& R0, const Vec3f& T0, const OBBRSS& b1, const OBBRSS& b2, Vec3f* P = NULL, Vec3f* Q = NULL);

}


#endif

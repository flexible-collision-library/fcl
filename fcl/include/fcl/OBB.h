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

#ifndef FCL_PQP_H
#define FCL_PQP_H

#include "fcl/BVH_internal.h"
#include "fcl/vec_3f.h"

/** \brief Main namespace */
namespace fcl
{

/** \brief OBB class */
class OBB
{
public:
  /** \brief Orientation of OBB */
  Vec3f axis[3]; // R[i] is the ith column of the orientation matrix, or the axis of the OBB

  /** \brief center of OBB */
  Vec3f To;
  
  /** \brief Half dimensions of OBB */
  Vec3f extent;

  OBB() {}

  /** \brief Check collision between two OBB */
  bool overlap(const OBB& other) const;

  /** \brief Check collision between two OBB and return the overlap part.
   * For OBB, we return nothing, as the overlap part of two obbs usually is not an obb
   */
  bool overlap(const OBB& other, OBB& overlap_part) const
  {
    return overlap(other);
  }

  /** \brief Check whether the OBB contains a point */
  inline bool contain(const Vec3f& p) const;

  /** \brief A simple way to merge the OBB and a point, not compact. */
  OBB& operator += (const Vec3f& p);

  /** \brief Merge the OBB and another OBB */
  OBB& operator += (const OBB& other)
  {
     *this = *this + other;
     return *this;
  }

  /** \brief Return the merged OBB of current OBB and the other one */
  OBB operator + (const OBB& other) const;

  /** \brief Width of the OBB */
  inline BVH_REAL width() const
  {
    return 2 * extent[0];
  }

  /** \brief Height of the OBB */
  inline BVH_REAL height() const
  {
    return 2 * extent[1];
  }

  /** \brief Depth of the OBB */
  inline BVH_REAL depth() const
  {
    return 2 * extent[2];
  }

  /** \brief Volume of the OBB */
  inline BVH_REAL volume() const
  {
    return width() * height() * depth();
  }

  /** \brief Size of the OBB, for split order */
  inline BVH_REAL size() const
  {
    return extent.sqrLength();
  }

  /** \brief Center of the OBB */
  inline Vec3f center() const
  {
    return To;
  }

  /** \brief The distance between two OBB
   * Not implemented
   */
  BVH_REAL distance(const OBB& other, Vec3f* P = NULL, Vec3f* Q = NULL) const;


private:

  /** Compute the 8 vertices of a OBB */
  void computeVertices(Vec3f vertex[8]) const;

  /** \brief OBB merge method when the centers of two smaller OBB are far away */
  static OBB merge_largedist(const OBB& b1, const OBB& b2);

  /** \brief OBB merge method when the centers of two smaller OBB are close */
  static OBB merge_smalldist(const OBB& b1, const OBB& b2);

public:
  /** Kernel check whether two OBB are disjoint */
  static bool obbDisjoint(const Vec3f B[3], const Vec3f& T, const Vec3f& a, const Vec3f& b);

};


bool overlap(const Vec3f R0[3], const Vec3f& T0, const OBB& b1, const OBB& b2);

}

#endif

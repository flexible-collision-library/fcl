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

#ifndef FCL_RSS_H
#define FCL_RSS_H

#include "fcl/BVH_internal.h"
#include "fcl/vec_3f.h"

namespace fcl
{

class RSS
{
public:
  /** \brief Orientation of RSS */
  Vec3f axis[3];

  /** \brief position of rectangle (origin of the rectangle) */
  Vec3f Tr;

  /** \brief side lengths of rectangle */
  BVH_REAL l[2];

  /** \brief radius of sphere summed with rectangle to form RSS */
  BVH_REAL r;

  RSS() {}

  /** \brief Check collision between two RSS */
  bool overlap(const RSS& other) const;

  /** \brief Check collision between two RSS and return the overlap part.
   * For RSS, we return nothing, as the overlap part of two RSSs usually is not a RSS
   */
  bool overlap(const RSS& other, RSS& overlap_part) const
  {
    return overlap(other);
  }

  /** \brief Check whether the RSS contains a point */
  inline bool contain(const Vec3f& p) const;

  /** \brief A simple way to merge the RSS and a point, not compact.
   * THIS FUNCTION STILL HAS PROBLEM!!
   */
  RSS& operator += (const Vec3f& p);

  /** \brief Merge the RSS and another RSS */
  inline RSS& operator += (const RSS& other)
  {
    *this = *this + other;
    return *this;
  }

  /** \brief Return the merged RSS of current RSS and the other one */
  RSS operator + (const RSS& other) const;

  /** \brief Width of the RSS */
  inline BVH_REAL width() const
  {
    return l[0] + 2 * r;
  }

  /** \brief Height of the RSS */
  inline BVH_REAL height() const
  {
    return l[1] + 2 * r;
  }

  /** \brief Depth of the RSS */
  inline BVH_REAL depth() const
  {
    return 2 * r;
  }

  /** \brief Volume of the RSS */
  inline BVH_REAL volume() const
  {
    return (l[0] * l[1] * 2 * r + 4 * 3.1415926 * r * r * r);
  }

  /** \brief Size of the RSS, for split order */
  inline BVH_REAL size() const
  {
    return (sqrt(l[0] * l[0] + l[1] * l[1]) + 2 * r);
  }

  /** \brief The RSS center */
  inline Vec3f center() const
  {
    return Tr;
  }

  /** \brief the distance between two RSS */
  BVH_REAL distance(const RSS& other, Vec3f* P = NULL, Vec3f* Q = NULL) const;

protected:

  /** \brief Clip val between a and b */
  static void clipToRange(BVH_REAL& val, BVH_REAL a, BVH_REAL b);

  /** \brief Finds the parameters t & u corresponding to the two closest points on a pair of line segments.
   * The first segment is defined as Pa + A*t, 0 <= t <= a,  where "Pa" is one endpoint of the segment, "A" is a unit vector
   * pointing to the other endpoint, and t is a scalar that produces all the points between the two endpoints. Since "A" is a unit
   * vector, "a" is the segment's length.
   * The second segment is defined as Pb + B*u, 0 <= u <= b.
   * Many of the terms needed by the algorithm are already computed for other purposes,so we just pass these terms into the function
   * instead of complete specifications of each segment. "T" in the dot products is the vector betweeen Pa and Pb.
   * Reference: "On fast computation of distance between line segments." Vladimir J. Lumelsky, in Information Processing Letters, no. 21, pages 55-61, 1985.
   */
  static void segCoords(BVH_REAL& t, BVH_REAL& u, BVH_REAL a, BVH_REAL b, BVH_REAL A_dot_B, BVH_REAL A_dot_T, BVH_REAL B_dot_T);

  /** \brief Returns whether the nearest point on rectangle edge
   * Pb + B*u, 0 <= u <= b, to the rectangle edge,
   * Pa + A*t, 0 <= t <= a, is within the half space
   * determined by the point Pa and the direction Anorm.
   *
   * A,B, and Anorm are unit vectors.
   * T is the vector between Pa and Pb.
   */
  static bool inVoronoi(BVH_REAL a, BVH_REAL b, BVH_REAL Anorm_dot_B, BVH_REAL Anorm_dot_T, BVH_REAL A_dot_B, BVH_REAL A_dot_T, BVH_REAL B_dot_T);

public:

  /** \brief distance between two oriented rectangles
   * P and Q (optional return values) are the closest points in the rectangles, both are in the local frame of the first rectangle
   */
  static BVH_REAL rectDistance(const Vec3f Rab[3], Vec3f const& Tab, const BVH_REAL a[2], const BVH_REAL b[2], Vec3f* P = NULL, Vec3f* Q = NULL);

};

/** \brief distance between two RSS bounding volumes
 * P and Q (optional return values) are the closest points in the rectangles, not the RSS. But the direction P - Q is the correct direction for cloest points
 * Notice that P and Q are both in the local frame of the first RSS (not global frame and not even the local frame of object 1)
 */
BVH_REAL distance(const Vec3f R0[3], const Vec3f& T0, const RSS& b1, const RSS& b2, Vec3f* P = NULL, Vec3f* Q = NULL);

bool overlap(const Vec3f R0[3], const Vec3f& T0, const RSS& b1, const RSS& b2);


}


#endif

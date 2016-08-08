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

#ifndef FCL_NARROWPHASE_DETAIL_SPHERECAPSULE_H
#define FCL_NARROWPHASE_DETAIL_SPHERECAPSULE_H

#include "fcl/collision_data.h"

namespace fcl
{

namespace details
{

// Compute the point on a line segment that is the closest point on the
// segment to to another point. The code is inspired by the explanation
// given by Dan Sunday's page:
//   http://geomalgorithms.com/a02-_lines.html
template <typename Scalar>
void lineSegmentPointClosestToPoint (const Vector3<Scalar> &p, const Vector3<Scalar> &s1, const Vector3<Scalar> &s2, Vector3<Scalar> &sp);

template <typename Scalar>
bool sphereCapsuleIntersect(const Sphere<Scalar>& s1, const Transform3<Scalar>& tf1,
                            const Capsule<Scalar>& s2, const Transform3<Scalar>& tf2,
                            std::vector<ContactPoint<Scalar>>* contacts);

template <typename Scalar>
bool sphereCapsuleDistance(const Sphere<Scalar>& s1, const Transform3<Scalar>& tf1,
                           const Capsule<Scalar>& s2, const Transform3<Scalar>& tf2,
                           Scalar* dist, Vector3<Scalar>* p1, Vector3<Scalar>* p2);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
void lineSegmentPointClosestToPoint (const Vector3<Scalar> &p, const Vector3<Scalar> &s1, const Vector3<Scalar> &s2, Vector3<Scalar> &sp) {
  Vector3<Scalar> v = s2 - s1;
  Vector3<Scalar> w = p - s1;

  Scalar c1 = w.dot(v);
  Scalar c2 = v.dot(v);

  if (c1 <= 0) {
    sp = s1;
  } else if (c2 <= c1) {
    sp = s2;
  } else {
    Scalar b = c1/c2;
    Vector3<Scalar> Pb = s1 + v * b;
    sp = Pb;
  }
}

//==============================================================================
template <typename Scalar>
bool sphereCapsuleIntersect(const Sphere<Scalar>& s1, const Transform3<Scalar>& tf1,
                            const Capsule<Scalar>& s2, const Transform3<Scalar>& tf2,
                            std::vector<ContactPoint<Scalar>>* contacts)
{
  const Vector3<Scalar> pos1(0., 0., 0.5 * s2.lz);
  const Vector3<Scalar> pos2(0., 0., -0.5 * s2.lz);
  const Vector3<Scalar> s_c = tf2.inverse(Eigen::Isometry) * tf1.translation();

  Vector3<Scalar> segment_point;

  lineSegmentPointClosestToPoint (s_c, pos1, pos2, segment_point);
  Vector3<Scalar> diff = s_c - segment_point;

  const Scalar distance = diff.norm() - s1.radius - s2.radius;

  if (distance > 0)
    return false;

  const Vector3<Scalar> local_normal = -diff.normalized();

  if (contacts)
  {
    const Vector3<Scalar> normal = tf2.linear() * local_normal;
    const Vector3<Scalar> point = tf2 * (segment_point + local_normal * distance);
    const Scalar penetration_depth = -distance;

    contacts->push_back(ContactPoint<Scalar>(normal, point, penetration_depth));
  }

  return true;
}

//==============================================================================
template <typename Scalar>
bool sphereCapsuleDistance(const Sphere<Scalar>& s1, const Transform3<Scalar>& tf1,
                           const Capsule<Scalar>& s2, const Transform3<Scalar>& tf2,
                           Scalar* dist, Vector3<Scalar>* p1, Vector3<Scalar>* p2)
{
  Vector3<Scalar> pos1(0., 0., 0.5 * s2.lz);
  Vector3<Scalar> pos2(0., 0., -0.5 * s2.lz);
  Vector3<Scalar> s_c = tf2.inverse(Eigen::Isometry) * tf1.translation();

  Vector3<Scalar> segment_point;

  lineSegmentPointClosestToPoint (s_c, pos1, pos2, segment_point);
  Vector3<Scalar> diff = s_c - segment_point;

  Scalar distance = diff.norm() - s1.radius - s2.radius;

  if(distance <= 0)
    return false;

  if(dist) *dist = distance;

  if(p1 || p2) diff.normalize();
  if(p1)
  {
    *p1 = s_c - diff * s1.radius;
    *p1 = tf1.inverse(Eigen::Isometry) * tf2 * (*p1);
  }

  if(p2) *p2 = segment_point + diff * s1.radius;

  return true;
}

} // namespace details

} // namespace fcl

#endif

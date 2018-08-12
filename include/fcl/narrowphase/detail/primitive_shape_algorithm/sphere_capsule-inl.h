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

/** @author Jia Pan */

#ifndef FCL_NARROWPHASE_DETAIL_SPHERECAPSULE_INL_H
#define FCL_NARROWPHASE_DETAIL_SPHERECAPSULE_INL_H

#include "fcl/narrowphase/detail/primitive_shape_algorithm/sphere_capsule.h"

namespace fcl
{

namespace detail
{

//==============================================================================
extern template
void lineSegmentPointClosestToPoint(
    const Vector3<double> &p,
    const Vector3<double> &s1,
    const Vector3<double> &s2,
    Vector3<double> &sp);

//==============================================================================
extern template
bool sphereCapsuleIntersect(const Sphere<double>& s1, const Transform3<double>& tf1,
                            const Capsule<double>& s2, const Transform3<double>& tf2,
                            std::vector<ContactPoint<double>>* contacts);

//==============================================================================
extern template
bool sphereCapsuleDistance(const Sphere<double>& s1, const Transform3<double>& tf1,
                           const Capsule<double>& s2, const Transform3<double>& tf2,
                           double* dist, Vector3<double>* p1, Vector3<double>* p2);

//==============================================================================
template <typename S>
void lineSegmentPointClosestToPoint (const Vector3<S> &p, const Vector3<S> &s1, const Vector3<S> &s2, Vector3<S> &sp) {
  Vector3<S> v = s2 - s1;
  Vector3<S> w = p - s1;

  S c1 = w.dot(v);
  S c2 = v.dot(v);

  if (c1 <= 0) {
    sp = s1;
  } else if (c2 <= c1) {
    sp = s2;
  } else {
    S b = c1/c2;
    Vector3<S> Pb = s1 + v * b;
    sp = Pb;
  }
}

//==============================================================================
template <typename S>
bool sphereCapsuleIntersect(const Sphere<S>& s1, const Transform3<S>& tf1,
                            const Capsule<S>& s2, const Transform3<S>& tf2,
                            std::vector<ContactPoint<S>>* contacts)
{
  const Vector3<S> pos1(0., 0., 0.5 * s2.lz);
  const Vector3<S> pos2(0., 0., -0.5 * s2.lz);
  const Vector3<S> s_c = tf2.inverse(Eigen::Isometry) * tf1.translation();

  Vector3<S> segment_point;

  lineSegmentPointClosestToPoint (s_c, pos1, pos2, segment_point);
  Vector3<S> diff = s_c - segment_point;

  const S distance = diff.norm() - s1.radius - s2.radius;

  if (distance > 0)
    return false;

  const Vector3<S> local_normal = -diff.normalized();

  if (contacts)
  {
    const Vector3<S> normal = tf2.linear() * local_normal;
    const Vector3<S> point = tf2 * (segment_point + local_normal * distance);
    const S penetration_depth = -distance;

    contacts->emplace_back(normal, point, penetration_depth);
  }

  return true;
}

//==============================================================================
template <typename S>
bool sphereCapsuleDistance(const Sphere<S>& s1, const Transform3<S>& tf1,
                           const Capsule<S>& s2, const Transform3<S>& tf2,
                           S* dist, Vector3<S>* p1, Vector3<S>* p2)
{
  Vector3<S> pos1(0., 0., 0.5 * s2.lz);
  Vector3<S> pos2(0., 0., -0.5 * s2.lz);
  Vector3<S> s_c = tf2.inverse(Eigen::Isometry) * tf1.translation();

  Vector3<S> segment_point;

  lineSegmentPointClosestToPoint (s_c, pos1, pos2, segment_point);
  Vector3<S> diff = s_c - segment_point;

  S distance = diff.norm() - s1.radius - s2.radius;

  if(distance <= 0) {
    // NOTE: By assigning this a negative value, it allows the ultimately
    // calling code in distance-inl.h (distance() method) to use collision to
    // determine penetration depth and contact points. NOTE: This is a
    // *horrible* thing.
    // TODO(SeanCurtis-TRI): Create a *signed* distance variant of this and use
    // it to determined signed distance, penetration, and distance.
    if (dist) *dist = -1;
    return false;
  }

  if(dist) *dist = distance;

  if(p1 || p2) diff.normalize();
  if(p1)
  {
    *p1 = s_c - diff * s1.radius;
    *p1 = tf2 * (*p1);
  }

  if(p2)
  {
    *p2 = segment_point + diff * s2.radius;
    *p2 = tf2 * (*p2);
  }

  return true;
}

} // namespace detail
} // namespace fcl

#endif

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

/** @author Mike Danielczuk */

#ifndef FCL_NARROWPHASE_DETAIL_PLANEPLANE_INL_H
#define FCL_NARROWPHASE_DETAIL_PLANEPLANE_INL_H

#include "fcl/narrowphase/detail/primitive_shape_algorithm/plane_plane.h"

namespace fcl {

namespace detail {

//==============================================================================
extern template FCL_EXPORT bool planePlaneIntersect(
    const Plane<double>& s1, const Transform3<double>& tf1,
    const Plane<double>& s2, const Transform3<double>& tf2, Plane<double>& s,
    Vector3<double>& p, Vector3<double>& d, double& penetration_depth,
    int& ret);

//==============================================================================
extern template FCL_EXPORT bool planePlaneDistance(
    const Plane<double>& s1, const Transform3<double>& tf1,
    const Plane<double>& s2, const Transform3<double>& tf2, double* dist,
    Vector3<double>* p1, Vector3<double>* p2);

//==============================================================================
extern template FCL_EXPORT bool planePlaneSignedDistance(
    const Plane<double>& s1, const Transform3<double>& tf1,
    const Plane<double>& s2, const Transform3<double>& tf2, double* dist,
    Vector3<double>* p1, Vector3<double>* p2);

//==============================================================================
template <typename S>
bool planePlaneIntersect(const Plane<S>& s1, const Transform3<S>& tf1,
                         const Plane<S>& s2, const Transform3<S>& tf2,
                         Plane<S>& s, Vector3<S>& p, Vector3<S>& d,
                         S& penetration_depth, int& ret) {
  Plane<S> new_s1 = transform(s1, tf1);
  Plane<S> new_s2 = transform(s2, tf2);

  ret = 0;

  Vector3<S> dir = (new_s1.n).cross(new_s2.n);
  S dir_norm = dir.squaredNorm();
  if (dir_norm < std::numeric_limits<S>::epsilon())  // parallel
  {
    if ((new_s1.n).dot(new_s2.n) > 0) {
      if (new_s1.d < new_s2.d)  // s1 is inside s2
      {
        ret = 1;
        penetration_depth = std::numeric_limits<S>::max();
        s = new_s1;
      } else  // s2 is inside s1
      {
        ret = 2;
        penetration_depth = std::numeric_limits<S>::max();
        s = new_s2;
      }
      return true;
    } else {
      if (new_s1.d + new_s2.d < 0)  // not collision
        return false;
      else  // in each other
      {
        ret = 3;
        penetration_depth = -(new_s1.d + new_s2.d);
        return true;
      }
    }
  }

  Vector3<S> n = new_s2.n * new_s1.d - new_s1.n * new_s2.d;
  Vector3<S> origin = n.cross(dir);
  origin *= (1.0 / dir_norm);

  p = origin;
  d = dir;
  ret = 4;
  penetration_depth = std::numeric_limits<S>::max();

  return true;
}

//==============================================================================
template <typename S>
bool planePlaneDistance(const Plane<S>& s1, const Transform3<S>& tf1,
                        const Plane<S>& s2, const Transform3<S>& tf2, S* dist,
                        Vector3<S>* p1, Vector3<S>* p2) {
  Plane<S> new_s1 = transform(s1, tf1);
  Plane<S> new_s2 = transform(s2, tf2);

  S a = (new_s1.n).dot(new_s2.n);
  if ((a == 1 && new_s1.d != new_s2.d) || (a == -1 && new_s1.d != -new_s2.d)) {
    // Find any point on new_s1
    Vector3<S> pt1;
    if (new_s1.n[0] != (S)0.0) {
      pt1 = Vector3<S>(new_s1.d / new_s1.n[0], (S)0.0, (S)0.0);
    } else if (new_s1.n[1] != (S)0.0) {
      pt1 = Vector3<S>((S)0.0, new_s1.d / new_s1.n[1], (S)0.0);
    } else {
      pt1 = Vector3<S>((S)0.0, (S)0.0, new_s1.d / new_s1.n[2]);
    }

    // Find signed distance from pt1 to new_s2
    S diff = new_s2.signedDistance(pt1);
    if (dist) *dist = std::fabs(diff);
    if (p1) *p1 = pt1;
    if (p2) *p2 = pt1 - diff * new_s2.n;
    return true;
  }

  // If dot product between normals is not +-1, planes are in collision
  if (dist) *dist = -1;
  return false;
}

//==============================================================================
template <typename S>
bool planePlaneSignedDistance(const Plane<S>& s1, const Transform3<S>& tf1,
                              const Plane<S>& s2, const Transform3<S>& tf2,
                              S* dist, Vector3<S>* p1, Vector3<S>* p2) {
  Plane<S> new_s1 = transform(s1, tf1);
  Plane<S> new_s2 = transform(s2, tf2);

  S a = (new_s1.n).dot(new_s2.n);
  if ((a == 1 && new_s1.d != new_s2.d) || (a == -1 && new_s1.d != -new_s2.d)) {
    // Find any point on new_s1
    Vector3<S> pt1;
    if (new_s1.n[0] != (S)0.0) {
      pt1 = Vector3<S>(new_s1.d / new_s1.n[0], (S)0.0, (S)0.0);
    } else if (new_s1.n[1] != (S)0.0) {
      pt1 = Vector3<S>((S)0.0, new_s1.d / new_s1.n[1], (S)0.0);
    } else {
      pt1 = Vector3<S>((S)0.0, (S)0.0, new_s1.d / new_s1.n[2]);
    }

    // Find signed distance from pt1 to new_s2
    S diff = new_s2.signedDistance(pt1);
    if (dist) *dist = diff;
    if (p1) *p1 = pt1;
    if (p2) *p2 = pt1 - diff * new_s2.n;
    return true;
  }

  // If dot product between normals is not +-1, planes are in collision
  if (dist) *dist = -std::numeric_limits<S>::max();
  return false;
}

}  // namespace detail
}  // namespace fcl

#endif

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

#ifndef FCL_NARROWPHASE_DETAIL_PLANEPLANE_H
#define FCL_NARROWPHASE_DETAIL_PLANEPLANE_H

#include "fcl/geometry/shape/plane.h"
#include "fcl/narrowphase/contact_point.h"

namespace fcl {

namespace detail {

template <typename S>
bool planePlaneIntersect(const Plane<S>& s1, const Transform3<S>& tf1,
                         const Plane<S>& s2, const Transform3<S>& tf2,
                         Plane<S>& s, Vector3<S>& p, Vector3<S>& d,
                         S& penetration_depth, int& ret);

template <typename S>
bool planePlaneDistance(const Plane<S>& s1, const Transform3<S>& tf1,
                        const Plane<S>& s2, const Transform3<S>& tf2, S* dist,
                        Vector3<S>* p1, Vector3<S>* p2);

template <typename S>
bool planePlaneSignedDistance(const Plane<S>& s1, const Transform3<S>& tf1,
                              const Plane<S>& s2, const Transform3<S>& tf2,
                              S* dist, Vector3<S>* p1, Vector3<S>* p2);

}  // namespace detail
}  // namespace fcl

#include "fcl/narrowphase/detail/primitive_shape_algorithm/plane_plane-inl.h"

#endif

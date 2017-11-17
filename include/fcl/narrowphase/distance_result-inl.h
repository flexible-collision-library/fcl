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

#ifndef FCL_DISTANCERESULT_INL_H
#define FCL_DISTANCERESULT_INL_H

#include "fcl/narrowphase/distance_result.h"

namespace fcl
{

//==============================================================================
extern template
struct DistanceResult<double>;

//==============================================================================
template <typename S>
FCL_EXPORT
DistanceResult<S>::DistanceResult(S min_distance_)
  : min_distance(min_distance_),
    o1(nullptr),
    o2(nullptr),
    b1(NONE),
    b2(NONE)
{
  // Do nothing
}

//==============================================================================
template <typename S>
FCL_EXPORT
void DistanceResult<S>::update(
    S distance,
    const CollisionGeometry<S>* o1_,
    const CollisionGeometry<S>* o2_,
    int b1_,
    int b2_)
{
  if(min_distance > distance)
  {
    min_distance = distance;
    o1 = o1_;
    o2 = o2_;
    b1 = b1_;
    b2 = b2_;
  }
}

//==============================================================================
template <typename S>
FCL_EXPORT
void DistanceResult<S>::update(
    S distance,
    const CollisionGeometry<S>* o1_,
    const CollisionGeometry<S>* o2_,
    int b1_,
    int b2_,
    const Vector3<S>& p1,
    const Vector3<S>& p2)
{
  if(min_distance > distance)
  {
    min_distance = distance;
    o1 = o1_;
    o2 = o2_;
    b1 = b1_;
    b2 = b2_;
    nearest_points[0] = p1;
    nearest_points[1] = p2;
  }
}

//==============================================================================
template <typename S>
FCL_EXPORT
void DistanceResult<S>::update(const DistanceResult& other_result)
{
  if(min_distance > other_result.min_distance)
  {
    min_distance = other_result.min_distance;
    o1 = other_result.o1;
    o2 = other_result.o2;
    b1 = other_result.b1;
    b2 = other_result.b2;
    nearest_points[0] = other_result.nearest_points[0];
    nearest_points[1] = other_result.nearest_points[1];
  }
}

//==============================================================================
template <typename S>
FCL_EXPORT
void DistanceResult<S>::clear()
{
  min_distance = std::numeric_limits<S>::max();
  o1 = nullptr;
  o2 = nullptr;
  b1 = NONE;
  b2 = NONE;
}

} // namespace fcl

#endif

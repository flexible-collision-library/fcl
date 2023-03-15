/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2014, Willow Garage, Inc.
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

#include "fcl/narrowphase/continuous_collision-inl.h"

namespace fcl
{

//==============================================================================
template
float continuousCollide(
    const CollisionGeometry<float>* o1,
    const MotionBase<float>* motion1,
    const CollisionGeometry<float>* o2,
    const MotionBase<float>* motion2,
    const ContinuousCollisionRequest<float>& request,
    ContinuousCollisionResult<float>& result);

template
double continuousCollide(
    const CollisionGeometry<double>* o1,
    const MotionBase<double>* motion1,
    const CollisionGeometry<double>* o2,
    const MotionBase<double>* motion2,
    const ContinuousCollisionRequest<double>& request,
    ContinuousCollisionResult<double>& result);

//==============================================================================
template
float continuousCollide(
    const CollisionGeometry<float>* o1,
    const Transform3<float>& tf1_beg,
    const Transform3<float>& tf1_end,
    const CollisionGeometry<float>* o2,
    const Transform3<float>& tf2_beg,
    const Transform3<float>& tf2_end,
    const ContinuousCollisionRequest<float>& request,
    ContinuousCollisionResult<float>& result);

template
double continuousCollide(
    const CollisionGeometry<double>* o1,
    const Transform3<double>& tf1_beg,
    const Transform3<double>& tf1_end,
    const CollisionGeometry<double>* o2,
    const Transform3<double>& tf2_beg,
    const Transform3<double>& tf2_end,
    const ContinuousCollisionRequest<double>& request,
    ContinuousCollisionResult<double>& result);

//==============================================================================
template
float continuousCollide(
    const CollisionObject<float>* o1,
    const Transform3<float>& tf1_end,
    const CollisionObject<float>* o2,
    const Transform3<float>& tf2_end,
    const ContinuousCollisionRequest<float>& request,
    ContinuousCollisionResult<float>& result);

template
double continuousCollide(
    const CollisionObject<double>* o1,
    const Transform3<double>& tf1_end,
    const CollisionObject<double>* o2,
    const Transform3<double>& tf2_end,
    const ContinuousCollisionRequest<double>& request,
    ContinuousCollisionResult<double>& result);

//==============================================================================
template
float collide(
    const ContinuousCollisionObject<float>* o1,
    const ContinuousCollisionObject<float>* o2,
    const ContinuousCollisionRequest<float>& request,
    ContinuousCollisionResult<float>& result);

template
double collide(
    const ContinuousCollisionObject<double>* o1,
    const ContinuousCollisionObject<double>* o2,
    const ContinuousCollisionRequest<double>& request,
    ContinuousCollisionResult<double>& result);

} // namespace fcl

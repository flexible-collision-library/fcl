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

#ifndef FCL_CONTINUOUS_COLLISION_H
#define FCL_CONTINUOUS_COLLISION_H

#include <iostream>
#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/continuous_collision_object.h"
#include "fcl/narrowphase/detail/gjk_solver_indep.h"
#include "fcl/narrowphase/detail/gjk_solver_libccd.h"
#include "fcl/narrowphase/detail/conservative_advancement_func_matrix.h"
#include "fcl/narrowphase/detail/traversal/collision/mesh_continuous_collision_traversal_node.h"

namespace fcl
{

/// @brief continous collision checking between two objects
template <typename S>
FCL_EXPORT
S continuousCollide(
    const CollisionGeometry<S>* o1,
    const MotionBase<S>* motion1,
    const CollisionGeometry<S>* o2,
    const MotionBase<S>* motion2,
    const ContinuousCollisionRequest<S>& request,
    ContinuousCollisionResult<S>& result);

template <typename S>
FCL_EXPORT
S continuousCollide(
    const CollisionGeometry<S>* o1,
    const Transform3<S>& tf1_beg,
    const Transform3<S>& tf1_end,
    const CollisionGeometry<S>* o2,
    const Transform3<S>& tf2_beg,
    const Transform3<S>& tf2_end,
    const ContinuousCollisionRequest<S>& request,
    ContinuousCollisionResult<S>& result);

template <typename S>
FCL_EXPORT
S continuousCollide(
    const CollisionObject<S>* o1,
    const Transform3<S>& tf1_end,
    const CollisionObject<S>* o2,
    const Transform3<S>& tf2_end,
    const ContinuousCollisionRequest<S>& request,
    ContinuousCollisionResult<S>& result);

template <typename S>
FCL_EXPORT
S collide(
    const ContinuousCollisionObject<S>* o1,
    const ContinuousCollisionObject<S>* o2,
    const ContinuousCollisionRequest<S>& request,
    ContinuousCollisionResult<S>& result);

} // namespace fcl

#include "fcl/narrowphase/continuous_collision-inl.h"

#endif

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

/** \author Jia Pan */

#ifndef FCL_CONTINUOUS_COLLISION_H
#define FCL_CONTINUOUS_COLLISION_H

#include "fcl/collision_object.h"
#include "fcl/continuous_collision_object.h"
#include "fcl/collision_data.h"

namespace fcl
{

/// @brief continous collision checking between two objects
FCL_REAL continuousCollide(const CollisionGeometryd* o1, const Transform3d& tf1_beg, const Transform3d& tf1_end,
                           const CollisionGeometryd* o2, const Transform3d& tf2_beg, const Transform3d& tf2_end,
                           const ContinuousCollisionRequestd& request,
                           ContinuousCollisionResultd& result);

FCL_REAL continuousCollide(const CollisionObject* o1, const Transform3d& tf1_end,
                           const CollisionObject* o2, const Transform3d& tf2_end,
                           const ContinuousCollisionRequestd& request,
                           ContinuousCollisionResultd& result);

FCL_REAL collide(const ContinuousCollisionObject* o1, const ContinuousCollisionObject* o2,
                 const ContinuousCollisionRequestd& request,
                 ContinuousCollisionResultd& result);
          
}

#endif

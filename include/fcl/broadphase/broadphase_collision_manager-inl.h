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

#ifndef FCL_BROADPHASE_BROADPHASECOLLISIONMANAGER_INL_H
#define FCL_BROADPHASE_BROADPHASECOLLISIONMANAGER_INL_H

#include "fcl/broadphase/broadphase_collision_manager.h"

#include "fcl/common/unused.h"

namespace fcl {

//==============================================================================
extern template
class FCL_EXPORT BroadPhaseCollisionManager<double>;

//==============================================================================
template <typename S>
BroadPhaseCollisionManager<S>::BroadPhaseCollisionManager()
  : enable_tested_set_(false)
{
  // Do nothing
}

//==============================================================================
template <typename S>
BroadPhaseCollisionManager<S>::~BroadPhaseCollisionManager()
{
  // Do nothing
}

//==============================================================================
template <typename S>
void BroadPhaseCollisionManager<S>::registerObjects(
    const std::vector<CollisionObject<S>*>& other_objs)
{
  for(size_t i = 0; i < other_objs.size(); ++i)
    registerObject(other_objs[i]);
}

//==============================================================================
template <typename S>
void BroadPhaseCollisionManager<S>::update(CollisionObject<S>* updated_obj)
{
  FCL_UNUSED(updated_obj);

  update();
}

//==============================================================================
template <typename S>
void BroadPhaseCollisionManager<S>::update(
    const std::vector<CollisionObject<S>*>& updated_objs)
{
  FCL_UNUSED(updated_objs);

  update();
}

//==============================================================================
template <typename S>
bool BroadPhaseCollisionManager<S>::inTestedSet(
    CollisionObject<S>* a, CollisionObject<S>* b) const
{
  if(a < b) return tested_set.find(std::make_pair(a, b)) != tested_set.end();
  else return tested_set.find(std::make_pair(b, a)) != tested_set.end();
}

//==============================================================================
template <typename S>
void BroadPhaseCollisionManager<S>::insertTestedSet(
    CollisionObject<S>* a, CollisionObject<S>* b) const
{
  if(a < b) tested_set.insert(std::make_pair(a, b));
  else tested_set.insert(std::make_pair(b, a));
}

} // namespace fcl

#endif

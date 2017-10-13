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

#ifndef FCL_BROADPHASE_BROADPHASECOLLISIONMANAGER_H
#define FCL_BROADPHASE_BROADPHASECOLLISIONMANAGER_H

#include <set>
#include <vector>

#include "fcl/narrowphase/collision_object.h"

namespace fcl
{

/// @brief Callback for collision between two objects. Return value is whether
/// can stop now.
template <typename S>
using CollisionCallBack = bool (*)(
    CollisionObject<S>* o1, CollisionObject<S>* o2, void* cdata);

/// @brief Callback for distance between two objects, Return value is whether
/// can stop now, also return the minimum distance till now.
template <typename S>
using DistanceCallBack = bool (*)(
    CollisionObject<S>* o1,
    CollisionObject<S>* o2, void* cdata, S& dist);

/// @brief Base class for broad phase collision. It helps to accelerate the
/// collision/distance between N objects. Also support self collision, self
/// distance and collision/distance with another M objects.
template <typename S>
class FCL_EXPORT BroadPhaseCollisionManager
{
public:
  BroadPhaseCollisionManager();

  virtual ~BroadPhaseCollisionManager();

  /// @brief add objects to the manager
  virtual void registerObjects(const std::vector<CollisionObject<S>*>& other_objs);

  /// @brief add one object to the manager
  virtual void registerObject(CollisionObject<S>* obj) = 0;

  /// @brief remove one object from the manager
  virtual void unregisterObject(CollisionObject<S>* obj) = 0;

  /// @brief initialize the manager, related with the specific type of manager
  virtual void setup() = 0;

  /// @brief update the condition of manager
  virtual void update() = 0;

  /// @brief update the manager by explicitly given the object updated
  virtual void update(CollisionObject<S>* updated_obj);

  /// @brief update the manager by explicitly given the set of objects update
  virtual void update(const std::vector<CollisionObject<S>*>& updated_objs);

  /// @brief clear the manager
  virtual void clear() = 0;

  /// @brief return the objects managed by the manager
  virtual void getObjects(std::vector<CollisionObject<S>*>& objs) const = 0;

  /// @brief perform collision test between one object and all the objects belonging to the manager
  virtual void collide(CollisionObject<S>* obj, void* cdata, CollisionCallBack<S> callback) const = 0;

  /// @brief perform distance computation between one object and all the objects belonging to the manager
  virtual void distance(CollisionObject<S>* obj, void* cdata, DistanceCallBack<S> callback) const = 0;

  /// @brief perform collision test for the objects belonging to the manager (i.e., N^2 self collision)
  virtual void collide(void* cdata, CollisionCallBack<S> callback) const = 0;

  /// @brief perform distance test for the objects belonging to the manager (i.e., N^2 self distance)
  virtual void distance(void* cdata, DistanceCallBack<S> callback) const = 0;

  /// @brief perform collision test with objects belonging to another manager
  virtual void collide(BroadPhaseCollisionManager* other_manager, void* cdata, CollisionCallBack<S> callback) const = 0;

  /// @brief perform distance test with objects belonging to another manager
  virtual void distance(BroadPhaseCollisionManager* other_manager, void* cdata, DistanceCallBack<S> callback) const = 0;

  /// @brief whether the manager is empty
  virtual bool empty() const = 0;
  
  /// @brief the number of objects managed by the manager
  virtual size_t size() const = 0;

protected:

  /// @brief tools help to avoid repeating collision or distance callback for the pairs of objects tested before. It can be useful for some of the broadphase algorithms.
  mutable std::set<std::pair<CollisionObject<S>*, CollisionObject<S>*> > tested_set;
  mutable bool enable_tested_set_;

  bool inTestedSet(CollisionObject<S>* a, CollisionObject<S>* b) const;

  void insertTestedSet(CollisionObject<S>* a, CollisionObject<S>* b) const;

};

using BroadPhaseCollisionManagerf = BroadPhaseCollisionManager<float>;
using BroadPhaseCollisionManagerd = BroadPhaseCollisionManager<double>;

} // namespace fcl

#include "fcl/broadphase/broadphase_collision_manager-inl.h"

#endif

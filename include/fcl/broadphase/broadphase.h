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



#ifndef FCL_BROAD_PHASE_H
#define FCL_BROAD_PHASE_H

#include "fcl/collision_object.h"
#include <set>
#include <vector>

namespace fcl
{


/// @brief Callback for collision between two objects. Return value is whether can stop now.
typedef bool (*CollisionCallBack)(CollisionObject* o1, CollisionObject* o2, void* cdata);

/// @brief Callback for distance between two objects, Return value is whether can stop now, also return the minimum distance till now.
typedef bool (*DistanceCallBack)(CollisionObject* o1, CollisionObject* o2, void* cdata, FCL_REAL& dist);


/// @brief Base class for broad phase collision. It helps to accelerate the collision/distance between N objects. Also support self collision, self distance and collision/distance with another M objects.
class BroadPhaseCollisionManager
{
public:
  BroadPhaseCollisionManager() : enable_tested_set_(false)
  {
  }

  virtual ~BroadPhaseCollisionManager() {}

  /// @brief add objects to the manager
  virtual void registerObjects(const std::vector<CollisionObject*>& other_objs)
  {
    for(size_t i = 0; i < other_objs.size(); ++i)
      registerObject(other_objs[i]);
  }

  /// @brief add one object to the manager
  virtual void registerObject(CollisionObject* obj) = 0;

  /// @brief remove one object from the manager
  virtual void unregisterObject(CollisionObject* obj) = 0;

  /// @brief initialize the manager, related with the specific type of manager
  virtual void setup() = 0;

  /// @brief update the condition of manager
  virtual void update() = 0;

  /// @brief update the manager by explicitly given the object updated
  virtual void update(CollisionObject* updated_obj)
  {
    update();
  }

  /// @brief update the manager by explicitly given the set of objects update
  virtual void update(const std::vector<CollisionObject*>& updated_objs)
  {
    update();
  }

  /// @brief clear the manager
  virtual void clear() = 0;

  /// @brief return the objects managed by the manager
  virtual void getObjects(std::vector<CollisionObject*>& objs) const = 0;

  /// @brief perform collision test between one object and all the objects belonging to the manager
  virtual void collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const = 0;

  /// @brief perform distance computation between one object and all the objects belonging to the manager
  virtual void distance(CollisionObject* obj, void* cdata, DistanceCallBack callback) const = 0;

  /// @brief perform collision test for the objects belonging to the manager (i.e., N^2 self collision)
  virtual void collide(void* cdata, CollisionCallBack callback) const = 0;

  /// @brief perform distance test for the objects belonging to the manager (i.e., N^2 self distance)
  virtual void distance(void* cdata, DistanceCallBack callback) const = 0;

  /// @brief perform collision test with objects belonging to another manager
  virtual void collide(BroadPhaseCollisionManager* other_manager, void* cdata, CollisionCallBack callback) const = 0;

  /// @brief perform distance test with objects belonging to another manager
  virtual void distance(BroadPhaseCollisionManager* other_manager, void* cdata, DistanceCallBack callback) const = 0;

  /// @brief whether the manager is empty
  virtual bool empty() const = 0;
  
  /// @brief the number of objects managed by the manager
  virtual size_t size() const = 0;

protected:

  /// @brief tools help to avoid repeating collision or distance callback for the pairs of objects tested before. It can be useful for some of the broadphase algorithms.
  mutable std::set<std::pair<CollisionObject*, CollisionObject*> > tested_set;
  mutable bool enable_tested_set_;

  bool inTestedSet(CollisionObject* a, CollisionObject* b) const
  {
    if(a < b) return tested_set.find(std::make_pair(a, b)) != tested_set.end();
    else return tested_set.find(std::make_pair(b, a)) != tested_set.end();
  }

  void insertTestedSet(CollisionObject* a, CollisionObject* b) const
  {
    if(a < b) tested_set.insert(std::make_pair(a, b));
    else tested_set.insert(std::make_pair(b, a));
  }

};


/// @brief Callback for continuous collision between two objects. Return value is whether can stop now.
typedef bool (*ContinuousCollisionCallBack)(ContinuousCollisionObject* o1, ContinuousCollisionObject* o2, void* cdata);

/// @brief Callback for continuous distance between two objects, Return value is whether can stop now, also return the minimum distance till now.
typedef bool (*ContinuousDistanceCallBack)(ContinuousCollisionObject* o1, ContinuousCollisionObject* o2, void* cdata, FCL_REAL& dist);


/// @brief Base class for broad phase continuous collision. It helps to accelerate the continuous collision/distance between N objects. Also support self collision, self distance and collision/distance with another M objects.
class BroadPhaseContinuousCollisionManager
{
public:
  BroadPhaseContinuousCollisionManager()
  {
  }

  virtual ~BroadPhaseContinuousCollisionManager() {}

  /// @brief add objects to the manager
  virtual void registerObjects(const std::vector<ContinuousCollisionObject*>& other_objs)
  {
    for(size_t i = 0; i < other_objs.size(); ++i)
      registerObject(other_objs[i]);
  }

  /// @brief add one object to the manager
  virtual void registerObject(ContinuousCollisionObject* obj) = 0;

  /// @brief remove one object from the manager
  virtual void unregisterObject(ContinuousCollisionObject* obj) = 0;

  /// @brief initialize the manager, related with the specific type of manager
  virtual void setup() = 0;

  /// @brief update the condition of manager
  virtual void update() = 0;

  /// @brief update the manager by explicitly given the object updated
  virtual void update(ContinuousCollisionObject* updated_obj)
  {
    update();
  }

  /// @brief update the manager by explicitly given the set of objects update
  virtual void update(const std::vector<ContinuousCollisionObject*>& updated_objs)
  {
    update();
  }

  /// @brief clear the manager
  virtual void clear() = 0;

  /// @brief return the objects managed by the manager
  virtual void getObjects(std::vector<ContinuousCollisionObject*>& objs) const = 0;

  /// @brief perform collision test between one object and all the objects belonging to the manager
  virtual void collide(ContinuousCollisionObject* obj, void* cdata, CollisionCallBack callback) const = 0;

  /// @brief perform distance computation between one object and all the objects belonging to the manager
  virtual void distance(ContinuousCollisionObject* obj, void* cdata, DistanceCallBack callback) const = 0;

  /// @brief perform collision test for the objects belonging to the manager (i.e., N^2 self collision)
  virtual void collide(void* cdata, CollisionCallBack callback) const = 0;

  /// @brief perform distance test for the objects belonging to the manager (i.e., N^2 self distance)
  virtual void distance(void* cdata, DistanceCallBack callback) const = 0;

  /// @brief perform collision test with objects belonging to another manager
  virtual void collide(BroadPhaseContinuousCollisionManager* other_manager, void* cdata, CollisionCallBack callback) const = 0;

  /// @brief perform distance test with objects belonging to another manager
  virtual void distance(BroadPhaseContinuousCollisionManager* other_manager, void* cdata, DistanceCallBack callback) const = 0;

  /// @brief whether the manager is empty
  virtual bool empty() const = 0;
  
  /// @brief the number of objects managed by the manager
  virtual size_t size() const = 0;
};


}


#include "fcl/broadphase/broadphase_bruteforce.h"
#include "fcl/broadphase/broadphase_spatialhash.h"
#include "fcl/broadphase/broadphase_SaP.h"
#include "fcl/broadphase/broadphase_SSaP.h"
#include "fcl/broadphase/broadphase_interval_tree.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree_array.h"

#endif

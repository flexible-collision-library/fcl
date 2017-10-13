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

#ifndef FCL_BROAD_PHASE_SAP_H
#define FCL_BROAD_PHASE_SAP_H

#include <map>
#include <list>

#include "fcl/broadphase/broadphase_collision_manager.h"

namespace fcl
{

/// @brief Rigorous SAP collision manager
template <typename S>
class FCL_EXPORT SaPCollisionManager : public BroadPhaseCollisionManager<S>
{
public:

  SaPCollisionManager();

  ~SaPCollisionManager();

  /// @brief add objects to the manager
  void registerObjects(const std::vector<CollisionObject<S>*>& other_objs);

  /// @brief remove one object from the manager
  void registerObject(CollisionObject<S>* obj);

  /// @brief add one object to the manager
  void unregisterObject(CollisionObject<S>* obj);

  /// @brief initialize the manager, related with the specific type of manager
  void setup();

  /// @brief update the condition of manager
  void update();

  /// @brief update the manager by explicitly given the object updated
  void update(CollisionObject<S>* updated_obj);

  /// @brief update the manager by explicitly given the set of objects update
  void update(const std::vector<CollisionObject<S>*>& updated_objs);

  /// @brief clear the manager
  void clear();

  /// @brief return the objects managed by the manager
  void getObjects(std::vector<CollisionObject<S>*>& objs) const;

  /// @brief perform collision test between one object and all the objects belonging to the manager
  void collide(CollisionObject<S>* obj, void* cdata, CollisionCallBack<S> callback) const;

  /// @brief perform distance computation between one object and all the objects belonging to the manager
  void distance(CollisionObject<S>* obj, void* cdata, DistanceCallBack<S> callback) const;

  /// @brief perform collision test for the objects belonging to the manager (i.e., N^2 self collision)
  void collide(void* cdata, CollisionCallBack<S> callback) const;

  /// @brief perform distance test for the objects belonging to the manager (i.e., N^2 self distance)
  void distance(void* cdata, DistanceCallBack<S> callback) const;

  /// @brief perform collision test with objects belonging to another manager
  void collide(BroadPhaseCollisionManager<S>* other_manager, void* cdata, CollisionCallBack<S> callback) const;

  /// @brief perform distance test with objects belonging to another manager
  void distance(BroadPhaseCollisionManager<S>* other_manager, void* cdata, DistanceCallBack<S> callback) const;

  /// @brief whether the manager is empty
  bool empty() const;
  
  /// @brief the number of objects managed by the manager
  size_t size() const;

protected:

  /// @brief SAP interval for one object
  struct SaPAABB;

  /// @brief End point for an interval
  struct EndPoint;

  /// @brief A pair of objects that are not culling away and should further check collision
  struct SaPPair;

  /// @brief Functor to help unregister one object
  class FCL_EXPORT isUnregistered;

  /// @brief Functor to help remove collision pairs no longer valid (i.e., should be culled away)
  class FCL_EXPORT isNotValidPair;

  void update_(SaPAABB* updated_aabb);

  void updateVelist();

  /// @brief End point list for x, y, z coordinates
  EndPoint* elist[3];
  
  /// @brief vector version of elist, for acceleration
  std::vector<EndPoint*> velist[3];

  /// @brief SAP interval list
  std::list<SaPAABB*> AABB_arr;

  /// @brief The pair of objects that should further check for collision
  std::list<SaPPair> overlap_pairs;

  size_t optimal_axis;

  std::map<CollisionObject<S>*, SaPAABB*> obj_aabb_map;

  bool distance_(CollisionObject<S>* obj, void* cdata, DistanceCallBack<S> callback, S& min_dist) const;

  bool collide_(CollisionObject<S>* obj, void* cdata, CollisionCallBack<S> callback) const;

  void addToOverlapPairs(const SaPPair& p);

  void removeFromOverlapPairs(const SaPPair& p);
};

using SaPCollisionManagerf = SaPCollisionManager<float>;
using SaPCollisionManagerd = SaPCollisionManager<double>;

/// @brief SAP interval for one object
template <typename S>
struct SaPCollisionManager<S>::SaPAABB
{
  /// @brief object
  CollisionObject<S>* obj;

  /// @brief lower bound end point of the interval
  typename SaPCollisionManager<S>::EndPoint* lo;

  /// @brief higher bound end point of the interval
  typename SaPCollisionManager<S>::EndPoint* hi;

  /// @brief cached AABB<S> value
  AABB<S> cached;
};

/// @brief End point for an interval
template <typename S>
struct SaPCollisionManager<S>::EndPoint
{
  /// @brief tag for whether it is a lower bound or higher bound of an interval, 0 for lo, and 1 for hi
  char minmax;

  /// @brief back pointer to SAP interval
  typename SaPCollisionManager<S>::SaPAABB* aabb;

  /// @brief the previous end point in the end point list
  EndPoint* prev[3];

  /// @brief the next end point in the end point list
  EndPoint* next[3];

  /// @brief get the value of the end point
  const Vector3<S>& getVal() const;

  /// @brief set the value of the end point
  Vector3<S>& getVal();

  S getVal(size_t i) const;

  S& getVal(size_t i);

};

/// @brief A pair of objects that are not culling away and should further check collision
template <typename S>
struct SaPCollisionManager<S>::SaPPair
{
  SaPPair(CollisionObject<S>* a, CollisionObject<S>* b);

  CollisionObject<S>* obj1;
  CollisionObject<S>* obj2;

  bool operator == (const SaPPair& other) const;
};

/// @brief Functor to help unregister one object
template <typename S>
class FCL_EXPORT SaPCollisionManager<S>::isUnregistered
{
  CollisionObject<S>* obj;

public:
  isUnregistered(CollisionObject<S>* obj_);

  bool operator() (const SaPPair& pair) const;
};

/// @brief Functor to help remove collision pairs no longer valid (i.e., should be culled away)
template <typename S>
class FCL_EXPORT SaPCollisionManager<S>::isNotValidPair
{
  CollisionObject<S>* obj1;
  CollisionObject<S>* obj2;

public:
  isNotValidPair(CollisionObject<S>* obj1_, CollisionObject<S>* obj2_);

  bool operator() (const SaPPair& pair);
};

} // namespace fcl

#include "fcl/broadphase/broadphase_SaP-inl.h"

#endif

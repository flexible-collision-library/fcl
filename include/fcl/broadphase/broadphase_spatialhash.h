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

#ifndef FCL_BROADPHASE_BROADPAHSESPATIALHASH_H
#define FCL_BROADPHASE_BROADPAHSESPATIALHASH_H

#include <list>
#include <map>
#include "fcl/math/bv/AABB.h"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include "fcl/broadphase/detail/simple_hash_table.h"
#include "fcl/broadphase/detail/sparse_hash_table.h"
#include "fcl/broadphase/detail/spatial_hash.h"

namespace fcl
{

/// @brief spatial hashing collision mananger
template<typename S,
         typename HashTable
             = detail::SimpleHashTable<AABB<S>, CollisionObject<S>*, detail::SpatialHash<S>> >
class FCL_EXPORT SpatialHashingCollisionManager : public BroadPhaseCollisionManager<S>
{
public:
  SpatialHashingCollisionManager(
      S cell_size,
      const Vector3<S>& scene_min,
      const Vector3<S>& scene_max,
      unsigned int default_table_size = 1000);

  ~SpatialHashingCollisionManager();

  /// @brief add one object to the manager
  void registerObject(CollisionObject<S>* obj);

  /// @brief remove one object from the manager
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

  /// @brief perform distance computation between one object and all the objects belonging ot the manager
  void distance(CollisionObject<S>* obj, void* cdata, DistanceCallBack<S> callback) const;

  /// @brief perform collision test for the objects belonging to the manager (i.e, N^2 self collision)
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

  /// @brief compute the bound for the environent
  static void computeBound(std::vector<CollisionObject<S>*>& objs, Vector3<S>& l, Vector3<S>& u);

protected:

  /// @brief perform collision test between one object and all the objects belonging to the manager
  bool collide_(CollisionObject<S>* obj, void* cdata, CollisionCallBack<S> callback) const;

  /// @brief perform distance computation between one object and all the objects belonging ot the manager
  bool distance_(CollisionObject<S>* obj, void* cdata, DistanceCallBack<S> callback, S& min_dist) const;

  /// @brief all objects in the scene
  std::list<CollisionObject<S>*> objs;

  /// @brief objects partially penetrating (not totally inside nor outside) the
  /// scene limit are in another list
  std::list<CollisionObject<S>*> objs_partially_penetrating_scene_limit;

  /// @brief objects outside the scene limit are in another list
  std::list<CollisionObject<S>*> objs_outside_scene_limit;

  /// @brief the size of the scene
  AABB<S> scene_limit;

  /// @brief store the map between objects and their aabbs. will make update more convenient
  std::map<CollisionObject<S>*, AABB<S>> obj_aabb_map;

  /// @brief objects in the scene limit (given by scene_min and scene_max) are in the spatial hash table
  HashTable* hash_table;

private:

  enum ObjectStatus
  {
    Inside,
    PartiallyPenetrating,
    Outside
  };

  template <typename Container>
  bool distanceObjectToObjects(
      CollisionObject<S>* obj,
      const Container& objs,
      void* cdata,
      DistanceCallBack<S> callback,
      S& min_dist) const;

};

template<typename HashTable = detail::SimpleHashTable<AABB<float>, CollisionObject<float>*, detail::SpatialHash<float>>>
using SpatialHashingCollisionManagerf = SpatialHashingCollisionManager<float, HashTable>;

template<typename HashTable = detail::SimpleHashTable<AABB<double>, CollisionObject<double>*, detail::SpatialHash<double>>>
using SpatialHashingCollisionManagerd = SpatialHashingCollisionManager<double, HashTable>;

} // namespace fcl

#include "fcl/broadphase/broadphase_spatialhash-inl.h"

#endif

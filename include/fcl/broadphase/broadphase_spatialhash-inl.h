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

#ifndef FCL_BROADPHASE_BROADPAHSESPATIALHASH_INL_H
#define FCL_BROADPHASE_BROADPAHSESPATIALHASH_INL_H

#include "fcl/broadphase/broadphase_spatialhash.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT SpatialHashingCollisionManager<
    double,
    detail::SimpleHashTable<
        AABB<double>, CollisionObject<double>*, detail::SpatialHash<double>>>;

//==============================================================================
template<typename S, typename HashTable>
SpatialHashingCollisionManager<S, HashTable>::SpatialHashingCollisionManager(
    S cell_size,
    const Vector3<S>& scene_min,
    const Vector3<S>& scene_max,
    unsigned int default_table_size)
  : scene_limit(AABB<S>(scene_min, scene_max)),
    hash_table(new HashTable(detail::SpatialHash<S>(scene_limit, cell_size)))
{
  hash_table->init(default_table_size);
}

//==============================================================================
template<typename S, typename HashTable>
SpatialHashingCollisionManager<S, HashTable>::~SpatialHashingCollisionManager()
{
  delete hash_table;
}

//==============================================================================
template<typename S, typename HashTable>
void SpatialHashingCollisionManager<S, HashTable>::registerObject(
    CollisionObject<S>* obj)
{
  objs.push_back(obj);

  const AABB<S>& obj_aabb = obj->getAABB();
  AABB<S> overlap_aabb;

  if(scene_limit.overlap(obj_aabb, overlap_aabb))
  {
    if(!scene_limit.contain(obj_aabb))
      objs_partially_penetrating_scene_limit.push_back(obj);

    hash_table->insert(overlap_aabb, obj);
  }
  else
  {
    objs_outside_scene_limit.push_back(obj);
  }

  obj_aabb_map[obj] = obj_aabb;
}

//==============================================================================
template<typename S, typename HashTable>
void SpatialHashingCollisionManager<S, HashTable>::unregisterObject(CollisionObject<S>* obj)
{
  objs.remove(obj);

  const AABB<S>& obj_aabb = obj->getAABB();
  AABB<S> overlap_aabb;

  if(scene_limit.overlap(obj_aabb, overlap_aabb))
  {
    if(!scene_limit.contain(obj_aabb))
      objs_partially_penetrating_scene_limit.remove(obj);

    hash_table->remove(overlap_aabb, obj);
  }
  else
  {
    objs_outside_scene_limit.remove(obj);
  }

  obj_aabb_map.erase(obj);
}

//==============================================================================
template<typename S, typename HashTable>
void SpatialHashingCollisionManager<S, HashTable>::setup()
{
  // Do nothing
}

//==============================================================================
template<typename S, typename HashTable>
void SpatialHashingCollisionManager<S, HashTable>::update()
{
  hash_table->clear();
  objs_partially_penetrating_scene_limit.clear();
  objs_outside_scene_limit.clear();

  for(auto it = objs.cbegin(), end = objs.cend(); it != end; ++it)
  {
    CollisionObject<S>* obj = *it;
    const AABB<S>& obj_aabb = obj->getAABB();
    AABB<S> overlap_aabb;

    if(scene_limit.overlap(obj_aabb, overlap_aabb))
    {
      if(!scene_limit.contain(obj_aabb))
        objs_partially_penetrating_scene_limit.push_back(obj);

      hash_table->insert(overlap_aabb, obj);
    }
    else
    {
      objs_outside_scene_limit.push_back(obj);
    }

    obj_aabb_map[obj] = obj_aabb;
  }
}

//==============================================================================
template<typename S, typename HashTable>
void SpatialHashingCollisionManager<S, HashTable>::update(CollisionObject<S>* updated_obj)
{
  const AABB<S>& new_aabb = updated_obj->getAABB();
  const AABB<S>& old_aabb = obj_aabb_map[updated_obj];

  AABB<S> old_overlap_aabb;
  const auto is_old_aabb_overlapping
      = scene_limit.overlap(old_aabb, old_overlap_aabb);
  if(is_old_aabb_overlapping)
    hash_table->remove(old_overlap_aabb, updated_obj);

  AABB<S> new_overlap_aabb;
  const auto is_new_aabb_overlapping
      = scene_limit.overlap(new_aabb, new_overlap_aabb);
  if(is_new_aabb_overlapping)
    hash_table->insert(new_overlap_aabb, updated_obj);

  ObjectStatus old_status;
  if(is_old_aabb_overlapping)
  {
    if(scene_limit.contain(old_aabb))
      old_status = Inside;
    else
      old_status = PartiallyPenetrating;
  }
  else
  {
    old_status = Outside;
  }

  if(is_new_aabb_overlapping)
  {
    if(scene_limit.contain(new_aabb))
    {
      if (old_status == PartiallyPenetrating)
      {
        // Status change: PartiallyPenetrating --> Inside
        // Required action(s):
        // - remove object from "objs_partially_penetrating_scene_limit"

        auto find_it = std::find(objs_partially_penetrating_scene_limit.begin(),
                                 objs_partially_penetrating_scene_limit.end(),
                                 updated_obj);
        objs_partially_penetrating_scene_limit.erase(find_it);
      }
      else if (old_status == Outside)
      {
        // Status change: Outside --> Inside
        // Required action(s):
        // - remove object from "objs_outside_scene_limit"

        auto find_it = std::find(objs_outside_scene_limit.begin(),
                                 objs_outside_scene_limit.end(),
                                 updated_obj);
        objs_outside_scene_limit.erase(find_it);
      }
    }
    else
    {
      if (old_status == Inside)
      {
        // Status change: Inside --> PartiallyPenetrating
        // Required action(s):
        // - add object to "objs_partially_penetrating_scene_limit"

        objs_partially_penetrating_scene_limit.push_back(updated_obj);
      }
      else if (old_status == Outside)
      {
        // Status change: Outside --> PartiallyPenetrating
        // Required action(s):
        // - remove object from "objs_outside_scene_limit"
        // - add object to "objs_partially_penetrating_scene_limit"

        auto find_it = std::find(objs_outside_scene_limit.begin(),
                                 objs_outside_scene_limit.end(),
                                 updated_obj);
        objs_outside_scene_limit.erase(find_it);

        objs_partially_penetrating_scene_limit.push_back(updated_obj);
      }
    }
  }
  else
  {
    if (old_status == Inside)
    {
      // Status change: Inside --> Outside
      // Required action(s):
      // - add object to "objs_outside_scene_limit"

      objs_outside_scene_limit.push_back(updated_obj);
    }
    else if (old_status == PartiallyPenetrating)
    {
      // Status change: PartiallyPenetrating --> Outside
      // Required action(s):
      // - remove object from "objs_partially_penetrating_scene_limit"
      // - add object to "objs_outside_scene_limit"

      auto find_it = std::find(objs_partially_penetrating_scene_limit.begin(),
                               objs_partially_penetrating_scene_limit.end(),
                               updated_obj);
      objs_partially_penetrating_scene_limit.erase(find_it);

      objs_outside_scene_limit.push_back(updated_obj);
    }
  }

  obj_aabb_map[updated_obj] = new_aabb;
}

//==============================================================================
template<typename S, typename HashTable>
void SpatialHashingCollisionManager<S, HashTable>::update(const std::vector<CollisionObject<S>*>& updated_objs)
{
  for(size_t i = 0; i < updated_objs.size(); ++i)
    update(updated_objs[i]);
}

//==============================================================================
template<typename S, typename HashTable>
void SpatialHashingCollisionManager<S, HashTable>::clear()
{
  objs.clear();
  hash_table->clear();
  objs_outside_scene_limit.clear();
  obj_aabb_map.clear();
}

//==============================================================================
template<typename S, typename HashTable>
void SpatialHashingCollisionManager<S, HashTable>::getObjects(std::vector<CollisionObject<S>*>& objs_) const
{
  objs_.resize(objs.size());
  std::copy(objs.begin(), objs.end(), objs_.begin());
}

//==============================================================================
template<typename S, typename HashTable>
void SpatialHashingCollisionManager<S, HashTable>::collide(CollisionObject<S>* obj, void* cdata, CollisionCallBack<S> callback) const
{
  if(size() == 0) return;
  collide_(obj, cdata, callback);
}

//==============================================================================
template<typename S, typename HashTable>
void SpatialHashingCollisionManager<S, HashTable>::distance(CollisionObject<S>* obj, void* cdata, DistanceCallBack<S> callback) const
{
  if(size() == 0) return;
  S min_dist = std::numeric_limits<S>::max();
  distance_(obj, cdata, callback, min_dist);
}

//==============================================================================
template<typename S, typename HashTable>
bool SpatialHashingCollisionManager<S, HashTable>::collide_(
    CollisionObject<S>* obj, void* cdata, CollisionCallBack<S> callback) const
{
  const auto& obj_aabb = obj->getAABB();
  AABB<S> overlap_aabb;

  if(scene_limit.overlap(obj_aabb, overlap_aabb))
  {
    const auto query_result = hash_table->query(overlap_aabb);
    for(const auto& obj2 : query_result)
    {
      if(obj == obj2)
        continue;

      if(callback(obj, obj2, cdata))
        return true;
    }

    if(!scene_limit.contain(obj_aabb))
    {
      for(const auto& obj2 : objs_outside_scene_limit)
      {
        if(obj == obj2)
          continue;

        if(callback(obj, obj2, cdata))
          return true;
      }
    }
  }
  else
  {
    for(const auto& obj2 : objs_partially_penetrating_scene_limit)
    {
      if(obj == obj2)
        continue;

      if(callback(obj, obj2, cdata))
        return true;
    }

    for(const auto& obj2 : objs_outside_scene_limit)
    {
      if(obj == obj2)
        continue;

      if(callback(obj, obj2, cdata))
        return true;
    }
  }

  return false;
}

//==============================================================================
template<typename S, typename HashTable>
bool SpatialHashingCollisionManager<S, HashTable>::distance_(
    CollisionObject<S>* obj, void* cdata, DistanceCallBack<S> callback, S& min_dist) const
{
  auto delta = (obj->getAABB().max_ - obj->getAABB().min_) * 0.5;
  auto aabb = obj->getAABB();
  if(min_dist < std::numeric_limits<S>::max())
  {
    Vector3<S> min_dist_delta(min_dist, min_dist, min_dist);
    aabb.expand(min_dist_delta);
  }

  AABB<S> overlap_aabb;

  auto status = 1;
  S old_min_distance;

  while(1)
  {
    old_min_distance = min_dist;

    if(scene_limit.overlap(aabb, overlap_aabb))
    {
      if (distanceObjectToObjects(
            obj, hash_table->query(overlap_aabb), cdata, callback, min_dist))
      {
        return true;
      }

      if(!scene_limit.contain(aabb))
      {
        if (distanceObjectToObjects(
              obj, objs_outside_scene_limit, cdata, callback, min_dist))
        {
          return true;
        }
      }
    }
    else
    {
      if (distanceObjectToObjects(
            obj, objs_partially_penetrating_scene_limit, cdata, callback, min_dist))
      {
        return true;
      }

      if (distanceObjectToObjects(
            obj, objs_outside_scene_limit, cdata, callback, min_dist))
      {
        return true;
      }
    }

    if(status == 1)
    {
      if(old_min_distance < std::numeric_limits<S>::max())
      {
        break;
      }
      else
      {
        if(min_dist < old_min_distance)
        {
          Vector3<S> min_dist_delta(min_dist, min_dist, min_dist);
          aabb = AABB<S>(obj->getAABB(), min_dist_delta);
          status = 0;
        }
        else
        {
          if(aabb.equal(obj->getAABB()))
            aabb.expand(delta);
          else
            aabb.expand(obj->getAABB(), 2.0);
        }
      }
    }
    else if(status == 0)
    {
      break;
    }
  }

  return false;
}

//==============================================================================
template<typename S, typename HashTable>
void SpatialHashingCollisionManager<S, HashTable>::collide(
    void* cdata, CollisionCallBack<S> callback) const
{
  if(size() == 0)
    return;

  for(const auto& obj1 : objs)
  {
    const auto& obj_aabb = obj1->getAABB();
    AABB<S> overlap_aabb;

    if(scene_limit.overlap(obj_aabb, overlap_aabb))
    {
      auto query_result = hash_table->query(overlap_aabb);
      for(const auto& obj2 : query_result)
      {
        if(obj1 < obj2)
        {
          if(callback(obj1, obj2, cdata))
            return;
        }
      }

      if(!scene_limit.contain(obj_aabb))
      {
        for(const auto& obj2 : objs_outside_scene_limit)
        {
          if(obj1 < obj2)
          {
            if(callback(obj1, obj2, cdata))
              return;
          }
        }
      }
    }
    else
    {
      for(const auto& obj2 : objs_partially_penetrating_scene_limit)
      {
        if(obj1 < obj2)
        {
          if(callback(obj1, obj2, cdata))
            return;
        }
      }

      for(const auto& obj2 : objs_outside_scene_limit)
      {
        if(obj1 < obj2)
        {
          if(callback(obj1, obj2, cdata))
            return;
        }
      }
    }
  }
}

//==============================================================================
template<typename S, typename HashTable>
void SpatialHashingCollisionManager<S, HashTable>::distance(
    void* cdata, DistanceCallBack<S> callback) const
{
  if(size() == 0)
    return;

  this->enable_tested_set_ = true;
  this->tested_set.clear();

  S min_dist = std::numeric_limits<S>::max();

  for(const auto& obj : objs)
  {
    if(distance_(obj, cdata, callback, min_dist))
      break;
  }

  this->enable_tested_set_ = false;
  this->tested_set.clear();
}

//==============================================================================
template<typename S, typename HashTable>
void SpatialHashingCollisionManager<S, HashTable>::collide(BroadPhaseCollisionManager<S>* other_manager_, void* cdata, CollisionCallBack<S> callback) const
{
  auto* other_manager = static_cast<SpatialHashingCollisionManager<S, HashTable>* >(other_manager_);

  if((size() == 0) || (other_manager->size() == 0))
    return;

  if(this == other_manager)
  {
    collide(cdata, callback);
    return;
  }

  if(this->size() < other_manager->size())
  {
    for(const auto& obj : objs)
    {
      if(other_manager->collide_(obj, cdata, callback))
        return;
    }
  }
  else
  {
    for(const auto& obj : other_manager->objs)
    {
      if(collide_(obj, cdata, callback))
        return;
    }
  }
}

//==============================================================================
template<typename S, typename HashTable>
void SpatialHashingCollisionManager<S, HashTable>::distance(BroadPhaseCollisionManager<S>* other_manager_, void* cdata, DistanceCallBack<S> callback) const
{
  auto* other_manager = static_cast<SpatialHashingCollisionManager<S, HashTable>* >(other_manager_);

  if((size() == 0) || (other_manager->size() == 0))
    return;

  if(this == other_manager)
  {
    distance(cdata, callback);
    return;
  }

  S min_dist = std::numeric_limits<S>::max();

  if(this->size() < other_manager->size())
  {
    for(const auto& obj : objs)
      if(other_manager->distance_(obj, cdata, callback, min_dist)) return;
  }
  else
  {
    for(const auto& obj : other_manager->objs)
      if(distance_(obj, cdata, callback, min_dist)) return;
  }
}

//==============================================================================
template<typename S, typename HashTable>
bool SpatialHashingCollisionManager<S, HashTable>::empty() const
{
  return objs.empty();
}

//==============================================================================
template<typename S, typename HashTable>
size_t SpatialHashingCollisionManager<S, HashTable>::size() const
{
  return objs.size();
}

//==============================================================================
template<typename S, typename HashTable>
void SpatialHashingCollisionManager<S, HashTable>::computeBound(
    std::vector<CollisionObject<S>*>& objs, Vector3<S>& l, Vector3<S>& u)
{
  AABB<S> bound;
  for(unsigned int i = 0; i < objs.size(); ++i)
    bound += objs[i]->getAABB();

  l = bound.min_;
  u = bound.max_;
}

//==============================================================================
template<typename S, typename HashTable>
template<typename Container>
bool SpatialHashingCollisionManager<S, HashTable>::distanceObjectToObjects(
    CollisionObject<S>* obj,
    const Container& objs,
    void* cdata,
    DistanceCallBack<S> callback,
    S& min_dist) const
{
  for(auto& obj2 : objs)
  {
    if(obj == obj2)
      continue;

    if(!this->enable_tested_set_)
    {
      if(obj->getAABB().distance(obj2->getAABB()) < min_dist)
      {
        if(callback(obj, obj2, cdata, min_dist))
          return true;
      }
    }
    else
    {
      if(!this->inTestedSet(obj, obj2))
      {
        if(obj->getAABB().distance(obj2->getAABB()) < min_dist)
        {
          if(callback(obj, obj2, cdata, min_dist))
            return true;
        }

        this->insertTestedSet(obj, obj2);
      }
    }
  }

  return false;
}

} // namespace fcl

#endif

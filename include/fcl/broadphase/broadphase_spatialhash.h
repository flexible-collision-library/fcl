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

#ifndef FCL_BROAD_PHASE_SPATIAL_HASH_H
#define FCL_BROAD_PHASE_SPATIAL_HASH_H

#include "fcl/broadphase/broadphase.h"
#include "fcl/broadphase/hash.h"
#include "fcl/BV/AABB.h"
#include <list>
#include <map>

namespace fcl
{

/// @brief Spatial hash function: hash an AABB to a set of integer values
template <typename Scalar>
struct SpatialHash
{
  SpatialHash(const AABB<Scalar>& scene_limit_, Scalar cell_size_) : cell_size(cell_size_),
                                                               scene_limit(scene_limit_)
  {
    width[0] = std::ceil(scene_limit.width() / cell_size);
    width[1] = std::ceil(scene_limit.height() / cell_size);
    width[2] = std::ceil(scene_limit.depth() / cell_size);
  }
    
  std::vector<unsigned int> operator() (const AABB<Scalar>& aabb) const
  {
    int min_x = std::floor((aabb.min_[0] - scene_limit.min_[0]) / cell_size);
    int max_x = std::ceil((aabb.max_[0] - scene_limit.min_[0]) / cell_size);
    int min_y = std::floor((aabb.min_[1] - scene_limit.min_[1]) / cell_size);
    int max_y = std::ceil((aabb.max_[1] - scene_limit.min_[1]) / cell_size);
    int min_z = std::floor((aabb.min_[2] - scene_limit.min_[2]) / cell_size);
    int max_z = std::ceil((aabb.max_[2] - scene_limit.min_[2]) / cell_size);

    std::vector<unsigned int> keys((max_x - min_x) * (max_y - min_y) * (max_z - min_z));
    int id = 0;
    for(int x = min_x; x < max_x; ++x)
    {
      for(int y = min_y; y < max_y; ++y)
      {
        for(int z = min_z; z < max_z; ++z)
        {
          keys[id++] = x + y * width[0] + z * width[0] * width[1];
        }
      }
    }
    return keys;
  }

private:

  Scalar cell_size;
  AABB<Scalar> scene_limit;
  unsigned int width[3];
};

using SpatialHashf = SpatialHash<float>;
using SpatialHashd = SpatialHash<double>;

/// @brief spatial hashing collision mananger
template<typename Scalar, typename HashTable = SimpleHashTable<AABB<Scalar>, CollisionObject<Scalar>*, SpatialHash<Scalar>> >
class SpatialHashingCollisionManager : public BroadPhaseCollisionManager<Scalar>
{
public:
  SpatialHashingCollisionManager(
      Scalar cell_size,
      const Vector3<Scalar>& scene_min,
      const Vector3<Scalar>& scene_max,
      unsigned int default_table_size = 1000)
    : scene_limit(AABB<Scalar>(scene_min, scene_max)),
      hash_table(new HashTable(SpatialHash<Scalar>(scene_limit, cell_size)))
  {
    hash_table->init(default_table_size);
  }

  ~SpatialHashingCollisionManager()
  {
    delete hash_table;
  }

  /// @brief add one object to the manager
  void registerObject(CollisionObject<Scalar>* obj);

  /// @brief remove one object from the manager
  void unregisterObject(CollisionObject<Scalar>* obj);

  /// @brief initialize the manager, related with the specific type of manager
  void setup();

  /// @brief update the condition of manager
  void update();

  /// @brief update the manager by explicitly given the object updated
  void update(CollisionObject<Scalar>* updated_obj);

  /// @brief update the manager by explicitly given the set of objects update
  void update(const std::vector<CollisionObject<Scalar>*>& updated_objs);

  /// @brief clear the manager
  void clear();

  /// @brief return the objects managed by the manager
  void getObjects(std::vector<CollisionObject<Scalar>*>& objs) const;

  /// @brief perform collision test between one object and all the objects belonging to the manager
  void collide(CollisionObject<Scalar>* obj, void* cdata, CollisionCallBack<Scalar> callback) const;

  /// @brief perform distance computation between one object and all the objects belonging ot the manager
  void distance(CollisionObject<Scalar>* obj, void* cdata, DistanceCallBack<Scalar> callback) const;

  /// @brief perform collision test for the objects belonging to the manager (i.e, N^2 self collision)
  void collide(void* cdata, CollisionCallBack<Scalar> callback) const;

  /// @brief perform distance test for the objects belonging to the manager (i.e., N^2 self distance)
  void distance(void* cdata, DistanceCallBack<Scalar> callback) const;

  /// @brief perform collision test with objects belonging to another manager
  void collide(BroadPhaseCollisionManager<Scalar>* other_manager, void* cdata, CollisionCallBack<Scalar> callback) const;

  /// @brief perform distance test with objects belonging to another manager
  void distance(BroadPhaseCollisionManager<Scalar>* other_manager, void* cdata, DistanceCallBack<Scalar> callback) const;

  /// @brief whether the manager is empty
  bool empty() const;

  /// @brief the number of objects managed by the manager
  size_t size() const;

  /// @brief compute the bound for the environent
  static void computeBound(std::vector<CollisionObject<Scalar>*>& objs, Vector3<Scalar>& l, Vector3<Scalar>& u)
  {
    AABB<Scalar> bound;
    for(unsigned int i = 0; i < objs.size(); ++i)
      bound += objs[i]->getAABB();
    
    l = bound.min_;
    u = bound.max_;
  }

protected:

  /// @brief perform collision test between one object and all the objects belonging to the manager
  bool collide_(CollisionObject<Scalar>* obj, void* cdata, CollisionCallBack<Scalar> callback) const;

  /// @brief perform distance computation between one object and all the objects belonging ot the manager
  bool distance_(CollisionObject<Scalar>* obj, void* cdata, DistanceCallBack<Scalar> callback, Scalar& min_dist) const;


  /// @brief all objects in the scene
  std::list<CollisionObject<Scalar>*> objs;

  /// @brief objects outside the scene limit are in another list
  std::list<CollisionObject<Scalar>*> objs_outside_scene_limit;

  /// @brief the size of the scene
  AABB<Scalar> scene_limit;

  /// @brief store the map between objects and their aabbs. will make update more convenient
  std::map<CollisionObject<Scalar>*, AABB<Scalar>> obj_aabb_map;

  /// @brief objects in the scene limit (given by scene_min and scene_max) are in the spatial hash table
  HashTable* hash_table;

};

template<typename HashTable = SimpleHashTable<AABB<float>, CollisionObject<float>*, SpatialHash<float>>>
using SpatialHashingCollisionManagerf = SpatialHashingCollisionManager<float, HashTable>;

template<typename HashTable = SimpleHashTable<AABB<double>, CollisionObject<double>*, SpatialHash<double>>>
using SpatialHashingCollisionManagerd = SpatialHashingCollisionManager<double, HashTable>;

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template<typename Scalar, typename HashTable>
void SpatialHashingCollisionManager<Scalar, HashTable>::registerObject(CollisionObject<Scalar>* obj)
{
  objs.push_back(obj);

  const AABB<Scalar>& obj_aabb = obj->getAABB();
  AABB<Scalar> overlap_aabb;

  if(scene_limit.overlap(obj_aabb, overlap_aabb))
  {
    if(!scene_limit.contain(obj_aabb))
      objs_outside_scene_limit.push_back(obj);

    hash_table->insert(overlap_aabb, obj);
  }
  else
    objs_outside_scene_limit.push_back(obj);

  obj_aabb_map[obj] = obj_aabb;
}

template<typename Scalar, typename HashTable>
void SpatialHashingCollisionManager<Scalar, HashTable>::unregisterObject(CollisionObject<Scalar>* obj)
{
  objs.remove(obj);

  const AABB<Scalar>& obj_aabb = obj->getAABB();
  AABB<Scalar> overlap_aabb;

  if(scene_limit.overlap(obj_aabb, overlap_aabb))
  {
    if(!scene_limit.contain(obj_aabb))
      objs_outside_scene_limit.remove(obj);

    hash_table->remove(overlap_aabb, obj);
  }
  else
    objs_outside_scene_limit.remove(obj);

  obj_aabb_map.erase(obj);
}

template<typename Scalar, typename HashTable>
void SpatialHashingCollisionManager<Scalar, HashTable>::setup()
{}

template<typename Scalar, typename HashTable>
void SpatialHashingCollisionManager<Scalar, HashTable>::update()
{
  hash_table->clear();
  objs_outside_scene_limit.clear();

  for(auto it = objs.cbegin(), end = objs.cend(); it != end; ++it)
  {
    CollisionObject<Scalar>* obj = *it;
    const AABB<Scalar>& obj_aabb = obj->getAABB();
    AABB<Scalar> overlap_aabb;

    if(scene_limit.overlap(obj_aabb, overlap_aabb))
    {
      if(!scene_limit.contain(obj_aabb))
        objs_outside_scene_limit.push_back(obj);

      hash_table->insert(overlap_aabb, obj);
    }
    else
      objs_outside_scene_limit.push_back(obj);

    obj_aabb_map[obj] = obj_aabb;
  }
}

template<typename Scalar, typename HashTable>
void SpatialHashingCollisionManager<Scalar, HashTable>::update(CollisionObject<Scalar>* updated_obj)
{
  const AABB<Scalar>& new_aabb = updated_obj->getAABB();
  const AABB<Scalar>& old_aabb = obj_aabb_map[updated_obj];

  if(!scene_limit.contain(old_aabb)) // previously not completely in scene limit
  {
    if(scene_limit.contain(new_aabb))
    {
      auto find_it = std::find(objs_outside_scene_limit.begin(),
                               objs_outside_scene_limit.end(),
                               updated_obj);

      objs_outside_scene_limit.erase(find_it);
    }
  }
  else if(!scene_limit.contain(new_aabb)) // previous completely in scenelimit, now not
    objs_outside_scene_limit.push_back(updated_obj);

  AABB<Scalar> old_overlap_aabb;
  if(scene_limit.overlap(old_aabb, old_overlap_aabb))
    hash_table->remove(old_overlap_aabb, updated_obj);

  AABB<Scalar> new_overlap_aabb;
  if(scene_limit.overlap(new_aabb, new_overlap_aabb))
    hash_table->insert(new_overlap_aabb, updated_obj);

  obj_aabb_map[updated_obj] = new_aabb;
}

template<typename Scalar, typename HashTable>
void SpatialHashingCollisionManager<Scalar, HashTable>::update(const std::vector<CollisionObject<Scalar>*>& updated_objs)
{
  for(size_t i = 0; i < updated_objs.size(); ++i)
    update(updated_objs[i]);
}


template<typename Scalar, typename HashTable>
void SpatialHashingCollisionManager<Scalar, HashTable>::clear()
{
  objs.clear();
  hash_table->clear();
  objs_outside_scene_limit.clear();
  obj_aabb_map.clear();
}

template<typename Scalar, typename HashTable>
void SpatialHashingCollisionManager<Scalar, HashTable>::getObjects(std::vector<CollisionObject<Scalar>*>& objs_) const
{
  objs_.resize(objs.size());
  std::copy(objs.begin(), objs.end(), objs_.begin());
}

template<typename Scalar, typename HashTable>
void SpatialHashingCollisionManager<Scalar, HashTable>::collide(CollisionObject<Scalar>* obj, void* cdata, CollisionCallBack<Scalar> callback) const
{
  if(size() == 0) return;
  collide_(obj, cdata, callback);
}

template<typename Scalar, typename HashTable>
void SpatialHashingCollisionManager<Scalar, HashTable>::distance(CollisionObject<Scalar>* obj, void* cdata, DistanceCallBack<Scalar> callback) const
{
  if(size() == 0) return;
  Scalar min_dist = std::numeric_limits<Scalar>::max();
  distance_(obj, cdata, callback, min_dist);
}

template<typename Scalar, typename HashTable>
bool SpatialHashingCollisionManager<Scalar, HashTable>::collide_(CollisionObject<Scalar>* obj, void* cdata, CollisionCallBack<Scalar> callback) const
{
  const AABB<Scalar>& obj_aabb = obj->getAABB();
  AABB<Scalar> overlap_aabb;

  if(scene_limit.overlap(obj_aabb, overlap_aabb))
  {
    if(!scene_limit.contain(obj_aabb))
    {
      for(auto it = objs_outside_scene_limit.cbegin(), end = objs_outside_scene_limit.cend();
          it != end; ++it)
      {
        if(obj == *it) continue;
        if(callback(obj, *it, cdata)) return true;
      }
    }

    std::vector<CollisionObject<Scalar>*> query_result = hash_table->query(overlap_aabb);
    for(unsigned int i = 0; i < query_result.size(); ++i)
    {
      if(obj == query_result[i]) continue;
      if(callback(obj, query_result[i], cdata)) return true;
    }
  }
  else
  {
    ;
    for(auto it = objs_outside_scene_limit.cbegin(), end = objs_outside_scene_limit.cend();
        it != end; ++it)
    {
      if(obj == *it) continue;
      if(callback(obj, *it, cdata)) return true;
    }
  }

  return false;
}

template<typename Scalar, typename HashTable>
bool SpatialHashingCollisionManager<Scalar, HashTable>::distance_(CollisionObject<Scalar>* obj, void* cdata, DistanceCallBack<Scalar> callback, Scalar& min_dist) const
{
  Vector3<Scalar> delta = (obj->getAABB().max_ - obj->getAABB().min_) * 0.5;
  AABB<Scalar> aabb = obj->getAABB();
  if(min_dist < std::numeric_limits<Scalar>::max())
  {
    Vector3<Scalar> min_dist_delta(min_dist, min_dist, min_dist);
    aabb.expand(min_dist_delta);
  }

  AABB<Scalar> overlap_aabb;

  int status = 1;
  Scalar old_min_distance;

  while(1)
  {
    old_min_distance = min_dist;

    if(scene_limit.overlap(aabb, overlap_aabb))
    {
      if(!scene_limit.contain(aabb))
      {
        for(auto it = objs_outside_scene_limit.cbegin(), end = objs_outside_scene_limit.cend();
            it != end; ++it)
        {
          if(obj == *it) continue;
          if(!this->enable_tested_set_)
          {
            if(obj->getAABB().distance((*it)->getAABB()) < min_dist)
              if(callback(obj, *it, cdata, min_dist)) return true;
          }
          else
          {
            if(!this->inTestedSet(obj, *it))
            {
              if(obj->getAABB().distance((*it)->getAABB()) < min_dist)
                if(callback(obj, *it, cdata, min_dist)) return true;
              this->insertTestedSet(obj, *it);
            }
          }
        }
      }

      std::vector<CollisionObject<Scalar>*> query_result = hash_table->query(overlap_aabb);
      for(unsigned int i = 0; i < query_result.size(); ++i)
      {
        if(obj == query_result[i]) continue;
        if(!this->enable_tested_set_)
        {
          if(obj->getAABB().distance(query_result[i]->getAABB()) < min_dist)
            if(callback(obj, query_result[i], cdata, min_dist)) return true;
        }
        else
        {
          if(!this->inTestedSet(obj, query_result[i]))
          {
            if(obj->getAABB().distance(query_result[i]->getAABB()) < min_dist)
              if(callback(obj, query_result[i], cdata, min_dist)) return true;
            this->insertTestedSet(obj, query_result[i]);
          }
        }
      }
    }
    else
    {
      for(auto it = objs_outside_scene_limit.cbegin(), end = objs_outside_scene_limit.cend();
          it != end; ++it)
      {
        if(obj == *it) continue;
        if(!this->enable_tested_set_)
        {
          if(obj->getAABB().distance((*it)->getAABB()) < min_dist)
            if(callback(obj, *it, cdata, min_dist)) return true;
        }
        else
        {
          if(!this->inTestedSet(obj, *it))
          {
            if(obj->getAABB().distance((*it)->getAABB()) < min_dist)
              if(callback(obj, *it, cdata, min_dist)) return true;
            this->insertTestedSet(obj, *it);
          }
        }
      }
    }

    if(status == 1)
    {
      if(old_min_distance < std::numeric_limits<Scalar>::max())
        break;
      else
      {
        if(min_dist < old_min_distance)
        {
          Vector3<Scalar> min_dist_delta(min_dist, min_dist, min_dist);
          aabb = AABB<Scalar>(obj->getAABB(), min_dist_delta);
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
      break;
  }

  return false;
}

template<typename Scalar, typename HashTable>
void SpatialHashingCollisionManager<Scalar, HashTable>::collide(void* cdata, CollisionCallBack<Scalar> callback) const
{
  if(size() == 0) return;

  for(auto it1 = objs.cbegin(), end1 = objs.cend(); it1 != end1; ++it1)
  {
    const AABB<Scalar>& obj_aabb = (*it1)->getAABB();
    AABB<Scalar> overlap_aabb;

    if(scene_limit.overlap(obj_aabb, overlap_aabb))
    {
      if(!scene_limit.contain(obj_aabb))
      {
        for(auto it2 = objs_outside_scene_limit.cbegin(), end2 = objs_outside_scene_limit.cend();
            it2 != end2; ++it2)
        {
          if(*it1 < *it2) { if(callback(*it1, *it2, cdata)) return; }
        }
      }

      std::vector<CollisionObject<Scalar>*> query_result = hash_table->query(overlap_aabb);
      for(unsigned int i = 0; i < query_result.size(); ++i)
      {
        if(*it1 < query_result[i]) { if(callback(*it1, query_result[i], cdata)) return; }
      }
    }
    else
    {
      for(auto it2 = objs_outside_scene_limit.cbegin(), end2 = objs_outside_scene_limit.cend();
          it2 != end2; ++it2)
      {
        if(*it1 < *it2) { if(callback(*it1, *it2, cdata)) return; }
      }
    }
  }
}

template<typename Scalar, typename HashTable>
void SpatialHashingCollisionManager<Scalar, HashTable>::distance(void* cdata, DistanceCallBack<Scalar> callback) const
{
  if(size() == 0) return;

  this->enable_tested_set_ = true;
  this->tested_set.clear();

  Scalar min_dist = std::numeric_limits<Scalar>::max();

  for(auto it = objs.cbegin(), end = objs.cend(); it != end; ++it)
    if(distance_(*it, cdata, callback, min_dist)) break;

  this->enable_tested_set_ = false;
  this->tested_set.clear();
}

template<typename Scalar, typename HashTable>
void SpatialHashingCollisionManager<Scalar, HashTable>::collide(BroadPhaseCollisionManager<Scalar>* other_manager_, void* cdata, CollisionCallBack<Scalar> callback) const
{
  auto* other_manager = static_cast<SpatialHashingCollisionManager<Scalar, HashTable>* >(other_manager_);

  if((size() == 0) || (other_manager->size() == 0)) return;

  if(this == other_manager)
  {
    collide(cdata, callback);
    return;
  }

  if(this->size() < other_manager->size())
  {
    for(auto it = objs.cbegin(), end = objs.cend(); it != end; ++it)
      if(other_manager->collide_(*it, cdata, callback)) return;
  }
  else
  {
    for(auto it = other_manager->objs.cbegin(), end = other_manager->objs.cend(); it != end; ++it)
      if(collide_(*it, cdata, callback)) return;
  }
}

template<typename Scalar, typename HashTable>
void SpatialHashingCollisionManager<Scalar, HashTable>::distance(BroadPhaseCollisionManager<Scalar>* other_manager_, void* cdata, DistanceCallBack<Scalar> callback) const
{
  auto* other_manager = static_cast<SpatialHashingCollisionManager<Scalar, HashTable>* >(other_manager_);

  if((size() == 0) || (other_manager->size() == 0)) return;

  if(this == other_manager)
  {
    distance(cdata, callback);
    return;
  }

  Scalar min_dist = std::numeric_limits<Scalar>::max();

  if(this->size() < other_manager->size())
  {
    for(auto it = objs.cbegin(), end = objs.cend(); it != end; ++it)
      if(other_manager->distance_(*it, cdata, callback, min_dist)) return;
  }
  else
  {
    for(auto it = other_manager->objs.cbegin(), end = other_manager->objs.cend(); it != end; ++it)
      if(distance_(*it, cdata, callback, min_dist)) return;
  }
}

template<typename Scalar, typename HashTable>
bool SpatialHashingCollisionManager<Scalar, HashTable>::empty() const
{
  return objs.empty();
}

template<typename Scalar, typename HashTable>
size_t SpatialHashingCollisionManager<Scalar, HashTable>::size() const
{
  return objs.size();
}

} // namespace fcl

#endif

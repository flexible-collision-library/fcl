/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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



#ifndef FCL_BROAD_PHASE_COLLISION_H
#define FCL_BROAD_PHASE_COLLISION_H

#include "fcl/collision_object.h"
#include "fcl/collision_data.h"
#include "fcl/BV/AABB.h"
#include "fcl/interval_tree.h"
#include "fcl/hierarchy_tree.h"
#include "fcl/hash.h"
#include "fcl/octree.h"
#include <vector>
#include <list>
#include <iostream>
#include <boost/unordered_map.hpp>
#include <map>
#include <boost/bind.hpp>

namespace fcl
{


/** \brief collision function for two objects o1 and o2 in broad phase. return value means whether the broad phase can stop now */
bool defaultCollisionFunction(CollisionObject* o1, CollisionObject* o2, void* cdata);

/** \brief distance function for two objects o1 and o2 in broad phase. return value means whether the broad phase can stop now. also return dist, i.e. the bmin distance till now */
bool defaultDistanceFunction(CollisionObject* o1, CollisionObject* o2, void* cdata, FCL_REAL& dist);


/** \brief return value is whether can stop now */
typedef bool (*CollisionCallBack)(CollisionObject* o1, CollisionObject* o2, void* cdata);

typedef bool (*DistanceCallBack)(CollisionObject* o1, CollisionObject* o2, void* cdata, FCL_REAL& dist);

typedef bool (*IsCostEnabledCallBack)(void* cdata);

/** \brief Base class for broad phase collision */
class BroadPhaseCollisionManager
{
public:
  BroadPhaseCollisionManager()
  {
    enable_tested_set_ = false;
  }

  virtual ~BroadPhaseCollisionManager() {}

  /** \brief add objects to the manager */
  virtual void registerObjects(const std::vector<CollisionObject*>& other_objs)
  {
    for(size_t i = 0; i < other_objs.size(); ++i)
      registerObject(other_objs[i]);
  }

  /** \brief add one object to the manager */
  virtual void registerObject(CollisionObject* obj) = 0;

  /** \brief remove one object from the manager */
  virtual void unregisterObject(CollisionObject* obj) = 0;

  /** \brief initialize the manager, related with the specific type of manager */
  virtual void setup() = 0;

  /** \brief update the condition of manager */
  virtual void update() = 0;

  /** \brief update the manager by explicitly given the object updated */
  virtual void update(CollisionObject* updated_obj)
  {
    update();
  }

  /** \brief update the manager by explicitly given the set of objects update */
  virtual void update(const std::vector<CollisionObject*>& updated_objs)
  {
    update();
  }

  /** \brief clear the manager */
  virtual void clear() = 0;

  /** \brief return the objects managed by the manager */
  virtual void getObjects(std::vector<CollisionObject*>& objs) const = 0;

  /** \brief perform collision test between one object and all the objects belonging to the manager */
  virtual void collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const = 0;

  /** \brief perform distance computation between one object and all the objects belonging to the manager */
  virtual void distance(CollisionObject* obj, void* cdata, DistanceCallBack callback) const = 0;

  /** \brief perform collision test for the objects belonging to the manager (i.e., N^2 self collision) */
  virtual void collide(void* cdata, CollisionCallBack callback) const = 0;

  /** \brief perform distance test for the objects belonging to the manager (i.e., N^2 self distance) */
  virtual void distance(void* cdata, DistanceCallBack callback) const = 0;

  /** \brief perform collision test with objects belonging to another manager */
  virtual void collide(BroadPhaseCollisionManager* other_manager, void* cdata, CollisionCallBack callback) const = 0;

  /** \brief perform distance test with objects belonging to another manager */
  virtual void distance(BroadPhaseCollisionManager* other_manager, void* cdata, DistanceCallBack callback) const = 0;

  /** \brief whether the manager is empty */
  virtual bool empty() const = 0;
  
  /** \brief the number of objects managed by the manager */
  virtual size_t size() const = 0;

protected:
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


/** \brief Brute force N-body collision manager */
class NaiveCollisionManager : public BroadPhaseCollisionManager
{
public:
  NaiveCollisionManager() {}

  /** \brief add objects to the manager */
  void registerObjects(const std::vector<CollisionObject*>& other_objs);

  /** \brief add one object to the manager */
  void registerObject(CollisionObject* obj);

  /** \brief remove one object from the manager */
  void unregisterObject(CollisionObject* obj);

  /** \brief initialize the manager, related with the specific type of manager */
  void setup();

  /** \brief update the condition of manager */
  void update();

  /** \brief clear the manager */
  void clear();

  /** \brief return the objects managed by the manager */
  void getObjects(std::vector<CollisionObject*>& objs) const;

  /** \brief perform collision test between one object and all the objects belonging to the manager */
  void collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const;

  /** \brief perform distance computation between one object and all the objects belonging to the manager */
  void distance(CollisionObject* obj, void* cdata, DistanceCallBack callback) const;

  /** \brief perform collision test for the objects belonging to the manager (i.e., N^2 self collision) */
  void collide(void* cdata, CollisionCallBack callback) const;

  /** \brief perform distance test for the objects belonging to the manager (i.e., N^2 self distance) */
  void distance(void* cdata, DistanceCallBack callback) const;

  /** \brief perform collision test with objects belonging to another manager */
  void collide(BroadPhaseCollisionManager* other_manager, void* cdata, CollisionCallBack callback) const;

  /** \brief perform distance test with objects belonging to another manager */
  void distance(BroadPhaseCollisionManager* other_manager, void* cdata, DistanceCallBack callback) const;

  /** \brief whether the manager is empty */
  bool empty() const;
  
  /** \brief the number of objects managed by the manager */
  inline size_t size() const { return objs.size(); }

protected:

  /** \brief objects belonging to the manager are stored in a list structure */
  std::list<CollisionObject*> objs;
};

/** \brief Spatial hash function: hash an AABB to a set of integer values */
struct SpatialHash
{
  SpatialHash(const AABB& scene_limit_, FCL_REAL cell_size_)
  {
    cell_size = cell_size_;
    scene_limit = scene_limit_;
    width[0] = ceil(scene_limit.width() / cell_size);
    width[1] = ceil(scene_limit.height() / cell_size);
    width[2] = ceil(scene_limit.depth() / cell_size);
  }
    
  std::vector<unsigned int> operator() (const AABB& aabb) const
  {
    int min_x = floor((aabb.min_[0] - scene_limit.min_[0]) / cell_size);
    int max_x = ceil((aabb.max_[0] - scene_limit.min_[0]) / cell_size);
    int min_y = floor((aabb.min_[1] - scene_limit.min_[1]) / cell_size);
    int max_y = ceil((aabb.max_[1] - scene_limit.min_[1]) / cell_size);
    int min_z = floor((aabb.min_[2] - scene_limit.min_[2]) / cell_size);
    int max_z = ceil((aabb.max_[2] - scene_limit.min_[2]) / cell_size);

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

  FCL_REAL cell_size;
  AABB scene_limit;
  unsigned int width[3];
};

/** \brief spatial hashing collision mananger */
template<typename HashTable = SimpleHashTable<AABB, CollisionObject*, SpatialHash> >
class SpatialHashingCollisionManager : public BroadPhaseCollisionManager
{
public:
  SpatialHashingCollisionManager(FCL_REAL cell_size, const Vec3f& scene_min, const Vec3f& scene_max, unsigned int default_table_size = 1000)
  {
    scene_limit = AABB(scene_min, scene_max);
    SpatialHash hasher(scene_limit, cell_size);
    hash_table = new HashTable(hasher);
    hash_table->init(default_table_size);
  }

  ~SpatialHashingCollisionManager()
  {
    delete hash_table;
  }

  /** \brief add one object to the manager */
  void registerObject(CollisionObject* obj);

  /** \brief remove one object from the manager */
  void unregisterObject(CollisionObject* obj);

  /** \brief initialize the manager, related with the specific type of manager */
  void setup();

  /** \brief update the condition of manager */
  void update();

  /** \brief update the manager by explicitly given the object updated */
  void update(CollisionObject* updated_obj);

  /** \brief update the manager by explicitly given the set of objects update */
  void update(const std::vector<CollisionObject*>& updated_objs);

  /** \brief clear the manager */
  void clear();

  /** \brief return the objects managed by the manager */
  void getObjects(std::vector<CollisionObject*>& objs) const;

  /** \brief perform collision test between one object and all the objects belonging to the manager */
  void collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const;

  /** \brief perform distance computation between one object and all the objects belonging ot the manager */
  void distance(CollisionObject* obj, void* cdata, DistanceCallBack callback) const;

  /** \brief perform collision test for the objects belonging to the manager (i.e, N^2 self collision) */
  void collide(void* cdata, CollisionCallBack callback) const;

  /** \brief perform distance test for the objects belonging to the manager (i.e., N^2 self distance) */
  void distance(void* cdata, DistanceCallBack callback) const;

  /** \brief perform collision test with objects belonging to another manager */
  void collide(BroadPhaseCollisionManager* other_manager, void* cdata, CollisionCallBack callback) const;

  /** \brief perform distance test with objects belonging to another manager */
  void distance(BroadPhaseCollisionManager* other_manager, void* cdata, DistanceCallBack callback) const;

  /** \brief whether the manager is empty */
  bool empty() const;

  /** \brief the number of objects managed by the manager */
  size_t size() const;

  /** \brief compute the bound for the environent */
  static void computeBound(std::vector<CollisionObject*>& objs, Vec3f& l, Vec3f& u)
  {
    AABB bound;
    for(unsigned int i = 0; i < objs.size(); ++i)
      bound += objs[i]->getAABB();
    
    l = bound.min_;
    u = bound.max_;
  }

protected:

  /** \brief perform collision test between one object and all the objects belonging to the manager */
  bool collide_(CollisionObject* obj, void* cdata, CollisionCallBack callback) const;

  /** \brief perform distance computation between one object and all the objects belonging ot the manager */
  bool distance_(CollisionObject* obj, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const;


  // all objects in the scene
  std::list<CollisionObject*> objs;

  // objects in the scene limit (given by scene_min and scene_max) are in the spatial hash table
  HashTable* hash_table;
  // objects outside the scene limit are in another list
  std::list<CollisionObject*> objs_outside_scene_limit;

  // the size of the scene
  AABB scene_limit;

  std::map<CollisionObject*, AABB> obj_aabb_map; // store the map between objects and their aabbs. will make update more convenient
};


template<typename HashTable>
void SpatialHashingCollisionManager<HashTable>::registerObject(CollisionObject* obj)
{
  objs.push_back(obj);

  const AABB& obj_aabb = obj->getAABB();
  AABB overlap_aabb;

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

template<typename HashTable>
void SpatialHashingCollisionManager<HashTable>::unregisterObject(CollisionObject* obj)
{
  objs.remove(obj);

  const AABB& obj_aabb = obj->getAABB();
  AABB overlap_aabb;

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

template<typename HashTable>
void SpatialHashingCollisionManager<HashTable>::setup()
{}

template<typename HashTable>
void SpatialHashingCollisionManager<HashTable>::update()
{
  hash_table->clear();
  objs_outside_scene_limit.clear();

  for(std::list<CollisionObject*>::const_iterator it = objs.begin(), end = objs.end(); 
      it != end; ++it)
  {
    CollisionObject* obj = *it;
    const AABB& obj_aabb = obj->getAABB();
    AABB overlap_aabb;

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

template<typename HashTable>
void SpatialHashingCollisionManager<HashTable>::update(CollisionObject* updated_obj)
{
  const AABB& new_aabb = updated_obj->getAABB();
  const AABB& old_aabb = obj_aabb_map[updated_obj];

  if(!scene_limit.contain(old_aabb)) // previously not completely in scene limit
  {
    if(scene_limit.contain(new_aabb))
    {
      std::list<CollisionObject*>::iterator find_it = std::find(objs_outside_scene_limit.begin(),
                                                                objs_outside_scene_limit.end(),
                                                                updated_obj);
    
      objs_outside_scene_limit.erase(find_it);
    }
  }
  else if(!scene_limit.contain(new_aabb)) // previous completely in scenelimit, now not
      objs_outside_scene_limit.push_back(updated_obj);
  
  AABB old_overlap_aabb;
  if(scene_limit.overlap(old_aabb, old_overlap_aabb))
    hash_table->remove(old_overlap_aabb, updated_obj);

  AABB new_overlap_aabb;
  if(scene_limit.overlap(new_aabb, new_overlap_aabb))
     hash_table->insert(new_overlap_aabb, updated_obj);

  obj_aabb_map[updated_obj] = new_aabb;
}

template<typename HashTable>
void SpatialHashingCollisionManager<HashTable>::update(const std::vector<CollisionObject*>& updated_objs)
{
  for(size_t i = 0; i < updated_objs.size(); ++i)
    update(updated_objs[i]);
}


template<typename HashTable>
void SpatialHashingCollisionManager<HashTable>::clear()
{
  objs.clear();
  hash_table->clear();
  objs_outside_scene_limit.clear();
  obj_aabb_map.clear();
}

template<typename HashTable>
void SpatialHashingCollisionManager<HashTable>::getObjects(std::vector<CollisionObject*>& objs_) const
{
  objs_.resize(objs.size());
  std::copy(objs.begin(), objs.end(), objs_.begin());
}

template<typename HashTable>
void SpatialHashingCollisionManager<HashTable>::collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const
{
  if(size() == 0) return;
  collide_(obj, cdata, callback);
}

template<typename HashTable>
void SpatialHashingCollisionManager<HashTable>::distance(CollisionObject* obj, void* cdata, DistanceCallBack callback) const
{
  if(size() == 0) return;
  FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
  distance_(obj, cdata, callback, min_dist);
}

template<typename HashTable>
bool SpatialHashingCollisionManager<HashTable>::collide_(CollisionObject* obj, void* cdata, CollisionCallBack callback) const
{
  const AABB& obj_aabb = obj->getAABB();
  AABB overlap_aabb;

  if(scene_limit.overlap(obj_aabb, overlap_aabb))
  {
    if(!scene_limit.contain(obj_aabb))
    {
      for(std::list<CollisionObject*>::const_iterator it = objs_outside_scene_limit.begin(), end = objs_outside_scene_limit.end(); 
          it != end; ++it)
      {
        if(obj == *it) continue;
        if(callback(obj, *it, cdata)) return true; 
      }
    }

    std::vector<CollisionObject*> query_result = hash_table->query(overlap_aabb);
    for(unsigned int i = 0; i < query_result.size(); ++i)
    {
      if(obj == query_result[i]) continue;
      if(callback(obj, query_result[i], cdata)) return true;
    }
  }
  else
  {
    ;
    for(std::list<CollisionObject*>::const_iterator it = objs_outside_scene_limit.begin(), end = objs_outside_scene_limit.end(); 
        it != end; ++it)
    {
      if(obj == *it) continue;
      if(callback(obj, *it, cdata)) return true;
    }
  }

  return false;
}

template<typename HashTable>
bool SpatialHashingCollisionManager<HashTable>::distance_(CollisionObject* obj, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const
{
  Vec3f delta = (obj->getAABB().max_ - obj->getAABB().min_) * 0.5;
  AABB aabb = obj->getAABB();
  if(min_dist < std::numeric_limits<FCL_REAL>::max())
  {
    Vec3f min_dist_delta(min_dist, min_dist, min_dist);
    aabb.expand(min_dist_delta);
  }

  AABB overlap_aabb;

  int status = 1;
  FCL_REAL old_min_distance;

  while(1)
  {
    old_min_distance = min_dist;

    if(scene_limit.overlap(aabb, overlap_aabb))
    {
      if(!scene_limit.contain(aabb))
      {
        for(std::list<CollisionObject*>::const_iterator it = objs_outside_scene_limit.begin(), end = objs_outside_scene_limit.end(); 
            it != end; ++it)
        {
          if(obj == *it) continue;
          if(!enable_tested_set_)
          {
            if(obj->getAABB().distance((*it)->getAABB()) < min_dist)
              if(callback(obj, *it, cdata, min_dist)) return true;
          }
          else
          {
            if(!inTestedSet(obj, *it))
            {
              if(obj->getAABB().distance((*it)->getAABB()) < min_dist)
                if(callback(obj, *it, cdata, min_dist)) return true;
              insertTestedSet(obj, *it);
            }
          }
        }
      }
      
      std::vector<CollisionObject*> query_result = hash_table->query(overlap_aabb);
      for(unsigned int i = 0; i < query_result.size(); ++i)
      {
        if(obj == query_result[i]) continue;
        if(!enable_tested_set_)
        {
          if(obj->getAABB().distance(query_result[i]->getAABB()) < min_dist)
            if(callback(obj, query_result[i], cdata, min_dist)) return true;
        }
        else
        {
          if(!inTestedSet(obj, query_result[i]))
          {
            if(obj->getAABB().distance(query_result[i]->getAABB()) < min_dist)
              if(callback(obj, query_result[i], cdata, min_dist)) return true;
            insertTestedSet(obj, query_result[i]);
          }
        }
      }
    }
    else
    {
      for(std::list<CollisionObject*>::const_iterator it = objs_outside_scene_limit.begin(), end = objs_outside_scene_limit.end(); 
          it != end; ++it)
      {
        if(obj == *it) continue;
        if(!enable_tested_set_)
        {
          if(obj->getAABB().distance((*it)->getAABB()) < min_dist)
            if(callback(obj, *it, cdata, min_dist)) return true;
        }
        else
        {
          if(!inTestedSet(obj, *it))
          {
            if(obj->getAABB().distance((*it)->getAABB()) < min_dist)
              if(callback(obj, *it, cdata, min_dist)) return true;
            insertTestedSet(obj, *it);
          }
        }
      }
    }

    if(status == 1)
    {
      if(old_min_distance < std::numeric_limits<FCL_REAL>::max())
        break;
      else
      {
        if(min_dist < old_min_distance)
        {
          Vec3f min_dist_delta(min_dist, min_dist, min_dist);
          aabb = AABB(obj->getAABB(), min_dist_delta);
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

template<typename HashTable>
void SpatialHashingCollisionManager<HashTable>::collide(void* cdata, CollisionCallBack callback) const
{
  if(size() == 0) return;

  for(std::list<CollisionObject*>::const_iterator it1 = objs.begin(), end1 = objs.end(); 
      it1 != end1; ++it1)
  {
    const AABB& obj_aabb = (*it1)->getAABB();
    AABB overlap_aabb;
    
    if(scene_limit.overlap(obj_aabb, overlap_aabb))
    {
      if(!scene_limit.contain(obj_aabb))
      {
        for(std::list<CollisionObject*>::const_iterator it2 = objs_outside_scene_limit.begin(), end2 = objs_outside_scene_limit.end(); 
            it2 != end2; ++it2)
        {
          if(*it1 < *it2) { if(callback(*it1, *it2, cdata)) return; }
        }
      }

      std::vector<CollisionObject*> query_result = hash_table->query(overlap_aabb);
      for(unsigned int i = 0; i < query_result.size(); ++i)
      {
        if(*it1 < query_result[i]) { if(callback(*it1, query_result[i], cdata)) return; }
      }
    }
    else
    {
      for(std::list<CollisionObject*>::const_iterator it2 = objs_outside_scene_limit.begin(), end2 = objs_outside_scene_limit.end(); 
          it2 != end2; ++it2)
      {
        if(*it1 < *it2) { if(callback(*it1, *it2, cdata)) return; }
      }
    }
  }
}

template<typename HashTable>
void SpatialHashingCollisionManager<HashTable>::distance(void* cdata, DistanceCallBack callback) const
{
  if(size() == 0) return;

  enable_tested_set_ = true;
  tested_set.clear();
  
  FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();

  for(std::list<CollisionObject*>::const_iterator it = objs.begin(), end = objs.end(); it != end; ++it)
    if(distance_(*it, cdata, callback, min_dist)) break;

  enable_tested_set_ = false;
  tested_set.clear();
}

template<typename HashTable>
void SpatialHashingCollisionManager<HashTable>::collide(BroadPhaseCollisionManager* other_manager_, void* cdata, CollisionCallBack callback) const
{
  SpatialHashingCollisionManager<HashTable>* other_manager = static_cast<SpatialHashingCollisionManager<HashTable>* >(other_manager_);

  if((size() == 0) || (other_manager->size() == 0)) return;

  if(this == other_manager)
  {
    collide(cdata, callback);
    return;
  }

  if(this->size() < other_manager->size())
  {
    for(std::list<CollisionObject*>::const_iterator it = objs.begin(), end = objs.end(); it != end; ++it)
      if(other_manager->collide_(*it, cdata, callback)) return;
  }
  else
  {
    for(std::list<CollisionObject*>::const_iterator it = other_manager->objs.begin(), end = other_manager->objs.end(); it != end; ++it)
      if(collide_(*it, cdata, callback)) return;
  }
}

template<typename HashTable>
void SpatialHashingCollisionManager<HashTable>::distance(BroadPhaseCollisionManager* other_manager_, void* cdata, DistanceCallBack callback) const
{
  SpatialHashingCollisionManager<HashTable>* other_manager = static_cast<SpatialHashingCollisionManager<HashTable>* >(other_manager_);

  if((size() == 0) || (other_manager->size() == 0)) return;

  if(this == other_manager)
  {
    distance(cdata, callback);
    return;
  }

  FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();

  if(this->size() < other_manager->size())
  {
    for(std::list<CollisionObject*>::const_iterator it = objs.begin(), end = objs.end(); it != end; ++it)
      if(other_manager->distance_(*it, cdata, callback, min_dist)) return;
  }
  else
  {
    for(std::list<CollisionObject*>::const_iterator it = other_manager->objs.begin(), end = other_manager->objs.end(); it != end; ++it)
      if(distance_(*it, cdata, callback, min_dist)) return;
  }
}

template<typename HashTable>
bool SpatialHashingCollisionManager<HashTable>::empty() const
{
  return objs.empty();
}

template<typename HashTable>
size_t SpatialHashingCollisionManager<HashTable>::size() const
{
  return objs.size();
}


/** Rigorous SAP collision manager */
class SaPCollisionManager : public BroadPhaseCollisionManager
{
public:

  SaPCollisionManager()
  {
    elist[0] = NULL;
    elist[1] = NULL;
    elist[2] = NULL;

    optimal_axis = 0;
  }

  ~SaPCollisionManager()
  {
    clear();
  }

  /** \brief add objects to the manager */
  void registerObjects(const std::vector<CollisionObject*>& other_objs);

  /** \brief remove one object from the manager */
  void registerObject(CollisionObject* obj);

  /** \brief add one object to the manager */
  void unregisterObject(CollisionObject* obj);

  /** \brief initialize the manager, related with the specific type of manager */
  void setup();

  /** \brief update the condition of manager */
  void update();

  /** \brief update the manager by explicitly given the object updated */
  void update(CollisionObject* updated_obj);

  /** \brief update the manager by explicitly given the set of objects update */
  void update(const std::vector<CollisionObject*>& updated_objs);

  /** \brief clear the manager */
  void clear();

  /** \brief return the objects managed by the manager */
  void getObjects(std::vector<CollisionObject*>& objs) const;

  /** \brief perform collision test between one object and all the objects belonging to the manager */
  void collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const;

  /** \brief perform distance computation between one object and all the objects belonging to the manager */
  void distance(CollisionObject* obj, void* cdata, DistanceCallBack callback) const;

  /** \brief perform collision test for the objects belonging to the manager (i.e., N^2 self collision) */
  void collide(void* cdata, CollisionCallBack callback) const;

  /** \brief perform distance test for the objects belonging to the manager (i.e., N^2 self distance) */
  void distance(void* cdata, DistanceCallBack callback) const;

  /** \brief perform collision test with objects belonging to another manager */
  void collide(BroadPhaseCollisionManager* other_manager, void* cdata, CollisionCallBack callback) const;

  /** \brief perform distance test with objects belonging to another manager */
  void distance(BroadPhaseCollisionManager* other_manager, void* cdata, DistanceCallBack callback) const;

  /** \brief whether the manager is empty */
  bool empty() const;
  
  /** \brief the number of objects managed by the manager */
  inline size_t size() const { return AABB_arr.size(); }

protected:

  struct EndPoint;

  /** \brief SAP interval for one object */
  struct SaPAABB
  {
    /** \brief object */
    CollisionObject* obj;

    /** \brief lower bound end point of the interval */
    EndPoint* lo;

    /** \brief higher bound end point of the interval */
    EndPoint* hi;

    /** \brief cached AABB value */
    AABB cached;
  };

  /** \brief End point for an interval */
  struct EndPoint
  {
    /** \brief tag for whether it is a lower bound or higher bound of an interval, 0 for lo, and 1 for hi */
    char minmax;

    /** \brief back pointer to SAP interval */
    SaPAABB* aabb;

    /** \brief the previous end point in the end point list */
    EndPoint* prev[3];
    /** \brief the next end point in the end point list */
    EndPoint* next[3];

    /** \brief get the value of the end point */
    inline const Vec3f& getVal() const
    {
      if(minmax) return aabb->cached.max_;
      else return aabb->cached.min_;
    }

    /** \brief set the value of the end point */
    inline Vec3f& getVal()
    {
      if(minmax) return aabb->cached.max_;
      else return aabb->cached.min_;
    }

    inline FCL_REAL getVal(size_t i) const
    {
      if(minmax) return aabb->cached.max_[i];
      else return aabb->cached.min_[i];
    }

    inline FCL_REAL& getVal(size_t i)
    {
      if(minmax) return aabb->cached.max_[i];
      else return aabb->cached.min_[i];
    }

  };

  /** \brief A pair of objects that are not culling away and should further check collision */
  struct SaPPair
  {
    SaPPair(CollisionObject* a, CollisionObject* b)
    {
      if(a < b)
      {
        obj1 = a;
        obj2 = b;
      }
      else
      {
        obj1 = b;
        obj2 = a;
      }
    }

    CollisionObject* obj1;
    CollisionObject* obj2;

    bool operator == (const SaPPair& other) const
    {
      return ((obj1 == other.obj1) && (obj2 == other.obj2));
    }
  };

  /** Functor to help unregister one object */
  class isUnregistered
  {
    CollisionObject* obj;

  public:
    isUnregistered(CollisionObject* obj_)
    {
      obj = obj_;
    }

    bool operator() (const SaPPair& pair)
    {
      return (pair.obj1 == obj) || (pair.obj2 == obj);
    }
  };

  /** Functor to help remove collision pairs no longer valid (i.e., should be culled away) */
  class isNotValidPair
  {
    CollisionObject* obj1;
    CollisionObject* obj2;

  public:
    isNotValidPair(CollisionObject* obj1_, CollisionObject* obj2_)
    {
      obj1 = obj1_;
      obj2 = obj2_;
    }

    bool operator() (const SaPPair& pair)
    {
      return (pair.obj1 == obj1) && (pair.obj2 == obj2);
    }
  };

  void update_(SaPAABB* updated_aabb);

  void updateVelist() 
  {
    for(int coord = 0; coord < 3; ++coord)
    {
      velist[coord].resize(size() * 2);
      EndPoint* current = elist[coord];
      size_t id = 0;
      while(current)
      {
        velist[coord][id] = current;
        current = current->next[coord];
        id++;
      }
    }    
  }

  /** \brief End point list for x, y, z coordinates */
  EndPoint* elist[3];
  
  /** \brief vector version of elist, for acceleration */
  std::vector<EndPoint*> velist[3];

  /** \brief SAP interval list */
  std::list<SaPAABB*> AABB_arr;

  /** \brief The pair of objects that should further check for collision */
  std::list<SaPPair> overlap_pairs;

  size_t optimal_axis;

  std::map<CollisionObject*, SaPAABB*> obj_aabb_map;

  bool distance_(CollisionObject* obj, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const;

  bool collide_(CollisionObject* obj, void* cdata, CollisionCallBack callback) const;

  void addToOverlapPairs(const SaPPair& p)
  {
    bool repeated = false;
    for(std::list<SaPPair>::iterator it = overlap_pairs.begin(), end = overlap_pairs.end();
        it != end;
        ++it)
    {
      if(*it == p)
      {
        repeated = true;
        break;
      }
    }

    if(!repeated)
      overlap_pairs.push_back(p);
  }

  void removeFromOverlapPairs(const SaPPair& p)
  {
    for(std::list<SaPPair>::iterator it = overlap_pairs.begin(), end = overlap_pairs.end();
        it != end;
        ++it)
    {
      if(*it == p)
      {
        overlap_pairs.erase(it);
        break;
      }
    }

    // or overlap_pairs.remove_if(isNotValidPair(p));
  }
};




/** Simple SAP collision manager */
class SSaPCollisionManager : public BroadPhaseCollisionManager
{
public:
  SSaPCollisionManager()
  {
    setup_ = false;
  }

  /** \brief remove one object from the manager */
  void registerObject(CollisionObject* obj);

  /** \brief add one object to the manager */
  void unregisterObject(CollisionObject* obj);

  /** \brief initialize the manager, related with the specific type of manager */
  void setup();

  /** \brief update the condition of manager */
  void update();

  /** \brief clear the manager */
  void clear();

  /** \brief return the objects managed by the manager */
  void getObjects(std::vector<CollisionObject*>& objs) const;

  /** \brief perform collision test between one object and all the objects belonging to the manager */
  void collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const;

  /** \brief perform distance computation between one object and all the objects belonging to the manager */
  void distance(CollisionObject* obj, void* cdata, DistanceCallBack callback) const;

  /** \brief perform collision test for the objects belonging to the manager (i.e., N^2 self collision) */
  void collide(void* cdata, CollisionCallBack callback) const;

  /** \brief perform distance test for the objects belonging to the manager (i.e., N^2 self distance) */
  void distance(void* cdata, DistanceCallBack callback) const;

  /** \brief perform collision test with objects belonging to another manager */
  void collide(BroadPhaseCollisionManager* other_manager, void* cdata, CollisionCallBack callback) const;

  /** \brief perform distance test with objects belonging to another manager */
  void distance(BroadPhaseCollisionManager* other_manager, void* cdata, DistanceCallBack callback) const;

  /** \brief whether the manager is empty */
  bool empty() const;
  
  /** \brief the number of objects managed by the manager */
  inline size_t size() const { return objs_x.size(); }

protected:
  /** \brief check collision between one object and a list of objects, return value is whether stop is possible */
  bool checkColl(std::vector<CollisionObject*>::const_iterator pos_start, std::vector<CollisionObject*>::const_iterator pos_end,
                 CollisionObject* obj, void* cdata, CollisionCallBack callback) const;
  
  /** \brief check distance between one object and a list of objects, return value is whether stop is possible */
  bool checkDis(std::vector<CollisionObject*>::const_iterator pos_start, std::vector<CollisionObject*>::const_iterator pos_end,
                CollisionObject* obj, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const;


  bool collide_(CollisionObject* obj, void* cdata, CollisionCallBack callback) const;
  
  bool distance_(CollisionObject* obj, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const;

  static inline size_t selectOptimalAxis(const std::vector<CollisionObject*>& objs_x, const std::vector<CollisionObject*>& objs_y, const std::vector<CollisionObject*>& objs_z, std::vector<CollisionObject*>::const_iterator& it_beg, std::vector<CollisionObject*>::const_iterator& it_end)
  {
    // simple sweep and prune method
    double delta_x = (objs_x[objs_x.size() - 1])->getAABB().min_[0] - (objs_x[0])->getAABB().min_[0];
    double delta_y = (objs_x[objs_y.size() - 1])->getAABB().min_[1] - (objs_y[0])->getAABB().min_[1];
    double delta_z = (objs_z[objs_z.size() - 1])->getAABB().min_[2] - (objs_z[0])->getAABB().min_[2];

    int axis = 0;
    if(delta_y > delta_x && delta_y > delta_z)
      axis = 1;
    else if(delta_z > delta_y && delta_z > delta_x)
      axis = 2;

    switch(axis)
    {
    case 0:
      it_beg = objs_x.begin();
      it_end = objs_x.end();
      break;
    case 1:
      it_beg = objs_y.begin();
      it_end = objs_y.end();
      break;
    case 2:
      it_beg = objs_z.begin();
      it_end = objs_z.end();
      break;
    }

    return axis;
  }


  /** \brief Objects sorted according to lower x value */
  std::vector<CollisionObject*> objs_x;

  /** \brief Objects sorted according to lower y value */
  std::vector<CollisionObject*> objs_y;

  /** \brief Objects sorted according to lower z value */
  std::vector<CollisionObject*> objs_z;

  /** \brief tag about whether the environment is maintained suitably (i.e., the objs_x, objs_y, objs_z are sorted correctly */
  bool setup_;
};

/** Collision manager based on interval tree */
class IntervalTreeCollisionManager : public BroadPhaseCollisionManager
{
public:
  IntervalTreeCollisionManager()
  {
    setup_ = false;
    for(int i = 0; i < 3; ++i)
      interval_trees[i] = NULL;
  }

  ~IntervalTreeCollisionManager()
  {
    clear();
  }

  /** \brief remove one object from the manager */
  void registerObject(CollisionObject* obj);

  /** \brief add one object to the manager */
  void unregisterObject(CollisionObject* obj);

  /** \brief initialize the manager, related with the specific type of manager */
  void setup();

  /** \brief update the condition of manager */
  void update();

  /** \brief update the manager by explicitly given the object updated */
  void update(CollisionObject* updated_obj);

  /** \brief update the manager by explicitly given the set of objects update */
  void update(const std::vector<CollisionObject*>& updated_objs);

  /** \brief clear the manager */
  void clear();

  /** \brief return the objects managed by the manager */
  void getObjects(std::vector<CollisionObject*>& objs) const;

  /** \brief perform collision test between one object and all the objects belonging to the manager */
  void collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const;

  /** \brief perform distance computation between one object and all the objects belonging to the manager */
  void distance(CollisionObject* obj, void* cdata, DistanceCallBack callback) const;

  /** \brief perform collision test for the objects belonging to the manager (i.e., N^2 self collision) */
  void collide(void* cdata, CollisionCallBack callback) const;

  /** \brief perform distance test for the objects belonging to the manager (i.e., N^2 self distance) */
  void distance(void* cdata, DistanceCallBack callback) const;

  /** \brief perform collision test with objects belonging to another manager */
  void collide(BroadPhaseCollisionManager* other_manager, void* cdata, CollisionCallBack callback) const;

  /** \brief perform distance test with objects belonging to another manager */
  void distance(BroadPhaseCollisionManager* other_manager, void* cdata, DistanceCallBack callback) const;

  /** \brief whether the manager is empty */
  bool empty() const;
  
  /** \brief the number of objects managed by the manager */
  inline size_t size() const { return endpoints[0].size() / 2; }

protected:


  /** \brief SAP end point */
  struct EndPoint
  {
    /** \brief object related with the end point */
    CollisionObject* obj;

    /** \brief end point value */
    FCL_REAL value;

    /** \brief tag for whether it is a lower bound or higher bound of an interval, 0 for lo, and 1 for hi */
    char minmax;
  };

  /** \brief Extention interval tree's interval to SAP interval, adding more information */
  struct SAPInterval : public SimpleInterval
  {
    CollisionObject* obj;
    SAPInterval(double low_, double high_, CollisionObject* obj_) : SimpleInterval()
    {
      low = low_;
      high = high_;
      obj = obj_;
    }
  };


  bool checkColl(std::deque<SimpleInterval*>::const_iterator pos_start, std::deque<SimpleInterval*>::const_iterator pos_end, CollisionObject* obj, void* cdata, CollisionCallBack callback) const;

  bool checkDist(std::deque<SimpleInterval*>::const_iterator pos_start, std::deque<SimpleInterval*>::const_iterator pos_end, CollisionObject* obj, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const;

  bool collide_(CollisionObject* obj, void* cdata, CollisionCallBack callback) const;

  bool distance_(CollisionObject* obj, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const;

  /** \brief vector stores all the end points */
  std::vector<EndPoint> endpoints[3];

  /** \brief  interval tree manages the intervals */
  IntervalTree* interval_trees[3];

  std::map<CollisionObject*, SAPInterval*> obj_interval_maps[3];

  /** \brief tag for whether the interval tree is maintained suitably */
  bool setup_;
};

class DynamicAABBTreeCollisionManager : public BroadPhaseCollisionManager
{
public:
  typedef NodeBase<AABB> DynamicAABBNode;
  typedef boost::unordered_map<CollisionObject*, DynamicAABBNode*> DynamicAABBTable;

  int max_tree_nonbalanced_level;
  int tree_incremental_balance_pass;
  int& tree_topdown_balance_threshold;
  int& tree_topdown_level;
  int tree_init_level;

  
  DynamicAABBTreeCollisionManager() : tree_topdown_balance_threshold(dtree.bu_threshold),
                                      tree_topdown_level(dtree.topdown_level)
  {
    max_tree_nonbalanced_level = 10;
    tree_incremental_balance_pass = 10;
    tree_topdown_balance_threshold = 2;
    tree_topdown_level = 0;
    tree_init_level = 0;
    setup_ = false;
  }

  /** \brief add objects to the manager */
  void registerObjects(const std::vector<CollisionObject*>& other_objs);
  
  /** \brief add one object to the manager */
  void registerObject(CollisionObject* obj);

  /** \brief remove one object from the manager */
  void unregisterObject(CollisionObject* obj);

  /** \brief initialize the manager, related with the specific type of manager */
  void setup();

  /** \brief update the condition of manager */
  void update();

  /** \brief update the manager by explicitly given the object updated */
  void update(CollisionObject* updated_obj);

  /** \brief update the manager by explicitly given the set of objects update */
  void update(const std::vector<CollisionObject*>& updated_objs);

  /** \brief clear the manager */
  void clear()
  {
    dtree.clear();
    table.clear();
  }

  /** \brief return the objects managed by the manager */
  void getObjects(std::vector<CollisionObject*>& objs) const
  {
    objs.resize(this->size());
    std::transform(table.begin(), table.end(), objs.begin(), boost::bind(&DynamicAABBTable::value_type::first, _1));
  }

  /** \brief perform collision test between one object and all the objects belonging to the manager */
  void collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const
  {
    if(size() == 0) return;
    switch(obj->getCollisionGeometry()->getNodeType())
    {
    case GEOM_OCTREE:
      {
        const OcTree* octree = static_cast<const OcTree*>(obj->getCollisionGeometry());
        collisionRecurse(dtree.getRoot(), octree, octree->getRoot(), octree->getRootBV(), obj->getTransform(), cdata, callback); 
      }
      break;
    default:
      collisionRecurse(dtree.getRoot(), obj, cdata, callback);
    }
  }

  /** \brief perform distance computation between one object and all the objects belonging to the manager */
  void distance(CollisionObject* obj, void* cdata, DistanceCallBack callback) const
  {
    if(size() == 0) return;
    FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
    switch(obj->getCollisionGeometry()->getNodeType())
    {
    case GEOM_OCTREE:
      {
        const OcTree* octree = static_cast<const OcTree*>(obj->getCollisionGeometry());
        distanceRecurse(dtree.getRoot(), octree, octree->getRoot(), octree->getRootBV(), obj->getTransform(), cdata, callback, min_dist);
      }
      break;
    default:
      distanceRecurse(dtree.getRoot(), obj, cdata, callback, min_dist);
    }
  }

  /** \brief perform collision test for the objects belonging to the manager (i.e., N^2 self collision) */
  void collide(void* cdata, CollisionCallBack callback) const
  {
    if(size() == 0) return;
    selfCollisionRecurse(dtree.getRoot(), cdata, callback);
  }

  /** \brief perform distance test for the objects belonging to the manager (i.e., N^2 self distance) */
  void distance(void* cdata, DistanceCallBack callback) const
  {
    if(size() == 0) return;
    FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
    selfDistanceRecurse(dtree.getRoot(), cdata, callback, min_dist);
  }

  /** \brief perform collision test with objects belonging to another manager */
  void collide(BroadPhaseCollisionManager* other_manager_, void* cdata, CollisionCallBack callback) const
  {
    DynamicAABBTreeCollisionManager* other_manager = static_cast<DynamicAABBTreeCollisionManager*>(other_manager_);
    if((size() == 0) || (other_manager->size() == 0)) return;
    collisionRecurse(dtree.getRoot(), other_manager->dtree.getRoot(), cdata, callback);
  }

  /** \brief perform distance test with objects belonging to another manager */
  void distance(BroadPhaseCollisionManager* other_manager_, void* cdata, DistanceCallBack callback) const
  {
    DynamicAABBTreeCollisionManager* other_manager = static_cast<DynamicAABBTreeCollisionManager*>(other_manager_);
    if((size() == 0) || (other_manager->size() == 0)) return;
    FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
    distanceRecurse(dtree.getRoot(), other_manager->dtree.getRoot(), cdata, callback, min_dist);
  }
  
  /** \brief whether the manager is empty */
  bool empty() const
  {
    return dtree.empty();
  }
  
  /** \brief the number of objects managed by the manager */
  size_t size() const
  {
    return dtree.size();
  }

  const HierarchyTree<AABB>& getTree() const { return dtree; }


private:
  HierarchyTree<AABB> dtree;
  boost::unordered_map<CollisionObject*, DynamicAABBNode*> table;

  bool setup_;

  bool collisionRecurse(DynamicAABBNode* root1, DynamicAABBNode* root2, void* cdata, CollisionCallBack callback) const;

  bool collisionRecurse(DynamicAABBNode* root, CollisionObject* query, void* cdata, CollisionCallBack callback) const;

  bool selfCollisionRecurse(DynamicAABBNode* root, void* cdata, CollisionCallBack callback) const;

  bool distanceRecurse(DynamicAABBNode* root1, DynamicAABBNode* root2, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const;

  bool distanceRecurse(DynamicAABBNode* root, CollisionObject* query, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const;

  bool selfDistanceRecurse(DynamicAABBNode* root, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const;  

  void update_(CollisionObject* updated_obj);

  /** \brief special manager-obj collision for octree */
  bool collisionRecurse(DynamicAABBNode* root1, const OcTree* tree2, const OcTree::OcTreeNode* root2, const AABB& root2_bv, const SimpleTransform& tf2, void* cdata, CollisionCallBack callback) const;

  bool distanceRecurse(DynamicAABBNode* root1, const OcTree* tree2, const OcTree::OcTreeNode* root2, const AABB& root2_bv, const SimpleTransform& tf2, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const;

};




class DynamicAABBTreeCollisionManager2 : public BroadPhaseCollisionManager
{
public:
  typedef alternative::NodeBase<AABB> DynamicAABBNode;
  typedef boost::unordered_map<CollisionObject*, size_t> DynamicAABBTable;

  int max_tree_nonbalanced_level;
  int tree_incremental_balance_pass;
  int& tree_topdown_balance_threshold;
  int& tree_topdown_level;
  int tree_init_level;
  
  DynamicAABBTreeCollisionManager2() : tree_topdown_balance_threshold(dtree.bu_threshold),
                                       tree_topdown_level(dtree.topdown_level)
  {
    max_tree_nonbalanced_level = 10;
    tree_incremental_balance_pass = 10;
    tree_topdown_balance_threshold = 2;
    tree_topdown_level = 0;
    tree_init_level = 0;
    setup_ = false;
  }

  /** \brief add objects to the manager */
  void registerObjects(const std::vector<CollisionObject*>& other_objs);
  
  /** \brief add one object to the manager */
  void registerObject(CollisionObject* obj);

  /** \brief remove one object from the manager */
  void unregisterObject(CollisionObject* obj);

  /** \brief initialize the manager, related with the specific type of manager */
  void setup();

  /** \brief update the condition of manager */
  void update();

  /** \brief update the manager by explicitly given the object updated */
  void update(CollisionObject* updated_obj);

  /** \brief update the manager by explicitly given the set of objects update */
  void update(const std::vector<CollisionObject*>& updated_objs);

  /** \brief clear the manager */
  void clear()
  {
    dtree.clear();
    table.clear();
  }

  /** \brief return the objects managed by the manager */
  void getObjects(std::vector<CollisionObject*>& objs) const
  {
    objs.resize(this->size());
    std::transform(table.begin(), table.end(), objs.begin(), boost::bind(&DynamicAABBTable::value_type::first, _1));
  }

  /** \brief perform collision test between one object and all the objects belonging to the manager */
  void collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const
  {
    if(size() == 0) return;
    switch(obj->getCollisionGeometry()->getNodeType())
    {
    case GEOM_OCTREE:
      {
        const OcTree* octree = static_cast<const OcTree*>(obj->getCollisionGeometry());
        collisionRecurse(dtree.getNodes(), dtree.getRoot(), octree, octree->getRoot(), octree->getRootBV(), obj->getTransform(), cdata, callback); 
      }
      break;
    default:
      collisionRecurse(dtree.getNodes(), dtree.getRoot(), obj, cdata, callback);
    }
  }

  /** \brief perform distance computation between one object and all the objects belonging to the manager */
  void distance(CollisionObject* obj, void* cdata, DistanceCallBack callback) const
  {
    if(size() == 0) return;
    FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
    switch(obj->getCollisionGeometry()->getNodeType())
    {
    case GEOM_OCTREE:
      {
        const OcTree* octree = static_cast<const OcTree*>(obj->getCollisionGeometry());
        distanceRecurse(dtree.getNodes(), dtree.getRoot(), octree, octree->getRoot(), octree->getRootBV(), obj->getTransform(), cdata, callback, min_dist);
      }
      break;
    default:
      distanceRecurse(dtree.getNodes(), dtree.getRoot(), obj, cdata, callback, min_dist);
    }
  }

  /** \brief perform collision test for the objects belonging to the manager (i.e., N^2 self collision) */
  void collide(void* cdata, CollisionCallBack callback) const
  {
    if(size() == 0) return;
    selfCollisionRecurse(dtree.getNodes(), dtree.getRoot(), cdata, callback);
  }

  /** \brief perform distance test for the objects belonging to the manager (i.e., N^2 self distance) */
  void distance(void* cdata, DistanceCallBack callback) const
  {
    if(size() == 0) return;
    FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
    selfDistanceRecurse(dtree.getNodes(), dtree.getRoot(), cdata, callback, min_dist);
  }

  /** \brief perform collision test with objects belonging to another manager */
  void collide(BroadPhaseCollisionManager* other_manager_, void* cdata, CollisionCallBack callback) const
  {
    DynamicAABBTreeCollisionManager2* other_manager = static_cast<DynamicAABBTreeCollisionManager2*>(other_manager_);
    if((size() == 0) || (other_manager->size() == 0)) return;
    collisionRecurse(dtree.getNodes(), dtree.getRoot(), other_manager->dtree.getNodes(), other_manager->dtree.getRoot(), cdata, callback);
  }

  /** \brief perform distance test with objects belonging to another manager */
  void distance(BroadPhaseCollisionManager* other_manager_, void* cdata, DistanceCallBack callback) const
  {
    DynamicAABBTreeCollisionManager2* other_manager = static_cast<DynamicAABBTreeCollisionManager2*>(other_manager_);
    if((size() == 0) || (other_manager->size() == 0)) return;
    FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
    distanceRecurse(dtree.getNodes(), dtree.getRoot(), other_manager->dtree.getNodes(), other_manager->dtree.getRoot(), cdata, callback, min_dist);
  }
  
  /** \brief whether the manager is empty */
  bool empty() const
  {
    return dtree.empty();
  }
  
  /** \brief the number of objects managed by the manager */
  size_t size() const
  {
    return dtree.size();
  }


  const alternative::HierarchyTree<AABB>& getTree() const { return dtree; }

private:
  alternative::HierarchyTree<AABB> dtree;
  boost::unordered_map<CollisionObject*, size_t> table;

  bool setup_;

  bool collisionRecurse(DynamicAABBNode* nodes1, size_t root1, DynamicAABBNode* nodes2, size_t root2, void* cdata, CollisionCallBack callback) const;

  bool collisionRecurse(DynamicAABBNode* nodes, size_t root, CollisionObject* query, void* cdata, CollisionCallBack callback) const;

  bool selfCollisionRecurse(DynamicAABBNode* nodes, size_t root, void* cdata, CollisionCallBack callback) const;

  bool distanceRecurse(DynamicAABBNode* nodes1, size_t root1, DynamicAABBNode* nodes2, size_t root2, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const;

  bool distanceRecurse(DynamicAABBNode* nodes, size_t root, CollisionObject* query, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const;

  bool selfDistanceRecurse(DynamicAABBNode* nodes, size_t root, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const;  

  void update_(CollisionObject* updated_obj);

  /** \brief special manager-obj collision for octree */
  bool collisionRecurse(DynamicAABBNode* nodes1, size_t root1_id, const OcTree* tree2, const OcTree::OcTreeNode* root2, const AABB& root2_bv, const SimpleTransform& tf2, void* cdata, CollisionCallBack callback) const;

  bool distanceRecurse(DynamicAABBNode* nodes1, size_t root1_id, const OcTree* tree2, const OcTree::OcTreeNode* root2, const AABB& root2_bv, const SimpleTransform& tf2, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const;

  
};

}

#endif

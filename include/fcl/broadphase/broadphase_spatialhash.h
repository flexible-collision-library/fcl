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
struct SpatialHash
{
  SpatialHash(const AABB& scene_limit_, FCL_REAL cell_size_) : cell_size(cell_size_),
                                                               scene_limit(scene_limit_)
  {
    width[0] = std::ceil(scene_limit.width() / cell_size);
    width[1] = std::ceil(scene_limit.height() / cell_size);
    width[2] = std::ceil(scene_limit.depth() / cell_size);
  }
    
  std::vector<unsigned int> operator() (const AABB& aabb) const
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

  FCL_REAL cell_size;
  AABB scene_limit;
  unsigned int width[3];
};

/// @brief spatial hashing collision mananger
template<typename HashTable = SimpleHashTable<AABB, CollisionObject*, SpatialHash> >
class SpatialHashingCollisionManager : public BroadPhaseCollisionManager
{
public:
  SpatialHashingCollisionManager(FCL_REAL cell_size, const Vec3f& scene_min, const Vec3f& scene_max, unsigned int default_table_size = 1000) : scene_limit(AABB(scene_min, scene_max)),
                                                                                                                                               hash_table(new HashTable(SpatialHash(scene_limit, cell_size)))
  {
    hash_table->init(default_table_size);
  }

  ~SpatialHashingCollisionManager()
  {
    delete hash_table;
  }

  /// @brief add one object to the manager
  void registerObject(CollisionObject* obj);

  /// @brief remove one object from the manager
  void unregisterObject(CollisionObject* obj);

  /// @brief initialize the manager, related with the specific type of manager
  void setup();

  /// @brief update the condition of manager
  void update();

  /// @brief update the manager by explicitly given the object updated
  void update(CollisionObject* updated_obj);

  /// @brief update the manager by explicitly given the set of objects update
  void update(const std::vector<CollisionObject*>& updated_objs);

  /// @brief clear the manager
  void clear();

  /// @brief return the objects managed by the manager
  void getObjects(std::vector<CollisionObject*>& objs) const;

  /// @brief perform collision test between one object and all the objects belonging to the manager
  void collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const;

  /// @brief perform distance computation between one object and all the objects belonging ot the manager
  void distance(CollisionObject* obj, void* cdata, DistanceCallBack callback) const;

  /// @brief perform collision test for the objects belonging to the manager (i.e, N^2 self collision)
  void collide(void* cdata, CollisionCallBack callback) const;

  /// @brief perform distance test for the objects belonging to the manager (i.e., N^2 self distance)
  void distance(void* cdata, DistanceCallBack callback) const;

  /// @brief perform collision test with objects belonging to another manager
  void collide(BroadPhaseCollisionManager* other_manager, void* cdata, CollisionCallBack callback) const;

  /// @brief perform distance test with objects belonging to another manager
  void distance(BroadPhaseCollisionManager* other_manager, void* cdata, DistanceCallBack callback) const;

  /// @brief whether the manager is empty
  bool empty() const;

  /// @brief the number of objects managed by the manager
  size_t size() const;

  /// @brief compute the bound for the environent
  static void computeBound(std::vector<CollisionObject*>& objs, Vec3f& l, Vec3f& u)
  {
    AABB bound;
    for(unsigned int i = 0; i < objs.size(); ++i)
      bound += objs[i]->getAABB();
    
    l = bound.min_;
    u = bound.max_;
  }

protected:

  /// @brief perform collision test between one object and all the objects belonging to the manager
  bool collide_(CollisionObject* obj, void* cdata, CollisionCallBack callback) const;

  /// @brief perform distance computation between one object and all the objects belonging ot the manager
  bool distance_(CollisionObject* obj, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const;


  /// @brief all objects in the scene
  std::list<CollisionObject*> objs;

  /// @brief objects outside the scene limit are in another list
  std::list<CollisionObject*> objs_outside_scene_limit;

  /// @brief the size of the scene
  AABB scene_limit;

  /// @brief store the map between objects and their aabbs. will make update more convenient
  std::map<CollisionObject*, AABB> obj_aabb_map; 

  /// @brief objects in the scene limit (given by scene_min and scene_max) are in the spatial hash table
  HashTable* hash_table;

};


}

#include "fcl/broadphase/broadphase_spatialhash.hxx"


#endif

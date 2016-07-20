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

namespace fcl
{

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

}

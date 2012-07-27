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


#include "fcl/broad_phase_collision.h"
#include "fcl/collision.h"
#include "fcl/distance.h"
#include "fcl/BV.h"
#include "fcl/geometric_shapes_utility.h"
#include <algorithm>
#include <set>
#include <deque>
#include <iostream>

namespace fcl
{


bool defaultCollisionFunction(CollisionObject* o1, CollisionObject* o2, void* cdata_)
{
  CollisionData* cdata = static_cast<CollisionData*>(cdata_);
  const CollisionRequest& request = cdata->request;
  CollisionResult& result = cdata->result;

  if(cdata->done) return true;

  collide(o1, o2, request, result);

  if( (!request.exhaustive) && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
    cdata->done = true;

  return cdata->done;
}

bool defaultDistanceFunction(CollisionObject* o1, CollisionObject* o2, void* cdata_, FCL_REAL& dist)
{
  DistanceData* cdata = static_cast<DistanceData*>(cdata_);
  const DistanceRequest& request = cdata->request;
  DistanceResult& result = cdata->result;

  if(cdata->done) { dist = result.min_distance; return true; }

  DistanceResult local_result;
  distance(o1, o2, request, local_result);
  FCL_REAL d = local_result.min_distance;
  if(result.min_distance > d) result.min_distance = d;
  
  dist = result.min_distance;

  if(dist <= 0) return true; // in collision or in touch

  return cdata->done;
}


void NaiveCollisionManager::registerObjects(const std::vector<CollisionObject*>& other_objs)
{
  std::copy(other_objs.begin(), other_objs.end(), std::back_inserter(objs));
}

void NaiveCollisionManager::unregisterObject(CollisionObject* obj)
{
  objs.remove(obj);
}

void NaiveCollisionManager::registerObject(CollisionObject* obj)
{
  objs.push_back(obj);
}

void NaiveCollisionManager::setup()
{

}

void NaiveCollisionManager::update()
{

}

void NaiveCollisionManager::clear()
{
  objs.clear();
}

void NaiveCollisionManager::getObjects(std::vector<CollisionObject*>& objs_) const
{
  objs_.resize(objs.size());
  std::copy(objs.begin(), objs.end(), objs_.begin());
}

void NaiveCollisionManager::collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const
{
  if(size() == 0) return;

  for(std::list<CollisionObject*>::const_iterator it = objs.begin(), end = objs.end(); it != end; ++it)
  {
    if(callback(obj, *it, cdata))
      return;
  }
}

void NaiveCollisionManager::distance(CollisionObject* obj, void* cdata, DistanceCallBack callback) const
{
  if(size() == 0) return;

  FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
  for(std::list<CollisionObject*>::const_iterator it = objs.begin(), end = objs.end(); 
      it != end; ++it)
  {
    if(obj->getAABB().distance((*it)->getAABB()) < min_dist)
    {
      if(callback(obj, *it, cdata, min_dist))
        return;
    }
  }
}

void NaiveCollisionManager::collide(void* cdata, CollisionCallBack callback) const
{
  if(size() == 0) return;

  for(std::list<CollisionObject*>::const_iterator it1 = objs.begin(), end = objs.end(); 
      it1 != end; ++it1)
  {
    std::list<CollisionObject*>::const_iterator it2 = it1; it2++;
    for(; it2 != end; ++it2)
    {
      if((*it1)->getAABB().overlap((*it2)->getAABB()))
        if(callback(*it1, *it2, cdata))
          return;
    }
  }
}

void NaiveCollisionManager::distance(void* cdata, DistanceCallBack callback) const
{
  if(size() == 0) return;
  
  FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
  for(std::list<CollisionObject*>::const_iterator it1 = objs.begin(), end = objs.end(); it1 != end; ++it1)
  {
    std::list<CollisionObject*>::const_iterator it2 = it1; it2++;
    for(; it2 != end; ++it2)
    {
      if((*it1)->getAABB().distance((*it2)->getAABB()) < min_dist)
      {
        if(callback(*it1, *it2, cdata, min_dist))
          return;
      }
    }
  }
}

void NaiveCollisionManager::collide(BroadPhaseCollisionManager* other_manager_, void* cdata, CollisionCallBack callback) const
{
  NaiveCollisionManager* other_manager = static_cast<NaiveCollisionManager*>(other_manager_);
  
  if((size() == 0) || (other_manager->size() == 0)) return;

  if(this == other_manager) 
  {
    collide(cdata, callback);
    return;
  }

  for(std::list<CollisionObject*>::const_iterator it1 = objs.begin(), end1 = objs.end(); it1 != end1; ++it1)
  {
    for(std::list<CollisionObject*>::const_iterator it2 = other_manager->objs.begin(), end2 = other_manager->objs.end(); it2 != end2; ++it2)
    {
      if((*it1)->getAABB().overlap((*it2)->getAABB()))
        if(callback((*it1), (*it2), cdata))
          return;
    }
  }
}

void NaiveCollisionManager::distance(BroadPhaseCollisionManager* other_manager_, void* cdata, DistanceCallBack callback) const
{
  NaiveCollisionManager* other_manager = static_cast<NaiveCollisionManager*>(other_manager_);

  if((size() == 0) || (other_manager->size() == 0)) return;

  if(this == other_manager)
  {
    distance(cdata, callback);
    return;
  }
  
  FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
  for(std::list<CollisionObject*>::const_iterator it1 = objs.begin(), end1 = objs.end(); it1 != end1; ++it1)
  {
    for(std::list<CollisionObject*>::const_iterator it2 = other_manager->objs.begin(), end2 = other_manager->objs.end(); it2 != end2; ++it2)
    {
      if((*it1)->getAABB().distance((*it2)->getAABB()) < min_dist)
      {
        if(callback(*it1, *it2, cdata, min_dist))
          return;
      }
    }
  }
}

bool NaiveCollisionManager::empty() const
{
  return objs.empty();
}


/** \brief Functor sorting objects according to the AABB lower x bound */
struct SortByXLow
{
  bool operator()(const CollisionObject* a, const CollisionObject* b) const
  {
    if(a->getAABB().min_[0] < b->getAABB().min_[0])
      return true;
    return false;
  }
};

/** \brief Functor sorting objects according to the AABB lower y bound */
struct SortByYLow
{
  bool operator()(const CollisionObject* a, const CollisionObject* b) const
  {
    if(a->getAABB().min_[1] < b->getAABB().min_[1])
      return true;
    return false;
  }
};

/** \brief Functor sorting objects according to the AABB lower z bound */
struct SortByZLow
{
  bool operator()(const CollisionObject* a, const CollisionObject* b) const
  {
    if(a->getAABB().min_[2] < b->getAABB().min_[2])
      return true;
    return false;
  }
};

/** \brief Dummy collision object with a point AABB */
class DummyCollisionObject : public CollisionObject
{
public:
  DummyCollisionObject(const AABB& aabb_) : CollisionObject()
  {
    aabb = aabb_;
  }

  void computeLocalAABB() {}
};


void SSaPCollisionManager::unregisterObject(CollisionObject* obj)
{
  setup();

  DummyCollisionObject dummyHigh(AABB(obj->getAABB().max_));

  std::vector<CollisionObject*>::iterator pos_start1 = objs_x.begin();
  std::vector<CollisionObject*>::iterator pos_end1 = std::upper_bound(pos_start1, objs_x.end(), &dummyHigh, SortByXLow());

  while(pos_start1 < pos_end1)
  {
    if(*pos_start1 == obj)
    {
      objs_x.erase(pos_start1);
      break;
    }
    ++pos_start1;
  }

  std::vector<CollisionObject*>::iterator pos_start2 = objs_y.begin();
  std::vector<CollisionObject*>::iterator pos_end2 = std::upper_bound(pos_start2, objs_y.end(), &dummyHigh, SortByYLow());

  while(pos_start2 < pos_end2)
  {
    if(*pos_start2 == obj)
    {
      objs_y.erase(pos_start2);
      break;
    }
    ++pos_start2;
  }

  std::vector<CollisionObject*>::iterator pos_start3 = objs_z.begin();
  std::vector<CollisionObject*>::iterator pos_end3 = std::upper_bound(pos_start3, objs_z.end(), &dummyHigh, SortByZLow());

  while(pos_start3 < pos_end3)
  {
    if(*pos_start3 == obj)
    {
      objs_z.erase(pos_start3);
      break;
    }
    ++pos_start3;
  }
}


void SSaPCollisionManager::registerObject(CollisionObject* obj)
{
  objs_x.push_back(obj);
  objs_y.push_back(obj);
  objs_z.push_back(obj);
  setup_ = false;
}

void SSaPCollisionManager::setup()
{
  if(!setup_)
  {
    std::sort(objs_x.begin(), objs_x.end(), SortByXLow());
    std::sort(objs_y.begin(), objs_y.end(), SortByYLow());
    std::sort(objs_z.begin(), objs_z.end(), SortByZLow());
    setup_ = true;
  }
}

void SSaPCollisionManager::update()
{
  setup_ = false;
  setup();
}

void SSaPCollisionManager::clear()
{
  objs_x.clear();
  objs_y.clear();
  objs_z.clear();
  setup_ = false;
}

void SSaPCollisionManager::getObjects(std::vector<CollisionObject*>& objs) const
{
  objs.resize(objs_x.size());
  std::copy(objs_x.begin(), objs_x.end(), objs.begin());
}

bool SSaPCollisionManager::checkColl(std::vector<CollisionObject*>::const_iterator pos_start, std::vector<CollisionObject*>::const_iterator pos_end,
                                     CollisionObject* obj, void* cdata, CollisionCallBack callback) const
{
  while(pos_start < pos_end)
  {
    if(*pos_start != obj) // no collision between the same object
    {
      if((*pos_start)->getAABB().overlap(obj->getAABB()))
      {
        if(callback(*pos_start, obj, cdata))
          return true;
      }
    }
    pos_start++;
  }
  return false;
}

bool SSaPCollisionManager::checkDis(std::vector<CollisionObject*>::const_iterator pos_start, std::vector<CollisionObject*>::const_iterator pos_end, CollisionObject* obj, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const
{
  while(pos_start < pos_end)
  {
    if(*pos_start != obj) // no distance between the same object
    {
      if((*pos_start)->getAABB().distance(obj->getAABB()) < min_dist)
      {
        if(callback(*pos_start, obj, cdata, min_dist)) 
          return true;
      }
    }
    pos_start++;
  }
  
  return false;
}



void SSaPCollisionManager::collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const
{
  if(size() == 0) return;

  collide_(obj, cdata, callback);
}

bool SSaPCollisionManager::collide_(CollisionObject* obj, void* cdata, CollisionCallBack callback) const
{
  static const unsigned int CUTOFF = 100;

  DummyCollisionObject dummyHigh(AABB(obj->getAABB().max_));
  bool coll_res = false;

  std::vector<CollisionObject*>::const_iterator pos_start1 = objs_x.begin();
  std::vector<CollisionObject*>::const_iterator pos_end1 = std::upper_bound(pos_start1, objs_x.end(), &dummyHigh, SortByXLow());
  unsigned int d1 = pos_end1 - pos_start1;

  if(d1 > CUTOFF)
  {
    std::vector<CollisionObject*>::const_iterator pos_start2 = objs_y.begin();
    std::vector<CollisionObject*>::const_iterator pos_end2 = std::upper_bound(pos_start2, objs_y.end(), &dummyHigh, SortByYLow());
    unsigned int d2 = pos_end2 - pos_start2;

    if(d2 > CUTOFF)
    {
      std::vector<CollisionObject*>::const_iterator pos_start3 = objs_z.begin();
      std::vector<CollisionObject*>::const_iterator pos_end3 = std::upper_bound(pos_start3, objs_z.end(), &dummyHigh, SortByZLow());
      unsigned int d3 = pos_end3 - pos_start3;

      if(d3 > CUTOFF)
      {
        if(d3 <= d2 && d3 <= d1)
          coll_res = checkColl(pos_start3, pos_end3, obj, cdata, callback);
        else
        {
          if(d2 <= d3 && d2 <= d1)
            coll_res = checkColl(pos_start2, pos_end2, obj, cdata, callback);
          else
            coll_res = checkColl(pos_start1, pos_end1, obj, cdata, callback);
        }
      }
      else
        coll_res = checkColl(pos_start3, pos_end3, obj, cdata, callback);
    }
    else
      coll_res = checkColl(pos_start2, pos_end2, obj, cdata, callback);
  }
  else
    coll_res = checkColl(pos_start1, pos_end1, obj, cdata, callback);

  return coll_res;
}


void SSaPCollisionManager::distance(CollisionObject* obj, void* cdata, DistanceCallBack callback) const
{
  if(size() == 0) return;

  FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
  distance_(obj, cdata, callback, min_dist); 
}

bool SSaPCollisionManager::distance_(CollisionObject* obj, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const
{
  static const unsigned int CUTOFF = 100;
  Vec3f delta = (obj->getAABB().max_ - obj->getAABB().min_) * 0.5;
  Vec3f dummy_vector = obj->getAABB().max_;
  if(min_dist < std::numeric_limits<FCL_REAL>::max())
    dummy_vector += Vec3f(min_dist, min_dist, min_dist);

  std::vector<CollisionObject*>::const_iterator pos_start1 = objs_x.begin();
  std::vector<CollisionObject*>::const_iterator pos_start2 = objs_y.begin();
  std::vector<CollisionObject*>::const_iterator pos_start3 = objs_z.begin();
  std::vector<CollisionObject*>::const_iterator pos_end1 = objs_x.end();
  std::vector<CollisionObject*>::const_iterator pos_end2 = objs_y.end();
  std::vector<CollisionObject*>::const_iterator pos_end3 = objs_z.end();

  int status = 1;
  FCL_REAL old_min_distance;

  while(1)
  {
    old_min_distance = min_dist;
    DummyCollisionObject dummyHigh((AABB(dummy_vector)));

    pos_end1 = std::upper_bound(pos_start1, objs_x.end(), &dummyHigh, SortByXLow());
    unsigned int d1 = pos_end1 - pos_start1;

    bool dist_res = false;
    
    if(d1 > CUTOFF)
    {
      pos_end2 = std::upper_bound(pos_start2, objs_y.end(), &dummyHigh, SortByYLow());
      unsigned int d2 = pos_end2 - pos_start2;

      if(d2 > CUTOFF)
      {
        pos_end3 = std::upper_bound(pos_start3, objs_z.end(), &dummyHigh, SortByZLow());
        unsigned int d3 = pos_end3 - pos_start3;

        if(d3 > CUTOFF)
        {
          if(d3 <= d2 && d3 <= d1)
            dist_res = checkDis(pos_start3, pos_end3, obj, cdata, callback, min_dist);
          else
          {
            if(d2 <= d3 && d2 <= d1)
              dist_res = checkDis(pos_start2, pos_end2, obj, cdata, callback, min_dist);
            else
              dist_res = checkDis(pos_start1, pos_end1, obj, cdata, callback, min_dist);
          }
        }
        else
          dist_res = checkDis(pos_start3, pos_end3, obj, cdata, callback, min_dist);
      }
      else
        dist_res = checkDis(pos_start2, pos_end2, obj, cdata, callback, min_dist);
    }
    else
    {
      dist_res = checkDis(pos_start1, pos_end1, obj, cdata, callback, min_dist);
    }

    if(dist_res) return true;

    if(status == 1)
    {
      if(old_min_distance < std::numeric_limits<FCL_REAL>::max())
        break;
      else
      {
        // from infinity to a finite one, only need one additional loop 
        // to check the possible missed ones to the right of the objs array
        if(min_dist < old_min_distance) 
        {
          dummy_vector = obj->getAABB().max_ + Vec3f(min_dist, min_dist, min_dist);
          status = 0;
        }
        else // need more loop
        {
          if(dummy_vector.equal(obj->getAABB().max_))
            dummy_vector = dummy_vector + delta;
          else
            dummy_vector = dummy_vector * 2 - obj->getAABB().max_;
        }
      }
      
      // yes, following is wrong, will result in too large distance.
      // if(pos_end1 != objs_x.end()) pos_start1 = pos_end1;
      // if(pos_end2 != objs_y.end()) pos_start2 = pos_end2;
      // if(pos_end3 != objs_z.end()) pos_start3 = pos_end3;
    }
    else if(status == 0)
      break;
  }

  return false;
}

void SSaPCollisionManager::collide(void* cdata, CollisionCallBack callback) const
{
  if(size() == 0) return;

  std::vector<CollisionObject*>::const_iterator pos, run_pos, pos_end;
  size_t axis = selectOptimalAxis(objs_x, objs_y, objs_z, 
                                  pos, pos_end);
  size_t axis2 = (axis + 1 > 2) ? 0 : (axis + 1);
  size_t axis3 = (axis2 + 1 > 2) ? 0 : (axis2 + 1);

  run_pos = pos;

  while((run_pos < pos_end) && (pos < pos_end))
  {
    CollisionObject* obj = *(pos++);

    while(1)
    {
      if((*run_pos)->getAABB().min_[axis] < obj->getAABB().min_[axis])
      {
        run_pos++;
        if(run_pos == pos_end) break;
        continue;
      }
      else
      {
        run_pos++;
        break;
      }
    }

    if(run_pos < pos_end)
    {
      std::vector<CollisionObject*>::const_iterator run_pos2 = run_pos;

      while((*run_pos2)->getAABB().min_[axis] <= obj->getAABB().max_[axis])
      {
        CollisionObject* obj2 = *run_pos2;
        run_pos2++;

        if((obj->getAABB().max_[axis2] >= obj2->getAABB().min_[axis2]) && (obj2->getAABB().max_[axis2] >= obj->getAABB().min_[axis2]))
        {
          if((obj->getAABB().max_[axis3] >= obj2->getAABB().min_[axis3]) && (obj2->getAABB().max_[axis3] >= obj->getAABB().min_[axis3]))
          {
            if(callback(obj, obj2, cdata))
              return;
          }
        }

        if(run_pos2 == pos_end) break;
      }
    }
  }
}


void SSaPCollisionManager::distance(void* cdata, DistanceCallBack callback) const
{
  if(size() == 0) return;

  std::vector<CollisionObject*>::const_iterator it, it_end;
  size_t axis = selectOptimalAxis(objs_x, objs_y, objs_z, it, it_end);

  FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
  for(; it != it_end; ++it)
  {
    if(distance_(*it, cdata, callback, min_dist))
      return;
  }
}

void SSaPCollisionManager::collide(BroadPhaseCollisionManager* other_manager_, void* cdata, CollisionCallBack callback) const
{
  SSaPCollisionManager* other_manager = static_cast<SSaPCollisionManager*>(other_manager_);
  
  if((size() == 0) || (other_manager->size() == 0)) return;

  if(this == other_manager)
  {
    collide(cdata, callback);
    return;
  }


  std::vector<CollisionObject*>::const_iterator it, end;
  if(this->size() < other_manager->size())
  {
    for(it = objs_x.begin(), end = objs_x.end(); it != end; ++it)
      if(other_manager->collide_(*it, cdata, callback)) return;
  }
  else
  {
    for(it = other_manager->objs_x.begin(), end != other_manager->objs_x.end(); it != end; ++it)
      if(collide_(*it, cdata, callback)) return;
  }  
}

void SSaPCollisionManager::distance(BroadPhaseCollisionManager* other_manager_, void* cdata, DistanceCallBack callback) const
{
  SSaPCollisionManager* other_manager = static_cast<SSaPCollisionManager*>(other_manager_);

  if((size() == 0) || (other_manager->size() == 0)) return;

  if(this == other_manager)
  {
    distance(cdata, callback);
    return;
  }
  
  FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
  std::vector<CollisionObject*>::const_iterator it, end;
  if(this->size() < other_manager->size())
  {
    for(it = objs_x.begin(), end = objs_x.end(); it != end; ++it)
      if(other_manager->distance_(*it, cdata, callback, min_dist)) return;
  }
  else
  {
    for(it = other_manager->objs_x.begin(), end = other_manager->objs_x.end(); it != end; ++it)
      if(distance_(*it, cdata, callback, min_dist)) return;
  }
}

bool SSaPCollisionManager::empty() const
{
  return objs_x.empty();
}


void SaPCollisionManager::unregisterObject(CollisionObject* obj)
{
  std::list<SaPAABB*>::iterator it = AABB_arr.begin();
  for(std::list<SaPAABB*>::iterator end = AABB_arr.end(); it != end; ++it)
  {
    if((*it)->obj == obj)
      break;
  }

  AABB_arr.erase(it);
  obj_aabb_map.erase(obj);

  if(it == AABB_arr.end())
    return;

  SaPAABB* curr = *it;
  *it = NULL;

  for(int coord = 0; coord < 3; ++coord)
  {
    //first delete the lo endpoint of the interval.
    if(curr->lo->prev[coord] == NULL)
      elist[coord] = curr->lo->next[coord];
    else
      curr->lo->prev[coord]->next[coord] = curr->lo->next[coord];

    curr->lo->next[coord]->prev[coord] = curr->lo->prev[coord];

    //then, delete the "hi" endpoint.
    if(curr->hi->prev[coord] == NULL)
      elist[coord] = curr->hi->next[coord];
    else
      curr->hi->prev[coord]->next[coord] = curr->hi->next[coord];

    if(curr->hi->next[coord] != NULL)
      curr->hi->next[coord]->prev[coord] = curr->hi->prev[coord];
  }

  delete curr->lo;
  delete curr->hi;
  delete curr;

  overlap_pairs.remove_if(isUnregistered(obj));
}

void SaPCollisionManager::registerObjects(const std::vector<CollisionObject*>& other_objs)
{
  if(size() > 0)
    BroadPhaseCollisionManager::registerObjects(other_objs);
  else
  {
    std::vector<EndPoint*> endpoints(2 * other_objs.size());
    
    for(size_t i = 0; i < other_objs.size(); ++i)
    {
      SaPAABB* sapaabb = new SaPAABB();
      sapaabb->obj = other_objs[i];
      sapaabb->lo = new EndPoint();
      sapaabb->hi = new EndPoint();
      sapaabb->cached = other_objs[i]->getAABB();
      endpoints[2 * i] = sapaabb->lo;
      endpoints[2 * i + 1] = sapaabb->hi;
      sapaabb->lo->minmax = 0;
      sapaabb->hi->minmax = 1;
      sapaabb->lo->aabb = sapaabb;
      sapaabb->hi->aabb = sapaabb;
      AABB_arr.push_back(sapaabb);
      obj_aabb_map[other_objs[i]] = sapaabb;
    }


    FCL_REAL scale[3];
    for(size_t coord = 0; coord < 3; ++coord)
    { 
      std::sort(endpoints.begin(), endpoints.end(), 
                boost::bind(std::less<FCL_REAL>(),
                            boost::bind(static_cast<FCL_REAL (EndPoint::*)(size_t) const >(&EndPoint::getVal), _1, coord),
                            boost::bind(static_cast<FCL_REAL (EndPoint::*)(size_t) const >(&EndPoint::getVal), _2, coord)));

      endpoints[0]->prev[coord] = NULL;
      endpoints[0]->next[coord] = endpoints[1];
      for(int i = 1; i < endpoints.size() - 1; ++i)
      {
        endpoints[i]->prev[coord] = endpoints[i-1];
        endpoints[i]->next[coord] = endpoints[i+1];
      }
      endpoints[endpoints.size() - 1]->prev[coord] = endpoints[endpoints.size() - 2];
      endpoints[endpoints.size() - 1]->next[coord] = NULL;

      elist[coord] = endpoints[0];

      scale[coord] = endpoints.back()->aabb->cached.max_[coord] - endpoints[0]->aabb->cached.min_[coord];
    }
    
    int axis = 0;
    if(scale[axis] < scale[1]) axis = 1;
    if(scale[axis] < scale[2]) axis = 2;

    EndPoint* pos = elist[axis];
    
    while(pos != NULL)
    {
      EndPoint* pos_next = NULL;
      SaPAABB* aabb = pos->aabb;
      EndPoint* pos_it = pos->next[axis];
      
      while(pos_it != NULL)
      {
        if(pos_it->aabb == aabb)
        {
          if(pos_next == NULL) pos_next = pos_it; 
          break;
        }
      
        if(pos_it->minmax == 0)
        {
          if(pos_next == NULL) pos_next = pos_it;
          if(pos_it->aabb->cached.overlap(aabb->cached))
            overlap_pairs.push_back(SaPPair(pos_it->aabb->obj, aabb->obj));
        }
        pos_it = pos_it->next[axis];
      }

      pos = pos_next;
    }
  }

  updateVelist();
}

void SaPCollisionManager::registerObject(CollisionObject* obj)
{
  SaPAABB* curr = new SaPAABB;
  curr->cached = obj->getAABB();
  curr->obj = obj;
  curr->lo = new EndPoint;
  curr->lo->minmax = 0;
  curr->lo->aabb = curr;

  curr->hi = new EndPoint;
  curr->hi->minmax = 1;
  curr->hi->aabb = curr;

  for(int coord = 0; coord < 3; ++coord)
  {
    EndPoint* current = elist[coord];

    // first insert the lo end point
    if(current == NULL) // empty list
    {
      elist[coord] = curr->lo;
      curr->lo->prev[coord] = curr->lo->next[coord] = NULL;
    }
    else // otherwise, find the correct location in the list and insert
    {
      EndPoint* curr_lo = curr->lo;
      FCL_REAL curr_lo_val = curr_lo->getVal()[coord];
      while((current->getVal()[coord] < curr_lo_val) && (current->next[coord] != NULL))
        current = current->next[coord];

      if(current->getVal()[coord] >= curr_lo_val)
      {
        curr_lo->prev[coord] = current->prev[coord];
        curr_lo->next[coord] = current;
        if(current->prev[coord] == NULL)
          elist[coord] = curr_lo;
        else
          current->prev[coord]->next[coord] = curr_lo;

        current->prev[coord] = curr_lo;
      }
      else
      {
        curr_lo->prev[coord] = current;
        curr_lo->next[coord] = NULL;
        current->next[coord] = curr_lo;
      }
    }

    // now insert hi end point
    current = curr->lo;

    EndPoint* curr_hi = curr->hi;
    FCL_REAL curr_hi_val = curr_hi->getVal()[coord];
    
    if(coord == 0)
    {
      while((current->getVal()[coord] < curr_hi_val) && (current->next[coord] != NULL))
      {
        if(current != curr->lo)
          if(current->aabb->cached.overlap(curr->cached))
            overlap_pairs.push_back(SaPPair(current->aabb->obj, obj));

        current = current->next[coord];
      }
    }
    else
    {
      while((current->getVal()[coord] < curr_hi_val) && (current->next[coord] != NULL))
        current = current->next[coord];
    }

    if(current->getVal()[coord] >= curr_hi_val)
    {
      curr_hi->prev[coord] = current->prev[coord];
      curr_hi->next[coord] = current;
      if(current->prev[coord] == NULL)
        elist[coord] = curr_hi;
      else
        current->prev[coord]->next[coord] = curr_hi;

      current->prev[coord] = curr_hi;
    }
    else
    {
      curr_hi->prev[coord] = current;
      curr_hi->next[coord] = NULL;
      current->next[coord] = curr_hi;
    }
  }

  AABB_arr.push_back(curr);

  obj_aabb_map[obj] = curr;

  updateVelist();
}

void SaPCollisionManager::setup()
{
  FCL_REAL scale[3];
  scale[0] = (velist[0].back())->getVal(0) - velist[0][0]->getVal(0);
  scale[1] = (velist[1].back())->getVal(1) - velist[1][0]->getVal(1);;
  scale[2] = (velist[2].back())->getVal(2) - velist[2][0]->getVal(2);
  size_t axis = 0;
  if(scale[axis] < scale[1]) axis = 1;
  if(scale[axis] < scale[2]) axis = 2;
  optimal_axis = axis;
}

void SaPCollisionManager::update_(SaPAABB* updated_aabb)
{
  if(updated_aabb->cached.equal(updated_aabb->obj->getAABB()))
    return;

  SaPAABB* current = updated_aabb;

  Vec3f new_min = current->obj->getAABB().min_;
  Vec3f new_max = current->obj->getAABB().max_;

  SaPAABB dummy;
  dummy.cached = current->obj->getAABB();

  for(int coord = 0; coord < 3; ++coord)
  {
    int direction; // -1 reverse, 0 nochange, 1 forward
    EndPoint* temp;

    if(current->lo->getVal(coord) > new_min[coord])
      direction = -1;
    else if(current->lo->getVal(coord) < new_min[coord])
      direction = 1;
    else direction = 0;

    if(direction == -1)
    {
      //first update the "lo" endpoint of the interval
      if(current->lo->prev[coord] != NULL)
      {
        temp = current->lo;
        while((temp != NULL) && (temp->getVal(coord) > new_min[coord]))
        {
          if(temp->minmax == 1)
            if(temp->aabb->cached.overlap(dummy.cached))
              addToOverlapPairs(SaPPair(temp->aabb->obj, current->obj));
          temp = temp->prev[coord];
        }

        if(temp == NULL)
        {
          current->lo->prev[coord]->next[coord] = current->lo->next[coord];
          current->lo->next[coord]->prev[coord] = current->lo->prev[coord];
          current->lo->prev[coord] = NULL;
          current->lo->next[coord] = elist[coord];
          elist[coord]->prev[coord] = current->lo;
          elist[coord] = current->lo;
        }
        else
        {
          current->lo->prev[coord]->next[coord] = current->lo->next[coord];
          current->lo->next[coord]->prev[coord] = current->lo->prev[coord];
          current->lo->prev[coord] = temp;
          current->lo->next[coord] = temp->next[coord];
          temp->next[coord]->prev[coord] = current->lo;
          temp->next[coord] = current->lo;
        }
      }

      current->lo->getVal(coord) = new_min[coord];

      // update hi end point
      temp = current->hi;
      while(temp->getVal(coord) > new_max[coord])
      {
        if((temp->minmax == 0) && (temp->aabb->cached.overlap(current->cached)))
          removeFromOverlapPairs(SaPPair(temp->aabb->obj, current->obj));
        temp = temp->prev[coord];
      }

      current->hi->prev[coord]->next[coord] = current->hi->next[coord];
      if(current->hi->next[coord] != NULL)
        current->hi->next[coord]->prev[coord] = current->hi->prev[coord];
      current->hi->prev[coord] = temp;
      current->hi->next[coord] = temp->next[coord];
      if(temp->next[coord] != NULL)
        temp->next[coord]->prev[coord] = current->hi;
      temp->next[coord] = current->hi;

      current->hi->getVal(coord) = new_max[coord];
    }
    else if(direction == 1)
    {
      //here, we first update the "hi" endpoint.
      if(current->hi->next[coord] != NULL)
      {
        temp = current->hi;
        while((temp->next[coord] != NULL) && (temp->getVal(coord) < new_max[coord]))
        {
          if(temp->minmax == 0)
            if(temp->aabb->cached.overlap(dummy.cached))
              addToOverlapPairs(SaPPair(temp->aabb->obj, current->obj));
          temp = temp->next[coord];
        }

        if(temp->getVal(coord) < new_max[coord])
        {
          current->hi->prev[coord]->next[coord] = current->hi->next[coord];
          current->hi->next[coord]->prev[coord] = current->hi->prev[coord];
          current->hi->prev[coord] = temp;
          current->hi->next[coord] = NULL;
          temp->next[coord] = current->hi;
        }
        else
        {
          current->hi->prev[coord]->next[coord] = current->hi->next[coord];
          current->hi->next[coord]->prev[coord] = current->hi->prev[coord];
          current->hi->prev[coord] = temp->prev[coord];
          current->hi->next[coord] = temp;
          temp->prev[coord]->next[coord] = current->hi;
          temp->prev[coord] = current->hi;
        }
      }

      current->hi->getVal(coord) = new_max[coord];

      //then, update the "lo" endpoint of the interval.
      temp = current->lo;

      while(temp->getVal(coord) < new_min[coord])
      {
        if((temp->minmax == 1) && (temp->aabb->cached.overlap(current->cached)))
          removeFromOverlapPairs(SaPPair(temp->aabb->obj, current->obj));
        temp = temp->next[coord];
      }

      if(current->lo->prev[coord] != NULL)
        current->lo->prev[coord]->next[coord] = current->lo->next[coord];
      else
        elist[coord] = current->lo->next[coord];
      current->lo->next[coord]->prev[coord] = current->lo->prev[coord];
      current->lo->prev[coord] = temp->prev[coord];
      current->lo->next[coord] = temp;
      if(temp->prev[coord] != NULL)
        temp->prev[coord]->next[coord] = current->lo;
      else
        elist[coord] = current->lo;
      temp->prev[coord] = current->lo;
      current->lo->getVal(coord) = new_min[coord];
    }
  }
}

void SaPCollisionManager::update(CollisionObject* updated_obj)
{
  update_(obj_aabb_map[updated_obj]);

  updateVelist();

  setup();
}

void SaPCollisionManager::update(const std::vector<CollisionObject*>& updated_objs)
{
  for(size_t i = 0; i < updated_objs.size(); ++i)
    update_(obj_aabb_map[updated_objs[i]]);

  updateVelist();

  setup();
}

void SaPCollisionManager::update()
{
  for(std::list<SaPAABB*>::const_iterator it = AABB_arr.begin(), end = AABB_arr.end(); it != end; ++it)
  {
    update_(*it);
  }

  updateVelist();

  setup();
}


void SaPCollisionManager::clear()
{
  for(std::list<SaPAABB*>::iterator it = AABB_arr.begin(), end = AABB_arr.end();
      it != end;
      ++it)
  {
    delete (*it)->hi;
    delete (*it)->lo;
    delete *it;
    *it = NULL;
  }

  AABB_arr.clear();
  overlap_pairs.clear();

  elist[0] = NULL;
  elist[1] = NULL;
  elist[2] = NULL;

  velist[0].clear();
  velist[1].clear();
  velist[2].clear();

  obj_aabb_map.clear();
}

void SaPCollisionManager::getObjects(std::vector<CollisionObject*>& objs) const
{
  objs.resize(AABB_arr.size());
  int i = 0;
  for(std::list<SaPAABB*>::const_iterator it = AABB_arr.begin(), end = AABB_arr.end(); it != end; ++it, ++i)
  {
    objs[i] = (*it)->obj;
  }
}

bool SaPCollisionManager::collide_(CollisionObject* obj, void* cdata, CollisionCallBack callback) const
{
  size_t axis = optimal_axis;
  const AABB& obj_aabb = obj->getAABB();

  FCL_REAL min_val = obj_aabb.min_[axis];
  FCL_REAL max_val = obj_aabb.max_[axis];

  EndPoint dummy;
  SaPAABB dummy_aabb;
  dummy_aabb.cached = obj_aabb;
  dummy.minmax = 1;
  dummy.aabb = &dummy_aabb;
  
  // compute stop_pos by binary search, this is cheaper than check it in while iteration linearly
  std::vector<EndPoint*>::const_iterator res_it = std::upper_bound(velist[axis].begin(), velist[axis].end(), &dummy,
                                                                   boost::bind(std::less<FCL_REAL>(),
                                                                               boost::bind(static_cast<FCL_REAL (EndPoint::*)(size_t) const>(&EndPoint::getVal), _1, axis),
                                                                               boost::bind(static_cast<FCL_REAL (EndPoint::*)(size_t) const>(&EndPoint::getVal), _2, axis)));
  
  EndPoint* end_pos = NULL;
  if(res_it != velist[axis].end())
    end_pos = *res_it;
  
  EndPoint* pos = elist[axis];

  while(pos != end_pos)
  {
    if(pos->aabb->obj != obj)
    {
      if((pos->minmax == 0) && (pos->aabb->hi->getVal(axis) >= min_val))
      {
        if(pos->aabb->cached.overlap(obj->getAABB()))
          if(callback(obj, pos->aabb->obj, cdata))
            return true;
      }
    }
    pos = pos->next[axis];
  } 

  return false;
}

void SaPCollisionManager::collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const
{
  if(size() == 0) return;
  
  collide_(obj, cdata, callback);
}

bool SaPCollisionManager::distance_(CollisionObject* obj, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const
{
  Vec3f delta = (obj->getAABB().max_ - obj->getAABB().min_) * 0.5;
  AABB aabb = obj->getAABB();

  if(min_dist < std::numeric_limits<FCL_REAL>::max())
  {
    Vec3f min_dist_delta(min_dist, min_dist, min_dist);
    aabb.expand(min_dist_delta);
  }

  size_t axis = optimal_axis;

  int status = 1;
  FCL_REAL old_min_distance;

  EndPoint* start_pos = elist[axis];

  while(1)
  {
    old_min_distance = min_dist;
    FCL_REAL min_val = aabb.min_[axis];
    FCL_REAL max_val = aabb.max_[axis];

    EndPoint dummy; 
    SaPAABB dummy_aabb;
    dummy_aabb.cached = aabb;
    dummy.minmax = 1;
    dummy.aabb = &dummy_aabb;
    
 
    std::vector<EndPoint*>::const_iterator res_it = std::upper_bound(velist[axis].begin(), velist[axis].end(), &dummy,
                                                                     boost::bind(std::less<FCL_REAL>(),
                                                                                 boost::bind(static_cast<FCL_REAL (EndPoint::*)(size_t) const>(&EndPoint::getVal), _1, axis),
                                                                                 boost::bind(static_cast<FCL_REAL (EndPoint::*)(size_t) const>(&EndPoint::getVal), _2, axis)));

    EndPoint* end_pos = NULL;
    if(res_it != velist[axis].end())
      end_pos = *res_it;

    EndPoint* pos = start_pos;

    while(pos != end_pos)
    {
      // can change to pos->aabb->hi->getVal(axis) >= min_val - min_dist, and then update start_pos to end_pos.
      // but this seems slower.
      if((pos->minmax == 0) && (pos->aabb->hi->getVal(axis) >= min_val)) 
      {
        CollisionObject* curr_obj = pos->aabb->obj;
        if(curr_obj != obj)
        {
          if(!enable_tested_set_)
          {
            if(pos->aabb->cached.distance(obj->getAABB()) < min_dist)
            {
              if(callback(curr_obj, obj, cdata, min_dist))
                return true;
            }
          }
          else
          {
            if(!inTestedSet(curr_obj, obj))
            {
              if(pos->aabb->cached.distance(obj->getAABB()) < min_dist)
              {
                if(callback(curr_obj, obj, cdata, min_dist))
                  return true;
              }
              
              insertTestedSet(curr_obj, obj);
            }
          }
        }
      }

      pos = pos->next[axis];
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

void SaPCollisionManager::distance(CollisionObject* obj, void* cdata, DistanceCallBack callback) const
{
  if(size() == 0) return;

  FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
  
  distance_(obj, cdata, callback, min_dist);
}

void SaPCollisionManager::collide(void* cdata, CollisionCallBack callback) const
{
  if(size() == 0) return;

  for(std::list<SaPPair>::const_iterator it = overlap_pairs.begin(), end = overlap_pairs.end(); it != end; ++it)
  {
    CollisionObject* obj1 = it->obj1;
    CollisionObject* obj2 = it->obj2;

    if(callback(obj1, obj2, cdata))
      return;
  }
}

void SaPCollisionManager::distance(void* cdata, DistanceCallBack callback) const
{
  if(size() == 0) return;

  enable_tested_set_ = true;
  tested_set.clear();
  
  FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();

  for(std::list<SaPAABB*>::const_iterator it = AABB_arr.begin(), end = AABB_arr.end(); it != end; ++it)
  {
    if(distance_((*it)->obj, cdata, callback, min_dist))
      break;
  }

  enable_tested_set_ = false;
  tested_set.clear();
}

void SaPCollisionManager::collide(BroadPhaseCollisionManager* other_manager_, void* cdata, CollisionCallBack callback) const
{
  SaPCollisionManager* other_manager = static_cast<SaPCollisionManager*>(other_manager_);

  if((size() == 0) || (other_manager->size() == 0)) return;

  if(this == other_manager)
  {
    collide(cdata, callback);
    return;
  }

  if(this->size() < other_manager->size())
  {
    for(std::list<SaPAABB*>::const_iterator it = AABB_arr.begin(); it != AABB_arr.end(); ++it)
    {
      if(other_manager->collide_((*it)->obj, cdata, callback))
        return;
    }
  }
  else
  {
    for(std::list<SaPAABB*>::const_iterator it = other_manager->AABB_arr.begin(), end = other_manager->AABB_arr.end(); it != end; ++it)
    {
      if(collide_((*it)->obj, cdata, callback))
        return;
    }
  }
}

void SaPCollisionManager::distance(BroadPhaseCollisionManager* other_manager_, void* cdata, DistanceCallBack callback) const
{
  SaPCollisionManager* other_manager = static_cast<SaPCollisionManager*>(other_manager_);

  if((size() == 0) || (other_manager->size() == 0)) return;

  if(this == other_manager)
  {
    distance(cdata, callback);
    return;
  }

  FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();

  if(this->size() < other_manager->size())
  {
    for(std::list<SaPAABB*>::const_iterator it = AABB_arr.begin(), end = AABB_arr.end(); it != end; ++it)
    {
      if(other_manager->distance_((*it)->obj, cdata, callback, min_dist))
        return;
    }
  }
  else
  {
    for(std::list<SaPAABB*>::const_iterator it = other_manager->AABB_arr.begin(), end = other_manager->AABB_arr.end(); it != end; ++it)
    {
      if(distance_((*it)->obj, cdata, callback, min_dist))
        return;
    }
  }
}

bool SaPCollisionManager::empty() const
{
  return AABB_arr.size();
}



void IntervalTreeCollisionManager::unregisterObject(CollisionObject* obj)
{
  // must sorted before
  setup();

  EndPoint p;
  p.value = obj->getAABB().min_[0];
  std::vector<EndPoint>::iterator start1 = std::lower_bound(endpoints[0].begin(), endpoints[0].end(), p, boost::bind(&EndPoint::value, _1) < boost::bind(&EndPoint::value, _2));
  p.value = obj->getAABB().max_[0];
  std::vector<EndPoint>::iterator end1 = std::upper_bound(start1, endpoints[0].end(), p, boost::bind(&EndPoint::value, _1) < boost::bind(&EndPoint::value, _2));

  if(start1 < end1)
  {
    unsigned int start_id = start1 - endpoints[0].begin();
    unsigned int end_id = end1 - endpoints[0].begin();
    unsigned int cur_id = start_id;
    for(unsigned int i = start_id; i < end_id; ++i)
    {
      if(endpoints[0][i].obj != obj)
      {
        if(i == cur_id) cur_id++;
        else
        {
          endpoints[0][cur_id] = endpoints[0][i];
          cur_id++;
        }
      }
    }
    if(cur_id < end_id)
      endpoints[0].resize(endpoints[0].size() - 2);
  }

  p.value = obj->getAABB().min_[1];
  std::vector<EndPoint>::iterator start2 = std::lower_bound(endpoints[1].begin(), endpoints[1].end(), p, boost::bind(&EndPoint::value, _1) < boost::bind(&EndPoint::value, _2));
  p.value = obj->getAABB().max_[1];
  std::vector<EndPoint>::iterator end2 = std::upper_bound(start2, endpoints[1].end(), p, boost::bind(&EndPoint::value, _1) < boost::bind(&EndPoint::value, _2));

  if(start2 < end2)
  {
    unsigned int start_id = start2 - endpoints[1].begin();
    unsigned int end_id = end2 - endpoints[1].begin();
    unsigned int cur_id = start_id;
    for(unsigned int i = start_id; i < end_id; ++i)
    {
      if(endpoints[1][i].obj != obj)
      {
        if(i == cur_id) cur_id++;
        else
        {
          endpoints[1][cur_id] = endpoints[1][i];
          cur_id++;
        }
      }
    }
    if(cur_id < end_id)
      endpoints[1].resize(endpoints[1].size() - 2);
  }


  p.value = obj->getAABB().min_[2];
  std::vector<EndPoint>::iterator start3 = std::lower_bound(endpoints[2].begin(), endpoints[2].end(), p, boost::bind(&EndPoint::value, _1) < boost::bind(&EndPoint::value, _2));
  p.value = obj->getAABB().max_[2];
  std::vector<EndPoint>::iterator end3 = std::upper_bound(start3, endpoints[2].end(), p, boost::bind(&EndPoint::value, _1) < boost::bind(&EndPoint::value, _2));

  if(start3 < end3)
  {
    unsigned int start_id = start3 - endpoints[2].begin();
    unsigned int end_id = end3 - endpoints[2].begin();
    unsigned int cur_id = start_id;
    for(unsigned int i = start_id; i < end_id; ++i)
    {
      if(endpoints[2][i].obj != obj)
      {
        if(i == cur_id) cur_id++;
        else
        {
          endpoints[2][cur_id] = endpoints[2][i];
          cur_id++;
        }
      }
    }
    if(cur_id < end_id)
      endpoints[2].resize(endpoints[2].size() - 2);
  }

  // update the interval tree
  if(obj_interval_maps[0].find(obj) != obj_interval_maps[0].end())
  {
    SAPInterval* ivl1 = obj_interval_maps[0][obj];
    SAPInterval* ivl2 = obj_interval_maps[1][obj];
    SAPInterval* ivl3 = obj_interval_maps[2][obj];

    interval_trees[0]->deleteNode(ivl1);
    interval_trees[1]->deleteNode(ivl2);
    interval_trees[2]->deleteNode(ivl3);

    delete ivl1;
    delete ivl2;
    delete ivl3;

    obj_interval_maps[0].erase(obj);
    obj_interval_maps[1].erase(obj);             
    obj_interval_maps[2].erase(obj);
  }
}

void IntervalTreeCollisionManager::registerObject(CollisionObject* obj)
{
  EndPoint p, q;

  p.obj = obj;
  q.obj = obj;
  p.minmax = 0;
  q.minmax = 1;
  p.value = obj->getAABB().min_[0];
  q.value = obj->getAABB().max_[0];
  endpoints[0].push_back(p);
  endpoints[0].push_back(q);

  p.value = obj->getAABB().min_[1];
  q.value = obj->getAABB().max_[1];
  endpoints[1].push_back(p);
  endpoints[1].push_back(q);

  p.value = obj->getAABB().min_[2];
  q.value = obj->getAABB().max_[2];
  endpoints[2].push_back(p);
  endpoints[2].push_back(q);
  setup_ = false;
}

void IntervalTreeCollisionManager::setup()
{
  if(!setup_)
  {
    std::sort(endpoints[0].begin(), endpoints[0].end(), boost::bind(&EndPoint::value, _1) < boost::bind(&EndPoint::value, _2));
    std::sort(endpoints[1].begin(), endpoints[1].end(), boost::bind(&EndPoint::value, _1) < boost::bind(&EndPoint::value, _2));
    std::sort(endpoints[2].begin(), endpoints[2].end(), boost::bind(&EndPoint::value, _1) < boost::bind(&EndPoint::value, _2));

    for(int i = 0; i < 3; ++i)
      delete interval_trees[i];

    for(int i = 0; i < 3; ++i)
      interval_trees[i] = new IntervalTree;

    for(unsigned int i = 0, size = endpoints[0].size(); i < size; ++i)
    {
      EndPoint p = endpoints[0][i];
      CollisionObject* obj = p.obj;
      if(p.minmax == 0)
      {
        SAPInterval* ivl1 = new SAPInterval(obj->getAABB().min_[0], obj->getAABB().max_[0], obj);
        SAPInterval* ivl2 = new SAPInterval(obj->getAABB().min_[1], obj->getAABB().max_[1], obj);
        SAPInterval* ivl3 = new SAPInterval(obj->getAABB().min_[2], obj->getAABB().max_[2], obj);

        interval_trees[0]->insert(ivl1);
        interval_trees[1]->insert(ivl2);
        interval_trees[2]->insert(ivl3);

        obj_interval_maps[0][obj] = ivl1;
        obj_interval_maps[1][obj] = ivl2;
        obj_interval_maps[2][obj] = ivl3;
      }
    }

    setup_ = true;
  }
}

void IntervalTreeCollisionManager::update()
{
  setup_ = false;

  for(unsigned int i = 0, size = endpoints[0].size(); i < size; ++i)
  {
    if(endpoints[0][i].minmax == 0)
      endpoints[0][i].value = endpoints[0][i].obj->getAABB().min_[0];
    else
      endpoints[0][i].value = endpoints[0][i].obj->getAABB().max_[0];
  }

  for(unsigned int i = 0, size = endpoints[1].size(); i < size; ++i)
  {
    if(endpoints[1][i].minmax == 0)
      endpoints[1][i].value = endpoints[1][i].obj->getAABB().min_[1];
    else
      endpoints[1][i].value = endpoints[1][i].obj->getAABB().max_[1];
  }

  for(unsigned int i = 0, size = endpoints[2].size(); i < size; ++i)
  {
    if(endpoints[2][i].minmax == 0)
      endpoints[2][i].value = endpoints[2][i].obj->getAABB().min_[2];
    else
      endpoints[2][i].value = endpoints[2][i].obj->getAABB().max_[2];
  }

  setup();

}


void IntervalTreeCollisionManager::update(CollisionObject* updated_obj)
{
  AABB old_aabb;
  const AABB& new_aabb = updated_obj->getAABB();
  for(int i = 0; i < 3; ++i)
  {
    std::map<CollisionObject*, SAPInterval*>::const_iterator it = obj_interval_maps[i].find(updated_obj);
    interval_trees[i]->deleteNode(it->second);
    old_aabb.min_[i] = it->second->low;
    old_aabb.max_[i] = it->second->high;
    it->second->low = new_aabb.min_[i];
    it->second->high = new_aabb.max_[i];
    interval_trees[i]->insert(it->second);
  }

  EndPoint dummy;
  std::vector<EndPoint>::iterator it;
  for(int i = 0; i < 3; ++i)
  {
    dummy.value = old_aabb.min_[i];
    it = std::lower_bound(endpoints[i].begin(), endpoints[i].end(), dummy, boost::bind(&EndPoint::value, _1) < boost::bind(&EndPoint::value, _2));
    for(; it != endpoints[i].end(); ++it)
    {
      if(it->obj == updated_obj && it->minmax == 0)
      {
        it->value = new_aabb.min_[i];
        break;
      }
    }

    dummy.value = old_aabb.max_[i];
    it = std::lower_bound(endpoints[i].begin(), endpoints[i].end(), dummy, boost::bind(&EndPoint::value, _1) < boost::bind(&EndPoint::value, _2));
    for(; it != endpoints[i].end(); ++it)
    {
      if(it->obj == updated_obj && it->minmax == 0)
      {
        it->value = new_aabb.max_[i];
        break;
      }
    }         

    std::sort(endpoints[i].begin(), endpoints[i].end(), boost::bind(&EndPoint::value, _1) < boost::bind(&EndPoint::value, _2));
  }
}

void IntervalTreeCollisionManager::update(const std::vector<CollisionObject*>& updated_objs)
{
  for(size_t i = 0; i < updated_objs.size(); ++i)
    update(updated_objs[i]);
}

void IntervalTreeCollisionManager::clear()
{
  endpoints[0].clear();
  endpoints[1].clear();
  endpoints[2].clear();

  delete interval_trees[0]; interval_trees[0] = NULL;
  delete interval_trees[1]; interval_trees[1] = NULL;
  delete interval_trees[2]; interval_trees[2] = NULL;

  for(int i = 0; i < 3; ++i)
  {
    for(std::map<CollisionObject*, SAPInterval*>::const_iterator it = obj_interval_maps[i].begin(), end = obj_interval_maps[i].end();
        it != end; ++it)
    {
      delete it->second;
    }
  }

  for(int i = 0; i < 3; ++i)
    obj_interval_maps[i].clear();

  setup_ = false;
}

void IntervalTreeCollisionManager::getObjects(std::vector<CollisionObject*>& objs) const
{
  objs.resize(endpoints[0].size() / 2);
  unsigned int j = 0;
  for(unsigned int i = 0, size = endpoints[0].size(); i < size; ++i)
  {
    if(endpoints[0][i].minmax == 0)
    {
      objs[j] = endpoints[0][i].obj; j++;
    }
  }
}

void IntervalTreeCollisionManager::collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const
{
  if(size() == 0) return;
  collide_(obj, cdata, callback);
}

bool IntervalTreeCollisionManager::collide_(CollisionObject* obj, void* cdata, CollisionCallBack callback) const
{
  static const unsigned int CUTOFF = 100;

  std::deque<SimpleInterval*> results0, results1, results2;

  results0 = interval_trees[0]->query(obj->getAABB().min_[0], obj->getAABB().max_[0]);
  if(results0.size() > CUTOFF)
  {
    results1 = interval_trees[1]->query(obj->getAABB().min_[1], obj->getAABB().max_[1]);
    if(results1.size() > CUTOFF)
    {
      results2 = interval_trees[2]->query(obj->getAABB().min_[2], obj->getAABB().max_[2]);
      if(results2.size() > CUTOFF)
      {
        int d1 = results0.size();
        int d2 = results1.size();
        int d3 = results2.size();

        if(d1 >= d2 && d1 >= d3)
          return checkColl(results0.begin(), results0.end(), obj, cdata, callback);
        else if(d2 >= d1 && d2 >= d3)
          return checkColl(results1.begin(), results1.end(), obj, cdata, callback);
        else
          return checkColl(results2.begin(), results2.end(), obj, cdata, callback);
      }
      else
        return checkColl(results2.begin(), results2.end(), obj, cdata, callback);
    }
    else
      return checkColl(results1.begin(), results1.end(), obj, cdata, callback);
  }
  else
    return checkColl(results0.begin(), results0.end(), obj, cdata, callback);
}

void IntervalTreeCollisionManager::distance(CollisionObject* obj, void* cdata, DistanceCallBack callback) const
{
  if(size() == 0) return;
  FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
  distance_(obj, cdata, callback, min_dist);
}

bool IntervalTreeCollisionManager::distance_(CollisionObject* obj, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const
{
  static const unsigned int CUTOFF = 100;

  Vec3f delta = (obj->getAABB().max_ - obj->getAABB().min_) * 0.5;
  AABB aabb = obj->getAABB();
  if(min_dist < std::numeric_limits<FCL_REAL>::max())
  {
    Vec3f min_dist_delta(min_dist, min_dist, min_dist);
    aabb.expand(min_dist_delta);
  }

  int status = 1;
  FCL_REAL old_min_distance;
  
  while(1)
  {
    bool dist_res = false;

    old_min_distance = min_dist;

    std::deque<SimpleInterval*> results0, results1, results2;

    results0 = interval_trees[0]->query(aabb.min_[0], aabb.max_[0]);
    if(results0.size() > CUTOFF)
    {
      results1 = interval_trees[1]->query(aabb.min_[1], aabb.max_[1]);
      if(results1.size() > CUTOFF)
      {
        results2 = interval_trees[2]->query(aabb.min_[2], aabb.max_[2]);
        if(results2.size() > CUTOFF)
        {
          int d1 = results0.size();
          int d2 = results1.size();
          int d3 = results2.size();

          if(d1 >= d2 && d1 >= d3)
            dist_res = checkDist(results0.begin(), results0.end(), obj, cdata, callback, min_dist);
          else if(d2 >= d1 && d2 >= d3)
            dist_res = checkDist(results1.begin(), results1.end(), obj, cdata, callback, min_dist);
          else
            dist_res = checkDist(results2.begin(), results2.end(), obj, cdata, callback, min_dist);
        }
        else
          dist_res = checkDist(results2.begin(), results2.end(), obj, cdata, callback, min_dist);
      }
      else
        dist_res = checkDist(results1.begin(), results1.end(), obj, cdata, callback, min_dist);
    }
    else
      dist_res = checkDist(results0.begin(), results0.end(), obj, cdata, callback, min_dist);

    if(dist_res) return true;

    results0.clear();
    results1.clear();
    results2.clear();

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

void IntervalTreeCollisionManager::collide(void* cdata, CollisionCallBack callback) const
{
  if(size() == 0) return;

  std::set<CollisionObject*> active;
  std::set<std::pair<CollisionObject*, CollisionObject*> > overlap;
  unsigned int n = endpoints[0].size();
  double diff_x = endpoints[0][0].value - endpoints[0][n-1].value;
  double diff_y = endpoints[1][0].value - endpoints[1][n-1].value;
  double diff_z = endpoints[2][0].value - endpoints[2][n-1].value;

  int axis = 0;
  if(diff_y > diff_x && diff_y > diff_z)
    axis = 1;
  else if(diff_z > diff_y && diff_z > diff_x)
    axis = 2;

  for(unsigned int i = 0; i < n; ++i)
  {
    const EndPoint& endpoint = endpoints[axis][i];
    CollisionObject* index = endpoint.obj;
    if(endpoint.minmax == 0)
    {
      std::set<CollisionObject*>::iterator iter = active.begin();
      std::set<CollisionObject*>::iterator end = active.end();
      for(; iter != end; ++iter)
      {
        CollisionObject* active_index = *iter;
        const AABB& b0 = active_index->getAABB();
        const AABB& b1 = index->getAABB();

        int axis2 = (axis + 1) % 3;
        int axis3 = (axis + 2) % 3;

        if(b0.axisOverlap(b1, axis2) && b0.axisOverlap(b1, axis3))
        {
          std::pair<std::set<std::pair<CollisionObject*, CollisionObject*> >::iterator, bool> insert_res;
          if(active_index < index)
            insert_res = overlap.insert(std::make_pair(active_index, index));
          else
            insert_res = overlap.insert(std::make_pair(index, active_index));

          if(insert_res.second)
          {
            if(callback(active_index, index, cdata))
              return;
          }
        }
      }
      active.insert(index);
    }
    else
      active.erase(index);
  }

}

void IntervalTreeCollisionManager::distance(void* cdata, DistanceCallBack callback) const
{
  if(size() == 0) return;

  enable_tested_set_ = true;
  tested_set.clear();
  FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
  
  for(size_t i = 0; i < endpoints[0].size(); ++i)
    if(distance_(endpoints[0][i].obj, cdata, callback, min_dist)) break;
  
  enable_tested_set_ = false;
  tested_set.clear();
}

void IntervalTreeCollisionManager::collide(BroadPhaseCollisionManager* other_manager_, void* cdata, CollisionCallBack callback) const
{
  IntervalTreeCollisionManager* other_manager = static_cast<IntervalTreeCollisionManager*>(other_manager_);

  if((size() == 0) || (other_manager->size() == 0)) return;

  if(this == other_manager)
  {
    collide(cdata, callback);
    return;
  }

  if(this->size() < other_manager->size())
  {
    for(size_t i = 0, size = endpoints[0].size(); i < size; ++i)
      if(other_manager->collide_(endpoints[0][i].obj, cdata, callback)) return;
  }
  else
  {
    for(size_t i = 0, size = other_manager->endpoints[0].size(); i < size; ++i)
      if(collide_(other_manager->endpoints[0][i].obj, cdata, callback)) return;
  }
}

void IntervalTreeCollisionManager::distance(BroadPhaseCollisionManager* other_manager_, void* cdata, DistanceCallBack callback) const
{
  IntervalTreeCollisionManager* other_manager = static_cast<IntervalTreeCollisionManager*>(other_manager_);

  if((size() == 0) || (other_manager->size() == 0)) return;

  if(this == other_manager)
  {
    distance(cdata, callback);
    return;
  }

  FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
  
  if(this->size() < other_manager->size())
  {
    for(size_t i = 0, size = endpoints[0].size(); i < size; ++i)
      if(other_manager->distance_(endpoints[0][i].obj, cdata, callback, min_dist)) return;
  }
  else
  {
    for(size_t i = 0, size = other_manager->endpoints[0].size(); i < size; ++i)
      if(distance_(other_manager->endpoints[0][i].obj, cdata, callback, min_dist)) return;
  }
}

bool IntervalTreeCollisionManager::empty() const
{
  return endpoints[0].empty();
}

bool IntervalTreeCollisionManager::checkColl(std::deque<SimpleInterval*>::const_iterator pos_start, std::deque<SimpleInterval*>::const_iterator pos_end, CollisionObject* obj, void* cdata, CollisionCallBack callback) const
{
  while(pos_start < pos_end)
  {
    SAPInterval* ivl = static_cast<SAPInterval*>(*pos_start); 
    if(ivl->obj != obj)
    {
      if(ivl->obj->getAABB().overlap(obj->getAABB()))
      {
        if(callback(ivl->obj, obj, cdata))
          return true;
      }
    }
      
    pos_start++;
  }

  return false;
}

bool IntervalTreeCollisionManager::checkDist(std::deque<SimpleInterval*>::const_iterator pos_start, std::deque<SimpleInterval*>::const_iterator pos_end, CollisionObject* obj, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const
{
  while(pos_start < pos_end)
  {
    SAPInterval* ivl = static_cast<SAPInterval*>(*pos_start);
    if(ivl->obj != obj)
    {
      if(!enable_tested_set_)
      {
        if(ivl->obj->getAABB().distance(obj->getAABB()) < min_dist)
        {
          if(callback(ivl->obj, obj, cdata, min_dist))
            return true;
        }
      }
      else
      {
        if(!inTestedSet(ivl->obj, obj))
        {
          if(ivl->obj->getAABB().distance(obj->getAABB()) < min_dist)
          {
            if(callback(ivl->obj, obj, cdata, min_dist))
              return true;
          }

          insertTestedSet(ivl->obj, obj);
        }
      }
    }

    pos_start++;
  }

  return false;
}

void DynamicAABBTreeCollisionManager::registerObjects(const std::vector<CollisionObject*>& other_objs)
{
  if(size() > 0)
  {
    BroadPhaseCollisionManager::registerObjects(other_objs);
  }
  else
  {
    std::vector<DynamicAABBNode*> leaves(other_objs.size());
    table.rehash(other_objs.size());
    for(size_t i = 0, size = other_objs.size(); i < size; ++i)
    {
      DynamicAABBNode* node = new DynamicAABBNode; // node will be managed by the dtree
      node->bv = other_objs[i]->getAABB();
      node->parent = NULL;
      node->childs[1] = NULL;
      node->data = other_objs[i];
      table[other_objs[i]] = node;
      leaves[i] = node;
    }
   
    dtree.init(leaves, tree_init_level);
   
    setup_ = true;
  }
}

void DynamicAABBTreeCollisionManager::registerObject(CollisionObject* obj)
{
  DynamicAABBNode* node = dtree.insert(obj->getAABB(), obj);
  table[obj] = node;
}

void DynamicAABBTreeCollisionManager::unregisterObject(CollisionObject* obj)
{
  DynamicAABBNode* node = table[obj];
  table.erase(obj);
  dtree.remove(node);
}

void DynamicAABBTreeCollisionManager::setup()
{
  if(!setup_)
  {
    int num = dtree.size();
    if(num == 0) 
    {
      setup_ = true; 
      return;
    }
    
    int height = dtree.getMaxHeight();

    
    if(height - std::log((FCL_REAL)num) / std::log(2.0) < max_tree_nonbalanced_level)
      dtree.balanceIncremental(tree_incremental_balance_pass);
    else
      dtree.balanceTopdown();

    setup_ = true;
  }
}


void DynamicAABBTreeCollisionManager::update()
{ 
  for(DynamicAABBTable::const_iterator it = table.begin(); it != table.end(); ++it)
  {
    CollisionObject* obj = it->first;
    DynamicAABBNode* node = it->second;
    node->bv = obj->getAABB();
  }

  dtree.refit();
  setup_ = false;

  setup();
}

void DynamicAABBTreeCollisionManager::update_(CollisionObject* updated_obj)
{
  DynamicAABBTable::const_iterator it = table.find(updated_obj);
  if(it != table.end())
  {
    DynamicAABBNode* node = it->second;
    if(!node->bv.equal(updated_obj->getAABB()))
      dtree.update(node, updated_obj->getAABB());
  }
  setup_ = false;
}

void DynamicAABBTreeCollisionManager::update(CollisionObject* updated_obj)
{
  update_(updated_obj);
  setup();
}

void DynamicAABBTreeCollisionManager::update(const std::vector<CollisionObject*>& updated_objs)
{
  for(size_t i = 0, size = updated_objs.size(); i < size; ++i)
    update_(updated_objs[i]);
  setup();
}

bool DynamicAABBTreeCollisionManager::collisionRecurse(DynamicAABBNode* root1, DynamicAABBNode* root2, void* cdata, CollisionCallBack callback) const
{
  if(root1->isLeaf() && root2->isLeaf())
  {
    if(!root1->bv.overlap(root2->bv)) return false;
    return callback(static_cast<CollisionObject*>(root1->data), static_cast<CollisionObject*>(root2->data), cdata);
  }
    
  if(!root1->bv.overlap(root2->bv)) return false;
    
  if(root2->isLeaf() || (!root1->isLeaf() && (root1->bv.size() > root2->bv.size())))
  {
    if(collisionRecurse(root1->childs[0], root2, cdata, callback))
      return true;
    if(collisionRecurse(root1->childs[1], root2, cdata, callback))
      return true;
  }
  else
  {
    if(collisionRecurse(root1, root2->childs[0], cdata, callback))
      return true;
    if(collisionRecurse(root1, root2->childs[1], cdata, callback))
      return true;
  }
  return false;
}

bool DynamicAABBTreeCollisionManager::collisionRecurse(DynamicAABBNode* root, CollisionObject* query, void* cdata, CollisionCallBack callback) const
{
  if(root->isLeaf())
  {
    if(!root->bv.overlap(query->getAABB())) return false;
    return callback(static_cast<CollisionObject*>(root->data), query, cdata);
  }
    
  if(!root->bv.overlap(query->getAABB())) return false;

  int select_res = select(query->getAABB(), *(root->childs[0]), *(root->childs[1]));
    
  if(collisionRecurse(root->childs[select_res], query, cdata, callback))
    return true;
    
  if(collisionRecurse(root->childs[1-select_res], query, cdata, callback))
    return true;

  return false;
}

bool DynamicAABBTreeCollisionManager::selfCollisionRecurse(DynamicAABBNode* root, void* cdata, CollisionCallBack callback) const
{
  if(root->isLeaf()) return false;

  if(selfCollisionRecurse(root->childs[0], cdata, callback))
    return true;

  if(selfCollisionRecurse(root->childs[1], cdata, callback))
    return true;

  if(collisionRecurse(root->childs[0], root->childs[1], cdata, callback))
    return true;

  return false;
}

bool DynamicAABBTreeCollisionManager::distanceRecurse(DynamicAABBNode* root1, DynamicAABBNode* root2, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const
{
  if(root1->isLeaf() && root2->isLeaf())
  {
    CollisionObject* root1_obj = static_cast<CollisionObject*>(root1->data);
    CollisionObject* root2_obj = static_cast<CollisionObject*>(root2->data);
    return callback(root1_obj, root2_obj, cdata, min_dist);
  }

  if(root2->isLeaf() || (!root1->isLeaf() && (root1->bv.size() > root2->bv.size())))
  {
    FCL_REAL d1 = root2->bv.distance(root1->childs[0]->bv);
    FCL_REAL d2 = root2->bv.distance(root1->childs[1]->bv);
      
    if(d2 < d1)
    {
      if(d2 < min_dist)
      {
        if(distanceRecurse(root1->childs[1], root2, cdata, callback, min_dist))
          return true;
      }
        
      if(d1 < min_dist)
      {
        if(distanceRecurse(root1->childs[0], root2, cdata, callback, min_dist))
          return true;
      }
    }
    else
    {
      if(d1 < min_dist)
      {
        if(distanceRecurse(root1->childs[0], root2, cdata, callback, min_dist))
          return true;
      }

      if(d2 < min_dist)
      {
        if(distanceRecurse(root1->childs[1], root2, cdata, callback, min_dist))
          return true;
      }
    }
  }
  else
  {
    FCL_REAL d1 = root1->bv.distance(root2->childs[0]->bv);
    FCL_REAL d2 = root1->bv.distance(root2->childs[1]->bv);
      
    if(d2 < d1)
    {
      if(d2 < min_dist)
      {
        if(distanceRecurse(root1, root2->childs[1], cdata, callback, min_dist))
          return true;
      }
        
      if(d1 < min_dist)
      {
        if(distanceRecurse(root1, root2->childs[0], cdata, callback, min_dist))
          return true;
      }
    }
    else
    {
      if(d1 < min_dist)
      {
        if(distanceRecurse(root1, root2->childs[0], cdata, callback, min_dist))
          return true;
      }

      if(d2 < min_dist)
      {
        if(distanceRecurse(root1, root2->childs[1], cdata, callback, min_dist))
          return true;
      }
    }
  }

  return false;
}

bool DynamicAABBTreeCollisionManager::distanceRecurse(DynamicAABBNode* root, CollisionObject* query, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const
{ 
  if(root->isLeaf())
  {
    CollisionObject* root_obj = static_cast<CollisionObject*>(root->data);
    return callback(root_obj, query, cdata, min_dist); 
  }

  FCL_REAL d1 = query->getAABB().distance(root->childs[0]->bv);
  FCL_REAL d2 = query->getAABB().distance(root->childs[1]->bv);
    
  if(d2 < d1)
  {
    if(d2 < min_dist)
    {
      if(distanceRecurse(root->childs[1], query, cdata, callback, min_dist))
        return true;
    }

    if(d1 < min_dist)
    {
      if(distanceRecurse(root->childs[0], query, cdata, callback, min_dist))
        return true;
    }
  }
  else
  {
    if(d1 < min_dist)
    {
      if(distanceRecurse(root->childs[0], query, cdata, callback, min_dist))
        return true;
    }

    if(d2 < min_dist)
    {
      if(distanceRecurse(root->childs[1], query, cdata, callback, min_dist))
        return true;
    }
  }

  return false;
}

bool DynamicAABBTreeCollisionManager::selfDistanceRecurse(DynamicAABBNode* root, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const
{
  if(root->isLeaf()) return false;

  if(selfDistanceRecurse(root->childs[0], cdata, callback, min_dist))
    return true;

  if(selfDistanceRecurse(root->childs[1], cdata, callback, min_dist))
    return true;

  if(distanceRecurse(root->childs[0], root->childs[1], cdata, callback, min_dist))
    return true;

  return false;
}


bool collisionRecurse_(DynamicAABBTreeCollisionManager::DynamicAABBNode* root1, const OcTree* tree2, const OcTree::OcTreeNode* root2, const AABB& root2_bv, const SimpleTransform& tf2, void* cdata, CollisionCallBack callback)
{
  if(root1->isLeaf() && !root2->hasChildren())
  {
    if(!tree2->isNodeFree(root2))
    {
      OBB obb1, obb2;
      convertBV(root1->bv, SimpleTransform(), obb1);
      convertBV(root2_bv, tf2, obb2);
      
      if(obb1.overlap(obb2))
      {
        Box* box = new Box();
        SimpleTransform box_tf;
        constructBox(root2_bv, tf2, *box, box_tf);

        box->cost_density = root2->getOccupancy();
        box->threshold_occupied = tree2->getOccupancyThres();

        CollisionObject obj(boost::shared_ptr<CollisionGeometry>(box), box_tf);
        return callback(static_cast<CollisionObject*>(root1->data), &obj, cdata);
      }
      else return false;
    }
    else return false;
  }

  OBB obb1, obb2;
  convertBV(root1->bv, SimpleTransform(), obb1);
  convertBV(root2_bv, tf2, obb2);

  if(tree2->isNodeFree(root2) || !obb1.overlap(obb2)) return false;

  if(!root2->hasChildren() || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size())))
  {
    if(collisionRecurse_(root1->childs[0], tree2, root2, root2_bv, tf2, cdata, callback))
      return true;
    if(collisionRecurse_(root1->childs[1], tree2, root2, root2_bv, tf2, cdata, callback))
      return true;
  }
  else
  {
    for(unsigned int i = 0; i < 8; ++i)
    {
      if(root2->childExists(i))
      {
        const OcTree::OcTreeNode* child = root2->getChild(i);
        AABB child_bv;
        computeChildBV(root2_bv, i, child_bv);

        if(collisionRecurse_(root1, tree2, child, child_bv, tf2, cdata, callback))
          return true;
      }
    }
  }
  return false;
}

bool collisionRecurse_(DynamicAABBTreeCollisionManager::DynamicAABBNode* root1, const OcTree* tree2, const OcTree::OcTreeNode* root2, const AABB& root2_bv, const Vec3f& tf2, void* cdata, CollisionCallBack callback)
{
  if(root1->isLeaf() && !root2->hasChildren())
  {
    if(!tree2->isNodeFree(root2))
    {      
      const AABB& root2_bv_t = translate(root2_bv, tf2);
      if(root1->bv.overlap(root2_bv_t))
      {
        Box* box = new Box();
        SimpleTransform box_tf;
        constructBox(root2_bv, tf2, *box, box_tf);

        box->cost_density = root2->getOccupancy();
        box->threshold_occupied = tree2->getOccupancyThres();

        CollisionObject obj(boost::shared_ptr<CollisionGeometry>(box), box_tf);
        return callback(static_cast<CollisionObject*>(root1->data), &obj, cdata);
      }
      else return false;
    }
    else return false;
  }

  const AABB& root2_bv_t = translate(root2_bv, tf2);
  if(tree2->isNodeFree(root2) || !root1->bv.overlap(root2_bv_t)) return false;

  if(!root2->hasChildren() || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size())))
  {
    if(collisionRecurse_(root1->childs[0], tree2, root2, root2_bv, tf2, cdata, callback))
      return true;
    if(collisionRecurse_(root1->childs[1], tree2, root2, root2_bv, tf2, cdata, callback))
      return true;
  }
  else
  {
    for(unsigned int i = 0; i < 8; ++i)
    {
      if(root2->childExists(i))
      {
        const OcTree::OcTreeNode* child = root2->getChild(i);
        AABB child_bv;
        computeChildBV(root2_bv, i, child_bv);

        if(collisionRecurse_(root1, tree2, child, child_bv, tf2, cdata, callback))
          return true;
      }
    }
  }
  return false;
}



bool DynamicAABBTreeCollisionManager::collisionRecurse(DynamicAABBNode* root1, const OcTree* tree2, const OcTree::OcTreeNode* root2, const AABB& root2_bv, const SimpleTransform& tf2, void* cdata, CollisionCallBack callback) const
{
  if(tf2.getQuatRotation().isIdentity())
    return collisionRecurse_(root1, tree2, root2, root2_bv, tf2.getTranslation(), cdata, callback);
  else // has rotation
    return collisionRecurse_(root1, tree2, root2, root2_bv, tf2, cdata, callback);
}


bool distanceRecurse_(DynamicAABBTreeCollisionManager::DynamicAABBNode* root1, const OcTree* tree2, const OcTree::OcTreeNode* root2, const AABB& root2_bv, const Vec3f& tf2, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist)
{
  if(root1->isLeaf() && !root2->hasChildren())
  {
    if(tree2->isNodeOccupied(root2))
    {
      Box* box = new Box();
      SimpleTransform box_tf;
      constructBox(root2_bv, tf2, *box, box_tf);
      CollisionObject obj(boost::shared_ptr<CollisionGeometry>(box), box_tf);
      return callback(static_cast<CollisionObject*>(root1->data), &obj, cdata, min_dist);
    }
    else return false;
  }
  
  if(!tree2->isNodeOccupied(root2)) return false;

  if(!root2->hasChildren() || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size())))
  {
    const AABB& aabb2 = translate(root2_bv, tf2);
    FCL_REAL d1 = aabb2.distance(root1->childs[0]->bv);
    FCL_REAL d2 = aabb2.distance(root1->childs[1]->bv);

    if(d2 < d1)
    {
      if(d2 < min_dist)
      {
        if(distanceRecurse_(root1->childs[1], tree2, root2, root2_bv, tf2, cdata, callback, min_dist))
          return true;
      }

      if(d1 < min_dist)
      {
        if(distanceRecurse_(root1->childs[0], tree2, root2, root2_bv, tf2, cdata, callback, min_dist))
          return true;
      }
    }
    else
    {
      if(d1 < min_dist)
      {
        if(distanceRecurse_(root1->childs[0], tree2, root2, root2_bv, tf2, cdata, callback, min_dist))
          return true;
      }
      
      if(d2 < min_dist)
      {
        if(distanceRecurse_(root1->childs[1], tree2, root2, root2_bv, tf2, cdata, callback, min_dist))
          return true;
      }
    }
  }
  else
  {
    for(unsigned int i = 0; i < 8; ++i)
    {
      if(root2->childExists(i))
      {
        const OcTree::OcTreeNode* child = root2->getChild(i);
        AABB child_bv;
        computeChildBV(root2_bv, i, child_bv);
        const AABB& aabb2 = translate(child_bv, tf2);
   
        FCL_REAL d = root1->bv.distance(aabb2);

        if(d < min_dist)
        {
          if(distanceRecurse_(root1, tree2, child, child_bv, tf2, cdata, callback, min_dist))
            return true;
        }
      }
    }
  }

  return false;
}


bool distanceRecurse_(DynamicAABBTreeCollisionManager::DynamicAABBNode* root1, const OcTree* tree2, const OcTree::OcTreeNode* root2, const AABB& root2_bv, const SimpleTransform& tf2, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist)
{
  if(root1->isLeaf() && !root2->hasChildren())
  {
    if(tree2->isNodeOccupied(root2))
    {
      Box* box = new Box();
      SimpleTransform box_tf;
      constructBox(root2_bv, tf2, *box, box_tf);
      CollisionObject obj(boost::shared_ptr<CollisionGeometry>(box), box_tf);
      return callback(static_cast<CollisionObject*>(root1->data), &obj, cdata, min_dist);
    }
    else return false;
  }
  
  if(!tree2->isNodeOccupied(root2)) return false;

  if(!root2->hasChildren() || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size())))
  {
    AABB aabb2;
    convertBV(root2_bv, tf2, aabb2);
    
    FCL_REAL d1 = aabb2.distance(root1->childs[0]->bv);
    FCL_REAL d2 = aabb2.distance(root1->childs[1]->bv);

    if(d2 < d1)
    {
      if(d2 < min_dist)
      {
        if(distanceRecurse_(root1->childs[1], tree2, root2, root2_bv, tf2, cdata, callback, min_dist))
          return true;
      }

      if(d1 < min_dist)
      {
        if(distanceRecurse_(root1->childs[0], tree2, root2, root2_bv, tf2, cdata, callback, min_dist))
          return true;
      }
    }
    else
    {
      if(d1 < min_dist)
      {
        if(distanceRecurse_(root1->childs[0], tree2, root2, root2_bv, tf2, cdata, callback, min_dist))
          return true;
      }
      
      if(d2 < min_dist)
      {
        if(distanceRecurse_(root1->childs[1], tree2, root2, root2_bv, tf2, cdata, callback, min_dist))
          return true;
      }
    }
  }
  else
  {
    for(unsigned int i = 0; i < 8; ++i)
    {
      if(root2->childExists(i))
      {
        const OcTree::OcTreeNode* child = root2->getChild(i);
        AABB child_bv;
        computeChildBV(root2_bv, i, child_bv);

        AABB aabb2;
        convertBV(child_bv, tf2, aabb2);
        FCL_REAL d = root1->bv.distance(aabb2);

        if(d < min_dist)
        {
          if(distanceRecurse_(root1, tree2, child, child_bv, tf2, cdata, callback, min_dist))
            return true;
        }
      }
    }
  }

  return false;
}

bool DynamicAABBTreeCollisionManager::distanceRecurse(DynamicAABBNode* root1, const OcTree* tree2, const OcTree::OcTreeNode* root2, const AABB& root2_bv, const SimpleTransform& tf2, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const
{
  if(tf2.getQuatRotation().isIdentity())
    return distanceRecurse_(root1, tree2, root2, root2_bv, tf2.getTranslation(), cdata, callback, min_dist);
  else
    return distanceRecurse_(root1, tree2, root2, root2_bv, tf2, cdata, callback, min_dist);
}




void DynamicAABBTreeCollisionManager2::registerObjects(const std::vector<CollisionObject*>& other_objs)
{
  if(size() > 0)
  {
    BroadPhaseCollisionManager::registerObjects(other_objs);
  }
  else
  {
    DynamicAABBNode* leaves = new DynamicAABBNode[other_objs.size()];
    table.rehash(other_objs.size());
    for(size_t i = 0, size = other_objs.size(); i < size; ++i)
    {
      leaves[i].bv = other_objs[i]->getAABB();
      leaves[i].parent = dtree.NULL_NODE;
      leaves[i].childs[1] = dtree.NULL_NODE;
      leaves[i].data = other_objs[i];
      table[other_objs[i]] = i;
    }
   
    int n_leaves = other_objs.size();

    dtree.init(leaves, n_leaves, tree_init_level);
   
    setup_ = true;
  }
}

void DynamicAABBTreeCollisionManager2::registerObject(CollisionObject* obj)
{
  size_t node = dtree.insert(obj->getAABB(), obj);
  table[obj] = node;
  
}

void DynamicAABBTreeCollisionManager2::unregisterObject(CollisionObject* obj)
{
  size_t node = table[obj];
  table.erase(obj);
  dtree.remove(node);
}

void DynamicAABBTreeCollisionManager2::setup()
{
  if(!setup_)
  {
    int num = dtree.size();
    if(num == 0) 
    {
      setup_ = true;
      return;
    }

    int height = dtree.getMaxHeight();

    
    if(height - std::log((FCL_REAL)num) / std::log(2.0) < max_tree_nonbalanced_level)
      dtree.balanceIncremental(tree_incremental_balance_pass);
    else
      dtree.balanceTopdown();

    setup_ = true;
  }
}


void DynamicAABBTreeCollisionManager2::update()
{ 
  for(DynamicAABBTable::const_iterator it = table.begin(), end = table.end(); it != end; ++it)
  {
    CollisionObject* obj = it->first;
    size_t node = it->second;
    dtree.getNodes()[node].bv = obj->getAABB();
  }

  dtree.refit();
  setup_ = false;

  setup();
}

void DynamicAABBTreeCollisionManager2::update_(CollisionObject* updated_obj)
{
  DynamicAABBTable::const_iterator it = table.find(updated_obj);
  if(it != table.end())
  {
    size_t node = it->second;
    if(!dtree.getNodes()[node].bv.equal(updated_obj->getAABB()))
      dtree.update(node, updated_obj->getAABB());
  }
  setup_ = false;
}

void DynamicAABBTreeCollisionManager2::update(CollisionObject* updated_obj)
{
  update_(updated_obj);
  setup();
}

void DynamicAABBTreeCollisionManager2::update(const std::vector<CollisionObject*>& updated_objs)
{
  for(size_t i = 0, size = updated_objs.size(); i < size; ++i)
    update_(updated_objs[i]);
  setup();
}

bool DynamicAABBTreeCollisionManager2::collisionRecurse(DynamicAABBNode* nodes1, size_t root1_id, 
                                                        DynamicAABBNode* nodes2, size_t root2_id, 
                                                        void* cdata, CollisionCallBack callback) const
{
  DynamicAABBNode* root1 = nodes1 + root1_id;
  DynamicAABBNode* root2 = nodes2 + root2_id;
  if(root1->isLeaf() && root2->isLeaf())
  {
    if(!root1->bv.overlap(root2->bv)) return false;
    return callback(static_cast<CollisionObject*>(root1->data), static_cast<CollisionObject*>(root2->data), cdata);
  }
    
  if(!root1->bv.overlap(root2->bv)) return false;
    
  if(root2->isLeaf() || (!root1->isLeaf() && (root1->bv.size() > root2->bv.size())))
  {
    if(collisionRecurse(nodes1, root1->childs[0], nodes2, root2_id, cdata, callback))
      return true;
    if(collisionRecurse(nodes1, root1->childs[1], nodes2, root2_id, cdata, callback))
      return true;
  }
  else
  {
    if(collisionRecurse(nodes1, root1_id, nodes2, root2->childs[0], cdata, callback))
      return true;
    if(collisionRecurse(nodes1, root1_id, nodes2, root2->childs[1], cdata, callback))
      return true;
  }
  return false;
}

bool DynamicAABBTreeCollisionManager2::collisionRecurse(DynamicAABBNode* nodes, size_t root_id, CollisionObject* query, void* cdata, CollisionCallBack callback) const
{
  DynamicAABBNode* root = nodes + root_id;
  if(root->isLeaf())
  {
    if(!root->bv.overlap(query->getAABB())) return false;
    return callback(static_cast<CollisionObject*>(root->data), query, cdata);
  }
    
  if(!root->bv.overlap(query->getAABB())) return false;

  int select_res = alternative::select(query->getAABB(), root->childs[0], root->childs[1], nodes);
    
  if(collisionRecurse(nodes, root->childs[select_res], query, cdata, callback))
    return true;
    
  if(collisionRecurse(nodes, root->childs[1-select_res], query, cdata, callback))
    return true;

  return false;
}

bool DynamicAABBTreeCollisionManager2::selfCollisionRecurse(DynamicAABBNode* nodes, size_t root_id, void* cdata, CollisionCallBack callback) const
{
  DynamicAABBNode* root = nodes + root_id;
  if(root->isLeaf()) return false;

  if(selfCollisionRecurse(nodes, root->childs[0], cdata, callback))
    return true;

  if(selfCollisionRecurse(nodes, root->childs[1], cdata, callback))
    return true;

  if(collisionRecurse(nodes, root->childs[0], nodes, root->childs[1], cdata, callback))
    return true;

  return false;
}

bool DynamicAABBTreeCollisionManager2::distanceRecurse(DynamicAABBNode* nodes1, size_t root1_id, 
                                                       DynamicAABBNode* nodes2, size_t root2_id, 
                                                       void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const
{
  DynamicAABBNode* root1 = nodes1 + root1_id;
  DynamicAABBNode* root2 = nodes2 + root2_id;
  if(root1->isLeaf() && root2->isLeaf())
  {
    CollisionObject* root1_obj = static_cast<CollisionObject*>(root1->data);
    CollisionObject* root2_obj = static_cast<CollisionObject*>(root2->data);
    return callback(root1_obj, root2_obj, cdata, min_dist);
  }

  if(root2->isLeaf() || (!root1->isLeaf() && (root1->bv.size() > root2->bv.size())))
  {
    FCL_REAL d1 = root2->bv.distance((nodes1 + root1->childs[0])->bv);
    FCL_REAL d2 = root2->bv.distance((nodes1 + root1->childs[1])->bv);
      
    if(d2 < d1)
    {
      if(d2 < min_dist)
      {
        if(distanceRecurse(nodes1, root1->childs[1], nodes2, root2_id, cdata, callback, min_dist))
          return true;
      }
        
      if(d1 < min_dist)
      {
        if(distanceRecurse(nodes1, root1->childs[0], nodes2, root2_id, cdata, callback, min_dist))
          return true;
      }
    }
    else
    {
      if(d1 < min_dist)
      {
        if(distanceRecurse(nodes1, root1->childs[0], nodes2, root2_id, cdata, callback, min_dist))
          return true;
      }

      if(d2 < min_dist)
      {
        if(distanceRecurse(nodes1, root1->childs[1], nodes2, root2_id, cdata, callback, min_dist))
          return true;
      }
    }
  }
  else
  {
    FCL_REAL d1 = root1->bv.distance((nodes2 + root2->childs[0])->bv);
    FCL_REAL d2 = root1->bv.distance((nodes2 + root2->childs[1])->bv);
      
    if(d2 < d1)
    {
      if(d2 < min_dist)
      {
        if(distanceRecurse(nodes1, root1_id, nodes2, root2->childs[1], cdata, callback, min_dist))
          return true;
      }
        
      if(d1 < min_dist)
      {
        if(distanceRecurse(nodes1, root1_id, nodes2, root2->childs[0], cdata, callback, min_dist))
          return true;
      }
    }
    else
    {
      if(d1 < min_dist)
      {
        if(distanceRecurse(nodes1, root1_id, nodes2, root2->childs[0], cdata, callback, min_dist))
          return true;
      }

      if(d2 < min_dist)
      {
        if(distanceRecurse(nodes1, root1_id, nodes2, root2->childs[1], cdata, callback, min_dist))
          return true;
      }
    }
  }

  return false;
}

bool DynamicAABBTreeCollisionManager2::distanceRecurse(DynamicAABBNode* nodes, size_t root_id, CollisionObject* query, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const
{ 
  DynamicAABBNode* root = nodes + root_id;
  if(root->isLeaf())
  {
    CollisionObject* root_obj = static_cast<CollisionObject*>(root->data);
    return callback(root_obj, query, cdata, min_dist); 
  }

  FCL_REAL d1 = query->getAABB().distance((nodes + root->childs[0])->bv);
  FCL_REAL d2 = query->getAABB().distance((nodes + root->childs[1])->bv);
    
  if(d2 < d1)
  {
    if(d2 < min_dist)
    {
      if(distanceRecurse(nodes, root->childs[1], query, cdata, callback, min_dist))
        return true;
    }

    if(d1 < min_dist)
    {
      if(distanceRecurse(nodes, root->childs[0], query, cdata, callback, min_dist))
        return true;
    }
  }
  else
  {
    if(d1 < min_dist)
    {
      if(distanceRecurse(nodes, root->childs[0], query, cdata, callback, min_dist))
        return true;
    }

    if(d2 < min_dist)
    {
      if(distanceRecurse(nodes, root->childs[1], query, cdata, callback, min_dist))
        return true;
    }
  }

  return false;
}

bool DynamicAABBTreeCollisionManager2::selfDistanceRecurse(DynamicAABBNode* nodes, size_t root_id, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const
{
  DynamicAABBNode* root = nodes + root_id;
  if(root->isLeaf()) return false;

  if(selfDistanceRecurse(nodes, root->childs[0], cdata, callback, min_dist))
    return true;

  if(selfDistanceRecurse(nodes, root->childs[1], cdata, callback, min_dist))
    return true;

  if(distanceRecurse(nodes, root->childs[0], nodes, root->childs[1], cdata, callback, min_dist))
    return true;

  return false;
}

bool collisionRecurse_(DynamicAABBTreeCollisionManager2::DynamicAABBNode* nodes1, size_t root1_id, const OcTree* tree2, const OcTree::OcTreeNode* root2, const AABB& root2_bv, const SimpleTransform& tf2, void* cdata, CollisionCallBack callback)
{
  DynamicAABBTreeCollisionManager2::DynamicAABBNode* root1 = nodes1 + root1_id;
  if(root1->isLeaf() && !root2->hasChildren())
  {
    if(!tree2->isNodeFree(root2))
    {
      OBB obb1, obb2;
      convertBV(root1->bv, SimpleTransform(), obb1);
      convertBV(root2_bv, tf2, obb2);
      
      if(obb1.overlap(obb2))
      {
        Box* box = new Box();
        SimpleTransform box_tf;
        constructBox(root2_bv, tf2, *box, box_tf);
        
        box->cost_density = root2->getOccupancy();
        box->threshold_occupied = tree2->getOccupancyThres();

        CollisionObject obj(boost::shared_ptr<CollisionGeometry>(box), box_tf);
        return callback(static_cast<CollisionObject*>(root1->data), &obj, cdata);
      }
      else return false;
    }
    else return false;
  }

  OBB obb1, obb2;
  convertBV(root1->bv, SimpleTransform(), obb1);
  convertBV(root2_bv, tf2, obb2);
  
  if(tree2->isNodeFree(root2) || !obb1.overlap(obb2)) return false;

  if(!root2->hasChildren() || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size())))
  {
    if(collisionRecurse_(nodes1, root1->childs[0], tree2, root2, root2_bv, tf2, cdata, callback))
      return true;
    if(collisionRecurse_(nodes1, root1->childs[1], tree2, root2, root2_bv, tf2, cdata, callback))
      return true;
  }
  else
  {
    for(unsigned int i = 0; i < 8; ++i)
    {
      if(root2->childExists(i))
      {
        const OcTree::OcTreeNode* child = root2->getChild(i);
        AABB child_bv;
        computeChildBV(root2_bv, i, child_bv);

        if(collisionRecurse_(nodes1, root1_id, tree2, child, child_bv, tf2, cdata, callback))
          return true;
      }
    }
  }

  return false;
}

bool collisionRecurse_(DynamicAABBTreeCollisionManager2::DynamicAABBNode* nodes1, size_t root1_id, const OcTree* tree2, const OcTree::OcTreeNode* root2, const AABB& root2_bv, const Vec3f& tf2, void* cdata, CollisionCallBack callback)
{
  DynamicAABBTreeCollisionManager2::DynamicAABBNode* root1 = nodes1 + root1_id;
  if(root1->isLeaf() && !root2->hasChildren())
  {
    if(!tree2->isNodeFree(root2))
    {
      const AABB& root_bv_t = translate(root2_bv, tf2);
      if(root1->bv.overlap(root_bv_t))
      {
        Box* box = new Box();
        SimpleTransform box_tf;
        constructBox(root2_bv, tf2, *box, box_tf);

        box->cost_density = root2->getOccupancy();
        box->threshold_occupied = tree2->getOccupancyThres();

        CollisionObject obj(boost::shared_ptr<CollisionGeometry>(box), box_tf);
        return callback(static_cast<CollisionObject*>(root1->data), &obj, cdata);
      }
      else return false;
    }
    else return false;
  }

  const AABB& root_bv_t = translate(root2_bv, tf2);
  if(tree2->isNodeFree(root2) || !root1->bv.overlap(root_bv_t)) return false;

  if(!root2->hasChildren() || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size())))
  {
    if(collisionRecurse_(nodes1, root1->childs[0], tree2, root2, root2_bv, tf2, cdata, callback))
      return true;
    if(collisionRecurse_(nodes1, root1->childs[1], tree2, root2, root2_bv, tf2, cdata, callback))
      return true;
  }
  else
  {
    for(unsigned int i = 0; i < 8; ++i)
    {
      if(root2->childExists(i))
      {
        const OcTree::OcTreeNode* child = root2->getChild(i);
        AABB child_bv;
        computeChildBV(root2_bv, i, child_bv);

        if(collisionRecurse_(nodes1, root1_id, tree2, child, child_bv, tf2, cdata, callback))
          return true;
      }
    }
  }

  return false;
}



bool DynamicAABBTreeCollisionManager2::collisionRecurse(DynamicAABBNode* nodes1, size_t root1_id, const OcTree* tree2, const OcTree::OcTreeNode* root2, const AABB& root2_bv, const SimpleTransform& tf2, void* cdata, CollisionCallBack callback) const
{
  if(tf2.getQuatRotation().isIdentity())
    return collisionRecurse_(nodes1, root1_id, tree2, root2, root2_bv, tf2.getTranslation(), cdata, callback);
  else
    return collisionRecurse_(nodes1, root1_id, tree2, root2, root2_bv, tf2, cdata, callback);
}


bool distanceRecurse_(DynamicAABBTreeCollisionManager2::DynamicAABBNode* nodes1, size_t root1_id, const OcTree* tree2, const OcTree::OcTreeNode* root2, const AABB& root2_bv, const SimpleTransform& tf2, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist)
{
  DynamicAABBTreeCollisionManager2::DynamicAABBNode* root1 = nodes1 + root1_id;
  if(root1->isLeaf() && !root2->hasChildren())
  {
    if(tree2->isNodeOccupied(root2))
    {
      Box* box = new Box();
      SimpleTransform box_tf;
      constructBox(root2_bv, tf2, *box, box_tf);
      CollisionObject obj(boost::shared_ptr<CollisionGeometry>(box), box_tf);
      return callback(static_cast<CollisionObject*>(root1->data), &obj, cdata, min_dist);
    }
    else return false;
  }

  if(!tree2->isNodeOccupied(root2)) return false;

  if(!root2->hasChildren() || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size())))
  {
    AABB aabb2;
    convertBV(root2_bv, tf2, aabb2);

    FCL_REAL d1 = aabb2.distance((nodes1 + root1->childs[0])->bv);
    FCL_REAL d2 = aabb2.distance((nodes1 + root1->childs[1])->bv);
      
    if(d2 < d1)
    {
      if(d2 < min_dist)
      {
        if(distanceRecurse_(nodes1, root1->childs[1], tree2, root2, root2_bv, tf2, cdata, callback, min_dist))
          return true;
      }
        
      if(d1 < min_dist)
      {
        if(distanceRecurse_(nodes1, root1->childs[0], tree2, root2, root2_bv, tf2, cdata, callback, min_dist))
          return true;
      }
    }
    else
    {
      if(d1 < min_dist)
      {
        if(distanceRecurse_(nodes1, root1->childs[0], tree2, root2, root2_bv, tf2, cdata, callback, min_dist))
          return true;
      }

      if(d2 < min_dist)
      {
        if(distanceRecurse_(nodes1, root1->childs[1], tree2, root2, root2_bv, tf2, cdata, callback, min_dist))
          return true;
      }
    }
  }
  else
  {
    for(unsigned int i = 0; i < 8; ++i)
    {
      if(root2->childExists(i))
      {
        const OcTree::OcTreeNode* child = root2->getChild(i);
        AABB child_bv;
        computeChildBV(root2_bv, i, child_bv);

        AABB aabb2;
        convertBV(child_bv, tf2, aabb2);
        FCL_REAL d = root1->bv.distance(aabb2);
        
        if(d < min_dist)
        {
          if(distanceRecurse_(nodes1, root1_id, tree2, child, child_bv, tf2, cdata, callback, min_dist))
            return true;
        }
      }
    }
  }
  
  return false;
}


bool distanceRecurse_(DynamicAABBTreeCollisionManager2::DynamicAABBNode* nodes1, size_t root1_id, const OcTree* tree2, const OcTree::OcTreeNode* root2, const AABB& root2_bv, const Vec3f& tf2, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist)
{
  DynamicAABBTreeCollisionManager2::DynamicAABBNode* root1 = nodes1 + root1_id;
  if(root1->isLeaf() && !root2->hasChildren())
  {
    if(tree2->isNodeOccupied(root2))
    {
      Box* box = new Box();
      SimpleTransform box_tf;
      constructBox(root2_bv, tf2, *box, box_tf);
      CollisionObject obj(boost::shared_ptr<CollisionGeometry>(box), box_tf);
      return callback(static_cast<CollisionObject*>(root1->data), &obj, cdata, min_dist);
    }
    else return false;
  }

  if(!tree2->isNodeOccupied(root2)) return false;

  if(!root2->hasChildren() || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size())))
  {
    const AABB& aabb2 = translate(root2_bv, tf2);

    FCL_REAL d1 = aabb2.distance((nodes1 + root1->childs[0])->bv);
    FCL_REAL d2 = aabb2.distance((nodes1 + root1->childs[1])->bv);
      
    if(d2 < d1)
    {
      if(d2 < min_dist)
      {
        if(distanceRecurse_(nodes1, root1->childs[1], tree2, root2, root2_bv, tf2, cdata, callback, min_dist))
          return true;
      }
        
      if(d1 < min_dist)
      {
        if(distanceRecurse_(nodes1, root1->childs[0], tree2, root2, root2_bv, tf2, cdata, callback, min_dist))
          return true;
      }
    }
    else
    {
      if(d1 < min_dist)
      {
        if(distanceRecurse_(nodes1, root1->childs[0], tree2, root2, root2_bv, tf2, cdata, callback, min_dist))
          return true;
      }

      if(d2 < min_dist)
      {
        if(distanceRecurse_(nodes1, root1->childs[1], tree2, root2, root2_bv, tf2, cdata, callback, min_dist))
          return true;
      }
    }
  }
  else
  {
    for(unsigned int i = 0; i < 8; ++i)
    {
      if(root2->childExists(i))
      {
        const OcTree::OcTreeNode* child = root2->getChild(i);
        AABB child_bv;
        computeChildBV(root2_bv, i, child_bv);

        const AABB& aabb2 = translate(child_bv, tf2);
        FCL_REAL d = root1->bv.distance(aabb2);
        
        if(d < min_dist)
        {
          if(distanceRecurse_(nodes1, root1_id, tree2, child, child_bv, tf2, cdata, callback, min_dist))
            return true;
        }
      }
    }
  }
  
  return false;
}

bool DynamicAABBTreeCollisionManager2::distanceRecurse(DynamicAABBNode* nodes1, size_t root1_id, const OcTree* tree2, const OcTree::OcTreeNode* root2, const AABB& root2_bv, const SimpleTransform& tf2, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const
{
  if(tf2.getQuatRotation().isIdentity())
    return distanceRecurse_(nodes1, root1_id, tree2, root2, root2_bv, tf2.getTranslation(), cdata, callback, min_dist);
  else
    return distanceRecurse_(nodes1, root1_id, tree2, root2, root2_bv, tf2, cdata, callback, min_dist);
}

}

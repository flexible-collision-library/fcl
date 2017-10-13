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

#ifndef FCL_BROAD_PHASE_SAP_INL_H
#define FCL_BROAD_PHASE_SAP_INL_H

#include "fcl/broadphase/broadphase_SaP.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT SaPCollisionManager<double>;

//==============================================================================
template <typename S>
void SaPCollisionManager<S>::unregisterObject(CollisionObject<S>* obj)
{
  auto it = AABB_arr.begin();
  for(auto end = AABB_arr.end(); it != end; ++it)
  {
    if((*it)->obj == obj)
      break;
  }

  AABB_arr.erase(it);
  obj_aabb_map.erase(obj);

  if(it == AABB_arr.end())
    return;

  SaPAABB* curr = *it;
  *it = nullptr;

  for(int coord = 0; coord < 3; ++coord)
  {
    //first delete the lo endpoint of the interval.
    if(curr->lo->prev[coord] == nullptr)
      elist[coord] = curr->lo->next[coord];
    else
      curr->lo->prev[coord]->next[coord] = curr->lo->next[coord];

    curr->lo->next[coord]->prev[coord] = curr->lo->prev[coord];

    //then, delete the "hi" endpoint.
    if(curr->hi->prev[coord] == nullptr)
      elist[coord] = curr->hi->next[coord];
    else
      curr->hi->prev[coord]->next[coord] = curr->hi->next[coord];

    if(curr->hi->next[coord] != nullptr)
      curr->hi->next[coord]->prev[coord] = curr->hi->prev[coord];
  }

  delete curr->lo;
  delete curr->hi;
  delete curr;

  overlap_pairs.remove_if(isUnregistered(obj));
}
\
//==============================================================================
template <typename S>
SaPCollisionManager<S>::SaPCollisionManager()
{
  elist[0] = nullptr;
  elist[1] = nullptr;
  elist[2] = nullptr;

  optimal_axis = 0;
}

//==============================================================================
template <typename S>
SaPCollisionManager<S>::~SaPCollisionManager()
{
  clear();
}

//==============================================================================
template <typename S>
void SaPCollisionManager<S>::registerObjects(const std::vector<CollisionObject<S>*>& other_objs)
{
  if(other_objs.empty()) return;

  if(size() > 0)
    BroadPhaseCollisionManager<S>::registerObjects(other_objs);
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


    S scale[3];
    for(size_t coord = 0; coord < 3; ++coord)
    {
      std::sort(endpoints.begin(), endpoints.end(),
                std::bind(std::less<S>(),
                            std::bind(static_cast<S (EndPoint::*)(size_t) const >(&EndPoint::getVal), std::placeholders::_1, coord),
                            std::bind(static_cast<S (EndPoint::*)(size_t) const >(&EndPoint::getVal), std::placeholders::_2, coord)));

      endpoints[0]->prev[coord] = nullptr;
      endpoints[0]->next[coord] = endpoints[1];
      for(size_t i = 1; i < endpoints.size() - 1; ++i)
      {
        endpoints[i]->prev[coord] = endpoints[i-1];
        endpoints[i]->next[coord] = endpoints[i+1];
      }
      endpoints[endpoints.size() - 1]->prev[coord] = endpoints[endpoints.size() - 2];
      endpoints[endpoints.size() - 1]->next[coord] = nullptr;

      elist[coord] = endpoints[0];

      scale[coord] = endpoints.back()->aabb->cached.max_[coord] - endpoints[0]->aabb->cached.min_[coord];
    }

    int axis = 0;
    if(scale[axis] < scale[1]) axis = 1;
    if(scale[axis] < scale[2]) axis = 2;

    EndPoint* pos = elist[axis];

    while(pos != nullptr)
    {
      EndPoint* pos_next = nullptr;
      SaPAABB* aabb = pos->aabb;
      EndPoint* pos_it = pos->next[axis];

      while(pos_it != nullptr)
      {
        if(pos_it->aabb == aabb)
        {
          if(pos_next == nullptr) pos_next = pos_it;
          break;
        }

        if(pos_it->minmax == 0)
        {
          if(pos_next == nullptr) pos_next = pos_it;
          if(pos_it->aabb->cached.overlap(aabb->cached))
            overlap_pairs.emplace_back(pos_it->aabb->obj, aabb->obj);
        }
        pos_it = pos_it->next[axis];
      }

      pos = pos_next;
    }
  }

  updateVelist();
}

//==============================================================================
template <typename S>
void SaPCollisionManager<S>::registerObject(CollisionObject<S>* obj)
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
    if(current == nullptr) // empty list
    {
      elist[coord] = curr->lo;
      curr->lo->prev[coord] = curr->lo->next[coord] = nullptr;
    }
    else // otherwise, find the correct location in the list and insert
    {
      EndPoint* curr_lo = curr->lo;
      S curr_lo_val = curr_lo->getVal()[coord];
      while((current->getVal()[coord] < curr_lo_val) && (current->next[coord] != nullptr))
        current = current->next[coord];

      if(current->getVal()[coord] >= curr_lo_val)
      {
        curr_lo->prev[coord] = current->prev[coord];
        curr_lo->next[coord] = current;
        if(current->prev[coord] == nullptr)
          elist[coord] = curr_lo;
        else
          current->prev[coord]->next[coord] = curr_lo;

        current->prev[coord] = curr_lo;
      }
      else
      {
        curr_lo->prev[coord] = current;
        curr_lo->next[coord] = nullptr;
        current->next[coord] = curr_lo;
      }
    }

    // now insert hi end point
    current = curr->lo;

    EndPoint* curr_hi = curr->hi;
    S curr_hi_val = curr_hi->getVal()[coord];

    if(coord == 0)
    {
      while((current->getVal()[coord] < curr_hi_val) && (current->next[coord] != nullptr))
      {
        if(current != curr->lo)
          if(current->aabb->cached.overlap(curr->cached))
            overlap_pairs.emplace_back(current->aabb->obj, obj);

        current = current->next[coord];
      }
    }
    else
    {
      while((current->getVal()[coord] < curr_hi_val) && (current->next[coord] != nullptr))
        current = current->next[coord];
    }

    if(current->getVal()[coord] >= curr_hi_val)
    {
      curr_hi->prev[coord] = current->prev[coord];
      curr_hi->next[coord] = current;
      if(current->prev[coord] == nullptr)
        elist[coord] = curr_hi;
      else
        current->prev[coord]->next[coord] = curr_hi;

      current->prev[coord] = curr_hi;
    }
    else
    {
      curr_hi->prev[coord] = current;
      curr_hi->next[coord] = nullptr;
      current->next[coord] = curr_hi;
    }
  }

  AABB_arr.push_back(curr);

  obj_aabb_map[obj] = curr;

  updateVelist();
}

//==============================================================================
template <typename S>
void SaPCollisionManager<S>::setup()
{
  if(size() == 0) return;

  S scale[3];
  scale[0] = (velist[0].back())->getVal(0) - velist[0][0]->getVal(0);
  scale[1] = (velist[1].back())->getVal(1) - velist[1][0]->getVal(1);;
  scale[2] = (velist[2].back())->getVal(2) - velist[2][0]->getVal(2);
  size_t axis = 0;
  if(scale[axis] < scale[1]) axis = 1;
  if(scale[axis] < scale[2]) axis = 2;
  optimal_axis = axis;
}

//==============================================================================
template <typename S>
void SaPCollisionManager<S>::update_(SaPAABB* updated_aabb)
{
  if(updated_aabb->cached.equal(updated_aabb->obj->getAABB()))
    return;

  SaPAABB* current = updated_aabb;

  Vector3<S> new_min = current->obj->getAABB().min_;
  Vector3<S> new_max = current->obj->getAABB().max_;

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
      if(current->lo->prev[coord] != nullptr)
      {
        temp = current->lo;
        while((temp != nullptr) && (temp->getVal(coord) > new_min[coord]))
        {
          if(temp->minmax == 1)
            if(temp->aabb->cached.overlap(dummy.cached))
              addToOverlapPairs(SaPPair(temp->aabb->obj, current->obj));
          temp = temp->prev[coord];
        }

        if(temp == nullptr)
        {
          current->lo->prev[coord]->next[coord] = current->lo->next[coord];
          current->lo->next[coord]->prev[coord] = current->lo->prev[coord];
          current->lo->prev[coord] = nullptr;
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
      if(current->hi->next[coord] != nullptr)
        current->hi->next[coord]->prev[coord] = current->hi->prev[coord];
      current->hi->prev[coord] = temp;
      current->hi->next[coord] = temp->next[coord];
      if(temp->next[coord] != nullptr)
        temp->next[coord]->prev[coord] = current->hi;
      temp->next[coord] = current->hi;

      current->hi->getVal(coord) = new_max[coord];
    }
    else if(direction == 1)
    {
      //here, we first update the "hi" endpoint.
      if(current->hi->next[coord] != nullptr)
      {
        temp = current->hi;
        while((temp->next[coord] != nullptr) && (temp->getVal(coord) < new_max[coord]))
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
          current->hi->next[coord] = nullptr;
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

      if(current->lo->prev[coord] != nullptr)
        current->lo->prev[coord]->next[coord] = current->lo->next[coord];
      else
        elist[coord] = current->lo->next[coord];
      current->lo->next[coord]->prev[coord] = current->lo->prev[coord];
      current->lo->prev[coord] = temp->prev[coord];
      current->lo->next[coord] = temp;
      if(temp->prev[coord] != nullptr)
        temp->prev[coord]->next[coord] = current->lo;
      else
        elist[coord] = current->lo;
      temp->prev[coord] = current->lo;
      current->lo->getVal(coord) = new_min[coord];
    }
  }
}

//==============================================================================
template <typename S>
void SaPCollisionManager<S>::updateVelist()
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

//==============================================================================
template <typename S>
void SaPCollisionManager<S>::update(CollisionObject<S>* updated_obj)
{
  update_(obj_aabb_map[updated_obj]);

  updateVelist();

  setup();
}

//==============================================================================
template <typename S>
void SaPCollisionManager<S>::update(const std::vector<CollisionObject<S>*>& updated_objs)
{
  for(size_t i = 0; i < updated_objs.size(); ++i)
    update_(obj_aabb_map[updated_objs[i]]);

  updateVelist();

  setup();
}

//==============================================================================
template <typename S>
void SaPCollisionManager<S>::update()
{
  for(auto it = AABB_arr.cbegin(), end = AABB_arr.cend(); it != end; ++it)
  {
    update_(*it);
  }

  updateVelist();

  setup();
}

//==============================================================================
template <typename S>
void SaPCollisionManager<S>::clear()
{
  for(auto it = AABB_arr.begin(), end = AABB_arr.end(); it != end; ++it)
  {
    delete (*it)->hi;
    delete (*it)->lo;
    delete *it;
    *it = nullptr;
  }

  AABB_arr.clear();
  overlap_pairs.clear();

  elist[0] = nullptr;
  elist[1] = nullptr;
  elist[2] = nullptr;

  velist[0].clear();
  velist[1].clear();
  velist[2].clear();

  obj_aabb_map.clear();
}

//==============================================================================
template <typename S>
void SaPCollisionManager<S>::getObjects(std::vector<CollisionObject<S>*>& objs) const
{
  objs.resize(AABB_arr.size());
  int i = 0;
  for(auto it = AABB_arr.cbegin(), end = AABB_arr.cend(); it != end; ++it, ++i)
  {
    objs[i] = (*it)->obj;
  }
}

//==============================================================================
template <typename S>
bool SaPCollisionManager<S>::collide_(CollisionObject<S>* obj, void* cdata, CollisionCallBack<S> callback) const
{
  size_t axis = optimal_axis;
  const AABB<S>& obj_aabb = obj->getAABB();

  S min_val = obj_aabb.min_[axis];
  //  S max_val = obj_aabb.max_[axis];

  EndPoint dummy;
  SaPAABB dummy_aabb;
  dummy_aabb.cached = obj_aabb;
  dummy.minmax = 1;
  dummy.aabb = &dummy_aabb;

  // compute stop_pos by binary search, this is cheaper than check it in while iteration linearly
  const auto res_it = std::upper_bound(velist[axis].begin(), velist[axis].end(), &dummy,
                                                                   std::bind(std::less<S>(),
                                                                               std::bind(static_cast<S (EndPoint::*)(size_t) const>(&EndPoint::getVal), std::placeholders::_1, axis),
                                                                               std::bind(static_cast<S (EndPoint::*)(size_t) const>(&EndPoint::getVal), std::placeholders::_2, axis)));

  EndPoint* end_pos = nullptr;
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

//==============================================================================
template <typename S>
void SaPCollisionManager<S>::addToOverlapPairs(
    const typename SaPCollisionManager<S>::SaPPair& p)
{
  bool repeated = false;
  for(auto it = overlap_pairs.begin(), end = overlap_pairs.end(); it != end; ++it)
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

//==============================================================================
template <typename S>
void SaPCollisionManager<S>::removeFromOverlapPairs(
    const typename SaPCollisionManager<S>::SaPPair& p)
{
  for(auto it = overlap_pairs.begin(), end = overlap_pairs.end();
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

//==============================================================================
template <typename S>
void SaPCollisionManager<S>::collide(CollisionObject<S>* obj, void* cdata, CollisionCallBack<S> callback) const
{
  if(size() == 0) return;

  collide_(obj, cdata, callback);
}

//==============================================================================
template <typename S>
bool SaPCollisionManager<S>::distance_(CollisionObject<S>* obj, void* cdata, DistanceCallBack<S> callback, S& min_dist) const
{
  Vector3<S> delta = (obj->getAABB().max_ - obj->getAABB().min_) * 0.5;
  AABB<S> aabb = obj->getAABB();

  if(min_dist < std::numeric_limits<S>::max())
  {
    Vector3<S> min_dist_delta(min_dist, min_dist, min_dist);
    aabb.expand(min_dist_delta);
  }

  size_t axis = optimal_axis;

  int status = 1;
  S old_min_distance;

  EndPoint* start_pos = elist[axis];

  while(1)
  {
    old_min_distance = min_dist;
    S min_val = aabb.min_[axis];
    //    S max_val = aabb.max_[axis];

    EndPoint dummy;
    SaPAABB dummy_aabb;
    dummy_aabb.cached = aabb;
    dummy.minmax = 1;
    dummy.aabb = &dummy_aabb;


    const auto res_it = std::upper_bound(velist[axis].begin(), velist[axis].end(), &dummy,
                                                                     std::bind(std::less<S>(),
                                                                                 std::bind(static_cast<S (EndPoint::*)(size_t) const>(&EndPoint::getVal), std::placeholders::_1, axis),
                                                                                 std::bind(static_cast<S (EndPoint::*)(size_t) const>(&EndPoint::getVal), std::placeholders::_2, axis)));

    EndPoint* end_pos = nullptr;
    if(res_it != velist[axis].end())
      end_pos = *res_it;

    EndPoint* pos = start_pos;

    while(pos != end_pos)
    {
      // can change to pos->aabb->hi->getVal(axis) >= min_val - min_dist, and then update start_pos to end_pos.
      // but this seems slower.
      if((pos->minmax == 0) && (pos->aabb->hi->getVal(axis) >= min_val))
      {
        CollisionObject<S>* curr_obj = pos->aabb->obj;
        if(curr_obj != obj)
        {
          if(!this->enable_tested_set_)
          {
            if(pos->aabb->cached.distance(obj->getAABB()) < min_dist)
            {
              if(callback(curr_obj, obj, cdata, min_dist))
                return true;
            }
          }
          else
          {
            if(!this->inTestedSet(curr_obj, obj))
            {
              if(pos->aabb->cached.distance(obj->getAABB()) < min_dist)
              {
                if(callback(curr_obj, obj, cdata, min_dist))
                  return true;
              }

              this->insertTestedSet(curr_obj, obj);
            }
          }
        }
      }

      pos = pos->next[axis];
    }

    if(status == 1)
    {
      if(old_min_distance < std::numeric_limits<S>::max())
        break;
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
      break;
  }

  return false;
}

//==============================================================================
template <typename S>
void SaPCollisionManager<S>::distance(CollisionObject<S>* obj, void* cdata, DistanceCallBack<S> callback) const
{
  if(size() == 0) return;

  S min_dist = std::numeric_limits<S>::max();

  distance_(obj, cdata, callback, min_dist);
}

//==============================================================================
template <typename S>
void SaPCollisionManager<S>::collide(void* cdata, CollisionCallBack<S> callback) const
{
  if(size() == 0) return;

  for(auto it = overlap_pairs.cbegin(), end = overlap_pairs.cend(); it != end; ++it)
  {
    CollisionObject<S>* obj1 = it->obj1;
    CollisionObject<S>* obj2 = it->obj2;

    if(callback(obj1, obj2, cdata))
      return;
  }
}

//==============================================================================
template <typename S>
void SaPCollisionManager<S>::distance(void* cdata, DistanceCallBack<S> callback) const
{
  if(size() == 0) return;

  this->enable_tested_set_ = true;
  this->tested_set.clear();

  S min_dist = std::numeric_limits<S>::max();

  for(auto it = AABB_arr.cbegin(), end = AABB_arr.cend(); it != end; ++it)
  {
    if(distance_((*it)->obj, cdata, callback, min_dist))
      break;
  }

  this->enable_tested_set_ = false;
  this->tested_set.clear();
}

//==============================================================================
template <typename S>
void SaPCollisionManager<S>::collide(BroadPhaseCollisionManager<S>* other_manager_, void* cdata, CollisionCallBack<S> callback) const
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
    for(auto it = AABB_arr.cbegin(); it != AABB_arr.cend(); ++it)
    {
      if(other_manager->collide_((*it)->obj, cdata, callback))
        return;
    }
  }
  else
  {
    for(auto it = other_manager->AABB_arr.cbegin(), end = other_manager->AABB_arr.cend(); it != end; ++it)
    {
      if(collide_((*it)->obj, cdata, callback))
        return;
    }
  }
}

//==============================================================================
template <typename S>
void SaPCollisionManager<S>::distance(BroadPhaseCollisionManager<S>* other_manager_, void* cdata, DistanceCallBack<S> callback) const
{
  SaPCollisionManager* other_manager = static_cast<SaPCollisionManager*>(other_manager_);

  if((size() == 0) || (other_manager->size() == 0)) return;

  if(this == other_manager)
  {
    distance(cdata, callback);
    return;
  }

  S min_dist = std::numeric_limits<S>::max();

  if(this->size() < other_manager->size())
  {
    for(auto it = AABB_arr.cbegin(), end = AABB_arr.cend(); it != end; ++it)
    {
      if(other_manager->distance_((*it)->obj, cdata, callback, min_dist))
        return;
    }
  }
  else
  {
    for(auto it = other_manager->AABB_arr.cbegin(), end = other_manager->AABB_arr.cend(); it != end; ++it)
    {
      if(distance_((*it)->obj, cdata, callback, min_dist))
        return;
    }
  }
}

//==============================================================================
template <typename S>
bool SaPCollisionManager<S>::empty() const
{
  return AABB_arr.size();
}

//==============================================================================
template <typename S>
size_t SaPCollisionManager<S>::size() const
{
  return AABB_arr.size();
}

//==============================================================================
template <typename S>
const Vector3<S>&SaPCollisionManager<S>::EndPoint::getVal() const
{
  if(minmax) return aabb->cached.max_;
  else return aabb->cached.min_;
}

//==============================================================================
template <typename S>
Vector3<S>&SaPCollisionManager<S>::EndPoint::getVal()
{
  if(minmax) return aabb->cached.max_;
  else return aabb->cached.min_;
}

//==============================================================================
template <typename S>
S SaPCollisionManager<S>::EndPoint::getVal(size_t i) const
{
  if(minmax)
    return aabb->cached.max_[i];
  else
    return aabb->cached.min_[i];
}

//==============================================================================
template <typename S>
S& SaPCollisionManager<S>::EndPoint::getVal(size_t i)
{
  if(minmax)
    return aabb->cached.max_[i];
  else
    return aabb->cached.min_[i];
}

//==============================================================================
template <typename S>
SaPCollisionManager<S>::SaPPair::SaPPair(CollisionObject<S>* a, CollisionObject<S>* b)
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

//==============================================================================
template <typename S>
bool SaPCollisionManager<S>::SaPPair::operator ==(const typename SaPCollisionManager<S>::SaPPair& other) const
{
  return ((obj1 == other.obj1) && (obj2 == other.obj2));
}

//==============================================================================
template <typename S>
SaPCollisionManager<S>::isUnregistered::isUnregistered(CollisionObject<S>* obj_) : obj(obj_)
{}

//==============================================================================
template <typename S>
bool SaPCollisionManager<S>::isUnregistered::operator()(const SaPPair& pair) const
{
  return (pair.obj1 == obj) || (pair.obj2 == obj);
}

//==============================================================================
template <typename S>
SaPCollisionManager<S>::isNotValidPair::isNotValidPair(CollisionObject<S>* obj1_, CollisionObject<S>* obj2_) : obj1(obj1_),
  obj2(obj2_)
{
  // Do nothing
}

//==============================================================================
template <typename S>
bool SaPCollisionManager<S>::isNotValidPair::operator()(const SaPPair& pair)
{
  return (pair.obj1 == obj1) && (pair.obj2 == obj2);
}

} // namespace fcl

#endif

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
#include <algorithm>
#include <set>
#include <deque>

#include <iostream>

namespace fcl
{


bool defaultCollisionFunction(CollisionObject* o1, CollisionObject* o2, void* cdata_)
{
  CollisionData* cdata = (CollisionData*)cdata_;

  if(cdata->done) return true;

  std::vector<Contact> contacts;
  int num_contacts = collide(o1, o2, 1, false, false, contacts);

  cdata->is_collision = (num_contacts > 0);
  for(int i = 0; i < num_contacts; ++i)
  {
    cdata->contacts.push_back(contacts[i]);
  }

  // set done flag
  if( (!cdata->exhaustive) && (cdata->is_collision) && (cdata->contacts.size() >= cdata->num_max_contacts))
    cdata->done = true;

  return cdata->done;
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
  std::list<CollisionObject*>::const_iterator it;
  int i = 0;
  for(it = objs.begin(); it != objs.end(); ++it, ++i)
    objs_[i] = *it;
}

void NaiveCollisionManager::collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const
{
  std::list<CollisionObject*>::const_iterator it;
  for(it = objs.begin(); it != objs.end(); ++it)
  {
    if(callback(obj, *it, cdata))
      return;
  }
}

void NaiveCollisionManager::collide(void* cdata, CollisionCallBack callback) const
{
  std::list<CollisionObject*>::const_iterator it1;
  std::list<CollisionObject*>::const_iterator it2;
  for(it1 = objs.begin(); it1 != objs.end(); ++it1)
  {
    it2 = it1; it2++;
    for(; it2 != objs.end(); ++it2)
    {
      if(callback(*it1, *it2, cdata))
        return;
    }
  }
}

bool NaiveCollisionManager::empty() const
{
  return objs.empty();
}



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
  for(unsigned int i = 0; i < objs.size(); ++i)
    objs[i] = objs_x[i];
}

void SSaPCollisionManager::checkColl(std::vector<CollisionObject*>::const_iterator pos_start, std::vector<CollisionObject*>::const_iterator pos_end,
                                     CollisionObject* obj, void* cdata, CollisionCallBack callback) const
{
  while(pos_start < pos_end)
  {
    if((*pos_start)->getAABB().overlap(obj->getAABB()))
    {
      if(callback(*pos_start, obj, cdata))
        return;
    }
    pos_start++;
  }
}


void SSaPCollisionManager::collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const
{
  static const unsigned int CUTOFF = 100;

  DummyCollisionObject dummyHigh(AABB(obj->getAABB().max_));

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
          checkColl(pos_start3, pos_end3, obj, cdata, callback);
        else
        {
          if(d2 <= d3 && d2 <= d1)
            checkColl(pos_start2, pos_end2, obj, cdata, callback);
          else
            checkColl(pos_start1, pos_end1, obj, cdata, callback);
        }
      }
      else
        checkColl(pos_start3, pos_end3, obj, cdata, callback);
    }
    else
      checkColl(pos_start2, pos_end2, obj, cdata, callback);
  }
  else
    checkColl(pos_start1, pos_end1, obj, cdata, callback);
}


void SSaPCollisionManager::collide(void* cdata, CollisionCallBack callback) const
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

  int axis2 = (axis + 1 > 2) ? 0 : (axis + 1);
  int axis3 = (axis2 + 1 > 2) ? 0 : (axis2 + 1);

  std::vector<CollisionObject*>::const_iterator pos, run_pos, pos_end;

  switch(axis)
  {
    case 0:
      pos = objs_x.begin();
      pos_end = objs_x.end();
      break;
    case 1:
      pos = objs_y.begin();
      pos_end = objs_y.end();
      break;
    case 2:
      pos = objs_z.begin();
      pos_end = objs_z.end();
      break;
  }
  run_pos = pos;

  while((run_pos != pos_end) && (pos != pos_end))
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

    if(run_pos != pos_end)
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

bool SSaPCollisionManager::empty() const
{
  return objs_x.empty();
}


void SaPCollisionManager::unregisterObject(CollisionObject* obj)
{
  std::list<SaPAABB*>::iterator it = AABB_arr.begin();
  for(; it != AABB_arr.end(); ++it)
  {
    if((*it)->obj == obj)
      break;
  }

  AABB_arr.erase(it);

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

  for(std::list<SaPAABB*>::const_iterator it = AABB_arr.begin(); it != AABB_arr.end(); ++it)
  {
    if((*it)->cached.overlap(curr->cached))
      overlap_pairs.push_back(SaPPair(obj, (*it)->obj));
  }

  AABB_arr.push_back(curr);

  for(int coord = 0; coord < 3; ++coord)
  {
    EndPoint* current = elist[coord];

    // first insert the hi end point
    if(current == NULL) // empty list
    {
      elist[coord] = curr->hi;
      curr->hi->prev[coord] = curr->hi->next[coord] = NULL;
    }
    else // otherwise, find the correct location in the list and insert
    {
      while((current->next[coord] != NULL) &&
          (current->getVal()[coord] < curr->hi->getVal()[coord]))
        current = current->next[coord];

      if(current->getVal()[coord] >= curr->hi->getVal()[coord])
      {
        curr->hi->prev[coord] = current->prev[coord];
        curr->hi->next[coord] = current;
        if(current->prev[coord] == NULL)
          elist[coord] = curr->hi;
        else
          current->prev[coord]->next[coord] = curr->hi;
      }
      else
      {
        curr->hi->prev[coord] = current;
        curr->hi->next[coord] = NULL;
        current->next[coord] = curr->hi;
      }
    }

    // now insert lo end point
    current = elist[coord];

    while ((current->next[coord] != NULL) && (current->getVal()[coord] < curr->lo->getVal()[coord]))
      current = current->next[coord];

    if (current->getVal()[coord] >= curr->lo->getVal()[coord])
    {
      curr->lo->prev[coord] = current->prev[coord];
      curr->lo->next[coord] = current;
      if(current->prev[coord] == NULL)
        elist[coord] = curr->lo;
      else
        current->prev[coord]->next[coord] = curr->lo;

      current->prev[coord] = curr->lo;
    }
    else
    {
      curr->lo->prev[coord] = current;
      curr->lo->next[coord] = NULL;
      current->next[coord] = curr->lo;
    }
  }
}

void SaPCollisionManager::setup()
{

}

void SaPCollisionManager::update()
{
  for(std::list<SaPAABB*>::const_iterator it = AABB_arr.begin(); it != AABB_arr.end(); ++it)
  {
    SaPAABB* current = *it;

    Vec3f new_min = current->obj->getAABB().min_;
    Vec3f new_max = current->obj->getAABB().max_;

    SaPAABB dummy;
    dummy.cached = current->obj->getAABB();

    EndPoint lo, hi;
    dummy.lo = &lo;
    dummy.hi = &hi;

    lo.minmax = 0;
    lo.aabb = &dummy;
    hi.minmax = 1;
    hi.aabb = &dummy;


    for(int coord = 0; coord < 3; ++coord)
    {
      int direction; // -1 reverse, 0 nochange, 1 forward
      EndPoint* temp;

      if(current->lo->getVal()[coord] > new_min[coord])
        direction = -1;
      else if(current->lo->getVal()[coord] < new_min[coord])
        direction = 1;
      else direction = 0;

      if(direction == -1)
      {
        //first update the "lo" endpoint of the interval
        if(current->lo->prev[coord] != NULL)
        {
          temp = current->lo;
          while((temp != NULL) && (temp->getVal()[coord] > new_min[coord]))
          {
            if(temp->minmax == 1)
              if(temp->aabb->cached.overlap(dummy.cached))
                overlap_pairs.push_back(SaPPair(temp->aabb->obj, current->obj));
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

        current->lo->getVal()[coord] = new_min[coord];

        // update hi end point
        temp = current->hi;
        while(temp->getVal()[coord] > new_max[coord])
        {
          if((temp->minmax == 0) && (temp->aabb->cached.overlap(current->cached)))
            overlap_pairs.remove_if(isNotValidPair(temp->aabb->obj, current->obj));
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

        current->hi->getVal()[coord] = new_max[coord];
      }
      else if (direction == 1)
      {
        //here, we first update the "hi" endpoint.
        if(current->hi->next[coord] != NULL)
        {
          temp = current->hi;
          while((temp->next[coord] != NULL) && (temp->getVal()[coord] < new_max[coord]))
          {
            if(temp->minmax == 0)
              if(temp->aabb->cached.overlap(dummy.cached))
                overlap_pairs.push_back(SaPPair(temp->aabb->obj, current->obj));
            temp = temp->next[coord];
          }

          if(temp->getVal()[coord] < new_max[coord])
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

        current->hi->getVal()[coord] = new_max[coord];

        //then, update the "lo" endpoint of the interval.
        temp = current->lo;

        while(temp->getVal()[coord] < new_min[coord])
        {
          if((temp->minmax == 1) && (temp->aabb->cached.overlap(current->cached)))
            overlap_pairs.remove_if(isNotValidPair(temp->aabb->obj, current->obj));
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
        current->lo->getVal()[coord] = new_min[coord];
      }
    }
  }
}


void SaPCollisionManager::clear()
{
  for(std::list<SaPAABB*>::iterator it = AABB_arr.begin();
      it != AABB_arr.end();
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
}

void SaPCollisionManager::getObjects(std::vector<CollisionObject*>& objs) const
{
  objs.resize(AABB_arr.size());
  int i = 0;
  for(std::list<SaPAABB*>::const_iterator it = AABB_arr.begin(); it != AABB_arr.end(); ++it, ++i)
  {
    objs[i] = (*it)->obj;
  }
}

void SaPCollisionManager::collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const
{
  for(std::list<SaPAABB*>::const_iterator it = AABB_arr.begin(); it != AABB_arr.end(); ++it)
  {
    if((*it)->obj->getAABB().overlap(obj->getAABB()))
    {
      if(callback(obj, (*it)->obj, cdata))
        return;
    }
  }
}

void SaPCollisionManager::collide(void* cdata, CollisionCallBack callback) const
{
  for(std::list<SaPPair>::const_iterator it = overlap_pairs.begin(); it != overlap_pairs.end(); ++it)
  {
    CollisionObject* obj1 = it->obj1;
    CollisionObject* obj2 = it->obj2;

    if(callback(obj1, obj2, cdata))
      return;
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
  std::vector<EndPoint>::iterator start1 = std::lower_bound(endpoints[0].begin(), endpoints[0].end(), p, SortByValue());
  p.value = obj->getAABB().max_[0];
  std::vector<EndPoint>::iterator end1 = std::upper_bound(start1, endpoints[0].end(), p, SortByValue());

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
  std::vector<EndPoint>::iterator start2 = std::lower_bound(endpoints[1].begin(), endpoints[1].end(), p, SortByValue());
  p.value = obj->getAABB().max_[1];
  std::vector<EndPoint>::iterator end2 = std::upper_bound(start2, endpoints[1].end(), p, SortByValue());

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
  std::vector<EndPoint>::iterator start3 = std::lower_bound(endpoints[2].begin(), endpoints[2].end(), p, SortByValue());
  p.value = obj->getAABB().max_[2];
  std::vector<EndPoint>::iterator end3 = std::upper_bound(start3, endpoints[2].end(), p, SortByValue());

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
    std::sort(endpoints[0].begin(), endpoints[0].end(), SortByValue());
    std::sort(endpoints[1].begin(), endpoints[1].end(), SortByValue());
    std::sort(endpoints[2].begin(), endpoints[2].end(), SortByValue());

    for(int i = 0; i < 3; ++i)
      delete interval_trees[i];

    for(int i = 0; i < 3; ++i)
      interval_trees[i] = new IntervalTree;

    for(unsigned int i = 0; i < endpoints[0].size(); ++i)
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
      }
    }

    setup_ = true;
  }
}

void IntervalTreeCollisionManager::update()
{
  setup_ = false;

  for(unsigned int i = 0; i < endpoints[0].size(); ++i)
  {
    if(endpoints[0][i].minmax == 0)
      endpoints[0][i].value = endpoints[0][i].obj->getAABB().min_[0];
    else
      endpoints[0][i].value = endpoints[0][i].obj->getAABB().max_[0];
  }

  for(unsigned int i = 0; i < endpoints[1].size(); ++i)
  {
    if(endpoints[1][i].minmax == 0)
      endpoints[1][i].value = endpoints[1][i].obj->getAABB().min_[1];
    else
      endpoints[1][i].value = endpoints[1][i].obj->getAABB().max_[1];
  }

  for(unsigned int i = 0; i < endpoints[2].size(); ++i)
  {
    if(endpoints[2][i].minmax == 0)
      endpoints[2][i].value = endpoints[2][i].obj->getAABB().min_[2];
    else
      endpoints[2][i].value = endpoints[2][i].obj->getAABB().max_[2];
  }

  setup();

}

void IntervalTreeCollisionManager::clear()
{
  endpoints[0].clear();
  endpoints[1].clear();
  endpoints[2].clear();
  setup_ = false;
}

void IntervalTreeCollisionManager::getObjects(std::vector<CollisionObject*>& objs) const
{
  objs.resize(endpoints[0].size() / 2);
  unsigned int j = 0;
  for(unsigned int i = 0; i < endpoints[0].size(); ++i)
  {
    if(endpoints[0][i].minmax == 0)
    {
      objs[j] = endpoints[0][i].obj; j++;
    }
  }
}

void IntervalTreeCollisionManager::checkColl(std::vector<CollisionObject*>::const_iterator pos_start, std::vector<CollisionObject*>::const_iterator pos_end,
                                             CollisionObject* obj, void* cdata, CollisionCallBack callback) const
{

}

void IntervalTreeCollisionManager::collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const
{
  static const unsigned int CUTOFF = 100;

  std::deque<Interval*> results0, results1, results2;

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
        {
          for(unsigned int i = 0; i < results0.size(); ++i)
          {
            SAPInterval* ivl = (SAPInterval*)results0[i];
            if(callback(ivl->obj, obj, cdata))
              break;
          }
        }
        else if(d2 >= d1 && d2 >= d3)
        {
          for(unsigned int i = 0; i < results1.size(); ++i)
          {
            SAPInterval* ivl = (SAPInterval*)results1[i];
            if(callback(ivl->obj, obj, cdata))
              break;
          }
        }
        else
        {
          for(unsigned int i = 0; i < results2.size(); ++i)
          {
            SAPInterval* ivl = (SAPInterval*)results2[i];
            if(callback(ivl->obj, obj, cdata))
              break;
          }
        }
      }
      else
      {
        for(unsigned int i = 0; i < results2.size(); ++i)
        {
          SAPInterval* ivl = (SAPInterval*)results2[i];
          if(callback(ivl->obj, obj, cdata))
            break;
        }
      }
    }
    else
    {
      for(unsigned int i = 0; i < results1.size(); ++i)
      {
        SAPInterval* ivl = (SAPInterval*)results1[i];
        if(callback(ivl->obj, obj, cdata))
          break;
      }
    }
  }
  else
  {
    for(unsigned int i = 0; i < results0.size(); ++i)
    {
      SAPInterval* ivl = (SAPInterval*)results0[i];
      if(callback(ivl->obj, obj, cdata))
        break;
    }
  }

  results0.clear();
  results1.clear();
  results2.clear();
}

void IntervalTreeCollisionManager::collide(void* cdata, CollisionCallBack callback) const
{
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

bool IntervalTreeCollisionManager::empty() const
{
  return endpoints[0].empty();
}



}

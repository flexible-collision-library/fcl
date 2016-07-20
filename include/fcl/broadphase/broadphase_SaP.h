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

#ifndef FCL_BROAD_PHASE_SAP_H
#define FCL_BROAD_PHASE_SAP_H

#include "fcl/broadphase/broadphase.h"

#include <map>
#include <list>

namespace fcl
{

/// @brief Rigorous SAP collision manager
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

  /// @brief add objects to the manager
  void registerObjects(const std::vector<CollisionObject*>& other_objs);

  /// @brief remove one object from the manager
  void registerObject(CollisionObject* obj);

  /// @brief add one object to the manager
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

  /// @brief perform distance computation between one object and all the objects belonging to the manager
  void distance(CollisionObject* obj, void* cdata, DistanceCallBack callback) const;

  /// @brief perform collision test for the objects belonging to the manager (i.e., N^2 self collision)
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
  inline size_t size() const { return AABB_arr.size(); }

protected:

  struct EndPoint;

  /// @brief SAP interval for one object
  struct SaPAABB
  {
    /// @brief object
    CollisionObject* obj;

    /// @brief lower bound end point of the interval
    EndPoint* lo;

    /// @brief higher bound end point of the interval
    EndPoint* hi;

    /// @brief cached AABB value
    AABB cached;
  };

  /// @brief End point for an interval
  struct EndPoint
  {
    /// @brief tag for whether it is a lower bound or higher bound of an interval, 0 for lo, and 1 for hi
    char minmax;

    /// @brief back pointer to SAP interval
    SaPAABB* aabb;

    /// @brief the previous end point in the end point list
    EndPoint* prev[3];
    /// @brief the next end point in the end point list
    EndPoint* next[3];

    /// @brief get the value of the end point
    inline const Vec3f& getVal() const
    {
      if(minmax) return aabb->cached.max_;
      else return aabb->cached.min_;
    }

    /// @brief set the value of the end point
    inline Vec3f& getVal()
    {
      if(minmax) return aabb->cached.max_;
      else return aabb->cached.min_;
    }

    inline Vec3f::U getVal(size_t i) const
    {
      if(minmax) return aabb->cached.max_[i];
      else return aabb->cached.min_[i];
    }

    inline Vec3f::U& getVal(size_t i)
    {
      if(minmax) return aabb->cached.max_[i];
      else return aabb->cached.min_[i];
    }

  };

  /// @brief A pair of objects that are not culling away and should further check collision
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

  /// @brief Functor to help unregister one object
  class isUnregistered
  {
    CollisionObject* obj;

  public:
    isUnregistered(CollisionObject* obj_) : obj(obj_)
    {}

    bool operator() (const SaPPair& pair) const
    {
      return (pair.obj1 == obj) || (pair.obj2 == obj);
    }
  };

  /// @brief Functor to help remove collision pairs no longer valid (i.e., should be culled away)
  class isNotValidPair
  {
    CollisionObject* obj1;
    CollisionObject* obj2;

  public:
    isNotValidPair(CollisionObject* obj1_, CollisionObject* obj2_) : obj1(obj1_),
                                                                     obj2(obj2_)
    {}

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

  /// @brief End point list for x, y, z coordinates
  EndPoint* elist[3];
  
  /// @brief vector version of elist, for acceleration
  std::vector<EndPoint*> velist[3];

  /// @brief SAP interval list
  std::list<SaPAABB*> AABB_arr;

  /// @brief The pair of objects that should further check for collision
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



}


#endif

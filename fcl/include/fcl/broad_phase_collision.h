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
#include "fcl/AABB.h"
#include "fcl/interval_tree.h"
#include <vector>
#include <list>

namespace fcl
{



bool defaultCollisionFunction(CollisionObject* o1, CollisionObject* o2, void* cdata);

/** \brief return value is whether can stop now */
typedef bool (*CollisionCallBack)(CollisionObject* o1, CollisionObject* o2, void* cdata);

class BroadPhaseCollisionManager
{
public:
  virtual void registerObject(CollisionObject* obj) = 0;

  virtual void unregisterObject(CollisionObject* obj) = 0;

  virtual void setup() = 0;

  virtual void update() = 0;

  virtual void clear() = 0;

  virtual void getObjects(std::vector<CollisionObject*>& objs) const = 0;

  virtual void collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const = 0;

  virtual void collide(void* cdata, CollisionCallBack callback) const = 0;

  virtual bool empty() const = 0;
};

class NaiveCollisionManager : public BroadPhaseCollisionManager
{
public:
  NaiveCollisionManager() {}

  void unregisterObject(CollisionObject* obj);

  void registerObject(CollisionObject* obj);

  void setup();

  void update();

  void clear();

  void getObjects(std::vector<CollisionObject*>& objs) const;

  void collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const;

  void collide(void* cdata, CollisionCallBack callback) const;

  bool empty() const;

protected:

  std::list<CollisionObject*> objs;
};

class SaPCollisionManager : public BroadPhaseCollisionManager
{
public:

  SaPCollisionManager()
  {
    elist[0] = NULL;
    elist[1] = NULL;
    elist[2] = NULL;
  }

  ~SaPCollisionManager()
  {
    clear();
  }

  void unregisterObject(CollisionObject* obj);

  void registerObject(CollisionObject* obj);

  void setup();

  void update();

  void clear();

  void getObjects(std::vector<CollisionObject*>& objs) const;

  void collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const;

  void collide(void* cdata, CollisionCallBack callback) const;

  bool empty() const;

protected:

  struct EndPoint;

  struct SaPAABB
  {
    CollisionObject* obj;
    EndPoint* lo;
    EndPoint* hi;
    AABB cached;
  };

  struct EndPoint
  {
    char minmax;
    SaPAABB* aabb;
    EndPoint* prev[3];
    EndPoint* next[3];
    const Vec3f& getVal() const
    {
      if(minmax == 0) return aabb->cached.min_;
      else return aabb->cached.max_;
    }

    Vec3f& getVal()
    {
      if(minmax == 0) return aabb->cached.min_;
      else return aabb->cached.max_;
    }
  };

  struct SaPPair
  {
    SaPPair(CollisionObject* a, CollisionObject* b)
    {
      obj1 = a;
      obj2 = b;
    }

    CollisionObject* obj1;
    CollisionObject* obj2;
  };

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
      return (pair.obj1 == obj1 && pair.obj2 == obj2) || (pair.obj1 == obj2 && pair.obj2 == obj1);
    }
  };


  EndPoint* elist[3];
  std::list<SaPAABB*> AABB_arr;

  std::list<SaPPair> overlap_pairs;
};

class SSaPCollisionManager : public BroadPhaseCollisionManager
{
public:
  SSaPCollisionManager()
  {
    setup_ = false;
  }

  void unregisterObject(CollisionObject* obj);

  void registerObject(CollisionObject* obj);

  void setup();

  void update();

  void clear();

  void getObjects(std::vector<CollisionObject*>& objs) const;

  void collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const;

  void collide(void* cdata, CollisionCallBack callback) const;

  bool empty() const;

protected:

  struct SortByXLow
  {
    bool operator()(const CollisionObject* a, const CollisionObject* b) const
    {
      if(a->getCachedAABB().min_[0] < b->getCachedAABB().min_[0])
        return true;
      return false;
    }
  };

  struct SortByYLow
   {
     bool operator()(const CollisionObject* a, const CollisionObject* b) const
     {
       if(a->getCachedAABB().min_[1] < b->getCachedAABB().min_[1])
         return true;
       return false;
     }
   };

   struct SortByZLow
   {
     bool operator()(const CollisionObject* a, const CollisionObject* b) const
     {
       if(a->getCachedAABB().min_[2] < b->getCachedAABB().min_[2])
         return true;
       return false;
     }
   };

   class DummyCollisionObject : public CollisionObject
   {
   public:
     DummyCollisionObject(const AABB& aabb)
     {
       aabb_cache = aabb;
     }

     void computeAABB() {}
   };

   void checkColl(std::vector<CollisionObject*>::const_iterator pos_start, std::vector<CollisionObject*>::const_iterator pos_end,
                  CollisionObject* obj, void* cdata, CollisionCallBack callback) const;


   std::vector<CollisionObject*> objs_x;
   std::vector<CollisionObject*> objs_y;
   std::vector<CollisionObject*> objs_z;

   bool setup_;
};


class IntervalTreeCollisionManager : public BroadPhaseCollisionManager
{
public:
  IntervalTreeCollisionManager()
  {
    setup_ = false;
    for(int i = 0; i < 3; ++i)
      interval_trees[i] = NULL;
  }

  void unregisterObject(CollisionObject* obj);

  void registerObject(CollisionObject* obj);

  void setup();

  void update();

  void clear();

  void getObjects(std::vector<CollisionObject*>& objs) const;

  void checkColl(std::vector<CollisionObject*>::const_iterator pos_start, std::vector<CollisionObject*>::const_iterator pos_end,
                 CollisionObject* obj, void* cdata, CollisionCallBack callback) const;

  void collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const;

  void collide(void* cdata, CollisionCallBack callback) const;

  bool empty() const;

protected:


  struct EndPoint
  {
    CollisionObject* obj; // pointer to endpoint geometry;
    BVH_REAL value; // endpoint value
    char minmax; // '0' if interval min, '1' if interval max
  };

  struct SortByValue
  {
    bool operator()(const EndPoint& a, const EndPoint& b) const
    {
      if(a.value < b.value)
        return true;
      return false;
    }
  };

  struct SAPInterval : public Interval
  {
    CollisionObject* obj;
    SAPInterval(double low_, double high_, CollisionObject* obj_)
    {
      low = low_;
      high = high_;
      obj = obj_;
    }
  };


  std::vector<EndPoint> endpoints[3];

  IntervalTree* interval_trees[3];

  bool setup_;

};


}

#endif

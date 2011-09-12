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

/** \brief Base class for broad phase collision */
class BroadPhaseCollisionManager
{
public:
  /** \brief add one object to the manager */
  virtual void registerObject(CollisionObject* obj) = 0;

  /** \brief remove one object from the manager */
  virtual void unregisterObject(CollisionObject* obj) = 0;

  /** \brief initialize the manager, related with the specific type of manager */
  virtual void setup() = 0;

  /** \brief update the condition of manager */
  virtual void update() = 0;

  /** \brief clear the manager */
  virtual void clear() = 0;

  /** \brief return the objects managed by the manager */
  virtual void getObjects(std::vector<CollisionObject*>& objs) const = 0;

  /** \brief perform collision test between one object and all the objects belonging to the manager */
  virtual void collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const = 0;

  /** \brief perform collision test for the objects belonging to the manager (i.e., N^2 self collision) */
  virtual void collide(void* cdata, CollisionCallBack callback) const = 0;

  /** \brief whether the manager is empty */
  virtual bool empty() const = 0;
  
  /** \brief the number of objects managed by the manager */
  virtual size_t size() const = 0;
};

/** \brief Brute force N-body collision manager */
class NaiveCollisionManager : public BroadPhaseCollisionManager
{
public:
  NaiveCollisionManager() {}

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

  /** \brief perform collision test for the objects belonging to the manager (i.e., N^2 self collision) */
  void collide(void* cdata, CollisionCallBack callback) const;

  /** \brief whether the manager is empty */
  bool empty() const;
  
  /** \brief the number of objects managed by the manager */
  inline size_t size() const { return objs.size(); }

protected:

  /** \brief objects belonging to the manager are stored in a list structure */
  std::list<CollisionObject*> objs;
};

/** Rigorous SAP collision manager */
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

  /** \brief perform collision test for the objects belonging to the manager (i.e., N^2 self collision) */
  void collide(void* cdata, CollisionCallBack callback) const;

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
    const Vec3f& getVal() const
    {
      if(minmax == 0) return aabb->cached.min_;
      else return aabb->cached.max_;
    }

    /** \brief set the value of the end point */
    Vec3f& getVal()
    {
      if(minmax == 0) return aabb->cached.min_;
      else return aabb->cached.max_;
    }
  };

  /** \brief A pair of objects that are not culling away and should further check collision */
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
      return (pair.obj1 == obj1 && pair.obj2 == obj2) || (pair.obj1 == obj2 && pair.obj2 == obj1);
    }
  };

  /** \brief End point list for x, y, z coordinates */
  EndPoint* elist[3];

  /** \brief SAP interval list */
  std::list<SaPAABB*> AABB_arr;

  /** \brief The pair of objects that should further check for collision */
  std::list<SaPPair> overlap_pairs;
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

  /** \brief perform collision test for the objects belonging to the manager (i.e., N^2 self collision) */
  void collide(void* cdata, CollisionCallBack callback) const;

  /** \brief whether the manager is empty */
  bool empty() const;
  
  /** \brief the number of objects managed by the manager */
  inline size_t size() const { return objs_x.size(); }

protected:

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
     DummyCollisionObject(const AABB& aabb_)
     {
       aabb = aabb_;
     }

     void computeLocalAABB() {}
   };

   /** \brief check collision between one object and a list of objects */
   void checkColl(std::vector<CollisionObject*>::const_iterator pos_start, std::vector<CollisionObject*>::const_iterator pos_end,
                  CollisionObject* obj, void* cdata, CollisionCallBack callback) const;


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

  /** \brief perform collision test for the objects belonging to the manager (i.e., N^2 self collision) */
  void collide(void* cdata, CollisionCallBack callback) const;

  /** \brief whether the manager is empty */
  bool empty() const;
  
  /** \brief the number of objects managed by the manager */
  inline size_t size() const { return endpoints[0].size() / 2; }

protected:

  /** \brief check collision between one object and a list of objects */
  void checkColl(std::vector<CollisionObject*>::const_iterator pos_start, std::vector<CollisionObject*>::const_iterator pos_end,
                 CollisionObject* obj, void* cdata, CollisionCallBack callback) const;


  /** \brief SAP end point */
  struct EndPoint
  {
    /** \brief object related with the end point */
    CollisionObject* obj;

    /** \brief end point value */
    BVH_REAL value;

    /** \brief tag for whether it is a lower bound or higher bound of an interval, 0 for lo, and 1 for hi */
    char minmax;
  };

  /** \brief Functor for sorting end points */
  struct SortByValue
  {
    bool operator()(const EndPoint& a, const EndPoint& b) const
    {
      if(a.value < b.value)
        return true;
      return false;
    }
  };

  /** \brief Extention interval tree's interval to SAP interval, adding more information */
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

  /** \brief vector stores all the end points */
  std::vector<EndPoint> endpoints[3];

  /** \brief  interval tree manages the intervals */
  IntervalTree* interval_trees[3];

  /** \brief tag for whether the interval tree is maintained suitably */
  bool setup_;

};


}

#endif

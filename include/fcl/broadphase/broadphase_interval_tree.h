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

#ifndef FCL_BROAD_PHASE_INTERVAL_TREE_H
#define FCL_BROAD_PHASE_INTERVAL_TREE_H

#include "fcl/broadphase/broadphase.h"
#include "fcl/broadphase/interval_tree.h"
#include <deque>
#include <map>

namespace fcl
{

/// @brief Collision manager based on interval tree
class IntervalTreeCollisionManager : public BroadPhaseCollisionManager
{
public:
  IntervalTreeCollisionManager() : setup_(false)
  {
    for(int i = 0; i < 3; ++i)
      interval_trees[i] = NULL;
  }

  ~IntervalTreeCollisionManager()
  {
    clear();
  }

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
  inline size_t size() const { return endpoints[0].size() / 2; }



protected:

  /// @brief SAP end point
  struct EndPoint
  {
    /// @brief object related with the end point
    CollisionObject* obj;

    /// @brief end point value
    FCL_REAL value;

    /// @brief tag for whether it is a lower bound or higher bound of an interval, 0 for lo, and 1 for hi
    char minmax;

    bool operator<(const EndPoint &p) const
    {
        return value < p.value;
    }
  };

  /// @brief Extention interval tree's interval to SAP interval, adding more information
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

  /// @brief vector stores all the end points
  std::vector<EndPoint> endpoints[3];

  /// @brief  interval tree manages the intervals
  IntervalTree* interval_trees[3];

  std::map<CollisionObject*, SAPInterval*> obj_interval_maps[3];

  /// @brief tag for whether the interval tree is maintained suitably
  bool setup_;
};


}

#endif

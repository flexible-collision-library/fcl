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

#ifndef FCL_BROAD_PHASE_SSAP_H
#define FCL_BROAD_PHASE_SSAP_H

#include <vector>
#include "fcl/broadphase/broadphase_collision_manager.h"

namespace fcl
{

/// @brief Simple SAP collision manager 
template <typename S>
class FCL_EXPORT SSaPCollisionManager : public BroadPhaseCollisionManager<S>
{
public:
  SSaPCollisionManager();

  /// @brief remove one object from the manager
  void registerObject(CollisionObject<S>* obj);

  /// @brief add one object to the manager
  void unregisterObject(CollisionObject<S>* obj);

  /// @brief initialize the manager, related with the specific type of manager
  void setup();

  /// @brief update the condition of manager
  void update();

  /// @brief clear the manager
  void clear();

  /// @brief return the objects managed by the manager
  void getObjects(std::vector<CollisionObject<S>*>& objs) const;

  /// @brief perform collision test between one object and all the objects belonging to the manager
  void collide(CollisionObject<S>* obj, void* cdata, CollisionCallBack<S> callback) const;

  /// @brief perform distance computation between one object and all the objects belonging to the manager
  void distance(CollisionObject<S>* obj, void* cdata, DistanceCallBack<S> callback) const;

  /// @brief perform collision test for the objects belonging to the manager (i.e., N^2 self collision)
  void collide(void* cdata, CollisionCallBack<S> callback) const;

  /// @brief perform distance test for the objects belonging to the manager (i.e., N^2 self distance)
  void distance(void* cdata, DistanceCallBack<S> callback) const;

  /// @brief perform collision test with objects belonging to another manager
  void collide(BroadPhaseCollisionManager<S>* other_manager, void* cdata, CollisionCallBack<S> callback) const;

  /// @brief perform distance test with objects belonging to another manager
  void distance(BroadPhaseCollisionManager<S>* other_manager, void* cdata, DistanceCallBack<S> callback) const;

  /// @brief whether the manager is empty
  bool empty() const;
  
  /// @brief the number of objects managed by the manager
  size_t size() const;

protected:
  /// @brief check collision between one object and a list of objects, return value is whether stop is possible
  bool checkColl(typename std::vector<CollisionObject<S>*>::const_iterator pos_start, typename std::vector<CollisionObject<S>*>::const_iterator pos_end,
                 CollisionObject<S>* obj, void* cdata, CollisionCallBack<S> callback) const;
  
  /// @brief check distance between one object and a list of objects, return value is whether stop is possible
  bool checkDis(typename std::vector<CollisionObject<S>*>::const_iterator pos_start, typename std::vector<CollisionObject<S>*>::const_iterator pos_end,
                CollisionObject<S>* obj, void* cdata, DistanceCallBack<S> callback, S& min_dist) const;

  bool collide_(CollisionObject<S>* obj, void* cdata, CollisionCallBack<S> callback) const;
  
  bool distance_(CollisionObject<S>* obj, void* cdata, DistanceCallBack<S> callback, S& min_dist) const;

  static size_t selectOptimalAxis(
      const std::vector<CollisionObject<S>*>& objs_x,
      const std::vector<CollisionObject<S>*>& objs_y,
      const std::vector<CollisionObject<S>*>& objs_z,
      typename std::vector<CollisionObject<S>*>::const_iterator& it_beg,
      typename std::vector<CollisionObject<S>*>::const_iterator& it_end);

  /// @brief Objects sorted according to lower x value
  std::vector<CollisionObject<S>*> objs_x;

  /// @brief Objects sorted according to lower y value
  std::vector<CollisionObject<S>*> objs_y;

  /// @brief Objects sorted according to lower z value
  std::vector<CollisionObject<S>*> objs_z;

  /// @brief tag about whether the environment is maintained suitably (i.e., the objs_x, objs_y, objs_z are sorted correctly
  bool setup_;
};

using SSaPCollisionManagerf = SSaPCollisionManager<float>;
using SSaPCollisionManagerd = SSaPCollisionManager<double>;

} // namespace fcl

#include "fcl/broadphase/broadphase_SSaP-inl.h"

#endif

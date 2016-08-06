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

#ifndef FCL_BROAD_PHASE_BRUTE_FORCE_H
#define FCL_BROAD_PHASE_BRUTE_FORCE_H

#include "fcl/broadphase/broadphase.h"
#include <list>


namespace fcl
{

/// @brief Brute force N-body collision manager
template <typename Scalar>
class NaiveCollisionManager : public BroadPhaseCollisionManager<Scalar>
{
public:
  NaiveCollisionManager() {}

  /// @brief add objects to the manager
  void registerObjects(const std::vector<CollisionObject<Scalar>*>& other_objs);

  /// @brief add one object to the manager
  void registerObject(CollisionObject<Scalar>* obj);

  /// @brief remove one object from the manager
  void unregisterObject(CollisionObject<Scalar>* obj);

  /// @brief initialize the manager, related with the specific type of manager
  void setup();

  /// @brief update the condition of manager
  void update();

  /// @brief clear the manager
  void clear();

  /// @brief return the objects managed by the manager
  void getObjects(std::vector<CollisionObject<Scalar>*>& objs) const;

  /// @brief perform collision test between one object and all the objects belonging to the manager
  void collide(CollisionObject<Scalar>* obj, void* cdata, CollisionCallBack<Scalar> callback) const;

  /// @brief perform distance computation between one object and all the objects belonging to the manager
  void distance(CollisionObject<Scalar>* obj, void* cdata, DistanceCallBack<Scalar> callback) const;

  /// @brief perform collision test for the objects belonging to the manager (i.e., N^2 self collision)
  void collide(void* cdata, CollisionCallBack<Scalar> callback) const;

  /// @brief perform distance test for the objects belonging to the manager (i.e., N^2 self distance)
  void distance(void* cdata, DistanceCallBack<Scalar> callback) const;

  /// @brief perform collision test with objects belonging to another manager
  void collide(BroadPhaseCollisionManager<Scalar>* other_manager, void* cdata, CollisionCallBack<Scalar> callback) const;

  /// @brief perform distance test with objects belonging to another manager
  void distance(BroadPhaseCollisionManager<Scalar>* other_manager, void* cdata, DistanceCallBack<Scalar> callback) const;

  /// @brief whether the manager is empty
  bool empty() const;
  
  /// @brief the number of objects managed by the manager
  inline size_t size() const { return objs.size(); }

protected:

  /// @brief objects belonging to the manager are stored in a list structure
  std::list<CollisionObject<Scalar>*> objs;
};

using NaiveCollisionManagerf = NaiveCollisionManager<float>;
using NaiveCollisionManagerd = NaiveCollisionManager<double>;

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
void NaiveCollisionManager<Scalar>::registerObjects(const std::vector<CollisionObject<Scalar>*>& other_objs)
{
  std::copy(other_objs.begin(), other_objs.end(), std::back_inserter(objs));
}

//==============================================================================
template <typename Scalar>
void NaiveCollisionManager<Scalar>::unregisterObject(CollisionObject<Scalar>* obj)
{
  objs.remove(obj);
}

//==============================================================================
template <typename Scalar>
void NaiveCollisionManager<Scalar>::registerObject(CollisionObject<Scalar>* obj)
{
  objs.push_back(obj);
}

//==============================================================================
template <typename Scalar>
void NaiveCollisionManager<Scalar>::setup()
{

}

//==============================================================================
template <typename Scalar>
void NaiveCollisionManager<Scalar>::update()
{

}

//==============================================================================
template <typename Scalar>
void NaiveCollisionManager<Scalar>::clear()
{
  objs.clear();
}

//==============================================================================
template <typename Scalar>
void NaiveCollisionManager<Scalar>::getObjects(std::vector<CollisionObject<Scalar>*>& objs_) const
{
  objs_.resize(objs.size());
  std::copy(objs.begin(), objs.end(), objs_.begin());
}

//==============================================================================
template <typename Scalar>
void NaiveCollisionManager<Scalar>::collide(CollisionObject<Scalar>* obj, void* cdata, CollisionCallBack<Scalar> callback) const
{
  if(size() == 0) return;

  for(auto* obj2 : objs)
  {
    if(callback(obj, obj2, cdata))
      return;
  }
}

//==============================================================================
template <typename Scalar>
void NaiveCollisionManager<Scalar>::distance(CollisionObject<Scalar>* obj, void* cdata, DistanceCallBack<Scalar> callback) const
{
  if(size() == 0) return;

  Scalar min_dist = std::numeric_limits<Scalar>::max();
  for(auto* obj2 : objs)
  {
    if(obj->getAABB().distance(obj2->getAABB()) < min_dist)
    {
      if(callback(obj, obj2, cdata, min_dist))
        return;
    }
  }
}

//==============================================================================
template <typename Scalar>
void NaiveCollisionManager<Scalar>::collide(void* cdata, CollisionCallBack<Scalar> callback) const
{
  if(size() == 0) return;

  for(typename std::list<CollisionObject<Scalar>*>::const_iterator it1 = objs.begin(), end = objs.end();
      it1 != end; ++it1)
  {
    typename std::list<CollisionObject<Scalar>*>::const_iterator it2 = it1; it2++;
    for(; it2 != end; ++it2)
    {
      if((*it1)->getAABB().overlap((*it2)->getAABB()))
      {
        if(callback(*it1, *it2, cdata))
          return;
      }
    }
  }
}

//==============================================================================
template <typename Scalar>
void NaiveCollisionManager<Scalar>::distance(void* cdata, DistanceCallBack<Scalar> callback) const
{
  if(size() == 0) return;

  Scalar min_dist = std::numeric_limits<Scalar>::max();
  for(typename std::list<CollisionObject<Scalar>*>::const_iterator it1 = objs.begin(), end = objs.end(); it1 != end; ++it1)
  {
    typename std::list<CollisionObject<Scalar>*>::const_iterator it2 = it1; it2++;
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

//==============================================================================
template <typename Scalar>
void NaiveCollisionManager<Scalar>::collide(BroadPhaseCollisionManager<Scalar>* other_manager_, void* cdata, CollisionCallBack<Scalar> callback) const
{
  NaiveCollisionManager* other_manager = static_cast<NaiveCollisionManager*>(other_manager_);

  if((size() == 0) || (other_manager->size() == 0)) return;

  if(this == other_manager)
  {
    collide(cdata, callback);
    return;
  }

  for(auto* obj1 : objs)
  {
    for(auto* obj2 : other_manager->objs)
    {
      if(obj1->getAABB().overlap(obj2->getAABB()))
      {
        if(callback(obj1, obj2, cdata))
          return;
      }
    }
  }
}

//==============================================================================
template <typename Scalar>
void NaiveCollisionManager<Scalar>::distance(BroadPhaseCollisionManager<Scalar>* other_manager_, void* cdata, DistanceCallBack<Scalar> callback) const
{
  NaiveCollisionManager* other_manager = static_cast<NaiveCollisionManager*>(other_manager_);

  if((size() == 0) || (other_manager->size() == 0)) return;

  if(this == other_manager)
  {
    distance(cdata, callback);
    return;
  }

  Scalar min_dist = std::numeric_limits<Scalar>::max();
  for(auto* obj1 : objs)
  {
    for(auto* obj2 : other_manager->objs)
    {
      if(obj1->getAABB().distance(obj2->getAABB()) < min_dist)
      {
        if(callback(obj1, obj2, cdata, min_dist))
          return;
      }
    }
  }
}

//==============================================================================
template <typename Scalar>
bool NaiveCollisionManager<Scalar>::empty() const
{
  return objs.empty();
}

} // namespace

#endif

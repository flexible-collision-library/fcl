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


#ifndef FCL_BROAD_PHASE_DYNAMIC_AABB_TREE_H
#define FCL_BROAD_PHASE_DYNAMIC_AABB_TREE_H

#include "fcl/broadphase/broadphase.h"
#include "fcl/broadphase/hierarchy_tree.h"
#include "fcl/BV/BV.h"
#include "fcl/shape/geometric_shapes_utility.h"
#include <unordered_map>
#include <functional>
#include <limits>


namespace fcl
{

class DynamicAABBTreeCollisionManager : public BroadPhaseCollisionManager
{
public:
  typedef NodeBase<AABB> DynamicAABBNode;
  typedef std::unordered_map<CollisionObject*, DynamicAABBNode*> DynamicAABBTable;

  int max_tree_nonbalanced_level;
  int tree_incremental_balance_pass;
  int& tree_topdown_balance_threshold;
  int& tree_topdown_level;
  int tree_init_level;

  bool octree_as_geometry_collide;
  bool octree_as_geometry_distance;

  
  DynamicAABBTreeCollisionManager() : tree_topdown_balance_threshold(dtree.bu_threshold),
                                      tree_topdown_level(dtree.topdown_level)
  {
    max_tree_nonbalanced_level = 10;
    tree_incremental_balance_pass = 10;
    tree_topdown_balance_threshold = 2;
    tree_topdown_level = 0;
    tree_init_level = 0;
    setup_ = false;

    // from experiment, this is the optimal setting
    octree_as_geometry_collide = true;
    octree_as_geometry_distance = false;
  }

  /// @brief add objects to the manager
  void registerObjects(const std::vector<CollisionObject*>& other_objs);
  
  /// @brief add one object to the manager
  void registerObject(CollisionObject* obj);

  /// @brief remove one object from the manager
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
  void clear()
  {
    dtree.clear();
    table.clear();
  }

  /// @brief return the objects managed by the manager
  void getObjects(std::vector<CollisionObject*>& objs) const
  {
    objs.resize(this->size());
    std::transform(table.begin(), table.end(), objs.begin(), std::bind(&DynamicAABBTable::value_type::first, std::placeholders::_1));
  }

  /// @brief perform collision test between one object and all the objects belonging to the manager
  void collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const;

  /// @brief perform distance computation between one object and all the objects belonging to the manager
  void distance(CollisionObject* obj, void* cdata, DistanceCallBack callback) const;

  /// @brief perform collision test for the objects belonging to the manager (i.e., N^2 self collision)
  void collide(void* cdata, CollisionCallBack callback) const;

  /// @brief perform distance test for the objects belonging to the manager (i.e., N^2 self distance)
  void distance(void* cdata, DistanceCallBack callback) const;

  /// @brief perform collision test with objects belonging to another manager
  void collide(BroadPhaseCollisionManager* other_manager_, void* cdata, CollisionCallBack callback) const;

  /// @brief perform distance test with objects belonging to another manager
  void distance(BroadPhaseCollisionManager* other_manager_, void* cdata, DistanceCallBack callback) const;
  
  /// @brief whether the manager is empty
  bool empty() const
  {
    return dtree.empty();
  }
  
  /// @brief the number of objects managed by the manager
  size_t size() const
  {
    return dtree.size();
  }

  const HierarchyTree<AABB>& getTree() const { return dtree; }


private:
  HierarchyTree<AABB> dtree;
  std::unordered_map<CollisionObject*, DynamicAABBNode*> table;

  bool setup_;

  void update_(CollisionObject* updated_obj);
};



}

#endif

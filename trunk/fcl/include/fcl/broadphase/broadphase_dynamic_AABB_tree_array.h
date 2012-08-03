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

#ifndef FCL_BROAD_PHASE_DYNAMIC_AABB_TREE_ARRAY_H
#define FCL_BROAD_PHASE_DYNAMIC_AABB_TREE_ARRAY_H

#include "fcl/broadphase/broadphase.h"
#include "fcl/broadphase/hierarchy_tree.h"
#include "fcl/octree.h"
#include "fcl/BV/BV.h"
#include "fcl/shape/geometric_shapes_utility.h"
#include <boost/unordered_map.hpp>
#include <boost/bind.hpp>
#include <limits>


namespace fcl
{

class DynamicAABBTreeCollisionManager_Array : public BroadPhaseCollisionManager
{
public:
  typedef implementation_array::NodeBase<AABB> DynamicAABBNode;
  typedef boost::unordered_map<CollisionObject*, size_t> DynamicAABBTable;

  int max_tree_nonbalanced_level;
  int tree_incremental_balance_pass;
  int& tree_topdown_balance_threshold;
  int& tree_topdown_level;
  int tree_init_level;

  bool octree_as_geometry_collide;
  bool octree_as_geometry_distance;
  
  DynamicAABBTreeCollisionManager_Array() : tree_topdown_balance_threshold(dtree.bu_threshold),
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
    std::transform(table.begin(), table.end(), objs.begin(), boost::bind(&DynamicAABBTable::value_type::first, _1));
  }

  /// @brief perform collision test between one object and all the objects belonging to the manager
  void collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const
  {
    if(size() == 0) return;
    switch(obj->getCollisionGeometry()->getNodeType())
    {
    case GEOM_OCTREE:
      {
        if(!octree_as_geometry_collide)
        {
          const OcTree* octree = static_cast<const OcTree*>(obj->getCollisionGeometry());
          collisionRecurse(dtree.getNodes(), dtree.getRoot(), octree, octree->getRoot(), octree->getRootBV(), obj->getTransform(), cdata, callback); 
        }
        else
          collisionRecurse(dtree.getNodes(), dtree.getRoot(), obj, cdata, callback);
      }
      break;
    default:
      collisionRecurse(dtree.getNodes(), dtree.getRoot(), obj, cdata, callback);
    }
  }

  /// @brief perform distance computation between one object and all the objects belonging to the manager
  void distance(CollisionObject* obj, void* cdata, DistanceCallBack callback) const
  {
    if(size() == 0) return;
    FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
    switch(obj->getCollisionGeometry()->getNodeType())
    {
    case GEOM_OCTREE:
      {
        if(!octree_as_geometry_distance)
        {
          const OcTree* octree = static_cast<const OcTree*>(obj->getCollisionGeometry());
          distanceRecurse(dtree.getNodes(), dtree.getRoot(), octree, octree->getRoot(), octree->getRootBV(), obj->getTransform(), cdata, callback, min_dist);
        }
        else
          distanceRecurse(dtree.getNodes(), dtree.getRoot(), obj, cdata, callback, min_dist);
      }
      break;
    default:
      distanceRecurse(dtree.getNodes(), dtree.getRoot(), obj, cdata, callback, min_dist);
    }
  }

  /// @brief perform collision test for the objects belonging to the manager (i.e., N^2 self collision)
  void collide(void* cdata, CollisionCallBack callback) const
  {
    if(size() == 0) return;
    selfCollisionRecurse(dtree.getNodes(), dtree.getRoot(), cdata, callback);
  }

  /// @brief perform distance test for the objects belonging to the manager (i.e., N^2 self distance)
  void distance(void* cdata, DistanceCallBack callback) const
  {
    if(size() == 0) return;
    FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
    selfDistanceRecurse(dtree.getNodes(), dtree.getRoot(), cdata, callback, min_dist);
  }

  /// @brief perform collision test with objects belonging to another manager
  void collide(BroadPhaseCollisionManager* other_manager_, void* cdata, CollisionCallBack callback) const
  {
    DynamicAABBTreeCollisionManager_Array* other_manager = static_cast<DynamicAABBTreeCollisionManager_Array*>(other_manager_);
    if((size() == 0) || (other_manager->size() == 0)) return;
    collisionRecurse(dtree.getNodes(), dtree.getRoot(), other_manager->dtree.getNodes(), other_manager->dtree.getRoot(), cdata, callback);
  }

  /// @brief perform distance test with objects belonging to another manager
  void distance(BroadPhaseCollisionManager* other_manager_, void* cdata, DistanceCallBack callback) const
  {
    DynamicAABBTreeCollisionManager_Array* other_manager = static_cast<DynamicAABBTreeCollisionManager_Array*>(other_manager_);
    if((size() == 0) || (other_manager->size() == 0)) return;
    FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
    distanceRecurse(dtree.getNodes(), dtree.getRoot(), other_manager->dtree.getNodes(), other_manager->dtree.getRoot(), cdata, callback, min_dist);
  }
  
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


  const implementation_array::HierarchyTree<AABB>& getTree() const { return dtree; }

private:
  implementation_array::HierarchyTree<AABB> dtree;
  boost::unordered_map<CollisionObject*, size_t> table;

  bool setup_;

  bool collisionRecurse(DynamicAABBNode* nodes1, size_t root1, DynamicAABBNode* nodes2, size_t root2, void* cdata, CollisionCallBack callback) const;

  bool collisionRecurse(DynamicAABBNode* nodes, size_t root, CollisionObject* query, void* cdata, CollisionCallBack callback) const;

  bool selfCollisionRecurse(DynamicAABBNode* nodes, size_t root, void* cdata, CollisionCallBack callback) const;

  bool distanceRecurse(DynamicAABBNode* nodes1, size_t root1, DynamicAABBNode* nodes2, size_t root2, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const;

  bool distanceRecurse(DynamicAABBNode* nodes, size_t root, CollisionObject* query, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const;

  bool selfDistanceRecurse(DynamicAABBNode* nodes, size_t root, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const;  

  void update_(CollisionObject* updated_obj);

  /// @brief special manager-obj collision for octree
  bool collisionRecurse(DynamicAABBNode* nodes1, size_t root1_id, const OcTree* tree2, const OcTree::OcTreeNode* root2, const AABB& root2_bv, const Transform3f& tf2, void* cdata, CollisionCallBack callback) const;

  bool distanceRecurse(DynamicAABBNode* nodes1, size_t root1_id, const OcTree* tree2, const OcTree::OcTreeNode* root2, const AABB& root2_bv, const Transform3f& tf2, void* cdata, DistanceCallBack callback, FCL_REAL& min_dist) const;

  
};


}

#endif

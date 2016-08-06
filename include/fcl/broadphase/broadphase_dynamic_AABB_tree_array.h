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

#ifndef FCL_BROAD_PHASE_DYNAMIC_AABB_TREE_ARRAY_H
#define FCL_BROAD_PHASE_DYNAMIC_AABB_TREE_ARRAY_H

#include <unordered_map>
#include <functional>
#include <limits>
#include "fcl/shape/box.h"
#include "fcl/shape/construct_box.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/broadphase/hierarchy_tree.h"
#include "fcl/BV/BV.h"
#if FCL_HAVE_OCTOMAP
#include "fcl/octree.h"
#endif

namespace fcl
{

template <typename Scalar>
class DynamicAABBTreeCollisionManager_Array : public BroadPhaseCollisionManager<Scalar>
{
public:

  using DynamicAABBNode = implementation_array::NodeBase<AABB<Scalar>>;
  using DynamicAABBTable = std::unordered_map<CollisionObject<Scalar>*, size_t>;

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
  void registerObjects(const std::vector<CollisionObject<Scalar>*>& other_objs);
  
  /// @brief add one object to the manager
  void registerObject(CollisionObject<Scalar>* obj);

  /// @brief remove one object from the manager
  void unregisterObject(CollisionObject<Scalar>* obj);

  /// @brief initialize the manager, related with the specific type of manager
  void setup();

  /// @brief update the condition of manager
  void update();

  /// @brief update the manager by explicitly given the object updated
  void update(CollisionObject<Scalar>* updated_obj);

  /// @brief update the manager by explicitly given the set of objects update
  void update(const std::vector<CollisionObject<Scalar>*>& updated_objs);

  /// @brief clear the manager
  void clear()
  {
    dtree.clear();
    table.clear();
  }

  /// @brief return the objects managed by the manager
  void getObjects(std::vector<CollisionObject<Scalar>*>& objs) const
  {
    objs.resize(this->size());
    std::transform(table.begin(), table.end(), objs.begin(), std::bind(&DynamicAABBTable::value_type::first, std::placeholders::_1));
  }

  /// @brief perform collision test between one object and all the objects belonging to the manager
  void collide(CollisionObject<Scalar>* obj, void* cdata, CollisionCallBack<Scalar> callback) const;

  /// @brief perform distance computation between one object and all the objects belonging to the manager
  void distance(CollisionObject<Scalar>* obj, void* cdata, DistanceCallBack<Scalar> callback) const;

  /// @brief perform collision test for the objects belonging to the manager (i.e., N^2 self collision)
  void collide(void* cdata, CollisionCallBack<Scalar> callback) const;

  /// @brief perform distance test for the objects belonging to the manager (i.e., N^2 self distance)
  void distance(void* cdata, DistanceCallBack<Scalar> callback) const;

  /// @brief perform collision test with objects belonging to another manager
  void collide(BroadPhaseCollisionManager<Scalar>* other_manager_, void* cdata, CollisionCallBack<Scalar> callback) const;

  /// @brief perform distance test with objects belonging to another manager
  void distance(BroadPhaseCollisionManager<Scalar>* other_manager_, void* cdata, DistanceCallBack<Scalar> callback) const;
  
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


  const implementation_array::HierarchyTree<AABB<Scalar>>& getTree() const { return dtree; }

private:
  implementation_array::HierarchyTree<AABB<Scalar>> dtree;
  std::unordered_map<CollisionObject<Scalar>*, size_t> table;

  bool setup_;

  void update_(CollisionObject<Scalar>* updated_obj);
};

using DynamicAABBTreeCollisionManager_Arrayf = DynamicAABBTreeCollisionManager_Array<float>;
using DynamicAABBTreeCollisionManager_Arrayd = DynamicAABBTreeCollisionManager_Array<double>;

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

namespace details
{

namespace dynamic_AABB_tree_array
{

#if FCL_HAVE_OCTOMAP

//==============================================================================
template <typename Scalar>
bool collisionRecurse_(
    typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* nodes1,
    size_t root1_id,
    const OcTree<Scalar>* tree2,
    const typename OcTree<Scalar>::OcTreeNode* root2,
    const AABB<Scalar>& root2_bv,
    const Transform3<Scalar>& tf2,
    void* cdata,
    CollisionCallBack<Scalar> callback)
{
  typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* root1 = nodes1 + root1_id;
  if(!root2)
  {
    if(root1->isLeaf())
    {
      CollisionObject<Scalar>* obj1 = static_cast<CollisionObject<Scalar>*>(root1->data);

      if(!obj1->isFree())
      {
        OBB<Scalar> obb1, obb2;
        convertBV(root1->bv, Transform3<Scalar>::Identity(), obb1);
        convertBV(root2_bv, tf2, obb2);

        if(obb1.overlap(obb2))
        {
          Box<Scalar>* box = new Box<Scalar>();
          Transform3<Scalar> box_tf;
          constructBox(root2_bv, tf2, *box, box_tf);

          box->cost_density = tree2->getDefaultOccupancy();

          CollisionObject<Scalar> obj2(std::shared_ptr<CollisionGeometry<Scalar>>(box), box_tf);
          return callback(obj1, &obj2, cdata);
        }
      }
    }
    else
    {
      if(collisionRecurse_<Scalar>(nodes1, root1->children[0], tree2, NULL, root2_bv, tf2, cdata, callback))
        return true;
      if(collisionRecurse_<Scalar>(nodes1, root1->children[1], tree2, NULL, root2_bv, tf2, cdata, callback))
        return true;
    }

    return false;
  }
  else if(root1->isLeaf() && !tree2->nodeHasChildren(root2))
  {
    CollisionObject<Scalar>* obj1 = static_cast<CollisionObject<Scalar>*>(root1->data);
    if(!tree2->isNodeFree(root2) && !obj1->isFree())
    {
      OBB<Scalar> obb1, obb2;
      convertBV(root1->bv, Transform3<Scalar>::Identity(), obb1);
      convertBV(root2_bv, tf2, obb2);

      if(obb1.overlap(obb2))
      {
        Box<Scalar>* box = new Box<Scalar>();
        Transform3<Scalar> box_tf;
        constructBox(root2_bv, tf2, *box, box_tf);

        box->cost_density = root2->getOccupancy();
        box->threshold_occupied = tree2->getOccupancyThres();

        CollisionObject<Scalar> obj2(std::shared_ptr<CollisionGeometry<Scalar>>(box), box_tf);
        return callback(obj1, &obj2, cdata);
      }
      else return false;
    }
    else return false;
  }

  OBB<Scalar> obb1, obb2;
  convertBV(root1->bv, Transform3<Scalar>::Identity(), obb1);
  convertBV(root2_bv, tf2, obb2);

  if(tree2->isNodeFree(root2) || !obb1.overlap(obb2)) return false;

  if(!tree2->nodeHasChildren(root2) || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size())))
  {
    if(collisionRecurse_(nodes1, root1->children[0], tree2, root2, root2_bv, tf2, cdata, callback))
      return true;
    if(collisionRecurse_(nodes1, root1->children[1], tree2, root2, root2_bv, tf2, cdata, callback))
      return true;
  }
  else
  {
    for(unsigned int i = 0; i < 8; ++i)
    {
      if(tree2->nodeChildExists(root2, i))
      {
        const typename OcTree<Scalar>::OcTreeNode* child = tree2->getNodeChild(root2, i);
        AABB<Scalar> child_bv;
        computeChildBV(root2_bv, i, child_bv);

        if(collisionRecurse_(nodes1, root1_id, tree2, child, child_bv, tf2, cdata, callback))
          return true;
      }
      else
      {
        AABB<Scalar> child_bv;
        computeChildBV(root2_bv, i, child_bv);
        if(collisionRecurse_<Scalar>(nodes1, root1_id, tree2, NULL, child_bv, tf2, cdata, callback))
          return true;
      }
    }
  }

  return false;
}

//==============================================================================
template <typename Scalar, typename Derived>
bool collisionRecurse_(
    typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* nodes1,
    size_t root1_id,
    const OcTree<Scalar>* tree2,
    const typename OcTree<Scalar>::OcTreeNode* root2,
    const AABB<Scalar>& root2_bv,
    const Eigen::MatrixBase<Derived>& translation2,
    void* cdata,
    CollisionCallBack<Scalar> callback)
{
  typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* root1 = nodes1 + root1_id;
  if(!root2)
  {
    if(root1->isLeaf())
    {
      CollisionObject<Scalar>* obj1 = static_cast<CollisionObject<Scalar>*>(root1->data);

      if(!obj1->isFree())
      {
        const AABB<Scalar>& root_bv_t = translate(root2_bv, translation2);
        if(root1->bv.overlap(root_bv_t))
        {
          Box<Scalar>* box = new Box<Scalar>();
          Transform3<Scalar> box_tf;
          Transform3<Scalar> tf2 = Transform3<Scalar>::Identity();
          tf2.translation() = translation2;
          constructBox(root2_bv, tf2, *box, box_tf);

          box->cost_density = tree2->getDefaultOccupancy();

          CollisionObject<Scalar> obj2(std::shared_ptr<CollisionGeometry<Scalar>>(box), box_tf);
          return callback(obj1, &obj2, cdata);
        }
      }
    }
    else
    {
      if(collisionRecurse_<Scalar>(nodes1, root1->children[0], tree2, NULL, root2_bv, translation2, cdata, callback))
        return true;
      if(collisionRecurse_<Scalar>(nodes1, root1->children[1], tree2, NULL, root2_bv, translation2, cdata, callback))
        return true;
    }

    return false;
  }
  else if(root1->isLeaf() && !tree2->nodeHasChildren(root2))
  {
    CollisionObject<Scalar>* obj1 = static_cast<CollisionObject<Scalar>*>(root1->data);
    if(!tree2->isNodeFree(root2) && !obj1->isFree())
    {
      const AABB<Scalar>& root_bv_t = translate(root2_bv, translation2);
      if(root1->bv.overlap(root_bv_t))
      {
        Box<Scalar>* box = new Box<Scalar>();
        Transform3<Scalar> box_tf;
        Transform3<Scalar> tf2 = Transform3<Scalar>::Identity();
        tf2.translation() = translation2;
        constructBox(root2_bv, tf2, *box, box_tf);

        box->cost_density = root2->getOccupancy();
        box->threshold_occupied = tree2->getOccupancyThres();

        CollisionObject<Scalar> obj2(std::shared_ptr<CollisionGeometry<Scalar>>(box), box_tf);
        return callback(obj1, &obj2, cdata);
      }
      else return false;
    }
    else return false;
  }

  const AABB<Scalar>& root_bv_t = translate(root2_bv, translation2);
  if(tree2->isNodeFree(root2) || !root1->bv.overlap(root_bv_t)) return false;

  if(!tree2->nodeHasChildren(root2) || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size())))
  {
    if(collisionRecurse_(nodes1, root1->children[0], tree2, root2, root2_bv, translation2, cdata, callback))
      return true;
    if(collisionRecurse_(nodes1, root1->children[1], tree2, root2, root2_bv, translation2, cdata, callback))
      return true;
  }
  else
  {
    for(unsigned int i = 0; i < 8; ++i)
    {
      if(tree2->nodeChildExists(root2, i))
      {
        const typename OcTree<Scalar>::OcTreeNode* child = tree2->getNodeChild(root2, i);
        AABB<Scalar> child_bv;
        computeChildBV(root2_bv, i, child_bv);

        if(collisionRecurse_(nodes1, root1_id, tree2, child, child_bv, translation2, cdata, callback))
          return true;
      }
      else
      {
        AABB<Scalar> child_bv;
        computeChildBV(root2_bv, i, child_bv);
        if(collisionRecurse_<Scalar>(nodes1, root1_id, tree2, NULL, child_bv, translation2, cdata, callback))
          return true;
      }
    }
  }

  return false;
}

//==============================================================================
template <typename Scalar>
bool distanceRecurse_(typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* nodes1, size_t root1_id, const OcTree<Scalar>* tree2, const typename OcTree<Scalar>::OcTreeNode* root2, const AABB<Scalar>& root2_bv, const Transform3<Scalar>& tf2, void* cdata, DistanceCallBack<Scalar> callback, Scalar& min_dist)
{
  typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* root1 = nodes1 + root1_id;
  if(root1->isLeaf() && !tree2->nodeHasChildren(root2))
  {
    if(tree2->isNodeOccupied(root2))
    {
      Box<Scalar>* box = new Box<Scalar>();
      Transform3<Scalar> box_tf;
      constructBox(root2_bv, tf2, *box, box_tf);
      CollisionObject<Scalar> obj(std::shared_ptr<CollisionGeometry<Scalar>>(box), box_tf);
      return callback(static_cast<CollisionObject<Scalar>*>(root1->data), &obj, cdata, min_dist);
    }
    else return false;
  }

  if(!tree2->isNodeOccupied(root2)) return false;

  if(!tree2->nodeHasChildren(root2) || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size())))
  {
    AABB<Scalar> aabb2;
    convertBV(root2_bv, tf2, aabb2);

    Scalar d1 = aabb2.distance((nodes1 + root1->children[0])->bv);
    Scalar d2 = aabb2.distance((nodes1 + root1->children[1])->bv);

    if(d2 < d1)
    {
      if(d2 < min_dist)
      {
        if(distanceRecurse_(nodes1, root1->children[1], tree2, root2, root2_bv, tf2, cdata, callback, min_dist))
          return true;
      }

      if(d1 < min_dist)
      {
        if(distanceRecurse_(nodes1, root1->children[0], tree2, root2, root2_bv, tf2, cdata, callback, min_dist))
          return true;
      }
    }
    else
    {
      if(d1 < min_dist)
      {
        if(distanceRecurse_(nodes1, root1->children[0], tree2, root2, root2_bv, tf2, cdata, callback, min_dist))
          return true;
      }

      if(d2 < min_dist)
      {
        if(distanceRecurse_(nodes1, root1->children[1], tree2, root2, root2_bv, tf2, cdata, callback, min_dist))
          return true;
      }
    }
  }
  else
  {
    for(unsigned int i = 0; i < 8; ++i)
    {
      if(tree2->nodeChildExists(root2, i))
      {
        const typename OcTree<Scalar>::OcTreeNode* child = tree2->getNodeChild(root2, i);
        AABB<Scalar> child_bv;
        computeChildBV(root2_bv, i, child_bv);

        AABB<Scalar> aabb2;
        convertBV(child_bv, tf2, aabb2);
        Scalar d = root1->bv.distance(aabb2);

        if(d < min_dist)
        {
          if(distanceRecurse_(nodes1, root1_id, tree2, child, child_bv, tf2, cdata, callback, min_dist))
            return true;
        }
      }
    }
  }

  return false;
}

//==============================================================================
template <typename Scalar, typename Derived>
bool distanceRecurse_(
    typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* nodes1,
    size_t root1_id,
    const OcTree<Scalar>* tree2,
    const typename OcTree<Scalar>::OcTreeNode* root2,
    const AABB<Scalar>& root2_bv,
    const Eigen::MatrixBase<Derived>& translation2,
    void* cdata,
    DistanceCallBack<Scalar> callback,
    Scalar& min_dist)
{
  typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* root1 = nodes1 + root1_id;
  if(root1->isLeaf() && !tree2->nodeHasChildren(root2))
  {
    if(tree2->isNodeOccupied(root2))
    {
      Box<Scalar>* box = new Box<Scalar>();
      Transform3<Scalar> box_tf;
      Transform3<Scalar> tf2 = Transform3<Scalar>::Identity();
      tf2.translation() = translation2;
      constructBox(root2_bv, tf2, *box, box_tf);
      CollisionObject<Scalar> obj(std::shared_ptr<CollisionGeometry<Scalar>>(box), box_tf);
      return callback(static_cast<CollisionObject<Scalar>*>(root1->data), &obj, cdata, min_dist);
    }
    else return false;
  }

  if(!tree2->isNodeOccupied(root2)) return false;

  if(!tree2->nodeHasChildren(root2) || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size())))
  {
    const AABB<Scalar>& aabb2 = translate(root2_bv, translation2);

    Scalar d1 = aabb2.distance((nodes1 + root1->children[0])->bv);
    Scalar d2 = aabb2.distance((nodes1 + root1->children[1])->bv);

    if(d2 < d1)
    {
      if(d2 < min_dist)
      {
        if(distanceRecurse_(nodes1, root1->children[1], tree2, root2, root2_bv, translation2, cdata, callback, min_dist))
          return true;
      }

      if(d1 < min_dist)
      {
        if(distanceRecurse_(nodes1, root1->children[0], tree2, root2, root2_bv, translation2, cdata, callback, min_dist))
          return true;
      }
    }
    else
    {
      if(d1 < min_dist)
      {
        if(distanceRecurse_(nodes1, root1->children[0], tree2, root2, root2_bv, translation2, cdata, callback, min_dist))
          return true;
      }

      if(d2 < min_dist)
      {
        if(distanceRecurse_(nodes1, root1->children[1], tree2, root2, root2_bv, translation2, cdata, callback, min_dist))
          return true;
      }
    }
  }
  else
  {
    for(unsigned int i = 0; i < 8; ++i)
    {
      if(tree2->nodeChildExists(root2, i))
      {
        const typename OcTree<Scalar>::OcTreeNode* child = tree2->getNodeChild(root2, i);
        AABB<Scalar> child_bv;
        computeChildBV(root2_bv, i, child_bv);

        const AABB<Scalar>& aabb2 = translate(child_bv, translation2);
        Scalar d = root1->bv.distance(aabb2);

        if(d < min_dist)
        {
          if(distanceRecurse_(nodes1, root1_id, tree2, child, child_bv, translation2, cdata, callback, min_dist))
            return true;
        }
      }
    }
  }

  return false;
}


#endif

//==============================================================================
template <typename Scalar>
bool collisionRecurse(typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* nodes1, size_t root1_id,
                      typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* nodes2, size_t root2_id,
                      void* cdata, CollisionCallBack<Scalar> callback)
{
  typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* root1 = nodes1 + root1_id;
  typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* root2 = nodes2 + root2_id;
  if(root1->isLeaf() && root2->isLeaf())
  {
    if(!root1->bv.overlap(root2->bv)) return false;
    return callback(static_cast<CollisionObject<Scalar>*>(root1->data), static_cast<CollisionObject<Scalar>*>(root2->data), cdata);
  }

  if(!root1->bv.overlap(root2->bv)) return false;

  if(root2->isLeaf() || (!root1->isLeaf() && (root1->bv.size() > root2->bv.size())))
  {
    if(collisionRecurse(nodes1, root1->children[0], nodes2, root2_id, cdata, callback))
      return true;
    if(collisionRecurse(nodes1, root1->children[1], nodes2, root2_id, cdata, callback))
      return true;
  }
  else
  {
    if(collisionRecurse(nodes1, root1_id, nodes2, root2->children[0], cdata, callback))
      return true;
    if(collisionRecurse(nodes1, root1_id, nodes2, root2->children[1], cdata, callback))
      return true;
  }
  return false;
}

//==============================================================================
template <typename Scalar>
bool collisionRecurse(typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* nodes, size_t root_id, CollisionObject<Scalar>* query, void* cdata, CollisionCallBack<Scalar> callback)
{
  typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* root = nodes + root_id;
  if(root->isLeaf())
  {
    if(!root->bv.overlap(query->getAABB())) return false;
    return callback(static_cast<CollisionObject<Scalar>*>(root->data), query, cdata);
  }

  if(!root->bv.overlap(query->getAABB())) return false;

  int select_res = implementation_array::select(query->getAABB(), root->children[0], root->children[1], nodes);

  if(collisionRecurse(nodes, root->children[select_res], query, cdata, callback))
    return true;

  if(collisionRecurse(nodes, root->children[1-select_res], query, cdata, callback))
    return true;

  return false;
}

//==============================================================================
template <typename Scalar>
bool selfCollisionRecurse(typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* nodes, size_t root_id, void* cdata, CollisionCallBack<Scalar> callback)
{
  typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* root = nodes + root_id;
  if(root->isLeaf()) return false;

  if(selfCollisionRecurse(nodes, root->children[0], cdata, callback))
    return true;

  if(selfCollisionRecurse(nodes, root->children[1], cdata, callback))
    return true;

  if(collisionRecurse(nodes, root->children[0], nodes, root->children[1], cdata, callback))
    return true;

  return false;
}

//==============================================================================
template <typename Scalar>
bool distanceRecurse(typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* nodes1, size_t root1_id,
                     typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* nodes2, size_t root2_id,
                     void* cdata, DistanceCallBack<Scalar> callback, Scalar& min_dist)
{
  typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* root1 = nodes1 + root1_id;
  typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* root2 = nodes2 + root2_id;
  if(root1->isLeaf() && root2->isLeaf())
  {
    CollisionObject<Scalar>* root1_obj = static_cast<CollisionObject<Scalar>*>(root1->data);
    CollisionObject<Scalar>* root2_obj = static_cast<CollisionObject<Scalar>*>(root2->data);
    return callback(root1_obj, root2_obj, cdata, min_dist);
  }

  if(root2->isLeaf() || (!root1->isLeaf() && (root1->bv.size() > root2->bv.size())))
  {
    Scalar d1 = root2->bv.distance((nodes1 + root1->children[0])->bv);
    Scalar d2 = root2->bv.distance((nodes1 + root1->children[1])->bv);

    if(d2 < d1)
    {
      if(d2 < min_dist)
      {
        if(distanceRecurse(nodes1, root1->children[1], nodes2, root2_id, cdata, callback, min_dist))
          return true;
      }

      if(d1 < min_dist)
      {
        if(distanceRecurse(nodes1, root1->children[0], nodes2, root2_id, cdata, callback, min_dist))
          return true;
      }
    }
    else
    {
      if(d1 < min_dist)
      {
        if(distanceRecurse(nodes1, root1->children[0], nodes2, root2_id, cdata, callback, min_dist))
          return true;
      }

      if(d2 < min_dist)
      {
        if(distanceRecurse(nodes1, root1->children[1], nodes2, root2_id, cdata, callback, min_dist))
          return true;
      }
    }
  }
  else
  {
    Scalar d1 = root1->bv.distance((nodes2 + root2->children[0])->bv);
    Scalar d2 = root1->bv.distance((nodes2 + root2->children[1])->bv);

    if(d2 < d1)
    {
      if(d2 < min_dist)
      {
        if(distanceRecurse(nodes1, root1_id, nodes2, root2->children[1], cdata, callback, min_dist))
          return true;
      }

      if(d1 < min_dist)
      {
        if(distanceRecurse(nodes1, root1_id, nodes2, root2->children[0], cdata, callback, min_dist))
          return true;
      }
    }
    else
    {
      if(d1 < min_dist)
      {
        if(distanceRecurse(nodes1, root1_id, nodes2, root2->children[0], cdata, callback, min_dist))
          return true;
      }

      if(d2 < min_dist)
      {
        if(distanceRecurse(nodes1, root1_id, nodes2, root2->children[1], cdata, callback, min_dist))
          return true;
      }
    }
  }

  return false;
}

//==============================================================================
template <typename Scalar>
bool distanceRecurse(typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* nodes, size_t root_id, CollisionObject<Scalar>* query, void* cdata, DistanceCallBack<Scalar> callback, Scalar& min_dist)
{
  typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* root = nodes + root_id;
  if(root->isLeaf())
  {
    CollisionObject<Scalar>* root_obj = static_cast<CollisionObject<Scalar>*>(root->data);
    return callback(root_obj, query, cdata, min_dist);
  }

  Scalar d1 = query->getAABB().distance((nodes + root->children[0])->bv);
  Scalar d2 = query->getAABB().distance((nodes + root->children[1])->bv);

  if(d2 < d1)
  {
    if(d2 < min_dist)
    {
      if(distanceRecurse(nodes, root->children[1], query, cdata, callback, min_dist))
        return true;
    }

    if(d1 < min_dist)
    {
      if(distanceRecurse(nodes, root->children[0], query, cdata, callback, min_dist))
        return true;
    }
  }
  else
  {
    if(d1 < min_dist)
    {
      if(distanceRecurse(nodes, root->children[0], query, cdata, callback, min_dist))
        return true;
    }

    if(d2 < min_dist)
    {
      if(distanceRecurse(nodes, root->children[1], query, cdata, callback, min_dist))
        return true;
    }
  }

  return false;
}

//==============================================================================
template <typename Scalar>
bool selfDistanceRecurse(typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* nodes, size_t root_id, void* cdata, DistanceCallBack<Scalar> callback, Scalar& min_dist)
{
  typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* root = nodes + root_id;
  if(root->isLeaf()) return false;

  if(selfDistanceRecurse(nodes, root->children[0], cdata, callback, min_dist))
    return true;

  if(selfDistanceRecurse(nodes, root->children[1], cdata, callback, min_dist))
    return true;

  if(distanceRecurse(nodes, root->children[0], nodes, root->children[1], cdata, callback, min_dist))
    return true;

  return false;
}


#if FCL_HAVE_OCTOMAP

//==============================================================================
template <typename Scalar>
bool collisionRecurse(typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* nodes1, size_t root1_id, const OcTree<Scalar>* tree2, const typename OcTree<Scalar>::OcTreeNode* root2, const AABB<Scalar>& root2_bv, const Transform3<Scalar>& tf2, void* cdata, CollisionCallBack<Scalar> callback)
{
  if(tf2.linear().isIdentity())
    return collisionRecurse_(nodes1, root1_id, tree2, root2, root2_bv, tf2.translation(), cdata, callback);
  else
    return collisionRecurse_(nodes1, root1_id, tree2, root2, root2_bv, tf2, cdata, callback);
}

//==============================================================================
template <typename Scalar>
bool distanceRecurse(typename DynamicAABBTreeCollisionManager_Array<Scalar>::DynamicAABBNode* nodes1, size_t root1_id, const OcTree<Scalar>* tree2, const typename OcTree<Scalar>::OcTreeNode* root2, const AABB<Scalar>& root2_bv, const Transform3<Scalar>& tf2, void* cdata, DistanceCallBack<Scalar> callback, Scalar& min_dist)
{
  if(tf2.linear().isIdentity())
    return distanceRecurse_(nodes1, root1_id, tree2, root2, root2_bv, tf2.translation(), cdata, callback, min_dist);
  else
    return distanceRecurse_(nodes1, root1_id, tree2, root2, root2_bv, tf2, cdata, callback, min_dist);
}

#endif

} // dynamic_AABB_tree_array

} // details

//==============================================================================
template <typename Scalar>
void DynamicAABBTreeCollisionManager_Array<Scalar>::registerObjects(const std::vector<CollisionObject<Scalar>*>& other_objs)
{
  if(other_objs.empty()) return;

  if(size() > 0)
  {
    BroadPhaseCollisionManager<Scalar>::registerObjects(other_objs);
  }
  else
  {
    DynamicAABBNode* leaves = new DynamicAABBNode[other_objs.size()];
    table.rehash(other_objs.size());
    for(size_t i = 0, size = other_objs.size(); i < size; ++i)
    {
      leaves[i].bv = other_objs[i]->getAABB();
      leaves[i].parent = dtree.NULL_NODE;
      leaves[i].children[1] = dtree.NULL_NODE;
      leaves[i].data = other_objs[i];
      table[other_objs[i]] = i;
    }

    int n_leaves = other_objs.size();

    dtree.init(leaves, n_leaves, tree_init_level);

    setup_ = true;
  }
}

//==============================================================================
template <typename Scalar>
void DynamicAABBTreeCollisionManager_Array<Scalar>::registerObject(CollisionObject<Scalar>* obj)
{
  size_t node = dtree.insert(obj->getAABB(), obj);
  table[obj] = node;
}

//==============================================================================
template <typename Scalar>
void DynamicAABBTreeCollisionManager_Array<Scalar>::unregisterObject(CollisionObject<Scalar>* obj)
{
  size_t node = table[obj];
  table.erase(obj);
  dtree.remove(node);
}

//==============================================================================
template <typename Scalar>
void DynamicAABBTreeCollisionManager_Array<Scalar>::setup()
{
  if(!setup_)
  {
    int num = dtree.size();
    if(num == 0)
    {
      setup_ = true;
      return;
    }

    int height = dtree.getMaxHeight();


    if(height - std::log((Scalar)num) / std::log(2.0) < max_tree_nonbalanced_level)
      dtree.balanceIncremental(tree_incremental_balance_pass);
    else
      dtree.balanceTopdown();

    setup_ = true;
  }
}

//==============================================================================
template <typename Scalar>
void DynamicAABBTreeCollisionManager_Array<Scalar>::update()
{
  for(auto it = table.cbegin(), end = table.cend(); it != end; ++it)
  {
    const CollisionObject<Scalar>* obj = it->first;
    size_t node = it->second;
    dtree.getNodes()[node].bv = obj->getAABB();
  }

  dtree.refit();
  setup_ = false;

  setup();
}

//==============================================================================
template <typename Scalar>
void DynamicAABBTreeCollisionManager_Array<Scalar>::update_(CollisionObject<Scalar>* updated_obj)
{
  const auto it = table.find(updated_obj);
  if(it != table.end())
  {
    size_t node = it->second;
    if(!dtree.getNodes()[node].bv.equal(updated_obj->getAABB()))
      dtree.update(node, updated_obj->getAABB());
  }
  setup_ = false;
}

//==============================================================================
template <typename Scalar>
void DynamicAABBTreeCollisionManager_Array<Scalar>::update(CollisionObject<Scalar>* updated_obj)
{
  update_(updated_obj);
  setup();
}

//==============================================================================
template <typename Scalar>
void DynamicAABBTreeCollisionManager_Array<Scalar>::update(const std::vector<CollisionObject<Scalar>*>& updated_objs)
{
  for(size_t i = 0, size = updated_objs.size(); i < size; ++i)
    update_(updated_objs[i]);
  setup();
}

//==============================================================================
template <typename Scalar>
void DynamicAABBTreeCollisionManager_Array<Scalar>::collide(CollisionObject<Scalar>* obj, void* cdata, CollisionCallBack<Scalar> callback) const
{
  if(size() == 0) return;
  switch(obj->collisionGeometry()->getNodeType())
  {
#if FCL_HAVE_OCTOMAP
  case GEOM_OCTREE:
    {
      if(!octree_as_geometry_collide)
      {
        const OcTree<Scalar>* octree = static_cast<const OcTree<Scalar>*>(obj->collisionGeometry().get());
        details::dynamic_AABB_tree_array::collisionRecurse(dtree.getNodes(), dtree.getRoot(), octree, octree->getRoot(), octree->getRootBV(), obj->getTransform(), cdata, callback);
      }
      else
        details::dynamic_AABB_tree_array::collisionRecurse(dtree.getNodes(), dtree.getRoot(), obj, cdata, callback);
    }
    break;
#endif
  default:
    details::dynamic_AABB_tree_array::collisionRecurse(dtree.getNodes(), dtree.getRoot(), obj, cdata, callback);
  }
}

//==============================================================================
template <typename Scalar>
void DynamicAABBTreeCollisionManager_Array<Scalar>::distance(CollisionObject<Scalar>* obj, void* cdata, DistanceCallBack<Scalar> callback) const
{
  if(size() == 0) return;
  Scalar min_dist = std::numeric_limits<Scalar>::max();
  switch(obj->collisionGeometry()->getNodeType())
  {
#if FCL_HAVE_OCTOMAP
  case GEOM_OCTREE:
    {
      if(!octree_as_geometry_distance)
      {
        const OcTree<Scalar>* octree = static_cast<const OcTree<Scalar>*>(obj->collisionGeometry().get());
        details::dynamic_AABB_tree_array::distanceRecurse(dtree.getNodes(), dtree.getRoot(), octree, octree->getRoot(), octree->getRootBV(), obj->getTransform(), cdata, callback, min_dist);
      }
      else
        details::dynamic_AABB_tree_array::distanceRecurse(dtree.getNodes(), dtree.getRoot(), obj, cdata, callback, min_dist);
    }
    break;
#endif
  default:
    details::dynamic_AABB_tree_array::distanceRecurse(dtree.getNodes(), dtree.getRoot(), obj, cdata, callback, min_dist);
  }
}

//==============================================================================
template <typename Scalar>
void DynamicAABBTreeCollisionManager_Array<Scalar>::collide(void* cdata, CollisionCallBack<Scalar> callback) const
{
  if(size() == 0) return;
  details::dynamic_AABB_tree_array::selfCollisionRecurse(dtree.getNodes(), dtree.getRoot(), cdata, callback);
}

//==============================================================================
template <typename Scalar>
void DynamicAABBTreeCollisionManager_Array<Scalar>::distance(void* cdata, DistanceCallBack<Scalar> callback) const
{
  if(size() == 0) return;
  Scalar min_dist = std::numeric_limits<Scalar>::max();
  details::dynamic_AABB_tree_array::selfDistanceRecurse(dtree.getNodes(), dtree.getRoot(), cdata, callback, min_dist);
}

//==============================================================================
template <typename Scalar>
void DynamicAABBTreeCollisionManager_Array<Scalar>::collide(BroadPhaseCollisionManager<Scalar>* other_manager_, void* cdata, CollisionCallBack<Scalar> callback) const
{
  DynamicAABBTreeCollisionManager_Array* other_manager = static_cast<DynamicAABBTreeCollisionManager_Array*>(other_manager_);
  if((size() == 0) || (other_manager->size() == 0)) return;
  details::dynamic_AABB_tree_array::collisionRecurse(dtree.getNodes(), dtree.getRoot(), other_manager->dtree.getNodes(), other_manager->dtree.getRoot(), cdata, callback);
}

//==============================================================================
template <typename Scalar>
void DynamicAABBTreeCollisionManager_Array<Scalar>::distance(BroadPhaseCollisionManager<Scalar>* other_manager_, void* cdata, DistanceCallBack<Scalar> callback) const
{
  DynamicAABBTreeCollisionManager_Array* other_manager = static_cast<DynamicAABBTreeCollisionManager_Array*>(other_manager_);
  if((size() == 0) || (other_manager->size() == 0)) return;
  Scalar min_dist = std::numeric_limits<Scalar>::max();
  details::dynamic_AABB_tree_array::distanceRecurse(dtree.getNodes(), dtree.getRoot(), other_manager->dtree.getNodes(), other_manager->dtree.getRoot(), cdata, callback, min_dist);
}

} // namespace fcl

#endif

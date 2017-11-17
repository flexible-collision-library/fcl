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

#ifndef FCL_BROAD_PHASE_DYNAMIC_AABB_TREE_ARRAY_INL_H
#define FCL_BROAD_PHASE_DYNAMIC_AABB_TREE_ARRAY_INL_H

#include "fcl/broadphase/broadphase_dynamic_AABB_tree_array.h"

#if FCL_HAVE_OCTOMAP
#include "fcl/geometry/octree/octree.h"
#endif

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT DynamicAABBTreeCollisionManager_Array<double>;

namespace detail
{

namespace dynamic_AABB_tree_array
{

#if FCL_HAVE_OCTOMAP

//==============================================================================
template <typename S>
FCL_EXPORT
bool collisionRecurse_(
    typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* nodes1,
    size_t root1_id,
    const OcTree<S>* tree2,
    const typename OcTree<S>::OcTreeNode* root2,
    const AABB<S>& root2_bv,
    const Transform3<S>& tf2,
    void* cdata,
    CollisionCallBack<S> callback)
{
  typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* root1 = nodes1 + root1_id;
  if(!root2)
  {
    if(root1->isLeaf())
    {
      CollisionObject<S>* obj1 = static_cast<CollisionObject<S>*>(root1->data);

      if(!obj1->isFree())
      {
        OBB<S> obb1, obb2;
        convertBV(root1->bv, Transform3<S>::Identity(), obb1);
        convertBV(root2_bv, tf2, obb2);

        if(obb1.overlap(obb2))
        {
          Box<S>* box = new Box<S>();
          Transform3<S> box_tf;
          constructBox(root2_bv, tf2, *box, box_tf);

          box->cost_density = tree2->getDefaultOccupancy();

          CollisionObject<S> obj2(std::shared_ptr<CollisionGeometry<S>>(box), box_tf);
          return callback(obj1, &obj2, cdata);
        }
      }
    }
    else
    {
      if(collisionRecurse_<S>(nodes1, root1->children[0], tree2, nullptr, root2_bv, tf2, cdata, callback))
        return true;
      if(collisionRecurse_<S>(nodes1, root1->children[1], tree2, nullptr, root2_bv, tf2, cdata, callback))
        return true;
    }

    return false;
  }
  else if(root1->isLeaf() && !tree2->nodeHasChildren(root2))
  {
    CollisionObject<S>* obj1 = static_cast<CollisionObject<S>*>(root1->data);
    if(!tree2->isNodeFree(root2) && !obj1->isFree())
    {
      OBB<S> obb1, obb2;
      convertBV(root1->bv, Transform3<S>::Identity(), obb1);
      convertBV(root2_bv, tf2, obb2);

      if(obb1.overlap(obb2))
      {
        Box<S>* box = new Box<S>();
        Transform3<S> box_tf;
        constructBox(root2_bv, tf2, *box, box_tf);

        box->cost_density = root2->getOccupancy();
        box->threshold_occupied = tree2->getOccupancyThres();

        CollisionObject<S> obj2(std::shared_ptr<CollisionGeometry<S>>(box), box_tf);
        return callback(obj1, &obj2, cdata);
      }
      else return false;
    }
    else return false;
  }

  OBB<S> obb1, obb2;
  convertBV(root1->bv, Transform3<S>::Identity(), obb1);
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
        const typename OcTree<S>::OcTreeNode* child = tree2->getNodeChild(root2, i);
        AABB<S> child_bv;
        computeChildBV(root2_bv, i, child_bv);

        if(collisionRecurse_(nodes1, root1_id, tree2, child, child_bv, tf2, cdata, callback))
          return true;
      }
      else
      {
        AABB<S> child_bv;
        computeChildBV(root2_bv, i, child_bv);
        if(collisionRecurse_<S>(nodes1, root1_id, tree2, nullptr, child_bv, tf2, cdata, callback))
          return true;
      }
    }
  }

  return false;
}

//==============================================================================
template <typename S, typename Derived>
FCL_EXPORT
bool collisionRecurse_(
    typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* nodes1,
    size_t root1_id,
    const OcTree<S>* tree2,
    const typename OcTree<S>::OcTreeNode* root2,
    const AABB<S>& root2_bv,
    const Eigen::MatrixBase<Derived>& translation2,
    void* cdata,
    CollisionCallBack<S> callback)
{
  typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* root1 = nodes1 + root1_id;
  if(!root2)
  {
    if(root1->isLeaf())
    {
      CollisionObject<S>* obj1 = static_cast<CollisionObject<S>*>(root1->data);

      if(!obj1->isFree())
      {
        const AABB<S>& root_bv_t = translate(root2_bv, translation2);
        if(root1->bv.overlap(root_bv_t))
        {
          Box<S>* box = new Box<S>();
          Transform3<S> box_tf;
          Transform3<S> tf2 = Transform3<S>::Identity();
          tf2.translation() = translation2;
          constructBox(root2_bv, tf2, *box, box_tf);

          box->cost_density = tree2->getDefaultOccupancy();

          CollisionObject<S> obj2(std::shared_ptr<CollisionGeometry<S>>(box), box_tf);
          return callback(obj1, &obj2, cdata);
        }
      }
    }
    else
    {
      if(collisionRecurse_<S>(nodes1, root1->children[0], tree2, nullptr, root2_bv, translation2, cdata, callback))
        return true;
      if(collisionRecurse_<S>(nodes1, root1->children[1], tree2, nullptr, root2_bv, translation2, cdata, callback))
        return true;
    }

    return false;
  }
  else if(root1->isLeaf() && !tree2->nodeHasChildren(root2))
  {
    CollisionObject<S>* obj1 = static_cast<CollisionObject<S>*>(root1->data);
    if(!tree2->isNodeFree(root2) && !obj1->isFree())
    {
      const AABB<S>& root_bv_t = translate(root2_bv, translation2);
      if(root1->bv.overlap(root_bv_t))
      {
        Box<S>* box = new Box<S>();
        Transform3<S> box_tf;
        Transform3<S> tf2 = Transform3<S>::Identity();
        tf2.translation() = translation2;
        constructBox(root2_bv, tf2, *box, box_tf);

        box->cost_density = root2->getOccupancy();
        box->threshold_occupied = tree2->getOccupancyThres();

        CollisionObject<S> obj2(std::shared_ptr<CollisionGeometry<S>>(box), box_tf);
        return callback(obj1, &obj2, cdata);
      }
      else return false;
    }
    else return false;
  }

  const AABB<S>& root_bv_t = translate(root2_bv, translation2);
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
        const typename OcTree<S>::OcTreeNode* child = tree2->getNodeChild(root2, i);
        AABB<S> child_bv;
        computeChildBV(root2_bv, i, child_bv);

        if(collisionRecurse_(nodes1, root1_id, tree2, child, child_bv, translation2, cdata, callback))
          return true;
      }
      else
      {
        AABB<S> child_bv;
        computeChildBV(root2_bv, i, child_bv);
        if(collisionRecurse_<S>(nodes1, root1_id, tree2, nullptr, child_bv, translation2, cdata, callback))
          return true;
      }
    }
  }

  return false;
}

//==============================================================================
template <typename S>
FCL_EXPORT
bool distanceRecurse_(typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* nodes1, size_t root1_id, const OcTree<S>* tree2, const typename OcTree<S>::OcTreeNode* root2, const AABB<S>& root2_bv, const Transform3<S>& tf2, void* cdata, DistanceCallBack<S> callback, S& min_dist)
{
  typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* root1 = nodes1 + root1_id;
  if(root1->isLeaf() && !tree2->nodeHasChildren(root2))
  {
    if(tree2->isNodeOccupied(root2))
    {
      Box<S>* box = new Box<S>();
      Transform3<S> box_tf;
      constructBox(root2_bv, tf2, *box, box_tf);
      CollisionObject<S> obj(std::shared_ptr<CollisionGeometry<S>>(box), box_tf);
      return callback(static_cast<CollisionObject<S>*>(root1->data), &obj, cdata, min_dist);
    }
    else return false;
  }

  if(!tree2->isNodeOccupied(root2)) return false;

  if(!tree2->nodeHasChildren(root2) || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size())))
  {
    AABB<S> aabb2;
    convertBV(root2_bv, tf2, aabb2);

    S d1 = aabb2.distance((nodes1 + root1->children[0])->bv);
    S d2 = aabb2.distance((nodes1 + root1->children[1])->bv);

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
        const typename OcTree<S>::OcTreeNode* child = tree2->getNodeChild(root2, i);
        AABB<S> child_bv;
        computeChildBV(root2_bv, i, child_bv);

        AABB<S> aabb2;
        convertBV(child_bv, tf2, aabb2);
        S d = root1->bv.distance(aabb2);

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
template <typename S, typename Derived>
FCL_EXPORT
bool distanceRecurse_(
    typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* nodes1,
    size_t root1_id,
    const OcTree<S>* tree2,
    const typename OcTree<S>::OcTreeNode* root2,
    const AABB<S>& root2_bv,
    const Eigen::MatrixBase<Derived>& translation2,
    void* cdata,
    DistanceCallBack<S> callback,
    S& min_dist)
{
  typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* root1 = nodes1 + root1_id;
  if(root1->isLeaf() && !tree2->nodeHasChildren(root2))
  {
    if(tree2->isNodeOccupied(root2))
    {
      Box<S>* box = new Box<S>();
      Transform3<S> box_tf;
      Transform3<S> tf2 = Transform3<S>::Identity();
      tf2.translation() = translation2;
      constructBox(root2_bv, tf2, *box, box_tf);
      CollisionObject<S> obj(std::shared_ptr<CollisionGeometry<S>>(box), box_tf);
      return callback(static_cast<CollisionObject<S>*>(root1->data), &obj, cdata, min_dist);
    }
    else return false;
  }

  if(!tree2->isNodeOccupied(root2)) return false;

  if(!tree2->nodeHasChildren(root2) || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size())))
  {
    const AABB<S>& aabb2 = translate(root2_bv, translation2);

    S d1 = aabb2.distance((nodes1 + root1->children[0])->bv);
    S d2 = aabb2.distance((nodes1 + root1->children[1])->bv);

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
        const typename OcTree<S>::OcTreeNode* child = tree2->getNodeChild(root2, i);
        AABB<S> child_bv;
        computeChildBV(root2_bv, i, child_bv);

        const AABB<S>& aabb2 = translate(child_bv, translation2);
        S d = root1->bv.distance(aabb2);

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
template <typename S>
FCL_EXPORT
bool collisionRecurse(typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* nodes1, size_t root1_id,
                      typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* nodes2, size_t root2_id,
                      void* cdata, CollisionCallBack<S> callback)
{
  typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* root1 = nodes1 + root1_id;
  typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* root2 = nodes2 + root2_id;
  if(root1->isLeaf() && root2->isLeaf())
  {
    if(!root1->bv.overlap(root2->bv)) return false;
    return callback(static_cast<CollisionObject<S>*>(root1->data), static_cast<CollisionObject<S>*>(root2->data), cdata);
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
template <typename S>
FCL_EXPORT
bool collisionRecurse(typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* nodes, size_t root_id, CollisionObject<S>* query, void* cdata, CollisionCallBack<S> callback)
{
  typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* root = nodes + root_id;
  if(root->isLeaf())
  {
    if(!root->bv.overlap(query->getAABB())) return false;
    return callback(static_cast<CollisionObject<S>*>(root->data), query, cdata);
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
template <typename S>
FCL_EXPORT
bool selfCollisionRecurse(typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* nodes, size_t root_id, void* cdata, CollisionCallBack<S> callback)
{
  typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* root = nodes + root_id;
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
template <typename S>
FCL_EXPORT
bool distanceRecurse(typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* nodes1, size_t root1_id,
                     typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* nodes2, size_t root2_id,
                     void* cdata, DistanceCallBack<S> callback, S& min_dist)
{
  typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* root1 = nodes1 + root1_id;
  typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* root2 = nodes2 + root2_id;
  if(root1->isLeaf() && root2->isLeaf())
  {
    CollisionObject<S>* root1_obj = static_cast<CollisionObject<S>*>(root1->data);
    CollisionObject<S>* root2_obj = static_cast<CollisionObject<S>*>(root2->data);
    return callback(root1_obj, root2_obj, cdata, min_dist);
  }

  if(root2->isLeaf() || (!root1->isLeaf() && (root1->bv.size() > root2->bv.size())))
  {
    S d1 = root2->bv.distance((nodes1 + root1->children[0])->bv);
    S d2 = root2->bv.distance((nodes1 + root1->children[1])->bv);

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
    S d1 = root1->bv.distance((nodes2 + root2->children[0])->bv);
    S d2 = root1->bv.distance((nodes2 + root2->children[1])->bv);

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
template <typename S>
FCL_EXPORT
bool distanceRecurse(typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* nodes, size_t root_id, CollisionObject<S>* query, void* cdata, DistanceCallBack<S> callback, S& min_dist)
{
  typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* root = nodes + root_id;
  if(root->isLeaf())
  {
    CollisionObject<S>* root_obj = static_cast<CollisionObject<S>*>(root->data);
    return callback(root_obj, query, cdata, min_dist);
  }

  S d1 = query->getAABB().distance((nodes + root->children[0])->bv);
  S d2 = query->getAABB().distance((nodes + root->children[1])->bv);

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
template <typename S>
FCL_EXPORT
bool selfDistanceRecurse(typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* nodes, size_t root_id, void* cdata, DistanceCallBack<S> callback, S& min_dist)
{
  typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* root = nodes + root_id;
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
template <typename S>
FCL_EXPORT
bool collisionRecurse(typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* nodes1, size_t root1_id, const OcTree<S>* tree2, const typename OcTree<S>::OcTreeNode* root2, const AABB<S>& root2_bv, const Transform3<S>& tf2, void* cdata, CollisionCallBack<S> callback)
{
  if(tf2.linear().isIdentity())
    return collisionRecurse_(nodes1, root1_id, tree2, root2, root2_bv, tf2.translation(), cdata, callback);
  else
    return collisionRecurse_(nodes1, root1_id, tree2, root2, root2_bv, tf2, cdata, callback);
}

//==============================================================================
template <typename S>
FCL_EXPORT
bool distanceRecurse(typename DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBNode* nodes1, size_t root1_id, const OcTree<S>* tree2, const typename OcTree<S>::OcTreeNode* root2, const AABB<S>& root2_bv, const Transform3<S>& tf2, void* cdata, DistanceCallBack<S> callback, S& min_dist)
{
  if(tf2.linear().isIdentity())
    return distanceRecurse_(nodes1, root1_id, tree2, root2, root2_bv, tf2.translation(), cdata, callback, min_dist);
  else
    return distanceRecurse_(nodes1, root1_id, tree2, root2, root2_bv, tf2, cdata, callback, min_dist);
}

#endif

} // namespace dynamic_AABB_tree_array

} // namespace detail

//==============================================================================
template <typename S>
FCL_EXPORT
DynamicAABBTreeCollisionManager_Array<S>::DynamicAABBTreeCollisionManager_Array()
  : tree_topdown_balance_threshold(dtree.bu_threshold),
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

//==============================================================================
template <typename S>
FCL_EXPORT
void DynamicAABBTreeCollisionManager_Array<S>::registerObjects(
    const std::vector<CollisionObject<S>*>& other_objs)
{
  if(other_objs.empty()) return;

  if(size() > 0)
  {
    BroadPhaseCollisionManager<S>::registerObjects(other_objs);
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
template <typename S>
FCL_EXPORT
void DynamicAABBTreeCollisionManager_Array<S>::registerObject(CollisionObject<S>* obj)
{
  size_t node = dtree.insert(obj->getAABB(), obj);
  table[obj] = node;
}

//==============================================================================
template <typename S>
FCL_EXPORT
void DynamicAABBTreeCollisionManager_Array<S>::unregisterObject(CollisionObject<S>* obj)
{
  size_t node = table[obj];
  table.erase(obj);
  dtree.remove(node);
}

//==============================================================================
template <typename S>
FCL_EXPORT
void DynamicAABBTreeCollisionManager_Array<S>::setup()
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


    if(height - std::log((S)num) / std::log(2.0) < max_tree_nonbalanced_level)
      dtree.balanceIncremental(tree_incremental_balance_pass);
    else
      dtree.balanceTopdown();

    setup_ = true;
  }
}

//==============================================================================
template <typename S>
FCL_EXPORT
void DynamicAABBTreeCollisionManager_Array<S>::update()
{
  for(auto it = table.cbegin(), end = table.cend(); it != end; ++it)
  {
    const CollisionObject<S>* obj = it->first;
    size_t node = it->second;
    dtree.getNodes()[node].bv = obj->getAABB();
  }

  dtree.refit();
  setup_ = false;

  setup();
}

//==============================================================================
template <typename S>
FCL_EXPORT
void DynamicAABBTreeCollisionManager_Array<S>::update_(CollisionObject<S>* updated_obj)
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
template <typename S>
FCL_EXPORT
void DynamicAABBTreeCollisionManager_Array<S>::update(CollisionObject<S>* updated_obj)
{
  update_(updated_obj);
  setup();
}

//==============================================================================
template <typename S>
FCL_EXPORT
void DynamicAABBTreeCollisionManager_Array<S>::update(const std::vector<CollisionObject<S>*>& updated_objs)
{
  for(size_t i = 0, size = updated_objs.size(); i < size; ++i)
    update_(updated_objs[i]);
  setup();
}

//==============================================================================
template <typename S>
FCL_EXPORT
void DynamicAABBTreeCollisionManager_Array<S>::clear()
{
  dtree.clear();
  table.clear();
}

//==============================================================================
template <typename S>
FCL_EXPORT
void DynamicAABBTreeCollisionManager_Array<S>::getObjects(std::vector<CollisionObject<S>*>& objs) const
{
  objs.resize(this->size());
  std::transform(table.begin(), table.end(), objs.begin(), std::bind(&DynamicAABBTable::value_type::first, std::placeholders::_1));
}

//==============================================================================
template <typename S>
FCL_EXPORT
void DynamicAABBTreeCollisionManager_Array<S>::collide(CollisionObject<S>* obj, void* cdata, CollisionCallBack<S> callback) const
{
  if(size() == 0) return;
  switch(obj->collisionGeometry()->getNodeType())
  {
#if FCL_HAVE_OCTOMAP
  case GEOM_OCTREE:
    {
      if(!octree_as_geometry_collide)
      {
        const OcTree<S>* octree = static_cast<const OcTree<S>*>(obj->collisionGeometry().get());
        detail::dynamic_AABB_tree_array::collisionRecurse(dtree.getNodes(), dtree.getRoot(), octree, octree->getRoot(), octree->getRootBV(), obj->getTransform(), cdata, callback);
      }
      else
        detail::dynamic_AABB_tree_array::collisionRecurse(dtree.getNodes(), dtree.getRoot(), obj, cdata, callback);
    }
    break;
#endif
  default:
    detail::dynamic_AABB_tree_array::collisionRecurse(dtree.getNodes(), dtree.getRoot(), obj, cdata, callback);
  }
}

//==============================================================================
template <typename S>
FCL_EXPORT
void DynamicAABBTreeCollisionManager_Array<S>::distance(CollisionObject<S>* obj, void* cdata, DistanceCallBack<S> callback) const
{
  if(size() == 0) return;
  S min_dist = std::numeric_limits<S>::max();
  switch(obj->collisionGeometry()->getNodeType())
  {
#if FCL_HAVE_OCTOMAP
  case GEOM_OCTREE:
    {
      if(!octree_as_geometry_distance)
      {
        const OcTree<S>* octree = static_cast<const OcTree<S>*>(obj->collisionGeometry().get());
        detail::dynamic_AABB_tree_array::distanceRecurse(dtree.getNodes(), dtree.getRoot(), octree, octree->getRoot(), octree->getRootBV(), obj->getTransform(), cdata, callback, min_dist);
      }
      else
        detail::dynamic_AABB_tree_array::distanceRecurse(dtree.getNodes(), dtree.getRoot(), obj, cdata, callback, min_dist);
    }
    break;
#endif
  default:
    detail::dynamic_AABB_tree_array::distanceRecurse(dtree.getNodes(), dtree.getRoot(), obj, cdata, callback, min_dist);
  }
}

//==============================================================================
template <typename S>
FCL_EXPORT
void DynamicAABBTreeCollisionManager_Array<S>::collide(void* cdata, CollisionCallBack<S> callback) const
{
  if(size() == 0) return;
  detail::dynamic_AABB_tree_array::selfCollisionRecurse(dtree.getNodes(), dtree.getRoot(), cdata, callback);
}

//==============================================================================
template <typename S>
FCL_EXPORT
void DynamicAABBTreeCollisionManager_Array<S>::distance(void* cdata, DistanceCallBack<S> callback) const
{
  if(size() == 0) return;
  S min_dist = std::numeric_limits<S>::max();
  detail::dynamic_AABB_tree_array::selfDistanceRecurse(dtree.getNodes(), dtree.getRoot(), cdata, callback, min_dist);
}

//==============================================================================
template <typename S>
FCL_EXPORT
void DynamicAABBTreeCollisionManager_Array<S>::collide(BroadPhaseCollisionManager<S>* other_manager_, void* cdata, CollisionCallBack<S> callback) const
{
  DynamicAABBTreeCollisionManager_Array* other_manager = static_cast<DynamicAABBTreeCollisionManager_Array*>(other_manager_);
  if((size() == 0) || (other_manager->size() == 0)) return;
  detail::dynamic_AABB_tree_array::collisionRecurse(dtree.getNodes(), dtree.getRoot(), other_manager->dtree.getNodes(), other_manager->dtree.getRoot(), cdata, callback);
}

//==============================================================================
template <typename S>
FCL_EXPORT
void DynamicAABBTreeCollisionManager_Array<S>::distance(BroadPhaseCollisionManager<S>* other_manager_, void* cdata, DistanceCallBack<S> callback) const
{
  DynamicAABBTreeCollisionManager_Array* other_manager = static_cast<DynamicAABBTreeCollisionManager_Array*>(other_manager_);
  if((size() == 0) || (other_manager->size() == 0)) return;
  S min_dist = std::numeric_limits<S>::max();
  detail::dynamic_AABB_tree_array::distanceRecurse(dtree.getNodes(), dtree.getRoot(), other_manager->dtree.getNodes(), other_manager->dtree.getRoot(), cdata, callback, min_dist);
}

//==============================================================================
template <typename S>
FCL_EXPORT
bool DynamicAABBTreeCollisionManager_Array<S>::empty() const
{
  return dtree.empty();
}

//==============================================================================
template <typename S>
FCL_EXPORT
size_t DynamicAABBTreeCollisionManager_Array<S>::size() const
{
  return dtree.size();
}

//==============================================================================
template <typename S>
FCL_EXPORT
const detail::implementation_array::HierarchyTree<AABB<S>>&
DynamicAABBTreeCollisionManager_Array<S>::getTree() const
{
  return dtree;
}

} // namespace fcl

#endif

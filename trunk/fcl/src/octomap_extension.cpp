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

#include "fcl/octomap_extension.h"
#include "fcl/geometric_shapes.h"

namespace fcl
{

class ExtendedBox : public Box
{
public:
  ExtendedBox(FCL_REAL x, FCL_REAL y, FCL_REAL z) : Box(x, y, z) 
  {
    prob = 0;
    node = NULL;
  }
  
  FCL_REAL prob;
  octomap::OcTreeNode* node;
};

bool collisionRecurse(DynamicAABBTreeCollisionManager::DynamicAABBNode* root1, 
                      octomap::OcTree* tree2, octomap::OcTreeNode* root2, const AABB& root2_bv,
                      void* cdata, CollisionCallBack callback)
{
  if(root1->isLeaf() && !root2->hasChildren())
  {
    if(tree2->isNodeOccupied(root2))
    {
      if(root1->bv.overlap(root2_bv))
      {
        Box* box = new Box(root2_bv.max_[0] - root2_bv.min_[0], 
                           root2_bv.max_[1] - root2_bv.min_[1],
                           root2_bv.max_[2] - root2_bv.min_[2]);
        CollisionObject obj(boost::shared_ptr<CollisionGeometry>(box), SimpleTransform(root2_bv.center()));
        return callback(static_cast<CollisionObject*>(root1->data), &obj, cdata);
      }
      else return false;
    }
    else return false;
  }
  
  if(!tree2->isNodeOccupied(root2) || !root1->bv.overlap(root2_bv)) return false;

  if(!root2->hasChildren() || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size())))
  {
    if(collisionRecurse(root1->childs[0], tree2, root2, root2_bv, cdata, callback))
      return true;
    if(collisionRecurse(root1->childs[1], tree2, root2, root2_bv, cdata, callback))
      return true;
  }
  else
  {
    for(unsigned int i = 0; i < 8; ++i)
    {
      if(root2->childExists(i))
      {
        octomap::OcTreeNode* child = root2->getChild(i);
        AABB child_bv;
        if(i&1)
        {
          child_bv.min_[0] = (root2_bv.min_[0] + root2_bv.max_[0]) * 0.5;
          child_bv.max_[0] = root2_bv.max_[0];
        }
        else
        {
          child_bv.min_[0] = root2_bv.min_[0];
          child_bv.max_[0] = (root2_bv.min_[0] + root2_bv.max_[0]) * 0.5;
        }

        if(i&2)
        {
          child_bv.min_[1] = (root2_bv.min_[1] + root2_bv.max_[1]) * 0.5;
          child_bv.max_[1] = root2_bv.max_[1];
        }
        else
        {
          child_bv.min_[1] = root2_bv.min_[1];
          child_bv.max_[1] = (root2_bv.min_[1] + root2_bv.max_[1]) * 0.5;
        }

        if(i&4)
        {
          child_bv.min_[2] = (root2_bv.min_[2] + root2_bv.max_[2]) * 0.5;
          child_bv.max_[2] = root2_bv.max_[2];
        }        
        else
        {
          child_bv.min_[2] = root2_bv.min_[2];
          child_bv.max_[2] = (root2_bv.min_[2] + root2_bv.max_[2]) * 0.5;
        }

        if(collisionRecurse(root1, tree2, child, child_bv, cdata, callback))
          return true;
      }
    }
  }

  return false;
}

void collide(DynamicAABBTreeCollisionManager* manager, octomap::OcTree* octree, void* cdata, CollisionCallBack callback)
{
  DynamicAABBTreeCollisionManager::DynamicAABBNode* root1 = manager->getTree().getRoot();
  octomap::OcTreeNode* root2 = octree->getRoot();

  FCL_REAL delta = (1 << octree->getTreeDepth()) * octree->getResolution() / 2;
  
  collisionRecurse(root1, octree, root2, AABB(Vec3f(-delta, -delta, -delta), Vec3f(delta, delta, delta)), 
                   cdata, callback);
}


bool collisionCostRecurse(DynamicAABBTreeCollisionManager::DynamicAABBNode* root1, 
                          octomap::OcTree* tree2, octomap::OcTreeNode* root2, const AABB& root2_bv,
                          void* cdata, CollisionCostOctomapCallBack callback, FCL_REAL& cost)
{
  if(root1->isLeaf() && !root2->hasChildren())
  {
    if(!tree2->isNodeOccupied(root2))
    {
      if(root1->bv.overlap(root2_bv))
      {
        ExtendedBox* box = new ExtendedBox(root2_bv.max_[0] - root2_bv.min_[0], 
                                           root2_bv.max_[1] - root2_bv.min_[1],
                                           root2_bv.max_[2] - root2_bv.min_[2]);
        box->prob = root2->getOccupancy();
        box->node = root2;

        CollisionObject obj(boost::shared_ptr<CollisionGeometry>(box), SimpleTransform(root2_bv.center()));
        return callback(static_cast<CollisionObject*>(root1->data), &obj, cdata, cost);
      }
    }
  }

  if(tree2->isNodeOccupied(root2) || !root1->bv.overlap(root2_bv)) return false;

  if(!root2->hasChildren() || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size())))
  {
    if(collisionCostRecurse(root1->childs[0], tree2, root2, root2_bv, cdata, callback, cost))
      return true;
    if(collisionCostRecurse(root1->childs[1], tree2, root2, root2_bv, cdata, callback, cost))
      return true;
  }
  else
  {
    for(int i = 0; i < 8; ++i)
    {
      if(root2->childExists(i))
      {
        octomap::OcTreeNode* child = root2->getChild(i);
        AABB child_bv;
        if(i&1)
        {
          child_bv.min_[0] = (root2_bv.min_[0] + root2_bv.max_[0]) * 0.5;
          child_bv.max_[0] = root2_bv.max_[0];
        }
        else
        {
          child_bv.min_[0] = root2_bv.min_[0];
          child_bv.max_[0] = (root2_bv.min_[0] + root2_bv.max_[0]) * 0.5;
        }

        if(i&2)
        {
          child_bv.min_[1] = (root2_bv.min_[1] + root2_bv.max_[1]) * 0.5;
          child_bv.max_[1] = root2_bv.max_[1];
        }
        else
        {
          child_bv.min_[1] = root2_bv.min_[1];
          child_bv.max_[1] = (root2_bv.min_[1] + root2_bv.max_[1]) * 0.5;
        }

        if(i&4)
        {
          child_bv.min_[2] = (root2_bv.min_[2] + root2_bv.max_[2]) * 0.5;
          child_bv.max_[2] = root2_bv.max_[2];
        }        
        else
        {
          child_bv.min_[2] = root2_bv.min_[2];
          child_bv.max_[2] = (root2_bv.min_[2] + root2_bv.max_[2]) * 0.5;
        }

        if(collisionCostRecurse(root1, tree2, child, child_bv, cdata, callback, cost))
          return true;
      }
    }
  }

  return false;
}


bool collisionCostExtRecurse(DynamicAABBTreeCollisionManager::DynamicAABBNode* root1, 
                             octomap::OcTree* tree2, octomap::OcTreeNode* root2, const AABB& root2_bv,
                             void* cdata, CollisionCostOctomapCallBackExt callback, FCL_REAL& cost, std::set<OcTreeNode_AABB_pair>& nodes)
{
  if(root1->isLeaf() && !root2->hasChildren())
  {
    if(!tree2->isNodeOccupied(root2))
    {
      if(root1->bv.overlap(root2_bv))
      {
        ExtendedBox* box = new ExtendedBox(root2_bv.max_[0] - root2_bv.min_[0], 
                                                 root2_bv.max_[1] - root2_bv.min_[1],
                                                 root2_bv.max_[2] - root2_bv.min_[2]);

        box->prob = root2->getOccupancy();
        CollisionObject obj(boost::shared_ptr<CollisionGeometry>(box), SimpleTransform(root2_bv.center()));
        return callback(static_cast<CollisionObject*>(root1->data), &obj, cdata, cost, nodes);
      }
    }
  }

  if(tree2->isNodeOccupied(root2) || !root1->bv.overlap(root2_bv)) return false;

  if(!root2->hasChildren() || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size())))
  {
    if(collisionCostExtRecurse(root1->childs[0], tree2, root2, root2_bv, cdata, callback, cost, nodes))
      return true;
    if(collisionCostExtRecurse(root1->childs[1], tree2, root2, root2_bv, cdata, callback, cost, nodes))
      return true;
  }
  else
  {
    for(int i = 0; i < 8; ++i)
    {
      if(root2->childExists(i))
      {
        octomap::OcTreeNode* child = root2->getChild(i);
        AABB child_bv;
        if(i&1)
        {
          child_bv.min_[0] = (root2_bv.min_[0] + root2_bv.max_[0]) * 0.5;
          child_bv.max_[0] = root2_bv.max_[0];
        }
        else
        {
          child_bv.min_[0] = root2_bv.min_[0];
          child_bv.max_[0] = (root2_bv.min_[0] + root2_bv.max_[0]) * 0.5;
        }

        if(i&2)
        {
          child_bv.min_[1] = (root2_bv.min_[1] + root2_bv.max_[1]) * 0.5;
          child_bv.max_[1] = root2_bv.max_[1];
        }
        else
        {
          child_bv.min_[1] = root2_bv.min_[1];
          child_bv.max_[1] = (root2_bv.min_[1] + root2_bv.max_[1]) * 0.5;
        }

        if(i&4)
        {
          child_bv.min_[2] = (root2_bv.min_[2] + root2_bv.max_[2]) * 0.5;
          child_bv.max_[2] = root2_bv.max_[2];
        }        
        else
        {
          child_bv.min_[2] = root2_bv.min_[2];
          child_bv.max_[2] = (root2_bv.min_[2] + root2_bv.max_[2]) * 0.5;
        }

        if(collisionCostExtRecurse(root1, tree2, child, child_bv, cdata, callback, cost, nodes))
          return true;
      }
    }
  }

  return false;
}



bool defaultCollisionCostOctomapFunction(CollisionObject* o1, CollisionObject* o2, void* cdata, FCL_REAL& cost)
{
  const AABB& aabb1 = o1->getAABB();
  const AABB& aabb2 = o2->getAABB();
  Vec3f delta = min(aabb1.max_, aabb2.max_) - max(aabb1.min_, aabb2.min_);
  const ExtendedBox* box = static_cast<const ExtendedBox*>(o2->getCollisionGeometry());
  cost += delta[0] * delta[1] * delta[2] * box->prob;
  return false;
}


bool defaultCollisionCostOctomapExtFunction(CollisionObject* o1, CollisionObject* o2, void* cdata, FCL_REAL& cost, std::set<OcTreeNode_AABB_pair>& nodes)
{
  const AABB& aabb1 = o1->getAABB();
  const AABB& aabb2 = o2->getAABB();
  Vec3f delta = min(aabb1.max_, aabb2.max_) - max(aabb1.min_, aabb2.min_);
  const ExtendedBox* box = static_cast<const ExtendedBox*>(o2->getCollisionGeometry());
  cost += delta[0] * delta[1] * delta[2] * box->prob;

  const Vec3f& c = box->aabb_center;
  const Vec3f& side = box->side;
  AABB aabb(Vec3f(c[0] - side[0] / 2, c[1] - side[1] / 2, c[2] - side[2] / 2), Vec3f(c[0] + side[0] / 2, c[1] + side[1] / 2, c[2] + side[2] / 2));
  nodes.insert(OcTreeNode_AABB_pair(box->node, aabb));
  return false;
}

FCL_REAL collideCost(DynamicAABBTreeCollisionManager* manager, octomap::OcTree* octree, void* cdata, CollisionCostOctomapCallBack callback)
{
  DynamicAABBTreeCollisionManager::DynamicAABBNode* root1 = manager->getTree().getRoot();
  octomap::OcTreeNode* root2 = octree->getRoot();

  FCL_REAL delta = (1 << octree->getTreeDepth()) * octree->getResolution() / 2;
  FCL_REAL cost = 0;
  collisionCostRecurse(root1, octree, root2, AABB(Vec3f(-delta, -delta, -delta), Vec3f(delta, delta, delta)), cdata, callback, cost);
  return cost;
}

FCL_REAL collideCost(DynamicAABBTreeCollisionManager* manager, octomap::OcTree* octree, void* cdata, CollisionCostOctomapCallBackExt callback, std::vector<AABB>& nodes)
{
  DynamicAABBTreeCollisionManager::DynamicAABBNode* root1 = manager->getTree().getRoot();
  octomap::OcTreeNode* root2 = octree->getRoot();

  FCL_REAL delta = (1 << octree->getTreeDepth()) * octree->getResolution() / 2;
  FCL_REAL cost = 0;
  std::set<OcTreeNode_AABB_pair> pairs;
  collisionCostExtRecurse(root1, octree, root2, AABB(Vec3f(-delta, -delta, -delta), Vec3f(delta, delta, delta)), cdata, callback, cost, pairs);
  for(std::set<OcTreeNode_AABB_pair>::iterator it = pairs.begin(), end = pairs.end(); it != end; ++it)
    nodes.push_back(it->aabb);
  return cost;
}





}

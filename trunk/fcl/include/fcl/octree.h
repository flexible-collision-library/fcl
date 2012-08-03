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


#ifndef FCL_OCTREE_H
#define FCL_OCTREE_H


#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>

#include <octomap/octomap.h>
#include "fcl/BV/AABB.h"
#include "fcl/collision_object.h"

namespace fcl
{

/// @brief Octree is one type of collision geometry which can encode uncertainty information in the sensor data.
class OcTree : public CollisionGeometry
{
private:
  boost::shared_ptr<octomap::OcTree> tree;
public:

  /// @brief OcTreeNode must implement the following interfaces:
  ///    1) childExists(i)
  ///    2) getChild(i)
  ///    3) hasChildren()
  typedef octomap::OcTreeNode OcTreeNode;

  /// @brief construct octree with a given resolution
  OcTree(FCL_REAL resolution) : tree(boost::shared_ptr<octomap::OcTree>(new octomap::OcTree(resolution))) {}

  /// @brief construct octree from octomap
  OcTree(const boost::shared_ptr<octomap::OcTree>& tree_) : tree(tree_) {}

  /// @brief compute the AABB for the octree in its local coordinate system
  void computeLocalAABB() 
  {
    aabb_local = getRootBV();
    aabb_center = aabb_local.center();
    aabb_radius = (aabb_local.min_ - aabb_center).length();
  }

  /// @brief get the bounding volume for the root
  inline AABB getRootBV() const
  {
    FCL_REAL delta = (1 << tree->getTreeDepth()) * tree->getResolution() / 2;
    return AABB(Vec3f(-delta, -delta, -delta), Vec3f(delta, delta, delta));
  }

  /// @brief get the root node of the octree
  inline OcTreeNode* getRoot() const
  {
    return tree->getRoot();
  }

  /// @brief whether one node is completely occupied
  inline bool isNodeOccupied(const OcTreeNode* node) const
  {
    return tree->isNodeOccupied(node);
  }  

  /// @brief whether one node is completely free
  inline bool isNodeFree(const OcTreeNode* node) const
  {
    return false; // default no definitely free node
  }

  /// @brief whether one node is uncertain
  inline bool isNodeUncertain(const OcTreeNode* node) const
  {
    return (!isNodeOccupied(node)) && (!isNodeFree(node));
  }

  /// @brief update the occupied information for a cell
  inline void updateNode(FCL_REAL x, FCL_REAL y, FCL_REAL z, bool occupied)
  {
    tree->updateNode(octomap::point3d(x, y, z), occupied);
  }

  /// @brief transform the octree into a bunch of boxes; uncertainty information is kept in the boxes. However, we
  /// only keep the occupied boxes (i.e., the boxes whose occupied probability is higher enough).
  inline std::vector<boost::array<FCL_REAL, 6> > toBoxes() const
  {
    std::vector<boost::array<FCL_REAL, 6> > boxes;
    boxes.reserve(tree->size() / 2);
    for(octomap::OcTree::iterator it = tree->begin(tree->getTreeDepth()), end = tree->end();
        it != end;
        ++it)
    {
      if(tree->isNodeOccupied(*it))
      {
        FCL_REAL size = it.getSize();
        FCL_REAL x = it.getX();
        FCL_REAL y = it.getY();
        FCL_REAL z = it.getZ();
        FCL_REAL c = (*it).getOccupancy();
        FCL_REAL t = tree->getOccupancyThres();

        boost::array<FCL_REAL, 6> box = {{x, y, z, size, c, t}};
        boxes.push_back(box);
      }
    }
    return boxes;
  }

  /// @brief the threshold used to decide whether one node is occupied
  FCL_REAL getOccupancyThres() const
  {
    return tree->getOccupancyThres();
  }

  /// @brief return object type, it is an octree
  OBJECT_TYPE getObjectType() const { return OT_OCTREE; }

  /// @brief return node type, it is an octree
  NODE_TYPE getNodeType() const { return GEOM_OCTREE; }
};

/// @brief compute the bounding volume of an octree node's i-th child
static inline void computeChildBV(const AABB& root_bv, unsigned int i, AABB& child_bv)
{
  if(i&1)
  {
    child_bv.min_[0] = (root_bv.min_[0] + root_bv.max_[0]) * 0.5;
    child_bv.max_[0] = root_bv.max_[0];
  }
  else
  {
    child_bv.min_[0] = root_bv.min_[0];
    child_bv.max_[0] = (root_bv.min_[0] + root_bv.max_[0]) * 0.5;
  }

  if(i&2)
  {
    child_bv.min_[1] = (root_bv.min_[1] + root_bv.max_[1]) * 0.5;
    child_bv.max_[1] = root_bv.max_[1];
  }
  else
  {
    child_bv.min_[1] = root_bv.min_[1];
    child_bv.max_[1] = (root_bv.min_[1] + root_bv.max_[1]) * 0.5;
  }

  if(i&4)
  {
    child_bv.min_[2] = (root_bv.min_[2] + root_bv.max_[2]) * 0.5;
    child_bv.max_[2] = root_bv.max_[2];
  }        
  else
  {
    child_bv.min_[2] = root_bv.min_[2];
    child_bv.max_[2] = (root_bv.min_[2] + root_bv.max_[2]) * 0.5;
  }
}



}

#endif

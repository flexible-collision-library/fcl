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

#ifndef FCL_OCTREE_INL_H
#define FCL_OCTREE_INL_H

#include "fcl/geometry/octree/octree.h"

#include "fcl/config.h"

#if FCL_HAVE_OCTOMAP

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT OcTree<double>;

//==============================================================================
extern template
void computeChildBV(const AABB<double>& root_bv, unsigned int i, AABB<double>& child_bv);

//==============================================================================
template <typename S>
OcTree<S>::OcTree(S resolution)
  : tree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(resolution)))
{
  default_occupancy = tree->getOccupancyThres();

  // default occupancy/free threshold is consistent with default setting from octomap
  occupancy_threshold = tree->getOccupancyThres();
  free_threshold = 0;
}

//==============================================================================
template <typename S>
OcTree<S>::OcTree(const std::shared_ptr<const octomap::OcTree>& tree_)
  : tree(tree_)
{
  default_occupancy = tree->getOccupancyThres();

  // default occupancy/free threshold is consistent with default setting from octomap
  occupancy_threshold = tree->getOccupancyThres();
  free_threshold = 0;
}

//==============================================================================
template <typename S>
void OcTree<S>::computeLocalAABB()
{
  this->aabb_local = getRootBV();
  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
}

//==============================================================================
template <typename S>
AABB<S> OcTree<S>::getRootBV() const
{
  S delta = (1 << tree->getTreeDepth()) * tree->getResolution() / 2;

  // std::cout << "octree size " << delta << std::endl;
  return AABB<S>(Vector3<S>(-delta, -delta, -delta), Vector3<S>(delta, delta, delta));
}

//==============================================================================
template <typename S>
typename OcTree<S>::OcTreeNode* OcTree<S>::getRoot() const
{
  return tree->getRoot();
}

//==============================================================================
template <typename S>
bool OcTree<S>::isNodeOccupied(const OcTree<S>::OcTreeNode* node) const
{
  // return tree->isNodeOccupied(node);
  return node->getOccupancy() >= occupancy_threshold;
}

//==============================================================================
template <typename S>
bool OcTree<S>::isNodeFree(const OcTree<S>::OcTreeNode* node) const
{
  // return false; // default no definitely free node
  return node->getOccupancy() <= free_threshold;
}

//==============================================================================
template <typename S>
bool OcTree<S>::isNodeUncertain(const OcTree<S>::OcTreeNode* node) const
{
  return (!isNodeOccupied(node)) && (!isNodeFree(node));
}

//==============================================================================
template <typename S>
S OcTree<S>::getOccupancyThres() const
{
  return occupancy_threshold;
}

//==============================================================================
template <typename S>
S OcTree<S>::getFreeThres() const
{
  return free_threshold;
}

//==============================================================================
template <typename S>
S OcTree<S>::getDefaultOccupancy() const
{
  return default_occupancy;
}

//==============================================================================
template <typename S>
void OcTree<S>::setCellDefaultOccupancy(S d)
{
  default_occupancy = d;
}

//==============================================================================
template <typename S>
void OcTree<S>::setOccupancyThres(S d)
{
  occupancy_threshold = d;
}

//==============================================================================
template <typename S>
void OcTree<S>::setFreeThres(S d)
{
  free_threshold = d;
}

//==============================================================================
template <typename S>
typename OcTree<S>::OcTreeNode* OcTree<S>::getNodeChild(
    typename OcTree<S>::OcTreeNode* node, unsigned int childIdx)
{
#if OCTOMAP_VERSION_AT_LEAST(1,8,0)
  return tree->getNodeChild(node, childIdx);
#else
  return node->getChild(childIdx);
#endif
}

//==============================================================================
template <typename S>
const typename OcTree<S>::OcTreeNode* OcTree<S>::getNodeChild(
    const typename OcTree<S>::OcTreeNode* node, unsigned int childIdx) const
{
#if OCTOMAP_VERSION_AT_LEAST(1,8,0)
  return tree->getNodeChild(node, childIdx);
#else
  return node->getChild(childIdx);
#endif
}

//==============================================================================
template <typename S>
bool OcTree<S>::nodeChildExists(
    const OcTree<S>::OcTreeNode* node, unsigned int childIdx) const
{
#if OCTOMAP_VERSION_AT_LEAST(1,8,0)
  return tree->nodeChildExists(node, childIdx);
#else
  return node->childExists(childIdx);
#endif
}

//==============================================================================
template <typename S>
bool OcTree<S>::nodeHasChildren(const OcTree<S>::OcTreeNode* node) const
{
#if OCTOMAP_VERSION_AT_LEAST(1,8,0)
  return tree->nodeHasChildren(node);
#else
  return node->hasChildren();
#endif
}

//==============================================================================
template <typename S>
OBJECT_TYPE OcTree<S>::getObjectType() const
{
  return OT_OCTREE;
}

//==============================================================================
template <typename S>
NODE_TYPE OcTree<S>::getNodeType() const
{
  return GEOM_OCTREE;
}

//==============================================================================
template <typename S>
std::vector<std::array<S, 6>> OcTree<S>::toBoxes() const
{
  std::vector<std::array<S, 6>> boxes;
  boxes.reserve(tree->size() / 2);
  for(auto it = tree->begin(tree->getTreeDepth()), end = tree->end();
      it != end;
      ++it)
  {
    // if(tree->isNodeOccupied(*it))
    if(isNodeOccupied(&*it))
    {
      S size = it.getSize();
      S x = it.getX();
      S y = it.getY();
      S z = it.getZ();
      S c = (*it).getOccupancy();
      S t = tree->getOccupancyThres();

      std::array<S, 6> box = {{x, y, z, size, c, t}};
      boxes.push_back(box);
    }
  }
  return boxes;
}

//==============================================================================
template <typename S>
void computeChildBV(const AABB<S>& root_bv, unsigned int i, AABB<S>& child_bv)
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

} // namespace fcl

#endif

#endif

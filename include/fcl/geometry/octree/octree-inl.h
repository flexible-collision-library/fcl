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

#include "fcl/geometry/shape/utility.h"

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
  occupancy_threshold_log_odds = tree->getOccupancyThresLog();
  free_threshold_log_odds = 0.0;
}

//==============================================================================
template <typename S>
OcTree<S>::OcTree(const std::shared_ptr<const octomap::OcTree>& tree_)
  : tree(tree_)
{
  default_occupancy = tree->getOccupancyThres();

  // default occupancy/free threshold is consistent with default setting from octomap
  occupancy_threshold_log_odds = tree->getOccupancyThresLog();
  free_threshold_log_odds = 0;
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
bool OcTree<S>::isNodeOccupied(const typename OcTree<S>::OcTreeNode* node) const
{
  // return tree->isNodeOccupied(node);
  return node->getLogOdds() >= occupancy_threshold_log_odds;
}

//==============================================================================
template <typename S>
bool OcTree<S>::isNodeFree(const typename OcTree<S>::OcTreeNode* node) const
{
  // return false; // default no definitely free node
  return node->getLogOdds() <= free_threshold_log_odds;
}

//==============================================================================
template <typename S>
bool OcTree<S>::isNodeUncertain(const typename OcTree<S>::OcTreeNode* node) const
{
  return (!isNodeOccupied(node)) && (!isNodeFree(node));
}

//==============================================================================
template <typename S>
S OcTree<S>::getOccupancyThres() const
{
  return octomap::probability(occupancy_threshold_log_odds);
}

//==============================================================================
template <typename S>
S OcTree<S>::getFreeThres() const
{
  return octomap::probability(free_threshold_log_odds);
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
  occupancy_threshold_log_odds = octomap::logodds(d);
}

//==============================================================================
template <typename S>
void OcTree<S>::setFreeThres(S d)
{
  free_threshold_log_odds = octomap::logodds(d);
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
    const typename OcTree<S>::OcTreeNode* node, unsigned int childIdx) const
{
#if OCTOMAP_VERSION_AT_LEAST(1,8,0)
  return tree->nodeChildExists(node, childIdx);
#else
  return node->childExists(childIdx);
#endif
}

//==============================================================================
template <typename S>
bool OcTree<S>::nodeHasChildren(const typename OcTree<S>::OcTreeNode* node) const
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

//==============================================================================
template <typename S>
const typename OcTree<S>::OcTreeNode* OcTree<S>::getNodeByQueryCellId(
    intptr_t id,
    const Vector3<S>& point,
    AABB<S>* aabb,
    octomap::OcTreeKey* key,
    unsigned int* depth) const
{
  octomap::OcTree::leaf_bbx_iterator it;
  if (!getOctomapIterator(id, point, &it))
  {
    return nullptr;
  }
  if (aabb != nullptr)
  {
    Vector3<S> center(it.getX(), it.getY(), it.getZ());
    double half_size = it.getSize() / 2.0;
    Vector3<S> half_extent(half_size, half_size, half_size);
    aabb->min_ = center - half_extent;
    aabb->max_ = center + half_extent;
  }
  if (key != nullptr)
    *key = it.getKey();
  if (depth != nullptr)
    *depth = it.getDepth();
  return &(*it);
}

//==============================================================================
template <typename S>
bool OcTree<S>::getOctomapIterator(
    intptr_t id,
    const Vector3<S>& point,
    octomap::OcTree::leaf_bbx_iterator* out) const
{
  assert(out != nullptr);
  // The octomap tree structure provides no way to find a node from its pointer
  // short of an exhaustive search. This could take a long time on a large
  // tree. Instead, require the user to supply the contact point or nearest
  // point returned by the query that also returned the id. Use the point to
  // create the bounds to search for the node pointer.
  const octomap::OcTreeKey point_key = tree->coordToKey(
      point[0], point[1], point[2]);
  // Set the min and max keys used for the bbx to the point key plus or minus
  // one (if not at the limits of the data type) so we are guaranteed to hit
  // the correct cell even when the point is on a boundary and rounds to the
  // wrong cell.
  octomap::OcTreeKey min_key, max_key;
  for (unsigned int i = 0; i < 3; ++i)
  {
    min_key[i] = (point_key[i] > std::numeric_limits<octomap::key_type>::min() ?
        point_key[i] - 1 : point_key[i]);
    max_key[i] = (point_key[i] < std::numeric_limits<octomap::key_type>::max() ?
        point_key[i] + 1 : point_key[i]);
  }
  octomap::OcTree::leaf_bbx_iterator it = tree->begin_leafs_bbx(
      min_key, max_key);
  const octomap::OcTree::leaf_bbx_iterator end = tree->end_leafs_bbx();
  const OcTreeNode* const node = getRoot() + id;
  // While it may appear like this loop could take forever, in reality it will
  // only take a few iterations. Octomap iterators use a fixed end iterator
  // (copied above), and we are guaranteed to get to the end iterator after
  // incrementing past the bounds of the octomap::OcTree::leaf_bbx_iterator.
  // Incrementing end will keep the iterator at end as well. This functionality
  // of octomap iterators is tested extensively in the octomap package tests.
  while (it != end)
  {
    if (node == &(*it))
    {
      *out = it;
      return true;
    }
    ++it;
  }
  return false;
}

} // namespace fcl

#endif

#endif

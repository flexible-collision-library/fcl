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

#ifndef FCL_OCTREE_H
#define FCL_OCTREE_H

#include "fcl/config.h"

#if FCL_HAVE_OCTOMAP

#include <memory>
#include <array>

#include <octomap/octomap.h>
#include "fcl/math/bv/AABB.h"
#include "fcl/geometry/shape/box.h"
#include "fcl/narrowphase/collision_object.h"

namespace fcl
{

/// @brief Octree is one type of collision geometry which can encode uncertainty
/// information in the sensor data.
///
/// @note OcTree will only be declared if octomap was found when FCL was built.
/// For any particular FCL install, FCL_HAVE_OCTOMAP will be set to 1 in
/// fcl/config.h if and only if octomap was found. Doxygen documentation will
/// be generated whether or not octomap was found.
template <typename S>
class FCL_EXPORT OcTree : public CollisionGeometry<S>
{
private:
  std::shared_ptr<const octomap::OcTree> tree;

  S default_occupancy;

  S occupancy_threshold_log_odds;
  S free_threshold_log_odds;

public:

  typedef octomap::OcTreeNode OcTreeNode;

  /// @brief construct octree with a given resolution
  OcTree(S resolution);

  /// @brief construct octree from octomap
  OcTree(const std::shared_ptr<const octomap::OcTree>& tree_);

  /// @brief compute the AABB<S> for the octree in its local coordinate system
  void computeLocalAABB();

  /// @brief get the bounding volume for the root
  AABB<S> getRootBV() const;

  /// @brief get the root node of the octree
  OcTreeNode* getRoot() const;

  /// @brief whether one node is completely occupied
  bool isNodeOccupied(const OcTreeNode* node) const;

  /// @brief whether one node is completely free
  bool isNodeFree(const OcTreeNode* node) const;

  /// @brief whether one node is uncertain
  bool isNodeUncertain(const OcTreeNode* node) const;

  /// @brief transform the octree into a bunch of boxes; uncertainty information
  /// is kept in the boxes. However, we only keep the occupied boxes (i.e., the
  /// boxes whose occupied probability is higher enough).
  std::vector<std::array<S, 6> > toBoxes() const;

  /// @brief the threshold used to decide whether one node is occupied, this is
  /// NOT the octree occupied_thresold
  S getOccupancyThres() const;

  /// @brief the threshold used to decide whether one node is occupied, this is
  /// NOT the octree free_threshold
  S getFreeThres() const;

  S getDefaultOccupancy() const;

  void setCellDefaultOccupancy(S d);

  void setOccupancyThres(S d);

  void setFreeThres(S d);

  /// @return ptr to child number childIdx of node
  OcTreeNode* getNodeChild(OcTreeNode* node, unsigned int childIdx);

  /// @return const ptr to child number childIdx of node
  const OcTreeNode* getNodeChild(const OcTreeNode* node, unsigned int childIdx) const;
      
  /// @brief return true if the child at childIdx exists
  bool nodeChildExists(const OcTreeNode* node, unsigned int childIdx) const;

  /// @brief return true if node has at least one child
  bool nodeHasChildren(const OcTreeNode* node) const;

  /// @brief return object type, it is an octree
  OBJECT_TYPE getObjectType() const;

  /// @brief return node type, it is an octree
  NODE_TYPE getNodeType() const;

  /// @brief Access the node and corresponding info by query cell id
  ///
  /// The results of distance and collision queries include various pieces of
  /// information about the objects involved in the query (in DistanceResult
  /// and Contact, respectively). When an object in the query is an OcTree the
  /// corresponding member (b1 or b2) contains an opaque identifier refered to
  /// as the "query cell id" of the corresponding tree cell.
  ///
  /// This method returns the node pointer of the given query cell id.
  /// In order to efficiently find the cell by its query cell id, the contact
  /// point or nearest point in or on the cell must be given. This method also
  /// provides optional access to properties of the OcTree cell associated
  /// with the cell.
  ///
  /// If the given cell can not be found, nullptr is returned and no
  /// information is stored in the output pointer locations. This might happen
  /// if some other point not in or on the cell is provided or if given a stale
  /// or invalid query cell id.
  ///
  /// @param[in]  id    The query cell id, as given in b1 or b2 in
  ///                   DistanceResult or Contact.
  /// @param[in]  point A point in or on the given cell id, from DistanceResult
  ///                   or Contact.
  /// @param[out] aabb  If valid, output pointer for the AABB of the cell.
  /// @param[out] key   If valid, output pointer for the octomap::OcTreeKey of
  ///                   the cell.
  /// @param[out] depth If valid, output pointer for the tree depth of the
  ///                   cell.
  /// @return The node pointer for the given query cell id, or nullptr if not
  ///         found.
  const OcTreeNode* getNodeByQueryCellId(
      intptr_t id,
      const Vector3<S>& point,
      AABB<S>* aabb = nullptr,
      octomap::OcTreeKey* key = nullptr,
      unsigned int* depth = nullptr) const;

private:
  // Get the leaf_bbx_iterator pointing to the given cell given a point on or
  // in the cell and the query id.
  // Returns true if the cell was found, and false otherwise.
  bool getOctomapIterator(
      intptr_t id,
      const Vector3<S>& point,
      octomap::OcTree::leaf_bbx_iterator* out) const;
};

using OcTreef = OcTree<float>;
using OcTreed = OcTree<double>;

/// @brief compute the bounding volume of an octree node's i-th child
template <typename S>
FCL_EXPORT
void computeChildBV(const AABB<S>& root_bv, unsigned int i, AABB<S>& child_bv);

} // namespace fcl

#include "fcl/geometry/octree/octree-inl.h"

#endif // #if FCL_HAVE_OCTOMAP

#endif

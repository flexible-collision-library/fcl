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

/** @author Jia Pan  */

#ifndef FCL_HIERARCHY_TREE_H
#define FCL_HIERARCHY_TREE_H

#include <vector>
#include <map>
#include <functional>
#include <iostream>
#include "fcl/common/warning.h"
#include "fcl/math/bv/AABB.h"
#include "fcl/broadphase/detail/morton.h"
#include "fcl/broadphase/detail/node_base.h"

namespace fcl
{

namespace detail
{

/// @brief Class for hierarchy tree structure
template<typename BV>
class FCL_EXPORT HierarchyTree
{
public:

  using S = typename BV::S;

  typedef NodeBase<BV> NodeType;

  /// @brief Create hierarchy tree with suitable setting.
  /// bu_threshold decides the height of tree node to start bottom-up construction / optimization;
  /// topdown_level decides different methods to construct tree in topdown manner.
  /// lower level method constructs tree with better quality but is slower.
  HierarchyTree(int bu_threshold_ = 16, int topdown_level_ = 0);

  ~HierarchyTree();
  
  /// @brief Initialize the tree by a set of leaves using algorithm with a given level.
  void init(std::vector<NodeType*>& leaves, int level = 0);

  /// @brief Insest a node
  NodeType* insert(const BV& bv, void* data);

  /// @brief Remove a leaf node
  void remove(NodeType* leaf);

  /// @brief Clear the tree 
  void clear();

  /// @brief Whether the tree is empty 
  bool empty() const;

  /// @brief Updates a `leaf` node. A use case is when the bounding volume
  /// of an object changes. Ensure every parent node has its bounding volume
  /// expand or shrink to fit the bounding volumes of its children.
  /// @note Strangely the structure of the tree may change even if the
  ///       bounding volume of the `leaf` node does not change. It is just
  ///       another valid tree of the same set of objects.  This is because
  ///       update() works by first removing `leaf` and then inserting `leaf`
  ///       back. The structural change could be as simple as switching the
  ///       order of two leaves if the sibling of the `leaf` is also a leaf.
  ///       Or it could be more complicated if the sibling is an internal
  ///       node.
  void update(NodeType* leaf, int lookahead_level = -1);

  /// @brief update the tree when the bounding volume of a given leaf has changed
  bool update(NodeType* leaf, const BV& bv);

  /// @brief update one leaf's bounding volume, with prediction 
  bool update(NodeType* leaf, const BV& bv, const Vector3<S>& vel, S margin);

  /// @brief update one leaf's bounding volume, with prediction 
  bool update(NodeType* leaf, const BV& bv, const Vector3<S>& vel);

  /// @brief get the max height of the tree
  size_t getMaxHeight() const;

  /// @brief get the max depth of the tree
  size_t getMaxDepth() const;

  /// @brief balance the tree from bottom 
  void balanceBottomup();

  /// @brief balance the tree from top 
  void balanceTopdown();
  
  /// @brief balance the tree in an incremental way 
  void balanceIncremental(int iterations);
  
  /// @brief refit the tree, i.e., when the leaf nodes' bounding volumes change, update the entire tree in a bottom-up manner
  void refit();

  /// @brief extract all the leaves of the tree 
  void extractLeaves(const NodeType* root, std::vector<NodeType*>& leaves) const;

  /// @brief number of leaves in the tree
  size_t size() const;

  /// @brief get the root of the tree
  NodeType* getRoot() const;

  NodeType*& getRoot();

  /// @brief print the tree in a recursive way
  void print(NodeType* root, int depth);

private:

  typedef typename std::vector<NodeBase<BV>* >::iterator NodeVecIterator;
  typedef typename std::vector<NodeBase<BV>* >::const_iterator NodeVecConstIterator;

  struct SortByMorton
  {
    bool operator() (const NodeType* a, const NodeType* b) const
    {
      return a->code < b->code;
    }
  };

  /// @brief construct a tree for a set of leaves from bottom -- very heavy way 
  void bottomup(const NodeVecIterator lbeg, const NodeVecIterator lend);

  /// @brief construct a tree for a set of leaves from top 
  NodeType* topdown(const NodeVecIterator lbeg, const NodeVecIterator lend);

  /// @brief compute the maximum height of a subtree rooted from a given node
  size_t getMaxHeight(NodeType* node) const;

  /// @brief compute the maximum depth of a subtree rooted from a given node
  void getMaxDepth(NodeType* node, size_t depth, size_t& max_depth) const;

  /// @brief construct a tree from a list of nodes stored in [lbeg, lend) in a topdown manner.
  /// During construction, first compute the best split axis as the axis along with the longest AABB<S> edge.
  /// Then compute the median of all nodes' center projection onto the axis and using it as the split threshold.
  NodeType* topdown_0(const NodeVecIterator lbeg, const NodeVecIterator lend);

  /// @brief construct a tree from a list of nodes stored in [lbeg, lend) in a topdown manner.
  /// During construction, first compute the best split thresholds for different axes as the average of all nodes' center.
  /// Then choose the split axis as the axis whose threshold can divide the nodes into two parts with almost similar size.
  /// This construction is more expensive then topdown_0, but also can provide tree with better quality.
  NodeType* topdown_1(const NodeVecIterator lbeg, const NodeVecIterator lend);

  /// @brief init tree from leaves in the topdown manner (topdown_0 or topdown_1)
  void init_0(std::vector<NodeType*>& leaves);

  /// @brief init tree from leaves using morton code. It uses morton_0, i.e., for nodes which is of depth more than the maximum bits of the morton code,
  /// we use bottomup method to construct the subtree, which is slow but can construct tree with high quality.
  void init_1(std::vector<NodeType*>& leaves);

  /// @brief init tree from leaves using morton code. It uses morton_0, i.e., for nodes which is of depth more than the maximum bits of the morton code,
  /// we split the leaves into two parts with the same size simply using the node index. 
  void init_2(std::vector<NodeType*>& leaves);

  /// @brief init tree from leaves using morton code. It uses morton_2, i.e., for all nodes, we simply divide the leaves into parts with the same size simply using the node index.
  void init_3(std::vector<NodeType*>& leaves);
  
  NodeType* mortonRecurse_0(const NodeVecIterator lbeg, const NodeVecIterator lend, const uint32& split, int bits);

  NodeType* mortonRecurse_1(const NodeVecIterator lbeg, const NodeVecIterator lend, const uint32& split, int bits);

  NodeType* mortonRecurse_2(const NodeVecIterator lbeg, const NodeVecIterator lend);

  /// @brief update one leaf node's bounding volume 
  void update_(NodeType* leaf, const BV& bv);

  /// @brief sort node n and its parent according to their memory position 
  NodeType* sort(NodeType* n, NodeType*& r);
  
  /// @brief Insert a leaf node and also update its ancestors. Maintain the
  /// tree as a full binary tree (every interior node has exactly two children).
  /// Furthermore, adjust the BV of interior nodes so that each parent's BV
  /// tightly fits its children's BVs.
  /// @param sub_root The root of the subtree into which we will insert the
  //                  leaf node.
  void insertLeaf(NodeType* const sub_root, NodeType* const leaf);

  /// @brief Remove a leaf. Maintain the tree as a full binary tree (every
  /// interior node has exactly two children). Furthermore, adjust the BV of
  /// interior nodes so that each parent's BV tightly fits its children's BVs.
  /// @note The leaf node itself is not deleted yet, but all the unnecessary
  ///       internal nodes are deleted.
  /// @returns the root of the subtree containing the nodes whose BVs were
  //           adjusted.
  NodeType* removeLeaf(NodeType* const leaf);

  /// @brief Delete all internal nodes and return all leaves nodes with given depth from root 
  void fetchLeaves(NodeType* root, std::vector<NodeType*>& leaves, int depth = -1);

  static size_t indexOf(NodeType* node);

  /// @brief create one node (leaf or internal)  
  NodeType* createNode(NodeType* parent, 
                       const BV& bv,
                       void* data);

  NodeType* createNode(NodeType* parent,
                       const BV& bv1,
                       const BV& bv2,
                       void* data);
  
  NodeType* createNode(NodeType* parent,
                       void* data);

  void deleteNode(NodeType* node);

  void recurseDeleteNode(NodeType* node);

  void recurseRefit(NodeType* node);

protected:
  NodeType* root_node;

  size_t n_leaves;

  unsigned int opath;

  /// This is a one NodeType cache, the reason is that we need to remove a node and then add it again frequently. 
  NodeType* free_node; 

  int max_lookahead_level;
  
public:
  /// @brief decide which topdown algorithm to use
  int topdown_level;

  /// @brief decide the depth to use expensive bottom-up algorithm
  int bu_threshold;
};

/// @brief Compare two nodes accoording to the d-th dimension of node center
template<typename BV>
bool nodeBaseLess(NodeBase<BV>* a, NodeBase<BV>* b, int d);

/// @brief select from node1 and node2 which is close to a given query. 0 for
/// node1 and 1 for node2
template<typename BV>
size_t select(
    const NodeBase<BV>& query,
    const NodeBase<BV>& node1,
    const NodeBase<BV>& node2);

/// @brief select from node1 and node2 which is close to a given query bounding
/// volume. 0 for node1 and 1 for node2
template<typename BV>
size_t select(
    const BV& query, const NodeBase<BV>& node1, const NodeBase<BV>& node2);

} // namespace detail
} // namespace fcl

#include "fcl/broadphase/detail/hierarchy_tree-inl.h"

#endif

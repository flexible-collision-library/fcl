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

#ifndef FCL_HIERARCHY_TREE_ARRAY_H
#define FCL_HIERARCHY_TREE_ARRAY_H

#include <vector>
#include <map>
#include <functional>
#include <iostream>
#include "fcl/common/warning.h"
#include "fcl/math/bv/AABB.h"
#include "fcl/broadphase/detail/morton.h"
#include "fcl/broadphase/detail/node_base_array.h"

namespace fcl
{

namespace detail
{

namespace implementation_array
{

/// @brief Class for hierarchy tree structure
template<typename BV>
class FCL_EXPORT HierarchyTree
{
  using S = typename BV::S;
  typedef NodeBase<BV> NodeType;
  
  struct SortByMorton
  {
    SortByMorton(NodeType* nodes_in) : nodes(nodes_in) {}
    SortByMorton(NodeType* nodes_in, uint32 split_in)
        : nodes(nodes_in), split(split_in) {}
    bool operator() (size_t a, size_t b) const
    {
      if((a != NULL_NODE) && (b != NULL_NODE))
        return nodes[a].code < nodes[b].code;
      else if(a == NULL_NODE)
        return split < nodes[b].code;
      else if(b == NULL_NODE)
        return nodes[a].code < split;

      return false;
    }

    NodeType* nodes{};
    uint32 split{};
  };

public:
  /// @brief Create hierarchy tree with suitable setting.
  /// bu_threshold decides the height of tree node to start bottom-up construction / optimization;
  /// topdown_level decides different methods to construct tree in topdown manner.
  /// lower level method constructs tree with better quality but is slower.
  HierarchyTree(int bu_threshold_ = 16, int topdown_level_ = 0);

  ~HierarchyTree();

  /// @brief Initialize the tree by a set of leaves using algorithm with a given level.
  void init(NodeType* leaves, int n_leaves_, int level = 0);

  /// @brief Initialize the tree by a set of leaves using algorithm with a given level.
  size_t insert(const BV& bv, void* data);

  /// @brief Remove a leaf node
  void remove(size_t leaf);

  /// @brief Clear the tree 
  void clear();

  /// @brief Whether the tree is empty 
  bool empty() const;
 
  /// @brief update one leaf node 
  void update(size_t leaf, int lookahead_level = -1);

  /// @brief update the tree when the bounding volume of a given leaf has changed
  bool update(size_t leaf, const BV& bv);

  /// @brief update one leaf's bounding volume, with prediction 
  bool update(size_t leaf, const BV& bv, const Vector3<S>& vel, S margin);

  /// @brief update one leaf's bounding volume, with prediction 
  bool update(size_t leaf, const BV& bv, const Vector3<S>& vel);

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
  void extractLeaves(size_t root, NodeType*& leaves) const;

  /// @brief number of leaves in the tree
  size_t size() const;

  /// @brief get the root of the tree
  size_t getRoot() const;

  /// @brief get the pointer to the nodes array
  NodeType* getNodes() const;

  /// @brief print the tree in a recursive way
  void print(size_t root, int depth);

private:

  /// @brief construct a tree for a set of leaves from bottom -- very heavy way 
  void bottomup(size_t* lbeg, size_t* lend);
  
  /// @brief construct a tree for a set of leaves from top 
  size_t topdown(size_t* lbeg, size_t* lend);

  /// @brief compute the maximum height of a subtree rooted from a given node
  size_t getMaxHeight(size_t node) const;

  /// @brief compute the maximum depth of a subtree rooted from a given node
  void getMaxDepth(size_t node, size_t depth, size_t& max_depth) const;

  /// @brief construct a tree from a list of nodes stored in [lbeg, lend) in a topdown manner.
  /// During construction, first compute the best split axis as the axis along with the longest AABB<S> edge.
  /// Then compute the median of all nodes' center projection onto the axis and using it as the split threshold.
  size_t topdown_0(size_t* lbeg, size_t* lend);

  /// @brief construct a tree from a list of nodes stored in [lbeg, lend) in a topdown manner.
  /// During construction, first compute the best split thresholds for different axes as the average of all nodes' center.
  /// Then choose the split axis as the axis whose threshold can divide the nodes into two parts with almost similar size.
  /// This construction is more expensive then topdown_0, but also can provide tree with better quality.
  size_t topdown_1(size_t* lbeg, size_t* lend);

  /// @brief init tree from leaves in the topdown manner (topdown_0 or topdown_1)
  void init_0(NodeType* leaves, int n_leaves_);

  /// @brief init tree from leaves using morton code. It uses morton_0, i.e., for nodes which is of depth more than the maximum bits of the morton code,
  /// we use bottomup method to construct the subtree, which is slow but can construct tree with high quality.
  void init_1(NodeType* leaves, int n_leaves_);

  /// @brief init tree from leaves using morton code. It uses morton_0, i.e., for nodes which is of depth more than the maximum bits of the morton code,
  /// we split the leaves into two parts with the same size simply using the node index. 
  void init_2(NodeType* leaves, int n_leaves_);

  /// @brief init tree from leaves using morton code. It uses morton_2, i.e., for all nodes, we simply divide the leaves into parts with the same size simply using the node index.
  void init_3(NodeType* leaves, int n_leaves_);

  size_t mortonRecurse_0(size_t* lbeg, size_t* lend, const uint32& split, int bits);

  size_t mortonRecurse_1(size_t* lbeg, size_t* lend, const uint32& split, int bits);

  size_t mortonRecurse_2(size_t* lbeg, size_t* lend);

  /// @brief update one leaf node's bounding volume 
  void update_(size_t leaf, const BV& bv);

  /// @brief Insert a leaf node and also update its ancestors 
  void insertLeaf(size_t root, size_t leaf);

  /// @brief Remove a leaf. The leaf node itself is not deleted yet, but all the unnecessary internal nodes are deleted.
  /// return the node with the smallest depth and is influenced by the remove operation 
  size_t removeLeaf(size_t leaf);

  /// @brief Delete all internal nodes and return all leaves nodes with given depth from root 
  void fetchLeaves(size_t root, NodeType*& leaves, int depth = -1);

  size_t indexOf(size_t node);

  size_t allocateNode();

  /// @brief create one node (leaf or internal)
  size_t createNode(size_t parent, 
                    const BV& bv1,
                    const BV& bv2,
                    void* data);

  size_t createNode(size_t parent,
                    const BV& bv, 
                    void* data);

  size_t createNode(size_t parent,
                    void* data);

  void deleteNode(size_t node);

  void recurseRefit(size_t node);

protected:
  size_t root_node;
  NodeType* nodes;
  size_t n_nodes;
  size_t n_nodes_alloc;
  
  size_t n_leaves;
  size_t freelist;
  unsigned int opath;

  int max_lookahead_level;

public:
  /// @brief decide which topdown algorithm to use
  int topdown_level;

  /// @brief decide the depth to use expensive bottom-up algorithm
  int bu_threshold;

public:
  static const size_t NULL_NODE = -1;
};

template<typename BV>
const size_t HierarchyTree<BV>::NULL_NODE;


/// @brief Functor comparing two nodes
template<typename BV>
struct nodeBaseLess
{
  nodeBaseLess(const NodeBase<BV>* nodes_, size_t d_);

  bool operator() (size_t i, size_t j) const;

private:

  /// @brief the nodes array
  const NodeBase<BV>* nodes;

  /// @brief the dimension to compare
  size_t d;
};

/// @brief select the node from node1 and node2 which is close to the query-th
/// node in the nodes. 0 for node1 and 1 for node2.
template<typename BV>
size_t select(size_t query, size_t node1, size_t node2, NodeBase<BV>* nodes);

/// @brief select the node from node1 and node2 which is close to the query
/// AABB<S>. 0 for node1 and 1 for node2.
template<typename BV>
size_t select(const BV& query, size_t node1, size_t node2, NodeBase<BV>* nodes);

} // namespace implementation_array
} // namespace detail
} // namespace fcl

#include "fcl/broadphase/detail/hierarchy_tree_array-inl.h"

#endif

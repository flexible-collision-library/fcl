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

#ifndef FCL_HIERARCHY_TREE_INL_H
#define FCL_HIERARCHY_TREE_INL_H

#include "fcl/broadphase/detail/hierarchy_tree.h"

namespace fcl
{

namespace detail
{

//==============================================================================
template<typename BV>
HierarchyTree<BV>::HierarchyTree(int bu_threshold_, int topdown_level_)
{
  root_node = nullptr;
  n_leaves = 0;
  free_node = nullptr;
  max_lookahead_level = -1;
  opath = 0;
  bu_threshold = bu_threshold_;
  topdown_level = topdown_level_;
}

//==============================================================================
template<typename BV>
HierarchyTree<BV>::~HierarchyTree()
{
  clear();
}

//==============================================================================
template<typename BV>
void HierarchyTree<BV>::init(std::vector<NodeType*>& leaves, int level)
{
  switch(level)
  {
  case 0:
    init_0(leaves);
    break;
  case 1:
    init_1(leaves);
    break;
  case 2:
    init_2(leaves);
    break;
  case 3:
    init_3(leaves);
    break;
  default:
    init_0(leaves);
  }
}

//==============================================================================
template<typename BV>
typename HierarchyTree<BV>::NodeType* HierarchyTree<BV>::insert(const BV& bv, void* data)
{
  NodeType* leaf = createNode(nullptr, bv, data);
  insertLeaf(root_node, leaf);
  ++n_leaves;
  return leaf;
}

//==============================================================================
template<typename BV>
void HierarchyTree<BV>::remove(NodeType* leaf)
{
  removeLeaf(leaf);
  deleteNode(leaf);
  --n_leaves;
}

//==============================================================================
template<typename BV>
void HierarchyTree<BV>::clear()
{
  if(root_node)
    recurseDeleteNode(root_node);
  n_leaves = 0;
  delete free_node;
  free_node = nullptr;
  max_lookahead_level = -1;
  opath = 0;
}

//==============================================================================
template<typename BV>
bool HierarchyTree<BV>::empty() const
{
  return (nullptr == root_node);
}

//==============================================================================
template<typename BV>
void HierarchyTree<BV>::update(NodeType* leaf, int lookahead_level)
{
  // TODO(DamrongGuoy): Since we update a leaf node by removing and
  //  inserting the same leaf node, it is likely to change the structure of
  //  the tree even if no object's pose has changed. In the future,
  //  find a way to preserve the structure of the tree to solve this issue:
  //  https://github.com/flexible-collision-library/fcl/issues/368

  // First we remove the leaf node pointed by `leaf` variable from the tree.
  // The `sub_root` variable is the root of the subtree containing nodes
  // whose BVs were adjusted due to node removal.
  typename HierarchyTree<BV>::NodeType* sub_root = removeLeaf(leaf);
  if(sub_root)
  {
    if(lookahead_level > 0)
    {
      // For positive `lookahead_level`, we move the `sub_root` variable up.
      for(int i = 0; (i < lookahead_level) && sub_root->parent; ++i)
        sub_root = sub_root->parent;
    }
    else
      // By default, lookahead_level = -1, and we reset the `sub_root` variable
      // to the root of the entire tree.
      sub_root = root_node;
  }
  // Then we insert the node pointed by `leaf` variable back into the
  // subtree rooted at `sub_root` variable.
  insertLeaf(sub_root, leaf);
}

//==============================================================================
template<typename BV>
bool HierarchyTree<BV>::update(NodeType* leaf, const BV& bv)
{
  if(leaf->bv.contain(bv)) return false;
  update_(leaf, bv);
  return true;
}

//==============================================================================
template <typename S, typename BV>
struct UpdateImpl
{
  static bool run(
      const HierarchyTree<BV>& tree,
      typename HierarchyTree<BV>::NodeType* leaf,
      const BV& bv,
      const Vector3<S>& /*vel*/,
      S /*margin*/)
  {
    if(leaf->bv.contain(bv)) return false;
    tree.update_(leaf, bv);
    return true;
  }

  static bool run(
      const HierarchyTree<BV>& tree,
      typename HierarchyTree<BV>::NodeType* leaf,
      const BV& bv,
      const Vector3<S>& /*vel*/)
  {
    if(leaf->bv.contain(bv)) return false;
    tree.update_(leaf, bv);
    return true;
  }
};

//==============================================================================
template<typename BV>
bool HierarchyTree<BV>::update(NodeType* leaf, const BV& bv, const Vector3<S>& vel, S margin)
{
  return UpdateImpl<typename BV::S, BV>::run(*this, leaf, bv, vel, margin);
}

//==============================================================================
template<typename BV>
bool HierarchyTree<BV>::update(NodeType* leaf, const BV& bv, const Vector3<S>& vel)
{
  return UpdateImpl<typename BV::S, BV>::run(*this, leaf, bv, vel);
}

//==============================================================================
template<typename BV>
size_t HierarchyTree<BV>::getMaxHeight() const
{
  if(!root_node)
    return 0;
  return getMaxHeight(root_node);
}

//==============================================================================
template<typename BV>
size_t HierarchyTree<BV>::getMaxDepth() const
{
  if(!root_node) return 0;

  size_t max_depth;
  getMaxDepth(root_node, 0, max_depth);
  return max_depth;
}

//==============================================================================
template<typename BV>
void HierarchyTree<BV>::balanceBottomup()
{
  if(root_node)
  {
    std::vector<NodeType*> leaves;
    leaves.reserve(n_leaves);
    fetchLeaves(root_node, leaves);
    bottomup(leaves.begin(), leaves.end());
    root_node = leaves[0];
  }
}

//==============================================================================
template<typename BV>
void HierarchyTree<BV>::balanceTopdown()
{
  if(root_node)
  {
    std::vector<NodeType*> leaves;
    leaves.reserve(n_leaves);
    fetchLeaves(root_node, leaves);
    root_node = topdown(leaves.begin(), leaves.end());
  }
}

//==============================================================================
template<typename BV>
void HierarchyTree<BV>::balanceIncremental(int iterations)
{
  if(iterations < 0) iterations = n_leaves;
  if(root_node && (iterations > 0))
  {
    for(int i = 0; i < iterations; ++i)
    {
      NodeType* node = root_node;
      unsigned int bit = 0;
      while(!node->isLeaf())
      {
        node = sort(node, root_node)->children[(opath>>bit)&1];
        bit = (bit+1)&(sizeof(unsigned int) * 8-1);
      }
      update(node);
      ++opath;
    }
  }
}

//==============================================================================
template<typename BV>
void HierarchyTree<BV>::refit()
{
  if(root_node)
    recurseRefit(root_node);
}

//==============================================================================
template<typename BV>
void HierarchyTree<BV>::extractLeaves(const NodeType* root, std::vector<NodeType*>& leaves) const
{
  if(!root->isLeaf())
  {
    extractLeaves(root->children[0], leaves);
    extractLeaves(root->children[1], leaves);
  }
  else
    leaves.push_back(root);
}

//==============================================================================
template<typename BV>
size_t HierarchyTree<BV>::size() const
{
  return n_leaves;
}

//==============================================================================
template<typename BV>
typename HierarchyTree<BV>::NodeType* HierarchyTree<BV>::getRoot() const
{
  return root_node;
}

//==============================================================================
template<typename BV>
typename HierarchyTree<BV>::NodeType*& HierarchyTree<BV>::getRoot()
{
  return root_node;
}

//==============================================================================
template<typename BV>
void HierarchyTree<BV>::print(NodeType* root, int depth)
{
  for(int i = 0; i < depth; ++i)
    std::cout << " ";
  std::cout << " (" << root->bv.min_[0] << ", " << root->bv.min_[1] << ", " << root->bv.min_[2] << "; " << root->bv.max_[0] << ", " << root->bv.max_[1] << ", " << root->bv.max_[2] << ")" << std::endl;
  if(root->isLeaf())
  {
  }
  else
  {
    print(root->children[0], depth+1);
    print(root->children[1], depth+1);
  }
}

//==============================================================================
template<typename BV>
void HierarchyTree<BV>::bottomup(const NodeVecIterator lbeg, const NodeVecIterator lend)
{
  NodeVecIterator lcur_end = lend;
  while(lbeg < lcur_end - 1)
  {
    NodeVecIterator min_it1, min_it2;
    S min_size = std::numeric_limits<S>::max();
    for(NodeVecIterator it1 = lbeg; it1 < lcur_end; ++it1)
    {
      for(NodeVecIterator it2 = it1 + 1; it2 < lcur_end; ++it2)
      {
        S cur_size = ((*it1)->bv + (*it2)->bv).size();
        if(cur_size < min_size)
        {
          min_size = cur_size;
          min_it1 = it1;
          min_it2 = it2;
        }
      }
    }

    NodeType* n[2] = {*min_it1, *min_it2};
    NodeType* p = createNode(nullptr, n[0]->bv, n[1]->bv, nullptr);
    p->children[0] = n[0];
    p->children[1] = n[1];
    n[0]->parent = p;
    n[1]->parent = p;
    *min_it1 = p;
    NodeType* tmp = *min_it2;
    lcur_end--;
    *min_it2 = *lcur_end;
    *lcur_end = tmp;
  }
}

//==============================================================================
template<typename BV>
typename HierarchyTree<BV>::NodeType* HierarchyTree<BV>::topdown(const NodeVecIterator lbeg, const NodeVecIterator lend)
{
  switch(topdown_level)
  {
  case 0:
    return topdown_0(lbeg, lend);
    break;
  case 1:
    return topdown_1(lbeg, lend);
    break;
  default:
    return topdown_0(lbeg, lend);
  }
}

//==============================================================================
template<typename BV>
size_t HierarchyTree<BV>::getMaxHeight(NodeType* node) const
{
  if(!node->isLeaf())
  {
    size_t height1 = getMaxHeight(node->children[0]);
    size_t height2 = getMaxHeight(node->children[1]);
    return std::max(height1, height2) + 1;
  }
  else
    return 0;
}

//==============================================================================
template<typename BV>
void HierarchyTree<BV>::getMaxDepth(NodeType* node, size_t depth, size_t& max_depth) const
{
  if(!node->isLeaf())
  {
    getMaxDepth(node->children[0], depth+1, max_depth);
    getMaxDepth(node->children[1], depth+1, max_depth);
  }
  else
    max_depth = std::max(max_depth, depth);
}

//==============================================================================
template<typename BV>
typename HierarchyTree<BV>::NodeType* HierarchyTree<BV>::topdown_0(const NodeVecIterator lbeg, const NodeVecIterator lend)
{
  int num_leaves = lend - lbeg;
  if(num_leaves > 1)
  {
    if(num_leaves > bu_threshold)
    {
      BV vol = (*lbeg)->bv;
      for(NodeVecIterator it = lbeg + 1; it < lend; ++it)
        vol += (*it)->bv;

      int best_axis = 0;
      S extent[3] = {vol.width(), vol.height(), vol.depth()};
      if(extent[1] > extent[0]) best_axis = 1;
      if(extent[2] > extent[best_axis]) best_axis = 2;

      // compute median
      NodeVecIterator lcenter = lbeg + num_leaves / 2;
      std::nth_element(lbeg, lcenter, lend, std::bind(&nodeBaseLess<BV>, std::placeholders::_1, std::placeholders::_2, std::ref(best_axis)));

      NodeType* node = createNode(nullptr, vol, nullptr);
      node->children[0] = topdown_0(lbeg, lcenter);
      node->children[1] = topdown_0(lcenter, lend);
      node->children[0]->parent = node;
      node->children[1]->parent = node;
      return node;
    }
    else
    {
      bottomup(lbeg, lend);
      return *lbeg;
    }
  }
  return *lbeg;
}

//==============================================================================
template<typename BV>
typename HierarchyTree<BV>::NodeType* HierarchyTree<BV>::topdown_1(const NodeVecIterator lbeg, const NodeVecIterator lend)
{
  int num_leaves = lend - lbeg;
  if(num_leaves > 1)
  {
    if(num_leaves > bu_threshold)
    {
      Vector3<S> split_p = (*lbeg)->bv.center();
      BV vol = (*lbeg)->bv;
      NodeVecIterator it;
      for(it = lbeg + 1; it < lend; ++it)
      {
        split_p += (*it)->bv.center();
        vol += (*it)->bv;
      }
      split_p /= (S)(num_leaves);
      int best_axis = -1;
      int bestmidp = num_leaves;
      int splitcount[3][2] = {{0,0}, {0,0}, {0,0}};
      for(it = lbeg; it < lend; ++it)
      {
        Vector3<S> x = (*it)->bv.center() - split_p;
        for(size_t j = 0; j < 3; ++j)
          ++splitcount[j][x[j] > 0 ? 1 : 0];
      }

      for(size_t i = 0; i < 3; ++i)
      {
        if((splitcount[i][0] > 0) && (splitcount[i][1] > 0))
        {
          int midp = std::abs(splitcount[i][0] - splitcount[i][1]);
          if(midp < bestmidp)
          {
            best_axis = i;
            bestmidp = midp;
          }
        }
      }

      if(best_axis < 0) best_axis = 0;

      S split_value = split_p[best_axis];
      NodeVecIterator lcenter = lbeg;
      for(it = lbeg; it < lend; ++it)
      {
        if((*it)->bv.center()[best_axis] < split_value)
        {
          NodeType* temp = *it;
          *it = *lcenter;
          *lcenter = temp;
          ++lcenter;
        }
      }

      NodeType* node = createNode(nullptr, vol, nullptr);
      node->children[0] = topdown_1(lbeg, lcenter);
      node->children[1] = topdown_1(lcenter, lend);
      node->children[0]->parent = node;
      node->children[1]->parent = node;
      return node;
    }
    else
    {
      bottomup(lbeg, lend);
      return *lbeg;
    }
  }
  return *lbeg;
}

//==============================================================================
template<typename BV>
void HierarchyTree<BV>::init_0(std::vector<NodeType*>& leaves)
{
  clear();
  root_node = topdown(leaves.begin(), leaves.end());
  n_leaves = leaves.size();
  max_lookahead_level = -1;
  opath = 0;
}

//==============================================================================
template<typename BV>
void HierarchyTree<BV>::init_1(std::vector<NodeType*>& leaves)
{
  clear();

  BV bound_bv;
  if(leaves.size() > 0)
    bound_bv = leaves[0]->bv;
  for(size_t i = 1; i < leaves.size(); ++i)
    bound_bv += leaves[i]->bv;

  morton_functor<typename BV::S, uint32> coder(bound_bv);
  for(size_t i = 0; i < leaves.size(); ++i)
    leaves[i]->code = coder(leaves[i]->bv.center());

  std::sort(leaves.begin(), leaves.end(), SortByMorton());

  root_node = mortonRecurse_0(leaves.begin(), leaves.end(), (1 << (coder.bits()-1)), coder.bits()-1);

  refit();
  n_leaves = leaves.size();
  max_lookahead_level = -1;
  opath = 0;
}

//==============================================================================
template<typename BV>
void HierarchyTree<BV>::init_2(std::vector<NodeType*>& leaves)
{
  clear();

  BV bound_bv;
  if(leaves.size() > 0)
    bound_bv = leaves[0]->bv;
  for(size_t i = 1; i < leaves.size(); ++i)
    bound_bv += leaves[i]->bv;

  morton_functor<typename BV::S, uint32> coder(bound_bv);
  for(size_t i = 0; i < leaves.size(); ++i)
    leaves[i]->code = coder(leaves[i]->bv.center());

  std::sort(leaves.begin(), leaves.end(), SortByMorton());

  root_node = mortonRecurse_1(leaves.begin(), leaves.end(), (1 << (coder.bits()-1)), coder.bits()-1);

  refit();
  n_leaves = leaves.size();
  max_lookahead_level = -1;
  opath = 0;
}

//==============================================================================
template<typename BV>
void HierarchyTree<BV>::init_3(std::vector<NodeType*>& leaves)
{
  clear();

  BV bound_bv;
  if(leaves.size() > 0)
    bound_bv = leaves[0]->bv;
  for(size_t i = 1; i < leaves.size(); ++i)
    bound_bv += leaves[i]->bv;

  morton_functor<typename BV::S, uint32> coder(bound_bv);
  for(size_t i = 0; i < leaves.size(); ++i)
    leaves[i]->code = coder(leaves[i]->bv.center());

  std::sort(leaves.begin(), leaves.end(), SortByMorton());

  root_node = mortonRecurse_2(leaves.begin(), leaves.end());

  refit();
  n_leaves = leaves.size();
  max_lookahead_level = -1;
  opath = 0;
}

//==============================================================================
template<typename BV>
typename HierarchyTree<BV>::NodeType* HierarchyTree<BV>::mortonRecurse_0(const NodeVecIterator lbeg, const NodeVecIterator lend, const uint32& split, int bits)
{
  int num_leaves = lend - lbeg;
  if(num_leaves > 1)
  {
    if(bits > 0)
    {
      NodeType dummy;
      dummy.code = split;
      NodeVecIterator lcenter = std::lower_bound(lbeg, lend, &dummy, SortByMorton());

      if(lcenter == lbeg)
      {
        uint32 split2 = split | (1 << (bits - 1));
        return mortonRecurse_0(lbeg, lend, split2, bits - 1);
      }
      else if(lcenter == lend)
      {
        uint32 split1 = (split & (~(1 << bits))) | (1 << (bits - 1));
        return mortonRecurse_0(lbeg, lend, split1, bits - 1);
      }
      else
      {
        uint32 split1 = (split & (~(1 << bits))) | (1 << (bits - 1));
        uint32 split2 = split | (1 << (bits - 1));

        NodeType* child1 = mortonRecurse_0(lbeg, lcenter, split1, bits - 1);
        NodeType* child2 = mortonRecurse_0(lcenter, lend, split2, bits - 1);
        NodeType* node = createNode(nullptr, nullptr);
        node->children[0] = child1;
        node->children[1] = child2;
        child1->parent = node;
        child2->parent = node;
        return node;
      }
    }
    else
    {
      NodeType* node = topdown(lbeg, lend);
      return node;
    }
  }
  else
    return *lbeg;
}

//==============================================================================
template<typename BV>
typename HierarchyTree<BV>::NodeType* HierarchyTree<BV>::mortonRecurse_1(const NodeVecIterator lbeg, const NodeVecIterator lend, const uint32& split, int bits)
{
  int num_leaves = lend - lbeg;
  if(num_leaves > 1)
  {
    if(bits > 0)
    {
      NodeType dummy;
      dummy.code = split;
      NodeVecIterator lcenter = std::lower_bound(lbeg, lend, &dummy, SortByMorton());

      if(lcenter == lbeg)
      {
        uint32 split2 = split | (1 << (bits - 1));
        return mortonRecurse_1(lbeg, lend, split2, bits - 1);
      }
      else if(lcenter == lend)
      {
        uint32 split1 = (split & (~(1 << bits))) | (1 << (bits - 1));
        return mortonRecurse_1(lbeg, lend, split1, bits - 1);
      }
      else
      {
        uint32 split1 = (split & (~(1 << bits))) | (1 << (bits - 1));
        uint32 split2 = split | (1 << (bits - 1));

        NodeType* child1 = mortonRecurse_1(lbeg, lcenter, split1, bits - 1);
        NodeType* child2 = mortonRecurse_1(lcenter, lend, split2, bits - 1);
        NodeType* node = createNode(nullptr, nullptr);
        node->children[0] = child1;
        node->children[1] = child2;
        child1->parent = node;
        child2->parent = node;
        return node;
      }
    }
    else
    {
      NodeType* child1 = mortonRecurse_1(lbeg, lbeg + num_leaves / 2, 0, bits - 1);
      NodeType* child2 = mortonRecurse_1(lbeg + num_leaves / 2, lend, 0, bits - 1);
      NodeType* node = createNode(nullptr, nullptr);
      node->children[0] = child1;
      node->children[1] = child2;
      child1->parent = node;
      child2->parent = node;
      return node;
    }
  }
  else
    return *lbeg;
}

//==============================================================================
template<typename BV>
typename HierarchyTree<BV>::NodeType* HierarchyTree<BV>::mortonRecurse_2(const NodeVecIterator lbeg, const NodeVecIterator lend)
{
  int num_leaves = lend - lbeg;
  if(num_leaves > 1)
  {
    NodeType* child1 = mortonRecurse_2(lbeg, lbeg + num_leaves / 2);
    NodeType* child2 = mortonRecurse_2(lbeg + num_leaves / 2, lend);
    NodeType* node = createNode(nullptr, nullptr);
    node->children[0] = child1;
    node->children[1] = child2;
    child1->parent = node;
    child2->parent = node;
    return node;
  }
  else
    return *lbeg;
}

//==============================================================================
template<typename BV>
void HierarchyTree<BV>::update_(NodeType* leaf, const BV& bv)
{
  NodeType* root = removeLeaf(leaf);
  if(root)
  {
    if(max_lookahead_level >= 0)
    {
      for(int i = 0; (i < max_lookahead_level) && root->parent; ++i)
        root = root->parent;
    }
    else
      root = root_node;
  }

  leaf->bv = bv;
  insertLeaf(root, leaf);
}

//==============================================================================
template<typename BV>
typename HierarchyTree<BV>::NodeType* HierarchyTree<BV>::sort(NodeType* n, NodeType*& r)
{
  NodeType* p = n->parent;
  if(p > n)
  {
    int i = indexOf(n);
    int j = 1 - i;
    NodeType* s = p->children[j];
    NodeType* q = p->parent;
    if(q) q->children[indexOf(p)] = n; else r = n;
    s->parent = n;
    p->parent = n;
    n->parent = q;
    p->children[0] = n->children[0];
    p->children[1] = n->children[1];
    n->children[0]->parent = p;
    n->children[1]->parent = p;
    n->children[i] = p;
    n->children[j] = s;
    std::swap(p->bv, n->bv);
    return p;
  }
  return n;
}

//==============================================================================
template<typename BV>
void HierarchyTree<BV>::insertLeaf(NodeType* const sub_root,
                                   NodeType* const leaf)
// Attempts to insert `leaf` into a subtree rooted at `sub_root`.
// 1. If the whole tree is empty, then `leaf` simply becomes the tree.
// 2. Otherwise, a leaf node called `sibling` of the subtree rooted at
//    `sub_root` is selected (the selection criteria is a black box for this
//    algorithm), and the `sibling` leaf is replaced by an internal node with
//    two children, `sibling` and `leaf`. The bounding volumes are updated as
//    necessary.
{
  if(!root_node)
  {
    // If the entire tree is empty, the node pointed by `leaf` variable will
    // become the root of the tree.
    root_node = leaf;
    leaf->parent = nullptr;
    return;
  }
  // Traverse the tree from the given `sub_root` down to an existing leaf
  // node that we call `sibling`. The `sibling` node will eventually become
  // the sibling of the given `leaf` node.
  NodeType* sibling = sub_root;
  while(!sibling->isLeaf())
  {
    sibling = sibling->children[select(*leaf, *(sibling->children[0]),
                                       *(sibling->children[1]))];
  }
  NodeType* prev = sibling->parent;
  // Create a new `node` that later will become the new parent of both the
  // `sibling` and the given `leaf`.
  NodeType* node = createNode(prev, leaf->bv, sibling->bv, nullptr);
  if(prev)
  // If the parent `prev` of the `sibling` is an interior node, we will
  // replace the `sibling` with the subtree {node {`sibling`, leaf}} like
  // this:
  //        Before                After
  //
  //          ╱                     ╱
  //        prev                  prev
  //        ╱  ╲        ─>        ╱  ╲
  //  sibling  ...             node  ...
  //                           ╱  ╲
  //                     sibling  leaf
  {
    prev->children[indexOf(sibling)] = node;
    node->children[0] = sibling; sibling->parent = node;
    node->children[1] = leaf; leaf->parent = node;
    // Now that we've inserted `leaf` some of the existing bounding
    // volumes might not fully enclose their children. Walk up the tree
    // looking for parents that don't already enclose their children
    // and create a new tight-fitting bounding volume for those.
    do
    {
      if(!prev->bv.contain(node->bv))
        prev->bv = prev->children[0]->bv + prev->children[1]->bv;
      else
        break;
      node = prev;
    } while (nullptr != (prev = node->parent));
  }
  else
  // If the `sibling` has no parent, i.e., the tree is a singleton,
  // we will replace it with the 3-node tree {node {`sibling`, `leaf`}} like
  // this:
  //
  //        node
  //        ╱  ╲
  //  sibling  leaf
  {
    node->children[0] = sibling; sibling->parent = node;
    node->children[1] = leaf; leaf->parent = node;
    root_node = node;
  }

  // Note that the above algorithm always adds the new `leaf` node as the right
  // child, i.e., children[1].  Calling removeLeaf(l) followed by calling
  // this function insertLeaf(l) where l is a left child will result in
  // switching l and its sibling even if no object's pose has changed.
}

//==============================================================================
template <typename BV>
typename HierarchyTree<BV>::NodeType*
HierarchyTree<BV>::removeLeaf(NodeType* const leaf) {
// Deletes `leaf` by replacing the subtree consisting of `leaf`, its sibling,
// and its parent with just its sibling. It then "tightens" the ancestor
// bounding volumes. Returns a pointer to the parent of the highest node whose
// BV changed due to the removal.
  if(leaf == root_node)
  {
    // If the `leaf` node is the only node in the tree, the tree becomes empty.
    root_node = nullptr;
    return nullptr;
  }
  NodeType* parent = leaf->parent;
  NodeType* prev = parent->parent;
  NodeType* sibling = parent->children[1-indexOf(leaf)];
  if(prev)
  {
    // If the parent node is not the root node, the sibling node will
    // replace the parent node like this:
    //
    //            Before              After
    //             ...                 ...
    //             ╱                   ╱
    //           prev                prev
    //          ╱   ╲               ╱   ╲
    //     parent   ...    ─>  sibling  ...
    //      ╱  ╲                 ╱╲
    //  leaf  sibling           ...
    //           ╱╲
    //          ...
    //
    // Step 1: replace the subtree {parent {leaf, sibling {...}}} with
    // {sibling {...}}.
    prev->children[indexOf(parent)] = sibling;
    sibling->parent = prev;
    deleteNode(parent);
    // Step 2: tighten up the BVs of the ancestor nodes.
    while(prev)
    {
      BV new_bv = prev->children[0]->bv + prev->children[1]->bv;
      if(!new_bv.equal(prev->bv))
      {
        prev->bv = new_bv;
        prev = prev->parent;
      }
      else break;
    }

    return prev ? prev : root_node;
  }
  else
  {
    // If the parent node is the root node, the sibling node will become the
    // root of the whole tree like this:
    //
    //     Before                   After
    //
    //     parent
    //      ╱  ╲
    //  leaf  sibling     ─>       sibling
    //           ╱╲                  ╱╲
    //          ...                 ...
    root_node = sibling;
    sibling->parent = nullptr;
    deleteNode(parent);
    return root_node;
  }
}

//==============================================================================
template<typename BV>
void HierarchyTree<BV>::fetchLeaves(NodeType* root, std::vector<NodeType*>& leaves, int depth)
{
  if((!root->isLeaf()) && depth)
  {
    fetchLeaves(root->children[0], leaves, depth-1);
    fetchLeaves(root->children[1], leaves, depth-1);
    deleteNode(root);
  }
  else
  {
    leaves.push_back(root);
  }
}

//==============================================================================
template<typename BV>
size_t HierarchyTree<BV>::indexOf(NodeType* node)
{
  // node cannot be nullptr
  return (node->parent->children[1] == node);
}

//==============================================================================
template<typename BV>
typename HierarchyTree<BV>::NodeType* HierarchyTree<BV>::createNode(NodeType* parent,
                                                                    const BV& bv,
                                                                    void* data)
{
  NodeType* node = createNode(parent, data);
  node->bv = bv;
  return node;
}

//==============================================================================
template<typename BV>
typename HierarchyTree<BV>::NodeType* HierarchyTree<BV>::createNode(NodeType* parent,
                                                                    const BV& bv1,
                                                                    const BV& bv2,
                                                                    void* data)
{
  NodeType* node = createNode(parent, data);
  node->bv = bv1 + bv2;
  return node;
}

//==============================================================================
template<typename BV>
typename HierarchyTree<BV>::NodeType* HierarchyTree<BV>::createNode(NodeType* parent, void* data)
{
  NodeType* node = nullptr;
  if(free_node)
  {
    node = free_node;
    free_node = nullptr;
  }
  else
    node = new NodeType();
  node->parent = parent;
  node->data = data;
  node->children[1] = 0;
  return node;
}

//==============================================================================
template<typename BV>
void HierarchyTree<BV>::deleteNode(NodeType* node)
{
  if(free_node != node)
  {
    delete free_node;
    free_node = node;
  }
}

//==============================================================================
template<typename BV>
void HierarchyTree<BV>::recurseDeleteNode(NodeType* node)
{
  if(!node->isLeaf())
  {
    recurseDeleteNode(node->children[0]);
    recurseDeleteNode(node->children[1]);
  }

  if(node == root_node) root_node = nullptr;
  deleteNode(node);
}

//==============================================================================
template<typename BV>
void HierarchyTree<BV>::recurseRefit(NodeType* node)
{
  if(!node->isLeaf())
  {
    recurseRefit(node->children[0]);
    recurseRefit(node->children[1]);
    node->bv = node->children[0]->bv + node->children[1]->bv;
  }
  else
    return;
}

//==============================================================================
template<typename BV>
bool nodeBaseLess(NodeBase<BV>* a, NodeBase<BV>* b, int d)
{
  if(a->bv.center()[d] < b->bv.center()[d]) return true;
  return false;
}

//==============================================================================
template <typename S, typename BV>
struct SelectImpl
{
  static std::size_t run(
      const NodeBase<BV>& /*query*/,
      const NodeBase<BV>& /*node1*/,
      const NodeBase<BV>& /*node2*/)
  {
    return 0;
  }

  static std::size_t run(
      const BV& /*query*/,
      const NodeBase<BV>& /*node1*/,
      const NodeBase<BV>& /*node2*/)
  {
    return 0;
  }
};

//==============================================================================
template<typename BV>
size_t select(
    const NodeBase<BV>& query,
    const NodeBase<BV>& node1,
    const NodeBase<BV>& node2)
{
  return SelectImpl<typename BV::S, BV>::run(query, node1, node2);
}

//==============================================================================
template<typename BV>
size_t select(
    const BV& query,
    const NodeBase<BV>& node1,
    const NodeBase<BV>& node2)
{
  return SelectImpl<typename BV::S, BV>::run(query, node1, node2);
}

//==============================================================================
template <typename S>
struct SelectImpl<S, AABB<S>>
{
  static std::size_t run(
      const NodeBase<AABB<S>>& node,
      const NodeBase<AABB<S>>& node1,
      const NodeBase<AABB<S>>& node2)
  {
    const AABB<S>& bv = node.bv;
    const AABB<S>& bv1 = node1.bv;
    const AABB<S>& bv2 = node2.bv;
    Vector3<S> v = bv.min_ + bv.max_;
    Vector3<S> v1 = v - (bv1.min_ + bv1.max_);
    Vector3<S> v2 = v - (bv2.min_ + bv2.max_);
    S d1 = fabs(v1[0]) + fabs(v1[1]) + fabs(v1[2]);
    S d2 = fabs(v2[0]) + fabs(v2[1]) + fabs(v2[2]);
    return (d1 < d2) ? 0 : 1;
  }

  static std::size_t run(
      const AABB<S>& query,
      const NodeBase<AABB<S>>& node1,
      const NodeBase<AABB<S>>& node2)
  {
    const AABB<S>& bv = query;
    const AABB<S>& bv1 = node1.bv;
    const AABB<S>& bv2 = node2.bv;
    Vector3<S> v = bv.min_ + bv.max_;
    Vector3<S> v1 = v - (bv1.min_ + bv1.max_);
    Vector3<S> v2 = v - (bv2.min_ + bv2.max_);
    S d1 = fabs(v1[0]) + fabs(v1[1]) + fabs(v1[2]);
    S d2 = fabs(v2[0]) + fabs(v2[1]) + fabs(v2[2]);
    return (d1 < d2) ? 0 : 1;
  }
};

} // namespace detail
} // namespace fcl

#endif

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

#ifndef FCL_HIERARCHY_TREE_H
#define FCL_HIERARCHY_TREE_H

#include <vector>
#include "fcl/BV/AABB.h"
#include "fcl/vec_3f.h"
#include <boost/bind.hpp>

namespace fcl
{


template<typename BV>
struct NodeBase
{
  BV bv;
  NodeBase<BV>* parent;
  bool isLeaf() const { return (childs[1] == NULL); }
  bool isInternal() const { return !isLeaf(); }
  union
  {
    NodeBase<BV>* childs[2];
    void* data;
  };
};

template<typename BV>
bool nodeBaseLess(NodeBase<BV>* a, NodeBase<BV>* b, int d)
{
  if(a->bv.center()[d] < b->bv.center()[d]) return true;
  return false;
}

template<typename BV>
size_t select(const NodeBase<BV>& query, const NodeBase<BV>& node1, const NodeBase<BV>& node2)
{
  return 0;
}

template<>
size_t select(const NodeBase<AABB>& node, const NodeBase<AABB>& node1, const NodeBase<AABB>& node2);

template<typename BV>
size_t select(const BV& query, const NodeBase<BV>& node1, const NodeBase<BV>& node2)
{
  return 0;
}

template<>
size_t select(const AABB& query, const NodeBase<AABB>& node1, const NodeBase<AABB>& node2);



template<typename BV>
class HierarchyTree
{
  typedef NodeBase<BV> NodeType;
  typedef typename std::vector<NodeBase<BV>* >::iterator NodeVecIterator;
public:

  HierarchyTree()
  {
    root_node = NULL;
    n_leaves = 0;
    free_node = NULL;
    max_lookahead_level = -1;
    opath = 0;
  }

  ~HierarchyTree()
  {
    clear();
  }

  void init(std::vector<NodeType*>& leaves, int bu_threshold)
  {
    clear();
    root_node = topdown(leaves.begin(), leaves.end(), bu_threshold);
    n_leaves = leaves.size();
    free_node = NULL;
    max_lookahead_level = -1;
    opath = 0;
  }

  /** \brief Insest a node */
  NodeType* insert(const BV& bv, void* data)
  {
    NodeType* leaf = createNode(NULL, bv, data);
    insertLeaf(root_node, leaf);
    ++n_leaves;
    return leaf;
  }

  /** \brief Remove a leaf node */
  void remove(NodeType* leaf)
  {
    removeLeaf(leaf);
    deleteNode(leaf);
    --n_leaves;
  }

  /** \brief Clear the tree */
  void clear()
  {
    if(root_node)
      recurseDeleteNode(root_node);
    delete free_node;
    free_node = NULL;
    max_lookahead_level = -1;
    opath = 0;
  }

  /** \brief Whether the tree is empty */
  bool empty() const
  {
    return (NULL == root_node);
  }

  /** \brief update one leaf node's bounding volume */
  void update_(NodeType* leaf, const BV& bv)
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

  /** \brief update one leaf node */
  void update(NodeType* leaf, int lookahead_level = -1)
  {
    NodeType* root = removeLeaf(leaf);
    if(root)
    {
      if(lookahead_level > 0)
      {
        for(int i = 0; (i < lookahead_level) && root->parent; ++i)
          root = root->parent;
      }
      else
        root = root_node;
    }
    insertLeaf(root, leaf);
  }

  bool update(NodeType* leaf, const BV& bv)
  {
    if(leaf->bv.contain(bv)) return false;
    update_(leaf, bv);
    return true;
  }

  /** \brief update one leaf's bounding volume, with prediction */
  bool update(NodeType* leaf, const BV& bv, const Vec3f& vel, BVH_REAL margin)
  {
    if(leaf->bv.contain(bv)) return false;
    update_(leaf, bv);
    return true;
  }

  /** \brief update one leaf's bounding volume, with prediction */
  bool update(NodeType* leaf, const BV& bv, const Vec3f& vel)
  {
    if(leaf->bv.contain(bv)) return false;
    update_(leaf, bv);
    return true;
  }

  /** \brief balance the tree from bottom */
  void balanceBottomup()
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

  /** \brief balance the tree from top */
  void balanceTopdown(int bu_threshold = 128)
  {
    if(root_node)
    {
      std::vector<NodeType*> leaves;
      leaves.reserve(n_leaves);
      fetchLeaves(root_node, leaves);
      root_node = topdown(leaves.begin(), leaves.end(), bu_threshold);
    }
  }

  /** \brief refit */
  void refit()
  {
    if(root_node)
    {
      recurseRefit(root_node);
    }
  }

  void recurseRefit(NodeType* node)
  {
    if(!node->isLeaf())
    {
      recurseRefit(node->childs[0]);
      recurseRefit(node->childs[1]);
      node->bv = node->childs[0]->bv + node->childs[1]->bv;
    }
    else
      return;
  }

  size_t getMaxHeight(NodeType* node) const
  {
    if(!node->isLeaf())
    {
      size_t height1 = getMaxHeight(node->childs[0]);
      size_t height2 = getMaxHeight(node->childs[1]);
      return std::max(height1, height2) + 1;
    }
    else
      return 0;
  }

  void getMaxDepth(NodeType* node, size_t depth, size_t& max_depth) const
  {
    if(!node->isLeaf())
    {
      getMaxDepth(node->childs[0], depth+1, max_depth);
      getMaxDepth(node->childs[1], depth+1, max_depth);
    }
    else
      max_depth = std::max(max_depth, depth);
  }

  /** \brief balance the tree in an incremental way */
  void balanceIncremental(int passes)
  {
    if(passes < 0) passes = n_leaves;
    if(root_node && (passes > 0))
    {
      for(int i = 0; i < passes; ++i)
      {
        NodeType* node = root_node;
        unsigned int bit = 0;
        while(!node->isLeaf())
        {
          node = sort(node, root_node)->childs[(opath>>bit)&1];
          bit = (bit+1)&(sizeof(unsigned int) * 8-1);
        }
        update(node);
        ++opath;
      }
    }
  }

  /** \brief extract all the leaves of the tree */
  void extractLeaves(const NodeType* root, std::vector<NodeType*>& leaves) const
  {
    if(!root->isLeaf())
    {
      extractLeaves(root->childs[0], leaves);
      extractLeaves(root->childs[1], leaves);
    }
    else
      leaves.push_back(root);
  }

  size_t size() const
  {
    return n_leaves;
  }

  inline NodeType* getRoot() const
  {
    return root_node;
  }

  inline NodeType*& getRoot()
  {
    return root_node;
  }

  void print(NodeType* root, int depth)
  {
    for(int i = 0; i < depth; ++i)
      std::cout << " ";
    std::cout << " (" << root->bv.min_[0] << ", " << root->bv.min_[1] << ", " << root->bv.min_[2] << "; " << root->bv.max_[0] << ", " << root->bv.max_[1] << ", " << root->bv.max_[2] << ")" << std::endl;
    if(root->isLeaf())
    {
    }
    else
    {
      print(root->childs[0], depth+1);
      print(root->childs[1], depth+1);
    }
  }

  /** \brief construct a tree for a set of leaves from bottom -- very heavy way */
  void bottomup(const NodeVecIterator lbeg, const NodeVecIterator lend)
  {
    NodeVecIterator lcur_end = lend;
    while(lbeg < lcur_end - 1)
    {
      NodeVecIterator min_it1, min_it2;
      BVH_REAL min_size = std::numeric_limits<BVH_REAL>::max();
      for(NodeVecIterator it1 = lbeg; it1 < lcur_end; ++it1)
      {
        for(NodeVecIterator it2 = it1 + 1; it2 < lcur_end; ++it2)
        {
          BVH_REAL cur_size = ((*it1)->bv + (*it2)->bv).size();
          if(cur_size < min_size)
          {
            min_size = cur_size;
            min_it1 = it1;
            min_it2 = it2;
          }
        }
      }
      
      NodeType* n[2] = {*min_it1, *min_it2};
      NodeType* p = createNode(NULL, n[0]->bv, n[1]->bv, NULL);
      p->childs[0] = n[0];
      p->childs[1] = n[1];
      n[0]->parent = p;
      n[1]->parent = p;
      *min_it1 = p;
      NodeType* tmp = *min_it2;
      lcur_end--;
      *min_it2 = *lcur_end;
      *lcur_end = tmp;
    }
  }

  /** \brief construct a tree for a set of leaves from top */
  NodeType* topdown(const NodeVecIterator lbeg, const NodeVecIterator lend, int bu_threshold)
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
        BVH_REAL extent[3] = {vol.width(), vol.height(), vol.depth()};
        if(extent[1] > extent[0]) best_axis = 1;
        if(extent[2] > extent[best_axis]) best_axis = 2;

        // compute median 
        NodeVecIterator lcenter = lbeg + num_leaves / 2;
        std::nth_element(lbeg, lcenter, lend, boost::bind(&nodeBaseLess<BV>, _1, _2, boost::ref(best_axis)));

        NodeType* node = createNode(NULL, vol, NULL);
        node->childs[0] = topdown(lbeg, lcenter, bu_threshold);
        node->childs[1] = topdown(lcenter, lend, bu_threshold);
        node->childs[0]->parent = node;
        node->childs[1]->parent = node;
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


  NodeType* topdown2(const NodeVecIterator lbeg, const NodeVecIterator lend, int bu_threshold)
  {
    int num_leaves = lend - lbeg;
    if(num_leaves > 1)
    {
      if(num_leaves > bu_threshold)
      {
        Vec3f split_p = (*lbeg)->bv.center();
        BV vol = (*lbeg)->bv;
        NodeVecIterator it;
        for(it = lbeg + 1; it < lend; ++it)
        {
          split_p += (*it)->bv.center();
          vol += (*it)->bv;
        }
        split_p /= (BVH_REAL)(num_leaves);
        int best_axis = -1;
        int bestmidp = num_leaves;
        int splitcount[3][2] = {{0,0}, {0,0}, {0,0}};
        for(it = lbeg; it < lend; ++it)
        {
          Vec3f x = (*it)->bv.center() - split_p;
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

        BVH_REAL split_value = split_p[best_axis];
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

        NodeType* node = createNode(NULL, vol, NULL);
        node->childs[0] = topdown2(lbeg, lcenter, bu_threshold);
        node->childs[1] = topdown2(lcenter, lend, bu_threshold);
        node->childs[0]->parent = node;
        node->childs[1]->parent = node;
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
  
private:

  /** \brief sort node n and its parent according to their memory position */
  NodeType* sort(NodeType* n, NodeType*& r)
  {
    NodeType* p = n->parent;
    if(p > n)
    {
      int i = indexOf(n);
      int j = 1 - i;
      NodeType* s = p->childs[j];
      NodeType* q = p->parent;
      if(q) q->childs[indexOf(p)] = n; else r = n;
      s->parent = n;
      p->parent = n;
      n->parent = q;
      p->childs[0] = n->childs[0];
      p->childs[1] = n->childs[1];
      n->childs[0]->parent = p;
      n->childs[1]->parent = p;
      n->childs[i] = p;
      n->childs[j] = s;
      std::swap(p->bv, n->bv);
      return p;
    }
    return n;
  }
  
  /** \brief Insert a leaf node and also update its ancestors */
  void insertLeaf(NodeType* root, NodeType* leaf)
  {
    if(!root_node)
    {
      root_node = leaf;
      leaf->parent = NULL;
    }
    else
    {
      if(!root->isLeaf())
      {
        do
        {
          root = root->childs[select(*leaf, *(root->childs[0]), *(root->childs[1]))];
        }
        while(!root->isLeaf());
      }

      NodeType* prev = root->parent;
      NodeType* node = createNode(prev, leaf->bv, root->bv, NULL);
      if(prev)
      {
        prev->childs[indexOf(root)] = node;
        node->childs[0] = root; root->parent = node;
        node->childs[1] = leaf; leaf->parent = node;
        do
        {
          if(!prev->bv.contain(node->bv))
            prev->bv = prev->childs[0]->bv + prev->childs[1]->bv;
          else 
            break;
          node = prev;
        } while (NULL != (prev = node->parent));
      }
      else
      {
        node->childs[0] = root; root->parent = node;
        node->childs[1] = leaf; leaf->parent = node;
        root_node = node;
      }
    }
  }

  /** \brief Remove a leaf. The leaf node itself is not deleted yet, but all the unnecessary internal nodes are deleted.
      return the node with the smallest depth and is influenced by the remove operation */
  NodeType* removeLeaf(NodeType* leaf)
  {
    if(leaf == root_node)
    {
      root_node = NULL;
      return NULL; 
    }
    else
    {
      NodeType* parent = leaf->parent;
      NodeType* prev = parent->parent;
      NodeType* sibling = parent->childs[1-indexOf(leaf)];
      if(prev)
      {
        prev->childs[indexOf(parent)] = sibling;
        sibling->parent = prev;
        deleteNode(parent);
        while(prev)
        {
          BV new_bv = prev->childs[0]->bv + prev->childs[1]->bv;
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
        root_node = sibling;
        sibling->parent = NULL;
        deleteNode(parent);
        return root_node;
      }
    }
  }

  /** \brief Delete all internal nodes and return all leaves nodes with given depth from root */
  void fetchLeaves(NodeType* root, std::vector<NodeType*>& leaves, int depth = -1)
  {
    if((!root->isLeaf()) && depth)
    {
      fetchLeaves(root->childs[0], leaves, depth-1);
      fetchLeaves(root->childs[1], leaves, depth-1);
      deleteNode(root);
    }
    else
    {
      leaves.push_back(root);
    }
  }

  static size_t indexOf(NodeType* node)
  {
    // node cannot be NULL
    return (node->parent->childs[1] == node);
  }
  
  NodeType* createNode(NodeType* parent, 
                       const BV& bv,
                       void* data)
  {
    NodeType* node = createNode(parent, data);
    node->bv = bv;
    return node;
  }

  /** \brief create one node (leaf or internal */
  NodeType* createNode(NodeType* parent,
                       const BV& bv1,
                       const BV& bv2,
                       void* data)
  {
    NodeType* node = createNode(parent, data);
    node->bv = bv1 + bv2;
    return node;
  }

  
  inline NodeType* createNode(NodeType* parent,
                              void* data)
  {
    NodeType* node = NULL;
    if(free_node)
    {
      node = free_node;
      free_node = NULL;
    }
    else
      node = new NodeType();
    node->parent = parent;
    node->data = data;
    node->childs[1] = 0;
    return node;
  }

  inline void deleteNode(NodeType* node)
  {
    if(free_node != node)
    {
      delete free_node;
      free_node = node;
    }
  }

  void recurseDeleteNode(NodeType* node)
  {
    if(!node->isLeaf())
    {
      recurseDeleteNode(node->childs[0]);
      recurseDeleteNode(node->childs[1]);
    }
    if(node == root_node) root_node = NULL;
    deleteNode(node);
  }

  static BV bounds(const std::vector<NodeType*>& leaves)
  {
    if(leaves.size() == 0) return BV();
    BV bv = leaves[0]->bv;
    for(size_t i = 1; i < leaves.size(); ++i)
    {
      bv += leaves[i]->bv;
    }

    return bv;
  }

  static BV bounds(const NodeVecIterator lbeg, const NodeVecIterator lend)
  {
    if(lbeg == lend) return BV();
    BV bv = *lbeg;
    for(NodeVecIterator it = lbeg + 1; it < lend; ++it)
    {
      bv += (*it)->bv;
    }

    return bv;
  }
  
protected:
  NodeType* root_node;
  size_t n_leaves;

  unsigned int opath;

  NodeType* free_node; // This is a one NodeType cache, the reason is that we need to remove a node and then add it again frequently. 

  int max_lookahead_level;


};


template<>
bool HierarchyTree<AABB>::update(NodeBase<AABB>* leaf, const AABB& bv_, const Vec3f& vel, BVH_REAL margin);

template<>
bool HierarchyTree<AABB>::update(NodeBase<AABB>* leaf, const AABB& bv_, const Vec3f& vel);



}


#endif

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

#include "fcl/interval_tree.h"
#include <iostream>
#include <cstdlib>


namespace fcl
{

IntervalTreeNode::IntervalTreeNode(){}

IntervalTreeNode::IntervalTreeNode(Interval* new_interval) :
    stored_interval (new_interval),
    key(new_interval->low),
    high(new_interval->high),
    max_high(high) {}

IntervalTreeNode::~IntervalTreeNode(){}


IntervalTree::IntervalTree()
{
  nil = new IntervalTreeNode;
  nil->left = nil->right = nil->parent = nil;
  nil->red = false;
  nil->key = nil->high = nil->max_high = -std::numeric_limits<double>::max();
  nil->stored_interval = NULL;

  root = new IntervalTreeNode;
  root->parent = root->left = root->right = nil;
  root->key = root->high = root->max_high = std::numeric_limits<double>::max();
  root->red = false;
  root->stored_interval = NULL;

  /* the following are used for the query function */
  recursion_node_stack_size = 128;
  recursion_node_stack = (it_recursion_node*)malloc(recursion_node_stack_size*sizeof(it_recursion_node));
  recursion_node_stack_top = 1;
  recursion_node_stack[0].start_node = NULL;
}


void IntervalTree::leftRotate(IntervalTreeNode* x)
{
  IntervalTreeNode* y;

  y = x->right;
  x->right = y->left;

  if(y->left != nil) y->left->parent = x;

  y->parent = x->parent;

  if(x == x->parent->left)
    x->parent->left = y;
  else
    x->parent->right = y;

  y->left = x;
  x->parent = y;

  x->max_high = std::max(x->left->max_high, std::max(x->right->max_high, x->high));
  y->max_high = std::max(x->max_high, std::max(y->right->max_high, y->high));
}


void IntervalTree::rightRotate(IntervalTreeNode* y)
{
  IntervalTreeNode* x;

  x = y->left;
  y->left = x->right;

  if(nil != x->right)  x->right->parent = y;

  x->parent = y->parent;
  if(y == y->parent->left)
    y->parent->left = x;
  else
    y->parent->right = x;

  x->right = y;
  y->parent = x;

  y->max_high = std::max(y->left->max_high, std::max(y->right->max_high, y->high));
  x->max_high = std::max(x->left->max_high, std::max(y->max_high, x->high));
}



/** \brief Inserts z into the tree as if it were a regular binary tree */
void IntervalTree::recursiveInsert(IntervalTreeNode* z)
{
  IntervalTreeNode* x;
  IntervalTreeNode* y;

  z->left = z->right = nil;
  y = root;
  x = root->left;
  while(x != nil)
  {
    y = x;
    if(x->key > z->key)
      x = x->left;
    else
      x = x->right;
  }
  z->parent = y;
  if((y == root) || (y->key > z->key))
    y->left = z;
  else
    y->right = z;
}


void IntervalTree::fixupMaxHigh(IntervalTreeNode* x)
{
  while(x != root)
  {
    x->max_high = std::max(x->high, std::max(x->left->max_high, x->right->max_high));
    x = x->parent;
  }
}

IntervalTreeNode* IntervalTree::insert(Interval* new_interval)
{
  IntervalTreeNode* y;
  IntervalTreeNode* x;
  IntervalTreeNode* new_node;

  x = new IntervalTreeNode(new_interval);
  recursiveInsert(x);
  fixupMaxHigh(x->parent);
  new_node = x;
  x->red = true;
  while(x->parent->red)
  {
    /* use sentinel instead of checking for root */
    if(x->parent == x->parent->parent->left)
    {
      y = x->parent->parent->right;
      if(y->red)
      {
        x->parent->red = true;
        y->red = true;
        x->parent->parent->red = true;
        x = x->parent->parent;
      }
      else
      {
        if(x == x->parent->right)
        {
          x = x->parent;
          leftRotate(x);
        }
        x->parent->red = false;
        x->parent->parent->red = true;
        rightRotate(x->parent->parent);
      }
    }
    else
    {
      y = x->parent->parent->left;
      if(y->red)
      {
        x->parent->red = false;
        y->red = false;
        x->parent->parent->red = true;
        x = x->parent->parent;
      }
      else
      {
        if(x == x->parent->left)
        {
          x = x->parent;
          rightRotate(x);
        }
        x->parent->red = false;
        x->parent->parent->red = true;
        leftRotate(x->parent->parent);
      }
    }
  }
  root->left->red = false;
  return new_node;
}

IntervalTreeNode* IntervalTree::getSuccessor(IntervalTreeNode* x) const
{
  IntervalTreeNode* y;

  if(nil != (y = x->right))
  {
    while(y->left != nil)
      y = y->left;
    return y;
  }
  else
  {
    y = x->parent;
    while(x == y->right)
    {
      x = y;
      y = y->parent;
    }
    if(y == root) return nil;
    return y;
  }
}


IntervalTreeNode* IntervalTree::getPredecessor(IntervalTreeNode* x) const
{
  IntervalTreeNode* y;

  if(nil != (y = x->left))
  {
    while(y->right != nil)
      y = y->right;
    return y;
  }
  else
  {
    y = x->parent;
    while(x == y->left)
    {
      if(y == root) return nil;
      x = y;
      y = y->parent;
    }
    return y;
  }
}

void IntervalTreeNode::print(IntervalTreeNode* nil, IntervalTreeNode* root) const
{
  stored_interval->print();
  std::cout << ", k = " << key << ", h = " << high << ", mH = " << max_high;
  std::cout << "  l->key = ";
  if(left == nil) std::cout << "NULL"; else std::cout << left->key;
  std::cout << "  r->key = ";
  if(right == nil) std::cout << "NULL"; else std::cout << right->key;
  std::cout << "  p->key = ";
  if(parent == root) std::cout << "NULL"; else std::cout << parent->key;
  std::cout << "  red = " << (int)red << std::endl;
}

void IntervalTree::recursivePrint(IntervalTreeNode* x) const
{
  if(x != nil)
  {
    recursivePrint(x->left);
    x->print(nil,root);
    recursivePrint(x->right);
  }
}

IntervalTree::~IntervalTree()
{
  IntervalTreeNode* x = root->left;
  std::deque<IntervalTreeNode*> nodes_to_free;

  if(x != nil)
  {
    if(x->left != nil)
    {
      nodes_to_free.push_back(x->left);
    }
    if(x->right != nil)
    {
      nodes_to_free.push_back(x->right);
    }

    delete x;
    while( nodes_to_free.size() > 0)
    {
      x = nodes_to_free.back();
      nodes_to_free.pop_back();
      if(x->left != nil)
      {
        nodes_to_free.push_back(x->left);
      }
      if(x->right != nil)
      {
        nodes_to_free.push_back(x->right);
      }
      delete x;
    }
  }
  delete nil;
  delete root;
  free(recursion_node_stack);
}


void IntervalTree::print() const
{
  recursivePrint(root->left);
}

void IntervalTree::deleteFixup(IntervalTreeNode* x)
{
  IntervalTreeNode* w;
  IntervalTreeNode* root_left_node = root->left;

  while((!x->red) && (root_left_node != x))
  {
    if(x == x->parent->left)
    {
      w = x->parent->right;
      if(w->red)
      {
        w->red = false;
        x->parent->red = true;
        leftRotate(x->parent);
        w = x->parent->right;
      }
      if((!w->right->red) && (!w->left->red))
      {
        w->red = true;
        x = x->parent;
      }
      else
      {
        if(!w->right->red)
        {
          w->left->red = false;
          w->red = true;
          rightRotate(w);
          w = x->parent->right;
        }
        w->red = x->parent->red;
        x->parent->red = false;
        w->right->red = false;
        leftRotate(x->parent);
        x = root_left_node;
      }
    }
    else
    {
      w = x->parent->left;
      if(w->red)
      {
        w->red = false;
        x->parent->red = true;
        rightRotate(x->parent);
        w = x->parent->left;
      }
      if((!w->right->red) && (!w->left->red))
      {
        w->red = true;
        x = x->parent;
      }
      else
      {
        if(!w->left->red)
        {
          w->right->red = false;
          w->red = true;
          leftRotate(w);
          w = x->parent->left;
        }
        w->red = x->parent->red;
        x->parent->red = false;
        w->left->red = false;
        rightRotate(x->parent);
        x = root_left_node;
      }
    }
  }
  x->red = false;
}

Interval* IntervalTree::deleteNode(IntervalTreeNode* z)
{
  IntervalTreeNode* y;
  IntervalTreeNode* x;
  Interval* node_to_delete = z->stored_interval;

  y= ((z->left == nil) || (z->right == nil)) ? z : getSuccessor(z);
  x= (y->left == nil) ? y->right : y->left;
  if(root == (x->parent = y->parent))
  {
    root->left = x;
  }
  else
  {
    if(y == y->parent->left)
    {
      y->parent->left = x;
    }
    else
    {
      y->parent->right = x;
    }
  }

  /* y should not be nil in this case */
  /* y is the node to splice out and x is its child */
  if(y != z)
  {
    y->max_high = -std::numeric_limits<double>::max();
    y->left = z->left;
    y->right = z->right;
    y->parent = z->parent;
    z->left->parent = z->right->parent = y;
    if(z == z->parent->left)
      z->parent->left = y;
    else
      z->parent->right = y;

    fixupMaxHigh(x->parent);
    if(!(y->red))
    {
      y->red = z->red;
      deleteFixup(x);
    }
    else
      y->red = z->red;
    delete z;
  }
  else
  {
    fixupMaxHigh(x->parent);
    if(!(y->red)) deleteFixup(x);
    delete y;
  }

  return node_to_delete;
}

/** \brief returns 1 if the intervals overlap, and 0 otherwise */
bool overlap(double a1, double a2, double b1, double b2)
{
  if(a1 <= b1)
  {
    return (b1 <= a2);
  }
  else
  {
    return (a1 <= b2);
  }
}

std::deque<Interval*> IntervalTree::query(double low, double high)
{
  std::deque<Interval*> result_stack;
  IntervalTreeNode* x = root->left;
  bool run = (x != nil);

  current_parent = 0;

  while(run)
  {
    if(overlap(low,high,x->key,x->high))
    {
      result_stack.push_back(x->stored_interval);
      recursion_node_stack[current_parent].try_right_branch = true;
    }
    if(x->left->max_high >= low)
    {
      if(recursion_node_stack_top == recursion_node_stack_size)
      {
        recursion_node_stack_size *= 2;
        recursion_node_stack = (it_recursion_node *)realloc(recursion_node_stack, recursion_node_stack_size * sizeof(it_recursion_node));
        if(recursion_node_stack == NULL)
          exit(1);
      }
      recursion_node_stack[recursion_node_stack_top].start_node = x;
      recursion_node_stack[recursion_node_stack_top].try_right_branch = false;
      recursion_node_stack[recursion_node_stack_top].parent_index = current_parent;
      current_parent = recursion_node_stack_top++;
      x = x->left;
    }
    else
      x = x->right;

    run = (x != nil);
    while((!run) && (recursion_node_stack_top > 1))
    {
      if(recursion_node_stack[--recursion_node_stack_top].try_right_branch)
      {
        x=recursion_node_stack[recursion_node_stack_top].start_node->right;
        current_parent=recursion_node_stack[recursion_node_stack_top].parent_index;
        recursion_node_stack[current_parent].try_right_branch = true;
        run = (x != nil);
      }
    }
  }
  return result_stack;
}

}

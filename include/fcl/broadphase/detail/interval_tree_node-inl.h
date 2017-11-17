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

#ifndef FCL_BROADPHASE_DETAIL_INTERVALTREENODE_INL_H
#define FCL_BROADPHASE_DETAIL_INTERVALTREENODE_INL_H

#include "fcl/broadphase/detail/interval_tree_node.h"

#include <iostream>

namespace fcl {
namespace detail {

//==============================================================================
extern template
class FCL_EXPORT IntervalTreeNode<double>;

//==============================================================================
template <typename S>
IntervalTreeNode<S>::IntervalTreeNode()
{
  // Do nothing
}

//==============================================================================
template <typename S>
IntervalTreeNode<S>::IntervalTreeNode(SimpleInterval<S>* new_interval)
  : stored_interval (new_interval),
    key(new_interval->low),
    high(new_interval->high),
    max_high(high)
{
  // Do nothing
}

//==============================================================================
template <typename S>
IntervalTreeNode<S>::~IntervalTreeNode()
{
  // Do nothing
}

//==============================================================================
template <typename S>
void IntervalTreeNode<S>::print(
    IntervalTreeNode<S>* nil, IntervalTreeNode<S>* root) const
{
  stored_interval->print();
  std::cout << ", k = " << key << ", h = " << high << ", mH = " << max_high;
  std::cout << "  l->key = ";
  if(left == nil) std::cout << "nullptr"; else std::cout << left->key;
  std::cout << "  r->key = ";
  if(right == nil) std::cout << "nullptr"; else std::cout << right->key;
  std::cout << "  p->key = ";
  if(parent == root) std::cout << "nullptr"; else std::cout << parent->key;
  std::cout << "  red = " << (int)red << std::endl;
}

} // namespace detail
} // namespace fcl

#endif

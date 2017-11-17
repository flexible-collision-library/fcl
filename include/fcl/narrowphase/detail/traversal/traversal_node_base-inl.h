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

#ifndef FCL_TRAVERSAL_TRAVERSALNODEBASE_INL_H
#define FCL_TRAVERSAL_TRAVERSALNODEBASE_INL_H

#include "fcl/narrowphase/detail/traversal/traversal_node_base.h"

#include "fcl/common/unused.h"

namespace fcl
{

namespace detail
{

//==============================================================================
extern template
class FCL_EXPORT TraversalNodeBase<double>;

//==============================================================================
template <typename S>
TraversalNodeBase<S>::~TraversalNodeBase()
{
  // Do nothing
}

//==============================================================================
template <typename S>
void TraversalNodeBase<S>::preprocess()
{
  // Do nothing
}

//==============================================================================
template <typename S>
void TraversalNodeBase<S>::postprocess()
{
  // Do nothing
}

//==============================================================================
template <typename S>
bool TraversalNodeBase<S>::isFirstNodeLeaf(int b) const
{
  FCL_UNUSED(b);

  return true;
}

//==============================================================================
template <typename S>
bool TraversalNodeBase<S>::isSecondNodeLeaf(int b) const
{
  FCL_UNUSED(b);

  return true;
}

//==============================================================================
template <typename S>
bool TraversalNodeBase<S>::firstOverSecond(int b1, int b2) const
{
  FCL_UNUSED(b1);
  FCL_UNUSED(b2);

  return true;
}

//==============================================================================
template <typename S>
int TraversalNodeBase<S>::getFirstLeftChild(int b) const
{
  return b;
}

//==============================================================================
template <typename S>
int TraversalNodeBase<S>::getFirstRightChild(int b) const
{
  return b;
}

//==============================================================================
template <typename S>
int TraversalNodeBase<S>::getSecondLeftChild(int b) const
{
  return b;
}

//==============================================================================
template <typename S>
int TraversalNodeBase<S>::getSecondRightChild(int b) const
{
  return b;
}

} // namespace detail
} // namespace fcl

#endif

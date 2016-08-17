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

#ifndef FCL_CONTACT_INL_H
#define FCL_CONTACT_INL_H

#include "fcl/narrowphase/contact.h"

namespace fcl
{

//==============================================================================
extern template
struct Contact<double>;

//==============================================================================
template <typename S>
Contact<S>::Contact()
  : o1(nullptr),
    o2(nullptr),
    b1(NONE),
    b2(NONE)
{
  // Do nothing
}

//==============================================================================
template <typename S>
Contact<S>::Contact(
    const CollisionGeometry<S>* o1_,
    const CollisionGeometry<S>* o2_,
    int b1_,
    int b2_)
  : o1(o1_),
    o2(o2_),
    b1(b1_),
    b2(b2_)
{
  // Do nothing
}

//==============================================================================
template <typename S>
Contact<S>::Contact(
    const CollisionGeometry<S>* o1_,
    const CollisionGeometry<S>* o2_,
    int b1_,
    int b2_,
    const Vector3<S>& pos_,
    const Vector3<S>& normal_,
    S depth_)
  : o1(o1_),
    o2(o2_),
    b1(b1_),
    b2(b2_),
    normal(normal_),
    pos(pos_),
    penetration_depth(depth_)
{
  // Do nothing
}

//==============================================================================
template <typename S>
bool Contact<S>::operator <(const Contact& other) const
{
  if(b1 == other.b1)
    return b2 < other.b2;
  return b1 < other.b1;
}

} // namespace fcl

#endif

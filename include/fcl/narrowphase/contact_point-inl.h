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

#ifndef FCL_CONTACTPOINT_INL_H
#define FCL_CONTACTPOINT_INL_H

#include "fcl/narrowphase/contact_point.h"

namespace fcl
{

//==============================================================================
extern template
struct ContactPoint<double>;

//==============================================================================
extern template
bool comparePenDepth(
    const ContactPoint<double>& _cp1, const ContactPoint<double>& _cp2);

//==============================================================================
extern template
void flipNormal(std::vector<ContactPoint<double>>& contacts);

//==============================================================================
template <typename S>
ContactPoint<S>::ContactPoint()
  : normal(Vector3<S>::Zero()),
    pos(Vector3<S>::Zero()),
    penetration_depth(0.0)
{
  // Do nothing
}

//==============================================================================
template <typename S>
ContactPoint<S>::ContactPoint(
    const Vector3<S>& n_, const Vector3<S>& p_, S d_)
  : normal(n_), pos(p_), penetration_depth(d_)
{
  // Do nothing
}

//==============================================================================
template <typename S>
bool comparePenDepth(const ContactPoint<S>& _cp1, const ContactPoint<S>& _cp2)
{
  return _cp1.penetration_depth < _cp2.penetration_depth;
}

//==============================================================================
template <typename S>
void flipNormal(std::vector<ContactPoint<S>>& contacts)
{
  for (auto& contact : contacts)
    contact.normal *= -1.0;
}

} // namespace fcl

#endif

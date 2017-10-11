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

#ifndef FCL_CONTACTPOINT_H
#define FCL_CONTACTPOINT_H

#include "fcl/common/types.h"

namespace fcl
{

/// @brief Minimal contact information returned by collision
template <typename S>
struct FCL_VISIBLE ContactPoint
{
  /// @brief Contact normal, pointing from o1 to o2
  Vector3<S> normal;

  /// @brief Contact position, in world space
  Vector3<S> pos;

  /// @brief Penetration depth
  S penetration_depth;

  /// @brief Constructor
  ContactPoint();

  /// @brief Constructor
  ContactPoint(const Vector3<S>& n_, const Vector3<S>& p_, S d_);
};

using ContactPointf = ContactPoint<float>;
using ContactPointd = ContactPoint<double>;

/// @brief Return true if _cp1's penentration depth is less than _cp2's.
template <typename S>
FCL_VISIBLE
bool comparePenDepth(
    const ContactPoint<S>& _cp1, const ContactPoint<S>& _cp2);

template <typename S>
FCL_VISIBLE
void flipNormal(std::vector<ContactPoint<S>>& contacts);

} // namespace fcl

#include "fcl/narrowphase/contact_point-inl.h"

#endif

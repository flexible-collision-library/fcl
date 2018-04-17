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

#ifndef FCL_COLLISIONREQUEST_INL_H
#define FCL_COLLISIONREQUEST_INL_H

#include "fcl/narrowphase/collision_request.h"

#include "fcl/narrowphase/collision_result.h"

namespace fcl
{

//==============================================================================
extern template
struct CollisionRequest<double>;

//==============================================================================
template <typename S>
CollisionRequest<S>::CollisionRequest(
    size_t num_max_contacts_,
    bool enable_contact_,
    size_t num_max_cost_sources_,
    bool enable_cost_,
    bool use_approximate_cost_,
    GJKSolverType gjk_solver_type_,
    Real gjk_tolerance_)
  : num_max_contacts(num_max_contacts_),
    enable_contact(enable_contact_),
    num_max_cost_sources(num_max_cost_sources_),
    enable_cost(enable_cost_),
    use_approximate_cost(use_approximate_cost_),
    gjk_solver_type(gjk_solver_type_),
    enable_cached_gjk_guess(false),
    cached_gjk_guess(Vector3<S>::UnitX()),
    gjk_tolerance(gjk_tolerance_)
{
  // Do nothing
}

//==============================================================================
template <typename S>
bool CollisionRequest<S>::isSatisfied(const CollisionResult<S>& result) const
{
  return (!enable_cost)
      && result.isCollision()
      && (num_max_contacts <= result.numContacts());
}

} // namespace fcl

#endif

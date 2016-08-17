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

#ifndef FCL_COSTSOURCE_INL_H
#define FCL_COSTSOURCE_INL_H

#include "fcl/narrowphase/cost_source.h"

namespace fcl
{

//==============================================================================
extern template
struct CostSource<double>;

//==============================================================================
template <typename S>
CostSource<S>::CostSource(
    const Vector3<S>& aabb_min_,
    const Vector3<S>& aabb_max_,
    S cost_density_)
  : aabb_min(aabb_min_),
    aabb_max(aabb_max_),
    cost_density(cost_density_)
{
  total_cost = cost_density
      * (aabb_max[0] - aabb_min[0])
      * (aabb_max[1] - aabb_min[1])
      * (aabb_max[2] - aabb_min[2]);
}

//==============================================================================
template <typename S>
CostSource<S>::CostSource(const AABB<S>& aabb, S cost_density_)
  : aabb_min(aabb.min_), aabb_max(aabb.max_), cost_density(cost_density_)
{
  total_cost = cost_density
      * (aabb_max[0] - aabb_min[0])
      * (aabb_max[1] - aabb_min[1])
      * (aabb_max[2] - aabb_min[2]);
}

//==============================================================================
template <typename S>
CostSource<S>::CostSource()
{
  // Do nothing
}

//==============================================================================
template <typename S>
bool CostSource<S>::operator <(const CostSource& other) const
{
  if(total_cost < other.total_cost)
    return false;
  if(total_cost > other.total_cost)
    return true;

  if(cost_density < other.cost_density)
    return false;
  if(cost_density > other.cost_density)
    return true;

  for(size_t i = 0; i < 3; ++i)
    if(aabb_min[i] != other.aabb_min[i])
      return aabb_min[i] < other.aabb_min[i];

  return false;
}

} // namespace fcl

#endif

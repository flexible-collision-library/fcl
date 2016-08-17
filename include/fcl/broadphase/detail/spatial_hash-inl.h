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

#ifndef FCL_BROADPHASE_SPATIALHASH_INL_H
#define FCL_BROADPHASE_SPATIALHASH_INL_H

#include "fcl/broadphase/detail/spatial_hash.h"

namespace fcl {
namespace detail {

//==============================================================================
extern template
struct SpatialHash<double>;

//==============================================================================
template <typename S>
SpatialHash<S>::SpatialHash(const AABB<S>& scene_limit_, S cell_size_)
  : cell_size(cell_size_), scene_limit(scene_limit_)
{
  width[0] = std::ceil(scene_limit.width() / cell_size);
  width[1] = std::ceil(scene_limit.height() / cell_size);
  width[2] = std::ceil(scene_limit.depth() / cell_size);
}

//==============================================================================
template <typename S>
std::vector<unsigned int> SpatialHash<S>::operator()(const AABB<S>& aabb) const
{
  int min_x = std::floor((aabb.min_[0] - scene_limit.min_[0]) / cell_size);
  int max_x = std::ceil((aabb.max_[0] - scene_limit.min_[0]) / cell_size);
  int min_y = std::floor((aabb.min_[1] - scene_limit.min_[1]) / cell_size);
  int max_y = std::ceil((aabb.max_[1] - scene_limit.min_[1]) / cell_size);
  int min_z = std::floor((aabb.min_[2] - scene_limit.min_[2]) / cell_size);
  int max_z = std::ceil((aabb.max_[2] - scene_limit.min_[2]) / cell_size);

  std::vector<unsigned int> keys((max_x - min_x) * (max_y - min_y) * (max_z - min_z));
  int id = 0;
  for(int x = min_x; x < max_x; ++x)
  {
    for(int y = min_y; y < max_y; ++y)
    {
      for(int z = min_z; z < max_z; ++z)
      {
        keys[id++] = x + y * width[0] + z * width[0] * width[1];
      }
    }
  }
  return keys;
}

} // namespace detail
} // namespace fcl

#endif

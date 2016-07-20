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

/** \author Jia Pan */

#include "fcl/BV/AABB.h"

#include <limits>
#include <iostream>

namespace fcl
{

AABB::AABB() : min_(std::numeric_limits<FCL_REAL>::max()),
               max_(-std::numeric_limits<FCL_REAL>::max())
{
}

FCL_REAL AABB::distance(const AABB& other, Vec3f* P, Vec3f* Q) const
{
  FCL_REAL result = 0;
  for(std::size_t i = 0; i < 3; ++i)
  {
    const FCL_REAL& amin = min_[i];
    const FCL_REAL& amax = max_[i];
    const FCL_REAL& bmin = other.min_[i];
    const FCL_REAL& bmax = other.max_[i];
    
    if(amin > bmax)
    {
      FCL_REAL delta = bmax - amin;
      result += delta * delta;
      if(P && Q)
      {
        (*P)[i] = amin;
        (*Q)[i] = bmax;
      }
    }
    else if(bmin > amax)
    {
      FCL_REAL delta = amax - bmin;
      result += delta * delta;
      if(P && Q)
      {
        (*P)[i] = amax;
        (*Q)[i] = bmin;
      }
    }
    else
    {
      if(P && Q)
      {
        if(bmin >= amin)
        {
          FCL_REAL t = 0.5 * (amax + bmin);
          (*P)[i] = t;
          (*Q)[i] = t;
        }
        else
        {
          FCL_REAL t = 0.5 * (amin + bmax);
          (*P)[i] = t;
          (*Q)[i] = t;
        }
      }
    }
  }

  return std::sqrt(result);
}

FCL_REAL AABB::distance(const AABB& other) const
{
  FCL_REAL result = 0;
  for(std::size_t i = 0; i < 3; ++i)
  {
    const FCL_REAL& amin = min_[i];
    const FCL_REAL& amax = max_[i];
    const FCL_REAL& bmin = other.min_[i];
    const FCL_REAL& bmax = other.max_[i];
    
    if(amin > bmax)
    {
      FCL_REAL delta = bmax - amin;
      result += delta * delta;
    }
    else if(bmin > amax)
    {
      FCL_REAL delta = amax - bmin;
      result += delta * delta;
    }
  }

  return std::sqrt(result);
}

}

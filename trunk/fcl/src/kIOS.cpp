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

#include "fcl/kIOS.h"
#include "fcl/BVH_utility.h"
#include "fcl/transform.h"

#include <iostream>
#include <limits>

namespace fcl
{

bool kIOS::overlap(const kIOS& other) const
{
  for(unsigned int i = 0; i < num_spheres; ++i)
  {
    for(unsigned int j = 0; j < other.num_spheres; ++j)
    {
      BVH_REAL o_dist = (spheres[i].o - other.spheres[j].o).sqrLength();
      BVH_REAL sum_r = spheres[i].r + other.spheres[j].r;
      if(o_dist > sum_r * sum_r)
        return false;
    }
  }

  return true;
}


bool kIOS::contain(const Vec3f& p) const
{
  for(unsigned int i = 0; i < num_spheres; ++i)
  {
    BVH_REAL r = spheres[i].r;
    if((spheres[i].o - p).sqrLength() > r * r)
      return false;
  }

  return true;
}

kIOS& kIOS::operator += (const Vec3f& p)
{
  for(unsigned int i = 0; i < num_spheres; ++i)
  {
    BVH_REAL r = spheres[i].r;
    BVH_REAL new_r_sqr = (p - spheres[i].o).sqrLength();
    if(new_r_sqr > r * r)
    {
      spheres[i].r = sqrt(new_r_sqr);
    }
  }
}

kIOS kIOS::operator + (const kIOS& other) const
{
  kIOS result;
  unsigned int new_num_spheres = std::min(num_spheres, other.num_spheres);
  for(unsigned int i = 0; i < new_num_spheres; ++i)
  {
    result.spheres[i] = encloseSphere(spheres[i], other.spheres[i]);
  }
    
  result.num_spheres = new_num_spheres;
  return result;
}

BVH_REAL kIOS::width() const
{
  BVH_REAL min_r = std::numeric_limits<BVH_REAL>::max();
  for(unsigned int i = 0; i < num_spheres; ++i)
  {
    if(spheres[i].r < min_r) min_r = spheres[i].r;
  }
  return min_r;
}

BVH_REAL kIOS::height() const
{
  return width();
}
  
BVH_REAL kIOS::depth() const
{
  return width();
}

BVH_REAL kIOS::volume() const
{
  BVH_REAL r = kIOS::width();
  return 4.0 / 3.0 * 3.1415926 * r * r * r;
}

BVH_REAL kIOS::size() const
{
  return volume();
}

BVH_REAL kIOS::distance(const kIOS& other, Vec3f* P, Vec3f* Q) const
{
  BVH_REAL d_max = 0;
  unsigned int id_a = -1, id_b = -1;
  for(unsigned int i = 0; i < num_spheres; ++i)
  {
    for(unsigned int j = 0; j < other.num_spheres; ++j)
    {
      BVH_REAL d = (spheres[i].o - other.spheres[j].o).length() - (spheres[i].r + other.spheres[j].r);
      if(d_max < d)
      {
        d_max = d;
        if(P && Q)
        {
          id_a = i; id_b = j;
        }
      }
    }
  }

  if(P && Q)
  {
    if(id_a != -1 && id_b != -1)
    {
      Vec3f v = spheres[id_a].o - spheres[id_b].o;
      BVH_REAL len_v = v.length();
      *P = spheres[id_a].o - v * (spheres[id_a].r / len_v);
      *Q = spheres[id_b].o + v * (spheres[id_b].r / len_v);
    }
  }

  return d_max;
}

  
bool overlap(const Matrix3f& R0, const Vec3f& T0, const kIOS& b1, const kIOS& b2)
{
  kIOS b2_temp = b2;
  for(unsigned int i = 0; i < b2_temp.num_spheres; ++i)
  {
    b2_temp.spheres[i].o = R0 * b2_temp.spheres[i].o + T0;
  }

  return b1.overlap(b2_temp);
}

BVH_REAL distance(const Matrix3f& R0, const Vec3f& T0, const kIOS& b1, const kIOS& b2, Vec3f* P, Vec3f* Q)
{
  kIOS b2_temp = b2;
  for(unsigned int i = 0; i < b2_temp.num_spheres; ++i)
  {
    b2_temp.spheres[i].o = R0 * b2_temp.spheres[i].o + T0;
  }

  return b1.distance(b2_temp, P, Q);
}


}

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

#include "fcl/BV_splitter.h"

namespace fcl
{

void BVSplitter<OBB>::computeRule_bvcenter(const OBB& bv, unsigned int* primitive_indices, int num_primitives)
{
  Vec3f center = bv.center();
  split_vector = bv.axis[0];
  split_value = center[0];
}

void BVSplitter<OBB>::computeRule_mean(const OBB& bv, unsigned int* primitive_indices, int num_primitives)
{
  split_vector = bv.axis[0];
  BVH_REAL sum = 0;
  if(type == BVH_MODEL_TRIANGLES)
  {
    for(int i = 0; i < num_primitives; ++i)
    {
      const Triangle& t = tri_indices[primitive_indices[i]];
      const Vec3f& p1 = vertices[t[0]];
      const Vec3f& p2 = vertices[t[1]];
      const Vec3f& p3 = vertices[t[2]];
      Vec3f centroid((p1[0] + p2[0] + p3[0]) / 3,
                     (p1[1] + p2[1] + p3[1]) / 3,
                     (p1[2] + p2[2] + p3[2]) / 3);

      sum += centroid.dot(split_vector);
    }
  }
  else if(type == BVH_MODEL_POINTCLOUD)
  {
    for(int i = 0; i < num_primitives; ++i)
    {
      const Vec3f& p = vertices[primitive_indices[i]];
      Vec3f v(p[0], p[1], p[2]);
      sum += v.dot(split_vector);
    }
  }

  split_value = sum / num_primitives;
}


void BVSplitter<OBB>::computeRule_median(const OBB& bv, unsigned int* primitive_indices, int num_primitives)
{
  split_vector = bv.axis[0];
  std::vector<BVH_REAL> proj(num_primitives);

  if(type == BVH_MODEL_TRIANGLES)
  {
    for(int i = 0; i < num_primitives; ++i)
    {
      const Triangle& t = tri_indices[primitive_indices[i]];
      const Vec3f& p1 = vertices[t[0]];
      const Vec3f& p2 = vertices[t[1]];
      const Vec3f& p3 = vertices[t[2]];
      Vec3f centroid((p1[0] + p2[0] + p3[0]) / 3,
                     (p1[1] + p2[1] + p3[1]) / 3,
                     (p1[2] + p2[2] + p3[2]) / 3);

      proj[i] = centroid.dot(split_vector);
    }
  }
  else if(type == BVH_MODEL_POINTCLOUD)
  {
    for(int i = 0; i < num_primitives; ++i)
    {
      const Vec3f& p = vertices[primitive_indices[i]];
      Vec3f v(p[0], p[1], p[2]);
      proj[i] = v.dot(split_vector);
    }
  }

  std::sort(proj.begin(), proj.end());

  if(num_primitives % 2 == 1)
  {
    split_value = proj[(num_primitives - 1) / 2];
  }
  else
  {
    split_value = (proj[num_primitives / 2] + proj[num_primitives / 2 - 1]) / 2;
  }
}

}

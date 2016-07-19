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


#ifndef FCL_BVH_UTILITY_H
#define FCL_BVH_UTILITY_H

#include "fcl/BVH/BVH_model.h"


namespace fcl
{
/// @brief Expand the BVH bounding boxes according to the variance matrix corresponding to the data stored within each BV node
template<typename BV>
void BVHExpand(BVHModel<BV>& model, const Variance3f* ucs, FCL_REAL r)
{
  for(int i = 0; i < model.num_bvs; ++i)
  {
    BVNode<BV>& bvnode = model.getBV(i);

    BV bv;
    for(int j = 0; j < bvnode.num_primitives; ++j)
    {
      int v_id = bvnode.first_primitive + j;
      const Variance3f& uc = ucs[v_id];

      Vec3f& v = model.vertices[bvnode.first_primitive + j];

      for(int k = 0; k < 3; ++k)
      {
        bv += (v + uc.axis[k] * (r * uc.sigma[k]));
        bv += (v - uc.axis[k] * (r * uc.sigma[k]));
      }
    }

    bvnode.bv = bv;
  }
}

/// @brief Expand the BVH bounding boxes according to the corresponding variance information, for OBB
void BVHExpand(BVHModel<OBB>& model, const Variance3f* ucs, FCL_REAL r);

/// @brief Expand the BVH bounding boxes according to the corresponding variance information, for RSS
void BVHExpand(BVHModel<RSS>& model, const Variance3f* ucs, FCL_REAL r);

/// @brief Compute the covariance matrix for a set or subset of points. if ts = null, then indices refer to points directly; otherwise refer to triangles
void getCovariance(Vec3f* ps, Vec3f* ps2, Triangle* ts, unsigned int* indices, int n, Matrix3f& M);

/// @brief Compute the RSS bounding volume parameters: radius, rectangle size and the origin, given the BV axises.
void getRadiusAndOriginAndRectangleSize(Vec3f* ps, Vec3f* ps2, Triangle* ts, unsigned int* indices, int n, Vec3f axis[3], Vec3f& origin, FCL_REAL l[2], FCL_REAL& r);

/// @brief Compute the bounding volume extent and center for a set or subset of points, given the BV axises.
void getExtentAndCenter(Vec3f* ps, Vec3f* ps2, Triangle* ts, unsigned int* indices, int n, Vec3f axis[3], Vec3f& center, Vec3f& extent);

/// @brief Compute the center and radius for a triangle's circumcircle
void circumCircleComputation(const Vec3f& a, const Vec3f& b, const Vec3f& c, Vec3f& center, FCL_REAL& radius);

/// @brief Compute the maximum distance from a given center point to a point cloud
FCL_REAL maximumDistance(Vec3f* ps, Vec3f* ps2, Triangle* ts, unsigned int* indices, int n, const Vec3f& query);


}

#endif

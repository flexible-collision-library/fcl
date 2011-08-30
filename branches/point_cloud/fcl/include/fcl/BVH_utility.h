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


#ifndef FCL_BVH_UTILITY_H
#define FCL_BVH_UTILITY_H

#include "fcl/primitive.h"
#include "fcl/vec_3f.h"
#include "fcl/BVH_model.h"


namespace fcl
{

/** \brief Expand the BVH bounding boxes according to uncertainty */
template<typename BV>
void BVHExpand(BVHModel<BV>& model, const Uncertainty* ucs, BVH_REAL r)
{
  for(int i = 0; i < model.num_bvs; ++i)
  {
    BVNode<BV>& bvnode = model.getBV(i);

    BV bv;
    for(int j = 0; j < bvnode.num_primitives; ++j)
    {
      int v_id = bvnode.first_primitive + j;
      const Uncertainty& uc = ucs[v_id];

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

/** \brief Expand the BVH bounding boxes according to uncertainty, for OBB */
void BVHExpand(BVHModel<OBB>& model, const Uncertainty* ucs, BVH_REAL r);

/** \brief Expand the BVH bounding boxes according to uncertainty, for RSS */
void BVHExpand(BVHModel<RSS>& model, const Uncertainty* ucs, BVH_REAL r);

/** \brief Estimate the uncertainty of point clouds due to sampling procedure */
void estimateSamplingUncertainty(Vec3f* vertices, int num_vertices, Uncertainty* ucs);

/** \brief Compute the covariance matrix for a set or subset of points. */
void getCovariance(Vec3f* ps, Vec3f* ps2, unsigned int* indices, int n, Vec3f M[3]);

/** \brief Compute the covariance matrix for a set or subset of triangles. */
void getCovariance(Vec3f* ps, Vec3f* ps2, Triangle* ts, unsigned int* indices, int n, Vec3f M[3]);


/** \brief Compute the RSS bounding volume parameters: radius, rectangle size and the origin.
 * The bounding volume axes are known.
 */
void getRadiusAndOriginAndRectangleSize(Vec3f* ps, Vec3f* ps2, unsigned int* indices, int n, Vec3f axis[3], Vec3f& origin, BVH_REAL l[2], BVH_REAL& r);


/** \brief Compute the RSS bounding volume parameters: radius, rectangle size and the origin.
 * The bounding volume axes are known.
 */
void getRadiusAndOriginAndRectangleSize(Vec3f* ps, Vec3f* ps2, Triangle* ts, unsigned int* indices, int n, Vec3f axis[3], Vec3f& origin, BVH_REAL l[2], BVH_REAL& r);

/** \brief Compute the bounding volume extent and center for a set or subset of points.
 * The bounding volume axes are known.
 */
void getExtentAndCenter(Vec3f* ps, Vec3f* ps2, unsigned int* indices, int n, Vec3f axis[3], Vec3f& center, Vec3f& extent);


/** \brief Compute the bounding volume extent and center for a set or subset of points.
 * The bounding volume axes are known.
 */
void getExtentAndCenter(Vec3f* ps, Vec3f* ps2, Triangle* ts, unsigned int* indices, int n, Vec3f axis[3], Vec3f& center, Vec3f& extent);

}

#endif

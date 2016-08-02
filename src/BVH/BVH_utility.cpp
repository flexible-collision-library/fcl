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

#include "fcl/BVH/BVH_utility.h"

#include "fcl/BV/fit.h"

namespace fcl
{

void BVHExpand(BVHModel<OBBd>& model, const Variance3d* ucs, FCL_REAL r = 1.0)
{
  for(int i = 0; i < model.getNumBVs(); ++i)
  {
    BVNode<OBBd>& bvnode = model.getBV(i);

    Vector3d* vs = new Vector3d[bvnode.num_primitives * 6];

    for(int j = 0; j < bvnode.num_primitives; ++j)
    {
      int v_id = bvnode.first_primitive + j;
      const Variance3d& uc = ucs[v_id];

      Vector3d&v = model.vertices[bvnode.first_primitive + j];

      for(int k = 0; k < 3; ++k)
      {
        vs[6 * j + 2 * k] = v + uc.axis.col(k) * (r * uc.sigma[k]);
        vs[6 * j + 2 * k + 1] = v - uc.axis.col(k) * (r * uc.sigma[k]);
      }
    }

    OBBd bv;
    fit(vs, bvnode.num_primitives * 6, bv);

    delete [] vs;

    bvnode.bv = bv;
  }
}

void BVHExpand(BVHModel<RSSd>& model, const Variance3d* ucs, FCL_REAL r = 1.0)
{
  for(int i = 0; i < model.getNumBVs(); ++i)
  {
    BVNode<RSSd>& bvnode = model.getBV(i);

    Vector3d* vs = new Vector3d[bvnode.num_primitives * 6];

    for(int j = 0; j < bvnode.num_primitives; ++j)
    {
      int v_id = bvnode.first_primitive + j;
      const Variance3d& uc = ucs[v_id];

      Vector3d&v = model.vertices[bvnode.first_primitive + j];

      for(int k = 0; k < 3; ++k)
      {
        vs[6 * j + 2 * k] = v + uc.axis.col(k) * (r * uc.sigma[k]);
        vs[6 * j + 2 * k + 1] = v - uc.axis.col(k) * (r * uc.sigma[k]);
      }
    }

    RSSd bv;
    fit(vs, bvnode.num_primitives * 6, bv);

    delete [] vs;

    bvnode.bv = bv;
  }
}

} // namespace fcl

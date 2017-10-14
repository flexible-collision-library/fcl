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

#ifndef FCL_BVH_UTILITY_INL_H
#define FCL_BVH_UTILITY_INL_H

#include "fcl/geometry/bvh/BVH_utility.h"

#include "fcl/math/bv/utility.h"

namespace fcl
{

//==============================================================================
extern template
void BVHExpand(
    BVHModel<OBB<double>>& model, const Variance3<double>* ucs, double r);

//==============================================================================
extern template
void BVHExpand(
    BVHModel<RSS<double>>& model, const Variance3<double>* ucs, double r);

//==============================================================================
template <typename S, typename BV>
FCL_EXPORT
void BVHExpand(BVHModel<BV>& model, const Variance3<S>* ucs, S r)
{
  for(int i = 0; i < model.num_bvs; ++i)
  {
    BVNode<BV>& bvnode = model.getBV(i);

    BV bv;
    for(int j = 0; j < bvnode.num_primitives; ++j)
    {
      int v_id = bvnode.first_primitive + j;
      const Variance3<S>& uc = ucs[v_id];

      Vector3<S>& v = model.vertices[bvnode.first_primitive + j];

      for(int k = 0; k < 3; ++k)
      {
        bv += (v + uc.axis.col(k) * (r * uc.sigma[k]));
        bv += (v - uc.axis.col(k) * (r * uc.sigma[k]));
      }
    }

    bvnode.bv = bv;
  }
}

//==============================================================================
template <typename S>
FCL_EXPORT
void BVHExpand(
    BVHModel<OBB<S>>& model,
    const Variance3<S>* ucs,
    S r)
{
  for(int i = 0; i < model.getNumBVs(); ++i)
  {
    BVNode<OBB<S>>& bvnode = model.getBV(i);

    Vector3<S>* vs = new Vector3<S>[bvnode.num_primitives * 6];
    // TODO(JS): We could use one std::vector outside of the outter for-loop,
    // and reuse it rather than creating and destructing the array every
    // iteration.

    for(int j = 0; j < bvnode.num_primitives; ++j)
    {
      int v_id = bvnode.first_primitive + j;
      const Variance3<S>& uc = ucs[v_id];

      Vector3<S>&v = model.vertices[bvnode.first_primitive + j];

      for(int k = 0; k < 3; ++k)
      {
        const auto index1 = 6 * j + 2 * k;
        const auto index2 = index1 + 1;
        vs[index1] = v;
        vs[index1].noalias() += uc.axis.col(k) * (r * uc.sigma[k]);
        vs[index2] = v;
        vs[index2].noalias() -= uc.axis.col(k) * (r * uc.sigma[k]);
      }
    }

    OBB<S> bv;
    fit(vs, bvnode.num_primitives * 6, bv);

    delete [] vs;

    bvnode.bv = bv;
  }
}

//==============================================================================
template <typename S>
FCL_EXPORT
void BVHExpand(
    BVHModel<RSS<S>>& model,
    const Variance3<S>* ucs,
    S r)
{
  for(int i = 0; i < model.getNumBVs(); ++i)
  {
    BVNode<RSS<S>>& bvnode = model.getBV(i);

    Vector3<S>* vs = new Vector3<S>[bvnode.num_primitives * 6];
    // TODO(JS): We could use one std::vector outside of the outter for-loop,
    // and reuse it rather than creating and destructing the array every
    // iteration.

    for(int j = 0; j < bvnode.num_primitives; ++j)
    {
      int v_id = bvnode.first_primitive + j;
      const Variance3<S>& uc = ucs[v_id];

      Vector3<S>&v = model.vertices[bvnode.first_primitive + j];

      for(int k = 0; k < 3; ++k)
      {
        vs[6 * j + 2 * k] = v + uc.axis.col(k) * (r * uc.sigma[k]);
        vs[6 * j + 2 * k + 1] = v - uc.axis.col(k) * (r * uc.sigma[k]);
      }
    }

    RSS<S> bv;
    fit(vs, bvnode.num_primitives * 6, bv);

    delete [] vs;

    bvnode.bv = bv;
  }
}

} // namespace fcl

#endif

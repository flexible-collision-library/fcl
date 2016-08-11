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

#ifndef FCL_SHAPE_DETAIL_BVCOMPUTERCONVEX_H
#define FCL_SHAPE_DETAIL_BVCOMPUTERCONVEX_H

#include "fcl/BV/AABB.h"
#include "fcl/BV/OBB.h"

namespace fcl
{
namespace detail
{

template <typename S>
struct BVComputer<S, AABB<S>, Convex<S>>;

template <typename S>
struct BVComputer<S, OBB<S>, Convex<S>>;

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename S>
struct BVComputer<S, AABB<S>, Convex<S>>
{
  static void compute(const Convex<S>& s, const Transform3<S>& tf, AABB<S>& bv)
  {
    const Matrix3<S>& R = tf.linear();
    const Vector3<S>& T = tf.translation();

    AABB<S> bv_;
    for(int i = 0; i < s.num_points; ++i)
    {
      Vector3<S> new_p = R * s.points[i] + T;
      bv_ += new_p;
    }

    bv = bv_;
  }
};

//==============================================================================
template <typename S>
struct BVComputer<S, OBB<S>, Convex<S>>
{
  static void compute(const Convex<S>& s, const Transform3<S>& tf, OBB<S>& bv)
  {
    fit(s.points, s.num_points, bv);

    bv.axis = tf.linear();
    bv.To = tf * bv.To;
  }
};

} // namespace detail
} // namespace fcl

#endif

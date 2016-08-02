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


#ifndef FCL_SHAPE_COMPUTE_BV_H
#define FCL_SHAPE_COMPUTE_BV_H

#include <vector>
#include "fcl/data_types.h"
#include "fcl/BV/fit.h"

namespace fcl
{

template <typename Scalar, typename BV, typename S>
struct ComputeBVImpl
{
  void operator()(const S& s, const Transform3<Scalar>& tf, BV& bv)
  {
    std::vector<Vector3<Scalar>> convex_bound_vertices = s.getBoundVertices(tf);
    fit(&convex_bound_vertices[0], (int)convex_bound_vertices.size(), bv);
  }
};
// TODO(JS): move this under detail namespace
// TODO(JS): remove Scalar template argument and replace with typename BV::Scalar

/// @brief calculate a bounding volume for a shape in a specific configuration
template <typename Scalar, typename BV, typename S>
void computeBV(const S& s, const Transform3<Scalar>& tf, BV& bv)
{
  ComputeBVImpl<Scalar, BV, S> computeBVImpl;
  computeBVImpl(s, tf, bv);
}

} // namespace fcl

#endif

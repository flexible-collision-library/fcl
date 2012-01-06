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


#ifndef FCL_GEOMETRIC_SHAPES_UTILITY_H
#define FCL_GEOMETRIC_SHAPES_UTILITY_H

#include "fcl/geometric_shapes.h"
#include "fcl/BV.h"

namespace fcl
{
  template<typename BV>
  void computeBV(const Box& s, const SimpleTransform& tf, BV& bv) {}

  template<typename BV>
  void computeBV(const Sphere& s, const SimpleTransform& tf, BV& bv) {}

  template<typename BV>
  void computeBV(const Capsule& s, const SimpleTransform& tf, BV& bv) {}

  template<typename BV>
  void computeBV(const Cone& s, const SimpleTransform& tf, BV& bv) {}

  template<typename BV>
  void computeBV(const Cylinder& s, const SimpleTransform& tf, BV& bv) {}

  template<typename BV>
  void computeBV(const Convex& s, const SimpleTransform& tf, BV& bv) {}

  /** the bounding volume for half space back of plane
   * for OBB, it is the plane itself
   */
  template<typename BV>
  void computeBV(const Plane& s, const SimpleTransform& tf, BV& bv) {}

  /** For AABB */
  template<>
  void computeBV<AABB>(const Box& s, const SimpleTransform& tf, AABB& bv);

  template<>
  void computeBV<AABB>(const Sphere& s, const SimpleTransform& tf, AABB& bv);

  template<>
  void computeBV<AABB>(const Capsule& s, const SimpleTransform& tf, AABB& bv);

  template<>
  void computeBV<AABB>(const Cone& s, const SimpleTransform& tf, AABB& bv);

  template<>
  void computeBV<AABB>(const Cylinder& s, const SimpleTransform& tf, AABB& bv);

  template<>
  void computeBV<AABB>(const Convex& s, const SimpleTransform& tf, AABB& bv);

  template<>
  void computeBV<AABB>(const Plane& s, const SimpleTransform& tf, AABB& bv);

  template<>
  void computeBV<OBB>(const Box& s, const SimpleTransform& tf, OBB& bv);

  template<>
  void computeBV<OBB>(const Sphere& s, const SimpleTransform& tf, OBB& bv);

  template<>
  void computeBV<OBB>(const Capsule& s, const SimpleTransform& tf, OBB& bv);

  template<>
  void computeBV<OBB>(const Cone& s, const SimpleTransform& tf, OBB& bv);

  template<>
  void computeBV<OBB>(const Cylinder& s, const SimpleTransform& tf, OBB& bv);

  template<>
  void computeBV<OBB>(const Convex& s, const SimpleTransform& tf, OBB& bv);

  template<>
  void computeBV<OBB>(const Plane& s, const SimpleTransform& tf, OBB& bv);

  // TODO: implement computeBV for RSS and KDOP
}

#endif

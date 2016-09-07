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

#ifndef FCL_SHAPE_GEOMETRICSHAPETOBVHMODEL_H
#define FCL_SHAPE_GEOMETRICSHAPETOBVHMODEL_H

#include "fcl/geometry/bvh/BVH_model.h"
#include "fcl/geometry/shape/box.h"
#include "fcl/geometry/shape/capsule.h"
#include "fcl/geometry/shape/cone.h"
#include "fcl/geometry/shape/convex.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/geometry/shape/ellipsoid.h"
#include "fcl/geometry/shape/halfspace.h"
#include "fcl/geometry/shape/plane.h"
#include "fcl/geometry/shape/sphere.h"
#include "fcl/geometry/shape/triangle_p.h"

namespace fcl
{

/// @brief Generate BVH model from box
template<typename BV>
void generateBVHModel(BVHModel<BV>& model, const Box<typename BV::S>& shape, const Transform3<typename BV::S>& pose);

/// @brief Generate BVH model from sphere, given the number of segments along longitude and number of rings along latitude.
template<typename BV>
void generateBVHModel(BVHModel<BV>& model, const Sphere<typename BV::S>& shape, const Transform3<typename BV::S>& pose, unsigned int seg, unsigned int ring);

/// @brief Generate BVH model from sphere
/// The difference between generateBVHModel is that it gives the number of triangles faces N for a sphere with unit radius. For sphere of radius r,
/// then the number of triangles is r * r * N so that the area represented by a single triangle is approximately the same.s
template<typename BV>
void generateBVHModel(BVHModel<BV>& model, const Sphere<typename BV::S>& shape, const Transform3<typename BV::S>& pose, unsigned int n_faces_for_unit_sphere);

/// @brief Generate BVH model from ellipsoid, given the number of segments along longitude and number of rings along latitude.
template<typename BV>
void generateBVHModel(BVHModel<BV>& model, const Ellipsoid<typename BV::S>& shape, const Transform3<typename BV::S>& pose, unsigned int seg, unsigned int ring);

/// @brief Generate BVH model from ellipsoid
/// The difference between generateBVHModel is that it gives the number of triangles faces N for an ellipsoid with unit radii (1, 1, 1). For ellipsoid of radii (a, b, c),
/// then the number of triangles is ((a^p * b^p + b^p * c^p + c^p * a^p)/3)^(1/p) * N, where p is 1.6075, so that the area represented by a single triangle is approximately the same.
/// Reference: https://en.wikipedia.org/wiki/Ellipsoid<S>#Approximate_formula
template<typename BV>
void generateBVHModel(BVHModel<BV>& model, const Ellipsoid<typename BV::S>& shape, const Transform3<typename BV::S>& pose, unsigned int n_faces_for_unit_ellipsoid);

/// @brief Generate BVH model from cylinder, given the number of segments along circle and the number of segments along axis.
template<typename BV>
void generateBVHModel(BVHModel<BV>& model, const Cylinder<typename BV::S>& shape, const Transform3<typename BV::S>& pose, unsigned int tot, unsigned int h_num);

/// @brief Generate BVH model from cylinder
/// Difference from generateBVHModel: is that it gives the circle split number tot for a cylinder with unit radius. For cylinder with
/// larger radius, the number of circle split number is r * tot.
template<typename BV>
void generateBVHModel(BVHModel<BV>& model, const Cylinder<typename BV::S>& shape, const Transform3<typename BV::S>& pose, unsigned int tot_for_unit_cylinder);

/// @brief Generate BVH model from cone, given the number of segments along circle and the number of segments along axis.
template<typename BV>
void generateBVHModel(BVHModel<BV>& model, const Cone<typename BV::S>& shape, const Transform3<typename BV::S>& pose, unsigned int tot, unsigned int h_num);

/// @brief Generate BVH model from cone
/// Difference from generateBVHModel: is that it gives the circle split number tot for a cylinder with unit radius. For cone with
/// larger radius, the number of circle split number is r * tot.
template<typename BV>
void generateBVHModel(BVHModel<BV>& model, const Cone<typename BV::S>& shape, const Transform3<typename BV::S>& pose, unsigned int tot_for_unit_cone);

} // namespace fcl

#include "fcl/geometry/geometric_shape_to_BVH_model-inl.h"

#endif

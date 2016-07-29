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


#ifndef FCL_GEOMETRIC_SHAPES_UTILITY_H
#define FCL_GEOMETRIC_SHAPES_UTILITY_H

#include <vector>

#include "fcl/BV/BV.h"

namespace fcl
{

template <typename Scalar>
class Box;
using Boxd = Box<double>;

template <typename Scalar>
class Sphere;
using Sphered = Sphere<double>;

template <typename Scalar>
class Ellipsoid;
using Ellipsoidd = Ellipsoid<double>;

template <typename Scalar>
class Capsule;
using Capsuled = Capsule<double>;

template <typename Scalar>
class Cone;
using Coned = Cone<double>;

template <typename Scalar>
class Cylinder;
using Cylinderd = Cylinder<double>;

template <typename Scalar>
class Convex;
using Convexd = Convex<double>;

template <typename Scalar>
class TriangleP;
using TrianglePd = TriangleP<double>;

template <typename Scalar>
class Halfspace;
using Halfspaced = Halfspace<double>;

template <typename Scalar>
class Plane;
using Planed = Plane<double>;

/// @cond IGNORE
namespace details
{
/// @brief get the vertices of some convex shape which can bound the given shape in a specific configuration
std::vector<Vector3d> getBoundVertices(const Boxd& box, const Transform3d& tf);
std::vector<Vector3d> getBoundVertices(const Sphered& sphere, const Transform3d& tf);
std::vector<Vector3d> getBoundVertices(const Ellipsoidd& ellipsoid, const Transform3d& tf);
std::vector<Vector3d> getBoundVertices(const Capsuled& capsule, const Transform3d& tf);
std::vector<Vector3d> getBoundVertices(const Coned& cone, const Transform3d& tf);
std::vector<Vector3d> getBoundVertices(const Cylinderd& cylinder, const Transform3d& tf);
std::vector<Vector3d> getBoundVertices(const Convexd& convex, const Transform3d& tf);
std::vector<Vector3d> getBoundVertices(const TrianglePd& triangle, const Transform3d& tf);
} 
/// @endcond


/// @brief calculate a bounding volume for a shape in a specific configuration
template<typename BV, typename S>
void computeBV(const S& s, const Transform3d& tf, BV& bv)
{
  std::vector<Vector3d> convex_bound_vertices = details::getBoundVertices(s, tf);
  fit(&convex_bound_vertices[0], (int)convex_bound_vertices.size(), bv);
}

template<>
void computeBV<AABB, Boxd>(const Boxd& s, const Transform3d& tf, AABB& bv);

template<>
void computeBV<AABB, Sphered>(const Sphered& s, const Transform3d& tf, AABB& bv);

template<>
void computeBV<AABB, Ellipsoidd>(const Ellipsoidd& s, const Transform3d& tf, AABB& bv);

template<>
void computeBV<AABB, Capsuled>(const Capsuled& s, const Transform3d& tf, AABB& bv);

template<>
void computeBV<AABB, Coned>(const Coned& s, const Transform3d& tf, AABB& bv);

template<>
void computeBV<AABB, Cylinderd>(const Cylinderd& s, const Transform3d& tf, AABB& bv);

template<>
void computeBV<AABB, Convexd>(const Convexd& s, const Transform3d& tf, AABB& bv);

template<>
void computeBV<AABB, TrianglePd>(const TrianglePd& s, const Transform3d& tf, AABB& bv);

template<>
void computeBV<AABB, Halfspaced>(const Halfspaced& s, const Transform3d& tf, AABB& bv);

template<>
void computeBV<AABB, Planed>(const Planed& s, const Transform3d& tf, AABB& bv);



template<>
void computeBV<OBB, Boxd>(const Boxd& s, const Transform3d& tf, OBB& bv);

template<>
void computeBV<OBB, Sphered>(const Sphered& s, const Transform3d& tf, OBB& bv);

template<>
void computeBV<OBB, Ellipsoidd>(const Ellipsoidd& s, const Transform3d& tf, OBB& bv);

template<>
void computeBV<OBB, Capsuled>(const Capsuled& s, const Transform3d& tf, OBB& bv);

template<>
void computeBV<OBB, Coned>(const Coned& s, const Transform3d& tf, OBB& bv);

template<>
void computeBV<OBB, Cylinderd>(const Cylinderd& s, const Transform3d& tf, OBB& bv);

template<>
void computeBV<OBB, Convexd>(const Convexd& s, const Transform3d& tf, OBB& bv);

template<>
void computeBV<OBB, Halfspaced>(const Halfspaced& s, const Transform3d& tf, OBB& bv);

template<>
void computeBV<RSS, Halfspaced>(const Halfspaced& s, const Transform3d& tf, RSS& bv);

template<>
void computeBV<OBBRSS, Halfspaced>(const Halfspaced& s, const Transform3d& tf, OBBRSS& bv);

template<>
void computeBV<kIOS, Halfspaced>(const Halfspaced& s, const Transform3d& tf, kIOS& bv);

template<>
void computeBV<KDOP<16>, Halfspaced>(const Halfspaced& s, const Transform3d& tf, KDOP<16>& bv);

template<>
void computeBV<KDOP<18>, Halfspaced>(const Halfspaced& s, const Transform3d& tf, KDOP<18>& bv);

template<>
void computeBV<KDOP<24>, Halfspaced>(const Halfspaced& s, const Transform3d& tf, KDOP<24>& bv);

template<>
void computeBV<OBB, Planed>(const Planed& s, const Transform3d& tf, OBB& bv);

template<>
void computeBV<RSS, Planed>(const Planed& s, const Transform3d& tf, RSS& bv);

template<>
void computeBV<OBBRSS, Planed>(const Planed& s, const Transform3d& tf, OBBRSS& bv);

template<>
void computeBV<kIOS, Planed>(const Planed& s, const Transform3d& tf, kIOS& bv);

template<>
void computeBV<KDOP<16>, Planed>(const Planed& s, const Transform3d& tf, KDOP<16>& bv);

template<>
void computeBV<KDOP<18>, Planed>(const Planed& s, const Transform3d& tf, KDOP<18>& bv);

template<>
void computeBV<KDOP<24>, Planed>(const Planed& s, const Transform3d& tf, KDOP<24>& bv);


/// @brief construct a box shape (with a configuration) from a given bounding volume
void constructBox(const AABB& bv, Boxd& box, Transform3d& tf);

void constructBox(const OBB& bv, Boxd& box, Transform3d& tf);

void constructBox(const OBBRSS& bv, Boxd& box, Transform3d& tf);

void constructBox(const kIOS& bv, Boxd& box, Transform3d& tf);

void constructBox(const RSS& bv, Boxd& box, Transform3d& tf);

void constructBox(const KDOP<16>& bv, Boxd& box, Transform3d& tf);

void constructBox(const KDOP<18>& bv, Boxd& box, Transform3d& tf);

void constructBox(const KDOP<24>& bv, Boxd& box, Transform3d& tf);

void constructBox(const AABB& bv, const Transform3d& tf_bv, Boxd& box, Transform3d& tf);

void constructBox(const OBB& bv, const Transform3d& tf_bv, Boxd& box, Transform3d& tf);

void constructBox(const OBBRSS& bv, const Transform3d& tf_bv, Boxd& box, Transform3d& tf);

void constructBox(const kIOS& bv, const Transform3d& tf_bv, Boxd& box, Transform3d& tf);

void constructBox(const RSS& bv, const Transform3d& tf_bv, Boxd& box, Transform3d& tf);

void constructBox(const KDOP<16>& bv, const Transform3d& tf_bv, Boxd& box, Transform3d& tf);

void constructBox(const KDOP<18>& bv, const Transform3d& tf_bv, Boxd& box, Transform3d& tf);

void constructBox(const KDOP<24>& bv, const Transform3d& tf_bv, Boxd& box, Transform3d& tf);

Halfspaced transform(const Halfspaced& a, const Transform3d& tf);

Planed transform(const Planed& a, const Transform3d& tf);

}

#endif

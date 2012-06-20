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

#include <vector>
#include "fcl/geometric_shapes.h"
#include "fcl/BV.h"

namespace fcl
{

namespace details
{
std::vector<Vec3f> getBoundVertices(const Box& box, const SimpleTransform& tf);
std::vector<Vec3f> getBoundVertices(const Sphere& sphere, const SimpleTransform& tf);
std::vector<Vec3f> getBoundVertices(const Capsule& capsule, const SimpleTransform& tf);
std::vector<Vec3f> getBoundVertices(const Cone& cone, const SimpleTransform& tf);
std::vector<Vec3f> getBoundVertices(const Cylinder& cylinder, const SimpleTransform& tf);
std::vector<Vec3f> getBoundVertices(const Convex& convex, const SimpleTransform& tf);
std::vector<Vec3f> getBoundVertices(const Triangle2& triangle, const SimpleTransform& tf);
} // end detail


template<typename BV, typename S>
void computeBV(const S& s, const SimpleTransform& tf, BV& bv)
{
  std::vector<Vec3f> convex_bound_vertices = details::getBoundVertices(s, tf);
  fit(&convex_bound_vertices[0], (int)convex_bound_vertices.size(), bv);
}

template<>
void computeBV<AABB, Box>(const Box& s, const SimpleTransform& tf, AABB& bv);

template<>
void computeBV<AABB, Sphere>(const Sphere& s, const SimpleTransform& tf, AABB& bv);

template<>
void computeBV<AABB, Capsule>(const Capsule& s, const SimpleTransform& tf, AABB& bv);

template<>
void computeBV<AABB, Cone>(const Cone& s, const SimpleTransform& tf, AABB& bv);

template<>
void computeBV<AABB, Cylinder>(const Cylinder& s, const SimpleTransform& tf, AABB& bv);

template<>
void computeBV<AABB, Convex>(const Convex& s, const SimpleTransform& tf, AABB& bv);

template<>
void computeBV<AABB, Triangle2>(const Triangle2& s, const SimpleTransform& tf, AABB& bv);


/** \brief the bounding volume for half space back of plane for OBB, it is the plane itself */
template<>
void computeBV<AABB, Plane>(const Plane& s, const SimpleTransform& tf, AABB& bv);



template<>
void computeBV<OBB, Box>(const Box& s, const SimpleTransform& tf, OBB& bv);

template<>
void computeBV<OBB, Sphere>(const Sphere& s, const SimpleTransform& tf, OBB& bv);

template<>
void computeBV<OBB, Capsule>(const Capsule& s, const SimpleTransform& tf, OBB& bv);

template<>
void computeBV<OBB, Cone>(const Cone& s, const SimpleTransform& tf, OBB& bv);

template<>
void computeBV<OBB, Cylinder>(const Cylinder& s, const SimpleTransform& tf, OBB& bv);

template<>
void computeBV<OBB, Convex>(const Convex& s, const SimpleTransform& tf, OBB& bv);

template<>
void computeBV<OBB, Plane>(const Plane& s, const SimpleTransform& tf, OBB& bv);

template<>
void computeBV<RSS, Plane>(const Plane& s, const SimpleTransform& tf, RSS& bv);

template<>
void computeBV<OBBRSS, Plane>(const Plane& s, const SimpleTransform& tf, OBBRSS& bv);

template<>
void computeBV<kIOS, Plane>(const Plane& s, const SimpleTransform& tf, kIOS& bv);

template<>
void computeBV<KDOP<16>, Plane>(const Plane& s, const SimpleTransform& tf, KDOP<16>& bv);

template<>
void computeBV<KDOP<18>, Plane>(const Plane& s, const SimpleTransform& tf, KDOP<18>& bv);

template<>
void computeBV<KDOP<24>, Plane>(const Plane& s, const SimpleTransform& tf, KDOP<24>& bv);

}

#endif

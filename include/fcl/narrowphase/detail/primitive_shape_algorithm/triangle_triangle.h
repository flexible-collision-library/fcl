/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Georgia Tech Research Corporation
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
 *   * Neither the name of Georgia Tech Research Corporation nor the names of its
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

/** \author Andrew Price <arprice@gatech.edu> */

#ifndef TRIANGLE_TRIANGLE_H
#define TRIANGLE_TRIANGLE_H

#include "fcl/math/geometry.h"

namespace fcl
{

namespace detail
{

template <typename S>
constexpr S getNearZeroThreshold();

template <typename S, bool ROBUST=true>
bool isZero(const S& v);

/**
 * @brief vector_less_than Performs a lexicographical comparison on two vectors
 */
template<int DIM, typename PointT>
struct vector_less_than;

/**
 * @brief computeNormal Computes the unit normal vector for a triangle specified
 *  with a right hand winding
 *
 * Faces follow a right hand winding:
 *    c
 *    /\
 *   /__\
 *  a    b
 * n = (b-a)x(c-a) points out of the screen
 */
template <typename S>
inline Vector3<S> computeNormal(const Vector3<S>& a,
                                const Vector3<S>& b,
                                const Vector3<S>& c);

/**
 * @brief projectToLine Projects a point to an offset from the origin of a line
 *
 * Given a line { x | x = O + tV } and a point p, find t s.t. l(t) is closest to p
 */
template <typename S>
inline S projectToLine(const Vector3<S>& lineVec,
                       const Vector3<S>& lineOrig,
                       const Vector3<S>& freePt);

/**
 * @brief solveCramer Solve a simple pair of linear equations
 *
 * Solves the system:
 * a1*x+b1*y=c1
 * a2*x+b2*y=c2
 * for x and y
 *
 * @param[out] x
 * @param[out] y
 */
template <typename S>
inline void solveCramer(const S a1, const S b1, const S c1,
                        const S a2, const S b2, const S c2,
                        S& x, S& y);

/**
 * @brief areCoplanar Tests whether <n1,t1> and <n2,t2> define the same plane
 */
template <typename S, bool ROBUST=true>
inline bool areCoplanar(const Vector3<S>& n1, const S t1,
                        const Vector3<S>& n2, const S t2);

/**
 * @brief cross2 Defines the perp product (2D cross product)
 */
template <typename S>
inline S cross2(const Vector2<S>& u, const Vector2<S>& v);

/**
 * @brief assuming that q lies along line p0p1, is q in the segment p0p1?
 */
template <typename S>
inline bool isInSegment(const Vector2<S>& p0, const Vector2<S>& p1,
                        const Vector2<S>& q);

/**
 * @brief lineSegmentIntersect Report whether and where two line segments intersect
 * @param p0 Start of line 1
 * @param p1 End of line 1
 * @param q0 Start of line 2
 * @param q1 End of line 2
 * @param[out] r0 First intersection (only valid if return > 0)
 * @param[out] r1 Second intersection (only valid if return == 2)
 * @return number of intersections (0, 1, or 2)
 */
template <typename S, bool ROBUST=true>
int lineSegmentIntersect(const Vector2<S>& p0, const Vector2<S>& p1,
                         const Vector2<S>& q0, const Vector2<S>& q1,
                         Vector2<S>& r0, Vector2<S>& r1);

/**
 * @brief pointInTriangle Tests whether point p lies within triangle abc
 */
template <typename S>
bool pointInTriangle(const Vector2<S>& a, const Vector2<S>& b, const Vector2<S>& c,
                     const Vector2<S>& p);

template <typename S, bool ROBUST=true>
bool overlapCoplanarTriangles(const Vector3<S>& p1, const Vector3<S>& p2, const Vector3<S>& p3,
                              const Vector3<S>& q1, const Vector3<S>& q2, const Vector3<S>& q3,
                              const Vector3<S>& n1, const S t1,
                              Vector3<S>* contact_points,
                              unsigned int* num_contact_points,
                              S* penetration_depth,
                              Vector3<S>* normal);

template <typename S, bool ROBUST=true>
bool intersectTriangles(const Vector3<S>& p1, const Vector3<S>& p2, const Vector3<S>& p3,
                        const Vector3<S>& q1, const Vector3<S>& q2, const Vector3<S>& q3,
                        Vector3<S>* contact_points,
                        unsigned int* num_contact_points,
                        S* penetration_depth,
                        Vector3<S>* normal);

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/primitive_shape_algorithm/triangle_triangle-inl.h"

#endif // TRIANGLE_TRIANGLE_H

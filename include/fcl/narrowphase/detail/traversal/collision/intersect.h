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

#ifndef FCL_NARROWPHASE_DETAIL_INTERSECT_H
#define FCL_NARROWPHASE_DETAIL_INTERSECT_H

#include <limits>
#include "fcl/common/types.h"
#include "fcl/math/geometry.h"
#include "fcl/math/detail/polysolver.h"

namespace fcl
{

namespace detail
{

/// @brief CCD intersect kernel among primitives 
template <typename S>
class FCL_EXPORT Intersect
{

public:

  /// @brief CCD intersect between one vertex and one face
  /// [a0, b0, c0] and [a1, b1, c1] are points for the triangle face in time t0 and t1
  /// p0 and p1 are points for vertex in time t0 and t1
  /// p_i returns the coordinate of the collision point
  static bool intersect_VF(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& c0, const Vector3<S>& p0,
                           const Vector3<S>& a1, const Vector3<S>& b1, const Vector3<S>& c1, const Vector3<S>& p1,
                           S* collision_time, Vector3<S>* p_i, bool useNewton = true);

  /// @brief CCD intersect between two edges
  /// [a0, b0] and [a1, b1] are points for one edge in time t0 and t1
  /// [c0, d0] and [c1, d1] are points for the other edge in time t0 and t1
  /// p_i returns the coordinate of the collision point
  static bool intersect_EE(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& c0, const Vector3<S>& d0,
                           const Vector3<S>& a1, const Vector3<S>& b1, const Vector3<S>& c1, const Vector3<S>& d1,
                           S* collision_time, Vector3<S>* p_i, bool useNewton = true);

  /// @brief CCD intersect between one vertex and one face, using additional filter 
  static bool intersect_VF_filtered(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& c0, const Vector3<S>& p0,
                                    const Vector3<S>& a1, const Vector3<S>& b1, const Vector3<S>& c1, const Vector3<S>& p1,
                                    S* collision_time, Vector3<S>* p_i, bool useNewton = true);

  /// @brief CCD intersect between two edges, using additional filter 
  static bool intersect_EE_filtered(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& c0, const Vector3<S>& d0,
                                    const Vector3<S>& a1, const Vector3<S>& b1, const Vector3<S>& c1, const Vector3<S>& d1,
                                    S* collision_time, Vector3<S>* p_i, bool useNewton = true);

  /// @brief CCD intersect between one vertex and and one edge 
  static bool intersect_VE(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& p0,
                           const Vector3<S>& a1, const Vector3<S>& b1, const Vector3<S>& p1,
                           const Vector3<S>& L);

  /// @brief CD intersect between two triangles [P1, P2, P3] and [Q1, Q2, Q3]
  static bool intersect_Triangle(
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      const Vector3<S>& Q1,
      const Vector3<S>& Q2,
      const Vector3<S>& Q3,
      Vector3<S>* contact_points = nullptr,
      unsigned int* num_contact_points = nullptr,
      S* penetration_depth = nullptr,
      Vector3<S>* normal = nullptr);

  /// @brief CD intersect between two triangles [P1, P2, P3] and [Q1, Q2, Q3]
  static bool intersect_Triangle_ODE_style(
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      const Vector3<S>& Q1,
      const Vector3<S>& Q2,
      const Vector3<S>& Q3,
      Vector3<S>* contact_points = nullptr,
      unsigned int* num_contact_points = nullptr,
      S* penetration_depth = nullptr,
      Vector3<S>* normal = nullptr);

  static bool intersect_Triangle(
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      const Vector3<S>& Q1,
      const Vector3<S>& Q2,
      const Vector3<S>& Q3,
      const Matrix3<S>& R,
      const Vector3<S>& T,
      Vector3<S>* contact_points = nullptr,
      unsigned int* num_contact_points = nullptr,
      S* penetration_depth = nullptr,
      Vector3<S>* normal = nullptr);

  static bool intersect_Triangle(
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      const Vector3<S>& Q1,
      const Vector3<S>& Q2,
      const Vector3<S>& Q3,
      const Transform3<S>& tf,
      Vector3<S>* contact_points = nullptr,
      unsigned int* num_contact_points = nullptr,
      S* penetration_depth = nullptr,
      Vector3<S>* normal = nullptr);
  
private:

  /// @brief Project function used in intersect_Triangle() 
  static int project6(const Vector3<S>& ax,
                      const Vector3<S>& p1, const Vector3<S>& p2, const Vector3<S>& p3,
                      const Vector3<S>& q1, const Vector3<S>& q2, const Vector3<S>& q3);

  /// @brief Check whether one value is zero 
  static bool isZero(S v);

  /// @brief Solve the cubic function using Newton method, also satisfies the interval restriction 
  static bool solveCubicWithIntervalNewton(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& c0, const Vector3<S>& d0,
                                           const Vector3<S>& va, const Vector3<S>& vb, const Vector3<S>& vc, const Vector3<S>& vd,
                                           S& l, S& r, bool bVF, S coeffs[], Vector3<S>* data = nullptr);

  /// @brief Check whether one point p is within triangle [a, b, c] 
  static bool insideTriangle(const Vector3<S>& a, const Vector3<S>& b, const Vector3<S>& c, const Vector3<S>&p);

  /// @brief Check whether one point p is within a line segment [a, b] 
  static bool insideLineSegment(const Vector3<S>& a, const Vector3<S>& b, const Vector3<S>& p);

  /// @brief Calculate the line segment papb that is the shortest route between
  /// two lines p1p2 and p3p4. Calculate also the values of mua and mub where
  ///                    pa = p1 + mua (p2 - p1)
  ///                    pb = p3 + mub (p4 - p3)
  /// return FALSE if no solution exists.
  static bool linelineIntersect(const Vector3<S>& p1, const Vector3<S>& p2, const Vector3<S>& p3, const Vector3<S>& p4,
                                Vector3<S>* pa, Vector3<S>* pb, S* mua, S* mub);

  /// @brief Check whether a root for VF intersection is valid (i.e. within the triangle at intersection t 
  static bool checkRootValidity_VF(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& c0, const Vector3<S>& p0,
                                   const Vector3<S>& va, const Vector3<S>& vb, const Vector3<S>& vc, const Vector3<S>& vp,
                                   S t);

  /// @brief Check whether a root for EE intersection is valid (i.e. within the two edges intersected at the given time 
  static bool checkRootValidity_EE(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& c0, const Vector3<S>& d0,
                                   const Vector3<S>& va, const Vector3<S>& vb, const Vector3<S>& vc, const Vector3<S>& vd,
                                   S t, Vector3<S>* q_i = nullptr);

  /// @brief Check whether a root for VE intersection is valid 
  static bool checkRootValidity_VE(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& p0,
                                   const Vector3<S>& va, const Vector3<S>& vb, const Vector3<S>& vp,
                                   S t);

  /// @brief Solve a square function for EE intersection (with interval restriction) 
  static bool solveSquare(S a, S b, S c,
                          const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& c0, const Vector3<S>& d0,
                          const Vector3<S>& va, const Vector3<S>& vb, const Vector3<S>& vc, const Vector3<S>& vd,
                          bool bVF,
                          S* ret);

  /// @brief Solve a square function for VE intersection (with interval restriction) 
  static bool solveSquare(S a, S b, S c,
                          const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& p0,
                          const Vector3<S>& va, const Vector3<S>& vb, const Vector3<S>& vp);

  /// @brief Compute the cubic coefficients for VF intersection
  /// See Paper "Interactive Continuous Collision Detection between Deformable Models using Connectivity-Based Culling", Equation 1.
   
  static void computeCubicCoeff_VF(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& c0, const Vector3<S>& p0,
                                   const Vector3<S>& va, const Vector3<S>& vb, const Vector3<S>& vc, const Vector3<S>& vp,
                                   S* a, S* b, S* c, S* d);

  /// @brief Compute the cubic coefficients for EE intersection 
  static void computeCubicCoeff_EE(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& c0, const Vector3<S>& d0,
                                   const Vector3<S>& va, const Vector3<S>& vb, const Vector3<S>& vc, const Vector3<S>& vd,
                                   S* a, S* b, S* c, S* d);

  /// @brief Compute the cubic coefficients for VE intersection 
  static void computeCubicCoeff_VE(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& p0,
                                   const Vector3<S>& va, const Vector3<S>& vb, const Vector3<S>& vp,
                                   const Vector3<S>& L,
                                   S* a, S* b, S* c);

  /// @brief filter for intersection, works for both VF and EE 
  static bool intersectPreFiltering(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& c0, const Vector3<S>& d0,
                                    const Vector3<S>& a1, const Vector3<S>& b1, const Vector3<S>& c1, const Vector3<S>& d1);

  /// @brief distance of point v to a plane n * x - t = 0 
  static S distanceToPlane(const Vector3<S>& n, S t, const Vector3<S>& v);

  /// @brief check wether points v1, v2, v2 are on the same side of plane n * x - t = 0 
  static bool sameSideOfPlane(const Vector3<S>& v1, const Vector3<S>& v2, const Vector3<S>& v3, const Vector3<S>& n, S t);

  /// @brief clip triangle v1, v2, v3 by the prism made by t1, t2 and t3. The normal of the prism is tn and is cutted up by to 
  static void clipTriangleByTriangleAndEdgePlanes(const Vector3<S>& v1, const Vector3<S>& v2, const Vector3<S>& v3,
                                                  const Vector3<S>& t1, const Vector3<S>& t2, const Vector3<S>& t3,
                                                  const Vector3<S>& tn, S to,
                                                  Vector3<S> clipped_points[], unsigned int* num_clipped_points, bool clip_triangle = false);

  /// @brief build a plane passed through triangle v1 v2 v3 
  static bool buildTrianglePlane(const Vector3<S>& v1, const Vector3<S>& v2, const Vector3<S>& v3, Vector3<S>* n, S* t);

  /// @brief build a plane pass through edge v1 and v2, normal is tn 
  static bool buildEdgePlane(const Vector3<S>& v1, const Vector3<S>& v2, const Vector3<S>& tn, Vector3<S>* n, S* t);

  /// @brief compute the points which has deepest penetration depth 
  static void computeDeepestPoints(Vector3<S>* clipped_points, unsigned int num_clipped_points, const Vector3<S>& n, S t, S* penetration_depth, Vector3<S>* deepest_points, unsigned int* num_deepest_points);

  /// @brief clip polygon by plane 
  static void clipPolygonByPlane(Vector3<S>* polygon_points, unsigned int num_polygon_points, const Vector3<S>& n, S t, Vector3<S> clipped_points[], unsigned int* num_clipped_points);

  /// @brief clip a line segment by plane 
  static void clipSegmentByPlane(const Vector3<S>& v1, const Vector3<S>& v2, const Vector3<S>& n, S t, Vector3<S>* clipped_point);

  /// @brief compute the cdf(x) 
  static S gaussianCDF(S x);

  static constexpr S getEpsilon();
  static constexpr S getNearZeroThreshold();
  static constexpr S getCcdResolution();
  static constexpr unsigned int getMaxTriangleClips();
};

using Intersectf = Intersect<float>;
using Intersectd = Intersect<double>;

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/traversal/collision/intersect-inl.h"

#endif

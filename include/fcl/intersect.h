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

#ifndef FCL_INTERSECT_H
#define FCL_INTERSECT_H

#include "fcl/math/transform.h"

namespace fcl
{

/// @brief A class solves polynomial degree (1,2,3) equations 
class PolySolver
{
public:
  /// @brief Solve a linear equation with coefficients c, return roots s and number of roots 
  static int solveLinear(FCL_REAL c[2], FCL_REAL s[1]);

  /// @brief Solve a quadratic function with coefficients c, return roots s and number of roots 
  static int solveQuadric(FCL_REAL c[3], FCL_REAL s[2]);

  /// @brief Solve a cubic function with coefficients c, return roots s and number of roots 
  static int solveCubic(FCL_REAL c[4], FCL_REAL s[3]);

private:
  /// @brief Check whether v is zero 
  static inline bool isZero(FCL_REAL v);

  /// @brief Compute v^{1/3} 
  static inline bool cbrt(FCL_REAL v);

  static const FCL_REAL NEAR_ZERO_THRESHOLD;
};


/// @brief CCD intersect kernel among primitives 
class Intersect
{

public:

  /// @brief CCD intersect between one vertex and one face
  /// [a0, b0, c0] and [a1, b1, c1] are points for the triangle face in time t0 and t1
  /// p0 and p1 are points for vertex in time t0 and t1
  /// p_i returns the coordinate of the collision point
  static bool intersect_VF(const Vec3f& a0, const Vec3f& b0, const Vec3f& c0, const Vec3f& p0,
                           const Vec3f& a1, const Vec3f& b1, const Vec3f& c1, const Vec3f& p1,
                           FCL_REAL* collision_time, Vec3f* p_i, bool useNewton = true);

  /// @brief CCD intersect between two edges
  /// [a0, b0] and [a1, b1] are points for one edge in time t0 and t1
  /// [c0, d0] and [c1, d1] are points for the other edge in time t0 and t1
  /// p_i returns the coordinate of the collision point
  static bool intersect_EE(const Vec3f& a0, const Vec3f& b0, const Vec3f& c0, const Vec3f& d0,
                           const Vec3f& a1, const Vec3f& b1, const Vec3f& c1, const Vec3f& d1,
                           FCL_REAL* collision_time, Vec3f* p_i, bool useNewton = true);

  /// @brief CCD intersect between one vertex and one face, using additional filter 
  static bool intersect_VF_filtered(const Vec3f& a0, const Vec3f& b0, const Vec3f& c0, const Vec3f& p0,
                                    const Vec3f& a1, const Vec3f& b1, const Vec3f& c1, const Vec3f& p1,
                                    FCL_REAL* collision_time, Vec3f* p_i, bool useNewton = true);

  /// @brief CCD intersect between two edges, using additional filter 
  static bool intersect_EE_filtered(const Vec3f& a0, const Vec3f& b0, const Vec3f& c0, const Vec3f& d0,
                                    const Vec3f& a1, const Vec3f& b1, const Vec3f& c1, const Vec3f& d1,
                                    FCL_REAL* collision_time, Vec3f* p_i, bool useNewton = true);

  /// @brief CCD intersect between one vertex and and one edge 
  static bool intersect_VE(const Vec3f& a0, const Vec3f& b0, const Vec3f& p0,
                           const Vec3f& a1, const Vec3f& b1, const Vec3f& p1,
                           const Vec3f& L);

  /// @brief CD intersect between two triangles [P1, P2, P3] and [Q1, Q2, Q3] 
  static bool intersect_Triangle(const Vec3f& P1, const Vec3f& P2, const Vec3f& P3,
                                 const Vec3f& Q1, const Vec3f& Q2, const Vec3f& Q3,
                                 Vec3f* contact_points = NULL,
                                 unsigned int* num_contact_points = NULL,
                                 FCL_REAL* penetration_depth = NULL,
                                 Vec3f* normal = NULL);

  static bool intersect_Triangle(const Vec3f& P1, const Vec3f& P2, const Vec3f& P3,
                                 const Vec3f& Q1, const Vec3f& Q2, const Vec3f& Q3,
                                 const Matrix3f& R, const Vec3f& T,
                                 Vec3f* contact_points = NULL,
                                 unsigned int* num_contact_points = NULL,
                                 FCL_REAL* penetration_depth = NULL,
                                 Vec3f* normal = NULL);

  static bool intersect_Triangle(const Vec3f& P1, const Vec3f& P2, const Vec3f& P3,
                                 const Vec3f& Q1, const Vec3f& Q2, const Vec3f& Q3,
                                 const Transform3f& tf,
                                 Vec3f* contact_points = NULL,
                                 unsigned int* num_contact_points = NULL,
                                 FCL_REAL* penetration_depth = NULL,
                                 Vec3f* normal = NULL);
  
private:

  /// @brief Project function used in intersect_Triangle() 
  static int project6(const Vec3f& ax,
                      const Vec3f& p1, const Vec3f& p2, const Vec3f& p3,
                      const Vec3f& q1, const Vec3f& q2, const Vec3f& q3);

  /// @brief Check whether one value is zero 
  static inline bool isZero(FCL_REAL v);

  /// @brief Solve the cubic function using Newton method, also satisfies the interval restriction 
  static bool solveCubicWithIntervalNewton(const Vec3f& a0, const Vec3f& b0, const Vec3f& c0, const Vec3f& d0,
                                           const Vec3f& va, const Vec3f& vb, const Vec3f& vc, const Vec3f& vd,
                                           FCL_REAL& l, FCL_REAL& r, bool bVF, FCL_REAL coeffs[], Vec3f* data = NULL);

  /// @brief Check whether one point p is within triangle [a, b, c] 
  static bool insideTriangle(const Vec3f& a, const Vec3f& b, const Vec3f& c, const Vec3f&p);

  /// @brief Check whether one point p is within a line segment [a, b] 
  static bool insideLineSegment(const Vec3f& a, const Vec3f& b, const Vec3f& p);

  /// @brief Calculate the line segment papb that is the shortest route between
  /// two lines p1p2 and p3p4. Calculate also the values of mua and mub where
  ///                    pa = p1 + mua (p2 - p1)
  ///                    pb = p3 + mub (p4 - p3)
  /// return FALSE if no solution exists.
  static bool linelineIntersect(const Vec3f& p1, const Vec3f& p2, const Vec3f& p3, const Vec3f& p4,
                                Vec3f* pa, Vec3f* pb, FCL_REAL* mua, FCL_REAL* mub);

  /// @brief Check whether a root for VF intersection is valid (i.e. within the triangle at intersection t 
  static bool checkRootValidity_VF(const Vec3f& a0, const Vec3f& b0, const Vec3f& c0, const Vec3f& p0,
                                   const Vec3f& va, const Vec3f& vb, const Vec3f& vc, const Vec3f& vp,
                                   FCL_REAL t);

  /// @brief Check whether a root for EE intersection is valid (i.e. within the two edges intersected at the given time 
  static bool checkRootValidity_EE(const Vec3f& a0, const Vec3f& b0, const Vec3f& c0, const Vec3f& d0,
                                   const Vec3f& va, const Vec3f& vb, const Vec3f& vc, const Vec3f& vd,
                                   FCL_REAL t, Vec3f* q_i = NULL);

  /// @brief Check whether a root for VE intersection is valid 
  static bool checkRootValidity_VE(const Vec3f& a0, const Vec3f& b0, const Vec3f& p0,
                                   const Vec3f& va, const Vec3f& vb, const Vec3f& vp,
                                   FCL_REAL t);

  /// @brief Solve a square function for EE intersection (with interval restriction) 
  static bool solveSquare(FCL_REAL a, FCL_REAL b, FCL_REAL c,
                          const Vec3f& a0, const Vec3f& b0, const Vec3f& c0, const Vec3f& d0,
                          const Vec3f& va, const Vec3f& vb, const Vec3f& vc, const Vec3f& vd,
                          bool bVF,
                          FCL_REAL* ret);

  /// @brief Solve a square function for VE intersection (with interval restriction) 
  static bool solveSquare(FCL_REAL a, FCL_REAL b, FCL_REAL c,
                          const Vec3f& a0, const Vec3f& b0, const Vec3f& p0,
                          const Vec3f& va, const Vec3f& vb, const Vec3f& vp);

  /// @brief Compute the cubic coefficients for VF intersection
  /// See Paper "Interactive Continuous Collision Detection between Deformable Models using Connectivity-Based Culling", Equation 1.
   
  static void computeCubicCoeff_VF(const Vec3f& a0, const Vec3f& b0, const Vec3f& c0, const Vec3f& p0,
                                   const Vec3f& va, const Vec3f& vb, const Vec3f& vc, const Vec3f& vp,
                                   FCL_REAL* a, FCL_REAL* b, FCL_REAL* c, FCL_REAL* d);

  /// @brief Compute the cubic coefficients for EE intersection 
  static void computeCubicCoeff_EE(const Vec3f& a0, const Vec3f& b0, const Vec3f& c0, const Vec3f& d0,
                                   const Vec3f& va, const Vec3f& vb, const Vec3f& vc, const Vec3f& vd,
                                   FCL_REAL* a, FCL_REAL* b, FCL_REAL* c, FCL_REAL* d);

  /// @brief Compute the cubic coefficients for VE intersection 
  static void computeCubicCoeff_VE(const Vec3f& a0, const Vec3f& b0, const Vec3f& p0,
                                   const Vec3f& va, const Vec3f& vb, const Vec3f& vp,
                                   const Vec3f& L,
                                   FCL_REAL* a, FCL_REAL* b, FCL_REAL* c);

  /// @brief filter for intersection, works for both VF and EE 
  static bool intersectPreFiltering(const Vec3f& a0, const Vec3f& b0, const Vec3f& c0, const Vec3f& d0,
                                    const Vec3f& a1, const Vec3f& b1, const Vec3f& c1, const Vec3f& d1);

  /// @brief distance of point v to a plane n * x - t = 0 
  static FCL_REAL distanceToPlane(const Vec3f& n, FCL_REAL t, const Vec3f& v);

  /// @brief check wether points v1, v2, v2 are on the same side of plane n * x - t = 0 
  static bool sameSideOfPlane(const Vec3f& v1, const Vec3f& v2, const Vec3f& v3, const Vec3f& n, FCL_REAL t);

  /// @brief clip triangle v1, v2, v3 by the prism made by t1, t2 and t3. The normal of the prism is tn and is cutted up by to 
  static void clipTriangleByTriangleAndEdgePlanes(const Vec3f& v1, const Vec3f& v2, const Vec3f& v3,
                                                  const Vec3f& t1, const Vec3f& t2, const Vec3f& t3,
                                                  const Vec3f& tn, FCL_REAL to,
                                                  Vec3f clipped_points[], unsigned int* num_clipped_points, bool clip_triangle = false);

  /// @brief build a plane passed through triangle v1 v2 v3 
  static bool buildTrianglePlane(const Vec3f& v1, const Vec3f& v2, const Vec3f& v3, Vec3f* n, FCL_REAL* t);

  /// @brief build a plane pass through edge v1 and v2, normal is tn 
  static bool buildEdgePlane(const Vec3f& v1, const Vec3f& v2, const Vec3f& tn, Vec3f* n, FCL_REAL* t);

  /// @brief compute the points which has deepest penetration depth 
  static void computeDeepestPoints(Vec3f* clipped_points, unsigned int num_clipped_points, const Vec3f& n, FCL_REAL t, FCL_REAL* penetration_depth, Vec3f* deepest_points, unsigned int* num_deepest_points);

  /// @brief clip polygon by plane 
  static void clipPolygonByPlane(Vec3f* polygon_points, unsigned int num_polygon_points, const Vec3f& n, FCL_REAL t, Vec3f clipped_points[], unsigned int* num_clipped_points);

  /// @brief clip a line segment by plane 
  static void clipSegmentByPlane(const Vec3f& v1, const Vec3f& v2, const Vec3f& n, FCL_REAL t, Vec3f* clipped_point);

  /// @brief compute the cdf(x) 
  static FCL_REAL gaussianCDF(FCL_REAL x)
  {
    return 0.5 * std::erfc(-x / sqrt(2.0));
  }


  static const FCL_REAL EPSILON;
  static const FCL_REAL NEAR_ZERO_THRESHOLD;
  static const FCL_REAL CCD_RESOLUTION;
  static const unsigned int MAX_TRIANGLE_CLIPS = 8;
};

/// @brief Project functions
class Project
{
public:
  struct ProjectResult
  {
    /// @brief Parameterization of the projected point (based on the simplex to be projected, use 2 or 3 or 4 of the array)
    FCL_REAL parameterization[4];

    /// @brief square distance from the query point to the projected simplex
    FCL_REAL sqr_distance;

    /// @brief the code of the projection type
    unsigned int encode;

    ProjectResult() : sqr_distance(-1), encode(0)
    {
    }
  };

  /// @brief Project point p onto line a-b
  static ProjectResult projectLine(const Vec3f& a, const Vec3f& b, const Vec3f& p);

  /// @brief Project point p onto triangle a-b-c
  static ProjectResult projectTriangle(const Vec3f& a, const Vec3f& b, const Vec3f& c, const Vec3f& p);

  /// @brief Project point p onto tetrahedra a-b-c-d
  static ProjectResult projectTetrahedra(const Vec3f& a, const Vec3f& b, const Vec3f& c, const Vec3f& d, const Vec3f& p);

  /// @brief Project origin (0) onto line a-b
  static ProjectResult projectLineOrigin(const Vec3f& a, const Vec3f& b);

  /// @brief Project origin (0) onto triangle a-b-c
  static ProjectResult projectTriangleOrigin(const Vec3f& a, const Vec3f& b, const Vec3f& c);

  /// @brief Project origin (0) onto tetrahedran a-b-c-d
  static ProjectResult projectTetrahedraOrigin(const Vec3f& a, const Vec3f& b, const Vec3f& c, const Vec3f& d);
};

/// @brief Triangle distance functions
class TriangleDistance
{
public:

  /// @brief Returns closest points between an segment pair.
  /// The first segment is P + t * A
  /// The second segment is Q + t * B
  /// X, Y are the closest points on the two segments
  /// VEC is the vector between X and Y
  static void segPoints(const Vec3f& P, const Vec3f& A, const Vec3f& Q, const Vec3f& B,
                        Vec3f& VEC, Vec3f& X, Vec3f& Y);

  /// @brief Compute the closest points on two triangles given their absolute coordinate, and returns the distance between them
  /// S and T are two triangles
  /// If the triangles are disjoint, P and Q give the closet points of S and T respectively. However,
  /// if the triangles overlap, P and Q are basically a random pair of points from the triangles, not
  /// coincident points on the intersection of the triangles, as might be expected.
  static FCL_REAL triDistance(const Vec3f S[3], const Vec3f T[3], Vec3f& P, Vec3f& Q);

  static FCL_REAL triDistance(const Vec3f& S1, const Vec3f& S2, const Vec3f& S3,
                              const Vec3f& T1, const Vec3f& T2, const Vec3f& T3,
                              Vec3f& P, Vec3f& Q);

  /// @brief Compute the closest points on two triangles given the relative transform between them, and returns the distance between them
  /// S and T are two triangles
  /// If the triangles are disjoint, P and Q give the closet points of S and T respectively. However,
  /// if the triangles overlap, P and Q are basically a random pair of points from the triangles, not
  /// coincident points on the intersection of the triangles, as might be expected.
  /// The returned P and Q are both in the coordinate of the first triangle's coordinate
  static FCL_REAL triDistance(const Vec3f S[3], const Vec3f T[3],
                              const Matrix3f& R, const Vec3f& Tl,
                              Vec3f& P, Vec3f& Q);

  static FCL_REAL triDistance(const Vec3f S[3], const Vec3f T[3],
                              const Transform3f& tf,
                              Vec3f& P, Vec3f& Q);

  static FCL_REAL triDistance(const Vec3f& S1, const Vec3f& S2, const Vec3f& S3,
                              const Vec3f& T1, const Vec3f& T2, const Vec3f& T3,
                              const Matrix3f& R, const Vec3f& Tl,
                              Vec3f& P, Vec3f& Q);

  static FCL_REAL triDistance(const Vec3f& S1, const Vec3f& S2, const Vec3f& S3,
                              const Vec3f& T1, const Vec3f& T2, const Vec3f& T3,
                              const Transform3f& tf,
                              Vec3f& P, Vec3f& Q);

};


}


#endif

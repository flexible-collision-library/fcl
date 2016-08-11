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

#include <iostream>
#include <limits>
#include <vector>
#include "fcl/data_types.h"
#include "fcl/math/geometry.h"

namespace fcl
{

/// @brief A class solves polynomial degree (1,2,3) equations 
template <typename Scalar>
class PolySolver
{
public:
  /// @brief Solve a linear equation with coefficients c, return roots s and number of roots 
  static int solveLinear(Scalar c[2], Scalar s[1]);

  /// @brief Solve a quadratic function with coefficients c, return roots s and number of roots 
  static int solveQuadric(Scalar c[3], Scalar s[2]);

  /// @brief Solve a cubic function with coefficients c, return roots s and number of roots 
  static int solveCubic(Scalar c[4], Scalar s[3]);

private:
  /// @brief Check whether v is zero 
  static inline bool isZero(Scalar v);

  /// @brief Compute v^{1/3} 
  static inline bool cbrt(Scalar v);

  static constexpr Scalar getNearZeroThreshold() { return 1e-9; }
};

using PolySolverf = PolySolver<float>;
using PolySolverd = PolySolver<double>;

/// @brief CCD intersect kernel among primitives 
template <typename Scalar>
class Intersect
{

public:

  /// @brief CCD intersect between one vertex and one face
  /// [a0, b0, c0] and [a1, b1, c1] are points for the triangle face in time t0 and t1
  /// p0 and p1 are points for vertex in time t0 and t1
  /// p_i returns the coordinate of the collision point
  static bool intersect_VF(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& c0, const Vector3<Scalar>& p0,
                           const Vector3<Scalar>& a1, const Vector3<Scalar>& b1, const Vector3<Scalar>& c1, const Vector3<Scalar>& p1,
                           Scalar* collision_time, Vector3<Scalar>* p_i, bool useNewton = true);

  /// @brief CCD intersect between two edges
  /// [a0, b0] and [a1, b1] are points for one edge in time t0 and t1
  /// [c0, d0] and [c1, d1] are points for the other edge in time t0 and t1
  /// p_i returns the coordinate of the collision point
  static bool intersect_EE(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& c0, const Vector3<Scalar>& d0,
                           const Vector3<Scalar>& a1, const Vector3<Scalar>& b1, const Vector3<Scalar>& c1, const Vector3<Scalar>& d1,
                           Scalar* collision_time, Vector3<Scalar>* p_i, bool useNewton = true);

  /// @brief CCD intersect between one vertex and one face, using additional filter 
  static bool intersect_VF_filtered(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& c0, const Vector3<Scalar>& p0,
                                    const Vector3<Scalar>& a1, const Vector3<Scalar>& b1, const Vector3<Scalar>& c1, const Vector3<Scalar>& p1,
                                    Scalar* collision_time, Vector3<Scalar>* p_i, bool useNewton = true);

  /// @brief CCD intersect between two edges, using additional filter 
  static bool intersect_EE_filtered(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& c0, const Vector3<Scalar>& d0,
                                    const Vector3<Scalar>& a1, const Vector3<Scalar>& b1, const Vector3<Scalar>& c1, const Vector3<Scalar>& d1,
                                    Scalar* collision_time, Vector3<Scalar>* p_i, bool useNewton = true);

  /// @brief CCD intersect between one vertex and and one edge 
  static bool intersect_VE(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& p0,
                           const Vector3<Scalar>& a1, const Vector3<Scalar>& b1, const Vector3<Scalar>& p1,
                           const Vector3<Scalar>& L);

  /// @brief CD intersect between two triangles [P1, P2, P3] and [Q1, Q2, Q3] 
  static bool intersect_Triangle(const Vector3<Scalar>& P1, const Vector3<Scalar>& P2, const Vector3<Scalar>& P3,
                                 const Vector3<Scalar>& Q1, const Vector3<Scalar>& Q2, const Vector3<Scalar>& Q3,
                                 Vector3<Scalar>* contact_points = NULL,
                                 unsigned int* num_contact_points = NULL,
                                 Scalar* penetration_depth = NULL,
                                 Vector3<Scalar>* normal = NULL);

  static bool intersect_Triangle(
      const Vector3<Scalar>& P1,
      const Vector3<Scalar>& P2,
      const Vector3<Scalar>& P3,
      const Vector3<Scalar>& Q1,
      const Vector3<Scalar>& Q2,
      const Vector3<Scalar>& Q3,
      const Matrix3<Scalar>& R,
      const Vector3<Scalar>& T,
      Vector3<Scalar>* contact_points = NULL,
      unsigned int* num_contact_points = NULL,
      Scalar* penetration_depth = NULL,
      Vector3<Scalar>* normal = NULL);

  static bool intersect_Triangle(
      const Vector3<Scalar>& P1,
      const Vector3<Scalar>& P2,
      const Vector3<Scalar>& P3,
      const Vector3<Scalar>& Q1,
      const Vector3<Scalar>& Q2,
      const Vector3<Scalar>& Q3,
      const Transform3<Scalar>& tf,
      Vector3<Scalar>* contact_points = NULL,
      unsigned int* num_contact_points = NULL,
      Scalar* penetration_depth = NULL,
      Vector3<Scalar>* normal = NULL);
  
private:

  /// @brief Project function used in intersect_Triangle() 
  static int project6(const Vector3<Scalar>& ax,
                      const Vector3<Scalar>& p1, const Vector3<Scalar>& p2, const Vector3<Scalar>& p3,
                      const Vector3<Scalar>& q1, const Vector3<Scalar>& q2, const Vector3<Scalar>& q3);

  /// @brief Check whether one value is zero 
  static inline bool isZero(Scalar v);

  /// @brief Solve the cubic function using Newton method, also satisfies the interval restriction 
  static bool solveCubicWithIntervalNewton(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& c0, const Vector3<Scalar>& d0,
                                           const Vector3<Scalar>& va, const Vector3<Scalar>& vb, const Vector3<Scalar>& vc, const Vector3<Scalar>& vd,
                                           Scalar& l, Scalar& r, bool bVF, Scalar coeffs[], Vector3<Scalar>* data = NULL);

  /// @brief Check whether one point p is within triangle [a, b, c] 
  static bool insideTriangle(const Vector3<Scalar>& a, const Vector3<Scalar>& b, const Vector3<Scalar>& c, const Vector3<Scalar>&p);

  /// @brief Check whether one point p is within a line segment [a, b] 
  static bool insideLineSegment(const Vector3<Scalar>& a, const Vector3<Scalar>& b, const Vector3<Scalar>& p);

  /// @brief Calculate the line segment papb that is the shortest route between
  /// two lines p1p2 and p3p4. Calculate also the values of mua and mub where
  ///                    pa = p1 + mua (p2 - p1)
  ///                    pb = p3 + mub (p4 - p3)
  /// return FALSE if no solution exists.
  static bool linelineIntersect(const Vector3<Scalar>& p1, const Vector3<Scalar>& p2, const Vector3<Scalar>& p3, const Vector3<Scalar>& p4,
                                Vector3<Scalar>* pa, Vector3<Scalar>* pb, Scalar* mua, Scalar* mub);

  /// @brief Check whether a root for VF intersection is valid (i.e. within the triangle at intersection t 
  static bool checkRootValidity_VF(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& c0, const Vector3<Scalar>& p0,
                                   const Vector3<Scalar>& va, const Vector3<Scalar>& vb, const Vector3<Scalar>& vc, const Vector3<Scalar>& vp,
                                   Scalar t);

  /// @brief Check whether a root for EE intersection is valid (i.e. within the two edges intersected at the given time 
  static bool checkRootValidity_EE(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& c0, const Vector3<Scalar>& d0,
                                   const Vector3<Scalar>& va, const Vector3<Scalar>& vb, const Vector3<Scalar>& vc, const Vector3<Scalar>& vd,
                                   Scalar t, Vector3<Scalar>* q_i = NULL);

  /// @brief Check whether a root for VE intersection is valid 
  static bool checkRootValidity_VE(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& p0,
                                   const Vector3<Scalar>& va, const Vector3<Scalar>& vb, const Vector3<Scalar>& vp,
                                   Scalar t);

  /// @brief Solve a square function for EE intersection (with interval restriction) 
  static bool solveSquare(Scalar a, Scalar b, Scalar c,
                          const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& c0, const Vector3<Scalar>& d0,
                          const Vector3<Scalar>& va, const Vector3<Scalar>& vb, const Vector3<Scalar>& vc, const Vector3<Scalar>& vd,
                          bool bVF,
                          Scalar* ret);

  /// @brief Solve a square function for VE intersection (with interval restriction) 
  static bool solveSquare(Scalar a, Scalar b, Scalar c,
                          const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& p0,
                          const Vector3<Scalar>& va, const Vector3<Scalar>& vb, const Vector3<Scalar>& vp);

  /// @brief Compute the cubic coefficients for VF intersection
  /// See Paper "Interactive Continuous Collision Detection between Deformable Models using Connectivity-Based Culling", Equation 1.
   
  static void computeCubicCoeff_VF(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& c0, const Vector3<Scalar>& p0,
                                   const Vector3<Scalar>& va, const Vector3<Scalar>& vb, const Vector3<Scalar>& vc, const Vector3<Scalar>& vp,
                                   Scalar* a, Scalar* b, Scalar* c, Scalar* d);

  /// @brief Compute the cubic coefficients for EE intersection 
  static void computeCubicCoeff_EE(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& c0, const Vector3<Scalar>& d0,
                                   const Vector3<Scalar>& va, const Vector3<Scalar>& vb, const Vector3<Scalar>& vc, const Vector3<Scalar>& vd,
                                   Scalar* a, Scalar* b, Scalar* c, Scalar* d);

  /// @brief Compute the cubic coefficients for VE intersection 
  static void computeCubicCoeff_VE(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& p0,
                                   const Vector3<Scalar>& va, const Vector3<Scalar>& vb, const Vector3<Scalar>& vp,
                                   const Vector3<Scalar>& L,
                                   Scalar* a, Scalar* b, Scalar* c);

  /// @brief filter for intersection, works for both VF and EE 
  static bool intersectPreFiltering(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& c0, const Vector3<Scalar>& d0,
                                    const Vector3<Scalar>& a1, const Vector3<Scalar>& b1, const Vector3<Scalar>& c1, const Vector3<Scalar>& d1);

  /// @brief distance of point v to a plane n * x - t = 0 
  static Scalar distanceToPlane(const Vector3<Scalar>& n, Scalar t, const Vector3<Scalar>& v);

  /// @brief check wether points v1, v2, v2 are on the same side of plane n * x - t = 0 
  static bool sameSideOfPlane(const Vector3<Scalar>& v1, const Vector3<Scalar>& v2, const Vector3<Scalar>& v3, const Vector3<Scalar>& n, Scalar t);

  /// @brief clip triangle v1, v2, v3 by the prism made by t1, t2 and t3. The normal of the prism is tn and is cutted up by to 
  static void clipTriangleByTriangleAndEdgePlanes(const Vector3<Scalar>& v1, const Vector3<Scalar>& v2, const Vector3<Scalar>& v3,
                                                  const Vector3<Scalar>& t1, const Vector3<Scalar>& t2, const Vector3<Scalar>& t3,
                                                  const Vector3<Scalar>& tn, Scalar to,
                                                  Vector3<Scalar> clipped_points[], unsigned int* num_clipped_points, bool clip_triangle = false);

  /// @brief build a plane passed through triangle v1 v2 v3 
  static bool buildTrianglePlane(const Vector3<Scalar>& v1, const Vector3<Scalar>& v2, const Vector3<Scalar>& v3, Vector3<Scalar>* n, Scalar* t);

  /// @brief build a plane pass through edge v1 and v2, normal is tn 
  static bool buildEdgePlane(const Vector3<Scalar>& v1, const Vector3<Scalar>& v2, const Vector3<Scalar>& tn, Vector3<Scalar>* n, Scalar* t);

  /// @brief compute the points which has deepest penetration depth 
  static void computeDeepestPoints(Vector3<Scalar>* clipped_points, unsigned int num_clipped_points, const Vector3<Scalar>& n, Scalar t, Scalar* penetration_depth, Vector3<Scalar>* deepest_points, unsigned int* num_deepest_points);

  /// @brief clip polygon by plane 
  static void clipPolygonByPlane(Vector3<Scalar>* polygon_points, unsigned int num_polygon_points, const Vector3<Scalar>& n, Scalar t, Vector3<Scalar> clipped_points[], unsigned int* num_clipped_points);

  /// @brief clip a line segment by plane 
  static void clipSegmentByPlane(const Vector3<Scalar>& v1, const Vector3<Scalar>& v2, const Vector3<Scalar>& n, Scalar t, Vector3<Scalar>* clipped_point);

  /// @brief compute the cdf(x) 
  static Scalar gaussianCDF(Scalar x)
  {
    return 0.5 * std::erfc(-x / sqrt(2.0));
  }


  static constexpr Scalar getEpsilon() { return 1e-5; }
  static constexpr Scalar getNearZeroThreshold() { return 1e-7; }
  static constexpr Scalar getCcdResolution() { return 1e-7; }
  static constexpr unsigned int getMaxTriangleClips() { return 8; }
};

using Intersectf = Intersect<float>;
using Intersectd = Intersect<double>;

/// @brief Project functions
template <typename Scalar>
class Project
{
public:
  struct ProjectResult
  {
    /// @brief Parameterization of the projected point (based on the simplex to be projected, use 2 or 3 or 4 of the array)
    Scalar parameterization[4];

    /// @brief square distance from the query point to the projected simplex
    Scalar sqr_distance;

    /// @brief the code of the projection type
    unsigned int encode;

    ProjectResult() : parameterization{0.0, 0.0, 0.0, 0.0}, sqr_distance(-1), encode(0)
    {
    }
  };

  /// @brief Project point p onto line a-b
  static ProjectResult projectLine(const Vector3<Scalar>& a, const Vector3<Scalar>& b, const Vector3<Scalar>& p);

  /// @brief Project point p onto triangle a-b-c
  static ProjectResult projectTriangle(const Vector3<Scalar>& a, const Vector3<Scalar>& b, const Vector3<Scalar>& c, const Vector3<Scalar>& p);

  /// @brief Project point p onto tetrahedra a-b-c-d
  static ProjectResult projectTetrahedra(const Vector3<Scalar>& a, const Vector3<Scalar>& b, const Vector3<Scalar>& c, const Vector3<Scalar>& d, const Vector3<Scalar>& p);

  /// @brief Project origin (0) onto line a-b
  static ProjectResult projectLineOrigin(const Vector3<Scalar>& a, const Vector3<Scalar>& b);

  /// @brief Project origin (0) onto triangle a-b-c
  static ProjectResult projectTriangleOrigin(const Vector3<Scalar>& a, const Vector3<Scalar>& b, const Vector3<Scalar>& c);

  /// @brief Project origin (0) onto tetrahedran a-b-c-d
  static ProjectResult projectTetrahedraOrigin(const Vector3<Scalar>& a, const Vector3<Scalar>& b, const Vector3<Scalar>& c, const Vector3<Scalar>& d);
};

using Projectf = Project<float>;
using Projectd = Project<double>;

/// @brief Triangle distance functions
template <typename Scalar>
class TriangleDistance
{
public:

  /// @brief Returns closest points between an segment pair.
  /// The first segment is P + t * A
  /// The second segment is Q + t * B
  /// X, Y are the closest points on the two segments
  /// VEC is the vector between X and Y
  static void segPoints(const Vector3<Scalar>& P, const Vector3<Scalar>& A, const Vector3<Scalar>& Q, const Vector3<Scalar>& B,
                        Vector3<Scalar>& VEC, Vector3<Scalar>& X, Vector3<Scalar>& Y);

  /// @brief Compute the closest points on two triangles given their absolute coordinate, and returns the distance between them
  /// S and T are two triangles
  /// If the triangles are disjoint, P and Q give the closet points of S and T respectively. However,
  /// if the triangles overlap, P and Q are basically a random pair of points from the triangles, not
  /// coincident points on the intersection of the triangles, as might be expected.
  static Scalar triDistance(const Vector3<Scalar> S[3], const Vector3<Scalar> T[3], Vector3<Scalar>& P, Vector3<Scalar>& Q);

  static Scalar triDistance(const Vector3<Scalar>& S1, const Vector3<Scalar>& S2, const Vector3<Scalar>& S3,
                              const Vector3<Scalar>& T1, const Vector3<Scalar>& T2, const Vector3<Scalar>& T3,
                              Vector3<Scalar>& P, Vector3<Scalar>& Q);

  /// @brief Compute the closest points on two triangles given the relative transform between them, and returns the distance between them
  /// S and T are two triangles
  /// If the triangles are disjoint, P and Q give the closet points of S and T respectively. However,
  /// if the triangles overlap, P and Q are basically a random pair of points from the triangles, not
  /// coincident points on the intersection of the triangles, as might be expected.
  /// The returned P and Q are both in the coordinate of the first triangle's coordinate
  static Scalar triDistance(const Vector3<Scalar> S[3], const Vector3<Scalar> T[3],
                              const Matrix3<Scalar>& R, const Vector3<Scalar>& Tl,
                              Vector3<Scalar>& P, Vector3<Scalar>& Q);

  static Scalar triDistance(const Vector3<Scalar> S[3], const Vector3<Scalar> T[3],
                              const Transform3<Scalar>& tf,
                              Vector3<Scalar>& P, Vector3<Scalar>& Q);

  FCL_DEPRECATED
  static Scalar triDistance(const Vector3<Scalar>& S1, const Vector3<Scalar>& S2, const Vector3<Scalar>& S3,
                              const Vector3<Scalar>& T1, const Vector3<Scalar>& T2, const Vector3<Scalar>& T3,
                              const Matrix3<Scalar>& R, const Vector3<Scalar>& Tl,
                              Vector3<Scalar>& P, Vector3<Scalar>& Q);

  static Scalar triDistance(
      const Vector3<Scalar>& S1,
      const Vector3<Scalar>& S2,
      const Vector3<Scalar>& S3,
      const Vector3<Scalar>& T1,
      const Vector3<Scalar>& T2,
      const Vector3<Scalar>& T3,
      const Transform3<Scalar>& tf,
      Vector3<Scalar>& P,
      Vector3<Scalar>& Q);

};

using TriangleDistancef = TriangleDistance<float>;
using TriangleDistanced = TriangleDistance<double>;

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
bool PolySolver<Scalar>::isZero(Scalar v)
{
  return (v < getNearZeroThreshold()) && (v > -getNearZeroThreshold());
}

//==============================================================================
template <typename Scalar>
bool PolySolver<Scalar>::cbrt(Scalar v)
{
  return powf(v, 1.0 / 3.0);
}

//==============================================================================
template <typename Scalar>
int PolySolver<Scalar>::solveLinear(Scalar c[2], Scalar s[1])
{
  if(isZero(c[1]))
    return 0;
  s[0] = - c[0] / c[1];
  return 1;
}

//==============================================================================
template <typename Scalar>
int PolySolver<Scalar>::solveQuadric(Scalar c[3], Scalar s[2])
{
  Scalar p, q, D;

  // make sure we have a d2 equation

  if(isZero(c[2]))
    return solveLinear(c, s);

  // normal for: x^2 + px + q
  p = c[1] / (2.0 * c[2]);
  q = c[0] / c[2];
  D = p * p - q;

  if(isZero(D))
  {
    // one Scalar root
    s[0] = s[1] = -p;
    return 1;
  }

  if(D < 0.0)
    // no real root
    return 0;
  else
  {
    // two real roots
    Scalar sqrt_D = sqrt(D);
    s[0] = sqrt_D - p;
    s[1] = -sqrt_D - p;
    return 2;
  }
}

//==============================================================================
template <typename Scalar>
int PolySolver<Scalar>::solveCubic(Scalar c[4], Scalar s[3])
{
  int i, num;
  Scalar sub, A, B, C, sq_A, p, q, cb_p, D;
  const Scalar ONE_OVER_THREE = 1 / 3.0;
  const Scalar PI = 3.14159265358979323846;

  // make sure we have a d2 equation
  if(isZero(c[3]))
    return solveQuadric(c, s);

  // normalize the equation:x ^ 3 + Ax ^ 2 + Bx  + C = 0
  A = c[2] / c[3];
  B = c[1] / c[3];
  C = c[0] / c[3];

  // substitute x = y - A / 3 to eliminate the quadratic term: x^3 + px + q = 0
  sq_A = A * A;
  p = (-ONE_OVER_THREE * sq_A + B) * ONE_OVER_THREE;
  q = 0.5 * (2.0 / 27.0 * A * sq_A - ONE_OVER_THREE * A * B + C);

  // use Cardano's formula
  cb_p = p * p * p;
  D = q * q + cb_p;

  if(isZero(D))
  {
    if(isZero(q))
    {
      // one triple solution
      s[0] = 0.0;
      num = 1;
    }
    else
    {
      // one single and one Scalar solution
      Scalar u = cbrt(-q);
      s[0] = 2.0 * u;
      s[1] = -u;
      num = 2;
    }
  }
  else
  {
    if(D < 0.0)
    {
      // three real solutions
      Scalar phi = ONE_OVER_THREE * acos(-q / sqrt(-cb_p));
      Scalar t = 2.0 * sqrt(-p);
      s[0] = t * cos(phi);
      s[1] = -t * cos(phi + PI / 3.0);
      s[2] = -t * cos(phi - PI / 3.0);
      num = 3;
    }
    else
    {
      // one real solution
      Scalar sqrt_D = sqrt(D);
      Scalar u = cbrt(sqrt_D + fabs(q));
      if(q > 0.0)
        s[0] = - u + p / u ;
      else
        s[0] = u - p / u;
      num = 1;
    }
  }

  // re-substitute
  sub = ONE_OVER_THREE * A;
  for(i = 0; i < num; i++)
    s[i] -= sub;
  return num;
}

//==============================================================================
template <typename Scalar>
bool Intersect<Scalar>::isZero(Scalar v)
{
  return (v < getNearZeroThreshold()) && (v > -getNearZeroThreshold());
}

//==============================================================================
/// @brief data: only used for EE, return the intersect point
template <typename Scalar>
bool Intersect<Scalar>::solveCubicWithIntervalNewton(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& c0, const Vector3<Scalar>& d0,
                                             const Vector3<Scalar>& va, const Vector3<Scalar>& vb, const Vector3<Scalar>& vc, const Vector3<Scalar>& vd,
                                             Scalar& l, Scalar& r, bool bVF, Scalar coeffs[], Vector3<Scalar>* data)
{
  Scalar v2[2]= {l*l,r*r};
  Scalar v[2]= {l,r};
  Scalar r_backup;

  unsigned char min3, min2, min1, max3, max2, max1;

  min3= *((unsigned char*)&coeffs[3]+7)>>7; max3=min3^1;
  min2= *((unsigned char*)&coeffs[2]+7)>>7; max2=min2^1;
  min1= *((unsigned char*)&coeffs[1]+7)>>7; max1=min1^1;

  // bound the cubic

  Scalar minor = coeffs[3]*v2[min3]*v[min3]+coeffs[2]*v2[min2]+coeffs[1]*v[min1]+coeffs[0];
  Scalar major = coeffs[3]*v2[max3]*v[max3]+coeffs[2]*v2[max2]+coeffs[1]*v[max1]+coeffs[0];

  if(major<0) return false;
  if(minor>0) return false;

  // starting here, the bounds have opposite values
  Scalar m = 0.5 * (r + l);

  // bound the derivative
  Scalar dminor = 3.0*coeffs[3]*v2[min3]+2.0*coeffs[2]*v[min2]+coeffs[1];
  Scalar dmajor = 3.0*coeffs[3]*v2[max3]+2.0*coeffs[2]*v[max2]+coeffs[1];

  if((dminor > 0)||(dmajor < 0)) // we can use Newton
  {
    Scalar m2 = m*m;
    Scalar fm = coeffs[3]*m2*m+coeffs[2]*m2+coeffs[1]*m+coeffs[0];
    Scalar nl = m;
    Scalar nu = m;
    if(fm>0)
    {
      nl-=(fm/dminor);
      nu-=(fm/dmajor);
    }
    else
    {
      nu-=(fm/dminor);
      nl-=(fm/dmajor);
    }

    //intersect with [l,r]

    if(nl>r) return false;
    if(nu<l) return false;
    if(nl>l)
    {
      if(nu<r) { l=nl; r=nu; m=0.5*(l+r); }
      else { l=nl; m=0.5*(l+r); }
    }
    else
    {
      if(nu<r) { r=nu; m=0.5*(l+r); }
    }
  }

  // sufficient temporal resolution, check root validity
  if((r-l)< getCcdResolution())
  {
    if(bVF)
      return checkRootValidity_VF(a0, b0, c0, d0, va, vb, vc, vd, r);
    else
      return checkRootValidity_EE(a0, b0, c0, d0, va, vb, vc, vd, r, data);
  }

  r_backup = r, r = m;
  if(solveCubicWithIntervalNewton(a0, b0, c0, d0, va, vb, vc, vd, l, r, bVF, coeffs, data))
    return true;

  l = m, r = r_backup;
  return solveCubicWithIntervalNewton(a0, b0, c0, d0, va, vb, vc, vd, l, r, bVF, coeffs, data);
}

//==============================================================================
template <typename Scalar>
bool Intersect<Scalar>::insideTriangle(const Vector3<Scalar>& a, const Vector3<Scalar>& b, const Vector3<Scalar>& c, const Vector3<Scalar>&p)
{
  Vector3<Scalar> ab = b - a;
  Vector3<Scalar> ac = c - a;
  Vector3<Scalar> n = ab.cross(ac);

  Vector3<Scalar> pa = a - p;
  Vector3<Scalar> pb = b - p;
  Vector3<Scalar> pc = c - p;

  if((pb.cross(pc)).dot(n) < -getEpsilon()) return false;
  if((pc.cross(pa)).dot(n) < -getEpsilon()) return false;
  if((pa.cross(pb)).dot(n) < -getEpsilon()) return false;

  return true;
}

//==============================================================================
template <typename Scalar>
bool Intersect<Scalar>::insideLineSegment(const Vector3<Scalar>& a, const Vector3<Scalar>& b, const Vector3<Scalar>& p)
{
  return (p - a).dot(p - b) <= 0;
}

//==============================================================================
/// @brief Calculate the line segment papb that is the shortest route between
/// two lines p1p2 and p3p4. Calculate also the values of mua and mub where
///    pa = p1 + mua (p2 - p1)
///    pb = p3 + mub (p4 - p3)
/// Return FALSE if no solution exists.
template <typename Scalar>
bool Intersect<Scalar>::linelineIntersect(const Vector3<Scalar>& p1, const Vector3<Scalar>& p2, const Vector3<Scalar>& p3, const Vector3<Scalar>& p4,
                                  Vector3<Scalar>* pa, Vector3<Scalar>* pb, Scalar* mua, Scalar* mub)
{
  Vector3<Scalar> p31 = p1 - p3;
  Vector3<Scalar> p34 = p4 - p3;
  if(fabs(p34[0]) < getEpsilon() && fabs(p34[1]) < getEpsilon() && fabs(p34[2]) < getEpsilon())
    return false;

  Vector3<Scalar> p12 = p2 - p1;
  if(fabs(p12[0]) < getEpsilon() && fabs(p12[1]) < getEpsilon() && fabs(p12[2]) < getEpsilon())
    return false;

  Scalar d3134 = p31.dot(p34);
  Scalar d3412 = p34.dot(p12);
  Scalar d3112 = p31.dot(p12);
  Scalar d3434 = p34.dot(p34);
  Scalar d1212 = p12.dot(p12);

  Scalar denom = d1212 * d3434 - d3412 * d3412;
  if(fabs(denom) < getEpsilon())
    return false;
  Scalar numer = d3134 * d3412 - d3112 * d3434;

  *mua = numer / denom;
  if(*mua < 0 || *mua > 1)
    return false;

  *mub = (d3134 + d3412 * (*mua)) / d3434;
  if(*mub < 0 || *mub > 1)
    return false;

  *pa = p1 + p12 * (*mua);
  *pb = p3 + p34 * (*mub);
  return true;
}

//==============================================================================
template <typename Scalar>
bool Intersect<Scalar>::checkRootValidity_VF(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& c0, const Vector3<Scalar>& p0,
                                     const Vector3<Scalar>& va, const Vector3<Scalar>& vb, const Vector3<Scalar>& vc, const Vector3<Scalar>& vp,
                                     Scalar t)
{
  return insideTriangle(a0 + va * t, b0 + vb * t, c0 + vc * t, p0 + vp * t);
}

//==============================================================================
template <typename Scalar>
bool Intersect<Scalar>::checkRootValidity_EE(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& c0, const Vector3<Scalar>& d0,
                                     const Vector3<Scalar>& va, const Vector3<Scalar>& vb, const Vector3<Scalar>& vc, const Vector3<Scalar>& vd,
                                     Scalar t, Vector3<Scalar>* q_i)
{
  Vector3<Scalar> a = a0 + va * t;
  Vector3<Scalar> b = b0 + vb * t;
  Vector3<Scalar> c = c0 + vc * t;
  Vector3<Scalar> d = d0 + vd * t;
  Vector3<Scalar> p1, p2;
  Scalar t_ab, t_cd;
  if(linelineIntersect(a, b, c, d, &p1, &p2, &t_ab, &t_cd))
  {
    if(q_i) *q_i = p1;
    return true;
  }

  return false;
}

//==============================================================================
template <typename Scalar>
bool Intersect<Scalar>::checkRootValidity_VE(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& p0,
                                     const Vector3<Scalar>& va, const Vector3<Scalar>& vb, const Vector3<Scalar>& vp,
                                     Scalar t)
{
  return insideLineSegment(a0 + va * t, b0 + vb * t, p0 + vp * t);
}

//==============================================================================
template <typename Scalar>
bool Intersect<Scalar>::solveSquare(Scalar a, Scalar b, Scalar c,
                            const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& c0, const Vector3<Scalar>& d0,
                            const Vector3<Scalar>& va, const Vector3<Scalar>& vb, const Vector3<Scalar>& vc, const Vector3<Scalar>& vd,
                            bool bVF,
                            Scalar* ret)
{
  Scalar discriminant = b * b - 4 * a * c;
  if(discriminant < 0)
    return false;

  Scalar sqrt_dis = sqrt(discriminant);
  Scalar r1 = (-b + sqrt_dis) / (2 * a);
  bool v1 = (r1 >= 0.0 && r1 <= 1.0) ? ((bVF) ? checkRootValidity_VF(a0, b0, c0, d0, va, vb, vc, vd, r1) : checkRootValidity_EE(a0, b0, c0, d0, va, vb, vc, vd, r1)) : false;

  Scalar r2 = (-b - sqrt_dis) / (2 * a);
  bool v2 = (r2 >= 0.0 && r2 <= 1.0) ? ((bVF) ? checkRootValidity_VF(a0, b0, c0, d0, va, vb, vc, vd, r2) : checkRootValidity_EE(a0, b0, c0, d0, va, vb, vc, vd, r2)) : false;

  if(v1 && v2)
  {
    *ret = (r1 > r2) ? r2 : r1;
    return true;
  }
  if(v1)
  {
    *ret = r1;
    return true;
  }
  if(v2)
  {
    *ret = r2;
    return true;
  }

  return false;
}

//==============================================================================
template <typename Scalar>
bool Intersect<Scalar>::solveSquare(Scalar a, Scalar b, Scalar c,
                            const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& p0,
                            const Vector3<Scalar>& va, const Vector3<Scalar>& vb, const Vector3<Scalar>& vp)
{
  if(isZero(a))
  {
    Scalar t = -c/b;
    return (t >= 0 && t <= 1) ? checkRootValidity_VE(a0, b0, p0, va, vb, vp, t) : false;
  }

  Scalar discriminant = b*b-4*a*c;
  if(discriminant < 0)
    return false;

  Scalar sqrt_dis = sqrt(discriminant);

  Scalar r1 = (-b+sqrt_dis) / (2 * a);
  bool v1 = (r1 >= 0.0 && r1 <= 1.0) ? checkRootValidity_VE(a0, b0, p0, va, vb, vp, r1) : false;
  if(v1) return true;

  Scalar r2 = (-b-sqrt_dis) / (2 * a);
  bool v2 = (r2 >= 0.0 && r2 <= 1.0) ? checkRootValidity_VE(a0, b0, p0, va, vb, vp, r2) : false;
  return v2;
}

//==============================================================================
/// @brief Compute the cubic coefficients for VF case
/// See Paper "Interactive Continuous Collision Detection between Deformable Models using Connectivity-Based Culling", Equation 1.
template <typename Scalar>
void Intersect<Scalar>::computeCubicCoeff_VF(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& c0, const Vector3<Scalar>& p0,
                                     const Vector3<Scalar>& va, const Vector3<Scalar>& vb, const Vector3<Scalar>& vc, const Vector3<Scalar>& vp,
                                     Scalar* a, Scalar* b, Scalar* c, Scalar* d)
{
  Vector3<Scalar> vavb = vb - va;
  Vector3<Scalar> vavc = vc - va;
  Vector3<Scalar> vavp = vp - va;
  Vector3<Scalar> a0b0 = b0 - a0;
  Vector3<Scalar> a0c0 = c0 - a0;
  Vector3<Scalar> a0p0 = p0 - a0;

  Vector3<Scalar> vavb_cross_vavc = vavb.cross(vavc);
  Vector3<Scalar> vavb_cross_a0c0 = vavb.cross(a0c0);
  Vector3<Scalar> a0b0_cross_vavc = a0b0.cross(vavc);
  Vector3<Scalar> a0b0_cross_a0c0 = a0b0.cross(a0c0);

  *a = vavp.dot(vavb_cross_vavc);
  *b = a0p0.dot(vavb_cross_vavc) + vavp.dot(vavb_cross_a0c0 + a0b0_cross_vavc);
  *c = vavp.dot(a0b0_cross_a0c0) + a0p0.dot(vavb_cross_a0c0 + a0b0_cross_vavc);
  *d = a0p0.dot(a0b0_cross_a0c0);
}

//==============================================================================
template <typename Scalar>
void Intersect<Scalar>::computeCubicCoeff_EE(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& c0, const Vector3<Scalar>& d0,
                                     const Vector3<Scalar>& va, const Vector3<Scalar>& vb, const Vector3<Scalar>& vc, const Vector3<Scalar>& vd,
                                     Scalar* a, Scalar* b, Scalar* c, Scalar* d)
{
  Vector3<Scalar> vavb = vb - va;
  Vector3<Scalar> vcvd = vd - vc;
  Vector3<Scalar> vavc = vc - va;
  Vector3<Scalar> c0d0 = d0 - c0;
  Vector3<Scalar> a0b0 = b0 - a0;
  Vector3<Scalar> a0c0 = c0 - a0;
  Vector3<Scalar> vavb_cross_vcvd = vavb.cross(vcvd);
  Vector3<Scalar> vavb_cross_c0d0 = vavb.cross(c0d0);
  Vector3<Scalar> a0b0_cross_vcvd = a0b0.cross(vcvd);
  Vector3<Scalar> a0b0_cross_c0d0 = a0b0.cross(c0d0);

  *a = vavc.dot(vavb_cross_vcvd);
  *b = a0c0.dot(vavb_cross_vcvd) + vavc.dot(vavb_cross_c0d0 + a0b0_cross_vcvd);
  *c = vavc.dot(a0b0_cross_c0d0) + a0c0.dot(vavb_cross_c0d0 + a0b0_cross_vcvd);
  *d = a0c0.dot(a0b0_cross_c0d0);
}

//==============================================================================
template <typename Scalar>
void Intersect<Scalar>::computeCubicCoeff_VE(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& p0,
                                     const Vector3<Scalar>& va, const Vector3<Scalar>& vb, const Vector3<Scalar>& vp,
                                     const Vector3<Scalar>& L,
                                     Scalar* a, Scalar* b, Scalar* c)
{
  Vector3<Scalar> vbva = va - vb;
  Vector3<Scalar> vbvp = vp - vb;
  Vector3<Scalar> b0a0 = a0 - b0;
  Vector3<Scalar> b0p0 = p0 - b0;

  Vector3<Scalar> L_cross_vbvp = L.cross(vbvp);
  Vector3<Scalar> L_cross_b0p0 = L.cross(b0p0);

  *a = L_cross_vbvp.dot(vbva);
  *b = L_cross_vbvp.dot(b0a0) + L_cross_b0p0.dot(vbva);
  *c = L_cross_b0p0.dot(b0a0);
}

//==============================================================================
template <typename Scalar>
bool Intersect<Scalar>::intersect_VF(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& c0, const Vector3<Scalar>& p0,
                             const Vector3<Scalar>& a1, const Vector3<Scalar>& b1, const Vector3<Scalar>& c1, const Vector3<Scalar>& p1,
                             Scalar* collision_time, Vector3<Scalar>* p_i, bool useNewton)
{
  *collision_time = 2.0;

  Vector3<Scalar> vp, va, vb, vc;
  vp = p1 - p0;
  va = a1 - a0;
  vb = b1 - b0;
  vc = c1 - c0;

  Scalar a, b, c, d;
  computeCubicCoeff_VF(a0, b0, c0, p0, va, vb, vc, vp, &a, &b, &c, &d);

  if(isZero(a) && isZero(b) && isZero(c) && isZero(d))
  {
    return false;
  }


  /// if(isZero(a))
  /// {
  ///   return solveSquare(b, c, d, a0, b0, c0, p0, va, vb, vc, vp, true, collision_time);
  /// }

  Scalar coeffs[4];
  coeffs[3] = a, coeffs[2] = b, coeffs[1] = c, coeffs[0] = d;

  if(useNewton)
  {
    Scalar l = 0;
    Scalar r = 1;

    if(solveCubicWithIntervalNewton(a0, b0, c0, p0, va, vb, vc, vp, l, r, true, coeffs))
    {
      *collision_time = 0.5 * (l + r);
    }
  }
  else
  {
    Scalar roots[3];
    int num = PolySolver<Scalar>::solveCubic(coeffs, roots);
    for(int i = 0; i < num; ++i)
    {
      Scalar r = roots[i];
      if(r < 0 || r > 1) continue;
      if(checkRootValidity_VF(a0, b0, c0, p0, va, vb, vc, vp, r))
      {
        *collision_time = r;
        break;
      }
    }
  }

  if(*collision_time > 1)
  {
    return false;
  }

  *p_i = vp * (*collision_time) + p0;
  return true;
}

//==============================================================================
template <typename Scalar>
bool Intersect<Scalar>::intersect_EE(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& c0, const Vector3<Scalar>& d0,
                             const Vector3<Scalar>& a1, const Vector3<Scalar>& b1, const Vector3<Scalar>& c1, const Vector3<Scalar>& d1,
                             Scalar* collision_time, Vector3<Scalar>* p_i, bool useNewton)
{
  *collision_time = 2.0;

  Vector3<Scalar> va, vb, vc, vd;
  va = a1 - a0;
  vb = b1 - b0;
  vc = c1 - c0;
  vd = d1 - d0;

  Scalar a, b, c, d;
  computeCubicCoeff_EE(a0, b0, c0, d0, va, vb, vc, vd, &a, &b, &c, &d);

  if(isZero(a) && isZero(b) && isZero(c) && isZero(d))
  {
    return false;
  }

  /// if(isZero(a))
  /// {
  ///   return solveSquare(b, c, d, a0, b0, c0, d0, va, vb, vc, vd, collision_time, false);
  /// }


  Scalar coeffs[4];
  coeffs[3] = a, coeffs[2] = b, coeffs[1] = c, coeffs[0] = d;

  if(useNewton)
  {
    Scalar l = 0;
    Scalar r = 1;

    if(solveCubicWithIntervalNewton(a0, b0, c0, d0, va, vb, vc, vd, l, r, false, coeffs, p_i))
    {
      *collision_time  = (l + r) * 0.5;
    }
  }
  else
  {
    Scalar roots[3];
    int num = PolySolver<Scalar>::solveCubic(coeffs, roots);
    for(int i = 0; i < num; ++i)
    {
      Scalar r = roots[i];
      if(r < 0 || r > 1) continue;

      if(checkRootValidity_EE(a0, b0, c0, d0, va, vb, vc, vd, r, p_i))
      {
        *collision_time = r;
        break;
      }
    }
  }

  if(*collision_time > 1)
  {
    return false;
  }

  return true;
}

//==============================================================================
template <typename Scalar>
bool Intersect<Scalar>::intersect_VE(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& p0,
                             const Vector3<Scalar>& a1, const Vector3<Scalar>& b1, const Vector3<Scalar>& p1,
                             const Vector3<Scalar>& L)
{
  Vector3<Scalar> va, vb, vp;
  va = a1 - a0;
  vb = b1 - b0;
  vp = p1 - p0;

  Scalar a, b, c;
  computeCubicCoeff_VE(a0, b0, p0, va, vb, vp, L, &a, &b, &c);

  if(isZero(a) && isZero(b) && isZero(c))
    return true;

  return solveSquare(a, b, c, a0, b0, p0, va, vb, vp);

}

//==============================================================================
/// @brief Prefilter for intersection, works for both VF and EE
template <typename Scalar>
bool Intersect<Scalar>::intersectPreFiltering(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& c0, const Vector3<Scalar>& d0,
                                      const Vector3<Scalar>& a1, const Vector3<Scalar>& b1, const Vector3<Scalar>& c1, const Vector3<Scalar>& d1)
{
  Vector3<Scalar> n0 = (b0 - a0).cross(c0 - a0);
  Vector3<Scalar> n1 = (b1 - a1).cross(c1 - a1);
  Vector3<Scalar> a0a1 = a1 - a0;
  Vector3<Scalar> b0b1 = b1 - b0;
  Vector3<Scalar> c0c1 = c1 - c0;
  Vector3<Scalar> delta = (b0b1 - a0a1).cross(c0c1 - a0a1);
  Vector3<Scalar> nx = (n0 + n1 - delta) * 0.5;

  Vector3<Scalar> a0d0 = d0 - a0;
  Vector3<Scalar> a1d1 = d1 - a1;

  Scalar A = n0.dot(a0d0);
  Scalar B = n1.dot(a1d1);
  Scalar C = nx.dot(a0d0);
  Scalar D = nx.dot(a1d1);
  Scalar E = n1.dot(a0d0);
  Scalar F = n0.dot(a1d1);

  if(A > 0 && B > 0 && (2*C +F) > 0 && (2*D+E) > 0)
    return false;
  if(A < 0 && B < 0 && (2*C +F) < 0 && (2*D+E) < 0)
    return false;

  return true;
}

//==============================================================================
template <typename Scalar>
bool Intersect<Scalar>::intersect_VF_filtered(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& c0, const Vector3<Scalar>& p0,
                                      const Vector3<Scalar>& a1, const Vector3<Scalar>& b1, const Vector3<Scalar>& c1, const Vector3<Scalar>& p1,
                                      Scalar* collision_time, Vector3<Scalar>* p_i, bool useNewton)
{
  if(intersectPreFiltering(a0, b0, c0, p0, a1, b1, c1, p1))
  {
    return intersect_VF(a0, b0, c0, p0, a1, b1, c1, p1, collision_time, p_i, useNewton);
  }
  else
    return false;
}

//==============================================================================
template <typename Scalar>
bool Intersect<Scalar>::intersect_EE_filtered(const Vector3<Scalar>& a0, const Vector3<Scalar>& b0, const Vector3<Scalar>& c0, const Vector3<Scalar>& d0,
                                      const Vector3<Scalar>& a1, const Vector3<Scalar>& b1, const Vector3<Scalar>& c1, const Vector3<Scalar>& d1,
                                      Scalar* collision_time, Vector3<Scalar>* p_i, bool useNewton)
{
  if(intersectPreFiltering(a0, b0, c0, d0, a1, b1, c1, d1))
  {
    return intersect_EE(a0, b0, c0, d0, a1, b1, c1, d1, collision_time, p_i, useNewton);
  }
  else
    return false;
}

//==============================================================================
template <typename Scalar>
bool Intersect<Scalar>::intersect_Triangle(const Vector3<Scalar>& P1, const Vector3<Scalar>& P2, const Vector3<Scalar>& P3,
                                   const Vector3<Scalar>& Q1, const Vector3<Scalar>& Q2, const Vector3<Scalar>& Q3,
                                   const Matrix3<Scalar>& R, const Vector3<Scalar>& T,
                                   Vector3<Scalar>* contact_points,
                                   unsigned int* num_contact_points,
                                   Scalar* penetration_depth,
                                   Vector3<Scalar>* normal)
{
  Vector3<Scalar> Q1_ = R * Q1 + T;
  Vector3<Scalar> Q2_ = R * Q2 + T;
  Vector3<Scalar> Q3_ = R * Q3 + T;

  return intersect_Triangle(P1, P2, P3, Q1_, Q2_, Q3_, contact_points, num_contact_points, penetration_depth, normal);
}

//==============================================================================
template <typename Scalar>
bool Intersect<Scalar>::intersect_Triangle(
    const Vector3<Scalar>& P1,
    const Vector3<Scalar>& P2,
    const Vector3<Scalar>& P3,
    const Vector3<Scalar>& Q1,
    const Vector3<Scalar>& Q2,
    const Vector3<Scalar>& Q3,
    const Transform3<Scalar>& tf,
    Vector3<Scalar>* contact_points,
    unsigned int* num_contact_points,
    Scalar* penetration_depth,
    Vector3<Scalar>* normal)
{
  Vector3<Scalar> Q1_ = tf * Q1;
  Vector3<Scalar> Q2_ = tf * Q2;
  Vector3<Scalar> Q3_ = tf * Q3;

  return intersect_Triangle(P1, P2, P3, Q1_, Q2_, Q3_, contact_points, num_contact_points, penetration_depth, normal);
}


#if ODE_STYLE
//==============================================================================
template <typename Scalar>
bool Intersect<Scalar>::intersect_Triangle(const Vector3<Scalar>& P1, const Vector3<Scalar>& P2, const Vector3<Scalar>& P3,
                                   const Vector3<Scalar>& Q1, const Vector3<Scalar>& Q2, const Vector3<Scalar>& Q3,
                                   Vector3<Scalar>* contact_points,
                                   unsigned int* num_contact_points,
                                   Scalar* penetration_depth,
                                   Vector3<Scalar>* normal)
{


  Vector3<Scalar> n1;
  Scalar t1;
  bool b1 = buildTrianglePlane(P1, P2, P3, &n1, &t1);
  if(!b1) return false;

  Vector3<Scalar> n2;
  Scalar t2;
  bool b2 = buildTrianglePlane(Q1, Q2, Q3, &n2, &t2);
  if(!b2) return false;

  if(sameSideOfPlane(P1, P2, P3, n2, t2))
    return false;

  if(sameSideOfPlane(Q1, Q2, Q3, n1, t1))
    return false;

  Vector3<Scalar> clipped_points1[getMaxTriangleClips()];
  unsigned int num_clipped_points1 = 0;
  Vector3<Scalar> clipped_points2[getMaxTriangleClips()];
  unsigned int num_clipped_points2 = 0;

  Vector3<Scalar> deepest_points1[getMaxTriangleClips()];
  unsigned int num_deepest_points1 = 0;
  Vector3<Scalar> deepest_points2[getMaxTriangleClips()];
  unsigned int num_deepest_points2 = 0;
  Scalar penetration_depth1 = -1, penetration_depth2 = -1;

  clipTriangleByTriangleAndEdgePlanes(Q1, Q2, Q3, P1, P2, P3, n1, t1, clipped_points2, &num_clipped_points2);

  if(num_clipped_points2 == 0)
    return false;

  computeDeepestPoints(clipped_points2, num_clipped_points2, n1, t1, &penetration_depth2, deepest_points2, &num_deepest_points2);
  if(num_deepest_points2 == 0)
    return false;

  clipTriangleByTriangleAndEdgePlanes(P1, P2, P3, Q1, Q2, Q3, n2, t2, clipped_points1, &num_clipped_points1);
  if(num_clipped_points1 == 0)
    return false;

  computeDeepestPoints(clipped_points1, num_clipped_points1, n2, t2, &penetration_depth1, deepest_points1, &num_deepest_points1);
  if(num_deepest_points1 == 0)
    return false;


  /// Return contact information
  if(contact_points && num_contact_points && penetration_depth && normal)
  {
    if(penetration_depth1 > penetration_depth2)
    {
      *num_contact_points = num_deepest_points2;
      for(unsigned int i = 0; i < num_deepest_points2; ++i)
      {
        contact_points[i] = deepest_points2[i];
      }

      *normal = n1;
      *penetration_depth = penetration_depth2;
    }
    else
    {
      *num_contact_points = num_deepest_points1;
      for(unsigned int i = 0; i < num_deepest_points1; ++i)
      {
        contact_points[i] = deepest_points1[i];
      }

      *normal = -n2;
      *penetration_depth = penetration_depth1;
    }
  }

  return true;
}
#else
//==============================================================================
template <typename Scalar>
bool Intersect<Scalar>::intersect_Triangle(const Vector3<Scalar>& P1, const Vector3<Scalar>& P2, const Vector3<Scalar>& P3,
                                   const Vector3<Scalar>& Q1, const Vector3<Scalar>& Q2, const Vector3<Scalar>& Q3,
                                   Vector3<Scalar>* contact_points,
                                   unsigned int* num_contact_points,
                                   Scalar* penetration_depth,
                                   Vector3<Scalar>* normal)
{
  Vector3<Scalar> p1 = P1 - P1;
  Vector3<Scalar> p2 = P2 - P1;
  Vector3<Scalar> p3 = P3 - P1;
  Vector3<Scalar> q1 = Q1 - P1;
  Vector3<Scalar> q2 = Q2 - P1;
  Vector3<Scalar> q3 = Q3 - P1;

  Vector3<Scalar> e1 = p2 - p1;
  Vector3<Scalar> e2 = p3 - p2;
  Vector3<Scalar> n1 = e1.cross(e2);
  if (!project6(n1, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<Scalar> f1 = q2 - q1;
  Vector3<Scalar> f2 = q3 - q2;
  Vector3<Scalar> m1 = f1.cross(f2);
  if (!project6(m1, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<Scalar> ef11 = e1.cross(f1);
  if (!project6(ef11, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<Scalar> ef12 = e1.cross(f2);
  if (!project6(ef12, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<Scalar> f3 = q1 - q3;
  Vector3<Scalar> ef13 = e1.cross(f3);
  if (!project6(ef13, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<Scalar> ef21 = e2.cross(f1);
  if (!project6(ef21, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<Scalar> ef22 = e2.cross(f2);
  if (!project6(ef22, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<Scalar> ef23 = e2.cross(f3);
  if (!project6(ef23, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<Scalar> e3 = p1 - p3;
  Vector3<Scalar> ef31 = e3.cross(f1);
  if (!project6(ef31, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<Scalar> ef32 = e3.cross(f2);
  if (!project6(ef32, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<Scalar> ef33 = e3.cross(f3);
  if (!project6(ef33, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<Scalar> g1 = e1.cross(n1);
  if (!project6(g1, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<Scalar> g2 = e2.cross(n1);
  if (!project6(g2, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<Scalar> g3 = e3.cross(n1);
  if (!project6(g3, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<Scalar> h1 = f1.cross(m1);
  if (!project6(h1, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<Scalar> h2 = f2.cross(m1);
  if (!project6(h2, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<Scalar> h3 = f3.cross(m1);
  if (!project6(h3, p1, p2, p3, q1, q2, q3)) return false;

  if(contact_points && num_contact_points && penetration_depth && normal)
  {
    Vector3<Scalar> n1, n2;
    Scalar t1, t2;
    buildTrianglePlane(P1, P2, P3, &n1, &t1);
    buildTrianglePlane(Q1, Q2, Q3, &n2, &t2);

    Vector3<Scalar> deepest_points1[3];
    unsigned int num_deepest_points1 = 0;
    Vector3<Scalar> deepest_points2[3];
    unsigned int num_deepest_points2 = 0;
    Scalar penetration_depth1, penetration_depth2;

    Vector3<Scalar> P[3] = {P1, P2, P3};
    Vector3<Scalar> Q[3] = {Q1, Q2, Q3};

    computeDeepestPoints(Q, 3, n1, t1, &penetration_depth2, deepest_points2, &num_deepest_points2);
    computeDeepestPoints(P, 3, n2, t2, &penetration_depth1, deepest_points1, &num_deepest_points1);


    if(penetration_depth1 > penetration_depth2)
    {
      *num_contact_points = std::min(num_deepest_points2, (unsigned int)2);
      for(unsigned int i = 0; i < *num_contact_points; ++i)
      {
        contact_points[i] = deepest_points2[i];
      }

      *normal = n1;
      *penetration_depth = penetration_depth2;
    }
    else
    {
      *num_contact_points = std::min(num_deepest_points1, (unsigned int)2);
      for(unsigned int i = 0; i < *num_contact_points; ++i)
      {
        contact_points[i] = deepest_points1[i];
      }

      *normal = -n2;
      *penetration_depth = penetration_depth1;
    }
  }

  return true;
}
#endif

//==============================================================================
template <typename Scalar>
void Intersect<Scalar>::computeDeepestPoints(Vector3<Scalar>* clipped_points, unsigned int num_clipped_points, const Vector3<Scalar>& n, Scalar t, Scalar* penetration_depth, Vector3<Scalar>* deepest_points, unsigned int* num_deepest_points)
{
  *num_deepest_points = 0;
  Scalar max_depth = -std::numeric_limits<Scalar>::max();
  unsigned int num_deepest_points_ = 0;
  unsigned int num_neg = 0;
  unsigned int num_pos = 0;
  unsigned int num_zero = 0;

  for(unsigned int i = 0; i < num_clipped_points; ++i)
  {
    Scalar dist = -distanceToPlane(n, t, clipped_points[i]);
    if(dist > getEpsilon()) num_pos++;
    else if(dist < -getEpsilon()) num_neg++;
    else num_zero++;
    if(dist > max_depth)
    {
      max_depth = dist;
      num_deepest_points_ = 1;
      deepest_points[num_deepest_points_ - 1] = clipped_points[i];
    }
    else if(dist + 1e-6 >= max_depth)
    {
      num_deepest_points_++;
      deepest_points[num_deepest_points_ - 1] = clipped_points[i];
    }
  }

  if(max_depth < -getEpsilon())
    num_deepest_points_ = 0;

  if(num_zero == 0 && ((num_neg == 0) || (num_pos == 0)))
    num_deepest_points_ = 0;

  *penetration_depth = max_depth;
  *num_deepest_points = num_deepest_points_;
}

//==============================================================================
template <typename Scalar>
void Intersect<Scalar>::clipTriangleByTriangleAndEdgePlanes(const Vector3<Scalar>& v1, const Vector3<Scalar>& v2, const Vector3<Scalar>& v3,
                                                    const Vector3<Scalar>& t1, const Vector3<Scalar>& t2, const Vector3<Scalar>& t3,
                                                    const Vector3<Scalar>& tn, Scalar to,
                                                    Vector3<Scalar> clipped_points[], unsigned int* num_clipped_points,
                                                    bool clip_triangle)
{
  *num_clipped_points = 0;
  Vector3<Scalar> temp_clip[getMaxTriangleClips()];
  Vector3<Scalar> temp_clip2[getMaxTriangleClips()];
  unsigned int num_temp_clip = 0;
  unsigned int num_temp_clip2 = 0;
  Vector3<Scalar> v[3] = {v1, v2, v3};

  Vector3<Scalar> plane_n;
  Scalar plane_dist;

  if(buildEdgePlane(t1, t2, tn, &plane_n, &plane_dist))
  {
    clipPolygonByPlane(v, 3, plane_n, plane_dist, temp_clip, &num_temp_clip);
    if(num_temp_clip > 0)
    {
      if(buildEdgePlane(t2, t3, tn, &plane_n, &plane_dist))
      {
        clipPolygonByPlane(temp_clip, num_temp_clip, plane_n, plane_dist, temp_clip2, &num_temp_clip2);
        if(num_temp_clip2 > 0)
        {
          if(buildEdgePlane(t3, t1, tn, &plane_n, &plane_dist))
          {
            if(clip_triangle)
            {
              num_temp_clip = 0;
              clipPolygonByPlane(temp_clip2, num_temp_clip2, plane_n, plane_dist, temp_clip, &num_temp_clip);
              if(num_temp_clip > 0)
              {
                clipPolygonByPlane(temp_clip, num_temp_clip, tn, to, clipped_points, num_clipped_points);
              }
            }
            else
            {
              clipPolygonByPlane(temp_clip2, num_temp_clip2, plane_n, plane_dist, clipped_points, num_clipped_points);
            }
          }
        }
      }
    }
  }
}

//==============================================================================
template <typename Scalar>
void Intersect<Scalar>::clipPolygonByPlane(Vector3<Scalar>* polygon_points, unsigned int num_polygon_points, const Vector3<Scalar>& n, Scalar t, Vector3<Scalar> clipped_points[], unsigned int* num_clipped_points)
{
  *num_clipped_points = 0;

  unsigned int num_clipped_points_ = 0;
  unsigned int vi;
  unsigned int prev_classify = 2;
  unsigned int classify;
  for(unsigned int i = 0; i <= num_polygon_points; ++i)
  {
    vi = (i % num_polygon_points);
    Scalar d = distanceToPlane(n, t, polygon_points[i]);
    classify = ((d > getEpsilon()) ? 1 : 0);
    if(classify == 0)
    {
      if(prev_classify == 1)
      {
        if(num_clipped_points_ < getMaxTriangleClips())
        {
          Vector3<Scalar> tmp;
          clipSegmentByPlane(polygon_points[i - 1], polygon_points[vi], n, t, &tmp);
          if(num_clipped_points_ > 0)
          {
            if((tmp - clipped_points[num_clipped_points_ - 1]).squaredNorm() > getEpsilon())
            {
              clipped_points[num_clipped_points_] = tmp;
              num_clipped_points_++;
            }
          }
          else
          {
            clipped_points[num_clipped_points_] = tmp;
            num_clipped_points_++;
          }
        }
      }

      if(num_clipped_points_ < getMaxTriangleClips() && i < num_polygon_points)
      {
        clipped_points[num_clipped_points_] = polygon_points[vi];
        num_clipped_points_++;
      }
    }
    else
    {
      if(prev_classify == 0)
      {
        if(num_clipped_points_ < getMaxTriangleClips())
        {
          Vector3<Scalar> tmp;
          clipSegmentByPlane(polygon_points[i - 1], polygon_points[vi], n, t, &tmp);
          if(num_clipped_points_ > 0)
          {
            if((tmp - clipped_points[num_clipped_points_ - 1]).squaredNorm() > getEpsilon())
            {
              clipped_points[num_clipped_points_] = tmp;
              num_clipped_points_++;
            }
          }
          else
          {
            clipped_points[num_clipped_points_] = tmp;
            num_clipped_points_++;
          }
        }
      }
    }

    prev_classify = classify;
  }

  if(num_clipped_points_ > 2)
  {
    if((clipped_points[0] - clipped_points[num_clipped_points_ - 1]).squaredNorm() < getEpsilon())
    {
      num_clipped_points_--;
    }
  }

  *num_clipped_points = num_clipped_points_;
}

//==============================================================================
template <typename Scalar>
void Intersect<Scalar>::clipSegmentByPlane(const Vector3<Scalar>& v1, const Vector3<Scalar>& v2, const Vector3<Scalar>& n, Scalar t, Vector3<Scalar>* clipped_point)
{
  Scalar dist1 = distanceToPlane(n, t, v1);
  Vector3<Scalar> tmp = v2 - v1;
  Scalar dist2 = tmp.dot(n);
  *clipped_point = tmp * (-dist1 / dist2) + v1;
}

//==============================================================================
template <typename Scalar>
Scalar Intersect<Scalar>::distanceToPlane(const Vector3<Scalar>& n, Scalar t, const Vector3<Scalar>& v)
{
  return n.dot(v) - t;
}

//==============================================================================
template <typename Scalar>
bool Intersect<Scalar>::buildTrianglePlane(const Vector3<Scalar>& v1, const Vector3<Scalar>& v2, const Vector3<Scalar>& v3, Vector3<Scalar>* n, Scalar* t)
{
  Vector3<Scalar> n_ = (v2 - v1).cross(v3 - v1);
  bool can_normalize = false;
  normalize(n_, &can_normalize);
  if(can_normalize)
  {
    *n = n_;
    *t = n_.dot(v1);
    return true;
  }

  return false;
}

//==============================================================================
template <typename Scalar>
bool Intersect<Scalar>::buildEdgePlane(const Vector3<Scalar>& v1, const Vector3<Scalar>& v2, const Vector3<Scalar>& tn, Vector3<Scalar>* n, Scalar* t)
{
  Vector3<Scalar> n_ = (v2 - v1).cross(tn);
  bool can_normalize = false;
  normalize(n_, &can_normalize);
  if(can_normalize)
  {
    *n = n_;
    *t = n_.dot(v1);
    return true;
  }

  return false;
}

//==============================================================================
template <typename Scalar>
bool Intersect<Scalar>::sameSideOfPlane(const Vector3<Scalar>& v1, const Vector3<Scalar>& v2, const Vector3<Scalar>& v3, const Vector3<Scalar>& n, Scalar t)
{
  Scalar dist1 = distanceToPlane(n, t, v1);
  Scalar dist2 = dist1 * distanceToPlane(n, t, v2);
  Scalar dist3 = dist1 * distanceToPlane(n, t, v3);
  if((dist2 > 0) && (dist3 > 0))
    return true;
  return false;
}

//==============================================================================
template <typename Scalar>
int Intersect<Scalar>::project6(const Vector3<Scalar>& ax,
                        const Vector3<Scalar>& p1, const Vector3<Scalar>& p2, const Vector3<Scalar>& p3,
                        const Vector3<Scalar>& q1, const Vector3<Scalar>& q2, const Vector3<Scalar>& q3)
{
  Scalar P1 = ax.dot(p1);
  Scalar P2 = ax.dot(p2);
  Scalar P3 = ax.dot(p3);
  Scalar Q1 = ax.dot(q1);
  Scalar Q2 = ax.dot(q2);
  Scalar Q3 = ax.dot(q3);

  Scalar mn1 = std::min(P1, std::min(P2, P3));
  Scalar mx2 = std::max(Q1, std::max(Q2, Q3));
  if(mn1 > mx2) return 0;

  Scalar mx1 = std::max(P1, std::max(P2, P3));
  Scalar mn2 = std::min(Q1, std::min(Q2, Q3));

  if(mn2 > mx1) return 0;
  return 1;
}

//==============================================================================
template <typename Scalar>
void TriangleDistance<Scalar>::segPoints(const Vector3<Scalar>& P, const Vector3<Scalar>& A, const Vector3<Scalar>& Q, const Vector3<Scalar>& B,
                                 Vector3<Scalar>& VEC, Vector3<Scalar>& X, Vector3<Scalar>& Y)
{
  Vector3<Scalar> T;
  Scalar A_dot_A, B_dot_B, A_dot_B, A_dot_T, B_dot_T;
  Vector3<Scalar> TMP;

  T = Q - P;
  A_dot_A = A.dot(A);
  B_dot_B = B.dot(B);
  A_dot_B = A.dot(B);
  A_dot_T = A.dot(T);
  B_dot_T = B.dot(T);

  // t parameterizes ray P,A
  // u parameterizes ray Q,B

  Scalar t, u;

  // compute t for the closest point on ray P,A to
  // ray Q,B

  Scalar denom = A_dot_A*B_dot_B - A_dot_B*A_dot_B;

  t = (A_dot_T*B_dot_B - B_dot_T*A_dot_B) / denom;

  // clamp result so t is on the segment P,A

  if((t < 0) || std::isnan(t)) t = 0; else if(t > 1) t = 1;

  // find u for point on ray Q,B closest to point at t

  u = (t*A_dot_B - B_dot_T) / B_dot_B;

  // if u is on segment Q,B, t and u correspond to
  // closest points, otherwise, clamp u, recompute and
  // clamp t

  if((u <= 0) || std::isnan(u))
  {
    Y = Q;

    t = A_dot_T / A_dot_A;

    if((t <= 0) || std::isnan(t))
    {
      X = P;
      VEC = Q - P;
    }
    else if(t >= 1)
    {
      X = P + A;
      VEC = Q - X;
    }
    else
    {
      X = P + A * t;
      TMP = T.cross(A);
      VEC = A.cross(TMP);
    }
  }
  else if (u >= 1)
  {
    Y = Q + B;

    t = (A_dot_B + A_dot_T) / A_dot_A;

    if((t <= 0) || std::isnan(t))
    {
      X = P;
      VEC = Y - P;
    }
    else if(t >= 1)
    {
      X = P + A;
      VEC = Y - X;
    }
    else
    {
      X = P + A * t;
      T = Y - P;
      TMP = T.cross(A);
      VEC= A.cross(TMP);
    }
  }
  else
  {
    Y = Q + B * u;

    if((t <= 0) || std::isnan(t))
    {
      X = P;
      TMP = T.cross(B);
      VEC = B.cross(TMP);
    }
    else if(t >= 1)
    {
      X = P + A;
      T = Q - X;
      TMP = T.cross(B);
      VEC = B.cross(TMP);
    }
    else
    {
      X = P + A * t;
      VEC = A.cross(B);
      if(VEC.dot(T) < 0)
      {
        VEC = VEC * (-1);
      }
    }
  }
}

//==============================================================================
template <typename Scalar>
Scalar TriangleDistance<Scalar>::triDistance(const Vector3<Scalar> S[3], const Vector3<Scalar> T[3], Vector3<Scalar>& P, Vector3<Scalar>& Q)
{
  // Compute vectors along the 6 sides

  Vector3<Scalar> Sv[3];
  Vector3<Scalar> Tv[3];
  Vector3<Scalar> VEC;

  Sv[0] = S[1] - S[0];
  Sv[1] = S[2] - S[1];
  Sv[2] = S[0] - S[2];

  Tv[0] = T[1] - T[0];
  Tv[1] = T[2] - T[1];
  Tv[2] = T[0] - T[2];

  // For each edge pair, the vector connecting the closest points
  // of the edges defines a slab (parallel planes at head and tail
  // enclose the slab). If we can show that the off-edge vertex of
  // each triangle is outside of the slab, then the closest points
  // of the edges are the closest points for the triangles.
  // Even if these tests fail, it may be helpful to know the closest
  // points found, and whether the triangles were shown disjoint

  Vector3<Scalar> V;
  Vector3<Scalar> Z;
  Vector3<Scalar> minP = Vector3<Scalar>::Zero();
  Vector3<Scalar> minQ = Vector3<Scalar>::Zero();
  Scalar mindd;
  int shown_disjoint = 0;

  mindd = (S[0] - T[0]).squaredNorm() + 1; // Set first minimum safely high

  for(int i = 0; i < 3; ++i)
  {
    for(int j = 0; j < 3; ++j)
    {
      // Find closest points on edges i & j, plus the
      // vector (and distance squared) between these points
      segPoints(S[i], Sv[i], T[j], Tv[j], VEC, P, Q);

      V = Q - P;
      Scalar dd = V.dot(V);

      // Verify this closest point pair only if the distance
      // squared is less than the minimum found thus far.

      if(dd <= mindd)
      {
        minP = P;
        minQ = Q;
        mindd = dd;

        Z = S[(i+2)%3] - P;
        Scalar a = Z.dot(VEC);
        Z = T[(j+2)%3] - Q;
        Scalar b = Z.dot(VEC);

        if((a <= 0) && (b >= 0)) return sqrt(dd);

        Scalar p = V.dot(VEC);

        if(a < 0) a = 0;
        if(b > 0) b = 0;
        if((p - a + b) > 0) shown_disjoint = 1;
      }
    }
  }

  // No edge pairs contained the closest points.
  // either:
  // 1. one of the closest points is a vertex, and the
  //    other point is interior to a face.
  // 2. the triangles are overlapping.
  // 3. an edge of one triangle is parallel to the other's face. If
  //    cases 1 and 2 are not true, then the closest points from the 9
  //    edge pairs checks above can be taken as closest points for the
  //    triangles.
  // 4. possibly, the triangles were degenerate.  When the
  //    triangle points are nearly colinear or coincident, one
  //    of above tests might fail even though the edges tested
  //    contain the closest points.

  // First check for case 1

  Vector3<Scalar> Sn;
  Scalar Snl;

  Sn = Sv[0].cross(Sv[1]); // Compute normal to S triangle
  Snl = Sn.dot(Sn);        // Compute square of length of normal

  // If cross product is long enough,

  if(Snl > 1e-15)
  {
    // Get projection lengths of T points

    Vector3<Scalar> Tp;

    V = S[0] - T[0];
    Tp[0] = V.dot(Sn);

    V = S[0] - T[1];
    Tp[1] = V.dot(Sn);

    V = S[0] - T[2];
    Tp[2] = V.dot(Sn);

    // If Sn is a separating direction,
    // find point with smallest projection

    int point = -1;
    if((Tp[0] > 0) && (Tp[1] > 0) && (Tp[2] > 0))
    {
      if(Tp[0] < Tp[1]) point = 0; else point = 1;
      if(Tp[2] < Tp[point]) point = 2;
    }
    else if((Tp[0] < 0) && (Tp[1] < 0) && (Tp[2] < 0))
    {
      if(Tp[0] > Tp[1]) point = 0; else point = 1;
      if(Tp[2] > Tp[point]) point = 2;
    }

    // If Sn is a separating direction,

    if(point >= 0)
    {
      shown_disjoint = 1;

      // Test whether the point found, when projected onto the
      // other triangle, lies within the face.

      V = T[point] - S[0];
      Z = Sn.cross(Sv[0]);
      if(V.dot(Z) > 0)
      {
        V = T[point] - S[1];
        Z = Sn.cross(Sv[1]);
        if(V.dot(Z) > 0)
        {
          V = T[point] - S[2];
          Z = Sn.cross(Sv[2]);
          if(V.dot(Z) > 0)
          {
            // T[point] passed the test - it's a closest point for
            // the T triangle; the other point is on the face of S
            P = T[point] + Sn * (Tp[point] / Snl);
            Q = T[point];
            return (P - Q).norm();
          }
        }
      }
    }
  }

  Vector3<Scalar> Tn;
  Scalar Tnl;

  Tn = Tv[0].cross(Tv[1]);
  Tnl = Tn.dot(Tn);

  if(Tnl > 1e-15)
  {
    Vector3<Scalar> Sp;

    V = T[0] - S[0];
    Sp[0] = V.dot(Tn);

    V = T[0] - S[1];
    Sp[1] = V.dot(Tn);

    V = T[0] - S[2];
    Sp[2] = V.dot(Tn);

    int point = -1;
    if((Sp[0] > 0) && (Sp[1] > 0) && (Sp[2] > 0))
    {
      if(Sp[0] < Sp[1]) point = 0; else point = 1;
      if(Sp[2] < Sp[point]) point = 2;
    }
    else if((Sp[0] < 0) && (Sp[1] < 0) && (Sp[2] < 0))
    {
      if(Sp[0] > Sp[1]) point = 0; else point = 1;
      if(Sp[2] > Sp[point]) point = 2;
    }

    if(point >= 0)
    {
      shown_disjoint = 1;

      V = S[point] - T[0];
      Z = Tn.cross(Tv[0]);
      if(V.dot(Z) > 0)
      {
        V = S[point] - T[1];
        Z = Tn.cross(Tv[1]);
        if(V.dot(Z) > 0)
        {
          V = S[point] - T[2];
          Z = Tn.cross(Tv[2]);
          if(V.dot(Z) > 0)
          {
            P = S[point];
            Q = S[point] + Tn * (Sp[point] / Tnl);
            return (P - Q).norm();
          }
        }
      }
    }
  }

  // Case 1 can't be shown.
  // If one of these tests showed the triangles disjoint,
  // we assume case 3 or 4, otherwise we conclude case 2,
  // that the triangles overlap.

  if(shown_disjoint)
  {
    P = minP;
    Q = minQ;
    return sqrt(mindd);
  }
  else return 0;
}

//==============================================================================
template <typename Scalar>
Scalar TriangleDistance<Scalar>::triDistance(const Vector3<Scalar>& S1, const Vector3<Scalar>& S2, const Vector3<Scalar>& S3,
                                       const Vector3<Scalar>& T1, const Vector3<Scalar>& T2, const Vector3<Scalar>& T3,
                                       Vector3<Scalar>& P, Vector3<Scalar>& Q)
{
  Vector3<Scalar> S[3];
  Vector3<Scalar> T[3];
  S[0] = S1; S[1] = S2; S[2] = S3;
  T[0] = T1; T[1] = T2; T[2] = T3;

  return triDistance(S, T, P, Q);
}

//==============================================================================
template <typename Scalar>
Scalar TriangleDistance<Scalar>::triDistance(const Vector3<Scalar> S[3], const Vector3<Scalar> T[3],
                                       const Matrix3<Scalar>& R, const Vector3<Scalar>& Tl,
                                       Vector3<Scalar>& P, Vector3<Scalar>& Q)
{
  Vector3<Scalar> T_transformed[3];
  T_transformed[0] = R * T[0] + Tl;
  T_transformed[1] = R * T[1] + Tl;
  T_transformed[2] = R * T[2] + Tl;

  return triDistance(S, T_transformed, P, Q);
}

//==============================================================================
template <typename Scalar>
Scalar TriangleDistance<Scalar>::triDistance(const Vector3<Scalar> S[3], const Vector3<Scalar> T[3],
                                       const Transform3<Scalar>& tf,
                                       Vector3<Scalar>& P, Vector3<Scalar>& Q)
{
  Vector3<Scalar> T_transformed[3];
  T_transformed[0] = tf * T[0];
  T_transformed[1] = tf * T[1];
  T_transformed[2] = tf * T[2];

  return triDistance(S, T_transformed, P, Q);
}

//==============================================================================
template <typename Scalar>
Scalar TriangleDistance<Scalar>::triDistance(const Vector3<Scalar>& S1, const Vector3<Scalar>& S2, const Vector3<Scalar>& S3,
                                       const Vector3<Scalar>& T1, const Vector3<Scalar>& T2, const Vector3<Scalar>& T3,
                                       const Matrix3<Scalar>& R, const Vector3<Scalar>& Tl,
                                       Vector3<Scalar>& P, Vector3<Scalar>& Q)
{
  Vector3<Scalar> T1_transformed = R * T1 + Tl;
  Vector3<Scalar> T2_transformed = R * T2 + Tl;
  Vector3<Scalar> T3_transformed = R * T3 + Tl;
  return triDistance(S1, S2, S3, T1_transformed, T2_transformed, T3_transformed, P, Q);
}

//==============================================================================
template <typename Scalar>
Scalar TriangleDistance<Scalar>::triDistance(const Vector3<Scalar>& S1, const Vector3<Scalar>& S2, const Vector3<Scalar>& S3,
                                       const Vector3<Scalar>& T1, const Vector3<Scalar>& T2, const Vector3<Scalar>& T3,
                                       const Transform3<Scalar>& tf,
                                       Vector3<Scalar>& P, Vector3<Scalar>& Q)
{
  Vector3<Scalar> T1_transformed = tf * T1;
  Vector3<Scalar> T2_transformed = tf * T2;
  Vector3<Scalar> T3_transformed = tf * T3;
  return triDistance(S1, S2, S3, T1_transformed, T2_transformed, T3_transformed, P, Q);
}

//==============================================================================
template <typename Scalar>
typename Project<Scalar>::ProjectResult Project<Scalar>::projectLine(const Vector3<Scalar>& a, const Vector3<Scalar>& b, const Vector3<Scalar>& p)
{
  ProjectResult res;

  const Vector3<Scalar> d = b - a;
  const Scalar l = d.squaredNorm();

  if(l > 0)
  {
    const Scalar t = (p - a).dot(d);
    res.parameterization[1] = (t >= l) ? 1 : ((t <= 0) ? 0 : (t / l));
    res.parameterization[0] = 1 - res.parameterization[1];
    if(t >= l) { res.sqr_distance = (p - b).squaredNorm(); res.encode = 2; /* 0x10 */ }
    else if(t <= 0) { res.sqr_distance = (p - a).squaredNorm(); res.encode = 1; /* 0x01 */ }
    else { res.sqr_distance = (a + d * res.parameterization[1] - p).squaredNorm(); res.encode = 3; /* 0x00 */ }
  }

  return res;
}

//==============================================================================
template <typename Scalar>
typename Project<Scalar>::ProjectResult Project<Scalar>::projectTriangle(const Vector3<Scalar>& a, const Vector3<Scalar>& b, const Vector3<Scalar>& c, const Vector3<Scalar>& p)
{
  ProjectResult res;

  static const size_t nexti[3] = {1, 2, 0};
  const Vector3<Scalar>* vt[] = {&a, &b, &c};
  const Vector3<Scalar> dl[] = {a - b, b - c, c - a};
  const Vector3<Scalar>& n = dl[0].cross(dl[1]);
  const Scalar l = n.squaredNorm();

  if(l > 0)
  {
    Scalar mindist = -1;
    for(size_t i = 0; i < 3; ++i)
    {
      if((*vt[i] - p).dot(dl[i].cross(n)) > 0) // origin is to the outside part of the triangle edge, then the optimal can only be on the edge
      {
        size_t j = nexti[i];
        ProjectResult res_line = projectLine(*vt[i], *vt[j], p);

        if(mindist < 0 || res_line.sqr_distance < mindist)
        {
          mindist = res_line.sqr_distance;
          res.encode = static_cast<size_t>(((res_line.encode&1)?1<<i:0) + ((res_line.encode&2)?1<<j:0));
          res.parameterization[i] = res_line.parameterization[0];
          res.parameterization[j] = res_line.parameterization[1];
          res.parameterization[nexti[j]] = 0;
        }
      }
    }

    if(mindist < 0) // the origin project is within the triangle
    {
      Scalar d = (a - p).dot(n);
      Scalar s = sqrt(l);
      Vector3<Scalar> p_to_project = n * (d / l);
      mindist = p_to_project.squaredNorm();
      res.encode = 7; // m = 0x111
      res.parameterization[0] = dl[1].cross(b - p -p_to_project).norm() / s;
      res.parameterization[1] = dl[2].cross(c - p -p_to_project).norm() / s;
      res.parameterization[2] = 1 - res.parameterization[0] - res.parameterization[1];
    }

    res.sqr_distance = mindist;
  }

  return  res;

}

//==============================================================================
template <typename Scalar>
typename Project<Scalar>::ProjectResult Project<Scalar>::projectTetrahedra(const Vector3<Scalar>& a, const Vector3<Scalar>& b, const Vector3<Scalar>& c, const Vector3<Scalar>& d, const Vector3<Scalar>& p)
{
  ProjectResult res;

  static const size_t nexti[] = {1, 2, 0};
  const Vector3<Scalar>* vt[] = {&a, &b, &c, &d};
  const Vector3<Scalar> dl[3] = {a-d, b-d, c-d};
  Scalar vl = triple(dl[0], dl[1], dl[2]);
  bool ng = (vl * (a-p).dot((b-c).cross(a-b))) <= 0;
  if(ng && std::abs(vl) > 0) // abs(vl) == 0, the tetrahedron is degenerated; if ng is false, then the last vertex in the tetrahedron does not grow toward the origin (in fact origin is on the other side of the abc face)
  {
    Scalar mindist = -1;

    for(size_t i = 0; i < 3; ++i)
    {
      size_t j = nexti[i];
      Scalar s = vl * (d-p).dot(dl[i].cross(dl[j]));
      if(s > 0) // the origin is to the outside part of a triangle face, then the optimal can only be on the triangle face
      {
        ProjectResult res_triangle = projectTriangle(*vt[i], *vt[j], d, p);
        if(mindist < 0 || res_triangle.sqr_distance < mindist)
        {
          mindist = res_triangle.sqr_distance;
          res.encode = static_cast<size_t>( (res_triangle.encode&1?1<<i:0) + (res_triangle.encode&2?1<<j:0) + (res_triangle.encode&4?8:0) );
          res.parameterization[i] = res_triangle.parameterization[0];
          res.parameterization[j] = res_triangle.parameterization[1];
          res.parameterization[nexti[j]] = 0;
          res.parameterization[3] = res_triangle.parameterization[2];
        }
      }
    }

    if(mindist < 0)
    {
      mindist = 0;
      res.encode = 15;
      res.parameterization[0] = triple(c - p, b - p, d - p) / vl;
      res.parameterization[1] = triple(a - p, c - p, d - p) / vl;
      res.parameterization[2] = triple(b - p, a - p, d - p) / vl;
      res.parameterization[3] = 1 - (res.parameterization[0] + res.parameterization[1] + res.parameterization[2]);
    }

    res.sqr_distance = mindist;
  }
  else if(!ng)
  {
    res = projectTriangle(a, b, c, p);
    res.parameterization[3] = 0;
  }
  return res;
}

//==============================================================================
template <typename Scalar>
typename Project<Scalar>::ProjectResult Project<Scalar>::projectLineOrigin(const Vector3<Scalar>& a, const Vector3<Scalar>& b)
{
  ProjectResult res;

  const Vector3<Scalar> d = b - a;
  const Scalar l = d.squaredNorm();

  if(l > 0)
  {
    const Scalar t = - a.dot(d);
    res.parameterization[1] = (t >= l) ? 1 : ((t <= 0) ? 0 : (t / l));
    res.parameterization[0] = 1 - res.parameterization[1];
    if(t >= l) { res.sqr_distance = b.squaredNorm(); res.encode = 2; /* 0x10 */ }
    else if(t <= 0) { res.sqr_distance = a.squaredNorm(); res.encode = 1; /* 0x01 */ }
    else { res.sqr_distance = (a + d * res.parameterization[1]).squaredNorm(); res.encode = 3; /* 0x00 */ }
  }

  return res;
}

//==============================================================================
template <typename Scalar>
typename Project<Scalar>::ProjectResult Project<Scalar>::projectTriangleOrigin(const Vector3<Scalar>& a, const Vector3<Scalar>& b, const Vector3<Scalar>& c)
{
  ProjectResult res;

  static const size_t nexti[3] = {1, 2, 0};
  const Vector3<Scalar>* vt[] = {&a, &b, &c};
  const Vector3<Scalar> dl[] = {a - b, b - c, c - a};
  const Vector3<Scalar>& n = dl[0].cross(dl[1]);
  const Scalar l = n.squaredNorm();

  if(l > 0)
  {
    Scalar mindist = -1;
    for(size_t i = 0; i < 3; ++i)
    {
      if(vt[i]->dot(dl[i].cross(n)) > 0) // origin is to the outside part of the triangle edge, then the optimal can only be on the edge
      {
        size_t j = nexti[i];
        ProjectResult res_line = projectLineOrigin(*vt[i], *vt[j]);

        if(mindist < 0 || res_line.sqr_distance < mindist)
        {
          mindist = res_line.sqr_distance;
          res.encode = static_cast<size_t>(((res_line.encode&1)?1<<i:0) + ((res_line.encode&2)?1<<j:0));
          res.parameterization[i] = res_line.parameterization[0];
          res.parameterization[j] = res_line.parameterization[1];
          res.parameterization[nexti[j]] = 0;
        }
      }
    }

    if(mindist < 0) // the origin project is within the triangle
    {
      Scalar d = a.dot(n);
      Scalar s = sqrt(l);
      Vector3<Scalar> o_to_project = n * (d / l);
      mindist = o_to_project.squaredNorm();
      res.encode = 7; // m = 0x111
      res.parameterization[0] = dl[1].cross(b - o_to_project).norm() / s;
      res.parameterization[1] = dl[2].cross(c - o_to_project).norm() / s;
      res.parameterization[2] = 1 - res.parameterization[0] - res.parameterization[1];
    }

    res.sqr_distance = mindist;
  }

  return  res;

}

//==============================================================================
template <typename Scalar>
typename Project<Scalar>::ProjectResult Project<Scalar>::projectTetrahedraOrigin(const Vector3<Scalar>& a, const Vector3<Scalar>& b, const Vector3<Scalar>& c, const Vector3<Scalar>& d)
{
  ProjectResult res;

  static const size_t nexti[] = {1, 2, 0};
  const Vector3<Scalar>* vt[] = {&a, &b, &c, &d};
  const Vector3<Scalar> dl[3] = {a-d, b-d, c-d};
  Scalar vl = triple(dl[0], dl[1], dl[2]);
  bool ng = (vl * a.dot((b-c).cross(a-b))) <= 0;
  if(ng && std::abs(vl) > 0) // abs(vl) == 0, the tetrahedron is degenerated; if ng is false, then the last vertex in the tetrahedron does not grow toward the origin (in fact origin is on the other side of the abc face)
  {
    Scalar mindist = -1;

    for(size_t i = 0; i < 3; ++i)
    {
      size_t j = nexti[i];
      Scalar s = vl * d.dot(dl[i].cross(dl[j]));
      if(s > 0) // the origin is to the outside part of a triangle face, then the optimal can only be on the triangle face
      {
        ProjectResult res_triangle = projectTriangleOrigin(*vt[i], *vt[j], d);
        if(mindist < 0 || res_triangle.sqr_distance < mindist)
        {
          mindist = res_triangle.sqr_distance;
          res.encode = static_cast<size_t>( (res_triangle.encode&1?1<<i:0) + (res_triangle.encode&2?1<<j:0) + (res_triangle.encode&4?8:0) );
          res.parameterization[i] = res_triangle.parameterization[0];
          res.parameterization[j] = res_triangle.parameterization[1];
          res.parameterization[nexti[j]] = 0;
          res.parameterization[3] = res_triangle.parameterization[2];
        }
      }
    }

    if(mindist < 0)
    {
      mindist = 0;
      res.encode = 15;
      res.parameterization[0] = triple(c, b, d) / vl;
      res.parameterization[1] = triple(a, c, d) / vl;
      res.parameterization[2] = triple(b, a, d) / vl;
      res.parameterization[3] = 1 - (res.parameterization[0] + res.parameterization[1] + res.parameterization[2]);
    }

    res.sqr_distance = mindist;
  }
  else if(!ng)
  {
    res = projectTriangleOrigin(a, b, c);
    res.parameterization[3] = 0;
  }
  return res;
}

} // namespace fcl

#endif

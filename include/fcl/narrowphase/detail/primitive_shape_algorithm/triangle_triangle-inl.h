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

#ifndef TRIANGLE_TRIANGLE_INL_H
#define TRIANGLE_TRIANGLE_INL_H

#include "fcl/math/geometry.h"

#include <algorithm>
#include <set>
#include <utility>

namespace fcl
{

namespace detail
{

/* sort so that a<=b */
#define SORT(a,b) if ((a) > (b)) { std::swap((a), (b)); }

//==============================================================================
template <typename S>
constexpr S getNearZeroThreshold()
{
  return 1e-7;
}

//==============================================================================
template <typename S, bool ROBUST>
bool isZero(const S& v)
{
  if (ROBUST)
  {
    return (v < getNearZeroThreshold<S>()) && (v > -getNearZeroThreshold<S>());
  }
  else
  {
    return 0 == v;
  }
}


//==============================================================================
template<int DIM,
         typename PointT>
struct vector_less_than
{
  bool operator()(const PointT& a,
                  const PointT& b) const
  {
    for(size_t i=0; i<DIM; ++i)
    {
      if(a[i]<b[i]) { return true;  }
      if(a[i]>b[i]) { return false; }
    }
    return false;
  }
};

//==============================================================================
template <typename S>
inline Vector3<S> computeNormal(const Vector3<S>& a, const Vector3<S>& b, const Vector3<S>& c)
{
  return (b-a).cross(c-a).normalized();
}

//==============================================================================
template <typename S>
inline S projectToLine(const Vector3<S>& lineVec, const Vector3<S>& lineOrig, const Vector3<S>& freePt)
{
  return lineVec.dot(freePt - lineOrig);
}

//==============================================================================
template <typename S>
inline void solveCramer(const S a1, const S b1, const S c1, const S a2, const S b2, const S c2, S& x, S& y)
{
  S det = a1*b2 - b1*a2;
  x = (c1*b2 - b1*c2) / det;
  y = (a1*c2 - c1*a2) / det;
}

//==============================================================================
template <typename S, bool ROBUST>
inline bool areCoplanar(const Vector3<S>& n1, const S t1, const Vector3<S>& n2, const S t2)
{
  assert(fabs(n1.norm()-1.0) < getNearZeroThreshold<S>());
  assert(fabs(n2.norm()-1.0) < getNearZeroThreshold<S>());
  if (ROBUST)
  {
    return ((n1 - n2).norm() < getNearZeroThreshold<S>() && isZero<S, ROBUST>(t1 - t2))
            || ((n1 + n2).norm() < getNearZeroThreshold<S>() && isZero<S, ROBUST>(t1 + t2));
  }
  else
  {
    return ((n1 == n2) && (t1 == t2))
            || ((n1 == -n2) && (t1 == -t2));
  }
}

//==============================================================================
template <typename S>
inline S cross2(const Vector2<S>& u, const Vector2<S>& v)
{
  return (u.x() * v.y()) - (u.y() * v.x());
}

//==============================================================================
template <typename S>
inline bool isInSegment(const Vector2<S>& p0, const Vector2<S>& p1, const Vector2<S>& q)
{
  if (p0.x() == p1.x())
  {
    // Vertical line, only compare vertical components
    return ((p0.y() <= q.y() && q.y() <= p1.y())
            || (p1.y() <= q.y() && q.y() <= p0.y()));
  }
  else
  {
    // Only compare vertical components
    return ((p0.x() <= q.x() && q.x() <= p1.x())
            || (p1.x() <= q.x() && q.x() <= p0.x()));
  }
}

//==============================================================================
// from http://geomalgorithms.com/a05-_intersect-1.html
template <typename S, bool ROBUST>
int lineSegmentIntersect(const Vector2<S>& p0, const Vector2<S>& p1,
                         const Vector2<S>& q0, const Vector2<S>& q1,
                         Vector2<S>& r0, Vector2<S>& r1)
{
  // Compute line vectors
  const Vector2<S> u = p1 - p0;
  const Vector2<S> v = q1 - q0;
  const Vector2<S> w = p0 - q0;

  // Check whether segments are parallel
  const S d = cross2(u, v);
  if (isZero<S, ROBUST>(d))
  {
    if (cross2(u, w) != 0 || cross2(v,w) != 0)
    {
      // Vectors are parallel but not collinear
      return 0;
    }

    // Handle degenerate cases (one or both segments are actually points)
    const S uLen = u.squaredNorm();
    const S vLen = v.squaredNorm();
    if (0 == uLen && 0 == vLen)
    {
      if (p0 == q0)
      {
        // All 4 points are identical
        r0 = p0;
        return 1;
      }
      // "Segments" are 2 distinct points
      return 0;
    }
    else if (0 == uLen)
    {
      if (isInSegment(q0, q1, p0))
      {
        // P is a single point in q0q1
        r0 = p0;
        return 1;
      }
      return 0;
    }
    else if (0 == vLen)
    {
      if (isInSegment(p0, p1, q0))
      {
        // Q is a single point in p0p1
        r0 = q0;
        return 1;
      }
      return 0;
    }

    // Non-degenerate, collinear segments
    Vector2<S> w2 = p1-q0;
    S t0, t1;
    if (v.x() != static_cast<S>(0))
    {
      t0 = w.x() / v.x();
      t1 = w2.x() / v.x();
    }
    else
    {
      t0 = w.y() / v.y();
      t1 = w2.y() / v.y();
    }

    // Make t0 the smaller length
    if (t0 > t1)
    {
      std::swap(t0, t1);
    }

    // Check for no overlap
    if (t0 > 1 || t1 < 0)
    {
      return 0;
    }

    // Clip to [0, 1]
    t0 = std::max(t0, static_cast<S>(0));
    t1 = std::min(t1, static_cast<S>(1));

    // Check for endpoints touching
    if (t0 == t1)
    {
      r0 = q0 + t0*v;
      return 1;
    }

    // Subsegment
    r0 = q0 + t0*v;
    r1 = q0 + t1*v;
    return 2;
  }

  S s0 = cross2(v,w) / d;
  if (s0 < 0 || s0 > 1)
  {
    return 0;
  }

  S s1 = cross2(u,w) / d;
  if (s1 < 0 || s1 > 1)
  {
    return 0;
  }

  // Single intersection point
  r0 = p0 + s0 * u;
  return 1;
}

//==============================================================================
// From http://blackpawn.com/texts/pointinpoly/default.html
template <typename S>
bool pointInTriangle(const Vector2<S>& a, const Vector2<S>& b, const Vector2<S>& c, const Vector2<S>& p)
{
  Vector2<S> v0 = c - a;
  Vector2<S> v1 = b - a;
  Vector2<S> v2 = p - a;

  S dot00 = v0.dot(v0);
  S dot01 = v0.dot(v1);
  S dot02 = v0.dot(v2);
  S dot11 = v1.dot(v1);
  S dot12 = v1.dot(v2);

  // Compute barycentric coordinates
  S denom = dot00 * dot11 - dot01 * dot01;
  S u = (dot11 * dot02 - dot01 * dot12) / denom;
  S v = (dot00 * dot12 - dot01 * dot02) / denom;

  return (u >= 0) && (v >= 0) && (u + v < 1);
}

//==============================================================================
template <typename S, bool ROBUST>
bool overlapCoplanarTriangles(
    const Vector3<S>& p1, const Vector3<S>& p2, const Vector3<S>& p3,
    const Vector3<S>& q1, const Vector3<S>& q2, const Vector3<S>& q3,
    const Vector3<S>& n1, const S t1,
    Vector3<S>* contact_points,
    unsigned int* num_contact_points,
    S* penetration_depth,
    Vector3<S>* normal)
{
  // Compute an orthonormal basis in the plane

  // Find a vector not equal to the normal
  Vector3<S> v = (n1.x() < 0.8 ? Vector3<S>::UnitX() : Vector3<S>::UnitY());
  Vector3<S> T1 = n1.cross(v).normalized();  // Tangent vector
  Vector3<S> T2 = n1.cross(T1).normalized(); // Cotangent vector

  // Project into 2D plane
  Vector2<S> p2D[3];
  Vector2<S> q2D[3];

  p2D[0] = Vector2<S>(T1.dot(p1), T2.dot(p1));
  p2D[1] = Vector2<S>(T1.dot(p2), T2.dot(p2));
  p2D[2] = Vector2<S>(T1.dot(p3), T2.dot(p3));
  q2D[0] = Vector2<S>(T1.dot(q1), T2.dot(q1));
  q2D[1] = Vector2<S>(T1.dot(q2), T2.dot(q2));
  q2D[2] = Vector2<S>(T1.dot(q3), T2.dot(q3));

  // Create a set to remove duplicate vertices (e.g. if both triangles are identical)
  std::set<Vector2<S>, vector_less_than<2, Vector2<S>>, Eigen::aligned_allocator<Vector2<S>>> vertices;

  // Test line segment pairs for intersection
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      Vector2<S> r1, r2; // Resultant intersections
      int num = lineSegmentIntersect<S, ROBUST>(p2D[i], p2D[(i+1)%3], q2D[j], q2D[(j+1)%3], r1, r2);
      if (num > 0)
      {
        vertices.insert(r1);
        if (num == 2)
        {
          vertices.insert(r2);
        }
      }
    }
  }

  // Test all vertices in either triangle
  for (int i = 0; i < 3; ++i)
  {
    if (pointInTriangle(p2D[0], p2D[1], p2D[2], q2D[i]))
    {
      vertices.insert(q2D[i]);
    }
    if (pointInTriangle(q2D[0], q2D[1], q2D[2], p2D[i]))
    {
      vertices.insert(p2D[i]);
    }
  }

  int count = 0;
  for (const Vector2<S>& vtx : vertices)
  {
    contact_points[count++] = (n1*t1)+(T1*vtx[0])+(T2*vtx[1]);
  }

  *normal = n1;
  *penetration_depth = 0;
  *num_contact_points = count;

  return true;
}

//==============================================================================
template <typename S, bool ROBUST=true>
void intersectTriangleLine(const std::array<std::reference_wrapper<const Vector3<S>>, 3>& tri,
                           const std::array<S, 3>& sdf,
                           const Vector3<S>& lineVec, const Vector3<S>& lineOrig,
                           S& t0, S& t1)
{
  // Holds the products of the two SDFs that do not correspond to the index
  std::array<S, 3> sgnOpp = {
      sdf[1] * sdf[2],
      sdf[0] * sdf[2],
      sdf[0] * sdf[1]
  };

  // Project vertices to line
  std::array<S, 3> proj;
  for (int i = 0; i < 3; ++i)
  {
    proj[i] = projectToLine(lineVec, lineOrig, tri[i].get());
  }

  // Check if one vertex lies directly on the line
  for (int i = 0; i < 3; ++i)
  {
    // NB: Works even if one of the other vertices is also
    // on the line of intersection
    if (isZero<S, ROBUST>(sdf[i]))
    {
      // The current vertex is already on the line
      t0 = proj[i];

      // If the products of the two remaining SDFs are positive,
      // they are both on the same side of the line, and the intersection interval
      // is a single point: t0 = t1
      if (sgnOpp[i] > 0.0)
      {
        t1 = t0;
        return;
      }

      // Points [idx0] and [idx1] straddle the line: compute intersection
      int idx0 = (i+1)%3;
      int idx1 = (i+2)%3;
      t1 = proj[idx0] + (proj[idx1] - proj[idx0]) * sdf[idx0] / (sdf[idx0] - sdf[idx1]);

      SORT(t0, t1);
      return;
    }
  }

  int oppSideIdx;
  if (sgnOpp[2] > 0.0)
  {
    // points [0] and [1] are on the same side of the plane, [2] is not
    oppSideIdx = 2;
  }
  else if (sgnOpp[1] > 0.0)
  {
    // points [0] and [2] are on the same side of the plane, [1] is not
    oppSideIdx = 1;
  }
  else
  {
    // points [1] and [2] are on the same side of the plane, [0] is not
    assert(sgnOpp[0] > 0.0);
    oppSideIdx = 0;
  }

  int idx0 = (oppSideIdx+1)%3;
  int idx1 = (oppSideIdx+2)%3;

  t0 = proj[idx0] + (proj[oppSideIdx] - proj[idx0]) * sdf[idx0] / (sdf[idx0] - sdf[oppSideIdx]);
  t1 = proj[idx1] + (proj[oppSideIdx] - proj[idx1]) * sdf[idx1] / (sdf[idx1] - sdf[oppSideIdx]);

  SORT(t0, t1);
}

//==============================================================================
template <typename S, bool ROBUST=true>
bool intersectTriangles(
    const Vector3<S>& p1, const Vector3<S>& p2, const Vector3<S>& p3,
    const Vector3<S>& q1, const Vector3<S>& q2, const Vector3<S>& q3,
    Vector3<S>* contact_points,
    unsigned int* num_contact_points,
    S* penetration_depth,
    Vector3<S>* normal)
{
  const std::array<std::reference_wrapper<const Vector3<S>>, 3> P = { p1, p2, p3 };
  const std::array<std::reference_wrapper<const Vector3<S>>, 3> Q = { q1, q2, q3 };

  // Compute the plane of P
  const Vector3<S> np = computeNormal(p1, p2, p3);
  const S tp = np.dot(p1);

  // Compute the signed distance of Q's vertices to P
  std::array<S, 3> sdfQtoP;
  for (int i = 0; i < 3; ++i)
  {
    sdfQtoP[i] = np.dot(Q[i].get()) - tp;
    if (ROBUST)
    {
      if (isZero<S, ROBUST>(sdfQtoP[i]))
      {
        sdfQtoP[i] = 0.0;
      }
    }
  }

  // If all signs are the same, Q lies entirely on one side of P
  if (sdfQtoP[0]*sdfQtoP[1] > 0.0 && sdfQtoP[0]*sdfQtoP[2] > 0)
  {
    return false;
  }

  // Compute the plane of Q
  const Vector3<S> nq = computeNormal(q1, q2, q3);
  const S tq = nq.dot(q1);

  // Compute the signed distance of P's vertices to Q
  std::array<S, 3> sdfPtoQ;
  for (int i = 0; i < 3; ++i)
  {
    sdfPtoQ[i] = nq.dot(P[i].get()) - tq;
    if (ROBUST)
    {
      if (isZero<S, ROBUST>(sdfPtoQ[i]))
      {
        sdfPtoQ[i] = 0.0;
      }
    }
  }

  // If all signs are the same, P lies entirely on one side of Q
  if (sdfPtoQ[0]*sdfPtoQ[1] > 0.0 && sdfPtoQ[0]*sdfPtoQ[2] > 0)
  {
    return false;
  }

  // Check for coplanar triangles
  if (areCoplanar<S, ROBUST>(np, tp, nq, tq))
  {
    return overlapCoplanarTriangles<S, ROBUST>(p1, p2, p3, q1, q2, q3, nq, tq,
                      contact_points, num_contact_points,
                      penetration_depth, normal);
  }

  // Compute the intersection line of planes P and Q
  const Vector3<S> lineVec = np.cross(nq).normalized();
  Vector3<S> linePt(0,0,0);
  if (fabs(lineVec.z()) > 0.1)
  {
    solveCramer(np[0], np[1], tp, nq[0], nq[1], tq, linePt[0], linePt[1]);
  }
  else if (fabs(lineVec.x()) > 0.1)
  {
    solveCramer(np[1], np[2], tp, nq[1], nq[2], tq, linePt[1], linePt[2]);
  }
  else
  {
    solveCramer(np[0], np[2], tp, nq[0], nq[2], tq, linePt[0], linePt[2]);
  }

  // Compute intersection intervals a and b
  S aMin =  std::numeric_limits<S>::max();
  S aMax = -std::numeric_limits<S>::max();
  S bMin =  std::numeric_limits<S>::max();
  S bMax = -std::numeric_limits<S>::max();
  intersectTriangleLine(P, sdfPtoQ, lineVec, linePt, aMin, aMax);
  intersectTriangleLine(Q, sdfQtoP, lineVec, linePt, bMin, bMax);

  S intMin = std::max(aMin, bMin);
  S intMax = std::min(aMax, bMax);

  // No intersection
  if (intMin > intMax)
  {
    return false;
  }

  if(contact_points && num_contact_points && penetration_depth && normal)
  {
    if (isZero<S, ROBUST>(intMax-intMin))
    {
      *num_contact_points = 1;
      contact_points[0] = linePt + intMin * lineVec;
    }
    else
    {
      *num_contact_points = 2;
      contact_points[0] = linePt + intMin * lineVec;
      contact_points[1] = linePt + intMax * lineVec;
    }

    S pd1 = -(*std::min_element(std::begin(sdfPtoQ), std::end(sdfPtoQ)));
    S pd2 = -(*std::min_element(std::begin(sdfQtoP), std::end(sdfQtoP)));
    if (pd2 > pd1)
    {
      // Q penetrates into P farther than vice versa, so return Q's normal
      // (depth here means the smallest motion along the normal required to remove contact)
      *penetration_depth = pd1;
      *normal = nq;
    }
    else
    {
      *penetration_depth = pd2;
      *normal = np;
    }
  }
  return true;
}

} // namespace detail
} // namespace fcl

#endif // TRIANGLE_TRIANGLE_INL_H

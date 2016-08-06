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

#ifndef FCL_NARROWPHASE_NARROWPHASEALGORITHMS_H
#define FCL_NARROWPHASE_NARROWPHASEALGORITHMS_H

#include <algorithm>

#include "fcl/collision_data.h"
#include "fcl/narrowphase/gjk.h"
#include "fcl/narrowphase/gjk_libccd.h"

namespace fcl
{

namespace details
{

//==============================================================================
// Clamp n to lie within the range [min, max]
template <typename Scalar>
Scalar clamp(Scalar n, Scalar min, Scalar max)
{
  if (n < min) return min;
  if (n > max) return max;
  return n;
}

//==============================================================================
// Computes closest points C1 and C2 of S1(s)=P1+s*(Q1-P1) and
// S2(t)=P2+t*(Q2-P2), returning s and t. Function result is squared
// distance between between S1(s) and S2(t)
template <typename Scalar>
Scalar closestPtSegmentSegment(
    Vector3<Scalar> p1, Vector3<Scalar> q1, Vector3<Scalar> p2, Vector3<Scalar> q2,
    Scalar &s, Scalar &t, Vector3<Scalar> &c1, Vector3<Scalar> &c2)
{
  const Scalar EPSILON = 0.001;
  Vector3<Scalar> d1 = q1 - p1; // Direction vector of segment S1
  Vector3<Scalar> d2 = q2 - p2; // Direction vector of segment S2
  Vector3<Scalar> r = p1 - p2;
  Scalar a = d1.dot(d1); // Squared length of segment S1, always nonnegative

  Scalar e = d2.dot(d2); // Squared length of segment S2, always nonnegative
  Scalar f = d2.dot(r);
  // Check if either or both segments degenerate into points
  if (a <= EPSILON && e <= EPSILON) {
    // Both segments degenerate into points
    s = t = 0.0;
    c1 = p1;
    c2 = p2;
    Vector3<Scalar> diff = c1-c2;
    Scalar res = diff.dot(diff);
    return res;
  }
  if (a <= EPSILON) {
    // First segment degenerates into a point
    s = 0.0;
    t = f / e; // s = 0 => t = (b*s + f) / e = f / e
    t = clamp(t, (Scalar)0.0, (Scalar)1.0);
  } else {
    Scalar c = d1.dot(r);
    if (e <= EPSILON) {
// Second segment degenerates into a point
t = 0.0;
s = clamp(-c / a, (Scalar)0.0, (Scalar)1.0); // t = 0 => s = (b*t - c) / a = -c / a
    } else {
// The general nondegenerate case starts here
Scalar b = d1.dot(d2);
Scalar denom = a*e-b*b; // Always nonnegative
// If segments not parallel, compute closest point on L1 to L2 and
// clamp to segment S1. Else pick arbitrary s (here 0)
if (denom != 0.0) {
  std::cerr << "denominator equals zero, using 0 as reference" << std::endl;
  s = clamp((b*f - c*e) / denom, (Scalar)0.0, (Scalar)1.0);
} else s = 0.0;
// Compute point on L2 closest to S1(s) using
// t = Dot((P1 + D1*s) - P2,D2) / Dot(D2,D2) = (b*s + f) / e
t = (b*s + f) / e;

//
//If t in [0,1] done. Else clamp t, recompute s for the new value
//of t using s = Dot((P2 + D2*t) - P1,D1) / Dot(D1,D1)= (t*b - c) / a
//and clamp s to [0, 1]
if(t < 0.0) {
  t = 0.0;
  s = clamp(-c / a, (Scalar)0.0, (Scalar)1.0);
} else if (t > 1.0) {
  t = 1.0;
  s = clamp((b - c) / a, (Scalar)0.0, (Scalar)1.0);
}
    }
  }
  c1 = p1 + d1 * s;
  c2 = p2 + d2 * t;
  Vector3<Scalar> diff = c1-c2;
  Scalar res = diff.dot(diff);
  return res;
}


//==============================================================================
// Computes closest points C1 and C2 of S1(s)=P1+s*(Q1-P1) and
// S2(t)=P2+t*(Q2-P2), returning s and t. Function result is squared
// distance between between S1(s) and S2(t)
template <typename Scalar>
bool capsuleCapsuleDistance(const Capsule<Scalar>& s1, const Transform3<Scalar>& tf1,
          const Capsule<Scalar>& s2, const Transform3<Scalar>& tf2,
          Scalar* dist, Vector3<Scalar>* p1_res, Vector3<Scalar>* p2_res)
{

  Vector3<Scalar> p1(tf1.translation());
  Vector3<Scalar> p2(tf2.translation());

  // line segment composes two points. First point is given by the origin, second point is computed by the origin transformed along z.
  // extension along z-axis means transformation with identity matrix and translation vector z pos
  Transform3<Scalar> transformQ1 = tf1 * Translation3<Scalar>(Vector3<Scalar>(0,0,s1.lz));
  Vector3<Scalar> q1 = transformQ1.translation();

  Transform3<Scalar> transformQ2 = tf2 * Translation3<Scalar>(Vector3<Scalar>(0,0,s2.lz));
  Vector3<Scalar> q2 = transformQ2.translation();

  // s and t correspont to the length of the line segment
  Scalar s, t;
  Vector3<Scalar> c1, c2;

  Scalar result = closestPtSegmentSegment(p1, q1, p2, q2, s, t, c1, c2);
  *dist = sqrt(result)-s1.radius-s2.radius;

  // getting directional unit vector
  Vector3<Scalar> distVec = c2 -c1;
  distVec.normalize();

  // extend the point to be border of the capsule.
  // Done by following the directional unit vector for the length of the capsule radius
  *p1_res = c1 + distVec*s1.radius;

  distVec = c1-c2;
  distVec.normalize();

  *p2_res = c2 + distVec*s2.radius;

  return true;
}

//==============================================================================
// Compute the point on a line segment that is the closest point on the
// segment to to another point. The code is inspired by the explanation
// given by Dan Sunday's page:
//   http://geomalgorithms.com/a02-_lines.html
template <typename Scalar>
static inline void lineSegmentPointClosestToPoint (const Vector3<Scalar> &p, const Vector3<Scalar> &s1, const Vector3<Scalar> &s2, Vector3<Scalar> &sp) {
  Vector3<Scalar> v = s2 - s1;
  Vector3<Scalar> w = p - s1;

  Scalar c1 = w.dot(v);
  Scalar c2 = v.dot(v);

  if (c1 <= 0) {
    sp = s1;
  } else if (c2 <= c1) {
    sp = s2;
  } else {
    Scalar b = c1/c2;
    Vector3<Scalar> Pb = s1 + v * b;
    sp = Pb;
  }
}

//==============================================================================
template <typename Scalar>
bool sphereCapsuleIntersect(const Sphere<Scalar>& s1, const Transform3<Scalar>& tf1,
                            const Capsule<Scalar>& s2, const Transform3<Scalar>& tf2,
                            std::vector<ContactPoint<Scalar>>* contacts)
{
  const Vector3<Scalar> pos1(0., 0., 0.5 * s2.lz);
  const Vector3<Scalar> pos2(0., 0., -0.5 * s2.lz);
  const Vector3<Scalar> s_c = tf2.inverse() * tf1.translation();

  Vector3<Scalar> segment_point;

  lineSegmentPointClosestToPoint (s_c, pos1, pos2, segment_point);
  Vector3<Scalar> diff = s_c - segment_point;

  const Scalar distance = diff.norm() - s1.radius - s2.radius;

  if (distance > 0)
    return false;

  const Vector3<Scalar> local_normal = -diff.normalized();

  if (contacts)
  {
    const Vector3<Scalar> normal = tf2.linear() * local_normal;
    const Vector3<Scalar> point = tf2 * (segment_point + local_normal * distance);
    const Scalar penetration_depth = -distance;

    contacts->push_back(ContactPoint<Scalar>(normal, point, penetration_depth));
  }

  return true;
}

//==============================================================================
template <typename Scalar>
bool sphereCapsuleDistance(const Sphere<Scalar>& s1, const Transform3<Scalar>& tf1,
                           const Capsule<Scalar>& s2, const Transform3<Scalar>& tf2,
                           Scalar* dist, Vector3<Scalar>* p1, Vector3<Scalar>* p2)
{
  Vector3<Scalar> pos1(0., 0., 0.5 * s2.lz);
  Vector3<Scalar> pos2(0., 0., -0.5 * s2.lz);
  Vector3<Scalar> s_c = tf2.inverse() * tf1.translation();

  Vector3<Scalar> segment_point;

  lineSegmentPointClosestToPoint (s_c, pos1, pos2, segment_point);
  Vector3<Scalar> diff = s_c - segment_point;

  Scalar distance = diff.norm() - s1.radius - s2.radius;

  if(distance <= 0)
    return false;

  if(dist) *dist = distance;

  if(p1 || p2) diff.normalize();
  if(p1)
  {
    *p1 = s_c - diff * s1.radius;
    *p1 = tf1.inverse() * tf2 * (*p1);
  }

  if(p2) *p2 = segment_point + diff * s1.radius;

  return true;
}

//==============================================================================
template <typename Scalar>
bool sphereSphereIntersect(const Sphere<Scalar>& s1, const Transform3<Scalar>& tf1,
                           const Sphere<Scalar>& s2, const Transform3<Scalar>& tf2,
                           std::vector<ContactPoint<Scalar>>* contacts)
{
  Vector3<Scalar> diff = tf2.translation() - tf1.translation();
  Scalar len = diff.norm();
  if(len > s1.radius + s2.radius)
    return false;

  if(contacts)
  {
    // If the centers of two sphere are at the same position, the normal is (0, 0, 0).
    // Otherwise, normal is pointing from center of object 1 to center of object 2
    const Vector3<Scalar> normal = len > 0 ? (diff / len).eval() : diff;
    const Vector3<Scalar> point = tf1.translation() + diff * s1.radius / (s1.radius + s2.radius);
    const Scalar penetration_depth = s1.radius + s2.radius - len;
    contacts->push_back(ContactPoint<Scalar>(normal, point, penetration_depth));
  }

  return true;
}

//==============================================================================
template <typename Scalar>
bool sphereSphereDistance(const Sphere<Scalar>& s1, const Transform3<Scalar>& tf1,
                          const Sphere<Scalar>& s2, const Transform3<Scalar>& tf2,
                          Scalar* dist, Vector3<Scalar>* p1, Vector3<Scalar>* p2)
{
  Vector3<Scalar> o1 = tf1.translation();
  Vector3<Scalar> o2 = tf2.translation();
  Vector3<Scalar> diff = o1 - o2;
  Scalar len = diff.norm();
  if(len > s1.radius + s2.radius)
  {
    if(dist) *dist = len - (s1.radius + s2.radius);
    if(p1) *p1 = tf1.inverse() * (o1 - diff * (s1.radius / len));
    if(p2) *p2 = tf2.inverse() * (o2 + diff * (s2.radius / len));
    return true;
  }

  if(dist) *dist = -1;
  return false;
}

//==============================================================================
/** \brief the minimum distance from a point to a line */
template <typename Scalar>
Scalar segmentSqrDistance(const Vector3<Scalar>& from, const Vector3<Scalar>& to,const Vector3<Scalar>& p, Vector3<Scalar>& nearest)
{
  Vector3<Scalar> diff = p - from;
  Vector3<Scalar> v = to - from;
  Scalar t = v.dot(diff);

  if(t > 0)
  {
    Scalar dotVV = v.dot(v);
    if(t < dotVV)
    {
      t /= dotVV;
      diff -= v * t;
    }
    else
    {
      t = 1;
      diff -= v;
    }
  }
  else
    t = 0;

  nearest = from + v * t;
  return diff.dot(diff);
}

//==============================================================================
/// @brief Whether a point's projection is in a triangle
template <typename Scalar>
bool projectInTriangle(const Vector3<Scalar>& p1, const Vector3<Scalar>& p2, const Vector3<Scalar>& p3, const Vector3<Scalar>& normal, const Vector3<Scalar>& p)
{
  Vector3<Scalar> edge1(p2 - p1);
  Vector3<Scalar> edge2(p3 - p2);
  Vector3<Scalar> edge3(p1 - p3);

  Vector3<Scalar> p1_to_p(p - p1);
  Vector3<Scalar> p2_to_p(p - p2);
  Vector3<Scalar> p3_to_p(p - p3);

  Vector3<Scalar> edge1_normal(edge1.cross(normal));
  Vector3<Scalar> edge2_normal(edge2.cross(normal));
  Vector3<Scalar> edge3_normal(edge3.cross(normal));

  Scalar r1, r2, r3;
  r1 = edge1_normal.dot(p1_to_p);
  r2 = edge2_normal.dot(p2_to_p);
  r3 = edge3_normal.dot(p3_to_p);
  if ( ( r1 > 0 && r2 > 0 && r3 > 0 ) ||
       ( r1 <= 0 && r2 <= 0 && r3 <= 0 ) )
    return true;
  return false;
}

//==============================================================================
template <typename Scalar>
bool sphereTriangleIntersect(const Sphere<Scalar>& s, const Transform3<Scalar>& tf,
                             const Vector3<Scalar>& P1, const Vector3<Scalar>& P2, const Vector3<Scalar>& P3, Vector3<Scalar>* contact_points, Scalar* penetration_depth, Vector3<Scalar>* normal_)
{
  Vector3<Scalar> normal = (P2 - P1).cross(P3 - P1);
  normal.normalize();
  const Vector3<Scalar>& center = tf.translation();
  const Scalar& radius = s.radius;
  Scalar radius_with_threshold = radius + std::numeric_limits<Scalar>::epsilon();
  Vector3<Scalar> p1_to_center = center - P1;
  Scalar distance_from_plane = p1_to_center.dot(normal);

  if(distance_from_plane < 0)
  {
    distance_from_plane *= -1;
    normal *= -1;
  }

  bool is_inside_contact_plane = (distance_from_plane < radius_with_threshold);

  bool has_contact = false;
  Vector3<Scalar> contact_point;
  if(is_inside_contact_plane)
  {
    if(projectInTriangle(P1, P2, P3, normal, center))
    {
      has_contact = true;
      contact_point = center - normal * distance_from_plane;
    }
    else
    {
      Scalar contact_capsule_radius_sqr = radius_with_threshold * radius_with_threshold;
      Vector3<Scalar> nearest_on_edge;
      Scalar distance_sqr;
      distance_sqr = segmentSqrDistance(P1, P2, center, nearest_on_edge);
      if(distance_sqr < contact_capsule_radius_sqr)
      {
        has_contact = true;
        contact_point = nearest_on_edge;
      }

      distance_sqr = segmentSqrDistance(P2, P3, center, nearest_on_edge);
      if(distance_sqr < contact_capsule_radius_sqr)
      {
        has_contact = true;
        contact_point = nearest_on_edge;
      }

      distance_sqr = segmentSqrDistance(P3, P1, center, nearest_on_edge);
      if(distance_sqr < contact_capsule_radius_sqr)
      {
        has_contact = true;
        contact_point = nearest_on_edge;
      }
    }
  }

  if(has_contact)
  {
    Vector3<Scalar> contact_to_center = contact_point - center;
    Scalar distance_sqr = contact_to_center.squaredNorm();

    if(distance_sqr < radius_with_threshold * radius_with_threshold)
    {
      if(distance_sqr > 0)
      {
        Scalar distance = std::sqrt(distance_sqr);
        if(normal_) *normal_ = contact_to_center.normalized();
        if(contact_points) *contact_points = contact_point;
        if(penetration_depth) *penetration_depth = -(radius - distance);
      }
      else
      {
        if(normal_) *normal_ = -normal;
        if(contact_points) *contact_points = contact_point;
        if(penetration_depth) *penetration_depth = -radius;
      }

      return true;
    }
  }

  return false;
}

//==============================================================================
template <typename Scalar>
bool sphereTriangleDistance(const Sphere<Scalar>& sp, const Transform3<Scalar>& tf,
                            const Vector3<Scalar>& P1, const Vector3<Scalar>& P2, const Vector3<Scalar>& P3,
                            Scalar* dist)
{
  // from geometric tools, very different from the collision code.

  const Vector3<Scalar>& center = tf.translation();
  Scalar radius = sp.radius;
  Vector3<Scalar> diff = P1 - center;
  Vector3<Scalar> edge0 = P2 - P1;
  Vector3<Scalar> edge1 = P3 - P1;
  Scalar a00 = edge0.squaredNorm();
  Scalar a01 = edge0.dot(edge1);
  Scalar a11 = edge1.squaredNorm();
  Scalar b0 = diff.dot(edge0);
  Scalar b1 = diff.dot(edge1);
  Scalar c = diff.squaredNorm();
  Scalar det = fabs(a00*a11 - a01*a01);
  Scalar s = a01*b1 - a11*b0;
  Scalar t = a01*b0 - a00*b1;

  Scalar sqr_dist;

  if(s + t <= det)
  {
    if(s < 0)
    {
      if(t < 0)  // region 4
      {
        if(b0 < 0)
        {
          t = 0;
          if(-b0 >= a00)
          {
            s = 1;
            sqr_dist = a00 + 2*b0 + c;
          }
          else
          {
            s = -b0/a00;
            sqr_dist = b0*s + c;
          }
        }
        else
        {
          s = 0;
          if(b1 >= 0)
          {
            t = 0;
            sqr_dist = c;
          }
          else if(-b1 >= a11)
          {
            t = 1;
            sqr_dist = a11 + 2*b1 + c;
          }
          else
          {
            t = -b1/a11;
            sqr_dist = b1*t + c;
          }
        }
      }
      else  // region 3
      {
        s = 0;
        if(b1 >= 0)
        {
          t = 0;
          sqr_dist = c;
        }
        else if(-b1 >= a11)
        {
          t = 1;
          sqr_dist = a11 + 2*b1 + c;
        }
        else
        {
          t = -b1/a11;
          sqr_dist = b1*t + c;
        }
      }
    }
    else if(t < 0)  // region 5
    {
      t = 0;
      if(b0 >= 0)
      {
        s = 0;
        sqr_dist = c;
      }
      else if(-b0 >= a00)
      {
        s = 1;
        sqr_dist = a00 + 2*b0 + c;
      }
      else
      {
        s = -b0/a00;
        sqr_dist = b0*s + c;
      }
    }
    else  // region 0
    {
      // minimum at interior point
      Scalar inv_det = (1)/det;
      s *= inv_det;
      t *= inv_det;
      sqr_dist = s*(a00*s + a01*t + 2*b0) + t*(a01*s + a11*t + 2*b1) + c;
    }
  }
  else
  {
    Scalar tmp0, tmp1, numer, denom;

    if(s < 0)  // region 2
    {
      tmp0 = a01 + b0;
      tmp1 = a11 + b1;
      if(tmp1 > tmp0)
      {
        numer = tmp1 - tmp0;
        denom = a00 - 2*a01 + a11;
        if(numer >= denom)
        {
          s = 1;
          t = 0;
          sqr_dist = a00 + 2*b0 + c;
        }
        else
        {
          s = numer/denom;
          t = 1 - s;
          sqr_dist = s*(a00*s + a01*t + 2*b0) + t*(a01*s + a11*t + 2*b1) + c;
        }
      }
      else
      {
        s = 0;
        if(tmp1 <= 0)
        {
          t = 1;
          sqr_dist = a11 + 2*b1 + c;
        }
        else if(b1 >= 0)
        {
          t = 0;
          sqr_dist = c;
        }
        else
        {
          t = -b1/a11;
          sqr_dist = b1*t + c;
        }
      }
    }
    else if(t < 0)  // region 6
    {
      tmp0 = a01 + b1;
      tmp1 = a00 + b0;
      if(tmp1 > tmp0)
      {
        numer = tmp1 - tmp0;
        denom = a00 - 2*a01 + a11;
        if(numer >= denom)
        {
          t = 1;
          s = 0;
          sqr_dist = a11 + 2*b1 + c;
        }
        else
        {
          t = numer/denom;
          s = 1 - t;
          sqr_dist = s*(a00*s + a01*t + 2*b0) + t*(a01*s + a11*t + 2*b1) + c;
        }
      }
      else
      {
        t = 0;
        if(tmp1 <= 0)
        {
          s = 1;
          sqr_dist = a00 + 2*b0 + c;
        }
        else if(b0 >= 0)
        {
          s = 0;
          sqr_dist = c;
        }
        else
        {
          s = -b0/a00;
          sqr_dist = b0*s + c;
        }
      }
    }
    else  // region 1
    {
      numer = a11 + b1 - a01 - b0;
      if(numer <= 0)
      {
        s = 0;
        t = 1;
        sqr_dist = a11 + 2*b1 + c;
      }
      else
      {
        denom = a00 - 2*a01 + a11;
        if(numer >= denom)
        {
          s = 1;
          t = 0;
          sqr_dist = a00 + 2*b0 + c;
        }
        else
        {
          s = numer/denom;
          t = 1 - s;
          sqr_dist = s*(a00*s + a01*t + 2*b0) + t*(a01*s + a11*t + 2*b1) + c;
        }
      }
    }
  }

  // Account for numerical round-off error.
  if(sqr_dist < 0)
    sqr_dist = 0;

  if(sqr_dist > radius * radius)
  {
    if(dist) *dist = std::sqrt(sqr_dist) - radius;
    return true;
  }
  else
  {
    if(dist) *dist = -1;
    return false;
  }
}

//==============================================================================
template <typename Scalar>
bool sphereTriangleDistance(const Sphere<Scalar>& sp, const Transform3<Scalar>& tf,
                            const Vector3<Scalar>& P1, const Vector3<Scalar>& P2, const Vector3<Scalar>& P3,
                            Scalar* dist, Vector3<Scalar>* p1, Vector3<Scalar>* p2)
{
  if(p1 || p2)
  {
    Vector3<Scalar> o = tf.translation();
    typename Project<Scalar>::ProjectResult result;
    result = Project<Scalar>::projectTriangle(P1, P2, P3, o);
    if(result.sqr_distance > sp.radius * sp.radius)
    {
      if(dist) *dist = std::sqrt(result.sqr_distance) - sp.radius;
      Vector3<Scalar> project_p = P1 * result.parameterization[0] + P2 * result.parameterization[1] + P3 * result.parameterization[2];
      Vector3<Scalar> dir = o - project_p;
      dir.normalize();
      if(p1) { *p1 = o - dir * sp.radius; *p1 = tf.inverse() * (*p1); }
      if(p2) *p2 = project_p;
      return true;
    }
    else
      return false;
  }
  else
  {
    return sphereTriangleDistance(sp, tf, P1, P2, P3, dist);
  }
}

//==============================================================================
template <typename Scalar>
bool sphereTriangleDistance(const Sphere<Scalar>& sp, const Transform3<Scalar>& tf1,
                            const Vector3<Scalar>& P1, const Vector3<Scalar>& P2, const Vector3<Scalar>& P3, const Transform3<Scalar>& tf2,
                            Scalar* dist, Vector3<Scalar>* p1, Vector3<Scalar>* p2)
{
  bool res = details::sphereTriangleDistance(sp, tf1, tf2 * P1, tf2 * P2, tf2 * P3, dist, p1, p2);
  if(p2) *p2 = tf2.inverse() * (*p2);

  return res;
}

//==============================================================================
template <typename Scalar>
static inline void lineClosestApproach(const Vector3<Scalar>& pa, const Vector3<Scalar>& ua,
                                       const Vector3<Scalar>& pb, const Vector3<Scalar>& ub,
                                       Scalar* alpha, Scalar* beta)
{
  Vector3<Scalar> p = pb - pa;
  Scalar uaub = ua.dot(ub);
  Scalar q1 = ua.dot(p);
  Scalar q2 = -ub.dot(p);
  Scalar d = 1 - uaub * uaub;
  if(d <= (Scalar)(0.0001f))
  {
    *alpha = 0;
    *beta = 0;
  }
  else
  {
    d = 1 / d;
    *alpha = (q1 + uaub * q2) * d;
    *beta = (uaub * q1 + q2) * d;
  }
}

//==============================================================================
// find all the intersection points between the 2D rectangle with vertices
// at (+/-h[0],+/-h[1]) and the 2D quadrilateral with vertices (p[0],p[1]),
// (p[2],p[3]),(p[4],p[5]),(p[6],p[7]).
//
// the intersection points are returned as x,y pairs in the 'ret' array.
// the number of intersection points is returned by the function (this will
// be in the range 0 to 8).
template <typename Scalar>
static int intersectRectQuad2(Scalar h[2], Scalar p[8], Scalar ret[16])
{
  // q (and r) contain nq (and nr) coordinate points for the current (and
  // chopped) polygons
  int nq = 4, nr = 0;
  Scalar buffer[16];
  Scalar* q = p;
  Scalar* r = ret;
  for(int dir = 0; dir <= 1; ++dir)
  {
    // direction notation: xy[0] = x axis, xy[1] = y axis
    for(int sign = -1; sign <= 1; sign += 2)
    {
      // chop q along the line xy[dir] = sign*h[dir]
      Scalar* pq = q;
      Scalar* pr = r;
      nr = 0;
      for(int i = nq; i > 0; --i)
      {
        // go through all points in q and all lines between adjacent points
        if(sign * pq[dir] < h[dir])
        {
          // this point is inside the chopping line
          pr[0] = pq[0];
          pr[1] = pq[1];
          pr += 2;
          nr++;
          if(nr & 8)
          {
            q = r;
            goto done;
          }
        }
        Scalar* nextq = (i > 1) ? pq+2 : q;
        if((sign*pq[dir] < h[dir]) ^ (sign*nextq[dir] < h[dir]))
        {
          // this line crosses the chopping line
          pr[1-dir] = pq[1-dir] + (nextq[1-dir]-pq[1-dir]) /
            (nextq[dir]-pq[dir]) * (sign*h[dir]-pq[dir]);
          pr[dir] = sign*h[dir];
          pr += 2;
          nr++;
          if(nr & 8)
          {
            q = r;
            goto done;
          }
        }
        pq += 2;
      }
      q = r;
      r = (q == ret) ? buffer : ret;
      nq = nr;
    }
  }

 done:
  if(q != ret) memcpy(ret, q, nr*2*sizeof(Scalar));
  return nr;
}

//==============================================================================
// given n points in the plane (array p, of size 2*n), generate m points that
// best represent the whole set. the definition of 'best' here is not
// predetermined - the idea is to select points that give good box-box
// collision detection behavior. the chosen point indexes are returned in the
// array iret (of size m). 'i0' is always the first entry in the array.
// n must be in the range [1..8]. m must be in the range [1..n]. i0 must be
// in the range [0..n-1].
template <typename Scalar>
static inline void cullPoints2(int n, Scalar p[], int m, int i0, int iret[])
{
  // compute the centroid of the polygon in cx,cy
  Scalar a, cx, cy, q;
  switch(n)
  {
  case 1:
    cx = p[0];
    cy = p[1];
    break;
  case 2:
    cx = 0.5 * (p[0] + p[2]);
    cy = 0.5 * (p[1] + p[3]);
    break;
  default:
    a = 0;
    cx = 0;
    cy = 0;
    for(int i = 0; i < n-1; ++i)
    {
      q = p[i*2]*p[i*2+3] - p[i*2+2]*p[i*2+1];
      a += q;
      cx += q*(p[i*2]+p[i*2+2]);
      cy += q*(p[i*2+1]+p[i*2+3]);
    }
    q = p[n*2-2]*p[1] - p[0]*p[n*2-1];
    if(std::abs(a+q) > std::numeric_limits<Scalar>::epsilon())
      a = 1/(3*(a+q));
    else
      a= 1e18f;

    cx = a*(cx + q*(p[n*2-2]+p[0]));
    cy = a*(cy + q*(p[n*2-1]+p[1]));
  }


  // compute the angle of each point w.r.t. the centroid
  Scalar A[8];
  for(int i = 0; i < n; ++i)
    A[i] = atan2(p[i*2+1]-cy,p[i*2]-cx);

  // search for points that have angles closest to A[i0] + i*(2*pi/m).
  int avail[8];
  for(int i = 0; i < n; ++i) avail[i] = 1;
  avail[i0] = 0;
  iret[0] = i0;
  iret++;
  const Scalar pi = constants<Scalar>::pi();
  for(int j = 1; j < m; ++j)
  {
    a = j*(2*pi/m) + A[i0];
    if (a > pi) a -= 2*pi;
    Scalar maxdiff= 1e9, diff;

    *iret = i0;	// iret is not allowed to keep this value, but it sometimes does, when diff=#QNAN0
    for(int i = 0; i < n; ++i)
    {
      if(avail[i])
      {
        diff = std::abs(A[i]-a);
        if(diff > pi) diff = 2*pi - diff;
        if(diff < maxdiff)
        {
          maxdiff = diff;
          *iret = i;
        }
      }
    }
    avail[*iret] = 0;
    iret++;
  }
}

//==============================================================================
template <typename Scalar, typename DerivedA, typename DerivedB>
int boxBox2(
    const Vector3<Scalar>& side1,
    const Eigen::MatrixBase<DerivedA>& R1,
    const Eigen::MatrixBase<DerivedB>& T1,
    const Vector3<Scalar>& side2,
    const Eigen::MatrixBase<DerivedA>& R2,
    const Eigen::MatrixBase<DerivedB>& T2,
    Vector3<Scalar>& normal,
    Scalar* depth,
    int* return_code,
    int maxc,
    std::vector<ContactPoint<Scalar>>& contacts)
{
  const Scalar fudge_factor = Scalar(1.05);
  Vector3<Scalar> normalC;
  Scalar s, s2, l;
  int invert_normal, code;

  Vector3<Scalar> p = T2 - T1; // get vector from centers of box 1 to box 2, relative to box 1
  Vector3<Scalar> pp = R1.transpose() * p; // get pp = p relative to body 1

  // get side lengths / 2
  Vector3<Scalar> A = side1 * 0.5;
  Vector3<Scalar> B = side2 * 0.5;

  // Rij is R1'*R2, i.e. the relative rotation between R1 and R2
  Matrix3<Scalar> R = R1.transpose() * R2;
  Matrix3<Scalar> Q = R.cwiseAbs();


  // for all 15 possible separating axes:
  //   * see if the axis separates the boxes. if so, return 0.
  //   * find the depth of the penetration along the separating axis (s2)
  //   * if this is the largest depth so far, record it.
  // the normal vector will be set to the separating axis with the smallest
  // depth. note: normalR is set to point to a column of R1 or R2 if that is
  // the smallest depth normal so far. otherwise normalR is 0 and normalC is
  // set to a vector relative to body 1. invert_normal is 1 if the sign of
  // the normal should be flipped.

  int best_col_id = -1;
  const Eigen::MatrixBase<DerivedA>* normalR = 0;
  Scalar tmp = 0;

  s = - std::numeric_limits<Scalar>::max();
  invert_normal = 0;
  code = 0;

  // separating axis = u1, u2, u3
  tmp = pp[0];
  s2 = std::abs(tmp) - (Q.row(0).dot(B) + A[0]);
  if(s2 > 0) { *return_code = 0; return 0; }
  if(s2 > s)
  {
    s = s2;
    best_col_id = 0;
    normalR = &R1;
    invert_normal = (tmp < 0);
    code = 1;
  }

  tmp = pp[1];
  s2 = std::abs(tmp) - (Q.row(1).dot(B) + A[1]);
  if(s2 > 0) { *return_code = 0; return 0; }
  if(s2 > s)
  {
    s = s2;
    best_col_id = 1;
    normalR = &R1;
    invert_normal = (tmp < 0);
    code = 2;
  }

  tmp = pp[2];
  s2 = std::abs(tmp) - (Q.row(2).dot(B) + A[2]);
  if(s2 > 0) { *return_code = 0; return 0; }
  if(s2 > s)
  {
    s = s2;
    best_col_id = 2;
    normalR = &R1;
    invert_normal = (tmp < 0);
    code = 3;
  }

  // separating axis = v1, v2, v3
  tmp = R2.col(0).dot(p);
  s2 = std::abs(tmp) - (Q.col(0).dot(A) + B[0]);
  if(s2 > 0) { *return_code = 0; return 0; }
  if(s2 > s)
  {
    s = s2;
    best_col_id = 0;
    normalR = &R2;
    invert_normal = (tmp < 0);
    code = 4;
  }

  tmp = R2.col(1).dot(p);
  s2 = std::abs(tmp) - (Q.col(1).dot(A) + B[1]);
  if(s2 > 0) { *return_code = 0; return 0; }
  if(s2 > s)
  {
    s = s2;
    best_col_id = 1;
    normalR = &R2;
    invert_normal = (tmp < 0);
    code = 5;
  }

  tmp = R2.col(2).dot(p);
  s2 =  std::abs(tmp) - (Q.col(2).dot(A) + B[2]);
  if(s2 > 0) { *return_code = 0; return 0; }
  if(s2 > s)
  {
    s = s2;
    best_col_id = 2;
    normalR = &R2;
    invert_normal = (tmp < 0);
    code = 6;
  }


  Scalar fudge2(1.0e-6);
  Q.array() += fudge2;

  Vector3<Scalar> n;
  Scalar eps = std::numeric_limits<Scalar>::epsilon();

  // separating axis = u1 x (v1,v2,v3)
  tmp = pp[2] * R(1, 0) - pp[1] * R(2, 0);
  s2 = std::abs(tmp) - (A[1] * Q(2, 0) + A[2] * Q(1, 0) + B[1] * Q(0, 2) + B[2] * Q(0, 1));
  if(s2 > 0) { *return_code = 0; return 0; }
  n = Vector3<Scalar>(0, -R(2, 0), R(1, 0));
  l = n.norm();
  if(l > eps)
  {
    s2 /= l;
    if(s2 * fudge_factor > s)
    {
      s = s2;
      best_col_id = -1;
      normalC = n / l;
      invert_normal = (tmp < 0);
      code = 7;
    }
  }

  tmp = pp[2] * R(1, 1) - pp[1] * R(2, 1);
  s2 = std::abs(tmp) - (A[1] * Q(2, 1) + A[2] * Q(1, 1) + B[0] * Q(0, 2) + B[2] * Q(0, 0));
  if(s2 > 0) { *return_code = 0; return 0; }
  n = Vector3<Scalar>(0, -R(2, 1), R(1, 1));
  l = n.norm();
  if(l > eps)
  {
    s2 /= l;
    if(s2 * fudge_factor > s)
    {
      s = s2;
      best_col_id = -1;
      normalC = n / l;
      invert_normal = (tmp < 0);
      code = 8;
    }
  }

  tmp = pp[2] * R(1, 2) - pp[1] * R(2, 2);
  s2 = std::abs(tmp) - (A[1] * Q(2, 2) + A[2] * Q(1, 2) + B[0] * Q(0, 1) + B[1] * Q(0, 0));
  if(s2 > 0) { *return_code = 0; return 0; }
  n = Vector3<Scalar>(0, -R(2, 2), R(1, 2));
  l = n.norm();
  if(l > eps)
  {
    s2 /= l;
    if(s2 * fudge_factor > s)
    {
      s = s2;
      best_col_id = -1;
      normalC = n / l;
      invert_normal = (tmp < 0);
      code = 9;
    }
  }

  // separating axis = u2 x (v1,v2,v3)
  tmp = pp[0] * R(2, 0) - pp[2] * R(0, 0);
  s2 = std::abs(tmp) - (A[0] * Q(2, 0) + A[2] * Q(0, 0) + B[1] * Q(1, 2) + B[2] * Q(1, 1));
  if(s2 > 0) { *return_code = 0; return 0; }
  n = Vector3<Scalar>(R(2, 0), 0, -R(0, 0));
  l = n.norm();
  if(l > eps)
  {
    s2 /= l;
    if(s2 * fudge_factor > s)
    {
      s = s2;
      best_col_id = -1;
      normalC = n / l;
      invert_normal = (tmp < 0);
      code = 10;
    }
  }

  tmp = pp[0] * R(2, 1) - pp[2] * R(0, 1);
  s2 = std::abs(tmp) - (A[0] * Q(2, 1) + A[2] * Q(0, 1) + B[0] * Q(1, 2) + B[2] * Q(1, 0));
  if(s2 > 0) { *return_code = 0; return 0; }
  n = Vector3<Scalar>(R(2, 1), 0, -R(0, 1));
  l = n.norm();
  if(l > eps)
  {
    s2 /= l;
    if(s2 * fudge_factor > s)
    {
      s = s2;
      best_col_id = -1;
      normalC = n / l;
      invert_normal = (tmp < 0);
      code = 11;
    }
  }

  tmp = pp[0] * R(2, 2) - pp[2] * R(0, 2);
  s2 = std::abs(tmp) - (A[0] * Q(2, 2) + A[2] * Q(0, 2) + B[0] * Q(1, 1) + B[1] * Q(1, 0));
  if(s2 > 0) { *return_code = 0; return 0; }
  n = Vector3<Scalar>(R(2, 2), 0, -R(0, 2));
  l = n.norm();
  if(l > eps)
  {
    s2 /= l;
    if(s2 * fudge_factor > s)
    {
      s = s2;
      best_col_id = -1;
      normalC = n / l;
      invert_normal = (tmp < 0);
      code = 12;
    }
  }

  // separating axis = u3 x (v1,v2,v3)
  tmp = pp[1] * R(0, 0) - pp[0] * R(1, 0);
  s2 = std::abs(tmp) - (A[0] * Q(1, 0) + A[1] * Q(0, 0) + B[1] * Q(2, 2) + B[2] * Q(2, 1));
  if(s2 > 0) { *return_code = 0; return 0; }
  n = Vector3<Scalar>(-R(1, 0), R(0, 0), 0);
  l = n.norm();
  if(l > eps)
  {
    s2 /= l;
    if(s2 * fudge_factor > s)
    {
      s = s2;
      best_col_id = -1;
      normalC = n / l;
      invert_normal = (tmp < 0);
      code = 13;
    }
  }

  tmp = pp[1] * R(0, 1) - pp[0] * R(1, 1);
  s2 = std::abs(tmp) - (A[0] * Q(1, 1) + A[1] * Q(0, 1) + B[0] * Q(2, 2) + B[2] * Q(2, 0));
  if(s2 > 0) { *return_code = 0; return 0; }
  n = Vector3<Scalar>(-R(1, 1), R(0, 1), 0);
  l = n.norm();
  if(l > eps)
  {
    s2 /= l;
    if(s2 * fudge_factor > s)
    {
      s = s2;
      best_col_id = -1;
      normalC = n / l;
      invert_normal = (tmp < 0);
      code = 14;
    }
  }

  tmp = pp[1] * R(0, 2) - pp[0] * R(1, 2);
  s2 = std::abs(tmp) - (A[0] * Q(1, 2) + A[1] * Q(0, 2) + B[0] * Q(2, 1) + B[1] * Q(2, 0));
  if(s2 > 0) { *return_code = 0; return 0; }
  n = Vector3<Scalar>(-R(1, 2), R(0, 2), 0);
  l = n.norm();
  if(l > eps)
  {
    s2 /= l;
    if(s2 * fudge_factor > s)
    {
      s = s2;
      best_col_id = -1;
      normalC = n / l;
      invert_normal = (tmp < 0);
      code = 15;
    }
  }



  if (!code) { *return_code = code; return 0; }

  // if we get to this point, the boxes interpenetrate. compute the normal
  // in global coordinates.
  if(best_col_id != -1)
    normal = normalR->col(best_col_id);
  else
    normal = R1 * normalC;

  if(invert_normal)
    normal = -normal;

  *depth = -s; // s is negative when the boxes are in collision

  // compute contact point(s)

  if(code > 6)
  {
    // an edge from box 1 touches an edge from box 2.
    // find a point pa on the intersecting edge of box 1
    Vector3<Scalar> pa(T1);
    Scalar sign;

    for(int j = 0; j < 3; ++j)
    {
      sign = (R1.col(j).dot(normal) > 0) ? 1 : -1;
      pa += R1.col(j) * (A[j] * sign);
    }

    // find a point pb on the intersecting edge of box 2
    Vector3<Scalar> pb(T2);

    for(int j = 0; j < 3; ++j)
    {
      sign = (R2.col(j).dot(normal) > 0) ? -1 : 1;
      pb += R2.col(j) * (B[j] * sign);
    }

    Scalar alpha, beta;
    Vector3<Scalar> ua(R1.col((code-7)/3));
    Vector3<Scalar> ub(R2.col((code-7)%3));

    lineClosestApproach(pa, ua, pb, ub, &alpha, &beta);
    pa += ua * alpha;
    pb += ub * beta;


    // Vector3<Scalar> pointInWorld((pa + pb) * 0.5);
    // contacts.push_back(ContactPoint<Scalar>(-normal, pointInWorld, -*depth));
    contacts.push_back(ContactPoint<Scalar>(normal,pb,-*depth));
    *return_code = code;

    return 1;
  }

  // okay, we have a face-something intersection (because the separating
  // axis is perpendicular to a face). define face 'a' to be the reference
  // face (i.e. the normal vector is perpendicular to this) and face 'b' to be
  // the incident face (the closest face of the other box).

  const Eigen::MatrixBase<DerivedA> *Ra, *Rb;
  const Eigen::MatrixBase<DerivedB> *pa, *pb;
  const Vector3<Scalar> *Sa, *Sb;

  if(code <= 3)
  {
    Ra = &R1;
    Rb = &R2;
    pa = &T1;
    pb = &T2;
    Sa = &A;
    Sb = &B;
  }
  else
  {
    Ra = &R2;
    Rb = &R1;
    pa = &T2;
    pb = &T1;
    Sa = &B;
    Sb = &A;
  }

  // nr = normal vector of reference face dotted with axes of incident box.
  // anr = absolute values of nr.
  Vector3<Scalar> normal2, nr, anr;
  if(code <= 3)
    normal2 = normal;
  else
    normal2 = -normal;

  nr = Rb->transpose() * normal2;
  anr = nr.cwiseAbs();

  // find the largest compontent of anr: this corresponds to the normal
  // for the indident face. the other axis numbers of the indicent face
  // are stored in a1,a2.
  int lanr, a1, a2;
  if(anr[1] > anr[0])
  {
    if(anr[1] > anr[2])
    {
      a1 = 0;
      lanr = 1;
      a2 = 2;
    }
    else
    {
      a1 = 0;
      a2 = 1;
      lanr = 2;
    }
  }
  else
  {
    if(anr[0] > anr[2])
    {
      lanr = 0;
      a1 = 1;
      a2 = 2;
    }
    else
    {
      a1 = 0;
      a2 = 1;
      lanr = 2;
    }
  }

  // compute center point of incident face, in reference-face coordinates
  Vector3<Scalar> center;
  if(nr[lanr] < 0)
    center = (*pb) - (*pa) + Rb->col(lanr) * ((*Sb)[lanr]);
  else
    center = (*pb) - (*pa) - Rb->col(lanr) * ((*Sb)[lanr]);

  // find the normal and non-normal axis numbers of the reference box
  int codeN, code1, code2;
  if(code <= 3)
    codeN = code-1;
  else codeN = code-4;

  if(codeN == 0)
  {
    code1 = 1;
    code2 = 2;
  }
  else if(codeN == 1)
  {
    code1 = 0;
    code2 = 2;
  }
  else
  {
    code1 = 0;
    code2 = 1;
  }

  // find the four corners of the incident face, in reference-face coordinates
  Scalar quad[8]; // 2D coordinate of incident face (x,y pairs)
  Scalar c1, c2, m11, m12, m21, m22;
  c1 = Ra->col(code1).dot(center);
  c2 = Ra->col(code2).dot(center);
  // optimize this? - we have already computed this data above, but it is not
  // stored in an easy-to-index format. for now it's quicker just to recompute
  // the four dot products.
  Vector3<Scalar> tempRac = Ra->col(code1);
  m11 = Rb->col(a1).dot(tempRac);
  m12 = Rb->col(a2).dot(tempRac);
  tempRac = Ra->col(code2);
  m21 = Rb->col(a1).dot(tempRac);
  m22 = Rb->col(a2).dot(tempRac);

  Scalar k1 = m11 * (*Sb)[a1];
  Scalar k2 = m21 * (*Sb)[a1];
  Scalar k3 = m12 * (*Sb)[a2];
  Scalar k4 = m22 * (*Sb)[a2];
  quad[0] = c1 - k1 - k3;
  quad[1] = c2 - k2 - k4;
  quad[2] = c1 - k1 + k3;
  quad[3] = c2 - k2 + k4;
  quad[4] = c1 + k1 + k3;
  quad[5] = c2 + k2 + k4;
  quad[6] = c1 + k1 - k3;
  quad[7] = c2 + k2 - k4;

  // find the size of the reference face
  Scalar rect[2];
  rect[0] = (*Sa)[code1];
  rect[1] = (*Sa)[code2];

  // intersect the incident and reference faces
  Scalar ret[16];
  int n_intersect = intersectRectQuad2(rect, quad, ret);
  if(n_intersect < 1) { *return_code = code; return 0; } // this should never happen

  // convert the intersection points into reference-face coordinates,
  // and compute the contact position and depth for each point. only keep
  // those points that have a positive (penetrating) depth. delete points in
  // the 'ret' array as necessary so that 'point' and 'ret' correspond.
  Vector3<Scalar> points[8]; // penetrating contact points
  Scalar dep[8]; // depths for those points
  Scalar det1 = 1.f/(m11*m22 - m12*m21);
  m11 *= det1;
  m12 *= det1;
  m21 *= det1;
  m22 *= det1;
  int cnum = 0;	// number of penetrating contact points found
  for(int j = 0; j < n_intersect; ++j)
  {
    Scalar k1 =  m22*(ret[j*2]-c1) - m12*(ret[j*2+1]-c2);
    Scalar k2 = -m21*(ret[j*2]-c1) + m11*(ret[j*2+1]-c2);
    points[cnum] = center + Rb->col(a1) * k1 + Rb->col(a2) * k2;
    dep[cnum] = (*Sa)[codeN] - normal2.dot(points[cnum]);
    if(dep[cnum] >= 0)
    {
      ret[cnum*2] = ret[j*2];
      ret[cnum*2+1] = ret[j*2+1];
      cnum++;
    }
  }
  if(cnum < 1) { *return_code = code; return 0; } // this should never happen

  // we can't generate more contacts than we actually have
  if(maxc > cnum) maxc = cnum;
  if(maxc < 1) maxc = 1;

  if(cnum <= maxc)
  {
    if(code<4)
    {
      // we have less contacts than we need, so we use them all
      for(int j = 0; j < cnum; ++j)
      {
        Vector3<Scalar> pointInWorld = points[j] + (*pa);
        contacts.push_back(ContactPoint<Scalar>(normal, pointInWorld, -dep[j]));
      }
    }
    else
    {
      // we have less contacts than we need, so we use them all
      for(int j = 0; j < cnum; ++j)
      {
        Vector3<Scalar> pointInWorld = points[j] + (*pa) - normal * dep[j];
        contacts.push_back(ContactPoint<Scalar>(normal, pointInWorld, -dep[j]));
      }
    }
  }
  else
  {
    // we have more contacts than are wanted, some of them must be culled.
    // find the deepest point, it is always the first contact.
    int i1 = 0;
    Scalar maxdepth = dep[0];
    for(int i = 1; i < cnum; ++i)
    {
      if(dep[i] > maxdepth)
      {
        maxdepth = dep[i];
        i1 = i;
      }
    }

    int iret[8];
    cullPoints2(cnum, ret, maxc, i1, iret);

    for(int j = 0; j < maxc; ++j)
    {
      Vector3<Scalar> posInWorld = points[iret[j]] + (*pa);
      if(code < 4)
        contacts.push_back(ContactPoint<Scalar>(normal, posInWorld, -dep[iret[j]]));
      else
        contacts.push_back(ContactPoint<Scalar>(normal, posInWorld - normal * dep[iret[j]], -dep[iret[j]]));
    }
    cnum = maxc;
  }

  *return_code = code;
  return cnum;
}

//==============================================================================
template <typename Scalar>
bool boxBoxIntersect(const Box<Scalar>& s1, const Transform3<Scalar>& tf1,
                     const Box<Scalar>& s2, const Transform3<Scalar>& tf2,
                     std::vector<ContactPoint<Scalar>>* contacts_)
{
  std::vector<ContactPoint<Scalar>> contacts;
  int return_code;
  Vector3<Scalar> normal;
  Scalar depth;
  /* int cnum = */ boxBox2(s1.side, tf1.linear(), tf1.translation(),
                           s2.side, tf2.linear(), tf2.translation(),
                           normal, &depth, &return_code,
                           4, contacts);

  if(contacts_)
    *contacts_ = contacts;

  return return_code != 0;
}

//==============================================================================
template <typename Scalar>
Scalar halfspaceIntersectTolerance()
{
  return 0;
}

//==============================================================================
template <>
inline float halfspaceIntersectTolerance()
{
  return 0.0001f;
}

//==============================================================================
template <>
inline double halfspaceIntersectTolerance()
{
  return 0.0000001;
}

//==============================================================================
template <typename Scalar>
bool sphereHalfspaceIntersect(const Sphere<Scalar>& s1, const Transform3<Scalar>& tf1,
                              const Halfspace<Scalar>& s2, const Transform3<Scalar>& tf2,
                              std::vector<ContactPoint<Scalar>>* contacts)
{
  const Halfspace<Scalar> new_s2 = transform(s2, tf2);
  const Vector3<Scalar>& center = tf1.translation();
  const Scalar depth = s1.radius - new_s2.signedDistance(center);

  if (depth >= 0)
  {
    if (contacts)
    {
      const Vector3<Scalar> normal = -new_s2.n; // pointing from s1 to s2
      const Vector3<Scalar> point = center - new_s2.n * s1.radius + new_s2.n * (depth * 0.5);
      const Scalar penetration_depth = depth;

      contacts->push_back(ContactPoint<Scalar>(normal, point, penetration_depth));
    }

    return true;
  }
  else
  {
    return false;
  }
}

//==============================================================================
template <typename Scalar>
bool ellipsoidHalfspaceIntersect(const Ellipsoid<Scalar>& s1, const Transform3<Scalar>& tf1,
                                 const Halfspace<Scalar>& s2, const Transform3<Scalar>& tf2,
                                 std::vector<ContactPoint<Scalar>>* contacts)
{
  // We first compute a single contact in the ellipsoid coordinates, tf1, then
  // will transform it to the world frame. So we use a new halfspace that is
  // expressed in the ellipsoid coordinates.
  const Halfspace<Scalar>& new_s2 = transform(s2, tf1.inverse() * tf2);

  // Compute distance between the ellipsoid's center and a contact plane, whose
  // normal is equal to the halfspace's normal.
  const Vector3<Scalar> normal2(std::pow(new_s2.n[0], 2), std::pow(new_s2.n[1], 2), std::pow(new_s2.n[2], 2));
  const Vector3<Scalar> radii2(std::pow(s1.radii[0], 2), std::pow(s1.radii[1], 2), std::pow(s1.radii[2], 2));
  const Scalar center_to_contact_plane = std::sqrt(normal2.dot(radii2));

  // Depth is the distance between the contact plane and the halfspace.
  const Scalar depth = center_to_contact_plane + new_s2.d;

  if (depth >= 0)
  {
    if (contacts)
    {
      // Transform the results to the world coordinates.
      const Vector3<Scalar> normal = tf1.linear() * -new_s2.n; // pointing from s1 to s2
      const Vector3<Scalar> support_vector = (1.0/center_to_contact_plane) * Vector3<Scalar>(radii2[0]*new_s2.n[0], radii2[1]*new_s2.n[1], radii2[2]*new_s2.n[2]);
      const Vector3<Scalar> point_in_halfspace_coords = support_vector * (0.5 * depth / new_s2.n.dot(support_vector) - 1.0);
      const Vector3<Scalar> point = tf1 * point_in_halfspace_coords; // roughly speaking, a middle point of the intersecting volume
      const Scalar penetration_depth = depth;

      contacts->push_back(ContactPoint<Scalar>(normal, point, penetration_depth));
    }

    return true;
  }
  else
  {
    return false;
  }
}

//==============================================================================
/// @brief box half space, a, b, c  = +/- edge size
/// n^T * (R(o + a v1 + b v2 + c v3) + T) <= d
/// so (R^T n) (a v1 + b v2 + c v3) + n * T <= d
/// check whether d - n * T - (R^T n) (a v1 + b v2 + c v3) >= 0 for some a, b, c
/// the max value of left side is d - n * T + |(R^T n) (a v1 + b v2 + c v3)|, check that is enough
template <typename Scalar>
bool boxHalfspaceIntersect(const Box<Scalar>& s1, const Transform3<Scalar>& tf1,
                           const Halfspace<Scalar>& s2, const Transform3<Scalar>& tf2)
{
  Halfspace<Scalar> new_s2 = transform(s2, tf2);

  const Matrix3<Scalar>& R = tf1.linear();
  const Vector3<Scalar>& T = tf1.translation();

  Vector3<Scalar> Q = R.transpose() * new_s2.n;
  Vector3<Scalar> A(Q[0] * s1.side[0], Q[1] * s1.side[1], Q[2] * s1.side[2]);
  Vector3<Scalar> B = A.cwiseAbs();

  Scalar depth = 0.5 * (B[0] + B[1] + B[2]) - new_s2.signedDistance(T);
  return (depth >= 0);
}

//==============================================================================
template <typename Scalar>
bool boxHalfspaceIntersect(const Box<Scalar>& s1, const Transform3<Scalar>& tf1,
                           const Halfspace<Scalar>& s2, const Transform3<Scalar>& tf2,
                           std::vector<ContactPoint<Scalar>>* contacts)
{
  if(!contacts)
  {
    return boxHalfspaceIntersect(s1, tf1, s2, tf2);
  }
  else
  {
    const Halfspace<Scalar> new_s2 = transform(s2, tf2);

    const Matrix3<Scalar>& R = tf1.linear();
    const Vector3<Scalar>& T = tf1.translation();

    Vector3<Scalar> Q = R.transpose() * new_s2.n;
    Vector3<Scalar> A(Q[0] * s1.side[0], Q[1] * s1.side[1], Q[2] * s1.side[2]);
    Vector3<Scalar> B = A.cwiseAbs();

    Scalar depth = 0.5 * (B[0] + B[1] + B[2]) - new_s2.signedDistance(T);
    if(depth < 0) return false;

    Vector3<Scalar> axis[3];
    axis[0] = R.col(0);
    axis[1] = R.col(1);
    axis[2] = R.col(2);

    /// find deepest point
    Vector3<Scalar> p(T);
    int sign = 0;

    if(std::abs(Q[0] - 1) < halfspaceIntersectTolerance<Scalar>() || std::abs(Q[0] + 1) < halfspaceIntersectTolerance<Scalar>())
    {
      sign = (A[0] > 0) ? -1 : 1;
      p += axis[0] * (0.5 * s1.side[0] * sign);
    }
    else if(std::abs(Q[1] - 1) < halfspaceIntersectTolerance<Scalar>() || std::abs(Q[1] + 1) < halfspaceIntersectTolerance<Scalar>())
    {
      sign = (A[1] > 0) ? -1 : 1;
      p += axis[1] * (0.5 * s1.side[1] * sign);
    }
    else if(std::abs(Q[2] - 1) < halfspaceIntersectTolerance<Scalar>() || std::abs(Q[2] + 1) < halfspaceIntersectTolerance<Scalar>())
    {
      sign = (A[2] > 0) ? -1 : 1;
      p += axis[2] * (0.5 * s1.side[2] * sign);
    }
    else
    {
      for(std::size_t i = 0; i < 3; ++i)
      {
        sign = (A[i] > 0) ? -1 : 1;
        p += axis[i] * (0.5 * s1.side[i] * sign);
      }
    }

    /// compute the contact point from the deepest point
    if (contacts)
    {
      const Vector3<Scalar> normal = -new_s2.n;
      const Vector3<Scalar> point = p + new_s2.n * (depth * 0.5);
      const Scalar penetration_depth = depth;

      contacts->push_back(ContactPoint<Scalar>(normal, point, penetration_depth));
    }

    return true;
  }
}

//==============================================================================
template <typename Scalar>
bool capsuleHalfspaceIntersect(const Capsule<Scalar>& s1, const Transform3<Scalar>& tf1,
                               const Halfspace<Scalar>& s2, const Transform3<Scalar>& tf2,
                               std::vector<ContactPoint<Scalar>>* contacts)
{
  Halfspace<Scalar> new_s2 = transform(s2, tf2);

  const Matrix3<Scalar>& R = tf1.linear();
  const Vector3<Scalar>& T = tf1.translation();

  Vector3<Scalar> dir_z = R.col(2);

  Scalar cosa = dir_z.dot(new_s2.n);
  if(std::abs(cosa) < halfspaceIntersectTolerance<Scalar>())
  {
    Scalar signed_dist = new_s2.signedDistance(T);
    Scalar depth = s1.radius - signed_dist;
    if(depth < 0) return false;

    if (contacts)
    {
      const Vector3<Scalar> normal = -new_s2.n;
      const Vector3<Scalar> point = T + new_s2.n * (0.5 * depth - s1.radius);
      const Scalar penetration_depth = depth;

      contacts->push_back(ContactPoint<Scalar>(normal, point, penetration_depth));
    }

    return true;
  }
  else
  {
    int sign = (cosa > 0) ? -1 : 1;
    Vector3<Scalar> p = T + dir_z * (s1.lz * 0.5 * sign);

    Scalar signed_dist = new_s2.signedDistance(p);
    Scalar depth = s1.radius - signed_dist;
    if(depth < 0) return false;

    if (contacts)
    {
      const Vector3<Scalar> normal = -new_s2.n;
      const Vector3<Scalar> point = p - new_s2.n * s1.radius + new_s2.n * (0.5 * depth);  // deepest point
      const Scalar penetration_depth = depth;

      contacts->push_back(ContactPoint<Scalar>(normal, point, penetration_depth));
    }

    return true;
  }
}

//==============================================================================
template <typename Scalar>
bool cylinderHalfspaceIntersect(const Cylinder<Scalar>& s1, const Transform3<Scalar>& tf1,
                                const Halfspace<Scalar>& s2, const Transform3<Scalar>& tf2,
                                std::vector<ContactPoint<Scalar>>* contacts)
{
  Halfspace<Scalar> new_s2 = transform(s2, tf2);

  const Matrix3<Scalar>& R = tf1.linear();
  const Vector3<Scalar>& T = tf1.translation();

  Vector3<Scalar> dir_z = R.col(2);
  Scalar cosa = dir_z.dot(new_s2.n);

  if(cosa < halfspaceIntersectTolerance<Scalar>())
  {
    Scalar signed_dist = new_s2.signedDistance(T);
    Scalar depth = s1.radius - signed_dist;
    if(depth < 0) return false;

    if (contacts)
    {
      const Vector3<Scalar> normal = -new_s2.n;
      const Vector3<Scalar> point = T + new_s2.n * (0.5 * depth - s1.radius);
      const Scalar penetration_depth = depth;

      contacts->push_back(ContactPoint<Scalar>(normal, point, penetration_depth));
    }

    return true;
  }
  else
  {
    Vector3<Scalar> C = dir_z * cosa - new_s2.n;
    if(std::abs(cosa + 1) < halfspaceIntersectTolerance<Scalar>() || std::abs(cosa - 1) < halfspaceIntersectTolerance<Scalar>())
      C = Vector3<Scalar>(0, 0, 0);
    else
    {
      Scalar s = C.norm();
      s = s1.radius / s;
      C *= s;
    }

    int sign = (cosa > 0) ? -1 : 1;
    // deepest point
    Vector3<Scalar> p = T + dir_z * (s1.lz * 0.5 * sign) + C;
    Scalar depth = -new_s2.signedDistance(p);
    if(depth < 0) return false;
    else
    {
      if (contacts)
      {
        const Vector3<Scalar> normal = -new_s2.n;
        const Vector3<Scalar> point = p + new_s2.n * (0.5 * depth);
        const Scalar penetration_depth = depth;

        contacts->push_back(ContactPoint<Scalar>(normal, point, penetration_depth));
      }

      return true;
    }
  }
}

//==============================================================================
template <typename Scalar>
bool coneHalfspaceIntersect(const Cone<Scalar>& s1, const Transform3<Scalar>& tf1,
                            const Halfspace<Scalar>& s2, const Transform3<Scalar>& tf2,
                            std::vector<ContactPoint<Scalar>>* contacts)
{
  Halfspace<Scalar> new_s2 = transform(s2, tf2);

  const Matrix3<Scalar>& R = tf1.linear();
  const Vector3<Scalar>& T = tf1.translation();

  Vector3<Scalar> dir_z = R.col(2);
  Scalar cosa = dir_z.dot(new_s2.n);

  if(cosa < halfspaceIntersectTolerance<Scalar>())
  {
    Scalar signed_dist = new_s2.signedDistance(T);
    Scalar depth = s1.radius - signed_dist;
    if(depth < 0) return false;
    else
    {
      if (contacts)
      {
        const Vector3<Scalar> normal = -new_s2.n;
        const Vector3<Scalar> point = T - dir_z * (s1.lz * 0.5) + new_s2.n * (0.5 * depth - s1.radius);
        const Scalar penetration_depth = depth;

        contacts->push_back(ContactPoint<Scalar>(normal, point, penetration_depth));
      }

      return true;
    }
  }
  else
  {
    Vector3<Scalar> C = dir_z * cosa - new_s2.n;
    if(std::abs(cosa + 1) < halfspaceIntersectTolerance<Scalar>() || std::abs(cosa - 1) < halfspaceIntersectTolerance<Scalar>())
      C = Vector3<Scalar>(0, 0, 0);
    else
    {
      Scalar s = C.norm();
      s = s1.radius / s;
      C *= s;
    }

    Vector3<Scalar> p1 = T + dir_z * (0.5 * s1.lz);
    Vector3<Scalar> p2 = T - dir_z * (0.5 * s1.lz) + C;

    Scalar d1 = new_s2.signedDistance(p1);
    Scalar d2 = new_s2.signedDistance(p2);

    if(d1 > 0 && d2 > 0) return false;
    else
    {
      if (contacts)
      {
        const Scalar penetration_depth = -std::min(d1, d2);
        const Vector3<Scalar> normal = -new_s2.n;
        const Vector3<Scalar> point = ((d1 < d2) ? p1 : p2) + new_s2.n * (0.5 * penetration_depth);

        contacts->push_back(ContactPoint<Scalar>(normal, point, penetration_depth));
      }

      return true;
    }
  }
}

//==============================================================================
template <typename Scalar>
bool convexHalfspaceIntersect(const Convex<Scalar>& s1, const Transform3<Scalar>& tf1,
                              const Halfspace<Scalar>& s2, const Transform3<Scalar>& tf2,
                              Vector3<Scalar>* contact_points, Scalar* penetration_depth, Vector3<Scalar>* normal)
{
  Halfspace<Scalar> new_s2 = transform(s2, tf2);

  Vector3<Scalar> v;
  Scalar depth = std::numeric_limits<Scalar>::max();

  for(int i = 0; i < s1.num_points; ++i)
  {
    Vector3<Scalar> p = tf1 * s1.points[i];

    Scalar d = new_s2.signedDistance(p);
    if(d < depth)
    {
      depth = d;
      v = p;
    }
  }

  if(depth <= 0)
  {
    if(contact_points) *contact_points = v - new_s2.n * (0.5 * depth);
    if(penetration_depth) *penetration_depth = depth;
    if(normal) *normal = -new_s2.n;
    return true;
  }
  else
    return false;
}

//==============================================================================
template <typename Scalar>
bool halfspaceTriangleIntersect(const Halfspace<Scalar>& s1, const Transform3<Scalar>& tf1,
                                const Vector3<Scalar>& P1, const Vector3<Scalar>& P2, const Vector3<Scalar>& P3, const Transform3<Scalar>& tf2,
                                Vector3<Scalar>* contact_points, Scalar* penetration_depth, Vector3<Scalar>* normal)
{
  Halfspace<Scalar> new_s1 = transform(s1, tf1);

  Vector3<Scalar> v = tf2 * P1;
  Scalar depth = new_s1.signedDistance(v);

  Vector3<Scalar> p = tf2 * P2;
  Scalar d = new_s1.signedDistance(p);
  if(d < depth)
  {
    depth = d;
    v = p;
  }

  p = tf2 * P3;
  d = new_s1.signedDistance(p);
  if(d < depth)
  {
    depth = d;
    v = p;
  }

  if(depth <= 0)
  {
    if(penetration_depth) *penetration_depth = -depth;
    if(normal) *normal = new_s1.n;
    if(contact_points) *contact_points = v - new_s1.n * (0.5 * depth);
    return true;
  }
  else
    return false;
}

//==============================================================================
/// @brief return whether plane collides with halfspace
/// if the separation plane of the halfspace is parallel with the plane
///     return code 1, if the plane's normal is the same with halfspace's normal and plane is inside halfspace, also return plane in pl
///     return code 2, if the plane's normal is oppositie to the halfspace's normal and plane is inside halfspace, also return plane in pl
///     plane is outside halfspace, collision-free
/// if not parallel
///     return the intersection ray, return code 3. ray origin is p and direction is d
template <typename Scalar>
bool planeHalfspaceIntersect(const Plane<Scalar>& s1, const Transform3<Scalar>& tf1,
                             const Halfspace<Scalar>& s2, const Transform3<Scalar>& tf2,
                             Plane<Scalar>& pl,
                             Vector3<Scalar>& p, Vector3<Scalar>& d,
                             Scalar& penetration_depth,
                             int& ret)
{
  Plane<Scalar> new_s1 = transform(s1, tf1);
  Halfspace<Scalar> new_s2 = transform(s2, tf2);

  ret = 0;

  Vector3<Scalar> dir = (new_s1.n).cross(new_s2.n);
  Scalar dir_norm = dir.squaredNorm();
  if(dir_norm < std::numeric_limits<Scalar>::epsilon()) // parallel
  {
    if((new_s1.n).dot(new_s2.n) > 0)
    {
      if(new_s1.d < new_s2.d)
      {
        penetration_depth = new_s2.d - new_s1.d;
        ret = 1;
        pl = new_s1;
        return true;
      }
      else
        return false;
    }
    else
    {
      if(new_s1.d + new_s2.d > 0)
        return false;
      else
      {
        penetration_depth = -(new_s1.d + new_s2.d);
        ret = 2;
        pl = new_s1;
        return true;
      }
    }
  }

  Vector3<Scalar> n = new_s2.n * new_s1.d - new_s1.n * new_s2.d;
  Vector3<Scalar> origin = n.cross(dir);
  origin *= (1.0 / dir_norm);

  p = origin;
  d = dir;
  ret = 3;
  penetration_depth = std::numeric_limits<Scalar>::max();

  return true;
}

//==============================================================================
///@ brief return whether two halfspace intersect
/// if the separation planes of the two halfspaces are parallel
///    return code 1, if two halfspaces' normal are same and s1 is in s2, also return s1 in s;
///    return code 2, if two halfspaces' normal are same and s2 is in s1, also return s2 in s;
///    return code 3, if two halfspaces' normal are opposite and s1 and s2 are into each other;
///    collision free, if two halfspaces' are separate;
/// if the separation planes of the two halfspaces are not parallel, return intersection ray, return code 4. ray origin is p and direction is d
/// collision free return code 0
template <typename Scalar>
bool halfspaceIntersect(const Halfspace<Scalar>& s1, const Transform3<Scalar>& tf1,
                        const Halfspace<Scalar>& s2, const Transform3<Scalar>& tf2,
                        Vector3<Scalar>& p, Vector3<Scalar>& d,
                        Halfspace<Scalar>& s,
                        Scalar& penetration_depth,
                        int& ret)
{
  Halfspace<Scalar> new_s1 = transform(s1, tf1);
  Halfspace<Scalar> new_s2 = transform(s2, tf2);

  ret = 0;

  Vector3<Scalar> dir = (new_s1.n).cross(new_s2.n);
  Scalar dir_norm = dir.squaredNorm();
  if(dir_norm < std::numeric_limits<Scalar>::epsilon()) // parallel
  {
    if((new_s1.n).dot(new_s2.n) > 0)
    {
      if(new_s1.d < new_s2.d) // s1 is inside s2
      {
        ret = 1;
        penetration_depth = std::numeric_limits<Scalar>::max();
        s = new_s1;
      }
      else // s2 is inside s1
      {
        ret = 2;
        penetration_depth = std::numeric_limits<Scalar>::max();
        s = new_s2;
      }
      return true;
    }
    else
    {
      if(new_s1.d + new_s2.d > 0) // not collision
        return false;
      else // in each other
      {
        ret = 3;
        penetration_depth = -(new_s1.d + new_s2.d);
        return true;
      }
    }
  }

  Vector3<Scalar> n = new_s2.n * new_s1.d - new_s1.n * new_s2.d;
  Vector3<Scalar> origin = n.cross(dir);
  origin *= (1.0 / dir_norm);

  p = origin;
  d = dir;
  ret = 4;
  penetration_depth = std::numeric_limits<Scalar>::max();

  return true;
}

//==============================================================================
template <typename Scalar>
Scalar planeIntersectTolerance()
{
  return 0;
}

//==============================================================================
template <>
inline double planeIntersectTolerance()
{
  return 0.0000001;
}

//==============================================================================
template <>
inline float planeIntersectTolerance()
{
  return 0.0001;
}

//==============================================================================
template <typename Scalar>
bool spherePlaneIntersect(const Sphere<Scalar>& s1, const Transform3<Scalar>& tf1,
                          const Plane<Scalar>& s2, const Transform3<Scalar>& tf2,
                          std::vector<ContactPoint<Scalar>>* contacts)
{
  const Plane<Scalar> new_s2 = transform(s2, tf2);

  const Vector3<Scalar>& center = tf1.translation();
  const Scalar signed_dist = new_s2.signedDistance(center);
  const Scalar depth = - std::abs(signed_dist) + s1.radius;

  if(depth >= 0)
  {
    if (contacts)
    {
      const Vector3<Scalar> normal = (signed_dist > 0) ? (-new_s2.n).eval() : new_s2.n;
      const Vector3<Scalar> point = center - new_s2.n * signed_dist;
      const Scalar penetration_depth = depth;

      contacts->push_back(ContactPoint<Scalar>(normal, point, penetration_depth));
    }

    return true;
  }
  else
  {
    return false;
  }
}

//==============================================================================
template <typename Scalar>
bool ellipsoidPlaneIntersect(const Ellipsoid<Scalar>& s1, const Transform3<Scalar>& tf1,
                             const Plane<Scalar>& s2, const Transform3<Scalar>& tf2,
                             std::vector<ContactPoint<Scalar>>* contacts)
{
  // We first compute a single contact in the ellipsoid coordinates, tf1, then
  // will transform it to the world frame. So we use a new plane that is
  // expressed in the ellipsoid coordinates.
  const Plane<Scalar>& new_s2 = transform(s2, tf1.inverse() * tf2);

  // Compute distance between the ellipsoid's center and a contact plane, whose
  // normal is equal to the plane's normal.
  const Vector3<Scalar> normal2(std::pow(new_s2.n[0], 2), std::pow(new_s2.n[1], 2), std::pow(new_s2.n[2], 2));
  const Vector3<Scalar> radii2(std::pow(s1.radii[0], 2), std::pow(s1.radii[1], 2), std::pow(s1.radii[2], 2));
  const Scalar center_to_contact_plane = std::sqrt(normal2.dot(radii2));

  const Scalar signed_dist = -new_s2.d;

  // Depth is the distance between the contact plane and the given plane.
  const Scalar depth = center_to_contact_plane - std::abs(signed_dist);

  if (depth >= 0)
  {
    if (contacts)
    {
      // Transform the results to the world coordinates.
      const Vector3<Scalar> normal = (signed_dist > 0) ? (tf1.linear() * -new_s2.n).eval() : (tf1.linear() * new_s2.n).eval(); // pointing from the ellipsoid's center to the plane
      const Vector3<Scalar> support_vector = (1.0/center_to_contact_plane) * Vector3<Scalar>(radii2[0]*new_s2.n[0], radii2[1]*new_s2.n[1], radii2[2]*new_s2.n[2]);
      const Vector3<Scalar> point_in_plane_coords = support_vector * (depth / new_s2.n.dot(support_vector) - 1.0);
      const Vector3<Scalar> point = (signed_dist > 0) ? tf1 * point_in_plane_coords : tf1 * -point_in_plane_coords; // a middle point of the intersecting volume
      const Scalar penetration_depth = depth;

      contacts->push_back(ContactPoint<Scalar>(normal, point, penetration_depth));
    }

    return true;
  }
  else
  {
    return false;
  }
}

//==============================================================================
/// @brief box half space, a, b, c  = +/- edge size
/// n^T * (R(o + a v1 + b v2 + c v3) + T) ~ d
/// so (R^T n) (a v1 + b v2 + c v3) + n * T ~ d
/// check whether d - n * T - (R^T n) (a v1 + b v2 + c v3) >= 0 for some a, b, c and <=0 for some a, b, c
/// so need to check whether |d - n * T| <= |(R^T n)(a v1 + b v2 + c v3)|, the reason is as follows:
/// (R^T n) (a v1 + b v2 + c v3) can get |(R^T n) (a v1 + b v2 + c v3)| for one a, b, c.
/// if |d - n * T| <= |(R^T n)(a v1 + b v2 + c v3)| then can get both positive and negative value on the right side.
template <typename Scalar>
bool boxPlaneIntersect(const Box<Scalar>& s1, const Transform3<Scalar>& tf1,
                       const Plane<Scalar>& s2, const Transform3<Scalar>& tf2,
                       std::vector<ContactPoint<Scalar>>* contacts)
{
  Plane<Scalar> new_s2 = transform(s2, tf2);

  const Matrix3<Scalar>& R = tf1.linear();
  const Vector3<Scalar>& T = tf1.translation();

  Vector3<Scalar> Q = R.transpose() * new_s2.n;
  Vector3<Scalar> A(Q[0] * s1.side[0], Q[1] * s1.side[1], Q[2] * s1.side[2]);
  Vector3<Scalar> B = A.cwiseAbs();

  Scalar signed_dist = new_s2.signedDistance(T);
  Scalar depth = 0.5 * (B[0] + B[1] + B[2]) - std::abs(signed_dist);
  if(depth < 0) return false;

  Vector3<Scalar> axis[3];
  axis[0] = R.col(0);
  axis[1] = R.col(1);
  axis[2] = R.col(2);

  // find the deepest point
  Vector3<Scalar> p = T;

  // when center is on the positive side of the plane, use a, b, c make (R^T n) (a v1 + b v2 + c v3) the minimum
  // otherwise, use a, b, c make (R^T n) (a v1 + b v2 + c v3) the maximum
  int sign = (signed_dist > 0) ? 1 : -1;

  if(std::abs(Q[0] - 1) < planeIntersectTolerance<Scalar>() || std::abs(Q[0] + 1) < planeIntersectTolerance<Scalar>())
  {
    int sign2 = (A[0] > 0) ? -1 : 1;
    sign2 *= sign;
    p += axis[0] * (0.5 * s1.side[0] * sign2);
  }
  else if(std::abs(Q[1] - 1) < planeIntersectTolerance<Scalar>() || std::abs(Q[1] + 1) < planeIntersectTolerance<Scalar>())
  {
    int sign2 = (A[1] > 0) ? -1 : 1;
    sign2 *= sign;
    p += axis[1] * (0.5 * s1.side[1] * sign2);
  }
  else if(std::abs(Q[2] - 1) < planeIntersectTolerance<Scalar>() || std::abs(Q[2] + 1) < planeIntersectTolerance<Scalar>())
  {
    int sign2 = (A[2] > 0) ? -1 : 1;
    sign2 *= sign;
    p += axis[2] * (0.5 * s1.side[2] * sign2);
  }
  else
  {
    for(std::size_t i = 0; i < 3; ++i)
    {
      int sign2 = (A[i] > 0) ? -1 : 1;
      sign2 *= sign;
      p += axis[i] * (0.5 * s1.side[i] * sign2);
    }
  }

  // compute the contact point by project the deepest point onto the plane
  if (contacts)
  {
    const Vector3<Scalar> normal = (signed_dist > 0) ? (-new_s2.n).eval() : new_s2.n;
    const Vector3<Scalar> point = p - new_s2.n * new_s2.signedDistance(p);
    const Scalar penetration_depth = depth;

    contacts->push_back(ContactPoint<Scalar>(normal, point, penetration_depth));
  }

  return true;
}

//==============================================================================
template <typename Scalar>
bool capsulePlaneIntersect(const Capsule<Scalar>& s1, const Transform3<Scalar>& tf1,
                           const Plane<Scalar>& s2, const Transform3<Scalar>& tf2)
{
  Plane<Scalar> new_s2 = transform(s2, tf2);

  const Matrix3<Scalar>& R = tf1.linear();
  const Vector3<Scalar>& T = tf1.translation();

  Vector3<Scalar> dir_z = R.col(2);
  Vector3<Scalar> p1 = T + dir_z * (0.5 * s1.lz);
  Vector3<Scalar> p2 = T - dir_z * (0.5 * s1.lz);

  Scalar d1 = new_s2.signedDistance(p1);
  Scalar d2 = new_s2.signedDistance(p2);

  // two end points on different side of the plane
  if(d1 * d2 <= 0)
    return true;

  // two end points on the same side of the plane, but the end point spheres might intersect the plane
  return (std::abs(d1) <= s1.radius) || (std::abs(d2) <= s1.radius);
}

//==============================================================================
template <typename Scalar>
bool capsulePlaneIntersect(const Capsule<Scalar>& s1, const Transform3<Scalar>& tf1,
                           const Plane<Scalar>& s2, const Transform3<Scalar>& tf2,
                           std::vector<ContactPoint<Scalar>>* contacts)
{
  if(!contacts)
  {
    return capsulePlaneIntersect(s1, tf1, s2, tf2);
  }
  else
  {
    Plane<Scalar> new_s2 = transform(s2, tf2);

    const Matrix3<Scalar>& R = tf1.linear();
    const Vector3<Scalar>& T = tf1.translation();

    Vector3<Scalar> dir_z = R.col(2);


    Vector3<Scalar> p1 = T + dir_z * (0.5 * s1.lz);
    Vector3<Scalar> p2 = T - dir_z * (0.5 * s1.lz);

    Scalar d1 = new_s2.signedDistance(p1);
    Scalar d2 = new_s2.signedDistance(p2);

    Scalar abs_d1 = std::abs(d1);
    Scalar abs_d2 = std::abs(d2);

    // two end points on different side of the plane
    // the contact point is the intersect of axis with the plane
    // the normal is the direction to avoid intersection
    // the depth is the minimum distance to resolve the collision
    if(d1 * d2 < -planeIntersectTolerance<Scalar>())
    {
      if(abs_d1 < abs_d2)
      {
        if (contacts)
        {
          const Vector3<Scalar> normal = (d1 < 0) ? (-new_s2.n).eval() : new_s2.n;
          const Vector3<Scalar> point = p1 * (abs_d2 / (abs_d1 + abs_d2)) + p2 * (abs_d1 / (abs_d1 + abs_d2));
          const Scalar penetration_depth = abs_d1 + s1.radius;

          contacts->push_back(ContactPoint<Scalar>(normal, point, penetration_depth));
        }
      }
      else
      {
        if (contacts)
        {
          const Vector3<Scalar> normal = (d2 < 0) ? (-new_s2.n).eval() : new_s2.n;
          const Vector3<Scalar> point = p1 * (abs_d2 / (abs_d1 + abs_d2)) + p2 * (abs_d1 / (abs_d1 + abs_d2));
          const Scalar penetration_depth = abs_d2 + s1.radius;

          contacts->push_back(ContactPoint<Scalar>(normal, point, penetration_depth));
        }
      }
      return true;
    }

    if(abs_d1 > s1.radius && abs_d2 > s1.radius)
    {
      return false;
    }
    else
    {
      if (contacts)
      {
        const Vector3<Scalar> normal = (d1 < 0) ? new_s2.n : (-new_s2.n).eval();
        const Scalar penetration_depth = s1.radius - std::min(abs_d1, abs_d2);
        Vector3<Scalar> point;
        if(abs_d1 <= s1.radius && abs_d2 <= s1.radius)
        {
          const Vector3<Scalar> c1 = p1 - new_s2.n * d2;
          const Vector3<Scalar> c2 = p2 - new_s2.n * d1;
          point = (c1 + c2) * 0.5;
        }
        else if(abs_d1 <= s1.radius)
        {
          const Vector3<Scalar> c = p1 - new_s2.n * d1;
          point = c;
        }
        else if(abs_d2 <= s1.radius)
        {
          const Vector3<Scalar> c = p2 - new_s2.n * d2;
          point = c;
        }

        contacts->push_back(ContactPoint<Scalar>(normal, point, penetration_depth));
      }

      return true;
    }
  }
}

//==============================================================================
/// @brief cylinder-plane intersect
/// n^T (R (r * cosa * v1 + r * sina * v2 + h * v3) + T) ~ d
/// need one point to be positive and one to be negative
/// (n^T * v3) * h + n * T -d + r * (cosa * (n^T * R * v1) + sina * (n^T * R * v2)) ~ 0
/// (n^T * v3) * h + r * (cosa * (n^T * R * v1) + sina * (n^T * R * v2)) + n * T - d ~ 0
template <typename Scalar>
bool cylinderPlaneIntersect(const Cylinder<Scalar>& s1, const Transform3<Scalar>& tf1,
                            const Plane<Scalar>& s2, const Transform3<Scalar>& tf2)
{
  Plane<Scalar> new_s2 = transform(s2, tf2);

  const Matrix3<Scalar>& R = tf1.linear();
  const Vector3<Scalar>& T = tf1.translation();

  Vector3<Scalar> Q = R.transpose() * new_s2.n;

  Scalar term = std::abs(Q[2]) * s1.lz + s1.radius * std::sqrt(Q[0] * Q[0] + Q[1] * Q[1]);
  Scalar dist = new_s2.distance(T);
  Scalar depth = term - dist;

  if(depth < 0)
    return false;
  else
    return true;
}

//==============================================================================
template <typename Scalar>
bool cylinderPlaneIntersect(const Cylinder<Scalar>& s1, const Transform3<Scalar>& tf1,
                            const Plane<Scalar>& s2, const Transform3<Scalar>& tf2,
                            std::vector<ContactPoint<Scalar>>* contacts)
{
  if(!contacts)
  {
    return cylinderPlaneIntersect(s1, tf1, s2, tf2);
  }
  else
  {
    Plane<Scalar> new_s2 = transform(s2, tf2);

    const Matrix3<Scalar>& R = tf1.linear();
    const Vector3<Scalar>& T = tf1.translation();

    Vector3<Scalar> dir_z = R.col(2);
    Scalar cosa = dir_z.dot(new_s2.n);

    if(std::abs(cosa) < planeIntersectTolerance<Scalar>())
    {
      Scalar d = new_s2.signedDistance(T);
      Scalar depth = s1.radius - std::abs(d);
      if(depth < 0) return false;
      else
      {
        if (contacts)
        {
          const Vector3<Scalar> normal = (d < 0) ? new_s2.n : (-new_s2.n).eval();
          const Vector3<Scalar> point = T - new_s2.n * d;
          const Scalar penetration_depth = depth;

          contacts->push_back(ContactPoint<Scalar>(normal, point, penetration_depth));
        }
        return true;
      }
    }
    else
    {
      Vector3<Scalar> C = dir_z * cosa - new_s2.n;
      if(std::abs(cosa + 1) < planeIntersectTolerance<Scalar>() || std::abs(cosa - 1) < planeIntersectTolerance<Scalar>())
        C = Vector3<Scalar>(0, 0, 0);
      else
      {
        Scalar s = C.norm();
        s = s1.radius / s;
        C *= s;
      }

      Vector3<Scalar> p1 = T + dir_z * (0.5 * s1.lz);
      Vector3<Scalar> p2 = T - dir_z * (0.5 * s1.lz);

      Vector3<Scalar> c1, c2;
      if(cosa > 0)
      {
        c1 = p1 - C;
        c2 = p2 + C;
      }
      else
      {
        c1 = p1 + C;
        c2 = p2 - C;
      }

      Scalar d1 = new_s2.signedDistance(c1);
      Scalar d2 = new_s2.signedDistance(c2);

      if(d1 * d2 <= 0)
      {
        Scalar abs_d1 = std::abs(d1);
        Scalar abs_d2 = std::abs(d2);

        if(abs_d1 > abs_d2)
        {
          if (contacts)
          {
            const Vector3<Scalar> normal = (d2 < 0) ? (-new_s2.n).eval() : new_s2.n;
            const Vector3<Scalar> point = c2 - new_s2.n * d2;
            const Scalar penetration_depth = abs_d2;

            contacts->push_back(ContactPoint<Scalar>(normal, point, penetration_depth));
          }
        }
        else
        {
          if (contacts)
          {
            const Vector3<Scalar> normal = (d1 < 0) ? (-new_s2.n).eval() : new_s2.n;
            const Vector3<Scalar> point = c1 - new_s2.n * d1;
            const Scalar penetration_depth = abs_d1;

            contacts->push_back(ContactPoint<Scalar>(normal, point, penetration_depth));
          }
        }
        return true;
      }
      else
      {
        return false;
      }
    }
  }
}

//==============================================================================
template <typename Scalar>
bool conePlaneIntersect(const Cone<Scalar>& s1, const Transform3<Scalar>& tf1,
                        const Plane<Scalar>& s2, const Transform3<Scalar>& tf2,
                        std::vector<ContactPoint<Scalar>>* contacts)
{
  Plane<Scalar> new_s2 = transform(s2, tf2);

  const Matrix3<Scalar>& R = tf1.linear();
  const Vector3<Scalar>& T = tf1.translation();

  Vector3<Scalar> dir_z = R.col(2);
  Scalar cosa = dir_z.dot(new_s2.n);

  if(std::abs(cosa) < planeIntersectTolerance<Scalar>())
  {
    Scalar d = new_s2.signedDistance(T);
    Scalar depth = s1.radius - std::abs(d);
    if(depth < 0) return false;
    else
    {
      if (contacts)
      {
        const Vector3<Scalar> normal = (d < 0) ? new_s2.n : (-new_s2.n).eval();
        const Vector3<Scalar> point = T - dir_z * (0.5 * s1.lz) + dir_z * (0.5 * depth / s1.radius * s1.lz) - new_s2.n * d;
        const Scalar penetration_depth = depth;

        contacts->push_back(ContactPoint<Scalar>(normal, point, penetration_depth));
      }

      return true;
    }
  }
  else
  {
    Vector3<Scalar> C = dir_z * cosa - new_s2.n;
    if(std::abs(cosa + 1) < planeIntersectTolerance<Scalar>() || std::abs(cosa - 1) < planeIntersectTolerance<Scalar>())
      C = Vector3<Scalar>(0, 0, 0);
    else
    {
      Scalar s = C.norm();
      s = s1.radius / s;
      C *= s;
    }

    Vector3<Scalar> c[3];
    c[0] = T + dir_z * (0.5 * s1.lz);
    c[1] = T - dir_z * (0.5 * s1.lz) + C;
    c[2] = T - dir_z * (0.5 * s1.lz) - C;

    Scalar d[3];
    d[0] = new_s2.signedDistance(c[0]);
    d[1] = new_s2.signedDistance(c[1]);
    d[2] = new_s2.signedDistance(c[2]);

    if((d[0] >= 0 && d[1] >= 0 && d[2] >= 0) || (d[0] <= 0 && d[1] <= 0 && d[2] <= 0))
      return false;
    else
    {
      bool positive[3];
      for(std::size_t i = 0; i < 3; ++i)
        positive[i] = (d[i] >= 0);

      int n_positive = 0;
      Scalar d_positive = 0, d_negative = 0;
      for(std::size_t i = 0; i < 3; ++i)
      {
        if(positive[i])
        {
          n_positive++;
          if(d_positive <= d[i]) d_positive = d[i];
        }
        else
        {
          if(d_negative <= -d[i]) d_negative = -d[i];
        }
      }

      if (contacts)
      {
        const Vector3<Scalar> normal = (d_positive > d_negative) ? (-new_s2.n).eval() : new_s2.n;
        const Scalar penetration_depth = std::min(d_positive, d_negative);

        Vector3<Scalar> point;
        Vector3<Scalar> p[2];
        Vector3<Scalar> q;

        Scalar p_d[2];
        Scalar q_d(0);

        if(n_positive == 2)
        {
          for(std::size_t i = 0, j = 0; i < 3; ++i)
          {
            if(positive[i]) { p[j] = c[i]; p_d[j] = d[i]; j++; }
            else { q = c[i]; q_d = d[i]; }
          }

          const Vector3<Scalar> t1 = (-p[0] * q_d + q * p_d[0]) / (-q_d + p_d[0]);
          const Vector3<Scalar> t2 = (-p[1] * q_d + q * p_d[1]) / (-q_d + p_d[1]);
          point = (t1 + t2) * 0.5;
        }
        else
        {
          for(std::size_t i = 0, j = 0; i < 3; ++i)
          {
            if(!positive[i]) { p[j] = c[i]; p_d[j] = d[i]; j++; }
            else { q = c[i]; q_d = d[i]; }
          }

          const Vector3<Scalar> t1 = (p[0] * q_d - q * p_d[0]) / (q_d - p_d[0]);
          const Vector3<Scalar> t2 = (p[1] * q_d - q * p_d[1]) / (q_d - p_d[1]);
          point = (t1 + t2) * 0.5;
        }

        contacts->push_back(ContactPoint<Scalar>(normal, point, penetration_depth));
      }

      return true;
    }
  }
}

//==============================================================================
template <typename Scalar>
bool convexPlaneIntersect(const Convex<Scalar>& s1, const Transform3<Scalar>& tf1,
                          const Plane<Scalar>& s2, const Transform3<Scalar>& tf2,
                          Vector3<Scalar>* contact_points, Scalar* penetration_depth, Vector3<Scalar>* normal)
{
  Plane<Scalar> new_s2 = transform(s2, tf2);

  Vector3<Scalar> v_min, v_max;
  Scalar d_min = std::numeric_limits<Scalar>::max(), d_max = -std::numeric_limits<Scalar>::max();

  for(int i = 0; i < s1.num_points; ++i)
  {
    Vector3<Scalar> p = tf1 * s1.points[i];

    Scalar d = new_s2.signedDistance(p);

    if(d < d_min) { d_min = d; v_min = p; }
    if(d > d_max) { d_max = d; v_max = p; }
  }

  if(d_min * d_max > 0) return false;
  else
  {
    if(d_min + d_max > 0)
    {
      if(penetration_depth) *penetration_depth = -d_min;
      if(normal) *normal = -new_s2.n;
      if(contact_points) *contact_points = v_min - new_s2.n * d_min;
    }
    else
    {
      if(penetration_depth) *penetration_depth = d_max;
      if(normal) *normal = new_s2.n;
      if(contact_points) *contact_points = v_max - new_s2.n * d_max;
    }
    return true;
  }
}

//==============================================================================
template <typename Scalar>
bool planeTriangleIntersect(const Plane<Scalar>& s1, const Transform3<Scalar>& tf1,
                            const Vector3<Scalar>& P1, const Vector3<Scalar>& P2, const Vector3<Scalar>& P3, const Transform3<Scalar>& tf2,
                            Vector3<Scalar>* contact_points, Scalar* penetration_depth, Vector3<Scalar>* normal)
{
  Plane<Scalar> new_s1 = transform(s1, tf1);

  Vector3<Scalar> c[3];
  c[0] = tf2 * P1;
  c[1] = tf2 * P2;
  c[2] = tf2 * P3;

  Scalar d[3];
  d[0] = new_s1.signedDistance(c[0]);
  d[1] = new_s1.signedDistance(c[1]);
  d[2] = new_s1.signedDistance(c[2]);

  if((d[0] >= 0 && d[1] >= 0 && d[2] >= 0) || (d[0] <= 0 && d[1] <= 0 && d[2] <= 0))
    return false;
  else
  {
    bool positive[3];
    for(std::size_t i = 0; i < 3; ++i)
      positive[i] = (d[i] > 0);

    int n_positive = 0;
    Scalar d_positive = 0, d_negative = 0;
    for(std::size_t i = 0; i < 3; ++i)
    {
      if(positive[i])
      {
        n_positive++;
        if(d_positive <= d[i]) d_positive = d[i];
      }
      else
      {
        if(d_negative <= -d[i]) d_negative = -d[i];
      }
    }

    if(penetration_depth) *penetration_depth = std::min(d_positive, d_negative);
    if(normal) *normal = (d_positive > d_negative) ? new_s1.n : (-new_s1.n).eval();
    if(contact_points)
    {
      Vector3<Scalar> p[2] = {Vector3<Scalar>::Zero(), Vector3<Scalar>::Zero()};
      Vector3<Scalar> q = Vector3<Scalar>::Zero();

      Scalar p_d[2];
      Scalar q_d(0);

      if(n_positive == 2)
      {
        for(std::size_t i = 0, j = 0; i < 3; ++i)
        {
          if(positive[i]) { p[j] = c[i]; p_d[j] = d[i]; j++; }
          else { q = c[i]; q_d = d[i]; }
        }

        Vector3<Scalar> t1 = (-p[0] * q_d + q * p_d[0]) / (-q_d + p_d[0]);
        Vector3<Scalar> t2 = (-p[1] * q_d + q * p_d[1]) / (-q_d + p_d[1]);
        *contact_points = (t1 + t2) * 0.5;
      }
      else
      {
        for(std::size_t i = 0, j = 0; i < 3; ++i)
        {
          if(!positive[i]) { p[j] = c[i]; p_d[j] = d[i]; j++; }
          else { q = c[i]; q_d = d[i]; }
        }

        Vector3<Scalar> t1 = (p[0] * q_d - q * p_d[0]) / (q_d - p_d[0]);
        Vector3<Scalar> t2 = (p[1] * q_d - q * p_d[1]) / (q_d - p_d[1]);
        *contact_points = (t1 + t2) * 0.5;
      }
    }
    return true;
  }
}

//==============================================================================
template <typename Scalar>
bool halfspacePlaneIntersect(const Halfspace<Scalar>& s1, const Transform3<Scalar>& tf1,
                             const Plane<Scalar>& s2, const Transform3<Scalar>& tf2,
                             Plane<Scalar>& pl, Vector3<Scalar>& p, Vector3<Scalar>& d,
                             Scalar& penetration_depth,
                             int& ret)
{
  return planeHalfspaceIntersect(s2, tf2, s1, tf1, pl, p, d, penetration_depth, ret);
}

//==============================================================================
template <typename Scalar>
bool planeIntersect(const Plane<Scalar>& s1, const Transform3<Scalar>& tf1,
                    const Plane<Scalar>& s2, const Transform3<Scalar>& tf2,
                    std::vector<ContactPoint<Scalar>>* /*contacts*/)
{
  Plane<Scalar> new_s1 = transform(s1, tf1);
  Plane<Scalar> new_s2 = transform(s2, tf2);

  Scalar a = (new_s1.n).dot(new_s2.n);
  if(a == 1 && new_s1.d != new_s2.d)
    return false;
  if(a == -1 && new_s1.d != -new_s2.d)
    return false;

  return true;
}

} // details

} // namespace fcl

#endif

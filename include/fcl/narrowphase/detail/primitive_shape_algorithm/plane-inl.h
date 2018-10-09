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

#ifndef FCL_NARROWPHASE_DETAIL_PLANE_INL_H
#define FCL_NARROWPHASE_DETAIL_PLANE_INL_H

#include "fcl/narrowphase/detail/primitive_shape_algorithm/plane.h"

namespace fcl
{

namespace detail
{

//==============================================================================
extern template
bool spherePlaneIntersect(const Sphere<double>& s1, const Transform3<double>& tf1,
                          const Plane<double>& s2, const Transform3<double>& tf2,
                          std::vector<ContactPoint<double>>* contacts);

//==============================================================================
extern template
bool ellipsoidPlaneIntersect(const Ellipsoid<double>& s1, const Transform3<double>& tf1,
                             const Plane<double>& s2, const Transform3<double>& tf2,
                             std::vector<ContactPoint<double>>* contacts);

//==============================================================================
extern template
bool boxPlaneIntersect(const Box<double>& s1, const Transform3<double>& tf1,
                       const Plane<double>& s2, const Transform3<double>& tf2,
                       std::vector<ContactPoint<double>>* contacts);

//==============================================================================
extern template
bool capsulePlaneIntersect(const Capsule<double>& s1, const Transform3<double>& tf1,
                           const Plane<double>& s2, const Transform3<double>& tf2);

//==============================================================================
extern template
bool capsulePlaneIntersect(const Capsule<double>& s1, const Transform3<double>& tf1,
                           const Plane<double>& s2, const Transform3<double>& tf2,
                           std::vector<ContactPoint<double>>* contacts);

//==============================================================================
extern template
bool cylinderPlaneIntersect(const Cylinder<double>& s1, const Transform3<double>& tf1,
                            const Plane<double>& s2, const Transform3<double>& tf2);

//==============================================================================
extern template
bool cylinderPlaneIntersect(const Cylinder<double>& s1, const Transform3<double>& tf1,
                            const Plane<double>& s2, const Transform3<double>& tf2,
                            std::vector<ContactPoint<double>>* contacts);

//==============================================================================
extern template
bool conePlaneIntersect(const Cone<double>& s1, const Transform3<double>& tf1,
                        const Plane<double>& s2, const Transform3<double>& tf2,
                        std::vector<ContactPoint<double>>* contacts);

//==============================================================================
extern template
bool convexPlaneIntersect(const Convex<double>& s1, const Transform3<double>& tf1,
                          const Plane<double>& s2, const Transform3<double>& tf2,
                          Vector3<double>* contact_points, double* penetration_depth, Vector3<double>* normal);

//==============================================================================
extern template
bool planeTriangleIntersect(const Plane<double>& s1, const Transform3<double>& tf1,
                            const Vector3<double>& P1, const Vector3<double>& P2, const Vector3<double>& P3, const Transform3<double>& tf2,
                            Vector3<double>* contact_points, double* penetration_depth, Vector3<double>* normal);

//==============================================================================
extern template
bool planeIntersect(const Plane<double>& s1, const Transform3<double>& tf1,
                    const Plane<double>& s2, const Transform3<double>& tf2,
                    std::vector<ContactPoint<double>>* contacts);

//==============================================================================
template <typename S>
S planeIntersectTolerance()
{
  return 0;
}

//==============================================================================
template <typename S>
bool spherePlaneIntersect(const Sphere<S>& s1, const Transform3<S>& tf1,
                          const Plane<S>& s2, const Transform3<S>& tf2,
                          std::vector<ContactPoint<S>>* contacts)
{
  const Plane<S> new_s2 = transform(s2, tf2);

  const Vector3<S>& center = tf1.translation();
  const S signed_dist = new_s2.signedDistance(center);
  const S depth = - std::abs(signed_dist) + s1.radius;

  if(depth >= 0)
  {
    if (contacts)
    {
      const Vector3<S> normal = (signed_dist > 0) ? (-new_s2.n).eval() : new_s2.n;
      const Vector3<S> point = center - new_s2.n * signed_dist;
      const S penetration_depth = depth;

      contacts->emplace_back(normal, point, penetration_depth);
    }

    return true;
  }
  else
  {
    return false;
  }
}

//==============================================================================
template <typename S>
bool ellipsoidPlaneIntersect(const Ellipsoid<S>& s1, const Transform3<S>& tf1,
                             const Plane<S>& s2, const Transform3<S>& tf2,
                             std::vector<ContactPoint<S>>* contacts)
{
  // We first compute a single contact in the ellipsoid coordinates, tf1, then
  // will transform it to the world frame. So we use a new plane that is
  // expressed in the ellipsoid coordinates.
  const Plane<S>& new_s2 = transform(s2, tf1.inverse(Eigen::Isometry) * tf2);

  // Compute distance between the ellipsoid's center and a contact plane, whose
  // normal is equal to the plane's normal.
  const Vector3<S> normal2(std::pow(new_s2.n[0], 2), std::pow(new_s2.n[1], 2), std::pow(new_s2.n[2], 2));
  const Vector3<S> radii2(std::pow(s1.radii[0], 2), std::pow(s1.radii[1], 2), std::pow(s1.radii[2], 2));
  const S center_to_contact_plane = std::sqrt(normal2.dot(radii2));

  const S signed_dist = -new_s2.d;

  // Depth is the distance between the contact plane and the given plane.
  const S depth = center_to_contact_plane - std::abs(signed_dist);

  if (depth >= 0)
  {
    if (contacts)
    {
      // Transform the results to the world coordinates.
      const Vector3<S> normal = (signed_dist > 0) ? (tf1.linear() * -new_s2.n).eval() : (tf1.linear() * new_s2.n).eval(); // pointing from the ellipsoid's center to the plane
      const Vector3<S> support_vector = (1.0/center_to_contact_plane) * Vector3<S>(radii2[0]*new_s2.n[0], radii2[1]*new_s2.n[1], radii2[2]*new_s2.n[2]);
      const Vector3<S> point_in_plane_coords = support_vector * (depth / new_s2.n.dot(support_vector) - 1.0);
      const Vector3<S> point = (signed_dist > 0) ? tf1 * point_in_plane_coords : tf1 * -point_in_plane_coords; // a middle point of the intersecting volume
      const S penetration_depth = depth;

      contacts->emplace_back(normal, point, penetration_depth);
    }

    return true;
  }
  else
  {
    return false;
  }
}

//==============================================================================
template <typename S>
bool boxPlaneIntersect(const Box<S>& s1, const Transform3<S>& tf1,
                       const Plane<S>& s2, const Transform3<S>& tf2,
                       std::vector<ContactPoint<S>>* contacts)
{
  Plane<S> new_s2 = transform(s2, tf2);

  const Matrix3<S>& R = tf1.linear();
  const Vector3<S>& T = tf1.translation();

  Vector3<S> Q = R.transpose() * new_s2.n;
  Vector3<S> A(Q[0] * s1.side[0], Q[1] * s1.side[1], Q[2] * s1.side[2]);
  Vector3<S> B = A.cwiseAbs();

  S signed_dist = new_s2.signedDistance(T);
  S depth = 0.5 * (B[0] + B[1] + B[2]) - std::abs(signed_dist);
  if(depth < 0) return false;

  Vector3<S> axis[3];
  axis[0] = R.col(0);
  axis[1] = R.col(1);
  axis[2] = R.col(2);

  // find the deepest point
  Vector3<S> p = T;

  // when center is on the positive side of the plane, use a, b, c make (R^T n) (a v1 + b v2 + c v3) the minimum
  // otherwise, use a, b, c make (R^T n) (a v1 + b v2 + c v3) the maximum
  int sign = (signed_dist > 0) ? 1 : -1;

  if(std::abs(Q[0] - 1) < planeIntersectTolerance<S>() || std::abs(Q[0] + 1) < planeIntersectTolerance<S>())
  {
    int sign2 = (A[0] > 0) ? -1 : 1;
    sign2 *= sign;
    p += axis[0] * (0.5 * s1.side[0] * sign2);
  }
  else if(std::abs(Q[1] - 1) < planeIntersectTolerance<S>() || std::abs(Q[1] + 1) < planeIntersectTolerance<S>())
  {
    int sign2 = (A[1] > 0) ? -1 : 1;
    sign2 *= sign;
    p += axis[1] * (0.5 * s1.side[1] * sign2);
  }
  else if(std::abs(Q[2] - 1) < planeIntersectTolerance<S>() || std::abs(Q[2] + 1) < planeIntersectTolerance<S>())
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
    const Vector3<S> normal = (signed_dist > 0) ? (-new_s2.n).eval() : new_s2.n;
    const Vector3<S> point = p - new_s2.n * new_s2.signedDistance(p);
    const S penetration_depth = depth;

    contacts->emplace_back(normal, point, penetration_depth);
  }

  return true;
}

//==============================================================================
template <typename S>
bool capsulePlaneIntersect(const Capsule<S>& s1, const Transform3<S>& tf1,
                           const Plane<S>& s2, const Transform3<S>& tf2)
{
  Plane<S> new_s2 = transform(s2, tf2);

  const Matrix3<S>& R = tf1.linear();
  const Vector3<S>& T = tf1.translation();

  Vector3<S> dir_z = R.col(2);
  Vector3<S> p1 = T + dir_z * (0.5 * s1.lz);
  Vector3<S> p2 = T - dir_z * (0.5 * s1.lz);

  S d1 = new_s2.signedDistance(p1);
  S d2 = new_s2.signedDistance(p2);

  // two end points on different side of the plane
  if(d1 * d2 <= 0)
    return true;

  // two end points on the same side of the plane, but the end point spheres might intersect the plane
  return (std::abs(d1) <= s1.radius) || (std::abs(d2) <= s1.radius);
}

//==============================================================================
template <typename S>
bool capsulePlaneIntersect(const Capsule<S>& s1, const Transform3<S>& tf1,
                           const Plane<S>& s2, const Transform3<S>& tf2,
                           std::vector<ContactPoint<S>>* contacts)
{
  if(!contacts)
  {
    return capsulePlaneIntersect(s1, tf1, s2, tf2);
  }
  else
  {
    Plane<S> new_s2 = transform(s2, tf2);

    const Matrix3<S>& R = tf1.linear();
    const Vector3<S>& T = tf1.translation();

    Vector3<S> dir_z = R.col(2);


    Vector3<S> p1 = T + dir_z * (0.5 * s1.lz);
    Vector3<S> p2 = T - dir_z * (0.5 * s1.lz);

    S d1 = new_s2.signedDistance(p1);
    S d2 = new_s2.signedDistance(p2);

    S abs_d1 = std::abs(d1);
    S abs_d2 = std::abs(d2);

    // two end points on different side of the plane
    // the contact point is the intersect of axis with the plane
    // the normal is the direction to avoid intersection
    // the depth is the minimum distance to resolve the collision
    if(d1 * d2 < -planeIntersectTolerance<S>())
    {
      if(abs_d1 < abs_d2)
      {
        if (contacts)
        {
          const Vector3<S> normal = (d1 < 0) ? (-new_s2.n).eval() : new_s2.n;
          const Vector3<S> point = p1 * (abs_d2 / (abs_d1 + abs_d2)) + p2 * (abs_d1 / (abs_d1 + abs_d2));
          const S penetration_depth = abs_d1 + s1.radius;

          contacts->emplace_back(normal, point, penetration_depth);
        }
      }
      else
      {
        if (contacts)
        {
          const Vector3<S> normal = (d2 < 0) ? (-new_s2.n).eval() : new_s2.n;
          const Vector3<S> point = p1 * (abs_d2 / (abs_d1 + abs_d2)) + p2 * (abs_d1 / (abs_d1 + abs_d2));
          const S penetration_depth = abs_d2 + s1.radius;

          contacts->emplace_back(normal, point, penetration_depth);
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
        const Vector3<S> normal = (d1 < 0) ? new_s2.n : (-new_s2.n).eval();
        const S penetration_depth = s1.radius - std::min(abs_d1, abs_d2);
        Vector3<S> point;
        if(abs_d1 <= s1.radius && abs_d2 <= s1.radius)
        {
          const Vector3<S> c1 = p1 - new_s2.n * d2;
          const Vector3<S> c2 = p2 - new_s2.n * d1;
          point = (c1 + c2) * 0.5;
        }
        else if(abs_d1 <= s1.radius)
        {
          const Vector3<S> c = p1 - new_s2.n * d1;
          point = c;
        }
        else // (abs_d2 <= s1.radius)
        {
          assert(abs_d2 <= s1.radius);

          const Vector3<S> c = p2 - new_s2.n * d2;
          point = c;
        }

        contacts->emplace_back(normal, point, penetration_depth);
      }

      return true;
    }
  }
}

//==============================================================================
template <typename S>
bool cylinderPlaneIntersect(const Cylinder<S>& s1, const Transform3<S>& tf1,
                            const Plane<S>& s2, const Transform3<S>& tf2)
{
  Plane<S> new_s2 = transform(s2, tf2);

  const Matrix3<S>& R = tf1.linear();
  const Vector3<S>& T = tf1.translation();

  Vector3<S> Q = R.transpose() * new_s2.n;

  S term = std::abs(Q[2]) * s1.lz + s1.radius * std::sqrt(Q[0] * Q[0] + Q[1] * Q[1]);
  S dist = new_s2.distance(T);
  S depth = term - dist;

  if(depth < 0)
    return false;
  else
    return true;
}

//==============================================================================
template <typename S>
bool cylinderPlaneIntersect(const Cylinder<S>& s1, const Transform3<S>& tf1,
                            const Plane<S>& s2, const Transform3<S>& tf2,
                            std::vector<ContactPoint<S>>* contacts)
{
  if(!contacts)
  {
    return cylinderPlaneIntersect(s1, tf1, s2, tf2);
  }
  else
  {
    Plane<S> new_s2 = transform(s2, tf2);

    const Matrix3<S>& R = tf1.linear();
    const Vector3<S>& T = tf1.translation();

    Vector3<S> dir_z = R.col(2);
    S cosa = dir_z.dot(new_s2.n);

    if(std::abs(cosa) < planeIntersectTolerance<S>())
    {
      S d = new_s2.signedDistance(T);
      S depth = s1.radius - std::abs(d);
      if(depth < 0) return false;
      else
      {
        if (contacts)
        {
          const Vector3<S> normal = (d < 0) ? new_s2.n : (-new_s2.n).eval();
          const Vector3<S> point = T - new_s2.n * d;
          const S penetration_depth = depth;

          contacts->emplace_back(normal, point, penetration_depth);
        }
        return true;
      }
    }
    else
    {
      Vector3<S> C = dir_z * cosa - new_s2.n;
      if(std::abs(cosa + 1) < planeIntersectTolerance<S>() || std::abs(cosa - 1) < planeIntersectTolerance<S>())
        C = Vector3<S>(0, 0, 0);
      else
      {
        S s = C.norm();
        s = s1.radius / s;
        C *= s;
      }

      Vector3<S> p1 = T + dir_z * (0.5 * s1.lz);
      Vector3<S> p2 = T - dir_z * (0.5 * s1.lz);

      Vector3<S> c1, c2;
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

      S d1 = new_s2.signedDistance(c1);
      S d2 = new_s2.signedDistance(c2);

      if(d1 * d2 <= 0)
      {
        S abs_d1 = std::abs(d1);
        S abs_d2 = std::abs(d2);

        if(abs_d1 > abs_d2)
        {
          if (contacts)
          {
            const Vector3<S> normal = (d2 < 0) ? (-new_s2.n).eval() : new_s2.n;
            const Vector3<S> point = c2 - new_s2.n * d2;
            const S penetration_depth = abs_d2;

            contacts->emplace_back(normal, point, penetration_depth);
          }
        }
        else
        {
          if (contacts)
          {
            const Vector3<S> normal = (d1 < 0) ? (-new_s2.n).eval() : new_s2.n;
            const Vector3<S> point = c1 - new_s2.n * d1;
            const S penetration_depth = abs_d1;

            contacts->emplace_back(normal, point, penetration_depth);
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
template <typename S>
bool conePlaneIntersect(const Cone<S>& s1, const Transform3<S>& tf1,
                        const Plane<S>& s2, const Transform3<S>& tf2,
                        std::vector<ContactPoint<S>>* contacts)
{
  Plane<S> new_s2 = transform(s2, tf2);

  const Matrix3<S>& R = tf1.linear();
  const Vector3<S>& T = tf1.translation();

  Vector3<S> dir_z = R.col(2);
  S cosa = dir_z.dot(new_s2.n);

  if(std::abs(cosa) < planeIntersectTolerance<S>())
  {
    S d = new_s2.signedDistance(T);
    S depth = s1.radius - std::abs(d);
    if(depth < 0) return false;
    else
    {
      if (contacts)
      {
        const Vector3<S> normal = (d < 0) ? new_s2.n : (-new_s2.n).eval();
        const Vector3<S> point = T - dir_z * (0.5 * s1.lz) + dir_z * (0.5 * depth / s1.radius * s1.lz) - new_s2.n * d;
        const S penetration_depth = depth;

        contacts->emplace_back(normal, point, penetration_depth);
      }

      return true;
    }
  }
  else
  {
    Vector3<S> C = dir_z * cosa - new_s2.n;
    if(std::abs(cosa + 1) < planeIntersectTolerance<S>() || std::abs(cosa - 1) < planeIntersectTolerance<S>())
      C = Vector3<S>(0, 0, 0);
    else
    {
      S s = C.norm();
      s = s1.radius / s;
      C *= s;
    }

    Vector3<S> c[3];
    c[0] = T + dir_z * (0.5 * s1.lz);
    c[1] = T - dir_z * (0.5 * s1.lz) + C;
    c[2] = T - dir_z * (0.5 * s1.lz) - C;

    S d[3];
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
      S d_positive = 0, d_negative = 0;
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
        const Vector3<S> normal = (d_positive > d_negative) ? (-new_s2.n).eval() : new_s2.n;
        const S penetration_depth = std::min(d_positive, d_negative);

        Vector3<S> point;
        Vector3<S> p[2] { Vector3<S>::Zero(), Vector3<S>::Zero() };
        Vector3<S> q = Vector3<S>::Zero();

        S p_d[2];
        S q_d(0);

        if(n_positive == 2)
        {
          for(std::size_t i = 0, j = 0; i < 3; ++i)
          {
            if(positive[i]) { p[j] = c[i]; p_d[j] = d[i]; j++; }
            else { q = c[i]; q_d = d[i]; }
          }

          const Vector3<S> t1 = (-p[0] * q_d + q * p_d[0]) / (-q_d + p_d[0]);
          const Vector3<S> t2 = (-p[1] * q_d + q * p_d[1]) / (-q_d + p_d[1]);
          point = (t1 + t2) * 0.5;
        }
        else
        {
          for(std::size_t i = 0, j = 0; i < 3; ++i)
          {
            if(!positive[i]) { p[j] = c[i]; p_d[j] = d[i]; j++; }
            else { q = c[i]; q_d = d[i]; }
          }

          const Vector3<S> t1 = (p[0] * q_d - q * p_d[0]) / (q_d - p_d[0]);
          const Vector3<S> t2 = (p[1] * q_d - q * p_d[1]) / (q_d - p_d[1]);
          point = (t1 + t2) * 0.5;
        }

        contacts->emplace_back(normal, point, penetration_depth);
      }

      return true;
    }
  }
}

//==============================================================================
template <typename S>
bool convexPlaneIntersect(const Convex<S>& s1, const Transform3<S>& tf1,
                          const Plane<S>& s2, const Transform3<S>& tf2,
                          Vector3<S>* contact_points, S* penetration_depth, Vector3<S>* normal)
{
  Plane<S> new_s2 = transform(s2, tf2);

  Vector3<S> v_min, v_max;
  S d_min = std::numeric_limits<S>::max(), d_max = -std::numeric_limits<S>::max();

  for (const auto& vertex : s1.getVertices())
  {
    Vector3<S> p = tf1 * vertex;

    S d = new_s2.signedDistance(p);

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
template <typename S>
bool planeTriangleIntersect(const Plane<S>& s1, const Transform3<S>& tf1,
                            const Vector3<S>& P1, const Vector3<S>& P2, const Vector3<S>& P3, const Transform3<S>& tf2,
                            Vector3<S>* contact_points, S* penetration_depth, Vector3<S>* normal)
{
  Plane<S> new_s1 = transform(s1, tf1);

  Vector3<S> c[3];
  c[0] = tf2 * P1;
  c[1] = tf2 * P2;
  c[2] = tf2 * P3;

  S d[3];
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
    S d_positive = 0, d_negative = 0;
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
      Vector3<S> p[2] = {Vector3<S>::Zero(), Vector3<S>::Zero()};
      Vector3<S> q = Vector3<S>::Zero();

      S p_d[2];
      S q_d(0);

      if(n_positive == 2)
      {
        for(std::size_t i = 0, j = 0; i < 3; ++i)
        {
          if(positive[i]) { p[j] = c[i]; p_d[j] = d[i]; j++; }
          else { q = c[i]; q_d = d[i]; }
        }

        Vector3<S> t1 = (-p[0] * q_d + q * p_d[0]) / (-q_d + p_d[0]);
        Vector3<S> t2 = (-p[1] * q_d + q * p_d[1]) / (-q_d + p_d[1]);
        *contact_points = (t1 + t2) * 0.5;
      }
      else
      {
        for(std::size_t i = 0, j = 0; i < 3; ++i)
        {
          if(!positive[i]) { p[j] = c[i]; p_d[j] = d[i]; j++; }
          else { q = c[i]; q_d = d[i]; }
        }

        Vector3<S> t1 = (p[0] * q_d - q * p_d[0]) / (q_d - p_d[0]);
        Vector3<S> t2 = (p[1] * q_d - q * p_d[1]) / (q_d - p_d[1]);
        *contact_points = (t1 + t2) * 0.5;
      }
    }
    return true;
  }
}

//==============================================================================
template <typename S>
bool planeIntersect(const Plane<S>& s1, const Transform3<S>& tf1,
                    const Plane<S>& s2, const Transform3<S>& tf2,
                    std::vector<ContactPoint<S>>* /*contacts*/)
{
  Plane<S> new_s1 = transform(s1, tf1);
  Plane<S> new_s2 = transform(s2, tf2);

  S a = (new_s1.n).dot(new_s2.n);
  if(a == 1 && new_s1.d != new_s2.d)
    return false;
  if(a == -1 && new_s1.d != -new_s2.d)
    return false;

  return true;
}

} // namespace detail
} // namespace fcl

#endif

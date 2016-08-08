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

#ifndef FCL_NARROWPHASE_DETAIL_PLANE_H
#define FCL_NARROWPHASE_DETAIL_PLANE_H

#include "fcl/collision_data.h"

namespace fcl
{

namespace details
{

template <typename Scalar>
Scalar planeIntersectTolerance();

template <>
double planeIntersectTolerance();

template <>
float planeIntersectTolerance();

template <typename Scalar>
bool spherePlaneIntersect(const Sphere<Scalar>& s1, const Transform3<Scalar>& tf1,
                          const Plane<Scalar>& s2, const Transform3<Scalar>& tf2,
                          std::vector<ContactPoint<Scalar>>* contacts);

template <typename Scalar>
bool ellipsoidPlaneIntersect(const Ellipsoid<Scalar>& s1, const Transform3<Scalar>& tf1,
                             const Plane<Scalar>& s2, const Transform3<Scalar>& tf2,
                             std::vector<ContactPoint<Scalar>>* contacts);

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
                       std::vector<ContactPoint<Scalar>>* contacts);

template <typename Scalar>
bool capsulePlaneIntersect(const Capsule<Scalar>& s1, const Transform3<Scalar>& tf1,
                           const Plane<Scalar>& s2, const Transform3<Scalar>& tf2);

template <typename Scalar>
bool capsulePlaneIntersect(const Capsule<Scalar>& s1, const Transform3<Scalar>& tf1,
                           const Plane<Scalar>& s2, const Transform3<Scalar>& tf2,
                           std::vector<ContactPoint<Scalar>>* contacts);

/// @brief cylinder-plane intersect
/// n^T (R (r * cosa * v1 + r * sina * v2 + h * v3) + T) ~ d
/// need one point to be positive and one to be negative
/// (n^T * v3) * h + n * T -d + r * (cosa * (n^T * R * v1) + sina * (n^T * R * v2)) ~ 0
/// (n^T * v3) * h + r * (cosa * (n^T * R * v1) + sina * (n^T * R * v2)) + n * T - d ~ 0
template <typename Scalar>
bool cylinderPlaneIntersect(const Cylinder<Scalar>& s1, const Transform3<Scalar>& tf1,
                            const Plane<Scalar>& s2, const Transform3<Scalar>& tf2);

template <typename Scalar>
bool cylinderPlaneIntersect(const Cylinder<Scalar>& s1, const Transform3<Scalar>& tf1,
                            const Plane<Scalar>& s2, const Transform3<Scalar>& tf2,
                            std::vector<ContactPoint<Scalar>>* contacts);

template <typename Scalar>
bool conePlaneIntersect(const Cone<Scalar>& s1, const Transform3<Scalar>& tf1,
                        const Plane<Scalar>& s2, const Transform3<Scalar>& tf2,
                        std::vector<ContactPoint<Scalar>>* contacts);

template <typename Scalar>
bool convexPlaneIntersect(const Convex<Scalar>& s1, const Transform3<Scalar>& tf1,
                          const Plane<Scalar>& s2, const Transform3<Scalar>& tf2,
                          Vector3<Scalar>* contact_points, Scalar* penetration_depth, Vector3<Scalar>* normal);

template <typename Scalar>
bool planeTriangleIntersect(const Plane<Scalar>& s1, const Transform3<Scalar>& tf1,
                            const Vector3<Scalar>& P1, const Vector3<Scalar>& P2, const Vector3<Scalar>& P3, const Transform3<Scalar>& tf2,
                            Vector3<Scalar>* contact_points, Scalar* penetration_depth, Vector3<Scalar>* normal);

template <typename Scalar>
bool halfspacePlaneIntersect(const Halfspace<Scalar>& s1, const Transform3<Scalar>& tf1,
                             const Plane<Scalar>& s2, const Transform3<Scalar>& tf2,
                             Plane<Scalar>& pl, Vector3<Scalar>& p, Vector3<Scalar>& d,
                             Scalar& penetration_depth,
                             int& ret);

template <typename Scalar>
bool planeIntersect(const Plane<Scalar>& s1, const Transform3<Scalar>& tf1,
                    const Plane<Scalar>& s2, const Transform3<Scalar>& tf2,
                    std::vector<ContactPoint<Scalar>>* contacts);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

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
  const Plane<Scalar>& new_s2 = transform(s2, tf1.inverse(Eigen::Isometry) * tf2);

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

} // namespace details

} // namespace fcl

#endif

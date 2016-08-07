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

#ifndef FCL_NARROWPHASE_DETAIL_HALFSPACE_H
#define FCL_NARROWPHASE_DETAIL_HALFSPACE_H

#include "fcl/collision_data.h"

namespace fcl
{

namespace details
{

template <typename Scalar>
Scalar halfspaceIntersectTolerance();

template <>
float halfspaceIntersectTolerance();

template <>
double halfspaceIntersectTolerance();

template <typename Scalar>
bool sphereHalfspaceIntersect(const Sphere<Scalar>& s1, const Transform3<Scalar>& tf1,
                              const Halfspace<Scalar>& s2, const Transform3<Scalar>& tf2,
                              std::vector<ContactPoint<Scalar>>* contacts);

template <typename Scalar>
bool ellipsoidHalfspaceIntersect(const Ellipsoid<Scalar>& s1, const Transform3<Scalar>& tf1,
                                 const Halfspace<Scalar>& s2, const Transform3<Scalar>& tf2,
                                 std::vector<ContactPoint<Scalar>>* contacts);

/// @brief box half space, a, b, c  = +/- edge size
/// n^T * (R(o + a v1 + b v2 + c v3) + T) <= d
/// so (R^T n) (a v1 + b v2 + c v3) + n * T <= d
/// check whether d - n * T - (R^T n) (a v1 + b v2 + c v3) >= 0 for some a, b, c
/// the max value of left side is d - n * T + |(R^T n) (a v1 + b v2 + c v3)|, check that is enough
template <typename Scalar>
bool boxHalfspaceIntersect(const Box<Scalar>& s1, const Transform3<Scalar>& tf1,
                           const Halfspace<Scalar>& s2, const Transform3<Scalar>& tf2);

template <typename Scalar>
bool boxHalfspaceIntersect(const Box<Scalar>& s1, const Transform3<Scalar>& tf1,
                           const Halfspace<Scalar>& s2, const Transform3<Scalar>& tf2,
                           std::vector<ContactPoint<Scalar>>* contacts);

template <typename Scalar>
bool capsuleHalfspaceIntersect(const Capsule<Scalar>& s1, const Transform3<Scalar>& tf1,
                               const Halfspace<Scalar>& s2, const Transform3<Scalar>& tf2,
                               std::vector<ContactPoint<Scalar>>* contacts);

template <typename Scalar>
bool cylinderHalfspaceIntersect(const Cylinder<Scalar>& s1, const Transform3<Scalar>& tf1,
                                const Halfspace<Scalar>& s2, const Transform3<Scalar>& tf2,
                                std::vector<ContactPoint<Scalar>>* contacts);

template <typename Scalar>
bool coneHalfspaceIntersect(const Cone<Scalar>& s1, const Transform3<Scalar>& tf1,
                            const Halfspace<Scalar>& s2, const Transform3<Scalar>& tf2,
                            std::vector<ContactPoint<Scalar>>* contacts);

template <typename Scalar>
bool convexHalfspaceIntersect(const Convex<Scalar>& s1, const Transform3<Scalar>& tf1,
                              const Halfspace<Scalar>& s2, const Transform3<Scalar>& tf2,
                              Vector3<Scalar>* contact_points, Scalar* penetration_depth, Vector3<Scalar>* normal);

template <typename Scalar>
bool halfspaceTriangleIntersect(const Halfspace<Scalar>& s1, const Transform3<Scalar>& tf1,
                                const Vector3<Scalar>& P1, const Vector3<Scalar>& P2, const Vector3<Scalar>& P3, const Transform3<Scalar>& tf2,
                                Vector3<Scalar>* contact_points, Scalar* penetration_depth, Vector3<Scalar>* normal);

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
                             int& ret);

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
                        int& ret);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

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

} // namespace details

} // namespace fcl

#endif

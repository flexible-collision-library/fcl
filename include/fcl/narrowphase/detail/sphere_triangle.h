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

#ifndef FCL_NARROWPHASE_DETAIL_SPHERETRIANGLE_H
#define FCL_NARROWPHASE_DETAIL_SPHERETRIANGLE_H

#include "fcl/collision_data.h"

namespace fcl
{

namespace details
{

/** \brief the minimum distance from a point to a line */
template <typename Scalar>
Scalar segmentSqrDistance(const Vector3<Scalar>& from, const Vector3<Scalar>& to,const Vector3<Scalar>& p, Vector3<Scalar>& nearest);

/// @brief Whether a point's projection is in a triangle
template <typename Scalar>
bool projectInTriangle(const Vector3<Scalar>& p1, const Vector3<Scalar>& p2, const Vector3<Scalar>& p3, const Vector3<Scalar>& normal, const Vector3<Scalar>& p);

template <typename Scalar>
bool sphereTriangleIntersect(const Sphere<Scalar>& s, const Transform3<Scalar>& tf,
                             const Vector3<Scalar>& P1, const Vector3<Scalar>& P2, const Vector3<Scalar>& P3, Vector3<Scalar>* contact_points, Scalar* penetration_depth, Vector3<Scalar>* normal_);

template <typename Scalar>
bool sphereTriangleDistance(const Sphere<Scalar>& sp, const Transform3<Scalar>& tf,
                            const Vector3<Scalar>& P1, const Vector3<Scalar>& P2, const Vector3<Scalar>& P3,
                            Scalar* dist);

template <typename Scalar>
bool sphereTriangleDistance(const Sphere<Scalar>& sp, const Transform3<Scalar>& tf,
                            const Vector3<Scalar>& P1, const Vector3<Scalar>& P2, const Vector3<Scalar>& P3,
                            Scalar* dist, Vector3<Scalar>* p1, Vector3<Scalar>* p2);

template <typename Scalar>
bool sphereTriangleDistance(const Sphere<Scalar>& sp, const Transform3<Scalar>& tf1,
                            const Vector3<Scalar>& P1, const Vector3<Scalar>& P2, const Vector3<Scalar>& P3, const Transform3<Scalar>& tf2,
                            Scalar* dist, Vector3<Scalar>* p1, Vector3<Scalar>* p2);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
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
      if(p1) { *p1 = o - dir * sp.radius; *p1 = tf.inverse(Eigen::Isometry) * (*p1); }
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
  if(p2) *p2 = tf2.inverse(Eigen::Isometry) * (*p2);

  return res;
}

} // namespace details

} // namespace fcl

#endif

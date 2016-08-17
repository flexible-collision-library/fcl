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

#ifndef FCL_NARROWPHASE_DETAIL_SPHERETRIANGLE_INL_H
#define FCL_NARROWPHASE_DETAIL_SPHERETRIANGLE_INL_H

#include "fcl/narrowphase/detail/primitive_shape_algorithm/sphere_triangle.h"

#include "fcl/math/detail/project.h"

namespace fcl
{

namespace detail
{

//==============================================================================
extern template
double segmentSqrDistance(const Vector3<double>& from, const Vector3<double>& to,const Vector3<double>& p, Vector3<double>& nearest);

//==============================================================================
extern template
bool projectInTriangle(const Vector3<double>& p1, const Vector3<double>& p2, const Vector3<double>& p3, const Vector3<double>& normal, const Vector3<double>& p);

//==============================================================================
extern template
bool sphereTriangleIntersect(const Sphere<double>& s, const Transform3<double>& tf,
                             const Vector3<double>& P1, const Vector3<double>& P2, const Vector3<double>& P3, Vector3<double>* contact_points, double* penetration_depth, Vector3<double>* normal_);

//==============================================================================
extern template
bool sphereTriangleDistance(const Sphere<double>& sp, const Transform3<double>& tf,
                            const Vector3<double>& P1, const Vector3<double>& P2, const Vector3<double>& P3,
                            double* dist);

//==============================================================================
extern template
bool sphereTriangleDistance(const Sphere<double>& sp, const Transform3<double>& tf,
                            const Vector3<double>& P1, const Vector3<double>& P2, const Vector3<double>& P3,
                            double* dist, Vector3<double>* p1, Vector3<double>* p2);

//==============================================================================
extern template
bool sphereTriangleDistance(const Sphere<double>& sp, const Transform3<double>& tf1,
                            const Vector3<double>& P1, const Vector3<double>& P2, const Vector3<double>& P3, const Transform3<double>& tf2,
                            double* dist, Vector3<double>* p1, Vector3<double>* p2);

//==============================================================================
template <typename S>
S segmentSqrDistance(const Vector3<S>& from, const Vector3<S>& to,const Vector3<S>& p, Vector3<S>& nearest)
{
  Vector3<S> diff = p - from;
  Vector3<S> v = to - from;
  S t = v.dot(diff);

  if(t > 0)
  {
    S dotVV = v.dot(v);
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
template <typename S>
bool projectInTriangle(const Vector3<S>& p1, const Vector3<S>& p2, const Vector3<S>& p3, const Vector3<S>& normal, const Vector3<S>& p)
{
  Vector3<S> edge1(p2 - p1);
  Vector3<S> edge2(p3 - p2);
  Vector3<S> edge3(p1 - p3);

  Vector3<S> p1_to_p(p - p1);
  Vector3<S> p2_to_p(p - p2);
  Vector3<S> p3_to_p(p - p3);

  Vector3<S> edge1_normal(edge1.cross(normal));
  Vector3<S> edge2_normal(edge2.cross(normal));
  Vector3<S> edge3_normal(edge3.cross(normal));

  S r1, r2, r3;
  r1 = edge1_normal.dot(p1_to_p);
  r2 = edge2_normal.dot(p2_to_p);
  r3 = edge3_normal.dot(p3_to_p);
  if ( ( r1 > 0 && r2 > 0 && r3 > 0 ) ||
       ( r1 <= 0 && r2 <= 0 && r3 <= 0 ) )
    return true;
  return false;
}

//==============================================================================
template <typename S>
bool sphereTriangleIntersect(const Sphere<S>& s, const Transform3<S>& tf,
                             const Vector3<S>& P1, const Vector3<S>& P2, const Vector3<S>& P3, Vector3<S>* contact_points, S* penetration_depth, Vector3<S>* normal_)
{
  Vector3<S> normal = (P2 - P1).cross(P3 - P1);
  normal.normalize();
  const Vector3<S>& center = tf.translation();
  const S& radius = s.radius;
  S radius_with_threshold = radius + std::numeric_limits<S>::epsilon();
  Vector3<S> p1_to_center = center - P1;
  S distance_from_plane = p1_to_center.dot(normal);

  if(distance_from_plane < 0)
  {
    distance_from_plane *= -1;
    normal *= -1;
  }

  bool is_inside_contact_plane = (distance_from_plane < radius_with_threshold);

  bool has_contact = false;
  Vector3<S> contact_point;
  if(is_inside_contact_plane)
  {
    if(projectInTriangle(P1, P2, P3, normal, center))
    {
      has_contact = true;
      contact_point = center - normal * distance_from_plane;
    }
    else
    {
      S contact_capsule_radius_sqr = radius_with_threshold * radius_with_threshold;
      Vector3<S> nearest_on_edge;
      S distance_sqr;
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
    Vector3<S> contact_to_center = contact_point - center;
    S distance_sqr = contact_to_center.squaredNorm();

    if(distance_sqr < radius_with_threshold * radius_with_threshold)
    {
      if(distance_sqr > 0)
      {
        S distance = std::sqrt(distance_sqr);
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
template <typename S>
bool sphereTriangleDistance(const Sphere<S>& sp, const Transform3<S>& tf,
                            const Vector3<S>& P1, const Vector3<S>& P2, const Vector3<S>& P3,
                            S* dist)
{
  // from geometric tools, very different from the collision code.

  const Vector3<S>& center = tf.translation();
  S radius = sp.radius;
  Vector3<S> diff = P1 - center;
  Vector3<S> edge0 = P2 - P1;
  Vector3<S> edge1 = P3 - P1;
  S a00 = edge0.squaredNorm();
  S a01 = edge0.dot(edge1);
  S a11 = edge1.squaredNorm();
  S b0 = diff.dot(edge0);
  S b1 = diff.dot(edge1);
  S c = diff.squaredNorm();
  S det = fabs(a00*a11 - a01*a01);
  S s = a01*b1 - a11*b0;
  S t = a01*b0 - a00*b1;

  S sqr_dist;

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
      S inv_det = (1)/det;
      s *= inv_det;
      t *= inv_det;
      sqr_dist = s*(a00*s + a01*t + 2*b0) + t*(a01*s + a11*t + 2*b1) + c;
    }
  }
  else
  {
    S tmp0, tmp1, numer, denom;

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
template <typename S>
bool sphereTriangleDistance(const Sphere<S>& sp, const Transform3<S>& tf,
                            const Vector3<S>& P1, const Vector3<S>& P2, const Vector3<S>& P3,
                            S* dist, Vector3<S>* p1, Vector3<S>* p2)
{
  if(p1 || p2)
  {
    Vector3<S> o = tf.translation();
    typename Project<S>::ProjectResult result;
    result = Project<S>::projectTriangle(P1, P2, P3, o);
    if(result.sqr_distance > sp.radius * sp.radius)
    {
      if(dist) *dist = std::sqrt(result.sqr_distance) - sp.radius;
      Vector3<S> project_p = P1 * result.parameterization[0] + P2 * result.parameterization[1] + P3 * result.parameterization[2];
      Vector3<S> dir = o - project_p;
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
template <typename S>
bool sphereTriangleDistance(const Sphere<S>& sp, const Transform3<S>& tf1,
                            const Vector3<S>& P1, const Vector3<S>& P2, const Vector3<S>& P3, const Transform3<S>& tf2,
                            S* dist, Vector3<S>* p1, Vector3<S>* p2)
{
  bool res = detail::sphereTriangleDistance(sp, tf1, tf2 * P1, tf2 * P2, tf2 * P3, dist, p1, p2);
  if(p2) *p2 = tf2.inverse(Eigen::Isometry) * (*p2);

  return res;
}

} // namespace detail
} // namespace fcl

#endif

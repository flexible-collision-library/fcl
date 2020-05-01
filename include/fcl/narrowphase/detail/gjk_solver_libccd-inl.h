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

#ifndef FCL_NARROWPHASE_GJKSOLVERLIBCCD_INL_H
#define FCL_NARROWPHASE_GJKSOLVERLIBCCD_INL_H

#include "fcl/narrowphase/detail/gjk_solver_libccd.h"

#include <algorithm>

#include "fcl/common/unused.h"

#include "fcl/narrowphase/detail/convexity_based_algorithm/gjk_libccd.h"
#include "fcl/narrowphase/detail/primitive_shape_algorithm/capsule_capsule.h"
#include "fcl/narrowphase/detail/primitive_shape_algorithm/sphere_box.h"
#include "fcl/narrowphase/detail/primitive_shape_algorithm/sphere_capsule.h"
#include "fcl/narrowphase/detail/primitive_shape_algorithm/sphere_cylinder.h"
#include "fcl/narrowphase/detail/primitive_shape_algorithm/sphere_sphere.h"
#include "fcl/narrowphase/detail/primitive_shape_algorithm/sphere_triangle.h"
#include "fcl/narrowphase/detail/primitive_shape_algorithm/box_box.h"
#include "fcl/narrowphase/detail/primitive_shape_algorithm/halfspace.h"
#include "fcl/narrowphase/detail/primitive_shape_algorithm/plane.h"
#include "fcl/narrowphase/detail/failed_at_this_configuration.h"

namespace fcl
{

namespace detail
{

//==============================================================================
extern template
struct GJKSolver_libccd<double>;

//==============================================================================
template<typename S>
template<typename Shape1, typename Shape2>
bool GJKSolver_libccd<S>::shapeIntersect(
    const Shape1& s1, const Transform3<S>& tf1,
    const Shape2& s2, const Transform3<S>& tf2,
    Vector3<S>* contact_points,
    S* penetration_depth,
    Vector3<S>* normal) const
{
  bool res;

  if (contact_points || penetration_depth || normal)
  {
    std::vector<ContactPoint<S>> contacts;

    res = shapeIntersect(s1, tf1, s2, tf2, &contacts);

    if (!contacts.empty())
    {
      // Get the deepest contact point
      const ContactPoint<S>& maxDepthContact = *std::max_element(contacts.begin(), contacts.end(), comparePenDepth<S>);

      if (contact_points)
        *contact_points = maxDepthContact.pos;

      if (penetration_depth)
        *penetration_depth = maxDepthContact.penetration_depth;

      if (normal)
        *normal = maxDepthContact.normal;
    }
  }
  else
  {
    res = shapeIntersect(s1, tf1, s2, tf2, nullptr);
  }

  return res;
}

//==============================================================================
template<typename S, typename Shape1, typename Shape2>
struct ShapeIntersectLibccdImpl
{
  static bool run(
      const GJKSolver_libccd<S>& gjkSolver,
      const Shape1& s1, const Transform3<S>& tf1,
      const Shape2& s2, const Transform3<S>& tf2,
      std::vector<ContactPoint<S>>* contacts)
  {
    void* o1 = detail::GJKInitializer<S, Shape1>::createGJKObject(s1, tf1);
    void* o2 = detail::GJKInitializer<S, Shape2>::createGJKObject(s2, tf2);

    bool res;

    if(contacts)
    {
      Vector3<S> normal;
      Vector3<S> point;
      S depth;
      res = detail::GJKCollide<S>(
            o1,
            detail::GJKInitializer<S, Shape1>::getSupportFunction(),
            detail::GJKInitializer<S, Shape1>::getCenterFunction(),
            o2, detail::GJKInitializer<S, Shape2>::getSupportFunction(),
            detail::GJKInitializer<S, Shape2>::getCenterFunction(),
            gjkSolver.max_collision_iterations,
            gjkSolver.collision_tolerance,
            &point,
            &depth,
            &normal);
      contacts->emplace_back(normal, point, depth);
    }
    else
    {
      res = detail::GJKCollide<S>(
            o1,
            detail::GJKInitializer<S, Shape1>::getSupportFunction(),
            detail::GJKInitializer<S, Shape1>::getCenterFunction(),
            o2,
            detail::GJKInitializer<S, Shape2>::getSupportFunction(),
            detail::GJKInitializer<S, Shape2>::getCenterFunction(),
            gjkSolver.max_collision_iterations,
            gjkSolver.collision_tolerance,
            nullptr,
            nullptr,
            nullptr);
    }

    detail::GJKInitializer<S, Shape1>::deleteGJKObject(o1);
    detail::GJKInitializer<S, Shape2>::deleteGJKObject(o2);

    return res;
  }
};

//==============================================================================
template<typename S>
template<typename Shape1, typename Shape2>
bool GJKSolver_libccd<S>::shapeIntersect(
    const Shape1& s1, const Transform3<S>& tf1,
    const Shape2& s2, const Transform3<S>& tf2,
    std::vector<ContactPoint<S>>* contacts) const
{
  return ShapeIntersectLibccdImpl<S, Shape1, Shape2>::run(
        *this, s1, tf1, s2, tf2, contacts);
}

// Shape intersect algorithms not using libccd
//
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+----------+
// |            | box | sphere | ellipsoid | capsule | cone | cylinder | plane | half-space | triangle |  convex  |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+----------+
// | box        |  O  |   O    |           |         |      |          |   O   |      O     |          |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+----------+
// | sphere     |/////|   O    |           |    O    |      |    O     |   O   |      O     |    O     |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+----------+
// | ellipsoid  |/////|////////|           |         |      |          |   O   |      O     |   TODO   |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+----------+
// | capsule    |/////|////////|///////////|         |      |          |   O   |      O     |          |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+----------+
// | cone       |/////|////////|///////////|/////////|      |          |   O   |      O     |          |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+----------+
// | cylinder   |/////|////////|///////////|/////////|//////|          |   O   |      O     |          |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+----------+
// | plane      |/////|////////|///////////|/////////|//////|//////////|   O   |      O     |    O     |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+----------+
// | half-space |/////|////////|///////////|/////////|//////|//////////|///////|      O     |    O     |    O     |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+----------+
// | triangle   |/////|////////|///////////|/////////|//////|//////////|///////|////////////|          |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+----------+
// | convex     |/////|////////|///////////|/////////|//////|//////////|///////|////////////|//////////|          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+----------+

#define FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT_REG(SHAPE1, SHAPE2, ALG)\
  template <typename S>\
  struct ShapeIntersectLibccdImpl<S, SHAPE1<S>, SHAPE2<S>>\
  {\
    static bool run(\
        const GJKSolver_libccd<S>& /*gjkSolver*/,\
        const SHAPE1<S>& s1,\
        const Transform3<S>& tf1,\
        const SHAPE2<S>& s2,\
        const Transform3<S>& tf2,\
        std::vector<ContactPoint<S>>* contacts)\
    {\
      return ALG(s1, tf1, s2, tf2, contacts);\
    }\
  };

#define FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT_INV(SHAPE1, SHAPE2, ALG)\
  template <typename S>\
  struct ShapeIntersectLibccdImpl<S, SHAPE2<S>, SHAPE1<S>>\
  {\
    static bool run(\
        const GJKSolver_libccd<S>& /*gjkSolver*/,\
        const SHAPE2<S>& s1,\
        const Transform3<S>& tf1,\
        const SHAPE1<S>& s2,\
        const Transform3<S>& tf2,\
        std::vector<ContactPoint<S>>* contacts)\
    {\
      const bool res = ALG(s2, tf2, s1, tf1, contacts);\
      if (contacts) flipNormal(*contacts);\
      return res;\
    }\
  };

#define FCL_GJK_LIBCCD_SHAPE_INTERSECT(SHAPE, ALG)\
  FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT_REG(SHAPE, SHAPE, ALG)

#define FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(SHAPE1, SHAPE2, ALG)\
  FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT_REG(SHAPE1, SHAPE2, ALG)\
  FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT_INV(SHAPE1, SHAPE2, ALG)

FCL_GJK_LIBCCD_SHAPE_INTERSECT(Sphere, detail::sphereSphereIntersect)
FCL_GJK_LIBCCD_SHAPE_INTERSECT(Box, detail::boxBoxIntersect)

FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Sphere, Capsule, detail::sphereCapsuleIntersect)

FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Sphere, Box, detail::sphereBoxIntersect)

FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Sphere, Cylinder, detail::sphereCylinderIntersect)

FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Sphere, Halfspace, detail::sphereHalfspaceIntersect)
FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Ellipsoid, Halfspace, detail::ellipsoidHalfspaceIntersect)
FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Box, Halfspace, detail::boxHalfspaceIntersect)
FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Capsule, Halfspace, detail::capsuleHalfspaceIntersect)
FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Cylinder, Halfspace, detail::cylinderHalfspaceIntersect)
FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Cone, Halfspace, detail::coneHalfspaceIntersect)
FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Convex, Halfspace, detail::convexHalfspaceIntersect)

FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Sphere, Plane, detail::spherePlaneIntersect)
FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Ellipsoid, Plane, detail::ellipsoidPlaneIntersect)
FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Box, Plane, detail::boxPlaneIntersect)
FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Capsule, Plane, detail::capsulePlaneIntersect)
FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Cylinder, Plane, detail::cylinderPlaneIntersect)
FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Cone, Plane, detail::conePlaneIntersect)

template <typename S>
struct ShapeIntersectLibccdImpl<S, Halfspace<S>, Halfspace<S>>
{
  static bool run(
      const GJKSolver_libccd<S>& /*gjkSolver*/,
      const Halfspace<S>& s1,
      const Transform3<S>& tf1,
      const Halfspace<S>& s2,
      const Transform3<S>& tf2,
      std::vector<ContactPoint<S>>* contacts)
  {
    FCL_UNUSED(contacts);

    Halfspace<S> s;
    Vector3<S> p, d;
    S depth;
    int ret;
    return detail::halfspaceIntersect(s1, tf1, s2, tf2, p, d, s, depth, ret);
  }
};

template <typename S>
struct ShapeIntersectLibccdImpl<S, Plane<S>, Plane<S>>
{
  static bool run(
      const GJKSolver_libccd<S>& /*gjkSolver*/,
      const Plane<S>& s1,
      const Transform3<S>& tf1,
      const Plane<S>& s2,
      const Transform3<S>& tf2,
      std::vector<ContactPoint<S>>* contacts)
  {
    return detail::planeIntersect(s1, tf1, s2, tf2, contacts);
  }
};

template <typename S>
struct ShapeIntersectLibccdImpl<S, Plane<S>, Halfspace<S>>
{
  static bool run(
      const GJKSolver_libccd<S>& /*gjkSolver*/,
      const Plane<S>& s1,
      const Transform3<S>& tf1,
      const Halfspace<S>& s2,
      const Transform3<S>& tf2,
      std::vector<ContactPoint<S>>* contacts)
  {
    FCL_UNUSED(contacts);

    Plane<S> pl;
    Vector3<S> p, d;
    S depth;
    int ret;
    return detail::planeHalfspaceIntersect(s1, tf1, s2, tf2, pl, p, d, depth, ret);
  }
};

template <typename S>
struct ShapeIntersectLibccdImpl<S, Halfspace<S>, Plane<S>>
{
  static bool run(
      const GJKSolver_libccd<S>& /*gjkSolver*/,
      const Halfspace<S>& s1,
      const Transform3<S>& tf1,
      const Plane<S>& s2,
      const Transform3<S>& tf2,
      std::vector<ContactPoint<S>>* contacts)
  {
    FCL_UNUSED(contacts);

    Plane<S> pl;
    Vector3<S> p, d;
    S depth;
    int ret;
    return detail::halfspacePlaneIntersect(s1, tf1, s2, tf2, pl, p, d, depth, ret);
  }
};

//==============================================================================
template<typename S, typename Shape>
struct ShapeTriangleIntersectLibccdImpl
{
  static bool run(
      const GJKSolver_libccd<S>& gjkSolver,
      const Shape& s,
      const Transform3<S>& tf,
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      Vector3<S>* contact_points,
      S* penetration_depth,
      Vector3<S>* normal)
  {
    void* o1 = detail::GJKInitializer<S, Shape>::createGJKObject(s, tf);
    void* o2 = detail::triCreateGJKObject(P1, P2, P3);

    bool res = detail::GJKCollide<S>(
          o1,
          detail::GJKInitializer<S, Shape>::getSupportFunction(),
          detail::GJKInitializer<S, Shape>::getCenterFunction(),
          o2,
          detail::triGetSupportFunction(),
          detail::triGetCenterFunction(),
          gjkSolver.max_collision_iterations,
          gjkSolver.collision_tolerance,
          contact_points,
          penetration_depth,
          normal);

    detail::GJKInitializer<S, Shape>::deleteGJKObject(o1);
    detail::triDeleteGJKObject(o2);

    return res;
  }
};

template<typename S>
template<typename Shape>
bool GJKSolver_libccd<S>::shapeTriangleIntersect(
    const Shape& s,
    const Transform3<S>& tf,
    const Vector3<S>& P1,
    const Vector3<S>& P2,
    const Vector3<S>& P3,
    Vector3<S>* contact_points,
    S* penetration_depth,
    Vector3<S>* normal) const
{
  return ShapeTriangleIntersectLibccdImpl<S, Shape>::run(
        *this, s, tf, P1, P2, P3, contact_points, penetration_depth, normal);
}

//==============================================================================
template<typename S>
struct ShapeTriangleIntersectLibccdImpl<S, Sphere<S>>
{
  static bool run(
      const GJKSolver_libccd<S>& /*gjkSolver*/,
      const Sphere<S>& s,
      const Transform3<S>& tf,
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      Vector3<S>* contact_points,
      S* penetration_depth,
      Vector3<S>* normal)
  {
    return detail::sphereTriangleIntersect(
          s, tf, P1, P2, P3, contact_points, penetration_depth, normal);
  }
};

//==============================================================================
template<typename S, typename Shape>
struct ShapeTransformedTriangleIntersectLibccdImpl
{
  static bool run(
      const GJKSolver_libccd<S>& gjkSolver,
      const Shape& s,
      const Transform3<S>& tf1,
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      const Transform3<S>& tf2,
      Vector3<S>* contact_points,
      S* penetration_depth,
      Vector3<S>* normal)
  {
    void* o1 = detail::GJKInitializer<S, Shape>::createGJKObject(s, tf1);
    void* o2 = detail::triCreateGJKObject(P1, P2, P3, tf2);

    bool res = detail::GJKCollide<S>(
          o1,
          detail::GJKInitializer<S, Shape>::getSupportFunction(),
          detail::GJKInitializer<S, Shape>::getCenterFunction(),
          o2,
          detail::triGetSupportFunction(),
          detail::triGetCenterFunction(),
          gjkSolver.max_collision_iterations,
          gjkSolver.collision_tolerance,
          contact_points,
          penetration_depth,
          normal);

    detail::GJKInitializer<S, Shape>::deleteGJKObject(o1);
    detail::triDeleteGJKObject(o2);

    return res;
  }
};

template<typename S>
template<typename Shape>
bool GJKSolver_libccd<S>::shapeTriangleIntersect(
    const Shape& s,
    const Transform3<S>& tf1,
    const Vector3<S>& P1,
    const Vector3<S>& P2,
    const Vector3<S>& P3,
    const Transform3<S>& tf2,
    Vector3<S>* contact_points,
    S* penetration_depth,
    Vector3<S>* normal) const
{
  return ShapeTransformedTriangleIntersectLibccdImpl<S, Shape>::run(
        *this, s, tf1, P1, P2, P3, tf2,
        contact_points, penetration_depth, normal);
}

//==============================================================================
template<typename S>
struct ShapeTransformedTriangleIntersectLibccdImpl<S, Sphere<S>>
{
  static bool run(
      const GJKSolver_libccd<S>& /*gjkSolver*/,
      const Sphere<S>& s,
      const Transform3<S>& tf1,
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      const Transform3<S>& tf2,
      Vector3<S>* contact_points,
      S* penetration_depth,
      Vector3<S>* normal)
  {
    return detail::sphereTriangleIntersect(
          s, tf1, tf2 * P1, tf2 * P2, tf2 * P3,
          contact_points, penetration_depth, normal);
  }
};

//==============================================================================
template<typename S>
struct ShapeTransformedTriangleIntersectLibccdImpl<S, Halfspace<S>>
{
  static bool run(
      const GJKSolver_libccd<S>& /*gjkSolver*/,
      const Halfspace<S>& s,
      const Transform3<S>& tf1,
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      const Transform3<S>& tf2,
      Vector3<S>* contact_points,
      S* penetration_depth,
      Vector3<S>* normal)
  {
    return detail::halfspaceTriangleIntersect(
          s, tf1, P1, P2, P3, tf2,
          contact_points, penetration_depth, normal);
  }
};

//==============================================================================
template<typename S>
struct ShapeTransformedTriangleIntersectLibccdImpl<S, Plane<S>>
{
  static bool run(
      const GJKSolver_libccd<S>& /*gjkSolver*/,
      const Plane<S>& s,
      const Transform3<S>& tf1,
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      const Transform3<S>& tf2,
      Vector3<S>* contact_points,
      S* penetration_depth,
      Vector3<S>* normal)
  {
    return detail::planeTriangleIntersect(
          s, tf1, P1, P2, P3, tf2,
          contact_points, penetration_depth, normal);
  }
};


//==============================================================================
//==============================================================================
template<typename S, typename Shape1, typename Shape2>
struct ShapeSignedDistanceLibccdImpl
{
  static bool run(
      const GJKSolver_libccd<S>& gjkSolver,
      const Shape1& s1,
      const Transform3<S>& tf1,
      const Shape2& s2,
      const Transform3<S>& tf2,
      S* dist,
      Vector3<S>* p1,
      Vector3<S>* p2)
  {
    void* o1 = detail::GJKInitializer<S, Shape1>::createGJKObject(s1, tf1);
    void* o2 = detail::GJKInitializer<S, Shape2>::createGJKObject(s2, tf2);

    bool res = detail::GJKSignedDistance(
          o1,
          detail::GJKInitializer<S, Shape1>::getSupportFunction(),
          o2,
          detail::GJKInitializer<S, Shape2>::getSupportFunction(),
          gjkSolver.max_distance_iterations,
          gjkSolver.distance_tolerance,
          dist,
          p1,
          p2);

    detail::GJKInitializer<S, Shape1>::deleteGJKObject(o1);
    detail::GJKInitializer<S, Shape2>::deleteGJKObject(o2);

    return res;
  }
};

template<typename S>
template<typename Shape1, typename Shape2>
bool GJKSolver_libccd<S>::shapeSignedDistance(
    const Shape1& s1,
    const Transform3<S>& tf1,
    const Shape2& s2,
    const Transform3<S>& tf2,
    S* dist,
    Vector3<S>* p1,
    Vector3<S>* p2) const
{
  bool result = false;
  try {
    result = ShapeSignedDistanceLibccdImpl<S, Shape1, Shape2>::run(
        *this, s1, tf1, s2, tf2, dist, p1, p2);
  } catch (const FailedAtThisConfiguration& e) {
    ThrowDetailedConfiguration(s1, tf1, s2, tf2, *this, e);
  }
  return result;
}


//==============================================================================
template<typename S, typename Shape1, typename Shape2>
struct ShapeDistanceLibccdImpl
{
  static bool run(
      const GJKSolver_libccd<S>& gjkSolver,
      const Shape1& s1,
      const Transform3<S>& tf1,
      const Shape2& s2,
      const Transform3<S>& tf2,
      S* dist,
      Vector3<S>* p1,
      Vector3<S>* p2)
  {
    void* o1 = detail::GJKInitializer<S, Shape1>::createGJKObject(s1, tf1);
    void* o2 = detail::GJKInitializer<S, Shape2>::createGJKObject(s2, tf2);

    bool res =  detail::GJKDistance(
          o1,
          detail::GJKInitializer<S, Shape1>::getSupportFunction(),
          o2,
          detail::GJKInitializer<S, Shape2>::getSupportFunction(),
          gjkSolver.max_distance_iterations,
          gjkSolver.distance_tolerance,
          dist,
          p1,
          p2);

    detail::GJKInitializer<S, Shape1>::deleteGJKObject(o1);
    detail::GJKInitializer<S, Shape2>::deleteGJKObject(o2);

    return res;
  }
};

template<typename S>
template<typename Shape1, typename Shape2>
bool GJKSolver_libccd<S>::shapeDistance(
    const Shape1& s1,
    const Transform3<S>& tf1,
    const Shape2& s2,
    const Transform3<S>& tf2,
    S* dist,
    Vector3<S>* p1,
    Vector3<S>* p2) const
{
  return ShapeDistanceLibccdImpl<S, Shape1, Shape2>::run(
        *this, s1, tf1, s2, tf2, dist, p1, p2);
}

// Shape distance algorithms not using libccd
//
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// |            | box | sphere | ellipsoid | capsule | cone | cylinder | plane | half-space | triangle |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | box        |     |   O    |           |         |      |          |       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | sphere     |/////|   O    |           |    O    |      |    O     |       |            |     O    |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | ellipsoid  |/////|////////|           |         |      |          |       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | capsule    |/////|////////|///////////|    O    |      |          |       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | cone       |/////|////////|///////////|/////////|      |          |       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | cylinder   |/////|////////|///////////|/////////|//////|          |       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | plane      |/////|////////|///////////|/////////|//////|//////////|       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | half-space |/////|////////|///////////|/////////|//////|//////////|///////|            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | triangle   |/////|////////|///////////|/////////|//////|//////////|///////|////////////|          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+

//==============================================================================
template<typename S>
struct ShapeDistanceLibccdImpl<S, Sphere<S>, Box<S>>
{
  static bool run(
      const GJKSolver_libccd<S>& /*gjkSolver*/,
      const Sphere<S>& s1,
      const Transform3<S>& tf1,
      const Box<S>& s2,
      const Transform3<S>& tf2,
      S* dist,
      Vector3<S>* p1,
      Vector3<S>* p2)
  {
    return detail::sphereBoxDistance(s1, tf1, s2, tf2, dist, p1, p2);
  }
};

//==============================================================================
template<typename S>
struct ShapeDistanceLibccdImpl<S, Box<S>, Sphere<S>>
{
  static bool run(
      const GJKSolver_libccd<S>& /*gjkSolver*/,
      const Box<S>& s1,
      const Transform3<S>& tf1,
      const Sphere<S>& s2,
      const Transform3<S>& tf2,
      S* dist,
      Vector3<S>* p1,
      Vector3<S>* p2)
  {
    return detail::sphereBoxDistance(s2, tf2, s1, tf1, dist, p2, p1);
  }
};

//==============================================================================
template<typename S>
struct ShapeDistanceLibccdImpl<S, Sphere<S>, Capsule<S>>
{
  static bool run(
      const GJKSolver_libccd<S>& /*gjkSolver*/,
      const Sphere<S>& s1,
      const Transform3<S>& tf1,
      const Capsule<S>& s2,
      const Transform3<S>& tf2,
      S* dist,
      Vector3<S>* p1,
      Vector3<S>* p2)
  {
    return detail::sphereCapsuleDistance(s1, tf1, s2, tf2, dist, p1, p2);
  }
};

//==============================================================================
template<typename S>
struct ShapeDistanceLibccdImpl<S, Capsule<S>, Sphere<S>>
{
  static bool run(
      const GJKSolver_libccd<S>& /*gjkSolver*/,
      const Capsule<S>& s1,
      const Transform3<S>& tf1,
      const Sphere<S>& s2,
      const Transform3<S>& tf2,
      S* dist,
      Vector3<S>* p1,
      Vector3<S>* p2)
  {
    return detail::sphereCapsuleDistance(s2, tf2, s1, tf1, dist, p2, p1);
  }
};

//==============================================================================
template<typename S>
struct ShapeDistanceLibccdImpl<S, Sphere<S>, Cylinder<S>>
{
  static bool run(
      const GJKSolver_libccd<S>& /*gjkSolver*/,
      const Sphere<S>& s1,
      const Transform3<S>& tf1,
      const Cylinder<S>& s2,
      const Transform3<S>& tf2,
      S* dist,
      Vector3<S>* p1,
      Vector3<S>* p2)
  {
    return detail::sphereCylinderDistance(s1, tf1, s2, tf2, dist, p1, p2);
  }
};

//==============================================================================
template<typename S>
struct ShapeDistanceLibccdImpl<S, Cylinder<S>, Sphere<S>>
{
  static bool run(
      const GJKSolver_libccd<S>& /*gjkSolver*/,
      const Cylinder<S>& s1,
      const Transform3<S>& tf1,
      const Sphere<S>& s2,
      const Transform3<S>& tf2,
      S* dist,
      Vector3<S>* p1,
      Vector3<S>* p2)
  {
    return detail::sphereCylinderDistance(s2, tf2, s1, tf1, dist, p2, p1);
  }
};

//==============================================================================
template<typename S>
struct ShapeDistanceLibccdImpl<S, Sphere<S>, Sphere<S>>
{
  static bool run(
      const GJKSolver_libccd<S>& /*gjkSolver*/,
      const Sphere<S>& s1,
      const Transform3<S>& tf1,
      const Sphere<S>& s2,
      const Transform3<S>& tf2,
      S* dist,
      Vector3<S>* p1,
      Vector3<S>* p2)
  {
    return detail::sphereSphereDistance(s1, tf1, s2, tf2, dist, p1, p2);
  }
};

//==============================================================================
template<typename S>
struct ShapeDistanceLibccdImpl<S, Capsule<S>, Capsule<S>>
{
  static bool run(
      const GJKSolver_libccd<S>& /*gjkSolver*/,
      const Capsule<S>& s1,
      const Transform3<S>& tf1,
      const Capsule<S>& s2,
      const Transform3<S>& tf2,
      S* dist,
      Vector3<S>* p1,
      Vector3<S>* p2)
  {
    return detail::capsuleCapsuleDistance(s1, tf1, s2, tf2, dist, p1, p2);
  }
};

//==============================================================================
template<typename S, typename Shape>
struct ShapeTriangleDistanceLibccdImpl
{
  static bool run(
      const GJKSolver_libccd<S>& gjkSolver,
      const Shape& s,
      const Transform3<S>& tf,
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      S* dist,
      Vector3<S>* p1,
      Vector3<S>* p2)
  {
    void* o1 = detail::GJKInitializer<S, Shape>::createGJKObject(s, tf);
    void* o2 = detail::triCreateGJKObject(P1, P2, P3);

    bool res = detail::GJKDistance(
          o1,
          detail::GJKInitializer<S, Shape>::getSupportFunction(),
          o2,
          detail::triGetSupportFunction(),
          gjkSolver.max_distance_iterations,
          gjkSolver.distance_tolerance,
          dist,
          p1,
          p2);

    detail::GJKInitializer<S, Shape>::deleteGJKObject(o1);
    detail::triDeleteGJKObject(o2);

    return res;
  }
};

//==============================================================================
template<typename S>
template<typename Shape>
bool GJKSolver_libccd<S>::shapeTriangleDistance(
    const Shape& s,
    const Transform3<S>& tf,
    const Vector3<S>& P1,
    const Vector3<S>& P2,
    const Vector3<S>& P3,
    S* dist,
    Vector3<S>* p1,
    Vector3<S>* p2) const
{
  return ShapeTriangleDistanceLibccdImpl<S, Shape>::run(
        *this, s, tf, P1, P2, P3, dist, p1, p2);
}

//==============================================================================
template<typename S>
struct ShapeTriangleDistanceLibccdImpl<S, Sphere<S>>
{
  static bool run(
      const GJKSolver_libccd<S>& /*gjkSolver*/,
      const Sphere<S>& s,
      const Transform3<S>& tf,
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      S* dist,
      Vector3<S>* p1,
      Vector3<S>* p2)
  {
    return detail::sphereTriangleDistance(s, tf, P1, P2, P3, dist, p1, p2);
  }
};

//==============================================================================
template<typename S, typename Shape>
struct ShapeTransformedTriangleDistanceLibccdImpl
{
  static bool run(
      const GJKSolver_libccd<S>& gjkSolver,
      const Shape& s,
      const Transform3<S>& tf1,
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      const Transform3<S>& tf2,
      S* dist,
      Vector3<S>* p1,
      Vector3<S>* p2)
  {
    void* o1 = detail::GJKInitializer<S, Shape>::createGJKObject(s, tf1);
    void* o2 = detail::triCreateGJKObject(P1, P2, P3, tf2);

    bool res = detail::GJKDistance(
          o1,
          detail::GJKInitializer<S, Shape>::getSupportFunction(),
          o2,
          detail::triGetSupportFunction(),
          gjkSolver.max_distance_iterations,
          gjkSolver.distance_tolerance,
          dist,
          p1,
          p2);

    detail::GJKInitializer<S, Shape>::deleteGJKObject(o1);
    detail::triDeleteGJKObject(o2);

    return res;
  }
};

//==============================================================================
template<typename S>
template<typename Shape>
bool GJKSolver_libccd<S>::shapeTriangleDistance(
    const Shape& s,
    const Transform3<S>& tf1,
    const Vector3<S>& P1,
    const Vector3<S>& P2,
    const Vector3<S>& P3,
    const Transform3<S>& tf2,
    S* dist,
    Vector3<S>* p1,
    Vector3<S>* p2) const
{
  return ShapeTransformedTriangleDistanceLibccdImpl<S, Shape>::run(
        *this, s, tf1, P1, P2, P3, tf2, dist, p1, p2);
}

//==============================================================================
template<typename S>
struct ShapeTransformedTriangleDistanceLibccdImpl<S, Sphere<S>>
{
  static bool run(
      const GJKSolver_libccd<S>& /*gjkSolver*/,
      const Sphere<S>& s,
      const Transform3<S>& tf1,
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      const Transform3<S>& tf2,
      S* dist,
      Vector3<S>* p1,
      Vector3<S>* p2)
  {
    return detail::sphereTriangleDistance(
          s, tf1, P1, P2, P3, tf2, dist, p1, p2);
  }
};

//==============================================================================
template<typename S>
GJKSolver_libccd<S>::GJKSolver_libccd()
{
  max_collision_iterations = 500;
  max_distance_iterations = 1000;
  collision_tolerance = constants<S>::gjk_default_tolerance();
  distance_tolerance = 1e-6;
}

//==============================================================================
template<typename S>
void GJKSolver_libccd<S>::enableCachedGuess(bool if_enable) const
{
  FCL_UNUSED(if_enable);

  // TODO: need change libccd to exploit spatial coherence
}

//==============================================================================
template<typename S>
void GJKSolver_libccd<S>::setCachedGuess(
    const Vector3<S>& guess) const
{
  FCL_UNUSED(guess);

  // TODO: need change libccd to exploit spatial coherence
}

//==============================================================================
template<typename S>
Vector3<S> GJKSolver_libccd<S>::getCachedGuess() const
{
  return Vector3<S>(-1, 0, 0);
}

} // namespace detail
} // namespace fcl

#endif

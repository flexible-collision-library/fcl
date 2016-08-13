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

#ifndef FCL_NARROWPHASE_GJKSOLVERLIBCCD_H
#define FCL_NARROWPHASE_GJKSOLVERLIBCCD_H

#include <algorithm>

#include "fcl/collision_data.h"
#include "fcl/narrowphase/detail/gjk_libccd.h"
#include "fcl/narrowphase/detail/primitive_shape_algorithms.h"

namespace fcl
{

/// @brief collision and distance solver based on libccd library.
template <typename S_>
struct GJKSolver_libccd
{
  using S = S_;

  /// @brief intersection checking between two shapes
  /// @deprecated use shapeIntersect(const Shape1&, const Transform3<S>&, const Shape2&, const Transform3<S>&, std::vector<ContactPoint<S>>*) const
  template<typename Shape1, typename Shape2>
  FCL_DEPRECATED
  bool shapeIntersect(
      const Shape1& s1,
      const Transform3<S>& tf1,
      const Shape2& s2,
      const Transform3<S>& tf2,
      Vector3<S>* contact_points,
      S* penetration_depth,
      Vector3<S>* normal) const;

  /// @brief intersection checking between two shapes
  template<typename Shape1, typename Shape2>
  bool shapeIntersect(
      const Shape1& s1,
      const Transform3<S>& tf1,
      const Shape2& s2,
      const Transform3<S>& tf2,
      std::vector<ContactPoint<S>>* contacts = nullptr) const;

  /// @brief intersection checking between one shape and a triangle
  template<typename Shape>
  bool shapeTriangleIntersect(
      const Shape& s,
      const Transform3<S>& tf,
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      Vector3<S>* contact_points = nullptr,
      S* penetration_depth = nullptr,
      Vector3<S>* normal = nullptr) const;

  //// @brief intersection checking between one shape and a triangle with transformation
  template<typename Shape>
  bool shapeTriangleIntersect(
      const Shape& s,
      const Transform3<S>& tf1,
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      const Transform3<S>& tf2,
      Vector3<S>* contact_points = nullptr,
      S* penetration_depth = nullptr,
      Vector3<S>* normal = nullptr) const;

  /// @brief distance computation between two shapes
  template<typename Shape1, typename Shape2>
  bool shapeDistance(
      const Shape1& s1,
      const Transform3<S>& tf1,
      const Shape2& s2,
      const Transform3<S>& tf2,
      S* dist = nullptr,
      Vector3<S>* p1 = nullptr,
      Vector3<S>* p2 = nullptr) const;

  /// @brief distance computation between one shape and a triangle
  template<typename Shape>
  bool shapeTriangleDistance(
      const Shape& s,
      const Transform3<S>& tf,
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      S* dist = nullptr,
      Vector3<S>* p1 = nullptr,
      Vector3<S>* p2 = nullptr) const;
  
  /// @brief distance computation between one shape and a triangle with transformation
  template<typename Shape>
  bool shapeTriangleDistance(
      const Shape& s,
      const Transform3<S>& tf1,
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      const Transform3<S>& tf2,
      S* dist = nullptr,
      Vector3<S>* p1 = nullptr,
      Vector3<S>* p2 = nullptr) const;

  /// @brief default setting for GJK algorithm
  GJKSolver_libccd();

  void enableCachedGuess(bool if_enable) const;

  void setCachedGuess(const Vector3<S>& guess) const;

  Vector3<S> getCachedGuess() const;

  /// @brief maximum number of iterations used in GJK algorithm for collision
  unsigned int max_collision_iterations;

  /// @brief maximum number of iterations used in GJK algorithm for distance
  unsigned int max_distance_iterations;

  /// @brief the threshold used in GJK algorithm to stop collision iteration
  S collision_tolerance;

  /// @brief the threshold used in GJK algorithm to stop distance iteration
  S distance_tolerance;

};

using GJKSolver_libccdf = GJKSolver_libccd<float>;
using GJKSolver_libccdd = GJKSolver_libccd<double>;

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

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
    void* o1 = details::GJKInitializer<S, Shape1>::createGJKObject(s1, tf1);
    void* o2 = details::GJKInitializer<S, Shape2>::createGJKObject(s2, tf2);

    bool res;

    if(contacts)
    {
      Vector3<S> normal;
      Vector3<S> point;
      S depth;
      res = details::GJKCollide<S>(
            o1,
            details::GJKInitializer<S, Shape1>::getSupportFunction(),
            details::GJKInitializer<S, Shape1>::getCenterFunction(),
            o2, details::GJKInitializer<S, Shape2>::getSupportFunction(),
            details::GJKInitializer<S, Shape2>::getCenterFunction(),
            gjkSolver.max_collision_iterations,
            gjkSolver.collision_tolerance,
            &point,
            &depth,
            &normal);
      contacts->emplace_back(normal, point, depth);
    }
    else
    {
      res = details::GJKCollide<S>(
            o1,
            details::GJKInitializer<S, Shape1>::getSupportFunction(),
            details::GJKInitializer<S, Shape1>::getCenterFunction(),
            o2,
            details::GJKInitializer<S, Shape2>::getSupportFunction(),
            details::GJKInitializer<S, Shape2>::getCenterFunction(),
            gjkSolver.max_collision_iterations,
            gjkSolver.collision_tolerance,
            nullptr,
            nullptr,
            nullptr);
    }

    details::GJKInitializer<S, Shape1>::deleteGJKObject(o1);
    details::GJKInitializer<S, Shape2>::deleteGJKObject(o2);

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
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// |            | box | sphere | ellipsoid | capsule | cone | cylinder | plane | half-space | triangle |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | box        |  O  |        |           |         |      |          |   O   |      O     |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | sphere     |/////|   O    |           |    O    |      |          |   O   |      O     |    O     |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | ellipsoid  |/////|////////|           |         |      |          |   O   |      O     |   TODO   |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | capsule    |/////|////////|///////////|         |      |          |   O   |      O     |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | cone       |/////|////////|///////////|/////////|      |          |   O   |      O     |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | cylinder   |/////|////////|///////////|/////////|//////|          |   O   |      O     |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | plane      |/////|////////|///////////|/////////|//////|//////////|   O   |      O     |    O     |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | half-space |/////|////////|///////////|/////////|//////|//////////|///////|      O     |    O     |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | triangle   |/////|////////|///////////|/////////|//////|//////////|///////|////////////|          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+

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

FCL_GJK_LIBCCD_SHAPE_INTERSECT(Sphere, details::sphereSphereIntersect)
FCL_GJK_LIBCCD_SHAPE_INTERSECT(Box, details::boxBoxIntersect)

FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Sphere, Capsule, details::sphereCapsuleIntersect)

FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Sphere, Halfspace, details::sphereHalfspaceIntersect)
FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Ellipsoid, Halfspace, details::ellipsoidHalfspaceIntersect)
FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Box, Halfspace, details::boxHalfspaceIntersect)
FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Capsule, Halfspace, details::capsuleHalfspaceIntersect)
FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Cylinder, Halfspace, details::cylinderHalfspaceIntersect)
FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Cone, Halfspace, details::coneHalfspaceIntersect)

FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Sphere, Plane, details::spherePlaneIntersect)
FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Ellipsoid, Plane, details::ellipsoidPlaneIntersect)
FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Box, Plane, details::boxPlaneIntersect)
FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Capsule, Plane, details::capsulePlaneIntersect)
FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Cylinder, Plane, details::cylinderPlaneIntersect)
FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Cone, Plane, details::conePlaneIntersect)

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
    Halfspace<S> s;
    Vector3<S> p, d;
    S depth;
    int ret;
    return details::halfspaceIntersect(s1, tf1, s2, tf2, p, d, s, depth, ret);
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
    return details::planeIntersect(s1, tf1, s2, tf2, contacts);
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
    Plane<S> pl;
    Vector3<S> p, d;
    S depth;
    int ret;
    return details::planeHalfspaceIntersect(s1, tf1, s2, tf2, pl, p, d, depth, ret);
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
    Plane<S> pl;
    Vector3<S> p, d;
    S depth;
    int ret;
    return details::halfspacePlaneIntersect(s1, tf1, s2, tf2, pl, p, d, depth, ret);
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
    void* o1 = details::GJKInitializer<S, Shape>::createGJKObject(s, tf);
    void* o2 = details::triCreateGJKObject(P1, P2, P3);

    bool res = details::GJKCollide<S>(
          o1,
          details::GJKInitializer<S, Shape>::getSupportFunction(),
          details::GJKInitializer<S, Shape>::getCenterFunction(),
          o2,
          details::triGetSupportFunction(),
          details::triGetCenterFunction(),
          gjkSolver.max_collision_iterations,
          gjkSolver.collision_tolerance,
          contact_points,
          penetration_depth,
          normal);

    details::GJKInitializer<S, Shape>::deleteGJKObject(o1);
    details::triDeleteGJKObject(o2);

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
    return details::sphereTriangleIntersect(
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
    void* o1 = details::GJKInitializer<S, Shape>::createGJKObject(s, tf1);
    void* o2 = details::triCreateGJKObject(P1, P2, P3, tf2);

    bool res = details::GJKCollide<S>(
          o1,
          details::GJKInitializer<S, Shape>::getSupportFunction(),
          details::GJKInitializer<S, Shape>::getCenterFunction(),
          o2,
          details::triGetSupportFunction(),
          details::triGetCenterFunction(),
          gjkSolver.max_collision_iterations,
          gjkSolver.collision_tolerance,
          contact_points,
          penetration_depth,
          normal);

    details::GJKInitializer<S, Shape>::deleteGJKObject(o1);
    details::triDeleteGJKObject(o2);

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
    return details::sphereTriangleIntersect(
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
    return details::halfspaceTriangleIntersect(
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
    return details::planeTriangleIntersect(
          s, tf1, P1, P2, P3, tf2,
          contact_points, penetration_depth, normal);
  }
};

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
    void* o1 = details::GJKInitializer<S, Shape1>::createGJKObject(s1, tf1);
    void* o2 = details::GJKInitializer<S, Shape2>::createGJKObject(s2, tf2);

    bool res =  details::GJKDistance(
          o1,
          details::GJKInitializer<S, Shape1>::getSupportFunction(),
          o2,
          details::GJKInitializer<S, Shape2>::getSupportFunction(),
          gjkSolver.max_distance_iterations,
          gjkSolver.distance_tolerance,
          dist,
          p1,
          p2);

    if (p1)
      (*p1).noalias() = tf1.inverse(Eigen::Isometry) * *p1;

    if (p2)
      (*p2).noalias() = tf2.inverse(Eigen::Isometry) * *p2;

    details::GJKInitializer<S, Shape1>::deleteGJKObject(o1);
    details::GJKInitializer<S, Shape2>::deleteGJKObject(o2);

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
// | box        |     |        |           |         |      |          |       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | sphere     |/////|   O    |           |    O    |      |          |       |            |     O    |
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
    return details::sphereCapsuleDistance(s1, tf1, s2, tf2, dist, p1, p2);
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
    return details::sphereCapsuleDistance(s2, tf2, s1, tf1, dist, p2, p1);
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
    return details::sphereSphereDistance(s1, tf1, s2, tf2, dist, p1, p2);
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
    return details::capsuleCapsuleDistance(s1, tf1, s2, tf2, dist, p1, p2);
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
    void* o1 = details::GJKInitializer<S, Shape>::createGJKObject(s, tf);
    void* o2 = details::triCreateGJKObject(P1, P2, P3);

    bool res = details::GJKDistance(
          o1,
          details::GJKInitializer<S, Shape>::getSupportFunction(),
          o2,
          details::triGetSupportFunction(),
          gjkSolver.max_distance_iterations,
          gjkSolver.distance_tolerance,
          dist,
          p1,
          p2);
    if(p1)
      (*p1).noalias() = tf.inverse(Eigen::Isometry) * *p1;

    details::GJKInitializer<S, Shape>::deleteGJKObject(o1);
    details::triDeleteGJKObject(o2);

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
    return details::sphereTriangleDistance(s, tf, P1, P2, P3, dist, p1, p2);
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
    void* o1 = details::GJKInitializer<S, Shape>::createGJKObject(s, tf1);
    void* o2 = details::triCreateGJKObject(P1, P2, P3, tf2);

    bool res = details::GJKDistance(
          o1,
          details::GJKInitializer<S, Shape>::getSupportFunction(),
          o2,
          details::triGetSupportFunction(),
          gjkSolver.max_distance_iterations,
          gjkSolver.distance_tolerance,
          dist,
          p1,
          p2);
    if(p1)
      (*p1).noalias() = tf1.inverse(Eigen::Isometry) * *p1;
    if(p2)
      (*p2).noalias() = tf2.inverse(Eigen::Isometry) * *p2;

    details::GJKInitializer<S, Shape>::deleteGJKObject(o1);
    details::triDeleteGJKObject(o2);

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
    return details::sphereTriangleDistance(
          s, tf1, P1, P2, P3, tf2, dist, p1, p2);
  }
};

//==============================================================================
template<typename S>
GJKSolver_libccd<S>::GJKSolver_libccd()
{
  max_collision_iterations = 500;
  max_distance_iterations = 1000;
  collision_tolerance = 1e-6;
  distance_tolerance = 1e-6;
}

//==============================================================================
template<typename S>
void GJKSolver_libccd<S>::enableCachedGuess(bool if_enable) const
{
  // TODO: need change libccd to exploit spatial coherence
}

//==============================================================================
template<typename S>
void GJKSolver_libccd<S>::setCachedGuess(
    const Vector3<S>& guess) const
{
  // TODO: need change libccd to exploit spatial coherence
}

//==============================================================================
template<typename S>
Vector3<S> GJKSolver_libccd<S>::getCachedGuess() const
{
  return Vector3<S>(-1, 0, 0);
}

} // namespace fcl

#endif

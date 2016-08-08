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
template <typename ScalarT>
struct GJKSolver_libccd
{
  using Scalar = ScalarT;

  /// @brief intersection checking between two shapes
  /// @deprecated use shapeIntersect(const S1&, const Transform3<Scalar>&, const S2&, const Transform3<Scalar>&, std::vector<ContactPoint<Scalar>>*) const
  template<typename S1, typename S2>
  FCL_DEPRECATED
  bool shapeIntersect(
      const S1& s1,
      const Transform3<Scalar>& tf1,
      const S2& s2,
      const Transform3<Scalar>& tf2,
      Vector3<Scalar>* contact_points,
      Scalar* penetration_depth,
      Vector3<Scalar>* normal) const;

  /// @brief intersection checking between two shapes
  template<typename S1, typename S2>
  bool shapeIntersect(
      const S1& s1,
      const Transform3<Scalar>& tf1,
      const S2& s2,
      const Transform3<Scalar>& tf2,
      std::vector<ContactPoint<Scalar>>* contacts = NULL) const;

  /// @brief intersection checking between one shape and a triangle
  template<typename S>
  bool shapeTriangleIntersect(
      const S& s,
      const Transform3<Scalar>& tf,
      const Vector3<Scalar>& P1,
      const Vector3<Scalar>& P2,
      const Vector3<Scalar>& P3,
      Vector3<Scalar>* contact_points = NULL,
      Scalar* penetration_depth = NULL,
      Vector3<Scalar>* normal = NULL) const;

  //// @brief intersection checking between one shape and a triangle with transformation
  template<typename S>
  bool shapeTriangleIntersect(
      const S& s,
      const Transform3<Scalar>& tf1,
      const Vector3<Scalar>& P1,
      const Vector3<Scalar>& P2,
      const Vector3<Scalar>& P3,
      const Transform3<Scalar>& tf2,
      Vector3<Scalar>* contact_points = NULL,
      Scalar* penetration_depth = NULL,
      Vector3<Scalar>* normal = NULL) const;

  /// @brief distance computation between two shapes
  template<typename S1, typename S2>
  bool shapeDistance(
      const S1& s1,
      const Transform3<Scalar>& tf1,
      const S2& s2,
      const Transform3<Scalar>& tf2,
      Scalar* dist = NULL,
      Vector3<Scalar>* p1 = NULL,
      Vector3<Scalar>* p2 = NULL) const;

  /// @brief distance computation between one shape and a triangle
  template<typename S>
  bool shapeTriangleDistance(
      const S& s,
      const Transform3<Scalar>& tf,
      const Vector3<Scalar>& P1,
      const Vector3<Scalar>& P2,
      const Vector3<Scalar>& P3,
      Scalar* dist = NULL,
      Vector3<Scalar>* p1 = NULL,
      Vector3<Scalar>* p2 = NULL) const;
  
  /// @brief distance computation between one shape and a triangle with transformation
  template<typename S>
  bool shapeTriangleDistance(
      const S& s,
      const Transform3<Scalar>& tf1,
      const Vector3<Scalar>& P1,
      const Vector3<Scalar>& P2,
      const Vector3<Scalar>& P3,
      const Transform3<Scalar>& tf2,
      Scalar* dist = NULL,
      Vector3<Scalar>* p1 = NULL,
      Vector3<Scalar>* p2 = NULL) const;

  /// @brief default setting for GJK algorithm
  GJKSolver_libccd();

  void enableCachedGuess(bool if_enable) const;

  void setCachedGuess(const Vector3<Scalar>& guess) const;

  Vector3<Scalar> getCachedGuess() const;

  /// @brief maximum number of iterations used in GJK algorithm for collision
  unsigned int max_collision_iterations;

  /// @brief maximum number of iterations used in GJK algorithm for distance
  unsigned int max_distance_iterations;

  /// @brief the threshold used in GJK algorithm to stop collision iteration
  Scalar collision_tolerance;

  /// @brief the threshold used in GJK algorithm to stop distance iteration
  Scalar distance_tolerance;

};

using GJKSolver_libccdf = GJKSolver_libccd<float>;
using GJKSolver_libccdd = GJKSolver_libccd<double>;

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template<typename Scalar>
template<typename S1, typename S2>
bool GJKSolver_libccd<Scalar>::shapeIntersect(
    const S1& s1, const Transform3<Scalar>& tf1,
    const S2& s2, const Transform3<Scalar>& tf2,
    Vector3<Scalar>* contact_points,
    Scalar* penetration_depth,
    Vector3<Scalar>* normal) const
{
  bool res;

  if (contact_points || penetration_depth || normal)
  {
    std::vector<ContactPoint<Scalar>> contacts;

    res = shapeIntersect(s1, tf1, s2, tf2, &contacts);

    if (!contacts.empty())
    {
      // Get the deepest contact point
      const ContactPoint<Scalar>& maxDepthContact = *std::max_element(contacts.begin(), contacts.end(), comparePenDepth<Scalar>);

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
    res = shapeIntersect(s1, tf1, s2, tf2, NULL);
  }

  return res;
}

//==============================================================================
template<typename Scalar, typename S1, typename S2>
struct ShapeIntersectLibccdImpl
{
  bool operator()(
      const GJKSolver_libccd<Scalar>& gjkSolver,
      const S1& s1, const Transform3<Scalar>& tf1,
      const S2& s2, const Transform3<Scalar>& tf2,
      std::vector<ContactPoint<Scalar>>* contacts)
  {
    void* o1 = details::GJKInitializer<Scalar, S1>::createGJKObject(s1, tf1);
    void* o2 = details::GJKInitializer<Scalar, S2>::createGJKObject(s2, tf2);

    bool res;

    if(contacts)
    {
      Vector3<Scalar> normal;
      Vector3<Scalar> point;
      Scalar depth;
      res = details::GJKCollide<Scalar>(
            o1,
            details::GJKInitializer<Scalar, S1>::getSupportFunction(),
            details::GJKInitializer<Scalar, S1>::getCenterFunction(),
            o2, details::GJKInitializer<Scalar, S2>::getSupportFunction(),
            details::GJKInitializer<Scalar, S2>::getCenterFunction(),
            gjkSolver.max_collision_iterations,
            gjkSolver.collision_tolerance,
            &point,
            &depth,
            &normal);
      contacts->push_back(ContactPoint<Scalar>(normal, point, depth));
    }
    else
    {
      res = details::GJKCollide<Scalar>(
            o1,
            details::GJKInitializer<Scalar, S1>::getSupportFunction(),
            details::GJKInitializer<Scalar, S1>::getCenterFunction(),
            o2,
            details::GJKInitializer<Scalar, S2>::getSupportFunction(),
            details::GJKInitializer<Scalar, S2>::getCenterFunction(),
            gjkSolver.max_collision_iterations,
            gjkSolver.collision_tolerance,
            NULL,
            NULL,
            NULL);
    }

    details::GJKInitializer<Scalar, S1>::deleteGJKObject(o1);
    details::GJKInitializer<Scalar, S2>::deleteGJKObject(o2);

    return res;
  }
};

//==============================================================================
template<typename Scalar>
template<typename S1, typename S2>
bool GJKSolver_libccd<Scalar>::shapeIntersect(
    const S1& s1, const Transform3<Scalar>& tf1,
    const S2& s2, const Transform3<Scalar>& tf2,
    std::vector<ContactPoint<Scalar>>* contacts) const
{
  ShapeIntersectLibccdImpl<Scalar, S1, S2> shapeIntersectImpl;
  return shapeIntersectImpl(*this, s1, tf1, s2, tf2, contacts);
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
  template <typename Scalar>\
  struct ShapeIntersectLibccdImpl<Scalar, SHAPE1<Scalar>, SHAPE2<Scalar>>\
  {\
    bool operator()(\
        const GJKSolver_libccd<Scalar>& /*gjkSolver*/,\
        const SHAPE1<Scalar>& s1,\
        const Transform3<Scalar>& tf1,\
        const SHAPE2<Scalar>& s2,\
        const Transform3<Scalar>& tf2,\
        std::vector<ContactPoint<Scalar>>* contacts)\
    {\
      return ALG(s1, tf1, s2, tf2, contacts);\
    }\
  };

#define FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT_INV(SHAPE1, SHAPE2, ALG)\
  template <typename Scalar>\
  struct ShapeIntersectLibccdImpl<Scalar, SHAPE2<Scalar>, SHAPE1<Scalar>>\
  {\
    bool operator()(\
        const GJKSolver_libccd<Scalar>& /*gjkSolver*/,\
        const SHAPE2<Scalar>& s1,\
        const Transform3<Scalar>& tf1,\
        const SHAPE1<Scalar>& s2,\
        const Transform3<Scalar>& tf2,\
        std::vector<ContactPoint<Scalar>>* contacts)\
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

template <typename Scalar>
struct ShapeIntersectLibccdImpl<Scalar, Halfspace<Scalar>, Halfspace<Scalar>>
{
  bool operator()(
      const GJKSolver_libccd<Scalar>& /*gjkSolver*/,
      const Halfspace<Scalar>& s1,
      const Transform3<Scalar>& tf1,
      const Halfspace<Scalar>& s2,
      const Transform3<Scalar>& tf2,
      std::vector<ContactPoint<Scalar>>* contacts)
  {
    Halfspace<Scalar> s;
    Vector3<Scalar> p, d;
    Scalar depth;
    int ret;
    return details::halfspaceIntersect(s1, tf1, s2, tf2, p, d, s, depth, ret);
  }
};

template <typename Scalar>
struct ShapeIntersectLibccdImpl<Scalar, Plane<Scalar>, Plane<Scalar>>
{
  bool operator()(
      const GJKSolver_libccd<Scalar>& /*gjkSolver*/,
      const Plane<Scalar>& s1,
      const Transform3<Scalar>& tf1,
      const Plane<Scalar>& s2,
      const Transform3<Scalar>& tf2,
      std::vector<ContactPoint<Scalar>>* contacts)
  {
    return details::planeIntersect(s1, tf1, s2, tf2, contacts);
  }
};

template <typename Scalar>
struct ShapeIntersectLibccdImpl<Scalar, Plane<Scalar>, Halfspace<Scalar>>
{
  bool operator()(
      const GJKSolver_libccd<Scalar>& /*gjkSolver*/,
      const Plane<Scalar>& s1,
      const Transform3<Scalar>& tf1,
      const Halfspace<Scalar>& s2,
      const Transform3<Scalar>& tf2,
      std::vector<ContactPoint<Scalar>>* contacts)
  {
    Plane<Scalar> pl;
    Vector3<Scalar> p, d;
    Scalar depth;
    int ret;
    return details::planeHalfspaceIntersect(s1, tf1, s2, tf2, pl, p, d, depth, ret);
  }
};

template <typename Scalar>
struct ShapeIntersectLibccdImpl<Scalar, Halfspace<Scalar>, Plane<Scalar>>
{
  bool operator()(
      const GJKSolver_libccd<Scalar>& /*gjkSolver*/,
      const Halfspace<Scalar>& s1,
      const Transform3<Scalar>& tf1,
      const Plane<Scalar>& s2,
      const Transform3<Scalar>& tf2,
      std::vector<ContactPoint<Scalar>>* contacts)
  {
    Plane<Scalar> pl;
    Vector3<Scalar> p, d;
    Scalar depth;
    int ret;
    return details::halfspacePlaneIntersect(s1, tf1, s2, tf2, pl, p, d, depth, ret);
  }
};

//==============================================================================
template<typename Scalar, typename S>
struct ShapeTriangleIntersectLibccdImpl
{
  bool operator()(
      const GJKSolver_libccd<Scalar>& gjkSolver,
      const S& s,
      const Transform3<Scalar>& tf,
      const Vector3<Scalar>& P1,
      const Vector3<Scalar>& P2,
      const Vector3<Scalar>& P3,
      Vector3<Scalar>* contact_points,
      Scalar* penetration_depth,
      Vector3<Scalar>* normal)
  {
    void* o1 = details::GJKInitializer<Scalar, S>::createGJKObject(s, tf);
    void* o2 = details::triCreateGJKObject(P1, P2, P3);

    bool res = details::GJKCollide<Scalar>(
          o1,
          details::GJKInitializer<Scalar, S>::getSupportFunction(),
          details::GJKInitializer<Scalar, S>::getCenterFunction(),
          o2,
          details::triGetSupportFunction(),
          details::triGetCenterFunction(),
          gjkSolver.max_collision_iterations,
          gjkSolver.collision_tolerance,
          contact_points,
          penetration_depth,
          normal);

    details::GJKInitializer<Scalar, S>::deleteGJKObject(o1);
    details::triDeleteGJKObject(o2);

    return res;
  }
};

template<typename Scalar>
template<typename S>
bool GJKSolver_libccd<Scalar>::shapeTriangleIntersect(
    const S& s,
    const Transform3<Scalar>& tf,
    const Vector3<Scalar>& P1,
    const Vector3<Scalar>& P2,
    const Vector3<Scalar>& P3,
    Vector3<Scalar>* contact_points,
    Scalar* penetration_depth,
    Vector3<Scalar>* normal) const
{
  ShapeTriangleIntersectLibccdImpl<Scalar, S> shapeTriangleIntersectImpl;
  return shapeTriangleIntersectImpl(
        *this, s, tf, P1, P2, P3, contact_points, penetration_depth, normal);
}

//==============================================================================
template<typename Scalar>
struct ShapeTriangleIntersectLibccdImpl<Scalar, Sphere<Scalar>>
{
  bool operator()(
      const GJKSolver_libccd<Scalar>& /*gjkSolver*/,
      const Sphere<Scalar>& s,
      const Transform3<Scalar>& tf,
      const Vector3<Scalar>& P1,
      const Vector3<Scalar>& P2,
      const Vector3<Scalar>& P3,
      Vector3<Scalar>* contact_points,
      Scalar* penetration_depth,
      Vector3<Scalar>* normal)
  {
    return details::sphereTriangleIntersect(
          s, tf, P1, P2, P3, contact_points, penetration_depth, normal);
  }
};

//==============================================================================
template<typename Scalar, typename S>
struct ShapeTransformedTriangleIntersectLibccdImpl
{
  bool operator()(
      const GJKSolver_libccd<Scalar>& gjkSolver,
      const S& s,
      const Transform3<Scalar>& tf1,
      const Vector3<Scalar>& P1,
      const Vector3<Scalar>& P2,
      const Vector3<Scalar>& P3,
      const Transform3<Scalar>& tf2,
      Vector3<Scalar>* contact_points,
      Scalar* penetration_depth,
      Vector3<Scalar>* normal)
  {
    void* o1 = details::GJKInitializer<Scalar, S>::createGJKObject(s, tf1);
    void* o2 = details::triCreateGJKObject(P1, P2, P3, tf2);

    bool res = details::GJKCollide<Scalar>(
          o1,
          details::GJKInitializer<Scalar, S>::getSupportFunction(),
          details::GJKInitializer<Scalar, S>::getCenterFunction(),
          o2,
          details::triGetSupportFunction(),
          details::triGetCenterFunction(),
          gjkSolver.max_collision_iterations,
          gjkSolver.collision_tolerance,
          contact_points,
          penetration_depth,
          normal);

    details::GJKInitializer<Scalar, S>::deleteGJKObject(o1);
    details::triDeleteGJKObject(o2);

    return res;
  }
};

template<typename Scalar>
template<typename S>
bool GJKSolver_libccd<Scalar>::shapeTriangleIntersect(
    const S& s,
    const Transform3<Scalar>& tf1,
    const Vector3<Scalar>& P1,
    const Vector3<Scalar>& P2,
    const Vector3<Scalar>& P3,
    const Transform3<Scalar>& tf2,
    Vector3<Scalar>* contact_points,
    Scalar* penetration_depth,
    Vector3<Scalar>* normal) const
{
  ShapeTransformedTriangleIntersectLibccdImpl<Scalar, S> shapeTriangleIntersectImpl;
  return shapeTriangleIntersectImpl(
        *this, s, tf1, P1, P2, P3, tf2,
        contact_points, penetration_depth, normal);
}

//==============================================================================
template<typename Scalar>
struct ShapeTransformedTriangleIntersectLibccdImpl<Scalar, Sphere<Scalar>>
{
  bool operator()(
      const GJKSolver_libccd<Scalar>& /*gjkSolver*/,
      const Sphere<Scalar>& s,
      const Transform3<Scalar>& tf1,
      const Vector3<Scalar>& P1,
      const Vector3<Scalar>& P2,
      const Vector3<Scalar>& P3,
      const Transform3<Scalar>& tf2,
      Vector3<Scalar>* contact_points,
      Scalar* penetration_depth,
      Vector3<Scalar>* normal)
  {
    return details::sphereTriangleIntersect(
          s, tf1, tf2 * P1, tf2 * P2, tf2 * P3,
          contact_points, penetration_depth, normal);
  }
};

//==============================================================================
template<typename Scalar>
struct ShapeTransformedTriangleIntersectLibccdImpl<Scalar, Halfspace<Scalar>>
{
  bool operator()(
      const GJKSolver_libccd<Scalar>& /*gjkSolver*/,
      const Halfspace<Scalar>& s,
      const Transform3<Scalar>& tf1,
      const Vector3<Scalar>& P1,
      const Vector3<Scalar>& P2,
      const Vector3<Scalar>& P3,
      const Transform3<Scalar>& tf2,
      Vector3<Scalar>* contact_points,
      Scalar* penetration_depth,
      Vector3<Scalar>* normal)
  {
    return details::halfspaceTriangleIntersect(
          s, tf1, P1, P2, P3, tf2,
          contact_points, penetration_depth, normal);
  }
};

//==============================================================================
template<typename Scalar>
struct ShapeTransformedTriangleIntersectLibccdImpl<Scalar, Plane<Scalar>>
{
  bool operator()(
      const GJKSolver_libccd<Scalar>& /*gjkSolver*/,
      const Plane<Scalar>& s,
      const Transform3<Scalar>& tf1,
      const Vector3<Scalar>& P1,
      const Vector3<Scalar>& P2,
      const Vector3<Scalar>& P3,
      const Transform3<Scalar>& tf2,
      Vector3<Scalar>* contact_points,
      Scalar* penetration_depth,
      Vector3<Scalar>* normal)
  {
    return details::planeTriangleIntersect(
          s, tf1, P1, P2, P3, tf2,
          contact_points, penetration_depth, normal);
  }
};

//==============================================================================
template<typename Scalar, typename S1, typename S2>
struct ShapeDistanceLibccdImpl
{
  bool operator()(
      const GJKSolver_libccd<Scalar>& gjkSolver,
      const S1& s1,
      const Transform3<Scalar>& tf1,
      const S2& s2,
      const Transform3<Scalar>& tf2,
      Scalar* dist,
      Vector3<Scalar>* p1,
      Vector3<Scalar>* p2)
  {
    void* o1 = details::GJKInitializer<Scalar, S1>::createGJKObject(s1, tf1);
    void* o2 = details::GJKInitializer<Scalar, S2>::createGJKObject(s2, tf2);

    bool res =  details::GJKDistance(
          o1,
          details::GJKInitializer<Scalar, S1>::getSupportFunction(),
          o2,
          details::GJKInitializer<Scalar, S2>::getSupportFunction(),
          gjkSolver.max_distance_iterations,
          gjkSolver.distance_tolerance,
          dist,
          p1,
          p2);

    if (p1)
      *p1 = tf1.inverse(Eigen::Isometry) * *p1;

    if (p2)
      *p2 = tf2.inverse(Eigen::Isometry) * *p2;

    details::GJKInitializer<Scalar, S1>::deleteGJKObject(o1);
    details::GJKInitializer<Scalar, S2>::deleteGJKObject(o2);

    return res;
  }
};

template<typename Scalar>
template<typename S1, typename S2>
bool GJKSolver_libccd<Scalar>::shapeDistance(
    const S1& s1,
    const Transform3<Scalar>& tf1,
    const S2& s2,
    const Transform3<Scalar>& tf2,
    Scalar* dist,
    Vector3<Scalar>* p1,
    Vector3<Scalar>* p2) const
{
  ShapeDistanceLibccdImpl<Scalar, S1, S2> shapeDistanceImpl;
  return shapeDistanceImpl(*this, s1, tf1, s2, tf2, dist, p1, p2);
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
template<typename Scalar>
struct ShapeDistanceLibccdImpl<Scalar, Sphere<Scalar>, Capsule<Scalar>>
{
  bool operator()(
      const GJKSolver_libccd<Scalar>& /*gjkSolver*/,
      const Sphere<Scalar>& s1,
      const Transform3<Scalar>& tf1,
      const Capsule<Scalar>& s2,
      const Transform3<Scalar>& tf2,
      Scalar* dist,
      Vector3<Scalar>* p1,
      Vector3<Scalar>* p2)
  {
    return details::sphereCapsuleDistance(s1, tf1, s2, tf2, dist, p1, p2);
  }
};

//==============================================================================
template<typename Scalar>
struct ShapeDistanceLibccdImpl<Scalar, Capsule<Scalar>, Sphere<Scalar>>
{
  bool operator()(
      const GJKSolver_libccd<Scalar>& /*gjkSolver*/,
      const Capsule<Scalar>& s1,
      const Transform3<Scalar>& tf1,
      const Sphere<Scalar>& s2,
      const Transform3<Scalar>& tf2,
      Scalar* dist,
      Vector3<Scalar>* p1,
      Vector3<Scalar>* p2)
  {
    return details::sphereCapsuleDistance(s2, tf2, s1, tf1, dist, p2, p1);
  }
};

//==============================================================================
template<typename Scalar>
struct ShapeDistanceLibccdImpl<Scalar, Sphere<Scalar>, Sphere<Scalar>>
{
  bool operator()(
      const GJKSolver_libccd<Scalar>& /*gjkSolver*/,
      const Sphere<Scalar>& s1,
      const Transform3<Scalar>& tf1,
      const Sphere<Scalar>& s2,
      const Transform3<Scalar>& tf2,
      Scalar* dist,
      Vector3<Scalar>* p1,
      Vector3<Scalar>* p2)
  {
    return details::sphereSphereDistance(s1, tf1, s2, tf2, dist, p1, p2);
  }
};

//==============================================================================
template<typename Scalar>
struct ShapeDistanceLibccdImpl<Scalar, Capsule<Scalar>, Capsule<Scalar>>
{
  bool operator()(
      const GJKSolver_libccd<Scalar>& /*gjkSolver*/,
      const Capsule<Scalar>& s1,
      const Transform3<Scalar>& tf1,
      const Capsule<Scalar>& s2,
      const Transform3<Scalar>& tf2,
      Scalar* dist,
      Vector3<Scalar>* p1,
      Vector3<Scalar>* p2)
  {
    return details::capsuleCapsuleDistance(s1, tf1, s2, tf2, dist, p1, p2);
  }
};

//==============================================================================
template<typename Scalar, typename S>
struct ShapeTriangleDistanceLibccdImpl
{
  bool operator()(
      const GJKSolver_libccd<Scalar>& gjkSolver,
      const S& s,
      const Transform3<Scalar>& tf,
      const Vector3<Scalar>& P1,
      const Vector3<Scalar>& P2,
      const Vector3<Scalar>& P3,
      Scalar* dist,
      Vector3<Scalar>* p1,
      Vector3<Scalar>* p2)
  {
    void* o1 = details::GJKInitializer<Scalar, S>::createGJKObject(s, tf);
    void* o2 = details::triCreateGJKObject(P1, P2, P3);

    bool res = details::GJKDistance(
          o1,
          details::GJKInitializer<Scalar, S>::getSupportFunction(),
          o2,
          details::triGetSupportFunction(),
          gjkSolver.max_distance_iterations,
          gjkSolver.distance_tolerance,
          dist,
          p1,
          p2);
    if(p1) *p1 = tf.inverse(Eigen::Isometry) * *p1;

    details::GJKInitializer<Scalar, S>::deleteGJKObject(o1);
    details::triDeleteGJKObject(o2);

    return res;
  }
};

//==============================================================================
template<typename Scalar>
template<typename S>
bool GJKSolver_libccd<Scalar>::shapeTriangleDistance(
    const S& s,
    const Transform3<Scalar>& tf,
    const Vector3<Scalar>& P1,
    const Vector3<Scalar>& P2,
    const Vector3<Scalar>& P3,
    Scalar* dist,
    Vector3<Scalar>* p1,
    Vector3<Scalar>* p2) const
{
  ShapeTriangleDistanceLibccdImpl<Scalar, S> shapeTriangleDistanceImpl;
  return shapeTriangleDistanceImpl(
        *this, s, tf, P1, P2, P3, dist, p1, p2);
}

//==============================================================================
template<typename Scalar>
struct ShapeTriangleDistanceLibccdImpl<Scalar, Sphere<Scalar>>
{
  bool operator()(
      const GJKSolver_libccd<Scalar>& /*gjkSolver*/,
      const Sphere<Scalar>& s,
      const Transform3<Scalar>& tf,
      const Vector3<Scalar>& P1,
      const Vector3<Scalar>& P2,
      const Vector3<Scalar>& P3,
      Scalar* dist,
      Vector3<Scalar>* p1,
      Vector3<Scalar>* p2)
  {
    return details::sphereTriangleDistance(s, tf, P1, P2, P3, dist, p1, p2);
  }
};

//==============================================================================
template<typename Scalar, typename S>
struct ShapeTransformedTriangleDistanceLibccdImpl
{
  bool operator()(
      const GJKSolver_libccd<Scalar>& gjkSolver,
      const S& s,
      const Transform3<Scalar>& tf1,
      const Vector3<Scalar>& P1,
      const Vector3<Scalar>& P2,
      const Vector3<Scalar>& P3,
      const Transform3<Scalar>& tf2,
      Scalar* dist,
      Vector3<Scalar>* p1,
      Vector3<Scalar>* p2)
  {
    void* o1 = details::GJKInitializer<Scalar, S>::createGJKObject(s, tf1);
    void* o2 = details::triCreateGJKObject(P1, P2, P3, tf2);

    bool res = details::GJKDistance(
          o1,
          details::GJKInitializer<Scalar, S>::getSupportFunction(),
          o2,
          details::triGetSupportFunction(),
          gjkSolver.max_distance_iterations,
          gjkSolver.distance_tolerance,
          dist,
          p1,
          p2);
    if(p1) *p1 = tf1.inverse(Eigen::Isometry) * *p1;
    if(p2) *p2 = tf2.inverse(Eigen::Isometry) * *p2;

    details::GJKInitializer<Scalar, S>::deleteGJKObject(o1);
    details::triDeleteGJKObject(o2);

    return res;
  }
};

//==============================================================================
template<typename Scalar>
template<typename S>
bool GJKSolver_libccd<Scalar>::shapeTriangleDistance(
    const S& s,
    const Transform3<Scalar>& tf1,
    const Vector3<Scalar>& P1,
    const Vector3<Scalar>& P2,
    const Vector3<Scalar>& P3,
    const Transform3<Scalar>& tf2,
    Scalar* dist,
    Vector3<Scalar>* p1,
    Vector3<Scalar>* p2) const
{
  ShapeTransformedTriangleDistanceLibccdImpl<Scalar, S> shapeTriangleDistanceImpl;
  return shapeTriangleDistanceImpl(
        *this, s, tf1, P1, P2, P3, tf2, dist, p1, p2);
}

//==============================================================================
template<typename Scalar>
struct ShapeTransformedTriangleDistanceLibccdImpl<Scalar, Sphere<Scalar>>
{
  bool operator()(
      const GJKSolver_libccd<Scalar>& /*gjkSolver*/,
      const Sphere<Scalar>& s,
      const Transform3<Scalar>& tf1,
      const Vector3<Scalar>& P1,
      const Vector3<Scalar>& P2,
      const Vector3<Scalar>& P3,
      const Transform3<Scalar>& tf2,
      Scalar* dist,
      Vector3<Scalar>* p1,
      Vector3<Scalar>* p2)
  {
    return details::sphereTriangleDistance(
          s, tf1, P1, P2, P3, tf2, dist, p1, p2);
  }
};

//==============================================================================
template<typename Scalar>
GJKSolver_libccd<Scalar>::GJKSolver_libccd()
{
  max_collision_iterations = 500;
  max_distance_iterations = 1000;
  collision_tolerance = 1e-6;
  distance_tolerance = 1e-6;
}

//==============================================================================
template<typename Scalar>
void GJKSolver_libccd<Scalar>::enableCachedGuess(bool if_enable) const
{
  // TODO: need change libccd to exploit spatial coherence
}

//==============================================================================
template<typename Scalar>
void GJKSolver_libccd<Scalar>::setCachedGuess(
    const Vector3<GJKSolver_libccd<Scalar>::Scalar>& guess) const
{
  // TODO: need change libccd to exploit spatial coherence
}

//==============================================================================
template<typename Scalar>
Vector3<Scalar> GJKSolver_libccd<Scalar>::getCachedGuess() const
{
  return Vector3<Scalar>(-1, 0, 0);
}

} // namespace fcl

#endif

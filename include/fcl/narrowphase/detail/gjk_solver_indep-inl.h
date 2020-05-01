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

#ifndef FCL_NARROWPHASE_GJKSOLVERINDEP_INL_H
#define FCL_NARROWPHASE_GJKSOLVERINDEP_INL_H

#include "fcl/narrowphase/detail/gjk_solver_indep.h"

#include <algorithm>

#include "fcl/common/unused.h"

#include "fcl/geometry/shape/triangle_p.h"

#include "fcl/narrowphase/detail/convexity_based_algorithm/gjk.h"
#include "fcl/narrowphase/detail/convexity_based_algorithm/epa.h"
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
struct GJKSolver_indep<double>;

//==============================================================================
template <typename S>
template<typename Shape1, typename Shape2>
bool GJKSolver_indep<S>::shapeIntersect(const Shape1& s1, const Transform3<S>& tf1,
                                     const Shape2& s2, const Transform3<S>& tf2,
                                     Vector3<S>* contact_points, S* penetration_depth, Vector3<S>* normal) const
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
struct ShapeIntersectIndepImpl
{
  static bool run(
      const GJKSolver_indep<S>& gjkSolver,
      const Shape1& s1,
      const Transform3<S>& tf1,
      const Shape2& s2,
      const Transform3<S>& tf2,
      std::vector<ContactPoint<S>>* contacts)
  {
    Vector3<S> guess(1, 0, 0);
    if(gjkSolver.enable_cached_guess) guess = gjkSolver.cached_guess;

    detail::MinkowskiDiff<S> shape;
    shape.shapes[0] = &s1;
    shape.shapes[1] = &s2;
    shape.toshape1.noalias() = tf2.linear().transpose() * tf1.linear();
    shape.toshape0 = tf1.inverse(Eigen::Isometry) * tf2;

    detail::GJK<S> gjk(gjkSolver.gjk_max_iterations, gjkSolver.gjk_tolerance);
    typename detail::GJK<S>::Status gjk_status = gjk.evaluate(shape, -guess);
    if(gjkSolver.enable_cached_guess) gjkSolver.cached_guess = gjk.getGuessFromSimplex();

    switch(gjk_status)
    {
    case detail::GJK<S>::Inside:
      {
        detail::EPA<S> epa(gjkSolver.epa_max_face_num, gjkSolver.epa_max_vertex_num, gjkSolver.epa_max_iterations, gjkSolver.epa_tolerance);
        typename detail::EPA<S>::Status epa_status = epa.evaluate(gjk, -guess);
        if(epa_status != detail::EPA<S>::Failed)
        {
          Vector3<S> w0 = Vector3<S>::Zero();
          for(size_t i = 0; i < epa.result.rank; ++i)
          {
            w0.noalias() += shape.support(epa.result.c[i]->d, 0) * epa.result.p[i];
          }
          if(contacts)
          {
            Vector3<S> normal = epa.normal;
            Vector3<S> point = tf1 * (w0 - epa.normal*(epa.depth *0.5));
            S depth = -epa.depth;
            contacts->emplace_back(normal, point, depth);
          }
          return true;
        }
        else return false;
      }
      break;
    default:
      ;
    }

    return false;
  }
};

//==============================================================================
template<typename S>
template<typename Shape1, typename Shape2>
bool GJKSolver_indep<S>::shapeIntersect(
    const Shape1& s1,
    const Transform3<S>& tf1,
    const Shape2& s2,
    const Transform3<S>& tf2,
    std::vector<ContactPoint<S>>* contacts) const
{
  return ShapeIntersectIndepImpl<S, Shape1, Shape2>::run(
        *this, s1, tf1, s2, tf2, contacts);
}

// Shape intersect algorithms not using built-in GJK algorithm
//
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+----------+
// |            | box | sphere | ellipsoid | capsule | cone | cylinder | plane | half-space | triangle |  convex  |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+----------+
// | box        |  O  |   O    |           |         |      |          |   O   |      O     |          |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+----------+
// | sphere     |/////|   O    |           |    O    |      |    O     |   O   |      O     |     O    |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+----------+
// | ellipsoid  |/////|////////|           |         |      |          |   O   |      O     |          |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+----------+
// | capsule    |/////|////////|///////////|         |      |          |   O   |      O     |          |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+----------+
// | cone       |/////|////////|///////////|/////////|      |          |   O   |      O     |          |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+----------+
// | cylinder   |/////|////////|///////////|/////////|//////|          |   O   |      O     |          |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+----------+
// | plane      |/////|////////|///////////|/////////|//////|//////////|   O   |      O     |     O    |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+----------+
// | half-space |/////|////////|///////////|/////////|//////|//////////|///////|      O     |     O    |    O     |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+----------+
// | triangle   |/////|////////|///////////|/////////|//////|//////////|///////|////////////|          |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+----------+
// | convex     |/////|////////|///////////|/////////|//////|//////////|///////|////////////|//////////|          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+----------+

#define FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT_REG(SHAPE1, SHAPE2, ALG)\
  template <typename S>\
  struct ShapeIntersectIndepImpl<S, SHAPE1<S>, SHAPE2<S>>\
  {\
    static bool run(\
        const GJKSolver_indep<S>& /*gjkSolver*/,\
        const SHAPE1<S>& s1,\
        const Transform3<S>& tf1,\
        const SHAPE2<S>& s2,\
        const Transform3<S>& tf2,\
        std::vector<ContactPoint<S>>* contacts)\
    {\
      return ALG(s1, tf1, s2, tf2, contacts);\
    }\
  };

#define FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT_INV(SHAPE1, SHAPE2, ALG)\
  template <typename S>\
  struct ShapeIntersectIndepImpl<S, SHAPE2<S>, SHAPE1<S>>\
  {\
    static bool run(\
        const GJKSolver_indep<S>& /*gjkSolver*/,\
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

#define FCL_GJK_INDEP_SHAPE_INTERSECT(SHAPE, ALG)\
  FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT_REG(SHAPE, SHAPE, ALG)

#define FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(SHAPE1, SHAPE2, ALG)\
  FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT_REG(SHAPE1, SHAPE2, ALG)\
  FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT_INV(SHAPE1, SHAPE2, ALG)

FCL_GJK_INDEP_SHAPE_INTERSECT(Sphere, detail::sphereSphereIntersect)
FCL_GJK_INDEP_SHAPE_INTERSECT(Box, detail::boxBoxIntersect)

FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Sphere, Capsule, detail::sphereCapsuleIntersect)

FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Sphere, Box, detail::sphereBoxIntersect)

FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Sphere, Cylinder, detail::sphereCylinderIntersect)

FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Sphere, Halfspace, detail::sphereHalfspaceIntersect)
FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Ellipsoid, Halfspace, detail::ellipsoidHalfspaceIntersect)
FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Box, Halfspace, detail::boxHalfspaceIntersect)
FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Capsule, Halfspace, detail::capsuleHalfspaceIntersect)
FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Cylinder, Halfspace, detail::cylinderHalfspaceIntersect)
FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Cone, Halfspace, detail::coneHalfspaceIntersect)
FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Convex, Halfspace, detail::convexHalfspaceIntersect)

FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Sphere, Plane, detail::spherePlaneIntersect)
FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Ellipsoid, Plane, detail::ellipsoidPlaneIntersect)
FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Box, Plane, detail::boxPlaneIntersect)
FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Capsule, Plane, detail::capsulePlaneIntersect)
FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Cylinder, Plane, detail::cylinderPlaneIntersect)
FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Cone, Plane, detail::conePlaneIntersect)

template <typename S>
struct ShapeIntersectIndepImpl<S, Halfspace<S>, Halfspace<S>>
{
  static bool run(
      const GJKSolver_indep<S>& /*gjkSolver*/,
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
struct ShapeIntersectIndepImpl<S, Plane<S>, Plane<S>>
{
  static bool run(
      const GJKSolver_indep<S>& /*gjkSolver*/,
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
struct ShapeIntersectIndepImpl<S, Plane<S>, Halfspace<S>>
{
  static bool run(
      const GJKSolver_indep<S>& /*gjkSolver*/,
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
struct ShapeIntersectIndepImpl<S, Halfspace<S>, Plane<S>>
{
  static bool run(
      const GJKSolver_indep<S>& /*gjkSolver*/,
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
struct ShapeTriangleIntersectIndepImpl
{
  static bool run(
      const GJKSolver_indep<S>& gjkSolver,
      const Shape& s,
      const Transform3<S>& tf,
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      Vector3<S>* contact_points,
      S* penetration_depth,
      Vector3<S>* normal)
  {
    TriangleP<S> tri(P1, P2, P3);

    Vector3<S> guess(1, 0, 0);
    if(gjkSolver.enable_cached_guess) guess = gjkSolver.cached_guess;

    detail::MinkowskiDiff<S> shape;
    shape.shapes[0] = &s;
    shape.shapes[1] = &tri;
    shape.toshape1 = tf.linear();
    shape.toshape0 = tf.inverse(Eigen::Isometry);

    detail::GJK<S> gjk(gjkSolver.gjk_max_iterations, gjkSolver.gjk_tolerance);
    typename detail::GJK<S>::Status gjk_status = gjk.evaluate(shape, -guess);
    if(gjkSolver.enable_cached_guess) gjkSolver.cached_guess = gjk.getGuessFromSimplex();

    switch(gjk_status)
    {
    case detail::GJK<S>::Inside:
      {
        detail::EPA<S> epa(gjkSolver.epa_max_face_num, gjkSolver.epa_max_vertex_num, gjkSolver.epa_max_iterations, gjkSolver.epa_tolerance);
        typename detail::EPA<S>::Status epa_status = epa.evaluate(gjk, -guess);
        if(epa_status != detail::EPA<S>::Failed)
        {
          Vector3<S> w0 = Vector3<S>::Zero();
          for(size_t i = 0; i < epa.result.rank; ++i)
          {
            w0.noalias() += shape.support(epa.result.c[i]->d, 0) * epa.result.p[i];
          }
          if(penetration_depth) *penetration_depth = -epa.depth;
          if(normal) *normal = -epa.normal;
          if(contact_points) (*contact_points).noalias() = tf * (w0 - epa.normal*(epa.depth *0.5));
          return true;
        }
        else return false;
      }
      break;
    default:
      ;
    }

    return false;
  }
};

template<typename S>
template<typename Shape>
bool GJKSolver_indep<S>::shapeTriangleIntersect(
    const Shape& s,
    const Transform3<S>& tf,
    const Vector3<S>& P1,
    const Vector3<S>& P2,
    const Vector3<S>& P3,
    Vector3<S>* contact_points,
    S* penetration_depth,
    Vector3<S>* normal) const
{
  return ShapeTriangleIntersectIndepImpl<S, Shape>::run(
        *this, s, tf, P1, P2, P3, contact_points, penetration_depth, normal);
}

//==============================================================================
template<typename S>
struct ShapeTriangleIntersectIndepImpl<S, Sphere<S>>
{
  static bool run(
      const GJKSolver_indep<S>& /*gjkSolver*/,
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
struct ShapeTransformedTriangleIntersectIndepImpl
{
  static bool run(
      const GJKSolver_indep<S>& gjkSolver,
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
    TriangleP<S> tri(P1, P2, P3);

    Vector3<S> guess(1, 0, 0);
    if(gjkSolver.enable_cached_guess) guess = gjkSolver.cached_guess;

    detail::MinkowskiDiff<S> shape;
    shape.shapes[0] = &s;
    shape.shapes[1] = &tri;
    shape.toshape1.noalias() = tf2.linear().transpose() * tf1.linear();
    shape.toshape0 = tf1.inverse(Eigen::Isometry) * tf2;

    detail::GJK<S> gjk(gjkSolver.gjk_max_iterations, gjkSolver.gjk_tolerance);
    typename detail::GJK<S>::Status gjk_status = gjk.evaluate(shape, -guess);
    if(gjkSolver.enable_cached_guess) gjkSolver.cached_guess = gjk.getGuessFromSimplex();

    switch(gjk_status)
    {
    case detail::GJK<S>::Inside:
      {
        detail::EPA<S> epa(gjkSolver.epa_max_face_num, gjkSolver.epa_max_vertex_num, gjkSolver.epa_max_iterations, gjkSolver.epa_tolerance);
        typename detail::EPA<S>::Status epa_status = epa.evaluate(gjk, -guess);
        if(epa_status != detail::EPA<S>::Failed)
        {
          Vector3<S> w0 = Vector3<S>::Zero();
          for(size_t i = 0; i < epa.result.rank; ++i)
          {
            w0.noalias() += shape.support(epa.result.c[i]->d, 0) * epa.result.p[i];
          }
          if(penetration_depth) *penetration_depth = -epa.depth;
          if(normal) *normal = -epa.normal;
          if(contact_points) (*contact_points).noalias() = tf1 * (w0 - epa.normal*(epa.depth *0.5));
          return true;
        }
        else return false;
      }
      break;
    default:
      ;
    }

    return false;
  }
};

template<typename S>
template<typename Shape>
bool GJKSolver_indep<S>::shapeTriangleIntersect(
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
  return ShapeTransformedTriangleIntersectIndepImpl<S, Shape>::run(
        *this, s, tf1, P1, P2, P3, tf2,
        contact_points, penetration_depth, normal);
}

//==============================================================================
template<typename S>
struct ShapeTransformedTriangleIntersectIndepImpl<S, Sphere<S>>
{
  static bool run(
      const GJKSolver_indep<S>& /*gjkSolver*/,
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
struct ShapeTransformedTriangleIntersectIndepImpl<S, Halfspace<S>>
{
  static bool run(
      const GJKSolver_indep<S>& /*gjkSolver*/,
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
struct ShapeTransformedTriangleIntersectIndepImpl<S, Plane<S>>
{
  static bool run(
      const GJKSolver_indep<S>& /*gjkSolver*/,
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
template<typename S, typename Shape1, typename Shape2>
struct ShapeDistanceIndepImpl
{
  static bool run(
      const GJKSolver_indep<S>& gjkSolver,
      const Shape1& s1,
      const Transform3<S>& tf1,
      const Shape2& s2,
      const Transform3<S>& tf2,
      S* distance,
      Vector3<S>* p1,
      Vector3<S>* p2)
  {
    Vector3<S> guess(1, 0, 0);
    if(gjkSolver.enable_cached_guess) guess = gjkSolver.cached_guess;

    detail::MinkowskiDiff<S> shape;
    shape.shapes[0] = &s1;
    shape.shapes[1] = &s2;
    shape.toshape1.noalias() = tf2.linear().transpose() * tf1.linear();
    shape.toshape0 = tf1.inverse(Eigen::Isometry) * tf2;

    detail::GJK<S> gjk(gjkSolver.gjk_max_iterations, gjkSolver.gjk_tolerance);
    typename detail::GJK<S>::Status gjk_status = gjk.evaluate(shape, -guess);
    if(gjkSolver.enable_cached_guess) gjkSolver.cached_guess = gjk.getGuessFromSimplex();

    if(gjk_status == detail::GJK<S>::Valid)
    {
      Vector3<S> w0 = Vector3<S>::Zero();
      Vector3<S> w1 = Vector3<S>::Zero();
      for(size_t i = 0; i < gjk.getSimplex()->rank; ++i)
      {
        S p = gjk.getSimplex()->p[i];
        w0.noalias() += shape.support(gjk.getSimplex()->c[i]->d, 0) * p;
        w1.noalias() += shape.support(-gjk.getSimplex()->c[i]->d, 1) * p;
      }

      if(distance) *distance = (w0 - w1).norm();

      // Answer is solved in Shape1's local frame; answers are given in the
      // world frame.
      if(p1) p1->noalias() = tf1 * w0;
      if(p2) p2->noalias() = tf1 * w1;

      return true;
    }
    else
    {
      if(distance) *distance = -1;
      return false;
    }
  }
};

template<typename S>
template<typename Shape1, typename Shape2>
bool GJKSolver_indep<S>::shapeDistance(
    const Shape1& s1,
    const Transform3<S>& tf1,
    const Shape2& s2,
    const Transform3<S>& tf2,
    S* dist,
    Vector3<S>* p1,
    Vector3<S>* p2) const
{
  return ShapeDistanceIndepImpl<S, Shape1, Shape2>::run(
        *this, s1, tf1, s2, tf2, dist, p1, p2);
}


//==============================================================================
template<typename S>
template<typename Shape1, typename Shape2>
bool GJKSolver_indep<S>::shapeSignedDistance(
    const Shape1& s1,
    const Transform3<S>& tf1,
    const Shape2& s2,
    const Transform3<S>& tf2,
    S* dist,
    Vector3<S>* p1,
    Vector3<S>* p2) const
{
  // TODO: should implement the signed distance version
  bool result = false;
  try {
    result = ShapeDistanceIndepImpl<S, Shape1, Shape2>::run(
        *this, s1, tf1, s2, tf2, dist, p1, p2);
  } catch (const FailedAtThisConfiguration& e) {
    ThrowDetailedConfiguration(s1, tf1, s2, tf2, *this, e);
  }
  return result;
}

// Shape distance algorithms not using built-in GJK algorithm
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
struct ShapeDistanceIndepImpl<S, Sphere<S>, Box<S>>
{
  static bool run(
      const GJKSolver_indep<S>& /*gjkSolver*/,
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
struct ShapeDistanceIndepImpl<S, Box<S>, Sphere<S>>
{
  static bool run(
      const GJKSolver_indep<S>& /*gjkSolver*/,
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
struct ShapeDistanceIndepImpl<S, Sphere<S>, Capsule<S>>
{
  static bool run(
      const GJKSolver_indep<S>& /*gjkSolver*/,
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
struct ShapeDistanceIndepImpl<S, Capsule<S>, Sphere<S>>
{
  static bool run(
      const GJKSolver_indep<S>& /*gjkSolver*/,
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
struct ShapeDistanceIndepImpl<S, Sphere<S>, Cylinder<S>>
{
  static bool run(
      const GJKSolver_indep<S>& /*gjkSolver*/,
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
struct ShapeDistanceIndepImpl<S, Cylinder<S>, Sphere<S>>
{
  static bool run(
      const GJKSolver_indep<S>& /*gjkSolver*/,
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
struct ShapeDistanceIndepImpl<S, Sphere<S>, Sphere<S>>
{
  static bool run(
      const GJKSolver_indep<S>& /*gjkSolver*/,
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
struct ShapeDistanceIndepImpl<S, Capsule<S>, Capsule<S>>
{
  static bool run(
      const GJKSolver_indep<S>& /*gjkSolver*/,
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
struct ShapeTriangleDistanceIndepImpl
{
  static bool run(
      const GJKSolver_indep<S>& gjkSolver,
      const Shape& s,
      const Transform3<S>& tf,
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      S* distance,
      Vector3<S>* p1,
      Vector3<S>* p2)
  {
    TriangleP<S> tri(P1, P2, P3);
    Vector3<S> guess(1, 0, 0);
    if(gjkSolver.enable_cached_guess) guess = gjkSolver.cached_guess;

    detail::MinkowskiDiff<S> shape;
    shape.shapes[0] = &s;
    shape.shapes[1] = &tri;
    shape.toshape1 = tf.linear();
    shape.toshape0 = tf.inverse(Eigen::Isometry);

    detail::GJK<S> gjk(gjkSolver.gjk_max_iterations, gjkSolver.gjk_tolerance);
    typename detail::GJK<S>::Status gjk_status = gjk.evaluate(shape, -guess);
    if(gjkSolver.enable_cached_guess) gjkSolver.cached_guess = gjk.getGuessFromSimplex();

    if(gjk_status == detail::GJK<S>::Valid)
    {
      Vector3<S> w0 = Vector3<S>::Zero();
      Vector3<S> w1 = Vector3<S>::Zero();
      for(size_t i = 0; i < gjk.getSimplex()->rank; ++i)
      {
        S p = gjk.getSimplex()->p[i];
        w0.noalias() += shape.support(gjk.getSimplex()->c[i]->d, 0) * p;
        w1.noalias() += shape.support(-gjk.getSimplex()->c[i]->d, 1) * p;
      }

      if(distance) *distance = (w0 - w1).norm();
      // The answers are produced in world coordinates. Keep them there.
      if(p1) *p1 = w0;
      if(p2) *p2 = w1;
      return true;
    }
    else
    {
      if(distance) *distance = -1;
      return false;
    }
  }
};

//==============================================================================
template<typename S>
template<typename Shape>
bool GJKSolver_indep<S>::shapeTriangleDistance(
    const Shape& s,
    const Transform3<S>& tf,
    const Vector3<S>& P1,
    const Vector3<S>& P2,
    const Vector3<S>& P3,
    S* dist,
    Vector3<S>* p1,
    Vector3<S>* p2) const
{
  return ShapeTriangleDistanceIndepImpl<S, Shape>::run(
        *this, s, tf, P1, P2, P3, dist, p1, p2);
}

//==============================================================================
template<typename S>
struct ShapeTriangleDistanceIndepImpl<S, Sphere<S>>
{
  static bool run(
      const GJKSolver_indep<S>& /*gjkSolver*/,
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
struct ShapeTransformedTriangleDistanceIndepImpl
{
  static bool run(
      const GJKSolver_indep<S>& gjkSolver,
      const Shape& s,
      const Transform3<S>& tf1,
      const Vector3<S>& P1,
      const Vector3<S>& P2,
      const Vector3<S>& P3,
      const Transform3<S>& tf2,
      S* distance,
      Vector3<S>* p1,
      Vector3<S>* p2)
  {
    TriangleP<S> tri(P1, P2, P3);
    Vector3<S> guess(1, 0, 0);
    if(gjkSolver.enable_cached_guess) guess = gjkSolver.cached_guess;

    detail::MinkowskiDiff<S> shape;
    shape.shapes[0] = &s;
    shape.shapes[1] = &tri;
    shape.toshape1.noalias() = tf2.linear().transpose() * tf1.linear();
    shape.toshape0 = tf1.inverse(Eigen::Isometry) * tf2;

    detail::GJK<S> gjk(gjkSolver.gjk_max_iterations, gjkSolver.gjk_tolerance);
    typename detail::GJK<S>::Status gjk_status = gjk.evaluate(shape, -guess);
    if(gjkSolver.enable_cached_guess) gjkSolver.cached_guess = gjk.getGuessFromSimplex();

    if(gjk_status == detail::GJK<S>::Valid)
    {
      Vector3<S> w0 = Vector3<S>::Zero();
      Vector3<S> w1 = Vector3<S>::Zero();
      for(size_t i = 0; i < gjk.getSimplex()->rank; ++i)
      {
        S p = gjk.getSimplex()->p[i];
        w0.noalias() += shape.support(gjk.getSimplex()->c[i]->d, 0) * p;
        w1.noalias() += shape.support(-gjk.getSimplex()->c[i]->d, 1) * p;
      }

      if(distance) *distance = (w0 - w1).norm();
      if(p1) p1->noalias() = tf1 * w0;
      if(p2) p2->noalias() = tf1 * w1;
      return true;
    }
    else
    {
      if(distance) *distance = -1;
      return false;
    }
  }
};

//==============================================================================
template<typename S>
template<typename Shape>
bool GJKSolver_indep<S>::shapeTriangleDistance(
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
  return ShapeTransformedTriangleDistanceIndepImpl<S, Shape>::run(
        *this, s, tf1, P1, P2, P3, tf2, dist, p1, p2);
}

//==============================================================================
template<typename S>
struct ShapeTransformedTriangleDistanceIndepImpl<S, Sphere<S>>
{
  static bool run(
      const GJKSolver_indep<S>& /*gjkSolver*/,
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
template <typename S>
GJKSolver_indep<S>::GJKSolver_indep()
{
  gjk_max_iterations = 128;
  gjk_tolerance = constants<S>::gjk_default_tolerance();
  epa_max_face_num = 128;
  epa_max_vertex_num = 64;
  epa_max_iterations = 255;
  epa_tolerance = constants<S>::gjk_default_tolerance();
  enable_cached_guess = false;
  cached_guess = Vector3<S>(1, 0, 0);
}

//==============================================================================
template <typename S>
void GJKSolver_indep<S>::enableCachedGuess(bool if_enable) const
{
  enable_cached_guess = if_enable;
}

//==============================================================================
template <typename S>
void GJKSolver_indep<S>::setCachedGuess(const Vector3<S>& guess) const
{
  cached_guess = guess;
}

//==============================================================================
template <typename S>
Vector3<S> GJKSolver_indep<S>::getCachedGuess() const
{
  return cached_guess;
}

} // namespace detail
} // namespace fcl

#endif

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

#ifndef FCL_NARROWPHASE_GJKSOLVERINDEP_H
#define FCL_NARROWPHASE_GJKSOLVERINDEP_H

#include <algorithm>

#include "fcl/collision_data.h"
#include "fcl/narrowphase/detail/gjk.h"
#include "fcl/narrowphase/detail/primitive_shape_algorithms.h"

namespace fcl
{

/// @brief collision and distance solver based on GJK algorithm implemented in fcl (rewritten the code from the GJK in bullet)
template <typename ScalarT>
struct GJKSolver_indep
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
      Scalar* distance = NULL,
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
      Scalar* distance = NULL,
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
      Scalar* distance = NULL,
      Vector3<Scalar>* p1 = NULL,
      Vector3<Scalar>* p2 = NULL) const;
  
  /// @brief default setting for GJK algorithm
  GJKSolver_indep();

  void enableCachedGuess(bool if_enable) const;

  void setCachedGuess(const Vector3<Scalar>& guess) const;

  Vector3<Scalar> getCachedGuess() const;

  /// @brief maximum number of simplex face used in EPA algorithm
  unsigned int epa_max_face_num;

  /// @brief maximum number of simplex vertex used in EPA algorithm
  unsigned int epa_max_vertex_num;

  /// @brief maximum number of iterations used for EPA iterations
  unsigned int epa_max_iterations;

  /// @brief the threshold used in EPA to stop iteration
  Scalar epa_tolerance;

  /// @brief the threshold used in GJK to stop iteration
  Scalar gjk_tolerance;

  /// @brief maximum number of iterations used for GJK iterations
  Scalar gjk_max_iterations;

  /// @brief Whether smart guess can be provided
  mutable bool enable_cached_guess;

  /// @brief smart guess
  mutable Vector3<Scalar> cached_guess;
};

using GJKSolver_indepf = GJKSolver_indep<float>;
using GJKSolver_indepd = GJKSolver_indep<double>;

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
template<typename S1, typename S2>
bool GJKSolver_indep<Scalar>::shapeIntersect(const S1& s1, const Transform3<Scalar>& tf1,
                                     const S2& s2, const Transform3<Scalar>& tf2,
                                     Vector3<Scalar>* contact_points, Scalar* penetration_depth, Vector3<Scalar>* normal) const
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
struct ShapeIntersectIndepImpl
{
  bool operator()(
      const GJKSolver_indep<Scalar>& gjkSolver,
      const S1& s1,
      const Transform3<Scalar>& tf1,
      const S2& s2,
      const Transform3<Scalar>& tf2,
      std::vector<ContactPoint<Scalar>>* contacts)
  {
    Vector3<Scalar> guess(1, 0, 0);
    if(gjkSolver.enable_cached_guess) guess = gjkSolver.cached_guess;

    details::MinkowskiDiff<Scalar> shape;
    shape.shapes[0] = &s1;
    shape.shapes[1] = &s2;
    shape.toshape1 = tf2.linear().transpose() * tf1.linear();
    shape.toshape0 = tf1.inverse(Eigen::Isometry) * tf2;

    details::GJK<Scalar> gjk(gjkSolver.gjk_max_iterations, gjkSolver.gjk_tolerance);
    typename details::GJK<Scalar>::Status gjk_status = gjk.evaluate(shape, -guess);
    if(gjkSolver.enable_cached_guess) gjkSolver.cached_guess = gjk.getGuessFromSimplex();

    switch(gjk_status)
    {
    case details::GJK<Scalar>::Inside:
      {
        details::EPA<Scalar> epa(gjkSolver.epa_max_face_num, gjkSolver.epa_max_vertex_num, gjkSolver.epa_max_iterations, gjkSolver.epa_tolerance);
        typename details::EPA<Scalar>::Status epa_status = epa.evaluate(gjk, -guess);
        if(epa_status != details::EPA<Scalar>::Failed)
        {
          Vector3<Scalar> w0 = Vector3<Scalar>::Zero();
          for(size_t i = 0; i < epa.result.rank; ++i)
          {
            w0 += shape.support(epa.result.c[i]->d, 0) * epa.result.p[i];
          }
          if(contacts)
          {
            Vector3<Scalar> normal = epa.normal;
            Vector3<Scalar> point = tf1 * (w0 - epa.normal*(epa.depth *0.5));
            Scalar depth = -epa.depth;
            contacts->push_back(ContactPoint<Scalar>(normal, point, depth));
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
template<typename Scalar>
template<typename S1, typename S2>
bool GJKSolver_indep<Scalar>::shapeIntersect(
    const S1& s1,
    const Transform3<Scalar>& tf1,
    const S2& s2,
    const Transform3<Scalar>& tf2,
    std::vector<ContactPoint<Scalar>>* contacts) const
{
  ShapeIntersectIndepImpl<Scalar, S1, S2> shapeIntersectImpl;
  return shapeIntersectImpl(*this, s1, tf1, s2, tf2, contacts);
}

// Shape intersect algorithms not using built-in GJK algorithm
//
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// |            | box | sphere | ellipsoid | capsule | cone | cylinder | plane | half-space | triangle |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | box        |  O  |        |           |         |      |          |   O   |      O     |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | sphere     |/////|   O    |           |    O    |      |          |   O   |      O     |     O    |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | ellipsoid  |/////|////////|           |         |      |          |   O   |      O     |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | capsule    |/////|////////|///////////|         |      |          |   O   |      O     |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | cone       |/////|////////|///////////|/////////|      |          |   O   |      O     |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | cylinder   |/////|////////|///////////|/////////|//////|          |   O   |      O     |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | plane      |/////|////////|///////////|/////////|//////|//////////|   O   |      O     |     O    |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | half-space |/////|////////|///////////|/////////|//////|//////////|///////|      O     |     O    |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | triangle   |/////|////////|///////////|/////////|//////|//////////|///////|////////////|          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+

#define FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT_REG(SHAPE1, SHAPE2, ALG)\
  template <typename Scalar>\
  struct ShapeIntersectIndepImpl<Scalar, SHAPE1<Scalar>, SHAPE2<Scalar>>\
  {\
    bool operator()(\
        const GJKSolver_indep<Scalar>& /*gjkSolver*/,\
        const SHAPE1<Scalar>& s1,\
        const Transform3<Scalar>& tf1,\
        const SHAPE2<Scalar>& s2,\
        const Transform3<Scalar>& tf2,\
        std::vector<ContactPoint<Scalar>>* contacts)\
    {\
      return ALG(s1, tf1, s2, tf2, contacts);\
    }\
  };

#define FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT_INV(SHAPE1, SHAPE2, ALG)\
  template <typename Scalar>\
  struct ShapeIntersectIndepImpl<Scalar, SHAPE2<Scalar>, SHAPE1<Scalar>>\
  {\
    bool operator()(\
        const GJKSolver_indep<Scalar>& /*gjkSolver*/,\
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

#define FCL_GJK_INDEP_SHAPE_INTERSECT(SHAPE, ALG)\
  FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT_REG(SHAPE, SHAPE, ALG)

#define FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(SHAPE1, SHAPE2, ALG)\
  FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT_REG(SHAPE1, SHAPE2, ALG)\
  FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT_INV(SHAPE1, SHAPE2, ALG)

FCL_GJK_INDEP_SHAPE_INTERSECT(Sphere, details::sphereSphereIntersect)
FCL_GJK_INDEP_SHAPE_INTERSECT(Box, details::boxBoxIntersect)

FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Sphere, Capsule, details::sphereCapsuleIntersect)

FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Sphere, Halfspace, details::sphereHalfspaceIntersect)
FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Ellipsoid, Halfspace, details::ellipsoidHalfspaceIntersect)
FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Box, Halfspace, details::boxHalfspaceIntersect)
FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Capsule, Halfspace, details::capsuleHalfspaceIntersect)
FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Cylinder, Halfspace, details::cylinderHalfspaceIntersect)
FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Cone, Halfspace, details::coneHalfspaceIntersect)

FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Sphere, Plane, details::spherePlaneIntersect)
FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Ellipsoid, Plane, details::ellipsoidPlaneIntersect)
FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Box, Plane, details::boxPlaneIntersect)
FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Capsule, Plane, details::capsulePlaneIntersect)
FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Cylinder, Plane, details::cylinderPlaneIntersect)
FCL_GJK_INDEP_SHAPE_SHAPE_INTERSECT(Cone, Plane, details::conePlaneIntersect)

template <typename Scalar>
struct ShapeIntersectIndepImpl<Scalar, Halfspace<Scalar>, Halfspace<Scalar>>
{
  bool operator()(
      const GJKSolver_indep<Scalar>& /*gjkSolver*/,
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
struct ShapeIntersectIndepImpl<Scalar, Plane<Scalar>, Plane<Scalar>>
{
  bool operator()(
      const GJKSolver_indep<Scalar>& /*gjkSolver*/,
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
struct ShapeIntersectIndepImpl<Scalar, Plane<Scalar>, Halfspace<Scalar>>
{
  bool operator()(
      const GJKSolver_indep<Scalar>& /*gjkSolver*/,
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
struct ShapeIntersectIndepImpl<Scalar, Halfspace<Scalar>, Plane<Scalar>>
{
  bool operator()(
      const GJKSolver_indep<Scalar>& /*gjkSolver*/,
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
struct ShapeTriangleIntersectIndepImpl
{
  bool operator()(
      const GJKSolver_indep<Scalar>& gjkSolver,
      const S& s,
      const Transform3<Scalar>& tf,
      const Vector3<Scalar>& P1,
      const Vector3<Scalar>& P2,
      const Vector3<Scalar>& P3,
      Vector3<Scalar>* contact_points,
      Scalar* penetration_depth,
      Vector3<Scalar>* normal)
  {
    TriangleP<Scalar> tri(P1, P2, P3);

    Vector3<Scalar> guess(1, 0, 0);
    if(gjkSolver.enable_cached_guess) guess = gjkSolver.cached_guess;

    details::MinkowskiDiff<Scalar> shape;
    shape.shapes[0] = &s;
    shape.shapes[1] = &tri;
    shape.toshape1 = tf.linear();
    shape.toshape0 = tf.inverse(Eigen::Isometry);

    details::GJK<Scalar> gjk(gjkSolver.gjk_max_iterations, gjkSolver.gjk_tolerance);
    typename details::GJK<Scalar>::Status gjk_status = gjk.evaluate(shape, -guess);
    if(gjkSolver.enable_cached_guess) gjkSolver.cached_guess = gjk.getGuessFromSimplex();

    switch(gjk_status)
    {
    case details::GJK<Scalar>::Inside:
      {
        details::EPA<Scalar> epa(gjkSolver.epa_max_face_num, gjkSolver.epa_max_vertex_num, gjkSolver.epa_max_iterations, gjkSolver.epa_tolerance);
        typename details::EPA<Scalar>::Status epa_status = epa.evaluate(gjk, -guess);
        if(epa_status != details::EPA<Scalar>::Failed)
        {
          Vector3<Scalar> w0 = Vector3<Scalar>::Zero();
          for(size_t i = 0; i < epa.result.rank; ++i)
          {
            w0 += shape.support(epa.result.c[i]->d, 0) * epa.result.p[i];
          }
          if(penetration_depth) *penetration_depth = -epa.depth;
          if(normal) *normal = -epa.normal;
          if(contact_points) *contact_points = tf * (w0 - epa.normal*(epa.depth *0.5));
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

template<typename Scalar>
template<typename S>
bool GJKSolver_indep<Scalar>::shapeTriangleIntersect(
    const S& s,
    const Transform3<Scalar>& tf,
    const Vector3<Scalar>& P1,
    const Vector3<Scalar>& P2,
    const Vector3<Scalar>& P3,
    Vector3<Scalar>* contact_points,
    Scalar* penetration_depth,
    Vector3<Scalar>* normal) const
{
  ShapeTriangleIntersectIndepImpl<Scalar, S> shapeTriangleIntersectImpl;
  return shapeTriangleIntersectImpl(
        *this, s, tf, P1, P2, P3, contact_points, penetration_depth, normal);
}

//==============================================================================
template<typename Scalar>
struct ShapeTriangleIntersectIndepImpl<Scalar, Sphere<Scalar>>
{
  bool operator()(
      const GJKSolver_indep<Scalar>& /*gjkSolver*/,
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
struct ShapeTransformedTriangleIntersectIndepImpl
{
  bool operator()(
      const GJKSolver_indep<Scalar>& gjkSolver,
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
    TriangleP<Scalar> tri(P1, P2, P3);

    Vector3<Scalar> guess(1, 0, 0);
    if(gjkSolver.enable_cached_guess) guess = gjkSolver.cached_guess;

    details::MinkowskiDiff<Scalar> shape;
    shape.shapes[0] = &s;
    shape.shapes[1] = &tri;
    shape.toshape1 = tf2.linear().transpose() * tf1.linear();
    shape.toshape0 = tf1.inverse(Eigen::Isometry) * tf2;

    details::GJK<Scalar> gjk(gjkSolver.gjk_max_iterations, gjkSolver.gjk_tolerance);
    typename details::GJK<Scalar>::Status gjk_status = gjk.evaluate(shape, -guess);
    if(gjkSolver.enable_cached_guess) gjkSolver.cached_guess = gjk.getGuessFromSimplex();

    switch(gjk_status)
    {
    case details::GJK<Scalar>::Inside:
      {
        details::EPA<Scalar> epa(gjkSolver.epa_max_face_num, gjkSolver.epa_max_vertex_num, gjkSolver.epa_max_iterations, gjkSolver.epa_tolerance);
        typename details::EPA<Scalar>::Status epa_status = epa.evaluate(gjk, -guess);
        if(epa_status != details::EPA<Scalar>::Failed)
        {
          Vector3<Scalar> w0 = Vector3<Scalar>::Zero();
          for(size_t i = 0; i < epa.result.rank; ++i)
          {
            w0 += shape.support(epa.result.c[i]->d, 0) * epa.result.p[i];
          }
          if(penetration_depth) *penetration_depth = -epa.depth;
          if(normal) *normal = -epa.normal;
          if(contact_points) *contact_points = tf1 * (w0 - epa.normal*(epa.depth *0.5));
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

template<typename Scalar>
template<typename S>
bool GJKSolver_indep<Scalar>::shapeTriangleIntersect(
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
  ShapeTransformedTriangleIntersectIndepImpl<Scalar, S> shapeTriangleIntersectImpl;
  return shapeTriangleIntersectImpl(
        *this, s, tf1, P1, P2, P3, tf2,
        contact_points, penetration_depth, normal);
}

//==============================================================================
template<typename Scalar>
struct ShapeTransformedTriangleIntersectIndepImpl<Scalar, Sphere<Scalar>>
{
  bool operator()(
      const GJKSolver_indep<Scalar>& /*gjkSolver*/,
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
struct ShapeTransformedTriangleIntersectIndepImpl<Scalar, Halfspace<Scalar>>
{
  bool operator()(
      const GJKSolver_indep<Scalar>& /*gjkSolver*/,
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
struct ShapeTransformedTriangleIntersectIndepImpl<Scalar, Plane<Scalar>>
{
  bool operator()(
      const GJKSolver_indep<Scalar>& /*gjkSolver*/,
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
struct ShapeDistanceIndepImpl
{
  bool operator()(
      const GJKSolver_indep<Scalar>& gjkSolver,
      const S1& s1,
      const Transform3<Scalar>& tf1,
      const S2& s2,
      const Transform3<Scalar>& tf2,
      Scalar* distance,
      Vector3<Scalar>* p1,
      Vector3<Scalar>* p2)
  {
    Vector3<Scalar> guess(1, 0, 0);
    if(gjkSolver.enable_cached_guess) guess = gjkSolver.cached_guess;

    details::MinkowskiDiff<Scalar> shape;
    shape.shapes[0] = &s1;
    shape.shapes[1] = &s2;
    shape.toshape1 = tf2.linear().transpose() * tf1.linear();
    shape.toshape0 = tf1.inverse(Eigen::Isometry) * tf2;

    details::GJK<Scalar> gjk(gjkSolver.gjk_max_iterations, gjkSolver.gjk_tolerance);
    typename details::GJK<Scalar>::Status gjk_status = gjk.evaluate(shape, -guess);
    if(gjkSolver.enable_cached_guess) gjkSolver.cached_guess = gjk.getGuessFromSimplex();

    if(gjk_status == details::GJK<Scalar>::Valid)
    {
      Vector3<Scalar> w0 = Vector3<Scalar>::Zero();
      Vector3<Scalar> w1 = Vector3<Scalar>::Zero();
      for(size_t i = 0; i < gjk.getSimplex()->rank; ++i)
      {
        Scalar p = gjk.getSimplex()->p[i];
        w0 += shape.support(gjk.getSimplex()->c[i]->d, 0) * p;
        w1 += shape.support(-gjk.getSimplex()->c[i]->d, 1) * p;
      }

      if(distance) *distance = (w0 - w1).norm();

      if(p1) *p1 = w0;
      if(p2) *p2 = shape.toshape0 * w1;

      return true;
    }
    else
    {
      if(distance) *distance = -1;
      return false;
    }
  }
};

template<typename Scalar>
template<typename S1, typename S2>
bool GJKSolver_indep<Scalar>::shapeDistance(
    const S1& s1,
    const Transform3<Scalar>& tf1,
    const S2& s2,
    const Transform3<Scalar>& tf2,
    Scalar* dist,
    Vector3<Scalar>* p1,
    Vector3<Scalar>* p2) const
{
  ShapeDistanceIndepImpl<Scalar, S1, S2> shapeDistanceImpl;
  return shapeDistanceImpl(*this, s1, tf1, s2, tf2, dist, p1, p2);
}

// Shape distance algorithms not using built-in GJK algorithm
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
struct ShapeDistanceIndepImpl<Scalar, Sphere<Scalar>, Capsule<Scalar>>
{
  bool operator()(
      const GJKSolver_indep<Scalar>& /*gjkSolver*/,
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
struct ShapeDistanceIndepImpl<Scalar, Capsule<Scalar>, Sphere<Scalar>>
{
  bool operator()(
      const GJKSolver_indep<Scalar>& /*gjkSolver*/,
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
struct ShapeDistanceIndepImpl<Scalar, Sphere<Scalar>, Sphere<Scalar>>
{
  bool operator()(
      const GJKSolver_indep<Scalar>& /*gjkSolver*/,
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
struct ShapeDistanceIndepImpl<Scalar, Capsule<Scalar>, Capsule<Scalar>>
{
  bool operator()(
      const GJKSolver_indep<Scalar>& /*gjkSolver*/,
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
struct ShapeTriangleDistanceIndepImpl
{
  bool operator()(
      const GJKSolver_indep<Scalar>& gjkSolver,
      const S& s,
      const Transform3<Scalar>& tf,
      const Vector3<Scalar>& P1,
      const Vector3<Scalar>& P2,
      const Vector3<Scalar>& P3,
      Scalar* distance,
      Vector3<Scalar>* p1,
      Vector3<Scalar>* p2)
  {
    TriangleP<Scalar> tri(P1, P2, P3);
    Vector3<Scalar> guess(1, 0, 0);
    if(gjkSolver.enable_cached_guess) guess = gjkSolver.cached_guess;

    details::MinkowskiDiff<Scalar> shape;
    shape.shapes[0] = &s;
    shape.shapes[1] = &tri;
    shape.toshape1 = tf.linear();
    shape.toshape0 = tf.inverse(Eigen::Isometry);

    details::GJK<Scalar> gjk(gjkSolver.gjk_max_iterations, gjkSolver.gjk_tolerance);
    typename details::GJK<Scalar>::Status gjk_status = gjk.evaluate(shape, -guess);
    if(gjkSolver.enable_cached_guess) gjkSolver.cached_guess = gjk.getGuessFromSimplex();

    if(gjk_status == details::GJK<Scalar>::Valid)
    {
      Vector3<Scalar> w0 = Vector3<Scalar>::Zero();
      Vector3<Scalar> w1 = Vector3<Scalar>::Zero();
      for(size_t i = 0; i < gjk.getSimplex()->rank; ++i)
      {
        Scalar p = gjk.getSimplex()->p[i];
        w0 += shape.support(gjk.getSimplex()->c[i]->d, 0) * p;
        w1 += shape.support(-gjk.getSimplex()->c[i]->d, 1) * p;
      }

      if(distance) *distance = (w0 - w1).norm();
      if(p1) *p1 = w0;
      if(p2) *p2 = shape.toshape0 * w1;
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
template<typename Scalar>
template<typename S>
bool GJKSolver_indep<Scalar>::shapeTriangleDistance(
    const S& s,
    const Transform3<Scalar>& tf,
    const Vector3<Scalar>& P1,
    const Vector3<Scalar>& P2,
    const Vector3<Scalar>& P3,
    Scalar* dist,
    Vector3<Scalar>* p1,
    Vector3<Scalar>* p2) const
{
  ShapeTriangleDistanceIndepImpl<Scalar, S> shapeTriangleDistanceImpl;
  return shapeTriangleDistanceImpl(
        *this, s, tf, P1, P2, P3, dist, p1, p2);
}

//==============================================================================
template<typename Scalar>
struct ShapeTriangleDistanceIndepImpl<Scalar, Sphere<Scalar>>
{
  bool operator()(
      const GJKSolver_indep<Scalar>& /*gjkSolver*/,
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
struct ShapeTransformedTriangleDistanceIndepImpl
{
  bool operator()(
      const GJKSolver_indep<Scalar>& gjkSolver,
      const S& s,
      const Transform3<Scalar>& tf1,
      const Vector3<Scalar>& P1,
      const Vector3<Scalar>& P2,
      const Vector3<Scalar>& P3,
      const Transform3<Scalar>& tf2,
      Scalar* distance,
      Vector3<Scalar>* p1,
      Vector3<Scalar>* p2)
  {
    TriangleP<Scalar> tri(P1, P2, P3);
    Vector3<Scalar> guess(1, 0, 0);
    if(gjkSolver.enable_cached_guess) guess = gjkSolver.cached_guess;

    details::MinkowskiDiff<Scalar> shape;
    shape.shapes[0] = &s;
    shape.shapes[1] = &tri;
    shape.toshape1 = tf2.linear().transpose() * tf1.linear();
    shape.toshape0 = tf1.inverse(Eigen::Isometry) * tf2;

    details::GJK<Scalar> gjk(gjkSolver.gjk_max_iterations, gjkSolver.gjk_tolerance);
    typename details::GJK<Scalar>::Status gjk_status = gjk.evaluate(shape, -guess);
    if(gjkSolver.enable_cached_guess) gjkSolver.cached_guess = gjk.getGuessFromSimplex();

    if(gjk_status == details::GJK<Scalar>::Valid)
    {
      Vector3<Scalar> w0 = Vector3<Scalar>::Zero();
      Vector3<Scalar> w1 = Vector3<Scalar>::Zero();
      for(size_t i = 0; i < gjk.getSimplex()->rank; ++i)
      {
        Scalar p = gjk.getSimplex()->p[i];
        w0 += shape.support(gjk.getSimplex()->c[i]->d, 0) * p;
        w1 += shape.support(-gjk.getSimplex()->c[i]->d, 1) * p;
      }

      if(distance) *distance = (w0 - w1).norm();
      if(p1) *p1 = w0;
      if(p2) *p2 = shape.toshape0 * w1;
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
template<typename Scalar>
template<typename S>
bool GJKSolver_indep<Scalar>::shapeTriangleDistance(
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
  ShapeTransformedTriangleDistanceIndepImpl<Scalar, S> shapeTriangleDistanceImpl;
  return shapeTriangleDistanceImpl(
        *this, s, tf1, P1, P2, P3, tf2, dist, p1, p2);
}

//==============================================================================
template<typename Scalar>
struct ShapeTransformedTriangleDistanceIndepImpl<Scalar, Sphere<Scalar>>
{
  bool operator()(
      const GJKSolver_indep<Scalar>& /*gjkSolver*/,
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
template <typename Scalar>
GJKSolver_indep<Scalar>::GJKSolver_indep()
{
  gjk_max_iterations = 128;
  gjk_tolerance = 1e-6;
  epa_max_face_num = 128;
  epa_max_vertex_num = 64;
  epa_max_iterations = 255;
  epa_tolerance = 1e-6;
  enable_cached_guess = false;
  cached_guess = Vector3<Scalar>(1, 0, 0);
}

//==============================================================================
template <typename Scalar>
void GJKSolver_indep<Scalar>::enableCachedGuess(bool if_enable) const
{
  enable_cached_guess = if_enable;
}

//==============================================================================
template <typename Scalar>
void GJKSolver_indep<Scalar>::setCachedGuess(const Vector3<Scalar>& guess) const
{
  cached_guess = guess;
}

//==============================================================================
template <typename Scalar>
Vector3<Scalar> GJKSolver_indep<Scalar>::getCachedGuess() const
{
  return cached_guess;
}

} // namespace fcl

#endif

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

#ifndef FCL_NARROWPHASE_H
#define FCL_NARROWPHASE_H

#include <algorithm>

#include "fcl/collision_data.h"
#include "fcl/narrowphase/gjk.h"
#include "fcl/narrowphase/gjk_libccd.h"

namespace fcl
{
/// @brief collision and distance solver based on libccd library.
struct GJKSolver_libccd
{
  /// @brief intersection checking between two shapes
  /// @deprecated use shapeIntersect(const S1&, const Transform3f&, const S2&, const Transform3f&, std::vector<ContactPoint>*) const
  template<typename S1, typename S2>
  FCL_DEPRECATED
  bool shapeIntersect(const S1& s1, const Transform3f& tf1,
                      const S2& s2, const Transform3f& tf2,
                      Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  /// @brief intersection checking between two shapes
  template<typename S1, typename S2>
  bool shapeIntersect(const S1& s1, const Transform3f& tf1,
                      const S2& s2, const Transform3f& tf2,
                      std::vector<ContactPoint>* contacts = NULL) const
  {
    void* o1 = details::GJKInitializer<S1>::createGJKObject(s1, tf1);
    void* o2 = details::GJKInitializer<S2>::createGJKObject(s2, tf2);

    bool res;

    if(contacts)
    {
      Vec3f normal;
      Vec3f point;
      FCL_REAL depth;
      res = details::GJKCollide(o1, details::GJKInitializer<S1>::getSupportFunction(), details::GJKInitializer<S1>::getCenterFunction(),
                                o2, details::GJKInitializer<S2>::getSupportFunction(), details::GJKInitializer<S2>::getCenterFunction(),
                                max_collision_iterations, collision_tolerance,
                                &point, &depth, &normal);
      contacts->push_back(ContactPoint(normal, point, depth));
    }
    else
    {
      res = details::GJKCollide(o1, details::GJKInitializer<S1>::getSupportFunction(), details::GJKInitializer<S1>::getCenterFunction(),
                                o2, details::GJKInitializer<S2>::getSupportFunction(), details::GJKInitializer<S2>::getCenterFunction(),
                                max_collision_iterations, collision_tolerance,
                                NULL, NULL, NULL);
    }

    details::GJKInitializer<S1>::deleteGJKObject(o1);
    details::GJKInitializer<S2>::deleteGJKObject(o2);

    return res;
  }

  /// @brief intersection checking between one shape and a triangle
  template<typename S>
  bool shapeTriangleIntersect(const S& s, const Transform3f& tf,
                              const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, Vec3f* contact_points = NULL, FCL_REAL* penetration_depth = NULL, Vec3f* normal = NULL) const
  {
    void* o1 = details::GJKInitializer<S>::createGJKObject(s, tf);
    void* o2 = details::triCreateGJKObject(P1, P2, P3);

    bool res = details::GJKCollide(o1, details::GJKInitializer<S>::getSupportFunction(), details::GJKInitializer<S>::getCenterFunction(),
                                   o2, details::triGetSupportFunction(), details::triGetCenterFunction(),
                                   max_collision_iterations, collision_tolerance,
                                   contact_points, penetration_depth, normal);

    details::GJKInitializer<S>::deleteGJKObject(o1);
    details::triDeleteGJKObject(o2);

    return res;
  }

  //// @brief intersection checking between one shape and a triangle with transformation
  template<typename S>
  bool shapeTriangleIntersect(const S& s, const Transform3f& tf1,
                              const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Transform3f& tf2,
                              Vec3f* contact_points = NULL, FCL_REAL* penetration_depth = NULL, Vec3f* normal = NULL) const
  {
    void* o1 = details::GJKInitializer<S>::createGJKObject(s, tf1);
    void* o2 = details::triCreateGJKObject(P1, P2, P3, tf2);

    bool res = details::GJKCollide(o1, details::GJKInitializer<S>::getSupportFunction(), details::GJKInitializer<S>::getCenterFunction(),
                                   o2, details::triGetSupportFunction(), details::triGetCenterFunction(),
                                   max_collision_iterations, collision_tolerance,
                                   contact_points, penetration_depth, normal);

    details::GJKInitializer<S>::deleteGJKObject(o1);
    details::triDeleteGJKObject(o2);

    return res;
  }


  /// @brief distance computation between two shapes
  template<typename S1, typename S2>
  bool shapeDistance(const S1& s1, const Transform3f& tf1,
                     const S2& s2, const Transform3f& tf2,
                     FCL_REAL* dist = NULL, Vec3f* p1 = NULL, Vec3f* p2 = NULL) const
  {
    void* o1 = details::GJKInitializer<S1>::createGJKObject(s1, tf1);
    void* o2 = details::GJKInitializer<S2>::createGJKObject(s2, tf2);

    bool res =  details::GJKDistance(o1, details::GJKInitializer<S1>::getSupportFunction(),
                                     o2, details::GJKInitializer<S2>::getSupportFunction(),
                                     max_distance_iterations, distance_tolerance,
                                     dist, p1, p2);

    if(p1) *p1 = inverse(tf1).transform(*p1);
    if(p2) *p2 = inverse(tf2).transform(*p2);

    details::GJKInitializer<S1>::deleteGJKObject(o1);
    details::GJKInitializer<S2>::deleteGJKObject(o2);

    return res;
  }


  /// @brief distance computation between one shape and a triangle
  template<typename S>
  bool shapeTriangleDistance(const S& s, const Transform3f& tf,
                             const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, 
                             FCL_REAL* dist = NULL, Vec3f* p1 = NULL, Vec3f* p2 = NULL) const
  {
    void* o1 = details::GJKInitializer<S>::createGJKObject(s, tf);
    void* o2 = details::triCreateGJKObject(P1, P2, P3);

    bool res = details::GJKDistance(o1, details::GJKInitializer<S>::getSupportFunction(), 
                                    o2, details::triGetSupportFunction(),
                                    max_distance_iterations, distance_tolerance,
                                    dist, p1, p2);
    if(p1) *p1 = inverse(tf).transform(*p1);
  
    details::GJKInitializer<S>::deleteGJKObject(o1);
    details::triDeleteGJKObject(o2);

    return res;
  }
  
  /// @brief distance computation between one shape and a triangle with transformation
  template<typename S>
  bool shapeTriangleDistance(const S& s, const Transform3f& tf1,
                             const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Transform3f& tf2,
                             FCL_REAL* dist = NULL, Vec3f* p1 = NULL, Vec3f* p2 = NULL) const
  {
    void* o1 = details::GJKInitializer<S>::createGJKObject(s, tf1);
    void* o2 = details::triCreateGJKObject(P1, P2, P3, tf2);

    bool res = details::GJKDistance(o1, details::GJKInitializer<S>::getSupportFunction(),
                                    o2, details::triGetSupportFunction(),
                                    max_distance_iterations, distance_tolerance,
                                    dist, p1, p2);
    if(p1) *p1 = inverse(tf1).transform(*p1);
    if(p2) *p2 = inverse(tf2).transform(*p2);
  
    details::GJKInitializer<S>::deleteGJKObject(o1);
    details::triDeleteGJKObject(o2);

    return res;
  }

  /// @brief default setting for GJK algorithm
  GJKSolver_libccd()
  {
    max_collision_iterations = 500;
    max_distance_iterations = 1000;
    collision_tolerance = 1e-6;
    distance_tolerance = 1e-6;
  }


  void enableCachedGuess(bool if_enable) const
  {
    // TODO: need change libccd to exploit spatial coherence
  }

  void setCachedGuess(const Vec3f& guess) const
  {
    // TODO: need change libccd to exploit spatial coherence
  }

  Vec3f getCachedGuess() const
  {
    return Vec3f(-1, 0, 0);
  }


  /// @brief maximum number of iterations used in GJK algorithm for collision
  unsigned int max_collision_iterations;

  /// @brief maximum number of iterations used in GJK algorithm for distance
  unsigned int max_distance_iterations;

  /// @brief the threshold used in GJK algorithm to stop collision iteration
  FCL_REAL collision_tolerance;

  /// @brief the threshold used in GJK algorithm to stop distance iteration
  FCL_REAL distance_tolerance;
  

};

template<typename S1, typename S2>
bool GJKSolver_libccd::shapeIntersect(const S1& s1, const Transform3f& tf1,
                                      const S2& s2, const Transform3f& tf2,
                                      Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const
{
  bool res;

  if (contact_points || penetration_depth || normal)
  {
    std::vector<ContactPoint> contacts;

    res = shapeIntersect(s1, tf1, s2, tf2, &contacts);

    if (!contacts.empty())
    {
      // Get the deepest contact point
      const ContactPoint& maxDepthContact = *std::max_element(contacts.begin(), contacts.end(), comparePenDepth);

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

/// @brief Fast implementation for sphere-capsule collision
template<>
bool GJKSolver_libccd::shapeIntersect<Sphere, Capsule>(const Sphere& s1, const Transform3f& tf1,
                                                       const Capsule& s2, const Transform3f& tf2,
                                                       std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Capsule, Sphere>(const Capsule &s1, const Transform3f& tf1,
                                                       const Sphere &s2, const Transform3f& tf2,
                                                       std::vector<ContactPoint>* contacts) const;

/// @brief Fast implementation for sphere-sphere collision
template<>
bool GJKSolver_libccd::shapeIntersect<Sphere, Sphere>(const Sphere& s1, const Transform3f& tf1,
                                                      const Sphere& s2, const Transform3f& tf2,
                                                      std::vector<ContactPoint>* contacts) const;

/// @brief Fast implementation for box-box collision
template<>
bool GJKSolver_libccd::shapeIntersect<Box, Box>(const Box& s1, const Transform3f& tf1,
                                                const Box& s2, const Transform3f& tf2,
                                                std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Sphere, Halfspace>(const Sphere& s1, const Transform3f& tf1,
                                                         const Halfspace& s2, const Transform3f& tf2,
                                                         std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Halfspace, Sphere>(const Halfspace& s1, const Transform3f& tf1,
                                                         const Sphere& s2, const Transform3f& tf2,
                                                         std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Ellipsoid, Halfspace>(const Ellipsoid& s1, const Transform3f& tf1,
                                                            const Halfspace& s2, const Transform3f& tf2,
                                                            std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Halfspace, Ellipsoid>(const Halfspace& s1, const Transform3f& tf1,
                                                            const Ellipsoid& s2, const Transform3f& tf2,
                                                            std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Box, Halfspace>(const Box& s1, const Transform3f& tf1,
                                                      const Halfspace& s2, const Transform3f& tf2,
                                                      std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Halfspace, Box>(const Halfspace& s1, const Transform3f& tf1,
                                                      const Box& s2, const Transform3f& tf2,
                                                      std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Capsule, Halfspace>(const Capsule& s1, const Transform3f& tf1,
                                                          const Halfspace& s2, const Transform3f& tf2,
                                                          std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Halfspace, Capsule>(const Halfspace& s1, const Transform3f& tf1,
                                                          const Capsule& s2, const Transform3f& tf2,
                                                          std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Cylinder, Halfspace>(const Cylinder& s1, const Transform3f& tf1,
                                                           const Halfspace& s2, const Transform3f& tf2,
                                                           std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Halfspace, Cylinder>(const Halfspace& s1, const Transform3f& tf1,
                                                           const Cylinder& s2, const Transform3f& tf2,
                                                           std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Cone, Halfspace>(const Cone& s1, const Transform3f& tf1,
                                                       const Halfspace& s2, const Transform3f& tf2,
                                                       std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Halfspace, Cone>(const Halfspace& s1, const Transform3f& tf1,
                                                       const Cone& s2, const Transform3f& tf2,
                                                       std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Halfspace, Halfspace>(const Halfspace& s1, const Transform3f& tf1,
                                                            const Halfspace& s2, const Transform3f& tf2,
                                                            std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Plane, Halfspace>(const Plane& s1, const Transform3f& tf1,
                                                        const Halfspace& s2, const Transform3f& tf2,
                                                        std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Halfspace, Plane>(const Halfspace& s1, const Transform3f& tf1,
                                                        const Plane& s2, const Transform3f& tf2,
                                                        std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Sphere, Plane>(const Sphere& s1, const Transform3f& tf1,
                                                     const Plane& s2, const Transform3f& tf2,
                                                     std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Plane, Sphere>(const Plane& s1, const Transform3f& tf1,
                                                     const Sphere& s2, const Transform3f& tf2,
                                                     std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Ellipsoid, Plane>(const Ellipsoid& s1, const Transform3f& tf1,
                                                        const Plane& s2, const Transform3f& tf2,
                                                        std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Plane, Ellipsoid>(const Plane& s1, const Transform3f& tf1,
                                                        const Ellipsoid& s2, const Transform3f& tf2,
                                                        std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Box, Plane>(const Box& s1, const Transform3f& tf1,
                                                  const Plane& s2, const Transform3f& tf2,
                                                  std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Plane, Box>(const Plane& s1, const Transform3f& tf1,
                                                  const Box& s2, const Transform3f& tf2,
                                                  std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Capsule, Plane>(const Capsule& s1, const Transform3f& tf1,
                                                      const Plane& s2, const Transform3f& tf2,
                                                      std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Plane, Capsule>(const Plane& s1, const Transform3f& tf1,
                                                      const Capsule& s2, const Transform3f& tf2,
                                                      std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Cylinder, Plane>(const Cylinder& s1, const Transform3f& tf1,
                                                       const Plane& s2, const Transform3f& tf2,
                                                       std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Plane, Cylinder>(const Plane& s1, const Transform3f& tf1,
                                                       const Cylinder& s2, const Transform3f& tf2,
                                                       std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Cone, Plane>(const Cone& s1, const Transform3f& tf1,
                                                   const Plane& s2, const Transform3f& tf2,
                                                   std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Plane, Cone>(const Plane& s1, const Transform3f& tf1,
                                                   const Cone& s2, const Transform3f& tf2,
                                                   std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_libccd::shapeIntersect<Plane, Plane>(const Plane& s1, const Transform3f& tf1,
                                                    const Plane& s2, const Transform3f& tf2,
                                                    std::vector<ContactPoint>* contacts) const;

/// @brief Fast implementation for sphere-triangle collision
template<> 
bool GJKSolver_libccd::shapeTriangleIntersect(const Sphere& s, const Transform3f& tf,
                                              const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

/// @brief Fast implementation for sphere-triangle collision
template<> 
bool GJKSolver_libccd::shapeTriangleIntersect(const Sphere& s, const Transform3f& tf1,
                                              const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Transform3f& tf2, Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;


template<>
bool GJKSolver_libccd::shapeTriangleIntersect(const Halfspace& s, const Transform3f& tf1,
                                              const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Transform3f& tf2, Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

template<>
bool GJKSolver_libccd::shapeTriangleIntersect(const Plane& s, const Transform3f& tf1,
                                              const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Transform3f& tf2, Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

/// @brief Fast implementation for sphere-capsule distance
template<>
bool GJKSolver_libccd::shapeDistance<Sphere, Capsule>(const Sphere& s1, const Transform3f& tf1,
                                                      const Capsule& s2, const Transform3f& tf2,
                                                      FCL_REAL* dist, Vec3f* p1, Vec3f* p2) const;

template<>
bool GJKSolver_libccd::shapeDistance<Capsule, Sphere>(const Capsule& s1, const Transform3f& tf1,
                                                      const Sphere& s2, const Transform3f& tf2,
                                                      FCL_REAL* dist, Vec3f* p1, Vec3f* p2) const;

/// @brief Fast implementation for sphere-sphere distance
template<>
bool GJKSolver_libccd::shapeDistance<Sphere, Sphere>(const Sphere& s1, const Transform3f& tf1,
                                                     const Sphere& s2, const Transform3f& tf2,
                                                     FCL_REAL* dist, Vec3f* p1, Vec3f* p2) const;

// @brief Computation of the distance result for capsule capsule. Closest points are based on two line-segments.
template<>
bool GJKSolver_libccd::shapeDistance<Capsule, Capsule>(const Capsule& s1, const Transform3f& tf1,
                                                       const Capsule& s2, const Transform3f& tf2,
                                                       FCL_REAL* dist, Vec3f* p1, Vec3f* p2) const;

/// @brief Fast implementation for sphere-triangle distance
template<>
bool GJKSolver_libccd::shapeTriangleDistance<Sphere>(const Sphere& s, const Transform3f& tf,
                                                     const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, 
                                                     FCL_REAL* dist, Vec3f* p1, Vec3f* p2) const;

/// @brief Fast implementation for sphere-triangle distance
template<> 
bool GJKSolver_libccd::shapeTriangleDistance<Sphere>(const Sphere& s, const Transform3f& tf1, 
                                                     const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Transform3f& tf2,
                                                     FCL_REAL* dist, Vec3f* p1, Vec3f* p2) const;

/// @brief collision and distance solver based on GJK algorithm implemented in fcl (rewritten the code from the GJK in bullet)
struct GJKSolver_indep
{  
  /// @brief intersection checking between two shapes
  /// @deprecated use shapeIntersect(const S1&, const Transform3f&, const S2&, const Transform3f&, std::vector<ContactPoint>*) const
  template<typename S1, typename S2>
  FCL_DEPRECATED
  bool shapeIntersect(const S1& s1, const Transform3f& tf1,
                      const S2& s2, const Transform3f& tf2,
                      Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  /// @brief intersection checking between two shapes
  template<typename S1, typename S2>
  bool shapeIntersect(const S1& s1, const Transform3f& tf1,
                      const S2& s2, const Transform3f& tf2,
                      std::vector<ContactPoint>* contacts = NULL) const
  {
    Vec3f guess(1, 0, 0);
    if(enable_cached_guess) guess = cached_guess;

    details::MinkowskiDiff shape;
    shape.shapes[0] = &s1;
    shape.shapes[1] = &s2;
    shape.toshape1 = tf2.getRotation().transposeTimes(tf1.getRotation());
    shape.toshape0 = tf1.inverseTimes(tf2);

    details::GJK gjk(gjk_max_iterations, gjk_tolerance);
    details::GJK::Status gjk_status = gjk.evaluate(shape, -guess);
    if(enable_cached_guess) cached_guess = gjk.getGuessFromSimplex();

    switch(gjk_status)
    {
    case details::GJK::Inside:
      {
        details::EPA epa(epa_max_face_num, epa_max_vertex_num, epa_max_iterations, epa_tolerance);
        details::EPA::Status epa_status = epa.evaluate(gjk, -guess);
        if(epa_status != details::EPA::Failed)
        {
          Vec3f w0;
          for(size_t i = 0; i < epa.result.rank; ++i)
          {
            w0 += shape.support(epa.result.c[i]->d, 0) * epa.result.p[i];
          }
          if(contacts)
          {
            Vec3f normal = epa.normal;
            Vec3f point = tf1.transform(w0 - epa.normal*(epa.depth *0.5));
            FCL_REAL depth = -epa.depth;
            contacts->push_back(ContactPoint(normal, point, depth));
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

  /// @brief intersection checking between one shape and a triangle
  template<typename S>
  bool shapeTriangleIntersect(const S& s, const Transform3f& tf,
                              const Vec3f& P1, const Vec3f& P2, const Vec3f& P3,
                              Vec3f* contact_points = NULL, FCL_REAL* penetration_depth = NULL, Vec3f* normal = NULL) const
  {
    TriangleP tri(P1, P2, P3);
    
    Vec3f guess(1, 0, 0);
    if(enable_cached_guess) guess = cached_guess;

    details::MinkowskiDiff shape;
    shape.shapes[0] = &s;
    shape.shapes[1] = &tri;
    shape.toshape1 = tf.getRotation();
    shape.toshape0 = inverse(tf);
  
    details::GJK gjk(gjk_max_iterations, gjk_tolerance);
    details::GJK::Status gjk_status = gjk.evaluate(shape, -guess);
    if(enable_cached_guess) cached_guess = gjk.getGuessFromSimplex();

    switch(gjk_status)
    {
    case details::GJK::Inside:
      {
        details::EPA epa(epa_max_face_num, epa_max_vertex_num, epa_max_iterations, epa_tolerance);
        details::EPA::Status epa_status = epa.evaluate(gjk, -guess);
        if(epa_status != details::EPA::Failed)
        {
          Vec3f w0;
          for(size_t i = 0; i < epa.result.rank; ++i)
          {
            w0 += shape.support(epa.result.c[i]->d, 0) * epa.result.p[i];
          }
          if(penetration_depth) *penetration_depth = -epa.depth;
          if(normal) *normal = -epa.normal;
          if(contact_points) *contact_points = tf.transform(w0 - epa.normal*(epa.depth *0.5));
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

  //// @brief intersection checking between one shape and a triangle with transformation
  template<typename S>
  bool shapeTriangleIntersect(const S& s, const Transform3f& tf1,
                              const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Transform3f& tf2,
                              Vec3f* contact_points = NULL, FCL_REAL* penetration_depth = NULL, Vec3f* normal = NULL) const
  {
    TriangleP tri(P1, P2, P3);

    Vec3f guess(1, 0, 0);
    if(enable_cached_guess) guess = cached_guess;

    details::MinkowskiDiff shape;
    shape.shapes[0] = &s;
    shape.shapes[1] = &tri;
    shape.toshape1 = tf2.getRotation().transposeTimes(tf1.getRotation());
    shape.toshape0 = tf1.inverseTimes(tf2);
  
    details::GJK gjk(gjk_max_iterations, gjk_tolerance);
    details::GJK::Status gjk_status = gjk.evaluate(shape, -guess);
    if(enable_cached_guess) cached_guess = gjk.getGuessFromSimplex();

    switch(gjk_status)
    {
    case details::GJK::Inside:
      {
        details::EPA epa(epa_max_face_num, epa_max_vertex_num, epa_max_iterations, epa_tolerance);
        details::EPA::Status epa_status = epa.evaluate(gjk, -guess);
        if(epa_status != details::EPA::Failed)
        {
          Vec3f w0;
          for(size_t i = 0; i < epa.result.rank; ++i)
          {
            w0 += shape.support(epa.result.c[i]->d, 0) * epa.result.p[i];
          }
          if(penetration_depth) *penetration_depth = -epa.depth;
          if(normal) *normal = -epa.normal;
          if(contact_points) *contact_points = tf1.transform(w0 - epa.normal*(epa.depth *0.5));
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

  /// @brief distance computation between two shapes
  template<typename S1, typename S2>
  bool shapeDistance(const S1& s1, const Transform3f& tf1,
                     const S2& s2, const Transform3f& tf2,
                     FCL_REAL* distance = NULL, Vec3f* p1 = NULL, Vec3f* p2 = NULL) const
  {
    Vec3f guess(1, 0, 0);
    if(enable_cached_guess) guess = cached_guess;

    details::MinkowskiDiff shape;
    shape.shapes[0] = &s1;
    shape.shapes[1] = &s2;
    shape.toshape1 = tf2.getRotation().transposeTimes(tf1.getRotation());
    shape.toshape0 = tf1.inverseTimes(tf2);

    details::GJK gjk(gjk_max_iterations, gjk_tolerance);
    details::GJK::Status gjk_status = gjk.evaluate(shape, -guess);
    if(enable_cached_guess) cached_guess = gjk.getGuessFromSimplex();

    if(gjk_status == details::GJK::Valid)
    {
      Vec3f w0, w1;
      for(size_t i = 0; i < gjk.getSimplex()->rank; ++i)
      {
        FCL_REAL p = gjk.getSimplex()->p[i];
        w0 += shape.support(gjk.getSimplex()->c[i]->d, 0) * p;
        w1 += shape.support(-gjk.getSimplex()->c[i]->d, 1) * p;
      }

      if(distance) *distance = (w0 - w1).length();
      
      if(p1) *p1 = w0;
      if(p2) *p2 = shape.toshape0.transform(w1);
      
      return true;
    }
    else
    {
      if(distance) *distance = -1;
      return false;
    }
  }

  /// @brief distance computation between one shape and a triangle
  template<typename S>
  bool shapeTriangleDistance(const S& s, const Transform3f& tf,
                             const Vec3f& P1, const Vec3f& P2, const Vec3f& P3,
                             FCL_REAL* distance = NULL, Vec3f* p1 = NULL, Vec3f* p2 = NULL) const
  {
    TriangleP tri(P1, P2, P3);
    Vec3f guess(1, 0, 0);
    if(enable_cached_guess) guess = cached_guess;

    details::MinkowskiDiff shape;
    shape.shapes[0] = &s;
    shape.shapes[1] = &tri;
    shape.toshape1 = tf.getRotation();
    shape.toshape0 = inverse(tf);

    details::GJK gjk(gjk_max_iterations, gjk_tolerance);
    details::GJK::Status gjk_status = gjk.evaluate(shape, -guess);
    if(enable_cached_guess) cached_guess = gjk.getGuessFromSimplex();
    
    if(gjk_status == details::GJK::Valid)
    {
      Vec3f w0, w1;
      for(size_t i = 0; i < gjk.getSimplex()->rank; ++i)
      {
        FCL_REAL p = gjk.getSimplex()->p[i];
        w0 += shape.support(gjk.getSimplex()->c[i]->d, 0) * p;
        w1 += shape.support(-gjk.getSimplex()->c[i]->d, 1) * p;
      }

      if(distance) *distance = (w0 - w1).length();
      if(p1) *p1 = w0;
      if(p2) *p2 = shape.toshape0.transform(w1);
      return true;
    }
    else
    {
      if(distance) *distance = -1;
      return false;
    }
  }
  
  /// @brief distance computation between one shape and a triangle with transformation
  template<typename S>
  bool shapeTriangleDistance(const S& s, const Transform3f& tf1,
                             const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Transform3f& tf2,
                             FCL_REAL* distance = NULL, Vec3f* p1 = NULL, Vec3f* p2 = NULL) const
  {
    TriangleP tri(P1, P2, P3);
    Vec3f guess(1, 0, 0);
    if(enable_cached_guess) guess = cached_guess;
    
    details::MinkowskiDiff shape;
    shape.shapes[0] = &s;
    shape.shapes[1] = &tri;
    shape.toshape1 = tf2.getRotation().transposeTimes(tf1.getRotation());
    shape.toshape0 = tf1.inverseTimes(tf2);

    details::GJK gjk(gjk_max_iterations, gjk_tolerance);
    details::GJK::Status gjk_status = gjk.evaluate(shape, -guess);
    if(enable_cached_guess) cached_guess = gjk.getGuessFromSimplex();

    if(gjk_status == details::GJK::Valid)
    {
      Vec3f w0, w1;
      for(size_t i = 0; i < gjk.getSimplex()->rank; ++i)
      {
        FCL_REAL p = gjk.getSimplex()->p[i];
        w0 += shape.support(gjk.getSimplex()->c[i]->d, 0) * p;
        w1 += shape.support(-gjk.getSimplex()->c[i]->d, 1) * p;
      }

      if(distance) *distance = (w0 - w1).length();
      if(p1) *p1 = w0;
      if(p2) *p2 = shape.toshape0.transform(w1);
      return true;
    }
    else
    {
      if(distance) *distance = -1;
      return false;
    }
  }
  
  /// @brief default setting for GJK algorithm
  GJKSolver_indep()
  {
    gjk_max_iterations = 128;
    gjk_tolerance = 1e-6;
    epa_max_face_num = 128;
    epa_max_vertex_num = 64;
    epa_max_iterations = 255;
    epa_tolerance = 1e-6;
    enable_cached_guess = false;
    cached_guess = Vec3f(1, 0, 0);
  }

  void enableCachedGuess(bool if_enable) const
  {
    enable_cached_guess = if_enable;
  }

  void setCachedGuess(const Vec3f& guess) const
  {
    cached_guess = guess;
  }

  Vec3f getCachedGuess() const
  {
    return cached_guess;
  }

  /// @brief maximum number of simplex face used in EPA algorithm
  unsigned int epa_max_face_num;

  /// @brief maximum number of simplex vertex used in EPA algorithm
  unsigned int epa_max_vertex_num;

  /// @brief maximum number of iterations used for EPA iterations
  unsigned int epa_max_iterations;

  /// @brief the threshold used in EPA to stop iteration
  FCL_REAL epa_tolerance;

  /// @brief the threshold used in GJK to stop iteration
  FCL_REAL gjk_tolerance;

  /// @brief maximum number of iterations used for GJK iterations
  FCL_REAL gjk_max_iterations;

  /// @brief Whether smart guess can be provided
  mutable bool enable_cached_guess;

  /// @brief smart guess
  mutable Vec3f cached_guess;
};

template<typename S1, typename S2>
bool GJKSolver_indep::shapeIntersect(const S1& s1, const Transform3f& tf1,
                                     const S2& s2, const Transform3f& tf2,
                                     Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const
{
  bool res;

  if (contact_points || penetration_depth || normal)
  {
    std::vector<ContactPoint> contacts;

    res = shapeIntersect(s1, tf1, s2, tf2, &contacts);

    if (!contacts.empty())
    {
      // Get the deepest contact point
      const ContactPoint& maxDepthContact = *std::max_element(contacts.begin(), contacts.end(), comparePenDepth);

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

/// @brief Fast implementation for sphere-capsule collision
template<>
bool GJKSolver_indep::shapeIntersect<Sphere, Capsule>(const Sphere &s1, const Transform3f& tf1,
                                                      const Capsule &s2, const Transform3f& tf2,
                                                      std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Capsule, Sphere>(const Capsule &s1, const Transform3f& tf1,
                                                      const Sphere &s2, const Transform3f& tf2,
                                                      std::vector<ContactPoint>* contacts) const;

/// @brief Fast implementation for sphere-sphere collision
template<>
bool GJKSolver_indep::shapeIntersect<Sphere, Sphere>(const Sphere& s1, const Transform3f& tf1,
                                                     const Sphere& s2, const Transform3f& tf2,
                                                     std::vector<ContactPoint>* contacts) const;

/// @brief Fast implementation for box-box collision
template<>
bool GJKSolver_indep::shapeIntersect<Box, Box>(const Box& s1, const Transform3f& tf1,
                                               const Box& s2, const Transform3f& tf2,
                                               std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Sphere, Halfspace>(const Sphere& s1, const Transform3f& tf1,
                                                        const Halfspace& s2, const Transform3f& tf2,
                                                        std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Halfspace, Sphere>(const Halfspace& s1, const Transform3f& tf1,
                                                        const Sphere& s2, const Transform3f& tf2,
                                                        std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Ellipsoid, Halfspace>(const Ellipsoid& s1, const Transform3f& tf1,
                                                           const Halfspace& s2, const Transform3f& tf2,
                                                           std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Halfspace, Ellipsoid>(const Halfspace& s1, const Transform3f& tf1,
                                                           const Ellipsoid& s2, const Transform3f& tf2,
                                                           std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Box, Halfspace>(const Box& s1, const Transform3f& tf1,
                                                     const Halfspace& s2, const Transform3f& tf2,
                                                     std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Halfspace, Box>(const Halfspace& s1, const Transform3f& tf1,
                                                     const Box& s2, const Transform3f& tf2,
                                                     std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Capsule, Halfspace>(const Capsule& s1, const Transform3f& tf1,
                                                         const Halfspace& s2, const Transform3f& tf2,
                                                         std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Halfspace, Capsule>(const Halfspace& s1, const Transform3f& tf1,
                                                         const Capsule& s2, const Transform3f& tf2,
                                                         std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Cylinder, Halfspace>(const Cylinder& s1, const Transform3f& tf1,
                                                          const Halfspace& s2, const Transform3f& tf2,
                                                          std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Halfspace, Cylinder>(const Halfspace& s1, const Transform3f& tf1,
                                                          const Cylinder& s2, const Transform3f& tf2,
                                                          std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Cone, Halfspace>(const Cone& s1, const Transform3f& tf1,
                                                      const Halfspace& s2, const Transform3f& tf2,
                                                      std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Halfspace, Cone>(const Halfspace& s1, const Transform3f& tf1,
                                                      const Cone& s2, const Transform3f& tf2,
                                                      std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Halfspace, Halfspace>(const Halfspace& s1, const Transform3f& tf1,
                                                           const Halfspace& s2, const Transform3f& tf2,
                                                           std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Plane, Halfspace>(const Plane& s1, const Transform3f& tf1,
                                                       const Halfspace& s2, const Transform3f& tf2,
                                                       std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Halfspace, Plane>(const Halfspace& s1, const Transform3f& tf1,
                                                       const Plane& s2, const Transform3f& tf2,
                                                       std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Sphere, Plane>(const Sphere& s1, const Transform3f& tf1,
                                                    const Plane& s2, const Transform3f& tf2,
                                                    std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Plane, Sphere>(const Plane& s1, const Transform3f& tf1,
                                                    const Sphere& s2, const Transform3f& tf2,
                                                    std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Ellipsoid, Plane>(const Ellipsoid& s1, const Transform3f& tf1,
                                                       const Plane& s2, const Transform3f& tf2,
                                                       std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Plane, Ellipsoid>(const Plane& s1, const Transform3f& tf1,
                                                       const Ellipsoid& s2, const Transform3f& tf2,
                                                       std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Box, Plane>(const Box& s1, const Transform3f& tf1,
                                                 const Plane& s2, const Transform3f& tf2,
                                                 std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Plane, Box>(const Plane& s1, const Transform3f& tf1,
                                                 const Box& s2, const Transform3f& tf2,
                                                 std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Capsule, Plane>(const Capsule& s1, const Transform3f& tf1,
                                                     const Plane& s2, const Transform3f& tf2,
                                                     std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Plane, Capsule>(const Plane& s1, const Transform3f& tf1,
                                                     const Capsule& s2, const Transform3f& tf2,
                                                     std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Cylinder, Plane>(const Cylinder& s1, const Transform3f& tf1,
                                                      const Plane& s2, const Transform3f& tf2,
                                                      std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Plane, Cylinder>(const Plane& s1, const Transform3f& tf1,
                                                      const Cylinder& s2, const Transform3f& tf2,
                                                      std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Cone, Plane>(const Cone& s1, const Transform3f& tf1,
                                                  const Plane& s2, const Transform3f& tf2,
                                                  std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Plane, Cone>(const Plane& s1, const Transform3f& tf1,
                                                  const Cone& s2, const Transform3f& tf2,
                                                  std::vector<ContactPoint>* contacts) const;

template<>
bool GJKSolver_indep::shapeIntersect<Plane, Plane>(const Plane& s1, const Transform3f& tf1,
                                                   const Plane& s2, const Transform3f& tf2,
                                                   std::vector<ContactPoint>* contacts) const;

/// @brief Fast implementation for sphere-triangle collision
template<>
bool GJKSolver_indep::shapeTriangleIntersect(const Sphere& s, const Transform3f& tf,
                                             const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

/// @brief Fast implementation for sphere-triangle collision
template<>
bool GJKSolver_indep::shapeTriangleIntersect(const Sphere& s, const Transform3f& tf1,
                                             const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Transform3f& tf2, Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;


template<>
bool GJKSolver_indep::shapeTriangleIntersect(const Halfspace& s, const Transform3f& tf1,
                                             const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Transform3f& tf2, Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

template<>
bool GJKSolver_indep::shapeTriangleIntersect(const Plane& s, const Transform3f& tf1,
                                             const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Transform3f& tf2, Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

/// @brief Fast implementation for sphere-capsule distance
template<>
bool GJKSolver_indep::shapeDistance<Sphere, Capsule>(const Sphere& s1, const Transform3f& tf1,
                                                     const Capsule& s2, const Transform3f& tf2,
                                                     FCL_REAL* dist, Vec3f* p1, Vec3f* p2) const;

template<>
bool GJKSolver_indep::shapeDistance<Capsule, Sphere>(const Capsule& s1, const Transform3f& tf1,
                                                     const Sphere& s2, const Transform3f& tf2,
                                                     FCL_REAL* dist, Vec3f* p1, Vec3f* p2) const;

/// @brief Fast implementation for sphere-sphere distance
template<>
bool GJKSolver_indep::shapeDistance<Sphere, Sphere>(const Sphere& s1, const Transform3f& tf1,
                                                    const Sphere& s2, const Transform3f& tf2,
                                                    FCL_REAL* dist, Vec3f* p1, Vec3f* p2) const;

// @brief Computation of the distance result for capsule capsule. Closest points are based on two line-segments.
template<>
bool GJKSolver_indep::shapeDistance<Capsule, Capsule>(const Capsule& s1, const Transform3f& tf1,
                                                      const Capsule& s2, const Transform3f& tf2,
                                                      FCL_REAL* dist, Vec3f* p1, Vec3f* p2) const;

/// @brief Fast implementation for sphere-triangle distance
template<>
bool GJKSolver_indep::shapeTriangleDistance<Sphere>(const Sphere& s, const Transform3f& tf,
                                                    const Vec3f& P1, const Vec3f& P2, const Vec3f& P3,
                                                    FCL_REAL* dist, Vec3f* p1, Vec3f* p2) const;

/// @brief Fast implementation for sphere-triangle distance
template<>
bool GJKSolver_indep::shapeTriangleDistance<Sphere>(const Sphere& s, const Transform3f& tf1,
                                                    const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Transform3f& tf2,
                                                    FCL_REAL* dist, Vec3f* p1, Vec3f* p2) const;

}

#endif

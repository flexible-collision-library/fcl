/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#include "fcl/narrowphase/gjk.h"
#include "fcl/narrowphase/gjk_libccd.h"



namespace fcl
{

struct GJKSolver_libccd
{
  /** intersection checking between two shapes */
  template<typename S1, typename S2>
  bool shapeIntersect(const S1& s1, const SimpleTransform& tf1,
                      const S2& s2, const SimpleTransform& tf2,
                      Vec3f* contact_points, BVH_REAL* penetration_depth, Vec3f* normal) const
  {
    void* o1 = details::GJKInitializer<S1>::createGJKObject(s1, tf1);
    void* o2 = details::GJKInitializer<S2>::createGJKObject(s2, tf2);

    bool res = details::GJKCollide(o1, details::GJKInitializer<S1>::getSupportFunction(), details::GJKInitializer<S1>::getCenterFunction(),
                                   o2, details::GJKInitializer<S2>::getSupportFunction(), details::GJKInitializer<S2>::getCenterFunction(),
                                   max_collision_iterations, collision_tolerance,
                                   contact_points, penetration_depth, normal);

    details::GJKInitializer<S1>::deleteGJKObject(o1);
    details::GJKInitializer<S2>::deleteGJKObject(o2);

    return res;
  }

  /** \brief intersection checking between one shape and a triangle */
  template<typename S>
  bool shapeTriangleIntersect(const S& s, const SimpleTransform& tf,
                              const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, Vec3f* contact_points, BVH_REAL* penetration_depth, Vec3f* normal) const
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

  /** \brief intersection checking between one shape and a triangle with transformation */
  template<typename S>
  bool shapeTriangleIntersect(const S& s, const SimpleTransform& tf,
                              const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Matrix3f& R, const Vec3f& T,
                              Vec3f* contact_points, BVH_REAL* penetration_depth, Vec3f* normal) const
  {
    void* o1 = details::GJKInitializer<S>::createGJKObject(s, tf);
    void* o2 = details::triCreateGJKObject(P1, P2, P3, R, T);

    bool res = details::GJKCollide(o1, details::GJKInitializer<S>::getSupportFunction(), details::GJKInitializer<S>::getCenterFunction(),
                                   o2, details::triGetSupportFunction(), details::triGetCenterFunction(),
                                   max_collision_iterations, collision_tolerance,
                                   contact_points, penetration_depth, normal);

    details::GJKInitializer<S>::deleteGJKObject(o1);
    details::triDeleteGJKObject(o2);

    return res;
  }


  /** \brief distance computation between two shapes */
  template<typename S1, typename S2>
  bool shapeDistance(const S1& s1, const SimpleTransform& tf1,
                     const S2& s2, const SimpleTransform& tf2,
                     BVH_REAL* dist) const
  {
    void* o1 = details::GJKInitializer<S1>::createGJKObject(s1, tf1);
    void* o2 = details::GJKInitializer<S2>::createGJKObject(s2, tf2);

    bool res =  details::GJKDistance(o1, details::GJKInitializer<S1>::getSupportFunction(),
                                     o2, details::GJKInitializer<S2>::getSupportFunction(),
                                     max_distance_iterations, distance_tolerance,
                                     dist);

    if(*dist > 0) *dist = std::sqrt(*dist);

    details::GJKInitializer<S1>::deleteGJKObject(o1);
    details::GJKInitializer<S2>::deleteGJKObject(o2);

    return res;
  }


  /** \brief distance computation between one shape and a triangle */
  template<typename S>
  bool shapeTriangleDistance(const S& s, const SimpleTransform& tf,
                             const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, 
                             BVH_REAL* dist) const
  {
    void* o1 = details::GJKInitializer<S>::createGJKObject(s, tf);
    void* o2 = details::triCreateGJKObject(P1, P2, P3);

    bool res = details::GJKDistance(o1, details::GJKInitializer<S>::getSupportFunction(), 
                                    o2, details::triGetSupportFunction(),
                                    max_distance_iterations, distance_tolerance,
                                    dist);

    if(*dist > 0) *dist = std::sqrt(*dist);
  
    details::GJKInitializer<S>::deleteGJKObject(o1);
    details::triDeleteGJKObject(o2);

    return res;
  }

  /** \brief distance computation between one shape and a triangle with transformation */
  template<typename S>
  bool shapeTriangleDistance(const S& s, const SimpleTransform& tf,
                             const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Matrix3f& R, const Vec3f& T,
                             BVH_REAL* dist) const
  {
    void* o1 = details::GJKInitializer<S>::createGJKObject(s, tf);
    void* o2 = details::triCreateGJKObject(P1, P2, P3, R, T);

    bool res = details::GJKDistance(o1, details::GJKInitializer<S>::getSupportFunction(),
                                    o2, details::triGetSupportFunction(),
                                    max_distance_iterations, distance_tolerance,
                                    dist);

    if(*dist > 0) *dist = std::sqrt(*dist);
  
    details::GJKInitializer<S>::deleteGJKObject(o1);
    details::triDeleteGJKObject(o2);

    return res;
  }


  GJKSolver_libccd()
  {
    max_collision_iterations = 500;
    max_distance_iterations = 500;
    collision_tolerance = 1e-6;
    distance_tolerance = 1e-6;
  }

  unsigned int max_collision_iterations;
  unsigned int max_distance_iterations;
  BVH_REAL collision_tolerance;
  BVH_REAL distance_tolerance;
  

};



template<>
bool GJKSolver_libccd::shapeIntersect<Sphere, Sphere>(const Sphere& s1, const SimpleTransform& tf1,
                                                      const Sphere& s2, const SimpleTransform& tf2,
                                                      Vec3f* contact_points, BVH_REAL* penetration_depth, Vec3f* normal) const;
/*
template<> 
bool GJKSolver_libccd::shapeTriangleIntersect(const Sphere& s, const SimpleTransform& tf,
                                              const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, Vec3f* contact_points, BVH_REAL* penetration_depth, Vec3f* normal) const;

template<> 
bool GJKSolver_libccd::shapeTriangleIntersect(const Sphere& s, const SimpleTransform& tf,
                                              const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Matrix3f& R, const Vec3f& T, Vec3f* contact_points, BVH_REAL* penetration_depth, Vec3f* normal) const;


template<>
bool GJKSolver_libccd::shapeIntersect<Box, Box>(const Box& s1, const SimpleTransform& tf1,
                                                const Box& s2, const SimpleTransform& tf2,
                                                Vec3f* contact_points, BVH_REAL* penetration_depth, Vec3f* normal) const;
*/

template<>
bool GJKSolver_libccd::shapeDistance<Sphere, Sphere>(const Sphere& s1, const SimpleTransform& tf1,
                                                     const Sphere& s2, const SimpleTransform& tf2,
                                                     BVH_REAL* dist) const;

struct GJKSolver_indep
{
  template<typename S1, typename S2>
  bool shapeIntersect(const S1& s1, const SimpleTransform& tf1,
                      const S2& s2, const SimpleTransform& tf2,
                      Vec3f* contact_points, BVH_REAL* penetration_depth, Vec3f* normal) const
  {
    Vec3f guess(1, 0, 0);
    details::MinkowskiDiff shape;
    shape.shapes[0] = &s1;
    shape.shapes[1] = &s2;
    shape.toshape1 = tf2.getRotation().transposeTimes(tf1.getRotation());
    shape.toshape0 = tf1.inverseTimes(tf2);
  
    details::GJK gjk(gjk_max_iterations, gjk_tolerance);
    details::GJK::Status gjk_status = gjk.evaluate(shape, -guess);
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


  template<typename S>
  bool shapeTriangleIntersect(const S& s, const SimpleTransform& tf,
                              const Vec3f& P1, const Vec3f& P2, const Vec3f& P3,
                              Vec3f* contact_points, BVH_REAL* penetration_depth, Vec3f* normal) const
  {
    Triangle2 tri(P1, P2, P3);
    Vec3f guess(1, 0, 0);
    details::MinkowskiDiff shape;
    shape.shapes[0] = &s;
    shape.shapes[1] = &tri;
    shape.toshape1 = tf.getRotation();
    shape.toshape0 = tf.inverse();
  
    details::GJK gjk(gjk_max_iterations, gjk_tolerance);
    details::GJK::Status gjk_status = gjk.evaluate(shape, -guess);
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

  template<typename S>
  bool shapeTriangleIntersect(const S& s, const SimpleTransform& tf,
                              const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Matrix3f& R, const Vec3f& T,
                              Vec3f* contact_points, BVH_REAL* penetration_depth, Vec3f* normal) const
  {
    Triangle2 tri(P1, P2, P3);
    SimpleTransform tf2(R, T);
    Vec3f guess(1, 0, 0);
    details::MinkowskiDiff shape;
    shape.shapes[0] = &s;
    shape.shapes[1] = &tri;
    shape.toshape1 = tf2.getRotation().transposeTimes(tf.getRotation());
    shape.toshape0 = tf.inverseTimes(tf2);
  
    details::GJK gjk(gjk_max_iterations, gjk_tolerance);
    details::GJK::Status gjk_status = gjk.evaluate(shape, -guess);
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


  template<typename S1, typename S2>
  bool shapeDistance(const S1& s1, const SimpleTransform& tf1,
                     const S2& s2, const SimpleTransform& tf2,
                     BVH_REAL* distance) const
  {
    Vec3f guess(1, 0, 0);
    details::MinkowskiDiff shape;
    shape.shapes[0] = &s1;
    shape.shapes[1] = &s2;
    shape.toshape1 = tf2.getRotation().transposeTimes(tf1.getRotation());
    shape.toshape0 = tf1.inverseTimes(tf2);

    details::GJK gjk(gjk_max_iterations, gjk_tolerance);
    details::GJK::Status gjk_status = gjk.evaluate(shape, -guess);
    if(gjk_status == details::GJK::Valid)
    {
      Vec3f w0, w1;
      for(size_t i = 0; i < gjk.getSimplex()->rank; ++i)
      {
        BVH_REAL p = gjk.getSimplex()->p[i];
        w0 += shape.support(gjk.getSimplex()->c[i]->d, 0) * p;
        w1 += shape.support(-gjk.getSimplex()->c[i]->d, 1) * p;
      }

      *distance = (w0 - w1).length();
      return true;
    }
    else
    {
      *distance = -1;
      return false;
    }
  }




  template<typename S>
  bool shapeTriangleDistance(const S& s, const SimpleTransform& tf,
                             const Vec3f& P1, const Vec3f& P2, const Vec3f& P3,
                             BVH_REAL* distance) const
  {
    Triangle2 tri(P1, P2, P3);
    Vec3f guess(1, 0, 0);
    details::MinkowskiDiff shape;
    shape.shapes[0] = &s;
    shape.shapes[1] = &tri;
    shape.toshape1 = tf.getRotation();
    shape.toshape0 = tf.inverse();

    details::GJK gjk(gjk_max_iterations, gjk_tolerance);
    details::GJK::Status gjk_status = gjk.evaluate(shape, -guess);
    if(gjk_status == details::GJK::Valid)
    {
      Vec3f w0, w1;
      for(size_t i = 0; i < gjk.getSimplex()->rank; ++i)
      {
        BVH_REAL p = gjk.getSimplex()->p[i];
        w0 += shape.support(gjk.getSimplex()->c[i]->d, 0) * p;
        w1 += shape.support(-gjk.getSimplex()->c[i]->d, 1) * p;
      }

      *distance = (w0 - w1).length();
      return true;
    }
    else
    {
      *distance = -1;
      return false;
    }
  }

  template<typename S>
  bool shapeTriangleDistance(const S& s, const SimpleTransform& tf,
                             const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Matrix3f& R, const Vec3f& T,
                             BVH_REAL* distance) const
  {
    Triangle2 tri(P1, P2, P3);
    SimpleTransform tf2(R, T);
    Vec3f guess(1, 0, 0);
    details::MinkowskiDiff shape;
    shape.shapes[0] = &s;
    shape.shapes[1] = &tri;
    shape.toshape1 = tf2.getRotation().transposeTimes(tf.getRotation());
    shape.toshape0 = tf.inverseTimes(tf2);

    details::GJK gjk(gjk_max_iterations, gjk_tolerance);
    details::GJK::Status gjk_status = gjk.evaluate(shape, -guess);
    if(gjk_status == details::GJK::Valid)
    {
      Vec3f w0, w1;
      for(size_t i = 0; i < gjk.getSimplex()->rank; ++i)
      {
        BVH_REAL p = gjk.getSimplex()->p[i];
        w0 += shape.support(gjk.getSimplex()->c[i]->d, 0) * p;
        w1 += shape.support(-gjk.getSimplex()->c[i]->d, 1) * p;
      }

      *distance = (w0 - w1).length();
      return true;
    }
    else
    {
      *distance = -1;
      return false;
    }
  }


  GJKSolver_indep()
  {
    gjk_max_iterations = 128;
    gjk_tolerance = 1e-6;
    epa_max_face_num = 128;
    epa_max_vertex_num = 64;
    epa_max_iterations = 255;
    epa_tolerance = 1e-6;
  }

  unsigned int epa_max_face_num;
  unsigned int epa_max_vertex_num;
  unsigned int epa_max_iterations;
  BVH_REAL epa_tolerance;
  BVH_REAL gjk_tolerance;
  BVH_REAL gjk_max_iterations;
};

template<>
bool GJKSolver_indep::shapeIntersect<Sphere, Sphere>(const Sphere& s1, const SimpleTransform& tf1,
                                                      const Sphere& s2, const SimpleTransform& tf2,
                                                      Vec3f* contact_points, BVH_REAL* penetration_depth, Vec3f* normal) const;

/*
template<>
bool GJKSolver_indep::shapeIntersect<Box, Box>(const Box& s1, const SimpleTransform& tf1,
                                                const Box& s2, const SimpleTransform& tf2,
                                                Vec3f* contact_points, BVH_REAL* penetration_depth, Vec3f* normal) const;
*/

template<>
bool GJKSolver_indep::shapeDistance<Sphere, Sphere>(const Sphere& s1, const SimpleTransform& tf1,
                                                    const Sphere& s2, const SimpleTransform& tf2,
                                                    BVH_REAL* dist) const;

}

#endif

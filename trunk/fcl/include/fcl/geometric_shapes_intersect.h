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


#ifndef FCL_GEOMETRIC_SHAPES_INTERSECT_H
#define FCL_GEOMETRIC_SHAPES_INTERSECT_H

#include "fcl/geometric_shapes.h"
#include "fcl/transform.h"

#include <ccd/ccd.h>
#include <ccd/quat.h>

namespace fcl
{

/** recall function used by GJK algorithm */
typedef void (*GJKSupportFunction)(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v);
typedef void (*GJKCenterFunction)(const void* obj, ccd_vec3_t* c);

/** initialize GJK stuffs */
template<typename T>
class GJKInitializer
{
public:
  /** \brief Get GJK support function */
  static GJKSupportFunction getSupportFunction() { return NULL; }

  /** \brief Get GJK center function */
  static GJKCenterFunction getCenterFunction() { return NULL; }

  /** \brief Get GJK object from a shape
   * Notice that only local transformation is applied.
   * Gloal transformation are considered later
   */
  static void* createGJKObject(const T& s) { return NULL; }

  /** \brief Delete GJK object */
  static void deleteGJKObject(void* o) {}
};

template<>
class GJKInitializer<Cylinder>
{
public:
  static GJKSupportFunction getSupportFunction();
  static GJKCenterFunction getCenterFunction();
  static void* createGJKObject(const Cylinder& s);
  static void deleteGJKObject(void* o);
};


template<>
class GJKInitializer<Sphere>
{
public:
  static GJKSupportFunction getSupportFunction();
  static GJKCenterFunction getCenterFunction();
  static void* createGJKObject(const Sphere& s);
  static void deleteGJKObject(void* o);
};


template<>
class GJKInitializer<Box>
{
public:
  static GJKSupportFunction getSupportFunction();
  static GJKCenterFunction getCenterFunction();
  static void* createGJKObject(const Box& s);
  static void deleteGJKObject(void* o);
};


template<>
class GJKInitializer<Capsule>
{
public:
  static GJKSupportFunction getSupportFunction();
  static GJKCenterFunction getCenterFunction();
  static void* createGJKObject(const Capsule& s);
  static void deleteGJKObject(void* o);
};


template<>
class GJKInitializer<Cone>
{
public:
  static GJKSupportFunction getSupportFunction();
  static GJKCenterFunction getCenterFunction();
  static void* createGJKObject(const Cone& s);
  static void deleteGJKObject(void* o);
};

template<>
class GJKInitializer<Convex>
{
public:
  static GJKSupportFunction getSupportFunction();
  static GJKCenterFunction getCenterFunction();
  static void* createGJKObject(const Convex& s);
  static void deleteGJKObject(void* o);
};


GJKSupportFunction triGetSupportFunction();

GJKCenterFunction triGetCenterFunction();

void* triCreateGJKObject(const Vec3f& P1, const Vec3f& P2, const Vec3f& P3);

void* triCreateGJKObject(const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Vec3f R[3], const Vec3f& T);

void triDeleteGJKObject(void* o);

/** GJK collision algorithm */
bool GJKCollide(void* obj1, ccd_support_fn supp1, ccd_center_fn cen1,
               void* obj2, ccd_support_fn supp2, ccd_center_fn cen2,
               Vec3f* contact_points, BVH_REAL* penetration_depth, Vec3f* normal);

void transformGJKObject(void* obj, const Vec3f R[3], const Vec3f& T);


/** collision algorithm between two shapes */
template<typename S1, typename S2>
bool shapeIntersect(const S1& s1, const S2& s2, Vec3f* contact_points = NULL, BVH_REAL* penetration_depth = NULL, Vec3f* normal = NULL)
{
  void* o1 = GJKInitializer<S1>::createGJKObject(s1);
  void* o2 = GJKInitializer<S2>::createGJKObject(s2);

  return GJKCollide(o1, GJKInitializer<S1>::getSupportFunction(), GJKInitializer<S1>::getCenterFunction(),
                    o2, GJKInitializer<S2>::getSupportFunction(), GJKInitializer<S2>::getCenterFunction(),
                    contact_points, penetration_depth, normal);

  GJKInitializer<S1>::deleteGJKObject(o1);
  GJKInitializer<S2>::deleteGJKObject(o2);
}

template<typename S1, typename S2>
bool shapeIntersect(const S1& s1, const S2& s2, const Vec3f R1[3], const Vec3f& T1, const Vec3f R2[3], const Vec3f& T2,
                    Vec3f* contact_points = NULL, BVH_REAL* penetration_depth = NULL, Vec3f* normal = NULL)
{
  void* o1 = GJKInitializer<S1>::createGJKObject(s1);
  void* o2 = GJKInitializer<S2>::createGJKObject(s2);

  transformGJKObject(o1, R1, T1);
  transformGJKObject(o2, R2, T2);

  return GJKCollide(o1, GJKInitializer<S1>::getSupportFunction(), GJKInitializer<S1>::getCenterFunction(),
                    o2, GJKInitializer<S2>::getSupportFunction(), GJKInitializer<S2>::getCenterFunction(),
                    contact_points, penetration_depth, normal);

  GJKInitializer<S1>::deleteGJKObject(o1);
  GJKInitializer<S2>::deleteGJKObject(o2);
}

template<typename S1, typename S2>
bool shapeIntersect(const S1& s1, const S2& s2, const Vec3f R[3], const Vec3f& T,
                    Vec3f* contact_points = NULL, BVH_REAL* penetration_depth = NULL, Vec3f* normal = NULL)
{
  void* o1 = GJKInitializer<S1>::createGJKObject(s1);
  void* o2 = GJKInitializer<S2>::createGJKObject(s2);

  transformGJKObject(o2, R, T);

  return GJKCollide(o1, GJKInitializer<S1>::getSupportFunction(), GJKInitializer<S1>::getCenterFunction(),
                    o2, GJKInitializer<S2>::getSupportFunction(), GJKInitializer<S2>::getCenterFunction(),
                    contact_points, penetration_depth, normal);

  GJKInitializer<S1>::deleteGJKObject(o1);
  GJKInitializer<S2>::deleteGJKObject(o2);
}


template<typename S>
bool shapeTriangleIntersect(const S& s, const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, Vec3f* contact_points = NULL, BVH_REAL* penetration_depth = NULL, Vec3f* normal = NULL)
{
  void* o1 = GJKInitializer<S>::createGJKObject(s);
  void* o2 = triCreateGJKObject(P1, P2, P3);

  return GJKCollide(o1, GJKInitializer<S>::getSupportFunction(), GJKInitializer<S>::getCenterFunction(),
                    o2, triGetSupportFunction(), triGetCenterFunction(),
                    contact_points, penetration_depth, normal);

  GJKInitializer<S>::deleteGJKObject(o1);
  triDeleteGJKObject(o2);
}


template<typename S>
bool shapeTriangleIntersect(const S& s, const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Vec3f R1[3], const Vec3f& T1, const Vec3f R2[3], const Vec3f& T2,
                            Vec3f* contact_points = NULL, BVH_REAL* penetration_depth = NULL, Vec3f* normal = NULL)
{
  void* o1 = GJKInitializer<S>::createGJKObject(s);
  void* o2 = triCreateGJKObject(P1, P2, P3, R2, T2);

  transformGJKObject(o1, R1, T1);

  return GJKCollide(o1, GJKInitializer<S>::getSupportFunction(), GJKInitializer<S>::getCenterFunction(),
                    o2, triGetSupportFunction(), triGetCenterFunction(),
                    contact_points, penetration_depth, normal);

  GJKInitializer<S>::deleteGJKObject(o1);
  triDeleteGJKObject(o2);
}


template<typename S>
bool shapeTriangleIntersect(const S& s, const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Vec3f R[3], const Vec3f& T,
                            Vec3f* contact_points = NULL, BVH_REAL* penetration_depth = NULL, Vec3f* normal = NULL)
{
  void* o1 = GJKInitializer<S>::createGJKObject(s);
  void* o2 = triCreateGJKObject(P1, P2, P3, R, T);

  return GJKCollide(o1, GJKInitializer<S>::getSupportFunction(), GJKInitializer<S>::getCenterFunction(),
                    o2, triGetSupportFunction(), triGetCenterFunction(),
                    contact_points, penetration_depth, normal);

  GJKInitializer<S>::deleteGJKObject(o1);
  triDeleteGJKObject(o2);
}

template<typename S>
bool shapeTriangleIntersect(const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const S& s, const Vec3f R[3], const Vec3f& T,
                            Vec3f* contact_points = NULL, BVH_REAL* penetration_depth = NULL, Vec3f* normal = NULL)
{
  void* o1 = triCreateGJKObject(P1, P2, P3);
  void* o2 = GJKInitializer<S>::createGJKObject(s);
  transformGJKObject(o2, R, T);


  return GJKCollide(o1, triGetSupportFunction(), triGetCenterFunction(),
                    o2, GJKInitializer<S>::getSupportFunction(), GJKInitializer<S>::getCenterFunction(),
                    contact_points, penetration_depth, normal);

  triDeleteGJKObject(o1);
  GJKInitializer<S>::deleteGJKObject(o2);
}



}

#endif

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


#ifndef FCL_GJK_LIBCCD_H
#define FCL_GJK_LIBCCD_H

#include "fcl/shape/geometric_shapes.h"
#include "fcl/math/transform.h"

#include <ccd/ccd.h>
#include <ccd/quat.h>

namespace fcl
{

namespace details
{

/// @brief callback function used by GJK algorithm
typedef void (*GJKSupportFunction)(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v);
typedef void (*GJKCenterFunction)(const void* obj, ccd_vec3_t* c);

/// @brief initialize GJK stuffs
template<typename T>
class GJKInitializer
{
public:
  /// @brief Get GJK support function
  static GJKSupportFunction getSupportFunction() { return NULL; }

  /// @brief Get GJK center function
  static GJKCenterFunction getCenterFunction() { return NULL; }

  /// @brief Get GJK object from a shape
  /// Notice that only local transformation is applied.
  /// Gloal transformation are considered later
  static void* createGJKObject(const T& /* s */, const Transform3f& /*tf*/) { return NULL; }

  /// @brief Delete GJK object
  static void deleteGJKObject(void* o) {}
};

/// @brief initialize GJK Cylinder
template<>
class GJKInitializer<Cylinder>
{
public:
  static GJKSupportFunction getSupportFunction();
  static GJKCenterFunction getCenterFunction();
  static void* createGJKObject(const Cylinder& s, const Transform3f& tf);
  static void deleteGJKObject(void* o);
};

/// @brief initialize GJK Sphere
template<>
class GJKInitializer<Sphere>
{
public:
  static GJKSupportFunction getSupportFunction();
  static GJKCenterFunction getCenterFunction();
  static void* createGJKObject(const Sphere& s, const Transform3f& tf);
  static void deleteGJKObject(void* o);
};

/// @brief initialize GJK Ellipsoid
template<>
class GJKInitializer<Ellipsoid>
{
public:
  static GJKSupportFunction getSupportFunction();
  static GJKCenterFunction getCenterFunction();
  static void* createGJKObject(const Ellipsoid& s, const Transform3f& tf);
  static void deleteGJKObject(void* o);
};

/// @brief initialize GJK Box
template<>
class GJKInitializer<Box>
{
public:
  static GJKSupportFunction getSupportFunction();
  static GJKCenterFunction getCenterFunction();
  static void* createGJKObject(const Box& s, const Transform3f& tf);
  static void deleteGJKObject(void* o);
};

/// @brief initialize GJK Capsule
template<>
class GJKInitializer<Capsule>
{
public:
  static GJKSupportFunction getSupportFunction();
  static GJKCenterFunction getCenterFunction();
  static void* createGJKObject(const Capsule& s, const Transform3f& tf);
  static void deleteGJKObject(void* o);
};

/// @brief initialize GJK Cone
template<>
class GJKInitializer<Cone>
{
public:
  static GJKSupportFunction getSupportFunction();
  static GJKCenterFunction getCenterFunction();
  static void* createGJKObject(const Cone& s, const Transform3f& tf);
  static void deleteGJKObject(void* o);
};

/// @brief initialize GJK Convex
template<>
class GJKInitializer<Convex>
{
public:
  static GJKSupportFunction getSupportFunction();
  static GJKCenterFunction getCenterFunction();
  static void* createGJKObject(const Convex& s, const Transform3f& tf);
  static void deleteGJKObject(void* o);
};

/// @brief initialize GJK Triangle
GJKSupportFunction triGetSupportFunction();

GJKCenterFunction triGetCenterFunction();

void* triCreateGJKObject(const Vec3f& P1, const Vec3f& P2, const Vec3f& P3);

void* triCreateGJKObject(const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Transform3f& tf);

void triDeleteGJKObject(void* o);

/// @brief GJK collision algorithm
bool GJKCollide(void* obj1, ccd_support_fn supp1, ccd_center_fn cen1,
                void* obj2, ccd_support_fn supp2, ccd_center_fn cen2,
                unsigned int max_iterations, FCL_REAL tolerance,
                Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal);

bool GJKDistance(void* obj1, ccd_support_fn supp1,
                 void* obj2, ccd_support_fn supp2,
                 unsigned int max_iterations, FCL_REAL tolerance,
                 FCL_REAL* dist, Vec3f* p1, Vec3f* p2);


} // details


}

#endif

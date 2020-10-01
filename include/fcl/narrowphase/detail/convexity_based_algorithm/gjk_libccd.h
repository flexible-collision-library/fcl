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

#ifndef FCL_NARROWPHASE_DETAIL_GJKLIBCCD_H
#define FCL_NARROWPHASE_DETAIL_GJKLIBCCD_H

#include <ccd/ccd.h>
#include <ccd/quat.h>
#include <ccd/vec3.h>

#include "fcl/common/unused.h"

#include "fcl/geometry/shape/box.h"
#include "fcl/geometry/shape/capsule.h"
#include "fcl/geometry/shape/cone.h"
#include "fcl/geometry/shape/convex.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/geometry/shape/ellipsoid.h"
#include "fcl/geometry/shape/halfspace.h"
#include "fcl/geometry/shape/plane.h"
#include "fcl/geometry/shape/sphere.h"
#include "fcl/geometry/shape/triangle_p.h"

#include "fcl/narrowphase/detail/convexity_based_algorithm/simplex.h"
#include "fcl/narrowphase/detail/convexity_based_algorithm/polytope.h"
#include "fcl/narrowphase/detail/convexity_based_algorithm/alloc.h"
#include "fcl/narrowphase/detail/convexity_based_algorithm/list.h"

namespace fcl
{

namespace detail
{

/// @brief callback function used by GJK algorithm

using GJKSupportFunction = void (*)(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v);
using GJKCenterFunction = void (*)(const void* obj, ccd_vec3_t* c);

/// @brief initialize GJK stuffs
template <typename S, typename T>
class FCL_EXPORT GJKInitializer
{
public:
  /// @brief Get GJK support function
  static GJKSupportFunction getSupportFunction() { return nullptr; }

  /// @brief Get GJK center function
  static GJKCenterFunction getCenterFunction() { return nullptr; }

  /// @brief Get GJK object from a shape
  /// Notice that only local transformation is applied.
  /// Gloal transformation are considered later
  static void* createGJKObject(const T& /* s */, const Transform3<S>& /*tf*/) { return nullptr; }

  /// @brief Delete GJK object
  static void deleteGJKObject(void* o) { FCL_UNUSED(o); }
};

/// @brief initialize GJK Cylinder<S>
template <typename S>
class FCL_EXPORT GJKInitializer<S, Cylinder<S>>
{
public:
  static GJKSupportFunction getSupportFunction();
  static GJKCenterFunction getCenterFunction();
  static void* createGJKObject(const Cylinder<S>& s, const Transform3<S>& tf);
  static void deleteGJKObject(void* o);
};

/// @brief initialize GJK Sphere<S>
template <typename S>
class FCL_EXPORT GJKInitializer<S, Sphere<S>>
{
public:
  static GJKSupportFunction getSupportFunction();
  static GJKCenterFunction getCenterFunction();
  static void* createGJKObject(const Sphere<S>& s, const Transform3<S>& tf);
  static void deleteGJKObject(void* o);
};

/// @brief initialize GJK Ellipsoid<S>
template <typename S>
class FCL_EXPORT GJKInitializer<S, Ellipsoid<S>>
{
public:
  static GJKSupportFunction getSupportFunction();
  static GJKCenterFunction getCenterFunction();
  static void* createGJKObject(const Ellipsoid<S>& s, const Transform3<S>& tf);
  static void deleteGJKObject(void* o);
};

/// @brief initialize GJK Box<S>
template <typename S>
class FCL_EXPORT GJKInitializer<S, Box<S>>
{
public:
  static GJKSupportFunction getSupportFunction();
  static GJKCenterFunction getCenterFunction();
  static void* createGJKObject(const Box<S>& s, const Transform3<S>& tf);
  static void deleteGJKObject(void* o);
};

/// @brief initialize GJK Capsule<S>
template <typename S>
class FCL_EXPORT GJKInitializer<S, Capsule<S>>
{
public:
  static GJKSupportFunction getSupportFunction();
  static GJKCenterFunction getCenterFunction();
  static void* createGJKObject(const Capsule<S>& s, const Transform3<S>& tf);
  static void deleteGJKObject(void* o);
};

/// @brief initialize GJK Cone<S>
template <typename S>
class FCL_EXPORT GJKInitializer<S, Cone<S>>
{
public:
  static GJKSupportFunction getSupportFunction();
  static GJKCenterFunction getCenterFunction();
  static void* createGJKObject(const Cone<S>& s, const Transform3<S>& tf);
  static void deleteGJKObject(void* o);
};

/// @brief initialize GJK Convex<S>
template <typename S>
class FCL_EXPORT GJKInitializer<S, Convex<S>>
{
public:
  static GJKSupportFunction getSupportFunction();
  static GJKCenterFunction getCenterFunction();
  static void* createGJKObject(const Convex<S>& s, const Transform3<S>& tf);
  static void deleteGJKObject(void* o);
};

/// @brief initialize GJK Triangle
FCL_EXPORT
GJKSupportFunction triGetSupportFunction();

FCL_EXPORT
GJKCenterFunction triGetCenterFunction();

template <typename S>
FCL_EXPORT
void* triCreateGJKObject(const Vector3<S>& P1, const Vector3<S>& P2, const Vector3<S>& P3);

template <typename S>
FCL_EXPORT
void* triCreateGJKObject(const Vector3<S>& P1, const Vector3<S>& P2, const Vector3<S>& P3, const Transform3<S>& tf);

FCL_EXPORT
void triDeleteGJKObject(void* o);

/// @brief GJK collision algorithm
template <typename S>
FCL_EXPORT
bool GJKCollide(
    void* obj1,
    ccd_support_fn supp1,
    ccd_center_fn cen1,
    void* obj2,
    ccd_support_fn supp2,
    ccd_center_fn cen2,
    unsigned int max_iterations,
    S tolerance,
    Vector3<S>* contact_points,
    S* penetration_depth,
    Vector3<S>* normal);

/** Compute the distance between two objects using GJK algorithm.
 * @param[in] obj1 A convex geometric object.
 * @param[in] supp1 A function to compute the support of obj1 along some
 * direction.
 * @param[in] obj2 A convex geometric object.
 * @param[in] supp2 A function to compute the support of obj2 along some
 * direction.
 * @param max_iterations The maximal iterations before the GJK algorithm
 * terminates.
 * @param[in] tolerance The tolerance used in GJK. When the change of distance
 * is smaller than this tolerance, the algorithm terminates.
 * @param[out] dist The distance between the objects. When the two objects are
 * not colliding, this is the actual distance, a positive number. When the two
 * objects are colliding, it is -1.
 * @param[out] p1 The closest point on object 1 in the world frame.
 * @param[out] p2 The closest point on object 2 in the world frame.
 * @retval is_separated True if the objects are separated, false otherwise.
 */
template <typename S>
FCL_EXPORT
bool GJKDistance(void* obj1, ccd_support_fn supp1,
                 void* obj2, ccd_support_fn supp2,
                 unsigned int max_iterations, S tolerance,
                 S* dist, Vector3<S>* p1, Vector3<S>* p2);

/** Compute the signed distance between two objects using GJK and EPA algorithm.
 * @param[in] obj1 A convex geometric object.
 * @param[in] supp1 A function to compute the support of obj1 along some
 * direction.
 * @param[in] obj2 A convex geometric object.
 * @param[in] supp2 A function to compute the support of obj2 along some
 * direction.
 * @param[in] max_iterations The maximal iterations before the GJK algorithm
 * terminates.
 * @param[in] tolerance The tolerance used in GJK. When the change of distance
 * is smaller than this tolerance, the algorithm terminates.
 * @param[out] dist The distance between the objects. When the two objects are
 * not colliding, this is the actual distance, a positive number. When the two
 * objects are colliding, it is the negation of the penetration depth, a
 * negative value.
 * @param[out] p1 The closest point on object 1 in the world frame.
 * @param[out] p2 The closest point on object 2 in the world frame.
 * @retval is_separated True if the objects are separated, false otherwise.
 */
template <typename S>
FCL_EXPORT
bool GJKSignedDistance(void* obj1, ccd_support_fn supp1,
                       void* obj2, ccd_support_fn supp2,
                       unsigned int max_iterations, S tolerance,
                       S* dist, Vector3<S>* p1, Vector3<S>* p2);



} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/convexity_based_algorithm/gjk_libccd-inl.h"

#endif

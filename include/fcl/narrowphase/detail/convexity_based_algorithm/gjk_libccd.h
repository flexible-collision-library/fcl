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

#include "fcl/narrowphase/detail/convexity_based_algorithm/minkowski_diff.h"

namespace fcl
{

namespace detail
{

/// @brief GJK collision algorithm
template <typename S>
FCL_EXPORT
bool GJKCollide(const MinkowskiDiff<S>& shape, unsigned int max_iterations, S tolerance,
    Vector3<S>* contact_points, S* penetration_depth, Vector3<S>* normal);

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
bool GJKDistance(const MinkowskiDiff<S>& shape, unsigned int max_iterations, S tolerance,
                 S* dist, Vector3<S>* p1, Vector3<S>* p2);

/*
template <typename S>
FCL_EXPORT
bool GJKDistanceS(const MinkowskiDiff<S>& shape, unsigned int max_iterations, S tolerance,
                 S* dist, Vector3<S>* p1, Vector3<S>* p2);
*/

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
bool GJKSignedDistance(const MinkowskiDiff<S>& shape, unsigned int max_iterations, S tolerance,
                       S* dist, Vector3<S>* p1, Vector3<S>* p2);



} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/convexity_based_algorithm/gjk_libccd-inl.h"

#endif

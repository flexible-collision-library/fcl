/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Toyota Research Institute.
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

/** @author Sean Curtis <sean@tri.global> (2018) */

#ifndef FCL_NARROWPHASE_DETAIL_SPHEREBOX_H
#define FCL_NARROWPHASE_DETAIL_SPHEREBOX_H

#include "fcl/geometry/shape/box.h"
#include "fcl/geometry/shape/sphere.h"
#include "fcl/narrowphase/contact_point.h"

namespace fcl {

namespace detail {

/** @name       Custom box-sphere proximity algorithms

 These functions provide custom algorithms for analyzing the relationship
  between a sphere and a box.

 These functions make use of the
 [Drake monogram
 notation](http://drake.mit.edu/doxygen_cxx/group__multibody__notation__basics.html)
 to describe quantities (particularly the poses of shapes).

 Both shapes must be posed in a common frame (notated as F). This common frame
 is typically the world frame W. Regardless, if the optional output data is
 returned, it will be reported in this common frame F.

 The functions can be executed in one of two ways: to perform a strict boolean
 query (is colliding/is separated) or to get data which characterizes the
 colliding or separating state (e.g., penetration depth vs separating distance).
 The difference in usage is defined by whether the optional out parameters
 are null or non-null. In the documentation, these are labeled "(optional)".

 For these functions, if the sphere and box are detected to be *touching* this
 is considered a collision. As such, a collision query would report true and
 a separating distance query would report false.
 */

// NOTE: the choice to consider touching contact as collision is predicated on
// the implementation in sphere-sphere contact.

//@{

// NOTE: The handling of the discontinuities in normal and position was
// implicitly defined in the spherebox test in test_fcl_geometric_shapes.cpp. It
// provided backwards compatibility to that test.

/** Detect collision between the sphere and box. If colliding, return
 characterization of collision in the provided vector.

 The reported depth is guaranteed to be continuous with respect to the relative
 pose of the two objects. The normal and contact position are guaranteed to be
 continuous while the sphere center lies *outside* the box. However, if the
 sphere center lies *inside* the box, there are regions of discontinuity in both
 normal and contact position. This is due to the fact that there is not
 necessarily a *unique* characterization of penetration depth (i.e., the sphere
 center may be equidistant to multiple faces). In this case, the faces are
 arbitrarily prioritized and the face with the highest priority is used to
 compute normal and position. The priority order is +x, -x, +y, -y, +z, and -z.
 For example:

   - If the center is near the edge between the -y and +z faces, the normal will
     be defined w.r.t. to the -y face.
   - If the center is near the corner of the +x, +y, & +z faces, the +x face
     will be used.

 @param sphere         The sphere geometry.
 @param X_FS           The pose of the sphere S in the common frame F.
 @param box            The box geometry.
 @param X_FB           The pose of the box B in the common frame F.
 @param contacts[out]  (optional) If the shapes collide, the contact point data
                       will be appended to the end of this vector.
 @return True if the objects are colliding (including touching).
 @tparam S The scalar parameter (must be a valid Eigen scalar).  */
template <typename S>
FCL_EXPORT bool sphereBoxIntersect(const Sphere<S>& sphere,
                                   const Transform3<S>& X_FS, const Box<S>& box,
                                   const Transform3<S>& X_FB,
                                   std::vector<ContactPoint<S>>* contacts);

/** Evaluate the minimum separating distance between a sphere and box. If
 separated, the nearest points on each shape will be returned in frame F.
 @param sphere         The sphere geometry.
 @param X_FS           The pose of the sphere S in the common frame F.
 @param box            The box geometry.
 @param X_FB           The pose of the box B in the common frame F.
 @param distance[out]  (optional) The separating distance between the box
                       and sphere. Set to -1 if the shapes are penetrating.
 @param p_FSb[out]     (optional) The closest point on the *sphere* to the box
                       measured and expressed in frame F.
 @param p_FBs[out]     (optional) The closest point on the *box* to the sphere
                       measured and expressed in frame F.
 @return True if the objects are separated.
 @tparam S The scalar parameter (must be a valid Eigen scalar).  */
template <typename S>
FCL_EXPORT bool sphereBoxDistance(const Sphere<S>& sphere,
                                  const Transform3<S>& X_FS, const Box<S>& box,
                                  const Transform3<S>& X_FB, S* distance,
                                  Vector3<S>* p_FSb, Vector3<S>* p_FBs);

//@}

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/primitive_shape_algorithm/sphere_box-inl.h"

#endif // FCL_NARROWPHASE_DETAIL_SPHEREBOX_H

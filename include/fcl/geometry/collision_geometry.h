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

#ifndef FCL_COLLISION_GEOMETRY_H
#define FCL_COLLISION_GEOMETRY_H

#include <memory>

#include "fcl/math/bv/AABB.h"
#include "fcl/math/motion/motion_base.h"

namespace fcl
{

/// @brief object type: BVH (mesh, points), basic geometry, octree
enum OBJECT_TYPE {OT_UNKNOWN, OT_BVH, OT_GEOM, OT_OCTREE, OT_COUNT};

/// @brief traversal node type: bounding volume (AABB, OBB, RSS, kIOS, OBBRSS, KDOP16, KDOP18, kDOP24), basic shape (box, sphere, ellipsoid, capsule, cone, cylinder, convex, plane, halfspace, triangle), and octree
enum NODE_TYPE {BV_UNKNOWN, BV_AABB, BV_OBB, BV_RSS, BV_kIOS, BV_OBBRSS, BV_KDOP16, BV_KDOP18, BV_KDOP24,
                GEOM_BOX, GEOM_SPHERE, GEOM_ELLIPSOID, GEOM_CAPSULE, GEOM_CONE, GEOM_CYLINDER, GEOM_CONVEX, GEOM_PLANE, GEOM_HALFSPACE, GEOM_TRIANGLE, GEOM_OCTREE, NODE_COUNT};

/// @brief The geometry for the object for collision or distance computation
template <typename S>
class FCL_EXPORT CollisionGeometry
{
public:
  CollisionGeometry();

  virtual ~CollisionGeometry();

  /// @brief get the type of the object
  virtual OBJECT_TYPE getObjectType() const;

  /// @brief get the node type
  virtual NODE_TYPE getNodeType() const;

  /// @brief compute the AABB for object in local coordinate
  virtual void computeLocalAABB() = 0;

  /// @brief get user data in geometry
  void* getUserData() const;

  /// @brief set user data in geometry
  void setUserData(void *data);

  /// @brief whether the object is completely occupied
  bool isOccupied() const;

  /// @brief whether the object is completely free
  bool isFree() const;

  /// @brief whether the object has some uncertainty
  bool isUncertain() const;

  /// @brief AABB center in local coordinate
  Vector3<S> aabb_center;

  /// @brief AABB radius
  S aabb_radius;

  /// @brief AABB in local coordinate, used for tight AABB when only translation transform
  AABB<S> aabb_local;

  /// @brief pointer to user defined data specific to this object
  void* user_data;

  /// @brief collision cost for unit volume
  S cost_density;

  /// @brief threshold for occupied ( >= is occupied)
  S threshold_occupied;

  /// @brief threshold for free (<= is free)
  S threshold_free;

  /// @brief compute center of mass
  virtual Vector3<S> computeCOM() const;

  /// @brief compute the inertia matrix, related to the origin
  virtual Matrix3<S> computeMomentofInertia() const;

  /// @brief compute the volume
  virtual S computeVolume() const;

  /// @brief compute the inertia matrix, related to the com
  virtual Matrix3<S> computeMomentofInertiaRelatedToCOM() const;

};

using CollisionGeometryf = CollisionGeometry<float>;
using CollisionGeometryd = CollisionGeometry<double>;

} // namespace fcl

#include "fcl/geometry/collision_geometry-inl.h"

#endif

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

#ifndef FCL_CONTINUOUS_COLLISION_OBJECT_H
#define FCL_CONTINUOUS_COLLISION_OBJECT_H

#include <memory>

#include "fcl/geometry/collision_geometry.h"

namespace fcl
{

/// @brief the object for continuous collision or distance computation, contains
/// the geometry and the motion information
template <typename S>
class FCL_EXPORT ContinuousCollisionObject
{
public:
  ContinuousCollisionObject(
      const std::shared_ptr<CollisionGeometry<S>>& cgeom);

  ContinuousCollisionObject(
      const std::shared_ptr<CollisionGeometry<S>>& cgeom,
      const std::shared_ptr<MotionBase<S>>& motion);

  ~ContinuousCollisionObject();

  /// @brief get the type of the object
  OBJECT_TYPE getObjectType() const;

  /// @brief get the node type
  NODE_TYPE getNodeType() const;

  /// @brief get the AABB<S> in the world space for the motion
  const AABB<S>& getAABB() const;

  /// @brief compute the AABB<S> in the world space for the motion
  void computeAABB();

  /// @brief get user data in object
  void* getUserData() const;

  /// @brief set user data in object
  void setUserData(void* data);

  /// @brief get motion from the object instance
  MotionBase<S>* getMotion() const;

  /// @brief get geometry from the object instance
  FCL_DEPRECATED
  const CollisionGeometry<S>* getCollisionGeometry() const;

  /// @brief get geometry from the object instance
  const std::shared_ptr<const CollisionGeometry<S>>& collisionGeometry() const;

protected:

  std::shared_ptr<CollisionGeometry<S>> cgeom;
  std::shared_ptr<const CollisionGeometry<S>> cgeom_const;

  std::shared_ptr<MotionBase<S>> motion;

  /// @brief AABB<S> in the global coordinate for the motion
  mutable AABB<S> aabb;

  /// @brief pointer to user defined data specific to this object
  void* user_data;
};

using ContinuousCollisionObjectf = ContinuousCollisionObject<float>;
using ContinuousCollisionObjectd = ContinuousCollisionObject<double>;

} // namespace fcl

#include "fcl/narrowphase/continuous_collision_object-inl.h"

#endif

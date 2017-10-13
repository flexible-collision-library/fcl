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

#ifndef FCL_COLLISION_OBJECT_H
#define FCL_COLLISION_OBJECT_H

#include <memory>

#include "fcl/geometry/collision_geometry.h"

namespace fcl
{

/// @brief the object for collision or distance computation, contains the
/// geometry and the transform information
template <typename S>
class FCL_EXPORT CollisionObject
{
public:
  CollisionObject(const std::shared_ptr<CollisionGeometry<S>>& cgeom);

  CollisionObject(const std::shared_ptr<CollisionGeometry<S>>& cgeom,
                  const Transform3<S>& tf);

  CollisionObject(const std::shared_ptr<CollisionGeometry<S>>& cgeom,
                  const Matrix3<S>& R,
                  const Vector3<S>& T);

  ~CollisionObject();

  /// @brief get the type of the object
  OBJECT_TYPE getObjectType() const;

  /// @brief get the node type
  NODE_TYPE getNodeType() const;

  /// @brief get the AABB in world space
  const AABB<S>& getAABB() const;

  /// @brief compute the AABB in world space
  void computeAABB();

  /// @brief get user data in object
  void* getUserData() const;

  /// @brief set user data in object
  void setUserData(void *data);

  /// @brief get translation of the object
  const Vector3<S> getTranslation() const;

  /// @brief get matrix rotation of the object
  const Matrix3<S> getRotation() const;

  /// @brief get quaternion rotation of the object
  const Quaternion<S> getQuatRotation() const;

  /// @brief get object's transform
  const Transform3<S>& getTransform() const;

  /// @brief set object's rotation matrix
  void setRotation(const Matrix3<S>& R);

  /// @brief set object's translation
  void setTranslation(const Vector3<S>& T);

  /// @brief set object's quatenrion rotation
  void setQuatRotation(const Quaternion<S>& q);

  /// @brief set object's transform
  void setTransform(const Matrix3<S>& R, const Vector3<S>& T);

  /// @brief set object's transform
  void setTransform(const Quaternion<S>& q, const Vector3<S>& T);

  /// @brief set object's transform
  void setTransform(const Transform3<S>& tf);

  /// @brief whether the object is in local coordinate
  bool isIdentityTransform() const;

  /// @brief set the object in local coordinate
  void setIdentityTransform();

  /// @brief get geometry from the object instance
  FCL_DEPRECATED
  const CollisionGeometry<S>* getCollisionGeometry() const;

  /// @brief get geometry from the object instance
  const std::shared_ptr<const CollisionGeometry<S>>& collisionGeometry() const;

  /// @brief get object's cost density
  S getCostDensity() const;

  /// @brief set object's cost density
  void setCostDensity(S c);

  /// @brief whether the object is completely occupied
  bool isOccupied() const;

  /// @brief whether the object is completely free
  bool isFree() const;

  /// @brief whether the object is uncertain
  bool isUncertain() const;

protected:

  std::shared_ptr<CollisionGeometry<S>> cgeom;
  std::shared_ptr<const CollisionGeometry<S>> cgeom_const;

  Transform3<S> t;

  /// @brief AABB<S> in global coordinate
  mutable AABB<S> aabb;

  /// @brief pointer to user defined data specific to this object
  void *user_data;

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using CollisionObjectf = CollisionObject<float>;
using CollisionObjectd = CollisionObject<double>;

} // namespace fcl

#include "fcl/narrowphase/collision_object-inl.h"

#endif

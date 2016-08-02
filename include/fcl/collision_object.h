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


#ifndef FCL_COLLISION_OBJECT_H
#define FCL_COLLISION_OBJECT_H

#include <memory>

#include "fcl/collision_geometry.h"

namespace fcl
{

/// @brief the object for collision or distance computation, contains the geometry and the transform information
class CollisionObject
{
public:
 CollisionObject(const std::shared_ptr<CollisionGeometryd> &cgeom_) :
    cgeom(cgeom_), cgeom_const(cgeom_), t(Transform3d::Identity())
  {
    if (cgeom)
    {
      cgeom->computeLocalAABB();
      computeAABB();
    }
  }

  CollisionObject(const std::shared_ptr<CollisionGeometryd> &cgeom_, const Transform3d& tf) :
    cgeom(cgeom_), cgeom_const(cgeom_), t(tf)
  {
    cgeom->computeLocalAABB();
    computeAABB();
  }

  CollisionObject(const std::shared_ptr<CollisionGeometryd> &cgeom_, const Matrix3d& R, const Vector3d& T):
      cgeom(cgeom_), cgeom_const(cgeom_), t(Transform3d::Identity())
  {
    t.linear() = R;
    t.translation() = T;
    cgeom->computeLocalAABB();
    computeAABB();
  }

  ~CollisionObject()
  {
  }

  /// @brief get the type of the object
  OBJECT_TYPE getObjectType() const
  {
    return cgeom->getObjectType();
  }

  /// @brief get the node type
  NODE_TYPE getNodeType() const
  {
    return cgeom->getNodeType();
  }

  /// @brief get the AABBd in world space
  inline const AABBd& getAABB() const
  {
    return aabb;
  }

  /// @brief compute the AABBd in world space
  inline void computeAABB()
  {
    if(t.linear().isIdentity())
    {
      aabb = translate(cgeom->aabb_local, t.translation());
    }
    else
    {
      Vector3d center = t * cgeom->aabb_center;
      Vector3d delta = Vector3d::Constant(cgeom->aabb_radius);
      aabb.min_ = center - delta;
      aabb.max_ = center + delta;
    }
  }

  /// @brief get user data in object
  void* getUserData() const
  {
    return user_data;
  }

  /// @brief set user data in object
  void setUserData(void *data)
  {
    user_data = data;
  }

  /// @brief get translation of the object
  inline const Vector3d getTranslation() const
  {
    return t.translation();
  }

  /// @brief get matrix rotation of the object
  inline const Matrix3d getRotation() const
  {
    return t.linear();
  }

  /// @brief get quaternion rotation of the object
  inline const Quaternion3d getQuatRotation() const
  {
    return Quaternion3d(t.linear());
  }

  /// @brief get object's transform
  inline const Transform3d& getTransform() const
  {
    return t;
  }

  /// @brief set object's rotation matrix
  void setRotation(const Matrix3d& R)
  {
    t.linear() = R;
  }

  /// @brief set object's translation
  void setTranslation(const Vector3d& T)
  {
    t.translation() = T;
  }

  /// @brief set object's quatenrion rotation
  void setQuatRotation(const Quaternion3d& q)
  {
    t.linear() = q.toRotationMatrix();
  }

  /// @brief set object's transform
  void setTransform(const Matrix3d& R, const Vector3d& T)
  {
    setRotation(R);
    setTranslation(T);
  }

  /// @brief set object's transform
  void setTransform(const Quaternion3d& q, const Vector3d& T)
  {
    setQuatRotation(q);
    setTranslation(T);
  }

  /// @brief set object's transform
  void setTransform(const Transform3d& tf)
  {
    t = tf;
  }

  /// @brief whether the object is in local coordinate
  bool isIdentityTransform() const
  {
    return (t.linear().isIdentity() && t.translation().isZero());
  }

  /// @brief set the object in local coordinate
  void setIdentityTransform()
  {
    t.setIdentity();
  }

  /// @brief get geometry from the object instance
  FCL_DEPRECATED
  const CollisionGeometryd* getCollisionGeometryd() const
  {
    return cgeom.get();
  }

  /// @brief get geometry from the object instance
  const std::shared_ptr<const CollisionGeometryd>& collisionGeometry() const
  {
    return cgeom_const;
  }

  /// @brief get object's cost density
  FCL_REAL getCostDensity() const
  {
    return cgeom->cost_density;
  }

  /// @brief set object's cost density
  void setCostDensity(FCL_REAL c)
  {
    cgeom->cost_density = c;
  }

  /// @brief whether the object is completely occupied
  inline bool isOccupied() const
  {
    return cgeom->isOccupied();
  }

  /// @brief whether the object is completely free
  inline bool isFree() const
  {
    return cgeom->isFree();
  }

  /// @brief whether the object is uncertain
  inline bool isUncertain() const
  {
    return cgeom->isUncertain();
  }

protected:

  std::shared_ptr<CollisionGeometryd> cgeom;
  std::shared_ptr<const CollisionGeometryd> cgeom_const;

  Transform3d t;

  /// @brief AABBd in global coordinate
  mutable AABBd aabb;

  /// @brief pointer to user defined data specific to this object
  void *user_data;
};

}

#endif

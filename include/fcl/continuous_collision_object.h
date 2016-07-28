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

#ifndef FCL_CONTINUOUS_COLLISION_OBJECT_H
#define FCL_CONTINUOUS_COLLISION_OBJECT_H

#include <memory>

#include "fcl/collision_geometry.h"

namespace fcl
{

/// @brief the object for continuous collision or distance computation, contains the geometry and the motion information
class ContinuousCollisionObject
{
public:
  ContinuousCollisionObject(const std::shared_ptr<CollisionGeometryd>& cgeom_) :
    cgeom(cgeom_), cgeom_const(cgeom_)
  {
  }

  ContinuousCollisionObject(const std::shared_ptr<CollisionGeometryd>& cgeom_, const std::shared_ptr<MotionBase>& motion_) :
    cgeom(cgeom_), cgeom_const(cgeom), motion(motion_)
  {
  }

  ~ContinuousCollisionObject() {}

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

  /// @brief get the AABB in the world space for the motion
  inline const AABB& getAABB() const
  {
    return aabb;
  }

  /// @brief compute the AABB in the world space for the motion
  inline void computeAABB()
  {
    IVector3 box;
    TMatrix3 R;
    TVector3 T;
    motion->getTaylorModel(R, T);

    Vector3d p = cgeom->aabb_local.min_;
    box = (R * p + T).getTightBound();

    p[2] = cgeom->aabb_local.max_[2];
    box = bound(box, (R * p + T).getTightBound());

    p[1] = cgeom->aabb_local.max_[1];
    p[2] = cgeom->aabb_local.min_[2];
    box = bound(box, (R * p + T).getTightBound());

    p[2] = cgeom->aabb_local.max_[2];
    box = bound(box, (R * p + T).getTightBound());

    p[0] = cgeom->aabb_local.max_[0];
    p[1] = cgeom->aabb_local.min_[1];
    p[2] = cgeom->aabb_local.min_[2];
    box = bound(box, (R * p + T).getTightBound());

    p[2] = cgeom->aabb_local.max_[2];
    box = bound(box, (R * p + T).getTightBound());

    p[1] = cgeom->aabb_local.max_[1];
    p[2] = cgeom->aabb_local.min_[2];
    box = bound(box, (R * p + T).getTightBound());

    p[2] = cgeom->aabb_local.max_[2];
    box = bound(box, (R * p + T).getTightBound());

    aabb.min_ = box.getLow();
    aabb.max_ = box.getHigh();
  }

  /// @brief get user data in object
  void* getUserData() const
  {
    return user_data;
  }

  /// @brief set user data in object
  void setUserData(void* data)
  {
    user_data = data;
  }

  /// @brief get motion from the object instance
  inline MotionBase* getMotion() const
  {
    return motion.get();
  }

  /// @brief get geometry from the object instance
  FCL_DEPRECATED
  inline const CollisionGeometryd* getCollisionGeometryd() const
  {
    return cgeom.get();
  }

  /// @brief get geometry from the object instance
  inline const std::shared_ptr<const CollisionGeometryd>& collisionGeometry() const
  {
    return cgeom_const;
  }

protected:

  std::shared_ptr<CollisionGeometryd> cgeom;
  std::shared_ptr<const CollisionGeometryd> cgeom_const;

  std::shared_ptr<MotionBase> motion;

  /// @brief AABB in the global coordinate for the motion
  mutable AABB aabb;

  /// @brief pointer to user defined data specific to this object
  void* user_data;
};

}

#endif

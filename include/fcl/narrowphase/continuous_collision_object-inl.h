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

#ifndef FCL_CONTINUOUS_COLLISION_OBJECT_INL_H
#define FCL_CONTINUOUS_COLLISION_OBJECT_INL_H

#include "fcl/narrowphase/continuous_collision_object.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT ContinuousCollisionObject<double>;

//==============================================================================
template <typename S>
ContinuousCollisionObject<S>::ContinuousCollisionObject(
    const std::shared_ptr<CollisionGeometry<S>>& cgeom_)
  : cgeom(cgeom_), cgeom_const(cgeom_)
{
  // Do nothing
}

//==============================================================================
template <typename S>
ContinuousCollisionObject<S>::ContinuousCollisionObject(
    const std::shared_ptr<CollisionGeometry<S>>& cgeom_,
    const std::shared_ptr<MotionBase<S>>& motion_)
  : cgeom(cgeom_), cgeom_const(cgeom), motion(motion_)
{
  // Do nothing
}

//==============================================================================
template <typename S>
ContinuousCollisionObject<S>::~ContinuousCollisionObject()
{
  // Do nothing
}

//==============================================================================
template <typename S>
OBJECT_TYPE ContinuousCollisionObject<S>::getObjectType() const
{
  return cgeom->getObjectType();
}

//==============================================================================
template <typename S>
NODE_TYPE ContinuousCollisionObject<S>::getNodeType() const
{
  return cgeom->getNodeType();
}

//==============================================================================
template <typename S>
const AABB<S>&ContinuousCollisionObject<S>::getAABB() const
{
  return aabb;
}

//==============================================================================
template <typename S>
void ContinuousCollisionObject<S>::computeAABB()
{
  IVector3<S> box;
  TMatrix3<S> R;
  TVector3<S> T;
  motion->getTaylorModel(R, T);

  Vector3<S> p = cgeom->aabb_local.min_;
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

//==============================================================================
template <typename S>
void* ContinuousCollisionObject<S>::getUserData() const
{
  return user_data;
}

//==============================================================================
template <typename S>
void ContinuousCollisionObject<S>::setUserData(void* data)
{
  user_data = data;
}

//==============================================================================
template <typename S>
MotionBase<S>* ContinuousCollisionObject<S>::getMotion() const
{
  return motion.get();
}

//==============================================================================
template <typename S>
const CollisionGeometry<S>*
ContinuousCollisionObject<S>::getCollisionGeometry() const
{
  return cgeom.get();
}

//==============================================================================
template <typename S>
const std::shared_ptr<const CollisionGeometry<S>>&
ContinuousCollisionObject<S>::collisionGeometry() const
{
  return cgeom_const;
}

} // namespace fcl

#endif

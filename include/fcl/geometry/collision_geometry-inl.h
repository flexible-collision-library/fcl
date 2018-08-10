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

#ifndef FCL_COLLISION_GEOMETRY_INL_H
#define FCL_COLLISION_GEOMETRY_INL_H

#include "fcl/geometry/collision_geometry.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT CollisionGeometry<double>;

//==============================================================================
template <typename S>
CollisionGeometry<S>::CollisionGeometry()
  : aabb_center(Vector3<S>::Zero()),
    aabb_radius((S)0),
    user_data(nullptr),
    cost_density((S)1),
    threshold_occupied((S)1),
    threshold_free((S)0)
{
  // Do nothing
}

//==============================================================================
template <typename S>
CollisionGeometry<S>::~CollisionGeometry()
{
  // Do nothing
}

//==============================================================================
template <typename S>
OBJECT_TYPE CollisionGeometry<S>::getObjectType() const
{
  return OT_UNKNOWN;
}

//==============================================================================
template <typename S>
NODE_TYPE CollisionGeometry<S>::getNodeType() const
{
  return BV_UNKNOWN;
}

//==============================================================================
template <typename S>
void* CollisionGeometry<S>::getUserData() const
{
  return user_data;
}

//==============================================================================
template <typename S>
void CollisionGeometry<S>::setUserData(void* data)
{
  user_data = data;
}

//==============================================================================
template <typename S>
bool CollisionGeometry<S>::isOccupied() const
{
  return cost_density >= threshold_occupied;
}

//==============================================================================
template <typename S>
bool CollisionGeometry<S>::isFree() const
{
  return cost_density <= threshold_free;
}

//==============================================================================
template <typename S>
bool CollisionGeometry<S>::isUncertain() const
{
  return !isOccupied() && !isFree();
}

//==============================================================================
template <typename S>
Vector3<S> CollisionGeometry<S>::computeCOM() const
{
  return Vector3<S>::Zero();
}

//==============================================================================
template <typename S>
Matrix3<S> CollisionGeometry<S>::computeMomentofInertia() const
{
  return Matrix3<S>::Zero();
}

//==============================================================================
template <typename S>
S CollisionGeometry<S>::computeVolume() const
{
  return 0;
}

//==============================================================================
template <typename S>
Matrix3<S> CollisionGeometry<S>::computeMomentofInertiaRelatedToCOM() const
{
  // TODO(SeanCurtis-TRI): This is *horribly* inefficient. In complex cases,
  // this will require computing volume integrals three times. The
  // CollisionGeometry class should have a single virtual function that will
  // return all three quantities in one call so that particular sub-classes can
  // override this to process this answer more efficiently. The default
  // implementation can be exactly these three calls.
  // See: https://github.com/flexible-collision-library/fcl/issues/327.
  Matrix3<S> C = computeMomentofInertia();
  Vector3<S> com = computeCOM();
  S V = computeVolume();

  Matrix3<S> m;
  m << C(0, 0) - V * (com[1] * com[1] + com[2] * com[2]),
      C(0, 1) + V * com[0] * com[1],
      C(0, 2) + V * com[0] * com[2],
      C(1, 0) + V * com[1] * com[0],
      C(1, 1) - V * (com[0] * com[0] + com[2] * com[2]),
      C(1, 2) + V * com[1] * com[2],
      C(2, 0) + V * com[2] * com[0],
      C(2, 1) + V * com[2] * com[1],
      C(2, 2) - V * (com[0] * com[0] + com[1] * com[1]);

  return m;
}

} // namespace fcl

#endif

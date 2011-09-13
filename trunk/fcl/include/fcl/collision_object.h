/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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


#ifndef FCL_COLLISION_OBJECT_BASE_H
#define FCL_COLLISION_OBJECT_BASE_H

#include "fcl/AABB.h"
#include "fcl/transform.h"

namespace fcl
{

enum OBJECT_TYPE {OT_UNKNOWN, OT_BVH, OT_GEOM};

enum NODE_TYPE {BV_UNKNOWN, BV_AABB, BV_OBB, BV_RSS, BV_KDOP16, BV_KDOP18, BV_KDOP24,
                GEOM_BOX, GEOM_SPHERE, GEOM_CAPSULE, GEOM_CONE, GEOM_CYLINDER, GEOM_CONVEX, GEOM_PLANE};

/** \brief Base class for all objects participating in collision */
class CollisionObject
{
public:
  virtual ~CollisionObject() {}

  virtual OBJECT_TYPE getObjectType() const { return OT_UNKNOWN; }

  virtual NODE_TYPE getNodeType() const { return BV_UNKNOWN; }

  virtual void computeLocalAABB() = 0;

  inline const AABB& getAABB() const
  {
    return aabb;
  }

  inline void computeAABB()
  {
    Vec3f center = t.transform(aabb_center);
    Vec3f delta(aabb_radius, aabb_radius, aabb_radius);
    aabb.min_ = center - delta;
    aabb.max_ = center + delta;
  }

  inline const Vec3f& getTranslation() const
  {
    return t.getTranslation();
  }

  inline const Vec3f* getRotation() const
  {
    return t.getRotation();
  }

  inline const SimpleQuaternion& getQuatRotation() const
  {
    return t.getQuatRotation();
  }

  void setRotation(const Vec3f R[3])
  {
    t.setRotation(R);
  }

  void setTranslation(const Vec3f& T)
  {
    t.setTranslation(T);
  }

  void setQuatRotation(const SimpleQuaternion& q)
  {
    t.setQuatRotation(q);
  }

  void setTransform(const Vec3f R[3], const Vec3f& T)
  {
    t.setTransform(R, T);
  }

  void setTransform(const SimpleQuaternion& q, const Vec3f& T)
  {
    t.setTransform(q, T);
  }

  bool isIdentityTransform() const
  {
    return t.isIdentity();
  }

  void setIdentityTransform()
  {
    t.setIdentity();
  }

protected:

  /** AABB in global coordinate */
  mutable AABB aabb;

  /** AABB in local coordinate */
  AABB aabb_local;

  /** AABB center in local coordinate */
  Vec3f aabb_center;

  /** AABB radius */
  BVH_REAL aabb_radius;

  SimpleTransform t;
};


}

#endif

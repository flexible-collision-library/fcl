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

#include "fcl/BV/AABB.h"
#include "fcl/transform.h"
#include <boost/shared_ptr.hpp>

namespace fcl
{

enum OBJECT_TYPE {OT_UNKNOWN, OT_BVH, OT_GEOM};

enum NODE_TYPE {BV_UNKNOWN, BV_AABB, BV_OBB, BV_RSS, BV_kIOS, BV_OBBRSS, BV_KDOP16, BV_KDOP18, BV_KDOP24,
                GEOM_BOX, GEOM_SPHERE, GEOM_CAPSULE, GEOM_CONE, GEOM_CYLINDER, GEOM_CONVEX, GEOM_PLANE, GEOM_TRIANGLE};

class CollisionGeometry
{
public:
  virtual ~CollisionGeometry() {}

  virtual OBJECT_TYPE getObjectType() const { return OT_UNKNOWN; }

  virtual NODE_TYPE getNodeType() const { return BV_UNKNOWN; }

  virtual void computeLocalAABB() = 0;

  void* getUserData() const
  {
    return user_data;
  }

  void setUserData(void *data)
  {
    user_data = data;
  }

  /** AABB center in local coordinate */
  Vec3f aabb_center;

  /** AABB radius */
  FCL_REAL aabb_radius;

  /** pointer to user defined data specific to this object */
  void *user_data;
};

class CollisionObject
{
public:
  CollisionObject(const boost::shared_ptr<CollisionGeometry> &cgeom_) : cgeom(cgeom_)
  {
    computeAABB();
  }

  CollisionObject(const boost::shared_ptr<CollisionGeometry> &cgeom_, const SimpleTransform& tf) : cgeom(cgeom_), t(tf)
  {
    computeAABB();
  }

  CollisionObject(const boost::shared_ptr<CollisionGeometry> &cgeom_, const Matrix3f& R, const Vec3f& T):
      cgeom(cgeom_), t(SimpleTransform(R, T))
  {
    computeAABB();
  }

  CollisionObject()
  {
  }

  ~CollisionObject()
  {
  }

  OBJECT_TYPE getObjectType() const
  {
    return cgeom->getObjectType();
  }

  NODE_TYPE getNodeType() const
  {
    return cgeom->getNodeType();
  }

  inline const AABB& getAABB() const
  {
    return aabb;
  }

  inline void computeAABB()
  {
    Vec3f center = t.transform(cgeom->aabb_center);
    Vec3f delta(cgeom->aabb_radius, cgeom->aabb_radius, cgeom->aabb_radius);
    aabb.min_ = center - delta;
    aabb.max_ = center + delta;
  }

  void* getUserData() const
  {
    return user_data;
  }

  void setUserData(void *data)
  {
    user_data = data;
  }

  inline const Vec3f& getTranslation() const
  {
    return t.getTranslation();
  }

  inline const Matrix3f& getRotation() const
  {
    return t.getRotation();
  }

  inline const SimpleQuaternion& getQuatRotation() const
  {
    return t.getQuatRotation();
  }

  inline const SimpleTransform& getTransform() const
  {
    return t;
  }

  void setRotation(const Matrix3f& R)
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

  void setTransform(const Matrix3f& R, const Vec3f& T)
  {
    t.setTransform(R, T);
  }

  void setTransform(const SimpleQuaternion& q, const Vec3f& T)
  {
    t.setTransform(q, T);
  }

  void setTransform(const SimpleTransform& tf)
  {
    t = tf;
  }

  bool isIdentityTransform() const
  {
    return t.isIdentity();
  }

  void setIdentityTransform()
  {
    t.setIdentity();
  }

  const CollisionGeometry* getCollisionGeometry() const
  {
    return cgeom.get();
  }

protected:

  // const CollisionGeometry* cgeom;
  boost::shared_ptr<CollisionGeometry> cgeom;

  SimpleTransform t;

  /** AABB in global coordinate */
  mutable AABB aabb;

  /** pointer to user defined data specific to this object */
  void *user_data;
};

}

#endif

#ifndef FCL_CONTINUOUS_COLLISION_H
#define FCL_CONTINUOUS_COLLISION_H

#include "fcl/math/vec_3f.h"
#include "fcl/collision_object.h"
#include "fcl/collision_data.h"

namespace fcl
{

/// @brief continous collision checking between two objects
FCL_REAL continuousCollide(const CollisionGeometry* o1, const Transform3f& tf1_beg, const Transform3f& tf1_end,
                           const CollisionGeometry* o2, const Transform3f& tf2_beg, const Transform3f& tf2_end,
                           const ContinuousCollisionRequest& request,
                           ContinuousCollisionResult& result);

FCL_REAL continuousCollide(const CollisionObject* o1, const Transform3f& tf1_end,
                           const CollisionObject* o2, const Transform3f& tf2_end,
                           const ContinuousCollisionRequest& request,
                           ContinuousCollisionResult& result);

FCL_REAL collide(const ContinuousCollisionObject* o1, const ContinuousCollisionObject* o2,
                 const ContinuousCollisionRequest& request,
                 ContinuousCollisionResult& result);
          
}

#endif

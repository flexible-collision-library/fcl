#ifndef FCL_PENETRATION_DEPTH_H
#define FCL_PENETRATION_DEPTH_H

#include "fcl/math/vec_3f.h"
#include "fcl/collision_object.h"
#include "fcl/collision_data.h"

namespace fcl
{

/// @brief penetration depth computation between two in-collision objects
FCL_REAL penetration_depth(const CollisionGeometry* o1, const Transform3f& tf1,
                           const CollisionGeometry* o2, const Transform3f& tf2,
                           const PenetrationDepthRequest& request,
                           PenetrationDepthResult& result);

FCL_REAL penetration_depth(const CollisionObject* o1,
                           const CollisionObject* o2,
                           const PenetrationDepthRequest& request,
                           PenetrationDepthResult& result);

}

#endif

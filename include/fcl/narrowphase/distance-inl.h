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

#ifndef FCL_DISTANCE_INL_H
#define FCL_DISTANCE_INL_H

#include "fcl/narrowphase/distance.h"

#include "fcl/narrowphase/collision.h"

namespace fcl
{

//==============================================================================
extern template
double distance(
    const CollisionObject<double>* o1,
    const CollisionObject<double>* o2,
    const DistanceRequest<double>& request,
    DistanceResult<double>& result);

//==============================================================================
extern template
double distance(
    const CollisionGeometry<double>* o1, const Transform3<double>& tf1,
    const CollisionGeometry<double>* o2, const Transform3<double>& tf2,
    const DistanceRequest<double>& request, DistanceResult<double>& result);

//==============================================================================
template <typename GJKSolver>
detail::DistanceFunctionMatrix<GJKSolver>& getDistanceFunctionLookTable()
{
  static detail::DistanceFunctionMatrix<GJKSolver> table;
  return table;
}

//==============================================================================
template <typename NarrowPhaseSolver>
typename NarrowPhaseSolver::S distance(
    const CollisionObject<typename NarrowPhaseSolver::S>* o1,
    const CollisionObject<typename NarrowPhaseSolver::S>* o2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename NarrowPhaseSolver::S>& request,
    DistanceResult<typename NarrowPhaseSolver::S>& result)
{
  return distance<NarrowPhaseSolver>(
        o1->collisionGeometry().get(),
        o1->getTransform(),
        o2->collisionGeometry().get(),
        o2->getTransform(),
        nsolver,
        request,
        result);
}

//==============================================================================
template <typename NarrowPhaseSolver>
typename NarrowPhaseSolver::S distance(
    const CollisionGeometry<typename NarrowPhaseSolver::S>* o1,
    const Transform3<typename NarrowPhaseSolver::S>& tf1,
    const CollisionGeometry<typename NarrowPhaseSolver::S>* o2,
    const Transform3<typename NarrowPhaseSolver::S>& tf2,
    const NarrowPhaseSolver* nsolver_,
    const DistanceRequest<typename NarrowPhaseSolver::S>& request,
    DistanceResult<typename NarrowPhaseSolver::S>& result)
{
  using S = typename NarrowPhaseSolver::S;

  const NarrowPhaseSolver* nsolver = nsolver_;
  if(!nsolver_)
    nsolver = new NarrowPhaseSolver();

  const auto& looktable = getDistanceFunctionLookTable<NarrowPhaseSolver>();

  OBJECT_TYPE object_type1 = o1->getObjectType();
  NODE_TYPE node_type1 = o1->getNodeType();
  OBJECT_TYPE object_type2 = o2->getObjectType();
  NODE_TYPE node_type2 = o2->getNodeType();

  S res = std::numeric_limits<S>::max();


  if(object_type1 == OT_GEOM && object_type2 == OT_BVH)
  {
    if(!looktable.distance_matrix[node_type2][node_type1])
    {
      std::cerr << "Warning: distance function between node type " << node_type1 << " and node type " << node_type2 << " is not supported\n";
    }
    else
    {
      res = looktable.distance_matrix[node_type2][node_type1](o2, tf2, o1, tf1, nsolver, request, result);
    }
  }
  else
  {
    if(!looktable.distance_matrix[node_type1][node_type2])
    {
      std::cerr << "Warning: distance function between node type " << node_type1 << " and node type " << node_type2 << " is not supported\n";
    }
    else
    {
      res = looktable.distance_matrix[node_type1][node_type2](o1, tf1, o2, tf2, nsolver, request, result);
    }
  }

  // TODO(JS): FCL supports negative distance calculation only for OT_GEOM shape
  // types (i.e., primitive shapes like sphere, cylinder, box, and so on). As a
  // workaround for the rest shape types like mesh and octree, following
  // computes negative distance using additional penetration depth computation
  // of collision checking routine. The downside of this workaround is that the
  // pair of nearest points is not guaranteed to be on the surface of the
  // objects.
  if(res
     && result.min_distance < static_cast<S>(0)
     && request.enable_signed_distance)
  {
    if (std::is_same<NarrowPhaseSolver, detail::GJKSolver_libccd<S>>::value
        && object_type1 == OT_GEOM && object_type2 == OT_GEOM)
    {
      return res;
    }

    CollisionRequest<S> collision_request;
    collision_request.enable_contact = true;

    CollisionResult<S> collision_result;

    collide(o1, tf1, o2, tf2, nsolver, collision_request, collision_result);
    assert(collision_result.isCollision());

    std::size_t index = static_cast<std::size_t>(-1);
    S max_pen_depth = std::numeric_limits<S>::min();
    for (auto i = 0u; i < collision_result.numContacts(); ++i)
    {
      const auto& contact = collision_result.getContact(i);
      if (max_pen_depth < contact.penetration_depth)
      {
        max_pen_depth = contact.penetration_depth;
        index = i;
      }
    }
    result.min_distance = -max_pen_depth;
    assert(index != static_cast<std::size_t>(-1));

    if (request.enable_nearest_points)
    {
      const Vector3<S>& pos = collision_result.getContact(index).pos;
      result.nearest_points[0] = pos;
      result.nearest_points[1] = pos;
      // Note: The pair of nearest points is not guaranteed to be on the
      // surface of the objects.
    }
  }

  if(!nsolver_)
    delete nsolver;

  return res;
}

//==============================================================================
template <typename S>
S distance(
    const CollisionObject<S>* o1,
    const CollisionObject<S>* o2,
    const DistanceRequest<S>& request,
    DistanceResult<S>& result)
{
  switch(request.gjk_solver_type)
  {
  case GST_LIBCCD:
    {
      detail::GJKSolver_libccd<S> solver;
      solver.distance_tolerance = request.distance_tolerance;
      return distance(o1, o2, &solver, request, result);
    }
  case GST_INDEP:
    {
      detail::GJKSolver_indep<S> solver;
      solver.gjk_tolerance = request.distance_tolerance;
      return distance(o1, o2, &solver, request, result);
    }
  default:
    return -1; // error
  }
}

//==============================================================================
template <typename S>
S distance(
    const CollisionGeometry<S>* o1, const Transform3<S>& tf1,
    const CollisionGeometry<S>* o2, const Transform3<S>& tf2,
    const DistanceRequest<S>& request, DistanceResult<S>& result)
{
  switch(request.gjk_solver_type)
  {
  case GST_LIBCCD:
    {
      detail::GJKSolver_libccd<S> solver;
      solver.distance_tolerance = request.distance_tolerance;
      return distance(o1, tf1, o2, tf2, &solver, request, result);
    }
  case GST_INDEP:
    {
      detail::GJKSolver_indep<S> solver;
      solver.gjk_tolerance = request.distance_tolerance;
      return distance(o1, tf1, o2, tf2, &solver, request, result);
    }
  default:
    return -1;
  }
}

} // namespace fcl

#endif

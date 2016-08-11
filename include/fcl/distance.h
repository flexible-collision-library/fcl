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

#ifndef FCL_DISTANCE_H
#define FCL_DISTANCE_H

#include "fcl/collision_object.h"
#include "fcl/collision_data.h"
#include "fcl/distance_func_matrix.h"
#include "fcl/narrowphase/gjk_solver_indep.h"
#include "fcl/narrowphase/gjk_solver_libccd.h"

namespace fcl
{

/// @brief Main distance interface: given two collision objects, and the requirements for contacts, including whether return the nearest points, this function performs the distance between them. 
/// Return value is the minimum distance generated between the two objects.
template <typename S>
S distance(
    const CollisionObject<S>* o1, const CollisionObject<S>* o2,
    const DistanceRequest<S>& request, DistanceResult<S>& result);

template <typename S>
S distance(
    const CollisionGeometry<S>* o1, const Transform3<S>& tf1,
    const CollisionGeometry<S>* o2, const Transform3<S>& tf2,
    const DistanceRequest<S>& request, DistanceResult<S>& result);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename GJKSolver>
DistanceFunctionMatrix<GJKSolver>& getDistanceFunctionLookTable()
{
  static DistanceFunctionMatrix<GJKSolver> table;
  return table;
}

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
      std::cerr << "Warning: distance function between node type " << node_type1 << " and node type " << node_type2 << " is not supported" << std::endl;
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
      std::cerr << "Warning: distance function between node type " << node_type1 << " and node type " << node_type2 << " is not supported" << std::endl;
    }
    else
    {
      res = looktable.distance_matrix[node_type1][node_type2](o1, tf1, o2, tf2, nsolver, request, result);
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
      GJKSolver_libccd<S> solver;
      return distance<GJKSolver_libccd<S>>(o1, o2, &solver, request, result);
    }
  case GST_INDEP:
    {
      GJKSolver_indep<S> solver;
      return distance<GJKSolver_indep<S>>(o1, o2, &solver, request, result);
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
      GJKSolver_libccd<S> solver;
      return distance<GJKSolver_libccd<S>>(o1, tf1, o2, tf2, &solver, request, result);
    }
  case GST_INDEP:
    {
      GJKSolver_indep<S> solver;
      return distance<GJKSolver_indep<S>>(o1, tf1, o2, tf2, &solver, request, result);
    }
  default:
    return -1;
  }
}

} // namespace fcl

#endif

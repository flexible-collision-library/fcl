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

#ifndef FCL_COLLISION_INL_H
#define FCL_COLLISION_INL_H

#include "fcl/narrowphase/collision.h"

#include "fcl/narrowphase/detail/collision_func_matrix.h"
#include "fcl/narrowphase/detail/gjk_solver_indep.h"
#include "fcl/narrowphase/detail/gjk_solver_libccd.h"

namespace fcl
{

//==============================================================================
extern template
FCL_EXPORT
std::size_t collide(
    const CollisionObject<double>* o1,
    const CollisionObject<double>* o2,
    const CollisionRequest<double>& request,
    CollisionResult<double>& result);

//==============================================================================
extern template
FCL_EXPORT
std::size_t collide(
    const CollisionGeometry<double>* o1,
    const Transform3<double>& tf1,
    const CollisionGeometry<double>* o2,
    const Transform3<double>& tf2,
    const CollisionRequest<double>& request,
    CollisionResult<double>& result);

//==============================================================================
template<typename GJKSolver>
detail::CollisionFunctionMatrix<GJKSolver>& getCollisionFunctionLookTable()
{
  static detail::CollisionFunctionMatrix<GJKSolver> table;
  return table;
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
FCL_EXPORT
std::size_t collide(
    const CollisionObject<S>* o1,
    const CollisionObject<S>* o2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<S>& request,
    CollisionResult<S>& result)
{
  return collide(o1->collisionGeometry().get(), o1->getTransform(), o2->collisionGeometry().get(), o2->getTransform(),
                 nsolver, request, result);
}

//==============================================================================
template <typename S, typename NarrowPhaseSolver>
FCL_EXPORT
std::size_t collide(
    const CollisionGeometry<S>* o1,
    const Transform3<S>& tf1,
    const CollisionGeometry<S>* o2,
    const Transform3<S>& tf2,
    const NarrowPhaseSolver* nsolver_,
    const CollisionRequest<S>& request,
    CollisionResult<S>& result)
{
  const NarrowPhaseSolver* nsolver = nsolver_;
  if(!nsolver_)
    nsolver = new NarrowPhaseSolver();

  const auto& looktable = getCollisionFunctionLookTable<NarrowPhaseSolver>();

  std::size_t res;
  if(request.num_max_contacts == 0)
  {
    std::cerr << "Warning: should stop early as num_max_contact is " << request.num_max_contacts << " !" << std::endl;
    res = 0;
  }
  else
  {
    OBJECT_TYPE object_type1 = o1->getObjectType();
    OBJECT_TYPE object_type2 = o2->getObjectType();
    NODE_TYPE node_type1 = o1->getNodeType();
    NODE_TYPE node_type2 = o2->getNodeType();

    if(object_type1 == OT_GEOM && object_type2 == OT_BVH)
    {
      if(!looktable.collision_matrix[node_type2][node_type1])
      {
        std::cerr << "Warning: collision function between node type " << node_type1 << " and node type " << node_type2 << " is not supported"<< std::endl;
        res = 0;
      }
      else
        res = looktable.collision_matrix[node_type2][node_type1](o2, tf2, o1, tf1, nsolver, request, result);
    }
    else
    {
      if(!looktable.collision_matrix[node_type1][node_type2])
      {
        std::cerr << "Warning: collision function between node type " << node_type1 << " and node type " << node_type2 << " is not supported"<< std::endl;
        res = 0;
      }
      else
        res = looktable.collision_matrix[node_type1][node_type2](o1, tf1, o2, tf2, nsolver, request, result);
    }
  }

  if(!nsolver_)
    delete nsolver;

  return res;
}

//==============================================================================
template <typename S>
FCL_EXPORT
std::size_t collide(const CollisionObject<S>* o1, const CollisionObject<S>* o2,
                    const CollisionRequest<S>& request, CollisionResult<S>& result)
{
  switch(request.gjk_solver_type)
  {
  case GST_LIBCCD:
    {
      detail::GJKSolver_libccd<S> solver;
      solver.collision_tolerance = request.gjk_tolerance;
      return collide(o1, o2, &solver, request, result);
    }
  case GST_INDEP:
    {
      detail::GJKSolver_indep<S> solver;
      solver.gjk_tolerance = request.gjk_tolerance;
      solver.epa_tolerance = request.gjk_tolerance;
      return collide(o1, o2, &solver, request, result);
    }
  default:
    return -1; // error
  }
}

//==============================================================================
template <typename S>
FCL_EXPORT
std::size_t collide(
    const CollisionGeometry<S>* o1,
    const Transform3<S>& tf1,
    const CollisionGeometry<S>* o2,
    const Transform3<S>& tf2,
    const CollisionRequest<S>& request,
    CollisionResult<S>& result)
{
  switch(request.gjk_solver_type)
  {
  case GST_LIBCCD:
    {
      detail::GJKSolver_libccd<S> solver;
      solver.collision_tolerance = request.gjk_tolerance;
      return collide(o1, tf1, o2, tf2, &solver, request, result);
    }
  case GST_INDEP:
    {
      detail::GJKSolver_indep<S> solver;
      solver.gjk_tolerance = request.gjk_tolerance;
      solver.epa_tolerance = request.gjk_tolerance;
      return collide(o1, tf1, o2, tf2, &solver, request, result);
    }
  default:
    std::cerr << "Warning! Invalid GJK solver" << std::endl;
    return -1; // error
  }
}

} // namespace fcl

#endif

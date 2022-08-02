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

#ifndef FCL_CONTINUOUSCOLLISIONREQUEST_INL_H
#define FCL_CONTINUOUSCOLLISIONREQUEST_INL_H

#include "fcl/narrowphase/continuous_collision_request.h"

namespace fcl
{

//==============================================================================
#ifndef FCL_NARROWPHASE_CONTINUOUS_COLLISION_REQUEST_BUILDING
extern template
struct FCL_EXPORT_EXPL_INST_DECL ContinuousCollisionRequest<double>;
#endif

//==============================================================================
template <typename S>
ContinuousCollisionRequest<S>::ContinuousCollisionRequest(
    std::size_t num_max_iterations_,
    S toc_err_,
    CCDMotionType ccd_motion_type_,
    GJKSolverType gjk_solver_type_,
    CCDSolverType ccd_solver_type_)
  : num_max_iterations(num_max_iterations_),
    toc_err(toc_err_),
    ccd_motion_type(ccd_motion_type_),
    gjk_solver_type(gjk_solver_type_),
    ccd_solver_type(ccd_solver_type_)
{
  // Do nothing
}

} // namespace fcl

#endif

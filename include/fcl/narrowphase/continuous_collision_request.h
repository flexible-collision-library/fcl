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

#ifndef FCL_CONTINUOUSCOLLISIONREQUEST_H
#define FCL_CONTINUOUSCOLLISIONREQUEST_H

#include <cstddef>
#include "fcl/narrowphase/gjk_solver_type.h"
#include "fcl/export.h"

namespace fcl
{

enum CCDMotionType {CCDM_TRANS, CCDM_LINEAR, CCDM_SCREW, CCDM_SPLINE};
enum CCDSolverType {CCDC_NAIVE, CCDC_CONSERVATIVE_ADVANCEMENT, CCDC_RAY_SHOOTING, CCDC_POLYNOMIAL_SOLVER};

template <typename S>
struct FCL_EXPORT ContinuousCollisionRequest
{
  /// @brief maximum num of iterations
  std::size_t num_max_iterations;

  /// @brief error in first contact time
  S toc_err;

  /// @brief ccd motion type
  CCDMotionType ccd_motion_type;

  /// @brief gjk solver type
  GJKSolverType gjk_solver_type;

  /// @brief ccd solver type
  CCDSolverType ccd_solver_type;
  
  ContinuousCollisionRequest(std::size_t num_max_iterations_ = 10,
                             S toc_err_ = 0.0001,
                             CCDMotionType ccd_motion_type_ = CCDM_TRANS,
                             GJKSolverType gjk_solver_type_ = GST_LIBCCD,
                             CCDSolverType ccd_solver_type_ = CCDC_NAIVE);
  
};

using ContinuousCollisionRequestf = ContinuousCollisionRequest<float>;
using ContinuousCollisionRequestd = ContinuousCollisionRequest<double>;

} // namespace fcl

#include "fcl/narrowphase/continuous_collision_request-inl.h"

#endif

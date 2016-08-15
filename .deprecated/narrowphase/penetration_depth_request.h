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

#ifndef FCL_PENETRATIONDEPTHREQUEST_H
#define FCL_PENETRATIONDEPTHREQUEST_H

#include "fcl/common/types.h"
#include "fcl/narrowphase/gjk_solver.h"

namespace fcl
{

enum PenetrationDepthType {PDT_TRANSLATIONAL, PDT_GENERAL_EULER, PDT_GENERAL_QUAT, PDT_GENERAL_EULER_BALL, PDT_GENERAL_QUAT_BALL};

template <typename S>
struct PenetrationDepthRequest
{
  void* classifier;

  /// @brief PD algorithm type
  PenetrationDepthType pd_type;

  /// @brief gjk solver type
  GJKSolverType gjk_solver_type;

  Eigen::aligned_vector<Transform3<S>> contact_vectors;

  PenetrationDepthRequest(void* classifier_,
                          PenetrationDepthType pd_type_ = PDT_TRANSLATIONAL,
                          GJKSolverType gjk_solver_type_ = GST_LIBCCD);
};

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename S>
PenetrationDepthRequest<S>::PenetrationDepthRequest(
    void* classifier_,
    PenetrationDepthType pd_type_,
    GJKSolverType gjk_solver_type_)
  : classifier(classifier_),
    pd_type(pd_type_),
    gjk_solver_type(gjk_solver_type_)
{
}

} // namespace fcl

#endif

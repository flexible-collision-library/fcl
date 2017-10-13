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

#ifndef FCL_NARROWPHASE_DETAIL_PROJECT_H
#define FCL_NARROWPHASE_DETAIL_PROJECT_H

#include "fcl/common/types.h"
#include "fcl/math/geometry.h"

namespace fcl
{

namespace detail
{

/// @brief Project functions
template <typename S>
class FCL_EXPORT Project
{
public:
  struct ProjectResult
  {
    /// @brief Parameterization of the projected point (based on the simplex to be projected, use 2 or 3 or 4 of the array)
    S parameterization[4];

    /// @brief square distance from the query point to the projected simplex
    S sqr_distance;

    /// @brief the code of the projection type
    unsigned int encode;

    ProjectResult();
  };

  /// @brief Project point p onto line a-b
  static ProjectResult projectLine(const Vector3<S>& a, const Vector3<S>& b, const Vector3<S>& p);

  /// @brief Project point p onto triangle a-b-c
  static ProjectResult projectTriangle(const Vector3<S>& a, const Vector3<S>& b, const Vector3<S>& c, const Vector3<S>& p);

  /// @brief Project point p onto tetrahedra a-b-c-d
  static ProjectResult projectTetrahedra(const Vector3<S>& a, const Vector3<S>& b, const Vector3<S>& c, const Vector3<S>& d, const Vector3<S>& p);

  /// @brief Project origin (0) onto line a-b
  static ProjectResult projectLineOrigin(const Vector3<S>& a, const Vector3<S>& b);

  /// @brief Project origin (0) onto triangle a-b-c
  static ProjectResult projectTriangleOrigin(const Vector3<S>& a, const Vector3<S>& b, const Vector3<S>& c);

  /// @brief Project origin (0) onto tetrahedran a-b-c-d
  static ProjectResult projectTetrahedraOrigin(const Vector3<S>& a, const Vector3<S>& b, const Vector3<S>& c, const Vector3<S>& d);
};

using Projectf = Project<float>;
using Projectd = Project<double>;

} // namespace detail
} // namespace fcl

#include "fcl/math/detail/project-inl.h"

#endif

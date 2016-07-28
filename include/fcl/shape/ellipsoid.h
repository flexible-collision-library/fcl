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


#ifndef FCL_SHAPE_ELLIPSOID_H
#define FCL_SHAPE_ELLIPSOID_H

#include "fcl/shape/shape_base.h"

namespace fcl
{

/// @brief Center at zero point ellipsoid
class Ellipsoid : public ShapeBased
{
public:
  Ellipsoid(FCL_REAL a, FCL_REAL b, FCL_REAL c) : ShapeBase(), radii(a, b, c)
  {
  }

  Ellipsoid(const Vector3d& radii_) : ShapeBase(), radii(radii_)
  {
  }

  /// @brief Radii of the ellipsoid
  Vector3d radii;

  /// @brief Compute AABB
  void computeLocalAABB();

  /// @brief Get node type: a sphere
  NODE_TYPE getNodeType() const { return GEOM_ELLIPSOID; }

  Matrix3d computeMomentofInertia() const
  {
    const FCL_REAL V = computeVolume();

    const FCL_REAL a2 = radii[0] * radii[0] * V;
    const FCL_REAL b2 = radii[1] * radii[1] * V;
    const FCL_REAL c2 = radii[2] * radii[2] * V;

    return Vector3d(0.2 * (b2 + c2), 0.2 * (a2 + c2), 0.2 * (a2 + b2)).asDiagonal();
  }

  FCL_REAL computeVolume() const
  {
    const FCL_REAL pi = constants::pi;
    return 4.0 * pi * radii[0] * radii[1] * radii[2] / 3.0;
  }
};

}

#endif

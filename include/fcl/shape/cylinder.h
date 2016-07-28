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


#ifndef FCL_SHAPE_CYLINDER_H
#define FCL_SHAPE_CYLINDER_H

#include "fcl/shape/shape_base.h"

namespace fcl
{

/// @brief Center at zero cylinder 
class Cylinder : public ShapeBased
{
public:
  Cylinder(FCL_REAL radius_, FCL_REAL lz_) : ShapeBased(), radius(radius_), lz(lz_)
  {
  }

  
  /// @brief Radius of the cylinder 
  FCL_REAL radius;

  /// @brief Length along z axis 
  FCL_REAL lz;

  /// @brief Compute AABB 
  void computeLocalAABB();

  /// @brief Get node type: a cylinder 
  NODE_TYPE getNodeType() const { return GEOM_CYLINDER; }

  FCL_REAL computeVolume() const
  {
    return constants::pi * radius * radius * lz;
  }

  Matrix3d computeMomentofInertia() const
  {
    FCL_REAL V = computeVolume();
    FCL_REAL ix = V * (3 * radius * radius + lz * lz) / 12;
    FCL_REAL iz = V * radius * radius / 2;

    return Vector3d(ix, ix, iz).asDiagonal();
  }
};

}

#endif

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


#ifndef FCL_SHAPE_PLANE_H
#define FCL_SHAPE_PLANE_H

#include "fcl/shape/shape_base.h"

namespace fcl
{

/// @brief Infinite plane 
class Plane : public ShapeBased
{
public:
  /// @brief Construct a plane with normal direction and offset 
  Plane(const Vector3d& n_, FCL_REAL d_) : ShapeBased(), n(n_), d(d_)
  { 
    unitNormalTest(); 
  }
  
  /// @brief Construct a plane with normal direction and offset 
  Plane(FCL_REAL a, FCL_REAL b, FCL_REAL c, FCL_REAL d_) : ShapeBased(), n(a, b, c), d(d_)
  {
    unitNormalTest();
  }

  Plane() : ShapeBased(), n(1, 0, 0), d(0)
  {}

  FCL_REAL signedDistance(const Vector3d& p) const
  {
    return n.dot(p) - d;
  }

  FCL_REAL distance(const Vector3d& p) const
  {
    return std::abs(n.dot(p) - d);
  }

  /// @brief Compute AABB 
  void computeLocalAABB();

  /// @brief Get node type: a plane 
  NODE_TYPE getNodeType() const { return GEOM_PLANE; }

  /// @brief Plane normal 
  Vector3d n;

  /// @brief Plane offset 
  FCL_REAL d;

protected:
  
  /// @brief Turn non-unit normal into unit 
  void unitNormalTest();
};


}

#endif

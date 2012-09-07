/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

/** \author Dalibor Matura, Jia Pan */

#ifndef FCL_ARTICULATED_MODEL_JOINT_H
#define FCL_ARTICULATED_MODEL_JOINT_H

#include "fcl/math/transform.h"
#include "fcl/data_types.h"

#include <string>
#include <vector>
#include <map>
#include <limits>
#include <boost/shared_ptr.hpp>

namespace fcl
{

class JointConfig;

/// @brief Base Joint
class Joint
{
public:
  Joint(const std::string& name, std::size_t dofs_num);

  virtual ~Joint() {}

  /// @brief Get joint's name
  std::string getName() const;

  /// @brief Get Joint's degrees of freedom
  std::size_t getDOFs() const;

  /// @brief How joint works
  virtual Transform3f append(const JointConfig& q, const Transform3f& t) const = 0;

  bool isCompatible(const JointConfig& q) const;
  
private:
  std::string name_;	
  std::size_t dofs_num_;
};


class PrismaticJoint : public Joint
{
public:
  PrismaticJoint(const std::string& name, const Vec3f& axis);

  virtual ~PrismaticJoint() {}

  Vec3f getAxis() const;

  Transform3f append(const JointConfig& q, const Transform3f& t) const;

private:
  Vec3f axis_; // prismatic axis
};


class RevoluteJoint : public Joint
{
public:
  RevoluteJoint(const std::string& name, const Vec3f& axis);

  virtual ~RevoluteJoint() {}

  Vec3f getAxis() const;

  Transform3f append(const JointConfig& q, const Transform3f& t) const;
  
private:
  Vec3f axis_; // revolute axis
};

class SphericJoint : public Joint
{
public:
  SphericJoint(const std::string& name);

  virtual ~SphericJoint() {}

  Transform3f append(const JointConfig& q, const Transform3f& t) const;
 
private:
};


}

#endif

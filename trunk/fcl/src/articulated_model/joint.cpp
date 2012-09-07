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

#include "fcl/articulated_model/joint.h"
#include "fcl/articulated_model/joint_config.h"

namespace fcl
{

Joint::Joint(const std::string& name, std::size_t dofs_num) :
  name_(name),
  dofs_num_(dofs_num)
{
}

std::string Joint::getName() const
{
  return name_;
}

std::size_t Joint::getDOFs() const
{
  return dofs_num_;
}

bool Joint::isCompatible(const JointConfig& q) const
{
  return this == q.getJoint().get();
}

PrismaticJoint::PrismaticJoint(const std::string& name, const Vec3f& axis) :
  Joint(name, 1),
  axis_(axis)
{
  axis_.normalize();
}

Vec3f PrismaticJoint::getAxis() const
{
  return axis_;
}

Transform3f PrismaticJoint::append(const JointConfig& q, const Transform3f& t) const
{
  BOOST_ASSERT(isCompatible(q));
  /// axis_ is in local frame
  return Transform3f(axis_ * q[0]) * t; 
}

RevoluteJoint::RevoluteJoint(const std::string& name, const Vec3f& axis) :
  Joint(name, 1),
  axis_(axis)
{
  axis_.normalize();
}

Vec3f RevoluteJoint::getAxis() const
{
  return axis_;
}

Transform3f RevoluteJoint::append(const JointConfig& q, const Transform3f& t) const
{
  BOOST_ASSERT(isCompatible(q));
  Quaternion3f quat;
  quat.fromAxisAngle(axis_, q[0]);
  return Transform3f(quat) * t;
}

SphericJoint::SphericJoint(const std::string& name) :
  Joint(name, 3)
{}

Transform3f SphericJoint::append(const JointConfig& q, const Transform3f& t) const
{
  BOOST_ASSERT(isCompatible(q));
  Matrix3f rot;
  rot.setEulerYPR(q[0], q[1], q[2]);
  return Transform3f(rot) * t;
}





}

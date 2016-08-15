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

/** \author Dalibor Matura, Jia Pan */

#ifndef FCL_ARTICULATED_MODEL_JOINT_H
#define FCL_ARTICULATED_MODEL_JOINT_H

#include "fcl/common/data_types.h"

#include <string>
#include <vector>
#include <map>
#include <limits>
#include <memory>

namespace fcl
{

class JointConfig;
class Link;

enum JointType {JT_UNKNOWN, JT_PRISMATIC, JT_REVOLUTE, JT_BALLEULER};

/// @brief Base Joint
template <typename S>
class Joint
{
public:

  Joint(const std::shared_ptr<Link>& link_parent, const std::shared_ptr<Link>& link_child,
        const Transform3<S>& transform_to_parent,
        const std::string& name);

  Joint(const std::string& name);

  virtual ~Joint() {}

  const std::string& getName() const;
  void setName(const std::string& name);

  virtual Transform3<S> getLocalTransform() const = 0;

  virtual std::size_t getNumDofs() const = 0;

  std::shared_ptr<JointConfig> getJointConfig() const;
  void setJointConfig(const std::shared_ptr<JointConfig>& joint_cfg);

  std::shared_ptr<Link> getParentLink() const;
  std::shared_ptr<Link> getChildLink() const;

  void setParentLink(const std::shared_ptr<Link>& link);
  void setChildLink(const std::shared_ptr<Link>& link);

  JointType getJointType() const;

  const Transform3<S>& getTransformToParent() const;
  void setTransformToParent(const Transform3<S>& t);
  
protected:

  /// links to parent and child are only for connection, so weak_ptr to avoid cyclic dependency
  std::weak_ptr<Link> link_parent_, link_child_;

  JointType type_;

  std::string name_;
  
  std::shared_ptr<JointConfig> joint_cfg_;

  Transform3<S> transform_to_parent_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename S>
class PrismaticJoint : public Joint
{
public:
  PrismaticJoint(const std::shared_ptr<Link>& link_parent, const std::shared_ptr<Link>& link_child,
                 const Transform3<S>& transform_to_parent,
                 const std::string& name,
                 const Vector3<S>& axis);

  virtual ~PrismaticJoint() {}

  Transform3<S> getLocalTransform() const;

  std::size_t getNumDofs() const;

  const Vector3<S>& getAxis() const;

protected:
  Vector3<S> axis_;
};

template <typename S>
class RevoluteJoint : public Joint
{
public:
  RevoluteJoint(const std::shared_ptr<Link>& link_parent, const std::shared_ptr<Link>& link_child,
                const Transform3<S>& transform_to_parent,
                const std::string& name,
                const Vector3<S>& axis);

  virtual ~RevoluteJoint() {}

  Transform3<S> getLocalTransform() const;

  std::size_t getNumDofs() const;

  const Vector3<S>& getAxis() const;

protected:
  Vector3<S> axis_;
};

template <typename S>
class BallEulerJoint : public Joint
{
public:
  BallEulerJoint(const std::shared_ptr<Link>& link_parent, const std::shared_ptr<Link>& link_child,
                 const Transform3<S>& transform_to_parent,
                 const std::string& name);

  virtual ~BallEulerJoint() {}

  std::size_t getNumDofs() const;

  Transform3<S> getLocalTransform() const;
};

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename S>
Joint<S>::Joint(const std::shared_ptr<Link>& link_parent, const std::shared_ptr<Link>& link_child,
             const Transform3<S>& transform_to_parent,
             const std::string& name) :
  link_parent_(link_parent), link_child_(link_child),
  name_(name),
  transform_to_parent_(transform_to_parent)
{}

//==============================================================================
template <typename S>
Joint<S>::Joint(const std::string& name) :
  name_(name)
{
}

//==============================================================================
template <typename S>
const std::string& Joint<S>::getName() const
{
  return name_;
}

//==============================================================================
template <typename S>
void Joint<S>::setName(const std::string& name)
{
  name_ = name;
}

//==============================================================================
template <typename S>
std::shared_ptr<JointConfig> Joint<S>::getJointConfig() const
{
  return joint_cfg_;
}

//==============================================================================
template <typename S>
void Joint<S>::setJointConfig(const std::shared_ptr<JointConfig>& joint_cfg)
{
  joint_cfg_ = joint_cfg;
}

//==============================================================================
template <typename S>
std::shared_ptr<Link> Joint<S>::getParentLink() const
{
  return link_parent_.lock();
}

//==============================================================================
template <typename S>
std::shared_ptr<Link> Joint<S>::getChildLink() const
{
  return link_child_.lock();
}

//==============================================================================
template <typename S>
void Joint<S>::setParentLink(const std::shared_ptr<Link>& link)
{
  link_parent_ = link;
}

//==============================================================================
template <typename S>
void Joint<S>::setChildLink(const std::shared_ptr<Link>& link)
{
  link_child_ = link;
}

//==============================================================================
template <typename S>
JointType Joint<S>::getJointType() const
{
  return type_;
}

//==============================================================================
template <typename S>
const Transform3<S>& Joint<S>::getTransformToParent() const
{
  return transform_to_parent_;
}

//==============================================================================
template <typename S>
void Joint<S>::setTransformToParent(const Transform3<S>& t)
{
  transform_to_parent_ = t;
}

//==============================================================================
template <typename S>
PrismaticJoint<S>::PrismaticJoint(const std::shared_ptr<Link>& link_parent, const std::shared_ptr<Link>& link_child,
                               const Transform3<S>& transform_to_parent,
                               const std::string& name,
                               const Vector3<S>& axis) :
  Joint(link_parent, link_child, transform_to_parent, name),
  axis_(axis)
{
  type_ = JT_PRISMATIC;
}

//==============================================================================
template <typename S>
const Vector3<S>& PrismaticJoint<S>::getAxis() const
{
  return axis_;
}

//==============================================================================
template <typename S>
std::size_t PrismaticJoint<S>::getNumDofs() const
{
  return 1;
}

//==============================================================================
template <typename S>
Transform3<S> PrismaticJoint<S>::getLocalTransform() const
{
  const Quaternion<S> quat(transform_to_parent_.linear());
  const Vector3<S>& transl = transform_to_parent_.translation();

  Transform3<S> tf = Transform3<S>::Identity();
  tf.linear() = quat.toRotationMatrix();
  tf.translation() = quat * (axis_ * (*joint_cfg_)[0]) + transl;

  return tf;
}

//==============================================================================
template <typename S>
RevoluteJoint<S>::RevoluteJoint(const std::shared_ptr<Link>& link_parent, const std::shared_ptr<Link>& link_child,
                             const Transform3<S>& transform_to_parent,
                             const std::string& name,
                             const Vector3<S>& axis) :
  Joint(link_parent, link_child, transform_to_parent, name),
  axis_(axis)
{
  type_ = JT_REVOLUTE;
}

//==============================================================================
template <typename S>
const Vector3<S>& RevoluteJoint<S>::getAxis() const
{
  return axis_;
}

//==============================================================================
template <typename S>
std::size_t RevoluteJoint<S>::getNumDofs() const
{
  return 1;
}

//==============================================================================
template <typename S>
Transform3<S> RevoluteJoint<S>::getLocalTransform() const
{
  Transform3<S> tf = Transform3<S>::Identity();
  tf.linear() = transform_to_parent_.linear() * AngleAxis<S>((*joint_cfg_)[0], axis_);
  tf.translation() = transform_to_parent_.translation();

  return tf;
}

//==============================================================================
template <typename S>
BallEulerJoint<S>::BallEulerJoint(const std::shared_ptr<Link>& link_parent, const std::shared_ptr<Link>& link_child,
                               const Transform3<S>& transform_to_parent,
                               const std::string& name) :
  Joint(link_parent, link_child, transform_to_parent, name)
{}

//==============================================================================
template <typename S>
std::size_t BallEulerJoint<S>::getNumDofs() const
{
  return 3;
}

//==============================================================================
template <typename S>
Transform3<S> BallEulerJoint<S>::getLocalTransform() const
{
  Matrix3<S> rot(
      AngleAxis<S>((*joint_cfg_)[0], Eigen::Vector3<S>::UnitX())
        * AngleAxis<S>((*joint_cfg_)[1], Eigen::Vector3<S>::UnitY())
        * AngleAxis<S>((*joint_cfg_)[2], Eigen::Vector3<S>::UnitZ()));

  Transform3<S> tf = Transform3<S>::Identity();
  tf.linear() = rot;

  return transform_to_parent_ * tf;
}

} // namespace fcl

#endif

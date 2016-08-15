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

#ifndef FCL_ARTICULATED_MODEL_LINK_H
#define FCL_ARTICULATED_MODEL_LINK_H

#include "fcl/common/data_types.h"
#include "fcl/object/collision_object.h"

#include <memory>
#include <vector>

namespace fcl
{

template <typename S>
class Joint;

template <typename S>
class Link
{
public:
  Link(const std::string& name);

  const std::string& getName() const;
  
  void setName(const std::string& name);
  
  void addChildJoint(const std::shared_ptr<Joint>& joint);
  
  void setParentJoint(const std::shared_ptr<Joint>& joint);
  
  void addObject(const std::shared_ptr<CollisionObject<S>>& object);
  
  std::size_t getNumChildJoints() const;
  
  std::size_t getNumObjects() const;
  
protected:
  std::string name_;

  std::vector<std::shared_ptr<CollisionObject<S>> > objects_;

  std::vector<std::shared_ptr<Joint> > children_joints_;

  std::shared_ptr<Joint> parent_joint_;
};

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename S>
Link<S>::Link(const std::string& name) : name_(name)
{}

//==============================================================================
template <typename S>
const std::string& Link<S>::getName() const
{
  return name_;
}

//==============================================================================
template <typename S>
void Link<S>::setName(const std::string& name)
{
  name_ = name;
}

//==============================================================================
template <typename S>
void Link<S>::addChildJoint(const std::shared_ptr<Joint>& joint)
{
  children_joints_.push_back(joint);
}

//==============================================================================
template <typename S>
void Link<S>::setParentJoint(const std::shared_ptr<Joint>& joint)
{
  parent_joint_ = joint;
}

//==============================================================================
template <typename S>
void Link<S>::addObject(const std::shared_ptr<CollisionObject<S>>& object)
{
  objects_.push_back(object);
}

//==============================================================================
template <typename S>
std::size_t Link<S>::getNumChildJoints() const
{
  return children_joints_.size();
}

//==============================================================================
template <typename S>
std::size_t Link<S>::getNumObjects() const
{
  return objects_.size();
}

} // namespace fcl

#endif

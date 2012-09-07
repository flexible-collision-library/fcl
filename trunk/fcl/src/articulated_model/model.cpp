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

#include "fcl/articulated_model/model.h"
#include "fcl/articulated_model/model_config.h"
#include <boost/assert.hpp>

namespace fcl
{

Model::Model(const std::string& name) :
  name_(name)
{
}

boost::shared_ptr<Model> Model::getSharedPtr()
{
  return shared_from_this();
}

std::string Model::getName() const
{
  return name_;
}

void Model::addJoint(boost::shared_ptr<Joint> joint, const std::string& parent_name)
{
  if(parent_name == "")
    joints_tree_.createRoot(joint->getName(), joint);
  else
    joints_tree_.createNode(joint->getName(), joint, parent_name);
}

boost::shared_ptr<Joint> Model::getJointByName(const std::string& joint_name) const
{
  boost::shared_ptr<GeneralTreeNode<std::string, boost::shared_ptr<Joint> > > node = joints_tree_.getNodeByKey(joint_name);

  return node->getValue();
}

boost::shared_ptr<Joint> Model::getJointParentByName(const std::string& joint_name) const
{
  boost::shared_ptr<GeneralTreeNode<std::string, boost::shared_ptr<Joint> > > node = joints_tree_.getNodeByKey(joint_name);

  boost::shared_ptr<GeneralTreeNode<std::string, boost::shared_ptr<Joint> > > parent_node = node->getParent();

  if(parent_node.use_count() == 0)
    return boost::shared_ptr<Joint>();
  else
    return parent_node->getValue();
}

boost::shared_ptr<Joint> Model::getJointParent(boost::shared_ptr<Joint> joint) const
{
  return getJointParentByName(joint->getName());
}

bool Model::hasJointParentByName(const std::string& joint_name) const
{
  boost::shared_ptr<GeneralTreeNode<std::string, boost::shared_ptr<Joint> > > node = joints_tree_.getNodeByKey(joint_name);

  return node->hasParent();
}

bool Model::hasJointParent(boost::shared_ptr<Joint> joint) const
{
  return hasJointParentByName(joint->getName());
}

std::size_t Model::getJointNum() const
{
  return joints_tree_.getNodesNum();
}

std::map<std::string, boost::shared_ptr<Joint> > Model::getJointsMap() const
{
  return joints_tree_.getValuesMap();
}

bool Model::isCompatible(const ModelConfig& model_config) const
{
  std::map<std::string, JointConfig> joint_cfgs_map = model_config.getJointCfgsMap();
  std::map<std::string, boost::shared_ptr<Joint> > joints_map = getJointsMap();

  std::map<std::string, boost::shared_ptr<Joint> >::const_iterator it;

  for(it = joints_map.begin(); it != joints_map.end(); ++it)
  {
    std::map<std::string, JointConfig>::const_iterator it2 = joint_cfgs_map.find(it->first);
    if(it2 == joint_cfgs_map.end()) return false;

    if(it2->second.getJoint() != it->second) return false;
  }

  return true;
}

std::vector<boost::shared_ptr<Joint> > Model::getJointsChain(const std::string& last_joint_name) const
{
  std::vector<boost::shared_ptr<Joint> > chain;
  boost::shared_ptr<Joint> joint = getJointByName(last_joint_name);

  while(joint.use_count() != 0)
  {
    chain.push_back(joint);
    joint = getJointParent(joint);
  }

  return chain;
}

}

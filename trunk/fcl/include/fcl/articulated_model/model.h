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

#ifndef FCL_ARTICULATED_MODEL_MODEL_H
#define FCL_ARTICULATED_MODEL_MODEL_H

#include "fcl/data_types.h"
#include "fcl/articulated_model/joint.h"

#include <map>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>


namespace fcl
{


template<typename Key, typename Value>
class GeneralTreeNode : public boost::enable_shared_from_this<GeneralTreeNode<Key, Value> >
{
public:
  GeneralTreeNode(Key key, Value value):
    key_(key),
    value_(value)
  {}

  void setParent(boost::weak_ptr<GeneralTreeNode<Key, Value> > parent)
  {
    this->parent_ = parent;

    boost::shared_ptr<GeneralTreeNode<Key, Value> > parent_shared;

    if(parent_shared = parent.lock())
      parent_shared->addChild(this->shared_from_this());
  }

  boost::shared_ptr<GeneralTreeNode<Key, Value> > getParent() const
  {
    boost::shared_ptr<GeneralTreeNode<Key, Value> > parent_shared;
    parent_shared = this->parent_.lock();
    return parent_shared;
  }

  bool hasParent() const
  {
    bool has_parent = false;

    if(this->parent_.lock())
      has_parent = true;

    return has_parent;
  }

  void addChild(boost::shared_ptr<GeneralTreeNode<Key, Value> > child)
  {
    this->children_map_[child->getKey()] = child;
  }

  std::map<Key, boost::shared_ptr<GeneralTreeNode<Key, Value> > > getChilds() const
  {
    return this->children_map_;
  }

  boost::shared_ptr<GeneralTreeNode<Key, Value> > getChildByKey(Key key) const
  {
    typename std::map<Key, boost::shared_ptr<GeneralTreeNode<Key, Value> > >::const_iterator it = this->children_map_.find(key);

    boost::shared_ptr<GeneralTreeNode<Key, Value> > child = it->second; 

    BOOST_ASSERT_MSG(child.use_count() != 0, "Child wasn't found! Maybe bad key.");

    return child;
  }

  Key getKey() const
  {
    return this->key_;
  }

  Value getValue() const
  {
    return this->value_;
  }

private:
  Key key_;
  Value value_;

  boost::weak_ptr<GeneralTreeNode<Key, Value> > parent_; // empty means that this Joint is root
  std::map<Key, boost::shared_ptr<GeneralTreeNode<Key, Value> > > children_map_;
};


template<typename Key, typename Value>
class GeneralTree
{
public:
  GeneralTree()
  {
  }	

  boost::shared_ptr<GeneralTreeNode<Key, Value> > createRoot(Key key, Value value)
  {
    boost::shared_ptr<GeneralTreeNode<Key, Value> > node(new GeneralTreeNode<Key, Value>(key,value));

    if(this->root.use_count() != 0)
    {
      this->root->setParent(node);
    }

    this->root = node;
    this->nodes_map[key] = node;

    return node;
  }

  boost::shared_ptr<GeneralTreeNode<Key, Value> > createNode(Key key, Value value, Key parentKey)
  {
    boost::shared_ptr<GeneralTreeNode<Key, Value> > parent(getNodeByKey(parentKey) );

    if (parent.use_count() == 0)
    {
      return boost::shared_ptr<GeneralTreeNode<Key, Value> >();
    }

    boost::shared_ptr<GeneralTreeNode<Key, Value> > node(new GeneralTreeNode<Key, Value>(key,value) );

    node->setParent(parent);
    this->nodes_map[key] = node;

    return node;
  }	

  boost::shared_ptr<GeneralTreeNode<Key, Value> > getNodeByKey(Key key) const
  {
    typename std::map<Key, boost::weak_ptr<GeneralTreeNode<Key, Value> > >::const_iterator it = nodes_map.find(key);
    boost::weak_ptr<GeneralTreeNode<Key, Value> > node = it->second;
    return node.lock();
  }

  bool existsKey(Key key) const
  {
    return nodes_map.find(key) != nodes_map.end();
  }

  FCL_INT32 getNodesNum() const
  {
    return this->nodes_map.size();
  }

  std::map<Key, Value> getValuesMap() const
  {
    std::map<Key, Value> values_map;

    typename std::map<Key, boost::weak_ptr<GeneralTreeNode<Key, Value> > >::const_iterator it;

    for(it = this->nodes_map.begin(); it != this->nodes_map.end(); ++it)
    {
      Key first = it->first;
      if(boost::shared_ptr<GeneralTreeNode<Key, Value> > second_shared = it->second.lock())
      {
        values_map[first] = second_shared->getValue();
      }
    }

    return values_map;
  }

private:

  boost::shared_ptr<GeneralTreeNode<Key, Value> > root;
  std::map<Key, boost::weak_ptr<GeneralTreeNode<Key, Value> > > nodes_map; // holds node under its key
};

class ModelConfig;

class Model : public boost::enable_shared_from_this<Model>
{
public:
  Model(const std::string& name);

  boost::shared_ptr<Model> getSharedPtr();

  std::string getName() const;

  void addJoint(boost::shared_ptr<Joint> joint, const std::string& parent_name = "");

  boost::shared_ptr<Joint> getJointByName(const std::string& joint_name) const;
  boost::shared_ptr<Joint> getJointParentByName(const std::string& joint_name) const;
  boost::shared_ptr<Joint> getJointParent(boost::shared_ptr<Joint> joint) const;

  bool hasJointParentByName(const std::string& joint_name) const;
  bool hasJointParent(boost::shared_ptr<Joint> joint) const;

  std::size_t getJointNum() const;

  std::map<std::string, boost::shared_ptr<Joint> > getJointsMap() const;

  ///FCL_REAL getMotionBoundFromParentToChildJoint(const std::string& child_name) const;
  ///FCL_REAL getMotionBoundFromParentToChildJoint(boost::shared_ptr<Joint> child_joint) const;

  ///Vec3f getVectorFromParentToChildJoint(const std::string& child_name, FCL_REAL time) const;
  ///Vec3f getVectorFromParentToChildJoint(boost::shared_ptr<Joint> child_joint, FCL_REAL time) const;

  std::vector<boost::shared_ptr<Joint> > getJointsChain(const std::string& last_joint_name) const;
  std::vector<boost::shared_ptr<Joint> > getJointsChain(boost::shared_ptr<Joint> last_joint) const;

  bool isCompatible(const ModelConfig& model_config) const;

private:
  std::string name_;
  GeneralTree<std::string, boost::shared_ptr<Joint> > joints_tree_;
};


}

#endif


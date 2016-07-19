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

#ifndef FCL_ARTICULATED_MODEL_JOINT_CONFIG_H
#define FCL_ARTICULATED_MODEL_JOINT_CONFIG_H

#include "fcl/data_types.h"
#include <memory>
#include <vector>

namespace fcl
{

class Joint;

class JointConfig
{
public:
  JointConfig();

  JointConfig(const JointConfig& joint_cfg);

  JointConfig(const std::shared_ptr<Joint>& joint,
              FCL_REAL default_value = 0,
              FCL_REAL default_value_min = 0,
              FCL_REAL default_value_max = 0);

  std::size_t getDim() const;

  inline FCL_REAL operator [] (std::size_t i) const
  {
    return values_[i];
  }

  inline FCL_REAL& operator [] (std::size_t i)
  {
    return values_[i];
  }

  FCL_REAL getValue(std::size_t i) const;

  FCL_REAL& getValue(std::size_t i);
  
  FCL_REAL getLimitMin(std::size_t i) const;
  
  FCL_REAL& getLimitMin(std::size_t i);
  
  FCL_REAL getLimitMax(std::size_t i) const;
  
  FCL_REAL& getLimitMax(std::size_t i);
  
  std::shared_ptr<Joint> getJoint() const;

private:
  std::weak_ptr<Joint> joint_;
  
  std::vector<FCL_REAL> values_;
  std::vector<FCL_REAL> limits_min_;
  std::vector<FCL_REAL> limits_max_;
};

}



#endif

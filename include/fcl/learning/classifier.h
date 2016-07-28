/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2014, Willow Garage, Inc.
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

#ifndef FCL_LEARNING_CLASSIFIER_H
#define FCL_LEARNING_CLASSIFIER_H

#include <vector>

#include "fcl/data_types.h"

namespace fcl
{
template<std::size_t N>
struct Item
{
  VectorNd<N> q;
  bool label;
  FCL_REAL w;

  Item(const VectorNd<N>& q_, bool label_, FCL_REAL w_ = 1) : q(q_),
                                                           label(label_),
                                                           w(w_)
  {}

  Item() {}
};

template<std::size_t N>
struct Scaler
{
  VectorNd<N> v_min, v_max;
  Scaler()
  {
    // default no scale
    for(std::size_t i = 0; i < N; ++i)
    {
      v_min[i] = 0;
      v_max[i] = 1;
    }
  }

  Scaler(const VectorNd<N>& v_min_, const VectorNd<N>& v_max_) : v_min(v_min_),
                                                           v_max(v_max_)
  {}

  VectorNd<N> scale(const VectorNd<N>& v) const
  {
    VectorNd<N> res;
    for(std::size_t i = 0; i < N; ++i)
      res[i] = (v[i] - v_min[i]) / (v_max[i] - v_min[i]);
    return res;
  }

  VectorNd<N> unscale(const VectorNd<N>& v) const
  {
    VectorNd<N> res;
    for(std::size_t i = 0; i < N; ++i)
      res[i] = v[i] * (v_max[i] - v_min[i]) + v_min[i];
    return res;
  }
};


struct PredictResult
{
  bool label;
  FCL_REAL prob;

  PredictResult() {}
  PredictResult(bool label_, FCL_REAL prob_ = 1) : label(label_),
                                                   prob(prob_)
  {}
};

template<std::size_t N>
class SVMClassifier
{
public:

  ~SVMClassifier() {}
  
  virtual PredictResult predict(const VectorNd<N>& q) const = 0;
  virtual std::vector<PredictResult> predict(const std::vector<VectorNd<N> >& qs) const = 0;

  virtual std::vector<Item<N> > getSupportVectors() const = 0;
  virtual void setScaler(const Scaler<N>& scaler) = 0;

  virtual void learn(const std::vector<Item<N> >& data) = 0;

  FCL_REAL error_rate(const std::vector<Item<N> >& data) const
  {
    std::size_t num = data.size();

    std::size_t error_num = 0;
    for(std::size_t i = 0; i < data.size(); ++i)
    {
      PredictResult res = predict(data[i].q);
      if(res.label != data[i].label)
        error_num++;
    }

    return error_num / (FCL_REAL)num;
  }
};

template<std::size_t N>
Scaler<N> computeScaler(const std::vector<Item<N> >& data)
{
  VectorNd<N> lower_bound, upper_bound;
  for(std::size_t j = 0; j < N; ++j)
  {
    lower_bound[j] = std::numeric_limits<FCL_REAL>::max();
    upper_bound[j] = -std::numeric_limits<FCL_REAL>::max();
  }
  
  for(std::size_t i = 0; i < data.size(); ++i)
  {
    for(std::size_t j = 0; j < N; ++j)
    {
      if(data[i].q[j] < lower_bound[j]) lower_bound[j] = data[i].q[j];
      if(data[i].q[j] > upper_bound[j]) upper_bound[j] = data[i].q[j];
    }
  }

  return Scaler<N>(lower_bound, upper_bound);
}

template<std::size_t N>
Scaler<N> computeScaler(const std::vector<VectorNd<N> >& data)
{
  VectorNd<N> lower_bound, upper_bound;
  for(std::size_t j = 0; j < N; ++j)
  {
    lower_bound[j] = std::numeric_limits<FCL_REAL>::max();
    upper_bound[j] = -std::numeric_limits<FCL_REAL>::max();
  }
  
  for(std::size_t i = 0; i < data.size(); ++i)
  {
    for(std::size_t j = 0; j < N; ++j)
    {
      if(data[i][j] < lower_bound[j]) lower_bound[j] = data[i][j];
      if(data[i][j] > upper_bound[j]) upper_bound[j] = data[i][j];
    }
  }

  return Scaler<N>(lower_bound, upper_bound);  
}

}

#endif


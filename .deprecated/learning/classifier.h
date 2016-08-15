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

#include "fcl/common/data_types.h"

namespace fcl
{

template<typename S, std::size_t N>
struct Item
{
  VectorN<S, N> q;
  bool label;
  S w;

  Item(const VectorN<S, N>& q_, bool label_, S w_ = 1);

  Item();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF_VECTORIZABLE_FIXED_SIZE(S, N)
};

template<typename S, std::size_t N>
struct Scaler
{
  VectorN<S, N> v_min, v_max;
  Scaler();

  Scaler(const VectorN<S, N>& v_min_, const VectorN<S, N>& v_max_);

  VectorN<S, N> scale(const VectorN<S, N>& v) const;

  VectorN<S, N> unscale(const VectorN<S, N>& v) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF_VECTORIZABLE_FIXED_SIZE(S, N)
};

template<typename S>
struct PredictResult
{
  bool label;
  S prob;

  PredictResult();
  PredictResult(bool label_, S prob_ = 1);
};

template<typename S, std::size_t N>
class SVMClassifier
{
public:

  ~SVMClassifier();
  
  virtual PredictResult<S> predict(const VectorN<S, N>& q) const = 0;
  virtual std::vector<PredictResult<S>> predict(
      const std::vector<VectorN<S, N> >& qs) const = 0;

  virtual std::vector<Item<S, N> > getSupportVectors() const = 0;
  virtual void setScaler(const Scaler<S, N>& scaler) = 0;

  virtual void learn(const std::vector<Item<S, N> >& data) = 0;

  S error_rate(const std::vector<Item<S, N> >& data) const;
};

template<typename S, std::size_t N>
Scaler<S, N> computeScaler(const std::vector<Item<S, N> >& data);

template<typename S, std::size_t N>
Scaler<S, N> computeScaler(const std::vector<VectorN<S, N> >& data);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template<typename S, std::size_t N>
Item<S, N>::Item(const VectorN<S, N>& q_, bool label_, S w_)
  : q(q_), label(label_), w(w_)
{
  // Do nothing
}

//==============================================================================
template<typename S, std::size_t N>
Item<S, N>::Item()
{
  // Do nothing
}

//==============================================================================
template<typename S, std::size_t N>
Scaler<S, N>::Scaler()
{
  // default no scale
  for(std::size_t i = 0; i < N; ++i)
  {
    v_min[i] = 0;
    v_max[i] = 1;
  }
}

//==============================================================================
template<typename S, std::size_t N>
Scaler<S, N>::Scaler(const VectorN<S, N>& v_min_, const VectorN<S, N>& v_max_)
  : v_min(v_min_), v_max(v_max_)
{
  // Do nothing
}

//==============================================================================
template<typename S, std::size_t N>
VectorN<S, N> Scaler<S, N>::scale(const VectorN<S, N>& v) const
{
  VectorN<S, N> res;
  for(std::size_t i = 0; i < N; ++i)
    res[i] = (v[i] - v_min[i]) / (v_max[i] - v_min[i]);
  return res;
}

//==============================================================================
template<typename S, std::size_t N>
VectorN<S, N> Scaler<S, N>::unscale(const VectorN<S, N>& v) const
{
  VectorN<S, N> res;
  for(std::size_t i = 0; i < N; ++i)
    res[i] = v[i] * (v_max[i] - v_min[i]) + v_min[i];
  return res;
}

//==============================================================================
template<typename S>
PredictResult<S>::PredictResult()
{
  // Do nothing
}

//==============================================================================
template<typename S>
PredictResult<S>::PredictResult(bool label_, S prob_)
  : label(label_), prob(prob_)
{
  // Do nothing
}

//==============================================================================
template<typename S, std::size_t N>
SVMClassifier<S, N>::~SVMClassifier()
{
  // Do nothing
}

//==============================================================================
template<typename S, std::size_t N>
S SVMClassifier<S, N>::error_rate(const std::vector<Item<S, N> >& data) const
{
  std::size_t num = data.size();

  std::size_t error_num = 0;
  for(std::size_t i = 0; i < data.size(); ++i)
  {
    PredictResult<S> res = predict(data[i].q);
    if(res.label != data[i].label)
      error_num++;
  }

  return error_num / (S)num;
}

//==============================================================================
template<typename S, std::size_t N>
Scaler<S, N> computeScaler(const std::vector<Item<S, N> >& data)
{
  VectorN<S, N> lower_bound, upper_bound;
  for(std::size_t j = 0; j < N; ++j)
  {
    lower_bound[j] = std::numeric_limits<S>::max();
    upper_bound[j] = -std::numeric_limits<S>::max();
  }

  for(std::size_t i = 0; i < data.size(); ++i)
  {
    for(std::size_t j = 0; j < N; ++j)
    {
      if(data[i].q[j] < lower_bound[j]) lower_bound[j] = data[i].q[j];
      if(data[i].q[j] > upper_bound[j]) upper_bound[j] = data[i].q[j];
    }
  }

  return Scaler<S, N>(lower_bound, upper_bound);
}

//==============================================================================
template<typename S, std::size_t N>
Scaler<S, N> computeScaler(const std::vector<VectorN<S, N> >& data)
{
  VectorN<S, N> lower_bound, upper_bound;
  for(std::size_t j = 0; j < N; ++j)
  {
    lower_bound[j] = std::numeric_limits<S>::max();
    upper_bound[j] = -std::numeric_limits<S>::max();
  }

  for(std::size_t i = 0; i < data.size(); ++i)
  {
    for(std::size_t j = 0; j < N; ++j)
    {
      if(data[i][j] < lower_bound[j]) lower_bound[j] = data[i][j];
      if(data[i][j] > upper_bound[j]) upper_bound[j] = data[i][j];
    }
  }

  return Scaler<S, N>(lower_bound, upper_bound);
}

} // namespace fcl

#endif

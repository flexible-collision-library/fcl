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

#ifndef FCL_TEST_LIBSVM_CLASSIFIER_H
#define FCL_TEST_LIBSVM_CLASSIFIER_H

#include "fcl/learning/classifier.h"
#include <libsvm/svm.h>

namespace fcl
{


template<std::size_t N>
class LibSVMClassifier : public SVMClassifier<N>
{
public:
  LibSVMClassifier()
  {
    param.svm_type = C_SVC;
    param.kernel_type = RBF;
    param.degree = 3;
    param.gamma = 0;	// 1/num_features
    param.coef0 = 0;
    param.nu = 0.5;
    param.cache_size = 100; // can change
    param.C = 1;
    param.eps = 1e-3;
    param.p = 0.1;
    param.shrinking = 1;    // use shrinking
    param.probability = 0;
    param.nr_weight = 0;
    param.weight_label = NULL;
    param.weight = NULL;
			
    param.nr_weight = 2;
    param.weight_label = (int *)realloc(param.weight_label, sizeof(int) * param.nr_weight);
    param.weight = (double *)realloc(param.weight, sizeof(double) * param.nr_weight);
    param.weight_label[0] = -1;
    param.weight_label[1] = 1;
    param.weight[0] = 1;
    param.weight[1] = 1;
			
    model = NULL;
    x_space = NULL;
    problem.x = NULL;
    problem.y = NULL;
    problem.W = NULL;
  }


  void setCSVM() { param.svm_type = C_SVC; }
  void setNuSVM() { param.svm_type = NU_SVC; }
  void setC(FCL_REAL C) { param.C = C; }
  void setNu(FCL_REAL nu) { param.nu = nu; }
  void setLinearClassifier() { param.kernel_type = LINEAR; }
  void setNonLinearClassifier() { param.kernel_type = RBF; }
  void setProbability(bool use_probability) { param.probability = use_probability; }
  virtual void setScaler(const Scaler<N>& scaler_)
  {
    scaler = scaler_;
  }

  void setNegativeWeight(FCL_REAL c)
  {
    param.weight[0] = c;
  }

  void setPositiveWeight(FCL_REAL c)
  {
    param.weight[1] = c;
  }

  void setEPS(FCL_REAL e)
  {
    param.eps = e;
  }

  void setGamma(FCL_REAL gamma)
  {
    param.gamma = gamma;
  }

  ~LibSVMClassifier()
  {
    svm_destroy_param(&param);
    svm_free_and_destroy_model(&model);
    delete [] x_space;
    delete [] problem.x;
    delete [] problem.y;
    delete [] problem.W;
  }

  virtual void learn(const std::vector<Item<N> >& data)
  {
    if(data.size() == 0) return;

    if(model) svm_free_and_destroy_model(&model);
    if(param.gamma == 0) param.gamma = 1.0 / N;

    problem.l = data.size();
    if(problem.y) delete [] problem.y;
    problem.y = new double [problem.l];
    if(problem.x) delete [] problem.x;
    problem.x = new svm_node* [problem.l];
    if(problem.W) delete [] problem.W;
    problem.W = new double [problem.l];
    if(x_space) delete [] x_space;
    x_space = new svm_node [(N + 1) * problem.l];

    for(std::size_t i = 0; i < data.size(); ++i)
    {
      svm_node* cur_x_space = x_space + (N + 1) * i;
      Vecnf<N> q_scaled = scaler.scale(data[i].q);
      for(std::size_t j = 0; j < N; ++j)
      {
        cur_x_space[j].index = j + 1;
        cur_x_space[j].value = q_scaled[j];
      }
      cur_x_space[N].index = -1;

      problem.x[i] = cur_x_space;
      problem.y[i] = (data[i].label ? 1 : -1);
      problem.W[i] = data[i].w;
    }

    model = svm_train(&problem, &param);
    hyperw_normsqr = svm_hyper_w_normsqr_twoclass(model);
  }

  virtual std::vector<PredictResult> predict(const std::vector<Vecnf<N> >& qs) const
  {
    std::vector<PredictResult> predict_results;

    int nr_class = svm_get_nr_class(model);
    double* prob_estimates = NULL;

    svm_node* x = (svm_node*)malloc((N + 1) * sizeof(svm_node));
    if(param.probability)
      prob_estimates = (double*)malloc(nr_class * sizeof(double));

    Vecnf<N> v;
    for(std::size_t i = 0; i < qs.size(); ++i)
    {
      v = scaler.scale(qs[i]);
      for(std::size_t j = 0; j < N; ++j)
      {
        x[j].index = j + 1;
        x[j].value = v[j];
      }
      x[N].index = -1;

      double predict_label;

      if(param.probability)
      {
        predict_label = svm_predict_probability(model, x, prob_estimates);
        predict_label = (predict_label > 0) ? 1 : 0;
        predict_results.push_back(PredictResult(predict_label, *prob_estimates));
      }
      else
      {
        predict_label = svm_predict(model, x);
        predict_label = (predict_label > 0) ? 1 : 0;
        predict_results.push_back(PredictResult(predict_label));
      } 
    }

    if(param.probability) free(prob_estimates);
    free(x);

    return predict_results;
  }

  virtual PredictResult predict(const Vecnf<N>& q) const
  {
    return (predict(std::vector<Vecnf<N> >(1, q)))[0];
  }

  void save(const std::string& filename) const
  {
    if(model)
      svm_save_model(filename.c_str(), model);
  }

  virtual std::vector<Item<N> > getSupportVectors() const
  {
    std::vector<Item<N> > results;
    Item<N> item;
    for(std::size_t i = 0; i < (std::size_t)model->l; ++i)
    {
      for(std::size_t j = 0; j < N; ++j)
        item.q[j] = model->SV[i][j].value;
      item.q = scaler.unscale(item.q);
      int id = model->sv_indices[i] - 1;
      item.label = (problem.y[id] > 0);
      results.push_back(item);
    }

    return results;
  }

  svm_parameter param;		
  svm_problem problem;
  svm_node* x_space;
  svm_model* model;
  double hyperw_normsqr;

  Scaler<N> scaler;
};


}

#endif

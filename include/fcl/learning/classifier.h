#ifndef FCL_LEARNING_CLASSIFIER_H
#define FCL_LEARNING_CLASSIFIER_H

#include "fcl/math/vec_nf.h"

namespace fcl
{
template<std::size_t N>
struct Item
{
  Vecnf<N> q;
  bool label;
  FCL_REAL w;

  Item(const Vecnf<N>& q_, bool label_, FCL_REAL w_ = 1) : q(q_),
                                                           label(label_),
                                                           w(w_)
  {}

  Item() {}
};

template<std::size_t N>
struct Scaler
{
  Vecnf<N> v_min, v_max;
  Scaler()
  {
    // default no scale
    for(std::size_t i = 0; i < N; ++i)
    {
      v_min[i] = 0;
      v_max[i] = 1;
    }
  }

  Scaler(const Vecnf<N>& v_min_, const Vecnf<N>& v_max_) : v_min(v_min_),
                                                           v_max(v_max_)
  {}

  Vecnf<N> scale(const Vecnf<N>& v) const
  {
    Vecnf<N> res;
    for(std::size_t i = 0; i < N; ++i)
      res[i] = (v[i] - v_min[i]) / (v_max[i] - v_min[i]);
    return res;
  }

  Vecnf<N> unscale(const Vecnf<N>& v) const
  {
    Vecnf<N> res;
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
  
  virtual PredictResult predict(const Vecnf<N>& q) const = 0;
  virtual std::vector<PredictResult> predict(const std::vector<Vecnf<N> >& qs) const = 0;

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
  Vecnf<N> lower_bound, upper_bound;
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
Scaler<N> computeScaler(const std::vector<Vecnf<N> >& data)
{
  Vecnf<N> lower_bound, upper_bound;
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


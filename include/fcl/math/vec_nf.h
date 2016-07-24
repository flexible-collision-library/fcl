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

#ifndef FCL_MATH_VEC_NF_H
#define FCL_MATH_VEC_NF_H

#include <cmath>
#include <iostream>
#include <limits>
#include <vector>
#include <cstdarg>
#include "fcl/data_types.h"

namespace fcl
{

template<typename T, std::size_t N>
class Vec_n
{
public:
  Vec_n()
  {
    data.resize(N, 0);
  }

  template<std::size_t M>
  Vec_n(const Vec_n<T, M>& data_)
  {
    std::size_t n = std::min(M, N);
    for(std::size_t i = 0; i < n; ++i)
      data[i] = data_[i];
  }

  Vec_n(const std::vector<T>& data_)
  {
    data.resize(N, 0);
    std::size_t n = std::min(data_.size(), N);
    for(std::size_t i = 0; i < n; ++i)
      data[i] = data_[i];
  }

  bool operator == (const Vec_n<T, N>& other) const
  {
    for(std::size_t i = 0; i < N; ++i)
    {
      if(data[i] != other[i]) return false;
    }
    return true;
  }

  bool operator != (const Vec_n<T, N>& other) const
  {
    for(std::size_t i = 0; i < N; ++i)
    {
      if(data[i] != other[i]) return true;
    }

    return false;
  }
  
  
  std::size_t dim() const
  {
    return N;
  }

  void setData(const std::vector<T>& data_)
  {
    std::size_t n = std::min(data.size(), N);
    for(std::size_t i = 0; i < n; ++i)
      data[i] = data_[i];
  }

  T operator [] (std::size_t i) const
  {
    return data[i];
  }

  T& operator [] (std::size_t i)
  {
    return data[i];
  }

  Vec_n<T, N> operator + (const Vec_n<T, N>& other) const
  {
    Vec_n<T, N> result;
    for(std::size_t i = 0; i < N; ++i)
      result[i] = data[i] + other[i];
    return result;
  }

  Vec_n<T, N>& operator += (const Vec_n<T, N>& other)
  {
    for(std::size_t i = 0; i < N; ++i)
      data[i] += other[i];
    return *this;
  }

  Vec_n<T, N> operator - (const Vec_n<T, N>& other) const
  {
    Vec_n<T, N> result;
    for(std::size_t i = 0; i < N; ++i)
      result[i] = data[i] - other[i];
    return result;
  }

  Vec_n<T, N>& operator -= (const Vec_n<T, N>& other)
  {
    for(std::size_t i = 0; i < N; ++i)
      data[i] -= other[i];
    return *this;
  }

  Vec_n<T, N> operator * (T t) const
  {
    Vec_n<T, N> result;
    for(std::size_t i = 0; i < N; ++i)
      result[i] = data[i] * t;
    return result;
  }

  Vec_n<T, N>& operator *= (T t)
  {
    for(std::size_t i = 0; i < N; ++i)
      data[i] *= t;
    return *this;
  }

  Vec_n<T, N> operator / (T t) const
  {
    Vec_n<T, N> result;
    for(std::size_t i = 0; i < N; ++i)
      result[i] = data[i] / 5;
    return result;
  }

  Vec_n<T, N>& operator /= (T t)
  {
    for(std::size_t i = 0; i < N; ++i)
      data[i] /= t;
    return *this;
  }

  Vec_n<T, N>& setZero()
  {
    for(std::size_t i = 0; i < N; ++i)
      data[i] = 0;
  }

  T dot(const Vec_n<T, N>& other) const
  {
    T sum = 0;
    for(std::size_t i = 0; i < N; ++i)
      sum += data[i] * other[i];
    return sum;
  }

  std::vector<T> getData() const
  {
    return data;
  }
  
protected:
  std::vector<T> data;
};

template<typename T1, std::size_t N1,
         typename T2, std::size_t N2>
void repack(const Vec_n<T1, N1>& input,
            Vec_n<T2, N2>& output)
{
  std::size_t n = std::min(N1, N2);
  for(std::size_t i = 0; i < n; ++i)
    output[i] = input[i];
}

template<typename T, std::size_t N>
Vec_n<T, N> operator * (T t, const Vec_n<T, N>& v)
{
  return v * t;
}

template<typename T, std::size_t N, std::size_t M>
Vec_n<T, M+N> combine(const Vec_n<T, N>& v1, const Vec_n<T, M>& v2)
{
  Vec_n<T, M+N> v;
  for(std::size_t i = 0; i < N; ++i)
    v[i] = v1[i];
  for(std::size_t i = 0; i < M; ++i)
    v[i + N] = v2[i];

  return v;
}

template<typename T, std::size_t N>
std::ostream& operator << (std::ostream& o, const Vec_n<T, N>& v)
{
  o << "(";
  for(std::size_t i = 0; i < N; ++i)
  {
    if(i == N - 1)
      o << v[i] << ")";
    else
      o << v[i] << " ";
  }
  return o;
}

// workaround for template alias
template<std::size_t N>
class Vecnf : public Vec_n<FCL_REAL, N>
{
public:
  Vecnf(const Vec_n<FCL_REAL, N>& other) : Vec_n<FCL_REAL, N>(other)
  {}

  Vecnf() : Vec_n<FCL_REAL, N>()
  {}

  template<std::size_t M>
  Vecnf(const Vec_n<FCL_REAL, M>& other) : Vec_n<FCL_REAL, N>(other)
  {}

  Vecnf(const std::vector<FCL_REAL>& data_) : Vec_n<FCL_REAL, N>(data_)
  {}

  
};


}

#endif

#ifndef FCL_MATH_VEC_NF_H
#define FCL_MATH_VEC_NF_H

#include <cmath>
#include <iostream>
#include <limits>
#include <vector>
#include <cstdarg>
#include "fcl/data_types.h"

#include <Eigen/Dense>

namespace fcl
{

template <typename T, int N>
using Vec_n = Eigen::Matrix<T, N, 1>;

template <int N>
using Vecnf = Vec_n<FCL_REAL, N>;

template <typename T, int M, int N>
Vec_n<T, M+N> combine(const Vec_n<T, M>& v1, const Vec_n<T, N>& v2)
{
  Vec_n<T, M+N> v;
  v << v1, v2;

  return v;
}

} // namespace fcl

#endif

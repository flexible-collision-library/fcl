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

/** \author Jia Pan */

#ifndef FCL_GEOMETRY_H
#define FCL_GEOMETRY_H

#include <cmath>
#include <iostream>

#include "fcl/config.h"
#include "fcl/data_types.h"

namespace fcl
{

inline void normalize(Vector3d& v, bool* signal)
{
  Vector3d::Scalar sqr_length = v.squaredNorm();

  if (sqr_length > 0)
  {
    v /= std::sqrt(sqr_length);
    *signal = true;
  }
  else
  {
    *signal = false;
  }
}

inline Vector3d::Scalar triple(const Vector3d& x, const Vector3d& y, const Vector3d& z)
{
  return x.dot(y.cross(z));
}

template <typename T>
void generateCoordinateSystem(const Vector3<T>& w, Vector3<T>& u, Vector3<T>& v)
{
  T inv_length;

  if(std::abs(w[0]) >= std::abs(w[1]))
  {
    inv_length = (T)1.0 / sqrt(w[0] * w[0] + w[2] * w[2]);
    u[0] = -w[2] * inv_length;
    u[1] = (T)0;
    u[2] =  w[0] * inv_length;
    v[0] =  w[1] * u[2];
    v[1] =  w[2] * u[0] - w[0] * u[2];
    v[2] = -w[1] * u[0];
  }
  else
  {
    inv_length = (T)1.0 / sqrt(w[1] * w[1] + w[2] * w[2]);
    u[0] = (T)0;
    u[1] =  w[2] * inv_length;
    u[2] = -w[1] * inv_length;
    v[0] =  w[1] * u[2] - w[2] * u[1];
    v[1] = -w[0] * u[2];
    v[2] =  w[0] * u[1];
  }
}

template <typename T, int M, int N>
VectorN<T, M+N> combine(const VectorN<T, M>& v1, const VectorN<T, N>& v2)
{
  VectorN<T, M+N> v;
  v << v1, v2;

  return v;
}

template <typename T>
void hat(Matrix3<T>& mat, const Vector3<T>& vec)
{
  mat << 0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0;
}

/// @brief compute the eigen vector and eigen vector of a matrix. dout is the
/// eigen values, vout is the eigen vectors
template<typename T>
void eigen(const Matrix3<T>& m, Vector3<T>& dout, Matrix3<T>& vout)
{
  // We assume that m is symmetric matrix.
  Eigen::SelfAdjointEigenSolver<Matrix3<T>> eigensolver(m);
  if (eigensolver.info() != Eigen::Success)
  {
    std::cerr << "[eigen] Failed to compute eigendecomposition.\n";
    return;
  }
  dout = eigensolver.eigenvalues();
  vout = eigensolver.eigenvectors();
}

template <typename T>
void generateCoordinateSystem(Matrix3<T>& axis)
{
  // Assum axis.col(0) is closest to z-axis
  assert(axis.col(0).maxCoeff() == 2);

  if(std::abs(axis(0, 0)) >= std::abs(axis(1, 0)))
  {
    // let axis.col(0) = (x, y, z)
    // axis.col(1) = (-z, 0, x) / length((-z, 0, x)) // so that axis.col(0) and
    //                                               // axis.col(1) are
    //                                               // othorgonal
    // axis.col(2) = axis.col(0).cross(axis.col(1))

    T inv_length = 1.0 / sqrt(std::pow(axis(0, 0), 2) + std::pow(axis(2, 0), 2));

    axis(0, 1) = -axis(2, 0) * inv_length;
    axis(1, 1) = 0;
    axis(2, 1) =  axis(0, 0) * inv_length;

    axis(0, 2) =  axis(1, 0) * axis(2, 1);
    axis(1, 2) =  axis(2, 0) * axis(0, 1) - axis(0, 0) * axis(2, 1);
    axis(2, 2) = -axis(1, 0) * axis(0, 1);
  }
  else
  {
    // let axis.col(0) = (x, y, z)
    // axis.col(1) = (0, z, -y) / length((0, z, -y)) // so that axis.col(0) and
    //                                               // axis.col(1) are
    //                                               // othorgonal
    // axis.col(2) = axis.col(0).cross(axis.col(1))

    T inv_length = 1.0 / sqrt(std::pow(axis(1, 0), 2) + std::pow(axis(2, 0), 2));

    axis(0, 1) = 0;
    axis(1, 1) =  axis(2, 0) * inv_length;
    axis(2, 1) = -axis(1, 0) * inv_length;

    axis(0, 2) =  axis(1, 0) * axis(2, 1) - axis(2, 0) * axis(1, 1);
    axis(1, 2) = -axis(0, 0) * axis(2, 1);
    axis(2, 2) =  axis(0, 0) * axis(1, 1);
  }
}

template <typename Derived1, typename Derived2, typename Derived3, typename Derived4>
void relativeTransform(
    const Eigen::MatrixBase<Derived1>& R1, const Eigen::MatrixBase<Derived2>& t1,
    const Eigen::MatrixBase<Derived1>& R2, const Eigen::MatrixBase<Derived2>& t2,
    Eigen::MatrixBase<Derived3>& R, Eigen::MatrixBase<Derived4>& t)
{
  EIGEN_STATIC_ASSERT(
        Derived1::RowsAtCompileTime == 3
        && Derived1::ColsAtCompileTime == 3,
        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

  EIGEN_STATIC_ASSERT(
        Derived2::RowsAtCompileTime == 3
        && Derived2::ColsAtCompileTime == 1,
        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

  EIGEN_STATIC_ASSERT(
        Derived3::RowsAtCompileTime == 3
        && Derived3::ColsAtCompileTime == 3,
        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

  EIGEN_STATIC_ASSERT(
        Derived4::RowsAtCompileTime == 3
        && Derived4::ColsAtCompileTime == 1,
        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

  R = R1.transpose() * R2;
  t = R1.transpose() * (t2 - t1);
}

template <typename Scalar, typename Derived1, typename Derived2>
void relativeTransform(
    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& T1,
    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& T2,
    Eigen::MatrixBase<Derived1>& R, Eigen::MatrixBase<Derived2>& t)
{
  EIGEN_STATIC_ASSERT(
        Derived1::RowsAtCompileTime == 3
        && Derived1::ColsAtCompileTime == 3,
        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

  EIGEN_STATIC_ASSERT(
        Derived2::RowsAtCompileTime == 3
        && Derived2::ColsAtCompileTime == 1,
        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

  relativeTransform(
        T1.linear(), T1.translation(), T2.linear(), T2.translation(), R, t);
}

} // namespace fcl

#endif

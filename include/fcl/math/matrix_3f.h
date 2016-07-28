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

#ifndef FCL_MATRIX_3F_H
#define FCL_MATRIX_3F_H

#include "fcl/math/vec_3f.h"

namespace fcl
{

template<typename T>
using Matrix3fX = Eigen::Matrix<T, 3, 3>;

using Matrix3f = Matrix3fX<FCL_REAL>;

template <typename T>
void hat(Matrix3fX<T>& mat, const Vec3fX<T>& vec)
{
  mat << 0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0;
}

/// @brief compute the eigen vector and eigen vector of a matrix. dout is the
/// eigen values, vout is the eigen vectors
template<typename T>
void eigen(const Matrix3fX<T>& m, Vec3fX<T>& dout, Matrix3fX<T>& vout)
{
  // We assume that m is symmetric matrix.
  Eigen::SelfAdjointEigenSolver<Matrix3fX<T>> eigensolver(m);
  if (eigensolver.info() != Eigen::Success)
  {
    std::cerr << "[eigen] Failed to compute eigendecomposition.\n";
    return;
  }
  dout = eigensolver.eigenvalues();
  vout = eigensolver.eigenvectors();
}

template <typename T>
void generateCoordinateSystem(Matrix3fX<T>& axis)
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

template<typename T>
void relativeTransform(const Matrix3fX<T>& R1, const Vec3fX<T>& t1,
                       const Matrix3fX<T>& R2, const Vec3fX<T>& t2,
                       Matrix3fX<T>& R, Vec3fX<T>& t)
{
  R = R1.transpose() * R2;
  t = R1.transpose() * (t2 - t1);
}

/// @brief Class for variance matrix in 3d
class Variance3f
{
public:
  /// @brief Variation matrix
  Matrix3f Sigma;

  /// @brief Variations along the eign axes
  Vec3f sigma;

  /// @brief Matrix whose columns are eigenvectors of Sigma
  Matrix3f axis;

  Variance3f() {}

  Variance3f(const Matrix3f& S) : Sigma(S)
  {
    init();
  }

  /// @brief init the Variance
  void init() 
  {
    eigen(Sigma, sigma, axis);
  }

  /// @brief Compute the sqrt of Sigma matrix based on the eigen decomposition result, this is useful when the uncertainty matrix is initialized as a square variation matrix
  Variance3f& sqrt()
  {
    for(std::size_t i = 0; i < 3; ++i)
    {
      if(sigma[i] < 0) sigma[i] = 0;
      sigma[i] = std::sqrt(sigma[i]);
    }

    Sigma.noalias()
        =  sigma[0] * axis.col(0) * axis.col(0).transpose();
    Sigma.noalias()
        += sigma[1] * axis.col(1) * axis.col(1).transpose();
    Sigma.noalias()
        += sigma[2] * axis.col(2) * axis.col(2).transpose();

    return *this;
  }
};

}



#endif

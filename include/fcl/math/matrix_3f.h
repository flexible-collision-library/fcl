/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
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

#if FCL_HAVE_EIGEN
# include "fcl/eigen/fcl_matrix.h"
#else
# include "fcl/math/matrix_3fx.h"
#endif

namespace fcl
{

#if FCL_HAVE_EIGEN
  typedef Eigen::FclMatrix<FCL_REAL, 3> Matrix3f;
#elif FCL_HAVE_SSE
  typedef Matrix3fX<details::sse_meta_f12> Matrix3f;
#else
  typedef Matrix3fX<details::Matrix3Data<FCL_REAL> > Matrix3f;
#endif

static inline std::ostream& operator << (std::ostream& o, const Matrix3f& m)
{
  o << "[(" << m(0, 0) << " " << m(0, 1) << " " << m(0, 2) << ")("
    << m(1, 0) << " " << m(1, 1) << " " << m(1, 2) << ")(" 
    << m(2, 0) << " " << m(2, 1) << " " << m(2, 2) << ")]";
  return o;
}



/// @brief Class for variance matrix in 3d
class Variance3f
{
public:
  /// @brief Variation matrix
  Matrix3f Sigma;

  /// @brief Variations along the eign axes
  Matrix3f::U sigma[3];

  /// @brief Eigen axes of the variation matrix
  Vec3f axis[3];

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


    Sigma.setZero();
    for(std::size_t i = 0; i < 3; ++i)
    {
      for(std::size_t j = 0; j < 3; ++j)
      {
        Sigma(i, j) += sigma[0] * axis[0][i] * axis[0][j];
        Sigma(i, j) += sigma[1] * axis[1][i] * axis[1][j];
        Sigma(i, j) += sigma[2] * axis[2][i] * axis[2][j];
      }
    }

    return *this;
  }
};

}



#endif

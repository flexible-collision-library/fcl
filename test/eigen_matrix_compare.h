/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018. Toyota Research Institute
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

// This code was taken from Drake.
// https://github.com/RobotLocomotion/drake/blob/master/common/test_utilities/eigen_matrix_compare.h

#ifndef FCL_EIGEN_MATRIX_COMPARE_H
#define FCL_EIGEN_MATRIX_COMPARE_H

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Dense>
#include <gtest/gtest.h>

namespace fcl {

enum class MatrixCompareType { absolute, relative };

/**
 * Compares two matrices to determine whether they are equal to within a certain
 * threshold.
 *
 * @param m1 The first matrix to compare.
 * @param m2 The second matrix to compare.
 * @param tolerance The tolerance for determining equivalence.
 * @param compare_type Whether the tolerance is absolute or relative.
 * @return true if the two matrices are equal based on the specified tolerance.
 */
template <typename DerivedA, typename DerivedB>
::testing::AssertionResult CompareMatrices(
    const Eigen::MatrixBase<DerivedA>& m1,
    const Eigen::MatrixBase<DerivedB>& m2, double tolerance = 0.0,
    MatrixCompareType compare_type = MatrixCompareType::absolute) {
  if (m1.rows() != m2.rows() || m1.cols() != m2.cols()) {
    return ::testing::AssertionFailure()
        << "Matrix size mismatch: (" << m1.rows() << " x " << m1.cols()
        << " vs. " << m2.rows() << " x " << m2.cols() << ")";
  }

  for (int ii = 0; ii < m1.rows(); ii++) {
    for (int jj = 0; jj < m1.cols(); jj++) {
      // First handle the corner cases of positive infinity, negative infinity,
      // and NaN
      const auto both_positive_infinity =
          m1(ii, jj) == std::numeric_limits<double>::infinity() &&
              m2(ii, jj) == std::numeric_limits<double>::infinity();

      const auto both_negative_infinity =
          m1(ii, jj) == -std::numeric_limits<double>::infinity() &&
              m2(ii, jj) == -std::numeric_limits<double>::infinity();

      using std::isnan;
      const auto both_nan = isnan(m1(ii, jj)) && isnan(m2(ii, jj));

      if (both_positive_infinity || both_negative_infinity || both_nan)
        continue;

      // Check for case where one value is NaN and the other is not
      if ((isnan(m1(ii, jj)) && !isnan(m2(ii, jj))) ||
          (!isnan(m1(ii, jj)) && isnan(m2(ii, jj)))) {
        return ::testing::AssertionFailure() << "NaN missmatch at (" << ii
                                             << ", " << jj << "):\nm1 =\n"
                                             << m1 << "\nm2 =\n"
                                             << m2;
      }

      // Determine whether the difference between the two matrices is less than
      // the tolerance.
      using std::abs;
      const auto delta = abs(m1(ii, jj) - m2(ii, jj));

      if (compare_type == MatrixCompareType::absolute) {
        // Perform comparison using absolute tolerance.

        if (delta > tolerance) {
          return ::testing::AssertionFailure()
              << "Values at (" << ii << ", " << jj
              << ") exceed tolerance: " << m1(ii, jj) << " vs. "
              << m2(ii, jj) << ", diff = " << delta
              << ", tolerance = " << tolerance << "\nm1 =\n"
              << m1 << "\nm2 =\n"
              << m2 << "\ndelta=\n"
              << (m1 - m2);
        }
      } else {
        // Perform comparison using relative tolerance, see:
        // http://realtimecollisiondetection.net/blog/?p=89
        using std::max;
        const auto max_value = max(abs(m1(ii, jj)), abs(m2(ii, jj)));
        const auto relative_tolerance =
            tolerance * max(decltype(max_value){1}, max_value);

        if (delta > relative_tolerance) {
          return ::testing::AssertionFailure()
              << "Values at (" << ii << ", " << jj
              << ") exceed tolerance: " << m1(ii, jj) << " vs. "
              << m2(ii, jj) << ", diff = " << delta
              << ", tolerance = " << tolerance
              << ", relative tolerance = " << relative_tolerance
              << "\nm1 =\n"
              << m1 << "\nm2 =\n"
              << m2 << "\ndelta=\n"
              << (m1 - m2);
        }
      }
    }
  }

  return ::testing::AssertionSuccess() << "m1 =\n"
                                       << m1
                                       << "\nis approximately equal to m2 =\n"
                                       << m2;
}

}  // namespace fcl

#endif  // FCL_EIGEN_MATRIX_COMPARE_H

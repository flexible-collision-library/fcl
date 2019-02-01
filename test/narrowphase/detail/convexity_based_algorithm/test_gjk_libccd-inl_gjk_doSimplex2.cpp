/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019. Toyota Research Institute
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
 *   * Neither the name of CNRS-LAAS and AIST nor the names of its
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

/** @author Sean Curtis (sean@tri.global) */

#include "fcl/narrowphase/detail/convexity_based_algorithm/gjk_libccd-inl.h"

#include <algorithm>

#include <gtest/gtest.h>

namespace fcl {
namespace detail {
namespace libccd_extension {
namespace {

using Vector3Ccd = Vector3<ccd_real_t>;

// Generally, we assume libccd is build with ccd_real_t = double. But we express
// these functions in terms of ccd_real_t to maintain compatibility.
ccd_vec3_t eigen_to_ccd(const Vector3Ccd& vector) {
  ccd_vec3_t out;
  out.v[0] = vector(0);
  out.v[1] = vector(1);
  out.v[2] = vector(2);
  return out;
}

Vector3Ccd ccd_to_eigen(const ccd_vec3_t& vector) {
  return Vector3Ccd{vector.v};
}

class DoSimplex2Test : public ::testing::Test {
 protected:
  // Sets the simplex to points A and B and _confirms_ the validity of the
  // simplex as meeting doSimplex2()'s assumptions.
  ::testing::AssertionResult SetValidSimplex(const ccd_vec3_t& A,
                                             const ccd_vec3_t& B,
                                             bool validate = true) {
    Vector3Ccd p_OA(A.v);
    Vector3Ccd p_OB(B.v);
    // NOTE: This assertion should echo the assertion in doSimplex2().
    if (validate && p_OA.dot(p_OB) > 0) {
      return ::testing::AssertionFailure()
             << "Simplex points are not valid; A is not in region 1: "
             << "\n  p_OA: " << p_OA.transpose()
             << "\n  p_OB: " << p_OB.transpose();
    }
    line_.ps[0].v = B;
    line_.ps[1].v = A;
    // This guarantees that whatever may have happened to `line` in previous
    // tests, that it's still configured to be a 2-simplex.
    line_.last = 1;
    // Reset dir_ so that tests are independent with a recognizable magic value.
    dir_ = {-1.23, 4.56, 7.89};
    return ::testing::AssertionSuccess();
  }

  // Configures the test for a death test; defines the point B to be at the
  // given distance (`dist_OB`) from the origin in an arbitrary direction.
  // Point A is defined by a linear combination of the direction to B (phat_OB)
  // and its normal (norm_OB) as: A = u·phat_OB + v·norm_OB.
  void ConfigureDeathTest(double u, double v, double dist_OB) {
    const Vector3Ccd phat_OB = Vector3Ccd(1, -2, 3).normalized();
    const Vector3Ccd norm_OB =
        Vector3Ccd(-phat_OB(2), 0, phat_OB(0)).normalized();
    EXPECT_EQ(norm_OB.dot(phat_OB), 0);
    const Vector3Ccd p_OB = phat_OB * dist_OB;

    const Vector3Ccd p_OA = phat_OB * u + norm_OB * v;
    SetValidSimplex(eigen_to_ccd(p_OA), eigen_to_ccd(p_OB), false);
  }

  // The simplex the tests will operate on.
  ccd_simplex_t line_;
  ccd_vec3_t dir_;

  // Interpretation of doSimplex2() return values; they can't be constexpr if
  // they are referenced in GTEST macros.
  static const int kNeedMoreComputing;
  static const int kNotSeparated;

  // Epsilon to use with tests.
  static const double kEps;
};

const int DoSimplex2Test::kNeedMoreComputing = 0;
const int DoSimplex2Test::kNotSeparated = 1;
const double DoSimplex2Test::kEps = std::numeric_limits<double>::epsilon();

// Tests the case where the origin lies on the simplex -- i.e., p_AB = k·p_OB.
// The direction value is never set (so the result is never tested), but
// doSimplex2() should always report not separated.
// NOTE: This limits the values A to being valid (i.e., within Region 1).
TEST_F(DoSimplex2Test, OriginInSimplex) {
  // Create a small perturbation *slightly larger* than numerical precision.
  const double delta = kEps * 2;

  // A non-axis-aligned line direction so the tests don't get fooled by easy
  // zeros.
  const Vector3Ccd phat_OB = Vector3Ccd(1, -2, 3).normalized();
  const Vector3Ccd norm_OB =
      Vector3Ccd(-phat_OB(2), 0, phat_OB(0)).normalized();
  EXPECT_EQ(norm_OB.dot(phat_OB), 0);
  const ccd_real_t dist_OB = 3;
  const Vector3Ccd p_OB = phat_OB * dist_OB;

  // Case: A *is* the origin.
  EXPECT_TRUE(SetValidSimplex({0., 0., 0.}, eigen_to_ccd(p_OB)));
  EXPECT_EQ(kNotSeparated, doSimplex2(&line_, &dir_));

  // Case: A is beyond the origin a very small amount.
  EXPECT_TRUE(
      SetValidSimplex(eigen_to_ccd(phat_OB * -delta), eigen_to_ccd(p_OB)));
  EXPECT_EQ(kNotSeparated, doSimplex2(&line_, &dir_));

  // Case: A is beyond the origin a HUGE amount.
  EXPECT_TRUE(
      SetValidSimplex(eigen_to_ccd(phat_OB * -1e10), eigen_to_ccd(p_OB)));
  EXPECT_EQ(kNotSeparated, doSimplex2(&line_, &dir_));

  // Case: A is as far from the origin as B, but it has an epsilon perturbation
  // off the line.
  EXPECT_TRUE(SetValidSimplex(eigen_to_ccd(phat_OB * -dist_OB + norm_OB * kEps),
                              eigen_to_ccd(p_OB)));
  EXPECT_EQ(kNotSeparated, doSimplex2(&line_, &dir_));

  // Larger perturbations from co-linear, which should be categorized as
  // "needs more computing", are evaluated in the NeedMoreComputing test.
}

// Tests the case where the origin does *not* lie on the simplex. The return
// value should always be that it needs more computing, and we validate the
// direction vector:
//   dir · p_AB = 0             // perpendicular to the segment AB.
//   dir · p_OA < 0             // points in the direction towards O from AB.
//                              // Must be strictly < 0, because we exclude
//                              // co-linearity cases in this test.
//   dir · (p_OA × p_AB) = 0    // Lies on the plane defined b A, B, and O.
TEST_F(DoSimplex2Test, NeedMoreComputing) {
  auto is_valid_dir = [this](const Vector3Ccd& p_OA, const Vector3Ccd& p_OB,
                             const Vector3Ccd& dir) {
    // If p_OA or p_OB are large vectors, we need to scale EPS up to be able
    // to recognize a valid direction to machine precision.
    const ccd_real_t eps = std::max({1., p_OA.norm(), p_OB.norm()}) * kEps;
    const Vector3Ccd phat_AB = (p_OB - p_OB).normalized();
    const Vector3Ccd dir_hat = dir.normalized();
    if (std::abs(dir_hat.dot(phat_AB)) > eps) {
      return ::testing::AssertionFailure()
             << "Direction is not perpendicular to the line segments:"
             << "\n  dir: " << dir.transpose()
             << "\n  p_OA: " << p_OA.transpose()
             << "\n  p_OB: " << p_OB.transpose()
             << "\n  dir_hat.dot(phat_AB): " << dir_hat.dot(phat_AB)
             << " bigger than tolerance << " << eps;
    }
    // Note, in this case dir · p_OA < 0 is treated as dir · p_OA < ε to
    // account for numerical issues arising from scale disparity in the vectors.
    if (dir_hat.dot(p_OA) >= eps) {
      return ::testing::AssertionFailure()
             << "Direction does not point toward origin:"
             << "\n  dir: " << dir.transpose()
             << "\n  p_OA: " << p_OA.transpose()
             << "\n  p_OB: " << p_OB.transpose()
             << "\n  dir_hat.dot(p_OA): " << dir_hat.dot(p_OA)
             << "; should be negative";
    }
    if (std::abs(dir.dot(p_OA.normalized().cross(phat_AB))) > eps) {
      return ::testing::AssertionFailure()
             << "Direction does not lie on the triangle formed by OAB:"
             << "\n  dir: " << dir.transpose()
             << "\n  p_OA: " << p_OA.transpose()
             << "\n  p_OB: " << p_OB.transpose()
             << "\n  dir.dot(phat_OA.cross(phat_AB)): "
             << dir.dot(p_OA.normalized().cross(phat_AB))
             << " bigger than tolerance << " << eps;
    }
    return ::testing::AssertionSuccess();
  };

  // Create a small perturbation *slightly larger* than numerical precision.
  const double delta = kEps * 2;

  // A non-axis-aligned line direction so the tests don't get fooled by easy
  // zeros.
  const Vector3Ccd phat_OB = Vector3Ccd(1, -2, 3).normalized();
  const Vector3Ccd norm_OB =
      Vector3Ccd(-phat_OB(2), 0, phat_OB(0)).normalized();
  EXPECT_EQ(norm_OB.dot(phat_OB), 0);
  const ccd_real_t dist_OB = 3;
  const Vector3Ccd p_OB = phat_OB * dist_OB;

  // Case 1a: A is *near* co-linear.
  {
    const ccd_real_t offset = delta * 2 * dist_OB;
    const Vector3Ccd p_OA = phat_OB * -dist_OB + norm_OB * offset;
    EXPECT_TRUE(SetValidSimplex(eigen_to_ccd(p_OA), eigen_to_ccd(p_OB)));
    EXPECT_EQ(kNeedMoreComputing, doSimplex2(&line_, &dir_));
    EXPECT_TRUE(is_valid_dir(p_OA, p_OB, ccd_to_eigen(dir_)));
  }

  // Case 1b: A is *near* co-linear on the other side of AB.
  {
    const ccd_real_t offset = delta * 2 * dist_OB;
    const Vector3Ccd p_OA = phat_OB * -dist_OB - norm_OB * offset;
    EXPECT_TRUE(SetValidSimplex(eigen_to_ccd(p_OA), eigen_to_ccd(p_OB)));
    EXPECT_EQ(kNeedMoreComputing, doSimplex2(&line_, &dir_));
    EXPECT_TRUE(is_valid_dir(p_OA, p_OB, ccd_to_eigen(dir_)));
  }

  // Case 2a: A is approximately same distance to origin as B, but far off
  // the line.
  {
    const Vector3Ccd p_OA = phat_OB * -dist_OB + norm_OB * dist_OB;
    EXPECT_TRUE(SetValidSimplex(eigen_to_ccd(p_OA), eigen_to_ccd(p_OB)));
    EXPECT_EQ(kNeedMoreComputing, doSimplex2(&line_, &dir_));
    EXPECT_TRUE(is_valid_dir(p_OA, p_OB, ccd_to_eigen(dir_)));
  }

  // Case2b : A is approximately same distance to origin as B, but far off
  // the line on the other side of AB.
  {
    const Vector3Ccd p_OA = phat_OB * -dist_OB - norm_OB * dist_OB;
    EXPECT_TRUE(SetValidSimplex(eigen_to_ccd(p_OA), eigen_to_ccd(p_OB)));
    EXPECT_EQ(kNeedMoreComputing, doSimplex2(&line_, &dir_));
    EXPECT_TRUE(is_valid_dir(p_OA, p_OB, ccd_to_eigen(dir_)));
  }

  // Case 3a: A is far away from the origin but, relatively, a small distance
  // off the line.
  {
    const Vector3Ccd p_OA = phat_OB * -1e10 + norm_OB * dist_OB;
    EXPECT_TRUE(SetValidSimplex(eigen_to_ccd(p_OA), eigen_to_ccd(p_OB)));
    EXPECT_EQ(kNeedMoreComputing, doSimplex2(&line_, &dir_));
    EXPECT_TRUE(is_valid_dir(p_OA, p_OB, ccd_to_eigen(dir_)));
  }

  // Case 3b: A is far away from the origin but, relatively, a small distance
  // off the line on the other side of AB.
  {
    const Vector3Ccd p_OA = phat_OB * -1e10 - norm_OB * dist_OB;
    EXPECT_TRUE(SetValidSimplex(eigen_to_ccd(p_OA), eigen_to_ccd(p_OB)));
    EXPECT_EQ(kNeedMoreComputing, doSimplex2(&line_, &dir_));
    EXPECT_TRUE(is_valid_dir(p_OA, p_OB, ccd_to_eigen(dir_)));
  }
}

#ifndef NDEBUG

// These test the various conditions under which the assertion in doSimplex2()
// determines that the simplex is ill-defined (in that point A lies in region 2
// or region 3). See documentation for doSimplex2() for details on these
// regions. There are a number of cases, each with a single failure instance.
// Exercised via an assertion, so only tested in debug mode.

// Test an A that is co-linear with O and B, but only barely in region 1.
TEST_F(DoSimplex2Test, Region1Boundary1) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  ConfigureDeathTest(kEps, 0, 3.);

  ASSERT_DEATH(doSimplex2(&line_, &dir_),
               ".*Assertion `p_OA.*dot\\(p_OB.*\\) <= 0' failed.*");
}

// Test an A that is barely in region 1, but as far removed from O as B.
TEST_F(DoSimplex2Test, Region1Boundary2) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  const double dist_OB = 3.;
  const double dist_OA = dist_OB;
  // As A gets farther from the O, the distance to the Region1 boundary needs
  // to increase to be detectable; less than this scaled epsilon is considered
  // *on the boundary*.
  ConfigureDeathTest(dist_OA * kEps, dist_OA, dist_OB);

  ASSERT_DEATH(doSimplex2(&line_, &dir_),
               ".*Assertion `p_OA.*dot\\(p_OB.*\\) <= 0' failed.*");
}

// Test an A that is barely in region 1, but far removed from O.
TEST_F(DoSimplex2Test, Region1Boundary3) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  const double dist_OB = 3.;
  const double dist_OA = 100 * dist_OB;
  // As A gets farther from the O, the distance to the Region1 boundary needs
  // to increase to be detectable; less than this scaled epsilon is considered
  // *on the boundary*.
  ConfigureDeathTest(dist_OA * kEps, dist_OA, dist_OB);

  ASSERT_DEATH(doSimplex2(&line_, &dir_),
               ".*Assertion `p_OA.*dot\\(p_OB.*\\) <= 0' failed.*");
}

#endif  // !NDEBUG

}  // namespace
}  // namespace libccd_extension
}  // namespace detail
}  // namespace fcl

//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

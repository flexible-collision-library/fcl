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
  void SetUp() override {
    // A non-axis-aligned line direction so the tests don't get fooled by easy
    // zeros.
    phat_OB_ = Vector3Ccd(1, -2, 3).normalized();
    norm_OB_ =
        Vector3Ccd(-phat_OB_(2), 0, phat_OB_(0)).normalized();
    EXPECT_EQ(norm_OB_.dot(phat_OB_), 0);
    dist_OB_ = 3;
    p_OB_ = phat_OB_ * dist_OB_;
  }

  // Sets the simplex to points A and B and _confirms_ the validity of the
  // simplex as meeting doSimplex2()'s assumptions.
  ::testing::AssertionResult SetValidSimplex(const ccd_vec3_t& A,
                                             const ccd_vec3_t& B,
                                             bool validate = true) {
    const Vector3Ccd p_OA(A.v);
    const Vector3Ccd p_OB(B.v);
    const ccd_real_t eps = constants<ccd_real_t>::eps();
    // NOTE: This assertion should echo the assertion in doSimplex2().
    if (validate && p_OA.dot(p_OB) > p_OB.squaredNorm() * eps) {
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
    dir_ = {{-1.23, 4.56, 7.89}};
    return ::testing::AssertionSuccess();
  }

  // Configures the test for a death test; given the test class's definition
  // of p_OB, defines point A as a linear combination of the direction to B
  // (phat_OB) and its a direction perpendicular to that direction (norm_OB) as:
  // A = u·phat_OB + v·norm_OB. For death tests, this skips testing that the
  // defined point A is in Region 1.
  void ConfigureDeathTest(double u, double v) {
    const Vector3Ccd p_OA = phat_OB_ * u + norm_OB_ * v;
    SetValidSimplex(eigen_to_ccd(p_OA), eigen_to_ccd(p_OB_), false);
  }

  // The simplex the tests will operate on.
  ccd_simplex_t line_;
  ccd_vec3_t dir_;

  // Configuration of O and B (tests configure A).
  Vector3Ccd phat_OB_;
  Vector3Ccd norm_OB_;
  ccd_real_t dist_OB_;
  Vector3Ccd p_OB_;

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

  // Case: A *is* the origin.
  EXPECT_TRUE(SetValidSimplex({{0., 0., 0.}}, eigen_to_ccd(p_OB_)));
  EXPECT_EQ(kNotSeparated, doSimplex2(&line_, &dir_));

  // Case: A is beyond the origin a very small amount.
  EXPECT_TRUE(
      SetValidSimplex(eigen_to_ccd(phat_OB_ * -delta), eigen_to_ccd(p_OB_)));
  EXPECT_EQ(kNotSeparated, doSimplex2(&line_, &dir_));

  // Case: A is beyond the origin a HUGE amount.
  EXPECT_TRUE(
      SetValidSimplex(eigen_to_ccd(phat_OB_ * -1e10), eigen_to_ccd(p_OB_)));
  EXPECT_EQ(kNotSeparated, doSimplex2(&line_, &dir_));

  // Case: A is as far from the origin as B, but it has an epsilon perturbation
  // off the line.
  EXPECT_TRUE(
      SetValidSimplex(eigen_to_ccd(phat_OB_ * -dist_OB_ + norm_OB_ * kEps),
                      eigen_to_ccd(p_OB_)));
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
//   dir · (p_OA × p_AB) = 0    // Lies on the plane defined by A, B, and O.
TEST_F(DoSimplex2Test, NeedMoreComputing) {
  auto is_valid_dir = [](const Vector3Ccd& p_OA, const Vector3Ccd& p_OB,
                         const Vector3Ccd& dir) {
    // If p_OA or p_OB are large vectors, we need to scale EPS up to be able
    // to recognize a valid direction to machine precision.
    const ccd_real_t eps =
        std::max({ccd_real_t(1), p_OA.norm(), p_OB.norm()}) * kEps;
    const Vector3Ccd phat_AB = (p_OB - p_OA).normalized();
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
             << "Direction does not lie on the plane formed by OAB:"
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

  // Case 1a: A is *near* co-linear.
  {
    const ccd_real_t offset = delta * 2 * dist_OB_;
    const Vector3Ccd p_OA = phat_OB_ * -dist_OB_ + norm_OB_ * offset;
    EXPECT_TRUE(SetValidSimplex(eigen_to_ccd(p_OA), eigen_to_ccd(p_OB_)));
    EXPECT_EQ(kNeedMoreComputing, doSimplex2(&line_, &dir_));
    EXPECT_TRUE(is_valid_dir(p_OA, p_OB_, ccd_to_eigen(dir_)));
  }

  // Case 1b: A is *near* co-linear on the other side of AB.
  {
    const ccd_real_t offset = delta * 2 * dist_OB_;
    const Vector3Ccd p_OA = phat_OB_ * -dist_OB_ - norm_OB_ * offset;
    EXPECT_TRUE(SetValidSimplex(eigen_to_ccd(p_OA), eigen_to_ccd(p_OB_)));
    EXPECT_EQ(kNeedMoreComputing, doSimplex2(&line_, &dir_));
    EXPECT_TRUE(is_valid_dir(p_OA, p_OB_, ccd_to_eigen(dir_)));
  }

  // Case 2a: A is approximately same distance to origin as B, but far off
  // the line.
  {
    const Vector3Ccd p_OA = phat_OB_ * -dist_OB_ + norm_OB_ * dist_OB_;
    EXPECT_TRUE(SetValidSimplex(eigen_to_ccd(p_OA), eigen_to_ccd(p_OB_)));
    EXPECT_EQ(kNeedMoreComputing, doSimplex2(&line_, &dir_));
    EXPECT_TRUE(is_valid_dir(p_OA, p_OB_, ccd_to_eigen(dir_)));
  }

  // Case2b : A is approximately same distance to origin as B, but far off
  // the line on the other side of AB.
  {
    const Vector3Ccd p_OA = phat_OB_ * -dist_OB_ - norm_OB_ * dist_OB_;
    EXPECT_TRUE(SetValidSimplex(eigen_to_ccd(p_OA), eigen_to_ccd(p_OB_)));
    EXPECT_EQ(kNeedMoreComputing, doSimplex2(&line_, &dir_));
    EXPECT_TRUE(is_valid_dir(p_OA, p_OB_, ccd_to_eigen(dir_)));
  }

  // Case 3a: A is far away from the origin but, relatively, a small distance
  // off the line.
  {
    const Vector3Ccd p_OA = phat_OB_ * -1e10 + norm_OB_ * dist_OB_;
    EXPECT_TRUE(SetValidSimplex(eigen_to_ccd(p_OA), eigen_to_ccd(p_OB_)));
    EXPECT_EQ(kNeedMoreComputing, doSimplex2(&line_, &dir_));
    EXPECT_TRUE(is_valid_dir(p_OA, p_OB_, ccd_to_eigen(dir_)));
  }

  // Case 3b: A is far away from the origin but, relatively, a small distance
  // off the line on the other side of AB.
  {
    const Vector3Ccd p_OA = phat_OB_ * -1e10 - norm_OB_ * dist_OB_;
    EXPECT_TRUE(SetValidSimplex(eigen_to_ccd(p_OA), eigen_to_ccd(p_OB_)));
    EXPECT_EQ(kNeedMoreComputing, doSimplex2(&line_, &dir_));
    EXPECT_TRUE(is_valid_dir(p_OA, p_OB_, ccd_to_eigen(dir_)));
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
  ConfigureDeathTest(dist_OB_ * 2 * kEps, 0);

  ASSERT_DEATH(doSimplex2(&line_, &dir_),
               ".*Assertion.*"
               "p_OA.dot\\(p_OB\\) <= p_OB.squaredNorm\\(\\) \\* eps.*");
}

// Test an A that is barely in region 1, but as far removed from O as B.
TEST_F(DoSimplex2Test, Region1Boundary2) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  const double dist_OA = dist_OB_;
  // As A gets farther from the O, the distance to the Region1 boundary needs
  // to increase to be detectable; less than this scaled epsilon is considered
  // *on the boundary*.
  ConfigureDeathTest(dist_OA * 2 * kEps, dist_OA);

  ASSERT_DEATH(doSimplex2(&line_, &dir_),
               ".*Assertion.*"
               "p_OA.dot\\(p_OB\\) <= p_OB.squaredNorm\\(\\) \\* eps.*");
}

// Test an A that is barely in region 1, but far removed from O.
TEST_F(DoSimplex2Test, Region1Boundary3) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  const double dist_OA = 100 * dist_OB_;
  // As A gets farther from the O, the distance to the Region1 boundary needs
  // to increase to be detectable; less than this scaled epsilon is considered
  // *on the boundary*.
  ConfigureDeathTest(dist_OA * 2 * kEps, dist_OA);

  ASSERT_DEATH(doSimplex2(&line_, &dir_),
               ".*Assertion.*"
               "p_OA.dot\\(p_OB\\) <= p_OB.squaredNorm\\(\\) \\* eps.*");
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

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Toyota Research Institute
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

/** @author Sean Curtis */

#include "fcl/math/constants.h"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>
#include <unsupported/Eigen/AutoDiff>

namespace fcl {
namespace {

// Some autodiff helpers
template <typename S>
using Vector2 = Eigen::Matrix<S, 2, 1>;
template <typename S>
using AutoDiff2 = Eigen::AutoDiffScalar<Vector2<S>>;

// Utility function for confirming that the value returned by `constants` is
// the expected value based on the scalar type.
template<typename S>
void expect_eps_values(const char* type_name) {
  static_assert(std::is_floating_point<S>::value,
                "Only use this helper for float and double types");
  S expected_eps = std::numeric_limits<S>::epsilon();
  // This is *explicitly* testing for perfect bit equivalency. The two values
  // should be absolutely the same.
  EXPECT_EQ(constants<S>::eps(), expected_eps) << "Failed for " << type_name;
  EXPECT_EQ(std::pow(expected_eps, S(0.5)), constants<S>::eps_12())
            << "Failed for " << type_name;
  EXPECT_EQ(std::pow(expected_eps, S(0.75)), constants<S>::eps_34())
            << "Failed for " << type_name;
  EXPECT_EQ(std::pow(expected_eps, S(0.875)), constants<S>::eps_78())
            << "Failed for " << type_name;
}

// Test that the values returned are truly a function of the precision of the
// underlying type.
GTEST_TEST(FCL_CONSTANTS_EPS, precision_dependent) {
  expect_eps_values<double>("double");
  expect_eps_values<float>("float");
  // Double check that the float value and double values are *not* equal.
  EXPECT_NE(constantsd::eps(), constantsf::eps());
  EXPECT_NE(constantsd::eps_12(), constantsf::eps_12());
  EXPECT_NE(constantsd::eps_34(), constantsf::eps_34());
  EXPECT_NE(constantsd::eps_78(), constantsf::eps_78());
}

template <typename S> void expect_autodiff_constants(const char *type_name) {
  EXPECT_TRUE((std::is_same<decltype(constants<AutoDiff2<S>>::pi()),
                            AutoDiff2<S>>::value))
      << "Failed for " << type_name;
  EXPECT_TRUE((std::is_same<decltype(constants<AutoDiff2<S>>::phi()),
                            AutoDiff2<S>>::value))
      << "Failed for " << type_name;
  EXPECT_TRUE(
      (std::is_same<decltype(constants<AutoDiff2<S>>::eps()), S>::value))
      << "Failed for " << type_name;
  EXPECT_TRUE(
      (std::is_same<decltype(constants<AutoDiff2<S>>::eps_78()), S>::value))
      << "Failed for " << type_name;
  EXPECT_TRUE(
      (std::is_same<decltype(constants<AutoDiff2<S>>::eps_34()), S>::value))
      << "Failed for " << type_name;
  EXPECT_TRUE(
      (std::is_same<decltype(constants<AutoDiff2<S>>::eps_12()), S>::value))
      << "Failed for " << type_name;
}

// Test the types returned by constants. pi and phi should return autodiff, but
// the tolerances should return real types.
GTEST_TEST(FCL_CONSTANTS_EPS, autodiff_compatibility) {
  expect_autodiff_constants<double>("double");
  expect_autodiff_constants<float>("float");
}

}  // namespace
}  // namespace fcl

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

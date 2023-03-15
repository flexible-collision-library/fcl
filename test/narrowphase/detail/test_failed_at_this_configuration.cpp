/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023. Toyota Research Institute
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

/** @author Sean Curtis (sean@tri.global) (2023) */

#include <gtest/gtest.h>

#include <regex>

#include "fcl/geometry/shape/box.h"
#include "fcl/geometry/shape/sphere.h"
#include "fcl/narrowphase/detail/failed_at_this_configuration.h"
#include "fcl/narrowphase/detail/gjk_solver_libccd.h"

namespace fcl {
namespace detail {
namespace {

// We're testing for the following:
//
//   1. The exception message is included.
//   2. The shapes have had representation() called on them. We'll use two
//      different shapes to confirm they get ordered correctly.
//   3. The first matrix is printed with the first shape, the second with the
//      second.
//   4. The matrices have commas.
GTEST_TEST(ConfigurationFailureTest, ConfirmFormatting) {
  const Sphered sphere(1.5);
  const Transform3d X_WS(Translation3d(12, 13, 14));
  const Boxd box(1, 2, 3);
  const Transform3d X_WB(Translation3d(17, 18, 19));
  const GJKSolver_libccd<double> solver;
  const std::logic_error e("dummy message");

  try {
    ThrowDetailedConfiguration(sphere, X_WS, box, X_WB, solver, e);
  } catch (const std::logic_error& e) {
    EXPECT_TRUE(
        std::regex_search(e.what(), std::regex("[^]+dummy message[^]+")));

    // Shape<S> is evidence that representation() got called.
    std::regex ordered_shapes_re("[^]+Sphere<double>[^]+Box<double>[^]+");
    EXPECT_TRUE(std::regex_search(e.what(), ordered_shapes_re)) << e.what();

    // Commas and order; we're looking for the translation values in order
    // following commas.
    std::regex matrices_re(
        "[^]+X_FS1[^]+, 12,[^]+, 13,[^]+, 14,[^]+, 1;"
        "[^]+X_FS2[^]+, 17,[^]+, 18,[^]+, 19,[^]+, 1;");
    EXPECT_TRUE(std::regex_search(e.what(), matrices_re)) << e.what();
  }
}

}  // namespace
}  // namespace detail
}  // namespace fcl

//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

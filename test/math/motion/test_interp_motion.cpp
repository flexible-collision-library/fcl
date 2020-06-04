/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020. Toyota Research Institute
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

/** @author Sean Curtis (sean@tri.global) (2020) */

#include "fcl/math/motion/interp_motion.h"

#include <gtest/gtest.h>

#include "eigen_matrix_compare.h"

namespace fcl {
namespace {

// TODO(SeanCurtis-TRI): Convert this to a parameterized test harness and use
// TEST_P.

template <typename S>
class InterpMotionTest : public ::testing::Test {};

typedef testing::Types<double, float> MyTypes;
TYPED_TEST_CASE(InterpMotionTest, MyTypes);

TYPED_TEST(InterpMotionTest, Construction) {
  using S = TypeParam;
  const Transform3<S> I = Transform3<S>::Identity();

  {
    // Default constructor; everything should be identity.
    InterpMotion<S> motion{};

    Transform3<S> X_FA;
    motion.getCurrentTransform(X_FA);
    EXPECT_TRUE(CompareMatrices(X_FA.matrix(), I.matrix()));
    EXPECT_TRUE(
        CompareMatrices(motion.getLinearVelocity(), Vector3<S>::Zero()));
    EXPECT_EQ(motion.getAngularVelocity(), S(0));
    EXPECT_TRUE(CompareMatrices(motion.getAngularAxis(), Vector3<S>::UnitX()));
    EXPECT_TRUE(
        CompareMatrices(motion.getReferencePoint(), Vector3<S>::Zero()));
  }

  {
    // Construct from start (R_FS, p_FSo) and goal (R_FG, p_FGo).
  }

  {
    // Construct from start X_FS and goal X_FG.
  }

  {
    // Construct from start (R_FS, p_FSo), goal (R_FG, p_FGo), and an origin of
    //   rotation (p_FO).
  }

  {
    // Construct from start X_FS, goal X_FG, and rotation origin p_FO.
  }
}

}  // namespace
}  // namespace fcl

//==============================================================================
int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

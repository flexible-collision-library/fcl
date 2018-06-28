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

/** @author Sean Curtis*/

#include "fcl/narrowphase/detail/primitive_shape_algorithm/box_box-inl.h"

#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "fcl/narrowphase/contact_point.h"

namespace fcl {
namespace detail {

// Dummy test.
GTEST_TEST(BoxBoxPrimitive, Test1) {
  const Vector3<double> side1(60, 59.0353, 59.0353);
  Transform3<double> X_WB1;
  X_WB1.translation() << -195.084, -17.8274, 24.9477;
  X_WB1.linear() << -0.872258, -0.487098, 0.0436128,
                     0.338058, -0.664996, -0.665955,
                     0.353388, -0.566141,  0.744716;

  const Vector3<double> side2(204.8, 204.8, 204.8);
  Transform3<double> X_WB2;
  X_WB2.translation() <<  -307.2, -102.4, -102.4;
  X_WB2.linear() = Matrix3<double>::Identity();

  int return_code{-10};
  Vector3<double> normal{0, 0, 0};
  double depth{0};
  std::vector<ContactPoint<double>> contacts;

  boxBox2(side1, X_WB1.linear(), X_WB1.translation(),
          side2, X_WB2.linear(), X_WB2.translation(),
          normal, &depth, &return_code, 20, contacts, false /* verbose */);
  EXPECT_EQ(return_code, 8);
}

}  // namespace detail
}  // namespace fcl



//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

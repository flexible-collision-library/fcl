/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
 *  Copyright (c) 2016, Toyota Research Institute
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

/** @author Jeongseok Lee */

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <unsupported/Eigen/AutoDiff>
#include "fcl/narrowphase/distance.h"

using namespace fcl;

//==============================================================================
template <typename S>
S getDistance(const Vector3<S>& p)
{
  detail::GJKSolver_libccd<S> solver;

  S dist;

  Sphere<S> s1(20);
  Sphere<S> s2(10);

  Transform3<S> tf1 = Transform3<S>::Identity();
  Transform3<S> tf2 = Transform3<S>::Identity();

  tf2.translation() = p;

  solver.shapeDistance(s1, tf1, s2, tf2, &dist);

  return dist;
}

//==============================================================================
template <typename S>
void test_basic()
{
  using derivative_t = Eigen::Matrix<S, 3, 1>;
  using scalar_t = Eigen::AutoDiffScalar<derivative_t>;
  using input_t = Eigen::Matrix<scalar_t, 3, 1>;

  input_t pos(40, 0, 0);
  pos(0).derivatives() = derivative_t::Unit(3, 0);
  pos(1).derivatives() = derivative_t::Unit(3, 1);
  pos(2).derivatives() = derivative_t::Unit(3, 2);

  auto dist = getDistance(pos);
  EXPECT_EQ(dist, (S)10);
  EXPECT_EQ(dist.value(), (S)10);
  EXPECT_EQ(dist.derivatives(), Vector3<S>(1, 0, 0));

  pos << 0, 40, 0;
  pos(0).derivatives() = derivative_t::Unit(3, 0);
  pos(1).derivatives() = derivative_t::Unit(3, 1);
  pos(2).derivatives() = derivative_t::Unit(3, 2);
  dist = getDistance(pos);
  EXPECT_EQ(dist, (S)10);
  EXPECT_EQ(dist.value(), (S)10);
  EXPECT_EQ(dist.derivatives(), Vector3<S>(0, 1, 0));

  pos << 0, 0, 40;
  pos(0).derivatives() = derivative_t::Unit(3, 0);
  pos(1).derivatives() = derivative_t::Unit(3, 1);
  pos(2).derivatives() = derivative_t::Unit(3, 2);
  dist = getDistance(pos);
  EXPECT_EQ(dist, (S)10);
  EXPECT_EQ(dist.value(), (S)10);
  EXPECT_EQ(dist.derivatives(), Vector3<S>(0, 0, 1));
}

//==============================================================================
GTEST_TEST(FCL_AUTO_DIFF, basic)
{
//  test_basic<float>();
  test_basic<double>();
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

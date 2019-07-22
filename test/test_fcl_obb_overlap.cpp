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

#include <gtest/gtest.h>

#include "fcl/common/types.h"
#include "fcl/math/bv/OBB.h"

using namespace fcl;

template <typename S>
void testObbDisjoint()
{
  fcl::Matrix3<S> B;
  fcl::Vector3<S> T;
  fcl::Vector3<S> a;
  fcl::Vector3<S> b;

  // NOTE: The following hard-coded values were extracted from real application.
  // The results from the original implementation is recorded and considered to be the ground truth.

  {
    B << 0.455308, 0.770672, 0.445824, -0.701088, 0.618991, -0.354014, -0.548789, -0.151377, 0.822141;
    T << -0.41102, 0.449435, 1.02396;
    a << 0.045, 0.615, 0.615;
    b << 0.39866, 0.124083, 0.117934;
    EXPECT_TRUE(fcl::obbDisjoint(B, T, a, b));
  }

  {
    B << 0.559316, 0.363178, 0.745163, -0.437684, 0.892787, -0.106603, -0.703987, -0.266521, 0.658307;
    T << -0.350612, 0.857327, 0.9279;
    a << 0.045, 0.615, 0.615;
    b << 0.39866, 0.124083, 0.117934;
    EXPECT_TRUE(fcl::obbDisjoint(B, T, a, b));
  }

  {
    B << 0.871089, -0.160559, -0.464138, -0.281678, -0.937498, -0.204341, -0.40232, 0.308737, -0.86187;
    T << 0.356551, 0.848477, 0.95376;
    a << 0.045, 0.615, 0.615;
    b << 0.39866, 0.124083, 0.117934;
    EXPECT_TRUE(fcl::obbDisjoint(B, T, a, b));
  }

  {
    B << 0.460642, 0.720323, 0.518597, 0.887536, -0.367624, -0.277726, -0.00940369, 0.588206, -0.808656;
    T << -0.385138, 0.74875, 0.807548;
    a << 0.045, 0.615, 0.615;
    b << 0.245512, 0.221752, 0.173233;
    EXPECT_TRUE(fcl::obbDisjoint(B, T, a, b));
  }

  {
    B << 0.195341, 0.942717, -0.270419, 0.756619, -0.320295, -0.570034, -0.623995, -0.0932533, -0.775844;
    T << -0.346916, 0.318086, 0.772772;
    a << 0.045, 0.615, 0.615;
    b << 0.245512, 0.221752, 0.173233;
    EXPECT_TRUE(fcl::obbDisjoint(B, T, a, b));
  }

  {
    B << 0.70142, 0.635512, 0.322699, 0.523294, -0.151779, -0.838527, -0.483915, 0.757026, -0.43902;
    T << -0.281496, -0.221942, 0.474881;
    a << 0.045, 0.615, 0.615;
    b << 0.245512, 0.221752, 0.173233;
    EXPECT_FALSE(fcl::obbDisjoint(B, T, a, b));
  }
}

GTEST_TEST(FCL_OBBOVERLAP, testObbDisjoint)
{
  testObbDisjoint<float>();
  testObbDisjoint<double>();
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

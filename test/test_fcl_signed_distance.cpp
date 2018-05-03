/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Open Source Robotics Foundation
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

#include "fcl/narrowphase/distance.h"
#include "fcl/narrowphase/detail/traversal/collision_node.h"
#include "fcl/narrowphase/detail/gjk_solver_libccd.h"
#include "test_fcl_utility.h"
#include "fcl_resources/config.h"

using namespace fcl;

bool verbose = false;

//==============================================================================
template <typename S>
void test_distance_spheresphere(GJKSolverType solver_type)
{
  Sphere<S> s1{20};
  Sphere<S> s2{10};

  Transform3<S> tf1{Transform3<S>::Identity()};
  Transform3<S> tf2{Transform3<S>::Identity()};

  DistanceRequest<S> request;
  request.enable_signed_distance = true;
  request.enable_nearest_points = true;
  request.gjk_solver_type = solver_type;
  request.distance_tolerance = 1e-14;

  DistanceResult<S> result;

  bool res{false};

  // Expecting distance to be 10
  result.clear();
  tf2.translation() = Vector3<S>(40, 0, 0);
  res = distance(&s1, tf1, &s2, tf2, request, result);
  EXPECT_TRUE(res);
  EXPECT_NEAR(result.min_distance, 10, 1e-6);
  EXPECT_NEAR(result.nearest_points[0](0), 20, 1e-6);
  EXPECT_NEAR(result.nearest_points[0](1),  0, 1e-6);
  EXPECT_NEAR(result.nearest_points[0](2),  0, 1e-6);
  EXPECT_NEAR(result.nearest_points[1](0), 30, 1e-6);
  EXPECT_NEAR(result.nearest_points[1](1),  0, 1e-6);
  EXPECT_NEAR(result.nearest_points[1](2),  0, 1e-6);

  // Expecting distance to be -5
  result.clear();
  tf2.translation() = Vector3<S>(25, 0, 0);
  res = distance(&s1, tf1, &s2, tf2, request, result);

  EXPECT_TRUE(res);
  EXPECT_NEAR(result.min_distance, -5, 1e-2);
  // TODO(JS): The negative distance computation using libccd requires
  // unnecessarily high error tolerance.

  // TODO(JS): Only GST_LIBCCD can compute the pair of nearest points on the
  // surface of the spheres.
  if (solver_type == GST_LIBCCD)
  {
    EXPECT_NEAR(result.nearest_points[0](0), 20, 1e-6);
    EXPECT_NEAR(result.nearest_points[0](1),  0, 1e-6);
    EXPECT_NEAR(result.nearest_points[0](2),  0, 1e-6);
    EXPECT_NEAR(result.nearest_points[1](0), 15, 1e-6);
    EXPECT_NEAR(result.nearest_points[1](1),  0, 1e-6);
    EXPECT_NEAR(result.nearest_points[1](2),  0, 1e-6);
  }

  // Expecting distance to be -1.715728753
  result.clear();
  tf2.translation() = Vector3<S>(20, 0, 20);
  res = distance(&s1, tf1, &s2, tf2, request, result);

  EXPECT_TRUE(res);
  const S expected_distance = tf2.translation().norm() - 30;
  EXPECT_NEAR(result.min_distance, expected_distance, 1e-6);
  // TODO(JS): The negative distance computation using libccd requires
  // unnecessarily high error tolerance.

  // TODO(JS): Only GST_LIBCCD can compute the pair of nearest points on the
  // surface of the spheres.
  if (solver_type == GST_LIBCCD)
  {
    const S xz0 = std::sqrt((20 * 20) / 2);
    const S xz1 = 20 - std::sqrt((10 * 10) / 2);
    EXPECT_NEAR(result.nearest_points[0](0), xz0, 1e-3);
    EXPECT_NEAR(result.nearest_points[0](1),   0, 1e-3);
    EXPECT_NEAR(result.nearest_points[0](2), xz0, 1e-3);
    EXPECT_NEAR(result.nearest_points[1](0), xz1, 1e-3);
    EXPECT_NEAR(result.nearest_points[1](1),   0, 1e-3);
    EXPECT_NEAR(result.nearest_points[1](2), xz1, 1e-3);
  }
}

template <typename S>
void test_distance_spherecapsule(GJKSolverType solver_type)
{
  Sphere<S> s1{20};
  Capsule<S> s2{10, 20};

  Transform3<S> tf1{Transform3<S>::Identity()};
  Transform3<S> tf2{Transform3<S>::Identity()};

  DistanceRequest<S> request;
  request.enable_signed_distance = true;
  request.enable_nearest_points = true;
  request.gjk_solver_type = solver_type;

  DistanceResult<S> result;

  bool res{false};

  // Expecting distance to be 10
  result.clear();
  tf2.translation() = Vector3<S>(40, 0, 0);
  res = distance(&s1, tf1, &s2, tf2, request, result);
  EXPECT_TRUE(res);
  EXPECT_NEAR(result.min_distance, 10, 1e-6);
  EXPECT_NEAR(result.nearest_points[0](0), 20, 1e-3);
  EXPECT_NEAR(result.nearest_points[0](1),  0, 1e-3);
  EXPECT_NEAR(result.nearest_points[0](2),  0, 1e-3);
  EXPECT_NEAR(result.nearest_points[1](0), 30, 1e-3);
  EXPECT_NEAR(result.nearest_points[1](1),  0, 1e-3);
  EXPECT_NEAR(result.nearest_points[1](2),  0, 1e-3);

  // Expecting distance to be -5
  result.clear();
  tf2.translation() = Vector3<S>(25, 0, 0);
  res = distance(&s1, tf1, &s2, tf2, request, result);

  EXPECT_TRUE(res);
  EXPECT_NEAR(result.min_distance, -5, 1e-2);
  // TODO(JS): The negative distance computation using libccd requires
  // unnecessarily high error tolerance.

  // TODO(JS): Only GST_LIBCCD can compute the pair of nearest points on the
  // surface of the spheres.
  if (solver_type == GST_LIBCCD)
  {
    EXPECT_NEAR(result.nearest_points[0](0), 20, 1e-6);
    EXPECT_NEAR(result.nearest_points[0](1),  0, 1e-6);
    EXPECT_NEAR(result.nearest_points[0](2),  0, 1e-6);
    EXPECT_NEAR(result.nearest_points[1](0), 15, 1e-6);
    EXPECT_NEAR(result.nearest_points[1](1),  0, 1e-6);
    EXPECT_NEAR(result.nearest_points[1](2),  0, 1e-6);
  }
}

//==============================================================================
GTEST_TEST(FCL_NEGATIVE_DISTANCE, sphere_sphere)
{
  test_distance_spheresphere<double>(GST_LIBCCD);
  test_distance_spheresphere<double>(GST_INDEP);

  test_distance_spherecapsule<double>(GST_LIBCCD);
  test_distance_spherecapsule<double>(GST_INDEP);
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

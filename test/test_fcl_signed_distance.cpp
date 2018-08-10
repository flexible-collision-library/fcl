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

#include "eigen_matrix_compare.h"
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
  const S radius_1 = 20;
  const S radius_2 = 10;
  Sphere<S> s1{radius_1};
  Sphere<S> s2{radius_2};

  Transform3<S> tf1{Transform3<S>::Identity()};
  Transform3<S> tf2{Transform3<S>::Identity()};

  DistanceRequest<S> request;
  request.enable_signed_distance = true;
  request.enable_nearest_points = true;
  request.gjk_solver_type = solver_type;

  DistanceResult<S> result;

  // Expecting distance to be 10
  result.clear();
  tf2.translation() = Vector3<S>(40, 0, 0);
  distance(&s1, tf1, &s2, tf2, request, result);
  EXPECT_NEAR(result.min_distance, 10, 1e-6);
  EXPECT_TRUE(CompareMatrices(result.nearest_points[0], Vector3<S>(20, 0, 0),
                              request.distance_tolerance));
  EXPECT_TRUE(CompareMatrices(result.nearest_points[1], Vector3<S>(30, 0, 0),
                              request.distance_tolerance));

  // request.distance_tolerance is actually the square of the distance
  // tolerance, namely the difference between distance returned from FCL's EPA
  // implementation and the actual distance, is less than
  // sqrt(request.distance_tolerance).
  const S distance_tolerance = std::sqrt(request.distance_tolerance);

  // Expecting distance to be -5
  result.clear();
  tf2.translation() = Vector3<S>(25, 0, 0);
  distance(&s1, tf1, &s2, tf2, request, result);
  EXPECT_NEAR(result.min_distance, -5, request.distance_tolerance);

  // TODO(JS): Only GST_LIBCCD can compute the pair of nearest points on the
  // surface of the penetrating spheres.
  if (solver_type == GST_LIBCCD)
  {
    EXPECT_TRUE(CompareMatrices(result.nearest_points[0], Vector3<S>(20, 0, 0),
                                distance_tolerance));
    EXPECT_TRUE(CompareMatrices(result.nearest_points[1], Vector3<S>(15, 0, 0),
                                distance_tolerance));
  }

  result.clear();
  tf2.translation() = Vector3<S>(20, 0, 20);
  distance(&s1, tf1, &s2, tf2, request, result);

  S expected_dist =
      (tf1.translation() - tf2.translation()).norm() - radius_1 - radius_2;
  EXPECT_NEAR(result.min_distance, expected_dist, distance_tolerance);
  // TODO(JS): Only GST_LIBCCD can compute the pair of nearest points on the
  // surface of the spheres.
  if (solver_type == GST_LIBCCD)
  {
    Vector3<S> dir = (tf2.translation() - tf1.translation()).normalized();
    Vector3<S> p0_expected = dir * radius_1;
    EXPECT_TRUE(CompareMatrices(result.nearest_points[0], p0_expected,
                                distance_tolerance));
    Vector3<S> p1_expected = tf2.translation() - dir * radius_2;
    EXPECT_TRUE(CompareMatrices(result.nearest_points[1], p1_expected,
                                distance_tolerance));
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

  // Expecting distance to be 10
  result.clear();
  tf2.translation() = Vector3<S>(40, 0, 0);
  distance(&s1, tf1, &s2, tf2, request, result);
  EXPECT_NEAR(result.min_distance, 10, request.distance_tolerance);
  EXPECT_TRUE(CompareMatrices(result.nearest_points[0], Vector3<S>(20, 0, 0),
                              request.distance_tolerance));
  EXPECT_TRUE(CompareMatrices(result.nearest_points[1], Vector3<S>(30, 0, 0),
                              request.distance_tolerance));

  // Expecting distance to be -5
  result.clear();
  tf2.translation() = Vector3<S>(25, 0, 0);
  distance(&s1, tf1, &s2, tf2, request, result);

  // request.distance_tolerance is actually the square of the distance
  // tolerance, namely the difference between distance returned from FCL's EPA
  // implementation and the actual distance, is less than
  // sqrt(request.distance_tolerance).
  const S distance_tolerance = std::sqrt(request.distance_tolerance);
  ASSERT_NEAR(result.min_distance, -5, distance_tolerance);
  if (solver_type == GST_LIBCCD)
  {
    // NOTE: Currently, only GST_LIBCCD computes the pair of nearest points.
    EXPECT_TRUE(CompareMatrices(result.nearest_points[0], Vector3<S>(20, 0, 0),
                                distance_tolerance * 100));
    EXPECT_TRUE(CompareMatrices(result.nearest_points[1], Vector3<S>(15, 0, 0),
                                distance_tolerance * 100));
  }
}

//==============================================================================

GTEST_TEST(FCL_NEGATIVE_DISTANCE, sphere_sphere_ccd)
{
  test_distance_spheresphere<double>(GST_LIBCCD);
}

GTEST_TEST(FCL_NEGATIVE_DISTANCE, sphere_sphere_indep)
{
  test_distance_spheresphere<double>(GST_INDEP);
}

GTEST_TEST(FCL_NEGATIVE_DISTANCE, sphere_capsule_ccd)
{
  test_distance_spherecapsule<double>(GST_LIBCCD);
}

GTEST_TEST(FCL_NEGATIVE_DISTANCE, sphere_capsule_indep)
{
  test_distance_spherecapsule<double>(GST_INDEP);
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

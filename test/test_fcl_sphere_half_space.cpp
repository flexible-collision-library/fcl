/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Toyota Research Institute
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

/** \author Alejandro Castro */

#include <iostream>

#include <gtest/gtest.h>

#include "fcl/fcl.h"

using namespace std;
using namespace fcl;

template <typename S>
void test_collision_sphere_half_space(fcl::GJKSolverType solver_type)
{
  // Numerical precision expected in the results.
  const double kTolerance = 20 * std::numeric_limits<double>::epsilon();

  const S radius = 0.1;
  auto sphere = std::make_shared<Sphere<S>>(radius);
  auto half_space = std::make_shared<Halfspace<S>>(Vector3<S>::UnitZ(), 0.0);

  // Pose of sphere frame S in the world frame W.
  Transform3<S> X_WS(Translation3<S>(Vector3<S>(0.0, 0.0, 0.11)));
  // Pose of half space frame H in the world frame W.
  Transform3<S> X_WH = Transform3<S>::Identity();
  
  CollisionObject<S> sphere_obj(sphere, X_WS);
  CollisionObject<S> half_space_obj(half_space, X_WH);

  fcl::DistanceResult<S> distance_result;
  fcl::DistanceRequest<S> distance_request;
  distance_request.gjk_solver_type = solver_type;
  distance_request.enable_nearest_points = true;
  fcl::distance(&half_space_obj, &sphere_obj, distance_request, distance_result);

  // The sphere is not touching the half space
  EXPECT_NEAR(distance_result.min_distance, 0.01, kTolerance);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, collision_sphere_half_space_libccd)
{
  test_collision_sphere_half_space<double>(fcl::GJKSolverType::GST_LIBCCD);
}

// tolerance too tight
// GTEST_TEST(FCL_GEOMETRIC_SHAPES, collision_sphere_half_space_indep)
// {
//   test_collision_sphere_half_space<double>(fcl::GJKSolverType::GST_INDEP);
// }

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

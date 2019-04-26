/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2016, CNRS-LAAS and AIST
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

/** @author Florent Lamiraux */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include "fcl/narrowphase/distance.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_object.h"

template <typename S>
void test_distance_capsule_box(fcl::GJKSolverType solver_type, S solver_tolerance, S test_tolerance)
{
  using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<S>>;

  // Capsule of radius 2 and of height 4
  CollisionGeometryPtr_t capsuleGeometry (new fcl::Capsule<S> (2., 4.));
  // Box of size 1 by 2 by 4
  CollisionGeometryPtr_t boxGeometry (new fcl::Box<S> (1., 2., 4.));

  // Enable computation of nearest points
  fcl::DistanceRequest<S> distanceRequest (true);
  fcl::DistanceResult<S> distanceResult;

  distanceRequest.gjk_solver_type = solver_type;
  distanceRequest.distance_tolerance = solver_tolerance;

  fcl::Transform3<S> tf1(fcl::Translation3<S>(fcl::Vector3<S> (3., 0, 0)));
  fcl::Transform3<S> tf2 = fcl::Transform3<S>::Identity();
  fcl::CollisionObject<S> capsule (capsuleGeometry, tf1);
  fcl::CollisionObject<S> box (boxGeometry, tf2);

  // test distance
  fcl::distance (&capsule, &box, distanceRequest, distanceResult);
  // Nearest point on capsule
  fcl::Vector3<S> o1 (distanceResult.nearest_points [0]);
  // Nearest point on box
  fcl::Vector3<S> o2 (distanceResult.nearest_points [1]);
  EXPECT_NEAR (distanceResult.min_distance, 0.5, test_tolerance);
  EXPECT_NEAR (o1 [0],  1.0, test_tolerance);
  EXPECT_NEAR (o1 [1],  0.0, test_tolerance);
  EXPECT_NEAR (o2 [0],  0.5, test_tolerance);
  EXPECT_NEAR (o2 [1],  0.0, test_tolerance);

  // Move capsule above box
  tf1 = fcl::Translation3<S>(fcl::Vector3<S> (0., 0., 8.));
  capsule.setTransform (tf1);

  // test distance
  distanceResult.clear ();
  fcl::distance (&capsule, &box, distanceRequest, distanceResult);
  o1 = distanceResult.nearest_points [0];
  o2 = distanceResult.nearest_points [1];

  EXPECT_NEAR (distanceResult.min_distance, 2.0, test_tolerance);
  EXPECT_NEAR (o1 [0],  0.0, test_tolerance);
  EXPECT_NEAR (o1 [1],  0.0, test_tolerance);
  EXPECT_NEAR (o1 [2],  4.0, test_tolerance);

  EXPECT_NEAR (o2 [0],  0.0, test_tolerance);
  EXPECT_NEAR (o2 [1],  0.0, test_tolerance);
  EXPECT_NEAR (o2 [2],  2.0, test_tolerance);

  // Rotate capsule around y axis by pi/2 and move it behind box
  tf1.translation() = fcl::Vector3<S>(-10., 0., 0.);
  tf1.linear() = fcl::Quaternion<S>(sqrt(2)/2, 0, sqrt(2)/2, 0).toRotationMatrix();
  capsule.setTransform (tf1);

  // test distance
  distanceResult.clear ();
  fcl::distance (&capsule, &box, distanceRequest, distanceResult);
  o1 = distanceResult.nearest_points [0];
  o2 = distanceResult.nearest_points [1];

  EXPECT_NEAR (distanceResult.min_distance, 5.5, test_tolerance);
  EXPECT_NEAR (o1 [0], -6.0, test_tolerance);
  EXPECT_NEAR (o1 [1],  0.0, test_tolerance);
  EXPECT_NEAR (o1 [2],  0.0, test_tolerance);
  EXPECT_NEAR (o2 [0], -0.5, test_tolerance);
  EXPECT_NEAR (o2 [1],  0.0, test_tolerance);
  EXPECT_NEAR (o2 [2],  0.0, test_tolerance);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, distance_capsule_box_ccd)
{
  test_distance_capsule_box<double>(fcl::GJKSolverType::GST_LIBCCD, 1e-8, 4e-4);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, distance_capsule_box_indep)
{
  test_distance_capsule_box<double>(fcl::GJKSolverType::GST_INDEP, 1e-8, 1e-4);
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

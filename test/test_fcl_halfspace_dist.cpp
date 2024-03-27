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
void test_distance_box_half_space(fcl::GJKSolverType solver_type)
{
  // Numerical precision expected in the results.
  const double kTolerance = 20 * std::numeric_limits<double>::epsilon();

  const S x = 0.1, y = 0.1, z = 0.2;

  auto half_space = std::make_shared<Halfspace<S>>(Vector3<S>::UnitZ(), 0.0);
  auto box = std::make_shared<Box<S>>(x, y, z);

  // Pose of box frame C in the world frame W.
  Transform3<S> X_WB(Translation3<S>(Vector3<S>(0.0, 0.0, 0.049)));

  // Pose of half space frame H in the world frame W.
  Transform3<S> X_WH = Transform3<S>::Identity();

  CollisionObject<S> half_space_co(half_space, X_WH);
  CollisionObject<S> box_co(box, X_WB);

  fcl::CollisionResult<S> result;
  static const int num_max_contacts = std::numeric_limits<int>::max();
  static const bool enable_contact = true;
  fcl::CollisionRequest<S> request(num_max_contacts, enable_contact);
  request.gjk_solver_type = solver_type;

  fcl::collide(&half_space_co, &box_co, request, result);
  vector<Contact<S>> contacts;
  result.getContacts(contacts);

  EXPECT_EQ(static_cast<int>(contacts.size()), 1);
  EXPECT_NEAR(contacts[0].penetration_depth, 0.051, kTolerance);

  // Now perform the same test but with the box's z axis Cz pointing down.
  X_WB.linear() = AngleAxis<S>(fcl::constants<S>::pi(), 
                               Vector3d::UnitX()).matrix();
  X_WB.translation() = Vector3<S>(0, 0, 0.049);
  box_co.setTransform(X_WB);

  result.clear();
  contacts.clear();

  fcl::collide(&half_space_co, &box_co, request, result);
  result.getContacts(contacts);

  EXPECT_EQ(static_cast<int>(contacts.size()), 1);
  EXPECT_NEAR(contacts[0].penetration_depth, 0.051, kTolerance);

  // finally rotate the box by half a pi and shift it by just over the radius in z direction
  X_WB.linear() = AngleAxis<S>(fcl::constants<S>::pi() / 2, 
                               Vector3d::UnitX()).matrix();
  X_WB.translation() = Vector3<S>(0, 0, 0.051);
  box_co.setTransform(X_WB);

  result.clear();
  contacts.clear();

  fcl::collide(&half_space_co, &box_co, request, result);
  result.getContacts(contacts);

  // now they should not collide
  EXPECT_EQ(static_cast<int>(contacts.size()), 0);

  // then query the distance
  box_co.setTransform(X_WB);
  fcl::DistanceResult<S> distance_result;
  fcl::DistanceRequest<S> distance_request;
  distance_request.gjk_solver_type = solver_type;
  distance_request.enable_nearest_points = true;
  fcl::distance(&half_space_co, &box_co, distance_request, distance_result);
  EXPECT_NEAR(distance_result.min_distance, 0.001, kTolerance);

  // swap the order of the objects
  fcl::distance(&box_co, &half_space_co, distance_request, distance_result);
  EXPECT_NEAR(distance_result.min_distance, 0.001, kTolerance);
}

template <typename S>
void test_distance_sphere_half_space(fcl::GJKSolverType solver_type)
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

template <typename S>
void test_distance_ellipsoid_half_space(fcl::GJKSolverType solver_type)
{
  // Numerical precision expected in the results.
  const double kTolerance = 20 * std::numeric_limits<double>::epsilon();

  const S radii[3] = {0.1, 0.1, 0.2};
  auto ellipsoid = std::make_shared<Ellipsoid<S>>(radii[0], radii[1], radii[2]);
  auto half_space = std::make_shared<Halfspace<S>>(Vector3<S>::UnitZ(), 0.0);

  // Pose of ellipsoid frame S in the world frame W.
  Transform3<S> X_WE(Translation3<S>(Vector3<S>(0.0, 0.0, 0.21)));
  // Pose of half space frame H in the world frame W.
  Transform3<S> X_WH = Transform3<S>::Identity();
  
  CollisionObject<S> ellipsoid_obj(ellipsoid, X_WE);
  CollisionObject<S> half_space_obj(half_space, X_WH);

  fcl::DistanceResult<S> distance_result;
  fcl::DistanceRequest<S> distance_request;
  distance_request.gjk_solver_type = solver_type;
  distance_request.enable_nearest_points = true;
  fcl::distance(&half_space_obj, &ellipsoid_obj, distance_request, distance_result);

  // The sphere is not touching the half space
  EXPECT_NEAR(distance_result.min_distance, 0.01, kTolerance);

  // Now perform the same test but with the ellipsoid's x axis pointing down.
  X_WE.linear() = AngleAxis<S>(fcl::constants<S>::pi() / 2,
                               Vector3d::UnitX()).matrix();
  ellipsoid_obj.setTransform(X_WE);

  distance_result.clear();
  fcl::distance(&half_space_obj, &ellipsoid_obj, distance_request, distance_result);
  EXPECT_NEAR(distance_result.min_distance, 0.11, kTolerance);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, collision_ellipsoid_half_space_libccd)
{
  test_distance_ellipsoid_half_space<double>(fcl::GJKSolverType::GST_LIBCCD);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, collision_sphere_half_space_libccd)
{
  test_distance_sphere_half_space<double>(fcl::GJKSolverType::GST_LIBCCD);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, collision_box_half_space_libccd)
{
  test_distance_box_half_space<double>(fcl::GJKSolverType::GST_LIBCCD);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, collision_box_half_space_indep)
{
  test_distance_box_half_space<double>(fcl::GJKSolverType::GST_INDEP);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

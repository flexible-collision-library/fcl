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

template <typename S>
void test_distance_capsule_half_space(fcl::GJKSolverType solver_type)
{
  // Numerical precision expected in the results.
  const double kTolerance = 20 * std::numeric_limits<double>::epsilon();

  const S radius = 0.1;
  const S lz = 0.2;
  auto capsule = std::make_shared<Capsule<S>>(radius, lz);
  auto half_space = std::make_shared<Halfspace<S>>(Vector3<S>::UnitZ(), 0.0);

  // Pose of capsule frame S in the world frame W.
  Transform3<S> X_WS(Translation3<S>(Vector3<S>(0.0, 0.0, 0.21)));
  // Pose of half space frame H in the world frame W.
  Transform3<S> X_WH = Transform3<S>::Identity();

  CollisionObject<S> capsule_obj(capsule, X_WS);
  CollisionObject<S> half_space_obj(half_space, X_WH);

  fcl::DistanceResult<S> distance_result;
  fcl::DistanceRequest<S> distance_request;
  distance_request.gjk_solver_type = solver_type;
  distance_request.enable_nearest_points = true;
  fcl::distance(&half_space_obj, &capsule_obj, distance_request, distance_result);

  // The capsule is not touching the half space
  EXPECT_NEAR(distance_result.min_distance, 0.01, kTolerance);

  // Now perform the same test but with the capsule's x axis pointing down.
  X_WS.linear() = AngleAxis<S>(fcl::constants<S>::pi() / 2,
                               Vector3d::UnitX()).matrix();
  capsule_obj.setTransform(X_WS);

  distance_result.clear();
  fcl::distance(&half_space_obj, &capsule_obj, distance_request, distance_result);
  EXPECT_NEAR(distance_result.min_distance, 0.11, kTolerance);
}

template <typename S>
void test_distance_cone_half_space(fcl::GJKSolverType solver_type)
{
  // Numerical precision expected in the results.
  const double kTolerance = 20 * std::numeric_limits<double>::epsilon();

  const S radius = 0.1;
  const S lz = 0.4;
  auto cone = std::make_shared<Cone<S>>(radius, lz);
  auto half_space = std::make_shared<Halfspace<S>>(Vector3<S>::UnitZ(), 0.0);

  // Pose of cone frame S in the world frame W.
  Transform3<S> X_WS(Translation3<S>(Vector3<S>(0.0, 0.0, 0.21)));
  // Pose of half space frame H in the world frame W.
  Transform3<S> X_WH = Transform3<S>::Identity();

  CollisionObject<S> cone_obj(cone, X_WS);
  CollisionObject<S> half_space_obj(half_space, X_WH);

  fcl::DistanceResult<S> distance_result;
  fcl::DistanceRequest<S> distance_request;
  distance_request.gjk_solver_type = solver_type;
  distance_request.enable_nearest_points = true;
  fcl::distance(&half_space_obj, &cone_obj, distance_request, distance_result);

  // The cone is not touching the half space
  EXPECT_NEAR(distance_result.min_distance, 0.01, kTolerance);

  // Now perform the same test but with the cone's x axis pointing down.
  X_WS.linear() = AngleAxis<S>(fcl::constants<S>::pi() / 2,
                               Vector3d::UnitX()).matrix();
  cone_obj.setTransform(X_WS);

  distance_result.clear();
  fcl::distance(&half_space_obj, &cone_obj, distance_request, distance_result);
  EXPECT_NEAR(distance_result.min_distance, 0.11, kTolerance);
}

template <typename S>
void test_distance_cylinder_half_space(fcl::GJKSolverType solver_type)
{
  // Numerical precision expected in the results.
  const double kTolerance = 20 * std::numeric_limits<double>::epsilon();

  const S radius = 0.1;
  const S lz = 0.4;
  auto cylinder = std::make_shared<Cylinder<S>>(radius, lz);
  auto half_space = std::make_shared<Halfspace<S>>(Vector3<S>::UnitZ(), 0.0);

  // Pose of cylinder frame S in the world frame W.
  Transform3<S> X_WS(Translation3<S>(Vector3<S>(0.0, 0.0, 0.21)));
  // Pose of half space frame H in the world frame W.
  Transform3<S> X_WH = Transform3<S>::Identity();

  CollisionObject<S> cylinder_obj(cylinder, X_WS);
  CollisionObject<S> half_space_obj(half_space, X_WH);

  fcl::DistanceResult<S> distance_result;
  fcl::DistanceRequest<S> distance_request;
  distance_request.gjk_solver_type = solver_type;
  distance_request.enable_nearest_points = true;
  fcl::distance(&half_space_obj, &cylinder_obj, distance_request, distance_result);

  // The cylinder is not touching the half space
  EXPECT_NEAR(distance_result.min_distance, 0.01, kTolerance);

  // Now perform the same test but with the cylinder's x axis pointing down.
  X_WS.linear() = AngleAxis<S>(fcl::constants<S>::pi() / 2,
                               Vector3d::UnitX()).matrix();
  cylinder_obj.setTransform(X_WS);

  distance_result.clear();
  fcl::distance(&half_space_obj, &cylinder_obj, distance_request, distance_result);
  EXPECT_NEAR(distance_result.min_distance, 0.11, kTolerance);
}

template <typename S>
void test_distance_plane_half_space(fcl::GJKSolverType solver_type)
{
  // Numerical precision expected in the results.
  const double kTolerance = 20 * std::numeric_limits<double>::epsilon();

  auto plane = std::make_shared<Plane<S>>(Vector3<S>::UnitZ(), 1.0);
  auto half_space = std::make_shared<Halfspace<S>>(Vector3<S>::UnitZ(), 0.0);

  // Pose of plane frame S in the world frame W.
  Transform3<S> X_WS = Transform3<S>::Identity();
  // Pose of half space frame H in the world frame W.
  Transform3<S> X_WH = Transform3<S>::Identity();

  CollisionObject<S> plane_obj(plane, X_WS);
  CollisionObject<S> half_space_obj(half_space, X_WH);

  fcl::DistanceResult<S> distance_result;
  fcl::DistanceRequest<S> distance_request;
  distance_request.gjk_solver_type = solver_type;
  distance_request.enable_nearest_points = true;
  fcl::distance(&half_space_obj, &plane_obj, distance_request, distance_result);

  // The plane is not touching the half space
  EXPECT_NEAR(distance_result.min_distance, 1.0, kTolerance);

  // tilt the plane so that it is not parallel to the half space
  plane = std::make_shared<Plane<S>>(Vector3<S>(0.1,0.1,0.9899494937), 1.0);
  plane_obj = CollisionObject<S>(plane, X_WS);

  distance_result.clear();
  fcl::distance(&half_space_obj, &plane_obj, distance_request, distance_result);
  EXPECT_NEAR(distance_result.min_distance, 0.0, kTolerance);
}

template <typename S>
void test_distance_half_space_half_space(fcl::GJKSolverType solver_type)
{
  // Numerical precision expected in the results.
  const double kTolerance = 20 * std::numeric_limits<double>::epsilon();

  auto half_space1 = std::make_shared<Halfspace<S>>(Vector3<S>::UnitZ(), 1.0);
  auto half_space2 = std::make_shared<Halfspace<S>>(Vector3<S>::UnitZ(), 0.0);

  Transform3<S> X_WH = Transform3<S>::Identity();

  CollisionObject<S> half_space_obj1(half_space1, X_WH);
  CollisionObject<S> half_space_obj2(half_space2, X_WH);

  fcl::DistanceResult<S> distance_result;
  fcl::DistanceRequest<S> distance_request;
  distance_request.gjk_solver_type = solver_type;
  distance_request.enable_nearest_points = true;
  fcl::distance(&half_space_obj1, &half_space_obj2, distance_request, distance_result);

  // halfspaces oriented in the same direction
  EXPECT_NEAR(distance_result.min_distance, 0.0, kTolerance);

  // halfspaces oriented in opposite directions
  half_space2 = std::make_shared<Halfspace<S>>(-Vector3<S>::UnitZ(), 0.0);
  half_space_obj2 = CollisionObject<S>(half_space2, X_WH);

  distance_result.clear();
  fcl::distance(&half_space_obj1, &half_space_obj2, distance_request, distance_result);
  EXPECT_NEAR(distance_result.min_distance, 1.0, kTolerance);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, distance_half_space_half_space_libccd)
{
  test_distance_half_space_half_space<double>(fcl::GJKSolverType::GST_LIBCCD);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, distance_plane_half_space_libccd)
{
  test_distance_plane_half_space<double>(fcl::GJKSolverType::GST_LIBCCD);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, distance_cylinder_half_space_libccd)
{
  test_distance_cylinder_half_space<double>(fcl::GJKSolverType::GST_LIBCCD);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, distance_cone_half_space_libccd)
{
  test_distance_cone_half_space<double>(fcl::GJKSolverType::GST_LIBCCD);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, distance_capsule_half_space_libccd)
{
  test_distance_capsule_half_space<double>(fcl::GJKSolverType::GST_LIBCCD);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, distance_ellipsoid_half_space_libccd)
{
  test_distance_ellipsoid_half_space<double>(fcl::GJKSolverType::GST_LIBCCD);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, distance_sphere_half_space_libccd)
{
  test_distance_sphere_half_space<double>(fcl::GJKSolverType::GST_LIBCCD);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, distance_box_half_space_libccd)
{
  test_distance_box_half_space<double>(fcl::GJKSolverType::GST_LIBCCD);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, distance_box_half_space_indep)
{
  test_distance_box_half_space<double>(fcl::GJKSolverType::GST_INDEP);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

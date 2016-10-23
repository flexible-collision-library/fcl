/*
 *  Software License Agreement (BSD License)
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

/** @author Martin Felis <martin.felis@iwr.uni-heidelberg.de> */

#include <gtest/gtest.h>

#include "fcl/math/constants.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/detail/gjk_solver_indep.h"
#include "fcl/narrowphase/detail/gjk_solver_libccd.h"

using namespace fcl;

template <typename S>
void test_Sphere_Capsule_Intersect_test_separated_z()
{
  detail::GJKSolver_libccd<S> solver;

  Sphere<S> sphere1 (50);
  Transform3<S> sphere1_transform;
  sphere1_transform.translation() = (Vector3<S> (0., 0., -50));

  Capsule<S> capsule (50, 200.);
  Transform3<S> capsule_transform(Translation3<S>(Vector3<S>(0., 0., 200)));

  EXPECT_TRUE (!solver.shapeIntersect(sphere1, sphere1_transform, capsule, capsule_transform, nullptr));
}

GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Intersect_test_separated_z)
{
//  test_Sphere_Capsule_Intersect_test_separated_z<float>();
  test_Sphere_Capsule_Intersect_test_separated_z<double>();
}

template <typename S>
void test_Sphere_Capsule_Intersect_test_separated_z_negative()
{
  detail::GJKSolver_libccd<S> solver;

  Sphere<S> sphere1 (50);
  Transform3<S> sphere1_transform;
  sphere1_transform.translation() = (Vector3<S> (0., 0., 50));

  Capsule<S> capsule (50, 200.);
  Transform3<S> capsule_transform(Translation3<S>(Vector3<S>(0., 0., -200)));

  EXPECT_TRUE (!solver.shapeIntersect(sphere1, sphere1_transform, capsule, capsule_transform, nullptr));
}

GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Intersect_test_separated_z_negative)
{
//  test_Sphere_Capsule_Intersect_test_separated_z_negative<float>();
  test_Sphere_Capsule_Intersect_test_separated_z_negative<double>();
}

template <typename S>
void test_Sphere_Capsule_Intersect_test_separated_x()
{
  detail::GJKSolver_libccd<S> solver;

  Sphere<S> sphere1 (50);
  Transform3<S> sphere1_transform;
  sphere1_transform.translation() = (Vector3<S> (0., 0., -50));

  Capsule<S> capsule (50, 200.);
  Transform3<S> capsule_transform(Translation3<S>(Vector3<S>(150., 0., 0.)));

  EXPECT_TRUE (!solver.shapeIntersect(sphere1, sphere1_transform, capsule, capsule_transform, nullptr));
}

GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Intersect_test_separated_x)
{
//  test_Sphere_Capsule_Intersect_test_separated_x<float>();
  test_Sphere_Capsule_Intersect_test_separated_x<double>();
}

template <typename S>
void test_Sphere_Capsule_Intersect_test_separated_capsule_rotated()
{
  detail::GJKSolver_libccd<S> solver;

  Sphere<S> sphere1 (50);
  Transform3<S> sphere1_transform;
  sphere1_transform.translation() = (Vector3<S> (0., 0., -50));

  Capsule<S> capsule (50, 200.);
  Matrix3<S> rotation(
        AngleAxis<S>(constants<S>::pi() * 0.5, Vector3<S>::UnitX())
      * AngleAxis<S>(0.0, Vector3<S>::UnitY())
      * AngleAxis<S>(0.0, Vector3<S>::UnitZ()));

  Transform3<S> capsule_transform = Transform3<S>::Identity();
  capsule_transform.linear() = rotation;
  capsule_transform.translation() = Vector3<S>(150., 0., 0.);

  EXPECT_TRUE (!solver.shapeIntersect(sphere1, sphere1_transform, capsule, capsule_transform, nullptr));
}

GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Intersect_test_separated_capsule_rotated)
{
//  test_Sphere_Capsule_Intersect_test_separated_capsule_rotated<float>();
  test_Sphere_Capsule_Intersect_test_separated_capsule_rotated<double>();
}

template <typename S>
void test_Sphere_Capsule_Intersect_test_penetration_z()
{
  detail::GJKSolver_libccd<S> solver;

  Sphere<S> sphere1 (50);
  Transform3<S> sphere1_transform(Translation3<S>(Vector3<S>(0., 0., -50)));

  Capsule<S> capsule (50, 200.);
  Transform3<S> capsule_transform(Translation3<S>(Vector3<S>(0., 0., 125)));

  std::vector<ContactPoint<S>> contacts;

  bool is_intersecting = solver.shapeIntersect(sphere1, sphere1_transform, capsule, capsule_transform, &contacts);

  S penetration = contacts[0].penetration_depth;
  Vector3<S> contact_point = contacts[0].pos;
  Vector3<S> normal = contacts[0].normal;

  EXPECT_TRUE (is_intersecting);
  EXPECT_TRUE (penetration == 25.);
  EXPECT_TRUE (Vector3<S> (0., 0., 1.).isApprox(normal));
  EXPECT_TRUE (Vector3<S> (0., 0., 0.).isApprox(contact_point));
}

GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Intersect_test_penetration_z)
{
//  test_Sphere_Capsule_Intersect_test_penetration_z<float>();
  test_Sphere_Capsule_Intersect_test_penetration_z<double>();
}

template <typename S>
void test_Sphere_Capsule_Intersect_test_penetration_z_rotated()
{
  detail::GJKSolver_libccd<S> solver;

  Sphere<S> sphere1 (50);
  Transform3<S> sphere1_transform = Transform3<S>::Identity();

  Capsule<S> capsule (50, 200.);
  Matrix3<S> rotation(
        AngleAxis<S>(constants<S>::pi() * 0.5, Vector3<S>::UnitX())
      * AngleAxis<S>(0.0, Vector3<S>::UnitY())
      * AngleAxis<S>(0.0, Vector3<S>::UnitZ()));
  Transform3<S> capsule_transform = Transform3<S>::Identity();
  capsule_transform.linear() = rotation;
  capsule_transform.translation() = Vector3<S> (0., 50., 75);

  std::vector<ContactPoint<S>> contacts;

  bool is_intersecting = solver.shapeIntersect(sphere1, sphere1_transform, capsule, capsule_transform, &contacts);

  S penetration = contacts[0].penetration_depth;
  Vector3<S> contact_point = contacts[0].pos;
  Vector3<S> normal = contacts[0].normal;

  EXPECT_TRUE (is_intersecting);
  EXPECT_NEAR (25, penetration, solver.collision_tolerance);
  EXPECT_TRUE (Vector3<S> (0., 0., 1.).isApprox(normal));
  EXPECT_TRUE (Vector3<S> (0., 0., 50.).isApprox(contact_point, solver.collision_tolerance));
}

GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Intersect_test_penetration_z_rotated)
{
//  test_Sphere_Capsule_Intersect_test_penetration_z_rotated<float>();
  test_Sphere_Capsule_Intersect_test_penetration_z_rotated<double>();
}

template <typename S>
void test_Sphere_Capsule_Distance_test_collision()
{
  detail::GJKSolver_libccd<S> solver;

  Sphere<S> sphere1 (50);
  Transform3<S> sphere1_transform(Translation3<S>(Vector3<S>(0., 0., -50)));

  Capsule<S> capsule (50, 200.);
  Transform3<S> capsule_transform(Translation3<S>(Vector3<S>(0., 0., 100)));

  S distance;

  EXPECT_TRUE (!solver.shapeDistance(sphere1, sphere1_transform, capsule, capsule_transform, &distance));

}

GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Distance_test_collision)
{
//  test_Sphere_Capsule_Distance_test_collision<float>();
  test_Sphere_Capsule_Distance_test_collision<double>();
}

template <typename S>
void test_Sphere_Capsule_Distance_test_separated()
{
  detail::GJKSolver_libccd<S> solver;

  Sphere<S> sphere1 (50);
  Transform3<S> sphere1_transform(Translation3<S>(Vector3<S>(0., 0., -50)));

  Capsule<S> capsule (50, 200.);
  Transform3<S> capsule_transform(Translation3<S>(Vector3<S>(0., 0., 175)));

  S distance = 0.;
  Vector3<S> p1;
  Vector3<S> p2;
  bool is_separated = solver.shapeDistance(sphere1, sphere1_transform, capsule, capsule_transform, &distance);

  EXPECT_TRUE (is_separated);
  EXPECT_TRUE (distance == 25.);
}

GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Distance_test_separated)
{
//  test_Sphere_Capsule_Distance_test_separated<float>();
  test_Sphere_Capsule_Distance_test_separated<double>();
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

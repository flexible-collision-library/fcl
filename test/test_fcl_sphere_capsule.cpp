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

/** \author Martin Felis <martin.felis@iwr.uni-heidelberg.de> */

#include <gtest/gtest.h>

#include "fcl/math/constants.h"
#include "fcl/collision.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/narrowphase/narrowphase.h"

using namespace fcl;

GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Intersect_test_separated_z)
{
	GJKSolver_libccd solver;

	Sphere sphere1 (50);
	Transform3d sphere1_transform;
  sphere1_transform.translation() = (Vector3d (0., 0., -50));

	Capsule capsule (50, 200.);
  Transform3d capsule_transform(Eigen::Translation3d(Vector3d(0., 0., 200)));

  EXPECT_TRUE (!solver.shapeIntersect(sphere1, sphere1_transform, capsule, capsule_transform, NULL));
}

GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Intersect_test_separated_z_negative)
{
	GJKSolver_libccd solver;

	Sphere sphere1 (50);
	Transform3d sphere1_transform;
  sphere1_transform.translation() = (Vector3d (0., 0., 50));

	Capsule capsule (50, 200.);
  Transform3d capsule_transform(Eigen::Translation3d(Vector3d(0., 0., -200)));

  EXPECT_TRUE (!solver.shapeIntersect(sphere1, sphere1_transform, capsule, capsule_transform, NULL));
}

GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Intersect_test_separated_x)
{
	GJKSolver_libccd solver;

	Sphere sphere1 (50);
	Transform3d sphere1_transform;
  sphere1_transform.translation() = (Vector3d (0., 0., -50));

	Capsule capsule (50, 200.);
  Transform3d capsule_transform(Eigen::Translation3d(Vector3d(150., 0., 0.)));

  EXPECT_TRUE (!solver.shapeIntersect(sphere1, sphere1_transform, capsule, capsule_transform, NULL));
}

GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Intersect_test_separated_capsule_rotated)
{
	GJKSolver_libccd solver;

	Sphere sphere1 (50);
	Transform3d sphere1_transform;
  sphere1_transform.translation() = (Vector3d (0., 0., -50));

	Capsule capsule (50, 200.);
  Matrix3d rotation(
        Eigen::AngleAxisd(constants::pi * 0.5, Vector3d::UnitX())
      * Eigen::AngleAxisd(0.0, Vector3d::UnitY())
      * Eigen::AngleAxisd(0.0, Vector3d::UnitZ()));

  Transform3d capsule_transform = Transform3d::Identity();
  capsule_transform.linear() = rotation;
  capsule_transform.translation() = Vector3d(150., 0., 0.);

  EXPECT_TRUE (!solver.shapeIntersect(sphere1, sphere1_transform, capsule, capsule_transform, NULL));
}

GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Intersect_test_penetration_z)
{
  GJKSolver_libccd solver;

  Sphere sphere1 (50);
  Transform3d sphere1_transform(Eigen::Translation3d(Vector3d(0., 0., -50)));

  Capsule capsule (50, 200.);
  Transform3d capsule_transform(Eigen::Translation3d(Vector3d(0., 0., 125)));

  std::vector<ContactPoint> contacts;

  bool is_intersecting = solver.shapeIntersect(sphere1, sphere1_transform, capsule, capsule_transform, &contacts);

  FCL_REAL penetration = contacts[0].penetration_depth;
  Vector3d contact_point = contacts[0].pos;
  Vector3d normal = contacts[0].normal;

  EXPECT_TRUE (is_intersecting);
  EXPECT_TRUE (penetration == 25.);
  EXPECT_TRUE (Vector3d (0., 0., 1.).isApprox(normal));
  EXPECT_TRUE (Vector3d (0., 0., 0.).isApprox(contact_point));
}

GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Intersect_test_penetration_z_rotated)
{
	GJKSolver_libccd solver;

	Sphere sphere1 (50);
  Transform3d sphere1_transform = Transform3d::Identity();

	Capsule capsule (50, 200.);
  Matrix3d rotation(
        Eigen::AngleAxisd(constants::pi * 0.5, Vector3d::UnitX())
      * Eigen::AngleAxisd(0.0, Vector3d::UnitY())
      * Eigen::AngleAxisd(0.0, Vector3d::UnitZ()));
  Transform3d capsule_transform = Transform3d::Identity();
  capsule_transform.linear() = rotation;
  capsule_transform.translation() = Vector3d (0., 50., 75);

  std::vector<ContactPoint> contacts;

  bool is_intersecting = solver.shapeIntersect(sphere1, sphere1_transform, capsule, capsule_transform, &contacts);

  FCL_REAL penetration = contacts[0].penetration_depth;
  Vector3d contact_point = contacts[0].pos;
  Vector3d normal = contacts[0].normal;

  EXPECT_TRUE (is_intersecting);
  EXPECT_NEAR (25, penetration, solver.collision_tolerance);
  EXPECT_TRUE (Vector3d (0., 0., 1.).isApprox(normal));
  EXPECT_TRUE (Vector3d (0., 0., 50.).isApprox(contact_point, solver.collision_tolerance));
}

GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Distance_test_collision)
{
	GJKSolver_libccd solver;

	Sphere sphere1 (50);
  Transform3d sphere1_transform(Eigen::Translation3d(Vector3d(0., 0., -50)));

	Capsule capsule (50, 200.);
  Transform3d capsule_transform(Eigen::Translation3d(Vector3d(0., 0., 100)));

	FCL_REAL distance;

  EXPECT_TRUE (!solver.shapeDistance(sphere1, sphere1_transform, capsule, capsule_transform, &distance));

}

GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Distance_test_separated)
{
	GJKSolver_libccd solver;

	Sphere sphere1 (50);
  Transform3d sphere1_transform(Eigen::Translation3d(Vector3d(0., 0., -50)));

	Capsule capsule (50, 200.);
  Transform3d capsule_transform(Eigen::Translation3d(Vector3d(0., 0., 175)));

	FCL_REAL distance = 0.;
	Vector3d p1;
	Vector3d p2;
	bool is_separated = solver.shapeDistance(sphere1, sphere1_transform, capsule, capsule_transform, &distance);

  EXPECT_TRUE (is_separated);
  EXPECT_TRUE (distance == 25.);
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

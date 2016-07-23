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
	Transform3f sphere1_transform;
	sphere1_transform.setTranslation (Vec3f (0., 0., -50));

	Capsule capsule (50, 200.);
	Transform3f capsule_transform (Vec3f (0., 0., 200));

  EXPECT_TRUE (!solver.shapeIntersect(sphere1, sphere1_transform, capsule, capsule_transform, NULL));
}

GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Intersect_test_separated_z_negative)
{
	GJKSolver_libccd solver;

	Sphere sphere1 (50);
	Transform3f sphere1_transform;
	sphere1_transform.setTranslation (Vec3f (0., 0., 50));

	Capsule capsule (50, 200.);
	Transform3f capsule_transform (Vec3f (0., 0., -200));

  EXPECT_TRUE (!solver.shapeIntersect(sphere1, sphere1_transform, capsule, capsule_transform, NULL));
}

GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Intersect_test_separated_x)
{
	GJKSolver_libccd solver;

	Sphere sphere1 (50);
	Transform3f sphere1_transform;
	sphere1_transform.setTranslation (Vec3f (0., 0., -50));

	Capsule capsule (50, 200.);
	Transform3f capsule_transform (Vec3f (150., 0., 0.));

  EXPECT_TRUE (!solver.shapeIntersect(sphere1, sphere1_transform, capsule, capsule_transform, NULL));
}

GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Intersect_test_separated_capsule_rotated)
{
	GJKSolver_libccd solver;

	Sphere sphere1 (50);
	Transform3f sphere1_transform;
	sphere1_transform.setTranslation (Vec3f (0., 0., -50));

	Capsule capsule (50, 200.);
	Matrix3f rotation;
	rotation.setEulerZYX (constants::pi * 0.5, 0., 0.);
	Transform3f capsule_transform (rotation, Vec3f (150., 0., 0.));

  EXPECT_TRUE (!solver.shapeIntersect(sphere1, sphere1_transform, capsule, capsule_transform, NULL));
}

GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Intersect_test_penetration_z)
{
  GJKSolver_libccd solver;

  Sphere sphere1 (50);
  Transform3f sphere1_transform;
  sphere1_transform.setTranslation (Vec3f (0., 0., -50));

  Capsule capsule (50, 200.);
  Transform3f capsule_transform (Vec3f (0., 0., 125));

  std::vector<ContactPoint> contacts;

  bool is_intersecting = solver.shapeIntersect(sphere1, sphere1_transform, capsule, capsule_transform, &contacts);

  FCL_REAL penetration = contacts[0].penetration_depth;
  Vec3f contact_point = contacts[0].pos;
  Vec3f normal = contacts[0].normal;

  EXPECT_TRUE (is_intersecting);
  EXPECT_TRUE (penetration == 25.);
  EXPECT_TRUE (Vec3f (0., 0., 1.).equal(normal));
  EXPECT_TRUE (Vec3f (0., 0., 0.).equal(contact_point));
}

GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Intersect_test_penetration_z_rotated)
{
	GJKSolver_libccd solver;

	Sphere sphere1 (50);
	Transform3f sphere1_transform;
	sphere1_transform.setTranslation (Vec3f (0., 0., 0));

	Capsule capsule (50, 200.);
	Matrix3f rotation;
	rotation.setEulerZYX (constants::pi * 0.5, 0., 0.);
	Transform3f capsule_transform (rotation, Vec3f (0., 50., 75));

  std::vector<ContactPoint> contacts;

  bool is_intersecting = solver.shapeIntersect(sphere1, sphere1_transform, capsule, capsule_transform, &contacts);

  FCL_REAL penetration = contacts[0].penetration_depth;
  Vec3f contact_point = contacts[0].pos;
  Vec3f normal = contacts[0].normal;

  EXPECT_TRUE (is_intersecting);
  EXPECT_NEAR (25, penetration, solver.collision_tolerance);
  EXPECT_TRUE (Vec3f (0., 0., 1.).equal(normal));
  EXPECT_TRUE (Vec3f (0., 0., 50.).equal(contact_point, solver.collision_tolerance));
}

GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Distance_test_collision)
{
	GJKSolver_libccd solver;

	Sphere sphere1 (50);
	Transform3f sphere1_transform;
	sphere1_transform.setTranslation (Vec3f (0., 0., -50));

	Capsule capsule (50, 200.);
	Transform3f capsule_transform (Vec3f (0., 0., 100));

	FCL_REAL distance;

  EXPECT_TRUE (!solver.shapeDistance(sphere1, sphere1_transform, capsule, capsule_transform, &distance));

}

GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Distance_test_separated)
{
	GJKSolver_libccd solver;

	Sphere sphere1 (50);
	Transform3f sphere1_transform;
	sphere1_transform.setTranslation (Vec3f (0., 0., -50));

	Capsule capsule (50, 200.);
	Transform3f capsule_transform (Vec3f (0., 0., 175));

	FCL_REAL distance = 0.;
	Vec3f p1;
	Vec3f p2;
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

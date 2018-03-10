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
void test_collision_cylinder_half_space(fcl::GJKSolverType solver_type)
{
  // Numerical precision expected in the results.
  const double kTolerance = 20 * std::numeric_limits<double>::epsilon();

  const S radius = 0.05;
  const S length = 4 * radius;

  auto half_space = std::make_shared<Halfspace<S>>(Vector3<S>::UnitZ(), 0.0);
  auto cylinder = std::make_shared<Cylinder<S>>(radius, length);

  // Pose of cylinder frame C in the world frame W.
  Transform3<S> X_WC(Translation3<S>(Vector3<S>(0.0, 0.0, 0.049)));

  // Pose of half space frame H in the world frame W.
  Transform3<S> X_WH = Transform3<S>::Identity();

  CollisionObject<S> half_space_co(half_space, X_WH);
  CollisionObject<S> cylinder_co(cylinder, X_WC);

  fcl::CollisionResult<S> result;
  static const int num_max_contacts = std::numeric_limits<int>::max();
  static const bool enable_contact = true;
  fcl::CollisionRequest<S> request(num_max_contacts, enable_contact);
  request.gjk_solver_type = solver_type;

  fcl::collide(&half_space_co, &cylinder_co, request, result);
  vector<Contact<S>> contacts;
  result.getContacts(contacts);

  EXPECT_EQ(static_cast<int>(contacts.size()), 1);
  EXPECT_NEAR(contacts[0].penetration_depth, 0.051, kTolerance);

  // Now perform the same test but with the cylinder's z axis Cz pointing down.
  X_WC.linear() = AngleAxis<S>(fcl::constants<S>::pi(), 
                               Vector3d::UnitX()).matrix();
  X_WC.translation() = Vector3<S>(0, 0, 0.049);
  cylinder_co.setTransform(X_WC);

  result.clear();
  contacts.clear();

  fcl::collide(&half_space_co, &cylinder_co, request, result);
  result.getContacts(contacts);

  EXPECT_EQ(static_cast<int>(contacts.size()), 1);
  EXPECT_NEAR(contacts[0].penetration_depth, 0.051, kTolerance);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, collision_cylinder_half_space_libccd)
{
  test_collision_cylinder_half_space<double>(fcl::GJKSolverType::GST_LIBCCD);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, collision_cylinder_half_space_indep)
{
  test_collision_cylinder_half_space<double>(fcl::GJKSolverType::GST_INDEP);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

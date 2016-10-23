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

/** @author Karsten Knese <Karsten.Knese@googlemail.com> */

#include <gtest/gtest.h>

#include "fcl/math/constants.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/detail/gjk_solver_indep.h"
#include "fcl/narrowphase/detail/gjk_solver_libccd.h"

#include <cmath>
using namespace fcl;

//==============================================================================
template <typename S>
void test_distance_capsulecapsule_origin()
{
  detail::GJKSolver_indep<S> solver;
  Capsule<S> s1(5, 10);
  Capsule<S> s2(5, 10);

  Vector3<S> closest_p1, closest_p2;

  Transform3<S> transform = Transform3<S>::Identity();
  Transform3<S> transform2 = Transform3<S>::Identity();
  transform2.translation() = Vector3<S>(20.1, 0,0);

  bool res;
  S dist;

  res = solver.template shapeDistance<Capsule<S>, Capsule<S>>(s1, transform, s2, transform2, &dist, &closest_p1, &closest_p2);
  std::cerr << "applied transformation of two caps: " << transform.translation() << " & " << transform2.translation() << std::endl;
  std::cerr << "computed points in caps to caps" << closest_p1 << " & " << closest_p2 << "with dist: " << dist << std::endl;

  EXPECT_TRUE(std::abs(dist - 10.1) < 0.001);
  EXPECT_TRUE(res);
}

//==============================================================================
template <typename S>
void test_distance_capsulecapsule_transformXY()
{
  detail::GJKSolver_indep<S> solver;
  Capsule<S> s1(5, 10);
  Capsule<S> s2(5, 10);

  Vector3<S> closest_p1, closest_p2;

  Transform3<S> transform = Transform3<S>::Identity();
  Transform3<S> transform2 = Transform3<S>::Identity();
  transform2.translation() = Vector3<S>(20, 20,0);

  bool res;
  S dist;

  res = solver.template shapeDistance<Capsule<S>, Capsule<S>>(s1, transform, s2, transform2, &dist, &closest_p1, &closest_p2);
  std::cerr << "applied transformation of two caps: " << transform.translation() << " & " << transform2.translation() << std::endl;
  std::cerr << "computed points in caps to caps" << closest_p1 << " & " << closest_p2 << "with dist: " << dist << std::endl;

  S expected = std::sqrt(S(800)) - 10;
  EXPECT_TRUE(std::abs(expected-dist) < 0.01);
  EXPECT_TRUE(res);
}

//==============================================================================
template <typename S>
void test_distance_capsulecapsule_transformZ()
{
  detail::GJKSolver_indep<S> solver;
  Capsule<S> s1(5, 10);
  Capsule<S> s2(5, 10);

  Vector3<S> closest_p1, closest_p2;

  Transform3<S> transform = Transform3<S>::Identity();
  Transform3<S> transform2 = Transform3<S>::Identity();
  transform2.translation() = Vector3<S>(0,0,20.1);

  bool res;
  S dist;

  res = solver.template shapeDistance<Capsule<S>, Capsule<S>>(s1, transform, s2, transform2, &dist, &closest_p1, &closest_p2);
  std::cerr << "applied transformation of two caps: " << transform.translation() << " & " << transform2.translation() << std::endl;
  std::cerr << "computed points in caps to caps" << closest_p1 << " & " << closest_p2 << "with dist: " << dist << std::endl;

  EXPECT_TRUE(std::abs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);
}

//==============================================================================
template <typename S>
void test_distance_capsulecapsule_transformZ2()
{
  const S Pi = constants<S>::pi();

  detail::GJKSolver_indep<S> solver;
  Capsule<S> s1(5, 10);
  Capsule<S> s2(5, 10);

  Vector3<S> closest_p1, closest_p2;

  Transform3<S> transform = Transform3<S>::Identity();
  Transform3<S> transform2 = Transform3<S>::Identity();
  transform2.translation() = Vector3<S>(0,0,25.1);
  Matrix3<S> rot2(
        AngleAxis<S>(0, Vector3<S>::UnitX())
        * AngleAxis<S>(Pi/2, Vector3<S>::UnitY())
        * AngleAxis<S>(0, Vector3<S>::UnitZ()));
  transform2.linear() = rot2;

  bool res;
  S dist;

  res = solver.template shapeDistance<Capsule<S>, Capsule<S>>(s1, transform, s2, transform2, &dist, &closest_p1, &closest_p2);
  std::cerr << "applied transformation of two caps: " << transform.translation() << " & " << transform2.translation() << std::endl;
  std::cerr << "applied transformation of two caps: " << transform.linear() << " & " << transform2.linear() << std::endl;
  std::cerr << "computed points in caps to caps" << closest_p1 << " & " << closest_p2 << "with dist: " << dist << std::endl;

  EXPECT_TRUE(std::abs(dist - 5.1) < 0.001);
  EXPECT_TRUE(res);
}

//==============================================================================
GTEST_TEST(FCL_CAPSULE_CAPSULE, distance_capsulecapsule_origin)
{
//  test_distance_capsulecapsule_origin<float>();
  test_distance_capsulecapsule_origin<double>();
}

//==============================================================================
GTEST_TEST(FCL_CAPSULE_CAPSULE, distance_capsulecapsule_transformXY)
{
//  test_distance_capsulecapsule_transformXY<float>();
  test_distance_capsulecapsule_transformXY<double>();
}

//==============================================================================
GTEST_TEST(FCL_CAPSULE_CAPSULE, distance_capsulecapsule_transformZ)
{
//  test_distance_capsulecapsule_transformZ<float>();
  test_distance_capsulecapsule_transformZ<double>();
}

//==============================================================================
GTEST_TEST(FCL_CAPSULE_CAPSULE, distance_capsulecapsule_transformZ2)
{
//  test_distance_capsulecapsule_transformZ2<float>();
  test_distance_capsulecapsule_transformZ2<double>();
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

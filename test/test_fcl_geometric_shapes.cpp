/*
 * Software License Agreement (BSD License)
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

/** @author Jia Pan */

#include <array>
#include <iostream>
#include <limits>

#include <gtest/gtest.h>

#include "fcl/common/unused.h"

#include "fcl/math/motion/translation_motion.h"

#include "fcl/geometry/shape/cone.h"
#include "fcl/geometry/shape/capsule.h"
#include "fcl/geometry/shape/ellipsoid.h"
#include "fcl/geometry/shape/halfspace.h"
#include "fcl/geometry/shape/plane.h"

#include "fcl/narrowphase/detail/gjk_solver_indep.h"
#include "fcl/narrowphase/detail/gjk_solver_libccd.h"
#include "fcl/narrowphase/collision.h"

#include "test_fcl_utility.h"

using namespace fcl;

template <typename S>
std::array<S, 6>& extents()
{
  static std::array<S, 6> static_extents{ {0, 0, 0, 10, 10, 10} };
  return static_extents;
}

template <typename S>
detail::GJKSolver_libccd<S>& solver1()
{
  static detail::GJKSolver_libccd<S> static_solver1;
  return static_solver1;
}

template <typename S>
detail::GJKSolver_indep<S>& solver2()
{
  static detail::GJKSolver_indep<S> static_solver2;
  return static_solver2;
}

template <typename S>
S tolerance();

template <>
float tolerance() { return 1e-4; }

template <>
double tolerance() { return 1e-12; }

template <typename S>
void test_sphere_shape()
{
  const S radius = 5.0;
  const S pi = constants<S>::pi();

  Sphere<S> s(radius);

  const auto volume = 4.0 / 3.0 * pi * radius * radius * radius;
  EXPECT_NEAR(volume, s.computeVolume(), tolerance<S>());
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, sphere_shape)
{
//  test_sphere_shape<float>();
  test_sphere_shape<double>();
}

template <typename S>
void test_gjkcache()
{
  Cylinder<S> s1(5, 10);
  Cone<S> s2(5, 10);

  CollisionRequest<S> request;
  request.enable_cached_gjk_guess = true;
  request.gjk_solver_type = GST_INDEP;

  TranslationMotion<S> motion(Transform3<S>(Translation3<S>(Vector3<S>(-20.0, -20.0, -20.0))), Transform3<S>(Translation3<S>(Vector3<S>(20.0, 20.0, 20.0))));

  int N = 1000;
  S dt = 1.0 / (N - 1);

  /// test exploiting spatial coherence
  test::Timer timer1;
  timer1.start();
  std::vector<bool> result1(N);
  for(int i = 0; i < N; ++i)
  {
    motion.integrate(dt * i);
    Transform3<S> tf;
    motion.getCurrentTransform(tf);

    CollisionResult<S> result;

    collide(&s1, Transform3<S>::Identity(), &s2, tf, request, result);
    result1[i] = result.isCollision();
    request.cached_gjk_guess = result.cached_gjk_guess; // use cached guess
  }

  timer1.stop();

  /// test without exploiting spatial coherence
  test::Timer timer2;
  timer2.start();
  std::vector<bool> result2(N);
  request.enable_cached_gjk_guess = false;
  for(int i = 0; i < N; ++i)
  {
    motion.integrate(dt * i);
    Transform3<S> tf;
    motion.getCurrentTransform(tf);

    CollisionResult<S> result;

    collide(&s1, Transform3<S>::Identity(), &s2, tf, request, result);
    result2[i] = result.isCollision();
  }

  timer2.stop();

  std::cout << timer1.getElapsedTime() << " " << timer2.getElapsedTime() << std::endl;

  for(std::size_t i = 0; i < result1.size(); ++i)
  {
    EXPECT_TRUE(result1[i] == result2[i]);
  }
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, gjkcache)
{
//  test_gjkcache<float>();
  test_gjkcache<double>();
}

template <typename Shape1, typename Shape2>
void printComparisonError(const std::string& comparison_type,
                          const Shape1& s1, const Transform3<typename Shape1::S>& tf1,
                          const Shape2& s2, const Transform3<typename Shape1::S>& tf2,
                          GJKSolverType solver_type,
                          const Vector3<typename Shape1::S>& expected_contact_or_normal,
                          const Vector3<typename Shape1::S>& actual_contact_or_normal,
                          bool check_opposite_normal,
                          typename Shape1::S tol)
{
  std::cout << "Disagreement between " << comparison_type
            << " and expected_" << comparison_type << " for "
            << test::getNodeTypeName(s1.getNodeType()) << " and "
            << test::getNodeTypeName(s2.getNodeType()) << " with '"
            << test::getGJKSolverName(solver_type) << "' solver." << std::endl
            << "tf1.linear: \n" << tf1.linear() << std::endl
            << "tf1.translation: " << tf1.translation().transpose() << std::endl
            << "tf2.linear: \n" << tf2.linear() << std::endl
            << "tf2.translation: " << tf2.translation().transpose() << std::endl
            << "expected_" << comparison_type << ": " << expected_contact_or_normal
            << "actual_" << comparison_type << "  : " << actual_contact_or_normal << std::endl;

  if (check_opposite_normal)
    std::cout << " or " << -expected_contact_or_normal;

  std::cout << std::endl
            << "difference: " << (actual_contact_or_normal - expected_contact_or_normal).norm() << std::endl
            << "tolerance: " << tol << std::endl;
}

template <typename Shape1, typename Shape2>
void printComparisonError(const std::string& comparison_type,
                          const Shape1& s1, const Transform3<typename Shape1::S>& tf1,
                          const Shape2& s2, const Transform3<typename Shape1::S>& tf2,
                          GJKSolverType solver_type,
                          typename Shape1::S expected_depth,
                          typename Shape1::S actual_depth,
                          typename Shape1::S tol)
{
  std::cout << "Disagreement between " << comparison_type
            << " and expected_" << comparison_type << " for "
            << test::getNodeTypeName(s1.getNodeType()) << " and "
            << test::getNodeTypeName(s2.getNodeType()) << " with '"
            << test::getGJKSolverName(solver_type) << "' solver." << std::endl
            << "tf1.linear: \n" << tf1.linear() << std::endl
            << "tf1.translation: " << tf1.translation().transpose() << std::endl
            << "tf2.linear: \n" << tf2.linear() << std::endl
            << "tf2.translation: " << tf2.translation().transpose() << std::endl
            << "expected_depth: " << expected_depth << std::endl
            << "actual_depth  : " << actual_depth << std::endl
            << "difference: " << std::abs(actual_depth - expected_depth) << std::endl
            << "tolerance: " << tol << std::endl;
}

template <typename Shape1, typename Shape2>
bool checkContactPointds(const Shape1& s1, const Transform3<typename Shape1::S>& tf1,
                        const Shape2& s2, const Transform3<typename Shape1::S>& tf2,
                        GJKSolverType solver_type,
                        const ContactPoint<typename Shape1::S>& expected, const ContactPoint<typename Shape1::S>& actual,
                        bool check_position = false,
                        bool check_depth = false,
                        bool check_normal = false,
                        bool check_opposite_normal = false,
                        typename Shape1::S tol = 1e-9)
{
  FCL_UNUSED(s1);
  FCL_UNUSED(tf1);
  FCL_UNUSED(s2);
  FCL_UNUSED(tf2);
  FCL_UNUSED(solver_type);

  if (check_position)
  {
    bool contact_equal = actual.pos.isApprox(expected.pos, tol);
    if (!contact_equal)
      return false;
  }

  if (check_depth)
  {
    bool depth_equal = std::abs(actual.penetration_depth - expected.penetration_depth) < tol;
    if (!depth_equal)
      return false;
  }

  if (check_normal)
  {
    bool normal_equal = actual.normal.isApprox(expected.normal, tol);

    if (!normal_equal && check_opposite_normal)
      normal_equal = actual.normal.isApprox(-expected.normal, tol);

    if (!normal_equal)
      return false;
  }

  return true;
}

template <typename Shape1, typename Shape2>
bool inspectContactPointds(const Shape1& s1, const Transform3<typename Shape1::S>& tf1,
                          const Shape2& s2, const Transform3<typename Shape1::S>& tf2,
                          GJKSolverType solver_type,
                          const std::vector<ContactPoint<typename Shape1::S>>& expected_contacts,
                          const std::vector<ContactPoint<typename Shape1::S>>& actual_contacts,
                          bool check_position = false,
                          bool check_depth = false,
                          bool check_normal = false,
                          bool check_opposite_normal = false,
                          typename Shape1::S tol = 1e-9)
{
  using S = typename Shape1::S;

  // Check number of contact points
  bool sameNumContacts = (actual_contacts.size() == expected_contacts.size());
  EXPECT_TRUE(sameNumContacts);
  if (!sameNumContacts)
  {
    std::cout << "\n"
              << "===== [ geometric shape collision test failure report ] ======\n"
              << "\n"
              << "Solver type: " << test::getGJKSolverName(solver_type) << "\n"
              << "\n"
              << "[ Shape 1 ]\n"
              << "Shape type     : " << test::getNodeTypeName(s1.getNodeType()) << "\n"
              << "tf1.linear     : \n" << tf1.linear() << "\n"
              << "tf1.translation: " << tf1.translation().transpose() << "\n"
              << "\n"
              << "[ Shape 2 ]\n"
              << "Shape type     : " << test::getNodeTypeName(s2.getNodeType()) << "\n"
              << "tf2.linear     : \n" << tf2.linear() << "\n"
              << "tf2.translation: " << tf2.translation().transpose() << "\n"
              << "\n"
              << "The numbers of expected contacts '"
              << expected_contacts.size()
              << "' and the number of actual contacts '"
              << actual_contacts.size()
              << "' are not equal.\n"
              << "\n";
    return false;
  }

  // Check if actual contacts and expected contacts are matched
  const size_t numContacts = actual_contacts.size();

  std::vector<int> index_to_actual_contacts(numContacts, -1);
  std::vector<int> index_to_expected_contacts(numContacts, -1);

  bool foundAll = true;
  for (size_t i = 0; i < numContacts; ++i)
  {
    const ContactPoint<S>& expected = expected_contacts[i];

    // Check if expected contact is in the list of actual contacts
    for (size_t j = 0; j < numContacts; ++j)
    {
      if (index_to_expected_contacts[j] != -1)
        continue;

      const ContactPoint<S>& actual = actual_contacts[j];

      bool found = checkContactPointds(
            s1, tf1, s2, tf2, solver_type,
            expected, actual,
            check_position,
            check_depth,
            check_normal, check_opposite_normal,
            tol);

      if (found)
      {
        index_to_actual_contacts[i] = j;
        index_to_expected_contacts[j] = i;
        break;
      }
    }

    if (index_to_actual_contacts[i] == -1)
      foundAll = false;
  }

  if (!foundAll)
  {
    std::cout << "\n"
              << "===== [ geometric shape collision test failure report ] ======\n"
              << "\n"
              << "Solver type: " << test::getGJKSolverName(solver_type) << "\n"
              << "\n"
              << "[ Shape 1 ]\n"
              << "Shape type     : " << test::getNodeTypeName(s1.getNodeType()) << "\n"
              << "tf1.linear     : \n" << tf1.linear() << "\n"
              << "tf1.translation: " << tf1.translation().transpose() << "\n"
              << "\n"
              << "[ Shape 2 ]\n"
              << "Shape type     : " << test::getNodeTypeName(s2.getNodeType()) << "\n"
              << "tf2.linear     : \n" << tf2.linear() << "\n"
              << "tf2.translation: " << tf2.translation().transpose() << "\n"
              << "\n"
              << "[ Expected Contacts: " << numContacts << " ]\n";
    for (size_t i = 0; i < numContacts; ++i)
    {
      const ContactPoint<S>& expected = expected_contacts[i];

      std::cout << "(" << i << ") pos: " << expected.pos.transpose() << ", "
                << "normal: " << expected.normal.transpose() << ", "
                << "depth: " << expected.penetration_depth
                << " ---- ";
      if (index_to_actual_contacts[i] != -1)
        std::cout << "found, actual (" << index_to_actual_contacts[i] << ")\n";
      else
        std::cout << "not found!\n";
    }
    std::cout << "\n"
              << "[ Actual Contacts: " << numContacts << " ]\n";
    for (size_t i = 0; i < numContacts; ++i)
    {
      const ContactPoint<S>& actual = actual_contacts[i];

      std::cout << "(" << i << ") pos: " << actual.pos.transpose() << ", "
                << "normal: " << actual.normal.transpose() << ", "
                << "depth: " << actual.penetration_depth
                << " ---- ";
      if (index_to_expected_contacts[i] != -1)
        std::cout << "found, expected (" << index_to_expected_contacts[i] << ")\n";
      else
        std::cout << "not found!\n";
    }

    std::cout << "\n";
  }

  return foundAll;
}

template <typename S>
void getContactPointdsFromResult(std::vector<ContactPoint<S>>& contacts, const CollisionResult<S>& result)
{
  const size_t numContacts = result.numContacts();
  contacts.resize(numContacts);

  for (size_t i = 0; i < numContacts; ++i)
  {
    const auto& cnt = result.getContact(i);

    contacts[i].pos = cnt.pos;
    contacts[i].normal = cnt.normal;
    contacts[i].penetration_depth = cnt.penetration_depth;
  }
}

template <typename Shape1, typename Shape2>
void testShapeIntersection(
    const Shape1& s1, const Transform3<typename Shape1::S>& tf1,
    const Shape2& s2, const Transform3<typename Shape1::S>& tf2,
    GJKSolverType solver_type,
    bool expected_res,
    const std::vector<ContactPoint<typename Shape1::S>>& expected_contacts = std::vector<ContactPoint<typename Shape1::S>>(),
    bool check_position = true,
    bool check_depth = true,
    bool check_normal = true,
    bool check_opposite_normal = false,
    typename Shape1::S tol = 1e-9)
{
  using S = typename Shape1::S;

  CollisionRequest<S> request;
  request.gjk_solver_type = solver_type;
  request.num_max_contacts = std::numeric_limits<size_t>::max();
  CollisionResult<S> result;

  std::vector<ContactPoint<S>> actual_contacts;

  bool res;

  // Part A: Check collisions using shapeIntersect()

  // Check only whether they are colliding or not.
  if (solver_type == GST_LIBCCD)
  {
    res = solver1<S>().shapeIntersect(s1, tf1, s2, tf2, nullptr);
  }
  else if (solver_type == GST_INDEP)
  {
    res = solver2<S>().shapeIntersect(s1, tf1, s2, tf2, nullptr);
  }
  else
  {
    std::cerr << "Invalid GJK solver. Test aborted." << std::endl;
    return;
  }
  EXPECT_EQ(expected_res, res);

  // Check contact information as well
  if (solver_type == GST_LIBCCD)
  {
    res = solver1<S>().shapeIntersect(s1, tf1, s2, tf2, &actual_contacts);
  }
  else if (solver_type == GST_INDEP)
  {
    res = solver2<S>().shapeIntersect(s1, tf1, s2, tf2, &actual_contacts);
  }
  else
  {
    std::cerr << "Invalid GJK solver. Test aborted." << std::endl;
    return;
  }
  EXPECT_EQ(expected_res, res);
  if (expected_res)
  {
    EXPECT_TRUE(inspectContactPointds(s1, tf1, s2, tf2, solver_type,
                                     expected_contacts, actual_contacts,
                                     check_position,
                                     check_depth,
                                     check_normal, check_opposite_normal,
                                     tol));
  }

  // Part B: Check collisions using collide()

  // Check only whether they are colliding or not.
  request.enable_contact = false;
  result.clear();
  res = (collide(&s1, tf1, &s2, tf2, request, result) > 0);
  EXPECT_EQ(expected_res, res);

  // Check contact information as well
  request.enable_contact = true;
  result.clear();
  res = (collide(&s1, tf1, &s2, tf2, request, result) > 0);
  EXPECT_EQ(expected_res, res);
  if (expected_res)
  {
    getContactPointdsFromResult(actual_contacts, result);
    EXPECT_TRUE(inspectContactPointds(s1, tf1, s2, tf2, solver_type,
                                     expected_contacts, actual_contacts,
                                     check_position,
                                     check_depth,
                                     check_normal, check_opposite_normal,
                                     tol));
  }
}

// Shape intersection test coverage (libccd)
//
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// |            | box | sphere | ellipsoid | capsule | cone | cylinder | plane | half-space | triangle |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | box        |  O  |   O    |           |         |      |          |   O   |      O     |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | sphere     |/////|   O    |           |         |      |          |   O   |      O     |    O     |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | ellipsoid  |/////|////////|     O     |         |      |          |   O   |      O     |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | capsule    |/////|////////|///////////|         |      |          |   O   |      O     |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | cone       |/////|////////|///////////|/////////|  O   |    O     |   O   |      O     |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | cylinder   |/////|////////|///////////|/////////|//////|    O     |   O   |      O     |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | plane      |/////|////////|///////////|/////////|//////|//////////|       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | half-space |/////|////////|///////////|/////////|//////|//////////|///////|            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | triangle   |/////|////////|///////////|/////////|//////|//////////|///////|////////////|          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+

template <typename S>
void test_shapeIntersection_spheresphere()
{
  Sphere<S> s1(20);
  Sphere<S> s2(10);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(30, 0, 0)));
  contacts.resize(1);
  contacts[0].normal << 1, 0, 0;
  contacts[0].pos << 20, 0, 0;
  contacts[0].penetration_depth = 0.0;
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(30.01, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(30.01, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(29.9, 0, 0)));
  contacts.resize(1);
  contacts[0].normal << 1, 0, 0;
  contacts[0].pos << 20.0 - 0.1 * 20.0/(20.0 + 10.0), 0, 0;
  contacts[0].penetration_depth = 0.1;
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(29.9, 0, 0)));
  contacts.resize(1);
  contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);
  contacts[0].pos = transform * Vector3<S>(20.0 - 0.1 * 20.0/(20.0 + 10.0), 0, 0);
  contacts[0].penetration_depth = 0.1;
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].normal.setZero();  // If the centers of two sphere are at the same position, the normal is (0, 0, 0)
  contacts[0].pos.setZero();
  contacts[0].penetration_depth = 20.0 + 10.0;
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].normal.setZero();  // If the centers of two sphere are at the same position, the normal is (0, 0, 0)
  contacts[0].pos = transform * Vector3<S>::Zero();
  contacts[0].penetration_depth = 20.0 + 10.0;
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-29.9, 0, 0)));
  contacts.resize(1);
  contacts[0].normal << -1, 0, 0;
  contacts[0].pos << -20.0 + 0.1 * 20.0/(20.0 + 10.0), 0, 0;
  contacts[0].penetration_depth = 0.1;
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-29.9, 0, 0)));
  contacts.resize(1);
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  contacts[0].pos = transform * Vector3<S>(-20.0 + 0.1 * 20.0/(20.0 + 10.0), 0, 0);
  contacts[0].penetration_depth = 0.1;
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-30.0, 0, 0)));
  contacts.resize(1);
  contacts[0].normal << -1, 0, 0;
  contacts[0].pos << -20, 0, 0;
  contacts[0].penetration_depth = 0.0;
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-30.01, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-30.01, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, false);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_spheresphere)
{
//  test_shapeIntersection_spheresphere<float>();
  test_shapeIntersection_spheresphere<double>();
}

template <typename S>
bool compareContactPointds1(const Vector3<S>& c1,const Vector3<S>& c2)
{
  return c1[2] < c2[2];
} // Ascending order

template <typename S>
bool compareContactPointds2(const ContactPoint<S>& cp1,const ContactPoint<S>& cp2)
{
  return cp1.pos[2] < cp2.pos[2];
} // Ascending order

// Simple aligned boxes intersecting each other
//
//        s2  ┌───┐
//            │   │
//  ┏━━━━━━━━━┿━━━┿━━━━━━━━━┓┄┄┄┄┄ z = 0
//  ┃         │┄┄┄│┄┄┄┄┄┄┄┄┄┃┄┄┄ Reported contact depth
//  ┃         └───┘         ┃
//  ┃                       ┃
//  ┃                       ┃
//  ┃    s1                 ┃
//  ┃                       ┃
//  ┃                       ┃
//  ┃                       ┃
//  ┗━━━━━━━━━━━━━━━━━━━━━━━┛
//
//  s1 is a cube, 100 units to a side. Shifted downward 50 units so that it's
//     top face is at z = 0.
//  s2 is a box with dimensions 10, 20, 30 in the x-, y-, and z-directions,
//     respectively. It is centered on the origin and oriented according to the
//     provided rotation matrix `R`.
//  The total penetration depth is the |z_max| where z_max is the z-position of
//  the most deeply penetrating vertex on s2. The contact position will be
//  located at -z_max / 2 (with the assumption that all penetration happens
//  *below* the z = 0 axis.
template <typename Derived>
void testBoxBoxContactPointds(const Eigen::MatrixBase<Derived>& R)
{
  using S = typename Derived::RealScalar;

  Box<S> s1(100, 100, 100);
  Box<S> s2(10, 20, 30);

  // Vertices of s2
  std::vector<Vector3<S>> vertices(8);
  vertices[0] <<  1,  1,  1;
  vertices[1] <<  1,  1, -1;
  vertices[2] <<  1, -1,  1;
  vertices[3] <<  1, -1, -1;
  vertices[4] << -1,  1,  1;
  vertices[5] << -1,  1, -1;
  vertices[6] << -1, -1,  1;
  vertices[7] << -1, -1, -1;

  for (int i = 0; i < 8; ++i)
  {
    vertices[i][0] *= 0.5 * s2.side[0];
    vertices[i][1] *= 0.5 * s2.side[1];
    vertices[i][2] *= 0.5 * s2.side[2];
  }

  Transform3<S> tf1 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -50)));
  Transform3<S> tf2 = Transform3<S>(R);

  std::vector<ContactPoint<S>> contacts;

  // Make sure the two boxes are colliding
  bool res = solver1<S>().shapeIntersect(s1, tf1, s2, tf2, &contacts);
  EXPECT_TRUE(res);

  // Compute global vertices
  for (int i = 0; i < 8; ++i)
    vertices[i] = tf2 * vertices[i];

  // Sort the vertices so that the lowest vertex along z-axis comes first
  std::sort(vertices.begin(), vertices.end(), compareContactPointds1<S>);
  std::sort(contacts.begin(), contacts.end(), compareContactPointds2<S>);

  // The lowest n vertex along z-axis should be the contact point
  size_t numContacts = contacts.size();
  numContacts = std::min(static_cast<size_t>(1), numContacts);
  // TODO: BoxBox algorithm seems not able to find all the colliding vertices.
  // We just check the deepest one as workaround.
  for (size_t i = 0; i < numContacts; ++i)
  {
    // The reported contact position will lie mid-way between the deepest
    // penetrating vertex on s2 and the z = 0 plane.
    Vector3<S> contact_pos(vertices[i]);
    contact_pos(2) /= 2;
    EXPECT_TRUE(contact_pos.isApprox(contacts[i].pos))
              << "\n\tExpected: " << contact_pos
              << "\n\tFound:    " << contacts[i].pos;
    EXPECT_TRUE(Vector3<S>(0, 0, 1).isApprox(contacts[i].normal));
  }
}

template <typename S>
void test_shapeIntersection_boxbox()
{
  Box<S> s1(20, 40, 50);
  Box<S> s2(10, 10, 10);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  Quaternion<S> q(AngleAxis<S>((S)3.140 / 6, Vector3<S>(0, 0, 1)));

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  // TODO: Need convention for normal when the centers of two objects are at same position. The current result is (1, 0, 0).
  contacts.resize(4);
  contacts[0].normal << 1, 0, 0;
  contacts[1].normal << 1, 0, 0;
  contacts[2].normal << 1, 0, 0;
  contacts[3].normal << 1, 0, 0;
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, true);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at same position. The current result is (1, 0, 0).
  contacts.resize(4);
  contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);
  contacts[1].normal = transform.linear() * Vector3<S>(1, 0, 0);
  contacts[2].normal = transform.linear() * Vector3<S>(1, 0, 0);
  contacts[3].normal = transform.linear() * Vector3<S>(1, 0, 0);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, true);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(15, 0, 0)));
  contacts.resize(4);
  contacts[0].normal = Vector3<S>(1, 0, 0);
  contacts[1].normal = Vector3<S>(1, 0, 0);
  contacts[2].normal = Vector3<S>(1, 0, 0);
  contacts[3].normal = Vector3<S>(1, 0, 0);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, true);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(15.01, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(q);
  contacts.resize(4);
  contacts[0].normal = Vector3<S>(1, 0, 0);
  contacts[1].normal = Vector3<S>(1, 0, 0);
  contacts[2].normal = Vector3<S>(1, 0, 0);
  contacts[3].normal = Vector3<S>(1, 0, 0);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, true);

  tf1 = transform;
  tf2 = transform * Transform3<S>(q);
  contacts.resize(4);
  contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);
  contacts[1].normal = transform.linear() * Vector3<S>(1, 0, 0);
  contacts[2].normal = transform.linear() * Vector3<S>(1, 0, 0);
  contacts[3].normal = transform.linear() * Vector3<S>(1, 0, 0);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, true);

  // Disabled for particular configurations: macOS + release + double (see #202)
#if !defined(FCL_OS_MACOS) || !defined(NDEBUG)
  uint32 numTests = 1e+2;
  for (uint32 i = 0; i < numTests; ++i)
  {
    Transform3<S> tf;
    test::generateRandomTransform(extents<S>(), tf);
    testBoxBoxContactPointds(tf.linear());
  }
#endif
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_boxbox)
{
//  test_shapeIntersection_boxbox<float>();
  test_shapeIntersection_boxbox<double>();
}

template <typename S>
void test_shapeIntersection_spherebox()
{
  // A collection of bool constants to make reading lists of bool easier.
  const bool collides = true;
  const bool check_position = true;
  const bool check_depth = true;
  const bool check_normal = true;
  const bool check_opposite_normal = false;

  Sphere<S> s1(20);
  Box<S> s2(5, 5, 5);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  // NOTE: The centers of the box and sphere are coincident. The documented
  // convention is that the normal is aligned with the smallest dimension of
  // the box, pointing in the negative direction of that axis. *This* test is
  // the driving basis for that definition.
  contacts.resize(1);
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, collides, contacts,
                        !check_position, !check_depth, check_normal);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, collides, contacts,
                        !check_position, !check_depth, !check_normal);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(22.5, 0, 0)));
  // As documented in sphere_box.h, touching is considered collision, so this
  // should produce a collision.
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, collides, contacts,
                        !check_position, !check_depth, !check_normal);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(22.501, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, !collides);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(22.4, 0, 0)));
  contacts.resize(1);
  contacts[0].normal << 1, 0, 0;
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, collides, contacts,
                        !check_position, !check_depth, check_normal);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(22.4, 0, 0)));
  contacts.resize(1);
  contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, collides, contacts,
                        !check_position, !check_depth, check_normal,
                        !check_opposite_normal, 1e-4);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_spherebox)
{
//  test_shapeIntersection_spherebox<float>();
  test_shapeIntersection_spherebox<double>();
}

template <typename S>
void test_shapeIntersection_spherecapsule()
{
  Sphere<S> s1(20);
  Capsule<S> s2(5, 10);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  // TODO: Need convention for normal when the centers of two objects are at same position.
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, false);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at same position.
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(24.9, 0, 0)));
  contacts.resize(1);
  contacts[0].normal << 1, 0, 0;
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, true);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(24.9, 0, 0)));
  contacts.resize(1);
  contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, true);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(25, 0, 0)));
  contacts.resize(1);
  contacts[0].normal << 1, 0, 0;
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, true);

  tf1 = transform;
  //tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(25, 0, 0)));
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(25 - 1e-6, 0, 0)));
  contacts.resize(1);
  contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, true);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(25.1, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(25.1, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, false);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_spherecapsule)
{
//  test_shapeIntersection_spherecapsule<float>();
  test_shapeIntersection_spherecapsule<double>();
}

template <typename S>
void test_shapeIntersection_cylindercylinder()
{
  Cylinder<S> s1(5, 10);
  Cylinder<S> s2(5, 10);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  // TODO: Need convention for normal when the centers of two objects are at same position.
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, false);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at same position.
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(9.9, 0, 0)));
  contacts.resize(1);
  contacts[0].normal << 1, 0, 0;
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, true);

  // Disabled for particular configurations: macOS + release + double (see #202)
#if !defined(FCL_OS_MACOS) || !defined(NDEBUG)
  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(9.9, 0, 0)));
  contacts.resize(1);
  contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, true, false, 1e-5);
#endif

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(10.01, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(10.01, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, false);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_cylindercylinder)
{
//  test_shapeIntersection_cylindercylinder<float>();
  test_shapeIntersection_cylindercylinder<double>();
}

template <typename S>
void test_shapeIntersection_conecone()
{
  Cone<S> s1(5, 10);
  Cone<S> s2(5, 10);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  // TODO: Need convention for normal when the centers of two objects are at same position.
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, false);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at same position.
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(9.9, 0, 0)));
  contacts.resize(1);
  contacts[0].normal << 1, 0, 0;
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, true);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(9.9, 0, 0)));
  contacts.resize(1);
  contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, true, false, 5e-5);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(10.001, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(10.001, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 9.9)));
  contacts.resize(1);
  contacts[0].normal << 0, 0, 1;
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, true);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 9.9)));
  contacts.resize(1);
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, 1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, true, false, 1e-5);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_conecone)
{
//  test_shapeIntersection_conecone<float>();
  test_shapeIntersection_conecone<double>();
}

template <typename S>
void test_shapeIntersection_cylindercone()
{
  Cylinder<S> s1(5, 10);
  Cone<S> s2(5, 10);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  // TODO: Need convention for normal when the centers of two objects are at same position.
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, false);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at same position.
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(9.9, 0, 0)));
  contacts.resize(1);
  contacts[0].normal << 1, 0, 0;
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, true, false, 0.061);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(9.9, 0, 0)));
  contacts.resize(1);
  contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, true, false, 0.46);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(10.01, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(10.01, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 9.9)));
  contacts.resize(1);
  contacts[0].normal << 0, 0, 1;
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, true);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 9.9)));
  contacts.resize(1);
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, 1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, true, false, 1e-4);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 10.01)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 10.01)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, false);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_cylindercone)
{
//  test_shapeIntersection_cylindercone<float>();
  test_shapeIntersection_cylindercone<double>();
}

template <typename S>
void test_shapeIntersection_ellipsoidellipsoid()
{
  Ellipsoid<S> s1(20, 40, 50);
  Ellipsoid<S> s2(10, 10, 10);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);
  Transform3<S> identity;

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(30, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(30.01, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(29.99, 0, 0)));
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(29.9, 0, 0)));
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, false);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-29.99, 0, 0)));
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-29.99, 0, 0)));
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, true, contacts, false, false, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-30, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-30.01, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_LIBCCD, false);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_ellipsoidellipsoid)
{
//  test_shapeIntersection_ellipsoidellipsoid<float>();
  test_shapeIntersection_ellipsoidellipsoid<double>();
}

template <typename S>
void test_shapeIntersection_spheretriangle()
{
  Sphere<S> s(10);
  Vector3<S> t[3];
  t[0] << 20, 0, 0;
  t[1] << -20, 0, 0;
  t[2] << 0, 20, 0;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  Vector3<S> normal;
  bool res;

  res = solver1<S>().shapeTriangleIntersect(s, Transform3<S>::Identity(), t[0], t[1], t[2], nullptr, nullptr, nullptr);
  EXPECT_TRUE(res);

  res =  solver1<S>().shapeTriangleIntersect(s, transform, t[0], t[1], t[2], transform, nullptr, nullptr, nullptr);
  EXPECT_TRUE(res);


  t[0] << 30, 0, 0;
  t[1] << 9.9, -20, 0;
  t[2] << 9.9, 20, 0;
  res = solver1<S>().shapeTriangleIntersect(s, Transform3<S>::Identity(), t[0], t[1], t[2], nullptr, nullptr, nullptr);
  EXPECT_TRUE(res);

  res =  solver1<S>().shapeTriangleIntersect(s, transform, t[0], t[1], t[2], transform, nullptr, nullptr, nullptr);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeTriangleIntersect(s, Transform3<S>::Identity(), t[0], t[1], t[2], nullptr, nullptr, &normal);
  EXPECT_TRUE(res);
  EXPECT_TRUE(normal.isApprox(Vector3<S>(1, 0, 0), 1e-9));

  res =  solver1<S>().shapeTriangleIntersect(s, transform, t[0], t[1], t[2], transform, nullptr, nullptr, &normal);
  EXPECT_TRUE(res);
  EXPECT_TRUE(normal.isApprox(transform.linear() * Vector3<S>(1, 0, 0), 1e-9));
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_spheretriangle)
{
//  test_shapeIntersection_spheretriangle<float>();
  test_shapeIntersection_spheretriangle<double>();
}

template <typename S>
void test_shapeIntersection_halfspacetriangle()
{
  Halfspace<S> hs(Vector3<S>(1, 0, 0), 0);
  Vector3<S> t[3];
  t[0] << 20, 0, 0;
  t[1] << -20, 0, 0;
  t[2] << 0, 20, 0;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  // Vector3<S> point;
  // S depth;
  Vector3<S> normal;
  bool res;

  res = solver1<S>().shapeTriangleIntersect(hs, Transform3<S>::Identity(), t[0], t[1], t[2], Transform3<S>::Identity(), nullptr, nullptr, nullptr);
  EXPECT_TRUE(res);

  res =  solver1<S>().shapeTriangleIntersect(hs, transform, t[0], t[1], t[2], transform, nullptr, nullptr, nullptr);
  EXPECT_TRUE(res);


  t[0] << 20, 0, 0;
  t[1] << 0, -20, 0;
  t[2] << 0, 20, 0;
  res = solver1<S>().shapeTriangleIntersect(hs, Transform3<S>::Identity(), t[0], t[1], t[2], Transform3<S>::Identity(), nullptr, nullptr, nullptr);
  EXPECT_TRUE(res);

  // Disabled for particular configurations: macOS + release + double (see #202)
#if !defined(FCL_OS_MACOS) || !defined(NDEBUG)
  res =  solver1<S>().shapeTriangleIntersect(hs, transform, t[0], t[1], t[2], transform, nullptr, nullptr, nullptr);
  EXPECT_TRUE(res);
#endif

  res = solver1<S>().shapeTriangleIntersect(hs, Transform3<S>::Identity(), t[0], t[1], t[2], Transform3<S>::Identity(), nullptr, nullptr, &normal);
  EXPECT_TRUE(res);
  EXPECT_TRUE(normal.isApprox(Vector3<S>(1, 0, 0), 1e-9));

  // Disabled for particular configurations: macOS + release + double (see #202)
#if !defined(FCL_OS_MACOS) || !defined(NDEBUG)
  res =  solver1<S>().shapeTriangleIntersect(hs, transform, t[0], t[1], t[2], transform, nullptr, nullptr, &normal);
  EXPECT_TRUE(res);
  EXPECT_TRUE(normal.isApprox(transform.linear() * Vector3<S>(1, 0, 0), 1e-9));
#endif
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_halfspacetriangle)
{
//  test_shapeIntersection_halfspacetriangle<float>();
  test_shapeIntersection_halfspacetriangle<double>();
}

template <typename S>
void test_shapeIntersection_planetriangle()
{
  Plane<S> hs(Vector3<S>(1, 0, 0), 0);
  Vector3<S> t[3];
  t[0] << 20, 0, 0;
  t[1] << -20, 0, 0;
  t[2] << 0, 20, 0;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  // Vector3<S> point;
  // S depth;
  Vector3<S> normal;
  bool res;

  res = solver1<S>().shapeTriangleIntersect(hs, Transform3<S>::Identity(), t[0], t[1], t[2], Transform3<S>::Identity(), nullptr, nullptr, nullptr);
  EXPECT_TRUE(res);

  res =  solver1<S>().shapeTriangleIntersect(hs, transform, t[0], t[1], t[2], transform, nullptr, nullptr, nullptr);
  EXPECT_TRUE(res);


  t[0] << 20, 0, 0;
  t[1] << -0.1, -20, 0;
  t[2] << -0.1, 20, 0;
  res = solver1<S>().shapeTriangleIntersect(hs, Transform3<S>::Identity(), t[0], t[1], t[2], Transform3<S>::Identity(), nullptr, nullptr, nullptr);
  EXPECT_TRUE(res);

  res =  solver1<S>().shapeTriangleIntersect(hs, transform, t[0], t[1], t[2], transform, nullptr, nullptr, nullptr);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeTriangleIntersect(hs, Transform3<S>::Identity(), t[0], t[1], t[2], Transform3<S>::Identity(), nullptr, nullptr, &normal);
  EXPECT_TRUE(res);
  EXPECT_TRUE(normal.isApprox(Vector3<S>(1, 0, 0), 1e-9));

  res =  solver1<S>().shapeTriangleIntersect(hs, transform, t[0], t[1], t[2], transform, nullptr, nullptr, &normal);
  EXPECT_TRUE(res);
  EXPECT_TRUE(normal.isApprox(transform.linear() * Vector3<S>(1, 0, 0), 1e-9));
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_planetriangle)
{
//  test_shapeIntersection_planetriangle<float>();
  test_shapeIntersection_planetriangle<double>();
}

template <typename S>
void test_shapeIntersection_halfspacesphere()
{
  Sphere<S> s(10);
  Halfspace<S> hs(Vector3<S>(1, 0, 0), 0);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << -5, 0, 0;
  contacts[0].penetration_depth = 10;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(-5, 0, 0);
  contacts[0].penetration_depth = 10;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << -2.5, 0, 0;
  contacts[0].penetration_depth = 15;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(-2.5, 0, 0);
  contacts[0].penetration_depth = 15;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << -7.5, 0, 0;
  contacts[0].penetration_depth = 5;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(-7.5, 0, 0);
  contacts[0].penetration_depth = 5;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-10.1, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-10.1, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(10.1, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << 0.05, 0, 0;
  contacts[0].penetration_depth = 20.1;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(10.1, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0.05, 0, 0);
  contacts[0].penetration_depth = 20.1;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_halfspacesphere)
{
//  test_shapeIntersection_halfspacesphere<float>();
  test_shapeIntersection_halfspacesphere<double>();
}

template <typename S>
void test_shapeIntersection_planesphere()
{
  Sphere<S> s(10);
  Plane<S> hs(Vector3<S>(1, 0, 0), 0);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos.setZero();
  contacts[0].penetration_depth = 10;
  contacts[0].normal << 1, 0, 0;  // (1, 0, 0) or (-1, 0, 0)
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, 0);
  contacts[0].penetration_depth = 10;
  contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);  // (1, 0, 0) or (-1, 0, 0)
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << 5, 0, 0;
  contacts[0].penetration_depth = 5;
  contacts[0].normal << 1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(5, 0, 0);
  contacts[0].penetration_depth = 5;
  contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << -5, 0, 0;
  contacts[0].penetration_depth = 5;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(-5, 0, 0);
  contacts[0].penetration_depth = 5;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-10.1, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-10.1, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(10.1, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(10.1, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_planesphere)
{
//  test_shapeIntersection_planesphere<float>();
  test_shapeIntersection_planesphere<double>();
}

template <typename S>
void test_shapeIntersection_halfspacebox()
{
  Box<S> s(5, 10, 20);
  Halfspace<S> hs(Vector3<S>(1, 0, 0), 0);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << -1.25, 0, 0;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(-1.25, 0, 0);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(1.25, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << -0.625, 0, 0;
  contacts[0].penetration_depth = 3.75;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(1.25, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(-0.625, 0, 0);
  contacts[0].penetration_depth = 3.75;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-1.25, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << -1.875, 0, 0;
  contacts[0].penetration_depth = 1.25;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-1.25, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(-1.875, 0, 0);
  contacts[0].penetration_depth = 1.25;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(2.51, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << 0.005, 0, 0;
  contacts[0].penetration_depth = 5.01;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(2.51, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0.005, 0, 0);
  contacts[0].penetration_depth = 5.01;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-2.51, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-2.51, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = Transform3<S>(transform.linear());
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, false, false, false);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_halfspacebox)
{
//  test_shapeIntersection_halfspacebox<float>();
  test_shapeIntersection_halfspacebox<double>();
}

template <typename S>
void test_shapeIntersection_planebox()
{
  Box<S> s(5, 10, 20);
  Plane<S> hs(Vector3<S>(1, 0, 0), 0);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << 0, 0, 0;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << 1, 0, 0;  // (1, 0, 0) or (-1, 0, 0)
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, 0);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);  // (1, 0, 0) or (-1, 0, 0)
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(1.25, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << 1.25, 0, 0;
  contacts[0].penetration_depth = 1.25;
  contacts[0].normal << 1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(1.25, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(1.25, 0, 0);
  contacts[0].penetration_depth = 1.25;
  contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-1.25, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << -1.25, 0, 0;
  contacts[0].penetration_depth = 1.25;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-1.25, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(-1.25, 0, 0);
  contacts[0].penetration_depth = 1.25;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(2.51, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(2.51, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-2.51, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-2.51, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = Transform3<S>(transform.linear());
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, false, false, false);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_planebox)
{
//  test_shapeIntersection_planebox<float>();
  test_shapeIntersection_planebox<double>();
}

template <typename S>
void test_shapeIntersection_halfspaceellipsoid()
{
  Ellipsoid<S> s(5, 10, 20);
  Halfspace<S> hs(Vector3<S>(1, 0, 0), 0);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << -2.5, 0, 0;
  contacts[0].penetration_depth = 5.0;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(-2.5, 0, 0);
  contacts[0].penetration_depth = 5.0;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(1.25, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << -1.875, 0, 0;
  contacts[0].penetration_depth = 6.25;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(1.25, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(-1.875, 0, 0);
  contacts[0].penetration_depth = 6.25;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-1.25, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << -3.125, 0, 0;
  contacts[0].penetration_depth = 3.75;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-1.25, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(-3.125, 0, 0);
  contacts[0].penetration_depth = 3.75;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(5.01, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << 0.005, 0, 0;
  contacts[0].penetration_depth = 10.01;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(5.01, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0.005, 0, 0);
  contacts[0].penetration_depth = 10.01;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-5.01, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-5.01, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);




  hs = Halfspace<S>(Vector3<S>(0, 1, 0), 0);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << 0, -5.0, 0;
  contacts[0].penetration_depth = 10.0;
  contacts[0].normal << 0, -1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, -5.0, 0);
  contacts[0].penetration_depth = 10.0;
  contacts[0].normal = transform.linear() * Vector3<S>(0, -1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 1.25, 0)));
  contacts.resize(1);
  contacts[0].pos << 0, -4.375, 0;
  contacts[0].penetration_depth = 11.25;
  contacts[0].normal << 0, -1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 1.25, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, -4.375, 0);
  contacts[0].penetration_depth = 11.25;
  contacts[0].normal = transform.linear() * Vector3<S>(0, -1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, -1.25, 0)));
  contacts.resize(1);
  contacts[0].pos << 0, -5.625, 0;
  contacts[0].penetration_depth = 8.75;
  contacts[0].normal << 0, -1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, -1.25, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, -5.625, 0);
  contacts[0].penetration_depth = 8.75;
  contacts[0].normal = transform.linear() * Vector3<S>(0, -1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 10.01, 0)));
  contacts.resize(1);
  contacts[0].pos << 0, 0.005, 0;
  contacts[0].penetration_depth = 20.01;
  contacts[0].normal << 0, -1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 10.01, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0.005, 0);
  contacts[0].penetration_depth = 20.01;
  contacts[0].normal = transform.linear() * Vector3<S>(0, -1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, -10.01, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, -10.01, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);




  hs = Halfspace<S>(Vector3<S>(0, 0, 1), 0);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << 0, 0, -10.0;
  contacts[0].penetration_depth = 20.0;
  contacts[0].normal << 0, 0, -1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, -10.0);
  contacts[0].penetration_depth = 20.0;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, -1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 1.25)));
  contacts.resize(1);
  contacts[0].pos << 0, 0, -9.375;
  contacts[0].penetration_depth = 21.25;
  contacts[0].normal << 0, 0, -1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 1.25)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, -9.375);
  contacts[0].penetration_depth = 21.25;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, -1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -1.25)));
  contacts.resize(1);
  contacts[0].pos << 0, 0, -10.625;
  contacts[0].penetration_depth = 18.75;
  contacts[0].normal << 0, 0, -1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -1.25)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, -10.625);
  contacts[0].penetration_depth = 18.75;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, -1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 20.01)));
  contacts.resize(1);
  contacts[0].pos << 0, 0, 0.005;
  contacts[0].penetration_depth = 40.01;
  contacts[0].normal << 0, 0, -1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 20.01)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, 0.005);
  contacts[0].penetration_depth = 40.01;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, -1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -20.01)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -20.01)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_halfspaceellipsoid)
{
//  test_shapeIntersection_halfspaceellipsoid<float>();
  test_shapeIntersection_halfspaceellipsoid<double>();
}

template <typename S>
void test_shapeIntersection_planeellipsoid()
{
  Ellipsoid<S> s(5, 10, 20);
  Plane<S> hs(Vector3<S>(1, 0, 0), 0);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << 0, 0, 0;
  contacts[0].penetration_depth = 5.0;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, 0);
  contacts[0].penetration_depth = 5.0;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(1.25, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << 1.25, 0, 0;
  contacts[0].penetration_depth = 3.75;
  contacts[0].normal << 1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(1.25, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(1.25, 0, 0);
  contacts[0].penetration_depth = 3.75;
  contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-1.25, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << -1.25, 0, 0;
  contacts[0].penetration_depth = 3.75;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-1.25, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(-1.25, 0, 0);
  contacts[0].penetration_depth = 3.75;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(5.01, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(5.01, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-5.01, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-5.01, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);




  hs = Plane<S>(Vector3<S>(0, 1, 0), 0);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << 0, 0.0, 0;
  contacts[0].penetration_depth = 10.0;
  contacts[0].normal << 0, -1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, 0);
  contacts[0].penetration_depth = 10.0;
  contacts[0].normal = transform.linear() * Vector3<S>(0, -1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 1.25, 0)));
  contacts.resize(1);
  contacts[0].pos << 0, 1.25, 0;
  contacts[0].penetration_depth = 8.75;
  contacts[0].normal << 0, 1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 1.25, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 1.25, 0);
  contacts[0].penetration_depth = 8.75;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, -1.25, 0)));
  contacts.resize(1);
  contacts[0].pos << 0, -1.25, 0;
  contacts[0].penetration_depth = 8.75;
  contacts[0].normal << 0, -1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, -1.25, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, -1.25, 0);
  contacts[0].penetration_depth = 8.75;
  contacts[0].normal = transform.linear() * Vector3<S>(0, -1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 10.01, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 10.01, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, -10.01, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, -10.01, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);




  hs = Plane<S>(Vector3<S>(0, 0, 1), 0);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << 0, 0, 0;
  contacts[0].penetration_depth = 20.0;
  contacts[0].normal << 0, 0, -1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, 0);
  contacts[0].penetration_depth = 20.0;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, -1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 1.25)));
  contacts.resize(1);
  contacts[0].pos << 0, 0, 1.25;
  contacts[0].penetration_depth = 18.75;
  contacts[0].normal << 0, 0, 1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 1.25)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, 1.25);
  contacts[0].penetration_depth = 18.75;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, 1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -1.25)));
  contacts.resize(1);
  contacts[0].pos << 0, 0, -1.25;
  contacts[0].penetration_depth = 18.75;
  contacts[0].normal << 0, 0, -1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -1.25)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, -1.25);
  contacts[0].penetration_depth = 18.75;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, -1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 20.01)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 20.01)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -20.01)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -20.01)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_planeellipsoid)
{
//  test_shapeIntersection_planeellipsoid<float>();
  test_shapeIntersection_planeellipsoid<double>();
}

template <typename S>
void test_shapeIntersection_halfspacecapsule()
{
  Capsule<S> s(5, 10);
  Halfspace<S> hs(Vector3<S>(1, 0, 0), 0);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << -2.5, 0, 0;
  contacts[0].penetration_depth = 5;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(-2.5, 0, 0);
  contacts[0].penetration_depth = 5;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(2.5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << -1.25, 0, 0;
  contacts[0].penetration_depth = 7.5;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(2.5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(-1.25, 0, 0);
  contacts[0].penetration_depth = 7.5;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-2.5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << -3.75, 0, 0;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-2.5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(-3.75, 0, 0);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(5.1, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << 0.05, 0, 0;
  contacts[0].penetration_depth = 10.1;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(5.1, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0.05, 0, 0);
  contacts[0].penetration_depth = 10.1;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-5.1, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-5.1, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);




  hs = Halfspace<S>(Vector3<S>(0, 1, 0), 0);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << 0, -2.5, 0;
  contacts[0].penetration_depth = 5;
  contacts[0].normal << 0, -1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, -2.5, 0);
  contacts[0].penetration_depth = 5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, -1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 2.5, 0)));
  contacts.resize(1);
  contacts[0].pos << 0, -1.25, 0;
  contacts[0].penetration_depth = 7.5;
  contacts[0].normal << 0, -1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 2.5, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, -1.25, 0);
  contacts[0].penetration_depth = 7.5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, -1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, -2.5, 0)));
  contacts.resize(1);
  contacts[0].pos << 0, -3.75, 0;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << 0, -1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, -2.5, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, -3.75, 0);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, -1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 5.1, 0)));
  contacts.resize(1);
  contacts[0].pos << 0, 0.05, 0;
  contacts[0].penetration_depth = 10.1;
  contacts[0].normal << 0, -1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 5.1, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0.05, 0);
  contacts[0].penetration_depth = 10.1;
  contacts[0].normal = transform.linear() * Vector3<S>(0, -1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, -5.1, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, -5.1, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);




  hs = Halfspace<S>(Vector3<S>(0, 0, 1), 0);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << 0, 0, -5;
  contacts[0].penetration_depth = 10;
  contacts[0].normal << 0, 0, -1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, -5);
  contacts[0].penetration_depth = 10;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, -1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 2.5)));
  contacts.resize(1);
  contacts[0].pos << 0, 0, -3.75;
  contacts[0].penetration_depth = 12.5;
  contacts[0].normal << 0, 0, -1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 2.5)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, -3.75);
  contacts[0].penetration_depth = 12.5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, -1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -2.5)));
  contacts.resize(1);
  contacts[0].pos << 0, 0, -6.25;
  contacts[0].penetration_depth = 7.5;
  contacts[0].normal << 0, 0, -1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -2.5)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, -6.25);
  contacts[0].penetration_depth  = 7.5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, -1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 10.1)));
  contacts.resize(1);
  contacts[0].pos << 0, 0, 0.05;
  contacts[0].penetration_depth = 20.1;
  contacts[0].normal << 0, 0, -1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 10.1)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, 0.05);
  contacts[0].penetration_depth = 20.1;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, -1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -10.1)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -10.1)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_halfspacecapsule)
{
//  test_shapeIntersection_halfspacecapsule<float>();
  test_shapeIntersection_halfspacecapsule<double>();
}

template <typename S>
void test_shapeIntersection_planecapsule()
{
  Capsule<S> s(5, 10);
  Plane<S> hs(Vector3<S>(1, 0, 0), 0);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << 0, 0, 0;
  contacts[0].penetration_depth = 5;
  contacts[0].normal << 1, 0, 0;  // (1, 0, 0) or (-1, 0, 0)
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, 0);
  contacts[0].penetration_depth = 5;
  contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);  // (1, 0, 0) or (-1, 0, 0)
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(2.5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << 2.5, 0, 0;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << 1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(2.5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(2.5, 0, 0);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-2.5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << -2.5, 0, 0;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-2.5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(-2.5, 0, 0);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(5.1, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(5.1, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-5.1, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-5.1, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);




  hs = Plane<S>(Vector3<S>(0, 1, 0), 0);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << 0, 0, 0;
  contacts[0].penetration_depth = 5;
  contacts[0].normal << 0, 1, 0;  // (0, 1, 0) or (0, -1, 0)
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, 0);
  contacts[0].penetration_depth = 5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 1, 0);  // (0, 1, 0) or (0, -1, 0)
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 2.5, 0)));
  contacts.resize(1);
  contacts[0].pos << 0, 2.5, 0;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << 0, 1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 2.5, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 2.5, 0);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, -2.5, 0)));
  contacts.resize(1);
  contacts[0].pos << 0, -2.5, 0;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << 0, -1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, -2.5, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, -2.5, 0);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, -1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 5.1, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 5.1, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, -5.1, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, -5.1, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);




  hs = Plane<S>(Vector3<S>(0, 0, 1), 0);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << 0, 0, 0;
  contacts[0].penetration_depth = 10;
  contacts[0].normal << 0, 0, 1;  // (0, 0, 1) or (0, 0, -1)
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, 0);
  contacts[0].penetration_depth = 10;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, 1);  // (0, 0, 1) or (0, 0, -1)
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 2.5)));
  contacts.resize(1);
  contacts[0].pos << 0, 0, 2.5;
  contacts[0].penetration_depth = 7.5;
  contacts[0].normal << 0, 0, 1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 2.5)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, 2.5);
  contacts[0].penetration_depth = 7.5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, 1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -2.5)));
  contacts.resize(1);
  contacts[0].pos << 0, 0, -2.5;
  contacts[0].penetration_depth = 7.5;
  contacts[0].normal << 0, 0, -1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -2.5)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, -2.5);
  contacts[0].penetration_depth = 7.5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, -1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 10.1)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 10.1)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -10.1)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -10.1)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_planecapsule)
{
//  test_shapeIntersection_planecapsule<float>();
  test_shapeIntersection_planecapsule<double>();
}

template <typename S>
void test_shapeIntersection_halfspacecylinder()
{
  Cylinder<S> s(5, 10);
  Halfspace<S> hs(Vector3<S>(1, 0, 0), 0);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << -2.5, 0, 0;
  contacts[0].penetration_depth = 5;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(-2.5, 0, 0);
  contacts[0].penetration_depth = 5;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(2.5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << -1.25, 0, 0;
  contacts[0].penetration_depth = 7.5;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(2.5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(-1.25, 0, 0);
  contacts[0].penetration_depth = 7.5;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-2.5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << -3.75, 0, 0;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-2.5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(-3.75, 0, 0);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(5.1, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << 0.05, 0, 0;
  contacts[0].penetration_depth = 10.1;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(5.1, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0.05, 0, 0);
  contacts[0].penetration_depth = 10.1;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-5.1, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-5.1, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);




  hs = Halfspace<S>(Vector3<S>(0, 1, 0), 0);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << 0, -2.5, 0;
  contacts[0].penetration_depth = 5;
  contacts[0].normal << 0, -1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, -2.5, 0);
  contacts[0].penetration_depth = 5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, -1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 2.5, 0)));
  contacts.resize(1);
  contacts[0].pos << 0, -1.25, 0;
  contacts[0].penetration_depth = 7.5;
  contacts[0].normal << 0, -1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 2.5, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, -1.25, 0);
  contacts[0].penetration_depth = 7.5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, -1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, -2.5, 0)));
  contacts.resize(1);
  contacts[0].pos << 0, -3.75, 0;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << 0, -1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, -2.5, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, -3.75, 0);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, -1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 5.1, 0)));
  contacts.resize(1);
  contacts[0].pos << 0, 0.05, 0;
  contacts[0].penetration_depth = 10.1;
  contacts[0].normal << 0, -1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 5.1, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0.05, 0);
  contacts[0].penetration_depth = 10.1;
  contacts[0].normal = transform.linear() * Vector3<S>(0, -1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, -5.1, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, -5.1, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);




  hs = Halfspace<S>(Vector3<S>(0, 0, 1), 0);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << 0, 0, -2.5;
  contacts[0].penetration_depth = 5;
  contacts[0].normal << 0, 0, -1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, -2.5);
  contacts[0].penetration_depth = 5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, -1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 2.5)));
  contacts.resize(1);
  contacts[0].pos << 0, 0, -1.25;
  contacts[0].penetration_depth = 7.5;
  contacts[0].normal << 0, 0, -1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 2.5)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, -1.25);
  contacts[0].penetration_depth = 7.5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, -1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -2.5)));
  contacts.resize(1);
  contacts[0].pos << 0, 0, -3.75;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << 0, 0, -1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -2.5)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, -3.75);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, -1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 5.1)));
  contacts.resize(1);
  contacts[0].pos << 0, 0, 0.05;
  contacts[0].penetration_depth = 10.1;
  contacts[0].normal << 0, 0, -1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 5.1)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, 0.05);
  contacts[0].penetration_depth = 10.1;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, -1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -5.1)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -5.1)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_halfspacecylinder)
{
//  test_shapeIntersection_halfspacecylinder<float>();
  test_shapeIntersection_halfspacecylinder<double>();
}

template <typename S>
void test_shapeIntersection_planecylinder()
{
  Cylinder<S> s(5, 10);
  Plane<S> hs(Vector3<S>(1, 0, 0), 0);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << 0, 0, 0;
  contacts[0].penetration_depth = 5;
  contacts[0].normal << 1, 0, 0;  // (1, 0, 0) or (-1, 0, 0)
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, 0);
  contacts[0].penetration_depth = 5;
  contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);  // (1, 0, 0) or (-1, 0, 0)
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(2.5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << 2.5, 0, 0;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << 1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(2.5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(2.5, 0, 0);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-2.5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << -2.5, 0, 0;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-2.5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(-2.5, 0, 0);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(5.1, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(5.1, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-5.1, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-5.1, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);




  hs = Plane<S>(Vector3<S>(0, 1, 0), 0);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << 0, 0, 0;
  contacts[0].penetration_depth = 5;
  contacts[0].normal << 0, 1, 0;  // (1, 0, 0) or (-1, 0, 0)
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, 0);
  contacts[0].penetration_depth = 5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 1, 0);  // (1, 0, 0) or (-1, 0, 0)
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 2.5, 0)));
  contacts.resize(1);
  contacts[0].pos << 0, 2.5, 0;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << 0, 1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 2.5, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 2.5, 0);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, -2.5, 0)));
  contacts.resize(1);
  contacts[0].pos << 0, -2.5, 0;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << 0, -1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, -2.5, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, -2.5, 0);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, -1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 5.1, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 5.1, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, -5.1, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, -5.1, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);




  hs = Plane<S>(Vector3<S>(0, 0, 1), 0);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << 0, 0, 0;
  contacts[0].penetration_depth = 5;
  contacts[0].normal << 0, 0, 1;  // (1, 0, 0) or (-1, 0, 0)
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, 0);
  contacts[0].penetration_depth = 5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, 1);  // (1, 0, 0) or (-1, 0, 0)
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 2.5)));
  contacts.resize(1);
  contacts[0].pos << 0, 0, 2.5;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << 0, 0, 1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 2.5)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, 2.5);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, 1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -2.5)));
  contacts.resize(1);
  contacts[0].pos << 0, 0, -2.5;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << 0, 0, -1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -2.5)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, -2.5);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, -1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 10.1)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 10.1)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -10.1)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -10.1)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_planecylinder)
{
//  test_shapeIntersection_planecylinder<float>();
  test_shapeIntersection_planecylinder<double>();
}

template <typename S>
void test_shapeIntersection_halfspacecone()
{
  Cone<S> s(5, 10);
  Halfspace<S> hs(Vector3<S>(1, 0, 0), 0);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << -2.5, 0, -5;
  contacts[0].penetration_depth = 5;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(-2.5, 0, -5);
  contacts[0].penetration_depth = 5;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(2.5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << -1.25, 0, -5;
  contacts[0].penetration_depth = 7.5;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(2.5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(-1.25, 0, -5);
  contacts[0].penetration_depth = 7.5;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-2.5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << -3.75, 0, -5;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-2.5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(-3.75, 0, -5);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(5.1, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << 0.05, 0, -5;
  contacts[0].penetration_depth = 10.1;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(5.1, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0.05, 0, -5);
  contacts[0].penetration_depth = 10.1;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-5.1, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-5.1, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);




  hs = Halfspace<S>(Vector3<S>(0, 1, 0), 0);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << 0, -2.5, -5;
  contacts[0].penetration_depth = 5;
  contacts[0].normal << 0, -1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, -2.5, -5);
  contacts[0].penetration_depth = 5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, -1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 2.5, 0)));
  contacts.resize(1);
  contacts[0].pos << 0, -1.25, -5;
  contacts[0].penetration_depth = 7.5;
  contacts[0].normal << 0, -1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 2.5, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, -1.25, -5);
  contacts[0].penetration_depth = 7.5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, -1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, -2.5, 0)));
  contacts.resize(1);
  contacts[0].pos << 0, -3.75, -5;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << 0, -1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, -2.5, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, -3.75, -5);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, -1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 5.1, 0)));
  contacts.resize(1);
  contacts[0].pos << 0, 0.05, -5;
  contacts[0].penetration_depth = 10.1;
  contacts[0].normal << 0, -1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 5.1, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0.05, -5);
  contacts[0].penetration_depth = 10.1;
  contacts[0].normal = transform.linear() * Vector3<S>(0, -1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, -5.1, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, -5.1, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);




  hs = Halfspace<S>(Vector3<S>(0, 0, 1), 0);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << 0, 0, -2.5;
  contacts[0].penetration_depth = 5;
  contacts[0].normal << 0, 0, -1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, -2.5);
  contacts[0].penetration_depth = 5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, -1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 2.5)));
  contacts.resize(1);
  contacts[0].pos << 0, 0, -1.25;
  contacts[0].penetration_depth = 7.5;
  contacts[0].normal << 0, 0, -1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 2.5)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, -1.25);
  contacts[0].penetration_depth = 7.5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, -1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -2.5)));
  contacts.resize(1);
  contacts[0].pos << 0, 0, -3.75;
  contacts[0].penetration_depth= 2.5;
  contacts[0].normal << 0, 0, -1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -2.5)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, -3.75);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, -1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 5.1)));
  contacts.resize(1);
  contacts[0].pos << 0, 0, 0.05;
  contacts[0].penetration_depth = 10.1;
  contacts[0].normal << 0, 0, -1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 5.1)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, 0.05);
  contacts[0].penetration_depth = 10.1;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, -1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -5.1)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -5.1)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_halfspacecone)
{
//  test_shapeIntersection_halfspacecone<float>();
  test_shapeIntersection_halfspacecone<double>();
}

template <typename S>
void test_shapeIntersection_planecone()
{
  Cone<S> s(5, 10);
  Plane<S> hs(Vector3<S>(1, 0, 0), 0);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << 0, 0, 0;
  contacts[0].penetration_depth = 5;
  contacts[0].normal << 1, 0, 0;  // (1, 0, 0) or (-1, 0, 0)
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, 0);
  contacts[0].penetration_depth = 5;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);  // (1, 0, 0) or (-1, 0, 0)
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(2.5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << 2.5, 0, -2.5;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << 1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(2.5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(2.5, 0, -2.5);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-2.5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos << -2.5, 0, -2.5;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << -1, 0, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-2.5, 0, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(-2.5, 0, -2.5);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(5.1, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(5.1, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-5.1, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-5.1, 0, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);




  hs = Plane<S>(Vector3<S>(0, 1, 0), 0);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << 0, 0, 0;
  contacts[0].penetration_depth = 5;
  contacts[0].normal << 0, 1, 0;  // (1, 0, 0) or (-1, 0, 0)
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, 0);
  contacts[0].penetration_depth = 5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 1, 0);  // (1, 0, 0) or (-1, 0, 0)
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 2.5, 0)));
  contacts.resize(1);
  contacts[0].pos << 0, 2.5, -2.5;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << 0, 1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 2.5, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 2.5, -2.5);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, -2.5, 0)));
  contacts.resize(1);
  contacts[0].pos << 0, -2.5, -2.5;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << 0, -1, 0;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, -2.5, 0)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, -2.5, -2.5);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, -1, 0);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 5.1, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 5.1, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, -5.1, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, -5.1, 0)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);




  hs = Plane<S>(Vector3<S>(0, 0, 1), 0);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].pos << 0, 0, 0;
  contacts[0].penetration_depth = 5;
  contacts[0].normal << 0, 0, 1;  // (1, 0, 0) or (-1, 0, 0)
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, 0);
  contacts[0].penetration_depth = 5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, 1);  // (1, 0, 0) or (-1, 0, 0)
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts, true, true, true, true);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 2.5)));
  contacts.resize(1);
  contacts[0].pos << 0, 0, 2.5;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << 0, 0, 1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 2.5)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, 2.5);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, 1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -2.5)));
  contacts.resize(1);
  contacts[0].pos << 0, 0, -2.5;
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal << 0, 0, -1;
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -2.5)));
  contacts.resize(1);
  contacts[0].pos = transform * Vector3<S>(0, 0, -2.5);
  contacts[0].penetration_depth = 2.5;
  contacts[0].normal = transform.linear() * Vector3<S>(0, 0, -1);
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 10.1)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 10.1)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -10.1)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, -10.1)));
  testShapeIntersection(s, tf1, hs, tf2, GST_LIBCCD, false);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_planecone)
{
//  test_shapeIntersection_planecone<float>();
  test_shapeIntersection_planecone<double>();
}

// Shape distance test coverage (libccd)
//
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// |            | box | sphere | ellipsoid | capsule | cone | cylinder | plane | half-space | triangle |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | box        |  O  |   O    |           |         |      |          |       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | sphere     |/////|   O    |           |         |      |          |       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | ellipsoid  |/////|////////|     O     |         |      |          |       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | capsule    |/////|////////|///////////|         |      |          |       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | cone       |/////|////////|///////////|/////////|  O   |    O     |       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | cylinder   |/////|////////|///////////|/////////|//////|    O     |       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | plane      |/////|////////|///////////|/////////|//////|//////////|       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | half-space |/////|////////|///////////|/////////|//////|//////////|///////|            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | triangle   |/////|////////|///////////|/////////|//////|//////////|///////|////////////|          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+

template <typename S>
void test_shapeDistance_spheresphere()
{
  Sphere<S> s1(20);
  Sphere<S> s2(10);

  Transform3<S> transform = Transform3<S>::Identity();
  //test::generateRandomTransform(extents<S>(), transform);

  bool res;
  S dist = -1;
  Vector3<S> closest_p1, closest_p2;
  res = solver1<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(0, 40, 0))), &dist, &closest_p1, &closest_p2);
  EXPECT_TRUE(fabs(dist - 10) < 0.001);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(30.1, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(29.9, 0, 0))), &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver1<S>().shapeDistance(s1, Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), s2, Transform3<S>::Identity(), &dist);
  EXPECT_TRUE(fabs(dist - 10) < 0.001);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, Transform3<S>(Translation3<S>(Vector3<S>(30.1, 0, 0))), s2, Transform3<S>::Identity(), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, Transform3<S>(Translation3<S>(Vector3<S>(29.9, 0, 0))), s2, Transform3<S>::Identity(), &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);


  res = solver1<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), &dist);
  // this is one problem: the precise is low sometimes
  EXPECT_TRUE(fabs(dist - 10) < 0.1);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(30.1, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.06);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(29.9, 0, 0))), &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver1<S>().shapeDistance(s1, transform * Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), s2, transform, &dist);
  EXPECT_TRUE(fabs(dist - 10) < 0.1);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, transform * Transform3<S>(Translation3<S>(Vector3<S>(30.1, 0, 0))), s2, transform, &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.1);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, transform * Transform3<S>(Translation3<S>(Vector3<S>(29.9, 0, 0))), s2, transform, &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeDistance_spheresphere)
{
//  test_shapeDistance_spheresphere<float>();
  test_shapeDistance_spheresphere<double>();
}

template <typename S>
void test_shapeDistance_boxbox()
{
  Box<S> s1(20, 40, 50);
  Box<S> s2(10, 10, 10);
  Vector3<S> closest_p1, closest_p2;

  Transform3<S> transform = Transform3<S>::Identity();
  //test::generateRandomTransform(extents<S>(), transform);

  bool res;
  S dist;

  res = solver1<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>::Identity(), &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver1<S>().shapeDistance(s1, transform, s2, transform, &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver1<S>().shapeDistance(s2, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(10.1, 0, 0))), &dist, &closest_p1, &closest_p2);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s2, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(20.1, 0, 0))), &dist, &closest_p1, &closest_p2);
  EXPECT_TRUE(fabs(dist - 10.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s2, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(0, 20.2, 0))), &dist, &closest_p1, &closest_p2);
  EXPECT_TRUE(fabs(dist - 10.2) < 0.001);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s2, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(10.1, 10.1, 0))), &dist, &closest_p1, &closest_p2);
  EXPECT_TRUE(fabs(dist - 0.1 * 1.414) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s2, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(10.1, 0, 0))), &dist, &closest_p1, &closest_p2);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s2, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(20.1, 0, 0))), &dist, &closest_p1, &closest_p2);
  EXPECT_TRUE(fabs(dist - 10.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s2, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(0, 20.1, 0))), &dist, &closest_p1, &closest_p2);
  EXPECT_TRUE(fabs(dist - 10.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s2, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(10.1, 10.1, 0))), &dist, &closest_p1, &closest_p2);
  EXPECT_TRUE(fabs(dist - 0.1 * 1.414) < 0.001);
  EXPECT_TRUE(res);


  res = solver1<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(15.1, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(20, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 5) < 0.001);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(20, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 5) < 0.001);
  EXPECT_TRUE(res);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeDistance_boxbox)
{
//  test_shapeDistance_boxbox<float>();
  test_shapeDistance_boxbox<double>();
}

template <typename S>
void test_shapeDistance_boxsphere()
{
  Sphere<S> s1(20);
  Box<S> s2(5, 5, 5);

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  bool res;
  S dist;

  res = solver1<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>::Identity(), &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver1<S>().shapeDistance(s1, transform, s2, transform, &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver1<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(22.6, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(22.6, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.05);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 17.5) < 0.001);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 17.5) < 0.001);
  EXPECT_TRUE(res);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeDistance_boxsphere)
{
//  test_shapeDistance_boxsphere<float>();
  test_shapeDistance_boxsphere<double>();
}

template <typename S>
void test_shapeDistance_cylindercylinder()
{
  Cylinder<S> s1(5, 10);
  Cylinder<S> s2(5, 10);

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  bool res;
  S dist;

  res = solver1<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>::Identity(), &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver1<S>().shapeDistance(s1, transform, s2, transform, &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver1<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(10.1, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(10.1, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 30) < 0.001);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 30) < 0.001);
  EXPECT_TRUE(res);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeDistance_cylindercylinder)
{
//  test_shapeDistance_cylindercylinder<float>();
  test_shapeDistance_cylindercylinder<double>();
}

template <typename S>
void test_shapeDistance_conecone()
{
  Cone<S> s1(5, 10);
  Cone<S> s2(5, 10);

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  bool res;
  S dist;

  res = solver1<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>::Identity(), &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver1<S>().shapeDistance(s1, transform, s2, transform, &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver1<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(10.1, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(10.1, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 40))), &dist);
  EXPECT_TRUE(fabs(dist - 30) < 1);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 40))), &dist);
  EXPECT_TRUE(fabs(dist - 30) < 1);
  EXPECT_TRUE(res);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeDistance_conecone)
{
//  test_shapeDistance_conecone<float>();
  test_shapeDistance_conecone<double>();
}

template <typename S>
void test_shapeDistance_conecylinder()
{
  Cylinder<S> s1(5, 10);
  Cone<S> s2(5, 10);

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  bool res;
  S dist;

  res = solver1<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>::Identity(), &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver1<S>().shapeDistance(s1, transform, s2, transform, &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver1<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(10.1, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.01);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(10.1, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.02);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 30) < 0.01);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 30) < 0.1);
  EXPECT_TRUE(res);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeDistance_conecylinder)
{
//  test_shapeDistance_conecylinder<float>();
  test_shapeDistance_conecylinder<double>();
}

template <typename S>
void test_shapeDistance_ellipsoidellipsoid()
{
  Ellipsoid<S> s1(20, 40, 50);
  Ellipsoid<S> s2(10, 10, 10);

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  bool res;
  S dist = -1;
  Vector3<S> closest_p1, closest_p2;

  res = solver1<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), &dist, &closest_p1, &closest_p2);
  EXPECT_TRUE(fabs(dist - 10) < 0.001);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(30.1, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(29.9, 0, 0))), &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver1<S>().shapeDistance(s1, Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), s2, Transform3<S>::Identity(), &dist);
  EXPECT_TRUE(fabs(dist - 10) < 0.001);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, Transform3<S>(Translation3<S>(Vector3<S>(30.1, 0, 0))), s2, Transform3<S>::Identity(), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, Transform3<S>(Translation3<S>(Vector3<S>(29.9, 0, 0))), s2, Transform3<S>::Identity(), &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);


  res = solver1<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 10) < 0.001);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(30.1, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(29.9, 0, 0))), &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver1<S>().shapeDistance(s1, transform * Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), s2, transform, &dist);
  EXPECT_TRUE(fabs(dist - 10) < 0.001);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, transform * Transform3<S>(Translation3<S>(Vector3<S>(30.1, 0, 0))), s2, transform, &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver1<S>().shapeDistance(s1, transform * Transform3<S>(Translation3<S>(Vector3<S>(29.9, 0, 0))), s2, transform, &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeDistance_ellipsoidellipsoid)
{
//  test_shapeDistance_ellipsoidellipsoid<float>();
  test_shapeDistance_ellipsoidellipsoid<double>();
}

// Shape intersection test coverage (built-in GJK)
//
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// |            | box | sphere | ellipsoid | capsule | cone | cylinder | plane | half-space | triangle |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | box        |  O  |   O    |           |         |      |          |       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | sphere     |/////|   O    |           |         |      |          |       |            |     O    |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | ellipsoid  |/////|////////|     O     |         |      |          |       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | capsule    |/////|////////|///////////|         |      |          |       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | cone       |/////|////////|///////////|/////////|  O   |    O     |       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | cylinder   |/////|////////|///////////|/////////|//////|    O     |       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | plane      |/////|////////|///////////|/////////|//////|//////////|       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | half-space |/////|////////|///////////|/////////|//////|//////////|///////|            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | triangle   |/////|////////|///////////|/////////|//////|//////////|///////|////////////|          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+

template <typename S>
void test_shapeIntersectionGJK_spheresphere()
{
  Sphere<S> s1(20);
  Sphere<S> s2(10);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(30, 0, 0)));
  contacts.resize(1);
  contacts[0].normal << 1, 0, 0;
  contacts[0].pos << 20, 0, 0;
  contacts[0].penetration_depth = 0.0;
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(30.01, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(30.01, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(29.9, 0, 0)));
  contacts.resize(1);
  contacts[0].normal << 1, 0, 0;
  contacts[0].pos << 20.0 - 0.1 * 20.0/(20.0 + 10.0), 0, 0;
  contacts[0].penetration_depth = 0.1;
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(29.9, 0, 0)));
  contacts.resize(1);
  contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);
  contacts[0].pos = transform * Vector3<S>(20.0 - 0.1 * 20.0/(20.0 + 10.0), 0, 0);
  contacts[0].penetration_depth = 0.1;
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  contacts[0].normal.setZero();  // If the centers of two sphere are at the same position, the normal is (0, 0, 0)
  contacts[0].pos.setZero();
  contacts[0].penetration_depth = 20.0 + 10.0;
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  contacts[0].normal.setZero();  // If the centers of two sphere are at the same position, the normal is (0, 0, 0)
  contacts[0].pos = transform * Vector3<S>::Zero();
  contacts[0].penetration_depth = 20.0 + 10.0;
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-29.9, 0, 0)));
  contacts.resize(1);
  contacts[0].normal << -1, 0, 0;
  contacts[0].pos << -20.0 + 0.1 * 20.0/(20.0 + 10.0), 0, 0;
  contacts[0].penetration_depth = 0.1;
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-29.9, 0, 0)));
  contacts.resize(1);
  contacts[0].normal = transform.linear() * Vector3<S>(-1, 0, 0);
  contacts[0].pos = transform * Vector3<S>(-20.0 + 0.1 * 20.0/(20.0 + 10.0), 0, 0);
  contacts[0].penetration_depth = 0.1;
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-30.0, 0, 0)));
  contacts.resize(1);
  contacts[0].normal << -1, 0, 0;
  contacts[0].pos << -20, 0, 0;
  contacts[0].penetration_depth = 0.0;
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-30.01, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-30.01, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, false);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersectionGJK_spheresphere)
{
//  test_shapeIntersectionGJK_spheresphere<float>();
  test_shapeIntersectionGJK_spheresphere<double>();
}

template <typename S>
void test_shapeIntersectionGJK_boxbox()
{
  Box<S> s1(20, 40, 50);
  Box<S> s2(10, 10, 10);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  Quaternion<S> q(AngleAxis<S>((S)3.140 / 6, Vector3<S>(0, 0, 1)));

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  // TODO: Need convention for normal when the centers of two objects are at same position. The current result is (1, 0, 0).
  contacts.resize(4);
  contacts[0].normal << 1, 0, 0;
  contacts[1].normal << 1, 0, 0;
  contacts[2].normal << 1, 0, 0;
  contacts[3].normal << 1, 0, 0;
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, true);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at same position. The current result is (1, 0, 0).
  contacts.resize(4);
  contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);
  contacts[1].normal = transform.linear() * Vector3<S>(1, 0, 0);
  contacts[2].normal = transform.linear() * Vector3<S>(1, 0, 0);
  contacts[3].normal = transform.linear() * Vector3<S>(1, 0, 0);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, true);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(15, 0, 0)));
  contacts.resize(4);
  contacts[0].normal = Vector3<S>(1, 0, 0);
  contacts[1].normal = Vector3<S>(1, 0, 0);
  contacts[2].normal = Vector3<S>(1, 0, 0);
  contacts[3].normal = Vector3<S>(1, 0, 0);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, true);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(15.01, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(q);
  contacts.resize(4);
  contacts[0].normal = Vector3<S>(1, 0, 0);
  contacts[1].normal = Vector3<S>(1, 0, 0);
  contacts[2].normal = Vector3<S>(1, 0, 0);
  contacts[3].normal = Vector3<S>(1, 0, 0);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, true);

  tf1 = transform;
  tf2 = transform * Transform3<S>(q);
  contacts.resize(4);
  contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);
  contacts[1].normal = transform.linear() * Vector3<S>(1, 0, 0);
  contacts[2].normal = transform.linear() * Vector3<S>(1, 0, 0);
  contacts[3].normal = transform.linear() * Vector3<S>(1, 0, 0);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, true);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersectionGJK_boxbox)
{
//  test_shapeIntersectionGJK_boxbox<float>();
  test_shapeIntersectionGJK_boxbox<double>();
}

template <typename S>
void test_shapeIntersectionGJK_spherebox()
{
  Sphere<S> s1(20);
  Box<S> s2(5, 5, 5);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  // TODO: Need convention for normal when the centers of two objects are at same position.
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at same position.
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(22.5, 0, 0)));
  contacts.resize(1);
  contacts[0].normal << 1, 0, 0;
  // TODO(SeanCurtis-TRI): This osculating contact is considered a collision
  // only for a relatively "loose" gjk tolerance. So, for this test to pass, we
  // set and restore the default gjk tolerances. We need to determine if this is
  // the correct/desired behavior and, if so, fix the defect that smaller
  // tolerances prevent this from reporting as a contact. NOTE: this is *not*
  // the same tolerance that is passed into the `testShapeIntersection` function
  // -- which isn't actually *used*.
  {
    detail::GJKSolver_indep<S>& solver = solver2<S>();
    const S old_gjk_tolerance = solver.gjk_tolerance;
    const S old_epa_tolerance = solver.epa_tolerance;

    // The historical tolerances for which this test passes.
    solver.gjk_tolerance = 1e-6;
    solver.epa_tolerance = 1e-6;

    testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false,
                          false, true, false,
                          1e-7 // built-in GJK solver requires larger tolerance than libccd
    );
    // TODO(SeanCurtis-TRI): If testShapeIntersection fails an *assert* this
    // code will not get fired off and the static solvers will not get reset.
    solver.gjk_tolerance = old_gjk_tolerance;
    solver.epa_tolerance = old_epa_tolerance;
  }

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(22.51, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(22.4, 0, 0)));
  contacts.resize(1);
  contacts[0].normal << 1, 0, 0;
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, true, false, 1e-2);  // built-in GJK solver requires larger tolerance than libccd

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(22.4, 0, 0)));
  contacts.resize(1);
  // contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);
  // built-in GJK solver returns incorrect normal.
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersectionGJK_spherebox)
{
//  test_shapeIntersectionGJK_spherebox<float>();
  test_shapeIntersectionGJK_spherebox<double>();
}

template <typename S>
void test_shapeIntersectionGJK_spherecapsule()
{
  Sphere<S> s1(20);
  Capsule<S> s2(5, 10);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  // TODO: Need convention for normal when the centers of two objects are at same position.
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at same position.
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(24.9, 0, 0)));
  contacts.resize(1);
  contacts[0].normal << 1, 0, 0;
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, true);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(24.9, 0, 0)));
  contacts.resize(1);
  contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, true);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(25, 0, 0)));
  contacts.resize(1);
  contacts[0].normal << 1, 0, 0;
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, true);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(25.1, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, false);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersectionGJK_spherecapsule)
{
//  test_shapeIntersectionGJK_spherecapsule<float>();
  test_shapeIntersectionGJK_spherecapsule<double>();
}

template <typename S>
void test_shapeIntersectionGJK_cylindercylinder()
{
  Cylinder<S> s1(5, 10);
  Cylinder<S> s2(5, 10);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  // TODO: Need convention for normal when the centers of two objects are at same position.
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at same position.
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(9.9, 0, 0)));
  contacts.resize(1);
  contacts[0].normal << 1, 0, 0;
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, true, false, 3e-1);  // built-in GJK solver requires larger tolerance than libccd

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(9.9, 0, 0)));
  contacts.resize(1);
  // contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);
  // built-in GJK solver returns incorrect normal.

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(10, 0, 0)));
  contacts.resize(1);
  contacts[0].normal << 1, 0, 0;
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, true);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(10.01, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, false);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersectionGJK_cylindercylinder)
{
//  test_shapeIntersectionGJK_cylindercylinder<float>();
  test_shapeIntersectionGJK_cylindercylinder<double>();
}

template <typename S>
void test_shapeIntersectionGJK_conecone()
{
  Cone<S> s1(5, 10);
  Cone<S> s2(5, 10);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  // TODO: Need convention for normal when the centers of two objects are at same position.
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at same position.
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(9.9, 0, 0)));
  contacts.resize(1);
  contacts[0].normal << 1, 0, 0;
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, true, false, 5.7e-1);  // built-in GJK solver requires larger tolerance than libccd

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(9.9, 0, 0)));
  contacts.resize(1);
  // contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);
  // built-in GJK solver returns incorrect normal.

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(10.1, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(10.1, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 9.9)));
  contacts.resize(1);
  contacts[0].normal << 0, 0, 1;
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, true);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 9.9)));
  contacts.resize(1);
  // contacts[0].normal = transform.linear() * Vector3<S>(0, 0, 1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);
  // built-in GJK solver returns incorrect normal.
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersectionGJK_conecone)
{
//  test_shapeIntersectionGJK_conecone<float>();
  test_shapeIntersectionGJK_conecone<double>();
}

template <typename S>
void test_shapeIntersectionGJK_cylindercone()
{
  Cylinder<S> s1(5, 10);
  Cone<S> s2(5, 10);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  std::vector<ContactPoint<S>> contacts;
  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  // TODO: Need convention for normal when the centers of two objects are at same position.
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at same position.
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(9.9, 0, 0)));
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(9.9, 0, 0)));
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(10, 0, 0)));
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(10, 0, 0)));
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 9.9)));
  contacts.resize(1);
  contacts[0].normal << 0, 0, 1;
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, true);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 9.9)));
  contacts.resize(1);
  // contacts[0].normal = transform.linear() * Vector3<S>(1, 0, 0);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);
  // built-in GJK solver returns incorrect normal.

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 10)));
  contacts.resize(1);
  contacts[0].normal << 0, 0, 1;
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, true);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 10.1)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, false);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersectionGJK_cylindercone)
{
//  test_shapeIntersectionGJK_cylindercone<float>();
  test_shapeIntersectionGJK_cylindercone<double>();
}

template <typename S>
void test_shapeIntersectionGJK_ellipsoidellipsoid()
{
  Ellipsoid<S> s1(20, 40, 50);
  Ellipsoid<S> s2(10, 10, 10);

  Transform3<S> tf1;
  Transform3<S> tf2;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);
  Transform3<S> identity;

  std::vector<ContactPoint<S>> contacts;

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(30, 0, 0)));
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(30.01, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(29.99, 0, 0)));
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(29.9, 0, 0)));
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>::Identity();
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);

  tf1 = transform;
  tf2 = transform;
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-29.99, 0, 0)));
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-29.99, 0, 0)));
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);

  tf1 = Transform3<S>::Identity();
  tf2 = Transform3<S>(Translation3<S>(Vector3<S>(-30, 0, 0)));
  contacts.resize(1);
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, true, contacts, false, false, false);

  tf1 = transform;
  tf2 = transform * Transform3<S>(Translation3<S>(Vector3<S>(-30.01, 0, 0)));
  testShapeIntersection(s1, tf1, s2, tf2, GST_INDEP, false);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersectionGJK_ellipsoidellipsoid)
{
//  test_shapeIntersectionGJK_ellipsoidellipsoid<float>();
  test_shapeIntersectionGJK_ellipsoidellipsoid<double>();
}

template <typename S>
void test_shapeIntersectionGJK_spheretriangle()
{
  Sphere<S> s(10);
  Vector3<S> t[3];
  t[0] << 20, 0, 0;
  t[1] << -20, 0, 0;
  t[2] << 0, 20, 0;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  // Vector3<S> point;
  // S depth;
  Vector3<S> normal;
  bool res;

  res = solver2<S>().shapeTriangleIntersect(s, Transform3<S>::Identity(), t[0], t[1], t[2], nullptr, nullptr, nullptr);
  EXPECT_TRUE(res);

  res =  solver2<S>().shapeTriangleIntersect(s, transform, t[0], t[1], t[2], transform, nullptr, nullptr, nullptr);
  EXPECT_TRUE(res);

  t[0] << 30, 0, 0;
  t[1] << 9.9, -20, 0;
  t[2] << 9.9, 20, 0;
  res = solver2<S>().shapeTriangleIntersect(s, Transform3<S>::Identity(), t[0], t[1], t[2], nullptr, nullptr, nullptr);
  EXPECT_TRUE(res);

  res =  solver2<S>().shapeTriangleIntersect(s, transform, t[0], t[1], t[2], transform, nullptr, nullptr, nullptr);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeTriangleIntersect(s, Transform3<S>::Identity(), t[0], t[1], t[2], nullptr, nullptr, &normal);
  EXPECT_TRUE(res);
  EXPECT_TRUE(normal.isApprox(Vector3<S>(1, 0, 0), 1e-9));

  res =  solver2<S>().shapeTriangleIntersect(s, transform, t[0], t[1], t[2], transform, nullptr, nullptr, &normal);
  EXPECT_TRUE(res);
  EXPECT_TRUE(normal.isApprox(transform.linear() * Vector3<S>(1, 0, 0), 1e-9));
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersectionGJK_spheretriangle)
{
//  test_shapeIntersectionGJK_spheretriangle<float>();
  test_shapeIntersectionGJK_spheretriangle<double>();
}

template <typename S>
void test_shapeIntersectionGJK_halfspacetriangle()
{
  Halfspace<S> hs(Vector3<S>(1, 0, 0), 0);
  Vector3<S> t[3];
  t[0] << 20, 0, 0;
  t[1] << -20, 0, 0;
  t[2] << 0, 20, 0;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  // Vector3<S> point;
  // S depth;
  Vector3<S> normal;
  bool res;

  res = solver2<S>().shapeTriangleIntersect(hs, Transform3<S>::Identity(), t[0], t[1], t[2], Transform3<S>::Identity(), nullptr, nullptr, nullptr);
  EXPECT_TRUE(res);

  res =  solver2<S>().shapeTriangleIntersect(hs, transform, t[0], t[1], t[2], transform, nullptr, nullptr, nullptr);
  EXPECT_TRUE(res);


  t[0] << 20, 0, 0;
  t[1] << -0.1, -20, 0;
  t[2] << -0.1, 20, 0;
  res = solver2<S>().shapeTriangleIntersect(hs, Transform3<S>::Identity(), t[0], t[1], t[2], Transform3<S>::Identity(), nullptr, nullptr, nullptr);
  EXPECT_TRUE(res);

  res =  solver2<S>().shapeTriangleIntersect(hs, transform, t[0], t[1], t[2], transform, nullptr, nullptr, nullptr);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeTriangleIntersect(hs, Transform3<S>::Identity(), t[0], t[1], t[2], Transform3<S>::Identity(), nullptr, nullptr, &normal);
  EXPECT_TRUE(res);
  EXPECT_TRUE(normal.isApprox(Vector3<S>(1, 0, 0), 1e-9));

  res =  solver2<S>().shapeTriangleIntersect(hs, transform, t[0], t[1], t[2], transform, nullptr, nullptr, &normal);
  EXPECT_TRUE(res);
  EXPECT_TRUE(normal.isApprox(transform.linear() * Vector3<S>(1, 0, 0), 1e-9));
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersectionGJK_halfspacetriangle)
{
//  test_shapeIntersectionGJK_halfspacetriangle<float>();
  test_shapeIntersectionGJK_halfspacetriangle<double>();
}

template <typename S>
void test_shapeIntersectionGJK_planetriangle()
{
  Plane<S> hs(Vector3<S>(1, 0, 0), 0);
  Vector3<S> t[3];
  t[0] << 20, 0, 0;
  t[1] << -20, 0, 0;
  t[2] << 0, 20, 0;

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  // Vector3<S> point;
  // S depth;
  Vector3<S> normal;
  bool res;

  res = solver1<S>().shapeTriangleIntersect(hs, Transform3<S>::Identity(), t[0], t[1], t[2], Transform3<S>::Identity(), nullptr, nullptr, nullptr);
  EXPECT_TRUE(res);

  res =  solver1<S>().shapeTriangleIntersect(hs, transform, t[0], t[1], t[2], transform, nullptr, nullptr, nullptr);
  EXPECT_TRUE(res);


  t[0] << 20, 0, 0;
  t[1] << -0.1, -20, 0;
  t[2] << -0.1, 20, 0;
  res = solver2<S>().shapeTriangleIntersect(hs, Transform3<S>::Identity(), t[0], t[1], t[2], Transform3<S>::Identity(), nullptr, nullptr, nullptr);
  EXPECT_TRUE(res);

  res =  solver2<S>().shapeTriangleIntersect(hs, transform, t[0], t[1], t[2], transform, nullptr, nullptr, nullptr);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeTriangleIntersect(hs, Transform3<S>::Identity(), t[0], t[1], t[2], Transform3<S>::Identity(), nullptr, nullptr, &normal);
  EXPECT_TRUE(res);
  EXPECT_TRUE(normal.isApprox(Vector3<S>(1, 0, 0), 1e-9));

  res =  solver2<S>().shapeTriangleIntersect(hs, transform, t[0], t[1], t[2], transform, nullptr, nullptr, &normal);
  EXPECT_TRUE(res);
  EXPECT_TRUE(normal.isApprox(transform.linear() * Vector3<S>(1, 0, 0), 1e-9));
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersectionGJK_planetriangle)
{
//  test_shapeIntersectionGJK_planetriangle<float>();
  test_shapeIntersectionGJK_planetriangle<double>();
}

// Shape distance test coverage (built-in GJK)
//
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// |            | box | sphere | ellipsoid | capsule | cone | cylinder | plane | half-space | triangle |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | box        |  O  |   O    |           |         |      |          |       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | sphere     |/////|   O    |           |         |      |          |       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | ellipsoid  |/////|////////|     O     |         |      |          |       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | capsule    |/////|////////|///////////|         |      |          |       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | cone       |/////|////////|///////////|/////////|  O   |          |       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | cylinder   |/////|////////|///////////|/////////|//////|    O     |       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | plane      |/////|////////|///////////|/////////|//////|//////////|       |            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | half-space |/////|////////|///////////|/////////|//////|//////////|///////|            |          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+
// | triangle   |/////|////////|///////////|/////////|//////|//////////|///////|////////////|          |
// +------------+-----+--------+-----------+---------+------+----------+-------+------------+----------+

template <typename S>
void test_shapeDistanceGJK_spheresphere()
{
  Sphere<S> s1(20);
  Sphere<S> s2(10);

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  bool res;
  S dist = -1;

  res = solver2<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 10) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(30.1, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(29.9, 0, 0))), &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver2<S>().shapeDistance(s1, Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), s2, Transform3<S>::Identity(), &dist);
  EXPECT_TRUE(fabs(dist - 10) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, Transform3<S>(Translation3<S>(Vector3<S>(30.1, 0, 0))), s2, Transform3<S>::Identity(), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, Transform3<S>(Translation3<S>(Vector3<S>(29.9, 0, 0))), s2, Transform3<S>::Identity(), &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);


  res = solver2<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 10) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(30.1, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(29.9, 0, 0))), &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver2<S>().shapeDistance(s1, transform * Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), s2, transform, &dist);
  EXPECT_TRUE(fabs(dist - 10) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, transform * Transform3<S>(Translation3<S>(Vector3<S>(30.1, 0, 0))), s2, transform, &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, transform * Transform3<S>(Translation3<S>(Vector3<S>(29.9, 0, 0))), s2, transform, &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeDistanceGJK_spheresphere)
{
//  test_shapeDistanceGJK_spheresphere<float>();
  test_shapeDistanceGJK_spheresphere<double>();
}

template <typename S>
void test_shapeDistanceGJK_boxbox()
{
  Box<S> s1(20, 40, 50);
  Box<S> s2(10, 10, 10);

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  bool res;
  S dist;

  res = solver2<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>::Identity(), &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver2<S>().shapeDistance(s1, transform, s2, transform, &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver2<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(15.1, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(15.1, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(20, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 5) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(20, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 5) < 0.001);
  EXPECT_TRUE(res);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeDistanceGJK_boxbox)
{
//  test_shapeDistanceGJK_boxbox<float>();
  test_shapeDistanceGJK_boxbox<double>();
}

template <typename S>
void test_shapeDistanceGJK_boxsphere()
{
  Sphere<S> s1(20);
  Box<S> s2(5, 5, 5);

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  bool res;
  S dist;

  res = solver2<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>::Identity(), &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver2<S>().shapeDistance(s1, transform, s2, transform, &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver2<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(22.6, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.01);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(22.6, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.01);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 17.5) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 17.5) < 0.001);
  EXPECT_TRUE(res);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeDistanceGJK_boxsphere)
{
//  test_shapeDistanceGJK_boxsphere<float>();
  test_shapeDistanceGJK_boxsphere<double>();
}

template <typename S>
void test_shapeDistanceGJK_cylindercylinder()
{
  Cylinder<S> s1(5, 10);
  Cylinder<S> s2(5, 10);

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  bool res;
  S dist;

  res = solver2<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>::Identity(), &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver2<S>().shapeDistance(s1, transform, s2, transform, &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver2<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(10.1, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(10.1, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 30) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 30) < 0.001);
  EXPECT_TRUE(res);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeDistanceGJK_cylindercylinder)
{
//  test_shapeDistanceGJK_cylindercylinder<float>();
  test_shapeDistanceGJK_cylindercylinder<double>();
}

template <typename S>
void test_shapeDistanceGJK_conecone()
{
  Cone<S> s1(5, 10);
  Cone<S> s2(5, 10);

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  bool res;
  S dist;

  res = solver2<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>::Identity(), &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver2<S>().shapeDistance(s1, transform, s2, transform, &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver2<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(10.1, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(10.1, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 40))), &dist);
  EXPECT_TRUE(fabs(dist - 30) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(0, 0, 40))), &dist);
  EXPECT_TRUE(fabs(dist - 30) < 0.001);
  EXPECT_TRUE(res);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeDistanceGJK_conecone)
{
//  test_shapeDistanceGJK_conecone<float>();
  test_shapeDistanceGJK_conecone<double>();
}

template <typename S>
void test_shapeDistanceGJK_ellipsoidellipsoid()
{
  Ellipsoid<S> s1(20, 40, 50);
  Ellipsoid<S> s2(10, 10, 10);

  Transform3<S> transform = Transform3<S>::Identity();
  test::generateRandomTransform(extents<S>(), transform);

  bool res;
  S dist = -1;

  res = solver2<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 10) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(30.1, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, Transform3<S>::Identity(), s2, Transform3<S>(Translation3<S>(Vector3<S>(29.9, 0, 0))), &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver2<S>().shapeDistance(s1, Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), s2, Transform3<S>::Identity(), &dist);
  EXPECT_TRUE(fabs(dist - 10) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, Transform3<S>(Translation3<S>(Vector3<S>(30.1, 0, 0))), s2, Transform3<S>::Identity(), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, Transform3<S>(Translation3<S>(Vector3<S>(29.9, 0, 0))), s2, Transform3<S>::Identity(), &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver2<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 10) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(30.1, 0, 0))), &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, transform, s2, transform * Transform3<S>(Translation3<S>(Vector3<S>(29.9, 0, 0))), &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);

  res = solver2<S>().shapeDistance(s1, transform * Transform3<S>(Translation3<S>(Vector3<S>(40, 0, 0))), s2, transform, &dist);
  EXPECT_TRUE(fabs(dist - 10) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, transform * Transform3<S>(Translation3<S>(Vector3<S>(30.1, 0, 0))), s2, transform, &dist);
  EXPECT_TRUE(fabs(dist - 0.1) < 0.001);
  EXPECT_TRUE(res);

  res = solver2<S>().shapeDistance(s1, transform * Transform3<S>(Translation3<S>(Vector3<S>(29.9, 0, 0))), s2, transform, &dist);
  EXPECT_TRUE(dist < 0);
  EXPECT_FALSE(res);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeDistanceGJK_ellipsoidellipsoid)
{
//  test_shapeDistanceGJK_ellipsoidellipsoid<float>();
  test_shapeDistanceGJK_ellipsoidellipsoid<double>();
}

template<typename Shape1, typename Shape2>
void testReversibleShapeIntersection(const Shape1& s1, const Shape2& s2, typename Shape2::S distance)
{
  using S = typename Shape2::S;

  Transform3<S> tf1(Translation3<S>(Vector3<S>(-0.5 * distance, 0.0, 0.0)));
  Transform3<S> tf2(Translation3<S>(Vector3<S>(+0.5 * distance, 0.0, 0.0)));

  std::vector<ContactPoint<S>> contactsA;
  std::vector<ContactPoint<S>> contactsB;

  bool resA;
  bool resB;

  const double tol = 1e-6;

  resA = solver1<S>().shapeIntersect(s1, tf1, s2, tf2, &contactsA);
  resB = solver1<S>().shapeIntersect(s2, tf2, s1, tf1, &contactsB);

  // normal should be opposite
  for (size_t i = 0; i < contactsB.size(); ++i)
    contactsB[i].normal = -contactsB[i].normal;

  EXPECT_TRUE(resA);
  EXPECT_TRUE(resB);
  EXPECT_TRUE(inspectContactPointds(s1, tf1, s2, tf2, GST_LIBCCD,
                                   contactsA, contactsB,
                                   true, true, true, false, tol));

  resA = solver2<S>().shapeIntersect(s1, tf1, s2, tf2, &contactsA);
  resB = solver2<S>().shapeIntersect(s2, tf2, s1, tf1, &contactsB);

  // normal should be opposite
  for (size_t i = 0; i < contactsB.size(); ++i)
    contactsB[i].normal = -contactsB[i].normal;

  EXPECT_TRUE(resA);
  EXPECT_TRUE(resB);
  EXPECT_TRUE(inspectContactPointds(s1, tf1, s2, tf2, GST_INDEP,
                                   contactsA, contactsB,
                                   true, true, true, false, tol));
}

template <typename S>
void test_reversibleShapeIntersection_allshapes()
{
  // This test check whether a shape intersection algorithm is called for the
  // reverse case as well. For example, if FCL has sphere-capsule intersection
  // algorithm, then this algorithm should be called for capsule-sphere case.

  // Prepare all kinds of primitive shapes (8) -- box, sphere, ellipsoid, capsule, cone, cylinder, plane, halfspace
  Box<S> box(10, 10, 10);
  Sphere<S> sphere(5);
  Ellipsoid<S> ellipsoid(5, 5, 5);
  Capsule<S> capsule(5, 10);
  Cone<S> cone(5, 10);
  Cylinder<S> cylinder(5, 10);
  Plane<S> plane(Vector3<S>::Zero(), 0.0);
  Halfspace<S> halfspace(Vector3<S>::Zero(), 0.0);

  // Use sufficiently short distance so that all the primitive shapes can intersect
  S distance = 5.0;

  // If new shape intersection algorithm is added for two distinct primitive
  // shapes, uncomment associated lines. For example, box-sphere intersection
  // algorithm is added, then uncomment box-sphere.

//  testReversibleShapeIntersection(box, sphere, distance);
//  testReversibleShapeIntersection(box, ellipsoid, distance);
//  testReversibleShapeIntersection(box, capsule, distance);
//  testReversibleShapeIntersection(box, cone, distance);
//  testReversibleShapeIntersection(box, cylinder, distance);
  testReversibleShapeIntersection(box, plane, distance);
  testReversibleShapeIntersection(box, halfspace, distance);

//  testReversibleShapeIntersection(sphere, ellipsoid, distance);
  testReversibleShapeIntersection(sphere, capsule, distance);
//  testReversibleShapeIntersection(sphere, cone, distance);
//  testReversibleShapeIntersection(sphere, cylinder, distance);
  testReversibleShapeIntersection(sphere, plane, distance);
  testReversibleShapeIntersection(sphere, halfspace, distance);

//  testReversibleShapeIntersection(ellipsoid, capsule, distance);
//  testReversibleShapeIntersection(ellipsoid, cone, distance);
//  testReversibleShapeIntersection(ellipsoid, cylinder, distance);
//  testReversibleShapeIntersection(ellipsoid, plane, distance);
//  testReversibleShapeIntersection(ellipsoid, halfspace, distance);

//  testReversibleShapeIntersection(capsule, cone, distance);
//  testReversibleShapeIntersection(capsule, cylinder, distance);
  testReversibleShapeIntersection(capsule, plane, distance);
  testReversibleShapeIntersection(capsule, halfspace, distance);

//  testReversibleShapeIntersection(cone, cylinder, distance);
  testReversibleShapeIntersection(cone, plane, distance);
  testReversibleShapeIntersection(cone, halfspace, distance);

  testReversibleShapeIntersection(cylinder, plane, distance);
  testReversibleShapeIntersection(cylinder, halfspace, distance);

  testReversibleShapeIntersection(plane, halfspace, distance);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, reversibleShapeIntersection_allshapes)
{
//  test_reversibleShapeIntersection_allshapes<float>();
  test_reversibleShapeIntersection_allshapes<double>();
}

template<typename Shape1, typename Shape2>
void testReversibleShapeDistance(const Shape1& s1, const Shape2& s2, typename Shape2::S distance)
{
  using S = typename Shape2::S;

  Transform3<S> tf1(Translation3<S>(Vector3<S>(-0.5 * distance, 0.0, 0.0)));
  Transform3<S> tf2(Translation3<S>(Vector3<S>(+0.5 * distance, 0.0, 0.0)));

  S distA;
  S distB;
  Vector3<S> p1A;
  Vector3<S> p1B;
  Vector3<S> p2A;
  Vector3<S> p2B;

  bool resA;
  bool resB;

  const double tol = 1e-6;

  resA = solver1<S>().shapeDistance(s1, tf1, s2, tf2, &distA, &p1A, &p2A);
  resB = solver1<S>().shapeDistance(s2, tf2, s1, tf1, &distB, &p1B, &p2B);

  EXPECT_TRUE(resA);
  EXPECT_TRUE(resB);
  EXPECT_NEAR(distA, distB, tol);  // distances should be same
  EXPECT_TRUE(p1A.isApprox(p2B, tol));  // closest points should in reverse order
  EXPECT_TRUE(p2A.isApprox(p1B, tol));

  resA = solver2<S>().shapeDistance(s1, tf1, s2, tf2, &distA, &p1A, &p2A);
  resB = solver2<S>().shapeDistance(s2, tf2, s1, tf1, &distB, &p1B, &p2B);

  EXPECT_TRUE(resA);
  EXPECT_TRUE(resB);
  EXPECT_NEAR(distA, distB, tol);
  EXPECT_TRUE(p1A.isApprox(p2B, tol));
  EXPECT_TRUE(p2A.isApprox(p1B, tol));
}

template <typename S>
void test_reversibleShapeDistance_allshapes()
{
  // This test check whether a shape distance algorithm is called for the
  // reverse case as well. For example, if FCL has sphere-capsule distance
  // algorithm, then this algorithm should be called for capsule-sphere case.

  // Prepare all kinds of primitive shapes (8) -- box, sphere, ellipsoid, capsule, cone, cylinder, plane, halfspace
  Box<S> box(10, 10, 10);
  Sphere<S> sphere(5);
  Ellipsoid<S> ellipsoid(5, 5, 5);
  Capsule<S> capsule(5, 10);
  Cone<S> cone(5, 10);
  Cylinder<S> cylinder(5, 10);
  Plane<S> plane(Vector3<S>::Zero(), 0.0);
  Halfspace<S> halfspace(Vector3<S>::Zero(), 0.0);

  // Use sufficiently long distance so that all the primitive shapes CANNOT intersect
  S distance = 15.0;

  // If new shape distance algorithm is added for two distinct primitive
  // shapes, uncomment associated lines. For example, box-sphere intersection
  // algorithm is added, then uncomment box-sphere.

//  testReversibleShapeDistance(box, sphere, distance);
//  testReversibleShapeDistance(box, ellipsoid, distance);
//  testReversibleShapeDistance(box, capsule, distance);
//  testReversibleShapeDistance(box, cone, distance);
//  testReversibleShapeDistance(box, cylinder, distance);
//  testReversibleShapeDistance(box, plane, distance);
//  testReversibleShapeDistance(box, halfspace, distance);

//  testReversibleShapeDistance(sphere, ellipsoid, distance);
  testReversibleShapeDistance(sphere, capsule, distance);
//  testReversibleShapeDistance(sphere, cone, distance);
//  testReversibleShapeDistance(sphere, cylinder, distance);
//  testReversibleShapeDistance(sphere, plane, distance);
//  testReversibleShapeDistance(sphere, halfspace, distance);

//  testReversibleShapeDistance(ellipsoid, capsule, distance);
//  testReversibleShapeDistance(ellipsoid, cone, distance);
//  testReversibleShapeDistance(ellipsoid, cylinder, distance);
//  testReversibleShapeDistance(ellipsoid, plane, distance);
//  testReversibleShapeDistance(ellipsoid, halfspace, distance);

//  testReversibleShapeDistance(capsule, cone, distance);
//  testReversibleShapeDistance(capsule, cylinder, distance);
//  testReversibleShapeDistance(capsule, plane, distance);
//  testReversibleShapeDistance(capsule, halfspace, distance);

//  testReversibleShapeDistance(cone, cylinder, distance);
//  testReversibleShapeDistance(cone, plane, distance);
//  testReversibleShapeDistance(cone, halfspace, distance);

//  testReversibleShapeDistance(cylinder, plane, distance);
//  testReversibleShapeDistance(cylinder, halfspace, distance);

//  testReversibleShapeDistance(plane, halfspace, distance);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, reversibleShapeDistance_allshapes)
{
//  test_reversibleShapeDistance_allshapes<float>();
  test_reversibleShapeDistance_allshapes<double>();
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

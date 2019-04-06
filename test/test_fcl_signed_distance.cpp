/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include "eigen_matrix_compare.h"
#include "fcl/narrowphase/distance.h"
#include "fcl/narrowphase/detail/traversal/collision_node.h"
#include "fcl/narrowphase/detail/gjk_solver_libccd.h"
#include "test_fcl_utility.h"
#include "fcl_resources/config.h"

using namespace fcl;

bool verbose = false;

//==============================================================================
template <typename S>
void test_distance_spheresphere(GJKSolverType solver_type)
{
  const S radius_1 = 20;
  const S radius_2 = 10;
  Sphere<S> s1{radius_1};
  Sphere<S> s2{radius_2};

  Transform3<S> tf1{Transform3<S>::Identity()};
  Transform3<S> tf2{Transform3<S>::Identity()};

  DistanceRequest<S> request;
  request.enable_signed_distance = true;
  request.enable_nearest_points = true;
  request.gjk_solver_type = solver_type;

  DistanceResult<S> result;

  // Expecting distance to be 10
  result.clear();
  tf2.translation() = Vector3<S>(40, 0, 0);
  distance(&s1, tf1, &s2, tf2, request, result);
  EXPECT_NEAR(result.min_distance, 10, 1e-6);
  EXPECT_TRUE(CompareMatrices(result.nearest_points[0], Vector3<S>(20, 0, 0),
                              request.distance_tolerance));
  EXPECT_TRUE(CompareMatrices(result.nearest_points[1], Vector3<S>(30, 0, 0),
                              request.distance_tolerance));

  // request.distance_tolerance is actually the square of the distance
  // tolerance, namely the difference between distance returned from FCL's EPA
  // implementation and the actual distance, is less than
  // sqrt(request.distance_tolerance).
  const S distance_tolerance = std::sqrt(request.distance_tolerance);

  // Expecting distance to be -5
  result.clear();
  tf2.translation() = Vector3<S>(25, 0, 0);
  distance(&s1, tf1, &s2, tf2, request, result);
  EXPECT_NEAR(result.min_distance, -5, request.distance_tolerance);

  // TODO(JS): Only GST_LIBCCD can compute the pair of nearest points on the
  // surface of the penetrating spheres.
  if (solver_type == GST_LIBCCD)
  {
    EXPECT_TRUE(CompareMatrices(result.nearest_points[0], Vector3<S>(20, 0, 0),
                                distance_tolerance));
    EXPECT_TRUE(CompareMatrices(result.nearest_points[1], Vector3<S>(15, 0, 0),
                                distance_tolerance));
  }

  result.clear();
  tf2.translation() = Vector3<S>(20, 0, 20);
  distance(&s1, tf1, &s2, tf2, request, result);

  S expected_dist =
      (tf1.translation() - tf2.translation()).norm() - radius_1 - radius_2;
  EXPECT_NEAR(result.min_distance, expected_dist, distance_tolerance);
  // TODO(JS): Only GST_LIBCCD can compute the pair of nearest points on the
  // surface of the spheres.
  if (solver_type == GST_LIBCCD)
  {
    Vector3<S> dir = (tf2.translation() - tf1.translation()).normalized();
    Vector3<S> p0_expected = dir * radius_1;
    EXPECT_TRUE(CompareMatrices(result.nearest_points[0], p0_expected,
                                distance_tolerance));
    Vector3<S> p1_expected = tf2.translation() - dir * radius_2;
    EXPECT_TRUE(CompareMatrices(result.nearest_points[1], p1_expected,
                                distance_tolerance));
  }
}

template <typename S>
void test_distance_spherecapsule(GJKSolverType solver_type)
{
  Sphere<S> s1{20};
  Capsule<S> s2{10, 20};

  Transform3<S> tf1{Transform3<S>::Identity()};
  Transform3<S> tf2{Transform3<S>::Identity()};

  DistanceRequest<S> request;
  request.enable_signed_distance = true;
  request.enable_nearest_points = true;
  request.gjk_solver_type = solver_type;

  DistanceResult<S> result;

  // Expecting distance to be 10
  result.clear();
  tf2.translation() = Vector3<S>(40, 0, 0);
  distance(&s1, tf1, &s2, tf2, request, result);
  EXPECT_NEAR(result.min_distance, 10, request.distance_tolerance);
  EXPECT_TRUE(CompareMatrices(result.nearest_points[0], Vector3<S>(20, 0, 0),
                              request.distance_tolerance));
  EXPECT_TRUE(CompareMatrices(result.nearest_points[1], Vector3<S>(30, 0, 0),
                              request.distance_tolerance));

  // Expecting distance to be -5
  result.clear();
  tf2.translation() = Vector3<S>(25, 0, 0);
  distance(&s1, tf1, &s2, tf2, request, result);

  // request.distance_tolerance is actually the square of the distance
  // tolerance, namely the difference between distance returned from FCL's EPA
  // implementation and the actual distance, is less than
  // sqrt(request.distance_tolerance).
  const S distance_tolerance = std::sqrt(request.distance_tolerance);
  ASSERT_NEAR(result.min_distance, -5, distance_tolerance);
  if (solver_type == GST_LIBCCD)
  {
    // NOTE: Currently, only GST_LIBCCD computes the pair of nearest points.
    EXPECT_TRUE(CompareMatrices(result.nearest_points[0], Vector3<S>(20, 0, 0),
                                distance_tolerance * 100));
    EXPECT_TRUE(CompareMatrices(result.nearest_points[1], Vector3<S>(15, 0, 0),
                                distance_tolerance * 100));
  }
}

template <typename S>
void test_distance_cylinder_sphere1() {
  // This is a specific case that has cropped up in the wild that reaches the
  // unexpected `expandPolytope()` error, where the nearest point and the new
  // vertiex botih lie on an edge. It turns out that libccd incorrectly thinks
  // that the edge is the nearest feature, while actually one of the
  // neighbouring face is. This test confirms that the bug is fixed, by the
  // function validateNearestFeatureOfPolytopeBeingEdge.
  using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<S>>;
  const S cylinder_radius = 0.03;
  const S cylinder_length = 0.65;
  CollisionGeometryPtr_t cylinder_geo(
      new fcl::Cylinder<S>(cylinder_radius, cylinder_length));
  Transform3<S> X_WC = Transform3<S>::Identity();
  X_WC.translation() <<0.6, 0,0.325;  
  fcl::CollisionObject<S> cylinder(cylinder_geo, X_WC);

  const S sphere_radius = 0.055;
  CollisionGeometryPtr_t sphere_geo(new fcl::Sphere<S>(sphere_radius));
  Transform3<S> X_WS = Transform3<S>::Identity();
  // clang-format off
  X_WS.matrix() << -0.9954758066974283004, -0.029586630161622884394,  0.090291470227168560414,   0.54197940181714598928,
 -0.085103478665251586222,  -0.14494505659711237611,  -0.98577295990868651909, -0.062117502536077888464,
  0.042253002250460247602,  -0.98899725069574340175,   0.14177137199407846557,   0.60162365763662117857, 
  0, 0, 0, 1;
  // clang-format on
  fcl::CollisionObject<S> sphere(sphere_geo, X_WS);

  fcl::DistanceRequest<S> request;
  request.gjk_solver_type = GJKSolverType::GST_LIBCCD;
  request.distance_tolerance = 9.9999999999999995475e-07;
  request.enable_signed_distance = true;
  fcl::DistanceResult<S> result;

  EXPECT_NO_THROW(fcl::distance(&cylinder, &sphere, request, result));
  // The two objects are penetrating.
  EXPECT_NEAR(-(result.nearest_points[0] - result.nearest_points[1]).norm(),
              result.min_distance, request.distance_tolerance);
  // p_CPc is the position of the witness point Pc on the cylinder, measured
  // and expressed in the cylinder frame C.
  const Vector3<S> p_CPc = X_WC.inverse() * result.nearest_points[0];
  EXPECT_LE(std::abs(p_CPc(2)), cylinder_length / 2);
  EXPECT_LE(p_CPc.template head<2>().norm(), cylinder_radius);
  // p_SPs is the position of the witness point Ps on the sphere, measured and
  // expressed in the sphere frame S.
  const Vector3<S> p_SPs = X_WS.inverse() * result.nearest_points[1];
  EXPECT_LE(p_SPs.norm(), sphere_radius);
}

// This is a *specific* case that has cropped up in the wild that reaches the
// unexpected `expandPolytope()` error. This error was reported in
// https://github.com/flexible-collision-library/fcl/issues/319
template <typename S>
void test_distance_cylinder_box1() {
  using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometryd>;
  const S cylinder_radius = 0.05;
  const S cylinder_length = 0.06;
  CollisionGeometryPtr_t cylinder_geo(
      new fcl::Cylinder<S>(cylinder_radius, cylinder_length));
  Transform3<S> X_WC = Transform3<S>::Identity();
  X_WC.matrix() << -0.99999999997999022838, 6.2572835802045040178e-10,
      6.3260669852976095481e-06, 0.57500009756757608503,
      6.3260669851683709551e-06, -6.3943303429958554955e-10,
      0.99999999997999056145, -0.42711963046787942977,
      6.2573180158128459924e-10, 1, 6.3942912945996747041e-10,
      1.1867093358746836351, 0, 0, 0, 1;
  fcl::CollisionObject<S> cylinder(cylinder_geo, X_WC);

  const Vector3<S> box_size(0.025, 0.35, 1.845);
  CollisionGeometryPtr_t box_geo(
      new fcl::Box<S>(box_size(0), box_size(1), box_size(2)));
  Transform3<S> X_WB = Transform3<S>::Identity();
  X_WB.matrix() << 6.1232339957367660359e-17, -1, 0, 0.80000000000000004441, 1,
      6.1232339957367660359e-17, 0, -0.45750000000000001776, 0, 0, 1,
      1.0224999999999999645, 0, 0, 0, 1;
  fcl::CollisionObject<S> box(box_geo, X_WB);

  fcl::DistanceRequest<S> request;
  request.gjk_solver_type = GJKSolverType::GST_LIBCCD;
  request.distance_tolerance = 9.9999999999999995475e-07;
  request.enable_signed_distance = true;
  fcl::DistanceResult<S> result;

  EXPECT_NO_THROW(fcl::distance(&cylinder, &box, request, result));
  EXPECT_NEAR(result.min_distance,
              (result.nearest_points[0] - result.nearest_points[1]).norm(),
              request.distance_tolerance);
  // p_CPc is the position of the witness point Pc on the cylinder, measured
  // and expressed in the cylinder frame C.
  const Vector3<S> p_CPc = X_WC.inverse() * result.nearest_points[0];
  EXPECT_LE(std::abs(p_CPc(2)), cylinder_length / 2);
  EXPECT_LE(p_CPc.template head<2>().norm(), cylinder_radius);
  // p_BPb is the position of the witness point Pb on the box, measured and
  // expressed in the box frame B.
  const Vector3<S> p_BPb = X_WB.inverse() * result.nearest_points[1];
  EXPECT_TRUE((p_BPb.array().abs() <=
               box_size.array() / 2 + 10 * std::numeric_limits<S>::epsilon())
                  .all());
}

//==============================================================================

GTEST_TEST(FCL_NEGATIVE_DISTANCE, sphere_sphere_ccd)
{
  test_distance_spheresphere<double>(GST_LIBCCD);
}

GTEST_TEST(FCL_NEGATIVE_DISTANCE, sphere_sphere_indep)
{
  test_distance_spheresphere<double>(GST_INDEP);
}

GTEST_TEST(FCL_NEGATIVE_DISTANCE, sphere_capsule_ccd)
{
  test_distance_spherecapsule<double>(GST_LIBCCD);
}

GTEST_TEST(FCL_NEGATIVE_DISTANCE, sphere_capsule_indep)
{
  test_distance_spherecapsule<double>(GST_INDEP);
}

GTEST_TEST(FCL_SIGNED_DISTANCE, cylinder_sphere1_ccd)
{
  test_distance_cylinder_sphere1<double>();
}

GTEST_TEST(FCL_SIGNED_DISTANCE, cylinder_box1_ccd)
{
  test_distance_cylinder_box1<double>();
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

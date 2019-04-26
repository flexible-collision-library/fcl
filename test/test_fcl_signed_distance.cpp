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
using std::abs;

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
  // vertex both lie on an edge. It turns out that libccd incorrectly thinks
  // that the edge is the nearest feature, while actually one of the
  // neighbouring faces is. This test confirms that the bug is fixed, by the
  // function validateNearestFeatureOfPolytopeBeingEdge().
  // This case was reported in
  // https://github.com/flexible-collision-library/fcl/issues/391.
  using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<S>>;
  const S cylinder_radius = 0.03;
  const S cylinder_length = 0.65;
  CollisionGeometryPtr_t cylinder_geo(
      new fcl::Cylinder<S>(cylinder_radius, cylinder_length));
  Transform3<S> X_WC = Transform3<S>::Identity();
  X_WC.translation() << 0.6, 0, 0.325;
  fcl::CollisionObject<S> cylinder(cylinder_geo, X_WC);

  const S sphere_radius = 0.055;
  CollisionGeometryPtr_t sphere_geo(new fcl::Sphere<S>(sphere_radius));
  Transform3<S> X_WS = Transform3<S>::Identity();
  // clang-format off
  X_WS.matrix() << -0.9954758066,  -0.0295866301,  0.0902914702,  0.5419794018,
                   -0.0851034786,  -0.1449450565, -0.9857729599, -0.0621175025,
                    0.0422530022,  -0.9889972506,  0.1417713719,  0.6016236576,
                               0,              0,             0,             1;
  // clang-format on
  fcl::CollisionObject<S> sphere(sphere_geo, X_WS);

  fcl::DistanceRequest<S> request;
  request.gjk_solver_type = GJKSolverType::GST_LIBCCD;
  request.distance_tolerance = 1e-6;
  request.enable_signed_distance = true;
  fcl::DistanceResult<S> result;

  ASSERT_NO_THROW(fcl::distance(&cylinder, &sphere, request, result));
  // The two objects are penetrating.
  EXPECT_NEAR(-(result.nearest_points[0] - result.nearest_points[1]).norm(),
              result.min_distance, request.distance_tolerance);
  // p_CPc is the position of the witness point Pc on the cylinder, measured
  // and expressed in the cylinder frame C.
  const Vector3<S> p_CPc = X_WC.inverse() * result.nearest_points[0];
  EXPECT_LE(abs(p_CPc(2)), cylinder_length / 2);
  EXPECT_LE(p_CPc.template head<2>().norm(), cylinder_radius);
  // p_SPs is the position of the witness point Ps on the sphere, measured and
  // expressed in the sphere frame S.
  const Vector3<S> p_SPs = X_WS.inverse() * result.nearest_points[1];
  EXPECT_LE(p_SPs.norm(), sphere_radius);
}

template <typename S>
void test_distance_cylinder_box_helper(S cylinder_radius, S cylinder_length,
                                       const Transform3<S>& X_WC,
                                       const Vector3<S>& box_size,
                                       const Transform3<S>& X_WB) {
  using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometryd>;
  CollisionGeometryPtr_t cylinder_geo(
      new fcl::Cylinder<S>(cylinder_radius, cylinder_length));
  fcl::CollisionObject<S> cylinder(cylinder_geo, X_WC);

  CollisionGeometryPtr_t box_geo(
      new fcl::Box<S>(box_size(0), box_size(1), box_size(2)));
  fcl::CollisionObject<S> box(box_geo, X_WB);

  fcl::DistanceRequest<S> request;
  request.gjk_solver_type = GJKSolverType::GST_LIBCCD;
  request.distance_tolerance = 1e-6;
  request.enable_signed_distance = true;
  fcl::DistanceResult<S> result;

  ASSERT_NO_THROW(fcl::distance(&cylinder, &box, request, result));
  EXPECT_NEAR(abs(result.min_distance),
              (result.nearest_points[0] - result.nearest_points[1]).norm(),
              request.distance_tolerance);
  // p_CPc is the position of the witness point Pc on the cylinder, measured
  // and expressed in the cylinder frame C.
  const Vector3<S> p_CPc = X_WC.inverse() * result.nearest_points[0];
  EXPECT_LE(abs(p_CPc(2)), cylinder_length / 2);
  EXPECT_LE(p_CPc.template head<2>().norm(), cylinder_radius);
  // p_BPb is the position of the witness point Pb on the box, measured and
  // expressed in the box frame B.
  const Vector3<S> p_BPb = X_WB.inverse() * result.nearest_points[1];
  const S tol = 10 * std::numeric_limits<S>::epsilon();
  EXPECT_TRUE((p_BPb.array().abs() <= box_size.array() / 2 + tol).all());
}

template <typename S>
void test_distance_cylinder_box() {
  // This is a *specific* case that has cropped up in the wild that reaches the
  // unexpected `expandPolytope()` error. This error was reported in
  // https://github.com/flexible-collision-library/fcl/issues/319
  // This test would fail in Debug mode without the function
  // validateNearestFeatureOfPolytopeBeingEdge in PR #388.
  S cylinder_radius = 0.05;
  S cylinder_length = 0.06;

  Transform3<S> X_WC = Transform3<S>::Identity();
  X_WC.matrix() << -0.99999999997999022838, 6.2572835802045040178e-10,
      6.3260669852976095481e-06, 0.57500009756757608503,
      6.3260669851683709551e-06, -6.3943303429958554955e-10,
      0.99999999997999056145, -0.42711963046787942977,
      6.2573180158128459924e-10, 1, 6.3942912945996747041e-10,
      1.1867093358746836351, 0, 0, 0, 1;
  Vector3<S> box_size(0.025, 0.35, 1.845);
  Transform3<S> X_WB = Transform3<S>::Identity();
  // clang-format off
  X_WB.matrix() << 0, -1, 0, 0.8,
                   1, 0, 0, -0.4575,
                   0, 0, 1, 1.0225,
                   0, 0, 0, 1;
  // clang-format on
  test_distance_cylinder_box_helper(cylinder_radius, cylinder_length, X_WC,
                                    box_size, X_WB);
  // This is a specific case reported in
  // https://github.com/flexible-collision-library/fcl/issues/390#issuecomment-481634606
  X_WC.matrix() << -0.97313010759279283679, -0.12202804064972551379,
      0.19526123781136842106, 0.87472781461138560122, 0.20950801135757171623,
      -0.11745920593569325607, 0.97072639199619581429, -0.4038687881347159947,
      -0.095520609678929752073, 0.98555187191549953329, 0.13986894183635001365,
      1.5871328698116491385, 0, 0, 0, 1;
  // clang-format off
  X_WB.matrix() << 0, -1, 0, 0.8,
                   1, 0, 0, -0.4575,
                   0, 0, 1, 1.0225,
                   0, 0, 0, 1;
  // clang-format on
  test_distance_cylinder_box_helper(cylinder_radius, cylinder_length, X_WC,
                                    box_size, X_WB);
}

template <typename S>
void test_distance_box_box_helper(const Vector3<S>& box1_size,
                                  const Transform3<S>& X_WB1,
                                  const Vector3<S>& box2_size,
                                  const Transform3<S>& X_WB2) {
  using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometryd>;
  CollisionGeometryPtr_t box1_geo(
      new fcl::Box<S>(box1_size(0), box1_size(1), box1_size(2)));
  fcl::CollisionObject<S> box1(box1_geo, X_WB1);

  CollisionGeometryPtr_t box2_geo(
      new fcl::Box<S>(box2_size(0), box2_size(1), box2_size(2)));
  fcl::CollisionObject<S> box2(box2_geo, X_WB2);

  fcl::DistanceRequest<S> request;
  request.gjk_solver_type = GJKSolverType::GST_LIBCCD;
  request.distance_tolerance = 1e-6;
  request.enable_signed_distance = true;
  fcl::DistanceResult<S> result;

  ASSERT_NO_THROW(fcl::distance(&box1, &box2, request, result));
  EXPECT_NEAR(abs(result.min_distance),
              (result.nearest_points[0] - result.nearest_points[1]).norm(),
              request.distance_tolerance);
  // p_B1P1 is the position of the witness point P1 on box 1, measured
  // and expressed in the box 1 frame B1.
  const Vector3<S> p_B1P1 = X_WB1.inverse() * result.nearest_points[0];
  const double tol = 10 * std::numeric_limits<S>::epsilon();
  EXPECT_TRUE((p_B1P1.array().abs() <= (box1_size / 2).array() + tol).all());
  // p_B2P2 is the position of the witness point P2 on box 2, measured
  // and expressed in the box 2 frame B2.
  const Vector3<S> p_B2P2 = X_WB2.inverse() * result.nearest_points[1];
  EXPECT_TRUE((p_B2P2.array().abs() <= (box2_size / 2).array() + tol).all());
}

// This is a *specific* case that has cropped up in the wild that reaches the
// unexpected `validateNearestFeatureOfPolytopeBeingEdge` error. This error was
// reported in https://github.com/flexible-collision-library/fcl/issues/388
template <typename S>
void test_distance_box_box1() {
  const Vector3<S> box1_size(0.03, 0.12, 0.1);
  Transform3<S> X_WB1 = Transform3<S>::Identity();
  X_WB1.matrix() << -3.0627937852578681533e-08, -0.99999999999999888978,
      -2.8893865161583314238e-08, 0.63499979627350811029, 0.9999999999999980016,
      -3.0627939739957803544e-08, 6.4729926918527511769e-08,
      -0.48500002215636439651, -6.4729927722963847085e-08,
      -2.8893863029448751323e-08, 0.99999999999999711342, 1.0778146458339641356,
      0, 0, 0, 1;

  const Vector3<S> box2_size(0.025, 0.35, 1.845);
  Transform3<S> X_WB2 = Transform3<S>::Identity();
  // clang-format off
  X_WB2.matrix() << 0, -1, 0, 0.8,
                    1, 0, 0, -0.4575,
                    0, 0, 1, 1.0225, 
                    0, 0, 0, 1;
  // clang-format on
  test_distance_box_box_helper(box1_size, X_WB1, box2_size, X_WB2);
}

// This is a *specific* case that has cropped up in the wild that reaches the
// unexpected `triangle_size_is_zero` error. This error was
// reported in https://github.com/flexible-collision-library/fcl/issues/395
template <typename S>
void test_distance_box_box2() {
  const Vector3<S> box1_size(0.46, 0.48, 0.01);
  Transform3<S> X_WB1 = Transform3<S>::Identity();
  X_WB1.matrix() <<  1,0,0, -0.72099999999999997424,
                      0,1,0, -0.77200000000000001954,
                      0,0,1,  0.81000000000000005329,
                      0,0,0,1;

  const Vector3<S> box2_size(0.049521, 0.146, 0.0725);
  Transform3<S> X_WB2 = Transform3<S>::Identity();
  // clang-format off
  X_WB2.matrix() << 0.10758262492983036718,  -0.6624881850015212903, -0.74130653817877356637, -0.42677133002999478872,
 0.22682184885125472595,   -0.709614040775253474,   0.6670830248314786326, -0.76596851247746788882,
-0.96797615037608542021, -0.23991106241273435495,  0.07392465377049164954,  0.80746731400091054098,
                      0,                       0,                       0,                       1;
  // clang-format on
  test_distance_box_box_helper(box1_size, X_WB1, box2_size, X_WB2);
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

GTEST_TEST(FCL_SIGNED_DISTANCE, cylinder_box_ccd) {
  test_distance_cylinder_box<double>();
}

GTEST_TEST(FCL_SIGNED_DISTANCE, box_box1_ccd) {
  test_distance_box_box1<double>();
  test_distance_box_box2<double>();
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

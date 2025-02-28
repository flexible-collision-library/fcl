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
  const S tol = 10 * std::numeric_limits<S>::epsilon();
  EXPECT_LE(abs(p_CPc(2)), cylinder_length / 2 + tol);
  EXPECT_LE(p_CPc.template head<2>().norm(), cylinder_radius);
  // p_BPb is the position of the witness point Pb on the box, measured and
  // expressed in the box frame B.
  const Vector3<S> p_BPb = X_WB.inverse() * result.nearest_points[1];
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
                                  const Transform3<S>& X_WB2,
                                  const S* expected_distance = nullptr) {
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
  constexpr double tol = 10 * constants<S>::eps();
  const double tol_1 = tol * std::max(S(1), (box1_size / 2).maxCoeff());
  EXPECT_TRUE(
      (p_B1P1.array().abs() <= (box1_size / 2).array() + tol_1).all())
      << "\n  p_B1P1: " << p_B1P1.transpose()
      << "\n  box1_size / 2: " << (box1_size / 2).transpose()
      << "\n  tol: " << tol_1;
  // p_B2P2 is the position of the witness point P2 on box 2, measured
  // and expressed in the box 2 frame B2.
  const double tol_2 = tol * std::max(S(1), (box2_size / 2).maxCoeff());
  const Vector3<S> p_B2P2 = X_WB2.inverse() * result.nearest_points[1];
  EXPECT_TRUE(
      (p_B2P2.array().abs() <= (box2_size / 2).array() + tol_2).all())
      << "\n  p_B2P2: " << p_B2P2.transpose()
      << "\n  box2_size / 2: " << (box2_size / 2).transpose()
      << "\n  tol: " << tol_2;

  // An expected distance has been provided; let's test that the value is as
  // expected.
  if (expected_distance) {
    EXPECT_NEAR(result.min_distance, *expected_distance,
                constants<S>::eps_12());
  }
}

// This is a *specific* case that has cropped up in the wild that reaches the
// unexpected `validateNearestFeatureOfPolytopeBeingEdge` error. This error was
// reported in https://github.com/flexible-collision-library/fcl/issues/388
template <typename S>
void test_distance_box_box_regression1() {
  SCOPED_TRACE("test_distance_box_box_regression1");
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
void test_distance_box_box_regression2() {
  SCOPED_TRACE("test_distance_box_box_regression2");
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

// This is a *specific* case that has cropped up in the wild that reaches the
// unexpected `query point colinear with the edge` error. This error was
// reported in https://github.com/flexible-collision-library/fcl/issues/415
template <typename S>
void test_distance_box_box_regression3() {
  SCOPED_TRACE("test_distance_box_box_regression3");
  const Vector3<S> box1_size(0.49, 0.05, 0.21);
  Transform3<S> X_WB1 = Transform3<S>::Identity();
  // clang-format off
  X_WB1.matrix() << 4.8966386501092529215e-12, -1,0,-0.43999999999999994671,
                       1, 4.8966386501092529215e-12,0,-0.61499999999858001587,
                       0,0,1,0.35499999999999998224,
                       0,0,0,1;
  // clang-format on
  const Vector3<S> box2_size(0.035, 0.12, 0.03);
  Transform3<S> X_WB2 = Transform3<S>::Identity();
  // clang-format off
  X_WB2.matrix() << 0.83512153565236335595,    -0.55006546945762568868, -9.4542360608233572896e-16,    -0.40653441507331000704,
   0.55006546945762568868,     0.83512153565236313391,  1.1787444236552387666e-15,    -0.69166166923735727945,
1.2902271444330665572e-16, -1.4878153530113264589e-15,                          1,     0.43057093858718892276,
                        0,                          0,                          0,                          1;
  // clang-format on
  test_distance_box_box_helper(box1_size, X_WB1, box2_size, X_WB2);
}

// This is a *specific* case that has cropped up in the wild. This error was
// reported in https://github.com/flexible-collision-library/fcl/issues/398
template <typename S>
void test_distance_box_box_regression4() {
  SCOPED_TRACE("test_distance_box_box_regression4");
  const Vector3<S> box1_size(0.614, 3, 0.37);
  Transform3<S> X_WB1 = Transform3<S>::Identity();
  X_WB1.translation() << -0.675, 0, 0.9115;
  const Vector3<S> box2_size(0.494, 0.552, 0.01);
  Transform3<S> X_WB2 = Transform3<S>::Identity();
  X_WB2.translation() << -0.692, 0, 0.935;
  test_distance_box_box_helper(box1_size, X_WB1, box2_size, X_WB2);
}

// This is a *specific* case that has cropped up in the wild. This error was
// reported in https://github.com/flexible-collision-library/fcl/issues/428
template <typename S>
void test_distance_box_box_regression5() {
  SCOPED_TRACE("test_distance_box_box_regression5");
  const Vector3<S> box1_size(0.2, 0.33, 0.1);
  Transform3<S> X_WB1 = Transform3<S>::Identity();
  X_WB1.translation() << -0.071000000000000035305, -0.77200000000000001954, 0.79999999999999993339;
  const Vector3<S> box2_size(0.452, 0.27, 0.6);
  Transform3<S> X_WB2 = Transform3<S>::Identity();
  X_WB2.translation() << 0.12099999999999999645, -0.78769605692727695523, 0.53422044196125151316;
  test_distance_box_box_helper(box1_size, X_WB1, box2_size, X_WB2);
}

template <typename S>
void test_distance_box_box_regression6() {
  SCOPED_TRACE("test_distance_box_box_regression6");
  const Vector3<S> box1_size(0.31650000000000000355, 0.22759999999999999676, 0.1768000000000000127);
  Transform3<S> X_WB1 = Transform3<S>::Identity();
  // clang-format off
  X_WB1.matrix() << 0.44540578475530234748,    0.89532881496493399442,   -8.8937407685638678971e-09, 1.2652949075960071568,
                   -0.89532881496493377238,    0.44540578475530190339,   -2.8948680226084145336e-08, 1.4551012423210101243,
                   -2.1957263975186326105e-08, 2.0856732016652919226e-08, 0.99999999999999955591,    0.49480006232932938204,
                    0,                         0,                         0,                         1;
  // clang-format on
  const Vector3<S> box2_size(0.49430000000000001714, 0.35460000000000002629, 0.075200000000000002953);
  Transform3<S> X_WB2 = Transform3<S>::Identity();
  // clang-format off
  X_WB2.matrix() << 0.44171122913485860728,    0.8971572827861190591,    -1.622764514865468214e-09,  1.1304016226141906376,
                   -0.8971572827861190591,     0.44171122913485860728,   -5.1621053952306079594e-09, 1.8410802645284281009,
                   -3.9144271413829990148e-09, 3.7360349218094348098e-09, 1,                         0.44400006232932492933,
                    0,                         0,                         0,                         1;
  // clang-format on;
  const double expected_distance{0};
  test_distance_box_box_helper(box1_size, X_WB1, box2_size, X_WB2, &expected_distance);
}


template <typename S>
void test_distance_box_box_regression7() {
  // This is reported in https://github.com/RobotLocomotion/drake/issues/21673#issuecomment-2373031038
  SCOPED_TRACE("test_distance_box_box_regression7");
  const Vector3<S> box1_size(0.050000000000000002776, 0.55000000000000004441, 0.2999999999999999889);
  Transform3<S> X_WB1 = Transform3<S>::Identity();
  // clang-format off
  X_WB1.matrix() << 0.00079632671073326442932, -0.99999968293183538748, 0, 0.75000000000000055511,
  0.99999968293183538748, 0.00079632671073326442932, 0, -2.4083553715553102684e-15,
  0, 0, 1, 0.14999999999999999445,
  0, 0, 0, 1;
  // clang-format on
  const Vector3<S> box2_size(0.25, 0.2000000000000000111,
                             0.14999999999999999445);
  Transform3<S> X_WB2 = Transform3<S>::Identity();
  // clang-format off
  X_WB2.matrix() << 6.1232339957367660359e-17, 0, 1, 0.75,
  -1, 6.1232339957367660359e-17, 6.1232339957367660359e-17, 0,
  -6.1232339957367660359e-17, -1, 3.7493994566546440196e-33, 0.14999999999999999445,
  0, 0, 0, 1;
  // clang-format on;
  test_distance_box_box_helper(box1_size, X_WB1, box2_size, X_WB2);
}

// Issue #493 outlines a number of scenarios that caused signed distance
// failure. They consisted of two identical, stacked boxes. The boxes are
// slightly tilted. The boxes were essentially touching but were separated by
// infinitesimally small distances. The issue outlines three different examples.
// Rather than reproducing each of them verbatim, this test attempts to
// generalize those cases by testing the stacked scenario across various box
// sizes and separation amounts (ranging from slightly penetrating to slightly
// separated). These should essentially cover the variations described in the
// issue.
template <typename S>
void test_distance_box_box_regression_tilted_kissing_contact() {
  SCOPED_TRACE("test_distance_box_box_regression_tilted_kissing_contact");
  // The boxes are posed relative to each other in a common frame F (such that
  // it is easy to reason about their separation). The stack is rotated around
  // box A's origin and translated into the world frame.
  Matrix3<S> R_WF;
  R_WF <<
       0.94096063217417758029, 0.29296840037289501035, 0.16959541586174811667,
      -0.23569836841299879326, 0.92661523595848427348, -0.29296840037289506586,
      -0.2429801799032638987, 0.23569836841299884878, 0.94096063217417758029;

  for (const S dim : {S(0.01), S(0.25), S(0.5), S(10), S(1000)}) {
    const Vector3<S> box_size(dim, dim, dim);

    const Vector3<S> p_WA(0, 0, 5 * dim);
    Transform3<S> X_WA;
    X_WA.linear() = R_WF;
    X_WA.translation() = p_WA;
    Transform3<S> X_WB;
    X_WB.linear() = R_WF;

    // Both boxes always have the same orientation and the *stack* is always
    // located at p_WA. Only the translational component of X_WB changes with
    // varying separation distance.

    // By design, the distances are all near epsilon. We'll scale them up for
    // larger boxes to make sure the distance doesn't simply disappear in
    // the rounding noise.
    for (const S distance : {S(-1e-15), S(-2.5e-16), S(-1e-16), S(0), S(1e-16),
                             S(2.5e-16), S(1e-15)}) {
      const S scaled_distance = distance * std::max(S(1), dim);
      const Vector3<S> p_AB_F = Vector3<S>(0, dim + scaled_distance, 0);

      X_WB.translation() = p_WA + R_WF * p_AB_F;
      SCOPED_TRACE("dim: " + std::to_string(dim) +
                   ", distance: " + std::to_string(distance));
      test_distance_box_box_helper(box_size, X_WA, box_size, X_WB,
                                   &scaled_distance);
    }
  }
}

// This is a *specific* case that has cropped up in the wild that reaches the
// unexpected `validateNearestFeatureOfPolytopeBeingEdge` error. This error was
// reported in https://github.com/flexible-collision-library/fcl/issues/408
template <typename S>
void test_distance_sphere_box_regression1() {
  SCOPED_TRACE("test_distance_sphere_box_regression1");
  using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<S>>;
  const S sphere_radius = 0.06;
  CollisionGeometryPtr_t sphere_geo(new fcl::Sphere<S>(sphere_radius));
  Transform3<S> X_WS = Transform3<S>::Identity();
  // clang-format off
  X_WS.matrix() << -0.99999999999999955591,     -4.4637642593504144998e-09,  0,                      1.7855056639081962376e-10,
                    4.4637642593504144998e-09,  -0.99999999999999955591,     0,                      0.039999999999999993894,
                    0,                           0,                          1.0000000000000008882,  0.33000000000000012657,
                    0,                           0,                          0,                      1;
  // clang-format on
  fcl::CollisionObject<S> sphere(sphere_geo, X_WS);

  CollisionGeometryPtr_t box_geo(new fcl::Box<S>(0.1, 0.1, 0.1));
  Transform3<S> X_WB = Transform3<S>::Identity();
  // clang-format off
  X_WB.matrix() << 1, 0, 0, 0.05,
      0, 1, 0, 0.15,
      0, 0, 1, 0.35,
      0, 0, 0, 1;
  // clang-format on
  fcl::CollisionObject<S> box(box_geo, X_WB);
  fcl::DistanceRequest<S> request;
  request.gjk_solver_type = GJKSolverType::GST_LIBCCD;
  request.distance_tolerance = 1e-6;
  request.enable_signed_distance = true;
  fcl::DistanceResult<S> result;

  ASSERT_NO_THROW(fcl::distance(&sphere, &box, request, result));
  const S expected_distance = 0.06 - sphere_radius;
  EXPECT_NEAR(result.min_distance, expected_distance,
              request.distance_tolerance);
}

template <typename S>
void test_distance_convex_to_convex1() {
  // This is a real world failure reported in
  // https://github.com/RobotLocomotion/drake/issues/21673#issuecomment-2679026288.
  // The two convex shapes are almost in penetration.
  using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometryd>;
  CollisionGeometryPtr_t convex_geo1(new fcl::Convex<S>(
      std::make_shared<const std::vector<Vector3<S>>>(
          std::initializer_list<Vector3<S>>{
              Vector3<S>(0.5, 0.70710700000000004106, -0.70710700000000004106),
              Vector3<S>(0.5, -0, -1),
              Vector3<S>(0.5, 1, 0),
              Vector3<S>(0.5, -1, 0),
              Vector3<S>(0.5, -0.70710700000000004106, -0.70710700000000004106),
              Vector3<S>(-0.5, 0, 1),
              Vector3<S>(-0.5, -0.70710700000000004106, 0.70710700000000004106),
              Vector3<S>(-0.5, 0.70710700000000004106, 0.70710700000000004106),
              Vector3<S>(0.5, 0, 1),
              Vector3<S>(-0.5, -1, 0),
              Vector3<S>(-0.5, 0.70710700000000004106, -0.70710700000000004106),
              Vector3<S>(-0.5, -0, -1),
              Vector3<S>(-0.5, -0.70710700000000004106,
                         -0.70710700000000004106),
              Vector3<S>(-0.5, 1, 0),
              Vector3<S>(0.5, 0.70710700000000004106, 0.70710700000000004106),
              Vector3<S>(0.5, -0.70710700000000004106, 0.70710700000000004106),
          }),
      10,
      std::make_shared<std::vector<int>>(std::initializer_list<int>{
          4,  4,  12, 11, 1,  4,  13, 7,  14, 2,  4,  2, 0,  10, 13,
          4,  11, 10, 0,  1,  8,  2,  14, 8,  15, 3,  4, 1,  0,  4,
          3,  9,  12, 4,  4,  15, 6,  9,  3,  8,  12, 9, 6,  5,  7,
          13, 10, 11, 4,  14, 7,  5,  8,  4,  8,  5,  6, 15,
      }),
      false));
  Transform3<S> X_WG1 = Transform3<S>::Identity();
  // clang-format off
  X_WG1.matrix() << -2.0399676677885372849e-09, 0.77129744817973977522, -0.63647485922966484662, 11.477445682202462862,
  2.4720879917217940548e-09, 0.63647485922966484662, 0.77129744817973977522, 9.7785056920756936449,
  1, 0, -3.2051032938795742666e-09, 0,
  0, 0, 0, 1;
  // clang-format on
  fcl::CollisionObject<S> convex1(convex_geo1, X_WG1);

  CollisionGeometryPtr_t convex_geo2(new fcl::Convex<S>(
      std::make_shared<std::vector<Vector3<S>>>(
          std::initializer_list<Vector3<S>>{
              Vector3<S>(10.294759000000000881, 8.9321570000000001244, -0.5),
              Vector3<S>(11.618370999999999782, 9.4565629999999991639, -0.5),
              Vector3<S>(11.452372999999999692, 8.2276720000000000965, -0.5),
              Vector3<S>(10.294759000000000881, 8.9321570000000001244, 0.5),
              Vector3<S>(11.452372999999999692, 8.2276720000000000965, 0.5),
              Vector3<S>(11.618370999999999782, 9.4565629999999991639, 0.5),
          }),
      5,
      std::make_shared<std::vector<int>>(std::initializer_list<int>{
          3, 3, 4, 5, 4, 4, 2, 1, 5, 3, 2, 0, 1, 4, 3, 0, 2, 4, 4, 1, 0, 3, 5,
      }),
      false));
  const Transform3<S> X_WS2 = Transform3<S>::Identity();
  fcl::CollisionObject<S> convex2(convex_geo2, X_WS2);
  fcl::DistanceRequest<S> request;
  request.gjk_solver_type = GJKSolverType::GST_LIBCCD;
  request.distance_tolerance = 1e-6;
  request.enable_signed_distance = true;
  fcl::DistanceResult<S> result;
  ASSERT_NO_THROW(fcl::distance(&convex1, &convex2, request, result));
  EXPECT_NEAR(abs(result.min_distance),
              (result.nearest_points[0] - result.nearest_points[1]).norm(),
              request.distance_tolerance);
}

template <typename S>
void test_distance_convex_to_convex2() {
  // This is a real world failure reported in
  // https://github.com/RobotLocomotion/drake/issues/21673#issuecomment-2688753008
  using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometryd>;
  CollisionGeometryPtr_t convex_geo1(new fcl::Convex<S>(
      std::make_shared<const std::vector<Vector3<S>>>(
          std::initializer_list<Vector3<S>>{
              Vector3<S>(0.5, 0.5, -0.86602500000000004476),
              Vector3<S>(0.5, -0.5, -0.86602500000000004476),
              Vector3<S>(0.5, 1, 0),
              Vector3<S>(-0.5, -0.5, 0.86602500000000004476),
              Vector3<S>(-0.5, 0.5, 0.86602500000000004476),
              Vector3<S>(-0.5, -1, 0),
              Vector3<S>(0.5, -0.5, 0.86602500000000004476),
              Vector3<S>(-0.5, 0.5, -0.86602500000000004476),
              Vector3<S>(-0.5, -0.5, -0.86602500000000004476),
              Vector3<S>(-0.5, 1, 0),
              Vector3<S>(0.5, -1, 0),
              Vector3<S>(0.5, 0.5, 0.86602500000000004476),
          }),
      8,
      std::make_shared<std::vector<int>>(std::initializer_list<int>{
          4, 10, 5, 8, 1, 6, 2, 11, 6,  10, 1, 0,  4, 0,  7,
          9, 2,  4, 1, 8, 7, 0, 4,  2,  9,  4, 11, 6, 8,  5,
          3, 4,  9, 7, 4, 6, 3, 5,  10, 4,  4, 3,  6, 11,
      }),
      false));
  Transform3<S> X_WG1 = Transform3<S>::Identity();
  // clang-format off
  X_WG1.matrix() << 
    -3.0140487242790098936e-09, 0.34009658459984087875, -0.94039051098122172778, 4.2864836941313040342,
    1.090044683538143324e-09, 0.94039051098122172778, 0.34009658459984087875, 7.3869576872253679412,
    1, 0, -3.2051032938795742666e-09, 0,
    0, 0, 0, 1;
  // clang-format on
  fcl::CollisionObject<S> convex1(convex_geo1, X_WG1);

  CollisionGeometryPtr_t convex_geo2(new fcl::Convex<S>(
      std::make_shared<std::vector<Vector3<S>>>(
          std::initializer_list<Vector3<S>>{
              Vector3<S>(4.1494020000000002568, 8.4937090000000008416, -0.5),
              Vector3<S>(5.1884509999999997021, 7.476263000000000325, -0.5),
              Vector3<S>(3.9316469999999998919, 7.3745029999999998083, -0.5),
              Vector3<S>(4.1494020000000002568, 8.4937090000000008416, 0.5),
              Vector3<S>(3.9316469999999998919, 7.3745029999999998083, 0.5),
              Vector3<S>(5.1884509999999997021, 7.476263000000000325, 0.5),
          }),
      5,
      std::make_shared<std::vector<int>>(std::initializer_list<int>{
          3, 5, 3, 4, 4, 2, 1, 5, 4, 3, 2, 0, 1, 4, 3, 0, 2, 4, 4, 1, 0, 3, 5,
      }),
      false));
  const Transform3<S> X_WS2 = Transform3<S>::Identity();
  fcl::CollisionObject<S> convex2(convex_geo2, X_WS2);
  fcl::DistanceRequest<S> request;
  request.gjk_solver_type = GJKSolverType::GST_LIBCCD;
  request.distance_tolerance = 1e-6;
  request.enable_signed_distance = true;
  fcl::DistanceResult<S> result;
  ASSERT_NO_THROW(fcl::distance(&convex1, &convex2, request, result));
  EXPECT_NEAR(abs(result.min_distance),
              (result.nearest_points[0] - result.nearest_points[1]).norm(),
              request.distance_tolerance);
}

template <typename S>
void test_distance_convex_to_convex3() {
  // This is a real world failure reported in
  // https://github.com/RobotLocomotion/drake/issues/21673#issuecomment-2689035046
  using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometryd>;
  CollisionGeometryPtr_t convex_geo1(new fcl::Convex<S>(
      std::make_shared<const std::vector<Vector3<S>>>(
          std::initializer_list<Vector3<S>>{
              Vector3<S>(0.5, 1, 0),
              Vector3<S>(0.5, 0.62348999999999998867, -0.7818310000000000537),
              Vector3<S>(0.5, -0.22252099999999999658, -0.97492800000000001681),
              Vector3<S>(0.5, 0.62348999999999998867, 0.7818310000000000537),
              Vector3<S>(-0.5, -0.90096900000000001985, 0.4338839999999999919),
              Vector3<S>(-0.5, -0.22252099999999999658, 0.97492800000000001681),
              Vector3<S>(-0.5, -0.90096900000000001985, -0.4338839999999999919),
              Vector3<S>(0.5, -0.90096900000000001985, 0.4338839999999999919),
              Vector3<S>(-0.5, 0.62348999999999998867, -0.7818310000000000537),
              Vector3<S>(-0.5, 0.62348999999999998867, 0.7818310000000000537),
              Vector3<S>(-0.5, -0.22252099999999999658,
                         -0.97492800000000001681),
              Vector3<S>(-0.5, 1, 0),
              Vector3<S>(0.5, -0.90096900000000001985, -0.4338839999999999919),
              Vector3<S>(0.5, -0.22252099999999999658, 0.97492800000000001681),
          }),
      9,
      std::make_shared<std::vector<int>>(std::initializer_list<int>{
          4, 10, 2, 12, 6, 4,  1,  8, 11, 0, 4,  10, 8, 1,  2,  7, 2,
          1, 0,  3, 13, 7, 12, 4,  9, 3,  0, 11, 4,  9, 5,  13, 3, 7,
          8, 10, 6, 4,  5, 9,  11, 4, 12, 7, 4,  6,  4, 13, 5,  4, 7,
      }),
      false));
  Transform3<S> X_WG1 = Transform3<S>::Identity();
  // clang-format off
  X_WG1.matrix() << 
    -2.0479437360480804916e-09, -0.76923712747984118732, -0.63896341186844385351, 13.670417904867957049,
    -2.4654844510601008403e-09, 0.63896341186844385351, -0.76923712747984118732, 13.169816779836704512,
    1, 0, -3.2051032938795742666e-09, 0,
    0, 0, 0, 1;
  // clang-format on
  fcl::CollisionObject<S> convex1(convex_geo1, X_WG1);

  CollisionGeometryPtr_t convex_geo2(new fcl::Convex<S>(
      std::make_shared<std::vector<Vector3<S>>>(
          std::initializer_list<Vector3<S>>{
              Vector3<S>(14.835100999999999871, 13.060409999999999187, -0.5),
              Vector3<S>(13.785037000000000873, 12.890523999999999205, -0.5),
              Vector3<S>(14.767995000000000871, 13.150247999999999493, -0.5),
              Vector3<S>(14.835100999999999871, 13.060409999999999187, 0.5),
              Vector3<S>(14.767995000000000871, 13.150247999999999493, 0.5),
              Vector3<S>(13.785037000000000873, 12.890523999999999205, 0.5),
          }),
      5,
      std::make_shared<std::vector<int>>(std::initializer_list<int>{
          3, 3, 4, 5, 4, 2, 1, 5, 4, 3, 2, 0, 1, 4, 3, 0, 2, 4, 4, 5, 1, 0, 3,
      }),
      false));
  const Transform3<S> X_WS2 = Transform3<S>::Identity();
  fcl::CollisionObject<S> convex2(convex_geo2, X_WS2);
  fcl::DistanceRequest<S> request;
  request.gjk_solver_type = GJKSolverType::GST_LIBCCD;
  request.distance_tolerance = 1e-6;
  request.enable_signed_distance = true;
  fcl::DistanceResult<S> result;
  ASSERT_NO_THROW(fcl::distance(&convex1, &convex2, request, result));
  EXPECT_NEAR(abs(result.min_distance),
              (result.nearest_points[0] - result.nearest_points[1]).norm(),
              request.distance_tolerance);
}

template <typename S>
void test_distance_convex_to_convex4() {
  // This is a real world failure reported in
  // https://github.com/RobotLocomotion/drake/issues/21673#issuecomment-2689248075
  using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometryd>;
  CollisionGeometryPtr_t convex_geo1(new fcl::Convex<S>(
      std::make_shared<const std::vector<Vector3<S>>>(
          std::initializer_list<Vector3<S>>{
              Vector3<S>(0.5, -0.92388000000000003453, 0.38268299999999999539),
              Vector3<S>(-0.5, 0.92388000000000003453, -0.38268299999999999539),
              Vector3<S>(0.5, 0.92388000000000003453, -0.38268299999999999539),
              Vector3<S>(-0.5, 0.92388000000000003453, 0.38268299999999999539),
              Vector3<S>(-0.5, -0.92388000000000003453,
                         -0.38268299999999999539),
              Vector3<S>(0.5, 0.92388000000000003453, 0.38268299999999999539),
              Vector3<S>(-0.5, -0.92388000000000003453, 0.38268299999999999539),
              Vector3<S>(0.5, -0.92388000000000003453, -0.38268299999999999539),
          }),
      6,
      std::make_shared<std::vector<int>>(std::initializer_list<int>{
          4, 3, 6, 0, 5, 4, 7, 2, 5, 0, 4, 6, 4, 7, 0,
          4, 2, 1, 3, 5, 4, 3, 1, 4, 6, 4, 7, 4, 1, 2,
      }),
      false));
  Transform3<S> X_WG1 = Transform3<S>::Identity();
  // clang-format off
  X_WG1.matrix() << 
    -2.6487935924268804787e-09, -0.56303944378188575115, -0.82643002410717436579, 15.712812998638135298,
    -1.8045995758494454423e-09, 0.82643002410717436579, -0.56303944378188575115, 10.219809745800642276,
    1, 0, -3.2051032938795742666e-09, 0,
    0, 0, 0, 1;
  // clang-format on
  fcl::CollisionObject<S> convex1(convex_geo1, X_WG1);

  CollisionGeometryPtr_t convex_geo2(new fcl::Convex<S>(
      std::make_shared<std::vector<Vector3<S>>>(
          std::initializer_list<Vector3<S>>{
              Vector3<S>(15.417082000000000619, 9.6986209999999992704, -0.5),
              Vector3<S>(16.236605000000000842, 9.2332309999999999661, -0.5),
              Vector3<S>(16.224319999999998743, 8.9158489999999996911, -0.5),
              Vector3<S>(15.417082000000000619, 9.6986209999999992704, 0.5),
              Vector3<S>(16.224319999999998743, 8.9158489999999996911, 0.5),
              Vector3<S>(16.236605000000000842, 9.2332309999999999661, 0.5),
          }),
      5,
      std::make_shared<std::vector<int>>(std::initializer_list<int>{
          3, 3, 4, 5, 4, 2, 1, 5, 4, 3, 2, 0, 1, 4, 4, 3, 0, 2, 4, 5, 1, 0, 3,
      }),
      false));
  const Transform3<S> X_WS2 = Transform3<S>::Identity();
  fcl::CollisionObject<S> convex2(convex_geo2, X_WS2);
  fcl::DistanceRequest<S> request;
  request.gjk_solver_type = GJKSolverType::GST_LIBCCD;
  request.distance_tolerance = 1e-6;
  request.enable_signed_distance = true;
  fcl::DistanceResult<S> result;
  ASSERT_NO_THROW(fcl::distance(&convex1, &convex2, request, result));
  EXPECT_NEAR(abs(result.min_distance),
              (result.nearest_points[0] - result.nearest_points[1]).norm(),
              request.distance_tolerance);
}

template <typename S>
void test_distance_convex_to_convex5() {
  // This is a real world failure reported in
  // https://github.com/RobotLocomotion/drake/issues/21673#issuecomment-2689297095
  using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometryd>;
  CollisionGeometryPtr_t convex_geo1(new fcl::Convex<S>(
      std::make_shared<const std::vector<Vector3<S>>>(
          std::initializer_list<Vector3<S>>{
              Vector3<S>(0.5, 0.5, -0.86602500000000004476),
              Vector3<S>(0.5, -0.5, -0.86602500000000004476),
              Vector3<S>(0.5, 1, 0),
              Vector3<S>(-0.5, -0.5, 0.86602500000000004476),
              Vector3<S>(-0.5, 0.5, 0.86602500000000004476),
              Vector3<S>(-0.5, -1, 0),
              Vector3<S>(0.5, -0.5, 0.86602500000000004476),
              Vector3<S>(-0.5, 0.5, -0.86602500000000004476),
              Vector3<S>(-0.5, -0.5, -0.86602500000000004476),
              Vector3<S>(-0.5, 1, 0),
              Vector3<S>(0.5, -1, 0),
              Vector3<S>(0.5, 0.5, 0.86602500000000004476),
          }),
      5,
      std::make_shared<std::vector<int>>(std::initializer_list<int>{
          4, 10, 5, 8, 1, 6, 2, 11, 6,  10, 1, 0,  4, 0,  7,
          9, 2,  4, 1, 8, 7, 0, 4,  2,  9,  4, 11, 6, 8,  5,
          3, 4,  9, 7, 4, 6, 3, 5,  10, 4,  4, 3,  6, 11,
      }),
      false));
  Transform3<S> X_WG1 = Transform3<S>::Identity();
  // clang-format off
  X_WG1.matrix() << 
    -2.2411796174427631456e-09, 0.7148738189791493669, -0.69925347545662319693, 6.3335747278141161232,
    2.2912444319183419853e-09, 0.69925347545662319693, 0.7148738189791493669, 8.4631576303276716544,
    1, 0, -3.2051032938795742666e-09, 0,
    0, 0, 0, 1;
  // clang-format on
  fcl::CollisionObject<S> convex1(convex_geo1, X_WG1);

  CollisionGeometryPtr_t convex_geo2(new fcl::Convex<S>(
      std::make_shared<std::vector<Vector3<S>>>(
          std::initializer_list<Vector3<S>>{
              Vector3<S>(5.2423630000000001061, 9.1228110000000004476, -0.5),
              Vector3<S>(6.3596640000000004278, 8.2852599999999991809, -0.5),
              Vector3<S>(5.6703770000000002227, 7.5985389999999997102, -0.5),
              Vector3<S>(5.2423630000000001061, 9.1228110000000004476, 0.5),
              Vector3<S>(5.6703770000000002227, 7.5985389999999997102, 0.5),
              Vector3<S>(6.3596640000000004278, 8.2852599999999991809, 0.5),
          }),
      5,
      std::make_shared<std::vector<int>>(std::initializer_list<int>{
          3, 3, 4, 5, 4, 4, 2, 1, 5, 3, 2, 0, 1, 4, 3, 0, 2, 4, 4, 1, 0, 3, 5,
      }),
      false));
  const Transform3<S> X_WS2 = Transform3<S>::Identity();
  fcl::CollisionObject<S> convex2(convex_geo2, X_WS2);
  fcl::DistanceRequest<S> request;
  request.gjk_solver_type = GJKSolverType::GST_LIBCCD;
  request.distance_tolerance = 1e-6;
  request.enable_signed_distance = true;
  fcl::DistanceResult<S> result;
  ASSERT_NO_THROW(fcl::distance(&convex1, &convex2, request, result));
  EXPECT_NEAR(abs(result.min_distance),
              (result.nearest_points[0] - result.nearest_points[1]).norm(),
              request.distance_tolerance);
}

template <typename S>
void test_distance_convex_to_convex6() {
  // This is a real world failure reported in
  // https://github.com/RobotLocomotion/drake/issues/21673#issuecomment-2689297095
  using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometryd>;
  CollisionGeometryPtr_t convex_geo1(new fcl::Convex<S>(
      std::make_shared<const std::vector<Vector3<S>>>(
          std::initializer_list<Vector3<S>>{
              Vector3<S>(0.5, 0.30901699999999998614, -0.95105700000000004124),
              Vector3<S>(0.5, 1, 0),
              Vector3<S>(-0.5, -0.80901699999999998614, 0.58778500000000000192),
              Vector3<S>(-0.5, 0.30901699999999998614, 0.95105700000000004124),
              Vector3<S>(-0.5, -0.80901699999999998614,
                         -0.58778500000000000192),
              Vector3<S>(0.5, -0.80901699999999998614, 0.58778500000000000192),
              Vector3<S>(-0.5, 0.30901699999999998614, -0.95105700000000004124),
              Vector3<S>(-0.5, 1, 0),
              Vector3<S>(0.5, -0.80901699999999998614, -0.58778500000000000192),
              Vector3<S>(0.5, 0.30901699999999998614, 0.95105700000000004124),
          }),
      7,
      std::make_shared<std::vector<int>>(std::initializer_list<int>{
          4, 0, 6, 7, 1, 5, 9, 5, 8, 0, 1, 4, 0, 8, 4, 6, 4, 1, 7,
          3, 9, 5, 6, 4, 2, 3, 7, 4, 8, 5, 2, 4, 4, 3, 2, 5, 9,
      }),
      false));
  Transform3<S> X_WG1 = Transform3<S>::Identity();
  // clang-format off
  X_WG1.matrix() << 
    -3.1934130613594990691e-09, -0.085331461972016672823, -0.99635261910516315087, 11.261494468063983021,
    -2.7349614983807029573e-10, 0.99635261910516315087, -0.085331461972016672823, 5.7102314848013024928,
    1, 0, -3.2051032938795742666e-09, 0,
    0, 0, 0, 1;
  // clang-format on
  fcl::CollisionObject<S> convex1(convex_geo1, X_WG1);

  CollisionGeometryPtr_t convex_geo2(new fcl::Convex<S>(
      std::make_shared<std::vector<Vector3<S>>>(
          std::initializer_list<Vector3<S>>{
              Vector3<S>(10.924768999999999508, 5.6701019999999999754, -0.5),
              Vector3<S>(10.83925699999999992, 4.8329899999999996751, -0.5),
              Vector3<S>(11.452507000000000659, 5.5209809999999999164, -0.5),
              Vector3<S>(10.924768999999999508, 5.6701019999999999754, 0.5),
              Vector3<S>(11.452507000000000659, 5.5209809999999999164, 0.5),
              Vector3<S>(10.83925699999999992, 4.8329899999999996751, 0.5),
          }),
      5,
      std::make_shared<std::vector<int>>(std::initializer_list<int>{
          3, 4, 3, 5, 4, 5, 1, 2, 4, 3, 1, 0, 2, 4, 2, 0, 3, 4, 4, 5, 3, 0, 1,
      }),
      false));
  const Transform3<S> X_WS2 = Transform3<S>::Identity();
  fcl::CollisionObject<S> convex2(convex_geo2, X_WS2);
  fcl::DistanceRequest<S> request;
  request.gjk_solver_type = GJKSolverType::GST_LIBCCD;
  request.distance_tolerance = 1e-6;
  request.enable_signed_distance = true;
  fcl::DistanceResult<S> result;
  ASSERT_NO_THROW(fcl::distance(&convex1, &convex2, request, result));
  EXPECT_NEAR(abs(result.min_distance),
              (result.nearest_points[0] - result.nearest_points[1]).norm(),
              request.distance_tolerance);
}

template <typename S>
void test_distance_convex_to_convex7() {
  // This is a real world failure reported in
  // https://github.com/RobotLocomotion/drake/issues/21673#issuecomment-2689678480
  using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometryd>;
  CollisionGeometryPtr_t convex_geo1(new fcl::Convex<S>(
      std::make_shared<const std::vector<Vector3<S>>>(
          std::initializer_list<Vector3<S>>{
              Vector3<S>(-0.5, -0.5, -0.86602500000000004476),
              Vector3<S>(-0.5, -0.5, 0.86602500000000004476),
              Vector3<S>(-0.5, 1, 0),
              Vector3<S>(0.5, -0.5, -0.86602500000000004476),
              Vector3<S>(0.5, 1, 0),
              Vector3<S>(0.5, -0.5, 0.86602500000000004476),
          }),
      5,
      std::make_shared<std::vector<int>>(std::initializer_list<int>{
          3, 5, 3, 4, 4, 2, 1, 5, 4, 3, 2, 0, 1, 4, 3, 0, 2, 4, 4, 1, 0, 3, 5,
      }),
      false));
  Transform3<S> X_WG1 = Transform3<S>::Identity();
  // clang-format off
  X_WG1.matrix() << 
    -3.155367699156126597e-09, -0.17548349096519452739, -0.98448237383849002136, 9.4277544959429171456,
    -5.6244271491403150214e-10, 0.98448237383849002136, -0.17548349096519452739, 4.3549730982604870633,
    1, 0, -3.2051032938795742666e-09, 0,
    0, 0, 0, 1;
  // clang-format on
  fcl::CollisionObject<S> convex1(convex_geo1, X_WG1);

  CollisionGeometryPtr_t convex_geo2(new fcl::Convex<S>(
      std::make_shared<std::vector<Vector3<S>>>(
          std::initializer_list<Vector3<S>>{
              Vector3<S>(7.502455000000000318, 3.9926669999999999661, -0.5),
              Vector3<S>(7.9127499999999999503, 2.7334600000000000009, -0.5),
              Vector3<S>(8.6743819999999995929, 3.7108469999999997846, -0.5),
              Vector3<S>(7.502455000000000318, 3.9926669999999999661, 0.5),
              Vector3<S>(8.6743819999999995929, 3.7108469999999997846, 0.5),
              Vector3<S>(7.9127499999999999503, 2.7334600000000000009, 0.5),
          }),
      5,
      std::make_shared<std::vector<int>>(std::initializer_list<int>{
          3, 3, 5, 4, 4, 5, 1, 2, 4, 3, 1, 0, 2, 4, 4, 2, 0, 3, 4, 3, 0, 1, 5,
      }),
      false));
  const Transform3<S> X_WS2 = Transform3<S>::Identity();
  fcl::CollisionObject<S> convex2(convex_geo2, X_WS2);
  fcl::DistanceRequest<S> request;
  request.gjk_solver_type = GJKSolverType::GST_LIBCCD;
  request.distance_tolerance = 1e-6;
  request.enable_signed_distance = true;
  fcl::DistanceResult<S> result;
  ASSERT_NO_THROW(fcl::distance(&convex1, &convex2, request, result));
  EXPECT_NEAR(abs(result.min_distance),
              (result.nearest_points[0] - result.nearest_points[1]).norm(),
              request.distance_tolerance);
}

template <typename S>
void test_distance_convex_to_sphere() {
  // This is reported in
  // https://github.com/RobotLocomotion/drake/issues/21673#issue-2387206489
  using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometryd>;
  CollisionGeometryPtr_t convex_geo(new fcl::Convex<S>(
      std::make_shared<const std::vector<Vector3<S>>>(
          std::initializer_list<Vector3<S>>{
              Vector3<S>(0.027183880000000000543, -0.021052830000000001542,
                         -0.013271629999999999538),
              Vector3<S>(-0.029787089999999998707, -0.018163209999999999078,
                         0.0023578100000000001141),
              Vector3<S>(-0.029523460000000001369, -0.014922700000000000548,
                         0.0079630499999999992955),
              Vector3<S>(0.018158350000000000185, -0.028047450000000001546,
                         -0.016771020000000001066),
              Vector3<S>(-0.033257000000000001838, -0.0034294899999999998823,
                         -0.02402831000000000064),
              Vector3<S>(0.030779520000000001184, -0.016741050000000000375,
                         -0.0062909699999999998954),
              Vector3<S>(0.031590300000000001712, -0.011201890000000000805,
                         0.0022611499999999999655),
              Vector3<S>(0.030848339999999998401, -0.016967369999999998981,
                         -0.013022880000000000567),
              Vector3<S>(0.0089922000000000005399, -0.005754650000000000043,
                         -0.039821399999999999852),
              Vector3<S>(0.0084664200000000005425, -0.0017271199999999999573,
                         -0.04059970999999999719),
              Vector3<S>(-0.00074333999999999999467, -0.0034554999999999998044,
                         -0.042391610000000003178),
              Vector3<S>(-0.0031593099999999998094, -0.0056862400000000003913,
                         -0.042894870000000001609),
              Vector3<S>(0.028471980000000000899, -0.017014069999999999194,
                         -0.019219239999999998547),
              Vector3<S>(0.019703479999999998912, -0.0068651700000000002999,
                         -0.034910230000000000428),
              Vector3<S>(-0.0061926200000000002424, -0.0083001399999999992546,
                         -0.04281131999999999993),
              Vector3<S>(0.014279659999999999576, -0.032117050000000001153,
                         -0.010280209999999999695),
              Vector3<S>(-0.00242264000000000021, -0.028708859999999999241,
                         0.013088809999999999334),
              Vector3<S>(-0.0227650299999999986, -0.02248137000000000052,
                         0.0086427499999999993913),
              Vector3<S>(-0.0008474599999999999668, -0.0094668600000000005523,
                         0.059735150000000000747),
              Vector3<S>(0.0099082100000000006279, -0.032700279999999998237,
                         -0.00028732999999999997475),
              Vector3<S>(0.0054166600000000002246, -0.032023179999999998313,
                         0.003964360000000000106),
              Vector3<S>(-0.015149090000000000472, -0.030018500000000000044,
                         0.0038727300000000000092),
              Vector3<S>(-0.013153349999999999417, -0.028098009999999999653,
                         0.0098451100000000006662),
              Vector3<S>(-0.0053969600000000001253, -0.031911280000000000201,
                         0.0046099399999999998712),
              Vector3<S>(-0.018971149999999999125, -0.026639590000000001185,
                         0.0068275799999999997603),
              Vector3<S>(0.0087256299999999998446, -0.033899480000000002666,
                         -0.0082973800000000000332),
              Vector3<S>(0.013405669999999999739, -0.0027963400000000000235,
                         0.054792069999999998275),
              Vector3<S>(0.031803619999999997725, -0.0015181899999999999909,
                         0.0080110000000000007592),
              Vector3<S>(0.032492689999999997685, -0.0061319699999999996085,
                         0.0053189999999999999433),
              Vector3<S>(0.035098669999999998481, 0.0117753500000000004,
                         -0.011222400000000000431),
              Vector3<S>(-0.031461019999999999264, 0.0024656800000000000418,
                         -0.026877829999999998389),
              Vector3<S>(-0.037982410000000001171, -0.005085420000000000218,
                         -0.0077946800000000000114),
              Vector3<S>(-0.035593800000000001826, -0.0088536900000000008648,
                         0.0020318599999999999162),
              Vector3<S>(0.033346519999999997552, -0.012396209999999999521,
                         -0.009431099999999999553),
              Vector3<S>(0.034542349999999999444, -0.0068445700000000003413,
                         -0.004166679999999999863),
              Vector3<S>(0.033087320000000003395, 0.014188879999999999201,
                         -0.016763059999999999766),
              Vector3<S>(0.034359969999999996626, 0.0071343600000000001807,
                         -0.018503900000000000209),
              Vector3<S>(0.036166349999999999887, 0.0066439500000000000293,
                         -0.01100461999999999968),
              Vector3<S>(0.036419540000000000246, 0.00087060000000000001968,
                         -0.0079107799999999992013),
              Vector3<S>(0.011389210000000000306, 0.014450730000000000172,
                         -0.038099599999999997357),
              Vector3<S>(0.0067127400000000000263, 0.012087620000000000309,
                         -0.039504400000000002013),
              Vector3<S>(0.011038169999999999926, 0.0088140000000000006536,
                         -0.039545289999999996833),
              Vector3<S>(-0.0048918599999999996114, 0.021446070000000001105,
                         -0.035098230000000001094),
              Vector3<S>(-0.0082483799999999996011, -0.023113859999999999961,
                         -0.034008789999999997078),
              Vector3<S>(-0.0037171800000000000501, -0.023711949999999999,
                         -0.033219249999999998779),
              Vector3<S>(0.020971299999999998276, -0.015664210000000001327,
                         -0.028963780000000001469),
              Vector3<S>(0.023128180000000001654, -0.0092023399999999998256,
                         -0.031482669999999997323),
              Vector3<S>(0.023568780000000000979, -0.018194390000000001256,
                         -0.024328539999999999194),
              Vector3<S>(0.015401889999999999678, -0.025274760000000000282,
                         -0.023878230000000000288),
              Vector3<S>(0.027065840000000000731, -0.014684070000000000455,
                         -0.023979440000000001171),
              Vector3<S>(0.010216879999999999365, -0.018237280000000001545,
                         -0.033417189999999999395),
              Vector3<S>(0.01175185999999999939, -0.022262560000000000682,
                         -0.029706659999999999316),
              Vector3<S>(0.018324619999999999798, -0.011293390000000000373,
                         -0.033725839999999999985),
              Vector3<S>(-0.0018270199999999999461, -0.01097317000000000084,
                         -0.041432929999999999759),
              Vector3<S>(-0.0041633099999999999274, -0.017102530000000001204,
                         -0.039001269999999997606),
              Vector3<S>(0.0045154000000000001427, -0.021309399999999999176,
                         -0.032979870000000001407),
              Vector3<S>(-0.037169290000000000662, 0.0042412300000000003469,
                         -0.012396320000000000602),
              Vector3<S>(-0.036575099999999999278, -0.00016132000000000000723,
                         -0.016787119999999999126),
              Vector3<S>(-0.034129270000000003216, 0.0049926299999999996376,
                         -0.021876170000000000337),
              Vector3<S>(-0.038314090000000002034, -0.00010391999999999999437,
                         -0.0084661600000000003519),
              Vector3<S>(-0.035380439999999999112, 0.011075490000000000335,
                         -0.007142879999999999889),
              Vector3<S>(0.0037589899999999998141, -0.011433399999999999813,
                         0.056924339999999996886),
              Vector3<S>(0.0077904400000000000384, -0.030593829999999998942,
                         0.007280520000000000394),
              Vector3<S>(0.014042429999999999843, -0.026013370000000000937,
                         0.013001459999999999406),
              Vector3<S>(0.0064805000000000001492, -0.010864220000000000821,
                         0.056767869999999998054),
              Vector3<S>(0.011707050000000000095, -0.0064067999999999998215,
                         0.056213720000000001742),
              Vector3<S>(-0.037302330000000001597, 0.0061722100000000000228,
                         -0.0041634499999999999634),
              Vector3<S>(-0.0073784999999999996423, 0.0074570100000000000273,
                         0.060883899999999997743),
              Vector3<S>(0.00099029000000000000706, -0.0087579900000000002191,
                         0.062146310000000003215),
              Vector3<S>(-0.0045025100000000003661, -0.0029720599999999998929,
                         0.065337430000000001895),
              Vector3<S>(-0.012138249999999999734, -0.022980000000000000426,
                         -0.032674979999999999303),
              Vector3<S>(-0.0069329999999999999266, -0.03119887999999999828,
                         -0.021099469999999998643),
              Vector3<S>(-0.0085740999999999994302, -0.027816799999999999166,
                         -0.027707220000000001153),
              Vector3<S>(0.0073825999999999995793, -0.031321920000000003093,
                         -0.020334149999999998754),
              Vector3<S>(-0.0015560400000000000707, -0.033037619999999996712,
                         -0.016474869999999999098),
              Vector3<S>(0.0082950300000000001116, -0.033270510000000003137,
                         -0.014296379999999999158),
              Vector3<S>(-0.0026995500000000002133, -0.029481750000000001011,
                         -0.025187859999999999416),
              Vector3<S>(0.011101450000000000554, -0.027988699999999998302,
                         -0.023916280000000001565),
              Vector3<S>(0.013075609999999999664, -0.031302560000000000107,
                         -0.016460619999999998725),
              Vector3<S>(0.0052021200000000001815, -0.025824889999999999651,
                         -0.028767419999999998409),
              Vector3<S>(-0.0077482499999999999291, 0.00056930000000000001437,
                         0.061994699999999999807),
              Vector3<S>(-0.0050962200000000002637, -0.0002348399999999999945,
                         0.066903329999999996969),
              Vector3<S>(-0.03249925000000000036, -0.0069879800000000004065,
                         0.0091488599999999999784),
              Vector3<S>(-0.033674719999999998266, 0.0014019200000000000758,
                         0.0088447600000000001219),
              Vector3<S>(-0.033445429999999998216, -0.0026828600000000000191,
                         0.0092199699999999992078),
              Vector3<S>(-0.037028619999999998202, -0.0037831200000000000591,
                         0.0015029100000000000965),
              Vector3<S>(-0.037284619999999997486, 0.001632049999999999907,
                         0.00098930000000000003178),
              Vector3<S>(-0.034590629999999997213, 0.0067227800000000002487,
                         0.0044326799999999996066),
              Vector3<S>(-0.0094180500000000007266, 0.0049390099999999997546,
                         0.058648100000000001619),
              Vector3<S>(-0.0052111900000000000929, 0.0033614399999999998274,
                         0.067508929999999994775),
              Vector3<S>(0.0037212299999999998504, 0.010245650000000000243,
                         0.065721760000000004065),
              Vector3<S>(0.0076209499999999995995, 0.0093117500000000005822,
                         0.064336430000000000007),
              Vector3<S>(0.0054773900000000003835, 0.0087120500000000006963,
                         0.067009550000000001058),
              Vector3<S>(0.0020273199999999999901, 0.008266920000000000518,
                         0.067950499999999997014),
              Vector3<S>(0.011028349999999999265, 0.030240240000000001563,
                         0.0061388299999999996343),
              Vector3<S>(0.012777139999999999331, 0.032678089999999999915,
                         -0.0022642399999999998292),
              Vector3<S>(0.01710804000000000144, 0.030696379999999998806,
                         -0.00042582999999999998023),
              Vector3<S>(0.0054541199999999998016, 0.012533549999999999205,
                         0.058348329999999996864),
              Vector3<S>(0.0021105299999999997847, 0.011794060000000000307,
                         0.062529959999999995546),
              Vector3<S>(0.0068770999999999997063, 0.033260299999999999587,
                         -0.001305680000000000035),
              Vector3<S>(-0.0020801899999999999905, 0.0084830900000000004219,
                         0.066246540000000006532),
              Vector3<S>(-0.001460399999999999936, 0.0066289000000000000701,
                         0.068647559999999996361),
              Vector3<S>(-0.0050289200000000000915, 0.006102309999999999679,
                         0.066335690000000002708),
              Vector3<S>(0.00060586999999999999796, 0.0098527000000000006935,
                         0.065969500000000000361),
              Vector3<S>(0.0021326299999999999424, 0.0025524200000000001227,
                         0.071039140000000000619),
              Vector3<S>(-0.00066016000000000002196, 0.0050919499999999996015,
                         0.069951589999999994052),
              Vector3<S>(0.028187960000000001487, 0.015659420000000000284,
                         0.010167630000000000418),
              Vector3<S>(0.034470040000000000402, 0.013093240000000000781,
                         -0.0040440700000000003034),
              Vector3<S>(0.03283171000000000006, 0.017168470000000001646,
                         -0.0088750900000000004092),
              Vector3<S>(0.031600280000000001424, 0.0087607099999999996781,
                         0.0083618800000000003542),
              Vector3<S>(0.035661009999999999931, 0.0082465200000000002389,
                         -0.0032231099999999999507),
              Vector3<S>(0.034652780000000001082, 0.0013641099999999999753,
                         0.0011960399999999999938),
              Vector3<S>(0.031985189999999996679, 0.013254409999999999456,
                         0.0045567799999999998584),
              Vector3<S>(-0.010299060000000000506, -0.0092051000000000007817,
                         -0.041797470000000003232),
              Vector3<S>(-0.0092165700000000003178, -0.017394500000000000239,
                         -0.03853568000000000271),
              Vector3<S>(-0.022353430000000000522, -0.0027308499999999999441,
                         -0.03642955999999999972),
              Vector3<S>(-0.022225390000000001117, 0.0039618600000000002079,
                         -0.035656199999999999006),
              Vector3<S>(-0.015682279999999999831, -0.0056126099999999996479,
                         -0.04035566000000000153),
              Vector3<S>(-0.027846519999999999606, -0.0039228900000000000575,
                         -0.031703420000000002987),
              Vector3<S>(-0.027120209999999998762, 0.004829029999999999781,
                         -0.031571660000000001389),
              Vector3<S>(-0.0093324299999999991984, 0.010744770000000000709,
                         -0.039781480000000001007),
              Vector3<S>(-0.0014391000000000000146, 0.017773230000000000972,
                         -0.038229340000000000543),
              Vector3<S>(8.5039999999999998947e-05, 0.013986919999999999908,
                         -0.039357910000000002892),
              Vector3<S>(-0.0044233500000000003718, -0.0018558900000000000136,
                         -0.04265918999999999961),
              Vector3<S>(-0.014487369999999999415, 0.0016356499999999999945,
                         -0.04040901000000000215),
              Vector3<S>(-0.010538570000000000576, -0.0029345899999999999416,
                         -0.042129229999999996681),
              Vector3<S>(-0.015962330000000000269, 0.0090826499999999994045,
                         -0.03792977000000000154),
              Vector3<S>(-0.036574179999999997803, -0.0097543999999999998762,
                         -0.0075080199999999997981),
              Vector3<S>(-0.033704739999999996924, -0.013025870000000000157,
                         0.0018170899999999999379),
              Vector3<S>(0.034156060000000001975, -0.0046543399999999998662,
                         -0.017560309999999999114),
              Vector3<S>(0.034825160000000000837, -0.0074145100000000000243,
                         -0.011929109999999999714),
              Vector3<S>(0.03272314999999999946, -0.011940320000000000794,
                         -0.015893739999999999951),
              Vector3<S>(0.031828500000000002623, -0.0041785399999999996851,
                         -0.02266736000000000098),
              Vector3<S>(0.034179660000000000597, 0.00097746000000000009093,
                         -0.019572729999999999989),
              Vector3<S>(0.036204550000000002008, 0.0014915600000000000646,
                         -0.012584690000000000806),
              Vector3<S>(0.029729769999999999114, -0.010278580000000000702,
                         -0.023300259999999999722),
              Vector3<S>(0.031344660000000003353, 0.0077234399999999998193,
                         -0.024024639999999999884),
              Vector3<S>(0.021932480000000000725, 0.004916069999999999815,
                         -0.034479769999999999863),
              Vector3<S>(0.027761899999999999078, -0.00089461000000000000156,
                         -0.028433300000000001628),
              Vector3<S>(0.020942889999999998729, -0.0022780899999999999761,
                         -0.034853729999999999434),
              Vector3<S>(0.025047690000000000926, 0.0082217699999999997729,
                         -0.031514710000000001333),
              Vector3<S>(0.019440909999999998747, 0.010786099999999999854,
                         -0.035222089999999997567),
              Vector3<S>(0.01453940999999999914, 0.0032053099999999998433,
                         -0.038334540000000000282),
              Vector3<S>(0.011955900000000000208, -0.00039829999999999997785,
                         -0.039916790000000000604),
              Vector3<S>(0.01293090000000000038, -0.0034708500000000001504,
                         -0.039341760000000003394),
              Vector3<S>(0.028623920000000000613, 0.02182950000000000168,
                         -0.01518191999999999979),
              Vector3<S>(0.024512079999999998592, 0.027111690000000000789,
                         -0.0053190199999999998245),
              Vector3<S>(0.018370179999999999843, 0.031326470000000002092,
                         -0.0065683900000000003394),
              Vector3<S>(0.023603300000000000669, 0.026578919999999998935,
                         -0.00049631000000000002371),
              Vector3<S>(0.030971880000000000244, 0.019377669999999999617,
                         -0.0015668699999999999382),
              Vector3<S>(0.028304059999999998942, 0.023546589999999999188,
                         -0.0082835600000000005755),
              Vector3<S>(0.023538429999999999076, 0.027614610000000001239,
                         -0.013220100000000000254),
              Vector3<S>(0.027690380000000000549, 0.021773330000000000461,
                         0.002170529999999999942),
              Vector3<S>(0.0091080499999999994798, -0.0091594399999999995959,
                         0.057033790000000000875),
              Vector3<S>(0.023299000000000000266, -0.018284720000000000834,
                         0.013115949999999999484),
              Vector3<S>(0.014850709999999999605, -0.03002555999999999975,
                         0.0028377300000000001134),
              Vector3<S>(0.014895280000000000326, -0.031716779999999999973,
                         -0.0044480099999999996377),
              Vector3<S>(0.019292619999999999908, -0.029475859999999999561,
                         -0.0092679000000000007903),
              Vector3<S>(0.025049129999999999313, -0.023864019999999999677,
                         -0.0064475899999999995715),
              Vector3<S>(0.0045654800000000002325, -0.0046508900000000004057,
                         0.068109970000000005785),
              Vector3<S>(0.0071369399999999996037, -0.0053390900000000003287,
                         0.065027230000000005306),
              Vector3<S>(0.0047332199999999997886, -0.0086559299999999991804,
                         0.062252920000000003253),
              Vector3<S>(0.0016929899999999999029, -0.0053916600000000003759,
                         0.067386199999999993326),
              Vector3<S>(-0.0017180699999999999271, -0.0041145399999999998641,
                         0.067338880000000003734),
              Vector3<S>(-0.0032832799999999999659, 0.0043271400000000001529,
                         0.069377569999999999517),
              Vector3<S>(-0.0051890699999999996203, 0.0020436400000000002133,
                         0.067675780000000004821),
              Vector3<S>(-0.011610519999999999174, -0.03274645999999999807,
                         -0.0082870200000000008439),
              Vector3<S>(-0.0078259899999999996828, -0.033378369999999997486,
                         -0.012978679999999999384),
              Vector3<S>(-0.015327580000000000371, -0.031420950000000003044,
                         -0.0027512399999999998489),
              Vector3<S>(-0.0057267500000000000696, -0.033659630000000002881,
                         -0.0023374200000000002093),
              Vector3<S>(-0.016538089999999998347, -0.026016830000000001205,
                         -0.025430149999999998589),
              Vector3<S>(-0.014275190000000000101, -0.030094130000000000186,
                         -0.019378889999999999311),
              Vector3<S>(-0.016217579999999998802, -0.031097440000000000498,
                         -0.011813570000000000668),
              Vector3<S>(-0.033669419999999998516, -0.0092691600000000002463,
                         -0.020414029999999999676),
              Vector3<S>(-0.035915320000000000578, -0.0085079800000000000565,
                         -0.015324690000000000187),
              Vector3<S>(-0.030847889999999999339, -0.0098101700000000000013,
                         -0.025263540000000000996),
              Vector3<S>(-0.028496629999999998489, 0.018986469999999998348,
                         -0.002048889999999999826),
              Vector3<S>(-0.033265080000000002425, 0.011770229999999999582,
                         0.0010801400000000000497),
              Vector3<S>(-0.0055287299999999999292, 0.0096833400000000003499,
                         0.060793149999999997191),
              Vector3<S>(-0.030762709999999998667, 0.018029190000000000493,
                         -0.0082223499999999997978),
              Vector3<S>(-0.013886070000000000357, 0.02726142999999999969,
                         0.0060557900000000001312),
              Vector3<S>(-0.0022062000000000001748, 0.012459589999999999554,
                         0.058902210000000003454),
              Vector3<S>(-0.0072462400000000001463, 0.032024860000000002214,
                         -0.0012487999999999999969),
              Vector3<S>(-0.00055557999999999996277, 0.033331760000000001942,
                         -0.0021519700000000000113),
              Vector3<S>(-0.022859230000000001215, 0.024688370000000001148,
                         -0.0022040599999999998734),
              Vector3<S>(-0.018710540000000001198, 0.027941279999999998895,
                         -0.0027532300000000001737),
              Vector3<S>(-0.013369549999999999143, 0.030437059999999998422,
                         -0.0020633499999999998196),
              Vector3<S>(-0.03131828999999999863, 0.017519050000000001288,
                         -0.013494509999999999492),
              Vector3<S>(-0.033913869999999998739, 0.012510020000000000168,
                         -0.01645208000000000087),
              Vector3<S>(0.0058952299999999995678, 0.0057428599999999998052,
                         0.06945233000000000656),
              Vector3<S>(0.0048705800000000002217, 0.00077167999999999995712,
                         0.070842390000000005079),
              Vector3<S>(0.0086218300000000004657, 0.0046233699999999999103,
                         0.067643570000000000086),
              Vector3<S>(0.0098997500000000005632, 0.0042444800000000001278,
                         0.065378140000000001253),
              Vector3<S>(0.0081489600000000002311, -0.0017313700000000000877,
                         0.067056519999999994464),
              Vector3<S>(0.0072053400000000001821, 6.851000000000000644e-05,
                         0.068725449999999993489),
              Vector3<S>(0.0069279299999999996787, 0.0043608600000000002569,
                         0.069146330000000005844),
              Vector3<S>(0.0074433299999999997382, 0.0068037399999999999614,
                         0.06809990999999999961),
              Vector3<S>(0.0062098800000000000873, -0.0016449299999999999599,
                         0.069681800000000002071),
              Vector3<S>(0.0088494799999999998463, 0.0075207099999999998952,
                         0.06508793999999999691),
              Vector3<S>(-0.02067228000000000121, -0.011077009999999999981,
                         -0.035691390000000003391),
              Vector3<S>(-0.014811210000000000001, -0.011239040000000000488,
                         -0.039438069999999998683),
              Vector3<S>(-0.022293730000000001185, -0.0062162500000000004211,
                         -0.036217409999999998493),
              Vector3<S>(-0.027381650000000000433, -0.010060650000000000842,
                         -0.02982813000000000131),
              Vector3<S>(-0.019136090000000001432, -0.017794290000000000801,
                         -0.032516849999999999921),
              Vector3<S>(-0.013671449999999999922, -0.01684189000000000172,
                         -0.037140550000000001341),
              Vector3<S>(0.026086089999999999278, 0.017189750000000000169,
                         -0.026275239999999998319),
              Vector3<S>(0.029158739999999998965, 0.017896579999999998739,
                         -0.021124150000000001259),
              Vector3<S>(0.031662370000000002179, 0.01192606000000000048,
                         -0.021868649999999999894),
              Vector3<S>(0.025961120000000000724, 0.012260200000000000611,
                         -0.029249429999999999874),
              Vector3<S>(0.0040144799999999999582, 0.021106190000000000367,
                         -0.036003029999999998367),
              Vector3<S>(0.0090103400000000003628, 0.018446429999999999771,
                         -0.03725748000000000254),
              Vector3<S>(0.01735584000000000085, 0.017988569999999998866,
                         -0.03379345999999999739),
              Vector3<S>(0.019792589999999998795, 0.03005421000000000134,
                         -0.014539260000000000031),
              Vector3<S>(0.013834239999999999385, 0.033269439999999997348,
                         -0.0084886100000000005994),
              Vector3<S>(0.023422880000000000089, -0.023652739999999998488,
                         0.002346650000000000038),
              Vector3<S>(0.020811360000000000969, -0.021237510000000001109,
                         0.013375269999999999868),
              Vector3<S>(0.018234910000000000008, -0.024058920000000000999,
                         0.012448179999999999662),
              Vector3<S>(0.019704719999999998487, -0.028268549999999999928,
                         -0.00088683999999999995906),
              Vector3<S>(0.026966049999999998354, -0.020708139999999999881,
                         -0.00022148999999999999526),
              Vector3<S>(0.00011011999999999999599, -0.0010885700000000000758,
                         0.070988449999999994611),
              Vector3<S>(-0.00076446999999999994992, -0.0026546700000000000332,
                         0.069666649999999996634),
              Vector3<S>(-0.0028291200000000000722, -0.0011730099999999999381,
                         0.069221099999999993746),
              Vector3<S>(-0.002507459999999999984, 0.0013405199999999999928,
                         0.070080429999999999113),
              Vector3<S>(0.0031360899999999997979, -0.00095268999999999996048,
                         0.071229929999999996859),
              Vector3<S>(0.0016073999999999999313, 0.00057948999999999999642,
                         0.071602250000000006169),
              Vector3<S>(-0.00011024000000000000162, 0.0018162799999999999778,
                         0.071025510000000000033),
              Vector3<S>(0.0031515200000000001025, -0.0031407399999999999381,
                         0.070098980000000005175),
              Vector3<S>(-0.020332200000000001661, -0.028966740000000001237,
                         -0.011345940000000000539),
              Vector3<S>(-0.024064970000000001499, -0.025612119999999998637,
                         -0.0011907099999999999889),
              Vector3<S>(-0.030006450000000000483, -0.020231829999999999259,
                         -0.0054169200000000004153),
              Vector3<S>(-0.033556669999999996779, -0.015653580000000000272,
                         -0.0075860800000000002535),
              Vector3<S>(-0.020390309999999998297, -0.028651989999999998709,
                         -0.0034434199999999999878),
              Vector3<S>(-0.026516680000000000805, -0.021229620000000001129,
                         -0.019795799999999998814),
              Vector3<S>(-0.027428179999999999922, -0.015661109999999998921,
                         -0.025506339999999998874),
              Vector3<S>(-0.030514779999999998295, -0.016581829999999998793,
                         -0.019057480000000001641),
              Vector3<S>(-0.026496260000000000645, -0.024158320000000000488,
                         -0.011135060000000000444),
              Vector3<S>(-0.021157499999999999224, -0.026530709999999999016,
                         -0.01914973999999999843),
              Vector3<S>(-0.021090000000000001107, -0.021734989999999999172,
                         -0.026522790000000000948),
              Vector3<S>(-0.032053739999999997234, -0.016518999999999998962,
                         -0.014478790000000000063),
              Vector3<S>(-0.024509349999999999192, 0.025156549999999999745,
                         -0.012265329999999999636),
              Vector3<S>(-0.019466580000000000689, 0.028672710000000000558,
                         -0.0092381000000000008249),
              Vector3<S>(-0.02874861000000000083, 0.019476170000000000981,
                         -0.018492680000000000923),
              Vector3<S>(-0.031018560000000000576, 0.013365580000000000169,
                         -0.022315240000000000076),
              Vector3<S>(-0.029204250000000001042, 0.010545449999999999616,
                         -0.026987199999999999384),
              Vector3<S>(0.021474389999999999451, 0.026594119999999998871,
                         -0.020703679999999998612),
              Vector3<S>(0.022505580000000000723, 0.023238700000000001022,
                         -0.024652239999999998876),
              Vector3<S>(-0.00036181000000000000704, 0.034050490000000002699,
                         -0.01273379999999999998),
              Vector3<S>(-0.0044160400000000004164, 0.033712369999999998449,
                         -0.0080847599999999998632),
              Vector3<S>(0.0071834899999999998407, 0.03429912999999999712,
                         -0.0077288599999999997234),
              Vector3<S>(-0.0028971999999999999489, 0.033064589999999997871,
                         -0.017492839999999999084),
              Vector3<S>(-0.013011109999999999343, 0.031868019999999996905,
                         -0.011318840000000000151),
              Vector3<S>(-0.019506740000000001578, 0.028437899999999998596,
                         -0.01572874000000000147),
              Vector3<S>(-0.012869830000000000506, 0.030362710000000001087,
                         -0.019402499999999999608),
              Vector3<S>(-0.0056328899999999997722, 0.030305209999999999093,
                         -0.023471010000000000484),
              Vector3<S>(-0.0072697500000000001702, 0.026216130000000000683,
                         -0.029304009999999998393),
              Vector3<S>(-0.020643069999999999475, 0.020824269999999998892,
                         -0.02848161999999999916),
              Vector3<S>(-0.026844659999999999356, 0.01801700999999999997,
                         -0.024712649999999999201),
              Vector3<S>(-0.024168519999999998893, 0.023576659999999999284,
                         -0.021368100000000000982),
              Vector3<S>(-0.017981910000000000255, 0.027016669999999999574,
                         -0.0223778799999999993),
              Vector3<S>(-0.01639281999999999892, 0.024456990000000001367,
                         -0.027387100000000001027),
              Vector3<S>(-0.024190960000000000935, 0.013179309999999999636,
                         -0.030650560000000000305),
              Vector3<S>(-0.011886670000000000222, 0.015358280000000000196,
                         -0.037266779999999999351),
              Vector3<S>(-0.010615869999999999473, 0.020040129999999999749,
                         -0.034567359999999998366),
              Vector3<S>(0.010798570000000000391, 0.022242129999999998846,
                         -0.034256830000000002001),
              Vector3<S>(0.017773069999999998453, 0.022356720000000000065,
                         -0.03021180000000000046),
              Vector3<S>(0.015831810000000001715, 0.026751420000000001448,
                         -0.026548840000000000633),
              Vector3<S>(0.0050911300000000001345, 0.02498018999999999934,
                         -0.032360949999999999438),
              Vector3<S>(0.010559100000000000083, 0.031081709999999998506,
                         -0.022219590000000000868),
              Vector3<S>(0.011106800000000000006, 0.027135090000000000598,
                         -0.028797739999999998756),
              Vector3<S>(0.003574690000000000159, 0.029203719999999998985,
                         -0.026545699999999998464),
              Vector3<S>(0.015599059999999999662, 0.030550640000000000435,
                         -0.020270250000000000073),
              Vector3<S>(0.0019087799999999998952, 0.031599679999999998048,
                         -0.021888689999999998842),
              Vector3<S>(0.0094769299999999997292, 0.033574489999999998502,
                         -0.014093150000000000469),
          }),
      542,
      std::make_shared<std::vector<int>>(std::initializer_list<int>{
          3,   65,  154, 6,   3,   65,  6,   28,  3,   23,  61,  18,  3,   154,
          218, 6,   3,   26,  65,  28,  3,   229, 1,   128, 3,   17,  24,  18,
          3,   228, 24,  17,  3,   1,   228, 17,  3,   229, 228, 1,   3,   28,
          6,   34,  3,   158, 0,   7,   3,   12,  0,   3,   3,   7,   0,   12,
          3,   59,  85,  86,  3,   66,  59,  86,  3,   260, 126, 116, 3,   127,
          128, 32,  3,   6,   33,  34,  3,   133, 132, 136, 3,   108, 29,  35,
          3,   29,  36,  35,  3,   36,  133, 136, 3,   144, 53,  11,  3,   242,
          243, 58,  3,   224, 223, 190, 3,   153, 154, 65,  3,   193, 65,  26,
          3,   24,  22,  18,  3,   22,  23,  18,  3,   18,  61,  68,  3,   67,
          88,  102, 3,   178, 67,  102, 3,   102, 88,  89,  3,   157, 0,   158,
          3,   3,   0,   157, 3,   78,  3,   157, 3,   23,  16,  61,  3,   19,
          156, 155, 3,   19,  155, 62,  3,   25,  156, 19,  3,   128, 1,   2,
          3,   1,   17,  2,   3,   32,  2,   81,  3,   32,  128, 2,   3,   2,
          69,  81,  3,   2,   17,  69,  3,   49,  12,  47,  3,   47,  3,   48,
          3,   47,  12,  3,   3,   77,  3,   78,  3,   77,  48,  3,   3,   82,
          32,  81,  3,   85,  32,  82,  3,   91,  152, 148, 3,   91,  148, 96,
          3,   91,  198, 152, 3,   27,  28,  111, 3,   111, 34,  38,  3,   28,
          34,  111, 3,   111, 38,  110, 3,   243, 30,  58,  3,   30,  118, 4,
          3,   58,  30,  4,   3,   118, 201, 202, 3,   126, 124, 116, 3,   4,
          173, 174, 3,   58,  4,   57,  3,   57,  4,   174, 3,   118, 175, 4,
          3,   118, 202, 175, 3,   4,   175, 173, 3,   218, 5,   6,   3,   6,
          5,   33,  3,   5,   7,   33,  3,   158, 7,   5,   3,   218, 158, 5,
          3,   131, 7,   12,  3,   33,  7,   131, 3,   136, 132, 138, 3,   138,
          46,  139, 3,   138, 132, 46,  3,   108, 35,  145, 3,   35,  36,  207,
          3,   36,  136, 207, 3,   29,  37,  36,  3,   110, 38,  37,  3,   110,
          37,  29,  3,   9,   144, 11,  3,   8,   52,  53,  3,   144, 8,   53,
          3,   144, 52,  8,   3,   143, 144, 9,   3,   143, 9,   11,  3,   123,
          10,  11,  3,   41,  10,  123, 3,   143, 11,  10,  3,   41,  143, 10,
          3,   40,  41,  123, 3,   210, 40,  121, 3,   55,  79,  44,  3,   204,
          114, 70,  3,   70,  72,  170, 3,   113, 14,  114, 3,   11,  53,  14,
          3,   125, 14,  113, 3,   123, 11,  14,  3,   123, 14,  125, 3,   135,
          49,  46,  3,   132, 135, 46,  3,   135, 12,  49,  3,   131, 12,  135,
          3,   13,  52,  144, 3,   13,  46,  52,  3,   139, 13,  144, 3,   139,
          46,  13,  3,   54,  55,  44,  3,   14,  53,  54,  3,   14,  54,  114,
          3,   51,  48,  77,  3,   55,  51,  79,  3,   51,  77,  79,  3,   242,
          58,  188, 3,   68,  61,  161, 3,   223, 197, 190, 3,   223, 226, 197,
          3,   67,  178, 177, 3,   25,  15,  156, 3,   15,  157, 156, 3,   78,
          157, 15,  3,   75,  15,  25,  3,   75,  78,  15,  3,   23,  20,  16,
          3,   20,  19,  62,  3,   20,  62,  61,  3,   16,  20,  61,  3,   17,
          18,  163, 3,   17,  163, 69,  3,   163, 18,  68,  3,   227, 168, 231,
          3,   170, 72,  171, 3,   204, 70,  203, 3,   167, 25,  169, 3,   169,
          25,  19,  3,   169, 20,  23,  3,   169, 19,  20,  3,   21,  23,  22,
          3,   21,  169, 23,  3,   168, 169, 21,  3,   24,  21,  22,  3,   231,
          168, 21,  3,   228, 21,  24,  3,   228, 231, 21,  3,   167, 74,  25,
          3,   74,  75,  25,  3,   44,  79,  76,  3,   44,  76,  72,  3,   85,
          82,  84,  3,   178, 180, 185, 3,   96,  148, 147, 3,   213, 147, 212,
          3,   95,  213, 248, 3,   95,  96,  147, 3,   95,  147, 213, 3,   97,
          91,  96,  3,   189, 92,  93,  3,   181, 183, 182, 3,   181, 98,  183,
          3,   178, 181, 180, 3,   93,  103, 101, 3,   103, 98,  181, 3,   164,
          89,  165, 3,   102, 89,  164, 3,   101, 102, 164, 3,   105, 93,  101,
          3,   189, 93,  105, 3,   105, 101, 164, 3,   198, 106, 152, 3,   198,
          112, 106, 3,   189, 196, 92,  3,   196, 198, 91,  3,   196, 91,  92,
          3,   193, 26,  192, 3,   192, 27,  111, 3,   192, 28,  27,  3,   192,
          26,  28,  3,   198, 192, 112, 3,   112, 110, 107, 3,   107, 29,  108,
          3,   107, 110, 29,  3,   243, 119, 30,  3,   260, 116, 119, 3,   260,
          119, 243, 3,   119, 118, 30,  3,   125, 113, 117, 3,   124, 125, 117,
          3,   254, 42,  262, 3,   254, 266, 42,  3,   42,  121, 261, 3,   262,
          42,  261, 3,   261, 126, 260, 3,   174, 127, 31,  3,   59,  31,  85,
          3,   57,  31,  59,  3,   57,  174, 31,  3,   31,  32,  85,  3,   31,
          127, 32,  3,   174, 238, 127, 3,   130, 33,  131, 3,   34,  130, 38,
          3,   34,  33,  130, 3,   136, 138, 140, 3,   145, 245, 244, 3,   150,
          108, 145, 3,   35,  207, 206, 3,   145, 35,  206, 3,   145, 206, 245,
          3,   134, 133, 36,  3,   37,  134, 36,  3,   38,  134, 37,  3,   38,
          130, 134, 3,   141, 142, 41,  3,   41,  142, 143, 3,   121, 40,  122,
          3,   122, 40,  123, 3,   39,  41,  40,  3,   210, 39,  40,  3,   211,
          39,  210, 3,   211, 141, 39,  3,   141, 41,  39,  3,   209, 210, 121,
          3,   209, 121, 42,  3,   266, 209, 42,  3,   114, 43,  70,  3,   114,
          54,  43,  3,   54,  44,  43,  3,   43,  44,  72,  3,   70,  43,  72,
          3,   52,  45,  51,  3,   46,  45,  52,  3,   45,  47,  48,  3,   45,
          48,  51,  3,   46,  49,  45,  3,   49,  47,  45,  3,   52,  51,  50,
          3,   52,  50,  53,  3,   53,  50,  54,  3,   50,  55,  54,  3,   50,
          51,  55,  3,   177, 179, 60,  3,   66,  177, 60,  3,   58,  57,  56,
          3,   188, 58,  56,  3,   56,  57,  59,  3,   56,  59,  66,  3,   60,
          56,  66,  3,   188, 56,  60,  3,   179, 239, 187, 3,   60,  179, 187,
          3,   187, 188, 60,  3,   61,  64,  161, 3,   161, 64,  153, 3,   155,
          63,  64,  3,   62,  64,  61,  3,   62,  155, 64,  3,   155, 216, 63,
          3,   216, 153, 64,  3,   63,  216, 64,  3,   161, 153, 160, 3,   160,
          193, 197, 3,   160, 153, 65,  3,   160, 65,  193, 3,   66,  86,  87,
          3,   66,  87,  177, 3,   87,  88,  67,  3,   67,  177, 87,  3,   163,
          68,  162, 3,   68,  161, 162, 3,   81,  69,  221, 3,   69,  163, 221,
          3,   165, 81,  221, 3,   227, 172, 168, 3,   171, 167, 172, 3,   70,
          170, 237, 3,   203, 70,  237, 3,   72,  76,  71,  3,   76,  74,  71,
          3,   72,  71,  171, 3,   171, 71,  167, 3,   71,  74,  167, 3,   73,
          75,  74,  3,   76,  73,  74,  3,   77,  78,  73,  3,   73,  78,  75,
          3,   79,  77,  73,  3,   79,  73,  76,  3,   80,  81,  165, 3,   84,
          80,  165, 3,   82,  81,  80,  3,   84,  82,  80,  3,   83,  84,  165,
          3,   86,  85,  83,  3,   85,  84,  83,  3,   87,  83,  88,  3,   86,
          83,  87,  3,   83,  89,  88,  3,   83,  165, 89,  3,   178, 185, 184,
          3,   179, 184, 239, 3,   98,  99,  183, 3,   183, 99,  248, 3,   99,
          95,  248, 3,   91,  97,  90,  3,   90,  97,  98,  3,   92,  91,  90,
          3,   90,  98,  103, 3,   92,  90,  93,  3,   93,  90,  103, 3,   94,
          96,  95,  3,   97,  96,  94,  3,   98,  97,  94,  3,   98,  94,  99,
          3,   94,  95,  99,  3,   101, 100, 102, 3,   103, 100, 101, 3,   100,
          178, 102, 3,   100, 181, 178, 3,   103, 181, 100, 3,   224, 104, 105,
          3,   104, 189, 105, 3,   224, 190, 104, 3,   104, 190, 189, 3,   225,
          224, 105, 3,   164, 225, 105, 3,   106, 149, 152, 3,   106, 112, 149,
          3,   112, 107, 149, 3,   149, 107, 108, 3,   149, 108, 150, 3,   197,
          193, 194, 3,   109, 111, 110, 3,   192, 111, 109, 3,   112, 109, 110,
          3,   192, 109, 112, 3,   117, 200, 201, 3,   117, 113, 200, 3,   200,
          114, 204, 3,   113, 114, 200, 3,   116, 124, 115, 3,   124, 117, 115,
          3,   115, 117, 201, 3,   115, 201, 118, 3,   119, 115, 118, 3,   116,
          115, 119, 3,   121, 120, 261, 3,   121, 122, 120, 3,   122, 123, 120,
          3,   120, 125, 124, 3,   126, 120, 124, 3,   261, 120, 126, 3,   120,
          123, 125, 3,   238, 230, 127, 3,   230, 229, 128, 3,   127, 230, 128,
          3,   130, 131, 129, 3,   133, 129, 132, 3,   134, 129, 133, 3,   134,
          130, 129, 3,   129, 135, 132, 3,   129, 131, 135, 3,   207, 136, 208,
          3,   136, 140, 208, 3,   208, 141, 211, 3,   208, 140, 141, 3,   137,
          138, 139, 3,   140, 138, 137, 3,   140, 137, 141, 3,   141, 137, 142,
          3,   137, 143, 142, 3,   137, 139, 144, 3,   137, 144, 143, 3,   150,
          145, 151, 3,   212, 151, 244, 3,   151, 145, 244, 3,   147, 151, 212,
          3,   148, 146, 147, 3,   149, 150, 146, 3,   147, 146, 151, 3,   146,
          150, 151, 3,   148, 152, 146, 3,   152, 149, 146, 3,   248, 213, 272,
          3,   269, 266, 254, 3,   253, 269, 254, 3,   271, 269, 253, 3,   215,
          154, 153, 3,   216, 215, 153, 3,   215, 218, 154, 3,   155, 217, 216,
          3,   156, 217, 155, 3,   156, 157, 217, 3,   217, 158, 218, 3,   157,
          158, 217, 3,   161, 160, 159, 3,   162, 159, 226, 3,   162, 161, 159,
          3,   226, 159, 197, 3,   159, 160, 197, 3,   220, 162, 226, 3,   220,
          163, 162, 3,   221, 163, 220, 3,   164, 165, 222, 3,   165, 221, 222,
          3,   164, 222, 225, 3,   172, 167, 166, 3,   172, 166, 168, 3,   166,
          167, 169, 3,   166, 169, 168, 3,   237, 170, 236, 3,   170, 171, 236,
          3,   236, 172, 227, 3,   236, 171, 172, 3,   175, 234, 173, 3,   173,
          234, 174, 3,   174, 234, 238, 3,   202, 203, 233, 3,   203, 237, 233,
          3,   202, 233, 175, 3,   175, 233, 234, 3,   178, 176, 177, 3,   178,
          184, 176, 3,   177, 176, 179, 3,   176, 184, 179, 3,   186, 182, 250,
          3,   180, 186, 185, 3,   181, 186, 180, 3,   181, 182, 186, 3,   182,
          183, 247, 3,   182, 247, 250, 3,   183, 248, 247, 3,   184, 240, 239,
          3,   184, 185, 240, 3,   185, 186, 240, 3,   186, 250, 240, 3,   239,
          241, 187, 3,   239, 257, 241, 3,   187, 241, 188, 3,   241, 242, 188,
          3,   254, 262, 259, 3,   190, 195, 189, 3,   189, 195, 196, 3,   190,
          197, 195, 3,   193, 192, 191, 3,   194, 193, 191, 3,   195, 191, 196,
          3,   197, 194, 191, 3,   197, 191, 195, 3,   191, 198, 196, 3,   191,
          192, 198, 3,   201, 200, 199, 3,   201, 199, 202, 3,   199, 203, 202,
          3,   199, 204, 203, 3,   200, 204, 199, 3,   206, 207, 205, 3,   207,
          208, 205, 3,   206, 205, 245, 3,   245, 205, 264, 3,   205, 211, 264,
          3,   205, 208, 211, 3,   263, 210, 209, 3,   266, 263, 209, 3,   263,
          211, 210, 3,   264, 211, 263, 3,   212, 244, 270, 3,   213, 212, 270,
          3,   213, 270, 272, 3,   249, 253, 252, 3,   249, 271, 253, 3,   250,
          249, 252, 3,   272, 271, 249, 3,   216, 214, 215, 3,   217, 214, 216,
          3,   215, 214, 218, 3,   217, 218, 214, 3,   221, 220, 219, 3,   222,
          221, 219, 3,   219, 223, 224, 3,   225, 219, 224, 3,   222, 219, 225,
          3,   219, 226, 223, 3,   219, 220, 226, 3,   236, 227, 235, 3,   229,
          235, 228, 3,   230, 235, 229, 3,   238, 235, 230, 3,   235, 231, 228,
          3,   235, 227, 231, 3,   233, 232, 234, 3,   232, 236, 235, 3,   237,
          236, 232, 3,   233, 237, 232, 3,   234, 232, 238, 3,   238, 232, 235,
          3,   239, 251, 257, 3,   240, 251, 239, 3,   250, 252, 251, 3,   240,
          250, 251, 3,   241, 256, 242, 3,   257, 256, 241, 3,   256, 260, 243,
          3,   256, 243, 242, 3,   244, 245, 265, 3,   270, 244, 265, 3,   265,
          245, 264, 3,   247, 248, 246, 3,   248, 272, 246, 3,   246, 272, 249,
          3,   247, 246, 250, 3,   250, 246, 249, 3,   251, 252, 258, 3,   251,
          258, 257, 3,   252, 253, 258, 3,   253, 254, 258, 3,   258, 254, 259,
          3,   257, 255, 256, 3,   258, 255, 257, 3,   258, 259, 255, 3,   255,
          260, 256, 3,   255, 261, 260, 3,   262, 261, 255, 3,   259, 262, 255,
          3,   268, 264, 263, 3,   265, 264, 268, 3,   268, 263, 266, 3,   269,
          268, 266, 3,   270, 265, 268, 3,   267, 268, 269, 3,   267, 270, 268,
          3,   267, 269, 271, 3,   272, 267, 271, 3,   272, 270, 267,
      }),
      false));

  Transform3<S> X_WC = Transform3<S>::Identity();
  // clang-format off
  X_WC.matrix() <<0.80702990712831657039, -0.041644652171421492337, -0.58904028041041645025, -0.079283200204372406006,
  0.57999939852431148246, 0.24327232818565469596, 0.77744406361523388238, 0.083579264581203460693,
  0.11092081279858001519, -0.96906361880149660681, 0.22048191763373692353, 0.028067275881767272949,
  0, 0, 0, 1;
  // clang-format on
  fcl::CollisionObject<S> convex(convex_geo, X_WC);

  CollisionGeometryPtr_t sphere_geo(
      new fcl::Sphere<S>(0.0020000000000000000416));
  Transform3<S> X_WS = Transform3<S>::Identity();
  // clang-format off
  X_WS.matrix() << -0.11540962399721466092, 0.74508239352308680559, -0.65691159645018415425, -0.07581716522176343287,
  -0.21483559984200240045, -0.66440074140450477991, -0.7158333045211513479, 0.039059250509023583919,
  -0.96980734361487930251, 0.05851394434707302139, 0.23674846269321175862, 0.023432879780773328143,
  0, 0, 0, 1;
  //clang-format on
  fcl::CollisionObject<S> sphere(sphere_geo, X_WS);


  fcl::DistanceRequest<S> request;
  request.gjk_solver_type = GJKSolverType::GST_LIBCCD;
  request.distance_tolerance = 1e-6;
  request.enable_signed_distance = true;
  fcl::DistanceResult<S> result;
  ASSERT_NO_THROW(fcl::distance(&convex, &sphere, request, result));
  EXPECT_NEAR(abs(result.min_distance),
              (result.nearest_points[0] - result.nearest_points[1]).norm(),
              request.distance_tolerance);
}

//==============================================================================

GTEST_TEST(FCL_NEGATIVE_DISTANCE, sphere_sphere_ccd) {
  test_distance_spheresphere<double>(GST_LIBCCD);
}

GTEST_TEST(FCL_NEGATIVE_DISTANCE, sphere_sphere_indep) {
  test_distance_spheresphere<double>(GST_INDEP);
}

GTEST_TEST(FCL_NEGATIVE_DISTANCE, sphere_capsule_ccd) {
  test_distance_spherecapsule<double>(GST_LIBCCD);
}

GTEST_TEST(FCL_NEGATIVE_DISTANCE, sphere_capsule_indep) {
  test_distance_spherecapsule<double>(GST_INDEP);
}

GTEST_TEST(FCL_SIGNED_DISTANCE, cylinder_sphere1_ccd) {
  test_distance_cylinder_sphere1<double>();
}

GTEST_TEST(FCL_SIGNED_DISTANCE, cylinder_box_ccd) {
  test_distance_cylinder_box<double>();
}


// A collection of scenarios observed in practice that have created error
// conditions in previous commits of the code. Each test is a unique instance.
GTEST_TEST(FCL_SIGNED_DISTANCE, RealWorldRegression1) {
  test_distance_box_box_regression1<double>();
}

GTEST_TEST(FCL_SIGNED_DISTANCE, RealWorldRegression2) {
  test_distance_box_box_regression2<double>();
}

GTEST_TEST(FCL_SIGNED_DISTANCE, RealWorldRegression3) {
  test_distance_box_box_regression3<double>();
}

GTEST_TEST(FCL_SIGNED_DISTANCE, RealWorldRegression4) {
  test_distance_box_box_regression4<double>();
}

GTEST_TEST(FCL_SIGNED_DISTANCE, RealWorldRegression5) {
  test_distance_box_box_regression5<double>();
}

GTEST_TEST(FCL_SIGNED_DISTANCE, RealWorldRegression6) {
  test_distance_box_box_regression6<double>();
}

GTEST_TEST(FCL_SIGNED_DISTANCE, RealWorldRegression7) {
  test_distance_box_box_regression7<double>();
}

GTEST_TEST(FCL_SIGNED_DISTANCE, RealWorldRegression8) {
  test_distance_box_box_regression_tilted_kissing_contact<double>();
}

GTEST_TEST(FCL_SIGNED_DISTANCE, RealWorldRegression9) {
  test_distance_sphere_box_regression1<double>();
}

GTEST_TEST(FCL_SIGNED_DISTANCE, RealWorldRegression10) {
  test_distance_convex_to_convex1<double>();
}

GTEST_TEST(FCL_SIGNED_DISTANCE, RealWorldRegression11) {
  test_distance_convex_to_sphere<double>();
}

GTEST_TEST(FCL_SIGNED_DISTANCE, RealWorldRegression12) {
  test_distance_convex_to_convex2<double>();
}

GTEST_TEST(FCL_SIGNED_DISTANCE, RealWorldRegression13) {
  test_distance_convex_to_convex3<double>();
}

GTEST_TEST(FCL_SIGNED_DISTANCE, RealWorldRegression14) {
  test_distance_convex_to_convex4<double>();
}

GTEST_TEST(FCL_SIGNED_DISTANCE, RealWorldRegression15) {
  test_distance_convex_to_convex5<double>();
}

GTEST_TEST(FCL_SIGNED_DISTANCE, RealWorldRegression16) {
  test_distance_convex_to_convex6<double>();
}

GTEST_TEST(FCL_SIGNED_DISTANCE, RealWorldRegression17) {
  test_distance_convex_to_convex7<double>();
}
//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

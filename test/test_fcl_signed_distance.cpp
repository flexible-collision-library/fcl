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
  test_distance_convex_to_convex2<double>();
}

GTEST_TEST(FCL_SIGNED_DISTANCE, RealWorldRegression12) {
  test_distance_convex_to_convex3<double>();
}

GTEST_TEST(FCL_SIGNED_DISTANCE, RealWorldRegression13) {
  test_distance_convex_to_convex4<double>();
}

GTEST_TEST(FCL_SIGNED_DISTANCE, RealWorldRegression14) {
  test_distance_convex_to_convex5<double>();
}

GTEST_TEST(FCL_SIGNED_DISTANCE, RealWorldRegression15) {
  test_distance_convex_to_convex6<double>();
}

GTEST_TEST(FCL_SIGNED_DISTANCE, RealWorldRegression16) {
  test_distance_convex_to_convex7<double>();
}
//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

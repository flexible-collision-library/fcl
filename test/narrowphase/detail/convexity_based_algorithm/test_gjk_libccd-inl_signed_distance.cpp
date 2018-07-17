/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018. Toyota Research Institute
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
 *   * Neither the name of CNRS-LAAS and AIST nor the names of its
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

/** @author Hongkai Dai */
/**
 * Test the signed distance query between two convex objects, when calling with
 * solver = GST_LIBCCD.
 */
#include "fcl/narrowphase/detail/convexity_based_algorithm/gjk_libccd-inl.h"

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "fcl/narrowphase/detail/gjk_solver_libccd.h"

namespace fcl {
namespace detail {
// Given two spheres, sphere 1 has radius1, and centered at point A, whose
// position is p_F1 measured and expressed in frame F; sphere 2 has radius2,
// and centered at point B, whose position is p_F2 measured and expressed in
// frame F. Computes the signed distance between the two spheres.
// We use the monogram notation on spatial vectors. The monogram notation is
// explained in
// http://drake.mit.edu/doxygen_cxx/group__multibody__spatial__pose.html
template <typename S>
S ComputeSphereSphereDistance(S radius1, S radius2, const Vector3<S>& p_F1,
                              const Vector3<S>& p_F2) {
  S min_distance = (p_F1 - p_F2).norm() - radius1 - radius2;
  return min_distance;
}

template <typename S>
void TestSphereToSphereGJKSignedDistance(S radius1, S radius2,
                                         const Vector3<S>& p_F1,
                                         const Vector3<S>& p_F2,
                                         S solver_tolerance, S test_tol,
                                         S min_distance_expected) {
  // Test if GJKSignedDistance computes the right distance. Here we used sphere
  // to sphere as the geometries. The distance between sphere and sphere should
  // be computed using distance between primitives, instead of the GJK
  // algorithm. But here we choose spheres for simplicity.
  //
  // There are two tolerances: the solver_tolerance, and the test_tol.
  // solver_tolerance is used to determine when the algorithm terminates. If
  // the objects are separated, the GJK algorithm terminates when the change of
  // distance is below solver_tolerance, which does NOT mean that the separation
  // distance computed by GJK is within solver_tolerance to the true distance,
  // so we use test_tol as a separate tolerance to check the accuracy of the
  // computed distance.
  // If the objects penetrate, the EPA algorithm terminates when the difference
  // between the upper bound of penetration depth and the lower bound
  // is below solver_tolerance, which means that the EPA computed penetration
  // depth is within solver_tolerance to the true depth.
  // The tolerance is explained in GJKSignedDistance() in gjk_libccd-inl.h
  fcl::Sphere<S> s1(radius1);
  fcl::Sphere<S> s2(radius2);
  fcl::Transform3<S> tf1, tf2;
  tf1.setIdentity();
  tf2.setIdentity();
  tf1.translation() = p_F1;
  tf2.translation() = p_F2;
  void* o1 = GJKInitializer<S, fcl::Sphere<S>>::createGJKObject(s1, tf1);
  void* o2 = GJKInitializer<S, fcl::Sphere<S>>::createGJKObject(s2, tf2);

  S dist;
  // N1 and N2 are the witness points on sphere 1 and 2 respectively.
  Vector3<S> p_FN1, p_FN2;
  GJKSolver_libccd<S> gjkSolver;
  gjkSolver.distance_tolerance = solver_tolerance;
  bool res = GJKSignedDistance(
      o1, detail::GJKInitializer<S, Sphere<S>>::getSupportFunction(), o2,
      detail::GJKInitializer<S, Sphere<S>>::getSupportFunction(),
      gjkSolver.max_distance_iterations, gjkSolver.distance_tolerance, &dist,
      &p_FN1, &p_FN2);

  EXPECT_EQ(res, min_distance_expected >= 0);

  EXPECT_NEAR(dist, min_distance_expected, test_tol);
  // Now check if the distance between N1 and N2 matches with dist, they should
  // match independent of what solver_tolerance we choose.
  EXPECT_NEAR((p_FN1 - p_FN2).norm(), std::abs(dist),
              fcl::constants<S>::eps_78());
  // Check if p1 is on the boundary of sphere 1, and p2 is on the boundary of
  // sphere 2.
  EXPECT_NEAR((p_FN1 - p_F1).norm(), radius1, test_tol);
  EXPECT_NEAR((p_FN2 - p_F2).norm(), radius2, test_tol);
  // The witness points N1 and N2 are defined as by shifting geometry B with
  // the vector N1 - N2, the shifted geometry B' and A are touching. Hence
  // if we shift sphere 1 by p_FN2 - p_FN1, then the two spheres should be
  // touching. The shifted sphere is centered at p_F1 + p_FN2 - p_FN1.
  EXPECT_NEAR((p_F1 + p_FN2 - p_FN1 - p_F2).norm(), radius1 + radius2,
              test_tol);
  // Note that we do not check the computed witness points to the true witness
  // points. There are two reasons
  // 1. Generally, the witness points are NOT guaranteed to be unique (consider
  // plane-to-plane contact).
  // 2. Even if there are unique witness points, it is hard to infer the bounds
  // on the computed witness points, based on the tolerance of the solver. This
  // bounds depend on the curvature of the geometries near the contact region,
  // and we do not have a general approach to compute the bound for generic
  // geometries.
  // On the other hand, for sphere-sphere contact, it is possible to compute
  // the bounds, since the witness points are unique, and the curvature is
  // constant. For the moment, we are satisfied with the test above. If the
  // future maintainer wants to improve this test, he/she might compute these
  // bounds for the sphere-sphere case.

  GJKInitializer<S, fcl::Sphere<S>>::deleteGJKObject(o1);
  GJKInitializer<S, fcl::Sphere<S>>::deleteGJKObject(o2);
}

template <typename S>
struct SphereSpecification {
  SphereSpecification<S>(S radius_, const Vector3<S>& center_)
      : radius{radius_}, center{center_} {}
  S radius;
  Vector3<S> center;
};

template <typename S>
void TestNonCollidingSphereGJKSignedDistance() {
  std::vector<SphereSpecification<S>> spheres;
  spheres.emplace_back(0.5, Vector3<S>(0, 0, -1.2));
  spheres.emplace_back(0.5, Vector3<S>(1.25, 0, 0));
  spheres.emplace_back(0.3, Vector3<S>(-0.2, 0, 0));
  spheres.emplace_back(0.4, Vector3<S>(-0.2, 0, 1.1));
  for (int i = 0; i < static_cast<int>(spheres.size()); ++i) {
    for (int j = i + 1; j < static_cast<int>(spheres.size()); ++j) {
      if ((spheres[i].center - spheres[j].center).norm() >
          spheres[i].radius + spheres[j].radius) {
        // Not in collision.
        for (const S solver_tolerance : {S(1e-4), S(1e-5), S(1e-6)}) {
          const S min_distance_expected =
              ComputeSphereSphereDistance(spheres[i].radius, spheres[j].radius,
                                          spheres[i].center, spheres[j].center);
          // When the change of distance is below solver_tolerances[k], it does
          // not mean that the error in separating distance is below
          // solver_tolerances[k]. Empirically we find the error is less than 10
          // * solver_tolerances[k], but there is no proof.
          TestSphereToSphereGJKSignedDistance<S>(
              spheres[i].radius, spheres[j].radius, spheres[i].center,
              spheres[j].center, solver_tolerance, 10 * solver_tolerance,
              min_distance_expected);
        }
      } else {
        GTEST_FAIL() << "The two spheres collide."
                     << "\nSpheres[" << i << "] with radius "
                     << spheres[i].radius << ", centered at "
                     << spheres[i].center.transpose() << "\nSpheres[" << j
                     << "] with radius " << spheres[j].radius
                     << ", centered at " << spheres[j].center.transpose()
                     << "\n";
      }
    }
  }
}

template <typename S>
void TestCollidingSphereGJKSignedDistance() {
  std::vector<SphereSpecification<S>> spheres;
  spheres.emplace_back(0.5, Vector3<S>(0, 0, 0));
  spheres.emplace_back(0.5, Vector3<S>(0.75, 0, 0));
  spheres.emplace_back(0.3, Vector3<S>(0.2, 0, 0));
  spheres.emplace_back(0.4, Vector3<S>(0.2, 0, 0.4));
  for (int i = 0; i < static_cast<int>(spheres.size()); ++i) {
    for (int j = i + 1; j < static_cast<int>(spheres.size()); ++j) {
      if ((spheres[i].center - spheres[j].center).norm() <
          spheres[i].radius + spheres[j].radius) {
        // colliding
        const S min_distance_expected =
            ComputeSphereSphereDistance(spheres[i].radius, spheres[j].radius,
                                        spheres[i].center, spheres[j].center);
        for (const S solver_tolerance : {S(1E-4), S(1E-5), S(1E-6)}) {
          // For colliding spheres, the solver_tolerance is the bound on the
          // error relative to the true answer.
          TestSphereToSphereGJKSignedDistance<S>(
              spheres[i].radius, spheres[j].radius, spheres[i].center,
              spheres[j].center, solver_tolerance, solver_tolerance,
              min_distance_expected);
        }
      } else {
        GTEST_FAIL() << "The two spheres failed to collide."
                     << "\nSpheres[" << i << "] with radius "
                     << spheres[i].radius << ", centered at "
                     << spheres[i].center.transpose() << "\nSpheres[" << j
                     << "] with radius " << spheres[j].radius
                     << ", centered at " << spheres[j].center.transpose()
                     << "\n";
      }
    }
  }
}

GTEST_TEST(FCL_GJKSignedDistance, sphere_sphere) {
  TestNonCollidingSphereGJKSignedDistance<double>();
  TestNonCollidingSphereGJKSignedDistance<float>();
  TestCollidingSphereGJKSignedDistance<double>();
  TestCollidingSphereGJKSignedDistance<float>();
}

//----------------------------------------------------------------------------
//                 Box test
// Given two boxes, we can perturb the pose of one box so the boxes are
// penetrating, touching or separated.
template <typename S>
struct BoxSpecification {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BoxSpecification(const fcl::Vector3<S>& m_size) : size(m_size) {
    X_FB.setIdentity();
  }
  fcl::Vector3<S> size;
  fcl::Transform3<S> X_FB;
};

template <typename S>
void TestBoxesInFrameF(const Transform3<S>& X_WF) {
  const fcl::Vector3<S> box1_size(1, 1, 1);
  const fcl::Vector3<S> box2_size(0.6, 0.8, 1);
  // Put the two boxes on the xy plane of frame F.
  // B1 is the frame rigidly attached to box 1, B2 is the frame rigidly
  // attached to box 2. W is the world frame. F is a frame fixed to the world.
  // X_FB1 is the pose of box 1 expressed and measured in frame F, X_FB2 is the
  // pose of box 2 expressed and measured in frame F.
  fcl::Transform3<S> X_FB1, X_FB2;
  // Box 1 is fixed.
  X_FB1.setIdentity();
  X_FB1.translation() << 0, 0, 0.5;

  // First fix the orientation of box 2, such that one of its diagonal (the
  // one connecting the vertex (0.3, -0.4, 1) and (-0.3, 0.4, 1) is parallel to
  // the x axis in frame F. If we move the position of box 2, we get different
  // signed distance.
  X_FB2.setIdentity();
  X_FB2.linear() << 0.6, -0.8, 0, 0.8, 0.6, 0, 0, 0, 1;

  // p_xy_FN1 is the xy position of point N1 (the deepest penetration point on
  // box 1) measured and expressed in the frame F.
  // p_xy_FN2 is the xy position of point N2 (the deepest penetration point on
  // box 2) measured and expressed in the frame F.
  auto CheckDistance = [&box1_size, &box2_size, &X_FB1, &X_WF](
      const Transform3<S>& X_FB2, S distance_expected,
      const Vector2<S>& p_xy_FN1_expected, const Vector2<S>& p_xy_FN2_expected,
      S solver_distance_tolerance, S test_distance_tolerance,
      S test_witness_tolerance) {
    const fcl::Transform3<S> X_WB1 = X_WF * X_FB1;
    const fcl::Transform3<S> X_WB2 = X_WF * X_FB2;
    fcl::Box<S> box1(box1_size);
    fcl::Box<S> box2(box2_size);
    void* o1 = GJKInitializer<S, fcl::Box<S>>::createGJKObject(box1, X_WB1);
    void* o2 = GJKInitializer<S, fcl::Box<S>>::createGJKObject(box2, X_WB2);
    S dist;
    Vector3<S> p_WN1, p_WN2;
    GJKSolver_libccd<S> gjkSolver;
    gjkSolver.distance_tolerance = solver_distance_tolerance;
    bool res = GJKSignedDistance(
        o1, detail::GJKInitializer<S, Box<S>>::getSupportFunction(), o2,
        detail::GJKInitializer<S, Box<S>>::getSupportFunction(),
        gjkSolver.max_distance_iterations, gjkSolver.distance_tolerance, &dist,
        &p_WN1, &p_WN2);

    // It is unclear how FCL handles touching contact. It could return either
    // true or false for touching contact. So, we ignore the condition where
    // expected distance is zero.
    if (distance_expected < 0) {
      EXPECT_FALSE(res);
    } else if (distance_expected > 0) {
      EXPECT_TRUE(res);
    }

    EXPECT_NEAR(dist, distance_expected, test_distance_tolerance);
    const Vector3<S> p_FN1 =
        X_WF.linear().transpose() * (p_WN1 - X_WF.translation());
    const Vector3<S> p_FN2 =
        X_WF.linear().transpose() * (p_WN2 - X_WF.translation());

    EXPECT_TRUE(p_FN1.template head<2>().isApprox(p_xy_FN1_expected,
                                                  test_witness_tolerance));
    EXPECT_TRUE(p_FN2.template head<2>().isApprox(p_xy_FN2_expected,
                                                  test_witness_tolerance));
    // The z height of the closest points should be the same.
    EXPECT_NEAR(p_FN1(2), p_FN2(2), test_witness_tolerance);
    // The closest point is within object A/B, so the z height should be
    // within [0, 1]
    EXPECT_GE(p_FN1(2), 0);
    EXPECT_GE(p_FN2(2), 0);
    EXPECT_LE(p_FN1(2), 1);
    EXPECT_LE(p_FN2(2), 1);

    GJKInitializer<S, fcl::Sphere<S>>::deleteGJKObject(o1);
    GJKInitializer<S, fcl::Sphere<S>>::deleteGJKObject(o2);
  };

  auto CheckBoxEdgeBoxFaceDistance = [&CheckDistance](
      const Transform3<S>& X_FB2, S solver_distance_tolerance,
      S test_distance_tolerance, S test_witness_tolerance) {
    const double expected_distance = -X_FB2.translation()(0) - 1;
    CheckDistance(
        X_FB2, expected_distance, Vector2<S>(-0.5, X_FB2.translation()(1)),
        Vector2<S>(X_FB2.translation()(0) + 0.5, X_FB2.translation()(1)),
        solver_distance_tolerance, test_distance_tolerance,
        test_witness_tolerance);
  };

  //---------------------------------------------------------------
  //                      Touching contact
  // Test with different solver distance tolerances.
  std::vector<S> solver_tolerances = {S(1E-4), fcl::constants<S>::eps_12(),
                                      fcl::constants<S>::eps_34(),
                                      fcl::constants<S>::eps_78()};
  for (int i = 0; i < static_cast<int>(solver_tolerances.size()); ++i) {
    const S solver_distance_tolerance = solver_tolerances[i];
    // For touching contact, FCL might call either GJK or EPA algorithm. When it
    // calls GJK algorithm, there is no theoretical guarantee, on how the
    // distance error is related to solver's distance_tolerance.
    // Empirically we find 10x is reasonable.
    const S test_distance_tolerance = 10 * solver_distance_tolerance;
    const S test_witness_tolerance = test_distance_tolerance;
    // An edge of box 2 is touching a face of box 1
    X_FB2.translation() << -1, 0, 0.5;
    CheckBoxEdgeBoxFaceDistance(X_FB2, solver_distance_tolerance,
                                test_distance_tolerance,
                                test_witness_tolerance);

    // The touching face on box 1 is parallel to the y axis, so shifting box 2
    // on y axis still gives touching contact. Shift box 2 on y axis by 0.1m.
    X_FB2.translation() << -1, 0.1, 0.5;
    CheckBoxEdgeBoxFaceDistance(X_FB2, solver_distance_tolerance,
                                test_distance_tolerance,
                                test_witness_tolerance);

    // Shift box 2 on y axis by -0.1m.
    X_FB2.translation() << -1, -0.1, 0.5;
    CheckBoxEdgeBoxFaceDistance(X_FB2, solver_distance_tolerance,
                                test_distance_tolerance,
                                test_witness_tolerance);
    // TODO(hongkai.dai@tri.global): Add other touching contact cases, including
    // face-face, face-vertex, edge-edge, edge-vertex and vertex-vertex.
  }

  //--------------------------------------------------------------
  //                      Penetrating contact
  // An edge of box 2 penetrates into a face of box 1
  for (int i = 0; i < static_cast<int>(solver_tolerances.size()); ++i) {
    const S solver_distance_tolerance = solver_tolerances[i];
    // For penetrating contact, FCL calls EPA algorithm. When the solver
    // terminates, the computed distance should be within
    // solver.distance_tolerance to the actual distance.
    const S test_distance_tolerance = solver_distance_tolerance;
    const S test_witness_tolerance = test_distance_tolerance;

    X_FB2.translation() << -0.9, 0, 0.5;
    CheckBoxEdgeBoxFaceDistance(X_FB2, solver_distance_tolerance,
                                test_distance_tolerance,
                                test_witness_tolerance);

    // Shift box 2 on y axis by 0.1m.
    X_FB2.translation() << -0.9, 0.1, 0.5;
    CheckBoxEdgeBoxFaceDistance(X_FB2, solver_distance_tolerance,
                                test_distance_tolerance,
                                test_witness_tolerance);

    // Shift box 2 on y axis by -0.05m.
    X_FB2.translation() << -0.9, -0.05, 0.5;
    CheckBoxEdgeBoxFaceDistance(X_FB2, solver_distance_tolerance,
                                test_distance_tolerance,
                                test_witness_tolerance);

    // Shift box 2 on y axis by -0.1m.
    X_FB2.translation() << -0.9, -0.1, 0.5;
    CheckBoxEdgeBoxFaceDistance(X_FB2, solver_distance_tolerance,
                                test_distance_tolerance,
                                test_witness_tolerance);
  }
}

template <typename S>
void TestBoxes() {
  // Frame F coincides with the world frame W.
  Transform3<S> X_WF;
  X_WF.setIdentity();
  TestBoxesInFrameF(X_WF);

  // Frame F is shifted from the world frame W.
  X_WF.translation() << 0, 0, 1;
  TestBoxesInFrameF(X_WF);

  X_WF.translation() << 0, 1, 0;
  TestBoxesInFrameF(X_WF);

  X_WF.translation() << 1, 0, 0;
  TestBoxesInFrameF(X_WF);

  // Frame F is rotated from the world frame W.
  X_WF.setIdentity();
  const S kPi = fcl::constants<S>::pi();
  X_WF.linear() =
      Eigen::AngleAxis<S>(0.1 * kPi, Vector3<S>::UnitZ()).toRotationMatrix();
  TestBoxesInFrameF(X_WF);

  // TODO(hongkai.dai): This test exposes an error in simplexToPolytope4, that
  // the initial simplex can be degenerate. Should add the special case on
  // degenerate simplex in simplexToPolytope4.
  /*X_WF.translation() << 0, 0, 0;
  X_WF.linear() =
      Eigen::AngleAxis<S>(0.1 * kPi, Vector3<S>(1.0 / 3, 2.0 / 3, -2.0 / 3))
          .toRotationMatrix();
  TestBoxesInFrameF(X_WF);*/

  // Frame F is rotated and shifted from the world frame W.
  X_WF.translation() << 0.1, 0.2, 0.3;
  TestBoxesInFrameF(X_WF);
}

GTEST_TEST(FCL_GJKSignedDistance, box_box) {
  TestBoxes<double>();
  TestBoxes<float>();
}
}  // namespace detail
}  // namespace fcl

//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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

/** @author Sean Curtis <sean@tri.global> (2018) */

#include <memory>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "eigen_matrix_compare.h"
#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/distance.h"

// TODO(SeanCurtis-TRI): Modify this test so it can be re-used for the distance
// query -- such that the sphere is slightly separated instead of slightly
// penetrating.  See test_sphere_cylinder.cpp for an example.

// This collides a cylinder with a sphere. The cylinder is disk-like with a
// large radius (r_c) and small height (h_c) and its geometric frame is aligned
// with the world frame. The sphere has radius r_s and is positioned at
// (sx, sy, sz) with an identity orientation. In this configuration, the sphere
// penetrates the cylinder slightly on the top face near the edge. The contact
// is *fully* contained in that face. (As illustrated below.)
//
// Side view
//                     z   small sphere
//                     ┆   ↓
//        ┏━━━━━━━━━━━━┿━━━━◯━━━━━━┓      ┬
//  ┄┄┄┄┄┄╂┄┄┄┄┄┄┄┄┄┄┄┄┼┄┄┄┄┄┄┄┄┄┄┄╂┄ x  h_c
//        ┗━━━━━━━━━━━━┿━━━━━━━━━━━┛      ┴
//                     ┆
//
//                     ├──── r_c───┤
//
// Top view
//                     y
//                     ┆
//                  *******     small sphere    ┬
//               **    ┆    **╱                 │
//             *       ┆     ◯ *                │
//            *        ┆        *               │
//           *         ┆         *             r_c
//           *         ┆         *              │
//          *          ┆          *             │
//          *          ┆          *             │
//   ┄┄┄┄┄┄┄*┄┄┄┄┄┄┄┄┄┄┼┄┄┄┄┄┄┄┄┄┄*┄┄┄┄  x      ┴
//          *          ┆          *
//          *          ┆          *
//           *         ┆         *
//           *         ┆         *
//            *        ┆        *
//             *       ┆       *
//               **    ┆    **
//                  *******
//                     ┆
// Properties of expected outcome:
//  - One contact *should* be reported,
//  - Penetration depth δ should be: r_s - (sz - h_c / 2),
//  - Contact point should be at: [sx, sy, h_c / 2 - δ / 2], and
//  - Contact normal should be [0, 0, -1] (pointing from sphere into cylinder).
//
// NOTE: Orientation of the sphere should *not* make a difference and is not
// tested here; it relies on the sphere-cylinder primitive algorithm unit tests
// to have robustly tested that.
//
// This test *fails* if GJK is used to evaluate the collision. It passes if the
// custom sphere-cylinder algorithm is used, and, therefore, its purpose is to
// confirm that the custom algorithm is being applied. It doesn't exhaustively
// test sphere-cylinder collision. It merely confirms the tested primitive
// algorithm has been wired up correctly.
template <typename S>
void LargeCylinderSmallSphereTest(fcl::GJKSolverType solver_type) {
  using fcl::Vector3;
  using Real = typename fcl::constants<S>::Real;
  const Real eps = fcl::constants<S>::eps();

  // Configure geometry.

  // Cylinder and sphere dimensions.
  const Real r_c = 9;
  const Real h_c = 0.0025;
  const Real r_s = 0.0015;

  auto sphere_geometry = std::make_shared<fcl::Sphere<S>>(r_s);
  auto cylinder_geometry = std::make_shared<fcl::Cylinder<S>>(r_c, h_c);

  // Pose of the cylinder in the world frame.
  const fcl::Transform3<S> X_WC = fcl::Transform3<S>::Identity();

  // Compute multiple sphere locations. All at the same height to produce a
  // fixed, expected penetration depth of half of its radius. The reported
  // position of the contact will have the x- and y- values of the sphere
  // center, but be half the target_depth below the +z face, i.e.,
  //  pz = (h_c / 2) - (target_depth / 2)
  const Real target_depth = r_s * 0.5;
  // Sphere center's height (position in z).
  const Real sz = h_c / 2 + r_s - target_depth;
  const Real pz = h_c / 2 - target_depth / 2;
  // This transform will get repeatedly modified in the nested for loop below.
  fcl::Transform3<S> X_WS = fcl::Transform3<S>::Identity();

  fcl::CollisionObject<S> sphere(sphere_geometry, X_WS);
  fcl::CollisionObject<S> cylinder(cylinder_geometry, X_WC);

  // Expected results. (Expected position is defined inside the for loop as it
  // changes with each new position of the sphere.)
  const S expected_depth = target_depth;
  // This normal direction assumes the *sphere* is the first collision object.
  // If the order is reversed, the normal must be likewise reversed.
  const Vector3<S> expected_normal = -Vector3<S>::UnitZ();

  // Set up the collision request.
  fcl::CollisionRequest<S> collision_request(1 /* num contacts */,
                                             true /* enable_contact */);
  collision_request.gjk_solver_type = solver_type;

  // Sample around the surface of the +z face on the disk for a fixed
  // penetration depth. Note: the +z face is a disk with radius r_c. Notes on
  // the selected samples:
  //  - We've picked positions such that the *whole* sphere projects onto the
  //    +z face. This *guarantees* that the contact is completely contained in
  //    the +z face so there is no possible ambiguity in the results.
  //  - We've picked points out near the boundaries, in the middle, and *near*
  //    zero without being zero. The GJK algorithm can actually provide the
  //    correct result at the (eps, eps) sample points. We leave those sample
  //    points in to confirm no degradation.
  const std::vector<Real> r_values{0, eps, r_c / 2, r_c - r_s};
  const S pi = fcl::constants<S>::pi();
  const std::vector<Real> theta_values{0, pi/2, pi, 3 * pi / 4};

  for (const Real r : r_values) {
    for (const Real theta : theta_values ) {
      // Don't just evaluate at nice, axis-aligned directions. Pick some number
      // that can't be perfectly represented.
      const Real angle = theta + pi / 7;
      const Real sx = cos(angle) * r;
      const Real sy = sin(angle) * r;
      // Repose the sphere.
      X_WS.translation() << sx, sy, sz;
      sphere.setTransform(X_WS);

      auto evaluate_collision = [&collision_request, &X_WS](
          const fcl::CollisionObject<S>* s1, const fcl::CollisionObject<S>* s2,
          S expected_depth, const Vector3<S>& expected_normal,
          const Vector3<S>& expected_pos, Real eps) {
        // Compute collision.
        fcl::CollisionResult<S> collision_result;
        std::size_t contact_count =
            fcl::collide(s1, s2, collision_request, collision_result);

        // Test answers
        if (contact_count == collision_request.num_max_contacts) {
          std::vector<fcl::Contact<S>> contacts;
          collision_result.getContacts(contacts);
          GTEST_ASSERT_EQ(contacts.size(), collision_request.num_max_contacts);

          const fcl::Contact<S>& contact = contacts[0];
          EXPECT_NEAR(contact.penetration_depth, expected_depth, eps)
                    << "Sphere at: " << X_WS.translation().transpose();
          EXPECT_TRUE(fcl::CompareMatrices(contact.normal,
                                           expected_normal,
                                           eps,
                                           fcl::MatrixCompareType::absolute))
                    << "Sphere at: " << X_WS.translation().transpose();
          EXPECT_TRUE(fcl::CompareMatrices(
              contact.pos, expected_pos, eps, fcl::MatrixCompareType::absolute))
                    << "Sphere at: " << X_WS.translation().transpose();
        } else {
          EXPECT_TRUE(false) << "No contacts reported for sphere at: "
                             << X_WS.translation().transpose();
        }
      };

      Vector3<S> expected_pos{sx, sy, pz};
      evaluate_collision(&sphere, &cylinder, expected_depth, expected_normal,
                         expected_pos, eps);
      evaluate_collision(&cylinder, &sphere, expected_depth, -expected_normal,
                         expected_pos, eps);
    }
  }
}

GTEST_TEST(FCL_SPHERE_CYLINDER, LargCylinderSmallSphere_ccd) {
  LargeCylinderSmallSphereTest<double>(fcl::GJKSolverType::GST_LIBCCD);
  LargeCylinderSmallSphereTest<float>(fcl::GJKSolverType::GST_LIBCCD);
}

GTEST_TEST(FCL_SPHERE_CYLINDER, LargBoxSmallSphere_indep) {
  LargeCylinderSmallSphereTest<double>(fcl::GJKSolverType::GST_INDEP);
  LargeCylinderSmallSphereTest<float>(fcl::GJKSolverType::GST_INDEP);
}

//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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

/** @author Sean Curtis */

#include <functional>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "fcl/math/constants.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_object.h"

using std::map;
using std::pair;
using std::string;
using std::vector;

// Simple specification for defining a box collision object. Specifies the
// dimensions and pose of the box in some frame F (X_FB). For an explanation
// of the notation X_FB, see:
// http://drake.mit.edu/doxygen_cxx/group__multibody__spatial__pose.html
template <typename S> struct BoxSpecification {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  fcl::Vector3<S> size;
  fcl::Transform3<S> X_FB;
};

// Class for executing and evaluating various box-box tests.
// The class is initialized with two box specifications (size and pose).
// The test performs a collision test between the two boxes in 12 permutations:
// One axis is the order (box 1 vs box 2 and box 2 vs box 1).
// The other axis is the orientation box 2. Given that box 2 must be a cube, it
// can be oriented in six different configurations and still produced the same
// answer.
// The 12 permutations are the two orderings crossed with the six orientations.
template <typename S>
class BoxBoxTest {
 public:
   // Construct the test scenario with the given box specifications. Box 2
   // must be a *cube* (all sides equal).
   BoxBoxTest(const BoxSpecification<S>& box_spec_1,
              const BoxSpecification<S>& box_spec_2)
       : box_spec_1_(box_spec_1), box_spec_2_(box_spec_2) {
     using fcl::AngleAxis;
     using fcl::Transform3;
     using fcl::Vector3;

     // Confirm box 2 is a cube
     EXPECT_EQ(box_spec_2.size(0), box_spec_2.size(1));
     EXPECT_EQ(box_spec_2.size(0), box_spec_2.size(2));

     const S pi = fcl::constants<S>::pi();

     // Initialize isomorphic rotations of box 2.
     iso_poses_["top"] = Transform3<S>::Identity();
     iso_poses_["bottom"] =
         Transform3<S>{AngleAxis<S>(pi, Vector3<S>::UnitX())};
     iso_poses_["back"] =
         Transform3<S>{AngleAxis<S>(pi / 2, Vector3<S>::UnitX())};
     iso_poses_["front"] =
         Transform3<S>{AngleAxis<S>(3 * pi / 2, Vector3<S>::UnitX())};
     iso_poses_["left"] =
         Transform3<S>{AngleAxis<S>(pi / 2, Vector3<S>::UnitY())};
     iso_poses_["right"] =
         Transform3<S>{AngleAxis<S>(3 * pi / 2, Vector3<S>::UnitY())};
  }

  // Runs the 12 tests for the two specified boxes.
  //
  // @param solver_type       The solver type to use for computing collision.
  // @param test_tolerance    The tolerance to which the collision contact
  //                          results will be compared to the results.
  // @param expected_normal   The expected normal for the (1, 2) query order.
  // @param expected_depth    The expected penetration depth.
  // @param contact_pos_test  A function to evaluate the reported contact
  //                          position for validity; this should be written to
  //                          account for the possibility of the contact
  //                          position lying on some manifold (e.g., an edge, or
  //                          face). This function should invoke googletest
  //                          EXPECT_* methods.
  // @param origin_name       A string which is appended to error message to
  //                          more easily parse failures and which test failed.
  void
  RunTests(fcl::GJKSolverType solver_type, S test_tolerance,
           const fcl::Vector3<S>& expected_normal, S expected_depth,
           std::function<void(const fcl::Vector3<S> &, S, const std::string &)>
               contact_pos_test,
           const std::string& origin_name) {
    fcl::Contact<S> expected_contact;
    expected_contact.penetration_depth = expected_depth;

    for (const auto& reorient_pair : iso_poses_) {
      const std::string& top_face = reorient_pair.first;
      const fcl::Transform3<S>& pre_pose = reorient_pair.second;

      BoxSpecification<S> box_2_posed{
          box_spec_2_.size,
          box_spec_2_.X_FB * pre_pose
      };

      // Collide (1, 2)
      expected_contact.normal = expected_normal;
      RunSingleTest(box_spec_1_,
                    box_2_posed,
                    solver_type,
                    test_tolerance,
                    expected_contact,
                    contact_pos_test,
                    origin_name + " (1, 2) - " + top_face);

      // Collide (2, 1)
      expected_contact.normal = -expected_normal;
      RunSingleTest(box_2_posed,
                    box_spec_1_,
                    solver_type,
                    test_tolerance,
                    expected_contact,
                    contact_pos_test,
                    origin_name + " (2, 1) - " + top_face);
    }
  }

private:
// Performs a collision test between two boxes and tests the *single* contact
// result against given expectations.
//
// @param box_spec_A        A specification of the first box (treated as object
//                          1 in the query).
// @param box_spec_B        A specification of the second box (treated as object
//                          2 in the query).
// @param solver_type       The solver type to use for computing collision.
// @param test_tolerance    The tolerance to which the collision contact results
//                          will be compared to the results.
// @param expected_contact  The expected contact details (only penetration depth
//                          and normal are used).
// @param contact_pos_test  A function to evaluate the reported contact position
//                          for validity; this should be written to account for
//                          the possibility of the contact position lying on
//                          some manifold (e.g., an edge, or face). This
//                          function should invoke googletest EXPECT_* methods.
// @param origin_name       A string which is appended to error message to more
//                          easily parse failures and which test failed.
  void RunSingleTest(
      const BoxSpecification<S>& box_spec_A,
      const BoxSpecification<S>& box_spec_B, fcl::GJKSolverType solver_type,
      S test_tolerance, const fcl::Contact<S>& expected_contact,
      std::function<void(const fcl::Vector3<S>&, S, const std::string&)>
      contact_pos_test,
      const std::string& origin_name) {
    using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<S>>;
    CollisionGeometryPtr_t box_geometry_A(new fcl::Box<S>(box_spec_A.size));
    CollisionGeometryPtr_t box_geometry_B(new fcl::Box<S>(box_spec_B.size));

    fcl::CollisionObject<S> box_A(box_geometry_A, box_spec_A.X_FB);
    fcl::CollisionObject<S> box_B(box_geometry_B, box_spec_B.X_FB);

    // Compute collision - single contact and enable contact.
    fcl::CollisionRequest<S> collisionRequest(1, true);
    collisionRequest.gjk_solver_type = solver_type;
    fcl::CollisionResult<S> collisionResult;
    fcl::collide(&box_A, &box_B, collisionRequest, collisionResult);
    EXPECT_TRUE(collisionResult.isCollision()) << origin_name;
    std::vector<fcl::Contact<S>> contacts;
    collisionResult.getContacts(contacts);
    GTEST_ASSERT_EQ(contacts.size(), 1u) << origin_name;

    const fcl::Contact<S>& contact = contacts[0];
    EXPECT_NEAR(expected_contact.penetration_depth, contact.penetration_depth,
                test_tolerance)
              << origin_name;
    EXPECT_TRUE(expected_contact.normal.isApprox(contact.normal,
                                                 test_tolerance))
              << origin_name << ":\n\texpected: "
              << expected_contact.normal.transpose()
              << "\n\tcontact.normal: " << contact.normal.transpose();
    contact_pos_test(contact.pos, test_tolerance, origin_name);
  }

  const BoxSpecification<S> box_spec_1_;
  const BoxSpecification<S> box_spec_2_;
  map<string, fcl::Transform3<S>, std::less<string>,
      Eigen::aligned_allocator<std::pair<const string, fcl::Transform3<S>>>>
      iso_poses_;
};

// This test exercises the case of face-something contact. In the language of
// boxBox2() (in box_box-inl.h) it is for codes 1-6.
//
// More particularly it is designed to exercise the case where no contact points
// need be culled. It assumes that boxBox2() is invoked with a maximum contact
// count of 4. The collision produces a manifold that is a four-sided polygon.
//
// The test looks like this:
//
//            z
//            │
//            │
//           ╱│╲
//          ╱ │ ╲ Box1
//    ──┲━━╱━━O━━╲━━┱─── x
//      ┃  ╲  │  ╱  ┃
//      ┃   ╲ │ ╱   ┃
//      ┃    ╲│╱    ┃
//      ┃     │     ┃ Box2
//      ┃     │     ┃
//      ┗━━━━━┿━━━━━┛
//            │
//
// There are two boxes:
//   Box 1: A cube with side length of 1, centered on the world origin (O) and
//          rotated 45 degrees around the y-axis.
//   Box 2: A cube with side length of 3, moved in the negative z-direction such
//          that it's top face lies on the z = 0 plane.
//
// The penetration depth should be sqrt(2) / 2.
// The normal should be parallel with the z-axis. We test both the collision of
// 1 with 2 and 2 with 1. In those cases, the normal would be -z and +z,
// respectively.
// The contact position, should lie on an edge parallel with the y-axis at x = 0
// and z = sqrt(2) / 4.
template <typename S>
void test_collision_box_box_all_contacts(fcl::GJKSolverType solver_type,
                                         S test_tolerance)
{
  const S pi = fcl::constants<S>::pi();
  const S size_1 = 1;
  BoxSpecification<S> box_1{
      fcl::Vector3<S>{size_1, size_1, size_1},
      fcl::Transform3<S>{fcl::AngleAxis<S>(pi / 4, fcl::Vector3<S>::UnitY())}};

  const S size_2 = 3;
  BoxSpecification<S> box_2{fcl::Vector3<S>{size_2, size_2, size_2},
                            fcl::Transform3<S>{fcl::Translation3<S>(
                                fcl::Vector3<S>(0, 0, -size_2 / 2))}};

  fcl::Vector3<S> expected_normal{0, 0, -1};
  S expected_depth = size_1 * sqrt(2) / 2;

  auto contact_pos_test = [size_1](const fcl::Vector3<S> &pos, S tolerance,
                                   const std::string& origin_name) {
    const double expected_pos_z = -size_1 * std::sqrt(2) / 4;
    EXPECT_NEAR(expected_pos_z, pos(2), tolerance) << origin_name;
    EXPECT_NEAR(0, pos(0), tolerance) << origin_name;
    EXPECT_LE(pos(1), 0.5) << origin_name;
    EXPECT_GE(pos(1), -0.5) << origin_name;
  };

  BoxBoxTest<S> tests(box_1, box_2);
  tests.RunTests(solver_type, test_tolerance, expected_normal, expected_depth,
                 contact_pos_test, "test_colliion_box_box_all_contacts");
}

GTEST_TEST(FCL_BOX_BOX, collision_box_box_all_contacts_ccd)
{
  test_collision_box_box_all_contacts<double>(fcl::GJKSolverType::GST_LIBCCD,
                                              1e-14);
}

GTEST_TEST(FCL_BOX_BOX, collision_box_box_all_contacts_indep)
{
  test_collision_box_box_all_contacts<double>(fcl::GJKSolverType::GST_INDEP,
                                              1e-12);
}

// This test exercises the case of face-something contact. In the language of
// boxBox2() (in box_box-inl.h) it is for codes 1-6.
//
// In contrast with the previous test (test_collision_box_box_all_contacts),
// the contact manifold is an eight-sided polygon and contacts will need to be
// culled.
//
// The test looks like this:
//
//       Top view                           Side view
//
//              +y                             +z
//              │                             ┆
//             ╱│╲                            __━━━━━┓  Box1
//       ┏━━━╱━━┿━━╲━━━┓              ┏━━━━━━━ ┆     ┃
//       ┣━╱━━━━┿━━━━╲━┫              ┃        ┆     ┃
//       ╱      │      ╲              ┃        ┆     ┃
//  ___╱_┃______│______┃_╲__ +x     ┄┄┸┰┄┄┄┄┄┄─┼┄─┄─┄┸┰┄┄┄┄ +y
//     ╲ ┃      │      ┃ ╱             ┃       ┆      ┃
//       ╲      │      ╱             ┌─┨   ┏━━━┿━━━┓  ┠──┐
//       ┃ ╲    │    ╱ ┃             │ ┃   ┃   │   ┖━━┛  │
//       ┗━━━●━━┿━━●━━━┛             │ ┗━━━┛   │         │
//             ╲│╱                   │         │         │  Box2
//              │                    │         │         │
//              │                    │         │         │
//                                   └─────────┴─────────┘
//
//
// There are two boxes:
//   Box 1: A cube with side length of 1, centered on the world origin and
//          rotated θ = 22.5 degrees around the x-axis
//   Box 2: A cube with side length of 1, rotated 45 degrees around the z-axis
//          and centered on the point [0, 0, -0.75].
//
// The penetration depth should be √2/2 * cos(π/4 - θ) - 0.25 ≊ 0.4032814.
// The normal should be parallel with the z-axis. We test both the collision of
// 1 with 2 and 2 with 1. In those cases, the normal would be -z and +z,
// respectively.
// The contact position, should lie alone the line segment illustrated
// in the top view (indicated by the two points '●') at z = -0.25 - depth / 2.
template <typename S>
void test_collision_box_box_cull_contacts(fcl::GJKSolverType solver_type,
                                          S test_tolerance)
{
  const S pi = fcl::constants<S>::pi();
  const S size = 1;
  const S tilt = pi / 8;
  BoxSpecification<S> box_1{
      fcl::Vector3<S>{size, size, size},
      fcl::Transform3<S>{fcl::AngleAxis<S>(tilt, fcl::Vector3<S>::UnitX())}};

  BoxSpecification<S> box_2{fcl::Vector3<S>{size, size, size},
                            fcl::Transform3<S>{fcl::AngleAxis<S>(
                                pi / 4, fcl::Vector3<S>::UnitZ())}};
  box_2.X_FB.translation() << 0, 0, -0.75;

  fcl::Vector3<S> expected_normal{0, 0, -1};
  const S expected_depth{sqrt(2) / 2 * cos(pi / 4 - tilt) - 0.25};
  auto contact_pos_test = [expected_depth, tilt, pi](
      const fcl::Vector3<S> &pos, S tolerance, const std::string& origin_name) {
    // Edge is parallel to the x-axis at
    //  z = expected_z = -0.25 - depth / 2
    //  y = expected_y = -√2/2 * sin(π/4 - θ)
    //  x = lines in the range [-x_e, x_e] where
    //  x_e = √2/2 + expected_y
    const S expected_z{-0.25 - expected_depth / 2};
    const S expected_y{-sqrt(2) / 2 * sin(pi / 4 - tilt)};
    const S expected_x{sqrt(2) / 2 + expected_y +
                       std::numeric_limits<S>::epsilon()};
    EXPECT_NEAR(expected_z, pos(2), tolerance) << origin_name;
    EXPECT_NEAR(expected_y, pos(1), tolerance) << origin_name;
    EXPECT_GE(pos(0), -expected_x);
    EXPECT_LE(pos(0), expected_x);
  };

  BoxBoxTest<S> tests(box_1, box_2);
  tests.RunTests(solver_type, test_tolerance, expected_normal, expected_depth,
                 contact_pos_test, "test_collision_box_box_cull_contacts");
}

GTEST_TEST(FCL_BOX_BOX, collision_box_box_cull_contacts_ccd)
{
  test_collision_box_box_cull_contacts<double>(fcl::GJKSolverType::GST_LIBCCD,
                                               1e-14);
}

GTEST_TEST(FCL_BOX_BOX, collision_box_box_cull_contacts_indep)
{
  test_collision_box_box_cull_contacts<double>(fcl::GJKSolverType::GST_INDEP,
                                               1e-14);
}

// This test exercises the case where the contact is between edges (rather than
// face-something as in previous tests).
//
// The test looks like this.
//
//                            ╱╲
//                          ╱    ╲  Box1
//                 +z     ╱   1    ╲
//                      ╱╲           ╲
//                 ┆  ╱    ╲         ╱
//                 ┆╱        ╲     ╱
//            ┏━━━━━╲━━━━┓     ╲ ╱
//            ┃    ┆  ╲  ┃     ╱
//            ┃    ┆    ╲┃   ╱
//       ┄┄┄┄┄┃┄┄┄┄┼┄┄┄┄┄┃╲╱┄┄┄┄┄┄┄┄ +x
//            ┃    ┆     ┃
//            ┃    ┆     ┃  Box2
//            ┗━━━━━━━━━━┛
//                 ┆
//
// There are two boxes:
//    Box 1: A cube with side length of 1. It is rotated 45° around the world's
//           z-axis and then -45° around the world's y-axis. Given a target
//           penetration depth of 0.1, it's center is finally moved along the
//           vector <√2/2, √2/2> a distance of `1 * √2 - 0.1`.
//    Box 2: A cube with side length of 1. It is aligned with and centered on
//           the world frame.
//
template <typename S>
void test_collision_box_box_edge_contact(fcl::GJKSolverType solver_type,
                                         S test_tolerance) {
  const S pi = fcl::constants<S>::pi();
  const S size = 1;
  BoxSpecification<S> box_1{
      fcl::Vector3<S>{size, size, size},
      fcl::Transform3<S>{fcl::AngleAxis<S>(-pi / 4, fcl::Vector3<S>::UnitY())} *
      fcl::Transform3<S>{fcl::AngleAxis<S>(pi / 4, fcl::Vector3<S>::UnitZ())}};
  const fcl::Vector3<S> dir{sqrt(2) / 2, 0, sqrt(2) / 2};
  const S expected_depth = 0.1;
  box_1.X_FB.translation() = dir * (size * sqrt(2) - expected_depth);

  BoxSpecification<S> box_2{fcl::Vector3<S>{size, size, size},
                            fcl::Transform3<S>::Identity()};

  auto contact_pos_test = [expected_depth, size, dir](
      const fcl::Vector3<S> &pos, S tolerance, const std::string& origin_name) {
    // The contact point should unambiguously be a single point.
    const S dist = size * sqrt(2) / 2 - expected_depth / 2;
    const fcl::Vector3<S> expected_pos{dir * dist};
    EXPECT_TRUE(expected_pos.isApprox(pos, tolerance)) << origin_name
              << "\n\texpected: " << expected_pos.transpose()
              << "\n\tpos: " << pos.transpose();
  };

  BoxBoxTest<S> tests(box_1, box_2);
  tests.RunTests(solver_type, test_tolerance, -dir, expected_depth,
                 contact_pos_test, "test_collision_box_box_edge_contact");
}

GTEST_TEST(FCL_BOX_BOX, collision_box_box_edge_contact_ccd)
{
  test_collision_box_box_edge_contact<double>(fcl::GJKSolverType::GST_LIBCCD,
                                              1e-14);
}

GTEST_TEST(FCL_BOX_BOX, collision_box_box_edge_contact_indep)
{
  test_collision_box_box_edge_contact<double>(fcl::GJKSolverType::GST_INDEP,
                                              1e-14);
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

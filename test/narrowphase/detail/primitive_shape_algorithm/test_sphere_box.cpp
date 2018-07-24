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

/** @author Sean Curtis (sean@tri.global) */

// Tests the custom sphere-box tests: distance and collision.

#include "fcl/narrowphase/detail/primitive_shape_algorithm/sphere_box-inl.h"

#include <string>

#include <gtest/gtest.h>

#include "eigen_matrix_compare.h"
#include "fcl/geometry/shape/box.h"
#include "fcl/geometry/shape/sphere.h"

namespace fcl {
namespace detail {
namespace {

// In the worst case (with arbitrary frame orientations) it seems like I'm
// losing about 4 bits of precision in the solution (compared to performing
// the equivalent query without any rotations). This encodes that bit loss to
// an epsilon value appropriate to the scalar type.
template <typename S>
struct Eps {
  using Real = typename constants<S>::Real;
  static Real value() { return 16 * constants<S>::eps(); }
};

// Utility function for evaluating points inside boxes. Tests various
// configurations of points and boxes.
template <typename S> void NearestPointInBox() {
  // Picking sizes that are *not* powers of two and *not* uniform in size.
  Box<S> box{0.6, 1.2, 3.6};
  Vector3<S> p_BN;
  Vector3<S> p_BQ;

  // Case: query point at origin.
  p_BQ << 0, 0, 0;
  bool N_is_not_C = nearestPointInBox(box.side, p_BQ, p_BN);
  EXPECT_FALSE(N_is_not_C);
  EXPECT_TRUE(CompareMatrices(p_BN, p_BQ, 0, MatrixCompareType::absolute));

  Vector3<S> half_size = box.side * 0.5;
  // Per-octant tests:
  for (S x : {-1, 1}) {
    for (S y : {-1, 1}) {
      for (S z : {-1, 1}) {
        Vector3<S> quadrant{x, y, z};
        // Case: point inside (no clamped values).
        p_BQ = quadrant.cwiseProduct(half_size * 0.5);
        N_is_not_C = nearestPointInBox(box.side, p_BQ, p_BN);
        EXPECT_FALSE(N_is_not_C);
        EXPECT_TRUE(
            CompareMatrices(p_BN, p_BQ, 0, MatrixCompareType::absolute));

        // For each axis:
        for (int axis : {0, 1, 2}) {
          // Case: one direction clamped.
          Vector3<S> scale{0.5, 0.5, 0.5};
          scale(axis) = 1.5;
          p_BQ = quadrant.cwiseProduct(half_size.cwiseProduct(scale));
          N_is_not_C = nearestPointInBox(box.side, p_BQ, p_BN);
          EXPECT_TRUE(N_is_not_C);
          for (int i : {0, 1, 2}) {
            if (i == axis)
              EXPECT_EQ(p_BN(i), quadrant(i) * half_size(i));
            else
              EXPECT_EQ(p_BN(i), p_BQ(i));
          }

          // Case: One direction unclamped.
          scale << 1.5, 1.5, 1.5;
          scale(axis) = 0.5;
          p_BQ = quadrant.cwiseProduct(half_size.cwiseProduct(scale));
          N_is_not_C = nearestPointInBox(box.side, p_BQ, p_BN);
          EXPECT_TRUE(N_is_not_C);
          for (int i : {0, 1, 2}) {
            if (i == axis)
              EXPECT_EQ(p_BN(i), p_BQ(i));
            else
              EXPECT_EQ(p_BN(i), quadrant(i) * half_size(i));
          }

          // Case: query point on face in axis direction -- unclamped.
          scale << 0.5, 0.5, 0.5;
          scale(axis) = 1.0;
          p_BQ = quadrant.cwiseProduct(half_size.cwiseProduct(scale));
          N_is_not_C = nearestPointInBox(box.side, p_BQ, p_BN);
          EXPECT_FALSE(N_is_not_C);
          EXPECT_TRUE(
              CompareMatrices(p_BN, p_BQ, 0, MatrixCompareType::absolute));
        }

        // Case: external point in Voronoi region of corner (all axes clamped).
        p_BQ = quadrant.cwiseProduct(half_size * 1.5);
        N_is_not_C = nearestPointInBox(box.side, p_BQ, p_BN);
        EXPECT_TRUE(N_is_not_C);
        for (int i : {0, 1, 2})
          EXPECT_EQ(p_BN(i), quadrant(i) * half_size(i));
      }
    }
  }
}

// Defines the test configuration for a single test. It includes the geometry
// and the pose of the sphere in the box's frame B. It also includes the
// expected answers in that same frame.
//
// Collision and distance are complementary queries -- two objects in collision
// have no defined distance because they are *not* separated and vice versa.
// These configurations allow for the test of the complementarity property.
template <typename S>
struct TestConfiguration {
  TestConfiguration(const std::string &name_in, const Vector3<S> &half_size_in,
                    S radius, const Vector3<S> &p_BSo_in, bool colliding)
      : name(name_in), half_size(half_size_in), r(radius), p_BSo(p_BSo_in),
        expected_colliding(colliding) {}

  // Descriptive name of the test configuration.
  std::string name;
  // The half size of the axis-aligned, origin-centered box.
  Vector3<S> half_size;
  // Radius of the sphere.
  S r;
  // Position of the sphere's center in the box frame.
  Vector3<S> p_BSo;

  // Indicates if this test configuration is expected to be in collision.
  bool expected_colliding{false};

  // Collision values; only valid if expected_colliding is true.
  S expected_depth{-1};
  Vector3<S> expected_normal;
  Vector3<S> expected_pos;

  // Distance values; only valid if expected_colliding is false.
  S expected_distance{-1};
  // The points on sphere and box, respectively, closest to the others measured
  // and expressed in the box frame B. Only defined if separated.
  Vector3<S> expected_p_BSb;
  Vector3<S> expected_p_BBs;
};

// Returns a collection of configurations where sphere and box are uniformly
// scaled.
template <typename S>
std::vector<TestConfiguration<S>> GetUniformConfigurations() {
  // Common configuration values
  // Box and sphere dimensions.
  const S w = 0.6;
  const S d = 1.2;
  const S h = 3.6;
  const S r = 0.7;
  const Vector3<S> half_size{w / 2, d / 2, h / 2};
  const bool collides = true;

  std::vector<TestConfiguration<S>> configurations;

  {
  // Case: Completely separated. Nearest point on the +z face.
    const Vector3<S> p_BS{half_size(0) * S(0.5), half_size(1) * S(0.5),
                          half_size(2) + r * S(1.1)};
  configurations.emplace_back(
      "Separated; nearest face +z", half_size, r, p_BS, !collides);
    // Not colliding --> no collision values.
    TestConfiguration<S>& config = configurations.back();
    config.expected_distance = p_BS(2) - half_size(2) - r;
    config.expected_p_BBs = Vector3<S>{p_BS(0), p_BS(1), half_size(2)};
    config.expected_p_BSb = Vector3<S>{p_BS(0), p_BS(1), p_BS(2) - r};
  }

  {
  // Case: Sphere completely separated with center in vertex Voronoi region.
    const Vector3<S> p_BS = half_size + Vector3<S>{r, r, r} * S(1.25);
  configurations.emplace_back(
      "Separated; nearest +x, +y, +z corner", half_size, r, p_BS, !collides);
    // Not colliding --> no collision values.
    TestConfiguration<S>& config = configurations.back();
    // position vector from sphere center (S) to nearest point on box (N).
    const Vector3<S> r_SN = half_size - p_BS;
    const S len_r_SN = r_SN.norm();
    config.expected_distance = len_r_SN - r;
    config.expected_p_BBs = half_size;
    config.expected_p_BSb = p_BS + r_SN * (r / len_r_SN);
  }

  // Case: Intersection with the sphere center *outside* the box.
  // Subcase: sphere in face voronoi region -- normal should be in face
  // direction.
  // Intersects the z+ face with a depth of half the radius and a normal in the
  // -z direction.
  {
    const S target_depth = r * 0.5;
    const Vector3<S> p_BS{half_size + Vector3<S>{0, 0, r - target_depth}};
    configurations.emplace_back(
        "Colliding: center outside, center projects onto +z face", half_size, r,
        p_BS, collides);
    TestConfiguration<S>& config = configurations.back();
    config.expected_depth = target_depth;
    config.expected_normal = -Vector3<S>::UnitZ();
    config.expected_pos = Vector3<S>{p_BS(0), p_BS(1), (h - target_depth) / 2};

    // Colliding; no distance values required.
  }

  // Subcase: sphere in vertex Voronoi region -- normal should be in the
  // direction from sphere center to box corner.
  {
    const S target_depth = r * 0.5;
    const Vector3<S> n_SB_B = Vector3<S>(-1, -2, -3).normalized();
    const Vector3<S> p_BS = half_size  - n_SB_B * (r - target_depth);
    configurations.emplace_back(
        "Colliding: center outside, center nearest +x, +y, +z vertex",
        half_size, r, p_BS, collides);
    TestConfiguration<S>& config = configurations.back();
    config.expected_depth = target_depth;
    config.expected_normal = n_SB_B;
    config.expected_pos = half_size + n_SB_B * (target_depth * 0.5);

    // Colliding; no distance values required.
  }

  // Case: Intersection with the sphere center *inside* the box. The center is
  // *nearest* the +z face, so the normal should be in the -z direction, the
  // penetration depth is the radius *plus* the amount below h/2 the center lies
  // (i.e., h - cz + radius). The position is half the penetration depth below
  // the +z face.
  {
    // NOTE: The center is moved the least amount in the z-direction.
    const Vector3<S> p_BS = half_size + Vector3<S>{-0.2, -0.2, -0.1};
    configurations.emplace_back(
        "Colliding: center inside, center nearest +z face",
        half_size, r, p_BS, collides);
    TestConfiguration<S>& config = configurations.back();
    config.expected_depth = r + half_size(2) - p_BS(2);
    config.expected_normal = -Vector3<S>::UnitZ();
    config.expected_pos = Vector3<S>{p_BS(0), p_BS(1),
                                     half_size(2) - config.expected_depth / 2};

    // Colliding; no distance values required.
  }

  return configurations;
}

// Returns a collection of configurations where sphere and box are scaled
// very differently.
template <typename S>
std::vector<TestConfiguration<S>> GetNonUniformConfigurations() {
  std::vector<TestConfiguration<S>> configurations;

  {
    // Case: long "skinny" box and tiny sphere. Nearest feature is the +z face.
    const Vector3<S> half_size(15, 1, 1);
    const S r = 0.01;
    {
      // Subcase: colliding.
      const Vector3<S> p_BS{half_size(0) * S(0.95), S(0),
                            half_size(2) + r * S(0.5)};
      configurations.emplace_back("Long, skinny box collides with small sphere",
                                  half_size, r, p_BS, true /* colliding */);
      TestConfiguration<S>& config = configurations.back();
      config.expected_normal = -Vector3<S>::UnitZ();
      config.expected_depth = r - (p_BS(2) - half_size(2));
      config.expected_pos =
          Vector3<S>{p_BS(0), p_BS(1),
                     half_size(2) - config.expected_depth / 2};
    }
    {
      // Subcase: not-colliding.
      const S distance = r * 0.1;
      const Vector3<S> p_BS{half_size(0) * S(0.95), S(0),
                            half_size(2) + r + distance};
      configurations.emplace_back(
          "Long, skinny box *not* colliding with small sphere", half_size,
          r, p_BS, false /* not colliding */);
      TestConfiguration<S>& config = configurations.back();
      config.expected_distance = distance;
      config.expected_p_BSb = p_BS - Vector3<S>{0, 0, r};
      config.expected_p_BBs << p_BS(0), p_BS(1), half_size(2);
    }
  }

  {
    // Case: Large sphere collides with small box. Nearest feature is the +x,
    // +y, +z corner.
    const Vector3<S> half_size(0.1, 0.15, 0.2);
    const S r = 10;
    const Vector3<S> n_SB = Vector3<S>{-1, -2, -3}.normalized();
    {
      // Subcase: colliding.
      S target_depth = half_size.minCoeff() * 0.5;
      const Vector3<S> p_BS = half_size - n_SB * (r - target_depth);
      configurations.emplace_back("Large sphere colliding with tiny box",
                                  half_size, r, p_BS, true /* colliding */);
      TestConfiguration<S>& config = configurations.back();
      config.expected_normal = n_SB;
      config.expected_depth = target_depth;
      config.expected_pos = half_size + n_SB * (target_depth * 0.5);
    }
    {
      // Subcase: not colliding.
      S distance = half_size.minCoeff() * 0.1;
      const Vector3<S> p_BS = half_size - n_SB * (r + distance);
      configurations.emplace_back(
          "Large sphere *not* colliding with tiny box", half_size,
          r, p_BS, false /* not colliding */);
      TestConfiguration<S>& config = configurations.back();
      config.expected_distance = distance;
      config.expected_p_BSb = p_BS + n_SB * r;
      config.expected_p_BBs = half_size;
    }
  }

  return configurations;
}

template <typename S>
using EvalFunc =
    std::function<void(const TestConfiguration<S> &, const Transform3<S> &,
                       const Matrix3<S> &, S)>;

// This evaluates an instance of a test configuration and confirms the results
// match the expected data. The test configuration is defined in the box's
// frame with an unrotated sphere. The parameters provide the test
// configuration, an pose of the box's frame in the world frame.
//
// Evaluates the collision query twice. Once as the boolean "is colliding" test
// and once with the collision characterized with depth, normal, and position.
template <typename S>
void EvalCollisionForTestConfiguration(const TestConfiguration<S>& config,
                                       const Transform3<S>& X_WB,
                                       const Matrix3<S>& R_SB,
                                       S eps) {
  // Set up the experiment from input parameters and test configuration.
  Box<S> box{config.half_size * 2};
  Sphere<S> sphere{config.r};
  Transform3<S> X_BS = Transform3<S>::Identity();
  X_BS.translation() = config.p_BSo;
  X_BS.linear() = R_SB;
  Transform3<S> X_WS = X_WB * X_BS;

  bool colliding = sphereBoxIntersect<S>(sphere, X_WS, box, X_WB, nullptr);
  EXPECT_EQ(colliding, config.expected_colliding) << config.name;

  std::vector<ContactPoint<S>> contacts;
  colliding = sphereBoxIntersect<S>(sphere, X_WS, box, X_WB, &contacts);
  EXPECT_EQ(colliding, config.expected_colliding) << config.name;
  if (config.expected_colliding) {
    EXPECT_EQ(contacts.size(), 1u) << config.name;
    const ContactPoint<S> &contact = contacts[0];
    EXPECT_NEAR(contact.penetration_depth, config.expected_depth, eps)
              << config.name;
    EXPECT_TRUE(CompareMatrices(contact.normal,
                                X_WB.linear() * config.expected_normal, eps,
                                MatrixCompareType::absolute))
              << config.name;
    EXPECT_TRUE(CompareMatrices(contact.pos, X_WB * config.expected_pos, eps,
                                MatrixCompareType::absolute))
              << config.name;
  } else {
    EXPECT_EQ(contacts.size(), 0u) << config.name;
  }
}

// This evaluates an instance of a test configuration and confirms the results
// match the expected data. The test configuration is defined in the box's
// frame with an unrotated sphere. The parameters provide the test
// configuration.
//
// Evaluates the distance query twice. Once as the boolean "is separated" test
// and once with the separation characterized with distance and surface points.
template <typename S>
void EvalDistanceForTestConfiguration(const TestConfiguration<S>& config,
                                      const Transform3<S>& X_WB,
                                      const Matrix3<S>& R_SB,
                                      S eps) {
  // Set up the experiment from input parameters and test configuration.
  Box<S> box{config.half_size * 2};
  Sphere<S> sphere{config.r};
  Transform3<S> X_BS = Transform3<S>::Identity();
  X_BS.translation() = config.p_BSo;
  X_BS.linear() = R_SB;
  Transform3<S> X_WS = X_WB * X_BS;

  bool separated = sphereBoxDistance<S>(sphere, X_WS, box, X_WB, nullptr,
                                        nullptr, nullptr);
  EXPECT_NE(separated, config.expected_colliding) << config.name;

  // Initializing this to -2, to confirm that a non-colliding scenario sets
  // distance to -1.
  S distance{-2};
  Vector3<S> p_WSb{0, 0, 0};
  Vector3<S> p_WBs{0, 0, 0};

  separated =
      sphereBoxDistance<S>(sphere, X_WS, box, X_WB, &distance, &p_WSb, &p_WBs);
  EXPECT_NE(separated, config.expected_colliding) << config.name;
  if (!config.expected_colliding) {
    EXPECT_NEAR(distance, config.expected_distance, eps)
              << config.name;
    EXPECT_TRUE(CompareMatrices(p_WSb,
                                X_WB * config.expected_p_BSb, eps,
                                MatrixCompareType::absolute))
              << config.name;
    EXPECT_TRUE(CompareMatrices(p_WBs,
                                X_WB * config.expected_p_BBs, eps,
                                MatrixCompareType::absolute))
              << config.name;
  } else {
    EXPECT_EQ(distance, S(-1)) << config.name;
    EXPECT_TRUE(CompareMatrices(p_WSb, Vector3<S>::Zero(), 0,
                                MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(p_WBs, Vector3<S>::Zero(), 0,
                                MatrixCompareType::absolute));
  }
}

// This test defines the transforms for performing the single collision test.
template <typename S>
void QueryWithVaryingWorldFrames(
    const std::vector<TestConfiguration<S>>& configurations,
    EvalFunc<S> query_eval, const Matrix3<S> &R_BS = Matrix3<S>::Identity()) {
  // Evaluate all the configurations with the given box pose in frame F.
  auto evaluate_all = [&R_BS, query_eval, &configurations](
      const Transform3<S>& X_FB) {
    for (const auto config : configurations) {
      query_eval(config, X_FB, R_BS, Eps<S>::value());
    }
  };

  // Frame F is the box frame.
  Transform3<S> X_FB = Transform3<S>::Identity();
  evaluate_all(X_FB);

  // Simple arbitrary translation away from the origin.
  X_FB.translation() << 1.3, 2.7, 6.5;
  evaluate_all(X_FB);

  // 90 degree rotation around each axis.
  for (int axis = 0; axis < 3; ++axis) {
    AngleAxis<S> angle_axis{constants<S>::pi() / 2, Vector3<S>::Unit(axis)};
    X_FB.linear() << angle_axis.matrix();
    evaluate_all(X_FB);
  }

  // Arbitrary orientation.
  {
    AngleAxis<S> angle_axis{constants<S>::pi() / 3,
                            Vector3<S>{1, 2, 3}.normalized()};
    X_FB.linear() << angle_axis.matrix();
    evaluate_all(X_FB);
  }

  // Near axis aligned.
  {
    AngleAxis<S> angle_axis{constants<S>::eps_12(), Vector3<S>::UnitX()};
    X_FB.linear() << angle_axis.matrix();
    evaluate_all(X_FB);
  }
}

// Runs all test configurations across multiple poses in the world frame --
// changing the orientation of the sphere -- should have no affect on the
// results.
template <typename S>
void QueryWithOrientedSphere(
    const std::vector<TestConfiguration<S>>& configurations,
    EvalFunc<S> query_eval) {
  // 90 degree rotation around each axis.
  for (int axis = 0; axis < 3; ++axis) {
    AngleAxis<S> angle_axis{constants<S>::pi() / 2, Vector3<S>::Unit(axis)};
    QueryWithVaryingWorldFrames<S>(configurations, query_eval,
                                   angle_axis.matrix());
  }

  // Arbitrary orientation.
  {
    AngleAxis<S> angle_axis{constants<S>::pi() / 3,
                            Vector3<S>{1, 2, 3}.normalized()};
    QueryWithVaryingWorldFrames<S>(configurations, query_eval,
                                   angle_axis.matrix());
  }

  // Near axis aligned.
  {
    AngleAxis<S> angle_axis{constants<S>::eps_12(), Vector3<S>::UnitX()};
    QueryWithVaryingWorldFrames<S>(configurations, query_eval,
                                   angle_axis.matrix());
  }
}

//======================================================================

// Tests the helper function that finds the closest point in the box.
GTEST_TEST(SphereBoxPrimitiveTest, NearestPointInBox) {
  NearestPointInBox<float>();
  NearestPointInBox<double>();
}

// Evaluates collision on all test configurations across multiple poses in the
// world frame - but the sphere rotation is always the identity.
GTEST_TEST(SphereBoxPrimitiveTest, CollisionAcrossVaryingWorldFrames) {
  QueryWithVaryingWorldFrames<float>(GetUniformConfigurations<float>(),
                                     EvalCollisionForTestConfiguration<float>);
  QueryWithVaryingWorldFrames<double>(GetUniformConfigurations<double>(),
      EvalCollisionForTestConfiguration<double>);
}

// Evaluates collision on all test configurations across multiple poses in the
// world frame - the sphere is rotated arbitrarily.
GTEST_TEST(SphereBoxPrimitiveTest, CollisionWithSphereRotations) {
  QueryWithOrientedSphere<float>(GetUniformConfigurations<float>(),
                                 EvalCollisionForTestConfiguration<float>);
  QueryWithOrientedSphere<double>(GetUniformConfigurations<double>(),
                                  EvalCollisionForTestConfiguration<double>);
}

// Evaluates collision on a small set of configurations where the box and scale
// are of radically different scales - evaluation across multiple poses in the
// world frame.
GTEST_TEST(SphereBoxPrimitiveTest, CollisionIncompatibleScales) {
  QueryWithVaryingWorldFrames<float>(GetNonUniformConfigurations<float>(),
                                     EvalCollisionForTestConfiguration<float>);
  QueryWithVaryingWorldFrames<double>(GetNonUniformConfigurations<double>(),
      EvalCollisionForTestConfiguration<double>);
}

// Evaluates distance on all test configurations across multiple poses in the
// world frame - but the sphere rotation is always the identity.
GTEST_TEST(SphereBoxPrimitiveTest, DistanceAcrossVaryingWorldFrames) {
  QueryWithVaryingWorldFrames<float>(GetUniformConfigurations<float>(),
                                     EvalDistanceForTestConfiguration<float>);
  QueryWithVaryingWorldFrames<double>(GetUniformConfigurations<double>(),
                                      EvalDistanceForTestConfiguration<double>);
}

// Evaluates distance on all test configurations across multiple poses in the
// world frame - the sphere is rotated arbitrarily.
GTEST_TEST(SphereBoxPrimitiveTest, DistanceWithSphereRotations) {
  QueryWithOrientedSphere<float>(GetUniformConfigurations<float>(),
                                 EvalDistanceForTestConfiguration<float>);
  QueryWithOrientedSphere<double>(GetUniformConfigurations<double>(),
                                  EvalDistanceForTestConfiguration<double>);
}

// Evaluates distance on a small set of configurations where the box and scale
// are of radically different scales - evaluation across multiple poses in the
// world frame.
GTEST_TEST(SphereBoxPrimitiveTest, DistanceIncompatibleScales) {
  QueryWithVaryingWorldFrames<float>(GetNonUniformConfigurations<float>(),
                                     EvalDistanceForTestConfiguration<float>);
  QueryWithVaryingWorldFrames<double>(GetNonUniformConfigurations<double>(),
                                      EvalDistanceForTestConfiguration<double>);
}

} // namespace
} // namespace detail
} // namespace fcl

//==============================================================================
int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

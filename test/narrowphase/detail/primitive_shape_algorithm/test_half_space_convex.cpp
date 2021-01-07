/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020. Toyota Research Institute
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

/** @author Sean Curtis (sean@tri.global) (2020) */

// Tests the Halfspace-Convex primitive collision test.

// TODO(SeanCurtis-TRI): Currently, there are no half space-X distance primitive
//  implementations.

#include "fcl/narrowphase/detail/primitive_shape_algorithm/halfspace-inl.h"

#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "eigen_matrix_compare.h"
#include "fcl/geometry/shape/convex.h"
#include "fcl/geometry/shape/halfspace.h"

namespace fcl {
namespace detail {
namespace {

using std::make_shared;
using std::move;
using std::shared_ptr;
using std::string;
using std::vector;

/*
 Creates a simple tetrahedron as a Convex geometry.
       +z
        │
        │
        o v3
        │
        │
        │ v0       v2
        o─────────o───── +y
       ╱
      ╱
     ╱
    o v1
   ╱
  +x

  The tet is defined in geometry frame G, shown above with vertex positions
  marked. The tet is then posed in frame T, and the final convex mesh is
  produced with its vertices measured and expressed in frame T.
 */
template <typename S>
Convex<S> MakeTetrahedron(const Transform3<S>& X_TG) {
  auto vertices = make_shared<vector<Vector3<S>>>();
  vertices->emplace_back(X_TG * Vector3<S>{0, 0, 0});
  vertices->emplace_back(X_TG * Vector3<S>{1, 0, 0});
  vertices->emplace_back(X_TG * Vector3<S>{0, 1, 0});
  vertices->emplace_back(X_TG * Vector3<S>{0, 0, 1});

  auto faces = make_shared<vector<int>>();
  auto add_triangle = [&faces](int v0, int v1, int v2) {
    faces->push_back(3);  // number of vertices in the face.
    faces->push_back(v0);
    faces->push_back(v1);
    faces->push_back(v2);
  };

  // The ordering of the vertex indices in each triangle is important but the
  // ordering of the triangles isn't.
  add_triangle(0, 2, 1);  // face on xy plane.
  add_triangle(0, 1, 3);  // face on xz plane.
  add_triangle(0, 3, 2);  // face on yz plane.
  add_triangle(2, 3, 1);  // diagonal face.

  return {vertices, 4, faces};
}

/* Specifies the data necessary for a single test and the expected results.
 The Convex tetrahedron is defined in frame T, the Halfspace is defined in frame
 H, and both are posed relative to the configuration frame C with X_CT and X_CH,
 respectively. */
template <typename S>
struct Configuration {
  /* Convenience constructor for creating an expected _non-colliding_
   configuration. */
  Configuration(const string& name_in, const Transform3<S>& X_CT_in,
                const Transform3<S>& X_CH_in)
      : Configuration(name_in, X_CT_in, X_CH_in, false) {}

  /* Constructor for expected _colliding_ configuration.  */
  Configuration(const string& name_in, const Transform3<S>& X_CT_in,
                const Transform3<S>& X_CH_in, S depth,
                const Vector3<S>& normal_C, const Vector3<S> pos_C)
      : Configuration(name_in, X_CT_in, X_CH_in, true, depth, normal_C, pos_C) {
  }

  /* Full constructor. */
  Configuration(const string& label_in, const Transform3<S>& X_CT_in,
                const Transform3<S>& X_CH_in, bool expected, S depth = S(0),
                const Vector3<S>& normal_C = Vector3<S>::Zero(),
                const Vector3<S> pos_C = Vector3<S>::Zero())
      : label(label_in),
        X_CT(X_CT_in),
        X_CH(X_CH_in),
        expected_colliding(expected),
        expected_depth(depth),
        expected_normal_C(normal_C),
        expected_pos_C(pos_C) {}

  string label;
  // Pose of the tetrahedron in the configuration frame C.
  Transform3<S> X_CT;
  // Pose of the half space in the configuration frame C.
  Transform3<S> X_CH;

  // Expected results; if expected_colliding is false, the remaining fields
  // should be ignored.
  bool expected_colliding{false};
  S expected_depth{-1};
  Vector3<S> expected_normal_C;
  Vector3<S> expected_pos_C;
};

// Constructs a transform from an angle-axis rotation and origin point.
template <typename S>
Transform3<S> MakeTransform(const AngleAxis<S>& angle_axis,
                            const Vector3<S>& origin) {
  Transform3<S> transform = Transform3<S>::Identity();
  transform.linear() = angle_axis.matrix();
  transform.translation() = origin;
  return transform;
}

template <typename S>
aligned_vector<Configuration<S>> GetConfigurations() {
  const Transform3<S> I = Transform3<S>::Identity();
  const S kPi = constants<S>::pi();

  aligned_vector<Configuration<S>> configurations;

  // Vanilla configuration -- the un-rotated tet floats slightly above the
  // plane.
  configurations.push_back(Configuration<S>{
      "simple non-colliding",
      MakeTransform(AngleAxis<S>::Identity(), Vector3<S>{0, 0, S(0.1)}), I});

  // Simple penetration - the tet is rotated so the point V3 is down and
  // penetrates the plane.
  const AngleAxis<S> R_CT_point_down{kPi, Vector3<S>::UnitX()};
  configurations.push_back(Configuration<S>{
      "simple penetration",
      MakeTransform(R_CT_point_down, Vector3<S>{0, 0, 0.5}), I, S(0.5),
      Vector3<S>{0, 0, -1}, Vector3<S>{0, 0, S(-0.25)}});

  // Orient the half-space so it is not axis aligned and does not pass through
  // the origin. Then position the tet so that the point V3 is near penetration
  // and submit *two* configurations: one in collision, one not.

  const Transform3<S> X_CH =
      MakeTransform(AngleAxis<S>{kPi / 5, Vector3<S>{1, 2, 3}.normalized()},
                    Vector3<S>{S(0.25), S(0.5), S(0.75)});

  // Steal the orientation from the previous configuration so that V3 is
  // pointing downwards into the half space.
  const AngleAxis<S> R_HT_point_down = R_CT_point_down;

  const Transform3<S> X_HT_separated =
      MakeTransform(R_HT_point_down, Vector3<S>{0, 0, S(1.01)});
  configurations.push_back(Configuration<S>{
      "non-trivial half space, non-colliding", X_CH * X_HT_separated, X_CH});

  // We pose the tet relative to the half plane, and then transform it again into
  // the configuration frame. Offset of 0.5 in the z-direction gives us a
  // penetration depth of 0.5.
  const Transform3<S> X_HT_colliding =
      MakeTransform(R_HT_point_down, Vector3<S>{0, 0, S(0.5)});
  const Transform3<S> X_CT_colliding = X_CH * X_HT_colliding;
  const Vector3<S> Hz_C = X_CH.linear().col(2);
  // By construction, we're colliding V3 (0, 0, 1). So, let's find where it is
  // in this configuration.
  const Vector3<S> p_TV3 = Vector3<S>::UnitZ();
  const Vector3<S> p_CV3 = X_CT_colliding * p_TV3;
  const S depth(0.5);
  configurations.push_back(Configuration<S>{"non-trivial half space, colliding",
                                            X_CT_colliding, X_CH, depth, -Hz_C,
                                            p_CV3 + Hz_C * S(0.5) * depth});

  return configurations;
}

/* Evaluates convexHalfspaceIntersect() with the given parameters and tests
 the result based on the given test configuration's expectations.  */
template <typename S>
void TestCollision(const Convex<S>& convex_T, const Transform3<S>& X_WT,
                   const Halfspace<S>& half_space_H, const Transform3<S>& X_WH,
                   const Configuration<S>& config, const Transform3<S>& X_WC,
                   S eps, const string& label) {
  vector<ContactPoint<S>> results_W;
  bool colliding =
      convexHalfspaceIntersect(convex_T, X_WT, half_space_H, X_WH, &results_W);
  EXPECT_EQ(colliding, config.expected_colliding) << label;
  if (config.expected_colliding) {
    EXPECT_EQ(results_W.size(), 1u) << label;
    const ContactPoint<S>& point_W = results_W[0];
    EXPECT_NEAR(point_W.penetration_depth, config.expected_depth, eps) << label;
    // The expected vector quantities are given in Frame C, the results are in
    // Frame W. We need to put them in the same frame for a valid comparison.
    EXPECT_TRUE(CompareMatrices(point_W.normal,
                                X_WC.linear() * config.expected_normal_C, eps))
        << label;
    EXPECT_TRUE(CompareMatrices(point_W.pos, X_WC * config.expected_pos_C, eps))
        << label;
  }
}

/* This evaluates an instance of a test configuration and confirms the results
 match the expected data.

 The test configuration defines the relative position of tet and half space.
 in a configuration frame C. This is transformed into a test world frame W
 such that X_WC has no identities (0s or 1s) (to test numerical robustness
 against non-trivial transforms).  */
template <typename S>
void EvalCollisionForTestConfiguration(const Configuration<S>& config, S eps) {
  aligned_vector<Transform3<S>> X_WCs{};
  X_WCs.emplace_back(Transform3<S>::Identity());
  X_WCs.emplace_back(
      MakeTransform(AngleAxis<S>{constants<S>::pi() / 7,
                                 Vector3<S>{1, -2, S(-1.3)}.normalized()},
                    Vector3<S>{S(-0.25), S(0.5), S(-0.75)}));

  for (const auto& X_WC : X_WCs) {
    // For a given configuration, we can set it up two ways:
    // X_CT and X_CH multiply convex_T and half_space_H, respectively, or
    // identity multiplies convex_C and half_space_C.
    // We'll do both; their answers should be the same. This will allow us to
    // easily confirm that all of the values contribute.

    const Convex<S> convex_C = MakeTetrahedron<S>(config.X_CT);
    const Vector3<S> Hz_C = config.X_CH.linear().col(2);
    const S d = Hz_C.dot(config.X_CH.translation());
    const Halfspace<S> half_space_C(Hz_C, d);

    // First test without penetration data. Sampling with only one of the
    // geometry definitions seems sufficient.
    bool colliding = convexHalfspaceIntersect<S>(convex_C, X_WC, half_space_C,
                                                 X_WC, nullptr);
    EXPECT_EQ(colliding, config.expected_colliding) << config.label;

    // Test with penetration data.

    // First test where X_CH = X_CT = I and geometries are defined directly in
    // Frame C.
    TestCollision(convex_C, X_WC, half_space_C, X_WC, config, X_WC, eps,
                  config.label + ", X_CH = X_CT = I");

    // Now test X_CH != X_CT != I and geometries are defined in their canonical
    // frames H and T.
    const Convex<S> convex_T = MakeTetrahedron<S>(Transform3<S>::Identity());
    const Halfspace<S> half_space_H(Vector3<S>{0, 0, 1}, 0);
    TestCollision(convex_T, X_WC * config.X_CT, half_space_H,
                  X_WC * config.X_CH, config, X_WC, eps,
                  config.label + ", X_CH != X_CT != I");
  }
}

// Simply evaluate collision on each of the given configurations.
template <typename S>
void QueryCollision(const aligned_vector<Configuration<S>>& configs) {
  // Half space collision is clean enough that even under ugly rotations, we
  // still get good precision.
  const S eps = 2 * constants<S>::eps();
  for (const auto& config : configs) {
    EvalCollisionForTestConfiguration(config, eps);
  }
}

GTEST_TEST(HalfspaceConvexPrimitiveTest, CollisionTests) {
  QueryCollision<float>(GetConfigurations<float>());
  QueryCollision<double>(GetConfigurations<double>());
}

}  // namespace
}  // namespace detail
}  // namespace fcl

//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

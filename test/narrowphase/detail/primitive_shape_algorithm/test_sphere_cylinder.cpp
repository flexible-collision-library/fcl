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

/** @author Sean Curtis (sean@tri.global) (2018) */

// Tests the custom sphere-cylinder tests: distance and collision.

#include "fcl/narrowphase/detail/primitive_shape_algorithm/sphere_cylinder-inl.h"

#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "eigen_matrix_compare.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/geometry/shape/sphere.h"

namespace fcl {
namespace detail {
namespace {

// In the worst case (with arbitrary frame orientations) it seems like I'm
// losing about 4 bits of precision in the solution (compared to performing
// the equivalent query without any rotations). This encodes that bit loss to
// an epsilon value appropriate to the scalar type.
//
// TODO(SeanCurtis-TRI): These eps values are *not* optimal. They are the result
// of a *number* of issues.
// 1. Generally, for float the scalar must be at least 20 * ε. The arbitrary
//    rotation *really* beats up on the precision.
// 2. CI uses Eigen 3.2.0. The threshold must be 22 * ε for the tests to pass.
//    This is true, even for doubles. Later versions (e.g., 3.2.92, aka
//    3.3-beta1) can pass with a tolerance of 16 * ε.
// 3. Mac CI requires another bump in the multiplier for floats. So, floats here
//    are 24.
// Upgrade the Eigen version so that this tolerance can be reduced.
template <typename S>
struct Eps {
  using Real = typename constants<S>::Real;
  static Real value() { return 22 * constants<S>::eps(); }
};

template <>
struct Eps<float> {
  using Real = typename constants<float>::Real;
  static Real value() { return 24 * constants<float>::eps(); }
};

// Utility function for evaluating points inside cylinders. Tests various
// configurations of points and cylinders.
template <typename S> void NearestPointInCylinder() {
  // Picking sizes that are *not* powers of two and *not* uniform in size.
  const S r = 0.6;
  const S h = 1.8;
  Vector3<S> p_CN;
  Vector3<S> p_CQ;

  // Case: query point at origin.
  p_CQ << 0, 0, 0;
  bool N_is_not_S = nearestPointInCylinder(h, r, p_CQ, &p_CN);
  EXPECT_FALSE(N_is_not_S) << "point at origin";
  EXPECT_TRUE(CompareMatrices(p_CN, p_CQ, 0, MatrixCompareType::absolute))
            << "point at origin";

  // Per cylinder-half tests (i.e., above and below the z = 0 plane).
  for (S z_sign : {-1, 1}) {
      for (const auto& dir : {Vector3<S>(1, 0, 0),
                              Vector3<S>(0, 1, 0),
                              Vector3<S>(1, 1, 0).normalized(),
                              Vector3<S>(-1, 2, 0).normalized(),
                              Vector3<S>(1, -2, 0).normalized(),
                              Vector3<S>(-2, -3, 0).normalized()}) {
        const Vector3<S> z_offset_internal{0, 0, h * S(0.5) * z_sign};
        const Vector3<S> z_offset_external{0, 0, h * S(1.5) * z_sign};
        const Vector3<S> radial_offset_internal = dir * (r * S(0.5));
        const Vector3<S> radial_offset_external = dir * (r * S(1.5));

        using std::to_string;

        std::stringstream ss;
        ss << "dir: " << dir.transpose() << ", z: " << z_sign;
        const std::string parameters = ss.str();
        // Case: point inside (no clamped values).
        p_CQ = radial_offset_internal + z_offset_internal;
        N_is_not_S = nearestPointInCylinder(h, r, p_CQ, &p_CN);
        EXPECT_FALSE(N_is_not_S) << "Sphere at origin - " << parameters;
        EXPECT_TRUE(
            CompareMatrices(p_CN, p_CQ, 0, MatrixCompareType::absolute))
                  << "Sphere at origin - " << parameters;

        // Case: clamped only by the barrel.
        p_CQ = radial_offset_external + z_offset_internal;
        N_is_not_S = nearestPointInCylinder(h, r, p_CQ, &p_CN);
        EXPECT_TRUE(N_is_not_S)
                  << "Clamped by barrel - " << parameters;
        const Vector3<S> point_on_barrel = z_offset_internal + dir * r;
        EXPECT_NEAR(point_on_barrel(0), p_CN(0), Eps<S>::value())
                  << "Clamped by barrel - " << parameters;
        EXPECT_NEAR(point_on_barrel(1), p_CN(1), Eps<S>::value())
                  << "Clamped by barrel - " << parameters;
        EXPECT_EQ(p_CQ(2), p_CN(2))
                  << "Clamped by barrel - " << parameters;

        // Case: clamped only by the end face.
        p_CQ = radial_offset_internal + z_offset_external;
        N_is_not_S = nearestPointInCylinder(h, r, p_CQ, &p_CN);
        EXPECT_TRUE(N_is_not_S) << "Clamped by end face - " << parameters;
        EXPECT_EQ(p_CQ(0), p_CN(0)) << "Clamped by end face - " << parameters;
        EXPECT_EQ(p_CQ(1), p_CN(1)) << "Clamped by end face - " << parameters;
        EXPECT_EQ(0.5 * h * z_sign, p_CN(2)) << "Clamped by end face - "
                                             << parameters;

        // Case: clamped by both end face and barrel.
        p_CQ = radial_offset_external + z_offset_external;
        N_is_not_S = nearestPointInCylinder(h, r, p_CQ, &p_CN);
        EXPECT_TRUE(N_is_not_S) << "Fully clamped - " << parameters;
        EXPECT_NEAR(point_on_barrel(0), p_CN(0), Eps<S>::value())
                  << "Fully clamped - " << parameters;
        EXPECT_NEAR(point_on_barrel(1), p_CN(1), Eps<S>::value())
                  << "Fully clamped - " << parameters;
        EXPECT_EQ(0.5 * h * z_sign, p_CN(2)) << "Fully clamped - "
                                             << parameters;
      }
  }
}

// Defines the test configuration for a single test. It includes the geometry
// and the pose of the sphere in the cylinder's frame C. It also includes the
// expected answers in that same frame. It does not include those quantities
// that vary from test invocation to invocation (e.g., the pose of the cylinder
// in the world frame or the *orientation* of the sphere).
//
// Collision and distance are complementary queries -- two objects in collision
// have no defined distance because they are *not* separated and vice versa.
// These configurations allow for the test of the complementarity property.
template <typename S>
struct TestConfiguration {
  TestConfiguration(const std::string& name_in, const S& r_c_in,
                    const S& h_c_in, const S& r_s_in,
                    const Vector3<S> &p_CSo_in, bool colliding)
      : name(name_in),
        cylinder_half_len(h_c_in / 2),
        r_c(r_c_in),
        r_s(r_s_in),
        p_CSo(p_CSo_in),
        expected_colliding(colliding) {}

  // Descriptive name of the test configuration.
  std::string name;
  // Half the length of the cylinder along the z-axis.
  S cylinder_half_len;
  // Radius of the cylinder.
  S r_c;
  // Radius of the sphere.
  S r_s;
  // Position of the sphere's center in the cylinder frame.
  Vector3<S> p_CSo;

  // Indicates if this test configuration is expected to be in collision.
  bool expected_colliding{false};

  // Collision values; only valid if expected_colliding is true.
  S expected_depth{-1};
  Vector3<S> expected_normal;
  Vector3<S> expected_pos;

  // Distance values; only valid if expected_colliding is false.
  S expected_distance{-1};
  // The points on sphere and cylinder, respectively, closest to the other shape
  // measured and expressed in the cylinder frame C. Only defined if separated.
  Vector3<S> expected_p_CSc;
  Vector3<S> expected_p_CCs;
};

// Utility for creating a copy of the input configurations and appending more
// labels to the configuration name -- aids in debugging.
template <typename S>
std::vector<TestConfiguration<S>> AppendLabel(
    const std::vector<TestConfiguration<S>>& configurations,
    const std::string& label) {
  std::vector<TestConfiguration<S>> configs;
  for (const auto& config : configurations) {
    configs.push_back(config);
    configs.back().name += " - " + label;
  }
  return configs;
}

// Returns a collection of configurations where sphere and cylinder are similar
// in size.
template <typename S>
std::vector<TestConfiguration<S>> GetUniformConfigurations() {
  std::vector<TestConfiguration<S>> configurations;

  // Common configuration values
  // Cylinder and sphere dimensions.
  const S h_c = 0.6;
  const S half_h_c = h_c / 2;
  const S r_c = 1.2;
  const S r_s = 0.7;
  const bool collides = true;

  // Collection of directions parallel to the z = 0 plane. Used for sampling
  // queries in various directions around the barrel. Note: for the tests to be
  // correct, these normals must all have unit length and a zero z-component.
  std::vector<Vector3<S>> barrel_directions{
      Vector3<S>{1, 0, 0},
      Vector3<S>{0, 1, 0},
      Vector3<S>(1, S(0.5), 0).normalized(),
      Vector3<S>(-1, S(0.5), 0).normalized(),
      Vector3<S>(-1, -S(0.5), 0).normalized(),
      Vector3<S>(1, -S(0.5), 0).normalized()};

  {
    // Case: Completely separated. Nearest point on the +z face.
    const Vector3<S> p_CS{r_c * S(0.25), r_c * S(0.25),
                          half_h_c + r_s * S(1.1)};
    configurations.emplace_back(
        "Separated; nearest face +z", r_c, h_c, r_s, p_CS, !collides);

    TestConfiguration<S>& config = configurations.back();
    // Not colliding --> no collision values.
    config.expected_distance = p_CS(2) - half_h_c - r_s;
    config.expected_p_CCs = Vector3<S>{p_CS(0), p_CS(1), half_h_c};
    config.expected_p_CSc = Vector3<S>{p_CS(0), p_CS(1), p_CS(2) - r_s};
  }

  {
    // Case: Sphere completely separated with center in barrel Voronoi region.
    const S target_distance = 0.1;
    const Vector3<S> n_SC = Vector3<S>{-1, -1, 0}.normalized();
    const Vector3<S> p_CS = Vector3<S>{0, 0, half_h_c * S(0.5)} -
        n_SC * (r_s + r_c + target_distance);
    configurations.emplace_back(
        "Separated; near barrel", r_c, h_c, r_s, p_CS, !collides);

    TestConfiguration<S>& config = configurations.back();
    // Not colliding --> no collision values.
    config.expected_distance = target_distance;
    config.expected_p_CCs = -n_SC * r_c + Vector3<S>{0, 0, p_CS(2)};
    config.expected_p_CSc = p_CS + n_SC * r_s;
  }

  {
    // Case: Sphere completely separated with center in *edge* Voronoi region.
    const S target_distance =  .1;
    const Vector3<S> n_SC = Vector3<S>{-1, -1, -1}.normalized();
    const Vector3<S> p_CCs = Vector3<S>{0, 0, half_h_c} +
        Vector3<S>{-n_SC(0), -n_SC(1), 0}.normalized() * r_c;
    const Vector3<S> p_CS = p_CCs - n_SC * (r_s + target_distance);
    configurations.emplace_back(
        "Separated; near barrel edge", r_c, h_c, r_s, p_CS, !collides);

    TestConfiguration<S>& config = configurations.back();
    // Not colliding --> no collision values.
    config.expected_distance = target_distance;
    config.expected_p_CCs = p_CCs;
    config.expected_p_CSc = p_CS + n_SC * r_s;
  }

  using std::min;
  const S target_depth = min(r_c, r_s) * S(0.25);

  // Case contact - sphere center outside cylinder.
  // Sub-cases intersection *only* through cap face; one test for each face.
  for (S sign : {S(-1), S(1)}) {
    const Vector3<S> n_SC = Vector3<S>::UnitZ() * -sign;
    const Vector3<S> p_CCs = Vector3<S>{r_c * S(0.25), r_c * S(0.25),
                                        half_h_c * sign};
    const Vector3<S> p_CS = p_CCs - n_SC * (r_s - target_depth);
    configurations.emplace_back(
        "Colliding external sphere center; cap face in " +
            (sign < 0 ? std::string("-z") : std::string("+z")),
        r_c, h_c, r_s, p_CS, collides);

    TestConfiguration<S>& config = configurations.back();
    config.expected_depth = target_depth;
    config.expected_normal = n_SC;
    config.expected_pos = p_CCs + n_SC * (target_depth / 2);
    // Colliding; no distance values required.
  }

  // Sub-cases intersection *only* through barrel. Sampled in multiple
  // directions.
  for (const Vector3<S>& n_CS : barrel_directions) {
    const Vector3<S> p_CCs = Vector3<S>{0, 0, half_h_c * S(.1)} +
        n_CS * r_c;
    const Vector3<S> p_CS = p_CCs + n_CS * (r_s - target_depth);
    std::stringstream ss;
    ss << "Colliding external sphere center; barrel from sphere center in"
       << n_CS.transpose() << " direction";
    configurations.emplace_back(ss.str(), r_c, h_c, r_s, p_CS, collides);

    TestConfiguration<S>& config = configurations.back();
    config.expected_depth = target_depth;
    config.expected_normal = -n_CS;
    config.expected_pos = p_CCs - n_CS * (target_depth / 2);
    // Colliding; no distance values required.
  }

  // Sub-cases intersection through edge.
  for (S sign : {S(-1), S(1)}) {
    // Projection of vector from cylinder center to sphere center on the z=0
    // plane (and then normalized).
    for (const Vector3<S>& n_CS_xy : barrel_directions) {
      const Vector3<S> p_CCs = Vector3<S>{0, 0, sign * half_h_c} +
          n_CS_xy * r_c;
      const Vector3<S> n_CS = p_CCs.normalized();
      const Vector3<S> p_CS = p_CCs + n_CS * (r_s - target_depth);
      std::stringstream ss;
      ss << "Colliding external sphere center; edge from sphere center in"
         << n_CS.transpose() << " direction";
      configurations.emplace_back(ss.str(), r_c, h_c, r_s, p_CS, collides);

      TestConfiguration<S>& config = configurations.back();
      config.expected_depth = target_depth;
      config.expected_normal = -n_CS;
      config.expected_pos = p_CCs - n_CS * (target_depth / 2);
      // Colliding; no distance values required.
    }
  }

  // Case contact - sphere center *inside* cylinder.

  // Sub-cases: sphere is unambiguously closest to end face. One test for each
  // end face.
  for (S sign : {S(-1), S(1)}) {
    // Distance from sphere center S to face F.
    const S d_SF = 0.1;
    const Vector3<S> n_SC = Vector3<S>::UnitZ() * -sign;
    const Vector3<S> p_CCs = Vector3<S>{r_c * S(0.25), r_c * S(0.25),
                                        half_h_c * sign};
    const Vector3<S> p_CS = p_CCs + n_SC * d_SF;
    configurations.emplace_back(
        "Colliding internal sphere center; cap face in " +
            (sign < 0 ? std::string("-z") : std::string("+z")),
        r_c, h_c, r_s, p_CS, collides);

    TestConfiguration<S>& config = configurations.back();
    config.expected_depth = d_SF + r_s;
    config.expected_normal = n_SC;
    config.expected_pos = p_CCs + n_SC * (config.expected_depth / 2);
    // Colliding; no distance values required.
  }

  // Sub-cases: sphere is unambiguously closest to barrel; sampling multiple
  // directions.
  for (const Vector3<S>& n_CS : barrel_directions) {
    // Distance from sphere center S to point B on barrel.
    const S d_SB = 0.1;
    const Vector3<S> p_CCs = Vector3<S>{0, 0, half_h_c * S(.1)} +
        n_CS * r_c;
    const Vector3<S> p_CS = p_CCs - n_CS * d_SB;
    std::stringstream ss;
    ss << "Colliding internal sphere center; barrel from sphere located in "
       << n_CS.transpose() << " direction";
    configurations.emplace_back(ss.str(), r_c, h_c, r_s, p_CS, collides);

    TestConfiguration<S>& config = configurations.back();
    config.expected_depth = r_s + d_SB;
    config.expected_normal = -n_CS;
    config.expected_pos = p_CCs - n_CS * (config.expected_depth / 2);
    // Colliding; no distance values required.
  }

  // Case contact - sphere center is *near* error-dominated regions

  // Sub-case: Sphere center is within epsilon *outside* of end face.
  // Numerically, this is processed as if the center were inside the cylinder.
  // For face contact, there's no difference. This test subsumes the test where
  // the center lies *on* the surface of the cylinder.
  for (S sign : {S(-1), S(1)}) {
    // Distance from sphere center S to face F.
    const S d_SF = Eps<S>::value() / 2;
    const Vector3<S> n_SC = Vector3<S>::UnitZ() * -sign;
    const Vector3<S> p_CCs = Vector3<S>{r_c * S(0.25), r_c * S(0.25),
                                        half_h_c * sign};
    const Vector3<S> p_CS = p_CCs - n_SC * d_SF;
    configurations.emplace_back(
        "Colliding sphere center outside by ε; cap face in " +
            (sign < 0 ? std::string("-z") : std::string("+z")),
        r_c, h_c, r_s, p_CS, collides);

    TestConfiguration<S>& config = configurations.back();
    config.expected_depth = r_s;
    config.expected_normal = n_SC;
    config.expected_pos = p_CCs + n_SC * (config.expected_depth / 2);
    // Colliding; no distance values required.
  }

  // Sub-case: Sphere center is within epsilon *outside* of barrel.
  // Numerically, this is processed as if the center were inside the cylinder.
  // For barrel contact, there's no difference. This test subsumes the test
  // where the center lies *on* the surface of the cylinder.
  for (const Vector3<S>& n_CS : barrel_directions) {
    // Distance from sphere center S to point B on barrel.
    const S d_SB = Eps<S>::value() / 2;
    const Vector3<S> p_CCs = Vector3<S>{0, 0, half_h_c * S(.1)} +
        n_CS * r_c;
    const Vector3<S> p_CS = p_CCs - n_CS * d_SB;
    std::stringstream ss;
    ss << "Colliding sphere center outside by ε; barrel from sphere located in "
       << n_CS.transpose() << " direction";
    configurations.emplace_back(ss.str(), r_c, h_c, r_s, p_CS, collides);

    TestConfiguration<S>& config = configurations.back();
    config.expected_depth = r_s;
    config.expected_normal = -n_CS;
    config.expected_pos = p_CCs - n_CS * (config.expected_depth / 2);
    // Colliding; no distance values required.
  }

  // Sub-case: Sphere center is within epsilon *outside* of edge.
  // Numerically, this is processed as if the center were inside the cylinder.
  // If the center is in the Voronoi region of the edge, the reported normal
  // will be either the face or the barrel -- whichever is closer. In this
  // configuration, it is the face normal. This test subsumes the test where
  // the center lies *on* the surface of the cylinder.
  for (S sign : {S(-1), S(1)}) {
    // Projection of vector from cylinder center to sphere center on the z=0
    // plane (and then normalized).
    const S d_SC = Eps<S>::value() / 2;
    for (const Vector3<S>& n_CS_xy : barrel_directions) {
      const Vector3<S> p_CCs = Vector3<S>{0, 0, sign * half_h_c} +
          n_CS_xy * r_c;
      const Vector3<S> n_CS = p_CCs.normalized();
      const Vector3<S> p_CS = p_CCs + n_CS * d_SC;
      std::stringstream ss;
      ss << "Colliding sphere center outside by ε; edge from sphere center in"
         << n_CS.transpose() << " direction";
      configurations.emplace_back(ss.str(), r_c, h_c, r_s, p_CS, collides);

      TestConfiguration<S>& config = configurations.back();
      config.expected_depth = r_s;
      // NOTE: Epsilon *outside* is considered inside so the normal direction
      // will be either face or barrel -- and, in this case, it's face.
      config.expected_normal = -sign * Vector3<S>::UnitZ();
      config.expected_pos = p_CCs + config.expected_normal * (r_s / 2);
      // Colliding; no distance values required.
    }
  }

  {
    // Sub-case: Sphere center is on origin - face is closer. It should prefer
    // the +z face.
    const Vector3<S> p_CS = Vector3<S>::Zero();
    // Guarantee that the barrel is farther than the face.
    const S big_radius = h_c * 2;
    configurations.emplace_back(
        "Collision with sphere at origin; face nearest", big_radius, h_c, r_s,
        p_CS, collides);

    TestConfiguration<S>& config = configurations.back();
    config.expected_depth = r_s + h_c / 2;
    config.expected_normal = -Vector3<S>::UnitZ();
    config.expected_pos = Vector3<S>{0, 0, h_c / 2 - config.expected_depth / 2};
    // Colliding; no distance values required.
  }

  {
    // Sub-case: Sphere center is on origin - barrel is closer.
    const Vector3<S> p_CS = Vector3<S>::Zero();
    // Guarantee that the barrel is closer than the face.
    const S big_height = r_c * 4;
    configurations.emplace_back(
        "Collision with sphere at origin; barrel nearest", r_c, big_height, r_s,
        p_CS, collides);

    TestConfiguration<S>& config = configurations.back();
    config.expected_depth = r_s + r_c;
    config.expected_normal = -Vector3<S>::UnitX();
    config.expected_pos = Vector3<S>{r_c - config.expected_depth / 2, 0, 0};
    // Colliding; no distance values required.
  }

  return configurations;
}

// Returns a collection of configurations where sphere and cylinder are scaled
// very differently.
template <typename S>
std::vector<TestConfiguration<S>> GetNonUniformConfigurations() {
  std::vector<TestConfiguration<S>> configurations;

  // Case: Large, flat cylinder and tiny sphere.
  {
    const S r_c = 9;
    const S h_c = 0.1;
    const S r_s = 0.025;
    const bool collides = true;
    const S target_depth = r_s / 2;

    // Sub-case: Colliding -- contact with +z face.
    {
      // Colliding sub-case.
      const Vector3<S> p_CCs = Vector3<S>(1, 2, 0).normalized() * (r_c - r_s) +
          Vector3<S>::UnitZ() * (h_c / 2);
      const Vector3<S> p_CS{p_CCs + Vector3<S>::UnitZ() * (r_s - target_depth)};
      configurations.emplace_back(
          "Collision large disk, small sphere - contact at face",
          r_c, h_c, r_s, p_CS, collides);

      TestConfiguration<S>& config = configurations.back();
      config.expected_depth = target_depth;
      config.expected_normal = -Vector3<S>::UnitZ();
      config.expected_pos = p_CCs - Vector3<S>::UnitZ() * (target_depth / 2);
      // Colliding; no distance values required.
    }

    // Sub-case: Separated -- nearest feature +z face.
    {
      // Separated sub-case.
      const Vector3<S> p_CCs = Vector3<S>(1, 2, 0).normalized() * (r_c - r_s) +
          Vector3<S>::UnitZ() * (h_c / 2);
      const Vector3<S> p_CS{p_CCs +
          Vector3<S>::UnitZ() * (r_s + target_depth)};
      configurations.emplace_back(
          "Separation large disk, small sphere - nearest +z face",
          r_c, h_c, r_s, p_CS, !collides);

      TestConfiguration<S>& config = configurations.back();
      // Not colliding --> no collision values.
      config.expected_distance = target_depth;
      config.expected_p_CCs = p_CCs;
      config.expected_p_CSc = p_CS - Vector3<S>::UnitZ() * r_s;
    }

    // Sub-case: Colliding -- contact with barrel.
    const Vector3<S> n_CS = Vector3<S>(1, 2, 0).normalized();
    const Vector3<S> p_CCs = n_CS * r_c + Vector3<S>::UnitZ() * (r_s * 0.1);
    {
      // Colliding sub-case.
      const Vector3<S> p_CS{p_CCs + n_CS * (r_s - target_depth)};
      configurations.emplace_back(
          "Collision large disk, small sphere - contact at barrel",
          r_c, h_c, r_s, p_CS, collides);

      TestConfiguration<S>& config = configurations.back();
      config.expected_depth = target_depth;
      config.expected_normal = -n_CS;
      config.expected_pos = p_CCs - n_CS * (target_depth / 2);
      // Colliding; no distance values required.
    }

    // Sub-case: Separated -- nearest feature is barrel.
    {
      // Separated sub-case.
      const Vector3<S> p_CS{p_CCs + n_CS * (r_s + target_depth)};
      configurations.emplace_back(
          "Separation large disk, small sphere - nearest barrel",
          r_c, h_c, r_s, p_CS, !collides);

      TestConfiguration<S>& config = configurations.back();
      // Not colliding --> no collision values.
      config.expected_distance = target_depth;
      config.expected_p_CCs = p_CCs;
      config.expected_p_CSc = p_CS - n_CS * r_s;
    }
  }

  // Case: Large sphere and *tiny* cylinder.
  {
    const S r_c = 0.025;
    const S h_c = 0.1;
    const S r_s = 9;
    const bool collides = true;
    const S target_depth = r_c / 2;

    // Sub-case -- nearest feature is +z face.
    {
      const Vector3<S> p_CCs =
          Vector3<S>(1, 2, 0).normalized() * (r_c * S(0.5)) +
              Vector3<S>::UnitZ() * (h_c / 2);

      // Sub-case: Colliding.
      {
        const Vector3<S>
            p_CS{p_CCs + Vector3<S>::UnitZ() * (r_s - target_depth)};
        configurations.emplace_back(
            "Collision large sphere, small disk - contact at face",
            r_c, h_c, r_s, p_CS, collides);

        TestConfiguration<S>& config = configurations.back();
        config.expected_depth = target_depth;
        config.expected_normal = -Vector3<S>::UnitZ();
        config.expected_pos = p_CCs - Vector3<S>::UnitZ() * (target_depth / 2);
        // Colliding; no distance values required.
      }

      // Subsub-case: Separated
      {
        const Vector3<S> p_CS{p_CCs +
            Vector3<S>::UnitZ() * (r_s + target_depth)};
        configurations.emplace_back(
            "Separation large sphere, small disk - nearest +z face",
            r_c, h_c, r_s, p_CS, !collides);

        TestConfiguration<S>& config = configurations.back();
        // Not colliding --> no collision values.
        config.expected_distance = target_depth;
        config.expected_p_CCs = p_CCs;
        config.expected_p_CSc = p_CS - Vector3<S>::UnitZ() * r_s;
      }
    }

    // Sub-case: Nearest feature is barrel
    {
      const Vector3<S> n_CS = Vector3<S>(1, 2, 0).normalized();
      const Vector3<S> p_CCs = n_CS * r_c + Vector3<S>::UnitZ() * (h_c * 0.1);

      // Subsub-case: Colliding.
      {
        const Vector3<S> p_CS{p_CCs + n_CS * (r_s - target_depth)};
        configurations.emplace_back(
            "Collision large sphere, small disk - contact at barrel",
            r_c, h_c, r_s, p_CS, collides);

        TestConfiguration<S>& config = configurations.back();
        config.expected_depth = target_depth;
        config.expected_normal = -n_CS;
        config.expected_pos = p_CCs - n_CS * (target_depth / 2);
        // Colliding; no distance values required.
      }

      // Subsub-case: Separated .
      {
        const Vector3<S> p_CS{p_CCs + n_CS * (r_s + target_depth)};
        configurations.emplace_back(
            "Separation large sphere, small disk - nearest barrel",
            r_c, h_c, r_s, p_CS, !collides);

        TestConfiguration<S>& config = configurations.back();
        // Not colliding --> no collision values.
        config.expected_distance = target_depth;
        config.expected_p_CCs = p_CCs;
        config.expected_p_CSc = p_CS - n_CS * r_s;
      }
    }
  }

  return configurations;
}

template <typename S>
using EvalFunc =
std::function<void(const TestConfiguration<S>&, const Transform3<S>&,
                   const Matrix3<S>&, S)>;

// This evaluates an instance of a test configuration and confirms the results
// match the expected data. The test configuration is defined in the cylinder's
// frame with an unrotated sphere. The parameters provide the test parameters:
// the sphere orientation and the cylinder's pose in the world frame.
//
// Evaluates the collision query twice. Once as the boolean "is colliding" test
// and once with the collision characterized with depth, normal, and position.
template <typename S>
void EvalCollisionForTestConfiguration(const TestConfiguration<S>& config,
                                       const Transform3<S>& X_WC,
                                       const Matrix3<S>& R_CS,
                                       S eps) {
  // Set up the experiment from input parameters and test configuration.
  Cylinder<S> cylinder(config.r_c, config.cylinder_half_len * S(2));
  Sphere<S> sphere{config.r_s};
  Transform3<S> X_CS = Transform3<S>::Identity();
  X_CS.translation() = config.p_CSo;
  X_CS.linear() = R_CS;
  Transform3<S> X_WS = X_WC * X_CS;

  bool colliding = sphereCylinderIntersect<S>(sphere, X_WS, cylinder, X_WC,
                                              nullptr);
  EXPECT_EQ(config.expected_colliding, colliding) << config.name;

  std::vector<ContactPoint<S>> contacts;
  colliding = sphereCylinderIntersect<S>(sphere, X_WS, cylinder, X_WC, &contacts);
  EXPECT_EQ(colliding, config.expected_colliding) << config.name;
  if (config.expected_colliding) {
    EXPECT_EQ(1u, contacts.size()) << config.name;
    const ContactPoint<S> &contact = contacts[0];
    EXPECT_NEAR(config.expected_depth, contact.penetration_depth, eps)
              << config.name;
    EXPECT_TRUE(CompareMatrices(contact.normal,
                                X_WC.linear() * config.expected_normal, eps,
                                MatrixCompareType::absolute))
              << config.name;
    EXPECT_TRUE(CompareMatrices(contact.pos, X_WC * config.expected_pos, eps,
                                MatrixCompareType::absolute))
              << config.name;
  } else {
    EXPECT_EQ(contacts.size(), 0u) << config.name;
  }
}

// This evaluates an instance of a test configuration and confirms the results
// match the expected data. The test configuration is defined in the cylinder's
// frame with an unrotated sphere. The parameters provide the test
// configuration.
//
// Evaluates the distance query twice. Once as the boolean "is separated" test
// and once with the separation characterized with distance and surface points.
template <typename S>
void EvalDistanceForTestConfiguration(const TestConfiguration<S>& config,
                                      const Transform3<S>& X_WC,
                                      const Matrix3<S>& R_CS,
                                      S eps) {
  // Set up the experiment from input parameters and test configuration.
  Cylinder<S> cylinder(config.r_c, config.cylinder_half_len * S(2));
  Sphere<S> sphere{config.r_s};
  Transform3<S> X_CS = Transform3<S>::Identity();
  X_CS.translation() = config.p_CSo;
  X_CS.linear() = R_CS;
  Transform3<S> X_WS = X_WC * X_CS;

  bool separated = sphereCylinderDistance<S>(sphere, X_WS, cylinder, X_WC,
                                             nullptr, nullptr, nullptr);
  EXPECT_NE(separated, config.expected_colliding) << config.name;

  // Initializing this to -2, to confirm that a non-colliding scenario sets
  // distance to -1.
  S distance{-2};
  Vector3<S> p_WSc{0, 0, 0};
  Vector3<S> p_WCs{0, 0, 0};

  separated = sphereCylinderDistance<S>(sphere, X_WS, cylinder, X_WC, &distance,
                                        &p_WSc, &p_WCs);
  EXPECT_NE(separated, config.expected_colliding) << config.name;
  if (!config.expected_colliding) {
    EXPECT_NEAR(distance, config.expected_distance, eps)
              << config.name;
    EXPECT_TRUE(CompareMatrices(p_WSc,
                                X_WC * config.expected_p_CSc, eps,
                                MatrixCompareType::absolute))
              << config.name;
    EXPECT_TRUE(CompareMatrices(p_WCs,
                                X_WC * config.expected_p_CCs, eps,
                                MatrixCompareType::absolute))
              << config.name;
  } else {
    EXPECT_EQ(distance, S(-1)) << config.name;
    EXPECT_TRUE(CompareMatrices(p_WSc, Vector3<S>::Zero(), 0,
                                MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(p_WCs, Vector3<S>::Zero(), 0,
                                MatrixCompareType::absolute));
  }
}

// This test defines the transforms for performing the single collision test.
template <typename S>
void QueryWithVaryingWorldFrames(
    const std::vector<TestConfiguration<S>>& configurations,
    EvalFunc<S> query_eval, const Matrix3<S>& R_CS = Matrix3<S>::Identity()) {
  // Evaluate all the configurations with the given cylinder pose in frame F.
  auto evaluate_all = [&R_CS, query_eval](
      const std::vector<TestConfiguration<S>>& configs,
      const Transform3<S>& X_FC) {
    for (const auto config : configs) {
      query_eval(config, X_FC, R_CS, Eps<S>::value());
    }
  };

  // Frame F is coincident and aligned with the cylinder frame.
  Transform3<S> X_FC = Transform3<S>::Identity();
  evaluate_all(AppendLabel(configurations, "X_FC = I"), X_FC);

  // Simple arbitrary translation away from the origin.
  X_FC.translation() << 1.3, 2.7, 6.5;
  evaluate_all(AppendLabel(configurations, "X_FC is translation"), X_FC);

  std::string axis_name[] = {"x", "y", "z"};
  // 90 degree rotation around each axis.
  for (int axis = 0; axis < 3; ++axis) {
    std::string label = "X_FC is 90-degree rotation around " + axis_name[axis];
    AngleAxis<S> angle_axis{constants<S>::pi() / 2, Vector3<S>::Unit(axis)};
    X_FC.linear() << angle_axis.matrix();
    evaluate_all(AppendLabel(configurations, label), X_FC);
  }

  // Arbitrary orientation.
  {
    AngleAxis<S> angle_axis{constants<S>::pi() / 3,
                            Vector3<S>{1, 2, 3}.normalized()};
    X_FC.linear() << angle_axis.matrix();
    evaluate_all(AppendLabel(configurations, "X_FC is arbitrary rotation"),
                 X_FC);
  }

  // Near axis aligned.
  {
    AngleAxis<S> angle_axis{constants<S>::eps_12(), Vector3<S>::UnitX()};
    X_FC.linear() << angle_axis.matrix();
    evaluate_all(AppendLabel(configurations, "X_FC is near identity"),
                 X_FC);
  }
}

// Runs all test configurations across multiple poses in the world frame --
// changing the orientation of the sphere -- should have no affect on the
// results.
template <typename S>
void QueryWithOrientedSphere(
    const std::vector<TestConfiguration<S>>& configurations,
    EvalFunc<S> query_eval) {

  std::string axis_name[] = {"x", "y", "z"};

  // 90 degree rotation around each axis.
  for (int axis = 0; axis < 3; ++axis) {
    AngleAxis<S> angle_axis{constants<S>::pi() / 2, Vector3<S>::Unit(axis)};
    std::string label = "sphere rotate 90-degrees around " + axis_name[axis];
    QueryWithVaryingWorldFrames<S>(AppendLabel(configurations, label),
                                   query_eval, angle_axis.matrix());
  }

  // Arbitrary orientation.
  {
    AngleAxis<S> angle_axis{constants<S>::pi() / 3,
                            Vector3<S>{1, 2, 3}.normalized()};
    std::string label = "sphere rotated arbitrarily";
    QueryWithVaryingWorldFrames<S>(AppendLabel(configurations, label),
                                   query_eval, angle_axis.matrix());
  }

  // Near axis aligned.
  {
    AngleAxis<S> angle_axis{constants<S>::eps_12(), Vector3<S>::UnitX()};
    std::string label = "sphere rotated near axes";
    QueryWithVaryingWorldFrames<S>(AppendLabel(configurations, label),
                                   query_eval, angle_axis.matrix());
  }
}

//======================================================================

// Tests the helper function that finds the closest point in the cylinder.
GTEST_TEST(SphereCylinderPrimitiveTest, NearestPointInCylinder) {
  NearestPointInCylinder<float>();
  NearestPointInCylinder<double>();
}

// Evaluates collision on all test configurations across multiple poses in the
// world frame - but the sphere rotation is always the identity.
GTEST_TEST(SphereCylinderPrimitiveTest, CollisionAcrossVaryingWorldFrames) {
  QueryWithVaryingWorldFrames<float>(GetUniformConfigurations<float>(),
                                     EvalCollisionForTestConfiguration<float>);
  QueryWithVaryingWorldFrames<double>(
      GetUniformConfigurations<double>(),
      EvalCollisionForTestConfiguration<double>);
}

// Evaluates collision on all test configurations across multiple poses in the
// world frame - the sphere is rotated arbitrarily.
GTEST_TEST(SphereCylinderPrimitiveTest, CollisionWithSphereRotations) {
  QueryWithOrientedSphere<float>(GetUniformConfigurations<float>(),
                                 EvalCollisionForTestConfiguration<float>);
  QueryWithOrientedSphere<double>(GetUniformConfigurations<double>(),
                                  EvalCollisionForTestConfiguration<double>);
}

// Evaluates collision on a small set of configurations where the cylinder and
// sphere are of radically different scales - evaluation across multiple poses
// in the world frame.
GTEST_TEST(SphereCylinderPrimitiveTest, CollisionIncompatibleScales) {
  QueryWithVaryingWorldFrames<float>(GetNonUniformConfigurations<float>(),
                                     EvalCollisionForTestConfiguration<float>);
  QueryWithVaryingWorldFrames<double>(
      GetNonUniformConfigurations<double>(),
      EvalCollisionForTestConfiguration<double>);
}

// Evaluates distance on all test configurations across multiple poses in the
// world frame - but the sphere rotation is always the identity.
GTEST_TEST(SphereCylinderPrimitiveTest, DistanceAcrossVaryingWorldFrames) {
  QueryWithVaryingWorldFrames<float>(GetUniformConfigurations<float>(),
                                     EvalDistanceForTestConfiguration<float>);
  QueryWithVaryingWorldFrames<double>(GetUniformConfigurations<double>(),
                                      EvalDistanceForTestConfiguration<double>);
}

// Evaluates distance on all test configurations across multiple poses in the
// world frame - the sphere is rotated arbitrarily.
GTEST_TEST(SphereCylinderPrimitiveTest, DistanceWithSphereRotations) {
  QueryWithOrientedSphere<float>(GetUniformConfigurations<float>(),
                                 EvalDistanceForTestConfiguration<float>);
  QueryWithOrientedSphere<double>(GetUniformConfigurations<double>(),
                                  EvalDistanceForTestConfiguration<double>);
}

// Evaluates distance on a small set of configurations where the cylinder and
// sphere are of radically different scales - evaluation across multiple poses
// in the world frame.
GTEST_TEST(SphereCylinderPrimitiveTest, DistanceIncompatibleScales) {
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

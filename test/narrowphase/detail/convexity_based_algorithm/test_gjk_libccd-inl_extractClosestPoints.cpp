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

#include "fcl/narrowphase/detail/convexity_based_algorithm/gjk_libccd-inl.h"

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "fcl/common/types.h"

namespace fcl {
namespace detail {

using Vector3d = Vector3<double>;

// Helper functions to work with libccd types -- the ccd_vec3_t is a nuisance.
// This allows working with Eigen Vector3d's instead.

ccd_vec3_t eigen_to_ccd(const Vector3d& vector) {
  ccd_vec3_t out;
  out.v[0] = vector(0);
  out.v[1] = vector(1);
  out.v[2] = vector(2);
  return out;
}

Vector3d ccd_to_eigen(const ccd_vec3_t& vector) {
  // TODO(SeanCurtis-TRI): When libccd is *always* double precision, this can
  // become: `return Vector3d{vector.v};`
  return Vector3d{vector.v[0], vector.v[1], vector.v[2]};
}

// Tests that the given vector and libccd vector are the same with in a given
// tolerance.
::testing::AssertionResult are_same(const Vector3d& expected,
                                    const ccd_vec3_t& tested,
                                    double tolerance) {
  if (tolerance < 0) {
    return ::testing::AssertionFailure() << "Invalid tolerance: "
                                         << tolerance;
  }

  Vector3d ccd = ccd_to_eigen(tested);
  Vector3d error = (expected - ccd).cwiseAbs();

  for (int i = 0; i < 3; ++i) {
    if (error(i) > tolerance) {
      return ::testing::AssertionFailure()
          << "Values at index " << i
          << " exceed tolerance; " << expected(i) << " vs "
          << ccd(i) << ", diff = " << error(i)
          << ", tolerance = " << tolerance << "\nexpected = "
          << expected.transpose() << "\ntested = "
          << ccd.transpose() << "\n|delta| = "
          << error.transpose();
    }
  }
  return ::testing::AssertionSuccess();
}

// These are tests on the extractClosestPoints() function (and its underlying
// support functions).
//
// These functions map a simplex and a point of interest on the "surface" of the
// simplex into a pair of points. The simplex is defined by support vertices
// (ccd_support_t). Each support vertex contains three vectors in R3. They
// represent:
//   1. A vertex `v` on the boundary of Minkowski difference of two objects:
//      O₁ ⊕ -O₂. It is, by definition the difference of two vertex positions
//      on the two objects: `v = v₁ - v₂`.
//   2. The vertex `v₁` on O₁ that contributes to the definition of `v`.
//   3. The vertex `v₂` on O₂ that contributes to the definition of `v`.
//
// extractClosestPoints() supports 1-, 2- and 3-simplices.
//
// The corresponding points are written to output parameters. If the output
// parameters are null, the value is not computed. These tests exercise all
// permutations of the two output parameters.

// Simple wrapper to facilitate working with Eigen primitives in place of libccd
// primitives.
bool are_coincident(const Vector3d& p, const Vector3d& q) {
  return libccd_extension::are_coincident(eigen_to_ccd(p), eigen_to_ccd(q));
}

// Tests the `are_coincident` function.
GTEST_TEST(DegenerateGeometry, CoincidentPoints) {
  // The coincidence test uses this as the threshold.
  const ccd_real_t eps = constants<ccd_real_t>::eps();
  const ccd_real_t almost_eps = eps * 0.75;
  const ccd_real_t extra_eps = eps * 1.5;

  Vector3d p{1, 1, 1};
  Vector3d q{p};
  // Exact coincidence at unit scale (down to the last bit)
  EXPECT_TRUE(are_coincident(p, q));

  // Coincidence within a unit-scaled epsilon.
  EXPECT_TRUE(are_coincident(p + Vector3d{almost_eps, 0, 0}, q));
  EXPECT_TRUE(are_coincident(p + Vector3d{0, almost_eps, 0}, q));
  EXPECT_TRUE(are_coincident(p + Vector3d{0, 0, almost_eps}, q));

  // Distinct points just outside a unit-scaled epsilon.
  EXPECT_FALSE(are_coincident(p + Vector3d{extra_eps, 0, 0}, q));
  EXPECT_FALSE(are_coincident(p + Vector3d{0, extra_eps, 0}, q));
  EXPECT_FALSE(are_coincident(p + Vector3d{0, 0, extra_eps}, q));

  // Coincidence within a larger-than-unit-scale scale factor.
  const double scale = 100;
  p << scale, scale, scale;
  q = p;
  // Exact coincidence at a larger-than-unit-scale (down to the last bit).
  EXPECT_TRUE(are_coincident(p, q));

  // Coincidence within a larger-than-unit-scale epsilon.
  EXPECT_TRUE(are_coincident(p + Vector3d{almost_eps * scale, 0, 0}, q));
  EXPECT_TRUE(are_coincident(p + Vector3d{0, almost_eps * scale, 0}, q));
  EXPECT_TRUE(are_coincident(p + Vector3d{0, 0, almost_eps * scale}, q));

  // Distinct points just outside a larger-than-unit-scaled epsilon.
  EXPECT_FALSE(are_coincident(p + Vector3d{extra_eps * scale, 0, 0}, q));
  EXPECT_FALSE(are_coincident(p + Vector3d{0, extra_eps * scale, 0}, q));
  EXPECT_FALSE(are_coincident(p + Vector3d{0, 0, extra_eps * scale}, q));

  // Coincidence within a smaller-than-unit-scale scale factor. NOTE: eps
  // stays at an *absolute* tolerance as the scale gets smaller.
  p << 0.01, 0.01, 0.01;
  q = p;
  // Exact coincidence at a smaller-than-unit-scale (down to the last bit).
  EXPECT_TRUE(are_coincident(p, q));

  // Coincidence within a smaller-than-unit-scale epsilon.
  EXPECT_TRUE(are_coincident(p + Vector3d{almost_eps, 0, 0}, q));
  EXPECT_TRUE(are_coincident(p + Vector3d{0, almost_eps, 0}, q));
  EXPECT_TRUE(are_coincident(p + Vector3d{0, 0, almost_eps}, q));

  // Distinct points just outside a smaller-than-unit-scaled epsilon.
  EXPECT_FALSE(are_coincident(p + Vector3d{extra_eps, 0, 0}, q));
  EXPECT_FALSE(are_coincident(p + Vector3d{0, extra_eps, 0}, q));
  EXPECT_FALSE(are_coincident(p + Vector3d{0, 0, extra_eps}, q));
}

// Wrapper to allow invocation of triangle_area_is_zero with eigen primitives.
bool triangle_area_is_zero(const Vector3d& a, const Vector3d& b,
                           const Vector3d& c) {
  return fcl::detail::libccd_extension::triangle_area_is_zero(eigen_to_ccd(a),
                                                              eigen_to_ccd(b),
                                                              eigen_to_ccd(c));
}

// Tests the `triangle_area_is_zero()` function. NOTE: This computation
// makes use of `are_coincident()`. Only a single test of coincident
// vertices is provided (to exercise the code path). The permutations for how
// two points can be considered coincident are done in their own tests.
GTEST_TEST(DegenerateGeometry, ZeroAreaTriangle) {
  using std::asin;
  Vector3d a{0, 0, 0};
  Vector3d b{1, 0, 0};
  Vector3d c{0, 1, 0};

  // Viable triangle
  EXPECT_FALSE(triangle_area_is_zero(a, b, c));

  // Triangle with token coincident vertices to make sure it keys on it.
  EXPECT_TRUE(triangle_area_is_zero(a, a, c));

  // Co-linearity tests.

  // Obvious co-linearity
  EXPECT_TRUE(triangle_area_is_zero(a, b, 3 * b));

  // Test co-linearity
  // The co-linear threshold is based on the angles of the triangle. If the
  // triangle has an angle θ, such that |sin(θ)| < ε, then the triangle is
  // considered to be degenerate. This condition implicitly defines an envelope
  // around θ. We define θₑ such that sin(θₑ) = ε. The condition |sin(θ)| < ε
  // implies |θ| < θₑ. So, if the smallest angle is less than θₑ, the triangle
  // will be considered co-linear. If the smallest angle is larger than θₑ, the
  // triangle is considered valid.
  //
  // The test wants to provide proof as to the boundary of the tolerance
  // envelope. Therefore, it will construct two triangles. One whose smallest
  // angle is θₑ - δ and the other triangle's smallest angle is θₑ + δ, for
  // some suitably small δ.
  //
  // The `triangle_area_is_zero()` function has a second failure condition:
  // coincident points. We'll make sure that the points are all far enough away
  // from each other that this failure condition won't be triggered.
  //
  // The triangle consists of three vertices: A, B, C.
  //   A: [0, 0, 0] (for simplicity).
  //   B: [1, 1, 1] (again, for simplicity).
  //   C: We must define C such that the angle ∠CAB is θ = θₑ ± δ.
  //
  // We'll do it by construction:
  //   1. Pick a vector v perpendicular to [1, 1, 1] (e.g., v = [1, 1, -2]).
  //   2. C' = B + v̂ * |B|·tan(θ) (with v̂ = v / |v|). This produces a point C'
  //      that:
  //     a) forms a triangle where ∠C`AB < θₑ, if θ < θₑ, but
  //     b) the points B and C' *may* be coincident.
  //   3. Move the point C' farther away (in the AC' direction). This preserves
  //      the angle, but distances B from C'. So, C ≙ s·C' (for some scalar
  //      s ≠ 1). This trick works because A is the origin and we generally
  //      assume that |C'| >> ε.
  //
  // This triangle illustrates the construction (but rotated for ascii art).
  //
  //          A
  //         /|  θ = ∠BAC = ∠BAC'
  //        / |
  //     C'/__|
  //      /  / B
  //     / /
  //    //
  //    C

  const ccd_real_t eps = constants<ccd_real_t>::eps();
  const ccd_real_t theta_e = asin(eps);

  a << 0, 0, 0;
  b << 1, 1, 1;
  const Vector3d v_hat = Vector3d(1, 1, -2).normalized();

  // Triangle where θ < θₑ.
  const ccd_real_t colinear_theta = tan(theta_e * 0.75);
  Vector3d c_prime = b + (b.norm() * colinear_theta) * v_hat;
  c = c_prime * 2;
  EXPECT_FALSE(are_coincident(b, c));
  EXPECT_TRUE(triangle_area_is_zero(a, b, c));

  // Triangle where θ > θₑ.
  const ccd_real_t good_tan = tan(theta_e * 1.5);
  c_prime = b + (b.norm() * good_tan) * v_hat;
  c = c_prime * 2;
  // Confirm the test doesn't report false because the points are coincident.
  EXPECT_FALSE(are_coincident(b, c));
  ASSERT_FALSE(triangle_area_is_zero(a, b, c));
}

// This class creates a single simplex. It is the Minkowski difference of
// one triangles and a vertex.
class ExtractClosestPoint : public ::testing::Test {
 protected:
  void SetUp() override {
    // Configure a 3-simplex; "last" is the index of the last valid simplex.
    // For an n-simplex, last is always n - 1.
    simplex_.last = 2;
    for (int i = 0; i < 3; ++i) {
      const Vector3d minkowski_diff{v0_ - t1_[i]};
      write_support(minkowski_diff, v0_, t1_[i], &simplex_.ps[i]);
    }
  }

  // Write the three Eigen vector values into a ccd support vector.
  static void write_support(const Vector3d& minkowski_diff, const Vector3d& v0,
                            const Vector3d& v1, ccd_support_t* support) {
    support->v = eigen_to_ccd(minkowski_diff);
    support->v1 = eigen_to_ccd(v0);
    support->v2 = eigen_to_ccd(v1);
  }

  // Performs the common work of evaluating extractClosetPoint() on a
  // permutation of parameters.
  void EvaluateExtractClosestPoint(ccd_simplex_t* simplex,
                                   const Vector3d& p0_expected,
                                   const Vector3d& p1_expected,
                                   ccd_vec3_t* closest,
                                   const char* message) {
    using fcl::detail::libccd_extension::extractClosestPoints;

    const Vector3d& dummy1{-1, -2, -3};
    const Vector3d& dummy2{-2, -3, -4};
    ccd_vec3_t p0 = eigen_to_ccd(dummy1);
    ccd_vec3_t p1 = eigen_to_ccd(dummy2);

    // Confirm expected solution are not the dummy values.
    EXPECT_FALSE(are_same(p0_expected, p0, kTolerance));
    EXPECT_FALSE(are_same(p1_expected, p1, kTolerance));

    // Test extraction of neither.
    EXPECT_NO_THROW(extractClosestPoints(simplex, nullptr, nullptr, closest))
              << message;

    // Test extraction of p0.
    EXPECT_NO_THROW(extractClosestPoints(simplex, &p0, nullptr, closest))
              << message;
    EXPECT_TRUE(are_same(p0_expected, p0, kTolerance)) << message;

    // Test extraction of p1.
    EXPECT_NO_THROW(extractClosestPoints(simplex, nullptr, &p1, closest))
              << message;
    EXPECT_TRUE(are_same(p1_expected, p1, kTolerance)) << message;

    // Test extraction of both.
    p0 = eigen_to_ccd(dummy1);
    p1 = eigen_to_ccd(dummy2);
    EXPECT_NO_THROW(extractClosestPoints(simplex, &p0, &p1, closest))
              << message;
    EXPECT_TRUE(are_same(p0_expected, p0, kTolerance)) << message;
    EXPECT_TRUE(are_same(p1_expected, p1, kTolerance)) << message;
  }

  // Perform linear interpolation between points a & b.
  // @pre 0 <= s <= 1.
  static Vector3d lerp(const Vector3d& a, const Vector3d& b, double s) {
    return a * s + b * (1 - s);
  };

  ccd_simplex_t simplex_;

  // Vertex on object 0.
  const Vector3d v0_{0.5, 1, 0};

  // Vertices 0, 1, & 2 for triangle 1.
  const Vector3d t1_[3] = {{0, 0.25, 0.25}, {1, 0.25, 0.25}, {0.5, 0.8, 1.0}};

  // TODO(SeanCurtis-TRI): Change this to 1e-15 when the mac libccd
  // single/double precision has been worked out.
  const double kTolerance{constants<ccd_real_t>::eps_78()};
};

// Test extraction from a 1-simplex support method.
TEST_F(ExtractClosestPoint, ExtractFrom1SimplexSupport) {
  using namespace fcl::detail::libccd_extension;
  const Vector3d& dummy1{-1, -2, -3};
  const Vector3d& dummy2{-2, -3, -4};
  // Set up the single support *point*.
  ccd_vec3_t p0 = eigen_to_ccd(dummy1);
  ccd_vec3_t p1 = eigen_to_ccd(dummy2);

  // Test extraction of neither.
  EXPECT_NO_THROW(
      extractObjectPointsFromPoint(&simplex_.ps[0], nullptr, nullptr));

  // Test extraction of p1.
  EXPECT_NO_THROW(extractObjectPointsFromPoint(&simplex_.ps[0], &p0, nullptr));
  EXPECT_TRUE(are_same(v0_, p0, kTolerance));

  // Test extraction of p2.
  EXPECT_NO_THROW(extractObjectPointsFromPoint(&simplex_.ps[0], nullptr, &p1));
  EXPECT_TRUE(are_same(t1_[0], p1, kTolerance));

  // Test extraction of both.
  p0 = eigen_to_ccd(dummy1);
  p1 = eigen_to_ccd(dummy2);
  extractObjectPointsFromPoint(&simplex_.ps[0], &p0, &p1);
  EXPECT_TRUE(are_same(v0_, p0, kTolerance));
  EXPECT_TRUE(are_same(t1_[0], p1, kTolerance));
}

// Test extraction from a 1-simplex through the extractClosestPoints() method.
TEST_F(ExtractClosestPoint, ExtractFrom1Simplex) {
  simplex_.last = 0;

  // NOTE: For a 1-simplex, the closest point isn't used.
  ccd_vec3_t closest = eigen_to_ccd({0, 0, 0});

  EvaluateExtractClosestPoint(&simplex_, v0_, t1_[0], &closest,
                              "ExtractFrom1Simplex");
}

// Test extraction from a 2-simplex support method.
TEST_F(ExtractClosestPoint, ExtractFrom2SimplexSupport) {
  using namespace fcl::detail::libccd_extension;
  const Vector3d& dummy1{-1, -2, -3};
  const Vector3d& dummy2{-2, -3, -4};

  ccd_vec3_t p0 = eigen_to_ccd(dummy1);
  ccd_vec3_t p1 = eigen_to_ccd(dummy2);

  // The query point we're going to use is a simple linear combination of the
  // two end points.
  const Vector3d m0 = ccd_to_eigen(simplex_.ps[0].v);
  const Vector3d m1 = ccd_to_eigen(simplex_.ps[1].v);

  const double s = 1 / 3.0;
  ASSERT_TRUE(s >= 0 && s <= 1);

  ccd_vec3_t closest = eigen_to_ccd(lerp(m0, m1, s));
  const Vector3d p0_expected = v0_;
  const Vector3d p1_expected = lerp(t1_[0], t1_[1], s);

  // Test extraction of neither.
  EXPECT_NO_THROW(extractObjectPointsFromSegment(
      &simplex_.ps[0], &simplex_.ps[1], nullptr, nullptr, &closest));

  // Test extraction of p1.
  EXPECT_NO_THROW(extractObjectPointsFromSegment(
      &simplex_.ps[0], &simplex_.ps[1], &p0, nullptr, &closest));
  EXPECT_TRUE(are_same(p0_expected, p0, kTolerance));

  // Test extraction of p2.
  EXPECT_NO_THROW(extractObjectPointsFromSegment(
      &simplex_.ps[0], &simplex_.ps[1], nullptr, &p1, &closest));
  EXPECT_TRUE(are_same(p1_expected, p1, kTolerance));

  // Test extraction of both.
  p0 = eigen_to_ccd(dummy1);
  p1 = eigen_to_ccd(dummy2);
  EXPECT_NO_THROW(extractObjectPointsFromSegment(
      &simplex_.ps[0], &simplex_.ps[1], &p0, &p1, &closest));
  EXPECT_TRUE(are_same(p0_expected, p0, kTolerance));
  EXPECT_TRUE(are_same(p1_expected, p1, kTolerance));
}

// Test extraction from a 2-simplex through the extractClosestPoints() method.
TEST_F(ExtractClosestPoint, ExtractFrom2Simplex) {
  simplex_.last = 1;

  // The query point we're going to use is a simple linear combination of the
  // two end points.
  const Vector3d m0 = ccd_to_eigen(simplex_.ps[0].v);
  const Vector3d m1 = ccd_to_eigen(simplex_.ps[1].v);

  const double s = 1 / 3.0;
  ASSERT_TRUE(s >= 0 && s <= 1);

  ccd_vec3_t closest = eigen_to_ccd(lerp(m0, m1, s));
  const Vector3d p0_expected = v0_;
  const Vector3d p1_expected = lerp(t1_[0], t1_[1], s);

  EvaluateExtractClosestPoint(&simplex_, p0_expected, p1_expected, &closest,
                              "ExtractFrom2Simplex");
}

// Tests the case where the simplex is a degenerate simplex -- i.e., it is
// actually a line segment.
TEST_F(ExtractClosestPoint, ExtractFrom2SimplexDegenerate) {
  simplex_.last = 1;
  // NOTE: This exercises the knowledge that the coincidence tolerance is eps.
  const ccd_real_t eps = 0.5 * constants<ccd_real_t>::eps();

  // Copy the first support vertex into the second support vertex and then
  // perturb the second a small amount. We add a small amount to the
  // x-components of the minkowski sum *and* the object1 vertex.
  ccdSupportCopy(&simplex_.ps[1], &simplex_.ps[0]);
  simplex_.ps[1].v.v[0] += eps;
  simplex_.ps[1].v1.v[0] += eps;
  // Confirm that the input and expected answer (v0_) match.
  ASSERT_TRUE(are_same(v0_, simplex_.ps[0].v1, kTolerance));

  // The line segment is now of length 1; the answer should be essentially the
  // same as evaluating for a single point.
  ccd_vec3_t closest = eigen_to_ccd({0, 0, 0});

  EvaluateExtractClosestPoint(&simplex_, v0_, t1_[0], &closest,
                              "ExtractFrom2SimplexDegenerate");
}

// Test extraction from a 3-simplex through the extractClosestPoints() method.
// Note: there is no support method for the 3-simplex like there is for the 1-
// and 2-simplices.
TEST_F(ExtractClosestPoint, ExtractFrom3Simplex) {
  // Compute a "closest point" based on arbitrary barycentric coordinates.
  const double alpha = 0.25;
  const double beta = 0.33;
  ASSERT_TRUE(alpha >= 0 && alpha <= 1 && beta >= 0 && beta <= 1 &&
              alpha + beta <= 1);

  const Vector3d m0 = ccd_to_eigen(simplex_.ps[0].v);
  const Vector3d m1 = ccd_to_eigen(simplex_.ps[1].v);
  const Vector3d m2 = ccd_to_eigen(simplex_.ps[2].v);

  // Interpolate three vertices via barycentric coordinates s1 and s2.
  auto interpolate = [](const Vector3d& a, const Vector3d& b, const Vector3d& c,
                        double s1, double s2) -> Vector3d {
    return a * s1 + b * s2 + c * (1 - s1 - s2);
  };

  ccd_vec3_t closest = eigen_to_ccd(interpolate(m0, m1, m2, alpha, beta));
  const Vector3d p0_expected = v0_;
  const Vector3d p1_expected = interpolate(t1_[0], t1_[1], t1_[2], alpha, beta);

  EvaluateExtractClosestPoint(&simplex_, p0_expected, p1_expected, &closest,
                              "ExtractFrom3Simplex");
}

// Tests the case where the 3-simplex is degenerate -- the points are considered
// coincident.
TEST_F(ExtractClosestPoint, ExtractFrom3SimplesDegenerateCoincident) {
  // This test essentially reproduces the *valid* result from
  // ExtractFrom2Simplex(). The difference is that it *claims* to be a triangle.

  // NOTE: This exercises the knowledge that the coincidence tolerance is eps.
  const ccd_real_t eps = 0.5 * constants<ccd_real_t>::eps();

  // Copy the first support vertex into the second and third support vertices
  // and then perturb the copies a small amount. We add a small amount to the
  // x-components of the minkowski sum *and* the object1 vertex.
  for (auto i : {1, 2}) {
    ccdSupportCopy(&simplex_.ps[i], &simplex_.ps[0]);
    simplex_.ps[i].v.v[0] += eps;
    simplex_.ps[i].v1.v[0] += eps;
  }
  // Confirm that the input and expected answer (v0_) match.
  ASSERT_TRUE(are_same(v0_, simplex_.ps[0].v1, kTolerance));

  // The triangle has zero area because the vertices are all coincident;
  // the answer should be essentially the same as evaluating for a single point.
  ccd_vec3_t closest = eigen_to_ccd({0, 0, 0});

  EvaluateExtractClosestPoint(&simplex_, v0_, t1_[0], &closest,
                              "ExtractFrom3SimplexDegenerateCoincident");
}

// Tests the case where the 3-simplex is degenerate -- the points are considered
// co-linear.
TEST_F(ExtractClosestPoint, ExtractFrom3SimplesDegenerateColinear) {
  // The query point we're going to use is a simple linear combination of the
  // v0 and v1 (from the minkowski sum). That means the points on the triangle
  // should likewise be a combination of v0 and v1 from each shape.
  const Vector3d m0 = ccd_to_eigen(simplex_.ps[0].v);
  const Vector3d m1 = ccd_to_eigen(simplex_.ps[1].v);

  const double s = 1 / 3.0;
  ASSERT_TRUE(s >= 0 && s <= 1);

  ccd_vec3_t closest = eigen_to_ccd(lerp(m0, m1, s));
  const Vector3d p0_expected = v0_;
  const Vector3d p1_expected = lerp(t1_[0], t1_[1], s);

  // Now set up co-linear configuration. Vertex 2 will simply be a linear
  // combination of vertex 0 and vertex 1.
  //   v2 = 2 * (v1 - v0) + v0 = 2 * v1 - v0.
  // Note: this puts v2 on the same line, but not inside the line segment
  // spanned by v0 and v1. Because the closest point lies on the segment, this
  // confirms that arbitrarily extending the degeneracy doesn't change the
  // answer.
  auto linearly_combine = [](const ccd_vec3_t& a, const ccd_vec3_t& b,
                             ccd_vec3_t* dst) {
    auto A = ccd_to_eigen(a);
    auto B = ccd_to_eigen(b);
    auto C = 2 * B - A;
    *dst = eigen_to_ccd(C);
  };
  linearly_combine(simplex_.ps[0].v, simplex_.ps[1].v, &simplex_.ps[2].v);
  linearly_combine(simplex_.ps[0].v1, simplex_.ps[1].v1, &simplex_.ps[2].v1);
  linearly_combine(simplex_.ps[0].v2, simplex_.ps[1].v2, &simplex_.ps[2].v2);

  EvaluateExtractClosestPoint(&simplex_, p0_expected, p1_expected, &closest,
                              "ExtractFrom3SimplexDegenerateColinear");
}

}  // namespace detail
}  // namespace fcl

//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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

// Tests the implementation of a convex polytope geometry.

#include "fcl/geometry/shape/convex.h"

#include <vector>

#include <Eigen/StdVector>
#include <gtest/gtest.h>

#include "eigen_matrix_compare.h"
#include "expect_throws_message.h"
#include "fcl/common/types.h"

namespace fcl {
class ConvexTester {
 public:
  template <typename S>
  static bool find_extreme_via_neighbors(const Convex<S>& convex) {
      return convex.find_extreme_via_neighbors_;
  }

  // Override the built-in logic for disabling find_extreme_via_neighbors.
  template <typename S>
  static void force_find_extreme_via_neighbors(Convex<S>* convex) {
    convex->find_extreme_via_neighbors_ = true;
  }
};
namespace {

using std::max;

// Necessary to satisfy Eigen's alignment requirements. See
// http://eigen.tuxfamily.org/dox-devel/group__TopicStlContainers.html#StlContainers_vector
template <typename S>
using PoseVector = std::vector<Transform3<S>,
                               Eigen::aligned_allocator<Transform3<S>>>;

// Utilities to print scalar type in error messages.
template <typename S>
struct ScalarString {
  static std::string value() { return "unknown"; }
};

template <>
struct ScalarString<double> {
  static std::string value() { return "double"; }
};

template <>
struct ScalarString<float> {
  static std::string value() { return "float"; }
};

// Base definition of a "unit" convex polytope. Specific instances should define
// faces, vertices, and quantities such as volume, center of mass, and moment of
// inertia in terms of a scale factor.
template <typename S>
class Polytope {
 public:
  explicit Polytope(S scale)
    : vertices_(std::make_shared<std::vector<Vector3<S>>>()),
      polygons_(std::make_shared<std::vector<int>>()), scale_(scale) {}

  Polytope(const Polytope &other)
    : vertices_(std::make_shared<std::vector<Vector3<S>>>(*(other.vertices_))),
      polygons_(std::make_shared<std::vector<int>>(*(other.polygons_))),
      scale_(other.scale_) {}

  virtual int face_count() const = 0;
  virtual int vertex_count() const = 0;
  virtual S volume() const { throw std::logic_error("Not implemented yet"); }
  virtual Vector3<S> com() const {
    throw std::logic_error("Not implemented yet");
  }
  virtual Matrix3<S> principal_inertia_tensor() const {
    throw std::logic_error("Not implemented yet");
  }
  virtual std::string description() const {
    throw std::logic_error("Not implemented yet");
  }

  // The scale of the polytope to use with test tolerances.
  S scale() const { return scale_; }
  std::shared_ptr<const std::vector<Vector3<S>>> points() const {
    return vertices_;
  }
  std::shared_ptr<const std::vector<int>> polygons() const {
    return polygons_;
  }
  Convex<S> MakeConvex(bool throw_if_invalid = true) const {
    return Convex<S>(points(), face_count(), polygons(), throw_if_invalid);
  }
  Vector3<S> min_point() const {
    Vector3<S> m;
    m.setConstant(std::numeric_limits<S>::max());
    for (const auto& v : *vertices_) {
      for (int i = 0; i < 3; ++i) {
        if (v(i) < m(i)) m(i) = v(i);
      }
    }
    return m;
  }
  Vector3<S> max_point() const {
    Vector3<S> m;
    m.setConstant(-std::numeric_limits<S>::max());
    for (const auto& v : *vertices_) {
      for (int i = 0; i < 3; ++i) {
        if (v(i) > m(i)) m(i) = v(i);
      }
    }
    return m;
  }
  Vector3<S> aabb_center() const {
    return (max_point() + min_point()) / 2;
  }
  S aabb_radius() const { return (min_point() - aabb_center()).norm(); }
  void SetPose(const Transform3<S>& X_WP) {
    for (auto& v : *vertices_) {
      v = X_WP * v;
    }
  }

 protected:
  void add_vertex(const Vector3<S>& vertex) { vertices_->push_back(vertex); }
  void add_face(std::initializer_list<int> indices) {
    polygons_->push_back(static_cast<int>(indices.size()));
    polygons_->insert(polygons_->end(), indices);
  }
  // Confirms the number of vertices and number of polygons matches the counts
  // implied by vertex_count() and face_count(), respectively.
  void confirm_data() {
    // Confirm point count.
    GTEST_ASSERT_EQ(vertex_count(), static_cast<int>(vertices_->size()));

    // Confirm face count.
    // Count the number of faces encoded in polygons_;
    int count = 0;
    int i = 0;
    while (i < static_cast<int>(polygons_->size())) {
      ++count;
      i += (*polygons_)[i] + 1;
    }
    GTEST_ASSERT_EQ(i, static_cast<int>(polygons_->size()))
                  << "Badly defined polygons";
    GTEST_ASSERT_EQ(face_count(), count);
  }

 protected:
  std::shared_ptr<std::vector<Vector3<S>>> vertices_;
  std::shared_ptr<std::vector<int>> polygons_;
  S scale_{};
};

// A simple regular tetrahedron with edges of length `scale` centered on the
// origin.
template <typename S>
class EquilateralTetrahedron : public Polytope<S> {
 public:
  // Constructs the tetrahedron (of edge length `s`).
  explicit EquilateralTetrahedron(S scale) : Polytope<S>(scale) {
    // Tetrahedron vertices in the tet's canonical frame T. The tet is
    // "centered" on the origin so that it's center of mass is simple [0, 0, 0].
    const S z_base = -1 / S(2 * sqrt(6.));
    Vector3<S> points_T[] = {{S(0.5), S(-0.5 / sqrt(3.)), z_base},
                             {S(-0.5), S(-0.5 / sqrt(3.)), z_base},
                             {S(0), S(1. / sqrt(3.)), z_base},
                             {S(0), S(0), S(sqrt(3. / 8))}};
    for (const auto& v : points_T) {
      this->add_vertex(scale * v);
    };

    // Now add the polygons
    this->add_face({0, 1, 2});
    this->add_face({1, 0, 3});
    this->add_face({0, 2, 3});
    this->add_face({2, 1, 3});

    this->confirm_data();
  }
  // Properties of the polytope.
  int face_count() const final { return 4; }
  int vertex_count() const final { return 4; }
  S volume() const final {
    // This assumes unit mass.
    const S s = this->scale_;
    return S(sqrt(2) / 12) * s * s * s;
  }
  Vector3<S> com() const final { return Vector3<S>::Zero(); }
  std::string description() const final {
    return "Tetrahedron with scale: " + std::to_string(this->scale());
  }
};

// A simple cube with sides of length `scale`.
template <typename S>
class Cube : public Polytope<S> {
 public:
  Cube(S scale) : Polytope<S>(scale) {
    // Cube vertices in the cube's canonical frame C.
    Vector3<S> points_C[] = {{S(-0.5), S(-0.5), S(-0.5)},   // v0
                             {S(0.5), S(-0.5), S(-0.5)},    // v1
                             {S(-0.5), S(0.5), S(-0.5)},    // v2
                             {S(0.5), S(0.5), S(-0.5)},     // v3
                             {S(-0.5), S(-0.5), S(0.5)},    // v4
                             {S(0.5), S(-0.5), S(0.5)},     // v5
                             {S(-0.5), S(0.5), S(0.5)},     // v6
                             {S(0.5), S(0.5), S(0.5)}};     // v7
    for (const auto& v : points_C) {
      this->add_vertex(scale * v);
    }

    // Now add the polygons
    this->add_face({1, 3, 7, 5});    // +x
    this->add_face({0, 4, 6, 2});    // -x
    this->add_face({4, 5, 7, 6});    // +y
    this->add_face({0, 2, 3, 1});    // -y
    this->add_face({6, 7, 3, 2});    // +z
    this->add_face({0, 1, 5, 4});    // -z

    this->confirm_data();
  }

  // Polytope properties
  int face_count() const final { return 6; }
  int vertex_count() const final { return 8; }
  virtual S volume() const final {
    S s = this->scale();
    return s * s * s;
  }
  virtual Vector3<S> com() const final { return Vector3<S>::Zero(); }
  virtual Matrix3<S> principal_inertia_tensor() const {
    S scale_sqd = this->scale() * this->scale();
    // This assumes unit mass.
    return Eigen::DiagonalMatrix<S, 3>(1. / 6., 1. / 6., 1. / 6.) * scale_sqd;
  };
  std::string description() const final {
    return "Cube with scale: " + std::to_string(this->scale());
  }
};

void testConvexConstruction() {
  Cube<double> cube{1};
  // Set the cube at some other location to make sure that the interior point
  // test/ doesn't pass just because it initialized to zero.
  Vector3<double> p_WB(1, 2, 3);
  cube.SetPose(Transform3<double>(Eigen::Translation3d(p_WB)));
  Convex<double> convex = cube.MakeConvex();

  // This doesn't depend on the correct logic in the constructor. But this is
  // as convenient a time as any to test that it reports the right node type.
  EXPECT_EQ(convex.getNodeType(), GEOM_CONVEX);

  // The constructor computes the interior point.
  EXPECT_TRUE(CompareMatrices(convex.getInteriorPoint(), p_WB));
}

template <template <typename> class Shape, typename S>
void testAABBComputation(const Shape<S>& model, const Transform3<S>& X_WS) {
  Shape<S> shape(model);
  shape.SetPose(X_WS);
  Convex<S> convex = shape.MakeConvex();
  convex.computeLocalAABB();

  typename constants<S>::Real eps = constants<S>::eps();
  EXPECT_NEAR(shape.aabb_radius(), convex.aabb_radius, eps);
  EXPECT_TRUE(CompareMatrices(shape.aabb_center(), convex.aabb_center, eps));
  EXPECT_TRUE(CompareMatrices(shape.min_point(), convex.aabb_local.min_, eps));
  EXPECT_TRUE(CompareMatrices(shape.max_point(), convex.aabb_local.max_, eps));
}

template <template <typename> class Shape, typename S>
void testVolume(const Shape<S>& model, const Transform3<S>& X_WS,
                int bits_lost) {
  // If we're losing more than 10 bits, then we have a major problem.
  GTEST_ASSERT_LE(bits_lost, 10);

  Shape<S> shape(model);
  shape.SetPose(X_WS);
  Convex<S> convex = shape.MakeConvex();

  // We want the basic tolerance to be near machine precision. The invocation
  // of this function indicates how many bits of precision are expected to be
  // lost and the machine epsilon is modified to account for this.
  typename constants<S>::Real eps = (1 << bits_lost) * constants<S>::eps();

  // We want to do a *relative* comparison. We scale our eps by the volume so
  // that large volumes have tolerances proportional to the actual true value.
  S scale = max(shape.volume(), S(1));
  EXPECT_NEAR(shape.volume(), convex.computeVolume(), eps * scale)
            << shape.description() << " at\n" << X_WS.matrix()
            << "\nusing scalar: " << ScalarString<S>::value();
}

template <template <typename> class Shape, typename S>
void testCenterOfMass(const Shape<S>& model, const Transform3<S>& X_WS,
                      int bits_lost) {
  // If we're losing more than 10 bits, then we have a major problem.
  GTEST_ASSERT_LE(bits_lost, 10);

  Shape<S> shape(model);
  shape.SetPose(X_WS);
  Convex<S> convex = shape.MakeConvex();

  // We want the basic tolerance to be near machine precision. The invocation
  // of this function indicates how many bits of precision are expected to be
  // lost and the machine epsilon is modified to account for this.
  typename constants<S>::Real eps = (1 << bits_lost) * constants<S>::eps();

  // We want to do a *relative* comparison. The center-of-mass calculation is a
  // volume-weighted calculation. So, the relative tolerance should scale with
  // volume.
  S scale = max(shape.volume(), S(1));
  EXPECT_TRUE(
      CompareMatrices(X_WS * shape.com(), convex.computeCOM(), eps * scale))
            << shape.description() << " at\n" << X_WS.matrix()
            << "\nusing scalar: " << ScalarString<S>::value();
}

template <template <typename> class Shape, typename S>
void testMomentOfInertia(const Shape<S>& model, const Transform3<S>& X_WS,
                         int bits_lost) {
  // If we're losing more than 10 bits, then we have a major problem.
  GTEST_ASSERT_LE(bits_lost, 10);

  Shape<S> shape(model);
  shape.SetPose(X_WS);
  Convex<S> convex = shape.MakeConvex();

  // We want the basic tolerance to be near machine precision. The invocation
  // of this function indicates how many bits of precision are expected to be
  // lost and the machine epsilon is modified to account for this.
  typename constants<S>::Real eps = (1 << bits_lost) * constants<S>::eps();

  // We want to do a *relative* comparison. The inertia calculation is a
  // volume-weighted calculation. So, the relative tolerance should scale with
  // volume.
  S scale = max(shape.volume(), S(1));
  EXPECT_TRUE(
      CompareMatrices(X_WS.linear().transpose() *
                          shape.principal_inertia_tensor() * X_WS.linear(),
                      convex.computeMomentofInertiaRelatedToCOM(), eps * scale))
            << shape.description() << " at\n" << X_WS.matrix()
            << "\nusing scalar: " << ScalarString<S>::value();
}

// The definition of a support vertex test configuration.
template <typename S>
struct SupportVertexTest {
  // The direction for which we find the support vertex, expressed in the
  // shape's frame S.
  Vector3<S> v_S;
  // The position of the *expected* support vertex measured and expressed in the
  // shape's frame S.
  Vector3<S> p_SE;
};

template <template <typename> class Shape, typename S>
void testSupportVertex(const Shape<S>& model, const Transform3<S>& X_WS,
    const std::vector<SupportVertexTest<S>>& tests) {
  Shape<S> shape_W(model);
  shape_W.SetPose(X_WS);
  Convex<S> convex_W = shape_W.MakeConvex();
  for (const auto& test : tests) {
    const Vector3<S> v_W = X_WS.linear() * test.v_S;
    const Vector3<S> p_WE = X_WS * test.p_SE;
    // As long as we don't have directions parallel with face normals, the
    // answer should be unique and precise down to the last bit.
    EXPECT_TRUE(CompareMatrices(convex_W.findExtremeVertex(v_W), p_WE))
        << shape_W.description() << " at\n"
        << X_WS.matrix() << "\nusing scalar: " << ScalarString<S>::value()
        << "\n  v_W = " << v_W.transpose();
  }
}

template <typename S>
PoseVector<S> GetPoses() {
  PoseVector<S> poses;
  // Identity.
  poses.push_back(Transform3<S>::Identity());

  Transform3<S> X_WS;
  // 90-degree rotation around each axis, in turn.
  for (int i = 0; i < 3; ++i) {
    X_WS = Transform3<S>::Identity();
    X_WS.linear() = AngleAxis<S>(constants<S>::pi() / 2,
                                 Vector3<S>::Unit(i)).matrix();
    poses.push_back(X_WS);
  }

  // Small angle away from identity.
  X_WS.linear() = AngleAxis<S>(S(1e-5), Vector3<S>{1, 2, 3}.normalized())
      .matrix();
  poses.push_back(X_WS);

  // 45-degree angle to move away from axis-aligned as much as possible.
  X_WS.linear() = AngleAxis<S>(constants<S>::pi() / 4,
                               Vector3<S>{1, 2, 3}.normalized()).matrix();
  poses.push_back(X_WS);

  // We don't test translation because that would imply the geometry is
  // defined away from its own frame's origin. And that's just a recklessly
  // stupid thing to do. Given the *current* algorithms, this will degrade
  // the answers based on the *distance* to the origin.
  // TODO(SeanCurtis-TRI): When the algorithms are no longer sensitive to vertex
  // position relative to the origin, add tests that show that.

  return poses;
}

std::vector<double> get_test_scales() {
  return std::vector<double>{0.001, 1, 1000.};
}

template <template <typename> class Shape, typename S>
void testLocalAABBComputation(const Shape<S>& shape) {
  for (const auto& X_WP : GetPoses<S>()) {
    testAABBComputation<Shape>(shape, X_WP);
  }
}

template <template <typename> class Shape, typename S>
void testVolumeComputation(const Shape<S>& shape, int bits_lost) {
  for (const auto& X_WP : GetPoses<S>()) {
    testVolume<Shape>(shape, X_WP, bits_lost);
  }
}

template <template <typename> class Shape, typename S>
void testCenterOfMassComputation(const Shape<S>& shape, int bits_lost) {
  for (const auto& X_WP : GetPoses<S>()) {
    testCenterOfMass<Shape>(shape, X_WP, bits_lost);
  }
}

template <template <typename> class Shape, typename S>
void testMomentOfInertiaComputation(const Shape<S>& shape, int bits_lost) {
  for (const auto& X_WP : GetPoses<S>()) {
    testMomentOfInertia<Shape>(shape, X_WP, bits_lost);
  }
}

template <template <typename> class Shape, typename S>
void testSupportVertexComputation(
    const Shape<S>& shape, const std::vector<SupportVertexTest<S>>& tests) {
  for (const auto& X_WP : GetPoses<S>()) {
    testSupportVertex<Shape>(shape, X_WP, tests);
  }
}

GTEST_TEST(ConvexGeometry, Constructor) {
  testConvexConstruction();
}

GTEST_TEST(ConvexGeometry, LocalAABBComputation_Cube) {
  for (double scale : get_test_scales()) {
    Cube<double> cube_d(scale);
    testLocalAABBComputation(cube_d);
    Cube<float> cube_f(static_cast<float>(scale));
    testLocalAABBComputation(cube_f);
  }
}

GTEST_TEST(ConvexGeometry, Volume_Cube) {
  for (double scale : get_test_scales()) {
    Cube<double> cube_d(scale);
    testVolumeComputation(cube_d, 0);
    Cube<float> cube_f(static_cast<float>(scale));
    // Apparently, no bits of precision are lost (relative to machine precision)
    // on the cube volume *except* for the *large* cube in single precision.
    // The reason for this isn't obvious, but probably a coincidental artifact
    // of the particular configuration.
    const int bits_lost = scale > 1 ? 2 : 0;
    testVolumeComputation(cube_f, bits_lost);
  }
}

GTEST_TEST(ConvexGeometry, CenterOfMass_Cube) {
  for (double scale : get_test_scales()) {
    Cube<double> cube_d(scale);
    testCenterOfMassComputation(cube_d, 0);
    Cube<float> cube_f(static_cast<float>(scale));
    testCenterOfMassComputation(cube_f, 0);
  }
}

GTEST_TEST(ConvexGeometry, MomentOfInertia_Cube) {
  for (double scale : get_test_scales()) {
    Cube<double> cube_d(scale);
    testMomentOfInertiaComputation(cube_d, 0);
    Cube<float> cube_f(static_cast<float>(scale));
    testMomentOfInertiaComputation(cube_f, 0);
  }
}

GTEST_TEST(ConvexGeometry, LocalAABBComputation_Tetrahedron) {
  for (double scale : get_test_scales()) {
    EquilateralTetrahedron<double> tet_d(scale);
    testLocalAABBComputation(tet_d);
    EquilateralTetrahedron<float> tet_f(static_cast<float>(scale));
    testLocalAABBComputation(tet_f);
  }
}

GTEST_TEST(ConvexGeometry, Volume_Tetrahedron) {
  for (double scale : get_test_scales()) {
    EquilateralTetrahedron<double> tet_d(scale);
    // Apparently, no bits of precision are lost (relative to machine precision)
    // on the tet volume *except* for the *large* test in double precision.
    // The reason for this isn't obvious, but probably a coincidental artifact
    // of the particular configuration.
    const int bits_lost = scale > 1 ? 1 : 0;
    testVolumeComputation(tet_d, bits_lost);
    EquilateralTetrahedron<float> tet_f(static_cast<float>(scale));
    testVolumeComputation(tet_f, 0);
  }
}

GTEST_TEST(ConvexGeometry, CenterOfMass_Tetrahedron) {
  for (double scale : get_test_scales()) {
    EquilateralTetrahedron<double> tet_d(scale);
    testCenterOfMassComputation(tet_d, 0);
    EquilateralTetrahedron<float> tet_f(static_cast<float>(scale));
    testCenterOfMassComputation(tet_f, 0);
  }
}

// Defines a collection of support vertex directions and expected vertex
// results. We assume that the given polytope contains the origin so that the
// direction to every polytope vertex is unique. So, the direction to the vertex
// should uniquely produce that vertex as the support vertex.
template <typename S>
std::vector<SupportVertexTest<S>> BuildSupportVertexTests(
    const Polytope<S>& polytope) {
  std::vector<SupportVertexTest<S>> tests;
  for (const auto& p_SV : *polytope.points()) {
    tests.push_back({p_SV, p_SV});
  }
  return tests;
}

GTEST_TEST(ConvexGeometry, SupportVertex_Tetrahedron) {
  for (double scale : get_test_scales()) {
    EquilateralTetrahedron<double> tet_d(scale);
    std::vector<SupportVertexTest<double>> tests_d =
        BuildSupportVertexTests(tet_d);
    testSupportVertexComputation(tet_d, tests_d);

    EquilateralTetrahedron<float> tet_f(scale);
    std::vector<SupportVertexTest<float>> tests_f =
        BuildSupportVertexTests(tet_f);
    testSupportVertexComputation(tet_f, tests_f);
  }
}

// A tetrahedron whose bottom triangle consists of three co-planar faces. (In
// other words, we've injected a new vertex into the center of the bottom face.)
// That new vertex is vertex 0. Used for the SupportVertexCoPlanarFaces test.
class CoPlanarTetrahedron final : public Polytope<double> {
 public:
  CoPlanarTetrahedron() : Polytope<double>(1.0) {
    // Tetrahedron vertices in the tet's canonical frame T. The tet is placed
    // so that it's bottom face lies on the z = 0 plane.
    Vector3d points_T[] = {{0.5, -0.5 / sqrt(3.), 0},
                           {-0.5, -0.5 / sqrt(3.), 0},
                           {0, 1. / sqrt(3.), 0},
                           {0, 0, sqrt(3. / 8)}};
    const Vector3d center = (points_T[0] + points_T[1] + points_T[2]) / 3.0;
    this->add_vertex(center);
    for (const auto& v : points_T) {
      this->add_vertex(v);
    };

    // Now add the polygons
    this->add_face({0, 1, 2});
    this->add_face({0, 2, 3});
    this->add_face({0, 3, 1});
    this->add_face({1, 3, 4});
    this->add_face({3, 2, 4});
    this->add_face({1, 4, 2});

    this->confirm_data();
  }
  // Properties of the polytope.
  int face_count() const final { return 6; }
  int vertex_count() const final { return 5; }
};

// Test for special condition in findExtremeVertex which can arise iff the
// Convex shape has a vertex whose adjacent vertices are all co-planar with it,
// that vertex is the *starting* vertex of the search, *and* the query direction
// is perpendicular to the plane that those vertices all lie on.
GTEST_TEST(ConvexGeometry, SupportVertexCoPlanarFaces) {
  CoPlanarTetrahedron tet;
  Convex<double> convex_W = tet.MakeConvex();
  // Query direction is perpendicular to the bottom face.
  const Vector3d v_W{0, 0, 1};
  const Vector3d p_WE_expected = tet.points()->at(4);
  // With only five vertices, findExtremeVertex would default to a linear
  // search. For this test, we want to use the edge graph. So, we force it to
  // be enabled.
  ConvexTester::force_find_extreme_via_neighbors(&convex_W);

  // We can expect an exact answer down to the last bit.
  EXPECT_TRUE(CompareMatrices(convex_W.findExtremeVertex(v_W), p_WE_expected));
}

// A tetrahedron with a missing face.
class HoleTetrahedron final : public Polytope<double> {
 public:
  HoleTetrahedron() : Polytope<double>(1.0) {
    // We'll start with a good tetrahedron, copy all vertices and simply omit
    // its first face.
    EquilateralTetrahedron<double> tet(1.0);
    vertices_->insert(vertices_->begin(), tet.points()->begin(),
                      tet.points()->end());

    // Add the faces of the tet (skipping the first).
    const std::vector<int>& polys = *tet.polygons();
    // polys[0] is the number of vertices in face 0. So, face 1 starts at
    // that number plus one.
    const int face1 = polys[0] + 1;
    polygons_->insert(polygons_->end(), polys.begin() + face1, polys.end());

    this->confirm_data();
  }
  // Properties of the polytope.
  int face_count() const final { return 3; }
  int vertex_count() const final { return 4; }
};

// A tetrahedron with a structural crack; a vertex is duplicated (but left in
// the same location). It forms a break in the topology over its adjacent edges.
class CrackTetrahedron final : public Polytope<double> {
 public:
  CrackTetrahedron() : Polytope<double>(1.0) {
    EquilateralTetrahedron<double> tet(1.0);
    vertices_->insert(vertices_->begin(), tet.points()->begin(),
                      tet.points()->end());
    vertices_->push_back((*vertices_)[0]);

    // Now add the polygons by hand (copied and modified from
    // EquilateralTetrahedron).
    this->add_face({4, 1, 2});  // Vertex 4 swapped for vertex 0.
    this->add_face({1, 0, 3});
    this->add_face({0, 2, 3});
    this->add_face({2, 1, 3});

    this->confirm_data();
  }
  // Properties of the polytope.
  int face_count() const final { return 4; }
  int vertex_count() const final { return 5; }
};

// A tetrahedron with a "stray" vertex; a vertex not connected to any face.
class StrayVertexTetrahedron final : public Polytope<double> {
 public:
  StrayVertexTetrahedron() : Polytope<double>(1.0) {
    EquilateralTetrahedron<double> tet(1.0);
    vertices_->insert(vertices_->begin(), tet.points()->begin(),
                      tet.points()->end());
    // Add the stray.
    vertices_->push_back({-1, -1, -1});

    // Add the faces of the tet.
    const std::vector<int>& polys = *tet.polygons();
    polygons_->insert(polygons_->end(), polys.begin(), polys.end());

    this->confirm_data();
  }
  // Properties of the polytope.
  int face_count() const final { return 4; }
  int vertex_count() const final { return 5; }
};

// A tetrahedron with an extra face built off of one of the edges.
class NonManifoldTetrahedron final : public Polytope<double> {
 public:
  NonManifoldTetrahedron() : Polytope<double>(1.0) {
    EquilateralTetrahedron<double> tet(1.0);
    vertices_->insert(vertices_->begin(), tet.points()->begin(),
                      tet.points()->end());
    vertices_->push_back({0, 0, -5});

    polygons_->insert(polygons_->end(), tet.polygons()->begin(),
                      tet.polygons()->end());
    // The (0, 1) edge is now shared by 3 faces.
    this->add_face({0, 1, 4});

    this->confirm_data();
  }
  // Properties of the polytope.
  int face_count() const final { return 5; }
  int vertex_count() const final { return 5; }
};

// The test for the "watertight" validation conditions with several variations
// of invalid topologies. We don't *explicitly* test the *valid* case because it
// is implicitly tested every time a Convex is created in these tests. We also
// don't care about the scalar types because this test is purely about topology.
GTEST_TEST(ConvexGeometry, WaterTightValidation) {
  {
    // Hole in the convex mesh.
    HoleTetrahedron bad_tet;
    FCL_EXPECT_THROWS_MESSAGE(
        bad_tet.MakeConvex(), std::runtime_error,
        "Found errors in the Convex mesh[^]+ Edge between vertices \\d+ and "
        "\\d+ is shared by 1 faces .+");
  }

  {
    // Crack in an otherwise closed convex mesh due to duplicate vertices.
    StrayVertexTetrahedron bad_tet;
    FCL_EXPECT_THROWS_MESSAGE(
        bad_tet.MakeConvex(), std::runtime_error,
        "Found errors in the Convex mesh[^]+ Not all vertices are connected[^]+"
        " Vertex \\d+ is not included in any faces[^]*");
  }

  {
    // Crack in an otherwise closed convex mesh due to duplicate vertices.
    CrackTetrahedron bad_tet;
    FCL_EXPECT_THROWS_MESSAGE(
        bad_tet.MakeConvex(), std::runtime_error,
        "Found errors in the Convex mesh[^]+ Edge between vertices \\d+ and "
        "\\d+ is shared by 1 faces .+");
  }

  {
    // Non-manifold mesh (an edge is shared by three faces).
    NonManifoldTetrahedron bad_tet;
    FCL_EXPECT_THROWS_MESSAGE(
        bad_tet.MakeConvex(), std::runtime_error,
        "Found errors in the Convex mesh[^]+ Edge between vertices 0 and 1 is "
        "shared by 3 faces [^]+");
  }
}

// A tessellated unit sphere; 8 longitudinal wedges and 8 latitudinal bands.
class TessellatedSphere final : public Polytope<double> {
 public:
  TessellatedSphere() : Polytope<double>(1.0) {
      // The angle between the latitude lines measured along the prime meridian.
      const double dphi = M_PI / 8;
      auto slice_height = [dphi](int slice_index) {
          // Assumes 1 <= slice_index < 8.
          return std::cos(slice_index * dphi);
      };
      auto slice_radius = [dphi](int slice_index) {
        // Assumes 1 <= slice_index < 8.
        return std::sin(slice_index * dphi);
      };
      // North pole is top of slice 1.
      vertices_->push_back({0, 0, 1});
      // Now create the bands of vertices between slices 1 & 2, 2 & 3, etc.
      // The angle between the longitude lines measured along the equator.
      const double dtheta = 2 * M_PI / 8;
      for (int slice = 1; slice < 8; ++slice) {
          double z = slice_height(slice);
          double r = slice_radius(slice);
          for (int i = 0; i < 8; ++i) {
              const double theta = dtheta * i;
              vertices_->emplace_back(std::cos(theta) * r,
                                      std::sin(theta) * r,
                                      z);
          }
      }
      // South pole is slice bottom of slice 8.
      vertices_->push_back({0, 0, -1});

      // North pole triangle fan: slice 1.
      // [0, 8, 1], [0, 1, 2], [0, 2, 3], ..., [0, 7, 8].
      // The "previous" index is: 8, 1, 2, 3, ... 7 and i is 1, 2, 3, ..., 8.
      int prev = 8;
      int next = 1;
      for (; next <= 8; prev = next, ++next) {
          this->add_face({0, prev, next});
          prev = next;
      }
      // The rectangular facets for each latitude band. For slice 2, the quads
      // would be: [upper prev, prev, next, upper next]. I.e., [8, 16, 9, 1],
      // [1, 9, 10, 2], [2, 10, 11, 3], ..., [7, 15, 16, 8]. Such that
      // upper_prev = prev - 8, and upper_next = next - 8. So, we track
      // prev and next and compute the upper versions.
      for (int slice = 2; slice < 8; ++slice) {
          prev = slice * 8;
          next = prev - 7;
          for (int i = 0; i < 8; ++i, prev = next, ++next) {
              this->add_face({prev - 8, prev, next, next - 8});
          }
      }
      // South pole triangle fan: slice 8.
      prev = 56;  // slice 7 * 8.
      next = 49;  // The index of the first vertex on slice 8.
      for (int i = 0; i < 8; ++i, prev = next, ++next) {
          this->add_face({57, next, prev});
      }

      this->confirm_data();
  }
  // Properties of the polytope.
  int face_count() const final { return 64; }
  int vertex_count() const final { return 58; }
};

// Confirm that edge walking gets disabled in expected cases.
GTEST_TEST(ConvexGeometry, UseEdgeWalkingConditions) {
    const bool throw_if_invalid{true};
    {
        // Too few triangles.
        EquilateralTetrahedron<double> poly(1.0);
        Convex<double> convex = poly.MakeConvex(throw_if_invalid);
        EXPECT_FALSE(ConvexTester::find_extreme_via_neighbors(convex));
    }
    {
        // Hole in an unvalidated convex mesh.
        HoleTetrahedron poly;
        Convex<double> convex = poly.MakeConvex(!throw_if_invalid);
        EXPECT_FALSE(ConvexTester::find_extreme_via_neighbors(convex));
    }
    {
        // A *valid* mesh with sufficient number of vertices will enable edge
        // walking. Simply create a tessellated sphere.
        TessellatedSphere poly;
        Convex<double> convex = poly.MakeConvex(throw_if_invalid);
        EXPECT_TRUE(ConvexTester::find_extreme_via_neighbors(convex));
    }
}

// TODO(SeanCurtis-TRI): Add Tetrahedron inertia unit test.

// TODO(SeanCurtis-TRI): Extend the moment of inertia test.
//   Tesselate smooth geometries (sphere, ellipsoid, cone, etc) which have
//   well-known closed-form values for the tensor product. Confirm that as
//   the tesselation gets finer, that the answer converges to the reference
//   solution.

}  // namespace
}  // namespace fcl

//==============================================================================
int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


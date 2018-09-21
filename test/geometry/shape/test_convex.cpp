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
#include "fcl/common/types.h"

namespace fcl {
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
  virtual S volume() const = 0;
  virtual Vector3<S> com() const = 0;
  virtual Matrix3<S> principal_inertia_tensor() const = 0;
  virtual std::string description() const = 0;

  // The scale of the polytope to use with test tolerances.
  S scale() const { return scale_; }
  std::shared_ptr<const std::vector<Vector3<S>>> points() const {
    return vertices_;
  }
  std::shared_ptr<const std::vector<int>> polygons() const {
    return polygons_;
  }
  Convex<S> MakeConvex() const {
    // The Polytope class makes the pointers to vertices and faces const access.
    // The Convex class calls for non-const pointers. Temporarily const-casting
    // them to make it compatible.
    return Convex<S>(points(), face_count(), polygons());
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

 private:
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
  explicit EquilateralTetrahedron(S scale) : Polytope<S>(scale), scale_(scale) {
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
  virtual S volume() const final {
    // This assumes unit mass.
    S s = this->scale();
    return S(sqrt(2) / 12) * s * s * s;
  }
  virtual Vector3<S> com() const final { return Vector3<S>::Zero(); }
  virtual Matrix3<S> principal_inertia_tensor() const {
    // TODO(SeanCurtis-TRI): Replace this with a legitimate tensor.
    throw std::logic_error("Not implemented yet");
  };
  std::string description() const final {
    return "Tetrahedron with scale: " + std::to_string(this->scale());
  }

 private:
  S scale_{0};
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


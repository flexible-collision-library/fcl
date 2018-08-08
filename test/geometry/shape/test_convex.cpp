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

#include <gtest/gtest.h>

#include "eigen_matrix_compare.h"
#include "fcl/common/types.h"

namespace fcl {
namespace {

// Base definition of a convex polytope. Specific instances should define faces,
// vertices, and quantities such as volume, center of mass, and moment of
// inertia.
template <typename S>
class Polytope {
 public:
  // The scale of the polytope to use with test tolerances.
  virtual S scale() const = 0;
  virtual int face_count() const = 0;
  virtual int point_count() const = 0;
  virtual S volume() const = 0;
  virtual Vector3<S> com() const = 0;
  virtual Matrix3<S> principal_inertia_tensor() const = 0;
  virtual std::string description() const = 0;

  const Vector3<S>* points() const { return &vertices_[0]; }
  const int* polygons() const { return &polygons_[0]; }
  Convex<S> MakeConvex() const {
    // NOTE: The plane normals and distances aren't actually used.
    // The Polytope class makes the pointers to vertices and faces const access.
    // The Convex class calls for non-const pointers. Temporarily const-casting
    // them to make it compatible.
    return Convex<S>(point_count(), const_cast<Vector3<S>*>(points()),
                     face_count(), const_cast<int*>(polygons()));
  }
  Vector3<S> min_point() const {
    Vector3<S> m;
    m.setConstant(std::numeric_limits<S>::max());
    for (const Vector3<S>& v : vertices_) {
      for (int i = 0; i < 3; ++i) {
        if (v(i) < m(i)) m(i) = v(i);
      }
    }
    return m;
  }
  Vector3<S> max_point() const {
    Vector3<S> m;
    m.setConstant(-std::numeric_limits<S>::max());
    for (const Vector3<S>& v : vertices_) {
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
    for (auto& v : vertices_) {
      v = X_WP * v;
    }
  }

 protected:
  void add_vertex(const Vector3<S>& vertex) { vertices_.push_back(vertex); }
  void add_face(std::initializer_list<int> indices) {
    polygons_.push_back(static_cast<int>(indices.size()));
    polygons_.insert(polygons_.end(), indices);
  }
  // Confirms the number of vertices and number of polygons matches the counts
  // implied by point_count() and face_count(), respectively.
  void confirm_data() {
    // Confirm point count.
    GTEST_ASSERT_EQ(point_count(), static_cast<int>(vertices_.size()));

    // Confirm face count.
    // Count the number of faces encoded in polygons_;
    int count = 0;
    int i = 0;
    while (i < static_cast<int>(polygons_.size())) {
      ++count;
      i += polygons_[i] + 1;
    }
    GTEST_ASSERT_EQ(i, static_cast<int>(polygons_.size()))
                  << "Badly defined polygons";
    GTEST_ASSERT_EQ(face_count(), count);
  }

 private:
  std::vector<Vector3<S>> vertices_;
  std::vector<int> polygons_;
};

// A simple regular tetrahedron with edges of length 1 centered on the origin.
template <typename S>
class EquilateralTetrahedron : public Polytope<S> {
 public:
  // Constructs the tetrahedron (of edge length `s`).
  explicit EquilateralTetrahedron(S scale) : Polytope<S>(), scale_(scale) {
    // Tetrahedron vertices in the tet's canonical frame T. The tet is
    // "centered" on the origin so that it's center of mass is simple [0, 0, 0].
    const S z_base = -1 / S(2 * sqrt(6.));
    Vector3<S> points_T[] = {{S(0.5), S(-0.5 / sqrt(3)), z_base},
                             {S(-0.5), S(-0.5 / sqrt(3)), z_base},
                             {S(0), S(1) / sqrt(3), z_base},
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
  S scale() const final { return scale_; }
  int face_count() const final { return 4; }
  int point_count() const final { return 4; }
  virtual S volume() const final {
    return S(sqrt(2) / 12) * scale_ * scale_ * scale_;
  }
  virtual Vector3<S> com() const final { return Vector3<S>::Zero(); }
  virtual Matrix3<S> principal_inertia_tensor() const {
    // TODO(SeanCurtis-TRI): Replace this with a legitimate tensor.
    throw std::logic_error("Not implemented yet");
  };
  std::string description() const final {
    return "Tetrahedron with scale: " + std::to_string(scale_);
  }

 private:
  S scale_{0};
};

// A simple box with sides of length 1.
template <typename S>
class UnitBox : public Polytope<S> {
 public:
  UnitBox() : Polytope<S>() {
    // Box vertices in the box's canonical frame B.
    Vector3<S> points_B[] = {{S(-0.5), S(-0.5), S(-0.5)},   // v0
                             {S(0.5), S(-0.5), S(-0.5)},    // v1
                             {S(-0.5), S(0.5), S(-0.5)},    // v2
                             {S(0.5), S(0.5), S(-0.5)},     // v3
                             {S(-0.5), S(-0.5), S(0.5)},    // v4
                             {S(0.5), S(-0.5), S(0.5)},     // v5
                             {S(-0.5), S(0.5), S(0.5)},     // v6
                             {S(0.5), S(0.5), S(0.5)}};     // v7
    for (const auto& v : points_B) {
      this->add_vertex(v);
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
  S scale() const final { return 1; }
  int face_count() const final { return 6; }
  int point_count() const final { return 8; }
  virtual S volume() const final { return S(1.); }
  virtual Vector3<S> com() const final { return Vector3<S>::Zero(); }
  virtual Matrix3<S> principal_inertia_tensor() const {
    return Eigen::DiagonalMatrix<S, 3>(1. / 6., 1. / 6., 1. / 6.);
  };
  std::string description() const final {
    return "UnitBox";
  }
};

void testConvexConstruction() {
  UnitBox<double> box;
  // Set the box at some other location to make sure that the mean point test
  // doesn't pass just because it initialized to zero.
  Vector3<double> p_WB(1, 2, 3);
  box.SetPose(Transform3<double>(Eigen::Translation3d(p_WB)));
  Convex<double> convex = box.MakeConvex();

  // This doesn't depend on the correct logic in the constructor. But this is
  // as convenient a time as any to test that it reports the right node type.
  EXPECT_EQ(convex.getNodeType(), GEOM_CONVEX);

  // The constructor computes the mean point (which it calls "center").
  EXPECT_TRUE(CompareMatrices(convex.interior_point, p_WB));
}

template <template <typename> class Shape, typename S>
void testABBComputation(const Shape<S>& model, const Transform3<S>& X_WS) {
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
void testVolume(const Shape<S>& model, const Transform3<S>& X_WS) {
  Shape<S> shape(model);
  shape.SetPose(X_WS);
  Convex<S> convex = shape.MakeConvex();

  typename constants<S>::Real eps = constants<S>::eps();
  EXPECT_NEAR(shape.volume(), convex.computeVolume(), eps)
            << shape.description() << " at\n" << X_WS.matrix();
}

template <template <typename> class Shape, typename S>
void testCenterOfMass(const Shape<S>& model, const Transform3<S>& X_WS) {
  Shape<S> shape(model);
  shape.SetPose(X_WS);
  Convex<S> convex = shape.MakeConvex();
  // In the worst case (with arbitrary frame orientations) it seems like I'm
  // losing about 3 bits of precision in the solution (compared to performing
  // the equivalent query without any rotations). This encodes that bit loss to
  // an epsilon value appropriate to the scalar type.
  //
  // For *small* polytopes, we lose further precision. Each polytope reports a
  // single scalar indicating its scale; we assume the error is correlated to
  // the volume, so we take the cube of the scale factor. But for *large*
  // geometry we don't let the tolerance get arbitrarily small.
  S scale = shape.scale() * shape.scale() * shape.scale();
  typename constants<S>::Real tolerance =
      8 * constants<S>::eps() / ((scale < 1) ? scale : 1);
  EXPECT_TRUE(
      CompareMatrices(X_WS * shape.com(), convex.computeCOM(), tolerance))
            << shape.description() << " at\n" << X_WS.matrix();
}

template <template <typename> class Shape, typename S>
void testMomentOfInertia(const Shape<S>& model, const Transform3<S>& X_WS) {
  Shape<S> shape(model);
  shape.SetPose(X_WS);
  Convex<S> convex = shape.MakeConvex();
  // In the worst case (with arbitrary frame orientations) it seems like we're
  // losing about 4 bits of precision in the solution (compared to performing
  // the equivalent query without any rotations). This encodes that bit loss to
  // an epsilon value appropriate to the scalar type.
  //
  // For *small* polytopes, we lose further precision. Each polytope reports a
  // single scalar indicating its scale; we assume the error is correlated to
  // the volume, so we take the cube of the scale factor. But for *large*
  // geometry we don't let the tolerance get arbitrarily small.
  S scale = shape.scale() * shape.scale() * shape.scale();
  typename constants<S>::Real tolerance =
      16 * constants<S>::eps() / ((scale < 1) ? scale : 1);
  EXPECT_TRUE(
      CompareMatrices(X_WS.linear().transpose() *
                          shape.principal_inertia_tensor() * X_WS.linear(),
                      convex.computeMomentofInertiaRelatedToCOM(), tolerance))
            << shape.description() << " at\n" << X_WS.matrix();
}

template <typename S>
std::vector<Transform3<S>> GetPoses() {
  std::vector<Transform3<S>> poses;
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

  // Simple translation.
  X_WS.linear() = Matrix3<S>::Identity();
  X_WS.translation() << 1, -2, 3;
  poses.push_back(X_WS);

  return poses;
}

template <template <typename> class Shape, typename S>
void testLocalAABBComputation(const Shape<S>& shape) {
  for (const auto& X_WP : GetPoses<S>()) {
    testABBComputation<Shape>(shape, X_WP);
  }
}

template <template <typename> class Shape, typename S>
void testVolumeComputation(const Shape<S>& shape) {
  for (const auto& X_WP : GetPoses<S>()) {
    testVolume<Shape>(shape, X_WP);
  }
}

template <template <typename> class Shape, typename S>
void testCenterOfMassComputation(const Shape<S>& shape) {
  for (const auto& X_WP : GetPoses<S>()) {
    testCenterOfMass<Shape>(shape, X_WP);
  }
}

template <template <typename> class Shape, typename S>
void testMomentOfInertiaComputation(const Shape<S>& shape) {
  for (const auto& X_WP : GetPoses<S>()) {
    testMomentOfInertia<Shape>(shape, X_WP);
  }
}

GTEST_TEST(ConvexGeometry, Constructor) {
  testConvexConstruction();
}

GTEST_TEST(ConvexGeometry, LocalAABBComputation_Box) {
  UnitBox<double> box_d;
  testLocalAABBComputation(box_d);
  UnitBox<float> box_f;
  testLocalAABBComputation(box_f);
}

GTEST_TEST(ConvexGeometry, Volume_Box) {
  UnitBox<double> box_d;
  testVolumeComputation(box_d);
  UnitBox<float> box_f;
  testVolumeComputation(box_f);
}

GTEST_TEST(ConvexGeometry, CenterOfMass_Box) {
  UnitBox<double> box_d;
  testCenterOfMassComputation(box_d);
  UnitBox<float> box_f;
  testCenterOfMassComputation(box_f);
}

GTEST_TEST(ConvexGeometry, MomentOfInertia_Box) {
  UnitBox<double> box_d;
  testMomentOfInertiaComputation(box_d);
  UnitBox<float> box_f;
  testMomentOfInertiaComputation(box_f);
}

GTEST_TEST(ConvexGeometry, Volume_Tetrahedron) {
  EquilateralTetrahedron<double> tet_d(1.);
  testLocalAABBComputation(tet_d);
  EquilateralTetrahedron<float> tet_f(1.);
  testLocalAABBComputation(tet_f);
}

GTEST_TEST(ConvexGeometry, LocalAABBComputation_Tetrahedron) {
  EquilateralTetrahedron<double> tet_d(1.);
  testVolumeComputation(tet_d);
  EquilateralTetrahedron<float> tet_f(1.);
  testVolumeComputation(tet_f);
}

GTEST_TEST(ConvexGeometry, CenterOfMass_Tetrahedron) {
  for (double scale : {0.1, 1., 15.}) {
    EquilateralTetrahedron<double> tet_d(scale);
    testCenterOfMassComputation(tet_d);
    EquilateralTetrahedron<float> tet_f(static_cast<float>(scale));
    testCenterOfMassComputation(tet_f);
  }
}

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


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

// Tests the implementation of a convex hull.

#include "fcl/geometry/shape/convex.h"

#include <gtest/gtest.h>

#include "eigen_matrix_compare.h"
#include "fcl/common/types.h"

// TODO(SeanCurtis-TRI): This is only the *first* component of the tests for
// the geometry representation. The following list of features must *also* be
// tested
//   - computeMomentofInertia
//   - computeCOM()
//   - computeVolume()
//   - fillEdges() (via constructor)
//   - getBoundVertices()
//
// The tests should also include some *other* convex shape where the planes are
// not mutually orthogonal.

namespace fcl {

// A simple box with sides of length 1. It can provide the definition of its
// own convex hull.
template <typename S>
class UnitBox {
 public:
  // Instantiate the box with the given pose of the box in the world frame
  // (defaults to the identity pose).
  explicit UnitBox(const Transform3<S>& X_WB = Transform3<S>::Identity()) :
      mean_point_(X_WB.translation()){
    // Note: gcc-4.8 does *not* allow normal_ and points_ to be initialized
    // with these initializer lists at declaration. So, for broadest
    // compatability, they are initialized here.
    Vector3<S> normals_B[kFaceCount] = {{1, 0, 0},     // +x
                                        {-1, 0, 0},    // -x
                                        {0, 1, 0},     // +y
                                        {0, -1, 0},    // -y
                                        {0, 0, 1},     // +z
                                        {0, 0, -1}};   // -z
    for (int f = 0; f < kFaceCount; ++f) {
      normals_[f] = X_WB.linear() * normals_B[f];
    }

    Vector3<S> points_B[kPointCount] = {{S(-0.5), S(-0.5), S(-0.5)},
                                        {S(0.5), S(-0.5), S(-0.5)},
                                        {S(-0.5), S(0.5), S(-0.5)},
                                        {S(0.5), S(0.5), S(-0.5)},
                                        {S(-0.5), S(-0.5), S(0.5)},
                                         {S(0.5), S(-0.5), S(0.5)},
                                        {S(-0.5), S(0.5), S(0.5)},
                                        {S(0.5), S(0.5), S(0.5)}};
    for (int p = 0; p < kPointCount; ++p) {
      points_[p] = X_WB * points_B[p];
    }
  }

  // Getters for convex hull instantiation.
  Vector3<S>* plane_normals() { return &normals_[0]; }
  S* plane_offsets() { return &offsets_[0]; }
  int plane_count() const { return kFaceCount; }
  Vector3<S>* points() { return &points_[0]; }
  int point_count() const { return kPointCount; }
  int* polygons() { return &polygons_[0]; }
  Vector3<S> mean_point() const { return mean_point_; }
  S aabb_radius() const { return (min_point() - mean_point_).norm(); }
  Convex<S> hull() {
    return Convex<S>(plane_normals(), plane_offsets(), kFaceCount,
                     points(), kPointCount, polygons());
  }
  Vector3<S> min_point() const {
    Vector3<S> m;
    m.setConstant(std::numeric_limits<S>::max());
    for (int p = 0; p < kPointCount; ++p) {
      for (int i = 0; i < 3; ++i) {
        if (points_[p](i) < m(i)) m(i) = points_[p](i);
      }
    }
    return m;
  }
  Vector3<S> max_point() const {
    Vector3<S> m;
    m.setConstant(-std::numeric_limits<S>::max());
    for (int p = 0; p < kPointCount; ++p) {
      for (int i = 0; i < 3; ++i) {
        if (points_[p](i) > m(i)) m(i) = points_[p](i);
      }
    }
    return m;
  }

 private:
  static constexpr int kFaceCount = 6;
  static constexpr int kPointCount = 8;
  // The implicit equations of the six planes of the cube.
  Vector3<S> normals_[kFaceCount];
  S offsets_[kFaceCount] = {S(-0.5), S(-0.5), S(-0.5), S(-0.5), S(-0.5),
                            S(-0.5)};
  // The intersecting points of the planes (i.e., corners of the cube).
  Vector3<S> points_[kPointCount];
  // The convex-hull encoding of the cube faces. As a reality check, each vertex
  // index should appear three times.
  int polygons_[kFaceCount * 5] = {4, 1, 3, 7, 5,    // +x
                                   4, 0, 4, 6, 2,    // -x
                                   4, 4, 5, 7, 6,    // +y
                                   4, 0, 2, 3, 1,    // -y
                                   4, 6, 7, 3, 2,    // +z
                                   4, 0, 1, 5, 4};   // -z
  Vector3<S> mean_point_;
};

template <typename S>
void testConvexConstruction(const Transform3<S>& X_WB) {
  UnitBox<S> box(X_WB);
  Convex<S> hull = box.hull();

  // This doesn't depend on the correct logic in the constructor. But this is
  // as convenient a time as any to test that it reports the right node type.
  EXPECT_EQ(hull.getNodeType(), GEOM_CONVEX);

  // The constructor does work. It computes the mean point (which it calls
  // "center") and defines the edges. Let's confirm they are correct.
  EXPECT_TRUE(CompareMatrices(hull.center, box.mean_point()));
  GTEST_ASSERT_NE(hull.edges, nullptr);
  // TODO(SeanCurtis-TRI): Test the edge definitions.
}

template <typename S>
void testABBComputation(const Transform3<S>& X_WB) {
  UnitBox<S> box(X_WB);
  Convex<S> hull = box.hull();
  hull.computeLocalAABB();

  typename constants<S>::Real eps = constants<S>::eps();
  EXPECT_NEAR(box.aabb_radius(), hull.aabb_radius, eps);
  EXPECT_TRUE(CompareMatrices(box.mean_point(), hull.aabb_center, eps));
  EXPECT_TRUE(CompareMatrices(box.min_point(), hull.aabb_local.min_, eps));
  EXPECT_TRUE(CompareMatrices(box.max_point(), hull.aabb_local.max_, eps));
}

GTEST_TEST(ConvexGeometry, Constructor) {
  testConvexConstruction(Transform3<double>::Identity());
  testConvexConstruction(Transform3<float>::Identity());
}


GTEST_TEST(ConvexGeometry, LocalAABBComputation) {
  Transform3<double> X_WB = Transform3<double>::Identity();

  // Identity.
  testABBComputation(X_WB);
  testABBComputation(X_WB.cast<float>());

  // 90-degree rotation around each axis, in turn.
  for (int i = 0; i < 3; ++i) {
    X_WB.linear() = AngleAxis<double>(constants<double>::pi(),
                                      Vector3<double>::Unit(i)).matrix();
    testABBComputation(X_WB);
    testABBComputation(X_WB.cast<float>());
  }

  // Small angle away from identity.
  X_WB.linear() = AngleAxis<double>(1e-5, Vector3<double>{1, 2, 3}.normalized())
      .matrix();
  testABBComputation(X_WB);
  testABBComputation(X_WB.cast<float>());

  // Simple translation.
  X_WB.linear() = Matrix3<double>::Identity();
  X_WB.translation() << 1, -2, 3;
  testABBComputation(X_WB);
  testABBComputation(X_WB.cast<float>());
}

}  // namespace fcl

//==============================================================================
int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


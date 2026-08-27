#include <cmath>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "eigen_matrix_compare.h"
#include "fcl/common/types.h"
#include "fcl/geometry/shape/box.h"
#include "fcl/geometry/shape/halfspace.h"
#include "fcl/narrowphase/collision_object.h"

namespace fcl {
namespace {

// Room for the rounding of a rotation applied to a point of the given
// magnitude. The bound is exact, so a corner can land a couple of units in
// the last place outside the box it touches.
template <typename S>
S Tolerance(S magnitude) {
  return S(64) * std::numeric_limits<S>::epsilon() * std::max(S(1), magnitude);
}

template <typename S>
std::shared_ptr<Box<S>> MakeBox(const Vector3<S>& half_widths) {
  return std::make_shared<Box<S>>(2 * half_widths[0], 2 * half_widths[1],
                                  2 * half_widths[2]);
}

// An unrotated geometry keeps its local box, moved by the translation.
template <typename S>
void TestIdentityRotation() {
  const Vector3<S> half_widths(S(10), S(5), S(0.05));
  Transform3<S> X_WG = Transform3<S>::Identity();
  X_WG.translation() = Vector3<S>(S(1), S(2), S(3));

  const CollisionObject<S> object(MakeBox(half_widths), X_WG);

  const S tolerance = Tolerance<S>(half_widths.norm());
  EXPECT_TRUE(CompareMatrices(object.getAABB().min_,
                              X_WG.translation() - half_widths, tolerance));
  EXPECT_TRUE(CompareMatrices(object.getAABB().max_,
                              X_WG.translation() + half_widths, tolerance));
}

// A rotated box is bounded by the rotated local box, whose half-widths are
// |R| h. For a thin box turned 45 degrees about z, that is much tighter on
// the thin axis than the cube of half-width aabb_radius that bounds the
// circumscribing sphere.
template <typename S>
void TestRotatedBoxMatchesAnalyticBound() {
  const Vector3<S> half_widths(S(10), S(5), S(0.05));
  const auto box = MakeBox(half_widths);
  Transform3<S> X_WG = Transform3<S>::Identity();
  X_WG.linear() = AngleAxis<S>(constants<S>::pi() / 4, Vector3<S>::UnitZ())
                      .toRotationMatrix();
  X_WG.translation() = Vector3<S>(S(1), S(2), S(3));

  const CollisionObject<S> object(box, X_WG);

  // Rotating about z mixes the first two half-widths through cos(45 degrees)
  // and leaves the third alone.
  const S mixed = (half_widths[0] + half_widths[1]) / std::sqrt(S(2));
  const Vector3<S> expected(mixed, mixed, half_widths[2]);

  const S tolerance = Tolerance<S>(half_widths.norm());
  EXPECT_TRUE(CompareMatrices(object.getAABB().min_,
                              X_WG.translation() - expected, tolerance));
  EXPECT_TRUE(CompareMatrices(object.getAABB().max_,
                              X_WG.translation() + expected, tolerance));

}

// Rotating by 120 degrees about the (1, 1, 1) diagonal permutes the axes, so
// the bound's half-widths are the geometry's half-widths permuted. This is an
// exact case whose rotation is all zeros and ones.
template <typename S>
void TestAxisPermutingRotation() {
  const Vector3<S> half_widths(S(4), S(2), S(0.5));
  Transform3<S> X_WG = Transform3<S>::Identity();
  X_WG.linear() =
      AngleAxis<S>(2 * constants<S>::pi() / 3,
                   Vector3<S>(S(1), S(1), S(1)).normalized())
          .toRotationMatrix();

  const CollisionObject<S> object(MakeBox(half_widths), X_WG);

  const Vector3<S> expected(half_widths[2], half_widths[0], half_widths[1]);
  const S tolerance = Tolerance<S>(half_widths.norm());
  EXPECT_TRUE(CompareMatrices(object.getAABB().min_, -expected, tolerance));
  EXPECT_TRUE(CompareMatrices(object.getAABB().max_, expected, tolerance));
}

// A posed box, described by the pieces the test builds it from.
template <typename S>
struct PosedBox {
  const char* description;
  Vector3<S> half_widths;
  Vector3<S> rotation_axis;
  S rotation_angle;
  Vector3<S> translation;
};

// Whatever the pose, the bound must contain the geometry.
template <typename S>
void TestBoundContainsGeometry() {
  const S kQuarterTurn = constants<S>::pi() / 2;
  const PosedBox<S> cases[] = {
      // A thin plate turned within its own plane, which is what convex
      // decomposition produces and how such pieces usually rest.
      {"plate turned in plane",
       {S(1), S(0.5), S(0.01)},
       {S(0), S(0), S(1)},
       constants<S>::pi() / 4,
       {S(0), S(0), S(0)}},
      // The same plate under a rotation that mixes all three axes.
      {"plate, general rotation",
       {S(1), S(0.5), S(0.01)},
       {S(1), S(2), S(3)},
       S(0.7),
       {S(0.25), S(-0.5), S(2)}},
      // An elongated rod, the other extreme shape.
      {"rod",
       {S(5), S(0.05), S(0.05)},
       {S(0), S(1), S(0)},
       constants<S>::pi() / 6,
       {S(0), S(0), S(0)}},
      // A cube on its diagonal, where the two bounds are closest together.
      {"cube on its diagonal",
       {S(1), S(1), S(1)},
       {S(1), S(1), S(1)},
       2 * constants<S>::pi() / 3,
       {S(0), S(0), S(0)}},
      // A quarter turn leaves exact zeros in the rotation.
      {"quarter turn, near-degenerate extent",
       {S(2), S(2), S(1e-6)},
       {S(1), S(0), S(0)},
       kQuarterTurn,
       {S(0), S(0), S(0)}},
      // Far from the origin, where the center term dominates the half-width.
      {"far from the origin",
       {S(0.5), S(3), S(0.25)},
       {S(0), S(0), S(1)},
       S(0.2),
       {S(100), S(-50), S(7)}},
  };

  for (const PosedBox<S>& test_case : cases) {
    SCOPED_TRACE(test_case.description);
    const auto box = MakeBox(test_case.half_widths);
    Transform3<S> X_WG = Transform3<S>::Identity();
    X_WG.linear() = AngleAxis<S>(test_case.rotation_angle,
                                 test_case.rotation_axis.normalized())
                        .toRotationMatrix();
    X_WG.translation() = test_case.translation;

    const CollisionObject<S> object(box, X_WG);
    const AABB<S>& aabb = object.getAABB();
    const S tolerance = Tolerance<S>(test_case.half_widths.norm() +
                                     test_case.translation.norm());

    for (int corner = 0; corner < 8; ++corner) {
      const Vector3<S> p_GC(
          (corner & 1) ? test_case.half_widths[0] : -test_case.half_widths[0],
          (corner & 2) ? test_case.half_widths[1] : -test_case.half_widths[1],
          (corner & 4) ? test_case.half_widths[2] : -test_case.half_widths[2]);
      const Vector3<S> p_WC = X_WG * p_GC;
      EXPECT_TRUE(((p_WC - aabb.min_).array() >= -tolerance).all())
          << "corner " << corner;
      EXPECT_TRUE(((aabb.max_ - p_WC).array() >= -tolerance).all())
          << "corner " << corner;
    }
  }
}

template <typename S>
void TestUnboundedGeometry() {
  const auto halfspace =
      std::make_shared<Halfspace<S>>(Vector3<S>(0, 0, 1), S(0));

  // The first rotation is about the half space's own normal, which leaves
  // rows of exact zeros in the rotation; the second mixes all three axes.
  const Matrix3<S> rotations[] = {
      AngleAxis<S>(constants<S>::pi() / 4, Vector3<S>::UnitZ())
          .toRotationMatrix(),
      AngleAxis<S>(S(0.7), Vector3<S>(S(1), S(2), S(3)).normalized())
          .toRotationMatrix()};

  for (const Matrix3<S>& rotation : rotations) {
    Transform3<S> X_WG = Transform3<S>::Identity();
    X_WG.linear() = rotation;
    const CollisionObject<S> object(halfspace, X_WG);
    const AABB<S>& aabb = object.getAABB();

    for (int i = 0; i < 3; ++i) {
      ASSERT_FALSE(std::isnan(aabb.min_[i])) << "axis " << i;
      ASSERT_FALSE(std::isnan(aabb.max_[i])) << "axis " << i;
      EXPECT_TRUE(std::isinf(aabb.min_[i]) && aabb.min_[i] < S(0))
          << "axis " << i;
      EXPECT_TRUE(std::isinf(aabb.max_[i]) && aabb.max_[i] > S(0))
          << "axis " << i;
    }
  }
}

GTEST_TEST(CollisionObjectAABB, IdentityRotation) {
  TestIdentityRotation<double>();
  TestIdentityRotation<float>();
}

GTEST_TEST(CollisionObjectAABB, RotatedBoxMatchesAnalyticBound) {
  TestRotatedBoxMatchesAnalyticBound<double>();
  TestRotatedBoxMatchesAnalyticBound<float>();
}

GTEST_TEST(CollisionObjectAABB, AxisPermutingRotation) {
  TestAxisPermutingRotation<double>();
  TestAxisPermutingRotation<float>();
}

GTEST_TEST(CollisionObjectAABB, BoundContainsGeometry) {
  TestBoundContainsGeometry<double>();
  TestBoundContainsGeometry<float>();
}

GTEST_TEST(CollisionObjectAABB, UnboundedGeometry) {
  TestUnboundedGeometry<double>();
  TestUnboundedGeometry<float>();
}

}  // namespace
}  // namespace fcl

//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

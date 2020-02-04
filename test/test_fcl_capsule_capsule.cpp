/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Toyota Research Institute, Inc.
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
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
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

/** @author Sean Curtis <sean@tri.global> */

#include <gtest/gtest.h>

#include "fcl/math/constants.h"
#include "fcl/narrowphase/detail/primitive_shape_algorithm/capsule_capsule-inl.h"
#include "eigen_matrix_compare.h"

#include <cmath>
using namespace fcl;

// Test harness for exercising the closestPtSegmentSegment() function.
template <typename S>
class SegmentSegmentNearestPtTest : public ::testing::Test {
 protected:
  // Calls the closestPtSegmentSegment with the two segments defined by
  // endpoint pairs (p0, q0) and (p1, q1). The values s_, t_, n0_, and n1_ will
  // be updated as a result of this call.
  // Returns the squared distance between the two segments.
  S ComputeNearestPoint(const Vector3<S>& p0, const Vector3<S>& q0,
                        const Vector3<S>& p1, const Vector3<S>& q1) {
    return detail::closestPtSegmentSegment(p0, q0, p1, q1, &s_, &t_, &n0_,
                                           &n1_);
  }
  // s_ and t_ will contain the lengths of the two segments after calling
  // the nearest point function.
  S s_;
  S t_;
  // n0_ and n1_ will contain the points on the segments nearest each other
  // after calling the nearest point function.
  Vector3<S> n0_;
  Vector3<S> n1_;
};

using ScalarTypes = ::testing::Types<double, float>;

TYPED_TEST_CASE(SegmentSegmentNearestPtTest, ScalarTypes);

TYPED_TEST(SegmentSegmentNearestPtTest, BothZeroLength) {
  using S = TypeParam;

  // Keep the points near the unit sphere so that the epsilon value doesn't
  // need to be scaled.
  const Vector3<S> p0{S(0.25), S(0.5), S(1.0)};
  const Vector3<S> p1{S(0.6), S(1.0), S(-0.3)};

  // Case: Exactly zero.
  {
    const S squared_dist = this->ComputeNearestPoint(p0, p0, p1, p1);
    EXPECT_EQ(squared_dist, (p1 - p0).squaredNorm());
    EXPECT_TRUE(CompareMatrices(this->n0_, p0));
    EXPECT_TRUE(CompareMatrices(this->n1_, p1));
  }

  // We exploit the knowledge that the segment-segment function uses an
  // epsilon of eps_78()² on *squared distance*. This test will fail if we
  // change the epsilon in the algorithm and these brackets should move
  // accordingly.
  using std::sqrt;
  const S kEps = constants<S>::eps_78();

  // Case: Zero to within epsilon.
  {
    // Just below the epsilon for squared distance.
    const S eps = kEps * S(0.99);
    const Vector3<S> q0(p0(0) + eps, p0(1), p0(2));
    const Vector3<S> q1(p1(0) + eps, p1(1), p1(2));
    const S squared_dist = this->ComputeNearestPoint(p0, q0, p1, q1);
    EXPECT_EQ(squared_dist, (p1 - p0).squaredNorm());
    EXPECT_TRUE(CompareMatrices(this->n0_, p0));
    EXPECT_TRUE(CompareMatrices(this->n1_, p1));
  }

  // Case: Just outside epsilon length will no longer return the distance
  // between points p0 and p1. All that we care is that the distance is no
  // longer *exactly* the distance between p0 and p1. Thus it shows that we've
  // crossed the algorithm's threshold.
  {
    // Just above the epsilon for squared distance.
    const S eps = kEps * S(1.5);
    // Make sure the line segments are *not* parallel so we don't exercise any
    // degenerate cases.
    const Vector3<S> q0(p0(0) + eps, p0(1), p0(2));
    const Vector3<S> q1(p1(0), p1(1) + eps, p1(2));
    const S squared_dist = this->ComputeNearestPoint(p0, q0, p1, q1);
    EXPECT_NE(squared_dist, (p1 - p0).squaredNorm());
  }
}

// Tests the case where the one segment is zero length.
TYPED_TEST(SegmentSegmentNearestPtTest, OneZeroLength) {
  using S = TypeParam;

  // Segment 2 passes through the origin.
  Vector3<S> p1{S(-1), S(-2), S(-3)};
  Vector3<S> q1{S(1), S(2), S(3)};

  // Case: First zero-length segment is nearest p1.
  {
    const Vector3<S> p0 = p1 + p1;
    const S squared_dist = this->ComputeNearestPoint(p0, p0, p1, q1);
    EXPECT_EQ(squared_dist, (p1 - p0).squaredNorm());
    EXPECT_TRUE(CompareMatrices(this->n0_, p0));
    EXPECT_TRUE(CompareMatrices(this->n1_, p1));
  }

  // Case: First zero-length segment is nearest q1.
  {
    const Vector3<S> p0 = q1 + q1;
    const S squared_dist = this->ComputeNearestPoint(p0, p0, p1, q1);
    EXPECT_EQ(squared_dist, (q1 - p0).squaredNorm());
    EXPECT_TRUE(CompareMatrices(this->n0_, p0));
    EXPECT_TRUE(CompareMatrices(this->n1_, q1));
  }

  // Case: First zero-length segment is nearest interior of second segment.
  {
    const Vector3<S> dir1 = q1 - p1;
    // Create a vector perpendicular to the second segment, closest to the
    // origin.
    const Vector3<S> p0(dir1(1), -dir1(0), 0);
    const S squared_dist = this->ComputeNearestPoint(p0, p0, p1, q1);
    EXPECT_EQ(squared_dist, p0.squaredNorm());
    EXPECT_TRUE(CompareMatrices(this->n0_, p0));
    EXPECT_TRUE(
        CompareMatrices(this->n1_, Vector3<S>::Zero(), constants<S>::eps_78()));
  }

  // Now we want to reverse the roles (such that the second segment has zero
  // length) but otherwise perform the same tests. So, we'll alias the old
  // p1, q1 as p0, q0 and then modify p1, q1 to support the equivalent tests.

  const Vector3<S> p0(p1);
  const Vector3<S> q0(q1);

  // Case: Second zero-length segment is nearest p0.
  {
    const Vector3<S> p1 = p0 + p0;
    const S squared_dist = this->ComputeNearestPoint(p0, q0, p1, p1);
    EXPECT_EQ(squared_dist, (p1 - p0).squaredNorm());
    EXPECT_TRUE(CompareMatrices(this->n0_, p0));
    EXPECT_TRUE(CompareMatrices(this->n1_, p1));
  }

  // Case: Second zero-length segment is nearest q0.
  {
    const Vector3<S> p1 = q0 + q0;
    const S squared_dist = this->ComputeNearestPoint(p0, q0, p1, p1);
    EXPECT_EQ(squared_dist, (q0 - p1).squaredNorm());
    EXPECT_TRUE(CompareMatrices(this->n0_, q0));
    EXPECT_TRUE(CompareMatrices(this->n1_, p1));
  }

  // Case: Second zero-length segment is nearest interior of first segment.
  {
    const Vector3<S> dir1 = q0 - p0;
    // Create a vector perpendicular to the second segment, closest to the
    // origin.
    const Vector3<S> p1(dir1(1), -dir1(0), S(0));
    const S squared_dist = this->ComputeNearestPoint(p0, q0, p1, p1);
    EXPECT_EQ(squared_dist, p1.squaredNorm());
    EXPECT_TRUE(CompareMatrices(this->n1_, p1));
    EXPECT_TRUE(
        CompareMatrices(this->n0_, Vector3<S>::Zero(), constants<S>::eps_78()));
  }
}

// Tests the case where the line segments have length and are _parallel_.
TYPED_TEST(SegmentSegmentNearestPtTest, ParallelSegments) {
  using S = TypeParam;

  const S eps = constants<S>::eps_78();

  // A line to which all of the segments will be parallel.
  const Vector3<S> dir = Vector3<S>{S(1), S(2), S(3)}.normalized();
  // A direction perpendicular to the line given by `dir`.
  const Vector3<S> dir_perp = Vector3<S>{dir(1), -dir(0), S(0)}.normalized();
  ASSERT_NEAR(dir.dot(dir_perp), 0, eps);

  // In the case where there are multiple possible pairs (because the solution
  // is not unique). While we could directly express these tests based on known
  // details of the implementation, we decouple the test from those details by
  // equivalently testing for:
  //   1. The distance between the pairs should be the expected distance.
  //   2. We can determine that the nearest point lies in the correct region
  //      of one of the two segments.
  // Then the implementation can use any policy it chooses to pick one point
  // from the set and the test won't have to change.

  // Case: Multiple nearest pairs: co-linear and overlapping.
  {
    // Segment 0 is symmetric and passes through the origin.
    const Vector3<S> p0(-dir);
    const Vector3<S> q0(dir);
    // Segment 1 includes the origin and passes through q1 extending a similar
    // distance beyond.
    const Vector3<S> p1(Vector3<S>::Zero());
    const Vector3<S> q1(S(2) * dir);
    const S squared_dist = this->ComputeNearestPoint(p0, q0, p1, q1);

    EXPECT_NEAR(squared_dist, S(0), eps);

    // Test pair: distance between them and that n0 lies between the origin and
    // q0.
    EXPECT_NEAR((this->n0_ - this->n1_).norm(), S(0), eps);
    const S nearest_dist = this->n0_.norm();
    EXPECT_NEAR(nearest_dist, dir.dot(this->n0_), eps);
    EXPECT_LE(nearest_dist, q0.norm());
  }

  // Case: Multiple nearest pairs: separated parallel lines but with overlapping
  // projections.
  {
    // Same configuration as in the co-linear case, but the second segment is
    // moved perpendicularly to the first segment.
    const Vector3<S> p0(-dir);
    const Vector3<S> q0(dir);
    const S expected_dist{0.5};
    const Vector3<S> offset = dir_perp * expected_dist;
    const Vector3<S> p1 = Vector3<S>::Zero() + offset;
    const Vector3<S> q1 = S(2) * dir + offset;
    const S squared_dist = this->ComputeNearestPoint(p0, q0, p1, q1);

    EXPECT_NEAR(squared_dist, expected_dist * expected_dist, eps);
    // Test pair: distance between them and n0 lies between origin and q0.
    EXPECT_NEAR((this->n0_ - this->n1_).norm(), expected_dist, eps);
    const S nearest_dist = this->n0_.norm();
    EXPECT_NEAR(nearest_dist, dir.dot(this->n0_), eps);
    EXPECT_LE(nearest_dist, q0.norm());
  }

  // Case: Single solution: shared end point.
  {
    const Vector3<S> p0(-dir);
    const Vector3<S> q0(dir);  // also serves as p1.
    const Vector3<S> q1(S(2) * dir);

    const S squared_dist = this->ComputeNearestPoint(p0, q0, q0, q1);

    EXPECT_NEAR(squared_dist, S(0), eps);
    EXPECT_TRUE(CompareMatrices(this->n0_, q0));
    EXPECT_TRUE(CompareMatrices(this->n1_, q0));
  }

  // Case: Single solution: co-linear but non-overlapping such that (p1, q0)
  // are the nearest points.
  {
    const Vector3<S> p0(-dir);
    const Vector3<S> q0(dir);
    const Vector3<S> p1(S(2) * dir);
    const Vector3<S> q1(S(3) * dir);

    const S squared_dist = this->ComputeNearestPoint(p0, q0, p1, q1);

    EXPECT_NEAR(squared_dist, dir.norm(), eps);
    EXPECT_TRUE(CompareMatrices(this->n0_, q0));
    EXPECT_TRUE(CompareMatrices(this->n1_, p1));
  }

  // Case: Single solution: not co-linear and non-overlapping such that (p1, q0)
  // are the nearest points.
  {
    const Vector3<S> p0(-dir);
    const Vector3<S> q0(dir);
    const Vector3<S> p1(S(2) * dir + dir_perp);
    const Vector3<S> q1(S(3) * dir + dir_perp);

    const S squared_dist = this->ComputeNearestPoint(p0, q0, p1, q1);

    EXPECT_NEAR(squared_dist, (p1 - q0).squaredNorm(), eps);
    EXPECT_TRUE(CompareMatrices(this->n0_, q0));
    EXPECT_TRUE(CompareMatrices(this->n1_, p1));
  }

  // Case: Single solution: co-linear but non-overlapping such that (p0, q1)
  // are the nearest points.
  {
    const Vector3<S> p0(-dir);
    const Vector3<S> q0(dir);
    const Vector3<S> p1(S(-3) * dir);
    const Vector3<S> q1(S(-2) * dir);

    const S squared_dist = this->ComputeNearestPoint(p0, q0, p1, q1);

    EXPECT_NEAR(squared_dist, dir.norm(), eps);
    EXPECT_TRUE(CompareMatrices(this->n0_, p0));
    EXPECT_TRUE(CompareMatrices(this->n1_, q1));
  }

  // Case: Single solution: not co-linear and non-overlapping such that (p0, q1)
  // are the nearest points.
  {
    const Vector3<S> p0(-dir);
    const Vector3<S> q0(dir);
    const Vector3<S> p1(S(-3) * dir + dir_perp);
    const Vector3<S> q1(S(-2) * dir + dir_perp);

    const S squared_dist = this->ComputeNearestPoint(p0, q0, p1, q1);

    EXPECT_NEAR(squared_dist, (p0 - q1).squaredNorm(), eps);
    EXPECT_TRUE(CompareMatrices(this->n0_, p0));
    EXPECT_TRUE(CompareMatrices(this->n1_, q1));
  }
}

// Tests the nominal case where the segments have length and are *not* parallel.
TYPED_TEST(SegmentSegmentNearestPtTest, NominalSegments) {
  using S = TypeParam;

  const S eps = constants<S>::eps_78();

  // An arbitrary direction for segment A.
  const Vector3<S> dir_A = Vector3<S>{S(1), S(2), S(3)}.normalized();
  // An arbitrary direction for segment B such that it is neither parallel nor
  // perpendicular to dir_A. We're excluding perpendicular as that represents
  // the _best_ conditioning for the math and we want to characterize the
  // accuracy when the numbers _aren't_ that great.
  const Vector3<S> dir_B = Vector3<S>{S(-2), S(1), S(-0.5)}.normalized();
  using std::abs;
  GTEST_ASSERT_GE(abs(dir_A.dot(dir_B)), eps);
  GTEST_ASSERT_LT(abs(dir_A.dot(dir_B)), S(1) - eps);

  // A direction perpendicular to both A and B.
  const Vector3<S> perp = dir_A.cross(dir_B).normalized();

  // Configure the two segments so that they perfectly intersect at expected_n0.
  const Vector3<S> expected_n0 = Vector3<S>{S(-0.5), S(1.25), S(0.75)};
  const Vector3<S> p0 = expected_n0 + dir_A;
  const Vector3<S> q0 = expected_n0 - S(2) * dir_A;
  const Vector3<S> p1 = expected_n0 + dir_B;
  const Vector3<S> q1 = expected_n0 - S(2.5) * dir_B;
  const S expected_s = 1 / S(3);
  const S t_expected = 1 / S(3.5);

  // Case: literally intersecting (intersection point is *not* at the origin).
  {
    const S squared_dist = this->ComputeNearestPoint(p0, q0, p1, q1);

    EXPECT_NEAR(squared_dist, S(0), eps);
    EXPECT_NEAR(this->s_, expected_s, eps);
    EXPECT_NEAR(this->t_, t_expected, eps);
    EXPECT_TRUE(CompareMatrices(this->n0_, expected_n0, eps));
    EXPECT_TRUE(CompareMatrices(this->n1_, expected_n0, eps));
  }

  // Case: nearest points on interior of both. We simply separate the
  // intersecting lines in a direction perpendicular to segment A; n0 will still
  // be the same. And s and t will be unchanged.
  {
    const S expected_dist{1.25};
    const Vector3<S> offset = expected_dist * perp;
    const S squared_dist =
        this->ComputeNearestPoint(p0, q0, p1 + offset, q1 + offset);

    EXPECT_NEAR(squared_dist, expected_dist * expected_dist, eps);
    EXPECT_NEAR(this->s_, expected_s, eps);
    EXPECT_NEAR(this->t_, t_expected, eps);
    EXPECT_TRUE(CompareMatrices(this->n0_, expected_n0, eps));
    EXPECT_TRUE(CompareMatrices(this->n1_, this->n0_ + offset, eps));
  }

  // Projects the point Q onto the line segment AB and reports the projection
  // point N and the parameter u such that N = A + u(B - A)
  auto project_onto_segment = [](const Vector3<S>& A, const Vector3<S>& B,
                                 const Vector3<S>& Q, Vector3<S>* N, S* u) {
    const Vector3<S> d = B - A;
    *u = d.dot(Q - A) / d.squaredNorm();
    *N = A + *u * d;
  };

  // Case: p0 nearest to interior of B. We'll take the intersecting
  // configuration and slide segment B off of the p0 end of segment A, but
  // because they aren't perpendicular, the point on B that is nearest (and the
  // value of t), will depend on the distance.
  {
    const Vector3<S> offset = S(1.1) * (p0 - expected_n0);
    const Vector3<S> p1_shift = p1 + offset;
    const Vector3<S> q1_shift = q1 + offset;
    Vector3<S> expected_N1;
    S t_expected;
    project_onto_segment(p1_shift, q1_shift, p0, &expected_N1, &t_expected);
    const S expected_squared_dist = (p0 - expected_N1).squaredNorm();

    // Test p0 against interior of B.
    S squared_dist =
        this->ComputeNearestPoint(p0, q0, p1_shift, q1_shift);

    EXPECT_NEAR(squared_dist, expected_squared_dist, eps);
    EXPECT_NEAR(this->s_, S(0), eps);
    EXPECT_NEAR(this->t_, t_expected, eps);
    EXPECT_TRUE(CompareMatrices(this->n0_, p0, eps));
    EXPECT_TRUE(CompareMatrices(this->n1_, expected_N1, eps));

    // Test p1 against interior of A (by swapping parameter arguments). This
    // requires testing s against expected t, n0 against expected n1, and n1
    // against p0.
    squared_dist = this->ComputeNearestPoint(p1_shift, q1_shift, p0, q0);

    EXPECT_NEAR(squared_dist, expected_squared_dist, eps);
    EXPECT_NEAR(this->s_, t_expected, eps);
    EXPECT_NEAR(this->t_, S(0), eps);
    EXPECT_TRUE(CompareMatrices(this->n0_, expected_N1, eps));
    EXPECT_TRUE(CompareMatrices(this->n1_, p0, eps));
  }

  // Case: q0 nearest to center of B. Now slide B the other direction.
  {
    const Vector3<S> offset = S(1.1) * (q0 - expected_n0);
    const Vector3<S> p1_shift = p1 + offset;
    const Vector3<S> q1_shift = q1 + offset;
    Vector3<S> expected_N1;
    S t_expected;
    project_onto_segment(p1_shift, q1_shift, q0, &expected_N1, &t_expected);
    const S expected_squared_dist = (q0 - expected_N1).squaredNorm();

    // Test q0 against interior of B.
    S squared_dist =
        this->ComputeNearestPoint(p0, q0, p1_shift, q1_shift);

    EXPECT_NEAR(squared_dist, expected_squared_dist, eps);
    EXPECT_NEAR(this->s_, S(1), eps);
    EXPECT_NEAR(this->t_, t_expected, eps);
    EXPECT_TRUE(CompareMatrices(this->n0_, q0, eps));
    EXPECT_TRUE(CompareMatrices(this->n1_, expected_N1, eps));

    // Test q1 against interior of A (by swapping parameter arguments). This
    // requires testing s against expected t, n0 against expected n1, and n1
    // against q0.
    squared_dist = this->ComputeNearestPoint(p1_shift, q1_shift, p0, q0);

    EXPECT_NEAR(squared_dist, expected_squared_dist, eps);
    EXPECT_NEAR(this->s_, t_expected, eps);
    EXPECT_NEAR(this->t_, S(1), eps);
    EXPECT_TRUE(CompareMatrices(this->n0_, expected_N1, eps));
    EXPECT_TRUE(CompareMatrices(this->n1_, q0, eps));
  }

  // Case: p0 nearest end point of segment B (first p1, then q1). However, the
  // point on B cannot be co-linear with A. We won't reverse these roles
  // because the problem is already perfectly symmetric.
  {
    // Offset is guaranteed to have a positive dot product with dir the
    // direction along segment A towards p0, but _not_ be parallel (which
    // prevents co-linearity).
    const Vector3<S> p0_dir = p0 - expected_n0;
    const Vector3<S> offset =
        p0_dir.cwiseProduct(Vector3<S>{S(1.75), S(1.5), S(1.25)});
    const Vector3<S> p1_shift = p0 + offset;
    // q1 needs to be positioned such that it's always farther from segment A
    // than p1. We know the direction from segment A to p1 (offset). If
    // offset⋅dir_B is positive, then we extend in the dir_B direction,
    // otherwise the -dir_B direction.
    const Vector3<S> q1_shift =
        p1_shift + (offset.dot(dir_B) > S(0) ? dir_B : Vector3<S>{-dir_B});

    // p0 is nearest p1.
    S squared_dist = this->ComputeNearestPoint(p0, q0, p1_shift, q1_shift);

    EXPECT_NEAR(squared_dist, offset.squaredNorm(), eps);
    EXPECT_NEAR(this->s_, S(0), eps);
    EXPECT_NEAR(this->t_, S(0), eps);
    EXPECT_TRUE(CompareMatrices(this->n0_, p0, eps));
    EXPECT_TRUE(CompareMatrices(this->n1_, p1_shift, eps));

    // p0 is nearest q1 -- simply reverse p1_shift and q1_shift.
    squared_dist = this->ComputeNearestPoint(p0, q0, q1_shift, p1_shift);

    EXPECT_NEAR(squared_dist, offset.squaredNorm(), eps);
    EXPECT_NEAR(this->s_, S(0), eps);
    EXPECT_NEAR(this->t_, S(1), eps);
    EXPECT_TRUE(CompareMatrices(this->n0_, p0, eps));
    EXPECT_TRUE(CompareMatrices(this->n1_, p1_shift, eps));

    // q0 is nearest p1 -- reverse p0 and q0.
    squared_dist = this->ComputeNearestPoint(q0, p0, p1_shift, q1_shift);

    EXPECT_NEAR(squared_dist, offset.squaredNorm(), eps);
    EXPECT_NEAR(this->s_, S(1), eps);
    EXPECT_NEAR(this->t_, S(0), eps);
    EXPECT_TRUE(CompareMatrices(this->n0_, p0, eps));
    EXPECT_TRUE(CompareMatrices(this->n1_, p1_shift, eps));

    // q0 is nearest q1 -- reverse all points.
    squared_dist = this->ComputeNearestPoint(q0, p0, q1_shift, p1_shift);

    EXPECT_NEAR(squared_dist, offset.squaredNorm(), eps);
    EXPECT_NEAR(this->s_, S(1), eps);
    EXPECT_NEAR(this->t_, S(1), eps);
    EXPECT_TRUE(CompareMatrices(this->n0_, p0, eps));
    EXPECT_TRUE(CompareMatrices(this->n1_, p1_shift, eps));
  }
}

// Test harness for exercising the capsuleCapsuleDistance() function. Most of
// the functionality of this function relies on closestPtSegmentSegment() for
// correctness. That has been rigorously tested above. capsuleCapsuleDistance()
// simply has the responsibility of creating the correct segments out of
// arbitrarily posed capsules and then reporting the distance and nearest
// points correctly (to account for capsule radii).
template <typename S>
class CapsuleCapsuleSegmentTest : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CapsuleCapsuleSegmentTest()
      : ::testing::Test(), c1_(S(1.5), S(2.5)), c2_(S(2), S(3)) {}

 protected:
  // See the note below for the explanation of these four cases.
  enum Configuration {
    kSeparated,
    kIntersecting,
    kSingleOverlap,
    kMultipleOverlap
  };
  /* Configure the scene such that neither transform is ever near an identity
   matrix. But the scenario will essentially be one of:

         ●●●●●●●●●
       ●  _______  ●                                           ●●●
       ●           ●                                         ●     ●
         ●●●●●●●●●         ●●●●●●●●●            ○○○          ● ○○○ ●
                         ●  _______  ●       ●●●●●●●●●       ●○   ○●
           ○○○           ●   ○○○     ●     ●  _○ | ○_  ●     ●○ | ○●
          ○   ○            ●●●●●●●●●       ●   ○ | ○   ●     ●○ | ○●
          ○ | ○             ○ | ○            ●●●●●●●●●        ○●●●○
          ○ | ○             ○ | ○              ○ | ○          ○ | ○
          ○ | ○             ○ | ○              ○   ○          ○   ○
          ○ | ○             ○ | ○               ○○○            ○○○
          ○   ○             ○   ○
           ○○○               ○○○

        Separated        Intersecting       Single overlap    Multiple overlap

  Separated:        A single, well-defined pair of witness points to the
                    positive signed distance.
  Intersecting:     A single, well-defined pair of witness points to the
                    negative signed distance.
  Single overlap:   The capsule center lines intersect so there is no unique
                    witness pair to the negative signed distance.
  Multiple overlap: The capsule center lines overlap over an interval, so there
                    is no unique witness pair to the negative signed distance.

  We prevent the transforms from being "boring" by expressing the configuration
  above in a test frame T, but then evaluate the test in frame F, such that X_FT
  is arbitrarily ugly.
  */
  void Configure(Configuration config) {
    switch (config) {
      case kSeparated:
        ConfigureSeparated();
        break;
      case kIntersecting:
        ConfigureIntersecting();
        break;
      case kSingleOverlap:
        ConfigureSingleOverlap();
        break;
      case kMultipleOverlap:
        ConfigureMultipleOverlap();
        break;
    }
  }

  ::testing::AssertionResult RunTestConfiguration(Configuration config) {
    S distance;
    Vector3<S> p_FW1, p_FW2;

    this->Configure(config);

    detail::capsuleCapsuleDistance(this->c1_, this->X_FC1_, this->c2_,
                                   this->X_FC2_, &distance, &p_FW1, &p_FW2);
    const auto eps = constants<S>::eps_78();
    using std::abs;
    S error = abs(distance - this->expected_distance_);
    if (error > eps) {
      return ::testing::AssertionFailure()
             << "Error in reported distances"
             << "\n Actual: " << distance
             << "\n Expected: " << this->expected_distance_
             << "\n Error: " << error
             << "\n Tolerance: " << eps;
    }
    auto result1 = CompareMatrices(p_FW1, this->expected_p_FW1_, eps);
    if (!result1) return result1;
    return CompareMatrices(p_FW2, this->expected_p_FW2_, eps);
  }

  void ConfigureSeparated() {
    this->ConfigureViableT(0.25);
  }

  void ConfigureIntersecting() {
    this->ConfigureViableT(-0.25);
  }

  // Configures separated, instance, and single overlap. What they have in
  // common is that the the geometries form a "T" (which means that between
  // the *center lines* there will always be a unique witness point pair.
  void ConfigureViableT(S distance) {
    const auto pi = constants<S>::pi();

    // The reported distance cannot be less than -(r1 + r2); that is the deepest
    // penetration possible.
    using std::max;
    expected_distance_ = max(distance, -(c1_.radius + c2_.radius));

    Transform3<S> X_TC1 = Transform3<S>::Identity();
    Transform3<S> X_TC2(AngleAxis<S>(pi / 2, Vector3<S>::UnitY()));
    X_TC2.translation() << S(0), S(0),
        c1_.lz / 2 + c1_.radius + c2_.radius + distance;

    Vector3<S> p_TW1{S(0), S(0), c1_.lz / S(2) + c1_.radius};
    expected_p_FW1_ = X_FT() * p_TW1;
    Vector3<S> p_TW2{p_TW1(0), p_TW1(1), p_TW1(2) + distance};
    expected_p_FW2_ = X_FT() * p_TW2;

    X_FC1_ = X_FT() * X_TC1;
    X_FC2_ = X_FT() * X_TC2;
  }

  void ConfigureSingleOverlap() {
    expected_distance_ = -(c1_.radius + c2_.radius);

    const auto pi = constants<S>::pi();
    Transform3<S> X_TC1 = Transform3<S>::Identity();
    Transform3<S> X_TC2(AngleAxis<S>(pi / S(2), Vector3<S>::UnitY()));
    // Position C2 halfway up the upper length of C1.
    X_TC2.translation() << S(0), S(0), c1_.lz / S(4);

    // The witness points lie on a line parallel with the Ty direction that
    // passes through the point p_TC2. For each capsule there are *two* points
    // that lie on that line. This test has been specifically formulated with
    // knowledge of how the capsuleCapsuleDistance() function is formulated
    // to select *one* of those two. This allows the final test to simply
    // compare witness points directly. A change in this special case in the
    // function under test will cause this configuration to fail.
    Vector3<S> p_TW1{S(0), c1_.radius, c1_.lz / S(4)};
    expected_p_FW1_ = X_FT() * p_TW1;
    Vector3<S> p_TW2{S(0), -c2_.radius, c1_.lz / S(4)};
    expected_p_FW2_ = X_FT() * p_TW2;

    X_FC1_ = X_FT() * X_TC1;
    X_FC2_ = X_FT() * X_TC2;
  }

  void ConfigureMultipleOverlap() {
    expected_distance_ = -(c1_.radius + c2_.radius);

    Transform3<S> X_TC1 = Transform3<S>::Identity();
    Transform3<S> X_TC2 = Transform3<S>::Identity();
    // Position C2 so that the lower end of its center lines is at the origin
    // and overlaps with the upper end of C1's center line.
    X_TC2.translation() << S(0), S(0), c2_.lz / S(2);

    // When the two center lines are aligned and overlap, there is an infinite
    // set of witness points. We exploit the knowledge of how the function under
    // test resolves this to create *specific* witness points. We *know* it will
    // pick points that line up with the Tx axis. The biggest unknown is where
    // in the range [0, L1/2] along Tz the witness point is placed. Empirically,
    // it is shown to be at p0 (the top of C1's center line).
    Vector3<S> p_TW1{c1_.radius, S(0), c1_.lz / S(2)};
    expected_p_FW1_ = X_FT() * p_TW1;
    Vector3<S> p_TW2{-c2_.radius, S(0), c1_.lz / S(2)};
    expected_p_FW2_ = X_FT() * p_TW2;

    X_FC1_ = X_FT() * X_TC1;
    X_FC2_ = X_FT() * X_TC2;
  }

  Transform3<S> X_FT() const {
    // Create some arbitrarily ugly transform; the important bit that there
    // be no identities (zeros and ones).
    Transform3<S> X_FT_ = Transform3<S>::Identity();
    X_FT_.translation() << S(10.5), S(12.75), S(-2.5);
    X_FT_.linear() = AngleAxis<S>(constants<S>::pi() / S(7),
                                  Vector3<S>{S(1), S(2), S(3)}.normalized())
                         .matrix();
    return X_FT_;
  }

  Capsule<S> c1_;
  Capsule<S> c2_;
  Transform3<S> X_FC1_;
  Transform3<S> X_FC2_;

  // The expected values: signed distance and witness points on capsules 1 and
  // 2, respectively.
  S expected_distance_;
  Vector3<S> expected_p_FW1_;
  Vector3<S> expected_p_FW2_;
};

TYPED_TEST_CASE(CapsuleCapsuleSegmentTest, ScalarTypes);

// The nominal case will create two unique capsules with a straight forward
// relationship. Making sure the transforms are *not* identity will sufficiently
// exercise the functions logic. We assume that if we can correctly construct
// one pair of line segments, that we'll construct all line segments properly.
TYPED_TEST(CapsuleCapsuleSegmentTest, NominalSeparatedCase) {
  EXPECT_TRUE(this->RunTestConfiguration(
      CapsuleCapsuleSegmentTest<TypeParam>::kSeparated));
}

// Similar to the nominal separated case, but the capsules will be intersecting.
TYPED_TEST(CapsuleCapsuleSegmentTest, NominalIntersectingCase) {
  EXPECT_TRUE(this->RunTestConfiguration(
      CapsuleCapsuleSegmentTest<TypeParam>::kIntersecting));
}

// Tests the case where the capsules are positioned such their center lines
// intersect at a single point.
TYPED_TEST(CapsuleCapsuleSegmentTest, SingleIntersectionCenterLines) {
  EXPECT_TRUE(this->RunTestConfiguration(
      CapsuleCapsuleSegmentTest<TypeParam>::kSingleOverlap));
}

// Tests the case where the capsules are positioned such that their center lines
// overlap.
TYPED_TEST(CapsuleCapsuleSegmentTest, OverlappingCenterLines) {
  EXPECT_TRUE(this->RunTestConfiguration(
      CapsuleCapsuleSegmentTest<TypeParam>::kMultipleOverlap));
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

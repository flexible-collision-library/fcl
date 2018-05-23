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

/** @author Hongkai Dai*/
#include "fcl/narrowphase/detail/convexity_based_algorithm/gjk_libccd-inl.h"

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "fcl/narrowphase/detail/gjk_solver_libccd.h"

namespace fcl {
namespace detail {

template <typename S>
void TestSphereToSphereGJKSignedDistance(S radius1, S radius2,
                                         const Vector3<S>& center1,
                                         const Vector3<S>& center2, S tol) {
  // Test if GJKSignedDistance computes the right distance. Here we used sphere
  // to sphere as the geometries. The distance between sphere and sphere should
  // be computed using distance between primitives, instead of the GJK
  // algorithm. But here we choose spheres for simplicity.
  //
  // Two non-colliding spheres, both with radius 0.5. One sphere is centered
  // at (0, 0, 0), and the other is centered at (1.25, 0, 0).

  auto CheckSignedDistance = [](S radius1, S radius2, const Vector3<S>& center1,
                                const Vector3<S>& center2, S tol) {
    fcl::Sphere<S> s1(radius1);
    fcl::Sphere<S> s2(radius2);
    fcl::Transform3<S> tf1, tf2;
    tf1.setIdentity();
    tf2.setIdentity();
    tf1.translation() = center1;
    tf2.translation() = center2;
    void* o1 = GJKInitializer<S, fcl::Sphere<S>>::createGJKObject(s1, tf1);
    void* o2 = GJKInitializer<S, fcl::Sphere<S>>::createGJKObject(s2, tf2);

    S dist;
    Vector3<S> p1, p2;
    GJKSolver_libccd<S> gjkSolver;
    bool res = GJKSignedDistance(
        o1, detail::GJKInitializer<S, Sphere<S>>::getSupportFunction(), o2,
        detail::GJKInitializer<S, Sphere<S>>::getSupportFunction(),
        gjkSolver.max_distance_iterations, gjkSolver.distance_tolerance, &dist,
        &p1, &p2);

    const S dist_expected = (center1 - center2).norm() - radius1 - radius2;
    EXPECT_EQ(res, dist_expected >= 0);

    // Suppose the sphere 1 is centered at point A, sphere 2 is centered at
    // point B. The position of A in the frame F is p_FA = center1, the position
    // of B in the frame F is p_FB = center2.
    // Denote the vector AB as v_F
    const Vector3<S> v_F = center2 - center1;
    if (v_F.norm() < 1E-3) {
      throw std::logic_error(
          "The two sphere centers almost coincides. This testdoes not cover "
          "such case.");
    }
    const S v_F_norm = v_F.norm();
    const Vector3<S> p1_expected = center1 + radius1 * v_F / v_F_norm;
    const Vector3<S> p2_expected = center2 - radius2 * v_F / v_F_norm;
    EXPECT_NEAR(dist, dist_expected, tol);
    EXPECT_TRUE(p1.isApprox(p1_expected, tol));
    EXPECT_TRUE(p2.isApprox(p2_expected, tol));
  };

  CheckSignedDistance(radius1, radius2, center1, center2, tol);
  // Now switch the position of sphere 1 and sphere 2 and test again.
  CheckSignedDistance(radius2, radius1, center2, center1, tol);
}

template <typename S>
void TestNonCollidingSphereGJKSignedDistance(S tol) {
  TestSphereToSphereGJKSignedDistance<S>(0.5, 0.5, Vector3<S>(0, 0, 0),
                                         Vector3<S>(1.25, 0, 0), tol);
  TestSphereToSphereGJKSignedDistance<S>(0.5, 0.6, Vector3<S>(0, 0, 0),
                                         Vector3<S>(1.25, 0, 0), tol);
}

GTEST_TEST(FCL_GJKSignedDistance, sphere_sphere) {
  TestNonCollidingSphereGJKSignedDistance<double>(1E-14);
  TestNonCollidingSphereGJKSignedDistance<float>(1E-7);
}
}  // namespace detail
}  // namespace fcl

//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

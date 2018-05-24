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

// Given two spheres, sphere 1 has radius1, and centered at point A, whose
// position is p_FA measured and expressed in frame A; sphere 2 has radius2,
// and centered at point B, whose position is p_FB measured and expressed in
// frame B. Computes the signed distance between the two spheres, together with
// the two closest points Na on sphere 1 and Nb on sphere 2, returns the
// position of Na and Nb expressed in frame F.
// We use the monogram notation on spatial vectors. The monogram notation is
// explained in
// http://drake.mit.edu/doxygen_cxx/group__multibody__spatial__pose.html
template <typename S>
S ComputeSphereSphereDistance(S radius1, S radius2, const Vector3<S>& p_FA,
                              const Vector3<S>& p_FB, Vector3<S>* p_FNa,
                              Vector3<S>* p_FNb) {
  S min_distance = (p_FA - p_FB).norm() - radius1 - radius2;
  const Vector3<S> p_AB_F =
      p_FB - p_FA;  // The vector AB measured and expressed
                    // in frame F.
  *p_FNa = p_FA + p_AB_F.normalized() * radius1;
  *p_FNb = p_FB - p_AB_F.normalized() * radius2;
  return min_distance;
}

template <typename S>
void TestSphereToSphereGJKSignedDistance(S radius1, S radius2,
                                         const Vector3<S>& p_FA,
                                         const Vector3<S>& p_FB, S tol,
                                         S min_distance_expected,
                                         const Vector3<S>& p_FNa_expected,
                                         const Vector3<S>& p_FNb_expected) {
  // Test if GJKSignedDistance computes the right distance. Here we used sphere
  // to sphere as the geometries. The distance between sphere and sphere should
  // be computed using distance between primitives, instead of the GJK
  // algorithm. But here we choose spheres for simplicity.

  fcl::Sphere<S> s1(radius1);
  fcl::Sphere<S> s2(radius2);
  fcl::Transform3<S> tf1, tf2;
  tf1.setIdentity();
  tf2.setIdentity();
  tf1.translation() = p_FA;
  tf2.translation() = p_FB;
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

  EXPECT_EQ(res, min_distance_expected >= 0);

  EXPECT_NEAR(dist, min_distance_expected, tol);
  EXPECT_TRUE(p1.isApprox(p_FNa_expected, tol));
  EXPECT_TRUE(p2.isApprox(p_FNb_expected, tol));

  GJKInitializer<S, fcl::Sphere<S>>::deleteGJKObject(o1);
  GJKInitializer<S, fcl::Sphere<S>>::deleteGJKObject(o2);
}

template <typename S>
struct SphereSpecification {
  SphereSpecification<S>(S radius_, const Vector3<S>& center_)
      : radius{radius_}, center{center_} {}
  S radius;
  Vector3<S> center;
};

template <typename S>
void TestNonCollidingSphereGJKSignedDistance(S tol) {
  std::vector<SphereSpecification<S>> spheres;
  spheres.emplace_back(0.5, Vector3<S>(0, 0, 0));
  spheres.emplace_back(0.5, Vector3<S>(1.25, 0, 0));
  spheres.emplace_back(0.3, Vector3<S>(-0.2, 0, 0));
  spheres.emplace_back(0.4, Vector3<S>(-0.2, 0, 1.1));
  for (int i = 0; i < static_cast<int>(spheres.size()); ++i) {
    for (int j = i + 1; j < static_cast<int>(spheres.size()); ++j) {
      if ((spheres[i].center - spheres[j].center).norm() >
          spheres[i].radius + spheres[j].radius) {
        // Not in collision.
        Vector3<S> p_FNa, p_FNb;
        const S min_distance_expected = ComputeSphereSphereDistance(
            spheres[i].radius, spheres[j].radius, spheres[i].center,
            spheres[j].center, &p_FNa, &p_FNb);
        TestSphereToSphereGJKSignedDistance<S>(
            spheres[i].radius, spheres[j].radius, spheres[i].center,
            spheres[j].center, tol, min_distance_expected, p_FNa, p_FNb);
      }
    }
  }
}

GTEST_TEST(FCL_GJKSignedDistance, sphere_sphere) {
  // TODO(hongkai.dai@tri.global): By setting gjkSolver.distance_tolerance to
  // the default value (1E-6), the tolerance we get on the closest points are
  // only up to 1E-3. Should investigate why there is such a big difference.
  TestNonCollidingSphereGJKSignedDistance<double>(1E-3);
  TestNonCollidingSphereGJKSignedDistance<float>(1E-3);
}
}  // namespace detail
}  // namespace fcl

//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

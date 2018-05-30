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
/*
// Given two spheres, sphere 1 has radius1, and centered at point A, whose
// position is p_FA measured and expressed in frame F; sphere 2 has radius2,
// and centered at point B, whose position is p_FB measured and expressed in
// frame F. Computes the signed distance between the two spheres, together with
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
  spheres.emplace_back(0.5, Vector3<S>(0, 0, -1.2));
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
      } else {
        GTEST_FAIL() << "The two spheres collide."
                     << "\nSpheres[" << i << "] with radius "
                     << spheres[i].radius << ", centered at "
                     << spheres[i].center.transpose() << "\nSpheres[" << j
                     << "] with radius " << spheres[j].radius
                     << ", centered at " << spheres[j].center.transpose()
                     << "\n";
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
}*/

//----------------------------------------------------------------------------
//                 Box test
// Given two boxes, by changing the pose of one orientation, the boxes can
// either penetrate, touching or separating.
template <typename S>
struct BoxSpecification {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BoxSpecification(const fcl::Vector3<S>& m_size) : size(m_size) {
    X_FB.setIdentity();
  }
  fcl::Vector3<S> size;
  fcl::Transform3<S> X_FB;
};

template <typename S>
void TestBoxes(S tol) {
  const fcl::Vector3<S> box1_size(1, 1, 1);
  const fcl::Vector3<S> box2_size(0.6, 0.8, 1);
  // Put the two boxes on the xy plane.
  // B1 is the frame rigidly attached to box 1, B2 is the frame rigidly attached
  // to box 2. W is the world frame. X_WB1 is the pose of box 1 expressed and
  // measured in the world frame, X_WB2 is the pose of box 2 expressed and
  // measured in the world frame.
  fcl::Transform3<S> X_WB1, X_WB2;
  // Box 1 is fixed.
  X_WB1.setIdentity();
  X_WB1.translation() << 0, 0, 0.5;

  // First fix the orientation of box 2, such that one of its diagonal (the one
  // connecting the vertex (0.3, -0.4, 1) and (-0.3, 0.4, 1) is horizontal. If
  // we move the position of box 2, we get different signed distance.
  X_WB2.setIdentity();
  X_WB2.linear() << 0.6, -0.8, 0, 0.8, 0.6, 0, 0, 0, 1;

  auto CheckDistance = [&box1_size, &box2_size, &X_WB1](
      const Transform3<S>& X_WB2, S distance_expected,
      const Vector2<S>& p_xy_WNa_expected, const Vector2<S>& p_xy_WNb_expected,
      S tol) {
    fcl::Box<S> box1(box1_size);
    fcl::Box<S> box2(box2_size);
    void* o1 = GJKInitializer<S, fcl::Box<S>>::createGJKObject(box1, X_WB1);
    void* o2 = GJKInitializer<S, fcl::Box<S>>::createGJKObject(box2, X_WB2);
    S dist;
    Vector3<S> p1, p2;
    GJKSolver_libccd<S> gjkSolver;
    bool res = GJKSignedDistance(
        o1, detail::GJKInitializer<S, Box<S>>::getSupportFunction(), o2,
        detail::GJKInitializer<S, Box<S>>::getSupportFunction(),
        gjkSolver.max_distance_iterations, gjkSolver.distance_tolerance, &dist,
        &p1, &p2);

    if (distance_expected < 0) {
      EXPECT_FALSE(res);
    } else if (distance_expected > 0) {
      EXPECT_TRUE(res);
    }

    EXPECT_NEAR(dist, distance_expected, tol);
    std::cout << "p1: " << p1.transpose() << "\n";
    std::cout << "p2: " << p2.transpose() << "\n";
    EXPECT_TRUE(p1.template head<2>().isApprox(p_xy_WNa_expected, tol));
    EXPECT_TRUE(p2.template head<2>().isApprox(p_xy_WNb_expected, tol));
    // The z height of the closest points should be the same.
    EXPECT_NEAR(p1(2), p2(2), tol);
    EXPECT_GE(p1(2), 0);
    EXPECT_GE(p2(2), 0);
    EXPECT_LE(p1(2), 1);
    EXPECT_LE(p2(2), 1);

    GJKInitializer<S, fcl::Sphere<S>>::deleteGJKObject(o1);
    GJKInitializer<S, fcl::Sphere<S>>::deleteGJKObject(o2);
  };

  // An edge of box 2 is touching a face of box 1
  X_WB2.translation() << -1, 0, 0.5;
  CheckDistance(X_WB2, 0, Vector2<S>(-0.5, 0), Vector2<S>(-0.5, 0), tol);
}

GTEST_TEST(FCL_GJKSignedDistance, box_box) {
  // TODO(hongkai.dai@tri.global): By setting gjkSolver.distance_tolerance to
  // the default value (1E-6), the tolerance we get on the closest points are
  // only up to 1E-3. Should investigate why there is such a big difference.
  TestBoxes<double>(1E-3);
  TestBoxes<float>(1E-3);
}
}  // namespace detail
}  // namespace fcl

//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

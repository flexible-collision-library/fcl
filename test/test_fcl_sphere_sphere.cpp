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

/** @author Hongkai Dai <hongkai.dai@tri.global>*/

#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/distance.h"
#include "fcl/narrowphase/distance_request.h"
#include "fcl/narrowphase/distance_result.h"

// For two spheres 1, 2, sphere 1 has radius1, and is centered at point A, with
// coordinate p_FA in some frame F; sphere 2 has radius2, and is centered at
// point B, with coordinate p_FB in the same frame F. Compute the (optionally
// signed) distance between the two spheres.
template <typename S>
S ComputeSphereSphereDistance(S radius1, S radius2, const fcl::Vector3<S>& p_FA,
                              const fcl::Vector3<S>& p_FB,
                              fcl::GJKSolverType solver_type,
                              bool enable_nearest_points,
                              bool enable_signed_distance,
                              fcl::DistanceResult<S>* result) {
  fcl::Transform3<S> X_FA, X_FB;
  X_FA.setIdentity();
  X_FB.setIdentity();
  X_FA.translation() = p_FA;
  X_FB.translation() = p_FB;

  fcl::DistanceRequest<S> request;
  request.enable_nearest_points = enable_nearest_points;
  request.enable_signed_distance = enable_signed_distance;
  request.gjk_solver_type = solver_type;

  using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<S>>;
  CollisionGeometryPtr_t sphere_geometry_1(new fcl::Sphere<S>(radius1));
  CollisionGeometryPtr_t sphere_geometry_2(new fcl::Sphere<S>(radius2));

  fcl::CollisionObject<S> sphere_1(sphere_geometry_1, X_FA);
  fcl::CollisionObject<S> sphere_2(sphere_geometry_2, X_FB);
  const S min_distance = fcl::distance(&sphere_1, &sphere_2, request, *result);
  return min_distance;
}

template <typename S>
struct SphereSpecification {
  // @param p_FC_ the center of the sphere C measured and expressed in a frame F
  SphereSpecification(S m_radius, const fcl::Vector3<S>& p_FC_)
      : radius(m_radius), p_FC(p_FC_) {}
  S radius;
  fcl::Vector3<S> p_FC;
};

template <typename S>
void CheckSphereToSphereDistance(const SphereSpecification<S>& sphere1,
                                 const SphereSpecification<S>& sphere2,
                                 fcl::GJKSolverType solver_type,
                                 bool enable_nearest_points,
                                 bool enable_signed_distance,
                                 S min_distance_expected,
                                 const fcl::Vector3<S>& p1_expected,
                                 const fcl::Vector3<S>& p2_expected, S tol) {
  fcl::DistanceResult<S> result;
  const S min_distance = ComputeSphereSphereDistance<S>(
      sphere1.radius, sphere2.radius, sphere1.p_FC, sphere2.p_FC, solver_type,
      enable_nearest_points, enable_signed_distance, &result);
  EXPECT_NEAR(min_distance, min_distance_expected, tol);
  EXPECT_NEAR(result.min_distance, min_distance_expected, tol);
  EXPECT_TRUE(result.nearest_points[0].isApprox(p1_expected, tol));
  EXPECT_TRUE(result.nearest_points[1].isApprox(p2_expected, tol));
}

template <typename S>
struct SphereSphereDistance {
  SphereSphereDistance(const SphereSpecification<S>& m_sphere1,
                       const SphereSpecification<S>& m_sphere2)
      : sphere1(m_sphere1), sphere2(m_sphere2) {
    min_distance = (sphere1.p_FC - sphere2.p_FC).norm() - sphere1.radius -
                   sphere2.radius;
    const fcl::Vector3<S> AB = (sphere1.p_FC- sphere2.p_FC).normalized();
    p_S1P1 = -AB * sphere1.radius;
    p_S2P2 = AB * sphere2.radius;
  }
  SphereSpecification<S> sphere1;
  SphereSpecification<S> sphere2;
  S min_distance;
  // The closest points are expressed in the sphere body frame.
  // The closest point P1 on sphere 1 is expressed in the sphere 1 body frame
  // S1.
  // The closest point P2 on sphere 2 is expressed in the sphere 2 body frame
  // S2.
  fcl::Vector3<S> p_S1P1;
  fcl::Vector3<S> p_S2P2;
};

template <typename S>
void TestSeparatingSpheres(S tol, fcl::GJKSolverType solver_type) {
  std::vector<SphereSpecification<S>> spheres;
  spheres.emplace_back(0.5, fcl::Vector3<S>(0, 0, 0));
  spheres.emplace_back(0.6, fcl::Vector3<S>(1.2, 0, 0));
  spheres.emplace_back(0.4, fcl::Vector3<S>(-0.3, 0, 0));
  spheres.emplace_back(0.3, fcl::Vector3<S>(0, 0, 1));

  for (int i = 0; i < static_cast<int>(spheres.size()); ++i) {
    for (int j = i + 1; j < static_cast<int>(spheres.size()); ++j) {
      SphereSphereDistance<S> sphere_sphere_distance(spheres[i], spheres[j]);
      if (sphere_sphere_distance.min_distance > 0) {
        bool enable_signed_distance = true;
        CheckSphereToSphereDistance<S>(
            spheres[i], spheres[j], solver_type, true, enable_signed_distance,
            sphere_sphere_distance.min_distance, sphere_sphere_distance.p_S1P1,
            sphere_sphere_distance.p_S2P2, tol);

        // Now switch the order of sphere 1 with sphere 2 in calling
        // fcl::distance function, and test again.
        CheckSphereToSphereDistance<S>(
            spheres[j], spheres[i], solver_type, true, enable_signed_distance,
            sphere_sphere_distance.min_distance, sphere_sphere_distance.p_S2P2,
            sphere_sphere_distance.p_S1P1, tol);
        
        enable_signed_distance = false;
        CheckSphereToSphereDistance<S>(
            spheres[i], spheres[j], solver_type, true, enable_signed_distance,
            sphere_sphere_distance.min_distance, sphere_sphere_distance.p_S1P1,
            sphere_sphere_distance.p_S2P2, tol);
        // Now switch the order of sphere 1 with sphere 2 in calling
        // fcl::distance function, and test again.
        CheckSphereToSphereDistance<S>(
            spheres[j], spheres[i], solver_type, true, enable_signed_distance,
            sphere_sphere_distance.min_distance, sphere_sphere_distance.p_S2P2,
            sphere_sphere_distance.p_S1P1, tol);

        // TODO (hongkai.dai@tri.global): Test enable_nearest_points=false.
      }
    }
  }
}

GTEST_TEST(FCL_SPHERE_SPHERE, Separating_Spheres_INDEP) {
  TestSeparatingSpheres<double>(1E-14, fcl::GJKSolverType::GST_INDEP);
}

GTEST_TEST(FCL_SPHERE_SPHERE, Separating_Spheres_LIBCCD) {
  TestSeparatingSpheres<double>(1E-3, fcl::GJKSolverType::GST_LIBCCD);
}
//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

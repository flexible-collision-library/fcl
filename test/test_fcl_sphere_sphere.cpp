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
// point B, with coordinate p_FB in the same frame F. Compute the (signed)
// distance between the two spheres.
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
void CheckSphereToSphereDistance(
    S radius1, S radius2, const fcl::Vector3<S>& p_FA,
    const fcl::Vector3<S>& p_FB, fcl::GJKSolverType solver_type,
    bool enable_nearest_points, bool enable_signed_distance,
    S min_distance_expected, const fcl::Vector3<S>& p1_expected,
    const fcl::Vector3<S>& p2_expected, S tol) {
  fcl::DistanceResult<S> result;
  const S min_distance = ComputeSphereSphereDistance<S>(
      radius1, radius2, p_FA, p_FB, solver_type, enable_nearest_points,
      enable_signed_distance, &result);
  EXPECT_NEAR(min_distance, min_distance_expected, tol);
  EXPECT_NEAR(result.min_distance, min_distance_expected, tol);
  EXPECT_TRUE(result.nearest_points[0].isApprox(p1_expected, tol));
  EXPECT_TRUE(result.nearest_points[1].isApprox(p2_expected, tol));
}

template <typename S>
void TestSeparatingSpheres(S tol) {
  const S radius1 = 0.5;
  const S radius2 = 0.6;
  const fcl::Vector3<S> p_FA(0, 0, 0);
  const fcl::Vector3<S> p_FB(1.2, 0, 0);

  const fcl::Vector3<S> p1_expected(0.5, 0, 0);
  const fcl::Vector3<S> p2_expected(-0.6, 0, 0);
  for (const fcl::GJKSolverType solver_type :
       {fcl::GJKSolverType::GST_LIBCCD, fcl::GJKSolverType::GST_INDEP}) {
    CheckSphereToSphereDistance<S>(radius1, radius2, p_FA, p_FB, solver_type,
                                   true, true, 0.1, p1_expected, p2_expected,
                                   tol);

    // Now switch the order of sphere 1 with sphere 2 in calling fcl::distance
    // function, and test again.
    CheckSphereToSphereDistance<S>(radius2, radius1, p_FB, p_FA, solver_type,
                                   true, true, 0.1, p2_expected, p1_expected,
                                   tol);
  }
}

GTEST_TEST(FCL_SPHERE_SPHERE, Separating_Spheres) {
  TestSeparatingSpheres<double>(1E-14);
}

//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

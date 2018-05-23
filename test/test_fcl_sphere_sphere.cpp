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

#include <functional>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "fcl/math/constants.h"
#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/distance.h"
#include "fcl/narrowphase/distance_request.h"
#include "fcl/narrowphase/distance_result.h"

using std::map;
using std::pair;
using std::string;
using std::vector;

// Simple specification for defining a sphere collision object. Specifies the
// radius and center of the sphere
template <typename S>
struct SphereSpecification {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  S radius;
  fcl::Vector3<S> center;
};

// Class for executing and evaluating various box-box tests.
// The class is initialized with two box specifications (size and pose).
// The test performs a collision test between the two boxes in 12 permutations:
// One axis is the order (box 1 vs box 2 and box 2 vs box 1).
// The other axis is the orientation box 2. Given that box 2 must be a cube, it
// can be oriented in six different configurations and still produced the same
// answer.
// The 12 permutations are the two orderings crossed with the six orientations.
template <typename S>
class SphereSphereTest {
 public:
  // Construct the test scenario with the given sphere specifications.
  SphereSphereTest(const SphereSpecification<S>& sphere_spec_A,
                   const SphereSpecification<S>& sphere_spec_B)
      : sphere_spec_A_(sphere_spec_A), sphere_spec_B_(sphere_spec_B) {}

  void RunTests(fcl::GJKSolverType solver_type) {
    fcl::DistanceRequestd request;
    request.enable_nearest_points = true;
    request.enable_signed_distance = true;
    request.gjk_solver_type = solver_type;
    fcl::DistanceResultd result;

    fcl::Transform3<S> X_FA, X_FB;
    X_FA.setIdentity();
    X_FB.setIdentity();
    X_FA.translation() = sphere_spec_A_.center;
    X_FB.translation() = sphere_spec_B_.center;

    using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<S>>;
    CollisionGeometryPtr_t sphere_geometry_A(
        new fcl::Sphere<S>(sphere_spec_A_.radius));
    CollisionGeometryPtr_t sphere_geometry_B(
        new fcl::Sphere<S>(sphere_spec_B_.radius));

    fcl::CollisionObject<S> sphere_A(sphere_geometry_A, X_FA);
    fcl::CollisionObject<S> sphere_B(sphere_geometry_B, X_FB);
    const S min_distance = fcl::distance(&sphere_A, &sphere_B, request, result);
  }

  SphereSpecification<S> sphere_spec_A_;
  SphereSpecification<S> sphere_spec_B_;
};

GTEST_TEST(FCL_SPHERE_SPHERE, distance_ccd) {
  SphereSpecification<double> A_spec, B_spec;
  A_spec.radius = 0.5;
  A_spec.center = fcl::Vector3<double>(1.25, 0, 0);
  B_spec.radius = 0.5;
  B_spec.center = fcl::Vector3<double>(-0.5, 0, 0);

  SphereSphereTest<double> test(A_spec, B_spec);
  test.RunTests(fcl::GJKSolverType::GST_LIBCCD);
}

//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

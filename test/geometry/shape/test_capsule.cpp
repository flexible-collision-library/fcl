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

/** @author Gabriele Buondonno (gbuondon@laas.fr) (2019) */

// Tests the implementation of a capsule object.

#include "fcl/geometry/shape/capsule.h"

#include <vector>

#include <Eigen/StdVector>
#include <gtest/gtest.h>

#include "eigen_matrix_compare.h"

namespace fcl {
namespace {

typedef std::pair<double, double> paird;

std::vector<paird> get_test_sizes() {
  return std::vector<paird>{
      paird(2.,3.),
      paird(20.,30.),
      paird(3.,2.),
      paird(30.,20.)
  };
}

template <typename S>
void testLocalAABBComputation(Capsule<S>& shape, S tol) {
  shape.computeLocalAABB();

  S r = shape.radius;
  S l = shape.lz;
  EXPECT_NEAR(shape.aabb_radius, Vector3<S>(r, r, 0.5 * l + r).norm(), tol);
  EXPECT_TRUE(CompareMatrices(shape.aabb_center, Vector3<S>(0, 0, 0), tol));
}

template <typename S>
void testVolumeComputation(const Capsule<S>& shape, S tol) {
  S r = shape.radius;
  S l = shape.lz;

  S pi = constants<S>::pi();

  S v_cyl = pi*r*r*l;       // volume of a cylinder
  S v_sph = 4./3.*pi*r*r*r; // volume of a sphere
  S v_cap = v_cyl + v_sph;  // total volume

  EXPECT_TRUE(Eigen::internal::isApprox(shape.computeVolume(), v_cap, tol));
}

template <typename S>
void testMomentOfInertiaComputation(const Capsule<S>& shape, S tol) {
  S r = shape.radius;
  S l = shape.lz;

  S pi = constants<S>::pi();

  S v_cyl = pi*r*r*l;          // volume of a cylinder
  S v_sph = S(4./3.)*pi*r*r*r; // volume of a sphere

  S Iz_cyl = S(0.5) * v_cyl * r * r;   // inertia of a cylinder around z
  S Iz_sph = S(2./5.) * v_sph * r * r; // inertia of a sphere around z
  S Iz = Iz_cyl + Iz_sph;              // total inertia around z

  // inertia of a cylinder around x
  S Ix_cyl = v_cyl * (S(3.) * r * r + l * l) / S(12.);

  S v_hemi = v_sph / S(2.);                  // volume of a hemisphere
  S com_hemi = S(3.) * r / S(8.);            // CoM of a hemisphere from base
  S Ix0_hemi = Iz_sph / S(2.);               // inertia of a hemisphere around x
  S Ixc_hemi = Ix0_hemi - v_hemi * com_hemi * com_hemi; // inertia around CoM
  S dz = l / S(2.) + com_hemi;               // CoM translation along z-axis
  S Ix_hemi = Ixc_hemi + v_hemi * dz * dz;   // translated inertia around x

  S Ix = Ix_cyl + S(2.) * Ix_hemi;  // total inertia around x
  S Iy = Ix;                        // total inertia around y

  Eigen::Matrix<S,3,3> I_cap;
  I_cap <<    Ix, S(0.), S(0.),
           S(0.),    Iy, S(0.),
           S(0.), S(0.),    Iz;

  EXPECT_TRUE(shape.computeMomentofInertia().isApprox(I_cap, tol));
}

GTEST_TEST(Capsule, LocalAABBComputation_Capsule) {
  for (paird pair : get_test_sizes()) {
    double rd = pair.first;
    double ld = pair.second;
    Capsuled capsule_d(rd, ld);
    testLocalAABBComputation(capsule_d, 1e-15);

    float rf = static_cast<float>(pair.first);
    float lf = static_cast<float>(pair.second);
    Capsulef capsule_f(rf, lf);
    testLocalAABBComputation(capsule_f, 1e-8f);
  }
}

GTEST_TEST(Capsule, Volume_Capsule) {
  for (paird pair : get_test_sizes()) {
    double rd = pair.first;
    double ld = pair.second;
    Capsuled capsule_d(rd, ld);
    testVolumeComputation(capsule_d, 1e-15);

    float rf = static_cast<float>(pair.first);
    float lf = static_cast<float>(pair.second);
    Capsulef capsule_f(rf, lf);
    testVolumeComputation(capsule_f, 1e-8f);
  }
}

GTEST_TEST(Capsule, CenterOfMass_Capsule) {
  Eigen::Vector3d comd = Vector3d::Zero();
  Eigen::Vector3f comf = Vector3f::Zero();
  for (paird pair : get_test_sizes()) {
    double rd = pair.first;
    double ld = pair.second;
    Capsuled capsule_d(rd, ld);
    EXPECT_TRUE(CompareMatrices(capsule_d.computeCOM(), comd, 0.));

    float rf = static_cast<float>(pair.first);
    float lf = static_cast<float>(pair.second);
    Capsulef capsule_f(rf, lf);
    EXPECT_TRUE(CompareMatrices(capsule_f.computeCOM(), comf, 0.f));
  }
}

GTEST_TEST(Capsule, MomentOfInertia_Capsule) {
  for (paird pair : get_test_sizes()) {
    double rd = pair.first;
    double ld = pair.second;
    Capsuled capsule_d(rd, ld);
    testMomentOfInertiaComputation(capsule_d, 1e-14);

    float rf = static_cast<float>(pair.first);
    float lf = static_cast<float>(pair.second);
    Capsulef capsule_f(rf, lf);
    testMomentOfInertiaComputation(capsule_f, 1e-6f);
  }
}

}  // namespace
}  // namespace fcl

//==============================================================================
int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

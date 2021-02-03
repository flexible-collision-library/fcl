/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021. Toyota Research Institute
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

/** @author Sean Curtis (sean@tri.global) (2021) */

// Tests the custom sphere-box tests: distance and collision.

#include "fcl/narrowphase/detail/collision_func_matrix.h"

#include <array>

#include <gtest/gtest.h>

#include "fcl/geometry/collision_geometry.h"
#include "fcl/narrowphase/detail/gjk_solver_libccd.h"
#include "fcl/narrowphase/detail/gjk_solver_indep.h"

namespace fcl {

#define NODE_CASE(node) case node: \
  out << #node; \
  break;

std::ostream& operator<<(std::ostream& out, const NODE_TYPE& node) {
  switch (node) {
    NODE_CASE(BV_UNKNOWN)
    NODE_CASE(BV_AABB)
    NODE_CASE(BV_OBB)
    NODE_CASE(BV_RSS)
    NODE_CASE(BV_kIOS)
    NODE_CASE(BV_OBBRSS)
    NODE_CASE(BV_KDOP16)
    NODE_CASE(BV_KDOP18)
    NODE_CASE(BV_KDOP24)
    NODE_CASE(GEOM_BOX)
    NODE_CASE(GEOM_SPHERE)
    NODE_CASE(GEOM_ELLIPSOID)
    NODE_CASE(GEOM_CAPSULE)
    NODE_CASE(GEOM_CONE)
    NODE_CASE(GEOM_CYLINDER)
    NODE_CASE(GEOM_CONVEX)
    NODE_CASE(GEOM_PLANE)
    NODE_CASE(GEOM_HALFSPACE)
    NODE_CASE(GEOM_TRIANGLE)
    NODE_CASE(GEOM_OCTREE)
    NODE_CASE(NODE_COUNT)
  }
  return out;
}

#undef NODE_CASE

namespace detail {
namespace {

using std::array;

/* The collision function matrix defines the function that can be used to
 evaluate collision between two CollisionGeometry instances based on their
 node type. In NODE_TYPE, there are nine primitive types, eight BV types, and
 three special values (see below). We want to confirm the table satisfies the
 following invariants:

  1 There's a function for all primitive pairs, in both orderings.
  2 There's a function for each (BV, geometry) combination (but not
    (geometry, BV)).
  3 There's a function for each BV type with its *own* type.
  4 If octomap is available, there should be functions between GEOM_OCTREE
    and every geometry type (in both orderings), with itself, and with each
    BV type (in both orderings).

 Primitive geometries: GEOM_BOX, GEOM_SPHERE, GEOM_ELLIPSOID, GEOM_CAPSULE,
                       GEOM_CONE, GEOM_CYLINDER, GEOM_CONVEX, GEOM_PLANE,
                       GEOM_HALFSPACE
 BV types: BV_AABB, BV_OBB, BV_RSS, BV_kIOS, BV_OBBRSS, BV_KDOP16, BV_KDOP18,
           BV_KDOP24
 Special: GEOM_TRIANGLE, GEOM_OCTREE, NODE_COUNT, BV_UNKNOWN

 NODE_COUNT is merely a convenient symbol for reporting the number of enumerated
 values. BV_UNKNOWN and GEOM_TRIANGLE are not part of the function matrix. */
template <typename Solver>
void ConfirmSupportedGeometryForSolver() {
  const array<NODE_TYPE, 9> geoms{GEOM_BOX, GEOM_SPHERE, GEOM_ELLIPSOID,
                                  GEOM_CAPSULE, GEOM_CONE, GEOM_CYLINDER,
                                  GEOM_CONVEX, GEOM_PLANE, GEOM_HALFSPACE};
  const array<NODE_TYPE, 8> bvs{BV_AABB, BV_OBB, BV_RSS, BV_kIOS, BV_OBBRSS,
                                BV_KDOP16, BV_KDOP18, BV_KDOP24};

  const CollisionFunctionMatrix<Solver> matrix_struct;
  const auto& matrix = matrix_struct.collision_matrix;

  // 1) Confirm a function between geom_A -> geom_B for all geometries.
  for (const NODE_TYPE& g1 : geoms) {
    for (const NODE_TYPE& g2 : geoms) {
      EXPECT_NE(matrix[g1][g2], nullptr) << "(" << g1 << ", " << g2 << ")";
    }
  }

  // 2) and 3) There's a function for BV -> geometry and BV -> self.
  for (const NODE_TYPE& bv : bvs) {
    EXPECT_NE(matrix[bv][bv], nullptr) << "(" << bv << ", " << bv << ")";
    for (const NODE_TYPE& g : geoms) {
      EXPECT_NE(matrix[bv][g], nullptr) << "(" << bv << ", " << g << ")";
    }
  }

#if FCL_HAVE_OCTOMAP
  // 4) Octomap has functions.
  const NODE_TYPE oct = GEOM_OCTREE;
  EXPECT_NE(matrix[oct][oct], nullptr) << "(" << oct << ", " << oct << ")";
  for (const NODE_TYPE& g: geoms) {
    EXPECT_NE(matrix[oct][g], nullptr) << "(" << oct << ", " << g << ")";
    EXPECT_NE(matrix[g][oct], nullptr) << "(" << g << ", " << oct << ")";
  }
  for (const NODE_TYPE& bv: bvs) {
    EXPECT_NE(matrix[oct][bv], nullptr) << "(" << oct << ", " << bv << ")";
    EXPECT_NE(matrix[bv][oct], nullptr) << "(" << bv << ", " << oct << ")";
  }
#endif
}

GTEST_TEST(CollisionFuncMatrix, LibCccdSolverSupport) {
  ConfirmSupportedGeometryForSolver<GJKSolver_libccd<double>>();
}

GTEST_TEST(CollisionFuncMatrix, IndepSolverSupport) {
  ConfirmSupportedGeometryForSolver<GJKSolver_indep<double>>();
}

} // namespace
} // namespace detail
} // namespace fcl

//==============================================================================
int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

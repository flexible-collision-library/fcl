/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020. Toyota Research Institute
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

/** @author Sean Curtis (sean.curtis@tri.global) */

/** @file Tests the "support function" used for convex geometries. */

#include "fcl/narrowphase/detail/convexity_based_algorithm/gjk_libccd.h"

#include <initializer_list>
#include <memory>
#include <utility>
#include <vector>

#include "fcl/geometry/shape/convex.h"
#include "eigen_matrix_compare.h"

namespace fcl {
namespace detail {
namespace {

using std::make_shared;
using std::move;
using std::pair;
using std::vector;

ccd_vec3_t to_ccd(const Vector3d& v) {
  ccd_vec3_t ccd_v;
  ccdVec3Set(&ccd_v, v[0], v[1], v[2]);
  return ccd_v;
}

Vector3d from_ccd(const ccd_vec3_t& v) {
  return Vector3d{v.v[0], v.v[1], v.v[2]};
}

/* Confirms that the GJKInitializer<S, Convex<S>>::getSupportFunction accounts
 for the pose of the Convex shape in the query space. We're assuming the
 support function uses Convex::findExtremeVertex() so we're doing some spot
 checking on the general correctness and not the numerical nitty gritty
 (assuming that has been taken care of in the Convex unit tests).  */
GTEST_TEST(GjkLibccdSupportFunction, ConvexSupport) {
  /* We'll create a simple cube as a convex shape; easy to reason about.

                               z       y
                               ┆     ╱
                         7 ○━━━┆━━━━━━━━○ 6
                          ╱┃   ┆   ╱   ╱┃
                        ╱  ┃   ┆  ╱  ╱  ┃
                    4 ○━━━━━━━━┿━━━○ 5  ┃
                      ┃    ┃   ┆╱  ┃    ┃
                  ┄┄┄─╂┄┄┄┄┄┄┄┄┼┄┄┄╂┄┄┄┄┄┄┄┄┄┄ x
                      ┃  3 ○━━━┆━━━┃━━━━○ 2
                      ┃   ╱  ╱ ┆   ┃   ╱
                      ┃ ╱   ╱  ┆   ┃ ╱
                      ○━━━━━━━━┿━━━○
                    0     ╱    ┆   1
                         ╱     ┆
                               ┆
                               ┆
   */
  // Note: Although the initializer list is acceptable for initializing a
  // vector, the inference of initializer list gets masked by the call to
  // make_shared. The solution is to explicitly declare it.
  auto vertices = make_shared<vector<Vector3d>>(std::initializer_list<Vector3d>{
      Vector3d{-1, -1, -1}, Vector3d{1, -1, -1}, Vector3d{1, 1, -1},
      Vector3d{-1, 1, -1}, Vector3d{-1, -1, 1}, Vector3d{1, -1, 1},
      Vector3d{1, 1, 1}, Vector3d{-1, 1, 1}});
  // clang-format off
  auto faces = make_shared<vector<int>>(
      std::initializer_list<int>{4, 0, 3, 2, 1,  // -z face
                                 4, 1, 2, 6, 5,  // +x face
                                 4, 5, 6, 7, 4,  // +z face
                                 4, 0, 4, 7, 3,  // -x face
                                 4, 0, 1, 5, 4,  // -y face
                                 4, 3, 7, 6, 2}  // +y face
      );
  // clang-format on
  const int kNumFaces = 6;
  const bool kThrowIfInvalid = true;
  const Convex<double> convex_C(move(vertices), kNumFaces, move(faces),
                                kThrowIfInvalid);

  /* Collection of arbitrary poses of the convex mesh: identity, translation,
   and rotation. */
  aligned_vector<Transform3d> X_WCs;
  X_WCs.push_back(Transform3d::Identity());
  X_WCs.emplace_back(Transform3d(Translation3d{-1, 2, -3}));
  X_WCs.emplace_back(Transform3d(Quaterniond{0.5, -0.5, 0.5, -0.5}));

  /* Collection of arbitrary directions (each expressed in Frame C).  */
  vector<Vector3d> directions_C{Vector3d{-2, -2, -2}, Vector3d{-2, 3, -2},
                                Vector3d{7, 1, -1}};

  for (const auto& dir_C : directions_C) {
    const Vector3d& p_CV_expected = convex_C.findExtremeVertex(dir_C);
    for (const auto& X_WC : X_WCs) {
      void* gjk_convex =
          GJKInitializer<double, Convex<double>>::createGJKObject(convex_C,
                                                                  X_WC);
      ccd_vec3_t dir_W = to_ccd(X_WC * dir_C);
      ccd_vec3_t p_WV;
      supportConvex<double>(gjk_convex, &dir_W, &p_WV);
      EXPECT_TRUE(
          CompareMatrices(from_ccd(p_WV), X_WC * p_CV_expected));
      /* Note: In order for this delete to get called, the previous four lines
       needs to execute without error. A more rigorous scope guard would be
       good. However, because this is in a unit test, the memory leak induced
       by not invoking this delete will be immediately eliminated by the
       testing process exiting.  */
      GJKInitializer<double, Convex<double>>::deleteGJKObject(gjk_convex);
    }
  }
}

// TODO(SeanCurtis-TRI): Repeat the ConvexSupport test for all other shapes.
// TODO(SeanCurtis-TRI): Test other aspects of GJKInitializer for all shapes:
//  - getCenterFunction
//  - createGJKObject
//  - deleteGJKObject

}  // namespace
}  // namespace detail
}  // namespace fcl

//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

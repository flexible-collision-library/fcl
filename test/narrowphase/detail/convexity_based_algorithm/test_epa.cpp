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

/** @author Hongkai Dai (hongkai.dai@tri.global) */

/** Tests the EPA implementation inside FCL. EPA computes the penetration
 * depth and the points with the deepest penetration between two convex objects.
 */

#include "fcl/narrowphase/detail/convexity_based_algorithm/gjk_libccd-inl.h"

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "fcl/narrowphase/detail/convexity_based_algorithm/polytope.h"

namespace fcl {
namespace detail {
class EquilateralTetrahedron {
 public:
  EquilateralTetrahedron() : polytope_(new ccd_pt_t) {
    ccdPtInit(polytope_);
    v_[0] = ccdPtAddVertexCoords(polytope_, 0.5, -0.5 / std::sqrt(3), 0);
    v_[1] = ccdPtAddVertexCoords(polytope_, -0.5, -0.5 / std::sqrt(3), 0);
    v_[2] = ccdPtAddVertexCoords(polytope_, 0, 1 / std::sqrt(3), 0);
    v_[3] = ccdPtAddVertexCoords(polytope_, 0, 0, std::sqrt(2.0 / 3.0));
    e_[0] = ccdPtAddEdge(polytope_, v_[0], v_[1]);
    e_[1] = ccdPtAddEdge(polytope_, v_[1], v_[2]);
    e_[2] = ccdPtAddEdge(polytope_, v_[2], v_[0]);
    e_[3] = ccdPtAddEdge(polytope_, v_[3], v_[0]);
    e_[4] = ccdPtAddEdge(polytope_, v_[3], v_[1]);
    e_[5] = ccdPtAddEdge(polytope_, v_[3], v_[2]);
    f_[0] = ccdPtAddFace(polytope_, e_[0], e_[1], e_[2]);
    f_[1] = ccdPtAddFace(polytope_, e_[0], e_[3], e_[4]);
    f_[2] = ccdPtAddFace(polytope_, e_[1], e_[4], e_[5]);
    f_[3] = ccdPtAddFace(polytope_, e_[2], e_[3], e_[5]);
  }

  ccd_pt_vertex_t* v(int i) const { return v_[i]; }

  ccd_pt_edge_t* e(int i) const { return e_[i]; }

  ccd_pt_face_t* f(int i) const { return f_[i]; }

  ccd_pt_t* polytope() const { return polytope_; }

  ~EquilateralTetrahedron() {
    ccdPtDestroy(polytope_);
    delete polytope_;
  }

 private:
  ccd_pt_t* polytope_;
  ccd_pt_vertex_t* v_[4];
  ccd_pt_edge_t* e_[6];
  ccd_pt_face_t* f_[4];
};

GTEST_TEST(FCL_GJK_EPA, faceNormalPointingOutward) {
  // Construct a equilateral tetrahedron, compute the normal on each face.
  EquilateralTetrahedron p;

  for (int i = 0; i < 4; ++i) {
    const ccd_vec3_t n =
        libccd_extension::faceNormalPointingOutward(p.polytope(), p.f(i));
    for (int j = 0; j < 4; ++j) {
      EXPECT_LE(ccdVec3Dot(&n, &p.v(j)->v.v),
                ccdVec3Dot(&n, &p.f(i)->edge[0]->vertex[0]->v.v) + 1E-10);
    }
  }
}

GTEST_TEST(FCL_GJK_EPA, outsidePolytopeFace) {
  EquilateralTetrahedron p;

  auto CheckPointOutsidePolytopeFace = [&p](ccd_real_t x, ccd_real_t y,
                                            ccd_real_t z, int face_index,
                                            bool is_outside_expected) {
    ccd_vec3_t pt;
    pt.v[0] = x;
    pt.v[1] = y;
    pt.v[2] = z;
    EXPECT_EQ(libccd_extension::outsidePolytopeFace(p.polytope(),
                                                    p.f(face_index), &pt),
              is_outside_expected);
  };

  // point (0, 0, 0.1) is inside the tetrahedron.
  CheckPointOutsidePolytopeFace(0, 0, 0.1, 0, false);
  CheckPointOutsidePolytopeFace(0, 0, 0.1, 1, false);
  CheckPointOutsidePolytopeFace(0, 0, 0.1, 2, false);
  CheckPointOutsidePolytopeFace(0, 0, 0.1, 3, false);

  // point(0, 0, 2) is outside the tetrahedron. But it is on the "inner" side
  // of the bottom face.
  CheckPointOutsidePolytopeFace(0, 0, 2, 0, false);
  CheckPointOutsidePolytopeFace(0, 0, 2, 1, true);
  CheckPointOutsidePolytopeFace(0, 0, 2, 2, true);
  CheckPointOutsidePolytopeFace(0, 0, 2, 3, true);

  // point (0, 0, 0) is right on the bottom face.
  CheckPointOutsidePolytopeFace(0, 0, 0, 0, false);
  CheckPointOutsidePolytopeFace(0, 0, 0, 1, false);
  CheckPointOutsidePolytopeFace(0, 0, 0, 2, false);
  CheckPointOutsidePolytopeFace(0, 0, 0, 3, false);
}
}  // namespace detail
}  // namespace fcl

//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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

/** Tests the Expanded Polytope Algorithm (EPA) implementation inside FCL. EPA
 * computes the penetration depth and the points with the deepest penetration
 * between two convex objects.
 */

#include "fcl/narrowphase/detail/convexity_based_algorithm/gjk_libccd-inl.h"

#include <memory>

#include <gtest/gtest.h>

#include "fcl/narrowphase/detail/convexity_based_algorithm/polytope.h"

namespace fcl {
namespace detail {

template <typename S>
struct BoxSpecification {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BoxSpecification(const fcl::Vector3<S>& m_size) : size(m_size) {
    X_FB.setIdentity();
  }
  fcl::Vector3<S> size;
  fcl::Transform3<S> X_FB;
};

// Test simplexToPolytope3 function.
// We construct a test scenario that two boxes are on the xy plane of frame F.
// The two boxes penetrate to each other, as shown in the bottom plot.
//              y
//          ┲━━━│━━━┱ Box1
//         ┲┃━━┱│   ┃
//      ───┃┃──┃O───┃─────x
//     box2┗┃━━┛│   ┃
//          ┗━━━│━━━┛
//              
// @param X_WF The pose of the frame F measured and expressed in the world frame
// W.             
template <typename S>
void SetUpBoxToBox(const Transform3<S>& X_WF, void* o1, void* o2, ccd_t* ccd) {
  const fcl::Vector3<S> box1_size(2, 2, 2);
  const fcl::Vector3<S> box2_size(1, 1, 2);
  fcl::Transform3<S> X_FB1, X_FB2;
  // Box 1 is fixed.
  X_FB1.setIdentity(); 
  X_FB1.translation() << 0, 0, 1;
  X_FB2.setIdentity();
  X_FB2.translation() << -0.6, 0, 1;

  const fcl::Transform3<S> X_WB1 = X_WF * X_FB1;
  const fcl::Transform3<S> X_WB2 = X_WF * X_FB2;
  fcl::Box<S> box1(box1_size);
  fcl::Box<S> box2(box2_size);
  o1 = GJKInitializer<S, fcl::Box<S>>::createGJKObject(box1, X_WB1);
  o2 = GJKInitializer<S, fcl::Box<S>>::createGJKObject(box1, X_WB2);

  // Set up ccd solver.
  CCD_INIT(ccd);
  ccd->support1 = detail::GJKInitializer<S, Box<S>>::getSupportFunction();
  ccd->support2 = detail::GJKInitializer<S, Box<S>>::getSupportFunction();
  ccd->max_iterations = 1000;
  ccd->dist_tolerance = 1E-6;
}

template <typename S>
void TestSimplexToPolytope3() {

}

GTEST_TEST(FCL_GJK_EPA, simplexToPolytope3) {
}

class EquilateralTetrahedron {
 public:
  EquilateralTetrahedron(ccd_real_t bottom_center_x = 0,
                         ccd_real_t bottom_center_y = 0,
                         ccd_real_t bottom_center_z = 0,
                         ccd_real_t edge_length = 1)
      : polytope_(new ccd_pt_t) {
    ccdPtInit(polytope_.get());
    auto AddTetrahedronVertex = [bottom_center_x, bottom_center_y,
                                 bottom_center_z, edge_length, this](
        ccd_real_t x, ccd_real_t y, ccd_real_t z) {
      return ccdPtAddVertexCoords(
          this->polytope_.get(), x * edge_length + bottom_center_x,
          y * edge_length + bottom_center_y, z * edge_length + bottom_center_z);
    };
    v_[0] = AddTetrahedronVertex(0.5, -0.5 / std::sqrt(3), 0);
    v_[1] = AddTetrahedronVertex(-0.5, -0.5 / std::sqrt(3), 0);
    v_[2] = AddTetrahedronVertex(0, 1 / std::sqrt(3), 0);
    v_[3] = AddTetrahedronVertex(0, 0, std::sqrt(2.0 / 3.0));
    e_[0] = ccdPtAddEdge(polytope_.get(), v_[0], v_[1]);
    e_[1] = ccdPtAddEdge(polytope_.get(), v_[1], v_[2]);
    e_[2] = ccdPtAddEdge(polytope_.get(), v_[2], v_[0]);
    e_[3] = ccdPtAddEdge(polytope_.get(), v_[0], v_[3]);
    e_[4] = ccdPtAddEdge(polytope_.get(), v_[1], v_[3]);
    e_[5] = ccdPtAddEdge(polytope_.get(), v_[2], v_[3]);
    f_[0] = ccdPtAddFace(polytope_.get(), e_[0], e_[1], e_[2]);
    f_[1] = ccdPtAddFace(polytope_.get(), e_[0], e_[3], e_[4]);
    f_[2] = ccdPtAddFace(polytope_.get(), e_[1], e_[4], e_[5]);
    f_[3] = ccdPtAddFace(polytope_.get(), e_[3], e_[5], e_[2]);
  }

  ccd_pt_vertex_t* v(int i) const { return v_[i]; }

  ccd_pt_edge_t* e(int i) const { return e_[i]; }

  ccd_pt_face_t* f(int i) const { return f_[i]; }

  ccd_pt_t* polytope() const { return polytope_.get(); }

  ~EquilateralTetrahedron() {
    ccdPtDestroy(polytope_.get());
    // ccdPtDestroy does not really delete the polytope_.get() pointer.
  }

 private:
  std::unique_ptr<ccd_pt_t> polytope_;
  ccd_pt_vertex_t* v_[4];
  ccd_pt_edge_t* e_[6];
  ccd_pt_face_t* f_[4];
};

GTEST_TEST(FCL_GJK_EPA, faceNormalPointingOutward) {
  // Construct a equilateral tetrahedron, compute the normal on each face.
  auto CheckTetrahedronFaceNormal = [](const EquilateralTetrahedron& p) {
    for (int i = 0; i < 4; ++i) {
      const ccd_vec3_t n =
          libccd_extension::faceNormalPointingOutward(p.polytope(), p.f(i));
      for (int j = 0; j < 4; ++j) {
        EXPECT_LE(ccdVec3Dot(&n, &p.v(j)->v.v),
                  ccdVec3Dot(&n, &p.f(i)->edge[0]->vertex[0]->v.v) + 1E-6);
      }
    }
  };
  EquilateralTetrahedron p1;
  CheckTetrahedronFaceNormal(p1);
  // Origin on the plane, and requires flipping the direction.
  EquilateralTetrahedron p2(1.0 / 6, -1.0 / (6 * std::sqrt(3)),
                            -std::sqrt(6) / 9);
  CheckTetrahedronFaceNormal(p2);
  EquilateralTetrahedron p3(0, 1.0 / (3 * std::sqrt(3)), -std::sqrt(6) / 9);
  CheckTetrahedronFaceNormal(p3);
  EquilateralTetrahedron p4(-1.0 / 6, -1.0 / (6 * std::sqrt(3)),
                            -std::sqrt(6) / 9);
  CheckTetrahedronFaceNormal(p3);

  // Check when the origin is within the polytope.
  EquilateralTetrahedron p5(0, 0, -0.1);
  CheckTetrahedronFaceNormal(p5);

  // Small tetrahedrons.
  EquilateralTetrahedron p6(0, 0, 0, 0.01);
  CheckTetrahedronFaceNormal(p6);
  EquilateralTetrahedron p7(0, 0, -0.002, 0.01);
  CheckTetrahedronFaceNormal(p7);
}

GTEST_TEST(FCL_GJK_EPA, supportEPADirection) {
  auto CheckSupportEPADirection = [](
      const ccd_pt_t* polytope, const ccd_pt_el_t* nearest_pt,
      const Eigen::Ref<const Vector3<ccd_real_t>>& dir_expected,
      ccd_real_t tol) {
    const ccd_vec3_t dir =
        libccd_extension::supportEPADirection(polytope, nearest_pt);
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(dir.v[i], dir_expected(i), tol);
    }
  };

  // Nearest point is on the bottom triangle.
  // The sampled direction should be -z unit vector.
  EquilateralTetrahedron p1(0, 0, -0.1);
  // The computation on Mac is very imprecise, thus the tolerance is big.
  const ccd_real_t tol = 3E-5;
  CheckSupportEPADirection(p1.polytope(),
                           reinterpret_cast<const ccd_pt_el_t*>(p1.f(0)),
                           Vector3<ccd_real_t>(0, 0, -1), tol);
  // Nearest point is on an edge, as the origin is on an edge.
  EquilateralTetrahedron p2(0, 0.5 / std::sqrt(3), 0);
  if (p2.e(0)->faces[0] == p2.f(0)) {
    CheckSupportEPADirection(p2.polytope(),
                             reinterpret_cast<const ccd_pt_el_t*>(p2.e(0)),
                             Vector3<ccd_real_t>(0, 0, -1), tol);
  } else {
    CheckSupportEPADirection(
        p2.polytope(), reinterpret_cast<const ccd_pt_el_t*>(p2.e(0)),
        Vector3<ccd_real_t>(0, -2 * std::sqrt(2) / 3, 1.0 / 3), tol);
  }
  // Nearest point is on a vertex, should throw an error.
  EquilateralTetrahedron p3(-0.5, 0.5 / std::sqrt(3), 0);
  EXPECT_THROW(
      libccd_extension::supportEPADirection(
          p3.polytope(), reinterpret_cast<const ccd_pt_el_t*>(p3.v(0))),
      std::runtime_error);

  // Origin is an internal point of the bottom triangle
  EquilateralTetrahedron p4(0, 0, 0);
  CheckSupportEPADirection(p4.polytope(),
                           reinterpret_cast<const ccd_pt_el_t*>(p4.f(0)),
                           Vector3<ccd_real_t>(0, 0, -1), tol);

  // Nearest point is on face(1)
  EquilateralTetrahedron p5(0, 1 / (3 * std::sqrt(3)),
                            -std::sqrt(6) / 9 + 0.01);
  CheckSupportEPADirection(
      p5.polytope(), reinterpret_cast<const ccd_pt_el_t*>(p5.f(1)),
      Vector3<ccd_real_t>(0, -2 * std::sqrt(2) / 3, 1.0 / 3), tol);
}

GTEST_TEST(FCL_GJK_EPA, isOutsidePolytopeFace) {
  EquilateralTetrahedron p;

  auto CheckPointOutsidePolytopeFace = [&p](ccd_real_t x, ccd_real_t y,
                                            ccd_real_t z, int face_index,
                                            bool is_outside_expected) {
    ccd_vec3_t pt;
    pt.v[0] = x;
    pt.v[1] = y;
    pt.v[2] = z;
    EXPECT_EQ(libccd_extension::isOutsidePolytopeFace(p.polytope(),
                                                      p.f(face_index), &pt),
              is_outside_expected);
  };

  const bool expect_inside = false;
  const bool expect_outside = true;
  // point (0, 0, 0.1) is inside the tetrahedron.
  CheckPointOutsidePolytopeFace(0, 0, 0.1, 0, expect_inside);
  CheckPointOutsidePolytopeFace(0, 0, 0.1, 1, expect_inside);
  CheckPointOutsidePolytopeFace(0, 0, 0.1, 2, expect_inside);
  CheckPointOutsidePolytopeFace(0, 0, 0.1, 3, expect_inside);

  // point(0, 0, 2) is outside the tetrahedron. But it is on the "inner" side
  // of the bottom face.
  CheckPointOutsidePolytopeFace(0, 0, 2, 0, expect_inside);
  CheckPointOutsidePolytopeFace(0, 0, 2, 1, expect_outside);
  CheckPointOutsidePolytopeFace(0, 0, 2, 2, expect_outside);
  CheckPointOutsidePolytopeFace(0, 0, 2, 3, expect_outside);

  // point (0, 0, 0) is right on the bottom face.
  CheckPointOutsidePolytopeFace(0, 0, 0, 0, expect_inside);
  CheckPointOutsidePolytopeFace(0, 0, 0, 1, expect_inside);
  CheckPointOutsidePolytopeFace(0, 0, 0, 2, expect_inside);
  CheckPointOutsidePolytopeFace(0, 0, 0, 3, expect_inside);
}

// Construct a polytope with the following shape, namely an equilateral triangle
// on the top, and an equilateral triangle of the same size, but rotate by 60
// degrees on the bottom. We will then connect the vertices of the equilateral
// triangles to form a convex polytope.
//         v₄
//     v₃__╱╲__v₅
//       ╲╱  ╲╱
//       ╱____╲
//     v₂  ╲╱  v₀
//         v₁
// The edges are in this order connected in a counter-clockwise order.
// e(0) connects v(0) and v(2).
// e(1) connects v(2) and v(4).
// e(2) connects v(4) and v(0).
// e(3) connects v(1) and v(3).
// e(4) connects v(3) and v(5).
// e(5) connects v(5) and v(1).
// e(6 + i) connects v(i) and v(i + 1).
// f(0) is the upper triangle.
// f(1) is the lower triangle.
// f(2 + i) is the triangle that connects v(i), v(i + 1) and v(i + 2), namely
// the triangles on the side.
class Hexagram {
 public:
  Hexagram(ccd_real_t bottom_center_x = 0, ccd_real_t bottom_center_y = 0,
           ccd_real_t bottom_center_z = 0)
      : polytope_(new ccd_pt_t) {
    ccdPtInit(polytope_.get());
    auto AddHexagramVertex = [bottom_center_x, bottom_center_y, bottom_center_z,
                              this](ccd_real_t x, ccd_real_t y, ccd_real_t z) {
      return ccdPtAddVertexCoords(this->polytope_.get(), x + bottom_center_x,
                                  y + bottom_center_y, z + bottom_center_z);
    };
    // right corner of upper triangle
    v_[0] = AddHexagramVertex(0.5, -1 / std::sqrt(3), 1);
    // bottom corner of lower triangle
    v_[1] = AddHexagramVertex(0, -2 / std::sqrt(3), 0);
    // left corner of upper triangle
    v_[2] = AddHexagramVertex(-0.5, -1 / std::sqrt(3), 1);
    // left corner of lower triangle
    v_[3] = AddHexagramVertex(-0.5, 1 / std::sqrt(3), 0);
    // top corner of upper triangle
    v_[4] = AddHexagramVertex(0, 2 / std::sqrt(3), 1);
    // right corner of lower triangle
    v_[5] = AddHexagramVertex(0.5, 1 / std::sqrt(3), 0);

    // edges on the upper triangle
    e_[0] = ccdPtAddEdge(polytope_.get(), v_[0], v_[2]);
    e_[1] = ccdPtAddEdge(polytope_.get(), v_[2], v_[4]);
    e_[2] = ccdPtAddEdge(polytope_.get(), v_[4], v_[0]);
    // edges on the lower triangle
    e_[3] = ccdPtAddEdge(polytope_.get(), v_[1], v_[3]);
    e_[4] = ccdPtAddEdge(polytope_.get(), v_[3], v_[5]);
    e_[5] = ccdPtAddEdge(polytope_.get(), v_[5], v_[1]);
    // edges connecting the upper triangle to the lower triangle
    for (int i = 0; i < 6; ++i) {
      e_[6 + i] = ccdPtAddEdge(polytope_.get(), v_[i], v_[(i + 1) % 6]);
    }

    // upper triangle
    f_[0] = ccdPtAddFace(polytope_.get(), e_[0], e_[1], e_[2]);
    // lower triangle
    f_[1] = ccdPtAddFace(polytope_.get(), e_[3], e_[4], e_[5]);
    // triangles on the side
    f_[2] = ccdPtAddFace(polytope_.get(), e_[0], e_[7], e_[6]);
    f_[3] = ccdPtAddFace(polytope_.get(), e_[7], e_[8], e_[3]);
    f_[4] = ccdPtAddFace(polytope_.get(), e_[8], e_[9], e_[1]);
    f_[5] = ccdPtAddFace(polytope_.get(), e_[9], e_[10], e_[4]);
    f_[6] = ccdPtAddFace(polytope_.get(), e_[10], e_[11], e_[2]);
    f_[7] = ccdPtAddFace(polytope_.get(), e_[11], e_[6], e_[5]);
  }

  ~Hexagram() { ccdPtDestroy(polytope_.get()); }

  ccd_pt_t* polytope() const { return polytope_.get(); }

  ccd_pt_vertex_t* v(int i) const { return v_[i]; }

  ccd_pt_edge_t* e(int i) const { return e_[i]; }

  ccd_pt_face_t* f(int i) const { return f_[i]; }

 private:
  std::unique_ptr<ccd_pt_t> polytope_;
  ccd_pt_vertex_t* v_[6];
  ccd_pt_edge_t* e_[12];
  ccd_pt_face_t* f_[8];
};

template <typename T>
bool IsElementInSet(const std::unordered_set<T>& S, const T& element) {
  return S.find(element) != S.end();
}

template <typename T>
void CheckComputeVisiblePatchCommon(
    const T& polytope, const std::unordered_set<ccd_pt_edge_t*>& border_edges,
    const std::unordered_set<ccd_pt_face_t*>& visible_faces,
    const std::unordered_set<ccd_pt_edge_t*> internal_edges,
    const std::unordered_set<int>& border_edge_indices_expected,
    const std::unordered_set<int>& visible_face_indices_expected,
    const std::unordered_set<int> internal_edges_indices_expected) {
  // Check border_edges
  EXPECT_EQ(border_edges.size(), border_edge_indices_expected.size());
  for (const int edge_index : border_edge_indices_expected) {
    EXPECT_TRUE(IsElementInSet(border_edges, polytope.e(edge_index)));
  }
  // Check visible_faces
  EXPECT_EQ(visible_faces.size(), visible_face_indices_expected.size());
  for (const int face_index : visible_face_indices_expected) {
    EXPECT_TRUE(IsElementInSet(visible_faces, polytope.f(face_index)));
  }
  // Check internal_edges
  EXPECT_EQ(internal_edges.size(), internal_edges_indices_expected.size());
  for (const auto edge_index : internal_edges_indices_expected) {
    EXPECT_TRUE(IsElementInSet(internal_edges, polytope.e(edge_index)));
  }
}

template <typename T>
void CheckComputeVisiblePatchRecursive(
    const T& polytope, ccd_pt_face_t* face,
    const std::vector<int>& edge_indices, const ccd_vec3_t& new_vertex,
    const std::unordered_set<int>& border_edge_indices_expected,
    const std::unordered_set<int>& visible_face_indices_expected,
    const std::unordered_set<int>& internal_edges_indices_expected) {
  std::unordered_set<ccd_pt_edge_t*> border_edges;
  std::unordered_set<ccd_pt_face_t*> visible_faces;
  visible_faces.insert(face);
  std::unordered_set<ccd_pt_edge_t*> internal_edges;
  for (const int edge_index : edge_indices) {
    libccd_extension::ComputeVisiblePatchRecursive(
        *polytope.polytope(), *face, edge_index, new_vertex, &border_edges,
        &visible_faces, &internal_edges);
  }
  CheckComputeVisiblePatchCommon(polytope, border_edges, visible_faces,
                                 internal_edges, border_edge_indices_expected,
                                 visible_face_indices_expected,
                                 internal_edges_indices_expected);
}

template <typename T>
void CheckComputeVisiblePatch(
    const T& polytope, ccd_pt_face_t* face, const ccd_vec3_t& new_vertex,
    const std::unordered_set<int>& border_edge_indices_expected,
    const std::unordered_set<int>& visible_face_indices_expected,
    const std::unordered_set<int>& internal_edges_indices_expected) {
  std::unordered_set<ccd_pt_edge_t*> border_edges;
  std::unordered_set<ccd_pt_face_t*> visible_faces;
  std::unordered_set<ccd_pt_edge_t*> internal_edges;
  libccd_extension::ComputeVisiblePatch(*polytope.polytope(), *face, new_vertex,
                                        &border_edges, &visible_faces,
                                        &internal_edges);

  CheckComputeVisiblePatchCommon(polytope, border_edges, visible_faces,
                                 internal_edges, border_edge_indices_expected,
                                 visible_face_indices_expected,
                                 internal_edges_indices_expected);
}

GTEST_TEST(FCL_GJK_EPA, ComputeVisiblePatch1) {
  // 1 visible face.
  Hexagram hex;
  // Point P is just slightly above the top triangle. Only the top triangle can
  // be seen from point P.
  ccd_vec3_t p;
  p.v[0] = 0;
  p.v[1] = 0;
  p.v[2] = 1.1;
  const std::unordered_set<int> empty_set;
  CheckComputeVisiblePatchRecursive(hex, hex.f(0), {0}, p, {0}, {0}, empty_set);

  CheckComputeVisiblePatch(hex, hex.f(0), p, {0, 1, 2}, {0}, empty_set);
}

GTEST_TEST(FCL_GJK_EPA, ComputeVisiblePatch2) {
  // 4 visible faces.
  Hexagram hex;
  // Point P is just above the top triangle by a certain height, such that it
  // can see the triangles on the side, which connects two vertices on the upper
  // triangle, and one vertex on the lower triangle.
  ccd_vec3_t p;
  p.v[0] = 0;
  p.v[1] = 0;
  p.v[2] = 2.1;
  CheckComputeVisiblePatchRecursive(hex, hex.f(0), {0}, p, {6, 7}, {0, 2}, {0});

  CheckComputeVisiblePatch(hex, hex.f(0), p, {6, 7, 8, 9, 10, 11}, {0, 2, 4, 6},
                           {0, 1, 2});
}

GTEST_TEST(FCL_GJK_EPA, ComputeVisiblePatch3) {
  // 2 visible faces.
  Hexagram hex;
  // Point P is just outside the upper triangle (face0) and the triangle face2,
  // it can see both face0 and face2, but not the other triangles.
  ccd_vec3_t p;
  p.v[0] = 0;
  p.v[1] = -1 / std::sqrt(3) - 0.1;
  p.v[2] = 1.1;
  CheckComputeVisiblePatchRecursive(hex, hex.f(0), {0}, p, {6, 7}, {0, 2}, {0});

  CheckComputeVisiblePatch(hex, hex.f(0), p, {1, 2, 6, 7}, {0, 2}, {0});
}

GTEST_TEST(FCL_GJK_EPA, ComputeVisiblePatch4) {
  // Test with the equilateral tetrahedron.
  // Point P is outside of an edge on the bottom triangle. It can see both faces
  // neighbouring that edge.

  EquilateralTetrahedron tetrahedron(0, 0, -0.1);
  ccd_vec3_t p;
  p.v[0] = 0;
  p.v[1] = -1 / std::sqrt(3) - 0.1;
  p.v[2] = -0.2;

  // Start with from face 0.
  CheckComputeVisiblePatch(tetrahedron, tetrahedron.f(0), p, {1, 2, 3, 4},
                           {0, 1}, {0});

  // Start with from face 1.
  CheckComputeVisiblePatch(tetrahedron, tetrahedron.f(1), p, {1, 2, 3, 4},
                           {0, 1}, {0});
}

// Returns true if the the distance difference between the two vertices are
// below tol.
bool VertexPositionCoincide(const ccd_pt_vertex_t* v1,
                            const ccd_pt_vertex_t* v2, ccd_real_t tol) {
  return ccdVec3Dist2(&v1->v.v, &v2->v.v) < tol * tol;
}

// Return true, if the vertices in e1 are all mapped to the vertices in e2,
// according to the mapping @p map_v1_to_v2.
bool EdgeMatch(const ccd_pt_edge_t* e1, const ccd_pt_edge_t* e2,
               const std::unordered_map<ccd_pt_vertex_t*, ccd_pt_vertex_t*>&
                   map_v1_to_v2) {
  ccd_pt_vertex_t* v2_expected[2];
  for (int i = 0; i < 2; ++i) {
    auto it = map_v1_to_v2.find(e1->vertex[i]);
    if (it == map_v1_to_v2.end()) {
      throw std::logic_error("vertex[" + std::to_string(i) +
                             "] in e1 is not found in map_v1_to_v2");
    }
    v2_expected[i] = it->second;
  }
  return (v2_expected[0] == e2->vertex[0] && v2_expected[1] == e2->vertex[1]) ||
         (v2_expected[0] == e2->vertex[1] && v2_expected[1] == e2->vertex[0]);
}

// Return true, if the edges in f1 are all mapped to the edges in f2, according
// to the mapping @p map_e1_to_e2.
bool TriangleMatch(
    const ccd_pt_face_t* f1, const ccd_pt_face_t* f2,
    const std::unordered_map<ccd_pt_edge_t*, ccd_pt_edge_t*>& map_e1_to_e2) {
  std::unordered_set<ccd_pt_edge_t*> e2_expected;
  for (int i = 0; i < 3; ++i) {
    auto it = map_e1_to_e2.find(f1->edge[i]);
    if (it == map_e1_to_e2.end()) {
      throw std::logic_error("edge[" + std::to_string(i) +
                             "] in f1 is not found in map_e1_to_e2");
    }
    e2_expected.insert(it->second);
  }
  // The edges in f1 have to be distinct.
  EXPECT_EQ(e2_expected.size(), 3u);
  for (int i = 0; i < 3; ++i) {
    auto it = e2_expected.find(f2->edge[i]);
    if (it == e2_expected.end()) {
      return false;
    }
  }
  return true;
}

// Construct the mapping from feature1_list to feature2_list. There should be a
// one-to-one correspondence between feature1_list and feature2_list.
// @param feature1_list[in] A list of features to be mapped from.
// @param feature2_list[in] A list of features to be mapped to.
// @param cmp_feature[in] Returns true if two features are identical, otherwise
// returns false.
// @param feature1[out] The set of features in feature1_list.
// @param feature2[out] The set of features in feature2_list.
// @param map_feature1_to_feature2[out] Maps a feature in feature1_list to
// a feature in feature2_list.
// @note The features in feature1_list should be distince, so are in
// feature2_list.
template <typename T>
void MapFeature1ToFeature2(
    const ccd_list_t* feature1_list, const ccd_list_t* feature2_list,
    std::function<bool(const T*, const T*)> cmp_feature,
    std::unordered_set<T*>* feature1, std::unordered_set<T*>* feature2,
    std::unordered_map<T*, T*>* map_feature1_to_feature2) {
  feature1->clear();
  feature2->clear();
  map_feature1_to_feature2->clear();
  T* f;
  ccdListForEachEntry(feature1_list, f, T, list) {
    auto it = feature1->find(f);
    assert(it == feature1->end());
    feature1->emplace_hint(it, f);
  }
  ccdListForEachEntry(feature2_list, f, T, list) {
    auto it = feature2->find(f);
    assert(it == feature2->end());
    feature2->emplace_hint(it, f);
  }
  EXPECT_EQ(feature1->size(), feature2->size());
  for (const auto& f1 : *feature1) {
    bool found_match = false;
    for (const auto& f2 : *feature2) {
      if (cmp_feature(f1, f2)) {
        map_feature1_to_feature2->emplace_hint(map_feature1_to_feature2->end(),
                                               f1, f2);
        found_match = true;
        break;
      }
    }
    EXPECT_TRUE(found_match);
  }
  // Every feature in feature1_list should be matched to a feature in
  // feature2_list.
  EXPECT_EQ(map_feature1_to_feature2->size(), feature1->size());
}

void ComparePolytope(const ccd_pt_t* polytope1, const ccd_pt_t* polytope2,
                     ccd_real_t tol) {
  // Build the mapping between the vertices in polytope1 to the vertices in
  // polytope2.
  std::unordered_set<ccd_pt_vertex_t *> v1_set, v2_set;
  std::unordered_map<ccd_pt_vertex_t*, ccd_pt_vertex_t*> map_v1_to_v2;
  MapFeature1ToFeature2<ccd_pt_vertex_t>(
      &polytope1->vertices, &polytope2->vertices,
      [tol](const ccd_pt_vertex_t* v1, const ccd_pt_vertex_t* v2) {
        return VertexPositionCoincide(v1, v2, tol);
      },
      &v1_set, &v2_set, &map_v1_to_v2);

  // Build the mapping between the edges in polytope1 to the edges in polytope2.
  std::unordered_set<ccd_pt_edge_t *> e1_set, e2_set;
  std::unordered_map<ccd_pt_edge_t*, ccd_pt_edge_t*> map_e1_to_e2;
  MapFeature1ToFeature2<ccd_pt_edge_t>(
      &polytope1->edges, &polytope2->edges,
      [map_v1_to_v2](const ccd_pt_edge_t* e1, const ccd_pt_edge_t* e2) {
        return EdgeMatch(e1, e2, map_v1_to_v2);
      },
      &e1_set, &e2_set, &map_e1_to_e2);

  // Build the mapping between the faces in polytope1 to the faces in polytope2.
  std::unordered_set<ccd_pt_face_t *> f1_set, f2_set;
  std::unordered_map<ccd_pt_face_t*, ccd_pt_face_t*> map_f1_to_f2;
  MapFeature1ToFeature2<ccd_pt_face_t>(
      &polytope1->faces, &polytope2->faces,
      [map_e1_to_e2](const ccd_pt_face_t* f1, const ccd_pt_face_t* f2) {
        return TriangleMatch(f1, f2, map_e1_to_e2);
      },
      &f1_set, &f2_set, &map_f1_to_f2);

  /* TODO(hongkai.dai@tri.global): enable the following check, when issue
   https://github.com/danfis/libccd/issues/46 has been fixed. Currently
   ccd_pt_vertex_t.edges are garbage.
  // Now make sure that the edges connected to a vertex in polytope 1, are the
  // same edges connected to the corresponding vertex in polytope 2.
  for (const auto& v1 : v1_set) {
    auto v2 = map_v1_to_v2[v1];
    std::unordered_set<ccd_pt_edge_t*> v1_edges, v2_edges;
    ccd_pt_edge_t* e;
    ccdListForEachEntry(&v1->edges, e, ccd_pt_edge_t, list) {
      v1_edges.insert(e);
    }
    ccdListForEachEntry(&v2->edges, e, ccd_pt_edge_t, list) {
      v2_edges.insert(e);
    }
    EXPECT_EQ(v1_edges.size(), v2_edges.size());
    // Now check for each edge connecting to v1, the corresponding edge is
    // connected to v2.
    for (const auto& v1_e : v1_edges) {
      auto it = map_e1_to_e2.find(v1_e);
      EXPECT_NE(it, map_e1_to_e2.end()) {
      auto v2_e = it->second;
      EXPECT_NE(v2_edges.find(v2_e), v2_edges.end());
    }
  }*/

  // Make sure that the faces connected to each edge in polytope 1, are the same
  // face connected to the corresponding face in polytope 2.
  for (const auto& e1 : e1_set) {
    auto e2 = map_e1_to_e2[e1];
    ccd_pt_face_t* f2_expected[2];
    for (int i = 0; i < 2; ++i) {
      f2_expected[i] = map_f1_to_f2[e1->faces[i]];
    }
    EXPECT_TRUE(
        (f2_expected[0] == e2->faces[0] && f2_expected[1] == e2->faces[1]) ||
        (f2_expected[0] == e2->faces[1] && f2_expected[1] == e2->faces[0]));
  }
}

GTEST_TEST(FCL_GJK_EPA, expandPolytope1) {
  // Expand the equilateral tetrahedron by adding a point just outside one of
  // the triangle face. That nearest triangle face will be deleted, and the
  // three new faces will be added, by connecting the new vertex with the three
  // vertices on the removed face.
  EquilateralTetrahedron polytope(0, 0, -0.1);
  // nearest point is on the bottom triangle
  ccd_support_t newv;
  newv.v.v[0] = 0;
  newv.v.v[1] = 0;
  newv.v.v[2] = -0.2;

  const int result = libccd_extension::expandPolytope(
      polytope.polytope(), (ccd_pt_el_t*)polytope.f(0), &newv);
  EXPECT_EQ(result, 0);

  // Construct the expanded polytope manually.
  EquilateralTetrahedron tetrahedron(0, 0, -0.1);
  ccd_pt_t* polytope_expected = tetrahedron.polytope();
  // The bottom face is removed.
  ccdPtDelFace(polytope_expected, tetrahedron.f(0));
  // Insert the vertex.
  ccd_pt_vertex_t* new_vertex =
      ccdPtAddVertexCoords(polytope_expected, 0, 0, -0.2);
  // Add new edges.
  ccd_pt_edge_t* new_edges[3];
  for (int i = 0; i < 3; ++i) {
    new_edges[i] =
        ccdPtAddEdge(polytope_expected, new_vertex, tetrahedron.v(i));
  }
  // Add new faces.
  ccdPtAddFace(polytope_expected, tetrahedron.e(0), new_edges[0], new_edges[1]);
  ccdPtAddFace(polytope_expected, tetrahedron.e(1), new_edges[1], new_edges[2]);
  ccdPtAddFace(polytope_expected, tetrahedron.e(2), new_edges[2], new_edges[0]);

  ComparePolytope(polytope.polytope(), polytope_expected, 1E-3);
}

GTEST_TEST(FCL_GJK_EPA, expandPolytope2) {
  // Expand the equilateral tetrahedron by adding a point just outside one edge.
  // The two neighbouring faces of that edge will be deleted. Four new faces
  // will be added, by connecting the new vertex with the remaining vertex on
  // the two removed faces, that is opposite to the removed edge.
  EquilateralTetrahedron polytope(0, 0, -0.1);
  // nearest point is on the bottom triangle
  ccd_support_t newv;
  newv.v.v[0] = 0;
  newv.v.v[1] = -0.5 / std::sqrt(3) - 0.1;
  newv.v.v[2] = -0.2;

  const int result = libccd_extension::expandPolytope(
      polytope.polytope(), (ccd_pt_el_t*)polytope.e(0), &newv);
  EXPECT_EQ(result, 0);

  // Construct the expanded polytope manually.
  EquilateralTetrahedron tetrahedron(0, 0, -0.1);
  ccd_pt_t* polytope_expected = tetrahedron.polytope();
  // The bottom face is removed.
  ccdPtDelFace(polytope_expected, tetrahedron.f(0));
  // The other face that neighbours with f(0) is removed.
  ccdPtDelFace(polytope_expected, tetrahedron.f(1));
  // The nearest edge is removed.
  ccdPtDelEdge(polytope_expected, tetrahedron.e(0));
  // Insert the vertex.
  ccd_pt_vertex_t* new_vertex = ccdPtAddVertexCoords(
      polytope_expected, newv.v.v[0], newv.v.v[1], newv.v.v[2]);
  // Add new edges.
  ccd_pt_edge_t* new_edges[4];
  new_edges[0] = ccdPtAddEdge(polytope_expected, new_vertex, tetrahedron.v(0));
  new_edges[1] = ccdPtAddEdge(polytope_expected, new_vertex, tetrahedron.v(1));
  new_edges[2] = ccdPtAddEdge(polytope_expected, new_vertex, tetrahedron.v(2));
  new_edges[3] = ccdPtAddEdge(polytope_expected, new_vertex, tetrahedron.v(3));
  // Add new faces.
  ccdPtAddFace(polytope_expected, tetrahedron.e(3), new_edges[0], new_edges[3]);
  ccdPtAddFace(polytope_expected, tetrahedron.e(2), new_edges[0], new_edges[2]);
  ccdPtAddFace(polytope_expected, tetrahedron.e(4), new_edges[1], new_edges[3]);
  ccdPtAddFace(polytope_expected, tetrahedron.e(1), new_edges[1], new_edges[2]);

  ComparePolytope(polytope.polytope(), polytope_expected, 1E-3);
}

GTEST_TEST(FCL_GJK_EPA, expandPolytope3) {
  // Expand the Hexagram by adding a point just above the upper triangle.
  // The upper triangle will be deleted. Three new faces will be added, by
  // connecting the new vertex with the three vertices of the removed triangle.
  Hexagram hex(0, 0, -0.9);
  // nearest point is on the top triangle
  ccd_support_t newv;
  newv.v.v[0] = 0;
  newv.v.v[1] = 0;
  newv.v.v[2] = 0.2;

  const int result = libccd_extension::expandPolytope(
      hex.polytope(), (ccd_pt_el_t*)hex.f(0), &newv);
  EXPECT_EQ(result, 0);

  // Construct the expanded polytope manually.
  Hexagram hex_duplicate(0, 0, -0.9);
  ccd_pt_t* polytope_expected = hex_duplicate.polytope();
  // Remove the upper triangle.
  ccdPtDelFace(polytope_expected, hex_duplicate.f(0));

  // Add the new vertex.
  ccd_pt_vertex_t* new_vertex = ccdPtAddVertexCoords(
      polytope_expected, newv.v.v[0], newv.v.v[1], newv.v.v[2]);
  // Add the new edges.
  ccd_pt_edge_t* new_edges[3];
  new_edges[0] =
      ccdPtAddEdge(polytope_expected, new_vertex, hex_duplicate.v(0));
  new_edges[1] =
      ccdPtAddEdge(polytope_expected, new_vertex, hex_duplicate.v(2));
  new_edges[2] =
      ccdPtAddEdge(polytope_expected, new_vertex, hex_duplicate.v(4));
  // Add the new faces.
  ccdPtAddFace(polytope_expected, new_edges[0], new_edges[1],
               hex_duplicate.e(0));
  ccdPtAddFace(polytope_expected, new_edges[1], new_edges[2],
               hex_duplicate.e(1));
  ccdPtAddFace(polytope_expected, new_edges[2], new_edges[0],
               hex_duplicate.e(2));

  ComparePolytope(hex.polytope(), polytope_expected, 1E-3);
}

GTEST_TEST(FCL_GJK_EPA, expandPolytope4) {
  // Expand the Hexagram by adding a point above the upper triangle by a certain
  // height, such that the new vertex can see the upper triangle, together with
  // the three triangles on the side of the hexagram. All these four triangles
  // will be removed. 6 new faces will be added by connecting the new vertex,
  // with eah vertex in the old hexagram.
  Hexagram hex(0, 0, -0.9);
  // nearest point is on the top triangle
  ccd_support_t newv;
  newv.v.v[0] = 0;
  newv.v.v[1] = 0;
  newv.v.v[2] = 1.2;

  const int result = libccd_extension::expandPolytope(
      hex.polytope(), (ccd_pt_el_t*)hex.f(0), &newv);
  EXPECT_EQ(result, 0);

  // Construct the expanded polytope manually.
  Hexagram hex_duplicate(0, 0, -0.9);
  ccd_pt_t* polytope_expected = hex_duplicate.polytope();
  // Remove the upper triangle.
  ccdPtDelFace(polytope_expected, hex_duplicate.f(0));
  // Remove the triangles on the side, which consists of two vertices on the
  // upper triangle, and one vertex on the lower triangle.
  ccdPtDelFace(polytope_expected, hex_duplicate.f(2));
  ccdPtDelFace(polytope_expected, hex_duplicate.f(4));
  ccdPtDelFace(polytope_expected, hex_duplicate.f(6));
  // Remove the edges of the upper triangle.
  ccdPtDelEdge(polytope_expected, hex_duplicate.e(0));
  ccdPtDelEdge(polytope_expected, hex_duplicate.e(1));
  ccdPtDelEdge(polytope_expected, hex_duplicate.e(2));

  // Add the new vertex.
  ccd_pt_vertex_t* new_vertex = ccdPtAddVertexCoords(
      polytope_expected, newv.v.v[0], newv.v.v[1], newv.v.v[2]);
  // Add the new edges.
  ccd_pt_edge_t* new_edges[6];
  for (int i = 0; i < 6; ++i) {
    new_edges[i] =
        ccdPtAddEdge(polytope_expected, new_vertex, hex_duplicate.v(i));
  }
  // Add the new faces.
  for (int i = 0; i < 6; ++i) {
    ccdPtAddFace(polytope_expected, new_edges[i % 6], new_edges[(i + 1) % 6],
                 hex_duplicate.e(i + 6));
  }
  ComparePolytope(hex.polytope(), polytope_expected, 1E-3);
}

void CompareCcdVec3(const ccd_vec3_t& v, const ccd_vec3_t& v_expected,
                    ccd_real_t tol) {
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(v.v[i], v_expected.v[i], tol);
  }
}

GTEST_TEST(FCL_GJK_EPA, penEPAPosClosest1) {
  // The nearest point is a vertex on the polytope.
  // tetrahedron.v(0) is the origin.
  EquilateralTetrahedron tetrahedron(-0.5, 0.5 / std::sqrt(3), 0);
  // Make sure that v1 - v2 = v.
  tetrahedron.v(0)->v.v1.v[0] = 1;
  tetrahedron.v(0)->v.v1.v[1] = 2;
  tetrahedron.v(0)->v.v1.v[2] = 3;
  for (int i = 0; i < 3; ++i) {
    tetrahedron.v(0)->v.v2.v[i] = tetrahedron.v(0)->v.v1.v[i];
  }
  ccd_vec3_t p1, p2;
  EXPECT_EQ(libccd_extension::penEPAPosClosest(
                (const ccd_pt_el_t*)tetrahedron.v(0), &p1, &p2),
            0);
  CompareCcdVec3(p1, tetrahedron.v(0)->v.v1, 1E-14);
  CompareCcdVec3(p2, tetrahedron.v(0)->v.v2, 1E-14);
}

GTEST_TEST(FCL_GJK_EPA, penEPAPosClosest2) {
  // The nearest point is on an edge of the polytope.
  // tetrahedron.e(1) contains the origin.
  EquilateralTetrahedron tetrahedron(0.25, -0.25 / std::sqrt(3), 0);
  // e(1) connects two vertices v(1) and v(2), make sure that v(1).v1 - v(1).v2
  // = v(1).v, also v(2).v1 - v(2).v2 = v(2).v
  tetrahedron.v(1)->v.v1.v[0] = 1;
  tetrahedron.v(1)->v.v1.v[1] = 2;
  tetrahedron.v(1)->v.v1.v[2] = 3;
  tetrahedron.v(2)->v.v1.v[0] = 4;
  tetrahedron.v(2)->v.v1.v[1] = 5;
  tetrahedron.v(2)->v.v1.v[2] = 6;
  for (int i = 1; i <= 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      tetrahedron.v(i)->v.v2.v[j] =
          tetrahedron.v(i)->v.v1.v[j] - tetrahedron.v(i)->v.v.v[j];
    }
  }
  // Notice that origin = 0.5*v(1).v + 0.5*v(2).v
  // So p1 = 0.5*v(1).v1 + 0.5*v(2).v1
  //    p2 = 0.5*v(1).v2 + 0.5*v(2).v2
  ccd_vec3_t p1, p2;
  EXPECT_EQ(
      libccd_extension::penEPAPosClosest(
          reinterpret_cast<const ccd_pt_el_t*>(tetrahedron.e(1)), &p1, &p2),
      0);
  ccd_vec3_t p1_expected, p2_expected;
  ccdVec3Copy(&p1_expected, &tetrahedron.v(1)->v.v1);
  ccdVec3Add(&p1_expected, &tetrahedron.v(2)->v.v1);
  ccdVec3Scale(&p1_expected, ccd_real_t(0.5));
  ccdVec3Copy(&p2_expected, &tetrahedron.v(1)->v.v2);
  ccdVec3Add(&p2_expected, &tetrahedron.v(2)->v.v2);
  ccdVec3Scale(&p2_expected, ccd_real_t(0.5));

  CompareCcdVec3(p1, p1_expected, constants<ccd_real_t>::eps_78());
  CompareCcdVec3(p2, p2_expected, constants<ccd_real_t>::eps_78());
}

GTEST_TEST(FCL_GJK_EPA, penEPAPosClosest3) {
  // The nearest point is on a face of the polytope, It is the center of
  // tetrahedron.f(1).
  const Vector3<ccd_real_t> bottom_center_pos =
      Vector3<ccd_real_t>(0, 1.0 / (3 * std::sqrt(3)), -std::sqrt(6) / 9) +
      0.01 * Vector3<ccd_real_t>(0, -2 * std::sqrt(2) / 3, 1.0 / 3);
  EquilateralTetrahedron tetrahedron(bottom_center_pos(0), bottom_center_pos(1),
                                     bottom_center_pos(2));
  // Assign v(i).v1 and v(i).v2 for i = 0, 1, 3, such that
  // v(i).v = v(i).v1 - v(i).v2
  tetrahedron.v(0)->v.v1.v[0] = 1;
  tetrahedron.v(0)->v.v1.v[1] = 2;
  tetrahedron.v(0)->v.v1.v[2] = 3;
  tetrahedron.v(1)->v.v1.v[0] = 4;
  tetrahedron.v(1)->v.v1.v[1] = 5;
  tetrahedron.v(1)->v.v1.v[2] = 6;
  tetrahedron.v(3)->v.v1.v[0] = 7;
  tetrahedron.v(3)->v.v1.v[1] = 8;
  tetrahedron.v(3)->v.v1.v[2] = 9;
  for (int i : {0, 1, 3}) {
    for (int j = 0; j < 3; ++j) {
      tetrahedron.v(i)->v.v2.v[j] =
          tetrahedron.v(i)->v.v1.v[j] - tetrahedron.v(i)->v.v.v[j];
    }
  }

  ccd_vec3_t p1, p2;
  EXPECT_EQ(libccd_extension::penEPAPosClosest(
                (const ccd_pt_el_t*)tetrahedron.f(1), &p1, &p2),
            0);

  // Notice that the nearest point = 1/3 * v(0).v + 1/3 * v(1).v + 1/3 * v(3).v
  // So p1 = 1/3 * (v(0).v1 + v(1).v1 + v(3).v1)
  //    p2 = 1/3 * (v(0).v2 + v(1).v2 + v(3).v2)
  ccd_vec3_t p1_expected, p2_expected;
  ccdVec3Copy(&p1_expected, &tetrahedron.v(0)->v.v1);
  ccdVec3Add(&p1_expected, &tetrahedron.v(1)->v.v1);
  ccdVec3Add(&p1_expected, &tetrahedron.v(3)->v.v1);
  ccdVec3Scale(&p1_expected, ccd_real_t(1.0 / 3));
  ccdVec3Copy(&p2_expected, &tetrahedron.v(0)->v.v2);
  ccdVec3Add(&p2_expected, &tetrahedron.v(1)->v.v2);
  ccdVec3Add(&p2_expected, &tetrahedron.v(3)->v.v2);
  ccdVec3Scale(&p2_expected, ccd_real_t(1.0 / 3));

  CompareCcdVec3(p1, p1_expected, constants<ccd_real_t>::eps_78());
  CompareCcdVec3(p2, p2_expected, constants<ccd_real_t>::eps_78());
}
}  // namespace detail
}  // namespace fcl

//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

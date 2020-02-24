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

#include <array>
#include <memory>

#include <gtest/gtest.h>

#include "fcl/narrowphase/detail/convexity_based_algorithm/polytope.h"
#include "expect_throws_message.h"

namespace fcl {
namespace detail {

class Polytope {
 public:
  Polytope() {
    polytope_ = new ccd_pt_t;
    ccdPtInit(polytope_);
  }

  ~Polytope() {
    // ccdPtDestroy() destroys the vertices, edges, and faces, contained in the
    // polytope, allowing the polytope itself to be subsequently deleted.
    ccdPtDestroy(polytope_);
    delete polytope_;
  }

  ccd_pt_vertex_t& v(int i) { return *v_[i]; }

  ccd_pt_edge_t& e(int i) { return *e_[i]; }

  ccd_pt_face_t& f(int i) { return *f_[i]; }

  ccd_pt_t& polytope() { return *polytope_; }

  const ccd_pt_vertex_t& v(int i) const { return *v_[i]; }

  const ccd_pt_edge_t& e(int i) const { return *e_[i]; }

  const ccd_pt_face_t& f(int i) const { return *f_[i]; }

  const ccd_pt_t& polytope() const { return *polytope_; }

 protected:
  std::vector<ccd_pt_vertex_t*>& v() { return v_; }
  std::vector<ccd_pt_edge_t*>& e() { return e_; }
  std::vector<ccd_pt_face_t*>& f() { return f_; }

 private:
  std::vector<ccd_pt_vertex_t*> v_;
  std::vector<ccd_pt_edge_t*> e_;
  std::vector<ccd_pt_face_t*> f_;
  ccd_pt_t* polytope_;
};

/**
 * A tetrahedron with some specific ordering on its edges, and faces.
 * The user should notice that due to the specific order of the edges, each face
 * has its own orientations. Namely for some faces f, f.e(0).cross(f.e(1))
 * points inward to the tetrahedron, for some other faces it points outward.
 */
class Tetrahedron : public Polytope {
 public:
  Tetrahedron(const std::array<fcl::Vector3<ccd_real_t>, 4>& vertices)
      : Polytope() {
    v().resize(4);
    e().resize(6);
    f().resize(4);
    for (int i = 0; i < 4; ++i) {
      v()[i] = ccdPtAddVertexCoords(&this->polytope(), vertices[i](0),
                                    vertices[i](1), vertices[i](2));
    }
    e()[0] = ccdPtAddEdge(&polytope(), &v(0), &v(1));
    e()[1] = ccdPtAddEdge(&polytope(), &v(1), &v(2));
    e()[2] = ccdPtAddEdge(&polytope(), &v(2), &v(0));
    e()[3] = ccdPtAddEdge(&polytope(), &v(0), &v(3));
    e()[4] = ccdPtAddEdge(&polytope(), &v(1), &v(3));
    e()[5] = ccdPtAddEdge(&polytope(), &v(2), &v(3));
    f()[0] = ccdPtAddFace(&polytope(), &e(0), &e(1), &e(2));
    f()[1] = ccdPtAddFace(&polytope(), &e(0), &e(3), &e(4));
    f()[2] = ccdPtAddFace(&polytope(), &e(1), &e(4), &e(5));
    f()[3] = ccdPtAddFace(&polytope(), &e(3), &e(5), &e(2));
  }
};

std::array<fcl::Vector3<ccd_real_t>, 4> EquilateralTetrahedronVertices(
    ccd_real_t bottom_center_x, ccd_real_t bottom_center_y,
    ccd_real_t bottom_center_z, ccd_real_t edge_length) {
  std::array<fcl::Vector3<ccd_real_t>, 4> vertices;
  auto compute_vertex = [bottom_center_x, bottom_center_y, bottom_center_z,
                         edge_length](ccd_real_t x, ccd_real_t y, ccd_real_t z,
                                      fcl::Vector3<ccd_real_t>* vertex) {
    *vertex << x * edge_length + bottom_center_x,
        y * edge_length + bottom_center_y, z * edge_length + bottom_center_z;
  };
  compute_vertex(0.5, -0.5 / std::sqrt(3), 0, &vertices[0]);
  compute_vertex(-0.5, -0.5 / std::sqrt(3), 0, &vertices[1]);
  compute_vertex(0, 1 / std::sqrt(3), 0, &vertices[2]);
  compute_vertex(0, 0, std::sqrt(2.0 / 3.0), &vertices[3]);
  return vertices;
}

// Produces a corrupted equilateral tetrahedron, but moves the top vertex to be
// on one of the bottom face's edges.
std::array<Vector3<ccd_real_t>, 4> TetrahedronColinearVertices() {
  std::array<Vector3<ccd_real_t>, 4> vertices = EquilateralTetrahedronVertices(
      0, 0, 0, 1);
  vertices[3] = (vertices[0] + vertices[1]) / 2;
  return vertices;
}

// Produces a corrupted equilateral tetrahedron, but the bottom face is shrunk
// down too small to be trusted.
std::array<Vector3<ccd_real_t>, 4> TetrahedronSmallFaceVertices() {
  std::array<Vector3<ccd_real_t>, 4> vertices = EquilateralTetrahedronVertices(
      0, 0, 0, 1);
  const ccd_real_t delta = constants<ccd_real_t>::eps() / 4;
  for (int i = 0; i < 3; ++i) vertices[i] *= delta;
  return vertices;
}

/**
  Simple equilateral tetrahedron.

Geometrically, its edge lengths are the given length (default to unit length).
Its "bottom" face is parallel with the z = 0 plane. It's default configuration
places the bottom face *on* the z = 0 plane with the origin contained in the
bottom face.

In representation, the edge ordering is arbitrary (i.e., an edge can be defined
as (vᵢ, vⱼ) or (vⱼ, vᵢ). However, given an arbitrary definition of edges, the
*faces* have been defined to have a specific winding which causes e₀ × e₁ to
point inwards or outwards for that face. This allows us to explicitly fully
exercise the functionality for computing an outward normal.
  - face 0: points outward
  - face 1: points inward (requires flipping)
  - face 2: points inward (requires flipping)
  - face 3: points outward

All property accessors are *mutable*.
*/
class EquilateralTetrahedron : public Tetrahedron {
 public:
  EquilateralTetrahedron(ccd_real_t bottom_center_x = 0,
                         ccd_real_t bottom_center_y = 0,
                         ccd_real_t bottom_center_z = 0,
                         ccd_real_t edge_length = 1)
      : Tetrahedron(EquilateralTetrahedronVertices(
            bottom_center_x, bottom_center_y, bottom_center_z, edge_length)) {}
};

void CheckTetrahedronFaceNormal(const Tetrahedron& p) {
  for (int i = 0; i < 4; ++i) {
    const ccd_vec3_t n =
        libccd_extension::faceNormalPointingOutward(&p.polytope(), &p.f(i));
    for (int j = 0; j < 4; ++j) {
      EXPECT_LE(ccdVec3Dot(&n, &p.v(j).v.v),
                ccdVec3Dot(&n, &p.f(i).edge[0]->vertex[0]->v.v) +
                    constants<ccd_real_t>::eps_34());
    }
  }
}

GTEST_TEST(FCL_GJK_EPA, faceNormalPointingOutward) {
  // Construct a equilateral tetrahedron, compute the normal on each face.
  /*
   p1-p4: The tetrahedron is positioned so that the origin is placed on each
   face (some requiring flipping, some not)
   p5: Origin is well within
   p6: Origin on the bottom face, but the tetrahedron is too small; it must
   evaluate all vertices and do a min/max comparison.
   p7: Small tetrahedron with origin properly inside.
   p8: Origin on the side face.
   We do not test the case that the origin is on a vertex of the polytope. When
   the origin coincides with a vertex, the two objects are touching, and we do
   not need to call faceNormalPointOutward function to compute the direction
   along which the polytope is expanded.
  */
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
  CheckTetrahedronFaceNormal(p4);

  // Check when the origin is within the polytope.
  EquilateralTetrahedron p5(0, 0, -0.1);
  CheckTetrahedronFaceNormal(p5);

  // Small tetrahedrons.
  EquilateralTetrahedron p6(0, 0, 0, 0.01);
  CheckTetrahedronFaceNormal(p6);
  EquilateralTetrahedron p7(0, 0, -0.002, 0.01);
  CheckTetrahedronFaceNormal(p7);
  EquilateralTetrahedron p8(0, 0.01 / (3 * std::sqrt(3)),
                            -0.01 * std::sqrt(6) / 9, 0.01);
  CheckTetrahedronFaceNormal(p8);
}

GTEST_TEST(FCL_GJK_EPA, faceNormalPointingOutwardOriginNearFace1) {
  // Creates a downward pointing tetrahedron which contains the origin. The
  // origin is just below the "top" face of this tetrahedron. The remaining
  // vertex is far enough away from the top face that it is considered a
  // reliable witness to determine the direction of the face's normal. The top
  // face is not quite parallel with the z = 0 plane. This test captures the
  // failure condition reported in PR 334 -- a logic error made it so the
  // reliable witness could be ignored.
  const double face0_origin_distance = 0.005;
  std::array<fcl::Vector3<ccd_real_t>, 4> vertices;
  vertices[0] << 0.5, -0.5, face0_origin_distance;
  vertices[1] << 0, 1, face0_origin_distance;
  vertices[2] << -0.5, -0.5, face0_origin_distance;
  vertices[3] << 0, 0, -1;
  const double kPi = constants<double>::pi();
  Eigen::AngleAxis<ccd_real_t> rotation(0.05 * kPi,
                                        Vector3<ccd_real_t>::UnitX());
  for (int i = 0; i < 4; ++i) {
    vertices[i] = rotation * vertices[i];
  }
  Tetrahedron p(vertices);
  {
    // Make sure that the e₀ × e₁ points upward.
    ccd_vec3_t f0_e0, f0_e1;
    ccdVec3Sub2(&f0_e0, &(p.f(0).edge[0]->vertex[1]->v.v),
                &(p.f(0).edge[0]->vertex[0]->v.v));
    ccdVec3Sub2(&f0_e1, &(p.f(0).edge[1]->vertex[1]->v.v),
                &(p.f(0).edge[1]->vertex[0]->v.v));
    ccd_vec3_t f0_e0_cross_e1;
    ccdVec3Cross(&f0_e0_cross_e1, &f0_e0, &f0_e1);
    EXPECT_GE(f0_e0_cross_e1.v[2], 0);
  }
  CheckTetrahedronFaceNormal(p);
}

GTEST_TEST(FCL_GJK_EPA, faceNormalPointingOutwardOriginNearFace2) {
  // Similar to faceNormalPointingOutwardOriginNearFace1 with an important
  // difference: the fourth vertex is no longer a reliable witness; it lies
  // within the distance tolerance. However, it is unambiguously farther off the
  // plane of the top face than those that form the face. This confirms that
  // when there are no obviously reliable witness that the most distant point
  // serves.
  const double face0_origin_distance = 0.005;
  std::array<fcl::Vector3<ccd_real_t>, 4> vertices;
  vertices[0] << 0.5, -0.5, face0_origin_distance;
  vertices[1] << 0, 1, face0_origin_distance;
  vertices[2] << -0.5, -0.5, face0_origin_distance;
  vertices[3] << 0, 0, -0.001;

  Tetrahedron p(vertices);
  CheckTetrahedronFaceNormal(p);
}

// Tests the error condition for this operation -- i.e., a degenerate triangle
// in a polytope.
GTEST_TEST(FCL_GJK_EPA, faceNormalPointingOutwardError) {
  {
    Tetrahedron bad_tet(TetrahedronColinearVertices());

    // Degenerate triangle (in this case, co-linear vertices) in polytope.
    // By construction, face 1 is the triangle that has been made degenerate.
    FCL_EXPECT_THROWS_MESSAGE_IF_DEBUG(
        libccd_extension::faceNormalPointingOutward(&bad_tet.polytope(),
                                                    &bad_tet.f(1)),
        FailedAtThisConfiguration,
        ".*faceNormalPointingOutward.*zero-area.*");
  }

  {
    Tetrahedron bad_tet(TetrahedronSmallFaceVertices());

    // Degenerate triangle (in this case, a face is too small) in polytope.
    // By construction, face 1 is the triangle that has been made degenerate.
    FCL_EXPECT_THROWS_MESSAGE_IF_DEBUG(
        libccd_extension::faceNormalPointingOutward(&bad_tet.polytope(),
                                                    &bad_tet.f(1)),
        FailedAtThisConfiguration,
        ".*faceNormalPointingOutward.*zero-area.*");
  }
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
  // TODO(hongkai.dai@tri.global): this tolerance should be cranked up once
  // #291 is resolved.
  const ccd_real_t tol = 3E-5;
  CheckSupportEPADirection(&p1.polytope(),
                           reinterpret_cast<const ccd_pt_el_t*>(&p1.f(0)),
                           Vector3<ccd_real_t>(0, 0, -1), tol);
  // Nearest point is on an edge, as the origin is on an edge.
  EquilateralTetrahedron p2(0, 0.5 / std::sqrt(3), 0);
  // e(0) has two neighbouring faces, f(0) and f(1). The support direction could
  // be the normal direction of either face.
  if (p2.e(0).faces[0] == &p2.f(0)) {
    // Check the support direction, should be the normal direction of f(0).
    CheckSupportEPADirection(&p2.polytope(),
                             reinterpret_cast<const ccd_pt_el_t*>(&p2.e(0)),
                             Vector3<ccd_real_t>(0, 0, -1), tol);
  } else {
    // The support direction should be the normal direction of f(1)
    CheckSupportEPADirection(
        &p2.polytope(), reinterpret_cast<const ccd_pt_el_t*>(&p2.e(0)),
        Vector3<ccd_real_t>(0, -2 * std::sqrt(2) / 3, 1.0 / 3), tol);
  }
  // Nearest point is on a vertex, should throw an error.
  EquilateralTetrahedron p3(-0.5, 0.5 / std::sqrt(3), 0);
  EXPECT_THROW(
      libccd_extension::supportEPADirection(
          &p3.polytope(), reinterpret_cast<const ccd_pt_el_t*>(&p3.v(0))),
      FailedAtThisConfiguration);

  // Origin is an internal point of the bottom triangle
  EquilateralTetrahedron p4(0, 0, 0);
  CheckSupportEPADirection(&p4.polytope(),
                           reinterpret_cast<const ccd_pt_el_t*>(&p4.f(0)),
                           Vector3<ccd_real_t>(0, 0, -1), tol);

  // Nearest point is on face(1)
  EquilateralTetrahedron p5(0, 1 / (3 * std::sqrt(3)),
                            -std::sqrt(6) / 9 + 0.01);
  CheckSupportEPADirection(
      &p5.polytope(), reinterpret_cast<const ccd_pt_el_t*>(&p5.f(1)),
      Vector3<ccd_real_t>(0, -2 * std::sqrt(2) / 3, 1.0 / 3), tol);
}

GTEST_TEST(FCL_GJK_EPA, supportEPADirectionError) {
  EquilateralTetrahedron tet;
  // Note: the exception is only thrown if the nearest feature's disatance is
  // zero.
  tet.v(1).dist = 0;
  FCL_EXPECT_THROWS_MESSAGE(
      libccd_extension::supportEPADirection(
          &tet.polytope(), reinterpret_cast<ccd_pt_el_t*>(&tet.v(1))),
      FailedAtThisConfiguration,
      ".+supportEPADirection.+nearest point to the origin is a vertex.+");
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
    EXPECT_EQ(libccd_extension::isOutsidePolytopeFace(&p.polytope(),
                                                      &p.f(face_index), &pt),
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

// Tests against a degenerate polytope.
GTEST_TEST(FCL_GJK_EPA, isOutsidePolytopeFaceError) {
  // The test point doesn't matter; it'll never get that far.
  // NOTE: For platform compatibility, the assertion message is pared down to
  // the simplest component: the actual function call in the assertion.
  ccd_vec3_t pt{{10, 10, 10}};

  {
    Tetrahedron bad_tet(TetrahedronColinearVertices());

    // Degenerate triangle (in this case, co-linear vertices) in polytope.
    // By construction, face 1 is the triangle that has been made degenerate.
    FCL_EXPECT_THROWS_MESSAGE_IF_DEBUG(
        libccd_extension::isOutsidePolytopeFace(&bad_tet.polytope(),
                                                &bad_tet.f(1), &pt),
        FailedAtThisConfiguration,
        ".*faceNormalPointingOutward.*zero-area.*");
  }

  {
    Tetrahedron bad_tet(TetrahedronSmallFaceVertices());

    // Degenerate triangle (in this case, a face is too small) in polytope.
    // By construction, face 1 is the triangle that has been made degenerate.
    FCL_EXPECT_THROWS_MESSAGE_IF_DEBUG(
        libccd_extension::isOutsidePolytopeFace(&bad_tet.polytope(),
                                                &bad_tet.f(0), &pt),
        FailedAtThisConfiguration,
        ".*faceNormalPointingOutward.*zero-area.*");
  }
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
// For each face, the edge e(0).cross(e(1)) has the following direction
// f(0) points inward.
// f(1) points outward.
// f(2) points inward.
// f(3) points outward.
// f(4) points inward.
// f(5) points outward.
// f(6) points inward
// f(7) points outward.
class Hexagram : public Polytope {
 public:
  Hexagram(ccd_real_t bottom_center_x = 0, ccd_real_t bottom_center_y = 0,
           ccd_real_t bottom_center_z = 0)
      : Polytope() {
    v().resize(6);
    e().resize(12);
    f().resize(8);
    auto AddHexagramVertex = [bottom_center_x, bottom_center_y, bottom_center_z,
                              this](ccd_real_t x, ccd_real_t y, ccd_real_t z) {
      return ccdPtAddVertexCoords(&this->polytope(), x + bottom_center_x,
                                  y + bottom_center_y, z + bottom_center_z);
    };
    // right corner of upper triangle
    v()[0] = AddHexagramVertex(0.5, -1 / std::sqrt(3), 1);
    // bottom corner of lower triangle
    v()[1] = AddHexagramVertex(0, -2 / std::sqrt(3), 0);
    // left corner of upper triangle
    v()[2] = AddHexagramVertex(-0.5, -1 / std::sqrt(3), 1);
    // left corner of lower triangle
    v()[3] = AddHexagramVertex(-0.5, 1 / std::sqrt(3), 0);
    // top corner of upper triangle
    v()[4] = AddHexagramVertex(0, 2 / std::sqrt(3), 1);
    // right corner of lower triangle
    v()[5] = AddHexagramVertex(0.5, 1 / std::sqrt(3), 0);

    // edges on the upper triangle
    e()[0] = ccdPtAddEdge(&polytope(), &v(0), &v(2));
    e()[1] = ccdPtAddEdge(&polytope(), &v(2), &v(4));
    e()[2] = ccdPtAddEdge(&polytope(), &v(4), &v(0));
    // edges on the lower triangle
    e()[3] = ccdPtAddEdge(&polytope(), &v(1), &v(3));
    e()[4] = ccdPtAddEdge(&polytope(), &v(3), &v(5));
    e()[5] = ccdPtAddEdge(&polytope(), &v(5), &v(1));
    // edges connecting the upper triangle to the lower triangle
    for (int i = 0; i < 6; ++i) {
      e()[6 + i] = ccdPtAddEdge(&polytope(), &v(i), &v((i + 1) % 6));
    }

    // upper triangle
    f()[0] = ccdPtAddFace(&polytope(), &e(0), &e(1), &e(2));
    // lower triangle
    f()[1] = ccdPtAddFace(&polytope(), &e(3), &e(4), &e(5));
    // triangles on the side
    f()[2] = ccdPtAddFace(&polytope(), &e(0), &e(7), &e(6));
    f()[3] = ccdPtAddFace(&polytope(), &e(7), &e(8), &e(3));
    f()[4] = ccdPtAddFace(&polytope(), &e(8), &e(9), &e(1));
    f()[5] = ccdPtAddFace(&polytope(), &e(9), &e(10), &e(4));
    f()[6] = ccdPtAddFace(&polytope(), &e(10), &e(11), &e(2));
    f()[7] = ccdPtAddFace(&polytope(), &e(11), &e(6), &e(5));
  }
};

template <typename T>
bool IsElementInSet(const std::unordered_set<T*>& S, const T* element) {
  return S.count(const_cast<T*>(element)) > 0;
}

// @param border_edge_indices_expected
// polytope.e(border_edge_indices_expected(i)) is a border edge. Similarly for
// visible_face_indices_expected and internal_edges_indices_expected.
void CheckComputeVisiblePatchCommon(
    const Polytope& polytope,
    const std::unordered_set<ccd_pt_edge_t*>& border_edges,
    const std::unordered_set<ccd_pt_face_t*>& visible_faces,
    const std::unordered_set<ccd_pt_edge_t*>& internal_edges,
    const std::unordered_set<int>& border_edge_indices_expected,
    const std::unordered_set<int>& visible_face_indices_expected,
    const std::unordered_set<int> internal_edges_indices_expected) {
  // Check border_edges
  EXPECT_EQ(border_edges.size(), border_edge_indices_expected.size());
  for (const int edge_index : border_edge_indices_expected) {
    EXPECT_TRUE(IsElementInSet(border_edges, &polytope.e(edge_index)));
  }
  // Check visible_faces
  EXPECT_EQ(visible_faces.size(), visible_face_indices_expected.size());
  for (const int face_index : visible_face_indices_expected) {
    EXPECT_TRUE(IsElementInSet(visible_faces, &polytope.f(face_index)));
  }
  // Check internal_edges
  EXPECT_EQ(internal_edges.size(), internal_edges_indices_expected.size());
  for (const auto edge_index : internal_edges_indices_expected) {
    EXPECT_TRUE(IsElementInSet(internal_edges, &polytope.e(edge_index)));
  }
}

// @param edge_indices we will call ComputeVisiblePatchRecursive(polytope, face,
// edge_index, new_vertex, ...) for each edge_index in edge_indices. Namely we
// will compute the visible patches, starting from face.e(edge_index).
void CheckComputeVisiblePatchRecursive(
    const Polytope& polytope, ccd_pt_face_t& face,
    const std::vector<int>& edge_indices, const ccd_vec3_t& new_vertex,
    const std::unordered_set<int>& border_edge_indices_expected,
    const std::unordered_set<int>& visible_face_indices_expected,
    const std::unordered_set<int>& internal_edges_indices_expected) {
  std::unordered_set<ccd_pt_edge_t*> border_edges;
  std::unordered_set<ccd_pt_face_t*> visible_faces;
  std::unordered_set<ccd_pt_face_t*> hidden_faces;
  visible_faces.insert(&face);
  std::unordered_set<ccd_pt_edge_t*> internal_edges;
  for (const int edge_index : edge_indices) {
    libccd_extension::ComputeVisiblePatchRecursive(
        polytope.polytope(), face, edge_index, new_vertex, &border_edges,
        &visible_faces, &hidden_faces, &internal_edges);
  }
  CheckComputeVisiblePatchCommon(polytope, border_edges, visible_faces,
                                 internal_edges, border_edge_indices_expected,
                                 visible_face_indices_expected,
                                 internal_edges_indices_expected);

  // Confirm that visible and hidden faces are disjoint sets.
  for (const auto hidden_face : hidden_faces) {
    EXPECT_EQ(visible_faces.count(hidden_face), 0u);
  }
}

void CheckComputeVisiblePatch(
    const Polytope& polytope, ccd_pt_face_t& face, const ccd_vec3_t& new_vertex,
    const std::unordered_set<int>& border_edge_indices_expected,
    const std::unordered_set<int>& visible_face_indices_expected,
    const std::unordered_set<int>& internal_edges_indices_expected) {
  std::unordered_set<ccd_pt_edge_t*> border_edges;
  std::unordered_set<ccd_pt_face_t*> visible_faces;
  std::unordered_set<ccd_pt_edge_t*> internal_edges;
  libccd_extension::ComputeVisiblePatch(polytope.polytope(), face, new_vertex,
                                        &border_edges, &visible_faces,
                                        &internal_edges);

  CheckComputeVisiblePatchCommon(polytope, border_edges, visible_faces,
                                 internal_edges, border_edge_indices_expected,
                                 visible_face_indices_expected,
                                 internal_edges_indices_expected);
}

GTEST_TEST(FCL_GJK_EPA, ComputeVisiblePatch_TopFaceVisible) {
  // 1 visible face.
  Hexagram hex;
  // Point P is just slightly above the top triangle. Only the top triangle can
  // be seen from point P.
  ccd_vec3_t p;
  p.v[0] = 0;
  p.v[1] = 0;
  p.v[2] = 1.1;
  const std::unordered_set<int> empty_set;
  // Test recursive implementation.
  CheckComputeVisiblePatchRecursive(hex, hex.f(0), {0}, p, {0}, {0}, empty_set);

  // Test ComputeVisiblePatch.
  CheckComputeVisiblePatch(hex, hex.f(0), p, {0, 1, 2}, {0}, empty_set);
}

GTEST_TEST(FCL_GJK_EPA, ComputeVisiblePatch_4FacesVisible) {
  // 4 visible faces.
  Hexagram hex;
  // Point P is just above the top triangle by a certain height, such that it
  // can see the triangles on the side, which connects two vertices on the upper
  // triangle, and one vertex on the lower triangle.
  ccd_vec3_t p;
  p.v[0] = 0;
  p.v[1] = 0;
  p.v[2] = 2.1;
  // Test recursive implementation.
  CheckComputeVisiblePatchRecursive(hex, hex.f(0), {0}, p, {6, 7}, {0, 2}, {0});

  // Test ComputeVisiblePatch.
  CheckComputeVisiblePatch(hex, hex.f(0), p, {6, 7, 8, 9, 10, 11}, {0, 2, 4, 6},
                           {0, 1, 2});
}

GTEST_TEST(FCL_GJK_EPA, ComputeVisiblePatch_TopAndSideFacesVisible) {
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

GTEST_TEST(FCL_GJK_EPA, ComputeVisiblePatch_2FacesVisible) {
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

/*
 * Given an equilateral tetrahedron, create a query point that is co-linear with
 * edge 0 as q = v₀ + ρ(v₀ - v₁), confirms that the correct tetrahedra faces are
 * included in the visible patch. Point q is co-linear with edge 0 which is
 * adjacent to faces f0 and f1. Face f3 is trivially visible from q.
 *
 * If the query point is co-linear with a tet edge, then both adjacent faces
 * should be visible. The behavior is sensitive to numerical accuracy issues and
 * we expose rho (ρ) as a parameter so that different scenarios can easily be
 * authored which exercise different code paths to determine visibility. (In the
 * code, "visibility" may be determined by multiple tests.)
 */
void CheckComputeVisiblePatchColinearNewVertex(EquilateralTetrahedron& tet,
                                               double rho) {
  // A new vertex is colinear with an edge e[0] of the tetrahedron. The border
  // edges should be e[1], e[4], e[5]. The visible faces should be f[0], f[1],
  // f[3], and the internal edges should be e[0], e[2], e[3].
  // For the numbering of the edges/vertices/faces in the equilateral
  // tetrahedron, please refer to the documentation of EquilateralTetrahedron.
  ccd_vec3_t query_point;
  for (int i = 0; i < 3; ++i) {
    query_point.v[i] = (1 + rho) * tet.v(0).v.v.v[i] - rho * tet.v(1).v.v.v[i];
  }
  std::unordered_set<ccd_pt_edge_t*> border_edges;
  std::unordered_set<ccd_pt_face_t*> visible_faces;
  std::unordered_set<ccd_pt_edge_t*> internal_edges;
  libccd_extension::ComputeVisiblePatch(tet.polytope(), tet.f(3), query_point,
                                        &border_edges, &visible_faces,
                                        &internal_edges);

  EXPECT_EQ(border_edges.size(), 3u);
  EXPECT_EQ(border_edges, std::unordered_set<ccd_pt_edge_t*>(
                              {&(tet.e(1)), &(tet.e(4)), &(tet.e(5))}));
  EXPECT_EQ(visible_faces.size(), 3u);
  EXPECT_EQ(visible_faces, std::unordered_set<ccd_pt_face_t*>(
                               {&(tet.f(0)), &(tet.f(1)), &(tet.f(3))}));
  EXPECT_EQ(internal_edges.size(), 3u);
  EXPECT_EQ(internal_edges, std::unordered_set<ccd_pt_edge_t*>(
                                {&(tet.e(0)), &(tet.e(2)), &(tet.e(3))}));
}

GTEST_TEST(FCL_GJK_EPA, ComputeVisiblePatchColinearNewVertex) {
  // Case 1: Visibility of faces f0 and f1 is not immediately apparent --
  // requires recognition that q, v0, and v1 are colinear.
  EquilateralTetrahedron tet1(-0.05, -0.13, 0.12);
  CheckComputeVisiblePatchColinearNewVertex(tet1, 1.9);
  // Case 2: Visibility of faces f0 and f1 are independently confirmed --
  // colinearity doesn't matter.
  EquilateralTetrahedron tet2(0.1, 0.2, 0.3);
  CheckComputeVisiblePatchColinearNewVertex(tet2, 0.3);
}

// Tests that the sanity check causes `ComputeVisiblePatch()` to throw in
// debug builds.
GTEST_TEST(FCL_GJK_EPA, ComputeVisiblePatchSanityCheck) {
#ifndef NDEBUG
  // NOTE: The sanity check function only gets compiled in debug mode.
  EquilateralTetrahedron tet;
  std::unordered_set<ccd_pt_edge_t*> border_edges;
  std::unordered_set<ccd_pt_face_t*> visible_faces;
  std::unordered_set<ccd_pt_edge_t*> internal_edges;

  //   Top view labels of vertices, edges and faces.
  //                                                                    .
  //          v2                                                        .
  //          /\                   /\                                   .
  //         / |\                 / |\                                  .
  //        /e5| \               /  | \                                 .
  //    e2 /   |  \             /   |f2\                                .
  //      /   ╱╲v3 \e1         /f3 ╱╲   \  // f0 is the bottom face.    .
  //     /  ╱    ╲  \         /  ╱    ╲  \                              .
  //    / ╱e3    e4╲ \       / ╱   f1   ╲ \                             .
  //   /╱____________╲\     /╱____________╲\                            .
  //  v0      e0       v1                                               .
  //
  // The tet is centered on the origin, pointing upwards (seen from above in
  // the diagram above. We define a point above this to define the visible patch
  // The *correct* patch should have the following:
  //  visible faces: f1, f2, f3
  //  internal edges: e3, e4, e5
  //  border edges: e0, e1, e2
  auto set_ideal = [&border_edges, &internal_edges, &visible_faces, &tet]() {
    border_edges =
        std::unordered_set<ccd_pt_edge_t*>{&tet.e(0), &tet.e(1), &tet.e(2)};
    internal_edges =
        std::unordered_set<ccd_pt_edge_t*>{&tet.e(3), &tet.e(4), &tet.e(5)};
    visible_faces =
        std::unordered_set<ccd_pt_face_t*>{&tet.f(1), &tet.f(2), &tet.f(3)};
  };

  set_ideal();
  EXPECT_TRUE(libccd_extension::ComputeVisiblePatchRecursiveSanityCheck(
      tet.polytope(), border_edges, visible_faces, internal_edges));

  // Failure conditions:
  //  Two adjacent faces have edge e --> e is internal edge.
  set_ideal();
  internal_edges.erase(&tet.e(5));
  EXPECT_FALSE(libccd_extension::ComputeVisiblePatchRecursiveSanityCheck(
      tet.polytope(), border_edges, visible_faces, internal_edges));

  //  Edge in internal edge --> two adjacent faces visible.
  set_ideal();
  visible_faces.erase(&tet.f(3));
  EXPECT_FALSE(libccd_extension::ComputeVisiblePatchRecursiveSanityCheck(
      tet.polytope(), border_edges, visible_faces, internal_edges));

  //  Edge in border_edges --> one (and only one) visible face.
  set_ideal();
  internal_edges.erase(&tet.e(5));
  border_edges.insert(&tet.e(5));
  EXPECT_FALSE(libccd_extension::ComputeVisiblePatchRecursiveSanityCheck(
      tet.polytope(), border_edges, visible_faces, internal_edges));

#endif  // NDEBUG
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
// @note The features in feature1_list should be unique, so are in
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
        if (!found_match) {
          map_feature1_to_feature2->emplace_hint(
              map_feature1_to_feature2->end(), f1, f2);
          found_match = true;
        } else {
          GTEST_FAIL() << "There should be only one element in feature2_list "
                          "that matches with an element in feature1_list.";
        }
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

GTEST_TEST(FCL_GJK_EPA, expandPolytope_tetrahedron1) {
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
      &polytope.polytope(), reinterpret_cast<ccd_pt_el_t*>(&polytope.f(0)),
      &newv);
  EXPECT_EQ(result, 0);

  // Construct the expanded polytope manually.
  EquilateralTetrahedron tetrahedron(0, 0, -0.1);
  ccd_pt_t& polytope_expected = tetrahedron.polytope();
  // The bottom face is removed.
  ccdPtDelFace(&polytope_expected, &tetrahedron.f(0));
  // Insert the vertex.
  ccd_pt_vertex_t* new_vertex =
      ccdPtAddVertexCoords(&polytope_expected, 0, 0, -0.2);
  // Add new edges.
  ccd_pt_edge_t* new_edges[3];
  for (int i = 0; i < 3; ++i) {
    new_edges[i] =
        ccdPtAddEdge(&polytope_expected, new_vertex, &tetrahedron.v(i));
  }
  // Add new faces.
  ccdPtAddFace(&polytope_expected, &tetrahedron.e(0), new_edges[0],
               new_edges[1]);
  ccdPtAddFace(&polytope_expected, &tetrahedron.e(1), new_edges[1],
               new_edges[2]);
  ccdPtAddFace(&polytope_expected, &tetrahedron.e(2), new_edges[2],
               new_edges[0]);

  ComparePolytope(&polytope.polytope(), &polytope_expected,
                  constants<ccd_real_t>::eps_34());
}

GTEST_TEST(FCL_GJK_EPA, expandPolytope_tetrahedron_2visible_faces) {
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
      &polytope.polytope(), reinterpret_cast<ccd_pt_el_t*>(&polytope.e(0)),
      &newv);
  EXPECT_EQ(result, 0);

  // Construct the expanded polytope manually.
  EquilateralTetrahedron tetrahedron(0, 0, -0.1);
  ccd_pt_t& polytope_expected = tetrahedron.polytope();
  // The bottom face is removed.
  ccdPtDelFace(&polytope_expected, &tetrahedron.f(0));
  // The other face that neighbours with f(0) is removed.
  ccdPtDelFace(&polytope_expected, &tetrahedron.f(1));
  // The nearest edge is removed.
  ccdPtDelEdge(&polytope_expected, &tetrahedron.e(0));
  // Insert the vertex.
  ccd_pt_vertex_t* new_vertex = ccdPtAddVertexCoords(
      &polytope_expected, newv.v.v[0], newv.v.v[1], newv.v.v[2]);
  // Add new edges.
  ccd_pt_edge_t* new_edges[4];
  new_edges[0] =
      ccdPtAddEdge(&polytope_expected, new_vertex, &tetrahedron.v(0));
  new_edges[1] =
      ccdPtAddEdge(&polytope_expected, new_vertex, &tetrahedron.v(1));
  new_edges[2] =
      ccdPtAddEdge(&polytope_expected, new_vertex, &tetrahedron.v(2));
  new_edges[3] =
      ccdPtAddEdge(&polytope_expected, new_vertex, &tetrahedron.v(3));
  // Add new faces.
  ccdPtAddFace(&polytope_expected, &tetrahedron.e(3), new_edges[0],
               new_edges[3]);
  ccdPtAddFace(&polytope_expected, &tetrahedron.e(2), new_edges[0],
               new_edges[2]);
  ccdPtAddFace(&polytope_expected, &tetrahedron.e(4), new_edges[1],
               new_edges[3]);
  ccdPtAddFace(&polytope_expected, &tetrahedron.e(1), new_edges[1],
               new_edges[2]);

  ComparePolytope(&polytope.polytope(), &polytope_expected,
                  constants<ccd_real_t>::eps_34());
}

GTEST_TEST(FCL_GJK_EPA, expandPolytope_hexagram_1visible_face) {
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
      &hex.polytope(), reinterpret_cast<ccd_pt_el_t*>(&hex.f(0)), &newv);
  EXPECT_EQ(result, 0);

  // Construct the expanded polytope manually.
  Hexagram hex_duplicate(0, 0, -0.9);
  ccd_pt_t& polytope_expected = hex_duplicate.polytope();
  // Remove the upper triangle.
  ccdPtDelFace(&polytope_expected, &hex_duplicate.f(0));

  // Add the new vertex.
  ccd_pt_vertex_t* new_vertex = ccdPtAddVertexCoords(
      &polytope_expected, newv.v.v[0], newv.v.v[1], newv.v.v[2]);
  // Add the new edges.
  ccd_pt_edge_t* new_edges[3];
  new_edges[0] =
      ccdPtAddEdge(&polytope_expected, new_vertex, &hex_duplicate.v(0));
  new_edges[1] =
      ccdPtAddEdge(&polytope_expected, new_vertex, &hex_duplicate.v(2));
  new_edges[2] =
      ccdPtAddEdge(&polytope_expected, new_vertex, &hex_duplicate.v(4));
  // Add the new faces.
  ccdPtAddFace(&polytope_expected, new_edges[0], new_edges[1],
               &hex_duplicate.e(0));
  ccdPtAddFace(&polytope_expected, new_edges[1], new_edges[2],
               &hex_duplicate.e(1));
  ccdPtAddFace(&polytope_expected, new_edges[2], new_edges[0],
               &hex_duplicate.e(2));

  ComparePolytope(&hex.polytope(), &polytope_expected,
                  constants<ccd_real_t>::eps_34());
}

GTEST_TEST(FCL_GJK_EPA, expandPolytope_hexagram_4_visible_faces) {
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
      &hex.polytope(), reinterpret_cast<ccd_pt_el_t*>(&hex.f(0)), &newv);
  EXPECT_EQ(result, 0);

  // Construct the expanded polytope manually.
  Hexagram hex_duplicate(0, 0, -0.9);
  ccd_pt_t& polytope_expected = hex_duplicate.polytope();
  // Remove the upper triangle.
  ccdPtDelFace(&polytope_expected, &hex_duplicate.f(0));
  // Remove the triangles on the side, which consists of two vertices on the
  // upper triangle, and one vertex on the lower triangle.
  ccdPtDelFace(&polytope_expected, &hex_duplicate.f(2));
  ccdPtDelFace(&polytope_expected, &hex_duplicate.f(4));
  ccdPtDelFace(&polytope_expected, &hex_duplicate.f(6));
  // Remove the edges of the upper triangle.
  ccdPtDelEdge(&polytope_expected, &hex_duplicate.e(0));
  ccdPtDelEdge(&polytope_expected, &hex_duplicate.e(1));
  ccdPtDelEdge(&polytope_expected, &hex_duplicate.e(2));

  // Add the new vertex.
  ccd_pt_vertex_t* new_vertex = ccdPtAddVertexCoords(
      &polytope_expected, newv.v.v[0], newv.v.v[1], newv.v.v[2]);
  // Add the new edges.
  ccd_pt_edge_t* new_edges[6];
  for (int i = 0; i < 6; ++i) {
    new_edges[i] =
        ccdPtAddEdge(&polytope_expected, new_vertex, &hex_duplicate.v(i));
  }
  // Add the new faces.
  for (int i = 0; i < 6; ++i) {
    ccdPtAddFace(&polytope_expected, new_edges[i % 6], new_edges[(i + 1) % 6],
                 &hex_duplicate.e(i + 6));
  }
  ComparePolytope(&hex.polytope(), &polytope_expected,
                  constants<ccd_real_t>::eps_34());
}

GTEST_TEST(FCL_GJK_EPA, expandPolytope_error) {
  EquilateralTetrahedron tet;
  ccd_support_t newv;

  // Error condition 1: expanding with the nearest feature being a vertex.
  FCL_EXPECT_THROWS_MESSAGE(
      libccd_extension::expandPolytope(
          &tet.polytope(), reinterpret_cast<ccd_pt_el_t*>(&tet.v(0)), &newv),
      FailedAtThisConfiguration,
      ".*expandPolytope.*The visible feature is a vertex.*");

  // Error condition 2: feature is edge, and vertex lies on the edge.
  ccd_vec3_t nearest;
  ccdVec3Copy(&nearest, &tet.v(0).v.v);
  ccdVec3Add(&nearest, &tet.v(1).v.v);
  ccdVec3Scale(&nearest, 0.5);
  newv.v = nearest;
  FCL_EXPECT_THROWS_MESSAGE(
      libccd_extension::expandPolytope(
          &tet.polytope(), reinterpret_cast<ccd_pt_el_t*>(&tet.e(0)), &newv),
      FailedAtThisConfiguration,
      ".*expandPolytope.* nearest point and the new vertex are on an edge.*");
}

void CompareCcdVec3(const ccd_vec3_t& v, const ccd_vec3_t& v_expected,
                    ccd_real_t tol) {
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(v.v[i], v_expected.v[i], tol);
  }
}

GTEST_TEST(FCL_GJK_EPA, penEPAPosClosest_vertex) {
  // The nearest point is a vertex on the polytope.
  // tetrahedron.v(0) is the origin.
  EquilateralTetrahedron tetrahedron(-0.5, 0.5 / std::sqrt(3), 0);
  // Make sure that v1 - v2 = v.
  tetrahedron.v(0).v.v1.v[0] = 1;
  tetrahedron.v(0).v.v1.v[1] = 2;
  tetrahedron.v(0).v.v1.v[2] = 3;
  for (int i = 0; i < 3; ++i) {
    tetrahedron.v(0).v.v2.v[i] = tetrahedron.v(0).v.v1.v[i];
  }
  ccd_vec3_t p1, p2;
  EXPECT_EQ(
      libccd_extension::penEPAPosClosest(
          reinterpret_cast<const ccd_pt_el_t*>(&tetrahedron.v(0)), &p1, &p2),
      0);
  CompareCcdVec3(p1, tetrahedron.v(0).v.v1, constants<ccd_real_t>::eps_78());
  CompareCcdVec3(p2, tetrahedron.v(0).v.v2, constants<ccd_real_t>::eps_78());
}

GTEST_TEST(FCL_GJK_EPA, penEPAPosClosest_edge) {
  // The nearest point is on an edge of the polytope.
  // tetrahedron.e(1) contains the origin.
  EquilateralTetrahedron tetrahedron(0.25, -0.25 / std::sqrt(3), 0);
  // e(1) connects two vertices v(1) and v(2), make sure that v(1).v1 - v(1).v2
  // = v(1).v, also v(2).v1 - v(2).v2 = v(2).v
  ccdVec3Set(&tetrahedron.v(1).v.v1, 1, 2, 3);
  ccdVec3Set(&tetrahedron.v(2).v.v1, 4, 5, 6);
  for (int i = 1; i <= 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      tetrahedron.v(i).v.v2.v[j] =
          tetrahedron.v(i).v.v1.v[j] - tetrahedron.v(i).v.v.v[j];
    }
  }
  // Notice that origin = 0.5*v(1).v + 0.5*v(2).v
  // So p1 = 0.5*v(1).v1 + 0.5*v(2).v1
  //    p2 = 0.5*v(1).v2 + 0.5*v(2).v2
  ccd_vec3_t p1, p2;
  EXPECT_EQ(
      libccd_extension::penEPAPosClosest(
          reinterpret_cast<const ccd_pt_el_t*>(&tetrahedron.e(1)), &p1, &p2),
      0);
  ccd_vec3_t p1_expected, p2_expected;
  ccdVec3Copy(&p1_expected, &tetrahedron.v(1).v.v1);
  ccdVec3Add(&p1_expected, &tetrahedron.v(2).v.v1);
  ccdVec3Scale(&p1_expected, ccd_real_t(0.5));
  ccdVec3Copy(&p2_expected, &tetrahedron.v(1).v.v2);
  ccdVec3Add(&p2_expected, &tetrahedron.v(2).v.v2);
  ccdVec3Scale(&p2_expected, ccd_real_t(0.5));

  CompareCcdVec3(p1, p1_expected, constants<ccd_real_t>::eps_78());
  CompareCcdVec3(p2, p2_expected, constants<ccd_real_t>::eps_78());
}

GTEST_TEST(FCL_GJK_EPA, penEPAPosClosest_face) {
  // The nearest point is on a face of the polytope, It is the center of
  // tetrahedron.f(1).
  const Vector3<ccd_real_t> bottom_center_pos =
      Vector3<ccd_real_t>(0, 1.0 / (3 * std::sqrt(3)), -std::sqrt(6) / 9) +
      0.01 * Vector3<ccd_real_t>(0, -2 * std::sqrt(2) / 3, 1.0 / 3);
  EquilateralTetrahedron tetrahedron(bottom_center_pos(0), bottom_center_pos(1),
                                     bottom_center_pos(2));
  // Assign v(i).v1 and v(i).v2 for i = 0, 1, 3, such that
  // v(i).v = v(i).v1 - v(i).v2
  tetrahedron.v(0).v.v1.v[0] = 1;
  tetrahedron.v(0).v.v1.v[1] = 2;
  tetrahedron.v(0).v.v1.v[2] = 3;
  tetrahedron.v(1).v.v1.v[0] = 4;
  tetrahedron.v(1).v.v1.v[1] = 5;
  tetrahedron.v(1).v.v1.v[2] = 6;
  tetrahedron.v(3).v.v1.v[0] = 7;
  tetrahedron.v(3).v.v1.v[1] = 8;
  tetrahedron.v(3).v.v1.v[2] = 9;
  for (int i : {0, 1, 3}) {
    for (int j = 0; j < 3; ++j) {
      tetrahedron.v(i).v.v2.v[j] =
          tetrahedron.v(i).v.v1.v[j] - tetrahedron.v(i).v.v.v[j];
    }
  }

  ccd_vec3_t p1, p2;
  EXPECT_EQ(
      libccd_extension::penEPAPosClosest(
          reinterpret_cast<const ccd_pt_el_t*>(&tetrahedron.f(1)), &p1, &p2),
      0);

  // Notice that the nearest point = 1/3 * v(0).v + 1/3 * v(1).v + 1/3 * v(3).v
  // So p1 = 1/3 * (v(0).v1 + v(1).v1 + v(3).v1)
  //    p2 = 1/3 * (v(0).v2 + v(1).v2 + v(3).v2)
  ccd_vec3_t p1_expected, p2_expected;
  ccdVec3Copy(&p1_expected, &tetrahedron.v(0).v.v1);
  ccdVec3Add(&p1_expected, &tetrahedron.v(1).v.v1);
  ccdVec3Add(&p1_expected, &tetrahedron.v(3).v.v1);
  ccdVec3Scale(&p1_expected, ccd_real_t(1.0 / 3));
  ccdVec3Copy(&p2_expected, &tetrahedron.v(0).v.v2);
  ccdVec3Add(&p2_expected, &tetrahedron.v(1).v.v2);
  ccdVec3Add(&p2_expected, &tetrahedron.v(3).v.v2);
  ccdVec3Scale(&p2_expected, ccd_real_t(1.0 / 3));

  CompareCcdVec3(p1, p1_expected, constants<ccd_real_t>::eps_78());
  CompareCcdVec3(p2, p2_expected, constants<ccd_real_t>::eps_78());
}

// Test convert2SimplexToTetrahedron function.
// We construct a test scenario that two boxes are on the xy plane of frame F.
// The two boxes penetrate to each other, as shown in the bottom plot.
//              y
//          ┏━━━│━━━┓Box1
//         ┏┃━━┓    ┃
//      ───┃┃──┃O───┃─────x
//     box2┗┃━━┛│   ┃
//          ┗━━━│━━━┛
//
// @param[in] X_WF The pose of the frame F measured and expressed in the world
// frame W.
// @param[out] o1 Box1
// @param[out] o2 Box2
// @param[out] ccd The ccd solver info.
// @param[out] X_FB1 The pose of box 1 frame measured and expressed in frame F.
// @param[out] X_FB2 The pose of box 2 frame measured and expressed in frame F.
template <typename S>
void SetUpBoxToBox(const Transform3<S>& X_WF, void** o1, void** o2, ccd_t* ccd,
                   fcl::Transform3<S>* X_FB1, fcl::Transform3<S>* X_FB2) {
  const fcl::Vector3<S> box1_size(2, 2, 2);
  const fcl::Vector3<S> box2_size(1, 1, 2);
  // Box 1 is fixed.
  X_FB1->setIdentity();
  X_FB1->translation() << 0, 0, 1;
  X_FB2->setIdentity();
  X_FB2->translation() << -0.6, 0, 1;

  const fcl::Transform3<S> X_WB1 = X_WF * (*X_FB1);
  const fcl::Transform3<S> X_WB2 = X_WF * (*X_FB2);
  fcl::Box<S> box1(box1_size);
  fcl::Box<S> box2(box2_size);
  *o1 = GJKInitializer<S, fcl::Box<S>>::createGJKObject(box1, X_WB1);
  *o2 = GJKInitializer<S, fcl::Box<S>>::createGJKObject(box2, X_WB2);

  // Set up ccd solver.
  CCD_INIT(ccd);
  ccd->support1 = detail::GJKInitializer<S, Box<S>>::getSupportFunction();
  ccd->support2 = detail::GJKInitializer<S, Box<S>>::getSupportFunction();
  ccd->max_iterations = 1000;
  ccd->dist_tolerance = 1E-6;
}

template <typename S>
Vector3<S> ToEigenVector(const ccd_vec3_t& v) {
  return (Vector3<S>() << v.v[0], v.v[1], v.v[2]).finished();
}

template <typename S>
ccd_vec3_t ToCcdVec3(const Eigen::Ref<const Vector3<S>>& v) {
  ccd_vec3_t u;
  u.v[0] = v(0);
  u.v[1] = v(1);
  u.v[2] = v(2);
  return u;
}

template <typename S>
void TestSimplexToPolytope3InGivenFrame(const Transform3<S>& X_WF) {
  void* o1 = NULL;
  void* o2 = NULL;
  ccd_t ccd;
  fcl::Transform3<S> X_FB1, X_FB2;
  SetUpBoxToBox(X_WF, &o1, &o2, &ccd, &X_FB1, &X_FB2);

  // Construct a 2-simplex that contains the origin. The vertices of this
  // 2-simplex are on the boundary of the Minkowski difference.
  ccd_simplex_t simplex;
  ccdSimplexInit(&simplex);
  ccd_support_t pts[3];
  // We find three points Pa1, Pb1, Pc1 on box 1, and three points Pa2, Pb2, Pc2
  // on box 2, such that the 2-simplex with vertices (Pa1 - Pa2, Pb1 - Pb2,
  // Pc1 - Pc2) contains the origin.
  const Vector3<S> p_FPa1(-1, -1, 0.1);
  const Vector3<S> p_FPa2(-0.1, 0.5, 0.1);
  pts[0].v = ToCcdVec3<S>(p_FPa1 - p_FPa2);
  pts[0].v1 = ToCcdVec3<S>(p_FPa1);
  pts[0].v2 = ToCcdVec3<S>(p_FPa2);

  const Vector3<S> p_FPb1(-1, 1, 0.1);
  const Vector3<S> p_FPb2(-0.1, 0.5, 0.1);
  pts[1].v = ToCcdVec3<S>(p_FPb1 - p_FPb2);
  pts[1].v1 = ToCcdVec3<S>(p_FPb1);
  pts[1].v2 = ToCcdVec3<S>(p_FPb2);

  const Vector3<S> p_FPc1(1, 1, 0.1);
  const Vector3<S> p_FPc2(-0.1, 0.5, 0.1);
  pts[2].v = ToCcdVec3<S>(p_FPc1 - p_FPc2);
  pts[2].v1 = ToCcdVec3<S>(p_FPc1);
  pts[2].v2 = ToCcdVec3<S>(p_FPc2);
  for (int i = 0; i < 3; ++i) {
    ccdSimplexAdd(&simplex, &pts[i]);
  }
  // Make sure that the origin is on the triangle.
  const Vector3<S> a = ToEigenVector<S>(pts[0].v);
  const Vector3<S> b = ToEigenVector<S>(pts[1].v);
  const Vector3<S> c = ToEigenVector<S>(pts[2].v);
  // We first check if the origin is co-planar with vertices a, b, and c.
  // If a, b, c and origin are co-planar, then aᵀ · (b × c)) = 0
  EXPECT_NEAR(a.dot(b.cross(c)), 0, 1E-10);
  // Now check if origin is within the triangle, by checking the condition
  // (a × b)ᵀ · (b × c) ≥ 0
  // (b × c)ᵀ · (c × a) ≥ 0
  // (c × a)ᵀ · (a × b) ≥ 0
  // Namely the cross product a × b, b × c, c × a all point to the same
  // direction.
  // Note the check above is valid when either a, b, c is a zero vector, or
  // they are co-linear.
  EXPECT_GE(a.cross(b).dot(b.cross(c)), 0);
  EXPECT_GE(b.cross(c).dot(c.cross(a)), 0);
  EXPECT_GE(c.cross(a).dot(a.cross(b)), 0);

  ccd_pt_t polytope;
  ccdPtInit(&polytope);
  ccd_pt_el_t* nearest{};
  libccd_extension::convert2SimplexToTetrahedron(o1, o2, &ccd, &simplex,
                                                 &polytope, &nearest);
  // Box1 and Box2 are not touching, so nearest is set to null.
  EXPECT_EQ(nearest, nullptr);

  // Check the polytope
  // The polytope should have 4 vertices, with three of them being the vertices
  // of the 2-simplex, and another vertex that has the maximal support along
  // the normal directions of the 2-simplex.
  // We first construct the set containing the polytope vertices.
  std::unordered_set<ccd_pt_vertex_t*> polytope_vertices;
  {
    ccd_pt_vertex_t* v;
    ccdListForEachEntry(&polytope.vertices, v, ccd_pt_vertex_t, list) {
      const auto it = polytope_vertices.find(v);
      EXPECT_EQ(it, polytope_vertices.end());
      polytope_vertices.emplace_hint(it, v);
    }
  }
  EXPECT_EQ(polytope_vertices.size(), 4u);
  // We need to find out the vertex on the polytope, that is not the vertex
  // of the simplex.
  ccd_pt_vertex_t* non_simplex_vertex{nullptr};
  // A simplex vertex matches with a polytope vertex if they coincide.
  int num_matched_vertices = 0;
  for (const auto& v : polytope_vertices) {
    bool found_match = false;
    for (int i = 0; i < 3; ++i) {
      if (ccdVec3Dist2(&v->v.v, &pts[i].v) < constants<S>::eps_78()) {
        num_matched_vertices++;
        found_match = true;
        break;
      }
    }
    if (!found_match) {
      non_simplex_vertex = v;
    }
  }
  EXPECT_EQ(num_matched_vertices, 3);
  EXPECT_NE(non_simplex_vertex, nullptr);
  // Make sure that the non-simplex vertex has the maximal support along the
  // simplex normal direction.
  // Find the two normal directions of the 2-simplex.
  Vector3<S> dir1, dir2;
  Vector3<S> ab, ac;
  ab = ToEigenVector<S>(pts[1].v) - ToEigenVector<S>(pts[0].v);
  ac = ToEigenVector<S>(pts[2].v) - ToEigenVector<S>(pts[0].v);
  dir1 = ab.cross(ac);
  dir2 = -dir1;
  // Now make sure non_simplex_vertex has the largest support
  // p_B1V1 are the position of the box 1 vertices in box1 frame B1.
  // p_B2V1 are the position of the box 2 vertices in box2 frame B2.
  const Vector3<S> box1_size{2, 2, 2};
  const Vector3<S> box2_size{1, 1, 2};
  Eigen::Matrix<S, 3, 8> p_B1V1, p_B2V2;
  p_B1V1 << -1, -1, -1, -1, 1, 1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1,
      1, -1, 1, -1, 1;
  p_B2V2 << -1, -1, -1, -1, 1, 1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1,
      1, -1, 1, -1, 1;
  for (int i = 0; i < 3; ++i) {
    p_B1V1.row(i) *= box1_size[i] / 2;
    p_B2V2.row(i) *= box2_size[i] / 2;
  }

  const Eigen::Matrix<S, 3, 8> p_FV1 = X_FB1 * p_B1V1;
  const Eigen::Matrix<S, 3, 8> p_FV2 = X_FB2 * p_B2V2;
  // The support of the Minkowski difference along direction dir1.
  const S max_support1 = (dir1.transpose() * p_FV1).maxCoeff() -
                         (dir1.transpose() * p_FV2).minCoeff();
  // The support of the Minkowski difference along direction dir2.
  const S max_support2 = (dir2.transpose() * p_FV1).maxCoeff() -
                         (dir2.transpose() * p_FV2).minCoeff();

  const double expected_max_support = std::max(max_support1, max_support2);
  const double non_simplex_vertex_support1 =
      ToEigenVector<S>(non_simplex_vertex->v.v).dot(dir1);
  const double non_simplex_vertex_support2 =
      ToEigenVector<S>(non_simplex_vertex->v.v).dot(dir2);
  EXPECT_NEAR(
      std::max(non_simplex_vertex_support1, non_simplex_vertex_support2),
      expected_max_support, constants<ccd_real_t>::eps_78());

  // Also make sure the non_simplex_vertex actually is inside the Minkowski
  // difference.
  // Call the non-simplex vertex as D. This vertex equals to the difference
  // between a point Dv1 in box1, and a point Dv2 in box2.
  const Vector3<S> p_B1Dv1 =
      (X_WF * X_FB1).inverse() * ToEigenVector<S>(non_simplex_vertex->v.v1);
  const Vector3<S> p_B2Dv2 =
      (X_WF * X_FB2).inverse() * ToEigenVector<S>(non_simplex_vertex->v.v2);
  // Now check if p_B1Dv1 is in box1, and p_B2Dv2 is in box2.
  for (int i = 0; i < 3; ++i) {
    EXPECT_LE(p_B1Dv1(i), box1_size(i) / 2 + constants<ccd_real_t>::eps_78());
    EXPECT_LE(p_B2Dv2(i), box2_size(i) / 2 + constants<ccd_real_t>::eps_78());
    EXPECT_GE(p_B1Dv1(i), -box1_size(i) / 2 - constants<ccd_real_t>::eps_78());
    EXPECT_GE(p_B2Dv2(i), -box2_size(i) / 2 - constants<ccd_real_t>::eps_78());
  }
  // Now that we make sure the vertices of the polytope is correct, we will
  // check the edges and faces of the polytope. We do so by constructing an
  // expected polytope, and compare it with the polytope obtained from
  // convert2SimplexToTetrahedron().
  ccd_pt_t polytope_expected;
  ccdPtInit(&polytope_expected);

  ccd_pt_vertex_t* vertices_expected[4];
  int v_count = 0;
  for (const auto& v : polytope_vertices) {
    vertices_expected[v_count++] = ccdPtAddVertex(&polytope_expected, &v->v);
  }
  ccd_pt_edge_t* edges_expected[6];
  edges_expected[0] = ccdPtAddEdge(&polytope_expected, vertices_expected[0],
                                   vertices_expected[1]);
  edges_expected[1] = ccdPtAddEdge(&polytope_expected, vertices_expected[1],
                                   vertices_expected[2]);
  edges_expected[2] = ccdPtAddEdge(&polytope_expected, vertices_expected[2],
                                   vertices_expected[0]);
  edges_expected[3] = ccdPtAddEdge(&polytope_expected, vertices_expected[3],
                                   vertices_expected[0]);
  edges_expected[4] = ccdPtAddEdge(&polytope_expected, vertices_expected[3],
                                   vertices_expected[1]);
  edges_expected[5] = ccdPtAddEdge(&polytope_expected, vertices_expected[3],
                                   vertices_expected[2]);

  ccdPtAddFace(&polytope_expected, edges_expected[0], edges_expected[3],
               edges_expected[4]);
  ccdPtAddFace(&polytope_expected, edges_expected[1], edges_expected[4],
               edges_expected[5]);
  ccdPtAddFace(&polytope_expected, edges_expected[2], edges_expected[3],
               edges_expected[5]);
  ccdPtAddFace(&polytope_expected, edges_expected[0], edges_expected[1],
               edges_expected[2]);

  ComparePolytope(&polytope, &polytope_expected, constants<S>::eps_34());

  ccdPtDestroy(&polytope_expected);
  ccdPtDestroy(&polytope);
}

template <typename S>
void TestSimplexToPolytope3() {
  Transform3<S> X_WF;
  X_WF.setIdentity();
  TestSimplexToPolytope3InGivenFrame(X_WF);

  X_WF.translation() << 0, 0, 1;
  TestSimplexToPolytope3InGivenFrame(X_WF);

  X_WF.translation() << -0.2, 0.4, 0.1;
  TestSimplexToPolytope3InGivenFrame(X_WF);
}

GTEST_TEST(FCL_GJK_EPA, convert2SimplexToTetrahedron) {
  TestSimplexToPolytope3<double>();
  TestSimplexToPolytope3<float>();
}

}  // namespace detail
}  // namespace fcl

//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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
  EquilateralTetrahedron(ccd_real_t bottom_center_x = 0,
                         ccd_real_t bottom_center_y = 0,
                         ccd_real_t bottom_center_z = 0)
      : polytope_(new ccd_pt_t) {
    ccdPtInit(polytope_);
    auto AddTetrahedronVertex = [bottom_center_x, bottom_center_y,
                                 bottom_center_z, this](
        ccd_real_t x, ccd_real_t y, ccd_real_t z) {
      return ccdPtAddVertexCoords(this->polytope_, x + bottom_center_x,
                                  y + bottom_center_y, z + bottom_center_z);
    };
    v_[0] = AddTetrahedronVertex(0.5, -0.5 / std::sqrt(3), 0);
    v_[1] = AddTetrahedronVertex(-0.5, -0.5 / std::sqrt(3), 0);
    v_[2] = AddTetrahedronVertex(0, 1 / std::sqrt(3), 0);
    v_[3] = AddTetrahedronVertex(0, 0, std::sqrt(2.0 / 3.0));
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

// Construct a polytope with the following shape, namely an equilateral triangle
// on the top, and an equilateral triangle of the same size, but rotate by 60
// degrees on the bottom. We will then connect the vertices of the equilateral
// triangles to form a convex polytope.
//       __╱╲__
//       ╲╱  ╲╱
//       ╱____╲
//         ╲╱
class Hexagram {
 public:
  Hexagram() : polytope_(new ccd_pt_t) {
    ccdPtInit(polytope_);
    // right corner of upper triangle
    v_[0] = ccdPtAddVertexCoords(polytope_, 0.5, -1 / std::sqrt(3), 1);
    // bottom corner of lower triangle
    v_[1] = ccdPtAddVertexCoords(polytope_, 0, -2 / std::sqrt(3), 0);
    // left corner of upper triangle
    v_[2] = ccdPtAddVertexCoords(polytope_, -0.5, -1 / std::sqrt(3), 1);
    // left corner of lower triangle
    v_[3] = ccdPtAddVertexCoords(polytope_, -0.5, 1 / std::sqrt(3), 0);
    // top corner of upper triangle
    v_[4] = ccdPtAddVertexCoords(polytope_, 0, 2 / std::sqrt(3), 1);
    // right corner of lower triangle
    v_[5] = ccdPtAddVertexCoords(polytope_, 0.5, 1 / std::sqrt(3), 0);

    // edges on the upper triangle
    e_[0] = ccdPtAddEdge(polytope_, v_[0], v_[2]);
    e_[1] = ccdPtAddEdge(polytope_, v_[2], v_[4]);
    e_[2] = ccdPtAddEdge(polytope_, v_[4], v_[0]);
    // edges on the lower triangle
    e_[3] = ccdPtAddEdge(polytope_, v_[1], v_[3]);
    e_[4] = ccdPtAddEdge(polytope_, v_[3], v_[5]);
    e_[5] = ccdPtAddEdge(polytope_, v_[5], v_[1]);
    // edges connecting the upper triangle to the lower triangle
    for (int i = 0; i < 6; ++i) {
      e_[6 + i] = ccdPtAddEdge(polytope_, v_[i], v_[(i + 1) % 6]);
    }

    // upper triangle
    f_[0] = ccdPtAddFace(polytope_, e_[0], e_[1], e_[2]);
    // lower triangle
    f_[1] = ccdPtAddFace(polytope_, e_[3], e_[4], e_[5]);
    // triangles on the side
    f_[2] = ccdPtAddFace(polytope_, e_[0], e_[7], e_[6]);
    f_[3] = ccdPtAddFace(polytope_, e_[7], e_[8], e_[3]);
    f_[4] = ccdPtAddFace(polytope_, e_[8], e_[9], e_[1]);
    f_[5] = ccdPtAddFace(polytope_, e_[9], e_[10], e_[4]);
    f_[6] = ccdPtAddFace(polytope_, e_[10], e_[11], e_[2]);
    f_[7] = ccdPtAddFace(polytope_, e_[11], e_[6], e_[5]);
  }

  ~Hexagram() {
    ccdPtDestroy(polytope_);
    delete polytope_;
  }

  ccd_pt_t* polytope() const { return polytope_; }

  ccd_pt_vertex_t* v(int i) const { return v_[i]; }

  ccd_pt_edge_t* e(int i) const { return e_[i]; }

  ccd_pt_face_t* f(int i) const { return f_[i]; }

 private:
  ccd_pt_t* polytope_;
  ccd_pt_vertex_t* v_[6];
  ccd_pt_edge_t* e_[12];
  ccd_pt_face_t* f_[8];
};

template <typename T>
bool IsElementInSet(const std::unordered_set<T>& S, const T& element) {
  return S.find(element) != S.end();
}

GTEST_TEST(FCL_GJK_EPA, floodFillSilhouette1) {
  Hexagram hex;
  // Point P is just slightly above the top triangle. Only the top triangle can
  // be seen from point P.
  ccd_vec3_t p;
  p.v[0] = 0;
  p.v[1] = 0;
  p.v[2] = 1.1;
  std::unordered_set<ccd_pt_edge_t*> silhouette_edges;
  std::unordered_set<ccd_pt_face_t*> obsolete_faces;
  obsolete_faces.insert(hex.f(0));
  std::unordered_set<ccd_pt_edge_t*> obsolete_edges;

  libccd_extension::floodFillSilhouette(hex.polytope(), hex.f(0), 0, &p,
                                        &silhouette_edges, &obsolete_faces,
                                        &obsolete_edges);
  EXPECT_TRUE(obsolete_edges.empty());
  EXPECT_EQ(obsolete_faces.size(), 1u);
  EXPECT_TRUE(IsElementInSet(obsolete_faces, hex.f(0)));
  EXPECT_EQ(silhouette_edges.size(), 1u);
  EXPECT_TRUE(IsElementInSet(silhouette_edges, hex.e(0)));

  // Run silhouette algorithm for the other edges
  libccd_extension::floodFillSilhouette(hex.polytope(), hex.f(0), 1, &p,
                                        &silhouette_edges, &obsolete_faces,
                                        &obsolete_edges);
  libccd_extension::floodFillSilhouette(hex.polytope(), hex.f(0), 2, &p,
                                        &silhouette_edges, &obsolete_faces,
                                        &obsolete_edges);

  EXPECT_TRUE(obsolete_edges.empty());
  EXPECT_EQ(obsolete_faces.size(), 1u);
  EXPECT_TRUE(IsElementInSet(obsolete_faces, hex.f(0)));
  EXPECT_EQ(silhouette_edges.size(), 3u);
  EXPECT_TRUE(IsElementInSet(silhouette_edges, hex.e(0)));
  EXPECT_TRUE(IsElementInSet(silhouette_edges, hex.e(1)));
  EXPECT_TRUE(IsElementInSet(silhouette_edges, hex.e(2)));
}

GTEST_TEST(FCL_GJK_EPA, floodFillSilhouette2) {
  Hexagram hex;
  // Point P is just above the top triangle by a certain height, such that it
  // can see the triangles on the side, which connects two vertices on the upper
  // triangle, and one vertex on the lower triangle.
  ccd_vec3_t p;
  p.v[0] = 0;
  p.v[1] = 0;
  p.v[2] = 2.1;
  std::unordered_set<ccd_pt_edge_t*> silhouette_edges;
  std::unordered_set<ccd_pt_face_t*> obsolete_faces;
  obsolete_faces.insert(hex.f(0));
  std::unordered_set<ccd_pt_edge_t*> obsolete_edges;

  libccd_extension::floodFillSilhouette(hex.polytope(), hex.f(0), 0, &p,
                                        &silhouette_edges, &obsolete_faces,
                                        &obsolete_edges);
  EXPECT_EQ(obsolete_edges.size(), 1u);
  EXPECT_TRUE(IsElementInSet(obsolete_edges, hex.e(0)));
  EXPECT_EQ(obsolete_faces.size(), 2u);
  EXPECT_TRUE(IsElementInSet(obsolete_faces, hex.f(0)));
  EXPECT_TRUE(IsElementInSet(obsolete_faces, hex.f(2)));
  EXPECT_EQ(silhouette_edges.size(), 2u);
  EXPECT_TRUE(IsElementInSet(silhouette_edges, hex.e(6)));
  EXPECT_TRUE(IsElementInSet(silhouette_edges, hex.e(7)));

  // Run silhouette algorithm for the other edges
  libccd_extension::floodFillSilhouette(hex.polytope(), hex.f(0), 1, &p,
                                        &silhouette_edges, &obsolete_faces,
                                        &obsolete_edges);
  libccd_extension::floodFillSilhouette(hex.polytope(), hex.f(0), 2, &p,
                                        &silhouette_edges, &obsolete_faces,
                                        &obsolete_edges);

  EXPECT_EQ(obsolete_edges.size(), 3u);
  EXPECT_TRUE(IsElementInSet(obsolete_edges, hex.e(0)));
  EXPECT_TRUE(IsElementInSet(obsolete_edges, hex.e(1)));
  EXPECT_TRUE(IsElementInSet(obsolete_edges, hex.e(2)));
  EXPECT_EQ(obsolete_faces.size(), 4u);
  EXPECT_TRUE(IsElementInSet(obsolete_faces, hex.f(0)));
  EXPECT_TRUE(IsElementInSet(obsolete_faces, hex.f(2)));
  EXPECT_TRUE(IsElementInSet(obsolete_faces, hex.f(4)));
  EXPECT_TRUE(IsElementInSet(obsolete_faces, hex.f(6)));
  EXPECT_EQ(silhouette_edges.size(), 6u);
  EXPECT_TRUE(IsElementInSet(silhouette_edges, hex.e(6)));
  EXPECT_TRUE(IsElementInSet(silhouette_edges, hex.e(7)));
  EXPECT_TRUE(IsElementInSet(silhouette_edges, hex.e(8)));
  EXPECT_TRUE(IsElementInSet(silhouette_edges, hex.e(9)));
  EXPECT_TRUE(IsElementInSet(silhouette_edges, hex.e(10)));
  EXPECT_TRUE(IsElementInSet(silhouette_edges, hex.e(11)));
}

GTEST_TEST(FCL_GJK_EPA, floodFillSilhouette3) {
  Hexagram hex;
  // Point P is just outside the upper triangle (face0) and the triangle face2,
  // it can see both face0 and face2, but not the other triangles.
  ccd_vec3_t p;
  p.v[0] = 0;
  p.v[1] = -1 / std::sqrt(3) - 0.1;
  p.v[2] = 1.1;
  std::unordered_set<ccd_pt_edge_t*> silhouette_edges;
  std::unordered_set<ccd_pt_face_t*> obsolete_faces;
  obsolete_faces.insert(hex.f(0));
  std::unordered_set<ccd_pt_edge_t*> obsolete_edges;

  libccd_extension::floodFillSilhouette(hex.polytope(), hex.f(0), 0, &p,
                                        &silhouette_edges, &obsolete_faces,
                                        &obsolete_edges);
  EXPECT_EQ(obsolete_edges.size(), 1u);
  EXPECT_TRUE(IsElementInSet(obsolete_edges, hex.e(0)));
  EXPECT_EQ(obsolete_faces.size(), 2u);
  EXPECT_TRUE(IsElementInSet(obsolete_faces, hex.f(0)));
  EXPECT_TRUE(IsElementInSet(obsolete_faces, hex.f(2)));
  EXPECT_EQ(silhouette_edges.size(), 2u);
  EXPECT_TRUE(IsElementInSet(silhouette_edges, hex.e(6)));
  EXPECT_TRUE(IsElementInSet(silhouette_edges, hex.e(7)));

  // Run silhouette algorithm for the other edges
  libccd_extension::floodFillSilhouette(hex.polytope(), hex.f(0), 1, &p,
                                        &silhouette_edges, &obsolete_faces,
                                        &obsolete_edges);
  libccd_extension::floodFillSilhouette(hex.polytope(), hex.f(0), 2, &p,
                                        &silhouette_edges, &obsolete_faces,
                                        &obsolete_edges);

  EXPECT_EQ(obsolete_edges.size(), 1u);
  EXPECT_TRUE(IsElementInSet(obsolete_edges, hex.e(0)));
  EXPECT_EQ(obsolete_faces.size(), 2u);
  EXPECT_TRUE(IsElementInSet(obsolete_faces, hex.f(0)));
  EXPECT_TRUE(IsElementInSet(obsolete_faces, hex.f(2)));
  EXPECT_EQ(silhouette_edges.size(), 4u);
  EXPECT_TRUE(IsElementInSet(silhouette_edges, hex.e(1)));
  EXPECT_TRUE(IsElementInSet(silhouette_edges, hex.e(2)));
  EXPECT_TRUE(IsElementInSet(silhouette_edges, hex.e(6)));
  EXPECT_TRUE(IsElementInSet(silhouette_edges, hex.e(7)));
}

template <typename T>
void ComparePolytopeFeature(const ccd_list_t* features_list1,
                            const ccd_list_t* features_list2) {
  std::unordered_set<T *> features1, features2;
  T* feature;
  ccdListForEachEntry(features_list1, feature, T, list) {
    features1.insert(feature);
  }
  ccdListForEachEntry(features_list2, feature, T, list) {
    features2.insert(feature);
  }
  EXPECT_EQ(features1.size(), features2.size());
  for (const auto& f : features1) {
    EXPECT_TRUE(IsElementInSet(features2, f));
  }
}

// Returns true if the the position difference between the two vertices are
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
  for (int i = 0; i < 3; ++i) {
    auto it = e2_expected.find(f2->edge[i]);
    if (it == e2_expected.end()) {
      return false;
    }
  }
  return true;
}

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
  ccdListForEachEntry(feature1_list, f, T, list) { feature1->insert(f); }
  ccdListForEachEntry(feature2_list, f, T, list) { feature2->insert(f); }
  EXPECT_EQ(feature1->size(), feature2->size());
  for (const auto& f1 : *feature1) {
    for (const auto& f2 : *feature2) {
      if (cmp_feature(f1, f2)) {
        map_feature1_to_feature2->emplace_hint(map_feature1_to_feature2->end(),
                                               f1, f2);
        break;
      }
    }
  }
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
      if (it == map_e1_to_e2.end()) {
        throw std::runtime_error("v1_e is not found in map_e1_to_e2.\n");
      }
      auto v2_e = it->second;
      if (v2_edges.find(v2_e) == v2_edges.end()) {
        std::cout << "error\n";
      }
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
  ccd_pt_face_t* new_faces[3];
  new_faces[0] = ccdPtAddFace(polytope_expected, tetrahedron.e(0), new_edges[0],
                              new_edges[1]);
  new_faces[1] = ccdPtAddFace(polytope_expected, tetrahedron.e(1), new_edges[1],
                              new_edges[2]);
  new_faces[2] = ccdPtAddFace(polytope_expected, tetrahedron.e(2), new_edges[2],
                              new_edges[0]);

  ComparePolytope(polytope.polytope(), polytope_expected, 1E-3);
}
}  // namespace detail
}  // namespace fcl

//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

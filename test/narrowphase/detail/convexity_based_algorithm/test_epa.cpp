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
                ccdVec3Dot(&n, &p.f(i)->edge[0]->vertex[0]->v.v) + 1E-6);
    }
  }
}

GTEST_TEST(FCL_GJK_EPA, sampledEPADirection) {
  auto CheckSampledEPADirection = [](
      const ccd_pt_t* polytope, const ccd_pt_el_t* nearest_pt, ccd_real_t dir_x,
      ccd_real_t dir_y, ccd_real_t dir_z, ccd_real_t tol) {
    const ccd_vec3_t dir =
        libccd_extension::sampledEPADirection(polytope, nearest_pt);
    EXPECT_NEAR(dir.v[0], dir_x, tol);
    EXPECT_NEAR(dir.v[1], dir_y, tol);
    EXPECT_NEAR(dir.v[2], dir_z, tol);
  };

  // Nearest point is on the bottom triangle.
  // The sampled direction should be -z unit vector.
  EquilateralTetrahedron p1(0, 0, -0.1);
  const ccd_real_t tol = 1E-6;
  CheckSampledEPADirection(p1.polytope(), (const ccd_pt_el_t*)p1.f(0), 0, 0, -1,
                           tol);
  // Nearest point is on an edge, as the origin is on an edge.
  EquilateralTetrahedron p2(0, 0.5 / std::sqrt(3), 0);
  if (p2.e(0)->faces[0] == p2.f(0)) {
    CheckSampledEPADirection(p2.polytope(), (const ccd_pt_el_t*)p2.e(0), 0, 0,
                             -1, tol);
  } else {
    CheckSampledEPADirection(p2.polytope(), (const ccd_pt_el_t*)p2.e(0), 0,
                             -std::sqrt(6) / 3, std::sqrt(3) / 3, tol);
  }
  // Nearest point is on a vertex, should throw an error.
  EquilateralTetrahedron p3(-0.5, 0.5 / std::sqrt(3), 0);
  EXPECT_THROW(libccd_extension::sampledEPADirection(
                   p3.polytope(), (const ccd_pt_el_t*)p3.v(0)),
               std::runtime_error);
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
  Hexagram(ccd_real_t bottom_center_x = 0, ccd_real_t bottom_center_y = 0,
           ccd_real_t bottom_center_z = 0)
      : polytope_(new ccd_pt_t) {
    ccdPtInit(polytope_);
    auto AddHexagramVertex = [bottom_center_x, bottom_center_y, bottom_center_z,
                              this](ccd_real_t x, ccd_real_t y, ccd_real_t z) {
      return ccdPtAddVertexCoords(this->polytope_, x + bottom_center_x,
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

template <typename T>
void CheckFloodFillSilhouette(
    const T& polytope, ccd_pt_face_t* face,
    const std::vector<int>& edge_indices, const ccd_vec3_t& new_vertex,
    const std::unordered_set<int>& silhouette_edge_indices_expected,
    const std::unordered_set<int>& obsolete_face_indices_expected,
    const std::unordered_set<int>& obsolete_edge_indices_expected) {
  std::unordered_set<ccd_pt_edge_t*> silhouette_edges;
  std::unordered_set<ccd_pt_face_t*> obsolete_faces;
  obsolete_faces.insert(face);
  std::unordered_set<ccd_pt_edge_t*> obsolete_edges;
  for (const int edge_index : edge_indices) {
    libccd_extension::floodFillSilhouette(polytope.polytope(), face, edge_index,
                                          &new_vertex, &silhouette_edges,
                                          &obsolete_faces, &obsolete_edges);
  }

  // Check silhouette_edges
  EXPECT_EQ(silhouette_edges.size(), silhouette_edge_indices_expected.size());
  for (const int edge_index : silhouette_edge_indices_expected) {
    EXPECT_TRUE(IsElementInSet(silhouette_edges, polytope.e(edge_index)));
  }
  // Check obsolete_faces
  EXPECT_EQ(obsolete_faces.size(), obsolete_face_indices_expected.size());
  for (const int face_index : obsolete_face_indices_expected) {
    EXPECT_TRUE(IsElementInSet(obsolete_faces, polytope.f(face_index)));
  }
  // Check obsolete_edges
  EXPECT_EQ(obsolete_edges.size(), obsolete_edge_indices_expected.size());
  for (const auto edge_index : obsolete_edge_indices_expected) {
    EXPECT_TRUE(IsElementInSet(obsolete_edges, polytope.e(edge_index)));
  }
}

GTEST_TEST(FCL_GJK_EPA, floodFillSilhouette1) {
  Hexagram hex;
  // Point P is just slightly above the top triangle. Only the top triangle can
  // be seen from point P.
  ccd_vec3_t p;
  p.v[0] = 0;
  p.v[1] = 0;
  p.v[2] = 1.1;
  const std::unordered_set<int> empty_set;
  CheckFloodFillSilhouette(hex, hex.f(0), {0}, p, {0}, {0}, empty_set);

  // Run silhouette algorithm for the other edges
  CheckFloodFillSilhouette(hex, hex.f(0), {0, 1, 2}, p, {0, 1, 2}, {0},
                           empty_set);
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
  CheckFloodFillSilhouette(hex, hex.f(0), {0}, p, {6, 7}, {0, 2}, {0});

  // Run silhouette algorithm for the other edges
  CheckFloodFillSilhouette(hex, hex.f(0), {0, 1, 2}, p, {6, 7, 8, 9, 10, 11},
                           {0, 2, 4, 6}, {0, 1, 2});
}

GTEST_TEST(FCL_GJK_EPA, floodFillSilhouette3) {
  Hexagram hex;
  // Point P is just outside the upper triangle (face0) and the triangle face2,
  // it can see both face0 and face2, but not the other triangles.
  ccd_vec3_t p;
  p.v[0] = 0;
  p.v[1] = -1 / std::sqrt(3) - 0.1;
  p.v[2] = 1.1;
  CheckFloodFillSilhouette(hex, hex.f(0), {0}, p, {6, 7}, {0, 2}, {0});

  // Run silhouette algorithm for the other edges
  CheckFloodFillSilhouette(hex, hex.f(0), {0, 1, 2}, p, {1, 2, 6, 7}, {0, 2},
                           {0});
}

GTEST_TEST(FCL_GJK_EPA, floodFillSilhouette4) {
  // Test with the equilateral tetrahedron.
  // Point P is outside of an edge on the bottom triangle. It can see both faces
  // neighbouring that edge.

  EquilateralTetrahedron tetrahedron(0, 0, -0.1);
  ccd_vec3_t p;
  p.v[0] = 0;
  p.v[1] = -1 / std::sqrt(3) - 0.1;
  p.v[2] = -0.2;

  // Start with from face 0.
  CheckFloodFillSilhouette(tetrahedron, tetrahedron.f(0), {0, 1, 2}, p,
                           {1, 2, 3, 4}, {0, 1}, {0});

  // Start with from face 1.
  CheckFloodFillSilhouette(tetrahedron, tetrahedron.f(1), {0, 1, 2}, p,
                           {1, 2, 3, 4}, {0, 1}, {0});
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
}  // namespace detail
}  // namespace fcl

//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

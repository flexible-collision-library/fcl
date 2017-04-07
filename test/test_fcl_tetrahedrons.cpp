/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Georgia Tech Research Corporation
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
 *   * Neither the name of Georgia Tech Research Corporation nor the names of its
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

/** \author Andrew Price <arprice@gatech.edu> */

#include <gtest/gtest.h>

#include "fcl/fcl.h"

using namespace fcl;

// Vertices for upper tetrahedron
Vector3<double> va[4] = {{-1.0,-1.0, 0.0},
                         { 1.0,-1.0, 0.0},
                         { 0.0, 1.0, 0.0},
                         { 0.0, 0.0, 1.0}};
// Vertices for lower tetrahedron (with penetration)
Vector3<double> vb[4] = {{-1.0,-1.0,-1.0},
                         { 1.0,-1.0,-1.0},
                         { 0.0, 1.0,-1.0},
                         { 0.0, 0.0, 0.0001}};
// Vertices for lower tetrahedron (with contact)
Vector3<double> vc[4] = {{-1.0,-1.0,-1.0},
                         { 1.0,-1.0,-1.0},
                         { 0.0, 1.0,-1.0},
                         { 0.0, 0.0, 0.0}};

// Face indices for either tetrahedron
int f[4][3] = {{0,2,1},
               {1,2,3},
               {0,1,3},
               {0,3,2}};

template <typename Scalar>
struct ConvexData
{
  std::vector<Vector3<Scalar>> plane_normals;
  std::vector<Scalar> plane_distances;
  std::vector<Vector3<Scalar>> vertices;
  std::vector<int> polygon_list;
  std::unique_ptr<Convex<Scalar>> cvx;
};

template <typename Scalar>
std::shared_ptr<ConvexData<Scalar>> constructConvex(const Vector3<double>* v)
{
  std::shared_ptr<ConvexData<Scalar>> cd = std::make_shared<ConvexData<Scalar>>();
  const int num_faces = 4;
  const int num_vertices = 4;

  // Add vertices
  for (int i = 0; i < num_vertices; ++i)
  {
    cd->vertices.push_back(v[i].cast<Scalar>());
  }

  // Compute face normals
  for (int i = 0; i < num_faces; ++i)
  {
    /**
     * Faces follow a right hand winding:
     *    c
     *    /\
     *   /__\
     *  a    b
     * n = (b-a)x(c-a) points out of the screen
     */
    cd->plane_normals.emplace_back(((v[f[i][1]]-v[f[i][0]]).cross(v[f[i][2]]-v[f[i][0]])).normalized().cast<Scalar>());
    cd->plane_distances.emplace_back(cd->plane_normals[i].dot(v[f[i][0]].cast<Scalar>()));
  }

  // Add polygons
  for (int i = 0; i < num_faces; ++i)
  {
    cd->polygon_list.push_back(3); // All faces are triangles
    for (int j = 0; j < 3; ++j)
    {
      cd->polygon_list.push_back(f[i][j]);
    }
  }

  cd->cvx = std::unique_ptr<Convex<Scalar>>(
               new Convex<Scalar>(cd->plane_normals.data(),
                                  cd->plane_distances.data(), 4,
                                  cd->vertices.data(), 4,
                                  cd->polygon_list.data()));
  return cd;
}

template <typename Scalar>
void test_individual_triangles(bool reverseOrder = false,
                               bool nonpenetrating = false)
{
  Vector3<double>* v1 = va;
  Vector3<double>* v2 = (nonpenetrating ? vc : vb);
  Vector3<Scalar> contact_points[3];
  unsigned int num_contact_points;
  Scalar penetration_depth;
  Vector3<Scalar> normal;
  for (int tri = 0; tri < 3; ++tri)
  {
    Vector3<Scalar> A[3] = { v1[f[0][0]].cast<Scalar>(),
                             v1[f[0][1]].cast<Scalar>(),
                             v1[f[0][2]].cast<Scalar>() };
    Vector3<Scalar> B[3] = { v2[f[tri+1][0]].cast<Scalar>(),
                             v2[f[tri+1][1]].cast<Scalar>(),
                             v2[f[tri+1][2]].cast<Scalar>() };
    if (reverseOrder)
    {
      std::swap(A, B);
    }
    bool res = detail::Intersect<Scalar>::intersect_Triangle(
                A[0], A[1], A[2],
                B[0], B[1], B[2],
                contact_points, &num_contact_points, &penetration_depth, &normal);

  if (!res)
  {
    std::cerr << "Configuration: "
              << (reverseOrder ? "B-A" : "A-B" ) << ", "
              << (nonpenetrating ? "non" : "") << "penetrating."
              << std::endl;
  }
    EXPECT_TRUE(res);
    EXPECT_EQ(1, num_contact_points);
    for (int i = 0; i < num_contact_points; ++i)
    {
      Vector3<Scalar> expected;
      if (nonpenetrating) { expected = Vector3<Scalar>::Zero(); }
      else { expected = vb[3].cast<Scalar>(); }
      EXPECT_LT((expected-contact_points[i]).norm(), 1e-6);
    }
  }
}

template <typename Scalar>
void test_convex_meshes(GJKSolverType Solver = GST_INDEP,
                        bool reverseOrder = false,
                        bool nonpenetrating = false)
{
  if (GST_LIBCCD == Solver && nonpenetrating)
  {
	// TODO: The libccd solver returns 0 for nonpenetrating contact.
    return;
  }
  std::shared_ptr<ConvexData<Scalar>> A = constructConvex<Scalar>(va);
  std::shared_ptr<ConvexData<Scalar>> B;
  if (nonpenetrating)
  {
    B = constructConvex<Scalar>(vc);
  }
  else
  {
    B = constructConvex<Scalar>(vb);
  }

  CollisionRequest<Scalar> req(100, true);
  req.enable_cached_gjk_guess = false;
  req.gjk_solver_type = Solver; //GST_LIBCCD; // GST_INDEP
  CollisionResult<Scalar>  res;
  int num_collisions;

  // Convex-convex collision
  if (reverseOrder)
  {
    // Flip the order
    num_collisions = collide(B->cvx.get(), Transform3<Scalar>::Identity(),
                             A->cvx.get(), Transform3<Scalar>::Identity(),
                             req, res);
  }
  else
  {
    num_collisions = collide(A->cvx.get(), Transform3<Scalar>::Identity(),
                             B->cvx.get(), Transform3<Scalar>::Identity(),
                             req, res);
  }

  if (1 != num_collisions)
  {
    std::cerr << "Configuration: "
              << (GST_LIBCCD == Solver ? "GST_LIBCCD" : "GST_INDEP") << ", "
              << (reverseOrder ? "B-A" : "A-B" ) << ", "
              << (nonpenetrating ? "non" : "") << "penetrating."
              << std::endl;
  }
  EXPECT_EQ(1, num_collisions);
  EXPECT_EQ(res.numContacts(), num_collisions);
  for (int i = 0; i < res.numContacts(); ++i)
  {
    EXPECT_LT(res.getContact(i).pos.norm(), 1e-3);
  }
}

template <typename Scalar>
void test_general_meshes()
{

}

//==============================================================================
GTEST_TEST(FCL_TETRAHEDRONS, individual_triangles)
{
  for (bool flipped : {false, true})
  {
    for (bool nonpenetrating : {false, true})
    {
      test_individual_triangles<float>(flipped, nonpenetrating);
      test_individual_triangles<double>(flipped, nonpenetrating);
    }
  }
}

//==============================================================================
GTEST_TEST(FCL_TETRAHEDRONS, convex_meshes)
{
  for (GJKSolverType solver : {GST_INDEP, GST_LIBCCD})
  {
    for (bool flipped : {false, true})
    {
      for (bool nonpenetrating : {false, true})
      {
        test_convex_meshes<float>(solver, flipped, nonpenetrating);
        test_convex_meshes<double>(solver, flipped, nonpenetrating);
      }
    }
  }
}

//==============================================================================
GTEST_TEST(FCL_TETRAHEDRONS, general_meshes)
{
  test_general_meshes<float>();
  test_general_meshes<double>();
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

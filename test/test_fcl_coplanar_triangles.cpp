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
#include "test_fcl_utility.h"
#include "test_fcl_hungarian.h"
#include "test_fcl_hungarian-inl.h"

using namespace fcl;

template <typename Scalar>
void verifyContacts(const Vector3<Scalar>* expected, const Vector3<Scalar>* actual, const int numContacts)
{
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> CostMatrix;

  CostMatrix C(numContacts, numContacts);
  for (int i = 0; i < numContacts; ++i)
  {
    for (int j = 0; j < numContacts; ++j)
    {
      C(i,j) = (expected[i]-actual[j]).squaredNorm();
    }
  }

  fcl::test::Hungarian<Scalar> H(C);

  std::vector<int> permutation = H.getAssignment();

  EXPECT_LT(H.getSolutionCost(), 1e-6);
  for (int i = 0; i < numContacts; ++i)
  {
    EXPECT_LT((expected[i]-actual[permutation[i]]).norm(), 1e-6);
  }
}

//==============================================================================
// Parallel edge test
//    _______
//   |\      |
//   | \___  |
//   | |\  | |
//   | | \ | |
//   | |__\| |
//   |     \ |
//   |______\|
//
// Transverse edge test
//    _______
//   |\      |
//   | \___  |
//   | |\ /| |
//   | | X | |
//   | |/_\| |
//   |     \ |
//   |______\|
// v_a[0]    v_a[1]

template <typename Scalar>
void test_collision_triangle_pairs()
{
  // Outline of a rectangle
  Vector3<Scalar> v_a[4] = {{-0.5,0.5,-0.5},
                            {-0.5,0.5, 0.5},
                            { 0.5,0.5,-0.5},
                            { 0.5,0.5, 0.5}};
  Vector3<Scalar> v_b[4] = {{-0.25,0.5,-0.25},
                            {-0.25,0.5, 0.25},
                            { 0.25,0.5,-0.25},
                            { 0.25,0.5, 0.25}};

  // Vertex lists for triangles
  int t_a[2][3] = {{0,1,2},  // Surrounding triangles
                   {3,2,1}};
  int t_bp[2][3] = {{0,1,2}, // Parallel crossing
                    {3,2,1}};
  int t_bx[2][3] = {{0,1,3}, // Transverse crossing
                    {3,2,0}};
  Vector3<Scalar> contact_points[3];
  unsigned int num_contact_points;
  Scalar penetration_depth;
  Vector3<Scalar> normal;

  // Test smaller triangle inside larger triangle

  bool res = detail::Intersect<Scalar>::intersect_Triangle(
        v_b[t_bp[0][0]], v_b[t_bp[0][1]], v_b[t_bp[0][2]],
        v_a[t_a[0][0]], v_a[t_a[0][1]], v_a[t_a[0][2]],
        contact_points, &num_contact_points, &penetration_depth, &normal);

  EXPECT_TRUE(res);
  EXPECT_EQ(num_contact_points, 3u);
  EXPECT_EQ(penetration_depth, (Scalar)0);
  std::vector<Vector3<Scalar>> expected = {v_b[t_bp[0][0]], v_b[t_bp[0][1]], v_b[t_bp[0][2]]};
  verifyContacts(expected.data(), contact_points, num_contact_points);

  // Flip the triangle order
  res = detail::Intersect<Scalar>::intersect_Triangle(
        v_a[t_a[0][0]], v_a[t_a[0][1]], v_a[t_a[0][2]],
        v_b[t_bp[0][0]], v_b[t_bp[0][1]], v_b[t_bp[0][2]],
        contact_points, &num_contact_points, &penetration_depth, &normal);

  EXPECT_TRUE(res);
  EXPECT_EQ(num_contact_points, 3u);
  EXPECT_EQ(penetration_depth, (Scalar)0);
  verifyContacts(expected.data(), contact_points, num_contact_points);
}

template <typename Scalar>
void test_contact_alignment()
{
  Vector3<Scalar> v[3] = {{-0.5,0.5,-0.5},
                          {-0.5,0.5, 0.5},
                          { 0.5,0.5,-0.5}};

  std::vector<Vector3<Scalar>> expected = {v[0], v[1], v[2]};
  verifyContacts(expected.data(), expected.data(), 3);
  std::vector<Vector3<Scalar>> actual = {v[1], v[2], v[0]};
  verifyContacts(expected.data(), actual.data(), 3);
  actual = {v[2], v[0], v[1]};
  verifyContacts(expected.data(), actual.data(), 3);
}

//==============================================================================
GTEST_TEST(FCL_COPLANAR_TRIANGLES, contact_alignment)
{
  test_contact_alignment<float>();
  test_contact_alignment<double>();
}

//==============================================================================
GTEST_TEST(FCL_COPLANAR_TRIANGLES, collision_triangle_pairs)
{
  test_collision_triangle_pairs<float>();
  test_collision_triangle_pairs<double>();
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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

#include "fcl/narrowphase/detail/primitive_shape_algorithm/triangle_triangle.h"
#include "fcl/narrowphase/detail/primitive_shape_algorithm/triangle_triangle-inl.h"

#include "fcl/math/constants.h"
#include "test_fcl_hungarian.h"

#include <gtest/gtest.h>

using namespace fcl;
using namespace fcl::detail;

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

template <typename S>
void test_general_tris()
{
	std::vector<Vector3<S>> expected;
	std::array<Vector3<S>, 6> contacts;
	unsigned int num_contacts;
    S penetration_depth;
    Vector3<S> normal;
	bool res;
	res = intersectTriangles<S, true>({0,0,0},{0,1,0},{0,0,1},
	                                  {0,0,0},{0,-1,0},{1,0,0},
	                                  contacts.data(), &num_contacts,
	                                  &penetration_depth, &normal);
	EXPECT_TRUE(res);
	EXPECT_EQ(1u, num_contacts);
	EXPECT_TRUE(fabs(penetration_depth) < 1e-6);
	EXPECT_TRUE(contacts[0].norm() < 1e-6);


	res = intersectTriangles<S, true>({1,0,0},{0,1,0},{0,-1,0},
	                                  {0,0,0},{0,1,1},{0,-1,1},
	                                  contacts.data(), &num_contacts,
	                                  &penetration_depth, &normal);
	EXPECT_TRUE(res);
	EXPECT_EQ(1u, num_contacts);
	EXPECT_TRUE(fabs(penetration_depth) < 1e-6);
	EXPECT_TRUE(contacts[0].norm() < 1e-6);


	res = intersectTriangles<S, true>({1,0,0},{0,1,0},{0,-1,0},
	                                  {0,0,0.5},{0,1,1},{0,-1,1},
	                                  contacts.data(), &num_contacts,
	                                  &penetration_depth, &normal);
	EXPECT_FALSE(res);


	res = intersectTriangles<S, true>({1,0,0},{0,1,0},{0,-1,0},
	                                  {1,0,0.1},{0,1,0.1},{0,-1,0.1},
	                                  contacts.data(), &num_contacts,
	                                  &penetration_depth, &normal);
	EXPECT_FALSE(res);


	res = intersectTriangles<S, true>({1,0,0},{0,1,0},{0,-1,0},
	                                  {1,0,0.1},{0,1,0.1},{0,-1,0.2},
	                                  contacts.data(), &num_contacts,
	                                  &penetration_depth, &normal);
	EXPECT_FALSE(res);


	res = intersectTriangles<S, true>({1,0,0},{0,1,0},{0,-1,0},
	                                  {0.5,0,0},{0,1,0},{0,-1,0},
	                                  contacts.data(), &num_contacts,
	                                  &penetration_depth, &normal);
	expected = {{0.5,0,0},{0,1,0},{0,-1,0}};
	EXPECT_TRUE(res);
	EXPECT_EQ(3u, num_contacts);
	EXPECT_TRUE(fabs(penetration_depth) < 1e-6);
	verifyContacts(expected.data(), contacts.data(), num_contacts);


	res = intersectTriangles<S, true>({1,0,0},{0,1,0},{0,-1,0},
	                                  {0.5,0,0},{0,0.5,0},{0,0.25,0},
	                                  contacts.data(), &num_contacts,
	                                  &penetration_depth, &normal);
	expected = {{0.5,0,0},{0,0.5,0},{0,0.25,0}};
	EXPECT_TRUE(res);
	EXPECT_EQ(3u, num_contacts);
	EXPECT_TRUE(fabs(penetration_depth) < 1e-6);
	verifyContacts(expected.data(), contacts.data(), num_contacts);


	res = intersectTriangles<S, true>({0.5,0,0},{0,0.5,0},{0,0.25,0},
	                                  {1,0,0},{0,1,0},{0,-1,0},
	                                  contacts.data(), &num_contacts,
	                                  &penetration_depth, &normal);
	expected = {{0.5,0,0},{0,0.5,0},{0,0.25,0}};
	EXPECT_TRUE(res);
	EXPECT_EQ(3u, num_contacts);
	EXPECT_TRUE(fabs(penetration_depth) < 1e-6);
	verifyContacts(expected.data(), contacts.data(), num_contacts);


	res = intersectTriangles<S, true>({1,-0.1,0},{-0.5,1,0},{-0.5,-0.1,0},
	                                  {0,0.1,-0.1},{0,0.1,1},{0,-1,-0.1},
	                                  contacts.data(), &num_contacts,
	                                  &penetration_depth, &normal);
	expected = {{0,0.1,0},{0,-0.1,0}};
	EXPECT_TRUE(res);
	EXPECT_EQ(2u, num_contacts);
	EXPECT_TRUE(fabs(penetration_depth-0.1) < 1e-6);
	EXPECT_TRUE(normal.cross(Vector3<S>::UnitZ()).norm() < 1e-6);
	verifyContacts(expected.data(), contacts.data(), num_contacts);


	res = intersectTriangles<S, true>({0,0.1,-0.1},{0,0.1,1},{0,-1,-0.1},
	                                  {1,-0.1,0},{-0.5,1,0},{-0.5,-0.1,0},
	                                  contacts.data(), &num_contacts,
	                                  &penetration_depth, &normal);
	expected = {{0,0.1,0},{0,-0.1,0}};
	EXPECT_TRUE(res);
	EXPECT_EQ(2u, num_contacts);
	EXPECT_TRUE(fabs(penetration_depth-0.1) < 1e-6);
	EXPECT_TRUE(normal.cross(Vector3<S>::UnitZ()).norm() < 1e-6);
	verifyContacts(expected.data(), contacts.data(), num_contacts);
}

//==============================================================================
GTEST_TEST(FCL_TRI_TRI_INTERSECTION, general_tris)
{
  test_general_tris<float>();
  test_general_tris<double>();
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Open Source Robotics Foundation
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
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
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

/** @author Jeongseok Lee */

#include <gtest/gtest.h>

#include "fcl/config.h"
#include "fcl/geometry/bvh/BVH_model.h"
#include "test_fcl_utility.h"
#include <iostream>

using namespace fcl;

template<typename BV>
void testBVHModelPointCloud()
{
  using S = typename BV::S;

  std::shared_ptr<BVHModel<BV> > model(new BVHModel<BV>);

  Box<S> box;
  auto a = box.side[0];
  auto b = box.side[1];
  auto c = box.side[2];
  std::vector<Vector3<S>> points(8);
  points[0] << 0.5 * a, -0.5 * b, 0.5 * c;
  points[1] << 0.5 * a, 0.5 * b, 0.5 * c;
  points[2] << -0.5 * a, 0.5 * b, 0.5 * c;
  points[3] << -0.5 * a, -0.5 * b, 0.5 * c;
  points[4] << 0.5 * a, -0.5 * b, -0.5 * c;
  points[5] << 0.5 * a, 0.5 * b, -0.5 * c;
  points[6] << -0.5 * a, 0.5 * b, -0.5 * c;
  points[7] << -0.5 * a, -0.5 * b, -0.5 * c;

  int result;

  result = model->beginModel();
  EXPECT_EQ(result, BVH_OK);

  for (std::size_t i = 0; i < points.size(); ++i)
  {
    result = model->addVertex(points[i]);
    EXPECT_EQ(result, BVH_OK);
  }

  result = model->endModel();
  EXPECT_EQ(result, BVH_OK);

  model->computeLocalAABB();

  EXPECT_EQ(model->num_vertices, 8);
  EXPECT_EQ(model->num_tris, 0);
  EXPECT_EQ(model->build_state, BVH_BUILD_STATE_PROCESSED);
}

template<typename BV>
void testBVHModelTriangles()
{
  using S = typename BV::S;

  std::shared_ptr<BVHModel<BV> > model(new BVHModel<BV>);
  Box<S> box;

  auto a = box.side[0];
  auto b = box.side[1];
  auto c = box.side[2];
  std::vector<Vector3<S>> points(8);
  std::vector<Triangle> tri_indices(12);
  points[0] << 0.5 * a, -0.5 * b, 0.5 * c;
  points[1] << 0.5 * a, 0.5 * b, 0.5 * c;
  points[2] << -0.5 * a, 0.5 * b, 0.5 * c;
  points[3] << -0.5 * a, -0.5 * b, 0.5 * c;
  points[4] << 0.5 * a, -0.5 * b, -0.5 * c;
  points[5] << 0.5 * a, 0.5 * b, -0.5 * c;
  points[6] << -0.5 * a, 0.5 * b, -0.5 * c;
  points[7] << -0.5 * a, -0.5 * b, -0.5 * c;

  tri_indices[0].set(0, 4, 1);
  tri_indices[1].set(1, 4, 5);
  tri_indices[2].set(2, 6, 3);
  tri_indices[3].set(3, 6, 7);
  tri_indices[4].set(3, 0, 2);
  tri_indices[5].set(2, 0, 1);
  tri_indices[6].set(6, 5, 7);
  tri_indices[7].set(7, 5, 4);
  tri_indices[8].set(1, 5, 2);
  tri_indices[9].set(2, 5, 6);
  tri_indices[10].set(3, 7, 0);
  tri_indices[11].set(0, 7, 4);

  int result;

  result = model->beginModel();
  EXPECT_EQ(result, BVH_OK);

  for (std::size_t i = 0; i < tri_indices.size(); ++i)
  {
    result = model->addTriangle(points[tri_indices[i][0]], points[tri_indices[i][1]], points[tri_indices[i][2]]);
    EXPECT_EQ(result, BVH_OK);
  }

  result = model->endModel();
  EXPECT_EQ(result, BVH_OK);

  model->computeLocalAABB();

  EXPECT_EQ(model->num_vertices, 12 * 3);
  EXPECT_EQ(model->num_tris, 12);
  EXPECT_EQ(model->build_state, BVH_BUILD_STATE_PROCESSED);
}

template<typename BV>
void testBVHModelSubModel()
{
  using S = typename BV::S;

  std::shared_ptr<BVHModel<BV> > model(new BVHModel<BV>);
  Box<S> box;

  auto a = box.side[0];
  auto b = box.side[1];
  auto c = box.side[2];
  std::vector<Vector3<S>> points(8);
  std::vector<Triangle> tri_indices(12);
  points[0] << 0.5 * a, -0.5 * b, 0.5 * c;
  points[1] << 0.5 * a, 0.5 * b, 0.5 * c;
  points[2] << -0.5 * a, 0.5 * b, 0.5 * c;
  points[3] << -0.5 * a, -0.5 * b, 0.5 * c;
  points[4] << 0.5 * a, -0.5 * b, -0.5 * c;
  points[5] << 0.5 * a, 0.5 * b, -0.5 * c;
  points[6] << -0.5 * a, 0.5 * b, -0.5 * c;
  points[7] << -0.5 * a, -0.5 * b, -0.5 * c;

  tri_indices[0].set(0, 4, 1);
  tri_indices[1].set(1, 4, 5);
  tri_indices[2].set(2, 6, 3);
  tri_indices[3].set(3, 6, 7);
  tri_indices[4].set(3, 0, 2);
  tri_indices[5].set(2, 0, 1);
  tri_indices[6].set(6, 5, 7);
  tri_indices[7].set(7, 5, 4);
  tri_indices[8].set(1, 5, 2);
  tri_indices[9].set(2, 5, 6);
  tri_indices[10].set(3, 7, 0);
  tri_indices[11].set(0, 7, 4);

  int result;

  result = model->beginModel();
  EXPECT_EQ(result, BVH_OK);

  result = model->addSubModel(points, tri_indices);
  EXPECT_EQ(result, BVH_OK);

  result = model->endModel();
  EXPECT_EQ(result, BVH_OK);

  model->computeLocalAABB();

  EXPECT_EQ(model->num_vertices, 8);
  EXPECT_EQ(model->num_tris, 12);
  EXPECT_EQ(model->build_state, BVH_BUILD_STATE_PROCESSED);
}

template<typename BV>
void testBVHModel()
{
  testBVHModelTriangles<BV>();
  testBVHModelPointCloud<BV>();
  testBVHModelSubModel<BV>();
}

GTEST_TEST(FCL_BVH_MODELS, building_bvh_models)
{
//  testBVHModel<AABB<float>>();
//  testBVHModel<OBB<float>>();
//  testBVHModel<RSS<float>>();
//  testBVHModel<kIOS<float>>();
//  testBVHModel<OBBRSS<float>>();
//  testBVHModel<KDOP<float, 16> >();
//  testBVHModel<KDOP<float, 18> >();
//  testBVHModel<KDOP<float, 24> >();

  testBVHModel<AABB<double>>();
  testBVHModel<OBB<double>>();
  testBVHModel<RSS<double>>();
  testBVHModel<kIOS<double>>();
  testBVHModel<OBBRSS<double>>();
  testBVHModel<KDOP<double, 16> >();
  testBVHModel<KDOP<double, 18> >();
  testBVHModel<KDOP<double, 24> >();
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

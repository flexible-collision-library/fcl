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

  if (model->getNodeType() != BV_AABB
      && model->getNodeType() != BV_KDOP16
      && model->getNodeType() != BV_KDOP18
      && model->getNodeType() != BV_KDOP24)
  {
    std::cout << "Abort test since '" << test::getNodeTypeName(model->getNodeType())
              << "' does not support point cloud model. "
              << "Please see issue #67." << std::endl;
    return;
  }

  Box<S> box(1, 1, 1);
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
  EXPECT_EQ(BVH_OK, result);

  for (std::size_t i = 0; i < points.size(); ++i)
  {
    result = model->addVertex(points[i]);
    EXPECT_EQ(BVH_OK, result);
  }

  result = model->endModel();
  EXPECT_EQ(BVH_OK, result);

  model->computeLocalAABB();

  EXPECT_EQ(8, model->num_vertices);
  EXPECT_EQ(0, model->num_tris);
  EXPECT_EQ(BVH_BUILD_STATE_PROCESSED, model->build_state);
}

template<typename BV>
void testBVHModelTriangles()
{
  using S = typename BV::S;

  std::shared_ptr<BVHModel<BV> > model(new BVHModel<BV>);
  Box<S> box(2, 1, 1);

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
  EXPECT_EQ(BVH_OK, result);

  for (std::size_t i = 0; i < tri_indices.size(); ++i)
  {
    result = model->addTriangle(points[tri_indices[i][0]], points[tri_indices[i][1]], points[tri_indices[i][2]]);
    EXPECT_EQ(BVH_OK, result);
  }

  result = model->endModel();
  EXPECT_EQ(BVH_OK, result);

  model->computeLocalAABB();

  EXPECT_EQ(12 * 3, model->num_vertices);
  EXPECT_EQ(12, model->num_tris);
  EXPECT_EQ(BVH_BUILD_STATE_PROCESSED, model->build_state);

  result = model->beginUpdateModel();
  EXPECT_EQ(BVH_OK, result);

  for (std::size_t i = 0; i < tri_indices.size(); ++i)
  {
    // Don't actually change anything
    result = model->updateTriangle(points[tri_indices[i][0]], points[tri_indices[i][1]], points[tri_indices[i][2]]);
  }
  EXPECT_EQ(BVH_OK, result);

  result = model->endUpdateModel(true, true);
  EXPECT_EQ(BVH_OK, result);

  S vol = model->computeVolume();
  EXPECT_LT(fabs(vol-(a*b*c)), 1e-6);
  Vector3<S> com = model->computeCOM();
  EXPECT_LT(com.norm(), 1e-6);
  Matrix3<S> inert = model->computeMomentofInertia();
  Matrix3<S> expected_inert;
  expected_inert << (b*b+c*c)/12.0, 0.0, 0.0,
                    0.0, (a*a+c*c)/12.0, 0.0,
                    0.0, 0.0, (a*a+b*b)/12.0;
  expected_inert *= vol;
  EXPECT_LT((expected_inert-inert).norm(), 1e-6);
}

template<typename BV>
void testBVHModelSubModel()
{
  using S = typename BV::S;

  std::shared_ptr<BVHModel<BV> > model(new BVHModel<BV>);
  Box<S> box(1, 1, 1);

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
  EXPECT_EQ(BVH_OK, result);

  result = model->addSubModel(points, tri_indices);
  EXPECT_EQ(BVH_OK, result);

  result = model->endModel();
  EXPECT_EQ(BVH_OK, result);

  model->computeLocalAABB();

  EXPECT_EQ(8, model->num_vertices);
  EXPECT_EQ(12, model->num_tris);
  EXPECT_EQ(BVH_BUILD_STATE_PROCESSED, model->build_state);
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

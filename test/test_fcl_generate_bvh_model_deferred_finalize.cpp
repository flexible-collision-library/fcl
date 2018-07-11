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

/** @author Nico van Duijn */

#include <gtest/gtest.h>

#include "fcl/config.h"
#include "fcl/geometry/geometric_shape_to_BVH_model.h"
#include "test_fcl_utility.h"
#include <iostream>

using namespace fcl;

/** 
@description   This file tests functionality in generateBVHModel(). Specifically,
               it tests that a BVHModel can be either finalized after adding a 
               geometric primitive, or left "open" in order to add further
               shapes at a later time. This functionality is tested without any
               regard to proper functionality or special cases in the conversion from
               geometric primitive to BVHModel. 
**/


/**
@details   This function tests adding geometric primitives to an empty model.
           It checks proper functionality of those simply by 
           verifying the return value, the number of vertices, triangles and the state of the model.
           In the process, the provided model will always be BVH_BUILD_STATE_BEGUN afterwards
**/
template<typename BV, typename ShapeType>
void checkAddToEmptyModel(BVHModel<BV>& model, const ShapeType& shape)
{  
  using S = typename BV::S;
  uint8_t n = 32; // Hard-coded mesh-resolution. Not testing corner cases where n=0 or such
  int ret;

  // Make sure we are given an empty model
  GTEST_ASSERT_EQ(model.build_state, BVH_BUILD_STATE_EMPTY);
  uint8_t num_vertices = model.num_vertices;
  uint8_t num_tris = model.num_tris;
  GTEST_ASSERT_EQ(num_vertices, 0);
  GTEST_ASSERT_EQ(num_tris, 0);

  // Add the shape to the model and count vertices and triangles to make sure it has been created
  ret = generateBVHModel(model, shape, Transform3<S>::Identity(), n, FinalizeModel::DONT);
  GTEST_ASSERT_EQ(ret, BVH_OK);
  EXPECT_GT(model.num_vertices, num_vertices);
  EXPECT_GT(model.num_tris, num_tris); 
  EXPECT_EQ(model.build_state, BVH_BUILD_STATE_BEGUN);
}

// Specialization for boxes
template<typename BV>
void checkAddToEmptyModel(BVHModel<BV>& model, const Box<typename BV::S>& shape)
{  
  using S = typename BV::S;
  int ret;

  // Make sure we are given an empty model
  GTEST_ASSERT_EQ(model.build_state, BVH_BUILD_STATE_EMPTY);
  GTEST_ASSERT_EQ(model.num_vertices, 0);
  GTEST_ASSERT_EQ(model.num_tris, 0);

  // Add the shape to the model and count vertices and triangles to make sure it has been created
  ret = generateBVHModel(model, shape, Transform3<S>::Identity(), FinalizeModel::DONT);
  GTEST_ASSERT_EQ(ret, BVH_OK);
  EXPECT_EQ(model.num_vertices, 8);
  EXPECT_EQ(model.num_tris, 12); 
  EXPECT_EQ(model.build_state, BVH_BUILD_STATE_BEGUN);
}


/**
@details   This function tests adding geometric primitives to an unfinalized model.
           It checks proper functionality by verifying the return value, 
           the number of vertices, triangles and the state of the model.
           After execution, the provided model will always be BVH_BUILD_STATE_BEGUN.
**/
template<typename BV, typename ShapeType>
void checkAddToUnfinalizedModel(BVHModel<BV>& model, const ShapeType& shape)
{  
  using S = typename BV::S;
  const uint8_t n = 32;
  int ret;

  // Make sure we are given a model that is already begun
  GTEST_ASSERT_EQ(model.build_state, BVH_BUILD_STATE_BEGUN);
  uint8_t num_vertices = model.num_vertices;
  uint8_t num_tris = model.num_tris;

  // Add the shape to the model and count vertices and triangles to make sure it has been created
  ret = generateBVHModel(model, shape, Transform3<S>(Translation3<S>(Vector3<S>(2.0, 2.0, 2.0))), n, FinalizeModel::DONT);
  GTEST_ASSERT_EQ(ret, BVH_OK);
  EXPECT_GT(model.num_vertices, num_vertices);
  EXPECT_GT(model.num_tris, num_tris); 
  EXPECT_EQ(model.build_state, BVH_BUILD_STATE_BEGUN);
}

// Specialization for boxes
template<typename BV>
void checkAddToUnfinalizedModel(BVHModel<BV>& model, const Box<typename BV::S>& shape)
{  
  using S = typename BV::S;
  int ret;

  // Make sure we are given a model that is already begun
  GTEST_ASSERT_EQ(model.build_state, BVH_BUILD_STATE_BEGUN);
  uint8_t num_vertices = model.num_vertices;
  uint8_t num_tris = model.num_tris;

  // Add the shape to the model and count vertices and triangles to make sure it has been created
  ret = generateBVHModel(model, shape, Transform3<S>(Translation3<S>(Vector3<S>(2.0, 2.0, 2.0))), FinalizeModel::DONT);
  GTEST_ASSERT_EQ(ret, BVH_OK);
  EXPECT_EQ(model.num_vertices, num_vertices + 8);
  EXPECT_EQ(model.num_tris, num_tris + 12); 
  EXPECT_EQ(model.build_state, BVH_BUILD_STATE_BEGUN);
}

/**
@details   This function tests adding primitives to a previously begun model
           It checks proper functionality by checking the return value,
           the number of vertices and triangles and the state of the model
           after execution. After this call, the model is finalized.

**/
template<typename BV, typename ShapeType>
void checkAddAndFinalizeModel(BVHModel<BV>& model, const ShapeType& shape){
  using S = typename BV::S;
  const uint8_t n = 32;
  int ret;

  // Make sure we are given a model that is already begun
  GTEST_ASSERT_EQ(model.build_state, BVH_BUILD_STATE_BEGUN);
  uint8_t num_vertices = model.num_vertices;
  uint8_t num_tris = model.num_tris;

  // Add another instance of the shape and make sure it was added to the model by counting vertices and tris
  ret = generateBVHModel(model, shape, Transform3<S>(Translation3<S>(Vector3<S>(2.0, 2.0, 2.0))), n, FinalizeModel::DO);
  GTEST_ASSERT_EQ(ret, BVH_OK);
  EXPECT_GT(model.num_vertices, num_vertices);
  EXPECT_GT(model.num_tris, num_tris);
  EXPECT_EQ(model.build_state, BVH_BUILD_STATE_PROCESSED);
}

// Specialization for boxes
template<typename BV>
void checkAddAndFinalizeModel(BVHModel<BV>& model, const Box<typename BV::S>& shape){
  using S = typename BV::S;
  int ret;

  // Make sure we are given a model that is already begun
  GTEST_ASSERT_EQ(model.build_state, BVH_BUILD_STATE_BEGUN);
  uint8_t num_vertices = model.num_vertices;
  uint8_t num_tris = model.num_tris;

  // Add another instance of the shape and make sure it was added to the model by counting vertices and tris
  ret = generateBVHModel(model, shape, Transform3<S>(Translation3<S>(Vector3<S>(3.0, 3.0, 3.0))), FinalizeModel::DO);
  GTEST_ASSERT_EQ(ret, BVH_OK);
  EXPECT_EQ(model.num_vertices, num_vertices + 8);
  EXPECT_EQ(model.num_tris, num_tris + 12);
  EXPECT_EQ(model.build_state, BVH_BUILD_STATE_PROCESSED);
}


/**
@details   This function tests that adding geometric primitives to a finalized model indeed
           returns the BVH error we would expect.
**/
template<typename BV, typename ShapeType>
void checkAddToFinalizedModel(BVHModel<BV>& model, const ShapeType& shape)
{
  using S = typename BV::S;
  const uint8_t n = 32; 

  GTEST_ASSERT_EQ(model.build_state, BVH_BUILD_STATE_PROCESSED);
  auto ret = generateBVHModel(model, shape, Transform3<S>::Identity(), n, FinalizeModel::DONT);
  EXPECT_EQ(ret, BVH_ERR_BUILD_OUT_OF_SEQUENCE);
}

// Specialization for boxes
template<typename BV>
void checkAddToFinalizedModel(BVHModel<BV>& model, const Box<typename BV::S>& shape)
{
  using S = typename BV::S;

  GTEST_ASSERT_EQ(model.build_state, BVH_BUILD_STATE_PROCESSED);
  auto ret = generateBVHModel(model, shape, Transform3<S>::Identity(), FinalizeModel::DONT);
  EXPECT_EQ(ret, BVH_ERR_BUILD_OUT_OF_SEQUENCE);
}

template<typename BV>
void testBVHModelFromBox()
{
  using S = typename BV::S;
  const S a = 1.0;
  const S b = 1.0;
  const S c = 1.0;

  std::shared_ptr<BVHModel<BV>> model(new BVHModel<BV>);
  Box<S> box(a, b, c);

  checkAddToEmptyModel(*model, box);
  checkAddToUnfinalizedModel(*model, box);
  checkAddAndFinalizeModel(*model, box);
  checkAddToFinalizedModel(*model, box);

}

template<typename BV>
void testBVHModelFromSphere()
{
  using S = typename BV::S;
  const S r = 1.0;

  Sphere<S> sphere(r);
  std::shared_ptr<BVHModel<BV>> model(new BVHModel<BV>);
  checkAddToEmptyModel(*model, sphere);
  checkAddToUnfinalizedModel(*model, sphere);
  checkAddAndFinalizeModel(*model, sphere);
  checkAddToFinalizedModel(*model, sphere);
}

template<typename BV>
void testBVHModelFromEllipsoid()
{
  using S = typename BV::S;
  const S a = 1.0;
  const S b = 1.0;
  const S c = 1.0;

  Ellipsoid<S> ellipsoid(a, b, c);
  std::shared_ptr<BVHModel<BV>> model(new BVHModel<BV>);

  checkAddToEmptyModel(*model, ellipsoid);
  checkAddToUnfinalizedModel(*model, ellipsoid);
  checkAddAndFinalizeModel(*model, ellipsoid);
  checkAddToFinalizedModel(*model, ellipsoid);
}

template<typename BV>
void testBVHModelFromCylinder()
{
  using S = typename BV::S;
  const S r = 1.0;
  const S h = 1.0;

  Cylinder<S> cylinder(r, h);
  std::shared_ptr<BVHModel<BV>> model(new BVHModel<BV>);

  checkAddToEmptyModel(*model, cylinder);
  checkAddToUnfinalizedModel(*model, cylinder);
  checkAddAndFinalizeModel(*model, cylinder);
  checkAddToFinalizedModel(*model, cylinder);
}

template<typename BV>
void testBVHModelFromCone()
{
 using S = typename BV::S;
  const S r = 1.0;
  const S h = 1.0;

  Cone<S> cone(r, h);
  std::shared_ptr<BVHModel<BV>> model(new BVHModel<BV>);

  checkAddToEmptyModel(*model, cone);
  checkAddToUnfinalizedModel(*model, cone);
  checkAddAndFinalizeModel(*model, cone);
  checkAddToFinalizedModel(*model, cone);
}

template<typename BV>
void testBVHModelFromPrimitives()
{
  testBVHModelFromBox<BV>();
  testBVHModelFromSphere<BV>();
  testBVHModelFromEllipsoid<BV>();
  testBVHModelFromCylinder<BV>();
  testBVHModelFromCone<BV>();
}

GTEST_TEST(FCL_GENERATE_BVH_MODELS, generating_bvh_models_from_primitives)
{
  testBVHModelFromPrimitives<AABB<double>>();
  testBVHModelFromPrimitives<OBB<double>>();
  testBVHModelFromPrimitives<RSS<double>>();
  testBVHModelFromPrimitives<kIOS<double>>();
  testBVHModelFromPrimitives<OBBRSS<double>>();
  testBVHModelFromPrimitives<KDOP<double, 16> >();
  testBVHModelFromPrimitives<KDOP<double, 18> >();
  testBVHModelFromPrimitives<KDOP<double, 24> >();
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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
@brief      This file tests creation of BVHModels from geometric primitives.

@details    It checks proper functionality of those simply by verifying the return value,
            the number of vertices, triangles and the state of the model.

@note       In the process, the provided model will always be finalized.

@warning    Currently, there are no checks that the geometric primitives created are
            actually correct in terms of tesselation resolution, dimensions, radii etc.
            The current state of this test is a very basic "bare bones" for a future more
            adequate test.

@todo       These logic checks should be implemented by someone knowledgeable enough about
            the inner workings of the generateBVHModel() functions.
**/


template<typename BV, typename ShapeType>
void checkNumVerticesAndTris(BVHModel<BV>& model, const ShapeType& shape, uint8_t n, int vertices, int tris)
{  
  using S = typename BV::S;

  GTEST_ASSERT_EQ(model.build_state, BVH_BUILD_STATE_EMPTY);
  GTEST_ASSERT_EQ(model.num_vertices, 0);
  GTEST_ASSERT_EQ(model.num_tris, 0);

  // Add the shape to the model and count vertices and triangles to make sure it has been created
  auto ret = generateBVHModel(model, shape, Transform3<S>::Identity(), n, FinalizeModel::DO);
  EXPECT_EQ(ret, BVH_OK);
  EXPECT_EQ(model.num_vertices, vertices);
  EXPECT_EQ(model.num_tris, tris); 
  EXPECT_EQ(model.build_state, BVH_BUILD_STATE_PROCESSED);
}

// Slightly different for boxes, cannot define numFaces
template<typename BV>
void checkNumVerticesAndTris(BVHModel<BV>& model, const Box<typename BV::S>& shape, int vertices, int tris)
{
  using S = typename BV::S;

  GTEST_ASSERT_EQ(model.build_state, BVH_BUILD_STATE_EMPTY);
  GTEST_ASSERT_EQ(model.num_vertices, 0);
  GTEST_ASSERT_EQ(model.num_tris, 0);

  // Add the shape to the model and count vertices and triangles to make sure it has been created
  auto ret = generateBVHModel(model, shape, Transform3<S>::Identity(), FinalizeModel::DO);
  EXPECT_EQ(ret, BVH_OK);
  EXPECT_EQ(model.num_vertices, vertices);
  EXPECT_EQ(model.num_tris, tris); 
  EXPECT_EQ(model.build_state, BVH_BUILD_STATE_PROCESSED);
}

template<typename BV>
void testBVHModelFromBox()
{
  using S = typename BV::S;

  // Test various box sizes
  for(S a : {0.5, 1.0, 2.0, 3.1}){
    for(S b : {0.5, 1.0, 2.0, 3.1}){
      for(S c : {0.5, 1.0, 2.0, 3.1}){

        std::shared_ptr<BVHModel<BV>> model(new BVHModel<BV>);
        Box<S> box(a, b, c);

        checkNumVerticesAndTris(*model, box, 8, 12);
      }
    }
  }
}

template<typename BV>
void testBVHModelFromSphere()
{
  using S = typename BV::S;

  // Test various radii
  for(S r : {0.5, 1.0, 2.0, 3.1}){
    Sphere<S> sphere(r);

    // Test various resolutions
    for(uint8_t n : {4, 8, 16, 32}){
      S n_low_bound = sqrtf(n / 2.0) * r * r;
      unsigned int ring = ceil(n_low_bound);

      std::shared_ptr<BVHModel<BV>> model(new BVHModel<BV>);
  
      checkNumVerticesAndTris(*model, sphere, n, static_cast<int>(2 + ring * ring), static_cast<int>(2 * ring * ring));
    }
  }
}

template<typename BV>
void testBVHModelFromEllipsoid()
{
  using S = typename BV::S;
  const S p = 1.6075;

  // Test various radii
  for(S a : {0.5, 1.0, 2.0, 3.1}){
    for(S b : {0.5, 1.0, 2.0, 3.1}){
      for(S c : {0.5, 1.0, 2.0, 3.1}){
        Ellipsoid<S> ellipsoid(a, b, c);
        const S& ap = std::pow(a, p);
        const S& bp = std::pow(b, p);
        const S& cp = std::pow(c, p);
        const S ratio = std::pow((ap * bp + bp * cp + cp * ap) / 3.0, 1.0 / p);

        // Test various resolutions
        for(uint8_t n : {4, 8, 16, 32}){
          const S n_low_bound = std::sqrt(n / 2.0) * ratio;
          const unsigned int ring = std::ceil(n_low_bound);
          std::shared_ptr<BVHModel<BV>> model(new BVHModel<BV>);

          checkNumVerticesAndTris(*model, ellipsoid, n, static_cast<int>(2 + ring * ring), static_cast<int>(2 * ring * ring));
        }
      }
    }
  }
}

template<typename BV>
void testBVHModelFromCylinder()
{
  using S = typename BV::S;
  const S pi = constants<S>::pi();

  // Try various resolutions, radii and heights
  for(S r : {0.5, 1.0, 2.0, 3.1}){
    for(S h : {0.5, 1.0, 2.0, 3.1}){
      Cylinder<S> cylinder(r, h);

      for(uint8_t n : {4, 8, 16, 32}){
        unsigned int n_tot = n * r;
        unsigned int h_num = ceil(h / ((pi * 2 / n_tot) * r));

        std::shared_ptr<BVHModel<BV>> model(new BVHModel<BV>);

        checkNumVerticesAndTris(*model, cylinder, n, static_cast<int>(2 + n_tot * (h_num + 1)), static_cast<int>((2 * h_num + 2) * n_tot));
      }
    }
  }
}

template<typename BV>
void testBVHModelFromCone()
{

  using S = typename BV::S;
  const S pi = constants<S>::pi();

  // Try various resolutions, radii and heights
  for(S r : {0.5, 1.0, 2.0, 3.1}){
    for(S h : {0.5, 1.0, 2.0, 3.1}){
      Cone<S> cone(r,h);
  
      for(uint8_t n : {4, 8, 16, 32}){
        unsigned int n_tot = n * r;
        unsigned int h_num = ceil(h / ((pi * 2 / n_tot) * r));

        std::shared_ptr<BVHModel<BV>> model(new BVHModel<BV>);
        checkNumVerticesAndTris(*model, cone, n, static_cast<int>(2 + n_tot * h_num), static_cast<int>(2 * n_tot * h_num));
      }
    }
  }
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

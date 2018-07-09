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

template<typename BV, typename ShapeType>
void checkNumVerticesAndTris(BVHModel<BV>& model, const ShapeType& shape, uint8_t n, int vertices, int tris)
{  
  using S = typename BV::S;

  // Add the shape to the model and count vertices and triangles to make sure it has been created
  generateBVHModel(model, shape, Transform3<S>::Identity(), n, FinalizeModel::DONT);
  EXPECT_EQ(model.num_vertices, vertices);
  EXPECT_EQ(model.num_tris, tris); 
  EXPECT_EQ(model.build_state, BVH_BUILD_STATE_BEGUN);

  // Add another instance of the shape and make sure it was added to the model by counting vertices and tris
  generateBVHModel(model, shape, Transform3<S>(Translation3<S>(Vector3<S>(2.0, 2.0, 2.0))), n);
  EXPECT_EQ(model.num_vertices, 2*vertices);
  EXPECT_EQ(model.num_tris, 2*tris);
  EXPECT_EQ(model.build_state, BVH_BUILD_STATE_PROCESSED);
}

// Slightly different for boxes, cannot define numFaces
template<typename BV>
void checkNumVerticesAndTris(BVHModel<BV>& model, const Box<typename BV::S>& shape, int vertices, int tris)
{
  using S = typename BV::S;
  auto ret  = generateBVHModel(model, shape, Transform3<S>::Identity(), FinalizeModel::DONT);
  EXPECT_EQ(ret, BVH_OK);
  EXPECT_EQ(model.num_vertices, vertices);
  EXPECT_EQ(model.num_tris, tris); 
  EXPECT_EQ(model.build_state, BVH_BUILD_STATE_BEGUN);
  ret = generateBVHModel(model, shape, Transform3<S>(Translation3<S>(Vector3<S>(2.0, 2.0, 2.0))));
  EXPECT_EQ(ret, BVH_OK);
  EXPECT_EQ(model.num_vertices, 2*vertices);
  EXPECT_EQ(model.num_tris, 2*tris);
  EXPECT_EQ(model.build_state, BVH_BUILD_STATE_PROCESSED);
}

template<typename BV, typename ShapeType>
void checkAddToFinishedModel(BVHModel<BV>& model, const ShapeType& shape, uint8_t n)
{
  using S = typename BV::S;
  EXPECT_EQ(model.build_state, BVH_BUILD_STATE_PROCESSED);
  auto ret = generateBVHModel(model, shape, Transform3<S>::Identity(), n, FinalizeModel::DONT);
  EXPECT_EQ(ret, BVH_ERR_BUILD_OUT_OF_SEQUENCE);
}

template<typename BV>
void checkAddToFinishedModel(BVHModel<BV>& model, const Box<typename BV::S>& shape)
{
  using S = typename BV::S;
  EXPECT_EQ(model.build_state, BVH_BUILD_STATE_PROCESSED);
  auto ret = generateBVHModel(model, shape, Transform3<S>::Identity(), FinalizeModel::DONT);
  EXPECT_EQ(ret, BVH_ERR_BUILD_OUT_OF_SEQUENCE);
}

template<typename BV>
void testBVHModelFromBox()
{
  using S = typename BV::S;

  // Test various box sizes
  for(S a = 0.5; a <= 2.1; a += 0.8){
    for(S b = 0.5; b <= 2.1; b += 0.8){
      for(S c = 0.5; c <= 2.1; c += 0.8){

        std::shared_ptr<BVHModel<BV>> model(new BVHModel<BV>);
        Box<S> box(a, b, c);

        checkNumVerticesAndTris(*model, box, 8, 12);
        checkAddToFinishedModel(*model, box);
      }
    }
  }
}

template<typename BV>
void testBVHModelFromSphere()
{
  using S = typename BV::S;

  // Test various radii
  for(S r = 0.5; r <= 2.1; r += 0.8){
    Sphere<S> sphere(r);

    // Test various resolutions
    for(uint8_t n = 4; n <= 32; n += 8){
      S n_low_bound = sqrtf(n / 2.0) * r * r;
      unsigned int ring = ceil(n_low_bound);

      std::shared_ptr<BVHModel<BV>> model(new BVHModel<BV>);
  
      checkNumVerticesAndTris(*model, sphere, n, static_cast<int>(2 + ring * ring), static_cast<int>(2 * ring * ring));
      checkAddToFinishedModel(*model, sphere, n);
    }
  }
}

template<typename BV>
void testBVHModelFromEllipsoid()
{
  using S = typename BV::S;
  const S p = 1.6075;

  // Test various radii
  for(S a = 0.5; a <= 2.1; a += 0.8){
    for(S b = 0.5; b <= 2.1; b += 0.8){
      for(S c = 0.5; c <= 2.1; c += 0.8){
        Ellipsoid<S> ellipsoid(a, b, c);
        const S& ap = std::pow(a, p);
        const S& bp = std::pow(b, p);
        const S& cp = std::pow(c, p);
        const S ratio = std::pow((ap * bp + bp * cp + cp * ap) / 3.0, 1.0 / p);

        // Test various resolutions
        for(uint8_t n = 4; n <= 32; n += 8){
          const S n_low_bound = std::sqrt(n / 2.0) * ratio;
          const unsigned int ring = std::ceil(n_low_bound);
          std::shared_ptr<BVHModel<BV>> model(new BVHModel<BV>);

          checkNumVerticesAndTris(*model, ellipsoid, n, static_cast<int>(2 + ring * ring), static_cast<int>(2 * ring * ring));
          checkAddToFinishedModel(*model, ellipsoid, n);
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
  for(S r = 0.5; r <= 2.1; r += 0.8){
    for(S h = 0.5; h <= 2.1; h += 0.8){
      Cylinder<S> cylinder(r, h);

      for(uint8_t n = 4; n <= 32; n += 8){
        unsigned int n_tot = n * r;
        unsigned int h_num = ceil(h / ((pi * 2 / n_tot) * r));

        std::shared_ptr<BVHModel<BV>> model(new BVHModel<BV>);

        checkNumVerticesAndTris(*model, cylinder, n, static_cast<int>(2 + n_tot * (h_num + 1)), static_cast<int>((2 * h_num + 2) * n_tot));
        checkAddToFinishedModel(*model, cylinder, n);
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
  for(S r = 0.5; r <= 2.1; r += 0.8){
    for(S h = 0.5; h <= 2.1; h += 0.8){
      Cone<S> cone(r,h);
  
      for(uint8_t n = 4; n <= 32; n += 8){
        unsigned int n_tot = n * r;
        unsigned int h_num = ceil(h / ((pi * 2 / n_tot) * r));

        std::shared_ptr<BVHModel<BV>> model(new BVHModel<BV>);
        checkNumVerticesAndTris(*model, cone, n, static_cast<int>(2 + n_tot * h_num), static_cast<int>(2 * n_tot * h_num));
        checkAddToFinishedModel(*model, cone, n);
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

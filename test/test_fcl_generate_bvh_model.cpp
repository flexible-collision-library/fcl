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

template<typename BV>
void testBVHModelFromBox()
{
  using S = typename BV::S;

  std::shared_ptr<BVHModel<BV> > model(new BVHModel<BV>);
  Box<S> box(1.0, 1.0, 1.0);

  generateBVHModel(*model, box, Transform3<S>::Identity(), FinalizeModel::DONT_FINALIZE);
  EXPECT_EQ(model->num_vertices, 8);
  EXPECT_EQ(model->num_tris, 12);
  EXPECT_EQ(model->build_state, BVH_BUILD_STATE_BEGUN);

  generateBVHModel(*model, box, Transform3<S>(Translation3<S>(Vector3<S>(2.0, 2.0, 2.0))));
  EXPECT_EQ(model->num_vertices, 16);
  EXPECT_EQ(model->num_tris, 24);
  EXPECT_EQ(model->build_state, BVH_BUILD_STATE_PROCESSED);

}

template<typename BV>
void testBVHModelFromSphere()
{
  using S = typename BV::S;

  std::shared_ptr<BVHModel<BV> > model(new BVHModel<BV>);
  Sphere<S> sphere(1.0);

  // Testing the overload with num_faces defined ends up in a call to both
  generateBVHModel(*model, sphere, Transform3<S>::Identity(), 32, FinalizeModel::DONT_FINALIZE);
  EXPECT_EQ(model->num_vertices, 18);
  EXPECT_EQ(model->num_tris, 32);
  EXPECT_EQ(model->build_state, BVH_BUILD_STATE_BEGUN);

  generateBVHModel(*model, sphere, Transform3<S>(Translation3<S>(Vector3<S>(2.0, 2.0, 2.0))), 32);
  EXPECT_EQ(model->num_vertices, 36);
  EXPECT_EQ(model->num_tris, 64);
  EXPECT_EQ(model->build_state, BVH_BUILD_STATE_PROCESSED);

}

template<typename BV>
void testBVHModelFromEllipsoid()
{
  using S = typename BV::S;

  std::shared_ptr<BVHModel<BV> > model(new BVHModel<BV>);
  Ellipsoid<S> ellipsoid(1.0, 1.0, 1.0);

  // Testing the overload with num_faces defined ends up in a call to both
  generateBVHModel(*model, ellipsoid, Transform3<S>::Identity(), 32, FinalizeModel::DONT_FINALIZE);
  EXPECT_EQ(model->num_vertices, 18);
  EXPECT_EQ(model->num_tris, 32);
  EXPECT_EQ(model->build_state, BVH_BUILD_STATE_BEGUN);

  generateBVHModel(*model, ellipsoid, Transform3<S>(Translation3<S>(Vector3<S>(2.0, 2.0, 2.0))), 32);
  EXPECT_EQ(model->num_vertices, 36);
  EXPECT_EQ(model->num_tris, 64);
  EXPECT_EQ(model->build_state, BVH_BUILD_STATE_PROCESSED);

}

template<typename BV>
void testBVHModelFromCylinder()
{
  using S = typename BV::S;
  const S pi = constants<S>::pi();

  // Try various resolutions, radii and heights
  for(uint8_t n = 4; n <= 32; n+=3){
    for(S r = 0.5; r <= 5.0; r+=0.8){
      for(S h = 0.5; h <= 5.0; h+=0.8){
        unsigned int n_tot = n * r;
        unsigned int h_num = ceil(h / ((pi * 2 / n_tot) * r));

        std::shared_ptr<BVHModel<BV> > model(new BVHModel<BV>);
        Cylinder<S> cylinder(r, h);

        // Testing the overload with num_faces defined ends up in a call to both
        generateBVHModel(*model, cylinder, Transform3<S>::Identity(), n, FinalizeModel::DONT_FINALIZE);

     //   EXPECT_EQ(model->num_vertices, 2+n_tot*h_num);
        EXPECT_EQ(model->num_tris, (2*h_num+2)*n_tot);
     //   EXPECT_EQ(model->build_state, BVH_BUILD_STATE_BEGUN);

        generateBVHModel(*model, cylinder, Transform3<S>(Translation3<S>(Vector3<S>(2.0, 2.0, 2.0))), 8);
     //   EXPECT_EQ(model->num_vertices, 52);
     //   EXPECT_EQ(model->num_tris, 96);
     //   EXPECT_EQ(model->build_state, BVH_BUILD_STATE_PROCESSED);
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
  for(uint8_t n = 4; n <= 32; n+=3){
    for(S r = 0.5; r <= 5.0; r+=0.8){
      for(S h = 0.5; h <= 5.0; h+=0.8){
        unsigned int n_tot = n * r;
        unsigned int h_num = ceil(h / ((pi * 2 / n_tot) * r));

        std::shared_ptr<BVHModel<BV> > model(new BVHModel<BV>);
        Cone<S> cone(r,h);

        // Testing the overload with num_faces defined ends up in a call to both
        generateBVHModel(*model, cone, Transform3<S>::Identity(), n, FinalizeModel::DONT_FINALIZE);
        EXPECT_EQ(model->num_vertices, 2+n_tot*h_num);
        EXPECT_EQ(model->num_tris, 2*n_tot*h_num); 
        EXPECT_EQ(model->build_state, BVH_BUILD_STATE_BEGUN);
 
        generateBVHModel(*model, cone, Transform3<S>(Translation3<S>(Vector3<S>(2.0, 2.0, 2.0))), n);
        EXPECT_EQ(model->num_vertices, 2*(2+n_tot*h_num));
        EXPECT_EQ(model->num_tris, 4*n_tot*h_num);
        EXPECT_EQ(model->build_state, BVH_BUILD_STATE_PROCESSED);
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

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
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

/** @author Jia Pan */

#include <gtest/gtest.h>

#include "fcl/narrowphase/detail/traversal/collision_node.h"
#include "test_fcl_utility.h"
#include "eigen_matrix_compare.h"
#include "fcl_resources/config.h"

// TODO(SeanCurtis-TRI): A file called `test_fcl_distance.cpp` should *not* have
// collision tests.

using namespace fcl;

bool verbose = false;

template <typename S>
S DELTA() { return 0.001; }

template<typename BV>
void distance_Test(const Transform3<typename BV::S>& tf,
                   const std::vector<Vector3<typename BV::S>>& vertices1, const std::vector<Triangle>& triangles1,
                   const std::vector<Vector3<typename BV::S>>& vertices2, const std::vector<Triangle>& triangles2, detail::SplitMethodType split_method,
                   int qsize,
                   test::DistanceRes<typename BV::S>& distance_result,
                   bool verbose = true);

template <typename S>
bool collide_Test_OBB(const Transform3<S>& tf,
                      const std::vector<Vector3<S>>& vertices1, const std::vector<Triangle>& triangles1,
                      const std::vector<Vector3<S>>& vertices2, const std::vector<Triangle>& triangles2, detail::SplitMethodType split_method, bool verbose);

template<typename BV, typename TraversalNode>
void distance_Test_Oriented(const Transform3<typename BV::S>& tf,
                            const std::vector<Vector3<typename BV::S>>& vertices1, const std::vector<Triangle>& triangles1,
                            const std::vector<Vector3<typename BV::S>>& vertices2, const std::vector<Triangle>& triangles2, detail::SplitMethodType split_method,
                            int qsize,
                            test::DistanceRes<typename BV::S>& distance_result,
                            bool verbose = true);

template <typename S>
bool nearlyEqual(const Vector3<S>& a, const Vector3<S>& b)
{
  if(fabs(a[0] - b[0]) > DELTA<S>()) return false;
  if(fabs(a[1] - b[1]) > DELTA<S>()) return false;
  if(fabs(a[2] - b[2]) > DELTA<S>()) return false;
  return true;
}

template <typename S>
void test_mesh_distance()
{
  std::vector<Vector3<S>> p1, p2;
  std::vector<Triangle> t1, t2;

  test::loadOBJFile(TEST_RESOURCES_DIR"/env.obj", p1, t1);
  test::loadOBJFile(TEST_RESOURCES_DIR"/rob.obj", p2, t2);

  aligned_vector<Transform3<S>> transforms; // t0
  S extents[] = {-3000, -3000, 0, 3000, 3000, 3000};
#ifdef NDEBUG
  std::size_t n = 10;
#else
  std::size_t n = 1;
#endif

  test::generateRandomTransforms(extents, transforms, n);

  double dis_time = 0;
  double col_time = 0;

  test::DistanceRes<S> res, res_now;
  for(std::size_t i = 0; i < transforms.size(); ++i)
  {
    test::Timer timer_col;
    timer_col.start();
    collide_Test_OBB(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, verbose);
    timer_col.stop();
    col_time += timer_col.getElapsedTimeInSec();

    test::Timer timer_dist;
    timer_dist.start();
    distance_Test_Oriented<RSS<S>, detail::MeshDistanceTraversalNodeRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, 2, res, verbose);
    timer_dist.stop();
    dis_time += timer_dist.getElapsedTimeInSec();

    distance_Test_Oriented<RSS<S>, detail::MeshDistanceTraversalNodeRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<RSS<S>, detail::MeshDistanceTraversalNodeRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<RSS<S>, detail::MeshDistanceTraversalNodeRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<RSS<S>, detail::MeshDistanceTraversalNodeRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<RSS<S>, detail::MeshDistanceTraversalNodeRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<kIOS<S>, detail::MeshDistanceTraversalNodekIOS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<kIOS<S>, detail::MeshDistanceTraversalNodekIOS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<kIOS<S>, detail::MeshDistanceTraversalNodekIOS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<kIOS<S>, detail::MeshDistanceTraversalNodekIOS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<kIOS<S>, detail::MeshDistanceTraversalNodekIOS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<kIOS<S>, detail::MeshDistanceTraversalNodekIOS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<OBBRSS<S>, detail::MeshDistanceTraversalNodeOBBRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<OBBRSS<S>, detail::MeshDistanceTraversalNodeOBBRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<OBBRSS<S>, detail::MeshDistanceTraversalNodeOBBRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<OBBRSS<S>, detail::MeshDistanceTraversalNodeOBBRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<OBBRSS<S>, detail::MeshDistanceTraversalNodeOBBRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<OBBRSS<S>, detail::MeshDistanceTraversalNodeOBBRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));



    distance_Test<RSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<RSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<RSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<RSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<RSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<RSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<kIOS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<kIOS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<kIOS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<kIOS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<kIOS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<kIOS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));


    distance_Test<OBBRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<OBBRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<OBBRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, 2, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<OBBRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<OBBRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

    distance_Test<OBBRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, 20, res_now, verbose);

    EXPECT_TRUE(fabs(res.distance - res_now.distance) < DELTA<S>());
    EXPECT_TRUE(fabs(res.distance) < DELTA<S>() || (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) && nearlyEqual(res.p2, res_now.p2)));

  }

  std::cout << "distance timing: " << dis_time << " sec" << std::endl;
  std::cout << "collision timing: " << col_time << " sec" << std::endl;
}

GTEST_TEST(FCL_DISTANCE, mesh_distance)
{
//  test_mesh_distance<float>();
  test_mesh_distance<double>();
}

template <typename S>
void NearestPointFromDegenerateSimplex() {
  // Tests a historical bug. In certain configurations, the distance query
  // would terminate with a degenerate 3-simplex; the triangle was actually a
  // line segment. As a result, nearest points were populated with NaN values.
  // See https://github.com/flexible-collision-library/fcl/issues/293 for
  // more discussion.
  // This test is only relevant if box-box distance is computed via GJK. We
  // intentionally short-circuit the mechanism for dispatching custom methods
  // to guarantee GJK is evaluated on these two boxes.
  DistanceResult<S> result;
  DistanceRequest<S> request;

  // These values were extracted from a real-world scenario that produced NaNs.
  std::shared_ptr<CollisionGeometry<S>> box_geometry_1(
      new Box<S>(2.750000, 6.000000, 0.050000));
  std::shared_ptr<CollisionGeometry<S>> box_geometry_2(
      new Box<S>(0.424000, 0.150000, 0.168600));
  CollisionObject<S> box_object_1(
      box_geometry_1, Eigen::Quaterniond(1, 0, 0, 0).matrix(),
      Eigen::Vector3d(1.625000, 0.000000, 0.500000));
  CollisionObject<S> box_object_2(
      box_geometry_2,
      Eigen::Quaterniond(0.672811, 0.340674, 0.155066, 0.638138)
          .normalized()
          .matrix(),
      Eigen::Vector3d(0.192074, -0.277870, 0.273546));

  // Direct invocation.
  // NOTE: This code is basically lifted from ShapeDistanceLibccdImpl::run() in
  // gjk_solver_libbd-inl.h.
  Box<S>* box1 = static_cast<Box<S>*>(box_geometry_1.get());
  Box<S>* box2 = static_cast<Box<S>*>(box_geometry_2.get());
  detail::GJKSolver_libccd<S> solver;
  void* o1 = detail::GJKInitializer<S, Box<S>>::createGJKObject(
      *box1, box_object_1.getTransform());
  void* o2 = detail::GJKInitializer<S, Box<S>>::createGJKObject(
      *box2, box_object_2.getTransform());
  detail::GJKDistance(
      o1, detail::GJKInitializer<S, Box<S>>::getSupportFunction(), o2,
      detail::GJKInitializer<S, Box<S>>::getSupportFunction(),
      solver.max_distance_iterations, request.distance_tolerance,
      &result.min_distance, &result.nearest_points[0],
      &result.nearest_points[1]);

  detail::GJKInitializer<S, Box<S>>::deleteGJKObject(o1);
  detail::GJKInitializer<S, Box<S>>::deleteGJKObject(o2);

  // These hard-coded values have been previously computed and visually
  // inspected and considered to be the ground truth for this very specific
  // test configuration.
  S expected_dist{0.053516162824549};
  // The "nearest" points (N1 and N2) measured and expressed in box 1's and
  // box 2's frames (B1 and B2, respectively).
  const Vector3<S> expected_p_B1N1{-1.375, -0.098881502700918666,
                                   -0.025000000000000022};
  const Vector3<S> expected_p_B2N2{0.21199965773384655, 0.074999692703297122,
                                   0.084299993303443954};
  // The nearest points in the world frame.
  const Vector3<S> expected_p_WN1 =
      box_object_1.getTransform() * expected_p_B1N1;
  const Vector3<S> expected_p_WN2 =
      box_object_2.getTransform() * expected_p_B2N2;
  EXPECT_TRUE(CompareMatrices(result.nearest_points[0], expected_p_WN1,
                              DELTA<S>(), MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(result.nearest_points[1], expected_p_WN2,
                              DELTA<S>(), MatrixCompareType::absolute));
  EXPECT_NEAR(expected_dist, result.min_distance,
              constants<ccd_real_t>::eps_78());
}

GTEST_TEST(FCL_DISTANCE, NearestPointFromDegenerateSimplex) {
  NearestPointFromDegenerateSimplex<double>();
}

template<typename BV, typename TraversalNode>
void distance_Test_Oriented(const Transform3<typename BV::S>& tf,
                            const std::vector<Vector3<typename BV::S>>& vertices1, const std::vector<Triangle>& triangles1,
                            const std::vector<Vector3<typename BV::S>>& vertices2, const std::vector<Triangle>& triangles2, detail::SplitMethodType split_method,
                            int qsize,
                            test::DistanceRes<typename BV::S>& distance_result,
                            bool verbose)
{
  using S = typename BV::S;

  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.reset(new detail::BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new detail::BVSplitter<BV>(split_method));


  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  DistanceResult<S> local_result;
  TraversalNode node;
  if(!initialize(node, (const BVHModel<BV>&)m1, tf, (const BVHModel<BV>&)m2, Transform3<S>::Identity(), DistanceRequest<S>(true), local_result))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;

  distance(&node, nullptr, qsize);

  // points are in local coordinate, to global coordinate
  Vector3<S> p1 = local_result.nearest_points[0];
  Vector3<S> p2 = local_result.nearest_points[1];


  distance_result.distance = local_result.min_distance;
  distance_result.p1 = p1;
  distance_result.p2 = p2;

  if(verbose)
  {
    std::cout << "distance " << local_result.min_distance << std::endl;

    std::cout << p1[0] << " " << p1[1] << " " << p1[2] << std::endl;
    std::cout << p2[0] << " " << p2[1] << " " << p2[2] << std::endl;
    std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
  }
}

template<typename BV>
void distance_Test(const Transform3<typename BV::S>& tf,
                   const std::vector<Vector3<typename BV::S>>& vertices1, const std::vector<Triangle>& triangles1,
                   const std::vector<Vector3<typename BV::S>>& vertices2, const std::vector<Triangle>& triangles2, detail::SplitMethodType split_method,
                   int qsize,
                   test::DistanceRes<typename BV::S>& distance_result,
                   bool verbose)
{
  using S = typename BV::S;

  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.reset(new detail::BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new detail::BVSplitter<BV>(split_method));


  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  Transform3<S> pose1(tf);
  Transform3<S> pose2 = Transform3<S>::Identity();

  DistanceResult<S> local_result;
  detail::MeshDistanceTraversalNode<BV> node;

  if(!detail::initialize<BV>(node, m1, pose1, m2, pose2, DistanceRequest<S>(true), local_result))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;

  distance(&node, nullptr, qsize);

  distance_result.distance = local_result.min_distance;
  distance_result.p1 = local_result.nearest_points[0];
  distance_result.p2 = local_result.nearest_points[1];

  if(verbose)
  {
    std::cout << "distance " << local_result.min_distance << std::endl;

    std::cout << local_result.nearest_points[0][0] << " " << local_result.nearest_points[0][1] << " " << local_result.nearest_points[0][2] << std::endl;
    std::cout << local_result.nearest_points[1][0] << " " << local_result.nearest_points[1][1] << " " << local_result.nearest_points[1][2] << std::endl;
    std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
  }
}

template <typename S>
bool collide_Test_OBB(const Transform3<S>& tf,
                      const std::vector<Vector3<S>>& vertices1, const std::vector<Triangle>& triangles1,
                      const std::vector<Vector3<S>>& vertices2, const std::vector<Triangle>& triangles2, detail::SplitMethodType split_method, bool verbose)
{
  BVHModel<OBB<S>> m1;
  BVHModel<OBB<S>> m2;
  m1.bv_splitter.reset(new detail::BVSplitter<OBB<S>>(split_method));
  m2.bv_splitter.reset(new detail::BVSplitter<OBB<S>>(split_method));

  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  CollisionResult<S> local_result;
  detail::MeshCollisionTraversalNodeOBB<S> node;
  if(!detail::initialize(node, (const BVHModel<OBB<S>>&)m1, tf, (const BVHModel<OBB<S>>&)m2, Transform3<S>::Identity(),
                 CollisionRequest<S>(), local_result))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;

  collide(&node);

  if(local_result.numContacts() > 0)
    return true;
  else
    return false;
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

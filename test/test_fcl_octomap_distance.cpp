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

/** \author Jia Pan */

#include <gtest/gtest.h>

#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_nodes.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"
#include "test_fcl_utility.h"
#include "fcl_resources/config.h"

using namespace fcl;

/// @brief Octomap distance with an environment with 3 * env_size objects
template <typename S>
void octomap_distance_test(S env_scale, std::size_t env_size, bool use_mesh, bool use_mesh_octomap, double resolution = 0.1);

template<typename BV>
void octomap_distance_test_BVH(std::size_t n, double resolution = 0.1);

template <typename S>
void test_octomap_distance()
{
#ifdef NDEBUG
  octomap_distance_test<S>(200, 100, false, false);
  octomap_distance_test<S>(200, 1000, false, false);
#else
  octomap_distance_test<S>(200, 2, false, false, 1.0);
  octomap_distance_test<S>(200, 10, false, false, 1.0);
#endif
}

GTEST_TEST(FCL_OCTOMAP, test_octomap_distance)
{
//  test_octomap_distance<float>();
  test_octomap_distance<double>();
}

template <typename S>
void test_octomap_distance_mesh()
{
#ifdef NDEBUG
  octomap_distance_test<S>(200, 100, true, true);
  octomap_distance_test<S>(200, 1000, true, true);
#else
  octomap_distance_test<S>(200, 2, true, true, 1.0);
  octomap_distance_test<S>(200, 5, true, true, 1.0);
#endif
}

GTEST_TEST(FCL_OCTOMAP, test_octomap_distance_mesh)
{
//  test_octomap_distance_mesh<float>();
  test_octomap_distance_mesh<double>();
}

template <typename S>
void test_octomap_distance_mesh_octomap_box()
{
#ifdef NDEBUG
  octomap_distance_test<S>(200, 100, true, false);
  octomap_distance_test<S>(200, 1000, true, false);
#else
  octomap_distance_test<S>(200, 2, true, false, 1.0);
  octomap_distance_test<S>(200, 5, true, false, 1.0);
#endif
}

GTEST_TEST(FCL_OCTOMAP, test_octomap_distance_mesh_octomap_box)
{
//  test_octomap_distance_mesh_octomap_box<float>();
  test_octomap_distance_mesh_octomap_box<double>();
}

template <typename S>
void test_octomap_bvh_rss_d_distance_rss()
{
#ifdef NDEBUG
  octomap_distance_test_BVH<RSS<S>>(5);
#else
  octomap_distance_test_BVH<RSS<S>>(1, 1.0);
#endif
}

GTEST_TEST(FCL_OCTOMAP, test_octomap_bvh_rss_d_distance_rss)
{
//  test_octomap_bvh_rss_d_distance_rss<float>();
  test_octomap_bvh_rss_d_distance_rss<double>();
}

template <typename S>
void test_octomap_bvh_obb_d_distance_obb()
{
#ifdef NDEBUG
  octomap_distance_test_BVH<OBBRSS<S>>(5);
#else
  octomap_distance_test_BVH<OBBRSS<S>>(1, 1.0);
#endif
}

GTEST_TEST(FCL_OCTOMAP, test_octomap_bvh_obb_d_distance_obb)
{
//  test_octomap_bvh_obb_d_distance_obb<float>();
  test_octomap_bvh_obb_d_distance_obb<double>();
}

template <typename S>
void test_octomap_bvh_kios_d_distance_kios()
{
#ifdef NDEBUG
  octomap_distance_test_BVH<kIOS<S>>(5);
#else
  octomap_distance_test_BVH<kIOS<S>>(1, 1.0);
#endif
}

GTEST_TEST(FCL_OCTOMAP, test_octomap_bvh_kios_d_distance_kios)
{
//  test_octomap_bvh_kios_d_distance_kios<float>();
  test_octomap_bvh_kios_d_distance_kios<double>();
}

template<typename BV>
void octomap_distance_test_BVH(std::size_t n, double resolution)
{
  using S = typename BV::S;

  std::vector<Vector3<S>> p1;
  std::vector<Triangle> t1;

  loadOBJFile(TEST_RESOURCES_DIR"/env.obj", p1, t1);

  BVHModel<BV>* m1 = new BVHModel<BV>();
  std::shared_ptr<CollisionGeometry<S>> m1_ptr(m1);

  m1->beginModel();
  m1->addSubModel(p1, t1);
  m1->endModel();

  OcTree<S>* tree = new OcTree<S>(std::shared_ptr<octomap::OcTree>(generateOcTree(resolution)));
  std::shared_ptr<CollisionGeometry<S>> tree_ptr(tree);

  Eigen::aligned_vector<Transform3<S>> transforms;
  S extents[] = {-10, -10, 10, 10, 10, 10};

  generateRandomTransforms(extents, transforms, n);

  for(std::size_t i = 0; i < n; ++i)
  {
    Transform3<S> tf(transforms[i]);

    CollisionObject<S> obj1(m1_ptr, tf);
    CollisionObject<S> obj2(tree_ptr, tf);
    DistanceData<S> cdata;
    S dist1 = std::numeric_limits<S>::max();
    defaultDistanceFunction(&obj1, &obj2, &cdata, dist1);


    std::vector<CollisionObject<S>*> boxes;
    generateBoxesFromOctomap(boxes, *tree);
    for(std::size_t j = 0; j < boxes.size(); ++j)
      boxes[j]->setTransform(tf * boxes[j]->getTransform());

    DynamicAABBTreeCollisionManager<S>* manager = new DynamicAABBTreeCollisionManager<S>();
    manager->registerObjects(boxes);
    manager->setup();

    DistanceData<S> cdata2;
    manager->distance(&obj1, &cdata2, defaultDistanceFunction);
    S dist2 = cdata2.result.min_distance;

    for(std::size_t j = 0; j < boxes.size(); ++j)
      delete boxes[j];
    delete manager;

    std::cout << dist1 << " " << dist2 << std::endl;
    EXPECT_TRUE(std::abs(dist1 - dist2) < 0.001);
  }
}

template <typename S>
void octomap_distance_test(S env_scale, std::size_t env_size, bool use_mesh, bool use_mesh_octomap, double resolution)
{
  // srand(1);
  std::vector<CollisionObject<S>*> env;
  if(use_mesh)
    generateEnvironmentsMesh(env, env_scale, env_size);
  else
    generateEnvironments(env, env_scale, env_size);

  OcTree<S>* tree = new OcTree<S>(std::shared_ptr<const octomap::OcTree>(generateOcTree(resolution)));
  CollisionObject<S> tree_obj((std::shared_ptr<CollisionGeometry<S>>(tree)));

  DynamicAABBTreeCollisionManager<S>* manager = new DynamicAABBTreeCollisionManager<S>();
  manager->registerObjects(env);
  manager->setup();

  DistanceData<S> cdata;
  TStruct t1;
  Timer timer1;
  timer1.start();
  manager->octree_as_geometry_collide = false;
  manager->octree_as_geometry_distance = false;
  manager->distance(&tree_obj, &cdata, defaultDistanceFunction);
  timer1.stop();
  t1.push_back(timer1.getElapsedTime());


  DistanceData<S> cdata3;
  TStruct t3;
  Timer timer3;
  timer3.start();
  manager->octree_as_geometry_collide = true;
  manager->octree_as_geometry_distance = true;
  manager->distance(&tree_obj, &cdata3, defaultDistanceFunction);
  timer3.stop();
  t3.push_back(timer3.getElapsedTime());


  TStruct t2;
  Timer timer2;
  timer2.start();
  std::vector<CollisionObject<S>*> boxes;
  if(use_mesh_octomap)
    generateBoxesFromOctomapMesh(boxes, *tree);
  else
    generateBoxesFromOctomap(boxes, *tree);
  timer2.stop();
  t2.push_back(timer2.getElapsedTime());

  timer2.start();
  DynamicAABBTreeCollisionManager<S>* manager2 = new DynamicAABBTreeCollisionManager<S>();
  manager2->registerObjects(boxes);
  manager2->setup();
  timer2.stop();
  t2.push_back(timer2.getElapsedTime());


  DistanceData<S> cdata2;

  timer2.start();
  manager->distance(manager2, &cdata2, defaultDistanceFunction);
  timer2.stop();
  t2.push_back(timer2.getElapsedTime());

  std::cout << cdata.result.min_distance << " " << cdata3.result.min_distance << " " << cdata2.result.min_distance << std::endl;

  if(cdata.result.min_distance < 0)
    EXPECT_TRUE(cdata2.result.min_distance <= 0);
  else
    EXPECT_TRUE(std::abs(cdata.result.min_distance - cdata2.result.min_distance) < 1e-3);

  delete manager;
  delete manager2;
  for(size_t i = 0; i < boxes.size(); ++i)
    delete boxes[i];


  std::cout << "1) octomap overall time: " << t1.overall_time << std::endl;
  std::cout << "1') octomap overall time (as geometry): " << t3.overall_time << std::endl;
  std::cout << "2) boxes overall time: " << t2.overall_time << std::endl;
  std::cout << "  a) to boxes: " << t2.records[0] << std::endl;
  std::cout << "  b) structure init: " << t2.records[1] << std::endl;
  std::cout << "  c) distance: " << t2.records[2] << std::endl;
  std::cout << "Note: octomap may need more collides when using mesh, because octomap collision uses box primitive inside" << std::endl;
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

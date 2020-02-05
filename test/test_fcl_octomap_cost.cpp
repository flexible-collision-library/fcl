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

#include "fcl/config.h"
#include "fcl/geometry/octree/octree.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/broadphase/broadphase_bruteforce.h"
#include "fcl/broadphase/broadphase_spatialhash.h"
#include "fcl/broadphase/broadphase_SaP.h"
#include "fcl/broadphase/broadphase_SSaP.h"
#include "fcl/broadphase/broadphase_interval_tree.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree_array.h"
#include "fcl/broadphase/default_broadphase_callbacks.h"
#include "fcl/geometry/geometric_shape_to_BVH_model.h"
#include "test_fcl_utility.h"
#include "fcl_resources/config.h"

using namespace fcl;

/// @brief Octomap collision with an environment with 3 * env_size objects, compute cost
template <typename S>
void octomap_cost_test(S env_scale, std::size_t env_size, std::size_t num_max_cost_sources, bool use_mesh, bool use_mesh_octomap, double resolution = 0.1);

template <typename S>
void test_octomap_cost()
{
#ifdef NDEBUG
  octomap_cost_test<S>(200, 100, 10, false, false);
  octomap_cost_test<S>(200, 1000, 10, false, false);
#else
  octomap_cost_test<S>(200, 10, 10, false, false, 0.1);
  octomap_cost_test<S>(200, 100, 10, false, false, 0.1);
#endif
}

GTEST_TEST(FCL_OCTOMAP, test_octomap_cost)
{
//  test_octomap_cost<float>();
  test_octomap_cost<double>();
}

template <typename S>
void test_octomap_cost_mesh()
{
#ifdef NDEBUG
  octomap_cost_test<S>(200, 100, 10, true, false);
  octomap_cost_test<S>(200, 1000, 10, true, false);
#else
  octomap_cost_test<S>(200, 2, 4, true, false, 1.0);
  octomap_cost_test<S>(200, 5, 4, true, false, 1.0);
#endif
}

GTEST_TEST(FCL_OCTOMAP, test_octomap_cost_mesh)
{
//  test_octomap_cost_mesh<float>();
  test_octomap_cost_mesh<double>();
}

template <typename S>
void octomap_cost_test(S env_scale, std::size_t env_size, std::size_t num_max_cost_sources, bool use_mesh, bool use_mesh_octomap, double resolution)
{
  std::vector<CollisionObject<S>*> env;
  if(use_mesh)
    test::generateEnvironmentsMesh(env, env_scale, env_size);
  else
    test::generateEnvironments(env, env_scale, env_size);

  OcTree<S>* tree = new OcTree<S>(std::shared_ptr<const octomap::OcTree>(test::generateOcTree(resolution)));
  CollisionObject<S> tree_obj((std::shared_ptr<CollisionGeometry<S>>(tree)));

  DynamicAABBTreeCollisionManager<S>* manager = new DynamicAABBTreeCollisionManager<S>();
  manager->registerObjects(env);
  manager->setup();

  DefaultCollisionData<S> cdata;
  cdata.request.enable_cost = true;
  cdata.request.num_max_cost_sources = num_max_cost_sources;

  test::TStruct t1;
  test::Timer timer1;
  timer1.start();
  manager->octree_as_geometry_collide = false;
  manager->octree_as_geometry_distance = false;
  manager->collide(&tree_obj, &cdata, DefaultCollisionFunction);
  timer1.stop();
  t1.push_back(timer1.getElapsedTime());

  DefaultCollisionData<S> cdata3;
  cdata3.request.enable_cost = true;
  cdata3.request.num_max_cost_sources = num_max_cost_sources;

  test::TStruct t3;
  test::Timer timer3;
  timer3.start();
  manager->octree_as_geometry_collide = true;
  manager->octree_as_geometry_distance = true;
  manager->collide(&tree_obj, &cdata3, DefaultCollisionFunction);
  timer3.stop();
  t3.push_back(timer3.getElapsedTime());

  test::TStruct t2;
  test::Timer timer2;
  timer2.start();
  std::vector<CollisionObject<S>*> boxes;
  if(use_mesh_octomap)
    test::generateBoxesFromOctomapMesh(boxes, *tree);
  else
    test::generateBoxesFromOctomap(boxes, *tree);
  timer2.stop();
  t2.push_back(timer2.getElapsedTime());

  timer2.start();
  DynamicAABBTreeCollisionManager<S>* manager2 = new DynamicAABBTreeCollisionManager<S>();
  manager2->registerObjects(boxes);
  manager2->setup();
  timer2.stop();
  t2.push_back(timer2.getElapsedTime());

  DefaultCollisionData<S> cdata2;
  cdata2.request.enable_cost = true;
  cdata3.request.num_max_cost_sources = num_max_cost_sources;

  timer2.start();
  manager->collide(manager2, &cdata2, DefaultCollisionFunction);
  timer2.stop();
  t2.push_back(timer2.getElapsedTime());

  std::cout << cdata.result.numContacts() << " " << cdata3.result.numContacts() << " " << cdata2.result.numContacts() << std::endl;
  std::cout << cdata.result.numCostSources() << " " << cdata3.result.numCostSources() << " " << cdata2.result.numCostSources() << std::endl;

  {
    std::vector<CostSource<S>> cost_sources;
    cdata.result.getCostSources(cost_sources);
    for(std::size_t i = 0; i < cost_sources.size(); ++i)
    {
      std::cout << cost_sources[i].aabb_min.transpose() << " " << cost_sources[i].aabb_max.transpose() << " " << cost_sources[i].cost_density << std::endl;
    }

    std::cout << std::endl;

    cdata3.result.getCostSources(cost_sources);
    for(std::size_t i = 0; i < cost_sources.size(); ++i)
    {
      std::cout << cost_sources[i].aabb_min.transpose() << " " << cost_sources[i].aabb_max.transpose() << " " << cost_sources[i].cost_density << std::endl;
    }

    std::cout << std::endl;

    cdata2.result.getCostSources(cost_sources);
    for(std::size_t i = 0; i < cost_sources.size(); ++i)
    {
      std::cout << cost_sources[i].aabb_min.transpose() << " " << cost_sources[i].aabb_max.transpose() << " " << cost_sources[i].cost_density << std::endl;
    }

    std::cout << std::endl;

  }

  if(use_mesh) EXPECT_TRUE((cdata.result.numContacts() > 0) >= (cdata2.result.numContacts() > 0));
  else EXPECT_TRUE(cdata.result.numContacts() >= cdata2.result.numContacts());

  delete manager;
  delete manager2;
  for(std::size_t i = 0; i < boxes.size(); ++i)
    delete boxes[i];

  std::cout << "collision cost" << std::endl;
  std::cout << "1) octomap overall time: " << t1.overall_time << std::endl;
  std::cout << "1') octomap overall time (as geometry): " << t3.overall_time << std::endl;
  std::cout << "2) boxes overall time: " << t2.overall_time << std::endl;
  std::cout << "  a) to boxes: " << t2.records[0] << std::endl;
  std::cout << "  b) structure init: " << t2.records[1] << std::endl;
  std::cout << "  c) collision: " << t2.records[2] << std::endl;
  std::cout << "Note: octomap may need more collides when using mesh, because octomap collision uses box primitive inside" << std::endl;
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

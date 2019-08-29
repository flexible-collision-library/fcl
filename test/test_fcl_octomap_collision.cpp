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

/// @brief Octomap collision with an environment with 3 * env_size objects
template <typename S>
void octomap_collision_test(S env_scale, std::size_t env_size, bool exhaustive, std::size_t num_max_contacts, bool use_mesh, bool use_mesh_octomap, double resolution = 0.1);

/// @brief Octomap collision with an environment mesh with 3 * env_size objects, asserting that correct triangle ids
/// are returned when performing collision tests
template <typename S>
void octomap_collision_test_contact_primitive_id(
    S env_scale,
    std::size_t env_size,
    std::size_t num_max_contacts,
    double resolution = 0.1);

template<typename BV>
void octomap_collision_test_BVH(std::size_t n, bool exhaustive, double resolution = 0.1);

template <typename S>
void test_octomap_collision()
{
#ifdef NDEBUG
  octomap_collision_test<S>(200, 100, false, 10, false, false);
  octomap_collision_test<S>(200, 1000, false, 10, false, false);
  octomap_collision_test<S>(200, 100, true, 1, false, false);
  octomap_collision_test<S>(200, 1000, true, 1, false, false);
#else
  octomap_collision_test<S>(200, 10, false, 10, false, false, 0.1);
  octomap_collision_test<S>(200, 100, false, 10, false, false, 0.1);
  octomap_collision_test<S>(200, 10, true, 1, false, false, 0.1);
  octomap_collision_test<S>(200, 100, true, 1, false, false, 0.1);
#endif
}

GTEST_TEST(FCL_OCTOMAP, test_octomap_collision)
{
//  test_octomap_collision<float>();
  test_octomap_collision<double>();
}

template <typename S>
void test_octomap_collision_mesh()
{
#ifdef NDEBUG
  octomap_collision_test<S>(200, 100, false, 10, true, true);
  octomap_collision_test<S>(200, 1000, false, 10, true, true);
  octomap_collision_test<S>(200, 100, true, 1, true, true);
  octomap_collision_test<S>(200, 1000, true, 1, true, true);
#else
  octomap_collision_test<S>(200, 4, false, 1, true, true, 1.0);
  octomap_collision_test<S>(200, 4, true, 1, true, true, 1.0);
#endif
}

GTEST_TEST(FCL_OCTOMAP, test_octomap_collision_mesh)
{
//  test_octomap_collision_mesh<float>();
  test_octomap_collision_mesh<double>();
}

template <typename S>
void test_octomap_collision_contact_primitive_id()
{
#ifdef NDEBUG
  octomap_collision_test_contact_primitive_id<S>(1, 30, 100000);
#else
  octomap_collision_test_contact_primitive_id<S>(1, 10, 10000, 1.0);
#endif
}

GTEST_TEST(FCL_OCTOMAP, test_octomap_collision_contact_primitive_id)
{
//  test_octomap_collision_contact_primitive_id<float>();
  test_octomap_collision_contact_primitive_id<double>();
}

template <typename S>
void test_octomap_collision_mesh_octomap_box()
{
#ifdef NDEBUG
  octomap_collision_test<S>(200, 100, false, 10, true, false);
  octomap_collision_test<S>(200, 1000, false, 10, true, false);
  octomap_collision_test<S>(200, 100, true, 1, true, false);
  octomap_collision_test<S>(200, 1000, true, 1, true, false);
#else
  octomap_collision_test<S>(200, 4, false, 4, true, false, 1.0);
  octomap_collision_test<S>(200, 4, true, 1, true, false, 1.0);
#endif
}

GTEST_TEST(FCL_OCTOMAP, test_octomap_collision_mesh_octomap_box)
{
//  test_octomap_collision_mesh_octomap_box<float>();
  test_octomap_collision_mesh_octomap_box<double>();
}

template <typename S>
void test_octomap_bvh_obb_collision_obb()
{
#ifdef NDEBUG
  octomap_collision_test_BVH<OBB<S>>(5, false);
  octomap_collision_test_BVH<OBB<S>>(5, true);
#else
  octomap_collision_test_BVH<OBB<S>>(1, false, 1.0);
  octomap_collision_test_BVH<OBB<S>>(1, true, 1.0);
#endif
}

GTEST_TEST(FCL_OCTOMAP, test_octomap_bvh_obb_collision_obb)
{
//  test_octomap_bvh_obb_collision_obb<float>();
  test_octomap_bvh_obb_collision_obb<double>();
}

template<typename BV>
void octomap_collision_test_BVH(std::size_t n, bool exhaustive, double resolution)
{
  using S = typename BV::S;

  std::vector<Vector3<S>> p1;
  std::vector<Triangle> t1;

  test::loadOBJFile(TEST_RESOURCES_DIR"/env.obj", p1, t1);

  BVHModel<BV>* m1 = new BVHModel<BV>();
  std::shared_ptr<CollisionGeometry<S>> m1_ptr(m1);

  m1->beginModel();
  m1->addSubModel(p1, t1);
  m1->endModel();

  auto octree = std::shared_ptr<const octomap::OcTree>(
      test::generateOcTree(resolution));
  OcTree<S>* tree = new OcTree<S>(octree);
  std::shared_ptr<CollisionGeometry<S>> tree_ptr(tree);

  // Check and make sure that the generated tree contains both free and
  // occupied space. There was a time when it was impossible to represent free
  // space, this part of the collision tests that both free and occupied space
  // are correctly represented.
  size_t free_nodes = 0;
  size_t occupied_nodes = 0;
  for (auto it = octree->begin(), end = octree->end(); it != end; ++it)
  {
    if (tree->isNodeFree(&*it))
      ++free_nodes;
    if (tree->isNodeOccupied(&*it))
      ++occupied_nodes;
  }
  EXPECT_GT(free_nodes, 0UL);
  EXPECT_GT(occupied_nodes, 0UL);

  aligned_vector<Transform3<S>> transforms;
  S extents[] = {-10, -10, 10, 10, 10, 10};

  test::generateRandomTransforms(extents, transforms, n);

  for(std::size_t i = 0; i < n; ++i)
  {
    Transform3<S> tf(transforms[i]);

    CollisionObject<S> obj1(m1_ptr, tf);
    CollisionObject<S> obj2(tree_ptr, tf);
    DefaultCollisionData<S> cdata;
    if(exhaustive) cdata.request.num_max_contacts = 100000;

    DefaultCollisionFunction(&obj1, &obj2, &cdata);


    std::vector<CollisionObject<S>*> boxes;
    test::generateBoxesFromOctomap(boxes, *tree);
    for(std::size_t j = 0; j < boxes.size(); ++j)
      boxes[j]->setTransform(tf * boxes[j]->getTransform());

    DynamicAABBTreeCollisionManager<S>* manager = new DynamicAABBTreeCollisionManager<S>();
    manager->registerObjects(boxes);
    manager->setup();

    DefaultCollisionData<S> cdata2;
    if(exhaustive) cdata2.request.num_max_contacts = 100000;
    manager->collide(&obj1, &cdata2, DefaultCollisionFunction);

    for(std::size_t j = 0; j < boxes.size(); ++j)
      delete boxes[j];
    delete manager;

    if(exhaustive)
    {
      std::cout << cdata.result.numContacts() << " " << cdata2.result.numContacts() << std::endl;
      EXPECT_TRUE(cdata.result.numContacts() == cdata2.result.numContacts());
    }
    else
    {
      std::cout << (cdata.result.numContacts() > 0) << " " << (cdata2.result.numContacts() > 0) << std::endl;
      EXPECT_TRUE((cdata.result.numContacts() > 0) == (cdata2.result.numContacts() > 0));
    }
  }
}

template <typename S>
void octomap_collision_test(S env_scale, std::size_t env_size, bool exhaustive, std::size_t num_max_contacts, bool use_mesh, bool use_mesh_octomap, double resolution)
{
  // srand(1);
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
  if(exhaustive) cdata.request.num_max_contacts = 100000;
  else cdata.request.num_max_contacts = num_max_contacts;

  test::TStruct t1;
  test::Timer timer1;
  timer1.start();
  manager->octree_as_geometry_collide = false;
  manager->octree_as_geometry_distance = false;
  manager->collide(&tree_obj, &cdata, DefaultCollisionFunction);
  timer1.stop();
  t1.push_back(timer1.getElapsedTime());

  DefaultCollisionData<S> cdata3;
  if(exhaustive) cdata3.request.num_max_contacts = 100000;
  else cdata3.request.num_max_contacts = num_max_contacts;

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
  if(exhaustive) cdata2.request.num_max_contacts = 100000;
  else cdata2.request.num_max_contacts = num_max_contacts;

  timer2.start();
  manager->collide(manager2, &cdata2, DefaultCollisionFunction);
  timer2.stop();
  t2.push_back(timer2.getElapsedTime());

  std::cout << cdata.result.numContacts() << " " << cdata3.result.numContacts() << " " << cdata2.result.numContacts() << std::endl;
  if(exhaustive)
  {
    if(use_mesh) EXPECT_TRUE((cdata.result.numContacts() > 0) >= (cdata2.result.numContacts() > 0));
    else EXPECT_TRUE(cdata.result.numContacts() == cdata2.result.numContacts());
  }
  else
  {
    if(use_mesh) EXPECT_TRUE((cdata.result.numContacts() > 0) >= (cdata2.result.numContacts() > 0));
    else EXPECT_TRUE((cdata.result.numContacts() > 0) >= (cdata2.result.numContacts() > 0)); // because AABB<S> return collision when two boxes contact
  }

  delete manager;
  delete manager2;
  for(size_t i = 0; i < boxes.size(); ++i)
    delete boxes[i];

  if(exhaustive) std::cout << "exhaustive collision" << std::endl;
  else std::cout << "non exhaustive collision" << std::endl;
  std::cout << "1) octomap overall time: " << t1.overall_time << std::endl;
  std::cout << "1') octomap overall time (as geometry): " << t3.overall_time << std::endl;
  std::cout << "2) boxes overall time: " << t2.overall_time << std::endl;
  std::cout << "  a) to boxes: " << t2.records[0] << std::endl;
  std::cout << "  b) structure init: " << t2.records[1] << std::endl;
  std::cout << "  c) collision: " << t2.records[2] << std::endl;
  std::cout << "Note: octomap may need more collides when using mesh, because octomap collision uses box primitive inside" << std::endl;
}

template <typename S>
void octomap_collision_test_contact_primitive_id(
    S env_scale,
    std::size_t env_size,
    std::size_t num_max_contacts,
    double resolution)
{
  std::vector<CollisionObject<S>*> env;
  test::generateEnvironmentsMesh(env, env_scale, env_size);

  std::shared_ptr<const octomap::OcTree> octree(
      test::generateOcTree(resolution));
  OcTree<S>* tree = new OcTree<S>(octree);
  CollisionObject<S> tree_obj((std::shared_ptr<CollisionGeometry<S>>(tree)));

  std::vector<CollisionObject<S>*> boxes;
  test::generateBoxesFromOctomap(boxes, *tree);
  for(typename std::vector<CollisionObject<S>*>::const_iterator cit = env.begin();
      cit != env.end(); ++cit)
  {
    fcl::CollisionRequest<S> req(num_max_contacts, true);
    fcl::CollisionResult<S> cResult;
    fcl::collide(&tree_obj, *cit, req, cResult);
    for(std::size_t index=0; index<cResult.numContacts(); ++index)
    {
      const Contact<S>& contact = cResult.getContact(index);

      const fcl::OcTree<S>* contact_tree = static_cast<const fcl::OcTree<S>*>(
          contact.o1);
      fcl::AABB<S> aabb;
      octomap::OcTreeKey key;
      unsigned int depth;
      auto get_node_rv = contact_tree->getNodeByQueryCellId(
          contact.b1,
          contact.pos,
          &aabb,
          &key,
          &depth);
      EXPECT_TRUE(get_node_rv != nullptr);
      auto center_octomap_point = octree->keyToCoord(key);
      double cell_size = octree->getNodeSize(depth);
      for (unsigned i = 0; i < 3; ++i)
      {
        EXPECT_FLOAT_EQ(
            aabb.min_[i], center_octomap_point(i) - cell_size / 2.0);
        EXPECT_FLOAT_EQ(
            aabb.max_[i], center_octomap_point(i) + cell_size / 2.0);
      }
      auto octree_node = octree->search(key, depth);
      EXPECT_TRUE(octree_node == get_node_rv);

      const fcl::BVHModel<fcl::OBBRSS<S>>* surface = static_cast<const fcl::BVHModel<fcl::OBBRSS<S>>*> (contact.o2);
      EXPECT_TRUE(surface->num_tris > contact.b2);
    }
  }
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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

/// @brief Octomap collision with an environment with 3 * env_size objects
template <typename Scalar>
void octomap_collision_test(Scalar env_scale, std::size_t env_size, bool exhaustive, std::size_t num_max_contacts, bool use_mesh, bool use_mesh_octomap, double resolution = 0.1);

/// @brief Octomap collision with an environment mesh with 3 * env_size objects, asserting that correct triangle ids
/// are returned when performing collision tests
template <typename Scalar>
void octomap_collision_test_mesh_triangle_id(Scalar env_scale, std::size_t env_size, std::size_t num_max_contacts, double resolution = 0.1);

template<typename BV>
void octomap_collision_test_BVH(std::size_t n, bool exhaustive, double resolution = 0.1);

template <typename Scalar>
void test_octomap_collision()
{
#if FCL_BUILD_TYPE_DEBUG
  octomap_collision_test<Scalar>(200, 10, false, 10, false, false, 0.1);
  octomap_collision_test<Scalar>(200, 100, false, 10, false, false, 0.1);
  octomap_collision_test<Scalar>(200, 10, true, 1, false, false, 0.1);
  octomap_collision_test<Scalar>(200, 100, true, 1, false, false, 0.1);
#else
  octomap_collision_test<Scalar>(200, 100, false, 10, false, false);
  octomap_collision_test<Scalar>(200, 1000, false, 10, false, false);
  octomap_collision_test<Scalar>(200, 100, true, 1, false, false);
  octomap_collision_test<Scalar>(200, 1000, true, 1, false, false);
#endif
}

GTEST_TEST(FCL_OCTOMAP, test_octomap_collision)
{
//  test_octomap_collision<float>();
  test_octomap_collision<double>();
}

template <typename Scalar>
void test_octomap_collision_mesh()
{
#if FCL_BUILD_TYPE_DEBUG
  octomap_collision_test<Scalar>(200, 4, false, 1, true, true, 1.0);
  octomap_collision_test<Scalar>(200, 4, true, 1, true, true, 1.0);
#else
  octomap_collision_test<Scalar>(200, 100, false, 10, true, true);
  octomap_collision_test<Scalar>(200, 1000, false, 10, true, true);
  octomap_collision_test<Scalar>(200, 100, true, 1, true, true);
  octomap_collision_test<Scalar>(200, 1000, true, 1, true, true);
#endif
}

GTEST_TEST(FCL_OCTOMAP, test_octomap_collision_mesh)
{
//  test_octomap_collision_mesh<float>();
  test_octomap_collision_mesh<double>();
}

template <typename Scalar>
void test_octomap_collision_mesh_triangle_id()
{
#if FCL_BUILD_TYPE_DEBUG
  octomap_collision_test_mesh_triangle_id<Scalar>(1, 10, 10000, 1.0);
#else
  octomap_collision_test_mesh_triangle_id<Scalar>(1, 30, 100000);
#endif
}

GTEST_TEST(FCL_OCTOMAP, test_octomap_collision_mesh_triangle_id)
{
//  test_octomap_collision_mesh_triangle_id<float>();
  test_octomap_collision_mesh_triangle_id<double>();
}

template <typename Scalar>
void test_octomap_collision_mesh_octomap_box()
{
#if FCL_BUILD_TYPE_DEBUG
  octomap_collision_test<Scalar>(200, 4, false, 4, true, false, 1.0);
  octomap_collision_test<Scalar>(200, 4, true, 1, true, false, 1.0);
#else
  octomap_collision_test<Scalar>(200, 100, false, 10, true, false);
  octomap_collision_test<Scalar>(200, 1000, false, 10, true, false);
  octomap_collision_test<Scalar>(200, 100, true, 1, true, false);
  octomap_collision_test<Scalar>(200, 1000, true, 1, true, false);
#endif
}

GTEST_TEST(FCL_OCTOMAP, test_octomap_collision_mesh_octomap_box)
{
//  test_octomap_collision_mesh_octomap_box<float>();
  test_octomap_collision_mesh_octomap_box<double>();
}

template <typename Scalar>
void test_octomap_bvh_obb_collision_obb()
{
#if FCL_BUILD_TYPE_DEBUG
  octomap_collision_test_BVH<OBB<Scalar>>(1, false, 1.0);
  octomap_collision_test_BVH<OBB<Scalar>>(1, true, 1.0);
#else
  octomap_collision_test_BVH<OBB<Scalar>>(5, false);
  octomap_collision_test_BVH<OBB<Scalar>>(5, true);
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
  using Scalar = typename BV::Scalar;

  std::vector<Vector3<Scalar>> p1;
  std::vector<Triangle> t1;

  loadOBJFile(TEST_RESOURCES_DIR"/env.obj", p1, t1);

  BVHModel<BV>* m1 = new BVHModel<BV>();
  std::shared_ptr<CollisionGeometry<Scalar>> m1_ptr(m1);

  m1->beginModel();
  m1->addSubModel(p1, t1);
  m1->endModel();

  OcTree<Scalar>* tree = new OcTree<Scalar>(std::shared_ptr<const octomap::OcTree>(generateOcTree(resolution)));
  std::shared_ptr<CollisionGeometry<Scalar>> tree_ptr(tree);

  Eigen::aligned_vector<Transform3<Scalar>> transforms;
  Scalar extents[] = {-10, -10, 10, 10, 10, 10};

  generateRandomTransforms(extents, transforms, n);
  
  for(std::size_t i = 0; i < n; ++i)
  {
    Transform3<Scalar> tf(transforms[i]);

    CollisionObject<Scalar> obj1(m1_ptr, tf);
    CollisionObject<Scalar> obj2(tree_ptr, tf);
    CollisionData<Scalar> cdata;
    if(exhaustive) cdata.request.num_max_contacts = 100000;

    defaultCollisionFunction(&obj1, &obj2, &cdata);


    std::vector<CollisionObject<Scalar>*> boxes;
    generateBoxesFromOctomap(boxes, *tree);
    for(std::size_t j = 0; j < boxes.size(); ++j)
      boxes[j]->setTransform(tf * boxes[j]->getTransform());
  
    DynamicAABBTreeCollisionManager<Scalar>* manager = new DynamicAABBTreeCollisionManager<Scalar>();
    manager->registerObjects(boxes);
    manager->setup();

    CollisionData<Scalar> cdata2;
    if(exhaustive) cdata2.request.num_max_contacts = 100000;
    manager->collide(&obj1, &cdata2, defaultCollisionFunction);

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

template <typename Scalar>
void octomap_collision_test(Scalar env_scale, std::size_t env_size, bool exhaustive, std::size_t num_max_contacts, bool use_mesh, bool use_mesh_octomap, double resolution)
{
  // srand(1);
  std::vector<CollisionObject<Scalar>*> env;
  if(use_mesh)
    generateEnvironmentsMesh(env, env_scale, env_size);
  else
    generateEnvironments(env, env_scale, env_size);

  OcTree<Scalar>* tree = new OcTree<Scalar>(std::shared_ptr<const octomap::OcTree>(generateOcTree(resolution)));
  CollisionObject<Scalar> tree_obj((std::shared_ptr<CollisionGeometry<Scalar>>(tree)));

  DynamicAABBTreeCollisionManager<Scalar>* manager = new DynamicAABBTreeCollisionManager<Scalar>();
  manager->registerObjects(env);
  manager->setup();
  
  CollisionData<Scalar> cdata;
  if(exhaustive) cdata.request.num_max_contacts = 100000;
  else cdata.request.num_max_contacts = num_max_contacts;

  TStruct t1;
  Timer timer1;
  timer1.start();
  manager->octree_as_geometry_collide = false;
  manager->octree_as_geometry_distance = false;
  manager->collide(&tree_obj, &cdata, defaultCollisionFunction);
  timer1.stop();
  t1.push_back(timer1.getElapsedTime());

  CollisionData<Scalar> cdata3;
  if(exhaustive) cdata3.request.num_max_contacts = 100000;
  else cdata3.request.num_max_contacts = num_max_contacts;

  TStruct t3;
  Timer timer3;
  timer3.start();
  manager->octree_as_geometry_collide = true;
  manager->octree_as_geometry_distance = true;
  manager->collide(&tree_obj, &cdata3, defaultCollisionFunction);
  timer3.stop();
  t3.push_back(timer3.getElapsedTime());

  TStruct t2;
  Timer timer2;
  timer2.start();
  std::vector<CollisionObject<Scalar>*> boxes;
  if(use_mesh_octomap)
    generateBoxesFromOctomapMesh(boxes, *tree);
  else
    generateBoxesFromOctomap(boxes, *tree);
  timer2.stop();
  t2.push_back(timer2.getElapsedTime());
  
  timer2.start();
  DynamicAABBTreeCollisionManager<Scalar>* manager2 = new DynamicAABBTreeCollisionManager<Scalar>();
  manager2->registerObjects(boxes);
  manager2->setup();
  timer2.stop();
  t2.push_back(timer2.getElapsedTime());


  CollisionData<Scalar> cdata2;
  if(exhaustive) cdata2.request.num_max_contacts = 100000;
  else cdata2.request.num_max_contacts = num_max_contacts;

  timer2.start();
  manager->collide(manager2, &cdata2, defaultCollisionFunction);
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
    else EXPECT_TRUE((cdata.result.numContacts() > 0) >= (cdata2.result.numContacts() > 0)); // because AABB<Scalar> return collision when two boxes contact
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

template <typename Scalar>
void octomap_collision_test_mesh_triangle_id(Scalar env_scale, std::size_t env_size, std::size_t num_max_contacts, double resolution)
{
  std::vector<CollisionObject<Scalar>*> env;
  generateEnvironmentsMesh(env, env_scale, env_size);

  OcTree<Scalar>* tree = new OcTree<Scalar>(std::shared_ptr<const octomap::OcTree>(generateOcTree(resolution)));
  CollisionObject<Scalar> tree_obj((std::shared_ptr<CollisionGeometry<Scalar>>(tree)));

  std::vector<CollisionObject<Scalar>*> boxes;
  generateBoxesFromOctomap(boxes, *tree);
  for(typename std::vector<CollisionObject<Scalar>*>::const_iterator cit = env.begin();
      cit != env.end(); ++cit)
  {
    fcl::CollisionRequest<Scalar> req(num_max_contacts, true);
    fcl::CollisionResult<Scalar> cResult;
    fcl::collide(&tree_obj, *cit, req, cResult);
    for(std::size_t index=0; index<cResult.numContacts(); ++index)
    {
      const Contact<Scalar>& contact = cResult.getContact(index);
      const fcl::BVHModel<fcl::OBBRSS<Scalar>>* surface = static_cast<const fcl::BVHModel<fcl::OBBRSS<Scalar>>*> (contact.o2);
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

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

#define BOOST_TEST_MODULE "FCL_OCTOMAP"
#include <boost/test/unit_test.hpp>

#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"
#include "fcl/math/transform.h"
#include "test_fcl_utility.h"
#include "fcl_resources/config.h"
#include <boost/filesystem.hpp>

using namespace fcl;


struct TStruct
{
  std::vector<double> records;
  double overall_time;

  TStruct() { overall_time = 0; }
  
  void push_back(double t)
  {
    records.push_back(t);
    overall_time += t;
  }
};

/// @brief Generate environment with 3 * n objects: n spheres, n boxes and n cylinders
void generateEnvironments(std::vector<CollisionObject*>& env, double env_scale, std::size_t n);

/// @brief Generate environment with 3 * n objects: n spheres, n boxes and n cylinders, all in mesh
void generateEnvironmentsMesh(std::vector<CollisionObject*>& env, double env_scale, std::size_t n);

/// @brief Generate boxes from the octomap
void generateBoxesFromOctomap(std::vector<CollisionObject*>& env, OcTree& tree);

/// @brief Generate boxes from the octomap
void generateBoxesFromOctomapMesh(std::vector<CollisionObject*>& env, OcTree& tree);

/// @brief Generate an octree
octomap::OcTree* generateOcTree();

/// @brief Octomap collision with an environment with 3 * env_size objects
void octomap_collision_test(double env_scale, std::size_t env_size, bool exhaustive, std::size_t num_max_contacts, bool use_mesh, bool use_mesh_octomap);

/// @brief Octomap collision with an environment with 3 * env_size objects, compute cost
void octomap_cost_test(double env_scale, std::size_t env_size, std::size_t num_max_cost_sources, bool use_mesh, bool use_mesh_octomap);

/// @brief Octomap distance with an environment with 3 * env_size objects
void octomap_distance_test(double env_scale, std::size_t env_size, bool use_mesh, bool use_mesh_octomap);

/// @brief Octomap collision with an environment mesh with 3 * env_size objects, asserting that correct triangle ids
/// are returned when performing collision tests
void octomap_collision_test_mesh_triangle_id(double env_scale, std::size_t env_size, std::size_t num_max_contacts);


template<typename BV>
void octomap_collision_test_BVH(std::size_t n, bool exhaustive);


template<typename BV>
void octomap_distance_test_BVH(std::size_t n);

BOOST_AUTO_TEST_CASE(test_octomap_cost)
{
  octomap_cost_test(200, 100, 10, false, false);
  octomap_cost_test(200, 1000, 10, false, false);
}

BOOST_AUTO_TEST_CASE(test_octomap_cost_mesh)
{
  octomap_cost_test(200, 100, 10, true, false);
  octomap_cost_test(200, 1000, 10, true, false);
}

BOOST_AUTO_TEST_CASE(test_octomap_collision)
{
  octomap_collision_test(200, 100, false, 10, false, false);
  octomap_collision_test(200, 1000, false, 10, false, false);
  octomap_collision_test(200, 100, true, 1, false, false);
  octomap_collision_test(200, 1000, true, 1, false, false);
}

BOOST_AUTO_TEST_CASE(test_octomap_collision_mesh)
{
#if FCL_BUILD_TYPE_DEBUG
  octomap_collision_test(200, 10, false, 10, true, true);
  octomap_collision_test(200, 100, false, 10, true, true);
  octomap_collision_test(200, 10, true, 1, true, true);
  octomap_collision_test(200, 100, true, 1, true, true);
#else
  octomap_collision_test(200, 100, false, 10, true, true);
  octomap_collision_test(200, 1000, false, 10, true, true);
  octomap_collision_test(200, 100, true, 1, true, true);
  octomap_collision_test(200, 1000, true, 1, true, true);
#endif
}

BOOST_AUTO_TEST_CASE(test_octomap_collision_mesh_triangle_id)
{
  octomap_collision_test_mesh_triangle_id(1, 30, 100000);
}


BOOST_AUTO_TEST_CASE(test_octomap_collision_mesh_octomap_box)
{
#if FCL_BUILD_TYPE_DEBUG
  octomap_collision_test(200, 10, false, 10, true, false);
  octomap_collision_test(200, 100, false, 10, true, false);
  octomap_collision_test(200, 10, true, 1, true, false);
  octomap_collision_test(200, 100, true, 1, true, false);
#else
  octomap_collision_test(200, 100, false, 10, true, false);
  octomap_collision_test(200, 1000, false, 10, true, false);
  octomap_collision_test(200, 100, true, 1, true, false);
  octomap_collision_test(200, 1000, true, 1, true, false);
#endif
}

BOOST_AUTO_TEST_CASE(test_octomap_distance)
{
  octomap_distance_test(200, 100, false, false);
  octomap_distance_test(200, 1000, false, false);
}

BOOST_AUTO_TEST_CASE(test_octomap_distance_mesh)
{
#if FCL_BUILD_TYPE_DEBUG
  octomap_distance_test(200, 5, true, true);
  octomap_distance_test(200, 50, true, true);
#else
  octomap_distance_test(200, 100, true, true);
  octomap_distance_test(200, 1000, true, true);
#endif
}

BOOST_AUTO_TEST_CASE(test_octomap_distance_mesh_octomap_box)
{
#if FCL_BUILD_TYPE_DEBUG
  octomap_distance_test(200, 10, true, false);
  octomap_distance_test(200, 100, true, false);
#else
  octomap_distance_test(200, 100, true, false);
  octomap_distance_test(200, 1000, true, false);
#endif
}

BOOST_AUTO_TEST_CASE(test_octomap_bvh_obb_collision_obb)
{
  octomap_collision_test_BVH<OBB>(5, false);
  octomap_collision_test_BVH<OBB>(5, true);
}

BOOST_AUTO_TEST_CASE(test_octomap_bvh_rss_d_distance_rss)
{
  octomap_distance_test_BVH<RSS>(5);
}

BOOST_AUTO_TEST_CASE(test_octomap_bvh_obb_d_distance_obb)
{
  octomap_distance_test_BVH<OBBRSS>(5);
}

BOOST_AUTO_TEST_CASE(test_octomap_bvh_kios_d_distance_kios)
{
#if FCL_BUILD_TYPE_DEBUG
  octomap_distance_test_BVH<kIOS>(2);
#else
  octomap_distance_test_BVH<kIOS>(5);
#endif
}

template<typename BV>
void octomap_collision_test_BVH(std::size_t n, bool exhaustive)
{
  std::vector<Vec3f> p1;
  std::vector<Triangle> t1;
  boost::filesystem::path path(TEST_RESOURCES_DIR);
  loadOBJFile((path / "env.obj").string().c_str(), p1, t1);

  BVHModel<BV>* m1 = new BVHModel<BV>();
  std::shared_ptr<CollisionGeometry> m1_ptr(m1);

  m1->beginModel();
  m1->addSubModel(p1, t1);
  m1->endModel();

  OcTree* tree = new OcTree(std::shared_ptr<const octomap::OcTree>(generateOcTree()));
  std::shared_ptr<CollisionGeometry> tree_ptr(tree);

  std::vector<Transform3f> transforms;
  FCL_REAL extents[] = {-10, -10, 10, 10, 10, 10};

  generateRandomTransforms(extents, transforms, n);
  
  for(std::size_t i = 0; i < n; ++i)
  {
    Transform3f tf(transforms[i]);

    CollisionObject obj1(m1_ptr, tf);
    CollisionObject obj2(tree_ptr, tf);
    CollisionData cdata;
    if(exhaustive) cdata.request.num_max_contacts = 100000;

    defaultCollisionFunction(&obj1, &obj2, &cdata);


    std::vector<CollisionObject*> boxes;
    generateBoxesFromOctomap(boxes, *tree);
    for(std::size_t j = 0; j < boxes.size(); ++j)
      boxes[j]->setTransform(tf * boxes[j]->getTransform());
  
    DynamicAABBTreeCollisionManager* manager = new DynamicAABBTreeCollisionManager();
    manager->registerObjects(boxes);
    manager->setup();

    CollisionData cdata2;
    if(exhaustive) cdata2.request.num_max_contacts = 100000;
    manager->collide(&obj1, &cdata2, defaultCollisionFunction);

    for(std::size_t j = 0; j < boxes.size(); ++j)
      delete boxes[j];
    delete manager;

    if(exhaustive)
    {
      std::cout << cdata.result.numContacts() << " " << cdata2.result.numContacts() << std::endl;
      BOOST_CHECK(cdata.result.numContacts() == cdata2.result.numContacts());
    }
    else
    {
      std::cout << (cdata.result.numContacts() > 0) << " " << (cdata2.result.numContacts() > 0) << std::endl;
      BOOST_CHECK((cdata.result.numContacts() > 0) == (cdata2.result.numContacts() > 0));
    }
  }
}


template<typename BV>
void octomap_distance_test_BVH(std::size_t n)
{
  std::vector<Vec3f> p1;
  std::vector<Triangle> t1;
  boost::filesystem::path path(TEST_RESOURCES_DIR);
  loadOBJFile((path / "env.obj").string().c_str(), p1, t1);

  BVHModel<BV>* m1 = new BVHModel<BV>();
  std::shared_ptr<CollisionGeometry> m1_ptr(m1);

  m1->beginModel();
  m1->addSubModel(p1, t1);
  m1->endModel();

  OcTree* tree = new OcTree(std::shared_ptr<octomap::OcTree>(generateOcTree()));
  std::shared_ptr<CollisionGeometry> tree_ptr(tree);

  std::vector<Transform3f> transforms;
  FCL_REAL extents[] = {-10, -10, 10, 10, 10, 10};

  generateRandomTransforms(extents, transforms, n);
  
  for(std::size_t i = 0; i < n; ++i)
  {
    Transform3f tf(transforms[i]);

    CollisionObject obj1(m1_ptr, tf);
    CollisionObject obj2(tree_ptr, tf);
    DistanceData cdata;
    FCL_REAL dist1 = std::numeric_limits<FCL_REAL>::max();
    defaultDistanceFunction(&obj1, &obj2, &cdata, dist1);


    std::vector<CollisionObject*> boxes;
    generateBoxesFromOctomap(boxes, *tree);
    for(std::size_t j = 0; j < boxes.size(); ++j)
      boxes[j]->setTransform(tf * boxes[j]->getTransform());
  
    DynamicAABBTreeCollisionManager* manager = new DynamicAABBTreeCollisionManager();
    manager->registerObjects(boxes);
    manager->setup();

    DistanceData cdata2;
    manager->distance(&obj1, &cdata2, defaultDistanceFunction);
    FCL_REAL dist2 = cdata2.result.min_distance;

    for(std::size_t j = 0; j < boxes.size(); ++j)
      delete boxes[j];
    delete manager;

    std::cout << dist1 << " " << dist2 << std::endl;
    BOOST_CHECK(std::abs(dist1 - dist2) < 0.001);
  }
}


void octomap_cost_test(double env_scale, std::size_t env_size, std::size_t num_max_cost_sources, bool use_mesh, bool use_mesh_octomap)
{
  std::vector<CollisionObject*> env;
  if(use_mesh)
    generateEnvironmentsMesh(env, env_scale, env_size);
  else
    generateEnvironments(env, env_scale, env_size);

  OcTree* tree = new OcTree(std::shared_ptr<const octomap::OcTree>(generateOcTree()));
  CollisionObject tree_obj((std::shared_ptr<CollisionGeometry>(tree)));

  DynamicAABBTreeCollisionManager* manager = new DynamicAABBTreeCollisionManager();
  manager->registerObjects(env);
  manager->setup();
  
  CollisionData cdata;
  cdata.request.enable_cost = true;
  cdata.request.num_max_cost_sources = num_max_cost_sources;

  TStruct t1;
  Timer timer1;
  timer1.start();
  manager->octree_as_geometry_collide = false;
  manager->octree_as_geometry_distance = false;
  manager->collide(&tree_obj, &cdata, defaultCollisionFunction);
  timer1.stop();
  t1.push_back(timer1.getElapsedTime());

  CollisionData cdata3;
  cdata3.request.enable_cost = true;
  cdata3.request.num_max_cost_sources = num_max_cost_sources;

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
  std::vector<CollisionObject*> boxes;
  if(use_mesh_octomap)
    generateBoxesFromOctomapMesh(boxes, *tree);
  else
    generateBoxesFromOctomap(boxes, *tree);
  timer2.stop();
  t2.push_back(timer2.getElapsedTime());
  
  timer2.start();
  DynamicAABBTreeCollisionManager* manager2 = new DynamicAABBTreeCollisionManager();
  manager2->registerObjects(boxes);
  manager2->setup();
  timer2.stop();
  t2.push_back(timer2.getElapsedTime());


  CollisionData cdata2;
  cdata2.request.enable_cost = true;
  cdata3.request.num_max_cost_sources = num_max_cost_sources;

  timer2.start();
  manager->collide(manager2, &cdata2, defaultCollisionFunction);
  timer2.stop();
  t2.push_back(timer2.getElapsedTime());

  std::cout << cdata.result.numContacts() << " " << cdata3.result.numContacts() << " " << cdata2.result.numContacts() << std::endl;
  std::cout << cdata.result.numCostSources() << " " << cdata3.result.numCostSources() << " " << cdata2.result.numCostSources() << std::endl;

  {
    std::vector<CostSource> cost_sources;
    cdata.result.getCostSources(cost_sources);
    for(std::size_t i = 0; i < cost_sources.size(); ++i)
    {
      std::cout << cost_sources[i].aabb_min << " " << cost_sources[i].aabb_max << " " << cost_sources[i].cost_density << std::endl;
    }

    std::cout << std::endl;

    cdata3.result.getCostSources(cost_sources);
    for(std::size_t i = 0; i < cost_sources.size(); ++i)
    {
      std::cout << cost_sources[i].aabb_min << " " << cost_sources[i].aabb_max << " " << cost_sources[i].cost_density << std::endl;
    }

    std::cout << std::endl;

    cdata2.result.getCostSources(cost_sources);
    for(std::size_t i = 0; i < cost_sources.size(); ++i)
    {
      std::cout << cost_sources[i].aabb_min << " " << cost_sources[i].aabb_max << " " << cost_sources[i].cost_density << std::endl;
    }

    std::cout << std::endl;

  }

  if(use_mesh) BOOST_CHECK((cdata.result.numContacts() > 0) >= (cdata2.result.numContacts() > 0));
  else BOOST_CHECK(cdata.result.numContacts() >= cdata2.result.numContacts());

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


void octomap_collision_test(double env_scale, std::size_t env_size, bool exhaustive, std::size_t num_max_contacts, bool use_mesh, bool use_mesh_octomap)
{
  // srand(1);
  std::vector<CollisionObject*> env;
  if(use_mesh)
    generateEnvironmentsMesh(env, env_scale, env_size);
  else
    generateEnvironments(env, env_scale, env_size);

  OcTree* tree = new OcTree(std::shared_ptr<const octomap::OcTree>(generateOcTree()));
  CollisionObject tree_obj((std::shared_ptr<CollisionGeometry>(tree)));

  DynamicAABBTreeCollisionManager* manager = new DynamicAABBTreeCollisionManager();
  manager->registerObjects(env);
  manager->setup();
  
  CollisionData cdata;
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

  CollisionData cdata3;
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
  std::vector<CollisionObject*> boxes;
  if(use_mesh_octomap)
    generateBoxesFromOctomapMesh(boxes, *tree);
  else
    generateBoxesFromOctomap(boxes, *tree);
  timer2.stop();
  t2.push_back(timer2.getElapsedTime());
  
  timer2.start();
  DynamicAABBTreeCollisionManager* manager2 = new DynamicAABBTreeCollisionManager();
  manager2->registerObjects(boxes);
  manager2->setup();
  timer2.stop();
  t2.push_back(timer2.getElapsedTime());


  CollisionData cdata2;
  if(exhaustive) cdata2.request.num_max_contacts = 100000;
  else cdata2.request.num_max_contacts = num_max_contacts;

  timer2.start();
  manager->collide(manager2, &cdata2, defaultCollisionFunction);
  timer2.stop();
  t2.push_back(timer2.getElapsedTime());

  std::cout << cdata.result.numContacts() << " " << cdata3.result.numContacts() << " " << cdata2.result.numContacts() << std::endl;
  if(exhaustive)
  {
    if(use_mesh) BOOST_CHECK((cdata.result.numContacts() > 0) >= (cdata2.result.numContacts() > 0));
    else BOOST_CHECK(cdata.result.numContacts() == cdata2.result.numContacts());
  }
  else
  {
    if(use_mesh) BOOST_CHECK((cdata.result.numContacts() > 0) >= (cdata2.result.numContacts() > 0));
    else BOOST_CHECK((cdata.result.numContacts() > 0) >= (cdata2.result.numContacts() > 0)); // because AABB return collision when two boxes contact
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

void octomap_collision_test_mesh_triangle_id(double env_scale, std::size_t env_size, std::size_t num_max_contacts)
{
  std::vector<CollisionObject*> env;
  generateEnvironmentsMesh(env, env_scale, env_size);

  OcTree* tree = new OcTree(std::shared_ptr<const octomap::OcTree>(generateOcTree()));
  CollisionObject tree_obj((std::shared_ptr<CollisionGeometry>(tree)));

  std::vector<CollisionObject*> boxes;
  generateBoxesFromOctomap(boxes, *tree);
  for(std::vector<CollisionObject*>::const_iterator cit = env.begin();
      cit != env.end(); ++cit)
  {
    fcl::CollisionRequest req(num_max_contacts, true);
    fcl::CollisionResult cResult;
    fcl::collide(&tree_obj, *cit, req, cResult);
    for(std::size_t index=0; index<cResult.numContacts(); ++index)
    {
      const Contact& contact = cResult.getContact(index);
      const fcl::BVHModel<fcl::OBBRSS>* surface = static_cast<const fcl::BVHModel<fcl::OBBRSS>*> (contact.o2);
      BOOST_CHECK(surface->num_tris > contact.b2);
    }
  }
}

void octomap_distance_test(double env_scale, std::size_t env_size, bool use_mesh, bool use_mesh_octomap)
{
  // srand(1);
  std::vector<CollisionObject*> env;
  if(use_mesh)
    generateEnvironmentsMesh(env, env_scale, env_size);
  else
    generateEnvironments(env, env_scale, env_size);

  OcTree* tree = new OcTree(std::shared_ptr<const octomap::OcTree>(generateOcTree()));
  CollisionObject tree_obj((std::shared_ptr<CollisionGeometry>(tree)));

  DynamicAABBTreeCollisionManager* manager = new DynamicAABBTreeCollisionManager();
  manager->registerObjects(env);
  manager->setup();
  
  DistanceData cdata;
  TStruct t1;
  Timer timer1;
  timer1.start();
  manager->octree_as_geometry_collide = false;
  manager->octree_as_geometry_distance = false;
  manager->distance(&tree_obj, &cdata, defaultDistanceFunction);
  timer1.stop();
  t1.push_back(timer1.getElapsedTime());

  
  DistanceData cdata3;
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
  std::vector<CollisionObject*> boxes;
  if(use_mesh_octomap)
    generateBoxesFromOctomapMesh(boxes, *tree);
  else
    generateBoxesFromOctomap(boxes, *tree);
  timer2.stop();
  t2.push_back(timer2.getElapsedTime());
  
  timer2.start();
  DynamicAABBTreeCollisionManager* manager2 = new DynamicAABBTreeCollisionManager();
  manager2->registerObjects(boxes);
  manager2->setup();
  timer2.stop();
  t2.push_back(timer2.getElapsedTime());


  DistanceData cdata2;

  timer2.start();
  manager->distance(manager2, &cdata2, defaultDistanceFunction);
  timer2.stop();
  t2.push_back(timer2.getElapsedTime());

  std::cout << cdata.result.min_distance << " " << cdata3.result.min_distance << " " << cdata2.result.min_distance << std::endl;

  if(cdata.result.min_distance < 0)
    BOOST_CHECK(cdata2.result.min_distance <= 0);
  else
    BOOST_CHECK(std::abs(cdata.result.min_distance - cdata2.result.min_distance) < 1e-3);

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



void generateBoxesFromOctomap(std::vector<CollisionObject*>& boxes, OcTree& tree)
{
  std::vector<std::array<FCL_REAL, 6> > boxes_ = tree.toBoxes();

  for(std::size_t i = 0; i < boxes_.size(); ++i)
  {
    FCL_REAL x = boxes_[i][0];
    FCL_REAL y = boxes_[i][1];
    FCL_REAL z = boxes_[i][2];
    FCL_REAL size = boxes_[i][3];
    FCL_REAL cost = boxes_[i][4];
    FCL_REAL threshold = boxes_[i][5];

    Box* box = new Box(size, size, size);
    box->cost_density = cost;
    box->threshold_occupied = threshold;
    CollisionObject* obj = new CollisionObject(std::shared_ptr<CollisionGeometry>(box), Transform3f(Vec3f(x, y, z)));
    boxes.push_back(obj);
  }

  std::cout << "boxes size: " << boxes.size() << std::endl;

}

void generateEnvironments(std::vector<CollisionObject*>& env, double env_scale, std::size_t n)
{
  FCL_REAL extents[] = {-env_scale, env_scale, -env_scale, env_scale, -env_scale, env_scale};
  std::vector<Transform3f> transforms;

  generateRandomTransforms(extents, transforms, n);
  for(std::size_t i = 0; i < n; ++i)
  {
    Box* box = new Box(5, 10, 20);
    env.push_back(new CollisionObject(std::shared_ptr<CollisionGeometry>(box), transforms[i]));
  }

  generateRandomTransforms(extents, transforms, n);
  for(std::size_t i = 0; i < n; ++i)
  {
    Sphere* sphere = new Sphere(30);
    env.push_back(new CollisionObject(std::shared_ptr<CollisionGeometry>(sphere), transforms[i]));
  }

  generateRandomTransforms(extents, transforms, n);
  for(std::size_t i = 0; i < n; ++i)
  {
    Cylinder* cylinder = new Cylinder(10, 40);
    env.push_back(new CollisionObject(std::shared_ptr<CollisionGeometry>(cylinder), transforms[i]));
  }

}


void generateBoxesFromOctomapMesh(std::vector<CollisionObject*>& boxes, OcTree& tree)
{
  std::vector<std::array<FCL_REAL, 6> > boxes_ = tree.toBoxes();

  for(std::size_t i = 0; i < boxes_.size(); ++i)
  {
    FCL_REAL x = boxes_[i][0];
    FCL_REAL y = boxes_[i][1];
    FCL_REAL z = boxes_[i][2];
    FCL_REAL size = boxes_[i][3];
    FCL_REAL cost = boxes_[i][4];
    FCL_REAL threshold = boxes_[i][5];

    Box box(size, size, size);
    BVHModel<OBBRSS>* model = new BVHModel<OBBRSS>();
    generateBVHModel(*model, box, Transform3f());
    model->cost_density = cost;
    model->threshold_occupied = threshold;
    CollisionObject* obj = new CollisionObject(std::shared_ptr<CollisionGeometry>(model), Transform3f(Vec3f(x, y, z)));
    boxes.push_back(obj);    
  }

  std::cout << "boxes size: " << boxes.size() << std::endl;
}

void generateEnvironmentsMesh(std::vector<CollisionObject*>& env, double env_scale, std::size_t n)
{
  FCL_REAL extents[] = {-env_scale, env_scale, -env_scale, env_scale, -env_scale, env_scale};
  std::vector<Transform3f> transforms;

  generateRandomTransforms(extents, transforms, n);
  Box box(5, 10, 20);
  for(std::size_t i = 0; i < n; ++i)
  {
    BVHModel<OBBRSS>* model = new BVHModel<OBBRSS>();
    generateBVHModel(*model, box, Transform3f());
    env.push_back(new CollisionObject(std::shared_ptr<CollisionGeometry>(model), transforms[i]));
  }

  generateRandomTransforms(extents, transforms, n);
  Sphere sphere(30);
  for(std::size_t i = 0; i < n; ++i)
  {
    BVHModel<OBBRSS>* model = new BVHModel<OBBRSS>();
    generateBVHModel(*model, sphere, Transform3f(), 16, 16);
    env.push_back(new CollisionObject(std::shared_ptr<CollisionGeometry>(model), transforms[i]));
  }

  generateRandomTransforms(extents, transforms, n);
  Cylinder cylinder(10, 40);
  for(std::size_t i = 0; i < n; ++i)
  {
    BVHModel<OBBRSS>* model = new BVHModel<OBBRSS>();
    generateBVHModel(*model, cylinder, Transform3f(), 16, 16);
    env.push_back(new CollisionObject(std::shared_ptr<CollisionGeometry>(model), transforms[i]));
  }
}


octomap::OcTree* generateOcTree()
{
  octomap::OcTree* tree = new octomap::OcTree(0.1);

  // insert some measurements of occupied cells
  for(int x = -20; x < 20; x++) 
  {
    for(int y = -20; y < 20; y++) 
    {
      for(int z = -20; z < 20; z++) 
      {
        tree->updateNode(octomap::point3d(x * 0.05, y * 0.05, z * 0.05), true);
      }
    }
  }

  // insert some measurements of free cells
  for(int x = -30; x < 30; x++) 
  {
    for(int y = -30; y < 30; y++) 
    {
      for(int z = -30; z < 30; z++) 
      {
        tree->updateNode(octomap::point3d(x*0.02 -1.0, y*0.02-1.0, z*0.02-1.0), false);
      }
    }
  }

  return tree;  
}



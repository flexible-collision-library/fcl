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


#define BOOST_TEST_MODULE "FCL_BROADPHASE"
#include <boost/test/unit_test.hpp>

#include "fcl/config.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"
#include "fcl/math/transform.h"
#include "test_fcl_utility.h"

#if USE_GOOGLEHASH
#include <sparsehash/sparse_hash_map>
#include <sparsehash/dense_hash_map>
#include <hash_map>
#endif

#include <iostream>
#include <iomanip>

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

/// @brief Generate environment with 3 * n objects: n boxes, n spheres and n cylinders.
void generateEnvironments(std::vector<CollisionObject*>& env, double env_scale, std::size_t n);

/// @brief Generate environment with 3 * n objects for self distance, so we try to make sure none of them collide with each other.
void generateSelfDistanceEnvironments(std::vector<CollisionObject*>& env, double env_scale, std::size_t n);

/// @brief Generate environment with 3 * n objects, but all in meshes.
void generateEnvironmentsMesh(std::vector<CollisionObject*>& env, double env_scale, std::size_t n);

/// @brief Generate environment with 3 * n objects for self distance, but all in meshes.
void generateSelfDistanceEnvironmentsMesh(std::vector<CollisionObject*>& env, double env_scale, std::size_t n);

/// @brief test for broad phase collision and self collision
void broad_phase_collision_test(double env_scale, std::size_t env_size, std::size_t query_size, std::size_t num_max_contacts = 1, bool exhaustive = false, bool use_mesh = false);

/// @brief test for broad phase distance
void broad_phase_distance_test(double env_scale, std::size_t env_size, std::size_t query_size, bool use_mesh = false);

/// @brief test for broad phase self distance
void broad_phase_self_distance_test(double env_scale, std::size_t env_size, bool use_mesh = false);

/// @brief test for broad phase update
void broad_phase_update_collision_test(double env_scale, std::size_t env_size, std::size_t query_size, std::size_t num_max_contacts = 1, bool exhaustive = false, bool use_mesh = false);

FCL_REAL DELTA = 0.01;


#if USE_GOOGLEHASH
template<typename U, typename V>
struct GoogleSparseHashTable : public google::sparse_hash_map<U, V, std::tr1::hash<size_t>, std::equal_to<size_t> > {};

template<typename U, typename V>
struct GoogleDenseHashTable : public google::dense_hash_map<U, V, std::tr1::hash<size_t>, std::equal_to<size_t> >
{
  GoogleDenseHashTable() : google::dense_hash_map<U, V, std::tr1::hash<size_t>, std::equal_to<size_t> >()
  {
    this->set_empty_key(NULL);
  }
};
#endif

/// check the update, only return collision or not
BOOST_AUTO_TEST_CASE(test_core_bf_broad_phase_update_collision_binary)
{
  broad_phase_update_collision_test(2000, 100, 1000, 1, false);
  broad_phase_update_collision_test(2000, 1000, 1000, 1, false);
}

/// check the update, return 10 contacts
BOOST_AUTO_TEST_CASE(test_core_bf_broad_phase_update_collision)
{
  broad_phase_update_collision_test(2000, 100, 1000, 10, false);
  broad_phase_update_collision_test(2000, 1000, 1000, 10, false);
}

/// check the update, exhaustive
BOOST_AUTO_TEST_CASE(test_core_bf_broad_phase_update_collision_exhaustive)
{
  broad_phase_update_collision_test(2000, 100, 1000, 1, true);
  broad_phase_update_collision_test(2000, 1000, 1000, 1, true);
}

/// check broad phase distance
BOOST_AUTO_TEST_CASE(test_core_bf_broad_phase_distance)
{
  broad_phase_distance_test(200, 100, 100);
  broad_phase_distance_test(200, 1000, 100);
  broad_phase_distance_test(2000, 100, 100);
  broad_phase_distance_test(2000, 1000, 100);
}

/// check broad phase self distance
BOOST_AUTO_TEST_CASE(test_core_bf_broad_phase_self_distance)
{
  broad_phase_self_distance_test(200, 512);
  broad_phase_self_distance_test(200, 1000);
  broad_phase_self_distance_test(200, 5000);
}

/// check broad phase collision for empty collision object set and queries
BOOST_AUTO_TEST_CASE(test_core_bf_broad_phase_collision_empty)
{
  broad_phase_collision_test(2000, 0, 0, 10, false, false);
  broad_phase_collision_test(2000, 0, 1000, 10, false, false);
  broad_phase_collision_test(2000, 100, 0, 10, false, false);

  broad_phase_collision_test(2000, 0, 0, 10, false, true);
  broad_phase_collision_test(2000, 0, 1000, 10, false, true);
  broad_phase_collision_test(2000, 100, 0, 10, false, true);

  broad_phase_collision_test(2000, 0, 0, 10, true, false);
  broad_phase_collision_test(2000, 0, 1000, 10, true, false);
  broad_phase_collision_test(2000, 100, 0, 10, true, false);

  broad_phase_collision_test(2000, 0, 0, 10, true, true);
  broad_phase_collision_test(2000, 0, 1000, 10, true, true);
  broad_phase_collision_test(2000, 100, 0, 10, true, true);
}

/// check broad phase collision and self collision, only return collision or not
BOOST_AUTO_TEST_CASE(test_core_bf_broad_phase_collision_binary)
{
#if FCL_BUILD_TYPE_DEBUG
  broad_phase_collision_test(2000, 10, 100, 1, false);
  broad_phase_collision_test(2000, 100, 100, 1, false);
  broad_phase_collision_test(2000, 10, 100, 1, true);
  broad_phase_collision_test(2000, 100, 100, 1, true);
#else
  broad_phase_collision_test(2000, 100, 1000, 1, false);
  broad_phase_collision_test(2000, 1000, 1000, 1, false);
  broad_phase_collision_test(2000, 100, 1000, 1, true);
  broad_phase_collision_test(2000, 1000, 1000, 1, true);
#endif
}

/// check broad phase collision and self collision, return 10 contacts
BOOST_AUTO_TEST_CASE(test_core_bf_broad_phase_collision)
{
  broad_phase_collision_test(2000, 100, 1000, 10, false);
  broad_phase_collision_test(2000, 1000, 1000, 10, false);
}

/// check broad phase update, in mesh, only return collision or not
BOOST_AUTO_TEST_CASE(test_core_mesh_bf_broad_phase_update_collision_mesh_binary)
{
  broad_phase_update_collision_test(2000, 100, 1000, 1, false, true);
  broad_phase_update_collision_test(2000, 1000, 1000, 1, false, true);
}

/// check broad phase update, in mesh, return 10 contacts
BOOST_AUTO_TEST_CASE(test_core_mesh_bf_broad_phase_update_collision_mesh)
{
#if FCL_BUILD_TYPE_DEBUG
  broad_phase_update_collision_test(200, 10, 100, 10, false, true);
  broad_phase_update_collision_test(200, 100, 100, 10, false, true);
#else
  broad_phase_update_collision_test(2000, 100, 1000, 10, false, true);
  broad_phase_update_collision_test(2000, 1000, 1000, 10, false, true);
#endif
}

/// check broad phase update, in mesh, exhaustive
BOOST_AUTO_TEST_CASE(test_core_mesh_bf_broad_phase_update_collision_mesh_exhaustive)
{
#if FCL_BUILD_TYPE_DEBUG
  broad_phase_update_collision_test(2000, 10, 100, 1, true, true);
  broad_phase_update_collision_test(2000, 100, 100, 1, true, true);
#else
  broad_phase_update_collision_test(2000, 100, 1000, 1, true, true);
  broad_phase_update_collision_test(2000, 1000, 1000, 1, true, true);
#endif
}

/// check broad phase distance
BOOST_AUTO_TEST_CASE(test_core_mesh_bf_broad_phase_distance_mesh)
{
#if FCL_BUILD_TYPE_DEBUG
  broad_phase_distance_test(200, 10, 10, true);
  broad_phase_distance_test(200, 100, 10, true);
  broad_phase_distance_test(2000, 10, 10, true);
  broad_phase_distance_test(2000, 100, 10, true);
#else
  broad_phase_distance_test(200, 100, 100, true);
  broad_phase_distance_test(200, 1000, 100, true);
  broad_phase_distance_test(2000, 100, 100, true);
  broad_phase_distance_test(2000, 1000, 100, true);
#endif
}

/// check broad phase self distance
BOOST_AUTO_TEST_CASE(test_core_mesh_bf_broad_phase_self_distance_mesh)
{
  broad_phase_self_distance_test(200, 512, true);
  broad_phase_self_distance_test(200, 1000, true);
  broad_phase_self_distance_test(200, 5000, true);
}

/// check broad phase collision and self collision, return only collision or not, in mesh
BOOST_AUTO_TEST_CASE(test_core_mesh_bf_broad_phase_collision_mesh_binary)
{
#if FCL_BUILD_TYPE_DEBUG
  broad_phase_collision_test(2000, 10, 100, 1, false, true);
  broad_phase_collision_test(2000, 100, 100, 1, false, true);
#else
  broad_phase_collision_test(2000, 100, 1000, 1, false, true);
  broad_phase_collision_test(2000, 1000, 1000, 1, false, true);
#endif
}

/// check broad phase collision and self collision, return 10 contacts, in mesh
BOOST_AUTO_TEST_CASE(test_core_mesh_bf_broad_phase_collision_mesh)
{
#if FCL_BUILD_TYPE_DEBUG
  broad_phase_collision_test(2000, 10, 100, 10, false, true);
  broad_phase_collision_test(2000, 100, 100, 10, false, true);
#else
  broad_phase_collision_test(2000, 100, 1000, 10, false, true);
  broad_phase_collision_test(2000, 1000, 1000, 10, false, true);
#endif
}

/// check broad phase collision and self collision, exhaustive, in mesh
BOOST_AUTO_TEST_CASE(test_core_mesh_bf_broad_phase_collision_mesh_exhaustive)
{
#if FCL_BUILD_TYPE_DEBUG
  broad_phase_collision_test(2000, 10, 100, 1, true, true);
  broad_phase_collision_test(2000, 100, 100, 1, true, true);
#else
  broad_phase_collision_test(2000, 100, 1000, 1, true, true);
  broad_phase_collision_test(2000, 1000, 1000, 1, true, true);
#endif
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

void generateSelfDistanceEnvironments(std::vector<CollisionObject*>& env, double env_scale, std::size_t n)
{
  unsigned int n_edge = std::floor(std::pow(n, 1/3.0));

  FCL_REAL step_size = env_scale * 2 / n_edge;
  FCL_REAL delta_size = step_size * 0.05;
  FCL_REAL single_size = step_size - 2 * delta_size;
  
  unsigned int i = 0;
  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Box* box = new Box(single_size, single_size, single_size);
    env.push_back(new CollisionObject(std::shared_ptr<CollisionGeometry>(box),
                                      Transform3f(Vec3f(x * step_size + delta_size + 0.5 * single_size - env_scale, 
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale))));
  }

  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Sphere* sphere = new Sphere(single_size / 2);
    env.push_back(new CollisionObject(std::shared_ptr<CollisionGeometry>(sphere),
                                      Transform3f(Vec3f(x * step_size + delta_size + 0.5 * single_size - env_scale, 
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale))));
  }

  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Ellipsoid* ellipsoid = new Ellipsoid(single_size / 2, single_size / 2, single_size / 2);
    env.push_back(new CollisionObject(std::shared_ptr<CollisionGeometry>(ellipsoid),
                                      Transform3f(Vec3f(x * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale))));
  }

  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Cylinder* cylinder = new Cylinder(single_size / 2, single_size);
    env.push_back(new CollisionObject(std::shared_ptr<CollisionGeometry>(cylinder),
                                      Transform3f(Vec3f(x * step_size + delta_size + 0.5 * single_size - env_scale, 
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale))));
  }

  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Cone* cone = new Cone(single_size / 2, single_size);
    env.push_back(new CollisionObject(std::shared_ptr<CollisionGeometry>(cone),
                                      Transform3f(Vec3f(x * step_size + delta_size + 0.5 * single_size - env_scale, 
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale))));
  }
}

void generateSelfDistanceEnvironmentsMesh(std::vector<CollisionObject*>& env, double env_scale, std::size_t n)
{
  unsigned int n_edge = std::floor(std::pow(n, 1/3.0));

  FCL_REAL step_size = env_scale * 2 / n_edge;
  FCL_REAL delta_size = step_size * 0.05;
  FCL_REAL single_size = step_size - 2 * delta_size;
  
  std::size_t i = 0;
  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Box box(single_size, single_size, single_size);
    BVHModel<OBBRSS>* model = new BVHModel<OBBRSS>();
    generateBVHModel(*model, box, Transform3f());
    env.push_back(new CollisionObject(std::shared_ptr<CollisionGeometry>(model),
                                      Transform3f(Vec3f(x * step_size + delta_size + 0.5 * single_size - env_scale, 
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale))));
  }

  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Sphere sphere(single_size / 2);
    BVHModel<OBBRSS>* model = new BVHModel<OBBRSS>();
    generateBVHModel(*model, sphere, Transform3f(), 16, 16);
    env.push_back(new CollisionObject(std::shared_ptr<CollisionGeometry>(model),
                                      Transform3f(Vec3f(x * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale))));
  }

  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Ellipsoid ellipsoid(single_size / 2, single_size / 2, single_size / 2);
    BVHModel<OBBRSS>* model = new BVHModel<OBBRSS>();
    generateBVHModel(*model, ellipsoid, Transform3f(), 16, 16);
    env.push_back(new CollisionObject(std::shared_ptr<CollisionGeometry>(model),
                                      Transform3f(Vec3f(x * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale))));
  }

  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Cylinder cylinder(single_size / 2, single_size);
    BVHModel<OBBRSS>* model = new BVHModel<OBBRSS>();
    generateBVHModel(*model, cylinder, Transform3f(), 16, 16);
    env.push_back(new CollisionObject(std::shared_ptr<CollisionGeometry>(model),
                                      Transform3f(Vec3f(x * step_size + delta_size + 0.5 * single_size - env_scale, 
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale))));
  }

  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Cone cone(single_size / 2, single_size);
    BVHModel<OBBRSS>* model = new BVHModel<OBBRSS>();
    generateBVHModel(*model, cone, Transform3f(), 16, 16);
    env.push_back(new CollisionObject(std::shared_ptr<CollisionGeometry>(model),
                                      Transform3f(Vec3f(x * step_size + delta_size + 0.5 * single_size - env_scale, 
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale))));
  }
}


void broad_phase_collision_test(double env_scale, std::size_t env_size, std::size_t query_size, std::size_t num_max_contacts, bool exhaustive, bool use_mesh)
{
  std::vector<TStruct> ts;
  std::vector<Timer> timers;

  std::vector<CollisionObject*> env;
  if(use_mesh)
    generateEnvironmentsMesh(env, env_scale, env_size);
  else
    generateEnvironments(env, env_scale, env_size);

  std::vector<CollisionObject*> query;
  if(use_mesh)
    generateEnvironmentsMesh(query, env_scale, query_size);
  else
    generateEnvironments(query, env_scale, query_size);

  std::vector<BroadPhaseCollisionManager*> managers;
  
  managers.push_back(new NaiveCollisionManager());
  managers.push_back(new SSaPCollisionManager());
  managers.push_back(new SaPCollisionManager());
  managers.push_back(new IntervalTreeCollisionManager());
  
  Vec3f lower_limit, upper_limit;
  SpatialHashingCollisionManager<>::computeBound(env, lower_limit, upper_limit);
  // FCL_REAL ncell_per_axis = std::pow((FCL_REAL)env_size / 10, 1 / 3.0);
  FCL_REAL ncell_per_axis = 20;
  FCL_REAL cell_size = std::min(std::min((upper_limit[0] - lower_limit[0]) / ncell_per_axis, (upper_limit[1] - lower_limit[1]) / ncell_per_axis), (upper_limit[2] - lower_limit[2]) / ncell_per_axis);
  // managers.push_back(new SpatialHashingCollisionManager<>(cell_size, lower_limit, upper_limit));
  managers.push_back(new SpatialHashingCollisionManager<SparseHashTable<AABB, CollisionObject*, SpatialHash> >(cell_size, lower_limit, upper_limit));
#if USE_GOOGLEHASH
  managers.push_back(new SpatialHashingCollisionManager<SparseHashTable<AABB, CollisionObject*, SpatialHash, GoogleSparseHashTable> >(cell_size, lower_limit, upper_limit));
  managers.push_back(new SpatialHashingCollisionManager<SparseHashTable<AABB, CollisionObject*, SpatialHash, GoogleDenseHashTable> >(cell_size, lower_limit, upper_limit));
#endif
  managers.push_back(new DynamicAABBTreeCollisionManager());

  managers.push_back(new DynamicAABBTreeCollisionManager_Array());

  {
    DynamicAABBTreeCollisionManager* m = new DynamicAABBTreeCollisionManager();
    m->tree_init_level = 2;
    managers.push_back(m);
  }

  {
    DynamicAABBTreeCollisionManager_Array* m = new DynamicAABBTreeCollisionManager_Array();
    m->tree_init_level = 2;
    managers.push_back(m);
  }

  ts.resize(managers.size());
  timers.resize(managers.size());

  for(size_t i = 0; i < managers.size(); ++i)
  {
    timers[i].start();
    managers[i]->registerObjects(env);
    timers[i].stop();
    ts[i].push_back(timers[i].getElapsedTime());
  }

  for(size_t i = 0; i < managers.size(); ++i)
  {
    timers[i].start();
    managers[i]->setup();
    timers[i].stop();
    ts[i].push_back(timers[i].getElapsedTime());
  }

  std::vector<CollisionData> self_data(managers.size());
  for(size_t i = 0; i < managers.size(); ++i)
  {
    if(exhaustive) self_data[i].request.num_max_contacts = 100000;
    else self_data[i].request.num_max_contacts = num_max_contacts;
  }

  for(size_t i = 0; i < managers.size(); ++i)
  {
    timers[i].start();
    managers[i]->collide(&self_data[i], defaultCollisionFunction);
    timers[i].stop();
    ts[i].push_back(timers[i].getElapsedTime());
  }

  for(size_t i = 0; i < managers.size(); ++i)
    std::cout << self_data[i].result.numContacts() << " ";
  std::cout << std::endl;

  if(exhaustive)
  {
    for(size_t i = 1; i < managers.size(); ++i)
      BOOST_CHECK(self_data[i].result.numContacts() == self_data[0].result.numContacts());
  }
  else
  {
    std::vector<bool> self_res(managers.size());
    for(size_t i = 0; i < self_res.size(); ++i)
      self_res[i] = (self_data[i].result.numContacts() > 0);
  
    for(size_t i = 1; i < self_res.size(); ++i)
      BOOST_CHECK(self_res[0] == self_res[i]);

    for(size_t i = 1; i < managers.size(); ++i)
      BOOST_CHECK(self_data[i].result.numContacts() == self_data[0].result.numContacts());
  }


  for(size_t i = 0; i < query.size(); ++i)
  {
    std::vector<CollisionData> query_data(managers.size());
    for(size_t j = 0; j < query_data.size(); ++j)
    {
      if(exhaustive) query_data[j].request.num_max_contacts = 100000;
      else query_data[j].request.num_max_contacts = num_max_contacts;
    }

    for(size_t j = 0; j < query_data.size(); ++j)
    {
      timers[j].start();
      managers[j]->collide(query[i], &query_data[j], defaultCollisionFunction);
      timers[j].stop();
      ts[j].push_back(timers[j].getElapsedTime());
    }


    // for(size_t j = 0; j < managers.size(); ++j)
    //   std::cout << query_data[j].result.numContacts() << " ";
    // std::cout << std::endl;
    
    if(exhaustive)
    {
      for(size_t j = 1; j < managers.size(); ++j)
        BOOST_CHECK(query_data[j].result.numContacts() == query_data[0].result.numContacts());
    }
    else
    {
      std::vector<bool> query_res(managers.size());
      for(size_t j = 0; j < query_res.size(); ++j)
        query_res[j] = (query_data[j].result.numContacts() > 0);
      for(size_t j = 1; j < query_res.size(); ++j)
        BOOST_CHECK(query_res[0] == query_res[j]);

      for(size_t j = 1; j < managers.size(); ++j)
        BOOST_CHECK(query_data[j].result.numContacts() == query_data[0].result.numContacts());
    }
  }

  for(size_t i = 0; i < env.size(); ++i)
    delete env[i];
  for(size_t i = 0; i < query.size(); ++i)
    delete query[i];

  for(size_t i = 0; i < managers.size(); ++i)
    delete managers[i];     

  std::cout.setf(std::ios_base::left, std::ios_base::adjustfield);
  size_t w = 7;

  std::cout << "collision timing summary" << std::endl;
  std::cout << env_size << " objs, " << query_size << " queries" << std::endl;
  std::cout << "register time" << std::endl;
  for(size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[0] << " ";
  std::cout << std::endl;

  std::cout << "setup time" << std::endl;
  for(size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[1] << " ";
  std::cout << std::endl;

  std::cout << "self collision time" << std::endl;
  for(size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[2] << " ";
  std::cout << std::endl;

  std::cout << "collision time" << std::endl;
  for(size_t i = 0; i < ts.size(); ++i)
  {
    FCL_REAL tmp = 0;
    for(size_t j = 3; j < ts[i].records.size(); ++j)
      tmp += ts[i].records[j];
    std::cout << std::setw(w) << tmp << " ";
  }
  std::cout << std::endl;


  std::cout << "overall time" << std::endl;
  for(size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].overall_time << " ";
  std::cout << std::endl;
  std::cout << std::endl;

}

void broad_phase_self_distance_test(double env_scale, std::size_t env_size, bool use_mesh)
{
  std::vector<TStruct> ts;
  std::vector<Timer> timers;

  std::vector<CollisionObject*> env;
  if(use_mesh)
    generateSelfDistanceEnvironmentsMesh(env, env_scale, env_size);
  else
    generateSelfDistanceEnvironments(env, env_scale, env_size);
  
  std::vector<BroadPhaseCollisionManager*> managers;
  
  managers.push_back(new NaiveCollisionManager());
  managers.push_back(new SSaPCollisionManager());
  managers.push_back(new SaPCollisionManager());
  managers.push_back(new IntervalTreeCollisionManager());
  
  Vec3f lower_limit, upper_limit;
  SpatialHashingCollisionManager<>::computeBound(env, lower_limit, upper_limit);
  FCL_REAL cell_size = std::min(std::min((upper_limit[0] - lower_limit[0]) / 5, (upper_limit[1] - lower_limit[1]) / 5), (upper_limit[2] - lower_limit[2]) / 5);
  // managers.push_back(new SpatialHashingCollisionManager<>(cell_size, lower_limit, upper_limit));
  managers.push_back(new SpatialHashingCollisionManager<SparseHashTable<AABB, CollisionObject*, SpatialHash> >(cell_size, lower_limit, upper_limit));
#if USE_GOOGLEHASH
  managers.push_back(new SpatialHashingCollisionManager<SparseHashTable<AABB, CollisionObject*, SpatialHash, GoogleSparseHashTable> >(cell_size, lower_limit, upper_limit));
  managers.push_back(new SpatialHashingCollisionManager<SparseHashTable<AABB, CollisionObject*, SpatialHash, GoogleDenseHashTable> >(cell_size, lower_limit, upper_limit));
#endif
  managers.push_back(new DynamicAABBTreeCollisionManager());
  managers.push_back(new DynamicAABBTreeCollisionManager_Array());

  {
    DynamicAABBTreeCollisionManager* m = new DynamicAABBTreeCollisionManager();
    m->tree_init_level = 2;
    managers.push_back(m);
  }

  {
    DynamicAABBTreeCollisionManager_Array* m = new DynamicAABBTreeCollisionManager_Array();
    m->tree_init_level = 2;
    managers.push_back(m);
  }

  ts.resize(managers.size());
  timers.resize(managers.size());

  for(size_t i = 0; i < managers.size(); ++i)
  {
    timers[i].start();
    managers[i]->registerObjects(env);
    timers[i].stop();
    ts[i].push_back(timers[i].getElapsedTime());
  }

  for(size_t i = 0; i < managers.size(); ++i)
  {
    timers[i].start();
    managers[i]->setup();
    timers[i].stop();
    ts[i].push_back(timers[i].getElapsedTime());
  }
 

  std::vector<DistanceData> self_data(managers.size());
                                      
  for(size_t i = 0; i < self_data.size(); ++i)
  {
    timers[i].start();
    managers[i]->distance(&self_data[i], defaultDistanceFunction);
    timers[i].stop();
    ts[i].push_back(timers[i].getElapsedTime());
    // std::cout << self_data[i].result.min_distance << " ";
  }
  // std::cout << std::endl;

  for(size_t i = 1; i < managers.size(); ++i)
    BOOST_CHECK(fabs(self_data[0].result.min_distance - self_data[i].result.min_distance) < DELTA ||
                fabs(self_data[0].result.min_distance - self_data[i].result.min_distance) / fabs(self_data[0].result.min_distance) < DELTA);

  for(size_t i = 0; i < env.size(); ++i)
    delete env[i];

  for(size_t i = 0; i < managers.size(); ++i)
    delete managers[i];     

  std::cout.setf(std::ios_base::left, std::ios_base::adjustfield);
  size_t w = 7;

  std::cout << "self distance timing summary" << std::endl;
  std::cout << env.size() << " objs" << std::endl;
  std::cout << "register time" << std::endl;
  for(size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[0] << " ";
  std::cout << std::endl;

  std::cout << "setup time" << std::endl;
  for(size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[1] << " ";
  std::cout << std::endl;

  std::cout << "self distance time" << std::endl;
  for(size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[2] << " ";
  std::cout << std::endl;

  std::cout << "overall time" << std::endl;
  for(size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].overall_time << " ";
  std::cout << std::endl;
  std::cout << std::endl;
}


void broad_phase_distance_test(double env_scale, std::size_t env_size, std::size_t query_size, bool use_mesh)
{
  std::vector<TStruct> ts;
  std::vector<Timer> timers;

  std::vector<CollisionObject*> env;
  if(use_mesh)
    generateEnvironmentsMesh(env, env_scale, env_size);
  else
    generateEnvironments(env, env_scale, env_size);

  std::vector<CollisionObject*> query;

  BroadPhaseCollisionManager* manager = new NaiveCollisionManager();
  for(std::size_t i = 0; i < env.size(); ++i)
    manager->registerObject(env[i]);
  manager->setup();

  while(1)
  {
    std::vector<CollisionObject*> candidates;
    if(use_mesh)
      generateEnvironmentsMesh(candidates, env_scale, query_size);
    else
      generateEnvironments(candidates, env_scale, query_size);

    for(std::size_t i = 0; i < candidates.size(); ++i)
    {
      CollisionData query_data;
      manager->collide(candidates[i], &query_data, defaultCollisionFunction);
      if(query_data.result.numContacts() == 0)
        query.push_back(candidates[i]);
      else
        delete candidates[i];
      if(query.size() == query_size) break;
    }

    if(query.size() == query_size) break;
  }

  delete manager;

  std::vector<BroadPhaseCollisionManager*> managers;

  managers.push_back(new NaiveCollisionManager());
  managers.push_back(new SSaPCollisionManager());
  managers.push_back(new SaPCollisionManager());
  managers.push_back(new IntervalTreeCollisionManager());
  
  Vec3f lower_limit, upper_limit;
  SpatialHashingCollisionManager<>::computeBound(env, lower_limit, upper_limit);
  FCL_REAL cell_size = std::min(std::min((upper_limit[0] - lower_limit[0]) / 20, (upper_limit[1] - lower_limit[1]) / 20), (upper_limit[2] - lower_limit[2])/20);
  // managers.push_back(new SpatialHashingCollisionManager<>(cell_size, lower_limit, upper_limit));
  managers.push_back(new SpatialHashingCollisionManager<SparseHashTable<AABB, CollisionObject*, SpatialHash> >(cell_size, lower_limit, upper_limit));
#if USE_GOOGLEHASH
  managers.push_back(new SpatialHashingCollisionManager<SparseHashTable<AABB, CollisionObject*, SpatialHash, GoogleSparseHashTable> >(cell_size, lower_limit, upper_limit));
  managers.push_back(new SpatialHashingCollisionManager<SparseHashTable<AABB, CollisionObject*, SpatialHash, GoogleDenseHashTable> >(cell_size, lower_limit, upper_limit));
#endif
  managers.push_back(new DynamicAABBTreeCollisionManager());
  managers.push_back(new DynamicAABBTreeCollisionManager_Array());

  {
    DynamicAABBTreeCollisionManager* m = new DynamicAABBTreeCollisionManager();
    m->tree_init_level = 2;
    managers.push_back(m);
  }

  {
    DynamicAABBTreeCollisionManager_Array* m = new DynamicAABBTreeCollisionManager_Array();
    m->tree_init_level = 2;
    managers.push_back(m);
  }

  ts.resize(managers.size());
  timers.resize(managers.size());

  for(size_t i = 0; i < managers.size(); ++i)
  {
    timers[i].start();
    managers[i]->registerObjects(env);
    timers[i].stop();
    ts[i].push_back(timers[i].getElapsedTime());
  }

  for(size_t i = 0; i < managers.size(); ++i)
  {
    timers[i].start();
    managers[i]->setup();
    timers[i].stop();
    ts[i].push_back(timers[i].getElapsedTime());
  }


  for(size_t i = 0; i < query.size(); ++i)
  {
    std::vector<DistanceData> query_data(managers.size());
    for(size_t j = 0; j < managers.size(); ++j)
    {
      timers[j].start();
      managers[j]->distance(query[i], &query_data[j], defaultDistanceFunction);
      timers[j].stop();
      ts[j].push_back(timers[j].getElapsedTime());
      // std::cout << query_data[j].result.min_distance << " ";
    }
    // std::cout << std::endl;

    for(size_t j = 1; j < managers.size(); ++j)
      BOOST_CHECK(fabs(query_data[0].result.min_distance - query_data[j].result.min_distance) < DELTA ||
                  fabs(query_data[0].result.min_distance - query_data[j].result.min_distance) / fabs(query_data[0].result.min_distance) < DELTA);
  }


  for(std::size_t i = 0; i < env.size(); ++i)
    delete env[i];
  for(std::size_t i = 0; i < query.size(); ++i)
    delete query[i];

  for(size_t i = 0; i < managers.size(); ++i)
    delete managers[i];


  std::cout.setf(std::ios_base::left, std::ios_base::adjustfield);
  size_t w = 7;

  std::cout << "distance timing summary" << std::endl;
  std::cout << env_size << " objs, " << query_size << " queries" << std::endl;
  std::cout << "register time" << std::endl;
  for(size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[0] << " ";
  std::cout << std::endl;

  std::cout << "setup time" << std::endl;
  for(size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[1] << " ";
  std::cout << std::endl;

  std::cout << "distance time" << std::endl;
  for(size_t i = 0; i < ts.size(); ++i)
  {
    FCL_REAL tmp = 0;
    for(size_t j = 2; j < ts[i].records.size(); ++j)
      tmp += ts[i].records[j];
    std::cout << std::setw(w) << tmp << " ";
  }
  std::cout << std::endl;

  std::cout << "overall time" << std::endl;
  for(size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].overall_time << " ";
  std::cout << std::endl;
  std::cout << std::endl;
}


void broad_phase_update_collision_test(double env_scale, std::size_t env_size, std::size_t query_size, std::size_t num_max_contacts, bool exhaustive, bool use_mesh)
{
  std::vector<TStruct> ts;
  std::vector<Timer> timers;

  std::vector<CollisionObject*> env;
  if(use_mesh)
    generateEnvironmentsMesh(env, env_scale, env_size);
  else
    generateEnvironments(env, env_scale, env_size);

  std::vector<CollisionObject*> query;
  if(use_mesh)
    generateEnvironmentsMesh(query, env_scale, query_size); 
  else
    generateEnvironments(query, env_scale, query_size); 

  std::vector<BroadPhaseCollisionManager*> managers;
  
  managers.push_back(new NaiveCollisionManager());
  managers.push_back(new SSaPCollisionManager());

  
  managers.push_back(new SaPCollisionManager());
  managers.push_back(new IntervalTreeCollisionManager());
  
  Vec3f lower_limit, upper_limit;
  SpatialHashingCollisionManager<>::computeBound(env, lower_limit, upper_limit);
  FCL_REAL cell_size = std::min(std::min((upper_limit[0] - lower_limit[0]) / 20, (upper_limit[1] - lower_limit[1]) / 20), (upper_limit[2] - lower_limit[2])/20);
  // managers.push_back(new SpatialHashingCollisionManager<>(cell_size, lower_limit, upper_limit));
  managers.push_back(new SpatialHashingCollisionManager<SparseHashTable<AABB, CollisionObject*, SpatialHash> >(cell_size, lower_limit, upper_limit));
#if USE_GOOGLEHASH
  managers.push_back(new SpatialHashingCollisionManager<SparseHashTable<AABB, CollisionObject*, SpatialHash, GoogleSparseHashTable> >(cell_size, lower_limit, upper_limit));
  managers.push_back(new SpatialHashingCollisionManager<SparseHashTable<AABB, CollisionObject*, SpatialHash, GoogleDenseHashTable> >(cell_size, lower_limit, upper_limit));
#endif
  managers.push_back(new DynamicAABBTreeCollisionManager());
  managers.push_back(new DynamicAABBTreeCollisionManager_Array());

  {
    DynamicAABBTreeCollisionManager* m = new DynamicAABBTreeCollisionManager();
    m->tree_init_level = 2;
    managers.push_back(m);
  }

  {
    DynamicAABBTreeCollisionManager_Array* m = new DynamicAABBTreeCollisionManager_Array();
    m->tree_init_level = 2;
    managers.push_back(m);
  }

  ts.resize(managers.size());
  timers.resize(managers.size());

  for(size_t i = 0; i < managers.size(); ++i)
  {
    timers[i].start();
    managers[i]->registerObjects(env);
    timers[i].stop();
    ts[i].push_back(timers[i].getElapsedTime());
  }
  
  for(size_t i = 0; i < managers.size(); ++i)
  {
    timers[i].start();
    managers[i]->setup();
    timers[i].stop();
    ts[i].push_back(timers[i].getElapsedTime());
  }

  // update the environment
  FCL_REAL delta_angle_max = 10 / 360.0 * 2 * constants::pi;
  FCL_REAL delta_trans_max = 0.01 * env_scale;
  for(size_t i = 0; i < env.size(); ++i)
  {
    FCL_REAL rand_angle_x = 2 * (rand() / (FCL_REAL)RAND_MAX - 0.5) * delta_angle_max;
    FCL_REAL rand_trans_x = 2 * (rand() / (FCL_REAL)RAND_MAX - 0.5) * delta_trans_max;
    FCL_REAL rand_angle_y = 2 * (rand() / (FCL_REAL)RAND_MAX - 0.5) * delta_angle_max;
    FCL_REAL rand_trans_y = 2 * (rand() / (FCL_REAL)RAND_MAX - 0.5) * delta_trans_max;
    FCL_REAL rand_angle_z = 2 * (rand() / (FCL_REAL)RAND_MAX - 0.5) * delta_angle_max;
    FCL_REAL rand_trans_z = 2 * (rand() / (FCL_REAL)RAND_MAX - 0.5) * delta_trans_max;

    Quaternion3f q1, q2, q3;
    q1.fromAxisAngle(Vec3f(1, 0, 0), rand_angle_x);
    q2.fromAxisAngle(Vec3f(0, 1, 0), rand_angle_y);
    q3.fromAxisAngle(Vec3f(0, 0, 1), rand_angle_z);
    Quaternion3f q = q1 * q2 * q3;
    Matrix3f dR;
    q.toRotation(dR);
    Vec3f dT(rand_trans_x, rand_trans_y, rand_trans_z);
    
    Matrix3f R = env[i]->getRotation();
    Vec3f T = env[i]->getTranslation();
    env[i]->setTransform(dR * R, dR * T + dT);
    env[i]->computeAABB();
  }

  for(size_t i = 0; i < managers.size(); ++i)
  {
    timers[i].start();
    managers[i]->update();
    timers[i].stop();
    ts[i].push_back(timers[i].getElapsedTime());
  }

  std::vector<CollisionData> self_data(managers.size());
  for(size_t i = 0; i < managers.size(); ++i)
  {
    if(exhaustive) self_data[i].request.num_max_contacts = 100000;
    else self_data[i].request.num_max_contacts = num_max_contacts;
  }

  for(size_t i = 0; i < managers.size(); ++i)
  {
    timers[i].start();
    managers[i]->collide(&self_data[i], defaultCollisionFunction);
    timers[i].stop();
    ts[i].push_back(timers[i].getElapsedTime());
  }


  for(size_t i = 0; i < managers.size(); ++i)
    std::cout << self_data[i].result.numContacts() << " ";
  std::cout << std::endl;

  if(exhaustive)
  {
    for(size_t i = 1; i < managers.size(); ++i)
      BOOST_CHECK(self_data[i].result.numContacts() == self_data[0].result.numContacts());
  }
  else
  {
    std::vector<bool> self_res(managers.size());
    for(size_t i = 0; i < self_res.size(); ++i)
      self_res[i] = (self_data[i].result.numContacts() > 0);
  
    for(size_t i = 1; i < self_res.size(); ++i)
      BOOST_CHECK(self_res[0] == self_res[i]);

    for(size_t i = 1; i < managers.size(); ++i)
      BOOST_CHECK(self_data[i].result.numContacts() == self_data[0].result.numContacts());
  }


  for(size_t i = 0; i < query.size(); ++i)
  {
    std::vector<CollisionData> query_data(managers.size());
    for(size_t j = 0; j < query_data.size(); ++j)
    {
      if(exhaustive) query_data[j].request.num_max_contacts = 100000;
      else query_data[j].request.num_max_contacts = num_max_contacts;
    }

    for(size_t j = 0; j < query_data.size(); ++j)
    {
      timers[j].start();
      managers[j]->collide(query[i], &query_data[j], defaultCollisionFunction);
      timers[j].stop();
      ts[j].push_back(timers[j].getElapsedTime());
    }


    // for(size_t j = 0; j < managers.size(); ++j)
    //   std::cout << query_data[j].result.numContacts() << " ";
    // std::cout << std::endl;
    
    if(exhaustive)
    {
      for(size_t j = 1; j < managers.size(); ++j)
        BOOST_CHECK(query_data[j].result.numContacts() == query_data[0].result.numContacts());
    }
    else
    {
      std::vector<bool> query_res(managers.size());
      for(size_t j = 0; j < query_res.size(); ++j)
        query_res[j] = (query_data[j].result.numContacts() > 0);
      for(size_t j = 1; j < query_res.size(); ++j)
        BOOST_CHECK(query_res[0] == query_res[j]);

      for(size_t j = 1; j < managers.size(); ++j)
        BOOST_CHECK(query_data[j].result.numContacts() == query_data[0].result.numContacts());
    }
  }


  for(size_t i = 0; i < env.size(); ++i)
    delete env[i];
  for(size_t i = 0; i < query.size(); ++i)
    delete query[i];

  for(size_t i = 0; i < managers.size(); ++i)
    delete managers[i];
  

  std::cout.setf(std::ios_base::left, std::ios_base::adjustfield);
  size_t w = 7;

  std::cout << "collision timing summary" << std::endl;
  std::cout << env_size << " objs, " << query_size << " queries" << std::endl;
  std::cout << "register time" << std::endl;
  for(size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[0] << " ";
  std::cout << std::endl;

  std::cout << "setup time" << std::endl;
  for(size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[1] << " ";
  std::cout << std::endl;

  std::cout << "update time" << std::endl;
  for(size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[2] << " ";
  std::cout << std::endl;

  std::cout << "self collision time" << std::endl;
  for(size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[3] << " ";
  std::cout << std::endl;

  std::cout << "collision time" << std::endl;
  for(size_t i = 0; i < ts.size(); ++i)
  {
    FCL_REAL tmp = 0;
    for(size_t j = 4; j < ts[i].records.size(); ++j)
      tmp += ts[i].records[j];
    std::cout << std::setw(w) << tmp << " ";
  }
  std::cout << std::endl;


  std::cout << "overall time" << std::endl;
  for(size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].overall_time << " ";
  std::cout << std::endl;
  std::cout << std::endl;
}




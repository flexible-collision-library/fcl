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
#include "fcl/broadphase/broadphase_bruteforce.h"
#include "fcl/broadphase/broadphase_spatialhash.h"
#include "fcl/broadphase/broadphase_SaP.h"
#include "fcl/broadphase/broadphase_SSaP.h"
#include "fcl/broadphase/broadphase_interval_tree.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree_array.h"
#include "fcl/broadphase/default_broadphase_callbacks.h"
#include "fcl/broadphase/detail/sparse_hash_table.h"
#include "fcl/broadphase/detail/spatial_hash.h"
#include "fcl/geometry/geometric_shape_to_BVH_model.h"
#include "test_fcl_utility.h"

#if USE_GOOGLEHASH
#include <sparsehash/sparse_hash_map>
#include <sparsehash/dense_hash_map>
#include <hash_map>
#endif

#include <iostream>
#include <iomanip>

using namespace fcl;

/// @brief Generate environment with 3 * n objects for self distance, so we try to make sure none of them collide with each other.
template <typename S>
void generateSelfDistanceEnvironments(std::vector<CollisionObject<S>*>& env, S env_scale, std::size_t n);

/// @brief Generate environment with 3 * n objects for self distance, but all in meshes.
template <typename S>
void generateSelfDistanceEnvironmentsMesh(std::vector<CollisionObject<S>*>& env, S env_scale, std::size_t n);

/// @brief test for broad phase distance
template <typename S>
void broad_phase_distance_test(S env_scale, std::size_t env_size, std::size_t query_size, bool use_mesh = false);

/// @brief test for broad phase self distance
template <typename S>
void broad_phase_self_distance_test(S env_scale, std::size_t env_size, bool use_mesh = false);

template <typename S>
S getDELTA() { return 0.01; }

#if USE_GOOGLEHASH
template<typename U, typename V>
struct GoogleSparseHashTable : public google::sparse_hash_map<U, V, std::tr1::hash<size_t>, std::equal_to<size_t> > {};

template<typename U, typename V>
struct GoogleDenseHashTable : public google::dense_hash_map<U, V, std::tr1::hash<size_t>, std::equal_to<size_t> >
{
  GoogleDenseHashTable() : google::dense_hash_map<U, V, std::tr1::hash<size_t>, std::equal_to<size_t> >()
  {
    this->set_empty_key(nullptr);
  }
};
#endif

/// check broad phase distance
GTEST_TEST(FCL_BROADPHASE, test_core_bf_broad_phase_distance)
{
#ifdef NDEBUG
  broad_phase_distance_test<double>(200, 100, 100);
  broad_phase_distance_test<double>(200, 1000, 100);
  broad_phase_distance_test<double>(2000, 100, 100);
  broad_phase_distance_test<double>(2000, 1000, 100);
#else
  broad_phase_distance_test<double>(200, 10, 10);
  broad_phase_distance_test<double>(200, 100, 10);
  broad_phase_distance_test<double>(2000, 10, 10);
  broad_phase_distance_test<double>(2000, 100, 10);
#endif
}

/// check broad phase self distance
GTEST_TEST(FCL_BROADPHASE, test_core_bf_broad_phase_self_distance)
{
#ifdef NDEBUG
  broad_phase_self_distance_test<double>(200, 512);
  broad_phase_self_distance_test<double>(200, 1000);
  broad_phase_self_distance_test<double>(200, 5000);
#else
  broad_phase_self_distance_test<double>(200, 256);
  broad_phase_self_distance_test<double>(200, 500);
  broad_phase_self_distance_test<double>(200, 2500);
#endif
}

/// check broad phase distance
GTEST_TEST(FCL_BROADPHASE, test_core_mesh_bf_broad_phase_distance_mesh)
{
#ifdef NDEBUG
  broad_phase_distance_test<double>(200, 100, 100, true);
  broad_phase_distance_test<double>(200, 1000, 100, true);
  broad_phase_distance_test<double>(2000, 100, 100, true);
  broad_phase_distance_test<double>(2000, 1000, 100, true);
#else
  broad_phase_distance_test<double>(200, 2, 2, true);
  broad_phase_distance_test<double>(200, 4, 2, true);
  broad_phase_distance_test<double>(2000, 2, 2, true);
  broad_phase_distance_test<double>(2000, 4, 2, true);
#endif
}

/// check broad phase self distance
GTEST_TEST(FCL_BROADPHASE, test_core_mesh_bf_broad_phase_self_distance_mesh)
{
#ifdef NDEBUG
  broad_phase_self_distance_test<double>(200, 512, true);
  broad_phase_self_distance_test<double>(200, 1000, true);
  broad_phase_self_distance_test<double>(200, 5000, true);
#else
  broad_phase_self_distance_test<double>(200, 128, true);
  broad_phase_self_distance_test<double>(200, 250, true);
  broad_phase_self_distance_test<double>(200, 1250, true);
#endif
}

template <typename S>
void generateSelfDistanceEnvironments(std::vector<CollisionObject<S>*>& env, S env_scale, std::size_t n)
{
  unsigned int n_edge = std::floor(std::pow(n, 1/3.0));

  S step_size = env_scale * 2 / n_edge;
  S delta_size = step_size * 0.05;
  S single_size = step_size - 2 * delta_size;

  unsigned int i = 0;
  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Box<S>* box = new Box<S>(single_size, single_size, single_size);
    env.push_back(new CollisionObject<S>(std::shared_ptr<CollisionGeometry<S>>(box),
                                      Transform3<S>(Translation3<S>(Vector3<S>(x * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale)))));
  }

  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Sphere<S>* sphere = new Sphere<S>(single_size / 2);
    env.push_back(new CollisionObject<S>(std::shared_ptr<CollisionGeometry<S>>(sphere),
                                      Transform3<S>(Translation3<S>(Vector3<S>(x * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale)))));
  }

  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Ellipsoid<S>* ellipsoid = new Ellipsoid<S>(single_size / 2, single_size / 2, single_size / 2);
    env.push_back(new CollisionObject<S>(std::shared_ptr<CollisionGeometry<S>>(ellipsoid),
                                      Transform3<S>(Translation3<S>(Vector3<S>(x * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale)))));
  }

  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Cylinder<S>* cylinder = new Cylinder<S>(single_size / 2, single_size);
    env.push_back(new CollisionObject<S>(std::shared_ptr<CollisionGeometry<S>>(cylinder),
                                      Transform3<S>(Translation3<S>(Vector3<S>(x * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale)))));
  }

  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Cone<S>* cone = new Cone<S>(single_size / 2, single_size);
    env.push_back(new CollisionObject<S>(std::shared_ptr<CollisionGeometry<S>>(cone),
                                      Transform3<S>(Translation3<S>(Vector3<S>(x * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale)))));
  }
}

template <typename S>
void generateSelfDistanceEnvironmentsMesh(std::vector<CollisionObject<S>*>& env, S env_scale, std::size_t n)
{
  unsigned int n_edge = std::floor(std::pow(n, 1/3.0));

  S step_size = env_scale * 2 / n_edge;
  S delta_size = step_size * 0.05;
  S single_size = step_size - 2 * delta_size;

  std::size_t i = 0;
  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Box<S> box(single_size, single_size, single_size);
    BVHModel<OBBRSS<S>>* model = new BVHModel<OBBRSS<S>>();
    generateBVHModel(*model, box, Transform3<S>::Identity());
    env.push_back(new CollisionObject<S>(std::shared_ptr<CollisionGeometry<S>>(model),
                                      Transform3<S>(Translation3<S>(Vector3<S>(x * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale)))));
  }

  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Sphere<S> sphere(single_size / 2);
    BVHModel<OBBRSS<S>>* model = new BVHModel<OBBRSS<S>>();
    generateBVHModel(*model, sphere, Transform3<S>::Identity(), 16, 16);
    env.push_back(new CollisionObject<S>(std::shared_ptr<CollisionGeometry<S>>(model),
                                      Transform3<S>(Translation3<S>(Vector3<S>(x * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale)))));
  }

  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Ellipsoid<S> ellipsoid(single_size / 2, single_size / 2, single_size / 2);
    BVHModel<OBBRSS<S>>* model = new BVHModel<OBBRSS<S>>();
    generateBVHModel(*model, ellipsoid, Transform3<S>::Identity(), 16, 16);
    env.push_back(new CollisionObject<S>(std::shared_ptr<CollisionGeometry<S>>(model),
                                      Transform3<S>(Translation3<S>(Vector3<S>(x * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale)))));
  }

  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Cylinder<S> cylinder(single_size / 2, single_size);
    BVHModel<OBBRSS<S>>* model = new BVHModel<OBBRSS<S>>();
    generateBVHModel(*model, cylinder, Transform3<S>::Identity(), 16, 16);
    env.push_back(new CollisionObject<S>(std::shared_ptr<CollisionGeometry<S>>(model),
                                      Transform3<S>(Translation3<S>(Vector3<S>(x * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale)))));
  }

  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Cone<S> cone(single_size / 2, single_size);
    BVHModel<OBBRSS<S>>* model = new BVHModel<OBBRSS<S>>();
    generateBVHModel(*model, cone, Transform3<S>::Identity(), 16, 16);
    env.push_back(new CollisionObject<S>(std::shared_ptr<CollisionGeometry<S>>(model),
                                      Transform3<S>(Translation3<S>(Vector3<S>(x * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale)))));
  }
}

template <typename S>
void broad_phase_self_distance_test(S env_scale, std::size_t env_size, bool use_mesh)
{
  std::vector<test::TStruct> ts;
  std::vector<test::Timer> timers;

  std::vector<CollisionObject<S>*> env;
  if(use_mesh)
    generateSelfDistanceEnvironmentsMesh(env, env_scale, env_size);
  else
    generateSelfDistanceEnvironments(env, env_scale, env_size);

  std::vector<BroadPhaseCollisionManager<S>*> managers;

  managers.push_back(new NaiveCollisionManager<S>());
  managers.push_back(new SSaPCollisionManager<S>());
  managers.push_back(new SaPCollisionManager<S>());
  managers.push_back(new IntervalTreeCollisionManager<S>());

  Vector3<S> lower_limit, upper_limit;
  SpatialHashingCollisionManager<S>::computeBound(env, lower_limit, upper_limit);
  S cell_size = std::min(std::min((upper_limit[0] - lower_limit[0]) / 5, (upper_limit[1] - lower_limit[1]) / 5), (upper_limit[2] - lower_limit[2]) / 5);
  // managers.push_back(new SpatialHashingCollisionManager<S>(cell_size, lower_limit, upper_limit));
  managers.push_back(new SpatialHashingCollisionManager<S, detail::SparseHashTable<AABB<S>, CollisionObject<S>*, detail::SpatialHash<S>> >(cell_size, lower_limit, upper_limit));
#if USE_GOOGLEHASH
  managers.push_back(new SpatialHashingCollisionManager<S, detail::SparseHashTable<AABB<S>, CollisionObject<S>*, detail::SpatialHash<S>, GoogleSparseHashTable> >(cell_size, lower_limit, upper_limit));
  managers.push_back(new SpatialHashingCollisionManager<S, detail::SparseHashTable<AABB<S>, CollisionObject<S>*, detail::SpatialHash<S>, GoogleDenseHashTable> >(cell_size, lower_limit, upper_limit));
#endif
  managers.push_back(new DynamicAABBTreeCollisionManager<S>());
  managers.push_back(new DynamicAABBTreeCollisionManager_Array<S>());

  {
    DynamicAABBTreeCollisionManager<S>* m = new DynamicAABBTreeCollisionManager<S>();
    m->tree_init_level = 2;
    managers.push_back(m);
  }

  {
    DynamicAABBTreeCollisionManager_Array<S>* m = new DynamicAABBTreeCollisionManager_Array<S>();
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


  std::vector<DefaultDistanceData<S>> self_data(managers.size());

  for(size_t i = 0; i < self_data.size(); ++i)
  {
    timers[i].start();
    managers[i]->distance(&self_data[i], DefaultDistanceFunction);
    timers[i].stop();
    ts[i].push_back(timers[i].getElapsedTime());
    // std::cout << self_data[i].result.min_distance << " ";
  }
  // std::cout << std::endl;

  for(size_t i = 1; i < managers.size(); ++i)
    EXPECT_TRUE(fabs(self_data[0].result.min_distance - self_data[i].result.min_distance) < getDELTA<S>() ||
                fabs(self_data[0].result.min_distance - self_data[i].result.min_distance) / fabs(self_data[0].result.min_distance) < getDELTA<S>());

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

template <typename S>
void broad_phase_distance_test(S env_scale, std::size_t env_size, std::size_t query_size, bool use_mesh)
{
  std::vector<test::TStruct> ts;
  std::vector<test::Timer> timers;

  std::vector<CollisionObject<S>*> env;
  if(use_mesh)
    test::generateEnvironmentsMesh(env, env_scale, env_size);
  else
    test::generateEnvironments(env, env_scale, env_size);

  std::vector<CollisionObject<S>*> query;

  BroadPhaseCollisionManager<S>* manager = new NaiveCollisionManager<S>();
  for(std::size_t i = 0; i < env.size(); ++i)
    manager->registerObject(env[i]);
  manager->setup();

  while(1)
  {
    std::vector<CollisionObject<S>*> candidates;
    if(use_mesh)
      test::generateEnvironmentsMesh(candidates, env_scale, query_size);
    else
      test::generateEnvironments(candidates, env_scale, query_size);

    for(std::size_t i = 0; i < candidates.size(); ++i)
    {
      DefaultCollisionData<S> query_data;
      manager->collide(candidates[i], &query_data, DefaultCollisionFunction);
      if(query_data.result.numContacts() == 0)
        query.push_back(candidates[i]);
      else
        delete candidates[i];
      if(query.size() == query_size) break;
    }

    if(query.size() == query_size) break;
  }

  delete manager;

  std::vector<BroadPhaseCollisionManager<S>*> managers;

  managers.push_back(new NaiveCollisionManager<S>());
  managers.push_back(new SSaPCollisionManager<S>());
  managers.push_back(new SaPCollisionManager<S>());
  managers.push_back(new IntervalTreeCollisionManager<S>());

  Vector3<S> lower_limit, upper_limit;
  SpatialHashingCollisionManager<S>::computeBound(env, lower_limit, upper_limit);
  S cell_size = std::min(std::min((upper_limit[0] - lower_limit[0]) / 20, (upper_limit[1] - lower_limit[1]) / 20), (upper_limit[2] - lower_limit[2])/20);
  // managers.push_back(new SpatialHashingCollisionManager<S>(cell_size, lower_limit, upper_limit));
  managers.push_back(new SpatialHashingCollisionManager<S, detail::SparseHashTable<AABB<S>, CollisionObject<S>*, detail::SpatialHash<S>> >(cell_size, lower_limit, upper_limit));
#if USE_GOOGLEHASH
  managers.push_back(new SpatialHashingCollisionManager<S, detail::SparseHashTable<AABB<S>, CollisionObject<S>*, detail::SpatialHash<S>, GoogleSparseHashTable> >(cell_size, lower_limit, upper_limit));
  managers.push_back(new SpatialHashingCollisionManager<S, detail::SparseHashTable<AABB<S>, CollisionObject<S>*, detail::SpatialHash<S>, GoogleDenseHashTable> >(cell_size, lower_limit, upper_limit));
#endif
  managers.push_back(new DynamicAABBTreeCollisionManager<S>());
  managers.push_back(new DynamicAABBTreeCollisionManager_Array<S>());

  {
    DynamicAABBTreeCollisionManager<S>* m = new DynamicAABBTreeCollisionManager<S>();
    m->tree_init_level = 2;
    managers.push_back(m);
  }

  {
    DynamicAABBTreeCollisionManager_Array<S>* m = new DynamicAABBTreeCollisionManager_Array<S>();
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
    std::vector<DefaultDistanceData<S>> query_data(managers.size());
    for(size_t j = 0; j < managers.size(); ++j)
    {
      timers[j].start();
      managers[j]->distance(query[i], &query_data[j], DefaultDistanceFunction);
      timers[j].stop();
      ts[j].push_back(timers[j].getElapsedTime());
      // std::cout << query_data[j].result.min_distance << " ";
    }
    // std::cout << std::endl;

    for(size_t j = 1; j < managers.size(); ++j)
      EXPECT_TRUE(fabs(query_data[0].result.min_distance - query_data[j].result.min_distance) < getDELTA<S>() ||
                  fabs(query_data[0].result.min_distance - query_data[j].result.min_distance) / fabs(query_data[0].result.min_distance) < getDELTA<S>());
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
    S tmp = 0;
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

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

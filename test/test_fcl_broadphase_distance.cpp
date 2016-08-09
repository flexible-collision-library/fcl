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
#include "fcl/broadphase/broadphase.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"
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

/// @brief Generate environment with 3 * n objects for self distance, so we try to make sure none of them collide with each other.
template <typename Scalar>
void generateSelfDistanceEnvironments(std::vector<CollisionObject<Scalar>*>& env, Scalar env_scale, std::size_t n);

/// @brief Generate environment with 3 * n objects for self distance, but all in meshes.
template <typename Scalar>
void generateSelfDistanceEnvironmentsMesh(std::vector<CollisionObject<Scalar>*>& env, Scalar env_scale, std::size_t n);

/// @brief test for broad phase distance
template <typename Scalar>
void broad_phase_distance_test(Scalar env_scale, std::size_t env_size, std::size_t query_size, bool use_mesh = false);

/// @brief test for broad phase self distance
template <typename Scalar>
void broad_phase_self_distance_test(Scalar env_scale, std::size_t env_size, bool use_mesh = false);

template <typename Scalar>
Scalar getDELTA() { return 0.01; }

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

/// check broad phase distance
GTEST_TEST(FCL_BROADPHASE, test_core_bf_broad_phase_distance)
{
#if FCL_BUILD_TYPE_DEBUG
  broad_phase_distance_test<double>(200, 10, 10);
  broad_phase_distance_test<double>(200, 100, 10);
  broad_phase_distance_test<double>(2000, 10, 10);
  broad_phase_distance_test<double>(2000, 100, 10);
#else
  broad_phase_distance_test<double>(200, 100, 100);
  broad_phase_distance_test<double>(200, 1000, 100);
  broad_phase_distance_test<double>(2000, 100, 100);
  broad_phase_distance_test<double>(2000, 1000, 100);
#endif
}

/// check broad phase self distance
GTEST_TEST(FCL_BROADPHASE, test_core_bf_broad_phase_self_distance)
{
#if FCL_BUILD_TYPE_DEBUG
  broad_phase_self_distance_test<double>(200, 256);
  broad_phase_self_distance_test<double>(200, 500);
  broad_phase_self_distance_test<double>(200, 2500);
#else
  broad_phase_self_distance_test<double>(200, 512);
  broad_phase_self_distance_test<double>(200, 1000);
  broad_phase_self_distance_test<double>(200, 5000);
#endif
}

/// check broad phase distance
GTEST_TEST(FCL_BROADPHASE, test_core_mesh_bf_broad_phase_distance_mesh)
{
#if FCL_BUILD_TYPE_DEBUG
  broad_phase_distance_test<double>(200, 2, 2, true);
  broad_phase_distance_test<double>(200, 4, 2, true);
  broad_phase_distance_test<double>(2000, 2, 2, true);
  broad_phase_distance_test<double>(2000, 4, 2, true);
#else
  broad_phase_distance_test<double>(200, 100, 100, true);
  broad_phase_distance_test<double>(200, 1000, 100, true);
  broad_phase_distance_test<double>(2000, 100, 100, true);
  broad_phase_distance_test<double>(2000, 1000, 100, true);
#endif
}

/// check broad phase self distance
GTEST_TEST(FCL_BROADPHASE, test_core_mesh_bf_broad_phase_self_distance_mesh)
{
#if FCL_BUILD_TYPE_DEBUG
  broad_phase_self_distance_test<double>(200, 128, true);
  broad_phase_self_distance_test<double>(200, 250, true);
  broad_phase_self_distance_test<double>(200, 1250, true);
#else
  broad_phase_self_distance_test<double>(200, 512, true);
  broad_phase_self_distance_test<double>(200, 1000, true);
  broad_phase_self_distance_test<double>(200, 5000, true);
#endif
}

template <typename Scalar>
void generateSelfDistanceEnvironments(std::vector<CollisionObject<Scalar>*>& env, Scalar env_scale, std::size_t n)
{
  unsigned int n_edge = std::floor(std::pow(n, 1/3.0));

  Scalar step_size = env_scale * 2 / n_edge;
  Scalar delta_size = step_size * 0.05;
  Scalar single_size = step_size - 2 * delta_size;
  
  unsigned int i = 0;
  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Box<Scalar>* box = new Box<Scalar>(single_size, single_size, single_size);
    env.push_back(new CollisionObject<Scalar>(std::shared_ptr<CollisionGeometry<Scalar>>(box),
                                      Transform3<Scalar>(Translation3<Scalar>(Vector3<Scalar>(x * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale)))));
  }

  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Sphere<Scalar>* sphere = new Sphere<Scalar>(single_size / 2);
    env.push_back(new CollisionObject<Scalar>(std::shared_ptr<CollisionGeometry<Scalar>>(sphere),
                                      Transform3<Scalar>(Translation3<Scalar>(Vector3<Scalar>(x * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale)))));
  }

  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Ellipsoid<Scalar>* ellipsoid = new Ellipsoid<Scalar>(single_size / 2, single_size / 2, single_size / 2);
    env.push_back(new CollisionObject<Scalar>(std::shared_ptr<CollisionGeometry<Scalar>>(ellipsoid),
                                      Transform3<Scalar>(Translation3<Scalar>(Vector3<Scalar>(x * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale)))));
  }

  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Cylinder<Scalar>* cylinder = new Cylinder<Scalar>(single_size / 2, single_size);
    env.push_back(new CollisionObject<Scalar>(std::shared_ptr<CollisionGeometry<Scalar>>(cylinder),
                                      Transform3<Scalar>(Translation3<Scalar>(Vector3<Scalar>(x * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale)))));
  }

  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Cone<Scalar>* cone = new Cone<Scalar>(single_size / 2, single_size);
    env.push_back(new CollisionObject<Scalar>(std::shared_ptr<CollisionGeometry<Scalar>>(cone),
                                      Transform3<Scalar>(Translation3<Scalar>(Vector3<Scalar>(x * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale)))));
  }
}

template <typename Scalar>
void generateSelfDistanceEnvironmentsMesh(std::vector<CollisionObject<Scalar>*>& env, Scalar env_scale, std::size_t n)
{
  unsigned int n_edge = std::floor(std::pow(n, 1/3.0));

  Scalar step_size = env_scale * 2 / n_edge;
  Scalar delta_size = step_size * 0.05;
  Scalar single_size = step_size - 2 * delta_size;
  
  std::size_t i = 0;
  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Box<Scalar> box(single_size, single_size, single_size);
    BVHModel<OBBRSS<Scalar>>* model = new BVHModel<OBBRSS<Scalar>>();
    generateBVHModel(*model, box, Transform3<Scalar>::Identity());
    env.push_back(new CollisionObject<Scalar>(std::shared_ptr<CollisionGeometry<Scalar>>(model),
                                      Transform3<Scalar>(Translation3<Scalar>(Vector3<Scalar>(x * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale)))));
  }

  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Sphere<Scalar> sphere(single_size / 2);
    BVHModel<OBBRSS<Scalar>>* model = new BVHModel<OBBRSS<Scalar>>();
    generateBVHModel(*model, sphere, Transform3<Scalar>::Identity(), 16, 16);
    env.push_back(new CollisionObject<Scalar>(std::shared_ptr<CollisionGeometry<Scalar>>(model),
                                      Transform3<Scalar>(Translation3<Scalar>(Vector3<Scalar>(x * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale)))));
  }

  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Ellipsoid<Scalar> ellipsoid(single_size / 2, single_size / 2, single_size / 2);
    BVHModel<OBBRSS<Scalar>>* model = new BVHModel<OBBRSS<Scalar>>();
    generateBVHModel(*model, ellipsoid, Transform3<Scalar>::Identity(), 16, 16);
    env.push_back(new CollisionObject<Scalar>(std::shared_ptr<CollisionGeometry<Scalar>>(model),
                                      Transform3<Scalar>(Translation3<Scalar>(Vector3<Scalar>(x * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale)))));
  }

  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Cylinder<Scalar> cylinder(single_size / 2, single_size);
    BVHModel<OBBRSS<Scalar>>* model = new BVHModel<OBBRSS<Scalar>>();
    generateBVHModel(*model, cylinder, Transform3<Scalar>::Identity(), 16, 16);
    env.push_back(new CollisionObject<Scalar>(std::shared_ptr<CollisionGeometry<Scalar>>(model),
                                      Transform3<Scalar>(Translation3<Scalar>(Vector3<Scalar>(x * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale)))));
  }

  for(; i < n_edge * n_edge * n_edge / 4; ++i)
  {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Cone<Scalar> cone(single_size / 2, single_size);
    BVHModel<OBBRSS<Scalar>>* model = new BVHModel<OBBRSS<Scalar>>();
    generateBVHModel(*model, cone, Transform3<Scalar>::Identity(), 16, 16);
    env.push_back(new CollisionObject<Scalar>(std::shared_ptr<CollisionGeometry<Scalar>>(model),
                                      Transform3<Scalar>(Translation3<Scalar>(Vector3<Scalar>(x * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            y * step_size + delta_size + 0.5 * single_size - env_scale,
                                                            z * step_size + delta_size + 0.5 * single_size - env_scale)))));
  }
}

template <typename Scalar>
void broad_phase_self_distance_test(Scalar env_scale, std::size_t env_size, bool use_mesh)
{
  std::vector<TStruct> ts;
  std::vector<Timer> timers;

  std::vector<CollisionObject<Scalar>*> env;
  if(use_mesh)
    generateSelfDistanceEnvironmentsMesh(env, env_scale, env_size);
  else
    generateSelfDistanceEnvironments(env, env_scale, env_size);
  
  std::vector<BroadPhaseCollisionManager<Scalar>*> managers;
  
  managers.push_back(new NaiveCollisionManager<Scalar>());
  managers.push_back(new SSaPCollisionManager<Scalar>());
  managers.push_back(new SaPCollisionManager<Scalar>());
  managers.push_back(new IntervalTreeCollisionManager<Scalar>());
  
  Vector3<Scalar> lower_limit, upper_limit;
  SpatialHashingCollisionManager<Scalar>::computeBound(env, lower_limit, upper_limit);
  Scalar cell_size = std::min(std::min((upper_limit[0] - lower_limit[0]) / 5, (upper_limit[1] - lower_limit[1]) / 5), (upper_limit[2] - lower_limit[2]) / 5);
  // managers.push_back(new SpatialHashingCollisionManager<Scalar>(cell_size, lower_limit, upper_limit));
  managers.push_back(new SpatialHashingCollisionManager<Scalar, SparseHashTable<AABB<Scalar>, CollisionObject<Scalar>*, SpatialHash<Scalar>> >(cell_size, lower_limit, upper_limit));
#if USE_GOOGLEHASH
  managers.push_back(new SpatialHashingCollisionManager<Scalar, SparseHashTable<AABB<Scalar>, CollisionObject<Scalar>*, SpatialHash<Scalar>, GoogleSparseHashTable> >(cell_size, lower_limit, upper_limit));
  managers.push_back(new SpatialHashingCollisionManager<Scalar, SparseHashTable<AABB<Scalar>, CollisionObject<Scalar>*, SpatialHash<Scalar>, GoogleDenseHashTable> >(cell_size, lower_limit, upper_limit));
#endif
  managers.push_back(new DynamicAABBTreeCollisionManager<Scalar>());
  managers.push_back(new DynamicAABBTreeCollisionManager_Array<Scalar>());

  {
    DynamicAABBTreeCollisionManager<Scalar>* m = new DynamicAABBTreeCollisionManager<Scalar>();
    m->tree_init_level = 2;
    managers.push_back(m);
  }

  {
    DynamicAABBTreeCollisionManager_Array<Scalar>* m = new DynamicAABBTreeCollisionManager_Array<Scalar>();
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
 

  std::vector<DistanceData<Scalar>> self_data(managers.size());
                                      
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
    EXPECT_TRUE(fabs(self_data[0].result.min_distance - self_data[i].result.min_distance) < getDELTA<Scalar>() ||
                fabs(self_data[0].result.min_distance - self_data[i].result.min_distance) / fabs(self_data[0].result.min_distance) < getDELTA<Scalar>());

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

template <typename Scalar>
void broad_phase_distance_test(Scalar env_scale, std::size_t env_size, std::size_t query_size, bool use_mesh)
{
  std::vector<TStruct> ts;
  std::vector<Timer> timers;

  std::vector<CollisionObject<Scalar>*> env;
  if(use_mesh)
    generateEnvironmentsMesh(env, env_scale, env_size);
  else
    generateEnvironments(env, env_scale, env_size);

  std::vector<CollisionObject<Scalar>*> query;

  BroadPhaseCollisionManager<Scalar>* manager = new NaiveCollisionManager<Scalar>();
  for(std::size_t i = 0; i < env.size(); ++i)
    manager->registerObject(env[i]);
  manager->setup();

  while(1)
  {
    std::vector<CollisionObject<Scalar>*> candidates;
    if(use_mesh)
      generateEnvironmentsMesh(candidates, env_scale, query_size);
    else
      generateEnvironments(candidates, env_scale, query_size);

    for(std::size_t i = 0; i < candidates.size(); ++i)
    {
      CollisionData<Scalar> query_data;
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

  std::vector<BroadPhaseCollisionManager<Scalar>*> managers;

  managers.push_back(new NaiveCollisionManager<Scalar>());
  managers.push_back(new SSaPCollisionManager<Scalar>());
  managers.push_back(new SaPCollisionManager<Scalar>());
  managers.push_back(new IntervalTreeCollisionManager<Scalar>());
  
  Vector3<Scalar> lower_limit, upper_limit;
  SpatialHashingCollisionManager<Scalar>::computeBound(env, lower_limit, upper_limit);
  Scalar cell_size = std::min(std::min((upper_limit[0] - lower_limit[0]) / 20, (upper_limit[1] - lower_limit[1]) / 20), (upper_limit[2] - lower_limit[2])/20);
  // managers.push_back(new SpatialHashingCollisionManager<Scalar>(cell_size, lower_limit, upper_limit));
  managers.push_back(new SpatialHashingCollisionManager<Scalar, SparseHashTable<AABB<Scalar>, CollisionObject<Scalar>*, SpatialHash<Scalar>> >(cell_size, lower_limit, upper_limit));
#if USE_GOOGLEHASH
  managers.push_back(new SpatialHashingCollisionManager<Scalar, SparseHashTable<AABB<Scalar>, CollisionObject<Scalar>*, SpatialHash<Scalar>, GoogleSparseHashTable> >(cell_size, lower_limit, upper_limit));
  managers.push_back(new SpatialHashingCollisionManager<Scalar, SparseHashTable<AABB<Scalar>, CollisionObject<Scalar>*, SpatialHash<Scalar>, GoogleDenseHashTable> >(cell_size, lower_limit, upper_limit));
#endif
  managers.push_back(new DynamicAABBTreeCollisionManager<Scalar>());
  managers.push_back(new DynamicAABBTreeCollisionManager_Array<Scalar>());

  {
    DynamicAABBTreeCollisionManager<Scalar>* m = new DynamicAABBTreeCollisionManager<Scalar>();
    m->tree_init_level = 2;
    managers.push_back(m);
  }

  {
    DynamicAABBTreeCollisionManager_Array<Scalar>* m = new DynamicAABBTreeCollisionManager_Array<Scalar>();
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
    std::vector<DistanceData<Scalar>> query_data(managers.size());
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
      EXPECT_TRUE(fabs(query_data[0].result.min_distance - query_data[j].result.min_distance) < getDELTA<Scalar>() ||
                  fabs(query_data[0].result.min_distance - query_data[j].result.min_distance) / fabs(query_data[0].result.min_distance) < getDELTA<Scalar>());
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
    Scalar tmp = 0;
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

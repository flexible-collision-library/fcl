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

/// @brief test for broad phase collision and self collision
template <typename S>
void broad_phase_collision_test(S env_scale, std::size_t env_size, std::size_t query_size, std::size_t num_max_contacts = 1, bool exhaustive = false, bool use_mesh = false);

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

/// check broad phase collision for empty collision object set and queries
GTEST_TEST(FCL_BROADPHASE, test_core_bf_broad_phase_collision_empty)
{
#ifdef NDEBUG
  broad_phase_collision_test<double>(2000, 0, 0, 10, false, false);
  broad_phase_collision_test<double>(2000, 0, 1000, 10, false, false);
  broad_phase_collision_test<double>(2000, 100, 0, 10, false, false);

  broad_phase_collision_test<double>(2000, 0, 0, 10, false, true);
  broad_phase_collision_test<double>(2000, 0, 1000, 10, false, true);
  broad_phase_collision_test<double>(2000, 100, 0, 10, false, true);

  broad_phase_collision_test<double>(2000, 0, 0, 10, true, false);
  broad_phase_collision_test<double>(2000, 0, 1000, 10, true, false);
  broad_phase_collision_test<double>(2000, 100, 0, 10, true, false);

  broad_phase_collision_test<double>(2000, 0, 0, 10, true, true);
  broad_phase_collision_test<double>(2000, 0, 1000, 10, true, true);
  broad_phase_collision_test<double>(2000, 100, 0, 10, true, true);
#else
  broad_phase_collision_test<double>(2000, 0, 0, 10, false, false);
  broad_phase_collision_test<double>(2000, 0, 5, 10, false, false);
  broad_phase_collision_test<double>(2000, 2, 0, 10, false, false);

  broad_phase_collision_test<double>(2000, 0, 0, 10, false, true);
  broad_phase_collision_test<double>(2000, 0, 5, 10, false, true);
  broad_phase_collision_test<double>(2000, 2, 0, 10, false, true);

  broad_phase_collision_test<double>(2000, 0, 0, 10, true, false);
  broad_phase_collision_test<double>(2000, 0, 5, 10, true, false);
  broad_phase_collision_test<double>(2000, 2, 0, 10, true, false);

  broad_phase_collision_test<double>(2000, 0, 0, 10, true, true);
  broad_phase_collision_test<double>(2000, 0, 5, 10, true, true);
  broad_phase_collision_test<double>(2000, 2, 0, 10, true, true);
#endif
}

/// check broad phase collision and self collision, only return collision or not
GTEST_TEST(FCL_BROADPHASE, test_core_bf_broad_phase_collision_binary)
{
#ifdef NDEBUG
  broad_phase_collision_test<double>(2000, 100, 1000, 1, false);
  broad_phase_collision_test<double>(2000, 1000, 1000, 1, false);
  broad_phase_collision_test<double>(2000, 100, 1000, 1, true);
  broad_phase_collision_test<double>(2000, 1000, 1000, 1, true);
#else
  broad_phase_collision_test<double>(2000, 10, 100, 1, false);
  broad_phase_collision_test<double>(2000, 100, 100, 1, false);
  broad_phase_collision_test<double>(2000, 10, 100, 1, true);
  broad_phase_collision_test<double>(2000, 100, 100, 1, true);
#endif
}

/// check broad phase collision and self collision, return 10 contacts
GTEST_TEST(FCL_BROADPHASE, test_core_bf_broad_phase_collision)
{
#ifdef NDEBUG
  broad_phase_collision_test<double>(2000, 100, 1000, 10, false);
  broad_phase_collision_test<double>(2000, 1000, 1000, 10, false);
#else
  broad_phase_collision_test<double>(2000, 10, 100, 10, false);
  broad_phase_collision_test<double>(2000, 100, 100, 10, false);
#endif
}

/// check broad phase collision and self collision, return only collision or not, in mesh
GTEST_TEST(FCL_BROADPHASE, test_core_mesh_bf_broad_phase_collision_mesh_binary)
{
#ifdef NDEBUG
  broad_phase_collision_test<double>(2000, 100, 1000, 1, false, true);
  broad_phase_collision_test<double>(2000, 1000, 1000, 1, false, true);
#else
  broad_phase_collision_test<double>(2000, 2, 5, 1, false, true);
  broad_phase_collision_test<double>(2000, 5, 5, 1, false, true);
#endif
}

/// check broad phase collision and self collision, return 10 contacts, in mesh
GTEST_TEST(FCL_BROADPHASE, test_core_mesh_bf_broad_phase_collision_mesh)
{
#ifdef NDEBUG
  broad_phase_collision_test<double>(2000, 100, 1000, 10, false, true);
  broad_phase_collision_test<double>(2000, 1000, 1000, 10, false, true);
#else
  broad_phase_collision_test<double>(2000, 2, 5, 10, false, true);
  broad_phase_collision_test<double>(2000, 5, 5, 10, false, true);
#endif
}

/// check broad phase collision and self collision, exhaustive, in mesh
GTEST_TEST(FCL_BROADPHASE, test_core_mesh_bf_broad_phase_collision_mesh_exhaustive)
{
#ifdef NDEBUG
  broad_phase_collision_test<double>(2000, 100, 1000, 1, true, true);
  broad_phase_collision_test<double>(2000, 1000, 1000, 1, true, true);
#else
  broad_phase_collision_test<double>(2000, 2, 5, 1, true, true);
  broad_phase_collision_test<double>(2000, 5, 5, 1, true, true);
#endif
}

template <typename S>
void broad_phase_collision_test(S env_scale, std::size_t env_size, std::size_t query_size, std::size_t num_max_contacts, bool exhaustive, bool use_mesh)
{
  std::vector<test::TStruct> ts;
  std::vector<test::Timer> timers;

  std::vector<CollisionObject<S>*> env;
  if(use_mesh)
    test::generateEnvironmentsMesh(env, env_scale, env_size);
  else
    test::generateEnvironments(env, env_scale, env_size);

  std::vector<CollisionObject<S>*> query;
  if(use_mesh)
    test::generateEnvironmentsMesh(query, env_scale, query_size);
  else
    test::generateEnvironments(query, env_scale, query_size);

  std::vector<BroadPhaseCollisionManager<S>*> managers;

  managers.push_back(new NaiveCollisionManager<S>());
  managers.push_back(new SSaPCollisionManager<S>());
  managers.push_back(new SaPCollisionManager<S>());
  managers.push_back(new IntervalTreeCollisionManager<S>());

  Vector3<S> lower_limit, upper_limit;
  SpatialHashingCollisionManager<S>::computeBound(env, lower_limit, upper_limit);
  // S ncell_per_axis = std::pow((S)env_size / 10, 1 / 3.0);
  S ncell_per_axis = 20;
  S cell_size = std::min(std::min((upper_limit[0] - lower_limit[0]) / ncell_per_axis, (upper_limit[1] - lower_limit[1]) / ncell_per_axis), (upper_limit[2] - lower_limit[2]) / ncell_per_axis);
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

  std::vector<DefaultCollisionData<S>> self_data(managers.size());
  for(size_t i = 0; i < managers.size(); ++i)
  {
    if(exhaustive) self_data[i].request.num_max_contacts = 100000;
    else self_data[i].request.num_max_contacts = num_max_contacts;
  }

  for(size_t i = 0; i < managers.size(); ++i)
  {
    timers[i].start();
    managers[i]->collide(&self_data[i], DefaultCollisionFunction);
    timers[i].stop();
    ts[i].push_back(timers[i].getElapsedTime());
  }

  for(size_t i = 0; i < managers.size(); ++i)
    std::cout << self_data[i].result.numContacts() << " ";
  std::cout << std::endl;

  if(exhaustive)
  {
    for(size_t i = 1; i < managers.size(); ++i)
      EXPECT_TRUE(self_data[i].result.numContacts() == self_data[0].result.numContacts());
  }
  else
  {
    std::vector<bool> self_res(managers.size());
    for(size_t i = 0; i < self_res.size(); ++i)
      self_res[i] = (self_data[i].result.numContacts() > 0);

    for(size_t i = 1; i < self_res.size(); ++i)
      EXPECT_TRUE(self_res[0] == self_res[i]);

    for(size_t i = 1; i < managers.size(); ++i)
      EXPECT_TRUE(self_data[i].result.numContacts() == self_data[0].result.numContacts());
  }


  for(size_t i = 0; i < query.size(); ++i)
  {
    std::vector<DefaultCollisionData<S>> query_data(managers.size());
    for(size_t j = 0; j < query_data.size(); ++j)
    {
      if(exhaustive) query_data[j].request.num_max_contacts = 100000;
      else query_data[j].request.num_max_contacts = num_max_contacts;
    }

    for(size_t j = 0; j < query_data.size(); ++j)
    {
      timers[j].start();
      managers[j]->collide(query[i], &query_data[j], DefaultCollisionFunction);
      timers[j].stop();
      ts[j].push_back(timers[j].getElapsedTime());
    }


    // for(size_t j = 0; j < managers.size(); ++j)
    //   std::cout << query_data[j].result.numContacts() << " ";
    // std::cout << std::endl;

    if(exhaustive)
    {
      for(size_t j = 1; j < managers.size(); ++j)
        EXPECT_TRUE(query_data[j].result.numContacts() == query_data[0].result.numContacts());
    }
    else
    {
      std::vector<bool> query_res(managers.size());
      for(size_t j = 0; j < query_res.size(); ++j)
        query_res[j] = (query_data[j].result.numContacts() > 0);
      for(size_t j = 1; j < query_res.size(); ++j)
        EXPECT_TRUE(query_res[0] == query_res[j]);

      for(size_t j = 1; j < managers.size(); ++j)
        EXPECT_TRUE(query_data[j].result.numContacts() == query_data[0].result.numContacts());
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
    S tmp = 0;
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

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

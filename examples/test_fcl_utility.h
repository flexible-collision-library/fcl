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

#ifndef TEST_FCL_UTILITY_H
#define TEST_FCL_UTILITY_H

#include <array>
#include <fstream>
#include <iostream>

#include "fcl/common/unused.h"

#include "fcl/math/constants.h"
#include "fcl/math/triangle.h"

#include "fcl/geometry/shape/box.h"
#include "fcl/geometry/shape/sphere.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/geometry/bvh/BVH_model.h"
#include "fcl/geometry/octree/octree.h"

#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/distance.h"
#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/collision_result.h"
#include "fcl/narrowphase/continuous_collision_object.h"
#include "fcl/narrowphase/continuous_collision_request.h"
#include "fcl/narrowphase/continuous_collision_result.h"

#ifdef _WIN32
#define NOMINMAX  // required to avoid compilation errors with Visual Studio 2010
#include <windows.h>
#else
#include <sys/time.h>
#endif

namespace fcl
{
namespace test
{

class Timer
{
public:
  Timer();
  ~Timer();

  void start();                               ///< start timer
  void stop();                                ///< stop the timer
  double getElapsedTime();                    ///< get elapsed time in milli-second
  double getElapsedTimeInSec();               ///< get elapsed time in second (same as getElapsedTime)
  double getElapsedTimeInMilliSec();          ///< get elapsed time in milli-second
  double getElapsedTimeInMicroSec();          ///< get elapsed time in micro-second

private:
  double startTimeInMicroSec;                 ///< starting time in micro-second
  double endTimeInMicroSec;                   ///< ending time in micro-second
  int stopped;                                ///< stop flag
#ifdef _WIN32
  LARGE_INTEGER frequency;                    ///< ticks per second
  LARGE_INTEGER startCount;
  LARGE_INTEGER endCount;
#else
  timeval startCount;
  timeval endCount;
#endif
};

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

/// @brief Load an obj mesh file
template <typename S>
void loadOBJFile(const char* filename, std::vector<Vector3<S>>& points, std::vector<Triangle>& triangles);

template <typename S>
void saveOBJFile(const char* filename, std::vector<Vector3<S>>& points, std::vector<Triangle>& triangles);

template <typename S>
S rand_interval(S rmin, S rmax);

template <typename S>
void eulerToMatrix(S a, S b, S c, Matrix3<S>& R);

/// @brief Generate one random transform whose translation is constrained by extents and rotation without constraints.
/// The translation is (x, y, z), and extents[0] <= x <= extents[3], extents[1] <= y <= extents[4], extents[2] <= z <= extents[5]
template <typename S>
void generateRandomTransform(S extents[6], Transform3<S>& transform);

/// @brief Generate n random transforms whose translations are constrained by extents.
template <typename S>
void generateRandomTransforms(S extents[6], aligned_vector<Transform3<S>>& transforms, std::size_t n);

/// @brief Generate n random transforms whose translations are constrained by extents.
//Also generate another transforms2 which have additional random translation & rotation to the transforms generated.
template <typename S>
void generateRandomTransforms(S extents[6], S delta_trans[3], S delta_rot, aligned_vector<Transform3<S>>& transforms, aligned_vector<Transform3<S>>& transforms2, std::size_t n);

/// @brief Generate n random tranforms and transform2 with addtional random translation/rotation. The transforms and transform2 are used as initial and goal configurations for the first mesh. The second mesh is in I. This is used for continuous collision detection checking.
template <typename S>
void generateRandomTransforms_ccd(S extents[6], aligned_vector<Transform3<S>>& transforms, aligned_vector<Transform3<S>>& transforms2, S delta_trans[3], S delta_rot, std::size_t n,
                                 const std::vector<Vector3<S>>& vertices1, const std::vector<Triangle>& triangles1,
                                 const std::vector<Vector3<S>>& vertices2, const std::vector<Triangle>& triangles2);

/// @brief Generate environment with 3 * n objects: n boxes, n spheres and n cylinders.
template <typename S>
void generateEnvironments(std::vector<CollisionObject<S>*>& env, S env_scale, std::size_t n);

/// @brief Generate environment with 3 * n objects, but all in meshes.
template <typename S>
void generateEnvironmentsMesh(std::vector<CollisionObject<S>*>& env, S env_scale, std::size_t n);

/// @brief Structure for minimum distance between two meshes and the corresponding nearest point pair
template <typename S>
struct DistanceRes
{
  S distance;
  Vector3<S> p1;
  Vector3<S> p2;
};

/// @brief Collision data stores the collision request and the result given by collision algorithm.
template <typename S>
struct CollisionData
{
  CollisionData()
  {
    done = false;
  }

  /// @brief Collision request
  CollisionRequest<S> request;

  /// @brief Collision result
  CollisionResult<S> result;

  /// @brief Whether the collision iteration can stop
  bool done;
};


/// @brief Distance data stores the distance request and the result given by distance algorithm.
template <typename S>
struct DistanceData
{
  DistanceData()
  {
    done = false;
  }

  /// @brief Distance request
  DistanceRequest<S> request;

  /// @brief Distance result
  DistanceResult<S> result;

  /// @brief Whether the distance iteration can stop
  bool done;

};

/// @brief Continuous collision data stores the continuous collision request and result given the continuous collision algorithm.
template <typename S>
struct ContinuousCollisionData
{
  ContinuousCollisionData()
  {
    done = false;
  }

  /// @brief Continuous collision request
  ContinuousCollisionRequest<S> request;

  /// @brief Continuous collision result
  ContinuousCollisionResult<S> result;

  /// @brief Whether the continuous collision iteration can stop
  bool done;
};

/// @brief Default collision callback for two objects o1 and o2 in broad phase. return value means whether the broad phase can stop now.
template <typename S>
bool defaultCollisionFunction(CollisionObject<S>* o1, CollisionObject<S>* o2, void* cdata_);

/// @brief Default distance callback for two objects o1 and o2 in broad phase. return value means whether the broad phase can stop now. also return dist, i.e. the bmin distance till now
template <typename S>
bool defaultDistanceFunction(CollisionObject<S>* o1, CollisionObject<S>* o2, void* cdata_, S& dist);

template <typename S>
bool defaultContinuousCollisionFunction(ContinuousCollisionObject<S>* o1, ContinuousCollisionObject<S>* o2, void* cdata_);

template <typename S>
bool defaultContinuousDistanceFunction(ContinuousCollisionObject<S>* o1, ContinuousCollisionObject<S>* o2, void* cdata_, S& dist);

std::string getNodeTypeName(NODE_TYPE node_type);

std::string getGJKSolverName(GJKSolverType solver_type);

#if FCL_HAVE_OCTOMAP

/// @brief Generate boxes from the octomap
template <typename S>
void generateBoxesFromOctomap(std::vector<CollisionObject<S>*>& env, OcTree<S>& tree);

/// @brief Generate boxes from the octomap
template <typename S>
void generateBoxesFromOctomapMesh(std::vector<CollisionObject<S>*>& env, OcTree<S>& tree);

/// @brief Generate an octree
octomap::OcTree* generateOcTree(double resolution = 0.1);

#endif

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename S>
void loadOBJFile(const char* filename, std::vector<Vector3<S>>& points, std::vector<Triangle>& triangles)
{

  FILE* file = fopen(filename, "rb");
  if(!file)
  {
    std::cerr << "file not exist" << std::endl;
    return;
  }

  bool has_normal = false;
  bool has_texture = false;
  char line_buffer[2000];
  while(fgets(line_buffer, 2000, file))
  {
    char* first_token = strtok(line_buffer, "\r\n\t ");
    if(!first_token || first_token[0] == '#' || first_token[0] == 0)
      continue;

    switch(first_token[0])
    {
    case 'v':
      {
        if(first_token[1] == 'n')
        {
          strtok(nullptr, "\t ");
          strtok(nullptr, "\t ");
          strtok(nullptr, "\t ");
          has_normal = true;
        }
        else if(first_token[1] == 't')
        {
          strtok(nullptr, "\t ");
          strtok(nullptr, "\t ");
          has_texture = true;
        }
        else
        {
          S x = (S)atof(strtok(nullptr, "\t "));
          S y = (S)atof(strtok(nullptr, "\t "));
          S z = (S)atof(strtok(nullptr, "\t "));
          points.emplace_back(x, y, z);
        }
      }
      break;
    case 'f':
      {
        Triangle tri;
        char* data[30];
        int n = 0;
        while((data[n] = strtok(nullptr, "\t \r\n")) != nullptr)
        {
          if(strlen(data[n]))
            n++;
        }

        for(int t = 0; t < (n - 2); ++t)
        {
          if((!has_texture) && (!has_normal))
          {
            tri[0] = atoi(data[0]) - 1;
            tri[1] = atoi(data[1]) - 1;
            tri[2] = atoi(data[2]) - 1;
          }
          else
          {
            const char *v1;
            for(int i = 0; i < 3; i++)
            {
              // vertex ID
              if(i == 0)
                v1 = data[0];
              else
                v1 = data[t + i];

              tri[i] = atoi(v1) - 1;
            }
          }
          triangles.push_back(tri);
        }
      }
    }
  }
}

//==============================================================================
template <typename S>
void saveOBJFile(const char* filename, std::vector<Vector3<S>>& points, std::vector<Triangle>& triangles)
{
  std::ofstream os(filename);
  if(!os)
  {
    std::cerr << "file not exist" << std::endl;
    return;
  }

  for(std::size_t i = 0; i < points.size(); ++i)
  {
    os << "v " << points[i][0] << " " << points[i][1] << " " << points[i][2] << std::endl;
  }

  for(std::size_t i = 0; i < triangles.size(); ++i)
  {
    os << "f " << triangles[i][0] + 1 << " " << triangles[i][1] + 1 << " " << triangles[i][2] + 1 << std::endl;
  }

  os.close();
}

//==============================================================================
template <typename S>
S rand_interval(S rmin, S rmax)
{
  S t = rand() / ((S)RAND_MAX + 1);
  return (t * (rmax - rmin) + rmin);
}

//==============================================================================
template <typename S>
void eulerToMatrix(S a, S b, S c, Matrix3<S>& R)
{
  auto c1 = std::cos(a);
  auto c2 = std::cos(b);
  auto c3 = std::cos(c);
  auto s1 = std::sin(a);
  auto s2 = std::sin(b);
  auto s3 = std::sin(c);

  R << c1 * c2, - c2 * s1, s2,
      c3 * s1 + c1 * s2 * s3, c1 * c3 - s1 * s2 * s3, - c2 * s3,
      s1 * s3 - c1 * c3 * s2, c3 * s1 * s2 + c1 * s3, c2 * c3;
}

//==============================================================================
template <typename S>
void generateRandomTransform(const std::array<S, 6>& extents, Transform3<S>& transform)
{
  auto x = rand_interval(extents[0], extents[3]);
  auto y = rand_interval(extents[1], extents[4]);
  auto z = rand_interval(extents[2], extents[5]);

  const auto pi = constants<S>::pi();
  auto a = rand_interval((S)0, 2 * pi);
  auto b = rand_interval((S)0, 2 * pi);
  auto c = rand_interval((S)0, 2 * pi);

  Matrix3<S> R;
  eulerToMatrix(a, b, c, R);
  Vector3<S> T(x, y, z);
  transform.linear() = R;
  transform.translation() = T;
}

//==============================================================================
template <typename S>
void generateRandomTransforms(S extents[6], aligned_vector<Transform3<S>>& transforms, std::size_t n)
{
  transforms.resize(n);
  for(std::size_t i = 0; i < n; ++i)
  {
    auto x = rand_interval(extents[0], extents[3]);
    auto y = rand_interval(extents[1], extents[4]);
    auto z = rand_interval(extents[2], extents[5]);

    const auto pi = constants<S>::pi();
    auto a = rand_interval((S)0, 2 * pi);
    auto b = rand_interval((S)0, 2 * pi);
    auto c = rand_interval((S)0, 2 * pi);

    {
      Matrix3<S> R;
      eulerToMatrix(a, b, c, R);
      Vector3<S> T(x, y, z);
      transforms[i].setIdentity();
      transforms[i].linear() = R;
      transforms[i].translation() = T;
    }
  }
}

//==============================================================================
template <typename S>
void generateRandomTransforms(S extents[6], S delta_trans[3], S delta_rot, aligned_vector<Transform3<S>>& transforms, aligned_vector<Transform3<S>>& transforms2, std::size_t n)
{
  transforms.resize(n);
  transforms2.resize(n);
  for(std::size_t i = 0; i < n; ++i)
  {
    auto x = rand_interval(extents[0], extents[3]);
    auto y = rand_interval(extents[1], extents[4]);
    auto z = rand_interval(extents[2], extents[5]);

    const auto pi = constants<S>::pi();
    auto a = rand_interval((S)0, 2 * pi);
    auto b = rand_interval((S)0, 2 * pi);
    auto c = rand_interval((S)0, 2 * pi);

    {
      Matrix3<S> R;
      eulerToMatrix(a, b, c, R);
      Vector3<S> T(x, y, z);
      transforms[i].setIdentity();
      transforms[i].linear() = R;
      transforms[i].translation() = T;
    }

    auto deltax = rand_interval(-delta_trans[0], delta_trans[0]);
    auto deltay = rand_interval(-delta_trans[1], delta_trans[1]);
    auto deltaz = rand_interval(-delta_trans[2], delta_trans[2]);

    auto deltaa = rand_interval(-delta_rot, delta_rot);
    auto deltab = rand_interval(-delta_rot, delta_rot);
    auto deltac = rand_interval(-delta_rot, delta_rot);

    {
      Matrix3<S> R;
      eulerToMatrix(a + deltaa, b + deltab, c + deltac, R);
      Vector3<S> T(x + deltax, y + deltay, z + deltaz);
      transforms2[i].setIdentity();
      transforms2[i].linear() = R;
      transforms2[i].translation() = T;
    }
  }
}

//==============================================================================
template <typename S>
void generateRandomTransforms_ccd(S extents[6], aligned_vector<Transform3<S>>& transforms, aligned_vector<Transform3<S>>& transforms2, S delta_trans[3], S delta_rot, std::size_t n,
                                 const std::vector<Vector3<S>>& vertices1, const std::vector<Triangle>& triangles1,
                                 const std::vector<Vector3<S>>& vertices2, const std::vector<Triangle>& triangles2)
{
  FCL_UNUSED(vertices1);
  FCL_UNUSED(triangles1);
  FCL_UNUSED(vertices2);
  FCL_UNUSED(triangles2);

  transforms.resize(n);
  transforms2.resize(n);

  for(std::size_t i = 0; i < n;)
  {
    auto x = rand_interval(extents[0], extents[3]);
    auto y = rand_interval(extents[1], extents[4]);
    auto z = rand_interval(extents[2], extents[5]);

    const auto pi = constants<S>::pi();
    auto a = rand_interval(0, 2 * pi);
    auto b = rand_interval(0, 2 * pi);
    auto c = rand_interval(0, 2 * pi);


    Matrix3<S> R;
    eulerToMatrix(a, b, c, R);
    Vector3<S> T(x, y, z);
    Transform3<S> tf(Transform3<S>::Identity());
    tf.linear() = R;
    tf.translation() = T;

    std::vector<std::pair<int, int> > results;
    {
      transforms[i] = tf;

      auto deltax = rand_interval(-delta_trans[0], delta_trans[0]);
      auto deltay = rand_interval(-delta_trans[1], delta_trans[1]);
      auto deltaz = rand_interval(-delta_trans[2], delta_trans[2]);

      auto deltaa = rand_interval(-delta_rot, delta_rot);
      auto deltab = rand_interval(-delta_rot, delta_rot);
      auto deltac = rand_interval(-delta_rot, delta_rot);

      Matrix3<S> R2;
      eulerToMatrix(a + deltaa, b + deltab, c + deltac, R2);
      Vector3<S> T2(x + deltax, y + deltay, z + deltaz);
      transforms2[i].linear() = R2;
      transforms2[i].translation() = T2;
      ++i;
    }
  }
}

//==============================================================================
template <typename S>
void generateEnvironments(std::vector<CollisionObject<S>*>& env, S env_scale, std::size_t n)
{
  S extents[] = {-env_scale, env_scale, -env_scale, env_scale, -env_scale, env_scale};
  aligned_vector<Transform3<S>> transforms;

  generateRandomTransforms(extents, transforms, n);
  for(std::size_t i = 0; i < n; ++i)
  {
    Box<S>* box = new Box<S>(5, 10, 20);
    env.push_back(new CollisionObject<S>(std::shared_ptr<CollisionGeometry<S>>(box), transforms[i]));
  }

  generateRandomTransforms(extents, transforms, n);
  for(std::size_t i = 0; i < n; ++i)
  {
    Sphere<S>* sphere = new Sphere<S>(30);
    env.push_back(new CollisionObject<S>(std::shared_ptr<CollisionGeometry<S>>(sphere), transforms[i]));
  }

  generateRandomTransforms(extents, transforms, n);
  for(std::size_t i = 0; i < n; ++i)
  {
    Cylinder<S>* cylinder = new Cylinder<S>(10, 40);
    env.push_back(new CollisionObject<S>(std::shared_ptr<CollisionGeometry<S>>(cylinder), transforms[i]));
  }
}

//==============================================================================
template <typename S>
void generateEnvironmentsMesh(std::vector<CollisionObject<S>*>& env, S env_scale, std::size_t n)
{
  S extents[] = {-env_scale, env_scale, -env_scale, env_scale, -env_scale, env_scale};
  aligned_vector<Transform3<S>> transforms;

  generateRandomTransforms(extents, transforms, n);
  Box<S> box(5, 10, 20);
  for(std::size_t i = 0; i < n; ++i)
  {
    BVHModel<OBBRSS<S>>* model = new BVHModel<OBBRSS<S>>();
    generateBVHModel(*model, box, Transform3<S>::Identity());
    env.push_back(new CollisionObject<S>(std::shared_ptr<CollisionGeometry<S>>(model), transforms[i]));
  }

  generateRandomTransforms(extents, transforms, n);
  Sphere<S> sphere(30);
  for(std::size_t i = 0; i < n; ++i)
  {
    BVHModel<OBBRSS<S>>* model = new BVHModel<OBBRSS<S>>();
    generateBVHModel(*model, sphere, Transform3<S>::Identity(), 16, 16);
    env.push_back(new CollisionObject<S>(std::shared_ptr<CollisionGeometry<S>>(model), transforms[i]));
  }

  generateRandomTransforms(extents, transforms, n);
  Cylinder<S> cylinder(10, 40);
  for(std::size_t i = 0; i < n; ++i)
  {
    BVHModel<OBBRSS<S>>* model = new BVHModel<OBBRSS<S>>();
    generateBVHModel(*model, cylinder, Transform3<S>::Identity(), 16, 16);
    env.push_back(new CollisionObject<S>(std::shared_ptr<CollisionGeometry<S>>(model), transforms[i]));
  }
}

//==============================================================================
template <typename S>
bool defaultCollisionFunction(CollisionObject<S>* o1, CollisionObject<S>* o2, void* cdata_)
{
  auto* cdata = static_cast<CollisionData<S>*>(cdata_);
  const auto& request = cdata->request;
  auto& result = cdata->result;

  if(cdata->done) return true;

  collide(o1, o2, request, result);

  if(!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
    cdata->done = true;

  return cdata->done;
}

//==============================================================================
template <typename S>
bool defaultDistanceFunction(CollisionObject<S>* o1, CollisionObject<S>* o2, void* cdata_, S& dist)
{
  auto* cdata = static_cast<DistanceData<S>*>(cdata_);
  const DistanceRequest<S>& request = cdata->request;
  DistanceResult<S>& result = cdata->result;

  if(cdata->done) { dist = result.min_distance; return true; }

  distance(o1, o2, request, result);

  dist = result.min_distance;

  if(dist <= 0) return true; // in collision or in touch

  return cdata->done;
}

//==============================================================================
template <typename S>
bool defaultContinuousCollisionFunction(ContinuousCollisionObject<S>* o1, ContinuousCollisionObject<S>* o2, void* cdata_)
{
  auto* cdata = static_cast<ContinuousCollisionData<S>*>(cdata_);
  const ContinuousCollisionRequest<S>& request = cdata->request;
  ContinuousCollisionResult<S>& result = cdata->result;

  if(cdata->done) return true;

  collide(o1, o2, request, result);

  return cdata->done;
}

//==============================================================================
template <typename S>
bool defaultContinuousDistanceFunction(ContinuousCollisionObject<S>* o1, ContinuousCollisionObject<S>* o2, void* cdata_, S& dist)
{
  FCL_UNUSED(o1);
  FCL_UNUSED(o2);
  FCL_UNUSED(cdata_);
  FCL_UNUSED(dist);

  return true;
}

#if FCL_HAVE_OCTOMAP

//==============================================================================
template <typename S>
void generateBoxesFromOctomap(std::vector<CollisionObject<S>*>& boxes, OcTree<S>& tree)
{
  std::vector<std::array<S, 6> > boxes_ = tree.toBoxes();

  for(std::size_t i = 0; i < boxes_.size(); ++i)
  {
    S x = boxes_[i][0];
    S y = boxes_[i][1];
    S z = boxes_[i][2];
    S size = boxes_[i][3];
    S cost = boxes_[i][4];
    S threshold = boxes_[i][5];

    Box<S>* box = new Box<S>(size, size, size);
    box->cost_density = cost;
    box->threshold_occupied = threshold;
    CollisionObject<S>* obj = new CollisionObject<S>(std::shared_ptr<CollisionGeometry<S>>(box), Transform3<S>(Translation3<S>(Vector3<S>(x, y, z))));
    boxes.push_back(obj);
  }

  std::cout << "boxes size: " << boxes.size() << std::endl;

}

//==============================================================================
template <typename S>
void generateBoxesFromOctomapMesh(std::vector<CollisionObject<S>*>& boxes, OcTree<S>& tree)
{
  std::vector<std::array<S, 6> > boxes_ = tree.toBoxes();

  for(std::size_t i = 0; i < boxes_.size(); ++i)
  {
    S x = boxes_[i][0];
    S y = boxes_[i][1];
    S z = boxes_[i][2];
    S size = boxes_[i][3];
    S cost = boxes_[i][4];
    S threshold = boxes_[i][5];

    Box<S> box(size, size, size);
    BVHModel<OBBRSS<S>>* model = new BVHModel<OBBRSS<S>>();
    generateBVHModel(*model, box, Transform3<S>::Identity());
    model->cost_density = cost;
    model->threshold_occupied = threshold;
    CollisionObject<S>* obj = new CollisionObject<S>(std::shared_ptr<CollisionGeometry<S>>(model), Transform3<S>(Translation3<S>(Vector3<S>(x, y, z))));
    boxes.push_back(obj);
  }

  std::cout << "boxes size: " << boxes.size() << std::endl;
}

//==============================================================================
inline octomap::OcTree* generateOcTree(double resolution)
{
  octomap::OcTree* tree = new octomap::OcTree(resolution);

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

#endif // FCL_HAVE_OCTOMAP

} // namespace test
} // namespace fcl

#endif

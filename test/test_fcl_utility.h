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

#ifndef TEST_FCL_UTILITY_H
#define TEST_FCL_UTILITY_H

#include <array>
#include <fstream>
#include <iostream>
#include "fcl/math/constants.h"
#include "fcl/math/triangle.h"
#include "fcl/collision.h"
#include "fcl/distance.h"
#include "fcl/collision_data.h"
#include "fcl/collision_object.h"
#include "fcl/continuous_collision_object.h"

#ifdef _WIN32
#define NOMINMAX  // required to avoid compilation errors with Visual Studio 2010
#include <windows.h>
#else
#include <sys/time.h>
#endif


namespace fcl
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

/// @brief Load an obj mesh file
template <typename Scalar>
void loadOBJFile(const char* filename, std::vector<Vector3<Scalar>>& points, std::vector<Triangle>& triangles);

template <typename Scalar>
void saveOBJFile(const char* filename, std::vector<Vector3<Scalar>>& points, std::vector<Triangle>& triangles);

template <typename Scalar>
Scalar rand_interval(Scalar rmin, Scalar rmax);

template <typename Scalar>
void eulerToMatrix(Scalar a, Scalar b, Scalar c, Matrix3<Scalar>& R);

/// @brief Generate one random transform whose translation is constrained by extents and rotation without constraints. 
/// The translation is (x, y, z), and extents[0] <= x <= extents[3], extents[1] <= y <= extents[4], extents[2] <= z <= extents[5]
template <typename Scalar>
void generateRandomTransform(Scalar extents[6], Transform3<Scalar>& transform);

/// @brief Generate n random transforms whose translations are constrained by extents.
template <typename Scalar>
void generateRandomTransforms(Scalar extents[6], Eigen::aligned_vector<Transform3<Scalar>>& transforms, std::size_t n);

/// @brief Generate n random transforms whose translations are constrained by extents. Also generate another transforms2 which have additional random translation & rotation to the transforms generated.
template <typename Scalar>
void generateRandomTransforms(Scalar extents[6], Scalar delta_trans[3], Scalar delta_rot, Eigen::aligned_vector<Transform3<Scalar>>& transforms, Eigen::aligned_vector<Transform3<Scalar>>& transforms2, std::size_t n);

/// @brief Generate n random tranforms and transform2 with addtional random translation/rotation. The transforms and transform2 are used as initial and goal configurations for the first mesh. The second mesh is in I. This is used for continuous collision detection checking.
template <typename Scalar>
void generateRandomTransforms_ccd(Scalar extents[6], Eigen::aligned_vector<Transform3<Scalar>>& transforms, Eigen::aligned_vector<Transform3<Scalar>>& transforms2, Scalar delta_trans[3], Scalar delta_rot, std::size_t n,
                                 const std::vector<Vector3<Scalar>>& vertices1, const std::vector<Triangle>& triangles1,
                                 const std::vector<Vector3<Scalar>>& vertices2, const std::vector<Triangle>& triangles2);

/// @brief Structure for minimum distance between two meshes and the corresponding nearest point pair
template <typename Scalar>
struct DistanceRes
{
  Scalar distance;
  Vector3<Scalar> p1;
  Vector3<Scalar> p2;
};

/// @brief Collision data stores the collision request and the result given by collision algorithm. 
template <typename Scalar>
struct CollisionData
{
  CollisionData()
  {
    done = false;
  }

  /// @brief Collision request
  CollisionRequest<Scalar> request;

  /// @brief Collision result
  CollisionResult<Scalar> result;

  /// @brief Whether the collision iteration can stop
  bool done;
};


/// @brief Distance data stores the distance request and the result given by distance algorithm. 
template <typename Scalar>
struct DistanceData
{
  DistanceData()
  {
    done = false;
  }

  /// @brief Distance request
  DistanceRequest<Scalar> request;

  /// @brief Distance result
  DistanceResult<Scalar> result;

  /// @brief Whether the distance iteration can stop
  bool done;

};

/// @brief Continuous collision data stores the continuous collision request and result given the continuous collision algorithm.
template <typename Scalar>
struct ContinuousCollisionData
{
  ContinuousCollisionData()
  {
    done = false;
  }

  /// @brief Continuous collision request
  ContinuousCollisionRequest<Scalar> request;

  /// @brief Continuous collision result
  ContinuousCollisionResult<Scalar> result;

  /// @brief Whether the continuous collision iteration can stop
  bool done;
};

/// @brief Default collision callback for two objects o1 and o2 in broad phase. return value means whether the broad phase can stop now.
template <typename Scalar>
bool defaultCollisionFunction(CollisionObject<Scalar>* o1, CollisionObject<Scalar>* o2, void* cdata_);

/// @brief Default distance callback for two objects o1 and o2 in broad phase. return value means whether the broad phase can stop now. also return dist, i.e. the bmin distance till now
template <typename Scalar>
bool defaultDistanceFunction(CollisionObject<Scalar>* o1, CollisionObject<Scalar>* o2, void* cdata_, Scalar& dist);

template <typename Scalar>
bool defaultContinuousCollisionFunction(ContinuousCollisionObject<Scalar>* o1, ContinuousCollisionObject<Scalar>* o2, void* cdata_);

template <typename Scalar>
bool defaultContinuousDistanceFunction(ContinuousCollisionObject<Scalar>* o1, ContinuousCollisionObject<Scalar>* o2, void* cdata_, Scalar& dist);

std::string getNodeTypeName(NODE_TYPE node_type);

std::string getGJKSolverName(GJKSolverType solver_type);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
void loadOBJFile(const char* filename, std::vector<Vector3<Scalar>>& points, std::vector<Triangle>& triangles)
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
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          has_normal = true;
        }
        else if(first_token[1] == 't')
        {
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          has_texture = true;
        }
        else
        {
          Scalar x = (Scalar)atof(strtok(NULL, "\t "));
          Scalar y = (Scalar)atof(strtok(NULL, "\t "));
          Scalar z = (Scalar)atof(strtok(NULL, "\t "));
          Vector3<Scalar> p(x, y, z);
          points.push_back(p);
        }
      }
      break;
    case 'f':
      {
        Triangle tri;
        char* data[30];
        int n = 0;
        while((data[n] = strtok(NULL, "\t \r\n")) != NULL)
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
template <typename Scalar>
void saveOBJFile(const char* filename, std::vector<Vector3<Scalar>>& points, std::vector<Triangle>& triangles)
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
template <typename Scalar>
Scalar rand_interval(Scalar rmin, Scalar rmax)
{
  Scalar t = rand() / ((Scalar)RAND_MAX + 1);
  return (t * (rmax - rmin) + rmin);
}

//==============================================================================
template <typename Scalar>
void eulerToMatrix(Scalar a, Scalar b, Scalar c, Matrix3<Scalar>& R)
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
template <typename Scalar>
void generateRandomTransform(const std::array<Scalar, 6>& extents, Transform3<Scalar>& transform)
{
  auto x = rand_interval(extents[0], extents[3]);
  auto y = rand_interval(extents[1], extents[4]);
  auto z = rand_interval(extents[2], extents[5]);

  const auto pi = constants<Scalar>::pi();
  auto a = rand_interval((Scalar)0, 2 * pi);
  auto b = rand_interval((Scalar)0, 2 * pi);
  auto c = rand_interval((Scalar)0, 2 * pi);

  Matrix3<Scalar> R;
  eulerToMatrix(a, b, c, R);
  Vector3<Scalar> T(x, y, z);
  transform.linear() = R;
  transform.translation() = T;
}

//==============================================================================
template <typename Scalar>
void generateRandomTransforms(Scalar extents[6], Eigen::aligned_vector<Transform3<Scalar>>& transforms, std::size_t n)
{
  transforms.resize(n);
  for(std::size_t i = 0; i < n; ++i)
  {
    auto x = rand_interval(extents[0], extents[3]);
    auto y = rand_interval(extents[1], extents[4]);
    auto z = rand_interval(extents[2], extents[5]);

    const auto pi = constants<Scalar>::pi();
    auto a = rand_interval((Scalar)0, 2 * pi);
    auto b = rand_interval((Scalar)0, 2 * pi);
    auto c = rand_interval((Scalar)0, 2 * pi);

    {
      Matrix3<Scalar> R;
      eulerToMatrix(a, b, c, R);
      Vector3<Scalar> T(x, y, z);
      transforms[i].setIdentity();
      transforms[i].linear() = R;
      transforms[i].translation() = T;
    }
  }
}

//==============================================================================
template <typename Scalar>
void generateRandomTransforms(Scalar extents[6], Scalar delta_trans[3], Scalar delta_rot, Eigen::aligned_vector<Transform3<Scalar>>& transforms, Eigen::aligned_vector<Transform3<Scalar>>& transforms2, std::size_t n)
{
  transforms.resize(n);
  transforms2.resize(n);
  for(std::size_t i = 0; i < n; ++i)
  {
    auto x = rand_interval(extents[0], extents[3]);
    auto y = rand_interval(extents[1], extents[4]);
    auto z = rand_interval(extents[2], extents[5]);

    const auto pi = constants<Scalar>::pi();
    auto a = rand_interval((Scalar)0, 2 * pi);
    auto b = rand_interval((Scalar)0, 2 * pi);
    auto c = rand_interval((Scalar)0, 2 * pi);

    {
      Matrix3<Scalar> R;
      eulerToMatrix(a, b, c, R);
      Vector3<Scalar> T(x, y, z);
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
      Matrix3<Scalar> R;
      eulerToMatrix(a + deltaa, b + deltab, c + deltac, R);
      Vector3<Scalar> T(x + deltax, y + deltay, z + deltaz);
      transforms2[i].setIdentity();
      transforms2[i].linear() = R;
      transforms2[i].translation() = T;
    }
  }
}

//==============================================================================
template <typename Scalar>
void generateRandomTransforms_ccd(Scalar extents[6], Eigen::aligned_vector<Transform3<Scalar>>& transforms, Eigen::aligned_vector<Transform3<Scalar>>& transforms2, Scalar delta_trans[3], Scalar delta_rot, std::size_t n,
                                 const std::vector<Vector3<Scalar>>& vertices1, const std::vector<Triangle>& triangles1,
                                 const std::vector<Vector3<Scalar>>& vertices2, const std::vector<Triangle>& triangles2)
{
  transforms.resize(n);
  transforms2.resize(n);

  for(std::size_t i = 0; i < n;)
  {
    auto x = rand_interval(extents[0], extents[3]);
    auto y = rand_interval(extents[1], extents[4]);
    auto z = rand_interval(extents[2], extents[5]);

    const auto pi = constants<Scalar>::pi();
    auto a = rand_interval(0, 2 * pi);
    auto b = rand_interval(0, 2 * pi);
    auto c = rand_interval(0, 2 * pi);


    Matrix3<Scalar> R;
    eulerToMatrix(a, b, c, R);
    Vector3<Scalar> T(x, y, z);
    Transform3<Scalar> tf(Transform3<Scalar>::Identity());
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

      Matrix3<Scalar> R2;
      eulerToMatrix(a + deltaa, b + deltab, c + deltac, R2);
      Vector3<Scalar> T2(x + deltax, y + deltay, z + deltaz);
      transforms2[i].linear() = R2;
      transforms2[i].translation() = T2;
      ++i;
    }
  }
}

//==============================================================================
template <typename Scalar>
bool defaultCollisionFunction(CollisionObject<Scalar>* o1, CollisionObject<Scalar>* o2, void* cdata_)
{
  auto* cdata = static_cast<CollisionData<Scalar>*>(cdata_);
  const auto& request = cdata->request;
  auto& result = cdata->result;

  if(cdata->done) return true;

  collide(o1, o2, request, result);

  if(!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
    cdata->done = true;

  return cdata->done;
}

//==============================================================================
template <typename Scalar>
bool defaultDistanceFunction(CollisionObject<Scalar>* o1, CollisionObject<Scalar>* o2, void* cdata_, Scalar& dist)
{
  auto* cdata = static_cast<DistanceData<Scalar>*>(cdata_);
  const DistanceRequest<Scalar>& request = cdata->request;
  DistanceResult<Scalar>& result = cdata->result;

  if(cdata->done) { dist = result.min_distance; return true; }

  distance(o1, o2, request, result);

  dist = result.min_distance;

  if(dist <= 0) return true; // in collision or in touch

  return cdata->done;
}

//==============================================================================
template <typename Scalar>
bool defaultContinuousCollisionFunction(ContinuousCollisionObject<Scalar>* o1, ContinuousCollisionObject<Scalar>* o2, void* cdata_)
{
  auto* cdata = static_cast<ContinuousCollisionData<Scalar>*>(cdata_);
  const ContinuousCollisionRequest<Scalar>& request = cdata->request;
  ContinuousCollisionResult<Scalar>& result = cdata->result;

  if(cdata->done) return true;

  collide(o1, o2, request, result);

  return cdata->done;
}

//==============================================================================
template <typename Scalar>
bool defaultContinuousDistanceFunction(ContinuousCollisionObject<Scalar>* o1, ContinuousCollisionObject<Scalar>* o2, void* cdata_, Scalar& dist)
{
  return true;
}

} // namespace fcl

#endif

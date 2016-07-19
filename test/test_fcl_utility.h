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

#include "fcl/math/transform.h"
#include "fcl/collision_data.h"
#include "fcl/collision_object.h"

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
void loadOBJFile(const char* filename, std::vector<Vec3f>& points, std::vector<Triangle>& triangles);

void saveOBJFile(const char* filename, std::vector<Vec3f>& points, std::vector<Triangle>& triangles);

/// @brief Generate one random transform whose translation is constrained by extents and rotation without constraints. 
/// The translation is (x, y, z), and extents[0] <= x <= extents[3], extents[1] <= y <= extents[4], extents[2] <= z <= extents[5]
void generateRandomTransform(FCL_REAL extents[6], Transform3f& transform);

/// @brief Generate n random transforms whose translations are constrained by extents.
void generateRandomTransforms(FCL_REAL extents[6], std::vector<Transform3f>& transforms, std::size_t n);

/// @brief Generate n random transforms whose translations are constrained by extents. Also generate another transforms2 which have additional random translation & rotation to the transforms generated.
void generateRandomTransforms(FCL_REAL extents[6], FCL_REAL delta_trans[3], FCL_REAL delta_rot, std::vector<Transform3f>& transforms, std::vector<Transform3f>& transforms2, std::size_t n);

/// @brief Generate n random tranforms and transform2 with addtional random translation/rotation. The transforms and transform2 are used as initial and goal configurations for the first mesh. The second mesh is in I. This is used for continuous collision detection checking.
void generateRandomTransforms_ccd(FCL_REAL extents[6], std::vector<Transform3f>& transforms, std::vector<Transform3f>& transforms2, FCL_REAL delta_trans[3], FCL_REAL delta_rot, std::size_t n,
                                 const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                                 const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2);


/// @ brief Structure for minimum distance between two meshes and the corresponding nearest point pair
struct DistanceRes
{
  double distance;
  Vec3f p1;
  Vec3f p2;
};

/// @brief Collision data stores the collision request and the result given by collision algorithm. 
struct CollisionData
{
  CollisionData()
  {
    done = false;
  }

  /// @brief Collision request
  CollisionRequest request;

  /// @brief Collision result
  CollisionResult result;

  /// @brief Whether the collision iteration can stop
  bool done;
};


/// @brief Distance data stores the distance request and the result given by distance algorithm. 
struct DistanceData
{
  DistanceData()
  {
    done = false;
  }

  /// @brief Distance request
  DistanceRequest request;

  /// @brief Distance result
  DistanceResult result;

  /// @brief Whether the distance iteration can stop
  bool done;

};

/// @brief Continuous collision data stores the continuous collision request and result given the continuous collision algorithm.
struct ContinuousCollisionData
{
  ContinuousCollisionData()
  {
    done = false;
  }

  /// @brief Continuous collision request
  ContinuousCollisionRequest request;

  /// @brief Continuous collision result
  ContinuousCollisionResult result;

  /// @brief Whether the continuous collision iteration can stop
  bool done;
};



/// @brief Default collision callback for two objects o1 and o2 in broad phase. return value means whether the broad phase can stop now.
bool defaultCollisionFunction(CollisionObject* o1, CollisionObject* o2, void* cdata);

/// @brief Default distance callback for two objects o1 and o2 in broad phase. return value means whether the broad phase can stop now. also return dist, i.e. the bmin distance till now
bool defaultDistanceFunction(CollisionObject* o1, CollisionObject* o2, void* cdata, FCL_REAL& dist);





bool defaultContinuousCollisionFunction(ContinuousCollisionObject* o1, ContinuousCollisionObject* o2, void* cdata_);

bool defaultContinuousDistanceFunction(ContinuousCollisionObject* o1, ContinuousCollisionObject* o2, void* cdata_, FCL_REAL& dist);


std::string getNodeTypeName(NODE_TYPE node_type);

std::string getGJKSolverName(GJKSolverType solver_type);

}

#endif

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

#include "test_fcl_utility.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/continuous_collision.h"
#include "fcl/narrowphase/distance.h"
#include <cstdio>
#include <cstddef>

namespace fcl
{

namespace test
{

//==============================================================================
Timer::Timer()
{
#ifdef _WIN32
  QueryPerformanceFrequency(&frequency);
  startCount.QuadPart = 0;
  endCount.QuadPart = 0;
#else
  startCount.tv_sec = startCount.tv_usec = 0;
  endCount.tv_sec = endCount.tv_usec = 0;
#endif

  stopped = 0;
  startTimeInMicroSec = 0;
  endTimeInMicroSec = 0;
}

//==============================================================================
Timer::~Timer()
{
  // Do nothing
}

//==============================================================================
void Timer::start()
{
  stopped = 0; // reset stop flag
#ifdef _WIN32
  QueryPerformanceCounter(&startCount);
#else
  gettimeofday(&startCount, nullptr);
#endif
}

//==============================================================================
void Timer::stop()
{
  stopped = 1; // set timer stopped flag

#ifdef _WIN32
  QueryPerformanceCounter(&endCount);
#else
  gettimeofday(&endCount, nullptr);
#endif
}

double Timer::getElapsedTimeInMicroSec()
{
#ifdef _WIN32
  if(!stopped)
    QueryPerformanceCounter(&endCount);

  startTimeInMicroSec = startCount.QuadPart * (1000000.0 / frequency.QuadPart);
  endTimeInMicroSec = endCount.QuadPart * (1000000.0 / frequency.QuadPart);
#else
  if(!stopped)
    gettimeofday(&endCount, nullptr);

  startTimeInMicroSec = (startCount.tv_sec * 1000000.0) + startCount.tv_usec;
  endTimeInMicroSec = (endCount.tv_sec * 1000000.0) + endCount.tv_usec;
#endif

  return endTimeInMicroSec - startTimeInMicroSec;
}

//==============================================================================
double Timer::getElapsedTimeInMilliSec()
{
  return this->getElapsedTimeInMicroSec() * 0.001;
}

//==============================================================================
double Timer::getElapsedTimeInSec()
{
  return this->getElapsedTimeInMicroSec() * 0.000001;
}

//==============================================================================
double Timer::getElapsedTime()
{
  return this->getElapsedTimeInMilliSec();
}

//==============================================================================
std::string getNodeTypeName(NODE_TYPE node_type)
{
  if (node_type == BV_UNKNOWN)
    return std::string("BV_UNKNOWN");
  else if (node_type == BV_AABB)
    return std::string("BV_AABB");
  else if (node_type == BV_OBB)
    return std::string("BV_OBB");
  else if (node_type == BV_RSS)
    return std::string("BV_RSS");
  else if (node_type == BV_kIOS)
    return std::string("BV_kIOS");
  else if (node_type == BV_OBBRSS)
    return std::string("BV_OBBRSS");
  else if (node_type == BV_KDOP16)
    return std::string("BV_KDOP16");
  else if (node_type == BV_KDOP18)
    return std::string("BV_KDOP18");
  else if (node_type == BV_KDOP24)
    return std::string("BV_KDOP24");
  else if (node_type == GEOM_BOX)
    return std::string("GEOM_BOX");
  else if (node_type == GEOM_SPHERE)
    return std::string("GEOM_SPHERE");
  else if (node_type == GEOM_ELLIPSOID)
    return std::string("GEOM_ELLIPSOID");
  else if (node_type == GEOM_CAPSULE)
    return std::string("GEOM_CAPSULE");
  else if (node_type == GEOM_CONE)
    return std::string("GEOM_CONE");
  else if (node_type == GEOM_CYLINDER)
    return std::string("GEOM_CYLINDER");
  else if (node_type == GEOM_CONVEX)
    return std::string("GEOM_CONVEX");
  else if (node_type == GEOM_PLANE)
    return std::string("GEOM_PLANE");
  else if (node_type == GEOM_HALFSPACE)
    return std::string("GEOM_HALFSPACE");
  else if (node_type == GEOM_TRIANGLE)
    return std::string("GEOM_TRIANGLE");
  else if (node_type == GEOM_OCTREE)
    return std::string("GEOM_OCTREE");
  else
    return std::string("invalid");
}

//==============================================================================
std::string getGJKSolverName(GJKSolverType solver_type)
{
  if (solver_type == GST_LIBCCD)
    return std::string("libccd");
  else if (solver_type == GST_INDEP)
    return std::string("built-in");
  else
    return std::string("invalid");
}

} // namespace test
} // namespace fcl

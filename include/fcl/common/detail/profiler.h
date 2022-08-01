/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008-2014, Willow Garage, Inc.
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

/** @author Ioan Sucan */

#ifndef FCL_COMMON_DETAIL_PROFILER_H
#define FCL_COMMON_DETAIL_PROFILER_H

#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <cmath>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include "fcl/common/time.h"
#include "fcl/export.h"

namespace fcl {
namespace detail {

/// @brief This is a simple thread-safe tool for counting time
/// spent in various chunks of code. This is different from
/// external profiling tools in that it allows the user to count
/// time spent in various bits of code (sub-function granularity)
/// or count how many times certain pieces of code are executed.
class FCL_EXPORT Profiler
{
public:
  // non-copyable
  Profiler(const Profiler&) = delete;
  Profiler& operator=(const Profiler&) = delete;

  /// @brief This instance will call Profiler::begin() when constructed and
  /// Profiler::end() when it goes out of scope.
  class FCL_EXPORT ScopedBlock;

  /// @brief This instance will call Profiler::start() when constructed and
  /// Profiler::stop() when it goes out of scope.
  /// If the profiler was already started, this block's constructor and
  /// destructor take no action
  class FCL_EXPORT ScopedStart;

  /// @brief Return an instance of the class
  static Profiler& Instance(void);

  /// @brief Constructor. It is allowed to separately instantiate this
  /// class (not only as a singleton)
  Profiler(bool printOnDestroy = false, bool autoStart = false);

  /// @brief Destructor
  ~Profiler(void);

  /// @brief Start counting time
  static void Start(void);

  /// @brief Stop counting time
  static void Stop(void);

  /// @brief Clear counted time and events
  static void Clear(void);

  /// @brief Start counting time
  void start(void);

  /// @brief Stop counting time
  void stop(void);

  /// @brief Clear counted time and events
  void clear(void);

  /// @brief Count a specific event for a number of times
  static void Event(const std::string& name, const unsigned int times = 1);

  /// @brief Count a specific event for a number of times
  void event(const std::string &name, const unsigned int times = 1);

  /// @brief Maintain the average of a specific value
  static void Average(const std::string& name, const double value);

  /// @brief Maintain the average of a specific value
  void average(const std::string &name, const double value);

  /// @brief Begin counting time for a specific chunk of code
  static void Begin(const std::string &name);

  /// @brief Stop counting time for a specific chunk of code
  static void End(const std::string &name);

  /// @brief Begin counting time for a specific chunk of code
  void begin(const std::string &name);

  /// @brief Stop counting time for a specific chunk of code
  void end(const std::string &name);

  /// @brief Print the status of the profiled code chunks and
  /// events. Optionally, computation done by different threads
  /// can be printed separately.
  static void Status(std::ostream &out = std::cout, bool merge = true);

  /// @brief Print the status of the profiled code chunks and
  /// events. Optionally, computation done by different threads
  /// can be printed separately.
  void status(std::ostream &out = std::cout, bool merge = true);

  /// @brief Check if the profiler is counting time or not
  bool running(void) const;

  /// @brief Check if the profiler is counting time or not
  static bool Running(void);

private:

  /// @brief Information about time spent in a section of the code
  struct TimeInfo
  {
    TimeInfo(void);

    /// @brief Total time counted.
    time::duration    total;

    /// @brief The shortest counted time interval
    time::duration    shortest;

    /// @brief The longest counted time interval
    time::duration    longest;

    /// @brief Number of times a chunk of time was added to this structure
    unsigned long int parts;

    /// @brief The point in time when counting time started
    time::point       start;

    /// @brief Begin counting time
    void set(void);

    /// @brief Add the counted time to the total time
    void update(void);
  };

  /// @brief Information maintained about averaged values
  struct AvgInfo
  {
    /// @brief The sum of the values to average
    double            total;

    /// @brief The sub of squares of the values to average
    double            totalSqr;

    /// @brief Number of times a value was added to this structure
    unsigned long int parts;
  };

  /// @brief Information to be maintained for each thread
  struct PerThread
  {
    /// @brief The stored events
    std::map<std::string, unsigned long int> events;

    /// @brief The stored averages
    std::map<std::string, AvgInfo>           avg;

    /// @brief The amount of time spent in various places
    std::map<std::string, TimeInfo>          time;
  };

  void printThreadInfo(std::ostream &out, const PerThread &data);

  std::mutex                             lock_;
  std::map<std::thread::id, PerThread>   data_;
  TimeInfo                               tinfo_;
  bool                                   running_;
  bool                                   printOnDestroy_;

};

/// @brief This instance will call Profiler::begin() when constructed and
/// Profiler::end() when it goes out of scope.
class FCL_EXPORT Profiler::ScopedBlock
{
public:
  /// @brief Start counting time for the block named \e name of the profiler
  /// \e prof
  ScopedBlock(const std::string &name, Profiler &prof = Profiler::Instance());

  ~ScopedBlock(void);

private:

  std::string  name_;
  Profiler    &prof_;
};

/// @brief This instance will call Profiler::start() when constructed and
/// Profiler::stop() when it goes out of scope.
/// If the profiler was already started, this block's constructor and
/// destructor take no action
class FCL_EXPORT Profiler::ScopedStart
{
public:

  /// @brief Take as argument the profiler instance to operate on (\e prof)
  ScopedStart(Profiler &prof = Profiler::Instance());

  ~ScopedStart(void);

private:

  Profiler &prof_;
  bool      wasRunning_;
};

} // namespace detail
} // namespace fcl

#endif // #ifndef FCL_COMMON_DETAIL_PROFILER_H

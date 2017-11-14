/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2014, Willow Garage, Inc.
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

#ifndef FCL_MATH_DETAIL_SEED_H
#define FCL_MATH_DETAIL_SEED_H

#include <cstdint>
#include "fcl/export.h"

namespace fcl
{
namespace detail
{

class FCL_EXPORT Seed
{
public:

  static bool isFirstSeedGenerated();

  static std::uint_fast32_t getUserSetSeed();

  static void setUserSetSeed(std::uint_fast32_t seed);

  static std::uint_fast32_t getFirstSeed();

  /// We use a different random number generator for the seeds of the
  /// Other random generators. The root seed is from the number of
  /// nano-seconds in the current time.
  static std::uint_fast32_t getNextSeed();

protected:

  Seed();

  static Seed& getInstance();

  /// The seed the user asked for (cannot be 0)
  std::uint_fast32_t userSetSeed;

  /// Flag indicating whether the first seed has already been generated or not
  bool firstSeedGenerated;

  /// The value of the first seed
  std::uint_fast32_t firstSeedValue;
};

} // namespace detail
} // namespace fcl

#endif

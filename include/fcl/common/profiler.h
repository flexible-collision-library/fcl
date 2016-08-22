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

/** @author Jeongseok Lee <jslee02@gmail.com> */

#ifndef FCL_COMMON_PROFILER_H
#define FCL_COMMON_PROFILER_H

#include "fcl/config.h"

#if FCL_ENABLE_PROFILING

  #define FCL_PROFILE_START                ::fcl::detail::Profiler::Start();
  #define FCL_PROFILE_STOP                 ::fcl::detail::Profiler::Stop();
  #define FCL_PROFILE_BLOCK_BEGIN(name)    ::fcl::detail::Profiler::Begin(name);
  #define FCL_PROFILE_BLOCK_END(name)      ::fcl::detail::Profiler::End(name);
  #define FCL_PROFILE_STATUS(stream)       ::fcl::detail::Profiler::Status(stream);

#else

  #define FCL_PROFILE_START
  #define FCL_PROFILE_STOP
  #define FCL_PROFILE_BLOCK_BEGIN(name)
  #define FCL_PROFILE_BLOCK_END(name)
  #define FCL_PROFILE_STATUS(stream)

#endif // #if FCL_ENABLE_PROFILING

#include "fcl/common/detail/profiler.h"

#endif // #ifndef FCL_COMMON_PROFILER_H

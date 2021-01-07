/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Open Source Robotics Foundation
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
 *   * Neither the name of CNRS-LAAS and AIST nor the names of its
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

/// @author Jeongseok Lee <jslee02@gmail.com>

#ifndef FCL_COMMON_WARNING_H
#define FCL_COMMON_WARNING_H

#include "fcl/config.h"

// We define two convenient macros that can be used to suppress
// deprecated-warnings for a specific code block rather than a whole project.
// This macros would be useful when you need to call deprecated function for
// some reason (e.g., for backward compatibility) but don't want warnings.
//
// Example code:
//
// deprecated_function()  // warning
//
// FCL_SUPPRESS_DEPRECATED_BEGIN
// deprecated_function()  // okay, no warning
// FCL_SUPPRESS_DEPRECATED_END
//
#if defined (FCL_COMPILER_GCC)

  #define FCL_SUPPRESS_DEPRECATED_BEGIN                             \
    _Pragma("GCC diagnostic push")                                  \
    _Pragma("GCC diagnostic ignored \"-Wdeprecated-declarations\"")

  #define FCL_SUPPRESS_DEPRECATED_END \
    _Pragma("GCC diagnostic pop")

  #define FCL_SUPPRESS_UNINITIALIZED_BEGIN                           \
    _Pragma("GCC diagnostic push")                                   \
    _Pragma("GCC diagnostic ignored \"-Wuninitialized\"")

  #define FCL_SUPPRESS_UNINITIALIZED_END \
    _Pragma("GCC diagnostic pop")

#elif defined (FCL_COMPILER_CLANG)

  #define FCL_SUPPRESS_DEPRECATED_BEGIN                               \
    _Pragma("clang diagnostic push")                                  \
    _Pragma("clang diagnostic ignored \"-Wdeprecated-declarations\"")

  #define FCL_SUPPRESS_DEPRECATED_END \
    _Pragma("clang diagnostic pop")

  #define FCL_SUPPRESS_UNINITIALIZED_BEGIN                            \
    _Pragma("clang diagnostic push")                                  \
    _Pragma("clang diagnostic ignored \"-Wuninitialized\"")

  #define FCL_SUPPRESS_UNINITIALIZED_END \
    _Pragma("clang diagnostic pop")

#elif defined (FCL_COMPILER_MSVC)

  #define FCL_SUPPRESS_DEPRECATED_BEGIN \
    __pragma(warning(push))             \
    __pragma(warning(disable:4996))

  #define FCL_SUPPRESS_DEPRECATED_END   \
    __pragma(warning(pop))

  #define FCL_SUPPRESS_UNINITIALIZED_BEGIN  // TODO

  #define FCL_SUPPRESS_UNINITIALIZED_END  // TODO

#endif

#endif // FCL_COMMON_WARNING_H

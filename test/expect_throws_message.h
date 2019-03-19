/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018. Toyota Research Institute
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

// This code was taken from Drake.
// https://github.com/RobotLocomotion/drake/blob/master/common/test_utilities/expect_throws_message.h

#ifndef FCL_EXPECT_THROWS_MESSAGE_H
#define FCL_EXPECT_THROWS_MESSAGE_H

#include <regex>
#include <string>

#ifdef FCL_DOXYGEN_CXX

/** Unit test helper macro for "expecting" an exception to be thrown but also
testing the error message against a provided regular expression. This is
like GTest's `EXPECT_THROW` but is fussier about the particular error message.
Usage example: @code
  FCL_EXPECT_THROWS_MESSAGE(
      StatementUnderTest(), // You expect this statement to throw ...
      std::logic_error,     // ... this exception with ...
      ".*some important.*phrases.*that must appear.*");  // ... this message.
@endcode
The regular expression must match the entire error message. If there is
boilerplate you don't care to match at the beginning and end, surround with
`.*` to ignore.

Following GTest's conventions, failure to perform as expected here is a
non-fatal test error. An `ASSERT` variant is provided to make it fatal. There
are also `*_IF_ARMED` variants. These require an exception in Debug builds. In
Release builds, the expression will pass if it _doesn't_ throw or if it throws
an exception that would pass the same test as in Debug builds. There is no
mechanism for testing _exclusive_ throwing behavior (i.e., only throws in
Debug).
@see FCL_ASSERT_THROWS_MESSAGE
@see FCL_EXPECT_THROWS_MESSAGE_IF_ARMED, FCL_ASSERT_THROWS_MESSAGE_IF_ARMED */
#define FCL_EXPECT_THROWS_MESSAGE(expression, exception, regexp)

/** Fatal error version of `FCL_EXPECT_THROWS_MESSAGE`.
@see FCL_EXPECT_THROWS_MESSAGE */
#define FCL_ASSERT_THROWS_MESSAGE(expression, exception, regexp)

/** Same as `FCL_EXPECT_THROWS_MESSAGE` in Debug builds, but doesn't require
a throw in Release builds. However, if the Release build does throw it must
throw the right message. More precisely, the thrown message is required
whenever `FCL_ENABLE_ASSERTS` is defined, which Debug builds do by default.
@see FCL_EXPECT_THROWS_MESSAGE */
#define FCL_EXPECT_THROWS_MESSAGE_IF_ARMED(expression, exception, regexp)

/** Same as `FCL_ASSERT_THROWS_MESSAGE` in Debug builds, but doesn't require
a throw in Release builds. However, if the Release build does throw it must
throw the right message. More precisely, the thrown message is required
whenever `FCL_ENABLE_ASSERTS` is defined, which Debug builds do by default.
@see FCL_ASSERT_THROWS_MESSAGE */
#define FCL_ASSERT_THROWS_MESSAGE_IF_ARMED(expression, exception, regexp)

#else  // FCL_DOXYGEN_CXX

#define FCL_EXPECT_THROWS_MESSAGE_HELPER(expression, exception, regexp, \
                                         must_throw, fatal_failure) \
try { \
  expression; \
  if (must_throw) { \
    if (fatal_failure) { \
      GTEST_FATAL_FAILURE_("\t" #expression " failed to throw " #exception); \
    } else { \
      GTEST_NONFATAL_FAILURE_("\t" #expression " failed to throw " #exception);\
    } \
  } \
} catch (const exception& err) { \
  auto matcher = [](const char* s, const std::string& re) { \
    return std::regex_match(s, std::regex(re)); }; \
  if (fatal_failure) { \
    ASSERT_PRED2(matcher, err.what(), regexp); \
  } else { \
    EXPECT_PRED2(matcher, err.what(), regexp); \
  } \
}

#define FCL_EXPECT_THROWS_MESSAGE(expression, exception, regexp) \
  FCL_EXPECT_THROWS_MESSAGE_HELPER(expression, exception, regexp, \
                                     true /*must_throw*/, false /*non-fatal*/)

#define FCL_ASSERT_THROWS_MESSAGE(expression, exception, regexp) \
  FCL_EXPECT_THROWS_MESSAGE_HELPER(expression, exception, regexp, \
                                   true /*must_throw*/, true /*fatal*/)

#ifdef NDEBUG
// Throwing the expected message is optional in this case.

#define FCL_EXPECT_THROWS_MESSAGE_IF_DEBUG(expression, exception, regexp) \
  FCL_EXPECT_THROWS_MESSAGE_HELPER(expression, exception, regexp, \
                                   false /*optional*/, false /*non-fatal*/)

#define FCL_ASSERT_THROWS_MESSAGE_IF_DEBUG(expression, exception, regexp) \
  FCL_EXPECT_THROWS_MESSAGE_HELPER(expression, exception, regexp, \
                                   false /*optional*/, true /*fatal*/)

#else  // NDEBUG
// Throwing the expected message is required in this case.

#define FCL_EXPECT_THROWS_MESSAGE_IF_DEBUG(expression, exception, regexp) \
  FCL_EXPECT_THROWS_MESSAGE(expression, exception, regexp)

#define FCL_ASSERT_THROWS_MESSAGE_IF_DEBUG(expression, exception, regexp) \
  FCL_ASSERT_THROWS_MESSAGE(expression, exception, regexp)

#endif  // NDEBUG

#endif  // FCL_DOXYGEN_CXX

#endif  // FCL_EXPECT_THROWS_MESSAGE_H

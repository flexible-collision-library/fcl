/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Toyota Research Institute
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

/** @author Sean Curtis */

#ifndef FCL_SHAPE_REPRESENTATION_UTIL_H
#define FCL_SHAPE_REPRESENTATION_UTIL_H

#include <regex>

#include <gtest/gtest.h>

namespace fcl {
namespace detail {

// For the representation() method we just need to show that it outputs
// a string that can be used to instantiate the shape. It is sufficient to show
// that it outputs *a* valid format (and not necessarily the original).
//
// It may be that formatting standards change, it's alright to reformulate
// the code captured here or the representation() method implementation to keep
// this test passing.
//
// To that end, we'll use a code string that gets converted into a string and
// gets compiled and confirm that it is equal to the string given by
// representation() (modulo some whitespace artifacts).
#define INSTANTIATE_AND_SAVE_STRING(code) \
  const std::string code_string(#code);   \
  const auto shape = code;

// Given the shape and a string comprising the code that constructs it, confirms
// two things:
//
//   1. With "sufficient" precision, the representation() of the shape matches
//      the given code string.
//   2. With "insufficient" precision, they don't match.
//
// Note: The parameters for the shape should have *at least* two digits of
// precision for this test to work (e.g., prefer 1.5 to 1). But within that
// requirement, things will work best if the values are perfectly represented
// by floating point values.
template <typename Shape>
::testing::AssertionResult ValidateRepresentation(
    const Shape& shape, const std::string& code_string) {
  bool success = true;
  ::testing::AssertionResult failure = ::testing::AssertionFailure();
  // Normalize whitespace; newlines and blocks of spaces become single spaces.
  const std::regex whitespace_re("[\\s\\n]+");
  const std::string expected =
      std::regex_replace(code_string, whitespace_re, " ");

  // Confirm precision matters. We get an exact match with sufficient precision.
  // Insufficient precision causes round-off and the strings don't match.
  {
    const std::string tested =
        std::regex_replace(shape.representation(20), whitespace_re, " ");
    if (tested != expected) {
        success = false;
      failure
          << "Representation string mismatch with sufficient precision:"
          << "\n  expected: " << expected
          << "\n  tested: " << tested;
    }
  }
  {
    const std::string tested =
        std::regex_replace(shape.representation(1), whitespace_re, " ");
    if (tested == expected) {
        success = false;
      failure
          << "Representation string still matches with insufficient precision:"
          << "\n  expected: " << expected
          << "\n  tested: " << tested;
    }
  }
  if (success) {
    return ::testing::AssertionSuccess();
  }
  return failure;
}

}  // namespace detail
}  // namespace fcl

#endif  // FCL_SHAPE_REPRESENTATION_UTIL_H

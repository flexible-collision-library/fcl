/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023. Toyota Research Institute
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

/** @author Sean Curtis (sean@tri.global) (2023) */

#include "fcl/geometry/shape/cylinder.h"

#include <regex>

#include <gtest/gtest.h>

namespace fcl {
namespace {

// TODO(SeanCurtis-TRI): Test everything in Cylinder's API.

// For the Representation() method we just need to show that it outputs
// a string that can be used to instantiate the shape. It is sufficient to show
// that it outputs *a* valid format (and not necessarily the original).
//
// It may be that formatting standards change, it's alright to reformulate
// the code captured here or the Representation() method implementation to keep
// this test passing.
//
// To that end, we'll use a code string that gets converted into a string and
// gets compiled and confirm that it is equal to the string given by
// Representation() (modulo some whitespace artifacts).
#define INSTANTIATE_AND_SAVE_STRING(code) \
  const std::string code_string(#code);   \
  const auto shape = code;

GTEST_TEST(CylinderGeometry, Representation) {
  INSTANTIATE_AND_SAVE_STRING(Cylinder<double>(2, 3);)
  // Normalize on whitespace; all newlines and blocks of spaces become single
  // spaces.
  const std::regex whitespace_re("[\\s\\n]+");
  EXPECT_EQ(std::regex_replace(shape.Representation(), whitespace_re, " "),
            std::regex_replace(code_string, whitespace_re, " "));
}

}
}  // namespace fcl

//==============================================================================
int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019. Toyota Research Institute
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

/** @author Sean Curtis (sean@tri.global) */

#ifndef FCL_FAILED_AT_THIS_CONFIGURATION_H
#define FCL_FAILED_AT_THIS_CONFIGURATION_H

#include <exception>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "fcl/export.h"

namespace fcl {
namespace detail {

/** A custom exception type that can be thrown in low-level code indicating
 that the program has reached an unexpected configuration and that the caller
 that ultimately has access to the collision objects and their transforms
 should report the configuration that led to the exception.

 This is strictly an _internal_ exception; it should *always* be caught and
 transformed into an exception that propagates to the operating system.

 Recommended usage is to throw by invoking the macro
 FCL_THROW_UNEXPECTED_CONFIGURATION_EXCEPTION defined below. Code that exercises
 functionality that throws this type of exception should catch it and transform
 it to a `std::logic_error` by invoking ThrowDetailedConfiguration().  */
class FCL_EXPORT FailedAtThisConfiguration final
    : public std::exception {
 public:
  FailedAtThisConfiguration(const std::string& message)
      : std::exception(), message_(message) {}

  const char* what() const noexcept final { return message_.c_str(); }

 private:
  std::string message_;
};

/** Helper function for dispatching an `FailedAtThisConfiguration`.
 Because this exception is designed to be caught and repackaged, we lose the
 automatic association with file and line number. This wraps them into the
 message of the exception so it can be preserved in the re-wrapping.

 @param message  The error message itself.
 @param func     The name of the invoking function.
 @param file     The name of the file associated with the exception.
 @param line     The line number where the exception is thrown.  */
FCL_EXPORT void ThrowFailedAtThisConfiguration(
    const std::string& message, const char* func, const char* file, int line);

/** Helper class for propagating a low-level exception upwards but with
 configuration-specific details appended. The parameters
 
 @param s1        The first shape in the query.
 @param X_FS1     The pose of the first shape in frame F.
 @param s2        The second shape in the query.
 @param X_FS2     The pose of the second shape in frame F.
 @param solver    The solver.
 @param e         The original exception.
 @tparam Shape1   The type of shape for shape 1.
 @tparam Shape2   The type of shape for shape 2.
 @tparam Solver   The solver type (with scalar type erase).
 @tparam Pose     The pose type (a Transform<S> with scalar type erased).
 */
template <typename Shape1, typename Shape2, typename Solver, typename Pose>
void ThrowDetailedConfiguration(const Shape1& s1, const Pose& X_FS1,
                                const Shape2& s2, const Pose& X_FS2,
                                const Solver& solver, const std::exception& e) {
  std::stringstream ss;
  ss << std::setprecision(20);
  ss << "Error with configuration"
     << "\n  Original error message: " << e.what()
     << "\n  Shape 1: " << s1
     << "\n  X_FS1\n" << X_FS1.matrix()
     << "\n  Shape 2: " << s2
     << "\n  X_FS2\n" << X_FS2.matrix()
     << "\n  Solver: " << solver;
  throw std::logic_error(ss.str());
}

}  // namespace detail
}  // namespace fcl

#define FCL_THROW_FAILED_AT_THIS_CONFIGURATION(message)            \
  ::fcl::detail::ThrowFailedAtThisConfiguration(message, __func__, __FILE__, \
                                                __LINE__)

#endif  // FCL_FAILED_AT_THIS_CONFIGURATION_H

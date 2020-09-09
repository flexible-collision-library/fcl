/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Rice University
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
 *   * Neither the name of Rice University nor the names of its
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

/** @author Mark Moll */

#ifndef FCL_MATH_CONSTANTS_
#define FCL_MATH_CONSTANTS_

#include "fcl/common/types.h"

#include <limits>
#include <cmath>

namespace fcl {

namespace detail {

// Helper struct for determining the underlying numerical type of scalars.
// Allows us to treat AutoDiffScalar<double> and double as double type and
// AutoDiffScalar<float> and float as float type.
template<typename S>
struct ScalarTrait {
  // NOTE: This relies on AutoDiffScalar's `Real` class member and serves as
  // an entry path for any custom scalar class that likewise defines a `Real`
  // class member.
  typedef typename S::Real type;
};

template<>
struct ScalarTrait<long double> {
  typedef long double type;
};

template<>
struct ScalarTrait<double> {
  typedef double type;
};

template<>
struct ScalarTrait<float> {
  typedef float type;
};

}  // namespace detail

/// A collection of scalar-dependent constants. This provides the ability to
/// get mathematical constants and tolerance values that are appropriately
/// scaled and typed to the scalar type `S`.
///
/// Constants `pi()` and `phi()` are returned in the literal scalar type `S`.
/// In other words, if `S` is an `AutoDiffScalar<...>`, then the value of `pi`
/// and `phi` are likewise `AutoDiffScalar<...>` typed.
///
/// Tolerances (e.g., `eps()` and its variants) are always provided in the
/// scalar's numerical representation. In other words, if `S` is a `double` or
/// `float`, the tolerances are given as `double` and `float`, respectively.
/// For `AutoDiffScalar` it is more interesting. The `AutoDiffScalar` has an
/// underlying numerical representation (e.g.,
/// `AutoDiffScalar<Matrix<double, 1, 3>>` has a double). It is the type of this
/// underlying numerical representation that is provided by the tolerance
/// functions.
///
/// This is designed to specifically work with `float`, `double`, `long double`,
/// and corresponding `AutoDiffScalar` types. However, custom scalars will also
/// work provided that the scalar type provides a class member type `Real`
/// which must be one of `long double`, `double`, or `float`. E.g.,
///
/// ```
/// struct MyScalar {
///  public:
///   typedef double Real;
///   ...
/// };
/// ```
///
/// @note The tolerance values provided are defined so as to provide varying
/// precision that *scales* with the underlying numerical type. The
/// following contrast will make it clear.
/// ```
/// S local_eps = 10 * std::numeric_limit<S>::epsilon();  // DON'T DO THIS!
/// ```
/// The above example shows a common but incorrect method for defining a local
/// epsilon. It defines it as being an order of magnitude larger (base 10) than
/// the machine epsilon for `S`. However, if `S` is a float, its epsilon is
/// essentially 1e-7. A full decimal digit of precision is 1/7th of the
/// available digits. In contrast, double epsilon is approximately 2e-16.
/// Throwing away a digit there reduces the precision by only 1/16th. This
/// technique disproportionately punishes lower-precision numerical
/// representations. Instead, by raising epsilon to a fractional power, we
/// *scale* the precision. Roughly, `ε^(1/2)` gives us half the
/// precision (3.5e-4 for floats and 1.5e-8 for doubles). Similarly powers of
/// 3/4 and 7/8 gives us three quarters and 7/8ths of the bits of precision. By
/// defining tolerances in this way, one can get some fraction of machine
/// precision, regardless of the choice of numeric type.
///
/// \tparam S The scalar type for which constant values will be retrieved.
template <typename S>
struct FCL_EXPORT constants
{
typedef typename detail::ScalarTrait<S>::type Real;

/// The mathematical constant pi.
static constexpr S pi() { return S(3.141592653589793238462643383279502884197169399375105820974944592L); }

/// The golden ratio.
static constexpr S phi() { return S(1.618033988749894848204586834365638117720309179805762862135448623L); }

/// Defines the default accuracy for gjk and epa tolerance. It is defined as
/// ε^(7/8) -- where ε is the machine precision epsilon for
/// the in-use Real. The value is a much smaller epsilon for doubles than
/// for floats (2e-14 vs 9e-7, respectively). The choice of ε^(7/8) as the
/// default GJK tolerance reflects a tolerance that is a *slightly* tighter
/// bound than the historical value of 1e-6 used for 32-bit floats.
static Real gjk_default_tolerance() {
  static const Real value = eps_78();
  return value;
}

/// Returns ε for the precision of the underlying scalar type.
static constexpr Real eps() {
  static_assert(std::is_floating_point<Real>::value,
                "Constants can only be evaluated for scalars with floating "
                "point implementations");
  return std::numeric_limits<Real>::epsilon();
}

// TODO(SeanCurtis-TRI) These are *not* declared constexpr because the clang
//  compiler available in the current CI configuration for ubuntu and mac does
//  not have std::pow declared as constexpr. When that changes, these can
//  likewise be declared as constexpr.

/// Returns ε^(7/8) for the precision of the underlying scalar type.
static Real eps_78() {
  static const Real value = std::pow(eps(), 7./8.);
  return value;
}

/// Returns ε^(3/4) for the precision of the underlying scalar type.
static Real eps_34() {
  static const Real value = std::pow(eps(), 3./4.);
  return value;
}

/// Returns ε^(1/2) for the precision of the underlying scalar type.
static Real eps_12() {
  static const Real value = std::pow(eps(), 1./2.);
  return value;
}

};

using constantsf = constants<float>;
using constantsd = constants<double>;

} // namespace fcl

#endif

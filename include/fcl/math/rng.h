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

/** \author Jia Pan */

#ifndef FCL_MATH_RNG_H
#define FCL_MATH_RNG_H

#include <cassert>
#include <random>
#include <iostream>

#include "fcl/math/constants.h"
#include "fcl/math/detail/seed.h"

namespace fcl
{

/// @brief Random number generation. An instance of this class
/// cannot be used by multiple threads at once (member functions
/// are not const). However, the constructor is thread safe and
/// different instances can be used safely in any number of
/// threads. It is also guaranteed that all created instances will
/// have a different random seed.
template <typename Scalar>
class RNG
{
public:
  /// @brief Constructor. Always sets a different random seed
  RNG();

  /// @brief Generate a random real between 0 and 1
  Scalar uniform01();

  /// @brief Generate a random real within given bounds: [\e lower_bound,
  /// \e upper_bound)
  Scalar uniformReal(Scalar lower_bound, Scalar upper_bound);

  /// @brief Generate a random integer within given bounds: [\e lower_bound,
  /// \e upper_bound]
  int uniformInt(int lower_bound, int upper_bound);

  /// @brief Generate a random boolean
  bool uniformBool();

  /// @brief Generate a random real using a normal distribution with mean 0 and
  /// variance 1
  Scalar gaussian01();

  /// @brief Generate a random real using a normal distribution with given mean
  /// and variance
  Scalar gaussian(Scalar mean, Scalar stddev);

  /// @brief Generate a random real using a half-normal distribution. The value
  /// is within specified bounds [\e r_min, \e r_max], but with a bias towards
  /// \e r_max. The function is implemended using a Gaussian distribution with
  /// mean at \e r_max - \e r_min. The distribution is 'folded' around \e r_max
  /// axis towards \e r_min. The variance of the distribution is (\e r_max -
  /// \e r_min) / \e focus. The higher the focus, the more probable it is that
  /// generated numbers are close to \e r_max.
  Scalar halfNormalReal(Scalar r_min, Scalar r_max, Scalar focus = 3.0);

  /// @brief Generate a random integer using a half-normal distribution. The
  /// value is within specified bounds ([\e r_min, \e r_max]), but with a bias
  /// towards \e r_max. The function is implemented on top of halfNormalReal()
  int halfNormalInt(int r_min, int r_max, Scalar focus = 3.0);

  /// @brief Uniform random unit quaternion sampling. The computed value has the
  /// order (x,y,z,w)
  void quaternion(Scalar value[4]);

  /// @brief Uniform random sampling of Euler roll-pitch-yaw angles, each in the
  /// range [-pi, pi). The computed value has the order (roll, pitch, yaw) */
  void eulerRPY(Scalar value[3]);

  /// @brief Uniform random sample on a disk with radius from r_min to r_max
  void disk(Scalar r_min, Scalar r_max, Scalar& x, Scalar& y);

  /// @brief Uniform random sample in a ball with radius from r_min to r_max
  void ball(Scalar r_min, Scalar r_max, Scalar& x, Scalar& y, Scalar& z);

  /// @brief Set the seed for random number generation. Use this function to
  /// ensure the same sequence of random numbers is generated.
  static void setSeed(std::uint_fast32_t seed);

  /// @brief Get the seed used for random number generation. Passing the
  /// returned value to setSeed() at a subsequent execution of the code will
  /// ensure deterministic (repeatable) behaviour. Useful for debugging.
  static std::uint_fast32_t getSeed();

private:

  std::mt19937                     generator_;
  std::uniform_real_distribution<> uniDist_;
  std::normal_distribution<>       normalDist_;

};

using RNGf = RNG<float>;
using RNGd = RNG<double>;

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
RNG<Scalar>::RNG()
  : generator_(detail::Seed::getNextSeed()), uniDist_(0, 1), normalDist_(0, 1)
{
}

//==============================================================================
template <typename Scalar>
Scalar RNG<Scalar>::uniform01()
{
  return uniDist_(generator_);
}

//==============================================================================
template <typename Scalar>
Scalar RNG<Scalar>::uniformReal(Scalar lower_bound, Scalar upper_bound)
{
  assert(lower_bound <= upper_bound);

  return (upper_bound - lower_bound) * uniDist_(generator_) + lower_bound;
}

//==============================================================================
template <typename Scalar>
int RNG<Scalar>::uniformInt(int lower_bound, int upper_bound)
{
  int r = (int)floor(uniformReal((Scalar)lower_bound, (Scalar)(upper_bound) + 1.0));

  return (r > upper_bound) ? upper_bound : r;
}

//==============================================================================
template <typename Scalar>
bool RNG<Scalar>::uniformBool()
{
  return uniDist_(generator_) <= 0.5;
}

//==============================================================================
template <typename Scalar>
Scalar RNG<Scalar>::gaussian01()
{
  return normalDist_(generator_);
}

//==============================================================================
template <typename Scalar>
Scalar RNG<Scalar>::gaussian(Scalar mean, Scalar stddev)
{
  return normalDist_(generator_) * stddev + mean;
}

//==============================================================================
template <typename Scalar>
Scalar RNG<Scalar>::halfNormalReal(Scalar r_min, Scalar r_max, Scalar focus)
{
  assert(r_min <= r_max);

  const auto mean = r_max - r_min;
  auto v = gaussian(mean, mean / focus);

  if (v > mean)
    v = 2.0 * mean - v;

  auto r = v >= 0.0 ? v + r_min : r_min;

  return r > r_max ? r_max : r;
}

//==============================================================================
template <typename Scalar>
int RNG<Scalar>::halfNormalInt(int r_min, int r_max, Scalar focus)
{
  int r = (int)std::floor(halfNormalReal(
                       (Scalar)r_min, (Scalar)(r_max) + 1.0, focus));

  return (r > r_max) ? r_max : r;
}

//==============================================================================
template <typename Scalar>
void RNG<Scalar>::quaternion(Scalar value[])
{
  auto x0 = uniDist_(generator_);
  auto r1 = std::sqrt(1.0 - x0), r2 = std::sqrt(x0);
  auto t1 = 2.0 * constants::pi * uniDist_(generator_);
  auto t2 = 2.0 * constants::pi * uniDist_(generator_);
  auto c1 = std::cos(t1);
  auto s1 = std::sin(t1);
  auto c2 = std::cos(t2);
  auto s2 = std::sin(t2);
  value[0] = s1 * r1;
  value[1] = c1 * r1;
  value[2] = s2 * r2;
  value[3] = c2 * r2;
}

//==============================================================================
template <typename Scalar>
void RNG<Scalar>::eulerRPY(Scalar value[])
{
  value[0] = constants::pi * (2.0 * uniDist_(generator_) - 1.0);
  value[1] = std::acos(1.0 - 2.0 * uniDist_(generator_)) - constants::pi / 2.0;
  value[2] = constants::pi * (2.0 * uniDist_(generator_) - 1.0);
}

//==============================================================================
template <typename Scalar>
void RNG<Scalar>::disk(Scalar r_min, Scalar r_max, Scalar& x, Scalar& y)
{
  auto a = uniform01();
  auto b = uniform01();
  auto r = std::sqrt(a * r_max * r_max + (1 - a) * r_min * r_min);
  auto theta = 2 * constants::pi * b;
  x = r * std::cos(theta);
  y = r * std::sin(theta);
}

//==============================================================================
template <typename Scalar>
void RNG<Scalar>::ball(
    Scalar r_min, Scalar r_max, Scalar& x, Scalar& y, Scalar& z)
{
  auto a = uniform01();
  auto b = uniform01();
  auto c = uniform01();
  auto r = std::pow(a*std::pow(r_max, 3) + (1 - a)*std::pow(r_min, 3), 1/3.0);
  auto theta = std::acos(1 - 2 * b);
  auto phi = 2 * constants::pi * c;

  auto costheta = std::cos(theta);
  auto sintheta = std::sin(theta);
  auto cosphi = std::cos(phi);
  auto sinphi = std::sin(phi);
  x = r * costheta;
  y = r * sintheta * cosphi;
  z = r * sintheta * sinphi;
}

//==============================================================================
template <typename Scalar>
void RNG<Scalar>::setSeed(uint_fast32_t seed)
{
  if (detail::Seed::isFirstSeedGenerated())
  {
    std::cerr << "Random number generation already started. Changing seed now "
              << "will not lead to deterministic sampling." << std::endl;
  }

  if (seed == 0)
  {
    std::cerr << "Random generator seed cannot be 0. Using 1 instead."
              << std::endl;
    detail::Seed::setUserSetSeed(1);
  }
  else
  {
    detail::Seed::setUserSetSeed(seed);
  }
}

//==============================================================================
template <typename Scalar>
uint_fast32_t RNG<Scalar>::getSeed()
{
  return detail::Seed::getFirstSeed();
}

}

#endif

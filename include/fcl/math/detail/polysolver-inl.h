/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
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

#ifndef FCL_NARROWPHASE_DETAIL_POLYSOLVER_INL_H
#define FCL_NARROWPHASE_DETAIL_POLYSOLVER_INL_H

#include "fcl/math/detail/polysolver.h"

#include <cmath>
#include "fcl/common/types.h"

namespace fcl
{

namespace detail {

//==============================================================================
extern template
class FCL_EXPORT PolySolver<double>;

//==============================================================================
template <typename S>
int PolySolver<S>::solveLinear(S c[2], S s[1])
{
  if(isZero(c[1]))
    return 0;
  s[0] = - c[0] / c[1];
  return 1;
}

//==============================================================================
template <typename S>
int PolySolver<S>::solveQuadric(S c[3], S s[2])
{
  S p, q, D;

  // make sure we have a d2 equation

  if(isZero(c[2]))
    return solveLinear(c, s);

  // normal for: x^2 + px + q
  p = c[1] / (2.0 * c[2]);
  q = c[0] / c[2];
  D = p * p - q;

  if(isZero(D))
  {
    // one S root
    s[0] = s[1] = -p;
    return 1;
  }

  if(D < 0.0)
    // no real root
    return 0;
  else
  {
    // two real roots
    S sqrt_D = sqrt(D);
    s[0] = sqrt_D - p;
    s[1] = -sqrt_D - p;
    return 2;
  }
}

//==============================================================================
template <typename S>
int PolySolver<S>::solveCubic(S c[4], S s[3])
{
  int i, num;
  S sub, A, B, C, sq_A, p, q, cb_p, D;
  const S ONE_OVER_THREE = 1 / 3.0;
  const S PI = 3.14159265358979323846;

  // make sure we have a d2 equation
  if(isZero(c[3]))
    return solveQuadric(c, s);

  // normalize the equation:x ^ 3 + Ax ^ 2 + Bx  + C = 0
  A = c[2] / c[3];
  B = c[1] / c[3];
  C = c[0] / c[3];

  // substitute x = y - A / 3 to eliminate the quadratic term: x^3 + px + q = 0
  sq_A = A * A;
  p = (-ONE_OVER_THREE * sq_A + B) * ONE_OVER_THREE;
  q = 0.5 * (2.0 / 27.0 * A * sq_A - ONE_OVER_THREE * A * B + C);

  // use Cardano's formula
  cb_p = p * p * p;
  D = q * q + cb_p;

  if(isZero(D))
  {
    if(isZero(q))
    {
      // one triple solution
      s[0] = 0.0;
      num = 1;
    }
    else
    {
      // one single and one S solution
      S u = cbrt(-q);
      s[0] = 2.0 * u;
      s[1] = -u;
      num = 2;
    }
  }
  else
  {
    if(D < 0.0)
    {
      // three real solutions
      S phi = ONE_OVER_THREE * acos(-q / sqrt(-cb_p));
      S t = 2.0 * sqrt(-p);
      s[0] = t * cos(phi);
      s[1] = -t * cos(phi + PI / 3.0);
      s[2] = -t * cos(phi - PI / 3.0);
      num = 3;
    }
    else
    {
      // one real solution
      S sqrt_D = sqrt(D);
      S u = cbrt(sqrt_D + fabs(q));
      if(q > 0.0)
        s[0] = - u + p / u ;
      else
        s[0] = u - p / u;
      num = 1;
    }
  }

  // re-substitute
  sub = ONE_OVER_THREE * A;
  for(i = 0; i < num; i++)
    s[i] -= sub;
  return num;
}

//==============================================================================
template <typename S>
bool PolySolver<S>::isZero(S v)
{
  return (v < getNearZeroThreshold()) && (v > -getNearZeroThreshold());
}

//==============================================================================
template <typename S>
bool PolySolver<S>::cbrt(S v)
{
  return std::pow(v, 1.0 / 3.0);
}

//==============================================================================
template <typename S>
constexpr S PolySolver<S>::getNearZeroThreshold()
{
  return 1e-9;
}

} // namespace detail
} // namespace fcl

#endif

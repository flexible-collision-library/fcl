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

#ifndef FCL_BV_KDOP_INL_H
#define FCL_BV_KDOP_INL_H

#include "fcl/math/bv/kDOP.h"

#include "fcl/common/unused.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT KDOP<double, 16>;

//==============================================================================
extern template
class FCL_EXPORT KDOP<double, 18>;

//==============================================================================
extern template
class FCL_EXPORT KDOP<double, 24>;

//==============================================================================
extern template
void minmax(double a, double b, double& minv, double& maxv);

//==============================================================================
extern template
void minmax(double p, double& minv, double& maxv);

//==============================================================================
extern template
void getDistances<double, 5>(const Vector3<double>& p, double* d);

//==============================================================================
extern template
void getDistances<double, 6>(const Vector3<double>& p, double* d);

//==============================================================================
extern template
void getDistances<double, 9>(const Vector3<double>& p, double* d);

//==============================================================================
template <typename S, std::size_t N>
FCL_EXPORT
KDOP<S, N>::KDOP()
{
  static_assert(N == 16 || N == 18 || N == 24, "N should be 16, 18, or 24");

  S real_max = std::numeric_limits<S>::max();
  for(std::size_t i = 0; i < N / 2; ++i)
  {
    dist_[i] = real_max;
    dist_[i + N / 2] = -real_max;
  }
}

//==============================================================================
template <typename S, std::size_t N>
FCL_EXPORT
KDOP<S, N>::KDOP(const Vector3<S>& v)
{
  for(std::size_t i = 0; i < 3; ++i)
  {
    dist_[i] = dist_[N / 2 + i] = v[i];
  }

  S d[(N - 6) / 2];
  getDistances<S, (N - 6) / 2>(v, d);
  for(std::size_t i = 0; i < (N - 6) / 2; ++i)
  {
    dist_[3 + i] = dist_[3 + i + N / 2] = d[i];
  }
}

//==============================================================================
template <typename S, std::size_t N>
FCL_EXPORT
KDOP<S, N>::KDOP(const Vector3<S>& a, const Vector3<S>& b)
{
  for(std::size_t i = 0; i < 3; ++i)
  {
    minmax(a[i], b[i], dist_[i], dist_[i + N / 2]);
  }

  S ad[(N - 6) / 2], bd[(N - 6) / 2];
  getDistances<S, (N - 6) / 2>(a, ad);
  getDistances<S, (N - 6) / 2>(b, bd);
  for(std::size_t i = 0; i < (N - 6) / 2; ++i)
  {
    minmax(ad[i], bd[i], dist_[3 + i], dist_[3 + i + N / 2]);
  }
}

//==============================================================================
template <typename S, std::size_t N>
FCL_EXPORT
bool KDOP<S, N>::overlap(const KDOP<S, N>& other) const
{
  for(std::size_t i = 0; i < N / 2; ++i)
  {
    if(dist_[i] > other.dist_[i + N / 2]) return false;
    if(dist_[i + N / 2] < other.dist_[i]) return false;
  }

  return true;
}

//==============================================================================
template <typename S, std::size_t N>
FCL_EXPORT
bool KDOP<S, N>::inside(const Vector3<S>& p) const
{
  for(std::size_t i = 0; i < 3; ++i)
  {
    if(p[i] < dist_[i] || p[i] > dist_[i + N / 2])
      return false;
  }

  S d[(N - 6) / 2];
  getDistances<S, (N - 6) / 2>(p, d);
  for(std::size_t i = 0; i < (N - 6) / 2; ++i)
  {
    if(d[i] < dist_[3 + i] || d[i] > dist_[i + 3 + N / 2])
      return false;
  }

  return true;
}

//==============================================================================
template <typename S, std::size_t N>
FCL_EXPORT
KDOP<S, N>& KDOP<S, N>::operator += (const Vector3<S>& p)
{
  for(std::size_t i = 0; i < 3; ++i)
  {
    minmax(p[i], dist_[i], dist_[N / 2 + i]);
  }

  S pd[(N - 6) / 2];
  getDistances<S, (N - 6) / 2>(p, pd);
  for(std::size_t i = 0; i < (N - 6) / 2; ++i)
  {
    minmax(pd[i], dist_[3 + i], dist_[3 + N / 2 + i]);
  }

  return *this;
}

//==============================================================================
template <typename S, std::size_t N>
FCL_EXPORT
KDOP<S, N>& KDOP<S, N>::operator += (const KDOP<S, N>& other)
{
  for(std::size_t i = 0; i < N / 2; ++i)
  {
    dist_[i] = std::min(other.dist_[i], dist_[i]);
    dist_[i + N / 2] = std::max(other.dist_[i + N / 2], dist_[i + N / 2]);
  }
  return *this;
}

//==============================================================================
template <typename S, std::size_t N>
FCL_EXPORT
KDOP<S, N> KDOP<S, N>::operator + (const KDOP<S, N>& other) const
{
  KDOP<S, N> res(*this);
  return res += other;
}

//==============================================================================
template <typename S, std::size_t N>
FCL_EXPORT
S KDOP<S, N>::width() const
{
  return dist_[N / 2] - dist_[0];
}

//==============================================================================
template <typename S, std::size_t N>
FCL_EXPORT
S KDOP<S, N>::height() const
{
  return dist_[N / 2 + 1] - dist_[1];
}

//==============================================================================
template <typename S, std::size_t N>
FCL_EXPORT
S KDOP<S, N>::depth() const
{
  return dist_[N / 2 + 2] - dist_[2];
}

//==============================================================================
template <typename S, std::size_t N>
FCL_EXPORT
S KDOP<S, N>::volume() const
{
  return width() * height() * depth();
}

//==============================================================================
template <typename S, std::size_t N>
FCL_EXPORT
S KDOP<S, N>::size() const
{
  return width() * width() + height() * height() + depth() * depth();
}

//==============================================================================
template <typename S, std::size_t N>
FCL_EXPORT
Vector3<S> KDOP<S, N>::center() const
{
  return Vector3<S>(dist_[0] + dist_[N / 2], dist_[1] + dist_[N / 2 + 1], dist_[2] + dist_[N / 2 + 2]) * 0.5;
}

//==============================================================================
template <typename S, std::size_t N>
FCL_EXPORT
S KDOP<S, N>::distance(const KDOP<S, N>& other, Vector3<S>* P, Vector3<S>* Q) const
{
  FCL_UNUSED(other);
  FCL_UNUSED(P);
  FCL_UNUSED(Q);

  std::cerr << "KDOP distance not implemented!\n";
  return 0.0;
}

//==============================================================================
template <typename S, std::size_t N>
FCL_EXPORT
S KDOP<S, N>::dist(std::size_t i) const
{
  return dist_[i];
}

//==============================================================================
template <typename S, std::size_t N>
FCL_EXPORT
S& KDOP<S, N>::dist(std::size_t i)
{
  return dist_[i];
}

//==============================================================================
template <typename S, std::size_t N, typename Derived>
FCL_EXPORT
KDOP<S, N> translate(
    const KDOP<S, N>& bv, const Eigen::MatrixBase<Derived>& t)
{
  KDOP<S, N> res(bv);
  for(std::size_t i = 0; i < 3; ++i)
  {
    res.dist(i) += t[i];
    res.dist(N / 2 + i) += t[i];
  }

  S d[(N - 6) / 2];
  getDistances<S, (N - 6) / 2>(t, d);
  for(std::size_t i = 0; i < (N - 6) / 2; ++i)
  {
    res.dist(3 + i) += d[i];
    res.dist(3 + i + N / 2) += d[i];
  }

  return res;
}

//==============================================================================
template <typename S>
FCL_EXPORT
void minmax(S a, S b, S& minv, S& maxv)
{
  if(a > b)
  {
    minv = b;
    maxv = a;
  }
  else
  {
    minv = a;
    maxv = b;
  }
}

//==============================================================================
template <typename S>
FCL_EXPORT
void minmax(S p, S& minv, S& maxv)
{
  if(p > maxv) maxv = p;
  if(p < minv) minv = p;
}

//==============================================================================
template <typename S, std::size_t N>
struct GetDistancesImpl
{
  static void run(const Vector3<S>& /*p*/, S* /*d*/)
  {
    // Do nothing
  }
};

//==============================================================================
template <typename S, std::size_t N>
FCL_EXPORT
void getDistances(const Vector3<S>& p, S* d)
{
  GetDistancesImpl<S, N>::run(p, d);
}

//==============================================================================
template <typename S>
struct FCL_EXPORT GetDistancesImpl<S, 5>
{
  static void run(const Vector3<S>& p, S* d)
  {
    d[0] = p[0] + p[1];
    d[1] = p[0] + p[2];
    d[2] = p[1] + p[2];
    d[3] = p[0] - p[1];
    d[4] = p[0] - p[2];
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT GetDistancesImpl<S, 6>
{
  static void run(const Vector3<S>& p, S* d)
  {
    d[0] = p[0] + p[1];
    d[1] = p[0] + p[2];
    d[2] = p[1] + p[2];
    d[3] = p[0] - p[1];
    d[4] = p[0] - p[2];
    d[5] = p[1] - p[2];
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT GetDistancesImpl<S, 9>
{
  static void run(const Vector3<S>& p, S* d)
  {
    d[0] = p[0] + p[1];
    d[1] = p[0] + p[2];
    d[2] = p[1] + p[2];
    d[3] = p[0] - p[1];
    d[4] = p[0] - p[2];
    d[5] = p[1] - p[2];
    d[6] = p[0] + p[1] - p[2];
    d[7] = p[0] + p[2] - p[1];
    d[8] = p[1] + p[2] - p[0];
  }
};

} // namespace fcl

#endif

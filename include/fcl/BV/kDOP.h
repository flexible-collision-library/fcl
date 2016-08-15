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

/** \author Jia Pan */

#ifndef FCL_BV_KDOP_H
#define FCL_BV_KDOP_H

#include <cstddef>
#include <iostream>

#include "fcl/data_types.h"

namespace fcl
{

/// @brief KDOP class describes the KDOP collision structures. K is set as the template parameter, which should be 16, 18, or 24
///  The KDOP structure is defined by some pairs of parallel planes defined by some axes. 
/// For K = 16, the planes are 6 AABB planes and 10 diagonal planes that cut off some space of the edges:
/// (-1,0,0) and (1,0,0)  -> indices 0 and 8
/// (0,-1,0) and (0,1,0)  -> indices 1 and 9
/// (0,0,-1) and (0,0,1)  -> indices 2 and 10
/// (-1,-1,0) and (1,1,0) -> indices 3 and 11
/// (-1,0,-1) and (1,0,1) -> indices 4 and 12
/// (0,-1,-1) and (0,1,1) -> indices 5 and 13
/// (-1,1,0) and (1,-1,0) -> indices 6 and 14
/// (-1,0,1) and (1,0,-1) -> indices 7 and 15
/// For K = 18, the planes are 6 AABB planes and 12 diagonal planes that cut off some space of the edges:
/// (-1,0,0) and (1,0,0)  -> indices 0 and 9
/// (0,-1,0) and (0,1,0)  -> indices 1 and 10
/// (0,0,-1) and (0,0,1)  -> indices 2 and 11
/// (-1,-1,0) and (1,1,0) -> indices 3 and 12
/// (-1,0,-1) and (1,0,1) -> indices 4 and 13
/// (0,-1,-1) and (0,1,1) -> indices 5 and 14
/// (-1,1,0) and (1,-1,0) -> indices 6 and 15
/// (-1,0,1) and (1,0,-1) -> indices 7 and 16
/// (0,-1,1) and (0,1,-1) -> indices 8 and 17
/// For K = 18, the planes are 6 AABB planes and 18 diagonal planes that cut off some space of the edges:
/// (-1,0,0) and (1,0,0)  -> indices 0 and 12
/// (0,-1,0) and (0,1,0)  -> indices 1 and 13
/// (0,0,-1) and (0,0,1)  -> indices 2 and 14
/// (-1,-1,0) and (1,1,0) -> indices 3 and 15
/// (-1,0,-1) and (1,0,1) -> indices 4 and 16
/// (0,-1,-1) and (0,1,1) -> indices 5 and 17
/// (-1,1,0) and (1,-1,0) -> indices 6 and 18
/// (-1,0,1) and (1,0,-1) -> indices 7 and 19
/// (0,-1,1) and (0,1,-1) -> indices 8 and 20
/// (-1, -1, 1) and (1, 1, -1) --> indices 9 and 21
/// (-1, 1, -1) and (1, -1, 1) --> indices 10 and 22
/// (1, -1, -1) and (-1, 1, 1) --> indices 11 and 23
template <typename S_, std::size_t N>
class KDOP
{
public:

  using S = S_;

  /// @brief Creating kDOP containing nothing
  KDOP();

  /// @brief Creating kDOP containing only one point
  KDOP(const Vector3<S>& v);

  /// @brief Creating kDOP containing two points
  KDOP(const Vector3<S>& a, const Vector3<S>& b);
  
  /// @brief Check whether two KDOPs are overlapped
  bool overlap(const KDOP<S, N>& other) const;

  //// @brief Check whether one point is inside the KDOP
  bool inside(const Vector3<S>& p) const;

  /// @brief Merge the point and the KDOP
  KDOP<S, N>& operator += (const Vector3<S>& p);

  /// @brief Merge two KDOPs
  KDOP<S, N>& operator += (const KDOP<S, N>& other);

  /// @brief Create a KDOP by mergin two KDOPs
  KDOP<S, N> operator + (const KDOP<S, N>& other) const;

  /// @brief The (AABB) width
  S width() const;

  /// @brief The (AABB) height
  S height() const;

  /// @brief The (AABB) depth
  S depth() const;

  /// @brief The (AABB) volume
  S volume() const;

  /// @brief Size of the kDOP (used in BV_Splitter to order two kDOPs)
  S size() const;

  /// @brief The (AABB) center
  Vector3<S> center() const;

  /// @brief The distance between two KDOP<S, N>. Not implemented.
  S distance(
      const KDOP<S, N>& other,
      Vector3<S>* P = nullptr, Vector3<S>* Q = nullptr) const;

private:
  /// @brief Origin's distances to N KDOP planes
  S dist_[N];

public:
  S dist(std::size_t i) const;

  S& dist(std::size_t i);

};

template <std::size_t N>
using KDOPf = KDOP<float, N>;
template <std::size_t N>
using KDOPd = KDOP<double, N>;

/// @brief Find the smaller and larger one of two values
template <typename S>
void minmax(S a, S b, S& minv, S& maxv);

/// @brief Merge the interval [minv, maxv] and value p/
template <typename S>
void minmax(S p, S& minv, S& maxv);

/// @brief Compute the distances to planes with normals from KDOP vectors except
/// those of AABB face planes
template <typename S, std::size_t N>
void getDistances(const Vector3<S>& p, S* d);

/// @brief translate the KDOP BV
template <typename S, std::size_t N, typename Derived>
KDOP<S, N> translate(
    const KDOP<S, N>& bv, const Eigen::MatrixBase<Derived>& t);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename S, std::size_t N>
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
KDOP<S, N> KDOP<S, N>::operator + (const KDOP<S, N>& other) const
{
  KDOP<S, N> res(*this);
  return res += other;
}

//==============================================================================
template <typename S, std::size_t N>
S KDOP<S, N>::width() const
{
  return dist_[N / 2] - dist_[0];
}

//==============================================================================
template <typename S, std::size_t N>
S KDOP<S, N>::height() const
{
  return dist_[N / 2 + 1] - dist_[1];
}

//==============================================================================
template <typename S, std::size_t N>
S KDOP<S, N>::depth() const
{
  return dist_[N / 2 + 2] - dist_[2];
}

//==============================================================================
template <typename S, std::size_t N>
S KDOP<S, N>::volume() const
{
  return width() * height() * depth();
}

//==============================================================================
template <typename S, std::size_t N>
S KDOP<S, N>::size() const
{
  return width() * width() + height() * height() + depth() * depth();
}

//==============================================================================
template <typename S, std::size_t N>
Vector3<S> KDOP<S, N>::center() const
{
  return Vector3<S>(dist_[0] + dist_[N / 2], dist_[1] + dist_[N / 2 + 1], dist_[2] + dist_[N / 2 + 2]) * 0.5;
}

//==============================================================================
template <typename S, std::size_t N>
S KDOP<S, N>::distance(const KDOP<S, N>& other, Vector3<S>* P, Vector3<S>* Q) const
{
  std::cerr << "KDOP distance not implemented!" << std::endl;
  return 0.0;
}

//==============================================================================
template <typename S, std::size_t N>
S KDOP<S, N>::dist(std::size_t i) const
{
  return dist_[i];
}

//==============================================================================
template <typename S, std::size_t N>
S& KDOP<S, N>::dist(std::size_t i)
{
  return dist_[i];
}

//==============================================================================
template <typename S, std::size_t N, typename Derived>
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
void getDistances(const Vector3<S>& p, S* d)
{
  GetDistancesImpl<S, N>::run(p, d);
}

//==============================================================================
template <typename S>
struct GetDistancesImpl<S, 5>
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
struct GetDistancesImpl<S, 6>
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
struct GetDistancesImpl<S, 9>
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

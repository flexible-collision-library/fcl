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

#ifndef FCL_BV_KIOS_H
#define FCL_BV_KIOS_H

#include "fcl/BV/OBB.h"

namespace fcl
{
 
/// @brief A class describing the kIOS collision structure, which is a set of spheres.
template <typename S_>
class kIOS
{
  /// @brief One sphere in kIOS
  struct kIOS_Sphere
  {
    Vector3<S_> o;
    S_ r;
  };

  /// @brief generate one sphere enclosing two spheres
  static kIOS_Sphere encloseSphere(
      const kIOS_Sphere& s0, const kIOS_Sphere& s1);

public:

  using S = S_;

  /// @brief The (at most) five spheres for intersection
  kIOS_Sphere spheres[5];

  /// @brief The number of spheres, no larger than 5
  unsigned int num_spheres;

  /// @brief OBB related with kIOS
  OBB<S> obb;

  /// @brief Check collision between two kIOS
  bool overlap(const kIOS<S>& other) const;

  /// @brief Check collision between two kIOS and return the overlap part.
  /// For kIOS, we return nothing, as the overlappart of two kIOS usually is not
  /// an kIOS
  /// @todo Not efficient. It first checks the sphere collisions and then use
  /// OBB for further culling.
  bool overlap(const kIOS<S>& other, kIOS<S>& overlap_part) const;

  /// @brief Check whether the kIOS contains a point
  inline bool contain(const Vector3<S>& p) const;

  /// @brief A simple way to merge the kIOS and a point
  kIOS<S>& operator += (const Vector3<S>& p);

  /// @brief Merge the kIOS and another kIOS
  kIOS<S>& operator += (const kIOS<S>& other);

  /// @brief Return the merged kIOS of current kIOS and the other one
  kIOS<S> operator + (const kIOS<S>& other) const;

  /// @brief Center of the kIOS
  const Vector3<S>& center() const;

  /// @brief Width of the kIOS
  S width() const;

  /// @brief Height of the kIOS
  S height() const;

  /// @brief Depth of the kIOS
  S depth() const;

  /// @brief Volume of the kIOS
  S volume() const;

  /// @brief size of the kIOS (used in BV_Splitter to order two kIOSs)
  S size() const;

  /// @brief The distance between two kIOS
  S distance(
      const kIOS<S>& other,
      Vector3<S>* P = nullptr, Vector3<S>* Q = nullptr) const;

  static constexpr S ratio() { return 1.5; }
  static constexpr S invSinA() { return 2; }
  static S cosA() { return std::sqrt(3.0) / 2.0; }
};

using kIOSf = kIOS<float>;
using kIOSd = kIOS<double>;

/// @brief Translate the kIOS BV
template <typename S, typename Derived>
kIOS<S> translate(
    const kIOS<S>& bv, const Eigen::MatrixBase<Derived>& t);

/// @brief Check collision between two kIOSs, b1 is in configuration (R0, T0)
/// and b2 is in identity.
/// @todo Not efficient
template <typename S, typename DerivedA, typename DerivedB>
bool overlap(
    const Eigen::MatrixBase<DerivedA>& R0,
    const Eigen::MatrixBase<DerivedB>& T0,
    const kIOS<S>& b1, const kIOS<S>& b2);

/// @brief Check collision between two kIOSs, b1 is in configuration (R0, T0)
/// and b2 is in identity.
/// @todo Not efficient
template <typename S>
bool overlap(
    const Transform3<S>& tf,
    const kIOS<S>& b1,
    const kIOS<S>& b2);

/// @brief Approximate distance between two kIOS bounding volumes
/// @todo P and Q is not returned, need implementation
template <typename S, typename DerivedA, typename DerivedB>
S distance(
    const Eigen::MatrixBase<DerivedA>& R0,
    const Eigen::MatrixBase<DerivedB>& T0,
    const kIOS<S>& b1,
    const kIOS<S>& b2,
    Vector3<S>* P = nullptr,
    Vector3<S>* Q = nullptr);

/// @brief Approximate distance between two kIOS bounding volumes
/// @todo P and Q is not returned, need implementation
template <typename S>
S distance(
    const Transform3<S>& tf,
    const kIOS<S>& b1,
    const kIOS<S>& b2,
    Vector3<S>* P = nullptr,
    Vector3<S>* Q = nullptr);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename S>
typename kIOS<S>::kIOS_Sphere kIOS<S>::encloseSphere(
    const typename kIOS<S>::kIOS_Sphere& s0, const typename kIOS<S>::kIOS_Sphere& s1)
{
  Vector3<S> d = s1.o - s0.o;
  S dist2 = d.squaredNorm();
  S diff_r = s1.r - s0.r;

  /** The sphere with the larger radius encloses the other */
  if(diff_r * diff_r >= dist2)
  {
    if(s1.r > s0.r) return s1;
    else return s0;
  }
  else /** spheres partially overlapping or disjoint */
  {
    float dist = std::sqrt(dist2);
    kIOS_Sphere s;
    s.r = dist + s0.r + s1.r;
    if(dist > 0)
      s.o = s0.o + d * ((s.r - s0.r) / dist);
    else
      s.o = s0.o;
    return s;
  }
}

//==============================================================================
template <typename S>
bool kIOS<S>::overlap(const kIOS<S>& other) const
{
  for(unsigned int i = 0; i < num_spheres; ++i)
  {
    for(unsigned int j = 0; j < other.num_spheres; ++j)
    {
      S o_dist = (spheres[i].o - other.spheres[j].o).squaredNorm();
      S sum_r = spheres[i].r + other.spheres[j].r;
      if(o_dist > sum_r * sum_r)
        return false;
    }
  }

  return obb.overlap(other.obb);

  return true;
}

//==============================================================================
template <typename S>
bool kIOS<S>::overlap(
    const kIOS<S>& other, kIOS<S>& /*overlap_part*/) const
{
  return overlap(other);
}

//==============================================================================
template <typename S>
bool kIOS<S>::contain(const Vector3<S>& p) const
{
  for(unsigned int i = 0; i < num_spheres; ++i)
  {
    S r = spheres[i].r;
    if((spheres[i].o - p).squaredNorm() > r * r)
      return false;
  }

  return true;
}

//==============================================================================
template <typename S>
kIOS<S>& kIOS<S>::operator += (const Vector3<S>& p)
{
  for(unsigned int i = 0; i < num_spheres; ++i)
  {
    S r = spheres[i].r;
    S new_r_sqr = (p - spheres[i].o).squaredNorm();
    if(new_r_sqr > r * r)
    {
      spheres[i].r = sqrt(new_r_sqr);
    }
  }

  obb += p;
  return *this;
}

//==============================================================================
template <typename S>
kIOS<S>& kIOS<S>::operator +=(const kIOS<S>& other)
{
  *this = *this + other;
  return *this;
}

//==============================================================================
template <typename S>
kIOS<S> kIOS<S>::operator + (const kIOS<S>& other) const
{
  kIOS<S> result;
  unsigned int new_num_spheres = std::min(num_spheres, other.num_spheres);
  for(unsigned int i = 0; i < new_num_spheres; ++i)
  {
    result.spheres[i] = encloseSphere(spheres[i], other.spheres[i]);
  }

  result.num_spheres = new_num_spheres;

  result.obb = obb + other.obb;

  return result;
}

//==============================================================================
template <typename S>
const Vector3<S>& kIOS<S>::center() const
{
  return spheres[0].o;
}

//==============================================================================
template <typename S>
S kIOS<S>::width() const
{
  return obb.width();
}

//==============================================================================
template <typename S>
S kIOS<S>::height() const
{
  return obb.height();
}

//==============================================================================
template <typename S>
S kIOS<S>::depth() const
{
  return obb.depth();
}

//==============================================================================
template <typename S>
S kIOS<S>::volume() const
{
  return obb.volume();
}

//==============================================================================
template <typename S>
S kIOS<S>::size() const
{
  return volume();
}

//==============================================================================
template <typename S>
S kIOS<S>::distance(
    const kIOS<S>& other,
    Vector3<S>* P,
    Vector3<S>* Q) const
{
  S d_max = 0;
  int id_a = -1, id_b = -1;
  for(unsigned int i = 0; i < num_spheres; ++i)
  {
    for(unsigned int j = 0; j < other.num_spheres; ++j)
    {
      S d = (spheres[i].o - other.spheres[j].o).norm() - (spheres[i].r + other.spheres[j].r);
      if(d_max < d)
      {
        d_max = d;
        if(P && Q)
        {
          id_a = i; id_b = j;
        }
      }
    }
  }

  if(P && Q)
  {
    if(id_a != -1 && id_b != -1)
    {
      Vector3<S> v = spheres[id_a].o - spheres[id_b].o;
      S len_v = v.norm();
      *P = spheres[id_a].o;
      (*P).noalias() -= v * (spheres[id_a].r / len_v);
      *Q = spheres[id_b].o;
      (*Q).noalias() += v * (spheres[id_b].r / len_v);
    }
  }

  return d_max;
}

//==============================================================================
template <typename S, typename DerivedA, typename DerivedB>
bool overlap(
    const Eigen::MatrixBase<DerivedA>& R0,
    const Eigen::MatrixBase<DerivedB>& T0,
    const kIOS<S>& b1, const kIOS<S>& b2)
{
  kIOS<S> b2_temp = b2;
  for(unsigned int i = 0; i < b2_temp.num_spheres; ++i)
    b2_temp.spheres[i].o = R0 * b2_temp.spheres[i].o + T0;

  b2_temp.obb.To = R0 * b2_temp.obb.To + T0;
  b2_temp.obb.axis = R0 * b2_temp.obb.axis;

  return b1.overlap(b2_temp);
}

//==============================================================================
template <typename S>
bool overlap(
    const Transform3<S>& tf,
    const kIOS<S>& b1,
    const kIOS<S>& b2)
{
  kIOS<S> b2_temp = b2;
  for(unsigned int i = 0; i < b2_temp.num_spheres; ++i)
    b2_temp.spheres[i].o = tf * b2_temp.spheres[i].o;

  b2_temp.obb.frame = tf * b2_temp.obb.frame;

  return b1.overlap(b2_temp);
}

//==============================================================================
template <typename S, typename DerivedA, typename DerivedB>
S distance(
    const Eigen::MatrixBase<DerivedA>& R0,
    const Eigen::MatrixBase<DerivedB>& T0,
    const kIOS<S>& b1, const kIOS<S>& b2,
    Vector3<S>* P, Vector3<S>* Q)
{
  kIOS<S> b2_temp = b2;
  for(unsigned int i = 0; i < b2_temp.num_spheres; ++i)
    b2_temp.spheres[i].o = R0 * b2_temp.spheres[i].o + T0;

  return b1.distance(b2_temp, P, Q);
}

//==============================================================================
template <typename S>
S distance(
    const Transform3<S>& tf,
    const kIOS<S>& b1,
    const kIOS<S>& b2,
    Vector3<S>* P,
    Vector3<S>* Q)
{
  kIOS<S> b2_temp = b2;

  for(unsigned int i = 0; i < b2_temp.num_spheres; ++i)
    b2_temp.spheres[i].o = tf * b2_temp.spheres[i].o;

  return b1.distance(b2_temp, P, Q);
}

//==============================================================================
template <typename S, typename Derived>
kIOS<S> translate(
    const kIOS<S>& bv, const Eigen::MatrixBase<Derived>& t)
{
  EIGEN_STATIC_ASSERT(
          Derived::RowsAtCompileTime == 3
          && Derived::ColsAtCompileTime == 1,
          THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

  kIOS<S> res(bv);
  for(size_t i = 0; i < res.num_spheres; ++i)
  {
    res.spheres[i].o += t;
  }

  translate(res.obb, t);
  return res;
}


} // namespace fcl

#endif

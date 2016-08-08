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

#ifndef FCL_BV_OBBRSS_H
#define FCL_BV_OBBRSS_H

#include "fcl/BV/OBB.h"
#include "fcl/BV/RSS.h"

namespace fcl
{

/// @brief Class merging the OBBd and RSS, can handle collision and distance
/// simultaneously
template <typename ScalarT>
class OBBRSS
{
public:

  using Scalar = ScalarT;

  /// @brief OBBd member, for rotation
  OBB<ScalarT> obb;

  /// @brief RSSd member, for distance
  RSS<ScalarT> rss;

  /// @brief Check collision between two OBBRSS
  bool overlap(const OBBRSS<ScalarT>& other) const;

  /// @brief Check collision between two OBBRSS and return the overlap part.
  bool overlap(const OBBRSS<ScalarT>& other, OBBRSS<ScalarT>& overlap_part) const;

  /// @brief Check whether the OBBRSS contains a point
  bool contain(const Vector3<ScalarT>& p) const;

  /// @brief Merge the OBBRSS and a point
  OBBRSS<ScalarT>& operator += (const Vector3<ScalarT>& p);

  /// @brief Merge two OBBRSS
  OBBRSS<ScalarT>& operator += (const OBBRSS<ScalarT>& other);

  /// @brief Merge two OBBRSS
  OBBRSS<ScalarT> operator + (const OBBRSS<ScalarT>& other) const;

  /// @brief Width of the OBRSS
  ScalarT width() const;

  /// @brief Height of the OBBRSS
  ScalarT height() const;

  /// @brief Depth of the OBBRSS
  ScalarT depth() const;

  /// @brief Volume of the OBBRSS
  ScalarT volume() const;

  /// @brief Size of the OBBRSS (used in BV_Splitter to order two OBBRSS)
  ScalarT size() const;

  /// @brief Center of the OBBRSS
  const Vector3<ScalarT> center() const;

  /// @brief Distance between two OBBRSS; P and Q , is not NULL, returns the nearest points
  ScalarT distance(const OBBRSS<ScalarT>& other,
                  Vector3<ScalarT>* P = NULL, Vector3<ScalarT>* Q = NULL) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

using OBBRSSf = OBBRSS<float>;
using OBBRSSd = OBBRSS<double>;

/// @brief Translate the OBBRSS bv
template <typename Scalar>
OBBRSS<Scalar> translate(const OBBRSS<Scalar>& bv, const Vector3<Scalar>& t);

/// @brief Check collision between two OBBRSS, b1 is in configuration (R0, T0)
/// and b2 is in indentity
//template <typename Scalar, typename DerivedA, typename DerivedB>
//FCL_DEPRECATED
//bool overlap(const Eigen::MatrixBase<DerivedA>& R0,
//             const Eigen::MatrixBase<DerivedB>& T0,
//             const OBBRSS<Scalar>& b1, const OBBRSS<Scalar>& b2);

/// @brief Check collision between two OBBRSS, b1 is in configuration (R0, T0)
/// and b2 is in indentity
template <typename Scalar>
bool overlap(
    const Transform3<Scalar>& tf,
    const OBBRSS<Scalar>& b1,
    const OBBRSS<Scalar>& b2);

/// @brief Computate distance between two OBBRSS, b1 is in configuation (R0, T0)
/// and b2 is in indentity; P and Q, is not NULL, returns the nearest points
//template <typename Scalar, typename DerivedA, typename DerivedB>
//FCL_DEPRECATED
//Scalar distance(
//    const Eigen::MatrixBase<DerivedA>& R0,
//    const Eigen::MatrixBase<DerivedB>& T0,
//    const OBBRSS<Scalar>& b1, const OBBRSS<Scalar>& b2,
//    Vector3<Scalar>* P = NULL, Vector3<Scalar>* Q = NULL);

/// @brief Computate distance between two OBBRSS, b1 is in configuation (R0, T0)
/// and b2 is in indentity; P and Q, is not NULL, returns the nearest points
template <typename Scalar>
Scalar distance(
    const Transform3<Scalar>& tf,
    const OBBRSS<Scalar>& b1,
    const OBBRSS<Scalar>& b2,
    Vector3<Scalar>* P = NULL,
    Vector3<Scalar>* Q = NULL);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
bool OBBRSS<Scalar>::overlap(const OBBRSS<Scalar>& other) const
{
  return obb.overlap(other.obb);
}

//==============================================================================
template <typename Scalar>
bool OBBRSS<Scalar>::overlap(const OBBRSS<Scalar>& other,
                             OBBRSS<Scalar>& /*overlap_part*/) const
{
  return overlap(other);
}

//==============================================================================
template <typename Scalar>
bool OBBRSS<Scalar>::contain(const Vector3<Scalar>& p) const
{
  return obb.contain(p);
}

//==============================================================================
template <typename Scalar>
OBBRSS<Scalar>& OBBRSS<Scalar>::operator +=(const Vector3<Scalar>& p)
{
  obb += p;
  rss += p;
  return *this;
}

//==============================================================================
template <typename Scalar>
OBBRSS<Scalar>& OBBRSS<Scalar>::operator +=(const OBBRSS<Scalar>& other)
{
  *this = *this + other;
  return *this;
}

//==============================================================================
template <typename Scalar>
OBBRSS<Scalar> OBBRSS<Scalar>::operator +(const OBBRSS<Scalar>& other) const
{
  OBBRSS<Scalar> result;
  result.obb = obb + other.obb;
  result.rss = rss + other.rss;
  return result;
}

//==============================================================================
template <typename Scalar>
Scalar OBBRSS<Scalar>::width() const
{
  return obb.width();
}

//==============================================================================
template <typename Scalar>
Scalar OBBRSS<Scalar>::height() const
{
  return obb.height();
}

//==============================================================================
template <typename Scalar>
Scalar OBBRSS<Scalar>::depth() const
{
  return obb.depth();
}

//==============================================================================
template <typename Scalar>
Scalar OBBRSS<Scalar>::volume() const
{
  return obb.volume();
}

//==============================================================================
template <typename Scalar>
Scalar OBBRSS<Scalar>::size() const
{
  return obb.size();
}

//==============================================================================
template <typename Scalar>
const Vector3<Scalar> OBBRSS<Scalar>::center() const
{
  return obb.center();
}

//==============================================================================
template <typename Scalar>
Scalar OBBRSS<Scalar>::distance(const OBBRSS<Scalar>& other,
                                Vector3<Scalar>* P, Vector3<Scalar>* Q) const
{
  return rss.distance(other.rss, P, Q);
}

//==============================================================================
//template <typename Scalar, typename DerivedA, typename DerivedB>
//bool overlap(const Eigen::MatrixBase<DerivedA>& R0,
//             const Eigen::MatrixBase<DerivedB>& T0,
//             const OBBRSS<Scalar>& b1, const OBBRSS<Scalar>& b2)
//{
//  return overlap(R0, T0, b1.obb, b2.obb);
//}

//==============================================================================
template <typename Scalar>
bool overlap(
    const Transform3<Scalar>& tf,
    const OBBRSS<Scalar>& b1,
    const OBBRSS<Scalar>& b2)
{
  return overlap(tf, b1.obb, b2.obb);
}

//==============================================================================
//template <typename Scalar, typename DerivedA, typename DerivedB>
//Scalar distance(
//    const Eigen::MatrixBase<DerivedA>& R0,
//    const Eigen::MatrixBase<DerivedB>& T0,
//    const OBBRSS<Scalar>& b1, const OBBRSS<Scalar>& b2,
//    Vector3<Scalar>* P, Vector3<Scalar>* Q)
//{
//  return distance(R0, T0, b1.rss, b2.rss, P, Q);
//}

//==============================================================================
template <typename Scalar>
Scalar distance(
    const Transform3<Scalar>& tf,
    const OBBRSS<Scalar>& b1,
    const OBBRSS<Scalar>& b2,
    Vector3<Scalar>* P,
    Vector3<Scalar>* Q)
{
  return distance(tf, b1.rss, b2.rss, P, Q);
}

//==============================================================================
template <typename Scalar>
OBBRSS<Scalar> translate(const OBBRSS<Scalar>& bv, const Vector3<Scalar>& t)
{
  OBBRSS<Scalar> res(bv);
  res.obb.frame.translation() += t;
  res.rss.frame.translation() += t;
  return res;
}

} // namespace fcl

#endif

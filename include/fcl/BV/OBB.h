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

#ifndef FCL_BV_OBB_H
#define FCL_BV_OBB_H

#include <iostream>

#include "fcl/data_types.h"
#include "fcl/math/geometry.h"

namespace fcl
{

/// @brief Oriented bounding box class
template <typename ScalarT>
class OBB
{
public:

  using Scalar = ScalarT;

  /// @brief Orientation and center of OBB. Rotation part of frame represents
  /// the orientation of the box; the axes of the rotation matrix are the
  /// principle directions of the box. We assume that the first column
  /// corresponds to the axis with the longest box edge, second column
  /// corresponds to the shorter one and the third coulumn corresponds to the
  /// shortest one.
  Transform3<ScalarT> frame;

  /// @brief Half dimensions of OBB
  Vector3<ScalarT> extent;

  /// Constructor
  OBB();

  /// @brief Check collision between two OBB, return true if collision happens. 
  bool overlap(const OBB<ScalarT>& other) const;
  
  /// @brief Check collision between two OBB and return the overlap part. For OBB, the overlap_part return value is NOT used as the overlap part of two obbs usually is not an obb. 
  bool overlap(const OBB<ScalarT>& other, OBB<ScalarT>& overlap_part) const;

  /// @brief Check whether the OBB contains a point.
  bool contain(const Vector3<ScalarT>& p) const;

  /// @brief A simple way to merge the OBB and a point (the result is not compact).
  OBB<ScalarT>& operator +=(const Vector3<ScalarT>& p);

  /// @brief Merge the OBB and another OBB (the result is not compact).
  OBB<ScalarT>& operator += (const OBB<ScalarT>& other);

  /// @brief Return the merged OBB of current OBB and the other one (the result is not compact).
  OBB<ScalarT> operator + (const OBB<ScalarT>& other) const;

  /// @brief Width of the OBB.
  ScalarT width() const;

  /// @brief Height of the OBB.
  ScalarT height() const;

  /// @brief Depth of the OBB
  ScalarT depth() const;

  /// @brief Volume of the OBB
  ScalarT volume() const;

  /// @brief Size of the OBB (used in BV_Splitter to order two OBBs)
  ScalarT size() const;

  /// @brief Center of the OBB
  const Vector3<ScalarT> center() const;

  /// @brief Distance between two OBBs, not implemented.
  ScalarT distance(const OBB& other, Vector3<ScalarT>* P = NULL,
                  Vector3<ScalarT>* Q = NULL) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

using OBBf = OBB<float>;
using OBBd = OBB<double>;

/// @brief Compute the 8 vertices of a OBBd
template <typename Scalar>
void computeVertices(const OBB<Scalar>& b, Vector3<Scalar> vertices[8]);

/// @brief OBBd merge method when the centers of two smaller OBBd are far away
template <typename Scalar>
OBB<Scalar> merge_largedist(const OBB<Scalar>& b1, const OBB<Scalar>& b2);

/// @brief OBBd merge method when the centers of two smaller OBBd are close
template <typename Scalar>
OBB<Scalar> merge_smalldist(const OBB<Scalar>& b1, const OBB<Scalar>& b2);

/// @brief Translate the OBB bv
template <typename Scalar, typename Derived>
OBB<Scalar> translate(
    const OBB<Scalar>& bv, const Eigen::MatrixBase<Derived>& t);

/// @brief Check collision between two obbs, b1 is in configuration (R0, T0) and
/// b2 is in identity.
//template <typename Scalar, typename DerivedA, typename DerivedB>
//FCL_DEPRECATED
//bool overlap(const Eigen::MatrixBase<DerivedA>& R0,
//             const Eigen::MatrixBase<DerivedB>& T0,
//             const OBB<Scalar>& b1, const OBB<Scalar>& b2);

/// @brief Check collision between two obbs, b1 is in configuration (R0, T0) and
/// b2 is in identity.
template <typename Scalar>
bool overlap(
    const Transform3<Scalar>& tf,
    const OBB<Scalar>& b1,
    const OBB<Scalar>& b2);

/// @brief Check collision between two boxes: the first box is in configuration
/// (R, T) and its half dimension is set by a; the second box is in identity
/// configuration and its half dimension is set by b.
//template <typename Scalar>
////FCL_DEPRECATED
//bool obbDisjoint(const Matrix3<Scalar>& B, const Vector3<Scalar>& T,
//                 const Vector3<Scalar>& a, const Vector3<Scalar>& b);

/// @brief Check collision between two boxes: the first box is in configuration
/// (R, T) and its half dimension is set by a; the second box is in identity
/// configuration and its half dimension is set by b.
template <typename Scalar>
bool obbDisjoint(
    const Transform3<Scalar>& tf,
    const Vector3<Scalar>& a,
    const Vector3<Scalar>& b);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
OBB<Scalar>::OBB() : frame(Transform3<Scalar>::Identity())
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
bool OBB<Scalar>::overlap(const OBB<Scalar>& other) const
{
  /// compute the relative transform that takes us from this->frame to
  /// other.frame

  return !obbDisjoint(frame.inverse(Eigen::Isometry) * other.frame, extent, other.extent);
}

//==============================================================================
template <typename Scalar>
bool OBB<Scalar>::overlap(const OBB& other, OBB& overlap_part) const
{
  return overlap(other);
}

//==============================================================================
template <typename Scalar>
bool OBB<Scalar>::contain(const Vector3<Scalar>& p) const
{
  Vector3<Scalar> local_p = p - frame.translation();
  Scalar proj = local_p.dot(frame.linear().col(0));
  if((proj > extent[0]) || (proj < -extent[0]))
    return false;

  proj = local_p.dot(frame.linear().col(1));
  if((proj > extent[1]) || (proj < -extent[1]))
    return false;

  proj = local_p.dot(frame.linear().col(2));
  if((proj > extent[2]) || (proj < -extent[2]))
    return false;

  return true;
}

//==============================================================================
template <typename Scalar>
OBB<Scalar>& OBB<Scalar>::operator +=(const Vector3<Scalar>& p)
{
  OBB<Scalar> bvp;
  bvp.frame.linear() = frame.linear();
  bvp.frame.translation() = p;
  bvp.extent.setZero();

  *this += bvp;
  return *this;
}

//==============================================================================
template <typename Scalar>
OBB<Scalar>& OBB<Scalar>::operator +=(const OBB<Scalar>& other)
{
  *this = *this + other;
  return *this;
}

//==============================================================================
template <typename Scalar>
OBB<Scalar> OBB<Scalar>::operator +(const OBB<Scalar>& other) const
{
  Vector3<Scalar> center_diff = frame.translation() - other.frame.translation();
  Scalar max_extent = std::max(std::max(extent[0], extent[1]), extent[2]);
  Scalar max_extent2 = std::max(std::max(other.extent[0], other.extent[1]), other.extent[2]);
  if(center_diff.norm() > 2 * (max_extent + max_extent2))
  {
    return merge_largedist(*this, other);
  }
  else
  {
    return merge_smalldist(*this, other);
  }
}

//==============================================================================
template <typename Scalar>
Scalar OBB<Scalar>::width() const
{
  return 2 * extent[0];
}

//==============================================================================
template <typename Scalar>
Scalar OBB<Scalar>::height() const
{
  return 2 * extent[1];
}

//==============================================================================
template <typename Scalar>
Scalar OBB<Scalar>::depth() const
{
  return 2 * extent[2];
}

//==============================================================================
template <typename Scalar>
Scalar OBB<Scalar>::volume() const
{
  return width() * height() * depth();
}

//==============================================================================
template <typename Scalar>
Scalar OBB<Scalar>::size() const
{
  return extent.squaredNorm();
}

//==============================================================================
template <typename Scalar>
const Vector3<Scalar> OBB<Scalar>::center() const
{
  return frame.translation();
}

//==============================================================================
template <typename Scalar>
Scalar OBB<Scalar>::distance(const OBB& other, Vector3<Scalar>* P,
                             Vector3<Scalar>* Q) const
{
  std::cerr << "OBB distance not implemented!" << std::endl;
  return 0.0;
}

//==============================================================================
template <typename Scalar>
void computeVertices(const OBB<Scalar>& b, Vector3<Scalar> vertices[8])
{
  const Vector3<Scalar>& extent = b.extent;
  const Vector3<Scalar>& To = b.frame.translation();

  Vector3<Scalar> extAxis0 = b.frame.linear().col(0) * extent[0];
  Vector3<Scalar> extAxis1 = b.frame.linear().col(1) * extent[1];
  Vector3<Scalar> extAxis2 = b.frame.linear().col(2) * extent[2];

  vertices[0] = To - extAxis0 - extAxis1 - extAxis2;
  vertices[1] = To + extAxis0 - extAxis1 - extAxis2;
  vertices[2] = To + extAxis0 + extAxis1 - extAxis2;
  vertices[3] = To - extAxis0 + extAxis1 - extAxis2;
  vertices[4] = To - extAxis0 - extAxis1 + extAxis2;
  vertices[5] = To + extAxis0 - extAxis1 + extAxis2;
  vertices[6] = To + extAxis0 + extAxis1 + extAxis2;
  vertices[7] = To - extAxis0 + extAxis1 + extAxis2;
}

//==============================================================================
template <typename Scalar>
OBB<Scalar> merge_largedist(const OBB<Scalar>& b1, const OBB<Scalar>& b2)
{
  OBB<Scalar> b;
  Vector3<Scalar> vertex[16];
  computeVertices(b1, vertex);
  computeVertices(b2, vertex + 8);
  Matrix3<Scalar> M;
  Matrix3<Scalar> E;
  Vector3<Scalar> s(0, 0, 0);

  b.frame.linear().col(0) = b1.frame.translation() - b2.frame.translation();
  b.frame.linear().col(0).normalize();

  Vector3<Scalar> vertex_proj[16];
  for(int i = 0; i < 16; ++i)
    vertex_proj[i] = vertex[i] - b.frame.linear().col(0) * vertex[i].dot(b.frame.linear().col(0));

  getCovariance<Scalar>(vertex_proj, NULL, NULL, NULL, 16, M);
  eigen_old(M, s, E);

  int min, mid, max;
  if (s[0] > s[1])
  {
    max = 0;
    min = 1;
  }
  else
  {
    min = 0;
    max = 1;
  }

  if (s[2] < s[min])
  {
    mid = min;
    min = 2;
  }
  else if (s[2] > s[max])
  {
    mid = max;
    max = 2;
  }
  else
  {
    mid = 2;
  }

  b.frame.linear().col(1) << E.col(0)[max], E.col(1)[max], E.col(2)[max];
  b.frame.linear().col(2) << E.col(0)[mid], E.col(1)[mid], E.col(2)[mid];

  // set obb centers and extensions
  Vector3<Scalar> center, extent;
  getExtentAndCenter<Scalar>(vertex, NULL, NULL, NULL, 16, b.frame.linear(), center, extent);

  b.frame.translation() = center;
  b.extent = extent;

  return b;
}

//==============================================================================
template <typename Scalar>
OBB<Scalar> merge_smalldist(const OBB<Scalar>& b1, const OBB<Scalar>& b2)
{
  OBB<Scalar> b;
  b.frame.translation() = (b1.frame.translation() + b2.frame.translation()) * 0.5;
  Quaternion<Scalar> q0(b1.frame.linear());
  Quaternion<Scalar> q1(b2.frame.linear());
  if(q0.dot(q1) < 0)
    q1.coeffs() = -q1.coeffs();

  Quaternion<Scalar> q(q0.coeffs() + q1.coeffs());
  q.normalize();
  b.frame.linear() = q.toRotationMatrix();


  Vector3<Scalar> vertex[8], diff;
  Scalar real_max = std::numeric_limits<Scalar>::max();
  Vector3<Scalar> pmin(real_max, real_max, real_max);
  Vector3<Scalar> pmax(-real_max, -real_max, -real_max);

  computeVertices(b1, vertex);
  for(int i = 0; i < 8; ++i)
  {
    diff = vertex[i] - b.frame.translation();
    for(int j = 0; j < 3; ++j)
    {
      Scalar dot = diff.dot(b.frame.linear().col(j));
      if(dot > pmax[j])
        pmax[j] = dot;
      else if(dot < pmin[j])
        pmin[j] = dot;
    }
  }

  computeVertices(b2, vertex);
  for(int i = 0; i < 8; ++i)
  {
    diff = vertex[i] - b.frame.translation();
    for(int j = 0; j < 3; ++j)
    {
      Scalar dot = diff.dot(b.frame.linear().col(j));
      if(dot > pmax[j])
        pmax[j] = dot;
      else if(dot < pmin[j])
        pmin[j] = dot;
    }
  }

  for(int j = 0; j < 3; ++j)
  {
    b.frame.translation() += (b.frame.linear().col(j) * (0.5 * (pmax[j] + pmin[j])));
    b.extent[j] = 0.5 * (pmax[j] - pmin[j]);
  }

  return b;
}

//==============================================================================
template <typename Scalar, typename Derived>
OBB<Scalar> translate(
    const OBB<Scalar>& bv, const Eigen::MatrixBase<Derived>& t)
{
  OBB<Scalar> res(bv);
  res.frame.translation() += t;
  return res;
}

////==============================================================================
//template <typename Scalar, typename DerivedA, typename DerivedB>
//bool overlap(const Eigen::MatrixBase<DerivedA>& R0,
//             const Eigen::MatrixBase<DerivedB>& T0,
//             const OBB<Scalar>& b1, const OBB<Scalar>& b2)
//{
//  typename DerivedA::PlainObject R0b2 = R0 * b2.frame.linear();
//  typename DerivedA::PlainObject R = b1.frame.linear().transpose() * R0b2;

//  typename DerivedB::PlainObject Ttemp = R0 * b2.frame.translation() + T0 - b1.frame.translation();
//  typename DerivedB::PlainObject T = Ttemp.transpose() * b1.frame.linear();

//  return !obbDisjoint(R, T, b1.extent, b2.extent);
//}

//==============================================================================
template <typename Scalar>
bool overlap(
    const Transform3<Scalar>& tf,
    const OBB<Scalar>& b1,
    const OBB<Scalar>& b2)
{
  return !obbDisjoint(
        b1.frame.inverse(Eigen::Isometry) * tf * b2.frame, b1.extent, b2.extent);
}

//==============================================================================
//template <typename Scalar>
//bool obbDisjoint(const Matrix3<Scalar>& B, const Vector3<Scalar>& T,
//                 const Vector3<Scalar>& a, const Vector3<Scalar>& b)
//{
//  register Scalar t, s;
//  const Scalar reps = 1e-6;

//  Matrix3<Scalar> Bf = B.cwiseAbs();
//  Bf.array() += reps;

//  // if any of these tests are one-sided, then the polyhedra are disjoint

//  // A1 x A2 = A0
//  t = ((T[0] < 0.0) ? -T[0] : T[0]);

//  if(t > (a[0] + Bf.row(0).dot(b)))
//    return true;

//  // B1 x B2 = B0
//  s =  B.col(0).dot(T);
//  t = ((s < 0.0) ? -s : s);

//  if(t > (b[0] + Bf.col(0).dot(a)))
//    return true;

//  // A2 x A0 = A1
//  t = ((T[1] < 0.0) ? -T[1] : T[1]);

//  if(t > (a[1] + Bf.row(1).dot(b)))
//    return true;

//  // A0 x A1 = A2
//  t =((T[2] < 0.0) ? -T[2] : T[2]);

//  if(t > (a[2] + Bf.row(2).dot(b)))
//    return true;

//  // B2 x B0 = B1
//  s = B.col(1).dot(T);
//  t = ((s < 0.0) ? -s : s);

//  if(t > (b[1] + Bf.col(1).dot(a)))
//    return true;

//  // B0 x B1 = B2
//  s = B.col(2).dot(T);
//  t = ((s < 0.0) ? -s : s);

//  if(t > (b[2] + Bf.col(2).dot(a)))
//    return true;

//  // A0 x B0
//  s = T[2] * B(1, 0) - T[1] * B(2, 0);
//  t = ((s < 0.0) ? -s : s);

//  if(t > (a[1] * Bf(2, 0) + a[2] * Bf(1, 0) +
//          b[1] * Bf(0, 2) + b[2] * Bf(0, 1)))
//    return true;

//  // A0 x B1
//  s = T[2] * B(1, 1) - T[1] * B(2, 1);
//  t = ((s < 0.0) ? -s : s);

//  if(t > (a[1] * Bf(2, 1) + a[2] * Bf(1, 1) +
//          b[0] * Bf(0, 2) + b[2] * Bf(0, 0)))
//    return true;

//  // A0 x B2
//  s = T[2] * B(1, 2) - T[1] * B(2, 2);
//  t = ((s < 0.0) ? -s : s);

//  if(t > (a[1] * Bf(2, 2) + a[2] * Bf(1, 2) +
//          b[0] * Bf(0, 1) + b[1] * Bf(0, 0)))
//    return true;

//  // A1 x B0
//  s = T[0] * B(2, 0) - T[2] * B(0, 0);
//  t = ((s < 0.0) ? -s : s);

//  if(t > (a[0] * Bf(2, 0) + a[2] * Bf(0, 0) +
//          b[1] * Bf(1, 2) + b[2] * Bf(1, 1)))
//    return true;

//  // A1 x B1
//  s = T[0] * B(2, 1) - T[2] * B(0, 1);
//  t = ((s < 0.0) ? -s : s);

//  if(t > (a[0] * Bf(2, 1) + a[2] * Bf(0, 1) +
//          b[0] * Bf(1, 2) + b[2] * Bf(1, 0)))
//    return true;

//  // A1 x B2
//  s = T[0] * B(2, 2) - T[2] * B(0, 2);
//  t = ((s < 0.0) ? -s : s);

//  if(t > (a[0] * Bf(2, 2) + a[2] * Bf(0, 2) +
//          b[0] * Bf(1, 1) + b[1] * Bf(1, 0)))
//    return true;

//  // A2 x B0
//  s = T[1] * B(0, 0) - T[0] * B(1, 0);
//  t = ((s < 0.0) ? -s : s);

//  if(t > (a[0] * Bf(1, 0) + a[1] * Bf(0, 0) +
//          b[1] * Bf(2, 2) + b[2] * Bf(2, 1)))
//    return true;

//  // A2 x B1
//  s = T[1] * B(0, 1) - T[0] * B(1, 1);
//  t = ((s < 0.0) ? -s : s);

//  if(t > (a[0] * Bf(1, 1) + a[1] * Bf(0, 1) +
//          b[0] * Bf(2, 2) + b[2] * Bf(2, 0)))
//    return true;

//  // A2 x B2
//  s = T[1] * B(0, 2) - T[0] * B(1, 2);
//  t = ((s < 0.0) ? -s : s);

//  if(t > (a[0] * Bf(1, 2) + a[1] * Bf(0, 2) +
//          b[0] * Bf(2, 1) + b[1] * Bf(2, 0)))
//    return true;

//  return false;

//}

//==============================================================================
template <typename Scalar>
bool obbDisjoint(
    const Transform3<Scalar>& tf,
    const Vector3<Scalar>& a,
    const Vector3<Scalar>& b)
{
  register Scalar t, s;
  const Scalar reps = 1e-6;

  Matrix3<Scalar> Bf = tf.linear().cwiseAbs();
  Bf.array() += reps;

  // if any of these tests are one-sided, then the polyhedra are disjoint

  // A1 x A2 = A0
  t = ((tf.translation()[0] < 0.0) ? -tf.translation()[0] : tf.translation()[0]);

  if(t > (a[0] + Bf.row(0).dot(b)))
    return true;

  // B1 x B2 = B0
  s =  tf.linear().col(0).dot(tf.translation());
  t = ((s < 0.0) ? -s : s);

  if(t > (b[0] + Bf.col(0).dot(a)))
    return true;

  // A2 x A0 = A1
  t = ((tf.translation()[1] < 0.0) ? -tf.translation()[1] : tf.translation()[1]);

  if(t > (a[1] + Bf.row(1).dot(b)))
    return true;

  // A0 x A1 = A2
  t =((tf.translation()[2] < 0.0) ? -tf.translation()[2] : tf.translation()[2]);

  if(t > (a[2] + Bf.row(2).dot(b)))
    return true;

  // B2 x B0 = B1
  s = tf.linear().col(1).dot(tf.translation());
  t = ((s < 0.0) ? -s : s);

  if(t > (b[1] + Bf.col(1).dot(a)))
    return true;

  // B0 x B1 = B2
  s = tf.linear().col(2).dot(tf.translation());
  t = ((s < 0.0) ? -s : s);

  if(t > (b[2] + Bf.col(2).dot(a)))
    return true;

  // A0 x B0
  s = tf.translation()[2] * tf.linear()(1, 0) - tf.translation()[1] * tf.linear()(2, 0);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[1] * Bf(2, 0) + a[2] * Bf(1, 0) +
          b[1] * Bf(0, 2) + b[2] * Bf(0, 1)))
    return true;

  // A0 x B1
  s = tf.translation()[2] * tf.linear()(1, 1) - tf.translation()[1] * tf.linear()(2, 1);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[1] * Bf(2, 1) + a[2] * Bf(1, 1) +
          b[0] * Bf(0, 2) + b[2] * Bf(0, 0)))
    return true;

  // A0 x B2
  s = tf.translation()[2] * tf.linear()(1, 2) - tf.translation()[1] * tf.linear()(2, 2);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[1] * Bf(2, 2) + a[2] * Bf(1, 2) +
          b[0] * Bf(0, 1) + b[1] * Bf(0, 0)))
    return true;

  // A1 x B0
  s = tf.translation()[0] * tf.linear()(2, 0) - tf.translation()[2] * tf.linear()(0, 0);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(2, 0) + a[2] * Bf(0, 0) +
          b[1] * Bf(1, 2) + b[2] * Bf(1, 1)))
    return true;

  // A1 x B1
  s = tf.translation()[0] * tf.linear()(2, 1) - tf.translation()[2] * tf.linear()(0, 1);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(2, 1) + a[2] * Bf(0, 1) +
          b[0] * Bf(1, 2) + b[2] * Bf(1, 0)))
    return true;

  // A1 x B2
  s = tf.translation()[0] * tf.linear()(2, 2) - tf.translation()[2] * tf.linear()(0, 2);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(2, 2) + a[2] * Bf(0, 2) +
          b[0] * Bf(1, 1) + b[1] * Bf(1, 0)))
    return true;

  // A2 x B0
  s = tf.translation()[1] * tf.linear()(0, 0) - tf.translation()[0] * tf.linear()(1, 0);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(1, 0) + a[1] * Bf(0, 0) +
          b[1] * Bf(2, 2) + b[2] * Bf(2, 1)))
    return true;

  // A2 x B1
  s = tf.translation()[1] * tf.linear()(0, 1) - tf.translation()[0] * tf.linear()(1, 1);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(1, 1) + a[1] * Bf(0, 1) +
          b[0] * Bf(2, 2) + b[2] * Bf(2, 0)))
    return true;

  // A2 x B2
  s = tf.translation()[1] * tf.linear()(0, 2) - tf.translation()[0] * tf.linear()(1, 2);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(1, 2) + a[1] * Bf(0, 2) +
          b[0] * Bf(2, 1) + b[1] * Bf(2, 0)))
    return true;

  return false;

}

} // namespace fcl

#endif

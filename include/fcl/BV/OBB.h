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

#ifndef FCL_OBB_H
#define FCL_OBB_H

#include <iostream>

#include "fcl/data_types.h"
#include "fcl/math/geometry.h"
#include "fcl/BV/bv_utility.h"

namespace fcl
{

/// @brief Oriented bounding box class
template <typename Scalar>
class OBB
{
public:
  /// @brief Orientation of OBB. axis[i] is the ith column of the orientation matrix for the box; it is also the i-th principle direction of the box. 
  /// We assume that axis[0] corresponds to the axis with the longest box edge, axis[1] corresponds to the shorter one and axis[2] corresponds to the shortest one.
  Matrix3<Scalar> axis;

  /// @brief Center of OBB
  Vector3<Scalar> To;
  
  /// @brief Half dimensions of OBB
  Vector3<Scalar> extent;

  /// @brief Check collision between two OBB, return true if collision happens. 
  bool overlap(const OBB<Scalar>& other) const;
  
  /// @brief Check collision between two OBB and return the overlap part. For OBB, the overlap_part return value is NOT used as the overlap part of two obbs usually is not an obb. 
  bool overlap(const OBB<Scalar>& other, OBB<Scalar>& overlap_part) const;

  /// @brief Check whether the OBB contains a point.
  bool contain(const Vector3<Scalar>& p) const;

  /// @brief A simple way to merge the OBB and a point (the result is not compact).
  OBB<Scalar>& operator +=(const Vector3<Scalar>& p);

  /// @brief Merge the OBB and another OBB (the result is not compact).
  OBB<Scalar>& operator += (const OBB<Scalar>& other);

  /// @brief Return the merged OBB of current OBB and the other one (the result is not compact).
  OBB<Scalar> operator + (const OBB<Scalar>& other) const;

  /// @brief Width of the OBB.
  Scalar width() const;

  /// @brief Height of the OBB.
  Scalar height() const;

  /// @brief Depth of the OBB
  Scalar depth() const;

  /// @brief Volume of the OBB
  Scalar volume() const;

  /// @brief Size of the OBB (used in BV_Splitter to order two OBBs)
  Scalar size() const;

  /// @brief Center of the OBB
  const Vector3<Scalar>& center() const;

  /// @brief Distance between two OBBs, not implemented.
  Scalar distance(const OBB& other, Vector3<Scalar>* P = NULL,
                  Vector3<Scalar>* Q = NULL) const;
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
template <typename Scalar>
OBB<Scalar> translate(const OBB<Scalar>& bv, const Vector3<Scalar>& t);

/// @brief Check collision between two obbs, b1 is in configuration (R0, T0) and
/// b2 is in identity.
template <typename Scalar, typename DerivedA, typename DerivedB>
bool overlap(const Eigen::MatrixBase<DerivedA>& R0,
             const Eigen::MatrixBase<DerivedB>& T0,
             const OBB<Scalar>& b1, const OBB<Scalar>& b2);

/// @brief Check collision between two boxes: the first box is in configuration
/// (R, T) and its half dimension is set by a; the second box is in identity
/// configuration and its half dimension is set by b.
template <typename Scalar>
bool obbDisjoint(const Matrix3<Scalar>& B, const Vector3<Scalar>& T,
                 const Vector3<Scalar>& a, const Vector3<Scalar>& b);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
bool OBB<Scalar>::overlap(const OBB<Scalar>& other) const
{
  /// compute what transform [R,T] that takes us from cs1 to cs2.
  /// [R,T] = [R1,T1]'[R2,T2] = [R1',-R1'T][R2,T2] = [R1'R2, R1'(T2-T1)]
  /// First compute the rotation part, then translation part
  Vector3<Scalar> t = other.To - To; // T2 - T1
  Vector3<Scalar> T = t.transpose()*axis; // R1'(T2-T1)
  Matrix3<Scalar> R = axis.transpose() * other.axis;

  return !obbDisjoint(R, T, extent, other.extent);
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
  Vector3<Scalar> local_p = p - To;
  Scalar proj = local_p.dot(axis.col(0));
  if((proj > extent[0]) || (proj < -extent[0]))
    return false;

  proj = local_p.dot(axis.col(1));
  if((proj > extent[1]) || (proj < -extent[1]))
    return false;

  proj = local_p.dot(axis.col(2));
  if((proj > extent[2]) || (proj < -extent[2]))
    return false;

  return true;
}

//==============================================================================
template <typename Scalar>
OBB<Scalar>& OBB<Scalar>::operator +=(const Vector3<Scalar>& p)
{
  OBB<Scalar> bvp;
  bvp.To = p;
  bvp.axis = axis;
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
  Vector3<Scalar> center_diff = To - other.To;
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
const Vector3<Scalar>& OBB<Scalar>::center() const
{
  return To;
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
  const Vector3<Scalar>& To = b.To;

  Vector3<Scalar> extAxis0 = b.axis.col(0) * extent[0];
  Vector3<Scalar> extAxis1 = b.axis.col(1) * extent[1];
  Vector3<Scalar> extAxis2 = b.axis.col(2) * extent[2];

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

  b.axis.col(0) = b1.To - b2.To;
  b.axis.col(0).normalize();

  Vector3<Scalar> vertex_proj[16];
  for(int i = 0; i < 16; ++i)
    vertex_proj[i] = vertex[i] - b.axis.col(0) * vertex[i].dot(b.axis.col(0));

  getCovariance<double>(vertex_proj, NULL, NULL, NULL, 16, M);
  eigen(M, s, E);

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

  b.axis.col(1) << E.col(0)[max], E.col(1)[max], E.col(2)[max];
  b.axis.col(2) << E.col(0)[mid], E.col(1)[mid], E.col(2)[mid];

  // set obb centers and extensions
  Vector3<Scalar> center, extent;
  getExtentAndCenter<double>(vertex, NULL, NULL, NULL, 16, b.axis, center, extent);

  b.To = center;
  b.extent = extent;

  return b;
}

//==============================================================================
template <typename Scalar>
OBB<Scalar> merge_smalldist(const OBB<Scalar>& b1, const OBB<Scalar>& b2)
{
  OBB<Scalar> b;
  b.To = (b1.To + b2.To) * 0.5;
  Quaternion3<Scalar> q0(b1.axis);
  Quaternion3<Scalar> q1(b2.axis);
  if(q0.dot(q1) < 0)
    q1.coeffs() = -q1.coeffs();

  Quaternion3<Scalar> q(q0.coeffs() + q1.coeffs());
  q.normalize();
  b.axis = q.toRotationMatrix();


  Vector3<Scalar> vertex[8], diff;
  Scalar real_max = std::numeric_limits<Scalar>::max();
  Vector3<Scalar> pmin(real_max, real_max, real_max);
  Vector3<Scalar> pmax(-real_max, -real_max, -real_max);

  computeVertices(b1, vertex);
  for(int i = 0; i < 8; ++i)
  {
    diff = vertex[i] - b.To;
    for(int j = 0; j < 3; ++j)
    {
      Scalar dot = diff.dot(b.axis.col(j));
      if(dot > pmax[j])
        pmax[j] = dot;
      else if(dot < pmin[j])
        pmin[j] = dot;
    }
  }

  computeVertices(b2, vertex);
  for(int i = 0; i < 8; ++i)
  {
    diff = vertex[i] - b.To;
    for(int j = 0; j < 3; ++j)
    {
      Scalar dot = diff.dot(b.axis.col(j));
      if(dot > pmax[j])
        pmax[j] = dot;
      else if(dot < pmin[j])
        pmin[j] = dot;
    }
  }

  for(int j = 0; j < 3; ++j)
  {
    b.To += (b.axis.col(j) * (0.5 * (pmax[j] + pmin[j])));
    b.extent[j] = 0.5 * (pmax[j] - pmin[j]);
  }

  return b;
}

//==============================================================================
template <typename Scalar>
OBB<Scalar> translate(const OBB<Scalar>& bv, const Vector3<Scalar>& t)
{
  OBB<Scalar> res(bv);
  res.To += t;
  return res;
}

//==============================================================================
template <typename Scalar, typename DerivedA, typename DerivedB>
bool overlap(const Eigen::MatrixBase<DerivedA>& R0,
             const Eigen::MatrixBase<DerivedB>& T0,
             const OBB<Scalar>& b1, const OBB<Scalar>& b2)
{
  typename DerivedA::PlainObject R0b2 = R0 * b2.axis;
  typename DerivedA::PlainObject R = b1.axis.transpose() * R0b2;

  typename DerivedB::PlainObject Ttemp = R0 * b2.To + T0 - b1.To;
  typename DerivedB::PlainObject T = Ttemp.transpose() * b1.axis;

  return !obbDisjoint(R, T, b1.extent, b2.extent);
}

//==============================================================================
template <typename Scalar>
bool obbDisjoint(const Matrix3<Scalar>& B, const Vector3<Scalar>& T,
                 const Vector3<Scalar>& a, const Vector3<Scalar>& b)
{
  register Scalar t, s;
  const Scalar reps = 1e-6;

  Matrix3<Scalar> Bf = B.cwiseAbs();
  Bf.array() += reps;

  // if any of these tests are one-sided, then the polyhedra are disjoint

  // A1 x A2 = A0
  t = ((T[0] < 0.0) ? -T[0] : T[0]);

  if(t > (a[0] + Bf.row(0).dot(b)))
    return true;

  // B1 x B2 = B0
  s =  B.col(0).dot(T);
  t = ((s < 0.0) ? -s : s);

  if(t > (b[0] + Bf.col(0).dot(a)))
    return true;

  // A2 x A0 = A1
  t = ((T[1] < 0.0) ? -T[1] : T[1]);

  if(t > (a[1] + Bf.row(1).dot(b)))
    return true;

  // A0 x A1 = A2
  t =((T[2] < 0.0) ? -T[2] : T[2]);

  if(t > (a[2] + Bf.row(2).dot(b)))
    return true;

  // B2 x B0 = B1
  s = B.col(1).dot(T);
  t = ((s < 0.0) ? -s : s);

  if(t > (b[1] + Bf.col(1).dot(a)))
    return true;

  // B0 x B1 = B2
  s = B.col(2).dot(T);
  t = ((s < 0.0) ? -s : s);

  if(t > (b[2] + Bf.col(2).dot(a)))
    return true;

  // A0 x B0
  s = T[2] * B(1, 0) - T[1] * B(2, 0);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[1] * Bf(2, 0) + a[2] * Bf(1, 0) +
          b[1] * Bf(0, 2) + b[2] * Bf(0, 1)))
    return true;

  // A0 x B1
  s = T[2] * B(1, 1) - T[1] * B(2, 1);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[1] * Bf(2, 1) + a[2] * Bf(1, 1) +
          b[0] * Bf(0, 2) + b[2] * Bf(0, 0)))
    return true;

  // A0 x B2
  s = T[2] * B(1, 2) - T[1] * B(2, 2);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[1] * Bf(2, 2) + a[2] * Bf(1, 2) +
          b[0] * Bf(0, 1) + b[1] * Bf(0, 0)))
    return true;

  // A1 x B0
  s = T[0] * B(2, 0) - T[2] * B(0, 0);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(2, 0) + a[2] * Bf(0, 0) +
          b[1] * Bf(1, 2) + b[2] * Bf(1, 1)))
    return true;

  // A1 x B1
  s = T[0] * B(2, 1) - T[2] * B(0, 1);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(2, 1) + a[2] * Bf(0, 1) +
          b[0] * Bf(1, 2) + b[2] * Bf(1, 0)))
    return true;

  // A1 x B2
  s = T[0] * B(2, 2) - T[2] * B(0, 2);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(2, 2) + a[2] * Bf(0, 2) +
          b[0] * Bf(1, 1) + b[1] * Bf(1, 0)))
    return true;

  // A2 x B0
  s = T[1] * B(0, 0) - T[0] * B(1, 0);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(1, 0) + a[1] * Bf(0, 0) +
          b[1] * Bf(2, 2) + b[2] * Bf(2, 1)))
    return true;

  // A2 x B1
  s = T[1] * B(0, 1) - T[0] * B(1, 1);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(1, 1) + a[1] * Bf(0, 1) +
          b[0] * Bf(2, 2) + b[2] * Bf(2, 0)))
    return true;

  // A2 x B2
  s = T[1] * B(0, 2) - T[0] * B(1, 2);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(1, 2) + a[1] * Bf(0, 2) +
          b[0] * Bf(2, 1) + b[1] * Bf(2, 0)))
    return true;

  return false;

}

} // namespace fcl

#endif

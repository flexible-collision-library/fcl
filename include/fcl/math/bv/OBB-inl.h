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

#ifndef FCL_BV_OBB_INL_H
#define FCL_BV_OBB_INL_H

#include "fcl/math/bv/OBB.h"

#include "fcl/common/unused.h"

#include "fcl/math/math_simd_details.h"

namespace fcl
{

#if defined (FCL_SSE_ENABLED) or defined(FCL_AVX_ENABLED)
  using namespace fcl::details;
#endif

//==============================================================================
extern template
class FCL_EXPORT OBB<double>;

//==============================================================================
extern template
void computeVertices(const OBB<double>& b, Vector3<double> vertices[8]);

//==============================================================================
extern template
OBB<double> merge_largedist(const OBB<double>& b1, const OBB<double>& b2);

//==============================================================================
extern template
OBB<double> merge_smalldist(const OBB<double>& b1, const OBB<double>& b2);

//==============================================================================
extern template
bool obbDisjoint(
    const Matrix3<double>& B,
    const Vector3<double>& T,
    const Vector3<double>& a,
    const Vector3<double>& b);

//==============================================================================
extern template
bool obbDisjoint(
    const Transform3<double>& tf,
    const Vector3<double>& a,
    const Vector3<double>& b);

//==============================================================================
template <typename S>
OBB<S>::OBB()
{
  // Do nothing
}

//==============================================================================
template <typename S>
OBB<S>::OBB(const Matrix3<S>& axis_,
                 const Vector3<S>& center_,
                 const Vector3<S>& extent_)
  : axis(axis_), To(center_), extent(extent_)
{
  // Do nothing
}

//==============================================================================
template <typename S>
bool OBB<S>::overlap(const OBB<S>& other) const
{
  /// compute the relative transform that takes us from this->frame to
  /// other.frame

  Vector3<S> t = other.To - To;
  Vector3<S> T(
        axis.col(0).dot(t), axis.col(1).dot(t), axis.col(2).dot(t));
  Matrix3<S> R = axis.transpose() * other.axis;

  return !obbDisjoint(R, T, extent, other.extent);
}

//==============================================================================
template <typename S>
bool OBB<S>::overlap(const OBB& other, OBB& overlap_part) const
{
  FCL_UNUSED(overlap_part);

  return overlap(other);
}

//==============================================================================
template <typename S>
bool OBB<S>::contain(const Vector3<S>& p) const
{
  Vector3<S> local_p = p - To;
  S proj = local_p.dot(axis.col(0));
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
template <typename S>
OBB<S>& OBB<S>::operator +=(const Vector3<S>& p)
{
  OBB<S> bvp(axis, p, Vector3<S>::Zero());
  *this += bvp;

  return *this;
}

//==============================================================================
template <typename S>
OBB<S>& OBB<S>::operator +=(const OBB<S>& other)
{
  *this = *this + other;

  return *this;
}

//==============================================================================
template <typename S>
OBB<S> OBB<S>::operator +(const OBB<S>& other) const
{
  Vector3<S> center_diff = To - other.To;
  S max_extent = std::max(std::max(extent[0], extent[1]), extent[2]);
  S max_extent2 = std::max(std::max(other.extent[0], other.extent[1]), other.extent[2]);
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
template <typename S>
S OBB<S>::width() const
{
  return 2 * extent[0];
}

//==============================================================================
template <typename S>
S OBB<S>::height() const
{
  return 2 * extent[1];
}

//==============================================================================
template <typename S>
S OBB<S>::depth() const
{
  return 2 * extent[2];
}

//==============================================================================
template <typename S>
S OBB<S>::volume() const
{
  return width() * height() * depth();
}

//==============================================================================
template <typename S>
S OBB<S>::size() const
{
  return extent.squaredNorm();
}

//==============================================================================
template <typename S>
const Vector3<S> OBB<S>::center() const
{
  return To;
}

//==============================================================================
template <typename S>
S OBB<S>::distance(const OBB& other, Vector3<S>* P,
                             Vector3<S>* Q) const
{
  FCL_UNUSED(other);
  FCL_UNUSED(P);
  FCL_UNUSED(Q);

  std::cerr << "OBB distance not implemented!" << std::endl;
  return 0.0;
}

//==============================================================================
template <typename S>
void computeVertices(const OBB<S>& b, Vector3<S> vertices[8])
{
  const Vector3<S>& extent = b.extent;
  const Vector3<S>& To = b.To;

  Vector3<S> extAxis0 = b.axis.col(0) * extent[0];
  Vector3<S> extAxis1 = b.axis.col(1) * extent[1];
  Vector3<S> extAxis2 = b.axis.col(2) * extent[2];

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
template <typename S>
OBB<S> merge_largedist(const OBB<S>& b1, const OBB<S>& b2)
{
  Vector3<S> vertex[16];
  computeVertices(b1, vertex);
  computeVertices(b2, vertex + 8);
  Matrix3<S> M;
  Matrix3<S> E;
  Vector3<S> s(0, 0, 0);

  OBB<S> b;
  b.axis.col(0) = b1.To - b2.To;
  b.axis.col(0).normalize();

  Vector3<S> vertex_proj[16];
  for(int i = 0; i < 16; ++i)
  {
    vertex_proj[i] = vertex[i];
    vertex_proj[i].noalias() -= b.axis.col(0) * vertex[i].dot(b.axis.col(0));
  }

  getCovariance<S>(vertex_proj, nullptr, nullptr, nullptr, 16, M);
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

  b.axis.col(1) << E.col(0)[max], E.col(1)[max], E.col(2)[max];
  b.axis.col(2) << E.col(0)[mid], E.col(1)[mid], E.col(2)[mid];

  // set obb centers and extensions
  getExtentAndCenter<S>(
        vertex, nullptr, nullptr, nullptr, 16, b.axis, b.To, b.extent);

  return b;
}

//==============================================================================
template <typename S>
OBB<S> merge_smalldist(const OBB<S>& b1, const OBB<S>& b2)
{
  OBB<S> b;
  b.To = (b1.To + b2.To) * 0.5;
  Quaternion<S> q0(b1.axis);
  Quaternion<S> q1(b2.axis);
  if(q0.dot(q1) < 0)
    q1.coeffs() = -q1.coeffs();

  Quaternion<S> q(q0.coeffs() + q1.coeffs());
  q.normalize();
  b.axis = q.toRotationMatrix();


  Vector3<S> vertex[8], diff;
  S real_max = std::numeric_limits<S>::max();
  Vector3<S> pmin(real_max, real_max, real_max);
  Vector3<S> pmax(-real_max, -real_max, -real_max);

  computeVertices(b1, vertex);
  for(int i = 0; i < 8; ++i)
  {
    diff = vertex[i] - b.To;
    for(int j = 0; j < 3; ++j)
    {
      S dot = diff.dot(b.axis.col(j));
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
      S dot = diff.dot(b.axis.col(j));
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
template <typename S, typename Derived>
OBB<S> translate(
    const OBB<S>& bv, const Eigen::MatrixBase<Derived>& t)
{
  OBB<S> res(bv);
  res.To += t;
  return res;
}

//==============================================================================
template <typename S, typename DerivedA, typename DerivedB>
bool overlap(const Eigen::MatrixBase<DerivedA>& R0,
             const Eigen::MatrixBase<DerivedB>& T0,
             const OBB<S>& b1, const OBB<S>& b2)
{
  typename DerivedA::PlainObject R0b2 = R0 * b2.axis;
  typename DerivedA::PlainObject R = b1.axis.transpose() * R0b2;

  typename DerivedB::PlainObject Ttemp = R0 * b2.To + T0 - b1.To;
  typename DerivedB::PlainObject T = Ttemp.transpose() * b1.axis;

  return !obbDisjoint(R, T, b1.extent, b2.extent);
}

//==============================================================================
#ifdef FCL_AVX_ENABLED
inline bool obbDisjoint(const __m256d* R, const __m256d& t, const __m256d& r1, const __m256d& r2)
{
  const double reps = 1e-6;
  const __m256d epsilonxyz = _mm256_setr_pd(reps, reps, reps, 0);
  __m256d AbsR[3];
  AbsR[0] = _mm256_add_pd(abs_pd(R[0]), epsilonxyz);
  AbsR[1] = _mm256_add_pd(abs_pd(R[1]), epsilonxyz);
  AbsR[2] = _mm256_add_pd(abs_pd(R[2]), epsilonxyz);

  __m256d ra;          // projection of OBB A's halfwidth along three axes
  __m256d rb;          // projection of OBB B's halfwidth along three axes
  __m256d center_dist; // projection of center distance along three axes

  // Test the three major axes of this OBB.
  if (any_gt_pd(abs_pd(t), _mm256_add_pd(r1, mat3x4_mul_vec4(AbsR, r2)))) {
    return true;
  }

  // Test the three major axes of the OBB b.
  center_dist = transp_mat3x4_mul_vec4(R, t);
  if (any_gt_pd(abs_pd(center_dist), _mm256_add_pd(transp_mat3x4_mul_vec4(AbsR, r1), r2))) {
    return true;
  }

  // Test the 9 different cross-axes.
  __m256d symmetric_matrix[3] = {
    _mm256_setr_pd(    0, r2[2], r2[1], 0),
    _mm256_setr_pd(r2[2],     0, r2[0], 0),
    _mm256_setr_pd(r2[1], r2[0],     0, 0),
  };

  // A.x <cross> B.x
  // A.x <cross> B.y
  // A.x <cross> B.z
  ra = fmadd_pd(vec_splat_pd(r1, 1), AbsR[2], _mm256_mul_pd(vec_splat_pd(r1, 2), AbsR[1]));
  rb = mat3x4_mul_vec4(symmetric_matrix, AbsR[0]);
  center_dist = fmsub_pd(vec_splat_pd(t, 2), R[1], _mm256_mul_pd(vec_splat_pd(t, 1), R[2]));
  if (any_gt_pd(abs_pd(center_dist), _mm256_add_pd(ra, rb))) {
    return true;
  }

  // A.y <cross> B.x
  // A.y <cross> B.y
  // A.y <cross> B.z
  ra = fmadd_pd(vec_splat_pd(r1, 0), AbsR[2], _mm256_mul_pd(vec_splat_pd(r1, 2), AbsR[0]));
  rb = mat3x4_mul_vec4(symmetric_matrix, AbsR[1]);
  center_dist = fmsub_pd(vec_splat_pd(t, 0), R[2], _mm256_mul_pd(vec_splat_pd(t, 2), R[0]));
  if (any_gt_pd(abs_pd(center_dist), _mm256_add_pd(ra, rb))) {
    return true;
  }

  // A.z <cross> B.x
  // A.z <cross> B.y
  // A.z <cross> B.z
  ra = fmadd_pd(vec_splat_pd(r1, 0), AbsR[1], _mm256_mul_pd(vec_splat_pd(r1, 1), AbsR[0]));
  rb = mat3x4_mul_vec4(symmetric_matrix, AbsR[2]);
  center_dist = fmsub_pd(vec_splat_pd(t, 1), R[0], _mm256_mul_pd(vec_splat_pd(t, 0), R[1]));
  return any_gt_pd(abs_pd(center_dist), _mm256_add_pd(ra, rb));
}
#endif

#ifdef FCL_SSE_ENABLED
inline bool obbDisjoint(const __m128* R, const __m128& t, const __m128& r1, const __m128& r2)
{
  const float reps = 1e-6;
  const __m128 epsilonxyz = _mm_setr_ps(reps, reps, reps, 0.f);
  __m128 AbsR[3];
  AbsR[0] = _mm_add_ps(abs_ps(R[0]), epsilonxyz);
  AbsR[1] = _mm_add_ps(abs_ps(R[1]), epsilonxyz);
  AbsR[2] = _mm_add_ps(abs_ps(R[2]), epsilonxyz);

  __m128 ra;          // projection of OBB A's halfwidth along three axes
  __m128 rb;          // projection of OBB B's halfwidth along three axes
  __m128 center_dist; // projection of center distance along three axes

  // Test the three major axes of this OBB.
  if (any_gt_ps(abs_ps(t), _mm_add_ps(r1, mat3x4_mul_vec4(AbsR, r2)))) {
    return true;
  }

  // Test the three major axes of the OBB b.
  center_dist = transp_mat3x4_mul_vec4(R, t);
  if (any_gt_ps(abs_ps(center_dist), _mm_add_ps(transp_mat3x4_mul_vec4(AbsR, r1), r2))) {
    return true;
  }

  // Test the 9 different cross-axes.
  __m128 symmetric_matrix[3] = {
    _mm_setr_ps(  0.f, r2[2], r2[1], 0.f),
    _mm_setr_ps(r2[2],   0.f, r2[0], 0.f),
    _mm_setr_ps(r2[1], r2[0],   0.f, 0.f),
  };

  // A.x <cross> B.x
  // A.x <cross> B.y
  // A.x <cross> B.z
  ra = fmadd_ps(vec_splat_ps(r1, 1), AbsR[2], _mm_mul_ps(vec_splat_ps(r1, 2), AbsR[1]));
  rb = mat3x4_mul_vec4(symmetric_matrix, AbsR[0]);
  center_dist = fmsub_ps(vec_splat_ps(t, 2), R[1], _mm_mul_ps(vec_splat_ps(t, 1), R[2]));
  if (any_gt_ps(abs_ps(center_dist), _mm_add_ps(ra, rb))) {
    return true;
  }

  // A.y <cross> B.x
  // A.y <cross> B.y
  // A.y <cross> B.z
  ra = fmadd_ps(vec_splat_ps(r1, 0), AbsR[2], _mm_mul_ps(vec_splat_ps(r1, 2), AbsR[0]));
  rb = mat3x4_mul_vec4(symmetric_matrix, AbsR[1]);
  center_dist = fmsub_ps(vec_splat_ps(t, 0), R[2], _mm_mul_ps(vec_splat_ps(t, 2), R[0]));
  if (any_gt_ps(abs_ps(center_dist), _mm_add_ps(ra, rb))) {
    return true;
  }

  // A.z <cross> B.x
  // A.z <cross> B.y
  // A.z <cross> B.z
  ra = fmadd_ps(vec_splat_ps(r1, 0), AbsR[1], _mm_mul_ps(vec_splat_ps(r1, 1), AbsR[0]));
  rb = mat3x4_mul_vec4(symmetric_matrix, AbsR[2]);
  center_dist = fmsub_ps(vec_splat_ps(t, 1), R[0], _mm_mul_ps(vec_splat_ps(t, 0), R[1]));
  return any_gt_ps(abs_ps(center_dist), _mm_add_ps(ra, rb));
}
#endif

//==============================================================================
#if defined (FCL_AVX_ENABLED)
template <typename S>
bool obbDisjoint(const Matrix3<S>& B, const Vector3<S>& T,
                 const Vector3<S>& a, const Vector3<S>& b)
{
  __m256d B_avx[3] = {
    _mm256_setr_pd(B(0, 0), B(0, 1), B(0, 2), 0),
    _mm256_setr_pd(B(1, 0), B(1, 1), B(1, 2), 0),
    _mm256_setr_pd(B(2, 0), B(2, 1), B(2, 2), 0),
  };
  __m256d T_avx = _mm256_setr_pd(T[0], T[1], T[2], 0);
  __m256d a_avx = _mm256_setr_pd(a[0], a[1], a[2], 0);
  __m256d b_avx = _mm256_setr_pd(b[0], b[1], b[2], 0);
  return obbDisjoint(B_avx, T_avx, a_avx, b_avx);
}
#elif defined (FCL_SSE_ENABLED)
template <typename S>
bool obbDisjoint(const Matrix3<S>& B, const Vector3<S>& T,
                 const Vector3<S>& a, const Vector3<S>& b)
{
  __m128 B_sse[3] = {
    _mm_setr_ps(B(0, 0), B(0, 1), B(0, 2), 0.f),
    _mm_setr_ps(B(1, 0), B(1, 1), B(1, 2), 0.f),
    _mm_setr_ps(B(2, 0), B(2, 1), B(2, 2), 0.f),
  };
  __m128 T_sse = _mm_setr_ps(T[0], T[1], T[2], 0.f);
  __m128 a_sse = _mm_setr_ps(a[0], a[1], a[2], 0.f);
  __m128 b_sse = _mm_setr_ps(b[0], b[1], b[2], 0.f);
  return obbDisjoint(B_sse, T_sse, a_sse, b_sse);
}
#else
template <typename S>
bool obbDisjoint(const Matrix3<S>& B, const Vector3<S>& T,
                 const Vector3<S>& a, const Vector3<S>& b)
{
  S t, s;
  const S reps = 1e-6;

  Matrix3<S> Bf = B.cwiseAbs();
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
#endif

//==============================================================================
template <typename S>
bool obbDisjoint(
    const Transform3<S>& tf,
    const Vector3<S>& a,
    const Vector3<S>& b)
{
  Matrix3<S> B = tf.linear();
  Vector3<S> T = tf.translation();
  return obbDisjoint(B, T, a, b);
}

} // namespace fcl

#endif

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

#ifndef FCL_BV_DETAIL_UTILITY_INL_H
#define FCL_BV_DETAIL_UTILITY_INL_H

#include "fcl/math/geometry.h"
#include "fcl/math/constants.h"

namespace fcl {

//==============================================================================
extern template
void normalize(Vector3d& v, bool* signal);

//==============================================================================
extern template
void hat(Matrix3d& mat, const Vector3d& vec);

//==============================================================================
extern template
void eigen(const Matrix3d& m, Vector3d& dout, Matrix3d& vout);

//==============================================================================
extern template
void eigen_old(const Matrix3d& m, Vector3d& dout, Matrix3d& vout);

//==============================================================================
extern template
void axisFromEigen(
    const Matrix3d& eigenV, const Vector3d& eigenS, Matrix3d& axis);

//==============================================================================
extern template
void axisFromEigen(const Matrix3d& eigenV,
                   const Vector3d& eigenS,
                   Transform3d& tf);

//==============================================================================
extern template
Matrix3d generateCoordinateSystem(const Vector3d& x_axis);

//==============================================================================
extern template
void getRadiusAndOriginAndRectangleSize(
    const Vector3d* const ps,
    const Vector3d* const ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    const Matrix3d& axis,
    Vector3d& origin,
    double l[2],
    double& r);

//==============================================================================
extern template
void getRadiusAndOriginAndRectangleSize(
    const Vector3d* const ps,
    const Vector3d* const ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    Transform3d& tf,
    double l[2],
    double& r);

//==============================================================================
extern template
void circumCircleComputation(
    const Vector3d& a,
    const Vector3d& b,
    const Vector3d& c,
    Vector3d& center,
    double& radius);

//==============================================================================
extern template
double maximumDistance(
    const Vector3d* const ps,
    const Vector3d* const ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    const Vector3d& query);

//==============================================================================
extern template
void getExtentAndCenter(
    const Vector3d* const ps,
    const Vector3d* const ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    const Matrix3d& axis,
    Vector3d& center,
    Vector3d& extent);

//==============================================================================
extern template
void getCovariance(
    const Vector3d* const ps,
    const Vector3d* const ps2,
    Triangle* ts,
    unsigned int* indices,
    int n, Matrix3d& M);

//==============================================================================
namespace detail {
//==============================================================================

//==============================================================================
template <typename S>
FCL_EXPORT
S maximumDistance_mesh(
    const Vector3<S>* const ps,
    const Vector3<S>* const ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    const Vector3<S>& query)
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  S maxD = 0;
  for(int i = 0; i < n; ++i)
  {
    unsigned int index = indirect_index ? indices[i] : i;
    const Triangle& t = ts[index];

    for(int j = 0; j < 3; ++j)
    {
      int point_id = t[j];
      const Vector3<S>& p = ps[point_id];

      S d = (p - query).squaredNorm();
      if(d > maxD) maxD = d;
    }

    if(ps2)
    {
      for(int j = 0; j < 3; ++j)
      {
        int point_id = t[j];
        const Vector3<S>& p = ps2[point_id];

        S d = (p - query).squaredNorm();
        if(d > maxD) maxD = d;
      }
    }
  }

  return std::sqrt(maxD);
}

//==============================================================================
template <typename S>
FCL_EXPORT
S maximumDistance_pointcloud(
    const Vector3<S>* const ps,
    const Vector3<S>* const ps2,
    unsigned int* indices,
    int n,
    const Vector3<S>& query)
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  S maxD = 0;
  for(int i = 0; i < n; ++i)
  {
    int index = indirect_index ? indices[i] : i;

    const Vector3<S>& p = ps[index];
    S d = (p - query).squaredNorm();
    if(d > maxD) maxD = d;

    if(ps2)
    {
      const Vector3<S>& v = ps2[index];
      S d = (v - query).squaredNorm();
      if(d > maxD) maxD = d;
    }
  }

  return std::sqrt(maxD);
}

//==============================================================================
/// @brief Compute the bounding volume extent and center for a set or subset of
/// points. The bounding volume axes are known.
template <typename S>
FCL_EXPORT
void getExtentAndCenter_pointcloud(
    const Vector3<S>* const ps,
    const Vector3<S>* const ps2,
    unsigned int* indices,
    int n,
    const Matrix3<S>& axis,
    Vector3<S>& center,
    Vector3<S>& extent)
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  auto real_max = std::numeric_limits<S>::max();

  Vector3<S> min_coord = Vector3<S>::Constant(real_max);
  Vector3<S> max_coord = Vector3<S>::Constant(-real_max);

  for(int i = 0; i < n; ++i)
  {
    int index = indirect_index ? indices[i] : i;

    const Vector3<S>& p = ps[index];
    Vector3<S> proj(
        axis.col(0).dot(p),
        axis.col(1).dot(p),
        axis.col(2).dot(p));

    for(int j = 0; j < 3; ++j)
    {
      if(proj[j] > max_coord[j])
        max_coord[j] = proj[j];

      if(proj[j] < min_coord[j])
        min_coord[j] = proj[j];
    }

    if(ps2)
    {
      const Vector3<S>& v = ps2[index];
      Vector3<S> proj(
          axis.col(0).dot(v),
          axis.col(1).dot(v),
          axis.col(2).dot(v));

      for(int j = 0; j < 3; ++j)
      {
        if(proj[j] > max_coord[j])
          max_coord[j] = proj[j];

        if(proj[j] < min_coord[j])
          min_coord[j] = proj[j];
      }
    }
  }

  const Vector3<S> o = (max_coord + min_coord) / 2;
  center.noalias() = axis * o;
  extent.noalias() = (max_coord - min_coord) * 0.5;
}

//==============================================================================
/// @brief Compute the bounding volume extent and center for a set or subset of
/// points. The bounding volume axes are known.
template <typename S>
FCL_EXPORT
void getExtentAndCenter_mesh(
    const Vector3<S>* const ps,
    const Vector3<S>* const ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    const Matrix3<S>& axis,
    Vector3<S>& center,
    Vector3<S>& extent)
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  auto real_max = std::numeric_limits<S>::max();

  Vector3<S> min_coord = Vector3<S>::Constant(real_max);
  Vector3<S> max_coord = Vector3<S>::Constant(-real_max);

  for(int i = 0; i < n; ++i)
  {
    unsigned int index = indirect_index? indices[i] : i;
    const Triangle& t = ts[index];

    for(int j = 0; j < 3; ++j)
    {
      int point_id = t[j];
      const Vector3<S>& p = ps[point_id];
      Vector3<S> proj(
          axis.col(0).dot(p),
          axis.col(1).dot(p),
          axis.col(2).dot(p));

      for(int k = 0; k < 3; ++k)
      {
        if(proj[k] > max_coord[k])
          max_coord[k] = proj[k];

        if(proj[k] < min_coord[k])
          min_coord[k] = proj[k];
      }
    }

    if(ps2)
    {
      for(int j = 0; j < 3; ++j)
      {
        int point_id = t[j];
        const Vector3<S>& p = ps2[point_id];
        Vector3<S> proj(
            axis.col(0).dot(p),
            axis.col(1).dot(p),
            axis.col(2).dot(p));

        for(int k = 0; k < 3; ++k)
        {
          if(proj[k] > max_coord[k])
            max_coord[k] = proj[k];

          if(proj[k] < min_coord[k])
            min_coord[k] = proj[k];
        }
      }
    }
  }

  const Vector3<S> o = (max_coord + min_coord) / 2;
  center.noalias() = axis * o;
  extent.noalias() = (max_coord - min_coord) / 2;
}

//==============================================================================
extern template
double maximumDistance_mesh(
    const Vector3d* const ps,
    const Vector3d* const ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    const Vector3d& query);

//==============================================================================
extern template
double maximumDistance_pointcloud(
    const Vector3d* const ps,
    const Vector3d* const ps2,
    unsigned int* indices,
    int n,
    const Vector3d& query);

//==============================================================================
extern template
void getExtentAndCenter_pointcloud(
    const Vector3d* const ps,
    const Vector3d* const ps2,
    unsigned int* indices,
    int n,
    const Matrix3d& axis,
    Vector3d& center,
    Vector3d& extent);

//==============================================================================
extern template
void getExtentAndCenter_mesh(
    const Vector3d* const ps,
    const Vector3d* const ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    const Matrix3d& axis,
    Vector3d& center,
    Vector3d& extent);

//==============================================================================
} // namespace detail
//==============================================================================

//==============================================================================
template <typename S>
FCL_EXPORT
void normalize(Vector3<S>& v, bool* signal)
{
  S sqr_length = v.squaredNorm();

  if (sqr_length > 0)
  {
    v /= std::sqrt(sqr_length);
    *signal = true;
  }
  else
  {
    *signal = false;
  }
}

//==============================================================================
template <typename Derived>
FCL_EXPORT
typename Derived::RealScalar triple(const Eigen::MatrixBase<Derived>& x,
                                    const Eigen::MatrixBase<Derived>& y,
                                    const Eigen::MatrixBase<Derived>& z)
{
  return x.dot(y.cross(z));
}

//==============================================================================
template <typename S, int M, int N>
FCL_EXPORT
VectorN<S, M+N> combine(
    const VectorN<S, M>& v1, const VectorN<S, N>& v2)
{
  VectorN<S, M+N> v;
  v << v1, v2;

  return v;
}

//==============================================================================
template <typename S>
FCL_EXPORT
void hat(Matrix3<S>& mat, const Vector3<S>& vec)
{
  mat << 0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0;
}

//==============================================================================
template<typename S>
FCL_EXPORT
void eigen(const Matrix3<S>& m, Vector3<S>& dout, Matrix3<S>& vout)
{
  // We assume that m is symmetric matrix.
  Eigen::SelfAdjointEigenSolver<Matrix3<S>> eigensolver(m);
  if (eigensolver.info() != Eigen::Success)
  {
    std::cerr << "[eigen] Failed to compute eigendecomposition.\n";
    return;
  }
  dout = eigensolver.eigenvalues();
  vout = eigensolver.eigenvectors();
}

//==============================================================================
template<typename S>
FCL_EXPORT
void eigen_old(const Matrix3<S>& m, Vector3<S>& dout, Matrix3<S>& vout)
{
  Matrix3<S> R(m);
  int n = 3;
  int j, iq, ip, i;
  S tresh, theta, tau, t, sm, s, h, g, c;
  int nrot;
  S b[3];
  S z[3];
  S v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  S d[3];

  for(ip = 0; ip < n; ++ip)
  {
    b[ip] = d[ip] = R(ip, ip);
    z[ip] = 0;
  }

  nrot = 0;

  for(i = 0; i < 50; ++i)
  {
    sm = 0;
    for(ip = 0; ip < n; ++ip)
      for(iq = ip + 1; iq < n; ++iq)
        sm += std::abs(R(ip, iq));
    if(sm == 0.0)
    {
      vout.col(0) << v[0][0], v[0][1], v[0][2];
      vout.col(1) << v[1][0], v[1][1], v[1][2];
      vout.col(2) << v[2][0], v[2][1], v[2][2];
      dout[0] = d[0]; dout[1] = d[1]; dout[2] = d[2];
      return;
    }

    if(i < 3) tresh = 0.2 * sm / (n * n);
    else tresh = 0.0;

    for(ip = 0; ip < n; ++ip)
    {
      for(iq= ip + 1; iq < n; ++iq)
      {
        g = 100.0 * std::abs(R(ip, iq));
        if(i > 3 &&
           std::abs(d[ip]) + g == std::abs(d[ip]) &&
           std::abs(d[iq]) + g == std::abs(d[iq]))
          R(ip, iq) = 0.0;
        else if(std::abs(R(ip, iq)) > tresh)
        {
          h = d[iq] - d[ip];
          if(std::abs(h) + g == std::abs(h)) t = (R(ip, iq)) / h;
          else
          {
            theta = 0.5 * h / (R(ip, iq));
            t = 1.0 /(std::abs(theta) + std::sqrt(1.0 + theta * theta));
            if(theta < 0.0) t = -t;
          }
          c = 1.0 / std::sqrt(1 + t * t);
          s = t * c;
          tau = s / (1.0 + c);
          h = t * R(ip, iq);
          z[ip] -= h;
          z[iq] += h;
          d[ip] -= h;
          d[iq] += h;
          R(ip, iq) = 0.0;
          for(j = 0; j < ip; ++j) { g = R(j, ip); h = R(j, iq); R(j, ip) = g - s * (h + g * tau); R(j, iq) = h + s * (g - h * tau); }
          for(j = ip + 1; j < iq; ++j) { g = R(ip, j); h = R(j, iq); R(ip, j) = g - s * (h + g * tau); R(j, iq) = h + s * (g - h * tau); }
          for(j = iq + 1; j < n; ++j) { g = R(ip, j); h = R(iq, j); R(ip, j) = g - s * (h + g * tau); R(iq, j) = h + s * (g - h * tau); }
          for(j = 0; j < n; ++j) { g = v[j][ip]; h = v[j][iq]; v[j][ip] = g - s * (h + g * tau); v[j][iq] = h + s * (g - h * tau); }
          nrot++;
        }
      }
    }
    for(ip = 0; ip < n; ++ip)
    {
      b[ip] += z[ip];
      d[ip] = b[ip];
      z[ip] = 0.0;
    }
  }

  std::cerr << "eigen: too many iterations in Jacobi transform.\n";

  return;
}

//==============================================================================
template <typename S>
FCL_EXPORT
void axisFromEigen(const Matrix3<S>& eigenV,
                   const Vector3<S>& eigenS,
                   Matrix3<S>& axis)
{
  int min, mid, max;

  if(eigenS[0] > eigenS[1])
  {
    max = 0;
    min = 1;
  }
  else
  {
    min = 0;
    max = 1;
  }

  if(eigenS[2] < eigenS[min])
  {
    mid = min;
    min = 2;
  }
  else if(eigenS[2] > eigenS[max])
  {
    mid = max;
    max = 2;
  }
  else
  {
    mid = 2;
  }

  axis.col(0) = eigenV.row(max);
  axis.col(1) = eigenV.row(mid);
  axis.col(2).noalias() = axis.col(0).cross(axis.col(1));
}

//==============================================================================
template <typename S>
FCL_EXPORT
void axisFromEigen(const Matrix3<S>& eigenV,
                   const Vector3<S>& eigenS,
                   Transform3<S>& tf)
{
  int min, mid, max;

  if(eigenS[0] > eigenS[1])
  {
    max = 0;
    min = 1;
  }
  else
  {
    min = 0;
    max = 1;
  }

  if(eigenS[2] < eigenS[min])
  {
    mid = min;
    min = 2;
  }
  else if(eigenS[2] > eigenS[max])
  {
    mid = max;
    max = 2;
  }
  else
  {
    mid = 2;
  }

  tf.linear().col(0) = eigenV.col(max);
  tf.linear().col(1) = eigenV.col(mid);
  tf.linear().col(2).noalias() = tf.linear().col(0).cross(tf.linear().col(1));
}

//==============================================================================
template <typename S>
FCL_EXPORT
Matrix3<S> generateCoordinateSystem(const Vector3<S>& x_axis)
{
  Matrix3<S> axis;
  axis.col(0).noalias() = x_axis.normalized();
  axis.col(1).noalias() = axis.col(0).unitOrthogonal();
  axis.col(2).noalias() = axis.col(0).cross(axis.col(1)).normalized();
  return axis;
}

//==============================================================================
template <typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD>
FCL_EXPORT
void relativeTransform(
    const Eigen::MatrixBase<DerivedA>& R1, const Eigen::MatrixBase<DerivedB>& t1,
    const Eigen::MatrixBase<DerivedA>& R2, const Eigen::MatrixBase<DerivedB>& t2,
    Eigen::MatrixBase<DerivedC>& R, Eigen::MatrixBase<DerivedD>& t)
{
  EIGEN_STATIC_ASSERT(
        DerivedA::RowsAtCompileTime == 3
        && DerivedA::ColsAtCompileTime == 3,
        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

  EIGEN_STATIC_ASSERT(
        DerivedB::RowsAtCompileTime == 3
        && DerivedB::ColsAtCompileTime == 1,
        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

  EIGEN_STATIC_ASSERT(
        DerivedC::RowsAtCompileTime == 3
        && DerivedC::ColsAtCompileTime == 3,
        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

  EIGEN_STATIC_ASSERT(
        DerivedD::RowsAtCompileTime == 3
        && DerivedD::ColsAtCompileTime == 1,
        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

  R.noalias() = R1.transpose() * R2;
  t.noalias() = R1.transpose() * (t2 - t1);
}

//==============================================================================
template <typename S, typename DerivedA, typename DerivedB>
FCL_EXPORT
void relativeTransform(
    const Transform3<S>& T1,
    const Transform3<S>& T2,
    Eigen::MatrixBase<DerivedA>& R, Eigen::MatrixBase<DerivedB>& t)
{
  EIGEN_STATIC_ASSERT(
        DerivedA::RowsAtCompileTime == 3
        && DerivedA::ColsAtCompileTime == 3,
        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

  EIGEN_STATIC_ASSERT(
        DerivedB::RowsAtCompileTime == 3
        && DerivedB::ColsAtCompileTime == 1,
        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

  R.noalias() = T1.linear().transpose() * T2.linear();
  t.noalias() = T1.linear().transpose() * (T2.translation() - T1.translation());
}

//==============================================================================
template <typename S>
FCL_EXPORT
void getRadiusAndOriginAndRectangleSize(
    const Vector3<S>* const ps,
    const Vector3<S>* const ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    const Matrix3<S>& axis,
    Vector3<S>& origin,
    S l[2],
    S& r)
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  int size_P = ((ps2) ? 2 : 1) * ((ts) ? 3 : 1) * n;

  std::vector<Vector3<S>> P(size_P);

  int P_id = 0;

  if(ts)
  {
    for(int i = 0; i < n; ++i)
    {
      int index = indirect_index ? indices[i] : i;
      const Triangle& t = ts[index];

      for(int j = 0; j < 3; ++j)
      {
        int point_id = t[j];
        const Vector3<S>& p = ps[point_id];
        P[P_id][0] = axis.col(0).dot(p);
        P[P_id][1] = axis.col(1).dot(p);
        P[P_id][2] = axis.col(2).dot(p);
        P_id++;
      }

      if(ps2)
      {
        for(int j = 0; j < 3; ++j)
        {
          int point_id = t[j];
          const Vector3<S>& p = ps2[point_id];
          P[P_id][0] = axis.col(0).dot(p);
          P[P_id][1] = axis.col(1).dot(p);
          P[P_id][2] = axis.col(2).dot(p);
          P_id++;
        }
      }
    }
  }
  else
  {
    for(int i = 0; i < n; ++i)
    {
      int index = indirect_index ? indices[i] : i;

      const Vector3<S>& p = ps[index];
      P[P_id][0] = axis.col(0).dot(p);
      P[P_id][1] = axis.col(1).dot(p);
      P[P_id][2] = axis.col(2).dot(p);
      P_id++;

      if(ps2)
      {
        const Vector3<S>& v = ps2[index];
        P[P_id][0] = axis.col(0).dot(v);
        P[P_id][1] = axis.col(1).dot(v);
        P[P_id][2] = axis.col(2).dot(v);
        P_id++;
      }
    }
  }

  S minx, maxx, miny, maxy, minz, maxz;

  S cz, radsqr;

  minz = maxz = P[0][2];

  for(int i = 1; i < size_P; ++i)
  {
    S z_value = P[i][2];
    if(z_value < minz) minz = z_value;
    else if(z_value > maxz) maxz = z_value;
  }

  r = (S)0.5 * (maxz - minz);
  radsqr = r * r;
  cz = (S)0.5 * (maxz + minz);

  // compute an initial length of rectangle along x direction

  // find minx and maxx as starting points

  int minindex, maxindex;
  minindex = maxindex = 0;
  S mintmp, maxtmp;
  mintmp = maxtmp = P[0][0];

  for(int i = 1; i < size_P; ++i)
  {
    S x_value = P[i][0];
    if(x_value < mintmp)
    {
      minindex = i;
      mintmp = x_value;
    }
    else if(x_value > maxtmp)
    {
      maxindex = i;
      maxtmp = x_value;
    }
  }

  S x, dz;
  dz = P[minindex][2] - cz;
  minx = P[minindex][0] + std::sqrt(std::max<S>(radsqr - dz * dz, 0));
  dz = P[maxindex][2] - cz;
  maxx = P[maxindex][0] - std::sqrt(std::max<S>(radsqr - dz * dz, 0));


  // grow minx

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][0] < minx)
    {
      dz = P[i][2] - cz;
      x = P[i][0] + std::sqrt(std::max<S>(radsqr - dz * dz, 0));
      if(x < minx) minx = x;
    }
  }

  // grow maxx

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][0] > maxx)
    {
      dz = P[i][2] - cz;
      x = P[i][0] - std::sqrt(std::max<S>(radsqr - dz * dz, 0));
      if(x > maxx) maxx = x;
    }
  }

  // compute an initial length of rectangle along y direction

  // find miny and maxy as starting points

  minindex = maxindex = 0;
  mintmp = maxtmp = P[0][1];
  for(int i = 1; i < size_P; ++i)
  {
    S y_value = P[i][1];
    if(y_value < mintmp)
    {
      minindex = i;
      mintmp = y_value;
    }
    else if(y_value > maxtmp)
    {
      maxindex = i;
      maxtmp = y_value;
    }
  }

  S y;
  dz = P[minindex][2] - cz;
  miny = P[minindex][1] + std::sqrt(std::max<S>(radsqr - dz * dz, 0));
  dz = P[maxindex][2] - cz;
  maxy = P[maxindex][1] - std::sqrt(std::max<S>(radsqr - dz * dz, 0));

  // grow miny

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][1] < miny)
    {
      dz = P[i][2] - cz;
      y = P[i][1] + std::sqrt(std::max<S>(radsqr - dz * dz, 0));
      if(y < miny) miny = y;
    }
  }

  // grow maxy

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][1] > maxy)
    {
      dz = P[i][2] - cz;
      y = P[i][1] - std::sqrt(std::max<S>(radsqr - dz * dz, 0));
      if(y > maxy) maxy = y;
    }
  }

  // corners may have some points which are not covered - grow lengths if necessary
  // quite conservative (can be improved)
  S dx, dy, u, t;
  S a = std::sqrt((S)0.5);
  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][0] > maxx)
    {
      if(P[i][1] > maxy)
      {
        dx = P[i][0] - maxx;
        dy = P[i][1] - maxy;
        u = dx * a + dy * a;
        t = (a*u - dx)*(a*u - dx) +
            (a*u - dy)*(a*u - dy) +
            (cz - P[i][2])*(cz - P[i][2]);
        u = u - std::sqrt(std::max<S>(radsqr - t, 0));
        if(u > 0)
        {
          maxx += u*a;
          maxy += u*a;
        }
      }
      else if(P[i][1] < miny)
      {
        dx = P[i][0] - maxx;
        dy = P[i][1] - miny;
        u = dx * a - dy * a;
        t = (a*u - dx)*(a*u - dx) +
            (-a*u - dy)*(-a*u - dy) +
            (cz - P[i][2])*(cz - P[i][2]);
        u = u - std::sqrt(std::max<S>(radsqr - t, 0));
        if(u > 0)
        {
          maxx += u*a;
          miny -= u*a;
        }
      }
    }
    else if(P[i][0] < minx)
    {
      if(P[i][1] > maxy)
      {
        dx = P[i][0] - minx;
        dy = P[i][1] - maxy;
        u = dy * a - dx * a;
        t = (-a*u - dx)*(-a*u - dx) +
            (a*u - dy)*(a*u - dy) +
            (cz - P[i][2])*(cz - P[i][2]);
        u = u - std::sqrt(std::max<S>(radsqr - t, 0));
        if(u > 0)
        {
          minx -= u*a;
          maxy += u*a;
        }
      }
      else if(P[i][1] < miny)
      {
        dx = P[i][0] - minx;
        dy = P[i][1] - miny;
        u = -dx * a - dy * a;
        t = (-a*u - dx)*(-a*u - dx) +
            (-a*u - dy)*(-a*u - dy) +
            (cz - P[i][2])*(cz - P[i][2]);
        u = u - std::sqrt(std::max<S>(radsqr - t, 0));
        if (u > 0)
        {
          minx -= u*a;
          miny -= u*a;
        }
      }
    }
  }

  origin = axis.col(0) * minx + axis.col(1) * miny + axis.col(2) * cz;

  l[0] = maxx - minx;
  if(l[0] < 0) l[0] = 0;
  l[1] = maxy - miny;
  if(l[1] < 0) l[1] = 0;

}

//==============================================================================
template <typename S>
FCL_EXPORT
void getRadiusAndOriginAndRectangleSize(
    const Vector3<S>* const ps,
    const Vector3<S>* const ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    Transform3<S>& tf,
    S l[2],
    S& r)
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  int size_P = ((ps2) ? 2 : 1) * ((ts) ? 3 : 1) * n;

  std::vector<Vector3<S>> P(size_P);

  int P_id = 0;

  if(ts)
  {
    for(int i = 0; i < n; ++i)
    {
      int index = indirect_index ? indices[i] : i;
      const Triangle& t = ts[index];

      for(int j = 0; j < 3; ++j)
      {
        int point_id = t[j];
        const Vector3<S>& p = ps[point_id];
        P[P_id][0] = tf.linear().col(0).dot(p);
        P[P_id][1] = tf.linear().col(1).dot(p);
        P[P_id][2] = tf.linear().col(2).dot(p);
        P_id++;
      }

      if(ps2)
      {
        for(int j = 0; j < 3; ++j)
        {
          int point_id = t[j];
          const Vector3<S>& p = ps2[point_id];
          P[P_id][0] = tf.linear().col(0).dot(p);
          P[P_id][1] = tf.linear().col(1).dot(p);
          P[P_id][2] = tf.linear().col(2).dot(p);
          P_id++;
        }
      }
    }
  }
  else
  {
    for(int i = 0; i < n; ++i)
    {
      int index = indirect_index ? indices[i] : i;

      const Vector3<S>& p = ps[index];
      P[P_id][0] = tf.linear().col(0).dot(p);
      P[P_id][1] = tf.linear().col(1).dot(p);
      P[P_id][2] = tf.linear().col(2).dot(p);
      P_id++;

      if(ps2)
      {
        P[P_id][0] = tf.linear().col(0).dot(ps2[index]);
        P[P_id][1] = tf.linear().col(1).dot(ps2[index]);
        P[P_id][2] = tf.linear().col(2).dot(ps2[index]);
        P_id++;
      }
    }
  }

  S minx, maxx, miny, maxy, minz, maxz;

  S cz, radsqr;

  minz = maxz = P[0][2];

  for(int i = 1; i < size_P; ++i)
  {
    S z_value = P[i][2];
    if(z_value < minz) minz = z_value;
    else if(z_value > maxz) maxz = z_value;
  }

  r = (S)0.5 * (maxz - minz);
  radsqr = r * r;
  cz = (S)0.5 * (maxz + minz);

  // compute an initial length of rectangle along x direction

  // find minx and maxx as starting points

  int minindex, maxindex;
  minindex = maxindex = 0;
  S mintmp, maxtmp;
  mintmp = maxtmp = P[0][0];

  for(int i = 1; i < size_P; ++i)
  {
    S x_value = P[i][0];
    if(x_value < mintmp)
    {
      minindex = i;
      mintmp = x_value;
    }
    else if(x_value > maxtmp)
    {
      maxindex = i;
      maxtmp = x_value;
    }
  }

  S x, dz;
  dz = P[minindex][2] - cz;
  minx = P[minindex][0] + std::sqrt(std::max<S>(radsqr - dz * dz, 0));
  dz = P[maxindex][2] - cz;
  maxx = P[maxindex][0] - std::sqrt(std::max<S>(radsqr - dz * dz, 0));


  // grow minx

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][0] < minx)
    {
      dz = P[i][2] - cz;
      x = P[i][0] + std::sqrt(std::max<S>(radsqr - dz * dz, 0));
      if(x < minx) minx = x;
    }
  }

  // grow maxx

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][0] > maxx)
    {
      dz = P[i][2] - cz;
      x = P[i][0] - std::sqrt(std::max<S>(radsqr - dz * dz, 0));
      if(x > maxx) maxx = x;
    }
  }

  // compute an initial length of rectangle along y direction

  // find miny and maxy as starting points

  minindex = maxindex = 0;
  mintmp = maxtmp = P[0][1];
  for(int i = 1; i < size_P; ++i)
  {
    S y_value = P[i][1];
    if(y_value < mintmp)
    {
      minindex = i;
      mintmp = y_value;
    }
    else if(y_value > maxtmp)
    {
      maxindex = i;
      maxtmp = y_value;
    }
  }

  S y;
  dz = P[minindex][2] - cz;
  miny = P[minindex][1] + std::sqrt(std::max<S>(radsqr - dz * dz, 0));
  dz = P[maxindex][2] - cz;
  maxy = P[maxindex][1] - std::sqrt(std::max<S>(radsqr - dz * dz, 0));

  // grow miny

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][1] < miny)
    {
      dz = P[i][2] - cz;
      y = P[i][1] + std::sqrt(std::max<S>(radsqr - dz * dz, 0));
      if(y < miny) miny = y;
    }
  }

  // grow maxy

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][1] > maxy)
    {
      dz = P[i][2] - cz;
      y = P[i][1] - std::sqrt(std::max<S>(radsqr - dz * dz, 0));
      if(y > maxy) maxy = y;
    }
  }

  // corners may have some points which are not covered - grow lengths if necessary
  // quite conservative (can be improved)
  S dx, dy, u, t;
  S a = std::sqrt((S)0.5);
  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][0] > maxx)
    {
      if(P[i][1] > maxy)
      {
        dx = P[i][0] - maxx;
        dy = P[i][1] - maxy;
        u = dx * a + dy * a;
        t = (a*u - dx)*(a*u - dx) +
            (a*u - dy)*(a*u - dy) +
            (cz - P[i][2])*(cz - P[i][2]);
        u = u - std::sqrt(std::max<S>(radsqr - t, 0));
        if(u > 0)
        {
          maxx += u*a;
          maxy += u*a;
        }
      }
      else if(P[i][1] < miny)
      {
        dx = P[i][0] - maxx;
        dy = P[i][1] - miny;
        u = dx * a - dy * a;
        t = (a*u - dx)*(a*u - dx) +
            (-a*u - dy)*(-a*u - dy) +
            (cz - P[i][2])*(cz - P[i][2]);
        u = u - std::sqrt(std::max<S>(radsqr - t, 0));
        if(u > 0)
        {
          maxx += u*a;
          miny -= u*a;
        }
      }
    }
    else if(P[i][0] < minx)
    {
      if(P[i][1] > maxy)
      {
        dx = P[i][0] - minx;
        dy = P[i][1] - maxy;
        u = dy * a - dx * a;
        t = (-a*u - dx)*(-a*u - dx) +
            (a*u - dy)*(a*u - dy) +
            (cz - P[i][2])*(cz - P[i][2]);
        u = u - std::sqrt(std::max<S>(radsqr - t, 0));
        if(u > 0)
        {
          minx -= u*a;
          maxy += u*a;
        }
      }
      else if(P[i][1] < miny)
      {
        dx = P[i][0] - minx;
        dy = P[i][1] - miny;
        u = -dx * a - dy * a;
        t = (-a*u - dx)*(-a*u - dx) +
            (-a*u - dy)*(-a*u - dy) +
            (cz - P[i][2])*(cz - P[i][2]);
        u = u - std::sqrt(std::max<S>(radsqr - t, 0));
        if (u > 0)
        {
          minx -= u*a;
          miny -= u*a;
        }
      }
    }
  }

  tf.translation().noalias() = tf.linear().col(0) * minx;
  tf.translation().noalias() += tf.linear().col(1) * miny;
  tf.translation().noalias() += tf.linear().col(2) * cz;

  l[0] = maxx - minx;
  if(l[0] < 0) l[0] = 0;
  l[1] = maxy - miny;
  if(l[1] < 0) l[1] = 0;
}

//==============================================================================
template <typename S>
FCL_EXPORT
void circumCircleComputation(
    const Vector3<S>& a,
    const Vector3<S>& b,
    const Vector3<S>& c,
    Vector3<S>& center,
    S& radius)
{
  Vector3<S> e1 = a - c;
  Vector3<S> e2 = b - c;
  S e1_len2 = e1.squaredNorm();
  S e2_len2 = e2.squaredNorm();
  Vector3<S> e3 = e1.cross(e2);
  S e3_len2 = e3.squaredNorm();
  radius = e1_len2 * e2_len2 * (e1 - e2).squaredNorm() / e3_len2;
  radius = std::sqrt(radius) * 0.5;

  center = c;
  center.noalias() += (e2 * e1_len2 - e1 * e2_len2).cross(e3) * (0.5 * 1 / e3_len2);
}



//==============================================================================
template <typename S>
FCL_EXPORT
S maximumDistance(
    const Vector3<S>* const ps,
    const Vector3<S>* const ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    const Vector3<S>& query)
{
  if(ts)
    return detail::maximumDistance_mesh(ps, ps2, ts, indices, n, query);
  else
    return detail::maximumDistance_pointcloud(ps, ps2, indices, n, query);
}

//==============================================================================
template <typename S>
FCL_EXPORT
void getExtentAndCenter(
    const Vector3<S>* const ps,
    const Vector3<S>* const ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    const Matrix3<S>& axis,
    Vector3<S>& center,
    Vector3<S>& extent)
{
  if(ts)
    detail::getExtentAndCenter_mesh(ps, ps2, ts, indices, n, axis, center, extent);
  else
    detail::getExtentAndCenter_pointcloud(ps, ps2, indices, n, axis, center, extent);
}

//==============================================================================
template <typename S>
FCL_EXPORT
void getCovariance(
    const Vector3<S>* const ps,
    const Vector3<S>* const ps2,
    Triangle* ts,
    unsigned int* indices,
    int n, Matrix3<S>& M)
{
  Vector3<S> S1 = Vector3<S>::Zero();
  Vector3<S> S2[3] = {
    Vector3<S>::Zero(), Vector3<S>::Zero(), Vector3<S>::Zero()
  };

  if(ts)
  {
    for(int i = 0; i < n; ++i)
    {
      const Triangle& t = (indices) ? ts[indices[i]] : ts[i];

      const Vector3<S>& p1 = ps[t[0]];
      const Vector3<S>& p2 = ps[t[1]];
      const Vector3<S>& p3 = ps[t[2]];

      S1 += (p1 + p2 + p3).eval();

      S2[0][0] += (p1[0] * p1[0] + p2[0] * p2[0] + p3[0] * p3[0]);
      S2[1][1] += (p1[1] * p1[1] + p2[1] * p2[1] + p3[1] * p3[1]);
      S2[2][2] += (p1[2] * p1[2] + p2[2] * p2[2] + p3[2] * p3[2]);
      S2[0][1] += (p1[0] * p1[1] + p2[0] * p2[1] + p3[0] * p3[1]);
      S2[0][2] += (p1[0] * p1[2] + p2[0] * p2[2] + p3[0] * p3[2]);
      S2[1][2] += (p1[1] * p1[2] + p2[1] * p2[2] + p3[1] * p3[2]);

      if(ps2)
      {
        const Vector3<S>& p1 = ps2[t[0]];
        const Vector3<S>& p2 = ps2[t[1]];
        const Vector3<S>& p3 = ps2[t[2]];

        S1 += (p1 + p2 + p3).eval();

        S2[0][0] += (p1[0] * p1[0] + p2[0] * p2[0] + p3[0] * p3[0]);
        S2[1][1] += (p1[1] * p1[1] + p2[1] * p2[1] + p3[1] * p3[1]);
        S2[2][2] += (p1[2] * p1[2] + p2[2] * p2[2] + p3[2] * p3[2]);
        S2[0][1] += (p1[0] * p1[1] + p2[0] * p2[1] + p3[0] * p3[1]);
        S2[0][2] += (p1[0] * p1[2] + p2[0] * p2[2] + p3[0] * p3[2]);
        S2[1][2] += (p1[1] * p1[2] + p2[1] * p2[2] + p3[1] * p3[2]);
      }
    }
  }
  else
  {
    for(int i = 0; i < n; ++i)
    {
      const Vector3<S>& p = (indices) ? ps[indices[i]] : ps[i];
      S1 += p;
      S2[0][0] += (p[0] * p[0]);
      S2[1][1] += (p[1] * p[1]);
      S2[2][2] += (p[2] * p[2]);
      S2[0][1] += (p[0] * p[1]);
      S2[0][2] += (p[0] * p[2]);
      S2[1][2] += (p[1] * p[2]);

      if(ps2) // another frame
      {
        const Vector3<S>& p = (indices) ? ps2[indices[i]] : ps2[i];
        S1 += p;
        S2[0][0] += (p[0] * p[0]);
        S2[1][1] += (p[1] * p[1]);
        S2[2][2] += (p[2] * p[2]);
        S2[0][1] += (p[0] * p[1]);
        S2[0][2] += (p[0] * p[2]);
        S2[1][2] += (p[1] * p[2]);
      }
    }
  }

  int n_points = ((ps2) ? 2 : 1) * ((ts) ? 3 : 1) * n;

  M(0, 0) = S2[0][0] - S1[0]*S1[0] / n_points;
  M(1, 1) = S2[1][1] - S1[1]*S1[1] / n_points;
  M(2, 2) = S2[2][2] - S1[2]*S1[2] / n_points;
  M(0, 1) = S2[0][1] - S1[0]*S1[1] / n_points;
  M(1, 2) = S2[1][2] - S1[1]*S1[2] / n_points;
  M(0, 2) = S2[0][2] - S1[0]*S1[2] / n_points;
  M(1, 0) = M(0, 1);
  M(2, 0) = M(0, 2);
  M(2, 1) = M(1, 2);
}

} // namespace fcl

#endif

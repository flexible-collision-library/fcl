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

#ifndef FCL_GEOMETRY_H
#define FCL_GEOMETRY_H

#include <cmath>
#include <iostream>
#include <vector>

#include "fcl/config.h"
#include "fcl/data_types.h"
#include "fcl/math/triangle.h"

namespace fcl {

template <typename Scalar>
void normalize(Vector3<Scalar>& v, bool* signal);

template <typename Derived>
typename Derived::RealScalar triple(const Eigen::MatrixBase<Derived>& x,
                                    const Eigen::MatrixBase<Derived>& y,
                                    const Eigen::MatrixBase<Derived>& z);

template <typename Derived>
void generateCoordinateSystem(
    const Eigen::MatrixBase<Derived>& w,
    Eigen::MatrixBase<Derived>& u,
    Eigen::MatrixBase<Derived>& v);

template <typename Scalar, int M, int N>
VectorN<Scalar, M+N> combine(
    const VectorN<Scalar, M>& v1, const VectorN<Scalar, N>& v2);

template <typename Scalar>
void hat(Matrix3<Scalar>& mat, const Vector3<Scalar>& vec);

/// @brief compute the eigen vector and eigen vector of a matrix. dout is the
/// eigen values, vout is the eigen vectors
template <typename Scalar>
void eigen(const Matrix3<Scalar>& m, Vector3<Scalar>& dout, Matrix3<Scalar>& vout);

template <typename Scalar>
void axisFromEigen(const Matrix3<Scalar>& eigenV,
                   const Vector3<Scalar>& eigenS,
                   Matrix3<Scalar>& axis);

template <typename Scalar>
void generateCoordinateSystem(Matrix3<Scalar>& axis);

template <typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD>
void relativeTransform(
    const Eigen::MatrixBase<DerivedA>& R1, const Eigen::MatrixBase<DerivedB>& t1,
    const Eigen::MatrixBase<DerivedA>& R2, const Eigen::MatrixBase<DerivedB>& t2,
    Eigen::MatrixBase<DerivedC>& R, Eigen::MatrixBase<DerivedD>& t);

template <typename Scalar, typename DerivedA, typename DerivedB>
void relativeTransform(
    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& T1,
    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& T2,
    Eigen::MatrixBase<DerivedA>& R, Eigen::MatrixBase<DerivedB>& t);

/// @brief Compute the RSSd bounding volume parameters: radius, rectangle size
/// and the origin, given the BV axises.
template <typename Scalar>
void getRadiusAndOriginAndRectangleSize(
    Vector3<Scalar>* ps,
    Vector3<Scalar>* ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    const Matrix3<Scalar>& axis,
    Vector3<Scalar>& origin,
    Scalar l[2],
    Scalar& r);

/// @brief Compute the center and radius for a triangle's circumcircle
template <typename Scalar>
void circumCircleComputation(
    const Vector3<Scalar>& a,
    const Vector3<Scalar>& b,
    const Vector3<Scalar>& c,
    Vector3<Scalar>& center,
    Scalar& radius);

template <typename Scalar>
Scalar maximumDistance_mesh(
    Vector3<Scalar>* ps,
    Vector3<Scalar>* ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    const Vector3<Scalar>& query);

template <typename Scalar>
Scalar maximumDistance_pointcloud(
    Vector3<Scalar>* ps,
    Vector3<Scalar>* ps2,
    unsigned int* indices,
    int n,
    const Vector3<Scalar>& query);

/// @brief Compute the maximum distance from a given center point to a point cloud
template <typename Scalar>
Scalar maximumDistance(
    Vector3<Scalar>* ps,
    Vector3<Scalar>* ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    const Vector3<Scalar>& query);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
void normalize(Vector3<Scalar>& v, bool* signal)
{
  Scalar sqr_length = v.squaredNorm();

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
typename Derived::RealScalar triple(const Eigen::MatrixBase<Derived>& x,
                                    const Eigen::MatrixBase<Derived>& y,
                                    const Eigen::MatrixBase<Derived>& z)
{
  return x.dot(y.cross(z));
}

//==============================================================================
template <typename Derived>
void generateCoordinateSystem(
    const Eigen::MatrixBase<Derived>& w,
    Eigen::MatrixBase<Derived>& u,
    Eigen::MatrixBase<Derived>& v)
{
  typename Derived::RealScalar inv_length;

  if(std::abs(w[0]) >= std::abs(w[1]))
  {
    inv_length = 1.0 / std::sqrt(w[0] * w[0] + w[2] * w[2]);
    u[0] = -w[2] * inv_length;
    u[1] = 0;
    u[2] =  w[0] * inv_length;
    v[0] =  w[1] * u[2];
    v[1] =  w[2] * u[0] - w[0] * u[2];
    v[2] = -w[1] * u[0];
  }
  else
  {
    inv_length = 1.0 / std::sqrt(w[1] * w[1] + w[2] * w[2]);
    u[0] = 0;
    u[1] =  w[2] * inv_length;
    u[2] = -w[1] * inv_length;
    v[0] =  w[1] * u[2] - w[2] * u[1];
    v[1] = -w[0] * u[2];
    v[2] =  w[0] * u[1];
  }
}

//==============================================================================
template <typename Scalar, int M, int N>
VectorN<Scalar, M+N> combine(
    const VectorN<Scalar, M>& v1, const VectorN<Scalar, N>& v2)
{
  VectorN<Scalar, M+N> v;
  v << v1, v2;

  return v;
}

//==============================================================================
template <typename Scalar>
void hat(Matrix3<Scalar>& mat, const Vector3<Scalar>& vec)
{
  mat << 0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0;
}

//==============================================================================
template<typename Scalar>
void eigen(const Matrix3<Scalar>& m, Vector3<Scalar>& dout, Matrix3<Scalar>& vout)
{
  // We assume that m is symmetric matrix.
  Eigen::SelfAdjointEigenSolver<Matrix3<Scalar>> eigensolver(m);
  if (eigensolver.info() != Eigen::Success)
  {
    std::cerr << "[eigen] Failed to compute eigendecomposition.\n";
    return;
  }
  dout = eigensolver.eigenvalues();
  vout = eigensolver.eigenvectors();
}

//==============================================================================
template <typename Scalar>
void axisFromEigen(const Matrix3<Scalar>& eigenV,
                   const Vector3<Scalar>& eigenS,
                   Matrix3<Scalar>& axis)
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
  axis.col(2) = axis.col(0).cross(axis.col(1));
}

//==============================================================================
template <typename Scalar>
void generateCoordinateSystem(Matrix3<Scalar>& axis)
{
  // Assum axis.col(0) is closest to z-axis
  assert(axis.col(0).maxCoeff() == 2);

  if(std::abs(axis(0, 0)) >= std::abs(axis(1, 0)))
  {
    // let axis.col(0) = (x, y, z)
    // axis.col(1) = (-z, 0, x) / length((-z, 0, x)) // so that axis.col(0) and
    //                                               // axis.col(1) are
    //                                               // othorgonal
    // axis.col(2) = axis.col(0).cross(axis.col(1))

    Scalar inv_length = 1.0 / sqrt(std::pow(axis(0, 0), 2) + std::pow(axis(2, 0), 2));

    axis(0, 1) = -axis(2, 0) * inv_length;
    axis(1, 1) = 0;
    axis(2, 1) =  axis(0, 0) * inv_length;

    axis(0, 2) =  axis(1, 0) * axis(2, 1);
    axis(1, 2) =  axis(2, 0) * axis(0, 1) - axis(0, 0) * axis(2, 1);
    axis(2, 2) = -axis(1, 0) * axis(0, 1);
  }
  else
  {
    // let axis.col(0) = (x, y, z)
    // axis.col(1) = (0, z, -y) / length((0, z, -y)) // so that axis.col(0) and
    //                                               // axis.col(1) are
    //                                               // othorgonal
    // axis.col(2) = axis.col(0).cross(axis.col(1))

    Scalar inv_length = 1.0 / sqrt(std::pow(axis(1, 0), 2) + std::pow(axis(2, 0), 2));

    axis(0, 1) = 0;
    axis(1, 1) =  axis(2, 0) * inv_length;
    axis(2, 1) = -axis(1, 0) * inv_length;

    axis(0, 2) =  axis(1, 0) * axis(2, 1) - axis(2, 0) * axis(1, 1);
    axis(1, 2) = -axis(0, 0) * axis(2, 1);
    axis(2, 2) =  axis(0, 0) * axis(1, 1);
  }
}

//==============================================================================
template <typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD>
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

  R = R1.transpose() * R2;
  t = R1.transpose() * (t2 - t1);
}

//==============================================================================
template <typename Scalar, typename DerivedA, typename DerivedB>
void relativeTransform(
    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& T1,
    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& T2,
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

  relativeTransform(
        T1.linear(), T1.translation(), T2.linear(), T2.translation(), R, t);
}

//==============================================================================
template <typename Scalar>
void getRadiusAndOriginAndRectangleSize(
    Vector3<Scalar>* ps,
    Vector3<Scalar>* ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    const Matrix3<Scalar>& axis,
    Vector3<Scalar>& origin,
    Scalar l[2],
    Scalar& r)
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  int size_P = ((ps2) ? 2 : 1) * ((ts) ? 3 : 1) * n;

  std::vector<Vector3<Scalar>> P(size_P);

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
        const Vector3<Scalar>& p = ps[point_id];
        Vector3<Scalar> v(p[0], p[1], p[2]);
        P[P_id] = v.transpose() * axis;
        P_id++;
      }

      if(ps2)
      {
        for(int j = 0; j < 3; ++j)
        {
          int point_id = t[j];
          const Vector3<Scalar>& p = ps2[point_id];
          Vector3<Scalar> v(p[0], p[1], p[2]);
          P[P_id] = v.transpose() * axis;
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

      const Vector3<Scalar>& p = ps[index];
      Vector3<Scalar> v(p[0], p[1], p[2]);
      P[P_id] = v.transpose() * axis;
      P_id++;

      if(ps2)
      {
        const Vector3<Scalar>& v = ps2[index];
        P[P_id] = v.transpose() * axis;
        P_id++;
      }
    }
  }

  Scalar minx, maxx, miny, maxy, minz, maxz;

  Scalar cz, radsqr;

  minz = maxz = P[0][2];

  for(int i = 1; i < size_P; ++i)
  {
    Scalar z_value = P[i][2];
    if(z_value < minz) minz = z_value;
    else if(z_value > maxz) maxz = z_value;
  }

  r = (Scalar)0.5 * (maxz - minz);
  radsqr = r * r;
  cz = (Scalar)0.5 * (maxz + minz);

  // compute an initial length of rectangle along x direction

  // find minx and maxx as starting points

  int minindex, maxindex;
  minindex = maxindex = 0;
  Scalar mintmp, maxtmp;
  mintmp = maxtmp = P[0][0];

  for(int i = 1; i < size_P; ++i)
  {
    Scalar x_value = P[i][0];
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

  Scalar x, dz;
  dz = P[minindex][2] - cz;
  minx = P[minindex][0] + std::sqrt(std::max<Scalar>(radsqr - dz * dz, 0));
  dz = P[maxindex][2] - cz;
  maxx = P[maxindex][0] - std::sqrt(std::max<Scalar>(radsqr - dz * dz, 0));


  // grow minx

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][0] < minx)
    {
      dz = P[i][2] - cz;
      x = P[i][0] + std::sqrt(std::max<Scalar>(radsqr - dz * dz, 0));
      if(x < minx) minx = x;
    }
  }

  // grow maxx

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][0] > maxx)
    {
      dz = P[i][2] - cz;
      x = P[i][0] - std::sqrt(std::max<Scalar>(radsqr - dz * dz, 0));
      if(x > maxx) maxx = x;
    }
  }

  // compute an initial length of rectangle along y direction

  // find miny and maxy as starting points

  minindex = maxindex = 0;
  mintmp = maxtmp = P[0][1];
  for(int i = 1; i < size_P; ++i)
  {
    Scalar y_value = P[i][1];
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

  Scalar y;
  dz = P[minindex][2] - cz;
  miny = P[minindex][1] + std::sqrt(std::max<Scalar>(radsqr - dz * dz, 0));
  dz = P[maxindex][2] - cz;
  maxy = P[maxindex][1] - std::sqrt(std::max<Scalar>(radsqr - dz * dz, 0));

  // grow miny

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][1] < miny)
    {
      dz = P[i][2] - cz;
      y = P[i][1] + std::sqrt(std::max<Scalar>(radsqr - dz * dz, 0));
      if(y < miny) miny = y;
    }
  }

  // grow maxy

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][1] > maxy)
    {
      dz = P[i][2] - cz;
      y = P[i][1] - std::sqrt(std::max<Scalar>(radsqr - dz * dz, 0));
      if(y > maxy) maxy = y;
    }
  }

  // corners may have some points which are not covered - grow lengths if necessary
  // quite conservative (can be improved)
  Scalar dx, dy, u, t;
  Scalar a = std::sqrt((Scalar)0.5);
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
        u = u - std::sqrt(std::max<Scalar>(radsqr - t, 0));
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
        u = u - std::sqrt(std::max<Scalar>(radsqr - t, 0));
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
        u = u - std::sqrt(std::max<Scalar>(radsqr - t, 0));
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
        u = u - std::sqrt(std::max<Scalar>(radsqr - t, 0));
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
template <typename Scalar>
void circumCircleComputation(
    const Vector3<Scalar>& a,
    const Vector3<Scalar>& b,
    const Vector3<Scalar>& c,
    Vector3<Scalar>& center,
    Scalar& radius)
{
  Vector3<Scalar> e1 = a - c;
  Vector3<Scalar> e2 = b - c;
  Scalar e1_len2 = e1.squaredNorm();
  Scalar e2_len2 = e2.squaredNorm();
  Vector3<Scalar> e3 = e1.cross(e2);
  Scalar e3_len2 = e3.squaredNorm();
  radius = e1_len2 * e2_len2 * (e1 - e2).squaredNorm() / e3_len2;
  radius = std::sqrt(radius) * 0.5;

  center = (e2 * e1_len2 - e1 * e2_len2).cross(e3) * (0.5 * 1 / e3_len2) + c;
}

//==============================================================================
template <typename Scalar>
Scalar maximumDistance_mesh(Vector3<Scalar>* ps, Vector3<Scalar>* ps2, Triangle* ts, unsigned int* indices, int n, const Vector3<Scalar>& query)
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  Scalar maxD = 0;
  for(int i = 0; i < n; ++i)
  {
    unsigned int index = indirect_index ? indices[i] : i;
    const Triangle& t = ts[index];

    for(int j = 0; j < 3; ++j)
    {
      int point_id = t[j];
      const Vector3<Scalar>& p = ps[point_id];

      Scalar d = (p - query).squaredNorm();
      if(d > maxD) maxD = d;
    }

    if(ps2)
    {
      for(int j = 0; j < 3; ++j)
      {
        int point_id = t[j];
        const Vector3<Scalar>& p = ps2[point_id];

        Scalar d = (p - query).squaredNorm();
        if(d > maxD) maxD = d;
      }
    }
  }

  return std::sqrt(maxD);
}

//==============================================================================
template <typename Scalar>
Scalar maximumDistance_pointcloud(Vector3<Scalar>* ps, Vector3<Scalar>* ps2, unsigned int* indices, int n, const Vector3<Scalar>& query)
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  Scalar maxD = 0;
  for(int i = 0; i < n; ++i)
  {
    int index = indirect_index ? indices[i] : i;

    const Vector3<Scalar>& p = ps[index];
    Scalar d = (p - query).squaredNorm();
    if(d > maxD) maxD = d;

    if(ps2)
    {
      const Vector3<Scalar>& v = ps2[index];
      Scalar d = (v - query).squaredNorm();
      if(d > maxD) maxD = d;
    }
  }

  return std::sqrt(maxD);
}

//==============================================================================
template <typename Scalar>
Scalar maximumDistance(
    Vector3<Scalar>* ps,
    Vector3<Scalar>* ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    const Vector3<Scalar>& query)
{
  if(ts)
    return maximumDistance_mesh(ps, ps2, ts, indices, n, query);
  else
    return maximumDistance_pointcloud(ps, ps2, indices, n, query);
}

} // namespace fcl

#endif

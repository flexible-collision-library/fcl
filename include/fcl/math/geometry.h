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

#ifndef FCL_MATH_GEOMETRY_H
#define FCL_MATH_GEOMETRY_H

#include <cmath>
#include <iostream>
#include <vector>

#include "fcl/config.h"
#include "fcl/common/types.h"
#include "fcl/math/triangle.h"

namespace fcl {

template <typename S>
FCL_EXPORT
void normalize(Vector3<S>& v, bool* signal);

template <typename Derived>
FCL_EXPORT
typename Derived::RealScalar triple(const Eigen::MatrixBase<Derived>& x,
                                    const Eigen::MatrixBase<Derived>& y,
                                    const Eigen::MatrixBase<Derived>& z);

template <typename S, int M, int N>
FCL_EXPORT
VectorN<S, M+N> combine(
    const VectorN<S, M>& v1, const VectorN<S, N>& v2);

template <typename S>
FCL_EXPORT
void hat(Matrix3<S>& mat, const Vector3<S>& vec);

/// @brief compute the eigen vector and eigen vector of a matrix. dout is the
/// eigen values, vout is the eigen vectors
template <typename S>
FCL_EXPORT
void eigen(const Matrix3<S>& m, Vector3<S>& dout, Matrix3<S>& vout);

/// @brief compute the eigen vector and eigen vector of a matrix. dout is the
/// eigen values, vout is the eigen vectors
template<typename S>
FCL_EXPORT
void eigen_old(const Matrix3<S>& m, Vector3<S>& dout, Matrix3<S>& vout);

template <typename S>
FCL_EXPORT
void axisFromEigen(const Matrix3<S>& eigenV,
                   const Vector3<S>& eigenS,
                   Matrix3<S>& axis);

template <typename S>
FCL_EXPORT
void axisFromEigen(const Matrix3<S>& eigenV,
                   const Vector3<S>& eigenS,
                   Transform3<S>& tf);

/// @brief compute orthogonal coordinate system basis with given x-axis.
///
/// @param  x_axis Direction of x-axis. Must have non-zero length.
///                For best results, length should be much greater than
///                constants<S>::eps_12().
/// @return The coordinate system in a Matrix3<S>.
///         Column 0 will be the normalized x-axis.
///         Columns 1 and 2 will be created orthogonal to the x-axis, and
///         orthogonal to each other. Otherwise, the orientation of the y-axis
///         and z-axis to the x-axis is arbitrary.
template <typename S>
FCL_EXPORT
Matrix3<S> generateCoordinateSystem(const Vector3<S>& x_axis);

template <typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD>
FCL_EXPORT
void relativeTransform(
    const Eigen::MatrixBase<DerivedA>& R1, const Eigen::MatrixBase<DerivedB>& t1,
    const Eigen::MatrixBase<DerivedA>& R2, const Eigen::MatrixBase<DerivedB>& t2,
    Eigen::MatrixBase<DerivedC>& R, Eigen::MatrixBase<DerivedD>& t);

template <typename S, typename DerivedA, typename DerivedB>
FCL_EXPORT
void relativeTransform(
    const Eigen::Transform<S, 3, Eigen::Isometry>& T1,
    const Eigen::Transform<S, 3, Eigen::Isometry>& T2,
    Eigen::MatrixBase<DerivedA>& R, Eigen::MatrixBase<DerivedB>& t);

/// @brief Compute the RSS bounding volume parameters: radius, rectangle size
/// and the origin, given the BV axises.
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
    S& r);

/// @brief Compute the RSS bounding volume parameters: radius, rectangle size
/// and the origin, given the BV axises.
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
    S& r);

/// @brief Compute the center and radius for a triangle's circumcircle
template <typename S>
FCL_EXPORT
void circumCircleComputation(
    const Vector3<S>& a,
    const Vector3<S>& b,
    const Vector3<S>& c,
    Vector3<S>& center,
    S& radius);

/// @brief Compute the maximum distance from a given center point to a point cloud
template <typename S>
FCL_EXPORT
S maximumDistance(
    const Vector3<S>* const ps,
    const Vector3<S>* const ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    const Vector3<S>& query);

/// @brief Compute the bounding volume extent and center for a set or subset of
/// points, given the BV axises.
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
    Vector3<S>& extent);

/// @brief Compute the bounding volume extent and center for a set or subset of
/// points, given the BV axises.
template <typename S>
FCL_EXPORT
void getExtentAndCenter(
    const Vector3<S>* const ps,
    const Vector3<S>* const ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    Transform3<S>& tf,
    Vector3<S>& extent);

/// @brief Compute the covariance matrix for a set or subset of points. if
/// ts = null, then indices refer to points directly; otherwise refer to
/// triangles
template <typename S>
FCL_EXPORT
void getCovariance(
    const Vector3<S>* const ps,
    const Vector3<S>* const ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    Matrix3<S>& M);

} // namespace fcl

#include "fcl/math/geometry-inl.h"

#endif

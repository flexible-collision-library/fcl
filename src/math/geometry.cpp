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

#include "fcl/math/geometry-inl.h"

namespace fcl {

//==============================================================================
template
void normalize(Vector3d& v, bool* signal);

//==============================================================================
template
void hat(Matrix3d& mat, const Vector3d& vec);

//==============================================================================
template
void eigen(const Matrix3d& m, Vector3d& dout, Matrix3d& vout);

//==============================================================================
template
void eigen_old(const Matrix3d& m, Vector3d& dout, Matrix3d& vout);

//==============================================================================
template
void axisFromEigen(
    const Matrix3d& eigenV, const Vector3d& eigenS, Matrix3d& axis);

//==============================================================================
template
void axisFromEigen(const Matrix3d& eigenV,
                   const Vector3d& eigenS,
                   Transform3d& tf);

//==============================================================================
template
Matrix3d generateCoordinateSystem(const Vector3d& x_axis);

//==============================================================================
template
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
template
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
template
void circumCircleComputation(
    const Vector3d& a,
    const Vector3d& b,
    const Vector3d& c,
    Vector3d& center,
    double& radius);

//==============================================================================
template
double maximumDistance(
    const Vector3d* const ps,
    const Vector3d* const ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    const Vector3d& query);

//==============================================================================
template
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
template
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
template
double maximumDistance_mesh(
    const Vector3d* const ps,
    const Vector3d* const ps2,
    Triangle* ts,
    unsigned int* indices,
    int n,
    const Vector3d& query);

//==============================================================================
template
double maximumDistance_pointcloud(
    const Vector3d* const ps,
    const Vector3d* const ps2,
    unsigned int* indices,
    int n,
    const Vector3d& query);

//==============================================================================
template
void getExtentAndCenter_pointcloud(
    const Vector3d* const ps,
    const Vector3d* const ps2,
    unsigned int* indices,
    int n,
    const Matrix3d& axis,
    Vector3d& center,
    Vector3d& extent);

//==============================================================================
template
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

} // namespace fcl

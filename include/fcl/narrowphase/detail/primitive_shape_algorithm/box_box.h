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

#ifndef FCL_NARROWPHASE_DETAIL_BOXBOX_H
#define FCL_NARROWPHASE_DETAIL_BOXBOX_H

#include "fcl/common/types.h"
#include "fcl/narrowphase/contact_point.h"
#include "fcl/geometry/shape/box.h"

namespace fcl
{

namespace detail
{

template <typename S>
FCL_EXPORT
void lineClosestApproach(const Vector3<S>& pa, const Vector3<S>& ua,
                         const Vector3<S>& pb, const Vector3<S>& ub,
                         S* alpha, S* beta);

// find all the intersection points between the 2D rectangle with vertices
// at (+/-h[0],+/-h[1]) and the 2D quadrilateral with vertices (p[0],p[1]),
// (p[2],p[3]),(p[4],p[5]),(p[6],p[7]).
//
// the intersection points are returned as x,y pairs in the 'ret' array.
// the number of intersection points is returned by the function (this will
// be in the range 0 to 8).
template <typename S>
FCL_EXPORT
int intersectRectQuad2(S h[2], S p[8], S ret[16]);

// given n points in the plane (array p, of size 2*n), generate m points that
// best represent the whole set. the definition of 'best' here is not
// predetermined - the idea is to select points that give good box-box
// collision detection behavior. the chosen point indexes are returned in the
// array iret (of size m). 'i0' is always the first entry in the array.
// n must be in the range [1..8]. m must be in the range [1..n]. i0 must be
// in the range [0..n-1].
template <typename S>
FCL_EXPORT
void cullPoints2(int n, S p[], int m, int i0, int iret[]);

template <typename S, typename DerivedA, typename DerivedB>
FCL_EXPORT
int boxBox2(
    const Vector3<S>& side1,
    const Eigen::MatrixBase<DerivedA>& R1,
    const Eigen::MatrixBase<DerivedB>& T1,
    const Vector3<S>& side2,
    const Eigen::MatrixBase<DerivedA>& R2,
    const Eigen::MatrixBase<DerivedB>& T2,
    Vector3<S>& normal,
    S* depth,
    int* return_code,
    int maxc,
    std::vector<ContactPoint<S>>& contacts);

template <typename S>
FCL_EXPORT
int boxBox2(
    const Vector3<S>& side1,
    const Transform3<S>& tf1,
    const Vector3<S>& side2,
    const Transform3<S>& tf2,
    Vector3<S>& normal,
    S* depth,
    int* return_code,
    int maxc,
    std::vector<ContactPoint<S>>& contacts);

template <typename S>
FCL_EXPORT
bool boxBoxIntersect(const Box<S>& s1, const Transform3<S>& tf1,
                     const Box<S>& s2, const Transform3<S>& tf2,
                     std::vector<ContactPoint<S>>* contacts_);

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/primitive_shape_algorithm/box_box-inl.h"

#endif

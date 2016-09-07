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

#include "fcl/math/bv/RSS-inl.h"

namespace fcl
{

//==============================================================================
template
class RSS<double>;

//==============================================================================
template
void clipToRange(double& val, double a, double b);

//==============================================================================
template
void segCoords(
        double& t,
        double& u,
        double a,
        double b,
        double A_dot_B,
        double A_dot_T,
        double B_dot_T);

//==============================================================================
template
bool inVoronoi(
        double a,
        double b,
        double Anorm_dot_B,
        double Anorm_dot_T,
        double A_dot_B,
        double A_dot_T,
        double B_dot_T);

//==============================================================================
template
double rectDistance(
    const Matrix3<double>& Rab,
    const Vector3<double>& Tab,
    const double a[2],
    const double b[2],
    Vector3<double>* P,
    Vector3<double>* Q);

//==============================================================================
template
double rectDistance(
    const Transform3<double>& tfab,
    const double a[2],
    const double b[2],
    Vector3<double>* P,
    Vector3<double>* Q);

//==============================================================================
template
RSS<double> translate(const RSS<double>& bv, const Vector3<double>& t);

} // namespace fcl

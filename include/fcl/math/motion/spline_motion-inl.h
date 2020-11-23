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

#ifndef FCL_CCD_SPLINEMOTION_INL_H
#define FCL_CCD_SPLINEMOTION_INL_H

#include "fcl/math/motion/spline_motion.h"

#include "fcl/common/unused.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT SplineMotion<double>;

//==============================================================================
template <typename S>
SplineMotion<S>::SplineMotion(
    const Vector3<S>& Td0, const Vector3<S>& Td1, const Vector3<S>& Td2, const Vector3<S>& Td3,
    const Vector3<S>& Rd0, const Vector3<S>& Rd1, const Vector3<S>& Rd2, const Vector3<S>& Rd3)
  : MotionBase<S>()
{
  Td[0] = Td0;
  Td[1] = Td1;
  Td[2] = Td2;
  Td[3] = Td3;

  Rd[0] = Rd0;
  Rd[1] = Rd1;
  Rd[2] = Rd2;
  Rd[3] = Rd3;

  Rd0Rd0 = Rd[0].dot(Rd[0]);
  Rd0Rd1 = Rd[0].dot(Rd[1]);
  Rd0Rd2 = Rd[0].dot(Rd[2]);
  Rd0Rd3 = Rd[0].dot(Rd[3]);
  Rd1Rd1 = Rd[1].dot(Rd[1]);
  Rd1Rd2 = Rd[1].dot(Rd[2]);
  Rd1Rd3 = Rd[1].dot(Rd[3]);
  Rd2Rd2 = Rd[2].dot(Rd[2]);
  Rd2Rd3 = Rd[2].dot(Rd[3]);
  Rd3Rd3 = Rd[3].dot(Rd[3]);

  TA = Td[1] * 3 - Td[2] * 3 + Td[3] - Td[0];
  TB = (Td[0] - Td[1] * 2 + Td[2]) * 3;
  TC = (Td[2] - Td[0]) * 3;

  RA = Rd[1] * 3 - Rd[2] * 3 + Rd[3] - Rd[0];
  RB = (Rd[0] - Rd[1] * 2 + Rd[2]) * 3;
  RC = (Rd[2] - Rd[0]) * 3;

  tf.setIdentity();
  integrate(0.0);
}

//==============================================================================
template <typename S>
SplineMotion<S>::SplineMotion(
    const Matrix3<S>& R1, const Vector3<S>& T1,
    const Matrix3<S>& R2, const Vector3<S>& T2)
  : MotionBase<S>()
{
  FCL_UNUSED(R1);
  FCL_UNUSED(T1);
  FCL_UNUSED(R2);
  FCL_UNUSED(T2);

  // TODO
}

//==============================================================================
template <typename S>
SplineMotion<S>::SplineMotion(
    const Transform3<S>& tf1, const Transform3<S>& tf2)
  : MotionBase<S>()
{
  FCL_UNUSED(tf1);
  FCL_UNUSED(tf2);

  // TODO
}

//==============================================================================
template <typename S>
bool SplineMotion<S>::integrate(S dt) const
{
  if(dt > 1) dt = 1;

  Vector3<S> cur_T = Td[0] * getWeight0(dt) + Td[1] * getWeight1(dt) + Td[2] * getWeight2(dt) + Td[3] * getWeight3(dt);
  Vector3<S> cur_w = Rd[0] * getWeight0(dt) + Rd[1] * getWeight1(dt) + Rd[2] * getWeight2(dt) + Rd[3] * getWeight3(dt);
  S cur_angle = cur_w.norm();
  if (cur_angle > 0.0) {
    cur_w /= cur_angle;
  }

  tf.linear() = AngleAxis<S>(cur_angle, cur_w).toRotationMatrix();
  tf.translation() = cur_T;

  tf_t = dt;

  return true;
}

//==============================================================================
template <typename S>
S SplineMotion<S>::computeMotionBound(const BVMotionBoundVisitor<S>& mb_visitor) const
{
  return mb_visitor.visit(*this);
}

//==============================================================================
template <typename S>
S SplineMotion<S>::computeMotionBound(const TriangleMotionBoundVisitor<S>& mb_visitor) const
{
  return mb_visitor.visit(*this);
}

//==============================================================================
template <typename S>
void SplineMotion<S>::getCurrentTransform(Transform3<S>& tf_) const
{
  tf_ = tf;
}

//==============================================================================
template <typename S>
void SplineMotion<S>::getTaylorModel(TMatrix3<S>& tm, TVector3<S>& tv) const
{
  // set tv
  Vector3<S> c[4];
  c[0] = (Td[0] + Td[1] * 4 + Td[2] + Td[3]) * (1/6.0);
  c[1] = (-Td[0] + Td[2]) * (1/2.0);
  c[2] = (Td[0] - Td[1] * 2 + Td[2]) * (1/2.0);
  c[3] = (-Td[0] + Td[1] * 3 - Td[2] * 3 + Td[3]) * (1/6.0);
  tv.setTimeInterval(this->getTimeInterval());
  for(std::size_t i = 0; i < 3; ++i)
  {
    for(std::size_t j = 0; j < 4; ++j)
    {
      tv[i].coeff(j) = c[j][i];
    }
  }

  // set tm
  Matrix3<S> I = Matrix3<S>::Identity();
  // R(t) = R(t0) + R'(t0) (t-t0) + 1/2 R''(t0)(t-t0)^2 + 1 / 6 R'''(t0) (t-t0)^3 + 1 / 24 R''''(l)(t-t0)^4; t0 = 0.5
  /// 1. compute M(1/2)
  Vector3<S> Rt0 = (Rd[0] + Rd[1] * 23 + Rd[2] * 23 + Rd[3]) * (1 / 48.0);
  S Rt0_len = Rt0.norm();
  S inv_Rt0_len = 1.0 / Rt0_len;
  S inv_Rt0_len_3 = inv_Rt0_len * inv_Rt0_len * inv_Rt0_len;
  S inv_Rt0_len_5 = inv_Rt0_len_3 * inv_Rt0_len * inv_Rt0_len;
  S theta0 = Rt0_len;
  S costheta0 = cos(theta0);
  S sintheta0 = sin(theta0);

  Vector3<S> Wt0 = Rt0 * inv_Rt0_len;
  Matrix3<S> hatWt0;
  hat(hatWt0, Wt0);
  Matrix3<S> hatWt0_sqr = hatWt0 * hatWt0;
  Matrix3<S> Mt0 = I + hatWt0 * sintheta0 + hatWt0_sqr * (1 - costheta0);
  // TODO(JS): this could be improved by using exp(Wt0)

  /// 2. compute M'(1/2)
  Vector3<S> dRt0 = (-Rd[0] - Rd[1] * 5 + Rd[2] * 5 + Rd[3]) * (1 / 8.0);
  S Rt0_dot_dRt0 = Rt0.dot(dRt0);
  S dtheta0 = Rt0_dot_dRt0 * inv_Rt0_len;
  Vector3<S> dWt0 = dRt0 * inv_Rt0_len - Rt0 * (Rt0_dot_dRt0 * inv_Rt0_len_3);
  Matrix3<S> hatdWt0;
  hat(hatdWt0, dWt0);
  Matrix3<S> dMt0 = hatdWt0 * sintheta0 + hatWt0 * (costheta0 * dtheta0) + hatWt0_sqr * (sintheta0 * dtheta0) + (hatWt0 * hatdWt0 + hatdWt0 * hatWt0) * (1 - costheta0);

  /// 3.1. compute M''(1/2)
  Vector3<S> ddRt0 = (Rd[0] - Rd[1] - Rd[2] + Rd[3]) * 0.5;
  S Rt0_dot_ddRt0 = Rt0.dot(ddRt0);
  S dRt0_dot_dRt0 = dRt0.squaredNorm();
  S ddtheta0 = (Rt0_dot_ddRt0 + dRt0_dot_dRt0) * inv_Rt0_len - Rt0_dot_dRt0 * Rt0_dot_dRt0 * inv_Rt0_len_3;
  Vector3<S> ddWt0 = ddRt0 * inv_Rt0_len - (dRt0 * (2 * Rt0_dot_dRt0) + Rt0 * (Rt0_dot_ddRt0 + dRt0_dot_dRt0)) * inv_Rt0_len_3 + (Rt0 * (3 * Rt0_dot_dRt0 * Rt0_dot_dRt0)) * inv_Rt0_len_5;
  Matrix3<S> hatddWt0;
  hat(hatddWt0, ddWt0);
  Matrix3<S> ddMt0 =
      hatddWt0 * sintheta0 +
      hatWt0 * (costheta0 * dtheta0 - sintheta0 * dtheta0 * dtheta0 + costheta0 * ddtheta0) +
      hatdWt0 * (costheta0 * dtheta0) +
      (hatWt0 * hatdWt0 + hatdWt0 * hatWt0) * (sintheta0 * dtheta0 * 2) +
      hatdWt0 * hatdWt0 * (2 * (1 - costheta0)) +
      hatWt0 * hatWt0 * (costheta0 * dtheta0 * dtheta0 + sintheta0 * ddtheta0) +
      (hatWt0 * hatddWt0 + hatddWt0 + hatWt0) * (1 - costheta0);

  tm.setTimeInterval(this->getTimeInterval());
  for(std::size_t i = 0; i < 3; ++i)
  {
    for(std::size_t j = 0; j < 3; ++j)
    {
      tm(i, j).coeff(0) = Mt0(i, j) - dMt0(i, j) * 0.5 + ddMt0(i, j) * 0.25 * 0.5;
      tm(i, j).coeff(1) = dMt0(i, j) - ddMt0(i, j) * 0.5;
      tm(i, j).coeff(2) = ddMt0(i, j) * 0.5;
      tm(i, j).coeff(3) = 0;

      tm(i, j).remainder() = Interval<S>(-1/48.0, 1/48.0); /// not correct, should fix
    }
  }
}

//==============================================================================
template <typename S>
void SplineMotion<S>::computeSplineParameter()
{
  // TODO(JS): Not implemented?
}

//==============================================================================
template <typename S>
S SplineMotion<S>::computeTBound(const Vector3<S>& n) const
{
  S Ta = TA.dot(n);
  S Tb = TB.dot(n);
  S Tc = TC.dot(n);

  std::vector<S> T_potential;
  T_potential.push_back(tf_t);
  T_potential.push_back(1);
  if(Tb * Tb - 3 * Ta * Tc >= 0)
  {
    if(Ta == 0)
    {
      if(Tb != 0)
      {
        S tmp = -Tc / (2 * Tb);
        if(tmp < 1 && tmp > tf_t)
          T_potential.push_back(tmp);
      }
    }
    else
    {
      S tmp_delta = sqrt(Tb * Tb - 3 * Ta * Tc);
      S tmp1 = (-Tb + tmp_delta) / (3 * Ta);
      S tmp2 = (-Tb - tmp_delta) / (3 * Ta);
      if(tmp1 < 1 && tmp1 > tf_t)
        T_potential.push_back(tmp1);
      if(tmp2 < 1 && tmp2 > tf_t)
        T_potential.push_back(tmp2);
    }
  }

  S T_bound = Ta * T_potential[0] * T_potential[0] * T_potential[0] + Tb * T_potential[0] * T_potential[0] + Tc * T_potential[0];
  for(unsigned int i = 1; i < T_potential.size(); ++i)
  {
    S T_bound_tmp = Ta * T_potential[i] * T_potential[i] * T_potential[i] + Tb * T_potential[i] * T_potential[i] + Tc * T_potential[i];
    if(T_bound_tmp > T_bound) T_bound = T_bound_tmp;
  }


  S cur_delta = Ta * tf_t * tf_t * tf_t + Tb * tf_t * tf_t + Tc * tf_t;

  T_bound -= cur_delta;
  T_bound /= 6.0;

  return T_bound;
}

//==============================================================================
template <typename S>
S SplineMotion<S>::computeDWMax() const
{
  // first compute ||w'||
  int a00[5] = {1,-4,6,-4,1};
  int a01[5] = {-3,10,-11,4,0};
  int a02[5] = {3,-8,6,0,-1};
  int a03[5] = {-1,2,-1,0,0};
  int a11[5] = {9,-24,16,0,0};
  int a12[5] = {-9,18,-5,-4,0};
  int a13[5] = {3,-4,0,0,0};
  int a22[5] = {9,-12,-2,4,1};
  int a23[5] = {-3,2,1,0,0};
  int a33[5] = {1,0,0,0,0};

  S a[5];

  for(int i = 0; i < 5; ++i)
  {
    a[i] = Rd0Rd0 * a00[i] + Rd0Rd1 * a01[i] + Rd0Rd2 * a02[i] + Rd0Rd3 * a03[i]
        + Rd0Rd1 * a01[i] + Rd1Rd1 * a11[i] + Rd1Rd2 * a12[i] + Rd1Rd3 * a13[i]
        + Rd0Rd2 * a02[i] + Rd1Rd2 * a12[i] + Rd2Rd2 * a22[i] + Rd2Rd3 * a23[i]
        + Rd0Rd3 * a03[i] + Rd1Rd3 * a13[i] + Rd2Rd3 * a23[i] + Rd3Rd3 * a33[i];
    a[i] /= 4.0;
  }

  // compute polynomial for ||w'||'
  int da00[4] = {4,-12,12,-4};
  int da01[4] = {-12,30,-22,4};
  int da02[4] = {12,-24,12,0};
  int da03[4] = {-4,6,-2,0};
  int da11[4] = {36,-72,32,0};
  int da12[4] = {-36,54,-10,-4};
  int da13[4] = {12,-12,0,0};
  int da22[4] = {36,-36,-4,4};
  int da23[4] = {-12,6,2,0};
  int da33[4] = {4,0,0,0};

  S da[4];
  for(int i = 0; i < 4; ++i)
  {
    da[i] = Rd0Rd0 * da00[i] + Rd0Rd1 * da01[i] + Rd0Rd2 * da02[i] + Rd0Rd3 * da03[i]
        + Rd0Rd1 * da01[i] + Rd1Rd1 * da11[i] + Rd1Rd2 * da12[i] + Rd1Rd3 * da13[i]
        + Rd0Rd2 * da02[i] + Rd1Rd2 * da12[i] + Rd2Rd2 * da22[i] + Rd2Rd3 * da23[i]
        + Rd0Rd3 * da03[i] + Rd1Rd3 * da13[i] + Rd2Rd3 * da23[i] + Rd3Rd3 * da33[i];
    da[i] /= 4.0;
  }

  S roots[3];

  int root_num = detail::PolySolver<S>::solveCubic(da, roots);

  S dWdW_max = a[0] * tf_t * tf_t * tf_t + a[1] * tf_t * tf_t * tf_t + a[2] * tf_t * tf_t + a[3] * tf_t + a[4];
  S dWdW_1 = a[0] + a[1] + a[2] + a[3] + a[4];
  if(dWdW_max < dWdW_1) dWdW_max = dWdW_1;
  for(int i = 0; i < root_num; ++i)
  {
    S v = roots[i];

    if(v >= tf_t && v <= 1)
    {
      S value = a[0] * v * v * v * v + a[1] * v * v * v + a[2] * v * v + a[3] * v + a[4];
      if(value > dWdW_max) dWdW_max = value;
    }
  }

  return sqrt(dWdW_max);
}

//==============================================================================
template <typename S>
S SplineMotion<S>::getCurrentTime() const
{
  return tf_t;
}

//==============================================================================
template <typename S>
S SplineMotion<S>::getWeight0(S t) const
{
  return (1 - 3 * t + 3 * t * t - t * t * t) / 6.0;
}

//==============================================================================
template <typename S>
S SplineMotion<S>::getWeight1(S t) const
{
  return (4 - 6 * t * t + 3 * t * t * t) / 6.0;
}

//==============================================================================
template <typename S>
S SplineMotion<S>::getWeight2(S t) const
{
  return (1 + 3 * t + 3 * t * t - 3 * t * t * t) / 6.0;
}

//==============================================================================
template <typename S>
S SplineMotion<S>::getWeight3(S t) const
{
  return t * t * t / 6.0;
}

} // namespace fcl

#endif

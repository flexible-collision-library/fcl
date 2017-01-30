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

#ifndef FCL_CCD_TBVMOTIONBOUNDVISITOR_INL_H
#define FCL_CCD_TBVMOTIONBOUNDVISITOR_INL_H

#include "fcl/math/motion/tbv_motion_bound_visitor.h"

#include "fcl/common/unused.h"

namespace fcl
{

//==============================================================================
template<typename BV>
TBVMotionBoundVisitor<BV>::TBVMotionBoundVisitor(
    const BV& bv_, const Vector3<typename BV::S>& n_)
  : bv(bv_), n(n_)
{
  // Do nothing
}

//==============================================================================
template <typename S, typename BV, typename MotionT>
struct TBVMotionBoundVisitorVisitImpl
{
  static S run(
      const TBVMotionBoundVisitor<BV>& /*visitor*/,
      const MotionT& /*motion*/)
  {
    return 0;
  }
};

//==============================================================================
template<typename BV>
typename BV::S TBVMotionBoundVisitor<BV>::visit(
    const MotionBase<S>& motion) const
{
  FCL_UNUSED(motion);

  return 0;
}

//==============================================================================
template<typename BV>
typename BV::S TBVMotionBoundVisitor<BV>::visit(
    const SplineMotion<S>& motion) const
{
  using S = typename BV::S;

  return TBVMotionBoundVisitorVisitImpl<
      S, BV, SplineMotion<S>>::run(*this, motion);
}

//==============================================================================
template<typename BV>
typename BV::S TBVMotionBoundVisitor<BV>::visit(
    const ScrewMotion<S>& motion) const
{
  using S = typename BV::S;

  return TBVMotionBoundVisitorVisitImpl<
      S, BV, ScrewMotion<S>>::run(*this, motion);
}

//==============================================================================
template<typename BV>
typename BV::S TBVMotionBoundVisitor<BV>::visit(
    const InterpMotion<S>& motion) const
{
  using S = typename BV::S;

  return TBVMotionBoundVisitorVisitImpl<
      S, BV, InterpMotion<S>>::run(*this, motion);
}

//==============================================================================
template<typename BV>
typename BV::S TBVMotionBoundVisitor<BV>::visit(
    const TranslationMotion<S>& motion) const
{
  using S = typename BV::S;

  return TBVMotionBoundVisitorVisitImpl<
      S, BV, TranslationMotion<S>>::run(*this, motion);
}

//==============================================================================
template <typename S>
struct TBVMotionBoundVisitorVisitImpl<S, RSS<S>, SplineMotion<S>>
{
  static S run(
      const TBVMotionBoundVisitor<RSS<S>>& visitor,
      const SplineMotion<S>& motion)
  {
    S T_bound = motion.computeTBound(visitor.n);
    S tf_t = motion.getCurrentTime();

    Vector3<S> c1 = visitor.bv.To;
    Vector3<S> c2 = visitor.bv.To + visitor.bv.axis.col(0) * visitor.bv.l[0];
    Vector3<S> c3 = visitor.bv.To + visitor.bv.axis.col(1) * visitor.bv.l[1];
    Vector3<S> c4 = visitor.bv.To + visitor.bv.axis.col(0) * visitor.bv.l[0] + visitor.bv.axis.col(1) * visitor.bv.l[1];

    S tmp;
    // max_i |c_i * n|
    S cn_max = std::abs(c1.dot(visitor.n));
    tmp = std::abs(c2.dot(visitor.n));
    if(tmp > cn_max) cn_max = tmp;
    tmp = std::abs(c3.dot(visitor.n));
    if(tmp > cn_max) cn_max = tmp;
    tmp = std::abs(c4.dot(visitor.n));
    if(tmp > cn_max) cn_max = tmp;

    // max_i ||c_i||
    S cmax = c1.squaredNorm();
    tmp = c2.squaredNorm();
    if(tmp > cmax) cmax = tmp;
    tmp = c3.squaredNorm();
    if(tmp > cmax) cmax = tmp;
    tmp = c4.squaredNorm();
    if(tmp > cmax) cmax = tmp;
    cmax = sqrt(cmax);

    // max_i ||c_i x n||
    S cxn_max = (c1.cross(visitor.n)).squaredNorm();
    tmp = (c2.cross(visitor.n)).squaredNorm();
    if(tmp > cxn_max) cxn_max = tmp;
    tmp = (c3.cross(visitor.n)).squaredNorm();
    if(tmp > cxn_max) cxn_max = tmp;
    tmp = (c4.cross(visitor.n)).squaredNorm();
    if(tmp > cxn_max) cxn_max = tmp;
    cxn_max = sqrt(cxn_max);

    S dWdW_max = motion.computeDWMax();
    S ratio = std::min(1 - tf_t, dWdW_max);

    S R_bound = 2 * (cn_max + cmax + cxn_max + 3 * visitor.bv.r) * ratio;


    // std::cout << R_bound << " " << T_bound << std::endl;

    return R_bound + T_bound;
  }
};

//==============================================================================
/// @brief Compute the motion bound for a bounding volume along a given direction n
/// according to mu < |v * n| + ||w x n||(r + max(||ci*||)) where ||ci*|| = ||R0(ci) x w||. w is the angular axis (normalized)
/// and ci are the endpoints of the generator primitives of RSS.
/// Notice that all bv parameters are in the local frame of the object, but n should be in the global frame (the reason is that the motion (t1, t2 and t) is in global frame)
template <typename S>
struct TBVMotionBoundVisitorVisitImpl<S, RSS<S>, ScrewMotion<S>>
{
  static S run(
      const TBVMotionBoundVisitor<RSS<S>>& visitor,
      const ScrewMotion<S>& motion)
  {
    Transform3<S> tf;
    motion.getCurrentTransform(tf);

    const Vector3<S>& axis = motion.getAxis();
    S linear_vel = motion.getLinearVelocity();
    S angular_vel = motion.getAngularVelocity();
    const Vector3<S>& p = motion.getAxisOrigin();

    S c_proj_max = ((tf.linear() * visitor.bv.To).cross(axis)).squaredNorm();
    S tmp;
    tmp = ((tf.linear() * (visitor.bv.To + visitor.bv.axis.col(0) * visitor.bv.l[0])).cross(axis)).squaredNorm();
    if(tmp > c_proj_max) c_proj_max = tmp;
    tmp = ((tf.linear() * (visitor.bv.To + visitor.bv.axis.col(1) * visitor.bv.l[1])).cross(axis)).squaredNorm();
    if(tmp > c_proj_max) c_proj_max = tmp;
    tmp = ((tf.linear() * (visitor.bv.To + visitor.bv.axis.col(0) * visitor.bv.l[0] + visitor.bv.axis.col(1) * visitor.bv.l[1])).cross(axis)).squaredNorm();
    if(tmp > c_proj_max) c_proj_max = tmp;

    c_proj_max = sqrt(c_proj_max);

    S v_dot_n = axis.dot(visitor.n) * linear_vel;
    S w_cross_n = (axis.cross(visitor.n)).norm() * angular_vel;
    S origin_proj = ((tf.translation() - p).cross(axis)).norm();

    S mu = v_dot_n + w_cross_n * (c_proj_max + visitor.bv.r + origin_proj);

    return mu;
  }
};

//==============================================================================
/// @brief Compute the motion bound for a bounding volume along a given direction n
/// according to mu < |v * n| + ||w x n||(r + max(||ci*||)) where ||ci*|| = ||R0(ci) x w||. w is the angular axis (normalized)
/// and ci are the endpoints of the generator primitives of RSS.
/// Notice that all bv parameters are in the local frame of the object, but n should be in the global frame (the reason is that the motion (t1, t2 and t) is in global frame)
template <typename S>
struct TBVMotionBoundVisitorVisitImpl<S, RSS<S>, InterpMotion<S>>
{
  static S run(
      const TBVMotionBoundVisitor<RSS<S>>& visitor,
      const InterpMotion<S>& motion)
  {
    Transform3<S> tf;
    motion.getCurrentTransform(tf);

    const Vector3<S>& reference_p = motion.getReferencePoint();
    const Vector3<S>& angular_axis = motion.getAngularAxis();
    S angular_vel = motion.getAngularVelocity();
    const Vector3<S>& linear_vel = motion.getLinearVelocity();

    S c_proj_max = ((tf.linear() * (visitor.bv.To - reference_p)).cross(angular_axis)).squaredNorm();
    S tmp;
    tmp = ((tf.linear() * (visitor.bv.To + visitor.bv.axis.col(0) * visitor.bv.l[0] - reference_p)).cross(angular_axis)).squaredNorm();
    if(tmp > c_proj_max) c_proj_max = tmp;
    tmp = ((tf.linear() * (visitor.bv.To + visitor.bv.axis.col(1) * visitor.bv.l[1] - reference_p)).cross(angular_axis)).squaredNorm();
    if(tmp > c_proj_max) c_proj_max = tmp;
    tmp = ((tf.linear() * (visitor.bv.To + visitor.bv.axis.col(0) * visitor.bv.l[0] + visitor.bv.axis.col(1) * visitor.bv.l[1] - reference_p)).cross(angular_axis)).squaredNorm();
    if(tmp > c_proj_max) c_proj_max = tmp;

    c_proj_max = std::sqrt(c_proj_max);

    S v_dot_n = linear_vel.dot(visitor.n);
    S w_cross_n = (angular_axis.cross(visitor.n)).norm() * angular_vel;
    S mu = v_dot_n + w_cross_n * (visitor.bv.r + c_proj_max);

    return mu;
  }
};

//==============================================================================
/// @brief Compute the motion bound for a bounding volume along a given direction n
template <typename S>
struct TBVMotionBoundVisitorVisitImpl<S, RSS<S>, TranslationMotion<S>>
{
  static S run(
      const TBVMotionBoundVisitor<RSS<S>>& visitor,
      const TranslationMotion<S>& motion)
  {
    return motion.getVelocity().dot(visitor.n);
  }
};

} // namespace fcl

#endif

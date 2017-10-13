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

#ifndef FCL_CCD_TRIANGLEMOTIONBOUNDVISITOR_INL_H
#define FCL_CCD_TRIANGLEMOTIONBOUNDVISITOR_INL_H

#include "fcl/math/motion/triangle_motion_bound_visitor.h"

#include "fcl/math/motion/spline_motion.h"
#include "fcl/math/motion/screw_motion.h"
#include "fcl/math/motion/interp_motion.h"
#include "fcl/math/motion/translation_motion.h"
#include "fcl/math/motion/triangle_motion_bound_visitor.h"

namespace fcl
{

//==============================================================================
extern template
class FCL_EXPORT TriangleMotionBoundVisitor<double>;

//==============================================================================
template<typename S>
TriangleMotionBoundVisitor<S>::TriangleMotionBoundVisitor(
    const Vector3<S>& a_, const Vector3<S>& b_,
    const Vector3<S>& c_, const Vector3<S>& n_)
  : a(a_), b(b_), c(c_), n(n_)
{
  // Do nothing
}

//==============================================================================
template <typename S, typename MotionT>
struct TriangleMotionBoundVisitorVisitImpl
{
  static S run(
      const TriangleMotionBoundVisitor<S>& /*visitor*/,
      const MotionT& /*motion*/)
  {
    return 0;
  }
};

//==============================================================================
template<typename S>
S TriangleMotionBoundVisitor<S>::visit(
    const SplineMotion<S>& motion) const
{
  return TriangleMotionBoundVisitorVisitImpl<
      S, SplineMotion<S>>::run(*this, motion);
}

//==============================================================================
template<typename S>
S TriangleMotionBoundVisitor<S>::visit(
    const ScrewMotion<S>& motion) const
{
  return TriangleMotionBoundVisitorVisitImpl<
      S, ScrewMotion<S>>::run(*this, motion);
}

//==============================================================================
template<typename S>
S TriangleMotionBoundVisitor<S>::visit(
    const InterpMotion<S>& motion) const
{
  return TriangleMotionBoundVisitorVisitImpl<
      S, InterpMotion<S>>::run(*this, motion);
}

//==============================================================================
template<typename S>
S TriangleMotionBoundVisitor<S>::visit(
    const TranslationMotion<S>& motion) const
{
  return TriangleMotionBoundVisitorVisitImpl<
      S, TranslationMotion<S>>::run(*this, motion);
}

//==============================================================================
/// @brief Compute the motion bound for a triangle along a given direction n
/// according to mu < |v * n| + ||w x n||(max||ci*||) where ||ci*|| = ||R0(ci) x w|| / \|w\|. w is the angular velocity
/// and ci are the triangle vertex coordinates.
/// Notice that the triangle is in the local frame of the object, but n should be in the global frame (the reason is that the motion (t1, t2 and t) is in global frame)
template <typename S>
struct TriangleMotionBoundVisitorVisitImpl<S, ScrewMotion<S>>
{
  static S run(
      const TriangleMotionBoundVisitor<S>& visitor,
      const ScrewMotion<S>& motion)
  {
    Transform3<S> tf;
    motion.getCurrentTransform(tf);

    const Vector3<S>& axis = motion.getAxis();
    S linear_vel = motion.getLinearVelocity();
    S angular_vel = motion.getAngularVelocity();
    const Vector3<S>& p = motion.getAxisOrigin();

    S proj_max = ((tf.linear() * visitor.a + tf.translation() - p).cross(axis)).squaredNorm();
    S tmp;
    tmp = ((tf.linear() * visitor.b + tf.translation() - p).cross(axis)).squaredNorm();
    if(tmp > proj_max) proj_max = tmp;
    tmp = ((tf.linear() * visitor.c + tf.translation() - p).cross(axis)).squaredNorm();
    if(tmp > proj_max) proj_max = tmp;

    proj_max = std::sqrt(proj_max);

    S v_dot_n = axis.dot(visitor.n) * linear_vel;
    S w_cross_n = (axis.cross(visitor.n)).norm() * angular_vel;
    S mu = v_dot_n + w_cross_n * proj_max;

    return mu;
  }
};

//==============================================================================
/// @brief Compute the motion bound for a triangle along a given direction n
/// according to mu < |v * n| + ||w x n||(max||ci*||) where ||ci*|| = ||R0(ci) x w|| / \|w\|. w is the angular velocity
/// and ci are the triangle vertex coordinates.
/// Notice that the triangle is in the local frame of the object, but n should be in the global frame (the reason is that the motion (t1, t2 and t) is in global frame)
template <typename S>
struct TriangleMotionBoundVisitorVisitImpl<S, InterpMotion<S>>
{
  static S run(
      const TriangleMotionBoundVisitor<S>& visitor,
      const InterpMotion<S>& motion)
  {
    Transform3<S> tf;
    motion.getCurrentTransform(tf);

    const Vector3<S>& reference_p = motion.getReferencePoint();
    const Vector3<S>& angular_axis = motion.getAngularAxis();
    S angular_vel = motion.getAngularVelocity();
    const Vector3<S>& linear_vel = motion.getLinearVelocity();

    S proj_max = ((tf.linear() * (visitor.a - reference_p)).cross(angular_axis)).squaredNorm();
    S tmp;
    tmp = ((tf.linear() * (visitor.b - reference_p)).cross(angular_axis)).squaredNorm();
    if(tmp > proj_max) proj_max = tmp;
    tmp = ((tf.linear() * (visitor.c - reference_p)).cross(angular_axis)).squaredNorm();
    if(tmp > proj_max) proj_max = tmp;

    proj_max = std::sqrt(proj_max);

    S v_dot_n = linear_vel.dot(visitor.n);
    S w_cross_n = (angular_axis.cross(visitor.n)).norm() * angular_vel;
    S mu = v_dot_n + w_cross_n * proj_max;

    return mu;
  }
};

//==============================================================================
template <typename S>
struct TriangleMotionBoundVisitorVisitImpl<S, SplineMotion<S>>
{
  static S run(
      const TriangleMotionBoundVisitor<S>& visitor,
      const SplineMotion<S>& motion)
  {
    S T_bound = motion.computeTBound(visitor.n);
    S tf_t = motion.getCurrentTime();

    S R_bound = std::abs(visitor.a.dot(visitor.n)) + visitor.a.norm() + (visitor.a.cross(visitor.n)).norm();
    S R_bound_tmp = std::abs(visitor.b.dot(visitor.n)) + visitor.b.norm() + (visitor.b.cross(visitor.n)).norm();
    if(R_bound_tmp > R_bound) R_bound = R_bound_tmp;
    R_bound_tmp = std::abs(visitor.c.dot(visitor.n)) + visitor.c.norm() + (visitor.c.cross(visitor.n)).norm();
    if(R_bound_tmp > R_bound) R_bound = R_bound_tmp;

    S dWdW_max = motion.computeDWMax();
    S ratio = std::min(1 - tf_t, dWdW_max);

    R_bound *= 2 * ratio;

    // std::cout << R_bound << " " << T_bound << std::endl;

    return R_bound + T_bound;
  }
};

//==============================================================================
/// @brief Compute the motion bound for a triangle along a given direction n
template <typename S>
struct TriangleMotionBoundVisitorVisitImpl<S, TranslationMotion<S>>
{
  static S run(
      const TriangleMotionBoundVisitor<S>& visitor,
      const TranslationMotion<S>& motion)
  {
    return motion.getVelocity().dot(visitor.n);
  }
};

} // namespace fcl

#endif
